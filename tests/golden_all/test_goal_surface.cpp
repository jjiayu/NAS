// The goal as a SURFACE instead of a position (AstarSearchConfig::goal_surface_id, and the footstep QP called with no
// goal position). On each scene, with the last surface as the goal:
//   - the search finds a path that ends on that surface, with the goal stance foot, deterministically;
//   - the QP succeeds, the last footstep lies on the last patch (plane and polygon, tolerance 1e-6 m) and is NOT
//     pinned to a point (it differs from the position-goal plan's last footstep, which is the surface's centroid);
//   - the position-goal behaviour is unchanged (its last footstep is the goal).
// The path may have fewer steps than the position goal's (reaching the surface is easier than reaching a point in it).
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {
// distance of a footstep to the patch (0 when on the patch): plane offset and outside-polygon distance in xy
double off_patch(const Node& n, const Point_3& f) {
    const auto& v = n.patch_vertices;
    Plane_3 pl(v[0], v[1], v[2]);
    double norm = std::sqrt(CGAL::to_double(pl.a() * pl.a() + pl.b() * pl.b() + pl.c() * pl.c()));
    double worst = std::abs(CGAL::to_double(pl.a() * f.x() + pl.b() * f.y() + pl.c() * f.z() + pl.d())) / norm;
    double cx = 0, cy = 0;
    for (const auto& q : v) { cx += CGAL::to_double(q.x()) / v.size(); cy += CGAL::to_double(q.y()) / v.size(); }
    for (size_t k = 0; k < v.size(); ++k) {
        const Point_3 &a = v[k], &b = v[(k + 1) % v.size()];
        double ex = CGAL::to_double(b.x() - a.x()), ey = CGAL::to_double(b.y() - a.y()), len = std::hypot(ex, ey);
        if (len < 1e-12) continue;
        double nx = -ey / len, ny = ex / len;
        if (nx * (cx - CGAL::to_double(a.x())) + ny * (cy - CGAL::to_double(a.y())) > 0) { nx = -nx; ny = -ny; }
        worst = std::max(worst, nx * CGAL::to_double(f.x() - a.x()) + ny * CGAL::to_double(f.y() - a.y()));
    }
    return worst;
}
} // namespace

int main() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    struct Scene { const char* name; Point_3 start; };
    int failures = 0;
    for (const Scene& s : std::vector<Scene>{{"Flat", Point_3(0, 0, 0)}, {"Stairs", Point_3(0.1, 0, 0)}, {"LongStairs", Point_3(0, 0, 0)},
                                              {"NarrowPassage", Point_3(0, 0, 0)}, {"Ramp", Point_3(-1, 0, 0)}}) {
        config::Scenario sc = config::load_scenario(s.name);
        const int last = static_cast<int>(sc.surfaces.size()) - 1;
        AstarSearchConfig pos;
        pos.start_position = s.start; pos.start_stance_foot = StanceFoot::Right; pos.goal_stance_foot = StanceFoot::Left;
        pos.goal_location = sc.surfaces.back().centroid;
        pos.expansion_params.rotation_enabled = true;
        AstarSearchConfig surf = pos;
        surf.goal_surface_id = last;

        AstarSearch a(sc.surfaces, reach, surf), b(sc.surfaces, reach, surf), p(sc.surfaces, reach, pos);
        a.search(); b.search(); p.search();
        FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
        QuadprogBackend backend;
        bool found = !a.result_path().empty();
        bool ends_on_surface = found && a.result_path().back()->surface_id == last && a.result_path().back()->stance_foot == StanceFoot::Left;
        bool det = a.expansion_count() == b.expansion_count() && a.result_path().size() == b.result_path().size();
        FootstepPlan plan_s, plan_p;
        if (found) plan_s = solve_footstep_qp(a.result_path(), s.start, std::nullopt, reach, qc, backend);
        if (!p.result_path().empty()) plan_p = solve_footstep_qp(p.result_path(), s.start, sc.surfaces.back().centroid, reach, qc, backend);
        double off = plan_s.success ? off_patch(*a.result_path().back(), plan_s.footsteps.back()) : 1e9;
        double to_centroid = plan_s.success ? std::hypot(CGAL::to_double(plan_s.footsteps.back().x() - sc.surfaces.back().centroid.x()),
                                                         CGAL::to_double(plan_s.footsteps.back().y() - sc.surfaces.back().centroid.y())) : 0.0;
        double pos_goal_err = plan_p.success ? std::hypot(CGAL::to_double(plan_p.footsteps.back().x() - sc.surfaces.back().centroid.x()),
                                                          CGAL::to_double(plan_p.footsteps.back().y() - sc.surfaces.back().centroid.y())) : 1e9;
        bool ok = found && ends_on_surface && det && plan_s.success && off <= 1e-6 && plan_p.success && pos_goal_err < 1e-6;
        failures += ok ? 0 : 1;
        std::printf("%s %-14s: surface goal: %d expansions, %zu nodes, QP %s, last footstep off the last patch by %.1e m, %.2f m from the centroid; position goal: %d expansions, %zu nodes, last footstep at the goal to %.1e m\n",
                    ok ? "ok:  " : "FAIL:", s.name, a.expansion_count(), a.result_path().size(), plan_s.success ? "ok" : "FAILED", off, to_centroid,
                    p.expansion_count(), p.result_path().size(), pos_goal_err);
    }
    return failures == 0 ? 0 : 1;
}
