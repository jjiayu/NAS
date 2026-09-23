// Inclined surfaces (Ramp, SteepRamp, SlopedGround, SideSlope: new scenes, the old
// environments.hpp is all horizontal). The reachability polytope lives in the support foot's frame,
// whose z axis is the contact surface's normal; the paper's Q (Eq. 2) is the yaw composed with the
// surface's rotation. For each scene:
//   - the search finds a path, is deterministic, and the footstep QP succeeds;
//   - the plan is feasible, checked with an INDEPENDENT implementation (frame_check.hpp: Eigen::AngleAxis,
//     normals from the patches' own vertices): reachability = Q^T (x_i - x_{i-1}) inside the polytope's
//     convex hull, footsteps on their patch plane and polygon, start and goal;
//   - negative control: the same plans checked with the tilt IGNORED (yaw-only frame) must violate the
//     constraints on at least one scene (where the reachability constraint is active; on others the plan
//     happens to satisfy both frames), so the check is sensitive to the tilt and the tilt matters.
#include "frame_check.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <CGAL/convex_hull_3.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;
using nas::test::frame_Q;
using nas::test::to_vec;
using nas::test::up_normal_of;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

struct Hull { std::vector<Eigen::Vector4d> planes; }; // (a, b, c, d) outward: a x + b y + c z + d <= 0 inside

Hull hull_of(const Polyhedron& mesh) {
    std::vector<Point_3> v;
    for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) v.push_back(it->point());
    Polyhedron hull;
    CGAL::convex_hull_3(v.begin(), v.end(), hull);
    Hull h;
    for (auto f = hull.facets_begin(); f != hull.facets_end(); ++f) {
        auto e = f->halfedge();
        Eigen::Vector3d p0 = to_vec(e->vertex()->point()), p1 = to_vec(e->next()->vertex()->point()), p2 = to_vec(e->next()->next()->vertex()->point());
        Eigen::Vector3d n = (p1 - p0).cross(p2 - p0).normalized(); // counter-clockwise from outside: outward
        h.planes.emplace_back(n.x(), n.y(), n.z(), -n.dot(p0));
    }
    return h;
}

// Worst violation of the plan; `tilted` selects the frame convention.
double violation(const std::vector<Node*>& path, const std::vector<Point_3>& feet, const Hull hulls[2], const Eigen::Vector3d& start_normal,
                 const Point_3& start, const Point_3& goal, bool tilted) {
    double worst = std::max((to_vec(feet.front()) - to_vec(start)).cwiseAbs().maxCoeff(), (to_vec(feet.back()) - to_vec(goal)).cwiseAbs().maxCoeff());
    const size_t n = path.size();
    for (size_t i = 1; i < n; ++i) {
        Eigen::Vector3d support_normal = (i == 1) ? start_normal : up_normal_of(path[i - 1]->patch_vertices);
        Eigen::Vector3d local = frame_Q(support_normal, path[i - 1]->foot_yaw, tilted).transpose() * (to_vec(feet[i]) - to_vec(feet[i - 1]));
        for (const Eigen::Vector4d& pl : hulls[static_cast<int>(path[i]->stance_foot)].planes)
            worst = std::max(worst, pl.head<3>().dot(local) + pl(3));
    }
    for (size_t i = 1; i + 1 < n; ++i) {
        const auto& v = path[i]->patch_vertices;
        Eigen::Vector3d nrm = up_normal_of(v);
        Eigen::Vector3d f = to_vec(feet[i]);
        worst = std::max(worst, std::abs(nrm.dot(f - to_vec(v[0]))));
        Eigen::Vector2d c = Eigen::Vector2d::Zero();
        for (const auto& q : v) c += to_vec(q).head<2>() / v.size();
        for (size_t k = 0; k < v.size(); ++k) {
            Eigen::Vector2d a = to_vec(v[k]).head<2>(), b = to_vec(v[(k + 1) % v.size()]).head<2>();
            Eigen::Vector2d e = b - a;
            if (e.norm() < 1e-12) continue;
            Eigen::Vector2d nn(-e.y(), e.x()); nn.normalize();
            if (nn.dot(c - a) > 0) nn = -nn;
            worst = std::max(worst, nn.dot(f.head<2>() - a));
        }
    }
    return worst;
}

struct Scene { const char* name; Point_3 start; Eigen::Vector3d start_normal; };

} // namespace

int main() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    const Hull hulls[2] = {hull_of(reach.query("LF", "RF", ReachabilityDirection::Forward)), hull_of(reach.query("RF", "LF", ReachabilityDirection::Forward))};
    const double t10 = std::tan(10.0 * M_PI / 180.0);
    const std::vector<Scene> scenes = {
        {"Ramp", Point_3(-1.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
        {"SteepRamp", Point_3(-1.0, 0.0, 0.0), Eigen::Vector3d::UnitZ()},
        {"SlopedGround", Point_3(-1.5, 0.0, -1.5 * t10), Eigen::Vector3d(-t10, 0.0, 1.0).normalized()},
        {"SideSlope", Point_3(-1.5, 0.0, 0.0), Eigen::Vector3d(0.0, -t10, 1.0).normalized()},
    };
    int failures = 0;
    double control = 0.0;
    for (const Scene& s : scenes) {
        config::Scenario sc = config::load_scenario(s.name);
        AstarSearchConfig cfg;
        cfg.start_position = s.start;
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.goal_location = sc.surfaces.back().centroid;
        cfg.goal_stance_foot = StanceFoot::Left;
        cfg.expansion_params.rotation_enabled = true;

        AstarSearch a(sc.surfaces, reach, cfg), b(sc.surfaces, reach, cfg);
        a.search(); b.search();
        const auto& path = a.result_path();
        bool found = !path.empty();
        bool det = a.expansion_count() == b.expansion_count() && a.result_path().size() == b.result_path().size();
        bool on_goal = found && path.back()->check_if_node_contains_point(cfg.goal_location);
        FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
        QuadprogBackend backend;
        FootstepPlan plan;
        if (found) plan = solve_footstep_qp(path, cfg.start_position, cfg.goal_location, reach, qc, backend);
        double v_ok = plan.success ? violation(path, plan.footsteps, hulls, s.start_normal, cfg.start_position, cfg.goal_location, true) : 1e9;
        double v_flat = plan.success ? violation(path, plan.footsteps, hulls, s.start_normal, cfg.start_position, cfg.goal_location, false) : 0.0;
        bool ok = found && on_goal && det && plan.success && v_ok <= 1e-6;
        failures += ok ? 0 : 1;
        control = std::max(control, v_flat);
        std::printf("%s %-13s: path %s (%d expansions, %zu nodes), QP %s, deterministic %d, worst violation with the tilted frame %.2e m; yaw-only frame (control) %.2e m\n",
                    ok ? "ok:  " : "FAIL:", s.name, found ? "found" : "NOT FOUND", a.expansion_count(), path.size(), plan.success ? "ok" : "FAILED", det, v_ok, v_flat);
    }
    bool control_ok = control > 1e-4;
    failures += control_ok ? 0 : 1;
    std::printf("%s negative control: ignoring the tilt violates the constraints by up to %.2e m (where the reachability constraint is active)\n", control_ok ? "ok:  " : "FAIL:", control);
    return failures == 0 ? 0 : 1;
}
