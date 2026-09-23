// Replays EVERY scenario captured in phase 0 (tests/golden/*_astar.json)
// through AstarSearch + footstep QP and compares against the plans the old code
// stored, using exactly the configuration tests/capture_golden_references.sh ran
// the old code with (start position / goal offset per scene, right stance start,
// left stance goal, EPA heuristic x10, rotation on with 3 yaws of 10deg, cycle
// detection on). The planner has a single behaviour (docs/paper-deltas.md,
// "Profil retenu"), so:
//   - path: same number of nodes and same (depth, stance_foot, surface_id) per
//     node as the old plan. foot_yaw is reported, not compared: equal-cost plans
//     differ only by yaw and the old code's choice was arbitrary (it varied with
//     heap state);
//   - QP, where the old QP succeeded: the new QP succeeds on the new path, same
//     footstep count, and the distance walked is within 6% of the old plan's
//     (yaw ties move footsteps by up to ~35cm, so positions are not compared);
//     where the old QP failed the new result is only reported;
//   - feasibility: whatever the QP returns is re-checked against the constraints with an
//     independent implementation (true reachability polytope = hull of the mesh vertices,
//     patch plane and polygon, start, goal): worst violation <= 1e-6 m;
//   - determinism: each search is run twice, the second time after a fragmentation
//     of the heap; the expansion count and the path must be identical (the old
//     code was not: NAS_SCRAMBLE showed up to 9 different results per scene).
// Scenes where the old search found no path are reported, not asserted.

#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"
#include "nas/core/reachability.hpp"
#include "nas/fixtures/golden_compare.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

#include <CGAL/convex_hull_3.h>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;
using json = nlohmann::json;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

namespace {

struct SceneSetup {
    const char* name;
    Point_3 start;
    Vector_3 goal_offset;
};

// From tests/capture_golden_references.sh's SCENARIOS table.
const std::vector<SceneSetup> kScenes = {
    {"NarrowPassage", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Stairs", Point_3(0.1, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongLongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Flat", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairsComplete", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairsExp", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"ThreePathsScene", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Stairs_Up_Down", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"ThreePathsNAS", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 1.0, 0.0)},
};

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    return ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
}

AstarSearchConfig old_constants_config(const SceneSetup& setup, const Point_3& goal) {
    AstarSearchConfig c;
    c.start_position = setup.start;
    c.start_stance_foot = StanceFoot::Right;
    c.start_foot_yaw = 0.0;
    c.goal_location = goal;
    c.goal_stance_foot = StanceFoot::Left;
    c.heuristic_weight = 10.0;
    c.node_similarity_threshold = 0.02;
    c.expansion_params.rotation_enabled = true;
    c.expansion_params.yaw_discretization_num = 3;
    c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    c.expansion_params.cycle_detection_enabled = true;
    return c;
}

struct Outcome {
    std::string name;
    bool old_found = false;
    bool new_found = false;
    bool path_ok = true;        // meaningful when old_found
    bool deterministic = true;
    bool feasible = true;       // independent check of the QP result (see max_violation)
    bool qp_ok = true;
    int yaw_differs = 0;
    std::string qp_status;      // human-readable
    int expansions = 0;
};

// Path signature (surface, stance, yaw per node) + expansion count, to compare runs.
std::string signature(const AstarSearch& search) {
    std::string sig = "expansions=" + std::to_string(search.expansion_count()) + " ";
    for (const auto* n : search.result_path())
        sig += std::to_string(n->surface_id) + ":" + std::to_string(static_cast<int>(n->stance_foot)) + ":" + std::to_string(n->foot_yaw) + " ";
    return sig;
}

// Fragments the heap in a seed-dependent way, so allocation-order-dependent
// behaviour (CGAL::convex_hull_3's triangulation) changes between two runs.
void scramble_heap(unsigned seed) {
    std::srand(seed);
    std::vector<void*> blocks;
    for (int i = 0; i < 4000; ++i) blocks.push_back(std::malloc(16 + std::rand() % 1500));
    for (size_t i = blocks.size(); i > 1; --i) std::swap(blocks[i - 1], blocks[std::rand() % i]);
    for (size_t i = 0; i < blocks.size(); i += 2) std::free(blocks[i]);
}

double walked_distance(const std::vector<Point_3>& steps) {
    double d = 0;
    for (size_t i = 1; i < steps.size(); ++i)
        d += std::hypot(CGAL::to_double(steps[i].x() - steps[i - 1].x()), CGAL::to_double(steps[i].y() - steps[i - 1].y()));
    return d;
}

// Independent feasibility check of a QP result: reachability against the TRUE polytope
// (planes of the facets of the convex hull of the reachability mesh's vertices, recomputed
// here, not through core/geometry), footstep on its patch plane and inside its polygon,
// start and goal. Returns the largest violation in metres.
double max_violation(const std::vector<Node*>& path, const std::vector<Point_3>& feet, const ReachabilityModel& reach,
                     const Point_3& start, const Point_3& goal) {
    auto hull_planes = [&](StanceFoot moving) {
        const Polyhedron& mesh = reach.query(effector_name(moving), effector_name(other_foot(moving)), ReachabilityDirection::Forward);
        std::vector<Point_3> v;
        for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) v.push_back(it->point());
        Polyhedron hull;
        CGAL::convex_hull_3(v.begin(), v.end(), hull);
        std::vector<Plane_3> planes; // CGAL hull facets are counter-clockwise seen from outside: normal points outward
        for (auto f = hull.facets_begin(); f != hull.facets_end(); ++f) {
            auto h = f->halfedge();
            planes.emplace_back(h->vertex()->point(), h->next()->vertex()->point(), h->next()->next()->vertex()->point());
        }
        return planes;
    };
    const std::vector<Plane_3> planes[2] = {hull_planes(StanceFoot::Left), hull_planes(StanceFoot::Right)};
    double worst = 0.0;
    auto d3 = [](const Point_3& a, const Point_3& b) {
        return std::max({std::abs(CGAL::to_double(a.x() - b.x())), std::abs(CGAL::to_double(a.y() - b.y())), std::abs(CGAL::to_double(a.z() - b.z()))});
    };
    worst = std::max({worst, d3(feet.front(), start), d3(feet.back(), goal)});
    const size_t n = path.size();
    for (size_t i = 1; i < n; ++i) {
        double yaw = path[i - 1]->foot_yaw, c = std::cos(yaw), s = std::sin(yaw);
        double dx = CGAL::to_double(feet[i].x() - feet[i - 1].x()), dy = CGAL::to_double(feet[i].y() - feet[i - 1].y()), dz = CGAL::to_double(feet[i].z() - feet[i - 1].z());
        Point_3 local(c * dx + s * dy, -s * dx + c * dy, dz); // R(yaw)^T * relative position
        for (const Plane_3& pl : planes[static_cast<int>(path[i]->stance_foot)]) {
            double norm = std::sqrt(CGAL::to_double(pl.a() * pl.a() + pl.b() * pl.b() + pl.c() * pl.c()));
            worst = std::max(worst, (CGAL::to_double(pl.a()) * CGAL::to_double(local.x()) + CGAL::to_double(pl.b()) * CGAL::to_double(local.y()) +
                                     CGAL::to_double(pl.c()) * CGAL::to_double(local.z()) + CGAL::to_double(pl.d())) / norm);
        }
    }
    for (size_t i = 1; i + 1 < n; ++i) {
        const auto& v = path[i]->patch_vertices;
        Plane_3 pp(v[0], v[1], v[2]);
        double pn = std::sqrt(CGAL::to_double(pp.a() * pp.a() + pp.b() * pp.b() + pp.c() * pp.c()));
        worst = std::max(worst, std::abs(CGAL::to_double(pp.a() * feet[i].x() + pp.b() * feet[i].y() + pp.c() * feet[i].z() + pp.d())) / pn);
        double cx = 0, cy = 0;
        for (const auto& q : v) { cx += CGAL::to_double(q.x()) / v.size(); cy += CGAL::to_double(q.y()) / v.size(); }
        for (size_t k = 0; k < v.size(); ++k) {
            const Point_3 &a = v[k], &b = v[(k + 1) % v.size()];
            double ex = CGAL::to_double(b.x() - a.x()), ey = CGAL::to_double(b.y() - a.y()), len = std::hypot(ex, ey);
            if (len < 1e-12) continue;
            double nx = -ey / len, ny = ex / len;
            if (nx * (cx - CGAL::to_double(a.x())) + ny * (cy - CGAL::to_double(a.y())) > 0) { nx = -nx; ny = -ny; } // outward
            worst = std::max(worst, nx * (CGAL::to_double(feet[i].x() - a.x())) + ny * (CGAL::to_double(feet[i].y() - a.y())));
        }
    }
    return worst;
}

} // namespace

int main() {
    ReachabilityModel reachability = make_forward_reachability();
    std::vector<Outcome> outcomes;
    unsigned seed = 1;

    for (const SceneSetup& setup : kScenes) {
        Outcome out;
        out.name = setup.name;
        std::cout << "\n=== " << setup.name << " ===\n";

        std::ifstream gf(std::string(GOLDEN_DATA_DIR) + "/" + setup.name + "_astar.json");
        json golden;
        gf >> golden;
        out.old_found = golden.value("success", false);

        config::Scenario scenario = config::load_scenario(setup.name);
        Point_3 goal = scenario.surfaces.back().centroid + setup.goal_offset;
        AstarSearchConfig cfg = old_constants_config(setup, goal);

        AstarSearch search(scenario.surfaces, reachability, cfg);
        search.search();
        const auto& path = search.result_path();
        out.new_found = !path.empty();
        out.expansions = search.expansion_count();
        std::string sig = signature(search);
        std::cout << "SIG " << setup.name << " " << sig << "\n";

        // determinism: same search after a different heap state
        scramble_heap(seed++);
        AstarSearch again(scenario.surfaces, reachability, cfg);
        again.search();
        out.deterministic = signature(again) == sig;
        std::cout << (out.deterministic ? "ok" : "FAIL") << ": search is identical after a heap fragmentation\n";

        if (!out.old_found) {
            out.qp_status = "n/a (old search found no path)";
            std::cout << "old code found no path; new code " << (out.new_found ? "FOUND one (differs)" : "also finds none") << "\n";
            outcomes.push_back(out);
            continue;
        }

        out.path_ok = fixtures::check_path_matches_golden(path, std::string(GOLDEN_DATA_DIR) + "/" + setup.name + "_astar.json");

        if (path.empty()) {
            out.qp_status = "n/a (new search found no path)";
            out.qp_ok = false;
            outcomes.push_back(out);
            continue;
        }

        FootstepQPConfig qp_config;
        qp_config.alpha_weight = 10.0;
        qp_config.rotation_enabled = true;
        QuadprogBackend backend;
        FootstepPlan plan = solve_footstep_qp(path, cfg.start_position, cfg.goal_location, reachability, qp_config, backend);

        if (plan.success) {
            double viol = max_violation(path, plan.footsteps, reachability, cfg.start_position, cfg.goal_location);
            bool feasible = viol <= 1e-6;
            std::cout << (feasible ? "ok" : "FAIL") << ": independent feasibility check of the QP result: worst constraint violation " << viol << " m\n";
            out.feasible = feasible;
        }
        const auto& gfp = golden["footstep_plan"];
        bool old_qp_ok = gfp.value("success", false);
        if (old_qp_ok) {
            const auto& gsteps = gfp["footsteps"];
            std::vector<Point_3> old_steps;
            for (const auto& g : gsteps) old_steps.emplace_back(g[0].get<double>(), g[1].get<double>(), g[2].get<double>());
            bool same_count = plan.footsteps.size() == old_steps.size();
            double w_new = walked_distance(plan.footsteps), w_old = walked_distance(old_steps);
            bool walk_ok = std::abs(w_new - w_old) <= 0.06 * w_old;
            out.qp_ok = plan.success && same_count && walk_ok;
            out.qp_status = std::string(plan.success ? "solved" : "FAILED") + ", " + std::to_string(plan.footsteps.size()) + " footsteps vs old " +
                            std::to_string(old_steps.size()) + ", walked " + std::to_string(w_new) + " m vs old " + std::to_string(w_old) + " m";
        } else {
            out.qp_status = std::string("old QP failed; new QP ") + (plan.success ? "SOLVES it" : "also fails") + " (reported only)";
        }
        std::cout << (out.qp_ok ? "ok" : "FAIL") << ": QP - " << out.qp_status << "\n";
        outcomes.push_back(out);
    }

    std::cout << "\n================ SUMMARY ================\n";
    bool all_ok = true;
    for (const Outcome& o : outcomes) {
        bool ok = o.deterministic && o.feasible && (!o.old_found || (o.path_ok && o.qp_ok));
        all_ok = all_ok && ok;
        std::cout << (ok ? "PASS  " : "FAIL  ") << o.name << ": " << o.expansions << " expansions, ";
        if (!o.old_found) {
            std::cout << "old found no path; new " << (o.new_found ? "finds one" : "finds none");
        } else {
            std::cout << "path " << (o.path_ok ? "same length/surfaces/stances as the old plan" : "DIFFERS from the old plan") << ", QP: " << o.qp_status;
        }
        std::cout << (o.deterministic ? "" : " [NOT DETERMINISTIC]") << "\n";
    }
    return all_ok ? 0 : 1;
}
