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
//   - determinism: each search is run twice, the second time after a fragmentation
//     of the heap; the expansion count and the path must be identical (the old
//     code was not: NAS_SCRAMBLE showed up to 9 different results per scene).
// Scenes where the old search found no path are reported, not asserted.

#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/fixtures/golden_compare.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

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
    c.distance_metric = DistanceMetric::Epa;
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
        bool ok = o.deterministic && (!o.old_found || (o.path_ok && o.qp_ok));
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
