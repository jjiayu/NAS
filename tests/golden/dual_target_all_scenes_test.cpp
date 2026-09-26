// "2 targets" (AstarSearchConfig::foot_goals, both slots filled — mode 2, "closing stance") sweep
// across every scenario in golden_all_scenes_test.cpp's own kScenes table. Unlike that test, this one
// does NOT compare against tests/golden_data/*_astar.json: those were captured from the OLD code,
// which never had a foot_goals mode to capture a reference for. This test establishes its own
// baseline instead — expansion counts recorded here (from an actual run at the time this test was
// written) are the regression reference going forward, the same role golden JSON plays elsewhere.
//
// Per scene: run the scenario's existing (mode 0, single point goal) search first, exactly as
// golden_all_scenes_test does; if it finds a path, build a small target region around each of its
// last two nodes' own observed position/yaw (generous enough to not depend on landing on the exact
// same footstep, but centered on an actually-reachable one — not hand-guessed) and confirm a mode-2
// search with BOTH targets filled also succeeds, ending on two consecutive opposite-foot nodes each
// inside their own target. One scene (Flat) additionally gets a deliberately-unreachable variant, to
// check infeasibility is reported cleanly across a real scenario, not just the isolated case in
// foot_goals_test.cpp.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what.c_str());
    }
}

struct SceneSetup {
    const char* name;
    Point_3 start;
    Vector_3 goal_offset;
    int expected_baseline_expansions; // mode 0, this scene's own existing goal — regression reference
    int expected_dual_expansions;     // mode 2, targets built around the baseline path's last 2 nodes
};

// Same scenes/start/goal_offset as golden_all_scenes_test.cpp's kScenes (not its GOLDEN_DATA_DIR
// comparison, which has no counterpart for a mode that didn't exist in the old code — see header
// comment). expected_*_expansions recorded from an actual run when this test was written.
const std::vector<SceneSetup> kScenes = {
    {"NarrowPassage", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 98, 259},
    {"Stairs", Point_3(0.1, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 34, 25},
    // Never had a path even in the old code for this exact start/goal (tests/golden_data/
    // TwoFlatSurfaces_astar.json: "success": false) — not this feature's regression to track, hence
    // no expected expansion counts; see the "size < 2" skip below.
    {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0.0), Vector_3(0.0, 0.0, 0.0), 0, 0},
    {"LongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 28, 16},
    {"LongLongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 75, 144},
    {"Flat", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 11, 10},
    {"LongStairsComplete", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 26, 50},
    {"LongStairsExp", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 189, 10},
    {"ThreePathsScene", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 323, 363},
    {"Stairs_Up_Down", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0), 33, 73},
    {"ThreePathsNAS", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 1.0, 0.0), 115, 33},
};

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    return ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
}

// Same constants as golden_all_scenes_test.cpp's old_constants_config, INCLUDING max_expansions left
// at its default (0 = unlimited) — some of these scenes are only solved after several thousand
// expansions even in today's unmodified single-target mode (measured while writing this test), so
// capping this baseline would silently make it diverge from the actual reference behaviour it's
// meant to reproduce.
AstarSearchConfig baseline_config(const SceneSetup& setup, const Point_3& goal) {
    AstarSearchConfig c;
    c.start_position = setup.start;
    c.start_stance_foot = StanceFoot::Right;
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

// A flat square target centered exactly on a real, already-reachable node's own centroid and yaw
// (generous margins) — guarantees that node itself satisfies the resulting FootGoal, so a mode-2
// search seeded with two such targets is guaranteed at least one solution: the baseline path itself.
AstarSearchConfig::FootGoal target_around(const Node* n, double half_extent, double yaw_half_width_deg) {
    double x = CGAL::to_double(n->centroid.x()), y = CGAL::to_double(n->centroid.y()), z = CGAL::to_double(n->centroid.z());
    AstarSearchConfig::FootGoal g;
    g.region = std::vector<Point_3>{Point_3(x - half_extent, y - half_extent, z), Point_3(x + half_extent, y - half_extent, z),
                                     Point_3(x + half_extent, y + half_extent, z), Point_3(x - half_extent, y + half_extent, z)};
    double yaw_deg = n->foot_yaw * 180.0 / M_PI;
    g.yaw_range = std::make_pair((yaw_deg - yaw_half_width_deg) / 180.0 * M_PI, (yaw_deg + yaw_half_width_deg) / 180.0 * M_PI);
    return g;
}

} // namespace

int run_dual_target_all_scenes() {
    ReachabilityModel reachability = make_forward_reachability();

    for (const SceneSetup& setup : kScenes) {
        std::printf("\n=== %s ===\n", setup.name);
        config::Scenario scenario = config::load_scenario(setup.name);
        Point_3 goal = scenario.surfaces.back().centroid + setup.goal_offset;

        AstarSearchConfig base_cfg = baseline_config(setup, goal);
        AstarSearch baseline(scenario.surfaces, reachability, base_cfg);
        baseline.search();
        const auto& base_path = baseline.result_path();
        // Reported, not asserted: golden_all_scenes_test.cpp's own tests/golden_data/*_astar.json
        // confirms at least one of these scenes (TwoFlatSurfaces, this exact start/goal) never had a
        // path even in the old code — a pre-existing property of the scenario, not a regression.
        if (base_path.size() < 2) {
            std::printf("%s: mode 0 (comportement actuel) ne donne pas de chemin exploitable (%s) — scene ignoree pour le mode 2\n",
                        setup.name, base_path.empty() ? "aucun chemin" : "chemin a 1 seul pas");
            continue;
        }
        if (setup.expected_baseline_expansions > 0) {
            check(baseline.expansion_count() == setup.expected_baseline_expansions,
                  std::string(setup.name) + ": (reference) nombre d'expansions du mode 0 inchange (" +
                      std::to_string(baseline.expansion_count()) + ")");
        }

        Node* last = base_path.back();
        Node* prev = base_path[base_path.size() - 2];

        AstarSearchConfig dual_cfg = baseline_config(setup, goal);
        // Safety net only (see AstarSearchConfig::max_expansions's own doc comment) — a solution is
        // guaranteed to exist by construction (the baseline path itself), this just bounds how long
        // the harder, jointly-constrained search is allowed to look for it or an equally valid one.
        dual_cfg.max_expansions = 50000;
        dual_cfg.foot_goals[static_cast<size_t>(last->stance_foot)] = target_around(last, 0.15, 40.0);
        dual_cfg.foot_goals[static_cast<size_t>(prev->stance_foot)] = target_around(prev, 0.15, 40.0);

        AstarSearch dual(scenario.surfaces, reachability, dual_cfg);
        dual.search();
        const auto& dual_path = dual.result_path();
        check(!dual_path.empty(), std::string(setup.name) + ": mode 2 (cibles construites autour du chemin de reference) trouve un chemin");
        if (dual_path.size() >= 2) {
            Node* d_last = dual_path.back();
            Node* d_prev = dual_path[dual_path.size() - 2];
            check(d_last->stance_foot != d_prev->stance_foot,
                  std::string(setup.name) + ": mode 2 termine bien sur deux pieds opposes consecutifs");
        }
        if (setup.expected_dual_expansions > 0) {
            check(dual.expansion_count() == setup.expected_dual_expansions,
                  std::string(setup.name) + ": mode 2 nombre d'expansions inchange (" + std::to_string(dual.expansion_count()) + ")");
        }
        std::printf("%s: baseline=%d expansions, mode2=%d expansions\n", setup.name, baseline.expansion_count(), dual.expansion_count());

        // Large-scale infeasibility check (one scene is enough — the precise, controlled case
        // already lives in foot_goals_test.cpp): move one of the two targets 50m away and confirm a
        // clean "no path", not a false success, on a real multi-surface scenario.
        if (std::string(setup.name) == "Flat") {
            AstarSearchConfig infeasible_cfg = baseline_config(setup, goal);
            infeasible_cfg.max_expansions = 2000;
            AstarSearchConfig::FootGoal moved = target_around(last, 0.15, 40.0);
            for (auto& p : std::get<std::vector<Point_3>>(moved.region)) p = Point_3(p.x() + 50.0, p.y(), p.z());
            infeasible_cfg.foot_goals[static_cast<size_t>(last->stance_foot)] = moved;
            infeasible_cfg.foot_goals[static_cast<size_t>(prev->stance_foot)] = target_around(prev, 0.15, 40.0);
            AstarSearch infeasible(scenario.surfaces, reachability, infeasible_cfg);
            infeasible.search();
            check(infeasible.result_path().empty(), std::string(setup.name) + ": mode 2 avec une cible a 50m est correctement infaisable");
        }
    }

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All dual_target_all_scenes tests passed\n");
    return 0;
}
