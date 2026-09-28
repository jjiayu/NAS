// AstarSearchConfig::scene_cubes / expand_cube_pickup, exercised through a real AstarSearch::
// search() run (not a direct function call, see tests/unit/cube_pickup_test.cpp for that) --
// confirms the mechanism is correctly wired into search()'s main loop (constructor validation +
// start-node state + the cube_state dispatch), using the on_child hook to observe pickup actually
// firing: with a plain position-only goal, a picked-up-but-never-placed cube is a strictly
// dominated detour (extra edge cost, no heuristic benefit), so it is never expected to survive
// onto the final result_path() itself -- see cube_pickup_and_placement_test.cpp for a scenario
// where picking the cube up (and placing it) is actually necessary to reach the goal at all.
//
// Scenario: Flat, start (0,0,0) facing +x, right stance (same setup as foot_goals_test.cpp).
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

} // namespace

int run_cube_pickup_search() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("Flat");

    auto base_config = [&]() {
        AstarSearchConfig cfg;
        cfg.start_position = Point_3(0, 0, 0);
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = AstarSearchConfig::FootGoal{Point_3(1.0, 0.0, 0.0), std::nullopt};
        cfg.expansion_params.rotation_enabled = true;
        cfg.cube_half_extent = 0.15;
        cfg.cube_height = 0.15;
        cfg.max_expansions = 2000;
        return cfg;
    };

    // A generous box straddling both plausible lateral offsets, well clear of the start node's own
    // degenerate 1-point patch at exactly (0,0,0) -- see foot_goals_test.cpp's own note on why a
    // target coincident with that node needs the (different, deliberately untested here) point/EPA
    // fallback branch of foot_goal_satisfied instead of the ordinary plane-slice path.
    auto broad_box = [](double x_lo, double x_hi) {
        return std::vector<Point_3>{Point_3(x_lo, -0.3, 0.0), Point_3(x_hi, -0.3, 0.0), Point_3(x_hi, 0.3, 0.0),
                                     Point_3(x_lo, 0.3, 0.0)};
    };

    // (A) mode 1 (single slot, left foot): a real search must, at some point while expanding
    // ordinary nodes, also produce a pickup child transitioning that node to InHand.
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = broad_box(0.05, 0.45);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};

        bool saw_pickup = false;
        cfg.on_child = [&](int, const Node& child, ChildAction) {
            if (child.cube_state == CubeState::InHand) saw_pickup = true;
        };

        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(A) mode 1 pickup: the plain goal is still reached");
        check(saw_pickup, "(A) mode 1 pickup: at least one InHand child was produced during the search");
    }

    // (B) mode 2 (both slots): a symmetric stance around the scene cube, tested on consecutive
    // opposite-foot steps exactly like foot_goals' own closing-stance mode.
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal left_g, right_g;
        left_g.region = broad_box(0.05, 0.45);
        right_g.region = broad_box(0.05, 0.45);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = right_g;
        cfg.scene_cubes = {cube};

        bool saw_pickup = false;
        cfg.on_child = [&](int, const Node& child, ChildAction) {
            if (child.cube_state == CubeState::InHand) saw_pickup = true;
        };

        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(B) mode 2 pickup: the plain goal is still reached");
        check(saw_pickup, "(B) mode 2 pickup: at least one InHand child was produced during the search");
    }

    // Negative controls at construction (mirroring foot_goals_test.cpp's own style).
    auto expect_throw = [&](AstarSearchConfig cfg, const std::string& what) {
        bool threw = false;
        try {
            AstarSearch bad(sc.surfaces, reach, cfg);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        check(threw, what);
    };

    {
        AstarSearchConfig cfg = base_config();
        cfg.cube_half_extent = 0.0; // scene_cubes without any cube geometry to place/step with
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.0, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "scene_cubes set with cube_half_extent <= 0 leve une erreur claire");
    }
    for (double too_small : {0.10, kDefaultInnerMargin}) {
        AstarSearchConfig cfg = base_config();
        cfg.cube_half_extent = too_small; // top of the cube would vanish under the inner margin
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.0, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "cube_half_extent <= inner_margin (aucun dessus praticable) leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::SceneCube cube; // neither pickup_affordance slot set
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "un SceneCube sans aucune case d'affordance remplie leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = std::vector<Point_3>{Point_3(0, 0, 0), Point_3(0.1, 0, 0)}; // only 2 vertices
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "une affordance polytope a moins de 3 sommets leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.0, 0.0);
        g.yaw_range = std::make_pair(0.5, 0.1); // max < min
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "un yaw_range d'affordance avec max < min leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        cfg.expansion_params.rotation_enabled = false;
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.0, 0.0);
        g.yaw_range = std::make_pair(0.0, 0.1);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "un yaw_range d'affordance sans rotation_enabled leve une erreur claire");
    }
    {
        // Transitively implied by the existing foot_goals-mode-2 x cube_half_extent>0 exclusion
        // (scene_cubes requires cube_half_extent>0), not a separate check -- confirmed here so a
        // future refactor of either exclusion can't silently drop this combination's safety.
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal left_g, right_g;
        left_g.region = Point_3(0.3, 0.15, 0.0);
        right_g.region = Point_3(0.3, -0.15, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Right)] = right_g;
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.0, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        cfg.scene_cubes = {cube};
        expect_throw(cfg, "scene_cubes + foot_goals mode 2 leve toujours une erreur (exclusion transitive)");
    }

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All cube_pickup_search tests passed\n");
    return 0;
}
