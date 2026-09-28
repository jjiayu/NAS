// The combined scenario: pick up a cube resting in the scene, then place it and step onto it to
// cross an obstacle a normal footstep can't reach directly -- like g1motion's boxcube.py (walk to
// a box, grasp it, place it at the foot of a raised staircase, step onto it). Reuses NAS's own
// StairsGap scenario/reachability combination, already proven (without a pickup step -- the cube
// there starts carried from the very beginning) to converge in 218 expansions with the former 15 cm-wide cube (now 30 cm wide, see docs/cube-extension-mechanism.md; see
// docs/cube-session-2026-09-23-handoff.md and scenario_library.cpp's own header comment on
// StairsGap): same geometry, same reachability, only now the robot starts empty-handed and must
// first walk to the cube's own resting position before the previously-proven place-and-step
// sequence can happen at all.
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

int run_cube_pickup_and_placement() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED_clamp_z18.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED_clamp_z18.obj", "LF", "RF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("StairsGap");

    AstarSearchConfig cfg;
    cfg.start_position = Point_3(0.1, 0.0, 0.0); // the proven StairsGap+cube setup, see file header
    cfg.start_stance_foot = StanceFoot::Right;
    cfg.expansion_params.rotation_enabled = true;
    cfg.expansion_params.yaw_discretization_num = 3;
    cfg.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    cfg.cube_half_extent = 0.15;
    cfg.cube_height = 0.15;
    cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = AstarSearchConfig::FootGoal{sc.surfaces.back().centroid, std::nullopt}; // Step 4
    // Generous margin over the 218-expansion proven baseline (which starts the cube InHand, no
    // pickup detour) -- this scenario adds a mandatory walk-to-the-cube-then-pick-it-up prefix, an
    // unmeasured but modest addition to the same otherwise-unchanged funnel. Measured below, not
    // assumed (see the printed expansion count), matching this codebase's own convention for a new
    // scenario establishing its own baseline (dual_target_all_scenes_test.cpp).
    cfg.max_expansions = 5000;

    // The cube rests on the floor, well short of its x<=0.55 edge and clear of the start node's own
    // degenerate 1-point patch at exactly start_position -- broad enough to reliably overlap an
    // early real (>=3-vertex) patch without needing an exact hand-tuned reachable point.
    AstarSearchConfig::SceneCube cube;
    AstarSearchConfig::FootGoal g;
    g.region = std::vector<Point_3>{Point_3(0.15, -0.3, 0.0), Point_3(0.45, -0.3, 0.0), Point_3(0.45, 0.3, 0.0),
                                     Point_3(0.15, 0.3, 0.0)};
    cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
    cfg.scene_cubes = {cube};

    AstarSearch search(sc.surfaces, reach, cfg);
    search.search();
    std::printf("cube_pickup_and_placement: %d expansions, path size %zu\n", search.expansion_count(),
                search.result_path().size());
    const auto& path = search.result_path();
    check(!path.empty(), "a full pickup-then-place-then-cross path is found within the expansion budget");
    if (path.empty()) {
        if (g_failures > 0) { std::fprintf(stderr, "%d test(s) FAILED\n", g_failures); return 1; }
        return 0;
    }

    check(path.front()->cube_state == CubeState::None, "the path starts empty-handed (None), not carrying a cube");

    int idx_in_hand = -1, idx_placed_active = -1, idx_placed_inactive = -1;
    for (size_t i = 0; i < path.size(); ++i) {
        if (idx_in_hand < 0 && path[i]->cube_state == CubeState::InHand) idx_in_hand = static_cast<int>(i);
        if (idx_placed_active < 0 && path[i]->cube_state == CubeState::PlacedActive) idx_placed_active = static_cast<int>(i);
        if (idx_placed_inactive < 0 && path[i]->cube_state == CubeState::PlacedInactive) idx_placed_inactive = static_cast<int>(i);
    }
    check(idx_in_hand >= 0, "the path passes through InHand (the cube was picked up)");
    check(idx_placed_active >= 0, "the path passes through PlacedActive (the cube was placed)");
    check(idx_placed_inactive >= 0, "the path passes through PlacedInactive (a step was taken onto the cube)");
    check(idx_in_hand >= 0 && idx_placed_active > idx_in_hand,
          "PlacedActive comes strictly after InHand along the path");
    check(idx_placed_active >= 0 && idx_placed_inactive > idx_placed_active,
          "PlacedInactive comes strictly after PlacedActive along the path");
    if (idx_placed_inactive >= 0) {
        check(path[static_cast<size_t>(idx_placed_inactive)]->surface_id == kOnCubeSurfaceId,
              "the PlacedInactive node's surface_id is the on-cube sentinel (a real step was taken onto the cube)");
    }

    check(path.back()->surface_id == sc.surfaces.back().surface_id,
          "the path ends on the goal surface (Step 4): the obstacle was actually crossed, not avoided");
    check((path.back()->cubes_picked_up & 0x1) != 0,
          "the scene's only cube is marked picked-up by the end of the path");

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All cube_pickup_and_placement tests passed\n");
    return 0;
}
