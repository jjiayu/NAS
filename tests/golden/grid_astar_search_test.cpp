#include "nas/planners/grid_astar_search.hpp"
#include "nas/config/scenario.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using namespace nas;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    std::vector<ReachabilityEntry> entries = {
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

void test_grid_environment_rasterizes_a_flat_surface() {
    config::Scenario scenario = config::load_scenario("Flat"); // single 7.45x2 rectangle
    GridEnvironment grid(0.1);
    grid.initialize_from_surfaces(scenario.surfaces);

    // The surface spans roughly x in [-2, 5.45], y in [-1, 1] (see
    // config/src/scenario_library.cpp's kFlat) — its center should land on
    // a traversable cell at height 0.
    auto [gx, gy] = grid.world_to_grid(Point_3(1.0, 0.0, 0.0));
    assert(grid.is_traversable(gx, gy));
    assert(near(grid.get_cell(gx, gy).height, 0.0, 1e-6));

    // Far outside the surface (well past the padding) must not be traversable.
    auto [far_gx, far_gy] = grid.world_to_grid(Point_3(100.0, 100.0, 0.0));
    assert(!grid.is_traversable(far_gx, far_gy));

    std::cout << "test_grid_environment_rasterizes_a_flat_surface passed\n";
}

GridAstarSearchConfig make_default_config(const Point_3& start, const Point_3& goal) {
    GridAstarSearchConfig config;
    config.start_position = start;
    config.start_stance_foot = StanceFoot::Right;
    config.goal_location = goal;
    config.goal_stance_foot = StanceFoot::Left;
    // Deliberately coarse and rotation-free: these tests are about
    // correctness of the port, not resolution, and a fine grid with 7
    // candidate yaws per cell multiplies node count (and memory) for no
    // extra coverage.
    config.cell_size = 0.1;
    config.search_radius_m = 1.5;
    config.heuristic_weight = 10.0;
    config.expansion_params.rotation_enabled = false;
    return config;
}

void test_flat_scene_finds_a_path() {
    config::Scenario scenario = config::load_scenario("Flat");
    ReachabilityModel reachability = make_forward_reachability();

    Point_3 goal(3.0, 0.0, 0.0);
    GridAstarSearchConfig config = make_default_config(Point_3(0.0, 0.0, 0.0), goal);

    GridAstarSearch search(scenario.surfaces, reachability, config);
    search.search();

    const auto& path = search.result_path();
    assert(!path.empty());
    assert(path.front()->depth == 0);
    assert(path.back()->stance_foot == config.goal_stance_foot);
    // Alternating stance feet all the way (biped contract).
    for (size_t i = 1; i < path.size(); ++i) {
        assert(path[i]->stance_foot != path[i - 1]->stance_foot);
    }

    auto goal_grid = search.grid_environment().world_to_grid(goal);
    auto reached_grid = search.grid_environment().world_to_grid(path.back()->centroid);
    assert(goal_grid == reached_grid);

    std::cout << "test_flat_scene_finds_a_path passed (" << path.size() << " nodes, "
              << search.expansion_count() << " expansions)\n";
}

void test_unreachable_goal_yields_no_path() {
    config::Scenario scenario = config::load_scenario("Flat"); // no surface anywhere near this goal
    ReachabilityModel reachability = make_forward_reachability();

    GridAstarSearchConfig config = make_default_config(Point_3(0.0, 0.0, 0.0), Point_3(1000.0, 1000.0, 0.0));
    // Coarser cells for this test: it must exhaust the *entire* reachable
    // set before concluding no path exists, and the default 5cm resolution
    // over Flat's ~7.45x2m surface would make that needlessly slow — the
    // point here is only "no path", not spatial precision.
    config.cell_size = 0.3;

    GridAstarSearch search(scenario.surfaces, reachability, config);
    search.search();

    assert(search.result_path().empty());
    std::cout << "test_unreachable_goal_yields_no_path passed\n";
}

// Pins the CURRENT behaviour, which matches the old astar_grid_plan
// binary (no path on NarrowPassage at 5cm cells) - NOT the CASSR paper's
// discretised A*, which does cross this scene (Table I: with rotation,
// 1851 nodes, 41 steps). Likely cause (unverified): a global grid anchored
// on the scene bounding box puts no cell center inside the ~2cm foot-shrunk
// passage, whereas the paper discretises the reachable set around the
// current foothold. See this module's README, "Open question vs. the paper".
void test_narrow_passage_is_not_crossable_at_five_cm_cells() {
    config::Scenario scenario = config::load_scenario("NarrowPassage");
    ReachabilityModel reachability = make_forward_reachability();

    GridAstarSearchConfig config = make_default_config(Point_3(0.0, 0.0, 0.0), scenario.surfaces.back().centroid);
    config.cell_size = 0.05;
    // Rotation stays off here on purpose: it doesn't change which cells
    // are traversable, it only multiplies the work by 7 candidate yaws per
    // cell. Checked 2026-09-18 with the old constants.hpp configuration
    // (5cm cells, rotation on): both the old astar_grid_plan binary and
    // this port run past 2 minutes on this scene without finding a path
    // (exhausting the reachable set with rotation on is just slow), while
    // with rotation off the same exhaustive search finishes in <1s.

    GridAstarSearch search(scenario.surfaces, reachability, config);
    search.search();

    assert(search.result_path().empty());
    std::cout << "test_narrow_passage_is_not_crossable_at_five_cm_cells passed (" << search.expansion_count()
              << " expansions before exhausting the reachable set)\n";
}

} // namespace

int run_grid_astar_search() {
    test_grid_environment_rasterizes_a_flat_surface();
    test_flat_scene_finds_a_path();
    test_unreachable_goal_yields_no_path();
    test_narrow_passage_is_not_crossable_at_five_cm_cells();
    std::cout << "All grid_astar_search tests passed.\n";
    return 0;
}
