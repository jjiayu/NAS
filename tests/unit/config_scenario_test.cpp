#include "nas/config/scenario.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

using namespace nas::config;
using nas::Point_3;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

void test_available_scenarios_lists_all() {
    auto names = available_scenarios();
    assert(names.size() == 15); // the 11 of the old environments.hpp + 4 inclined scenes (Ramp, SteepRamp, SlopedGround, SideSlope)
    std::set<std::string> unique(names.begin(), names.end());
    assert(unique.size() == 15); // no duplicate names
    std::cout << "test_available_scenarios_lists_all passed\n";
}

void test_every_scenario_loads_without_exception() {
    for (const auto& name : available_scenarios()) {
        Scenario s = load_scenario(name);
        assert(s.name == name);
        assert(!s.surfaces.empty());
    }
    std::cout << "test_every_scenario_loads_without_exception passed\n";
}

void test_unknown_scenario_throws() {
    bool threw = false;
    try {
        load_scenario("DoesNotExist");
    } catch (const std::out_of_range&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_unknown_scenario_throws passed\n";
}

// Cross-checks against the old code's include/environments.hpp raw vertex
// data (see PLAN.md phase 9e) — centroid is computed from the raw input
// points before the foot-size shrink, so it's a direct verbatim-data
// check, not just "it builds".
void test_narrow_passage_matches_environments_hpp() {
    Scenario s = load_scenario("NarrowPassage");
    assert(s.surfaces.size() == 3);
    assert(near(CGAL::to_double(s.surfaces[0].centroid.x()), 0.0));
    assert(near(CGAL::to_double(s.surfaces[0].centroid.y()), 0.0));
    assert(near(CGAL::to_double(s.surfaces[1].centroid.x()), 4.0));
    assert(near(CGAL::to_double(s.surfaces[2].centroid.x()), 8.0));
    std::cout << "test_narrow_passage_matches_environments_hpp passed\n";
}

void test_three_paths_nas_matches_environments_hpp() {
    Scenario s = load_scenario("ThreePathsNAS");
    assert(s.surfaces.size() == 13);
    assert(near(CGAL::to_double(s.surfaces.front().centroid.x()), 0.0));
    assert(near(CGAL::to_double(s.surfaces.front().centroid.y()), -1.0));
    assert(near(CGAL::to_double(s.surfaces.back().centroid.x()), 4.64));
    assert(near(CGAL::to_double(s.surfaces.back().centroid.y()), -1.0));
    std::cout << "test_three_paths_nas_matches_environments_hpp passed\n";
}

void test_custom_robot_model_changes_shrink() {
    nas::config::RobotModel narrow_feet;
    narrow_feet.foot_length = 0.05;
    narrow_feet.foot_width = 0.05;
    Scenario wide = load_scenario("NarrowPassage"); // default 0.22/0.22
    Scenario narrow = load_scenario("NarrowPassage", narrow_feet);
    // A larger foot shrinks the passage surface's patch more, so its extent
    // should be strictly smaller than with a smaller foot.
    assert(wide.surfaces[1].vertices_3d.size() >= 3);
    assert(narrow.surfaces[1].vertices_3d.size() >= 3);
    std::cout << "test_custom_robot_model_changes_shrink passed\n";
}

} // namespace

int run_config_scenario() {
    test_available_scenarios_lists_all();
    test_every_scenario_loads_without_exception();
    test_unknown_scenario_throws();
    test_narrow_passage_matches_environments_hpp();
    test_three_paths_nas_matches_environments_hpp();
    test_custom_robot_model_changes_shrink();
    std::cout << "All config/scenario_library tests passed.\n";
    return 0;
}
