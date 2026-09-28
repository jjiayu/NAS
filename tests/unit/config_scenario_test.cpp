#include "nas/config/scenario.hpp"

#include <algorithm>
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

double local_extent_y(const nas::Surface& s) {
    double lo = 1e9, hi = -1e9;
    for (const auto& v : s.vertices_2d) {
        lo = std::min(lo, CGAL::to_double(v.y()));
        hi = std::max(hi, CGAL::to_double(v.y()));
    }
    return hi - lo;
}

void test_scenario_surfaces_are_raw_and_erosion_is_applied_on_request() {
    // load_scenario returns the scene's true geometry: NarrowPassage's bridge (surface 1) is 0.24 m
    // wide in y. The margin is applied by erode_by_id/erode_surfaces (and by AstarSearch itself).
    Scenario s = load_scenario("NarrowPassage");
    assert(near(local_extent_y(s.surfaces[1]), 0.24));
    assert(near(local_extent_y(*nas::erode_by_id(s.surfaces, nas::kDefaultInnerMargin)[1]), 0.24 - 2 * 0.11));
    assert(near(local_extent_y(*nas::erode_by_id(s.surfaces, 0.05)[1]), 0.24 - 2 * 0.05));
    std::cout << "test_scenario_surfaces_are_raw_and_erosion_is_applied_on_request passed\n";
}

void test_surface_thinner_than_the_margin_keeps_its_index() {
    // 0.13 > 0.24 / 2: the bridge collapses. It is unusable (not mirrored, as the old foot hack did)
    // but keeps its slot, so surface ids never shift.
    Scenario s = load_scenario("NarrowPassage");
    auto by_id = nas::erode_by_id(s.surfaces, 0.13);
    assert(by_id.size() == s.surfaces.size() && !by_id[1] && by_id[0] && by_id[2]);
    std::vector<nas::Surface> compact = nas::erode_surfaces(s.surfaces, 0.13);
    assert(compact.size() == 2 && compact[0].surface_id == 0 && compact[1].surface_id == 2);
    std::cout << "test_surface_thinner_than_the_margin_keeps_its_index passed\n";
}

} // namespace

int run_config_scenario() {
    test_available_scenarios_lists_all();
    test_every_scenario_loads_without_exception();
    test_unknown_scenario_throws();
    test_narrow_passage_matches_environments_hpp();
    test_three_paths_nas_matches_environments_hpp();
    test_scenario_surfaces_are_raw_and_erosion_is_applied_on_request();
    test_surface_thinner_than_the_margin_keeps_its_index();
    std::cout << "All config/scenario_library tests passed.\n";
    return 0;
}
