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

void test_inner_margin_erodes_the_footprint() {
    // NarrowPassage's bridge (surface 1) is 0.24 m wide in y: 0.24 - 2*margin remains.
    Scenario def = load_scenario("NarrowPassage"); // kDefaultInnerMargin = 0.11
    Scenario thin = load_scenario("NarrowPassage", 0.05);
    assert(near(local_extent_y(def.surfaces[1]), 0.24 - 2 * 0.11));
    assert(near(local_extent_y(thin.surfaces[1]), 0.24 - 2 * 0.05));
    std::cout << "test_inner_margin_erodes_the_footprint passed\n";
}

void test_surface_thinner_than_the_margin_is_dropped() {
    // 0.13 > 0.24 / 2: the bridge collapses. It is dropped (not mirrored, as the old foot hack
    // did), and ids stay equal to indices, so the search's surfaces[surface_id] indexing holds.
    Scenario s = load_scenario("NarrowPassage", 0.13);
    assert(s.surfaces.size() == 2);
    for (size_t i = 0; i < s.surfaces.size(); ++i) assert(s.surfaces[i].surface_id == static_cast<int>(i));
    std::cout << "test_surface_thinner_than_the_margin_is_dropped passed\n";
}

} // namespace

int run_config_scenario() {
    test_available_scenarios_lists_all();
    test_every_scenario_loads_without_exception();
    test_unknown_scenario_throws();
    test_narrow_passage_matches_environments_hpp();
    test_three_paths_nas_matches_environments_hpp();
    test_inner_margin_erodes_the_footprint();
    test_surface_thinner_than_the_margin_is_dropped();
    std::cout << "All config/scenario_library tests passed.\n";
    return 0;
}
