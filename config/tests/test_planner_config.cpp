#include "nas/config/planner_config.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace nas::config;
using namespace nas;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

std::string data_path(const std::string& filename) {
    return std::string(TEST_DATA_DIR) + "/" + filename;
}

void test_full_config_overrides_every_field() {
    PlannerConfig config = load_planner_config(data_path("valid_planner_config.json"));

    assert(near(CGAL::to_double(config.astar.start_position.x()), 0.0));
    assert(near(CGAL::to_double(config.astar.goal_location.x()), 8.0));
    assert(config.astar.start_stance_foot == StanceFoot::Right);
    assert(config.astar.goal_stance_foot == StanceFoot::Left);
    assert(config.astar.distance_metric == DistanceMetric::Epa);
    assert(near(config.astar.heuristic_weight, 10.0));
    assert(near(config.astar.node_similarity_threshold, 0.02));
    assert(config.astar.expansion_params.rotation_enabled == true);
    assert(config.astar.expansion_params.yaw_discretization_num == 3);
    assert(near(config.astar.expansion_params.yaw_angle_increment, 10.0 / 180.0 * M_PI));
    assert(config.astar.expansion_params.cycle_detection_enabled == true);

    assert(near(config.qp.alpha_weight, 10.0));
    assert(config.qp.rotation_enabled == true);
    assert(near(config.qp.hessian_regularization, 1e-8));

    std::cout << "test_full_config_overrides_every_field passed\n";
}

void test_minimal_config_keeps_struct_defaults() {
    // Only start/goal set — everything else must fall back to
    // AstarSearchConfig/FootstepQPConfig's own default member initializers,
    // not some duplicated default baked into the loader.
    AstarSearchConfig defaults;
    FootstepQPConfig qp_defaults;

    PlannerConfig config = load_planner_config(data_path("minimal_planner_config.json"));

    assert(near(CGAL::to_double(config.astar.start_position.x()), 1.0));
    assert(near(CGAL::to_double(config.astar.goal_location.x()), 4.0));
    assert(config.astar.start_stance_foot == defaults.start_stance_foot);
    assert(near(config.astar.heuristic_weight, defaults.heuristic_weight));
    assert(near(config.astar.node_similarity_threshold, defaults.node_similarity_threshold));
    assert(config.astar.expansion_params.rotation_enabled == defaults.expansion_params.rotation_enabled);
    assert(near(config.qp.alpha_weight, qp_defaults.alpha_weight));

    std::cout << "test_minimal_config_keeps_struct_defaults passed\n";
}

void test_goal_offset_is_kept_for_the_caller_to_resolve() {
    // "goal_offset" = last surface's centroid + offset (old constants.hpp semantics);
    // only the scenario knows that centroid, so the loader just hands the offset back.
    PlannerConfig config = load_planner_config(data_path("goal_offset_planner_config.json"));
    assert(config.goal_offset.has_value());
    assert(near(CGAL::to_double(config.goal_offset->y()), 1.0));
    PlannerConfig absolute = load_planner_config(data_path("minimal_planner_config.json"));
    assert(!absolute.goal_offset.has_value());
    std::cout << "test_goal_offset_is_kept_for_the_caller_to_resolve passed\n";
}

void test_distance_metric_is_parsed() {
    PlannerConfig config = load_planner_config(data_path("euclidean_planner_config.json"));
    assert(config.astar.distance_metric == DistanceMetric::Euclidean);
    bool threw = false;
    try { load_planner_config(data_path("gjk_planner_config.json")); } catch (const std::runtime_error&) { threw = true; }
    assert(threw); // GJK was dropped
    std::cout << "test_distance_metric_is_parsed passed\n";
}

void test_resolve_goal_uses_the_last_surface_centroid() {
    PlannerConfig config = load_planner_config(data_path("goal_offset_planner_config.json")); // goal_offset (0, 1, 0)
    Scenario scenario = load_scenario("Flat");
    resolve_goal(config, scenario);
    Point_3 expected = scenario.surfaces.back().centroid + Vector_3(0.0, 1.0, 0.0);
    assert(near(CGAL::to_double(config.astar.goal_location.x()), CGAL::to_double(expected.x())));
    assert(near(CGAL::to_double(config.astar.goal_location.y()), CGAL::to_double(expected.y())));
    // an absolute goal is left alone
    PlannerConfig absolute = load_planner_config(data_path("minimal_planner_config.json"));
    Point_3 before = absolute.astar.goal_location;
    resolve_goal(absolute, scenario);
    assert(absolute.astar.goal_location == before);
    std::cout << "test_resolve_goal_uses_the_last_surface_centroid passed\n";
}

void test_goal_surface() {
    // the goal as a surface: parsed, resolved against the scenario (index checked), no goal position for the QP
    PlannerConfig config = load_planner_config(data_path("goal_surface_planner_config.json"));
    assert(config.astar.goal_surface_id == 2);
    Scenario ground = load_scenario("Stairs"); // 5 surfaces
    resolve_goal(config, ground);
    assert(!qp_goal(config).has_value());
    PlannerConfig position = load_planner_config(data_path("minimal_planner_config.json"));
    assert(position.astar.goal_surface_id == -1 && qp_goal(position).has_value());
    Scenario flat = load_scenario("Flat"); // 1 surface: index 2 does not exist
    bool threw = false;
    try { resolve_goal(config, flat); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    // two goals at once are refused
    threw = false;
    try { load_planner_config(data_path("two_goals_planner_config.json")); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "test_goal_surface passed\n";
}

void test_missing_astar_section_throws() {
    bool threw = false;
    try {
        load_planner_config(data_path("missing_astar_section.json"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_missing_astar_section_throws passed\n";
}

void test_missing_start_position_throws() {
    bool threw = false;
    try {
        load_planner_config(data_path("missing_start_position.json"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_missing_start_position_throws passed\n";
}

void test_nonexistent_file_throws() {
    bool threw = false;
    try {
        load_planner_config(data_path("does_not_exist.json"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_nonexistent_file_throws passed\n";
}

} // namespace

int main() {
    test_full_config_overrides_every_field();
    test_minimal_config_keeps_struct_defaults();
    test_goal_offset_is_kept_for_the_caller_to_resolve();
    test_distance_metric_is_parsed();
    test_resolve_goal_uses_the_last_surface_centroid();
    test_goal_surface();
    test_missing_astar_section_throws();
    test_missing_start_position_throws();
    test_nonexistent_file_throws();
    std::cout << "All config/planner_config tests passed.\n";
    return 0;
}
