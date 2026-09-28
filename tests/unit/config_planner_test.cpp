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
    assert(config.astar.foot_goals[0].has_value() && !config.astar.foot_goals[1].has_value());
    assert(near(CGAL::to_double(std::get<Point_3>(config.astar.foot_goals[0]->region).x()), 8.0));
    assert(config.astar.start_stance_foot == StanceFoot::Right);
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
    assert(near(CGAL::to_double(std::get<Point_3>(config.astar.foot_goals[0]->region).x()), 4.0));
    assert(config.astar.start_stance_foot == defaults.start_stance_foot);
    assert(near(config.astar.heuristic_weight, defaults.heuristic_weight));
    assert(near(config.astar.node_similarity_threshold, defaults.node_similarity_threshold));
    assert(config.astar.expansion_params.rotation_enabled == defaults.expansion_params.rotation_enabled);
    assert(near(config.qp.alpha_weight, qp_defaults.alpha_weight));

    std::cout << "test_minimal_config_keeps_struct_defaults passed\n";
}

void test_goal_offset_is_kept_for_the_caller_to_resolve() {
    // "offset" = last surface's centroid + offset (old constants.hpp semantics); only the scenario
    // knows that centroid, so the loader just parks it in pending_foot_goal_regions.
    PlannerConfig config = load_planner_config(data_path("goal_offset_planner_config.json"));
    assert(config.pending_foot_goal_regions[0].has_value());
    assert(config.pending_foot_goal_regions[0]->offset.has_value());
    assert(near(CGAL::to_double(config.pending_foot_goal_regions[0]->offset->y()), 1.0));
    PlannerConfig absolute = load_planner_config(data_path("minimal_planner_config.json"));
    assert(!absolute.pending_foot_goal_regions[0].has_value());
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
    PlannerConfig config = load_planner_config(data_path("goal_offset_planner_config.json")); // offset (0, 1, 0)
    Scenario scenario = load_scenario("Flat");
    resolve_goal(config, scenario);
    Point_3 expected = scenario.surfaces.back().centroid + Vector_3(0.0, 1.0, 0.0);
    Point_3 resolved = std::get<Point_3>(config.astar.foot_goals[0]->region);
    assert(near(CGAL::to_double(resolved.x()), CGAL::to_double(expected.x())));
    assert(near(CGAL::to_double(resolved.y()), CGAL::to_double(expected.y())));
    // an absolute goal is left alone
    PlannerConfig absolute = load_planner_config(data_path("minimal_planner_config.json"));
    Point_3 before = std::get<Point_3>(absolute.astar.foot_goals[0]->region);
    resolve_goal(absolute, scenario);
    assert(std::get<Point_3>(absolute.astar.foot_goals[0]->region) == before);
    std::cout << "test_resolve_goal_uses_the_last_surface_centroid passed\n";
}

void test_goal_surface() {
    // the goal as a surface: parsed, resolved against the scenario (index checked), no goal position for the QP
    PlannerConfig config = load_planner_config(data_path("goal_surface_planner_config.json"));
    assert(std::holds_alternative<int>(config.astar.foot_goals[0]->region) && std::get<int>(config.astar.foot_goals[0]->region) == 2);
    Scenario ground = load_scenario("Stairs"); // 5 surfaces
    resolve_goal(config, ground);
    assert(!qp_goal(config).has_value());
    PlannerConfig position = load_planner_config(data_path("minimal_planner_config.json"));
    assert(std::holds_alternative<Point_3>(position.astar.foot_goals[0]->region) && qp_goal(position).has_value());
    Scenario flat = load_scenario("Flat"); // 1 surface: index 2 does not exist
    bool threw = false;
    try { resolve_goal(config, flat); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    // two region keys for the same slot at once are refused
    threw = false;
    try { load_planner_config(data_path("two_goals_planner_config.json")); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "test_goal_surface passed\n";
}

void test_missing_foot_goals_throws() {
    bool threw = false;
    try {
        load_planner_config(data_path("missing_foot_goals.json"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_missing_foot_goals_throws passed\n";
}

void test_yaw_change_weight() {
    // optional edge cost on rotating: 0 (the paper's cost) unless the config sets it
    assert(near(load_planner_config(data_path("minimal_planner_config.json")).astar.yaw_change_weight, 0.0));
    assert(near(load_planner_config(data_path("yaw_cost_planner_config.json")).astar.yaw_change_weight, 0.1));
    assert(near(load_planner_config(data_path("minimal_planner_config.json")).astar.heading_weight, 0.0));
    assert(near(load_planner_config(data_path("yaw_cost_planner_config.json")).astar.heading_weight, 0.2));
    // step cost weight: 1 (the paper's edge cost) by default, 0 ignores it, a negative weight is refused
    assert(near(load_planner_config(data_path("minimal_planner_config.json")).astar.step_weight, 1.0));
    assert(near(load_planner_config(data_path("yaw_cost_planner_config.json")).astar.step_weight, 0.5));
    assert(near(load_planner_config(data_path("yaw_cost_planner_config.json")).astar.heuristic_weight, 0.0));
    bool threw = false;
    try { load_planner_config(data_path("negative_weight_planner_config.json")); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "test_yaw_change_weight passed\n";
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

int run_config_planner() {
    test_full_config_overrides_every_field();
    test_minimal_config_keeps_struct_defaults();
    test_goal_offset_is_kept_for_the_caller_to_resolve();
    test_distance_metric_is_parsed();
    test_resolve_goal_uses_the_last_surface_centroid();
    test_goal_surface();
    test_yaw_change_weight();
    test_missing_astar_section_throws();
    test_missing_start_position_throws();
    test_missing_foot_goals_throws();
    test_nonexistent_file_throws();
    std::cout << "All config/planner_config tests passed.\n";
    return 0;
}
