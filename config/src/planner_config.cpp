#include "nas/config/planner_config.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace nas::config {

namespace {

using json = nlohmann::json;

Point_3 point_from_json(const json& j) {
    if (!j.is_array() || j.size() != 3) {
        throw std::runtime_error("load_planner_config: expected a [x, y, z] array");
    }
    return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

StanceFoot stance_foot_from_string(const std::string& s) {
    if (s == "Left") return StanceFoot::Left;
    if (s == "Right") return StanceFoot::Right;
    throw std::runtime_error("load_planner_config: unknown stance foot '" + s + "' (expected 'Left'/'Right')");
}

DistanceMetric distance_metric_from_string(const std::string& s) {
    if (s == "Euclidean") return DistanceMetric::Euclidean;
    if (s == "Epa") return DistanceMetric::Epa;
    throw std::runtime_error("load_planner_config: unknown distance metric '" + s + "' (expected 'Euclidean'/'Epa')");
}

AstarSearchConfig parse_astar_config(const json& j, std::optional<Vector_3>& goal_offset) {
    AstarSearchConfig config; // starts from the struct's own defaults

    const int goals = int(j.contains("goal_location")) + int(j.contains("goal_offset")) + int(j.contains("goal_surface"));
    if (!j.contains("start_position") || goals == 0) {
        throw std::runtime_error("load_planner_config: \"astar.start_position\" and one of \"astar.goal_location\" / \"astar.goal_offset\" / \"astar.goal_surface\" are required");
    }
    if (goals > 1) {
        throw std::runtime_error("load_planner_config: give ONE goal: \"astar.goal_location\", \"astar.goal_offset\" or \"astar.goal_surface\"");
    }
    config.start_position = point_from_json(j.at("start_position"));
    if (j.contains("goal_surface")) {
        config.goal_surface_id = j.at("goal_surface").get<int>();
        if (config.goal_surface_id < 0) throw std::runtime_error("load_planner_config: \"astar.goal_surface\" must be a surface index >= 0");
    } else if (j.contains("goal_location")) {
        config.goal_location = point_from_json(j.at("goal_location"));
    } else {
        Point_3 o = point_from_json(j.at("goal_offset"));
        goal_offset = o - CGAL::ORIGIN;
    }

    if (j.contains("start_stance_foot")) config.start_stance_foot = stance_foot_from_string(j.at("start_stance_foot").get<std::string>());
    if (j.contains("start_foot_yaw")) config.start_foot_yaw = j.at("start_foot_yaw").get<double>();
    if (j.contains("goal_stance_foot")) config.goal_stance_foot = stance_foot_from_string(j.at("goal_stance_foot").get<std::string>());
    if (j.contains("distance_metric")) config.distance_metric = distance_metric_from_string(j.at("distance_metric").get<std::string>());
    if (j.contains("yaw_change_weight")) config.yaw_change_weight = j.at("yaw_change_weight").get<double>();
    if (j.contains("heuristic_weight")) config.heuristic_weight = j.at("heuristic_weight").get<double>();
    if (j.contains("node_similarity_threshold")) config.node_similarity_threshold = j.at("node_similarity_threshold").get<double>();

    if (j.contains("expansion")) {
        const json& e = j.at("expansion");
        if (e.contains("rotation_enabled")) config.expansion_params.rotation_enabled = e.at("rotation_enabled").get<bool>();
        if (e.contains("yaw_discretization_num")) config.expansion_params.yaw_discretization_num = e.at("yaw_discretization_num").get<int>();
        // Stored in degrees in the JSON for readability (matches how the
        // old constants.hpp comment described foot_yaw_angle_increment).
        if (e.contains("yaw_angle_increment_deg")) config.expansion_params.yaw_angle_increment = e.at("yaw_angle_increment_deg").get<double>() / 180.0 * M_PI;
        if (e.contains("cycle_detection_enabled")) config.expansion_params.cycle_detection_enabled = e.at("cycle_detection_enabled").get<bool>();
    }

    return config;
}

FootstepQPConfig parse_qp_config(const json& j) {
    FootstepQPConfig config;
    if (j.contains("alpha_weight")) config.alpha_weight = j.at("alpha_weight").get<double>();
    if (j.contains("rotation_enabled")) config.rotation_enabled = j.at("rotation_enabled").get<bool>();
    if (j.contains("hessian_regularization")) config.hessian_regularization = j.at("hessian_regularization").get<double>();
    return config;
}

} // namespace

PlannerConfig load_planner_config(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file) {
        throw std::runtime_error("load_planner_config: could not open '" + json_path + "'");
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("load_planner_config: failed to parse '" + json_path + "': " + e.what());
    }

    if (!j.contains("astar")) {
        throw std::runtime_error("load_planner_config: '" + json_path + "' is missing the required \"astar\" section");
    }

    PlannerConfig config;
    config.astar = parse_astar_config(j.at("astar"), config.goal_offset);
    if (j.contains("qp")) {
        config.qp = parse_qp_config(j.at("qp"));
    }
    return config;
}

void resolve_goal(PlannerConfig& config, const Scenario& scenario) {
    if (config.astar.goal_surface_id >= static_cast<int>(scenario.surfaces.size())) {
        throw std::runtime_error("resolve_goal: goal_surface " + std::to_string(config.astar.goal_surface_id) + " is not a surface of scenario '" +
                                 scenario.name + "' (" + std::to_string(scenario.surfaces.size()) + " surfaces)");
    }
    if (config.astar.goal_surface_id >= 0) {
        config.astar.goal_location = scenario.surfaces[static_cast<size_t>(config.astar.goal_surface_id)].centroid; // informative
    } else if (config.goal_offset) {
        config.astar.goal_location = scenario.surfaces.back().centroid + *config.goal_offset;
    }
}

std::optional<Point_3> qp_goal(const PlannerConfig& config) {
    if (config.astar.goal_surface_id >= 0) return std::nullopt;
    return config.astar.goal_location;
}

} // namespace nas::config
