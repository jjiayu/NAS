#include "nas/config/planner_config.hpp"

#include "nas/core/geometry.hpp"

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

Point_2 point_2d_from_json(const json& j) {
    if (!j.is_array() || j.size() != 2) {
        throw std::runtime_error("load_planner_config: expected a [u, v] array");
    }
    return Point_2(j[0].get<double>(), j[1].get<double>());
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

// One slot of "astar.foot_goals" ("left"/"right"): exactly one of "point"/"surface"/"polytope"/
// "offset" for its region, "polygon_2d" only alongside "surface", plus an optional "yaw_range_deg".
// "offset" and "surface"+"polygon_2d" can't be resolved without the Scenario, so they're parked in
// `pending` (see PendingFootGoalRegion) with a placeholder region, filled in by resolve_goal().
std::optional<AstarSearchConfig::FootGoal> parse_foot_goal_slot(const json& fg, const std::string& slot,
                                                                 std::optional<PendingFootGoalRegion>& pending) {
    if (!fg.contains(slot)) return std::nullopt;
    const json& s = fg.at(slot);
    const std::string ctx = "load_planner_config: \"astar.foot_goals." + slot + "\"";

    const int shape_keys = int(s.contains("point")) + int(s.contains("surface")) + int(s.contains("polytope")) + int(s.contains("offset"));
    if (shape_keys != 1) {
        throw std::runtime_error(ctx + " needs exactly one of \"point\"/\"surface\"/\"polytope\"/\"offset\"");
    }
    if (s.contains("polygon_2d") && !s.contains("surface")) {
        throw std::runtime_error(ctx + ".polygon_2d requires \"surface\"");
    }

    AstarSearchConfig::FootGoal goal;
    if (s.contains("point")) {
        goal.region = point_from_json(s.at("point"));
    } else if (s.contains("polytope")) {
        std::vector<Point_3> verts;
        for (const auto& p : s.at("polytope")) verts.push_back(point_from_json(p));
        if (verts.size() < 3) throw std::runtime_error(ctx + ".polytope needs at least 3 vertices");
        goal.region = std::move(verts);
    } else if (s.contains("offset")) {
        goal.region = Point_3(0, 0, 0); // placeholder -- resolve_goal fills this in
        PendingFootGoalRegion p;
        p.offset = point_from_json(s.at("offset")) - CGAL::ORIGIN;
        pending = std::move(p);
    } else { // "surface"
        int id = s.at("surface").get<int>();
        if (id < 0) throw std::runtime_error(ctx + ".surface must be a surface index >= 0");
        if (s.contains("polygon_2d")) {
            std::vector<Point_2> poly;
            for (const auto& p : s.at("polygon_2d")) poly.push_back(point_2d_from_json(p));
            if (poly.size() < 3) throw std::runtime_error(ctx + ".polygon_2d needs at least 3 vertices");
            goal.region = std::vector<Point_3>{}; // placeholder -- resolve_goal lifts this into 3D
            PendingFootGoalRegion p;
            p.polygon_on_surface = std::make_pair(id, std::move(poly));
            pending = std::move(p);
        } else {
            goal.region = id;
        }
    }

    if (s.contains("yaw_range_deg")) {
        const json& yr = s.at("yaw_range_deg");
        if (!yr.is_array() || yr.size() != 2) throw std::runtime_error(ctx + ".yaw_range_deg expects a [min, max] array");
        goal.yaw_range = std::make_pair(yr[0].get<double>() / 180.0 * M_PI, yr[1].get<double>() / 180.0 * M_PI);
        if (goal.yaw_range->second < goal.yaw_range->first) {
            throw std::runtime_error(ctx + ".yaw_range_deg's max is below its min -- use an unwrapped range "
                                      "(e.g. [170, 190], not [170, -170]) if it crosses +/-180");
        }
    }
    return goal;
}

AstarSearchConfig parse_astar_config(const json& j, std::array<std::optional<PendingFootGoalRegion>, 2>& pending) {
    AstarSearchConfig config; // starts from the struct's own defaults

    if (!j.contains("start_position")) {
        throw std::runtime_error("load_planner_config: \"astar.start_position\" is required");
    }
    config.start_position = point_from_json(j.at("start_position"));

    if (!j.contains("foot_goals")) {
        throw std::runtime_error("load_planner_config: \"astar.foot_goals\" is required (\"left\" and/or \"right\")");
    }
    const json& fg = j.at("foot_goals");
    config.foot_goals[0] = parse_foot_goal_slot(fg, "left", pending[0]);
    config.foot_goals[1] = parse_foot_goal_slot(fg, "right", pending[1]);
    if (!config.foot_goals[0] && !config.foot_goals[1]) {
        throw std::runtime_error("load_planner_config: \"astar.foot_goals\" needs at least \"left\" or \"right\"");
    }

    if (j.contains("start_stance_foot")) config.start_stance_foot = stance_foot_from_string(j.at("start_stance_foot").get<std::string>());
    if (j.contains("start_foot_yaw")) config.start_foot_yaw = j.at("start_foot_yaw").get<double>();
    if (j.contains("distance_metric")) config.distance_metric = distance_metric_from_string(j.at("distance_metric").get<std::string>());
    if (j.contains("heading_weight")) config.heading_weight = j.at("heading_weight").get<double>();
    if (j.contains("yaw_change_weight")) config.yaw_change_weight = j.at("yaw_change_weight").get<double>();
    if (j.contains("heuristic_weight")) config.heuristic_weight = j.at("heuristic_weight").get<double>();
    if (j.contains("step_weight")) config.step_weight = j.at("step_weight").get<double>();
    if (j.contains("goal_yaw_weight")) config.goal_yaw_weight = j.at("goal_yaw_weight").get<double>();
    // a weight of 0 ignores the cost; a negative one would break the search
    for (const auto& [key, w] : {std::pair<const char*, double>{"step_weight", config.step_weight},
                                 {"yaw_change_weight", config.yaw_change_weight},
                                 {"heading_weight", config.heading_weight},
                                 {"heuristic_weight", config.heuristic_weight},
                                 {"goal_yaw_weight", config.goal_yaw_weight}}) {
        if (w < 0.0) throw std::runtime_error(std::string("load_planner_config: \"astar.") + key + "\" must be >= 0 (0 ignores the cost)");
    }

    if (j.contains("node_similarity_threshold")) config.node_similarity_threshold = j.at("node_similarity_threshold").get<double>();
    if (j.contains("patch_index_cell_size")) config.patch_index_cell_size = j.at("patch_index_cell_size").get<double>();
    if (j.contains("inner_margin")) config.inner_margin = j.at("inner_margin").get<double>();
    if (config.inner_margin < 0.0) throw std::runtime_error("load_planner_config: \"astar.inner_margin\" must be >= 0");

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
    config.astar = parse_astar_config(j.at("astar"), config.pending_foot_goal_regions);
    if (j.contains("qp")) {
        config.qp = parse_qp_config(j.at("qp"));
    }
    return config;
}

void resolve_goal(PlannerConfig& config, const Scenario& scenario) {
    for (size_t i = 0; i < 2; ++i) {
        if (!config.astar.foot_goals[i]) continue;
        auto& region = config.astar.foot_goals[i]->region;

        if (std::holds_alternative<int>(region)) {
            int id = std::get<int>(region);
            if (id < 0 || static_cast<size_t>(id) >= scenario.surfaces.size()) {
                throw std::runtime_error("resolve_goal: foot_goals[" + std::to_string(i) + "]'s surface " + std::to_string(id) +
                                         " is not a surface of scenario '" + scenario.name + "' (" +
                                         std::to_string(scenario.surfaces.size()) + " surfaces)");
            }
        }

        if (!config.pending_foot_goal_regions[i]) continue;
        const auto& pending = *config.pending_foot_goal_regions[i];
        if (pending.offset) {
            region = scenario.surfaces.back().centroid + *pending.offset;
        } else {
            const auto& [surface_id, polygon_2d] = *pending.polygon_on_surface;
            if (surface_id < 0 || static_cast<size_t>(surface_id) >= scenario.surfaces.size()) {
                throw std::runtime_error("resolve_goal: foot_goals[" + std::to_string(i) + "]'s polygon_2d surface " +
                                         std::to_string(surface_id) + " is not a surface of scenario '" + scenario.name + "' (" +
                                         std::to_string(scenario.surfaces.size()) + " surfaces)");
            }
            region = transform_2d_points_to_world(polygon_2d, scenario.surfaces[static_cast<size_t>(surface_id)].transform_to_3d);
        }
    }
}

std::optional<Point_3> qp_goal(const PlannerConfig& config) {
    bool has_left = config.astar.foot_goals[0].has_value();
    bool has_right = config.astar.foot_goals[1].has_value();
    if (has_left == has_right) return std::nullopt; // both slots targeted: no single point to regularize against
    const auto& region = config.astar.foot_goals[has_left ? 0 : 1]->region;
    if (!std::holds_alternative<Point_3>(region)) return std::nullopt; // whole surface or polytope: no single point
    return std::get<Point_3>(region);
}

} // namespace nas::config
