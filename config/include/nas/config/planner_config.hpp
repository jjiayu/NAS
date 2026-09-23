#pragma once

// PlannerConfig — JSON-loaded bundle for AstarSearchConfig/FootstepQPConfig
// (see PLAN.md phase 9c), replacing the old code's constants.hpp globals
// (node_similarity_threshold, foot_yaw_*,
// alpha_weight, ...) and its comment-in/comment-out scenario selection.
//
// Deliberately not a new type duplicating those structs' fields: this only
// parses JSON into the AstarSearchConfig/ExpansionParams/FootstepQPConfig
// that already exist. Any field absent from the JSON keeps that struct's
// own default (see planner_config.cpp) — the JSON only needs to specify
// what differs from the default, except start/goal position, which have
// no sensible scenario-independent default and are required.

#include "nas/config/scenario.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/planners/astar_search.hpp"

#include <optional>
#include <string>

namespace nas::config {

struct PlannerConfig {
    AstarSearchConfig astar;
    FootstepQPConfig qp;
    // "astar.goal_offset": the goal is the centroid of the scenario's LAST
    // surface plus this vector (how the old constants.hpp defined it), as an
    // alternative to an absolute "astar.goal_location". Resolved by the caller,
    // who knows the scenario: astar.goal_location is left at its default here.
    std::optional<Vector_3> goal_offset;
};

// Throws std::runtime_error if the file can't be read or parsed, or if
// "astar.start_position" or all of "astar.goal_location", "astar.goal_offset" and "astar.goal_surface" are missing. See
// config/README.md for the full JSON schema.
PlannerConfig load_planner_config(const std::string& json_path);

// Resolves "astar.goal_offset" against the scenario: goal_location = centroid of the scenario's last
// surface + offset. No-op when the config gave an absolute "astar.goal_location". Every caller that
// loads a config for a named scenario must call it before searching (astar_plan, the Python bindings):
// without it a goal_offset config silently searches towards the default goal at the origin.
void resolve_goal(PlannerConfig& config, const Scenario& scenario);

// The goal to give solve_footstep_qp: the goal position, or empty when the goal is a surface
// ("astar.goal_surface": the last footstep is then free on the last patch).
std::optional<Point_3> qp_goal(const PlannerConfig& config);

} // namespace nas::config
