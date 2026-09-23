#pragma once

// PlannerConfig — JSON-loaded bundle for AstarSearchConfig/FootstepQPConfig
// (see PLAN.md phase 9c), replacing the old code's constants.hpp globals
// (a_star_distance_metric, node_similarity_threshold, foot_yaw_*,
// alpha_weight, ...) and its comment-in/comment-out scenario selection.
//
// Deliberately not a new type duplicating those structs' fields: this only
// parses JSON into the AstarSearchConfig/ExpansionParams/FootstepQPConfig
// that already exist. Any field absent from the JSON keeps that struct's
// own default (see planner_config.cpp) — the JSON only needs to specify
// what differs from the default, except start/goal position, which have
// no sensible scenario-independent default and are required.

#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/planners/astar_search.hpp"

#include <string>

namespace nas::config {

struct PlannerConfig {
    AstarSearchConfig astar;
    FootstepQPConfig qp;
};

// Throws std::runtime_error if the file can't be read or parsed, or if
// "astar.start_position"/"astar.goal_location" are missing. See
// config/README.md for the full JSON schema.
PlannerConfig load_planner_config(const std::string& json_path);

} // namespace nas::config
