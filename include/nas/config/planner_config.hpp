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

#include <array>
#include <optional>
#include <string>
#include <utility>

namespace nas::config {

// A foot_goals slot region that needs the Scenario to resolve (not known at JSON-parse time),
// mirroring the old "astar.goal_offset"'s own deferred-resolution precedent -- exactly one of the
// two is ever set:
//   - offset: JSON "offset" -- resolves to a Point_3 region = the scenario's LAST surface's
//     centroid + this vector.
//   - polygon_on_surface: JSON "surface" + "polygon_2d" together -- resolves to a polytope region,
//     the 2D polygon lifted into world space via that surface's own transform_to_3d.
struct PendingFootGoalRegion {
    std::optional<Vector_3> offset;
    std::optional<std::pair<int, std::vector<Point_2>>> polygon_on_surface;
};

struct PlannerConfig {
    AstarSearchConfig astar;
    FootstepQPConfig qp;
    // Index-aligned with astar.foot_goals: set for whichever slot(s) used "offset" or
    // "surface"+"polygon_2d" (astar.foot_goals[i]->region holds a placeholder until resolved).
    std::array<std::optional<PendingFootGoalRegion>, 2> pending_foot_goal_regions;
};

// Throws std::runtime_error if the file can't be read or parsed, if "astar.start_position" is
// missing, or if "astar.foot_goals" has neither "left" nor "right". See config/README.md for the
// full JSON schema.
PlannerConfig load_planner_config(const std::string& json_path);

// Resolves every foot_goals slot against the scenario: fills in pending_foot_goal_regions (see its
// own doc comment) and bounds-checks any "surface" index. No-op for a slot that gave a direct
// "point"/"surface"/"polytope". Every caller that loads a config for a named scenario must call it
// before searching (astar_plan, the Python bindings): without it a pending slot's region is left at
// its parse-time placeholder.
void resolve_goal(PlannerConfig& config, const Scenario& scenario);

// The goal to give solve_footstep_qp: the point of the single targeted foot_goals slot, or empty
// when the goal is a whole surface, an arbitrary polytope, or both slots are targeted (closing
// stance -- no single point to regularize the QP's last step against). Call after resolve_goal.
std::optional<Point_3> qp_goal(const PlannerConfig& config);

} // namespace nas::config
