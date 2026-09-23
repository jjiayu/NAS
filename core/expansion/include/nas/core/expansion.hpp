#pragma once

// expand_node() — the single expansion step shared by NAS/Tree and
// CASSR/AstarSearch, extracted from the ~90%-duplicated
// Tree::get_children/AstarSearch::get_children in the old code (see
// PLAN.md, "Pourquoi une réécriture"). Computes the Minkowski sum of the
// parent patch with the appropriate reachability polytope, intersects it
// with every surface, and creates one child Node per non-empty
// intersection (more if rotation is enabled — see ExpansionParams).
//
// Scope decided 2026-09-18 (see PLAN.md "Décisions clés"):
//  - 2 effectors only (StanceFoot::{Left,Right}) — the moving/support
//    effector names queried on ReachabilityModel are hardcoded via
//    effector_name() below, not a generic N-effector registry.
//  - No GaitSequencer — alternation is just other_foot(parent->stance_foot).
//  - Rotation is a real parameter (ExpansionParams::rotation_enabled) since
//    it's explicitly wanted for NAS eventually, even though only CASSR
//    enables it today (NAS's call site leaves it false, matching NAS's
//    current no-rotation behavior exactly).

#include "nas/core/geometry.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/core/surface.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace nas {

// "LF"/"RF" match the naming already used and tested against the real
// talosReachability assets in core/reachability's tests.
std::string effector_name(StanceFoot foot);

struct ExpansionParams {
    bool rotation_enabled = false;
    // Fan-out is 2*yaw_discretization_num + 1 candidate yaws per surface
    // when rotation_enabled; ignored otherwise (single yaw = 0.0, matching
    // NAS's current behavior exactly).
    int yaw_discretization_num = 3;
    double yaw_angle_increment = 10.0 / 180.0 * M_PI;
    bool cycle_detection_enabled = true;
    // Reproduce the old code's 2D clip (which can drop an intersection point,
    // see ClipMode in geometry.hpp). Off by default; on only for replaying the
    // old code's output bit for bit.
    bool legacy_clip = false;
    // Test seam only (tests/golden_all replays the old code's exact P_union
    // through it): when set, replaces the edge list of
    // minkowski_sum(parent patch, reachability polytope). CGAL::convex_hull_3's
    // triangulation depends on heap order, so this is the only way to feed two
    // runs the same hull.
    std::function<EdgeList(const Node& parent)> union_edges_override;
};

// Expands `parent` into its children. `direction` selects which
// ReachabilityModel polytopes to use: Forward for CASSR (search runs
// start->goal), Antecedent for NAS (search runs goal->start). New Node
// instances are owned by `pool` (see NodePool) — never freed individually,
// freed when the pool itself is destroyed.
std::vector<Node*> expand_node(Node* parent,
                                const std::vector<Surface>& surfaces,
                                const ReachabilityModel& reachability,
                                ReachabilityDirection direction,
                                const ExpansionParams& params,
                                NodePool& pool);

} // namespace nas
