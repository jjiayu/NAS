#pragma once

// expand_node() — the single expansion step shared by NAS/Tree and
// CASSR/AstarSearch, extracted from the ~90%-duplicated
// Tree::get_children/AstarSearch::get_children in the old code (see
// PLAN.md, "Pourquoi une réécriture"). Computes the Minkowski sum of the
// parent patch with the appropriate reachability polytope, intersects it
// with every surface, and creates one child Node per non-empty
// intersection (more if rotation is enabled — see ExpansionParams).
//
// A child's patch is the convex polygon of (P_union ∩ surface plane) ∩ surface
// footprint: vertices within 1 nm of their neighbours' line are dropped and the
// list starts at a canonical vertex, so it is identical from run to run (the
// exact convex hull's own start vertex and near-collinear vertices change with
// 1e-16 noise, i.e. with heap state). Node::centroid is the polygon's area
// centroid. See docs/paper-deltas.md, "Profil retenu".
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

#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/core/surface.hpp"

#include <cmath>
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

// Cube-placement action (spec §3.1, docs/cube-implementation-plan.md Etape 3): candidate
// only when parent->cube_state == InHand. Does not move the foot -- children keep
// parent's patch_vertices/stance_foot/foot_yaw/surface_id unchanged, only cube_state
// (-> PlacedActive) and cube (the new CubePlacement) differ. Queries reachability for
// ("Cube", effector_name(parent->stance_foot), Forward), rotated the same way by
// yaw/surface tilt as expand_node's own query. cube_half_extent is used only to pick a
// conservative placement-surface constraint (see the .cpp for why the existing
// foot-eroded surface.vertices_2d is reused rather than a cube-specific erosion --
// deliberate v1 simplification, always at least as conservative as a true cube erosion
// for a cube smaller than half the foot's own margin).
std::vector<Node*> expand_cube_placement(Node* parent,
                                          const std::vector<Surface>& surfaces,
                                          const ReachabilityModel& reachability,
                                          double cube_half_extent,
                                          const ExpansionParams& params,
                                          NodePool& pool);

} // namespace nas
