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

// round(2*pi / yaw_angle_increment) -- throws std::invalid_argument if the increment doesn't
// divide a full revolution evenly (within float tolerance). Node::foot_yaw_bin is wrapped modulo
// this count; if the increment didn't divide 360 degrees exactly, that wrap would land on the
// wrong physical angle at +/-180 deg (the congruence class wouldn't close up), silently merging
// or splitting nodes that shouldn't be (see docs/patchindex-scalability-note.md). Only meaningful
// when rotation is enabled -- called by expand_node/expand_onto_cube (cheap: once per expansion,
// not per node-similarity comparison) and by AstarSearch's constructor (validates once up front).
int yaw_bins_per_revolution(double yaw_angle_increment);

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
// yaw/surface tilt as expand_node's own query. `cube_support_surfaces` are the scene's surfaces
// already eroded so that a cube whose CENTER lies in their footprint rests entirely on the real
// surface (AstarSearch builds them from the raw scene, eroded by the cube's half-diagonal: valid
// for any cube yaw) -- NOT the foot-eroded footprints expand_node uses. The cube's reachable
// region K_cube is the affordance: it alone says where a cube may go relative to the foot.
std::vector<Node*> expand_cube_placement(Node* parent,
                                          const std::vector<Surface>& cube_support_surfaces,
                                          const ReachabilityModel& reachability,
                                          const ExpansionParams& params,
                                          NodePool& pool);

// Sentinel surface_id for a node standing on top of an active cube (spec §3.3) rather
// than any real, registered Surface -- distinct from -1 ("no surface", the start/root
// node only) so cycle_path_detection never conflates "stood at the start" with "stood on
// a cube" (both would otherwise collide on the same sentinel and could misfire the
// existing "no revisiting a left surface" rule).
constexpr int kOnCubeSurfaceId = -2;

// On-cube step (spec §3.3), candidate only when parent->cube_state == PlacedActive: same
// reachability query/rotation as an ordinary step (expand_node), but landing on the
// cube's own top plane (parent->cube's placement surface, offset by cube_height along its
// normal) instead of any registered Surface, with the extra coupling cut x' - c in
// carre_cube (a square of half side `top_half_extent`, centered on c) that keeps the landing point
// tied to where the cube this specific joint-state vertex is coupled to actually is -- not just
// "some point above the cube's placement area from ANY of its candidate positions".
// `top_half_extent` is the USABLE half side of the cube top: cube_half_extent minus the same
// inner_margin every surface is eroded by (AstarSearch passes it), so a foot stepping onto the cube
// keeps its whole footprint on it, exactly like on any other surface.
// Per docs/cube-implementation-plan.md's v1 scope (single cube, no reuse): the resulting
// child always drops to CubeState::PlacedInactive (cube reset to nullopt) immediately --
// spec §3.3's "second step on the same cube" (keeping it PlacedActive so the OTHER foot
// can also land on it) is out of scope for v1.
std::vector<Node*> expand_onto_cube(Node* parent,
                                     const ReachabilityModel& reachability,
                                     double cube_height,
                                     double top_half_extent,
                                     const ExpansionParams& params,
                                     NodePool& pool);

} // namespace nas
