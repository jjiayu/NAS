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
    // Node dedup keys (Node::centroid, Node::perimeter). Off (default) = the old
    // code's values: centroid = average of the raw clip vertices (collinear and
    // duplicate points included), perimeter = sum of every edge of the thin
    // prism polyhedron (triangulation diagonals included). Both change with the
    // convex hull's triangulation, i.e. with heap state. On = the area centroid
    // / the geometric perimeter of the patch's convex polygon, which do not
    // depend on how many collinear points the clip produced. Independent
    // switches (ablation). See docs/paper-deltas.md "Clés de dédoublonnage
    // canoniques": neither is on by default because they change which nodes
    // the search merges.
    // The patch IS the convex polygon: patch_vertices, the 3D polyhedron and the
    // keys are all built from the convex hull's vertices, not from the raw clip
    // output (collinear / duplicate points). A patch is convex by construction,
    // so the extra points carry no information; this also makes everything
    // derived from it independent of how many of them the clip produced. The
    // keys keep the old definitions (vertex average, prism edge sum) applied to
    // these vertices, unless canonical_* is also set. Off by default: the old
    // code keeps the raw points, and bit-exact replay needs that.
    bool convex_patch = false;
    bool canonical_centroid = false;
    bool canonical_perimeter = false;
    // Variant: keep the old definition (sum of every prism edge, diagonals
    // included) but build the prism from the convex polygon's own vertices, so
    // collinear clip points no longer change it.
    bool hull_prism_perimeter = false;
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
