#include "nas/planners/astar_search.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/squared_distance_2.h>
#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <stdexcept>
#include <string>

namespace nas {

namespace {

// Wrapped absolute angular difference, in [0, pi]. Used by every optional cost/constraint on yaw
// (yaw_change_weight, heading_weight, goal_yaw_*) — was duplicated inline in each until goal_yaw_*
// made it three copies.
double angdiff(double a, double b) {
    double d = std::fmod(std::abs(a - b), 2.0 * M_PI);
    return d > M_PI ? 2.0 * M_PI - d : d;
}

// Open-set order: f rounded to 1 nm, then creation order (node_id, older first).
// Many nodes have an f that is equal in exact arithmetic (distance 0 to the
// goal patch) and differ only by last-bit noise that changes with heap state;
// comparing raw f let that noise decide which was expanded first, so two runs
// of the same search could expand a different number of nodes and return
// different plans (docs/paper-deltas.md). The old code compared f only.
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        long long qa = std::llround(a->f_score * 1e9), qb = std::llround(b->f_score * 1e9);
        if (qa != qb) return qa > qb;
        return a->node_id > b->node_id;
    }
};

double point_to_polygon_boundary(const Point_2& p, const Polygon_2& poly) {
    double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y()), best = 1e300;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const Point_2 &a = poly.vertex(i), &b = poly.vertex((i + 1) % n);
        double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
        double ex = CGAL::to_double(b.x()) - ax, ey = CGAL::to_double(b.y()) - ay, l2 = ex * ex + ey * ey;
        double t = l2 > 0 ? std::max(0.0, std::min(1.0, ((px - ax) * ex + (py - ay) * ey) / l2)) : 0.0;
        best = std::min(best, std::hypot(px - (ax + t * ex), py - (ay + t * ey)));
    }
    return best;
}

// Both nodes' patches live in their (identical) surface's 2D frame.
double patch_distance(const Node& a, const Node& b) {
    double d = 0.0;
    for (auto v = a.patch_polygon_2d.vertices_begin(); v != a.patch_polygon_2d.vertices_end(); ++v)
        d = std::max(d, point_to_polygon_boundary(*v, b.patch_polygon_2d));
    for (auto v = b.patch_polygon_2d.vertices_begin(); v != b.patch_polygon_2d.vertices_end(); ++v)
        d = std::max(d, point_to_polygon_boundary(*v, a.patch_polygon_2d));
    return d;
}

// Set of nodes with a "find a similar node" query. Nodes are bucketed by
// (surface, stance, yaw bin, cell_size-wide centroid cell) and a query scans the 27
// neighbouring cells, so similar nodes are found across cell boundaries and the
// cost is independent of the set size. Used for both the open and closed sets.
//
// Yaw uses Node::foot_yaw_bin directly (an exact integer already wrapped to its congruence
// class, see core/expansion.hpp's yaw_bins_per_revolution() and Node::foot_yaw_bin's own
// comment) -- NOT re-derived here by dividing/casting the float foot_yaw, which is what
// docs/patchindex-scalability-note.md found to truncate asymmetrically around 0 (int(-0.99)
// == 0 but int(0.99) == 0 too, doubling that one bin's width). foot_yaw_bin is always 0 for
// both parent and child when rotation is disabled, so comparing it needs no extra
// rotation-enabled guard.
class PatchIndex {
public:
    PatchIndex(double tol, double cell_size) : tol_(tol), cell_size_(cell_size) {}

    Node* find(const Node* n) const {
        Cell c = cell_of(*n);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    Cell q = c;
                    q.x += dx; q.y += dy; q.z += dz;
                    auto it = cells_.find(q);
                    if (it == cells_.end()) continue;
                    for (Node* m : it->second)
                        if (similar(*n, *m)) return m;
                }
        return nullptr;
    }
    void insert(Node* n) { cells_[cell_of(*n)].push_back(n); }
    void erase(const Node* n) {
        auto it = cells_.find(cell_of(*n));
        if (it == cells_.end()) return;
        auto& v = it->second;
        v.erase(std::remove(v.begin(), v.end(), n), v.end());
    }

private:
    struct Cell {
        int surface, stance, yaw, x, y, z, cube_state;
        std::uint64_t cubes_picked_up;
        bool operator==(const Cell& o) const {
            return surface == o.surface && stance == o.stance && yaw == o.yaw && x == o.x && y == o.y && z == o.z &&
                   cube_state == o.cube_state && cubes_picked_up == o.cubes_picked_up;
        }
    };
    struct CellHash {
        size_t operator()(const Cell& c) const {
            size_t seed = 0;
            for (int v : {c.surface, c.stance, c.yaw, c.x, c.y, c.z, c.cube_state}) boost::hash_combine(seed, v);
            boost::hash_combine(seed, c.cubes_picked_up);
            return seed;
        }
    };
    Cell cell_of(const Node& n) const {
        return {n.surface_id, static_cast<int>(n.stance_foot), n.foot_yaw_bin,
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.x()) / cell_size_)),
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.y()) / cell_size_)),
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.z()) / cell_size_)),
                static_cast<int>(n.cube_state), n.cubes_picked_up};
    }
    bool similar(const Node& a, const Node& b) const {
        // Two nodes whose x-patch/surface/stance/yaw coincide are NOT the same state if
        // one carries a usable cube and the other doesn't (or a different cube_state
        // entirely, e.g. PlacedActive vs PlacedInactive) -- one can still take an on-cube
        // step later and the other can't, so merging them would silently drop a real
        // option (docs/cube-implementation-plan.md Etape 5, spec §5.3). cube_state alone
        // (not also comparing the cube patch itself) is enough for v1's single-cube scope:
        // there is never more than one PlacedActive cube live at a time to distinguish
        // further within that state. cubes_picked_up is compared too (docs/cube-pickup-
        // spec.md): with several scene cubes available, two nodes can share every field
        // above yet not be interchangeable -- one may still have a pickup option the
        // other has already used.
        if (a.cube_state != b.cube_state) return false;
        if (a.cubes_picked_up != b.cubes_picked_up) return false;
        if (a.surface_id != b.surface_id || a.stance_foot != b.stance_foot) return false;
        if (a.foot_yaw_bin != b.foot_yaw_bin) return false;
        return patch_distance(a, b) < tol_;
    }

    double tol_;
    double cell_size_;
    std::unordered_map<Cell, std::vector<Node*>, CellHash> cells_;
};

// Validates one FootGoal (>=3 vertices if a polytope, a valid index if a surface_id, a well-formed
// yaw_range requiring rotation) and returns its hull if polytope-shaped (nullopt for a point,
// surface_id or unset slot) -- shared by foot_goals and scene_cubes[*].pickup_affordance, identical
// shape/contract. `context` prefixes every thrown message so the two stay distinguishable.
std::optional<Polyhedron> validate_and_hull_foot_goal(const std::string& context,
                                                       const AstarSearchConfig::FootGoal& g,
                                                       bool rotation_enabled,
                                                       const std::vector<std::optional<Surface>>& eroded_by_id) {
    std::optional<Polyhedron> hull;
    if (std::holds_alternative<std::vector<Point_3>>(g.region)) {
        const auto& verts = std::get<std::vector<Point_3>>(g.region);
        if (verts.size() < 3) {
            throw std::invalid_argument(context + "'s polytope needs at least 3 vertices");
        }
        Polyhedron h;
        CGAL::convex_hull_3(verts.begin(), verts.end(), h);
        hull = std::move(h);
    } else if (std::holds_alternative<int>(g.region)) {
        int id = std::get<int>(g.region);
        if (id < 0 || static_cast<size_t>(id) >= eroded_by_id.size()) {
            throw std::invalid_argument(context + "'s surface index " + std::to_string(id) + " is not a surface of the scenario");
        }
        if (!eroded_by_id[static_cast<size_t>(id)]) {
            throw std::invalid_argument(context + "'s surface " + std::to_string(id) +
                                         " is thinner than 2 x inner_margin: no foot can stand on it");
        }
    }
    if (g.yaw_range) {
        if (g.yaw_range->second < g.yaw_range->first) {
            throw std::invalid_argument(context + "'s yaw_range max is below its min -- use an unwrapped range "
                                         "(e.g. {170deg, 190deg}, not {170deg, -170deg}) if it crosses +/-pi");
        }
        if (!rotation_enabled) {
            throw std::invalid_argument(context + " has a yaw_range but expansion_params.rotation_enabled is "
                                         "false -- every node's foot_yaw stays at 0 then");
        }
    }
    return hull;
}

} // namespace

AstarSearch::AstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, AstarSearchConfig config)
    : raw_surfaces_(std::move(surfaces)), reachability_(std::move(reachability)), config_(std::move(config)) {

    // Everything below (foot_goals' surface indices, Node::surface_id, cycle detection) treats a
    // surface_id as an index into the scene's surfaces.
    for (size_t i = 0; i < raw_surfaces_.size(); ++i) {
        if (raw_surfaces_[i].surface_id != static_cast<int>(i)) {
            throw std::invalid_argument("AstarSearch: surfaces[" + std::to_string(i) + "].surface_id is " +
                                         std::to_string(raw_surfaces_[i].surface_id) + ", expected its index");
        }
    }
    // The raw scene, eroded once: eroded_by_id_ keeps a collapsed surface's slot (nullopt) so ids
    // never shift, surfaces_ is what expand_node and the start-node lookup iterate over.
    eroded_by_id_ = erode_by_id(raw_surfaces_, config_.inner_margin);
    for (const std::optional<Surface>& s : eroded_by_id_) {
        if (s) surfaces_.push_back(*s);
    }

    // Validated up front (not just relied upon inside expand_node's hot loop) so a
    // misconfigured increment fails at construction, not partway through a search:
    // PatchIndex dedups nodes by Node::foot_yaw_bin, an integer wrapped modulo
    // yaw_bins_per_revolution(), and that wrap only lands on the same physical angle at
    // +/-180 deg if the increment divides 360 deg exactly (docs/patchindex-scalability-note.md).
    if (config_.expansion_params.rotation_enabled) {
        yaw_bins_per_revolution(config_.expansion_params.yaw_angle_increment);
    }
    const bool has_left_goal = config_.foot_goals[0].has_value();
    const bool has_right_goal = config_.foot_goals[1].has_value();
    if (!has_left_goal && !has_right_goal) {
        throw std::invalid_argument("AstarSearch: foot_goals has neither slot set — the search needs a goal");
    }
    if (has_left_goal && has_right_goal && config_.cube_half_extent > 0.0) {
        // See astar_search.hpp's own comment on foot_goals: expand_cube_placement gives its child the
        // same stance foot as its parent, breaking the alternation invariant the closing-stance mode
        // relies on for its node+parent termination test — rejected outright rather than silently
        // computed wrong. A single slot never reads the parent's own foot, so only "both slots" is
        // rejected here.
        throw std::invalid_argument("AstarSearch: foot_goals has both slots set together with cube_half_extent > 0 — "
                                     "the cube extension breaks the foot-alternation invariant the closing-stance "
                                     "mode relies on");
    }
    if (config_.goal_yaw_weight > 0.0) {
        if (has_left_goal == has_right_goal) {
            throw std::invalid_argument("AstarSearch: goal_yaw_weight is only meaningful with exactly one "
                                         "foot_goals slot filled");
        }
        const AstarSearchConfig::FootGoal& targeted = has_left_goal ? *config_.foot_goals[0] : *config_.foot_goals[1];
        if (!targeted.yaw_range) {
            throw std::invalid_argument("AstarSearch: goal_yaw_weight is set but the targeted foot_goals slot has "
                                         "no yaw_range — it would be silently ignored");
        }
    }
    for (size_t i = 0; i < 2; ++i) {
        if (!config_.foot_goals[i]) continue;
        target_polyhedra_[i] = validate_and_hull_foot_goal(
            "AstarSearch: foot_goals[" + std::to_string(i) + "]", *config_.foot_goals[i],
            config_.expansion_params.rotation_enabled, eroded_by_id_);
    }
    if (config_.cube_half_extent > 0.0 &&
        (!reachability_.has("Cube", "LF", ReachabilityDirection::Forward) || !reachability_.has("Cube", "RF", ReachabilityDirection::Forward))) {
        throw std::invalid_argument("AstarSearch: cube_half_extent > 0 but the reachability model has no \"Cube\" "
                                     "entry for LF and/or RF support — load Cube_constraints_in_{LF,RF}.obj alongside "
                                     "the usual foot-in-foot entries to enable the cube extension");
    }
    if (config_.cube_half_extent > 0.0) {
        if (config_.cube_half_extent <= config_.inner_margin) {
            throw std::invalid_argument("AstarSearch: cube_half_extent (" + std::to_string(config_.cube_half_extent) +
                                         ") must exceed inner_margin (" + std::to_string(config_.inner_margin) +
                                         "): a foot stepping onto the cube needs the same margin as on any surface, "
                                         "the cube top would be empty");
        }
        // Where the cube's center may go so that the whole cube rests on the real surface, any yaw.
        // Quiet on purpose (unlike erode_by_id): most surfaces (stair treads) are legitimately too
        // small to hold a cube.
        const double half_diagonal = config_.cube_half_extent * std::sqrt(2.0);
        for (const Surface& s : raw_surfaces_) {
            if (std::optional<Surface> support = s.inner_margin(half_diagonal)) cube_support_.push_back(std::move(*support));
        }
    }
    if (!config_.scene_cubes.empty() && config_.cube_half_extent <= 0.0) {
        throw std::invalid_argument("AstarSearch: scene_cubes is set but cube_half_extent <= 0 -- a picked-up cube "
                                     "could never be placed back down or stepped on");
    }
    if (config_.scene_cubes.size() > 64) {
        throw std::invalid_argument("AstarSearch: scene_cubes has more than 64 entries -- Node::cubes_picked_up is "
                                     "a 64-bit mask (chosen so it's cheap to copy/compare/hash in PatchIndex, "
                                     "unlike std::vector<bool>), well beyond any scene this planner has ever seen");
    }
    scene_cube_polyhedra_.resize(config_.scene_cubes.size());
    for (size_t i = 0; i < config_.scene_cubes.size(); ++i) {
        const auto& aff = config_.scene_cubes[i].pickup_affordance;
        if (!aff[0] && !aff[1]) {
            throw std::invalid_argument("AstarSearch: scene_cubes[" + std::to_string(i) +
                                         "].pickup_affordance has neither foot slot set -- this cube could never be "
                                         "picked up");
        }
        for (size_t f = 0; f < 2; ++f) {
            if (!aff[f]) continue;
            scene_cube_polyhedra_[i][f] = validate_and_hull_foot_goal(
                "AstarSearch: scene_cubes[" + std::to_string(i) + "].pickup_affordance[" + std::to_string(f) + "]",
                *aff[f], config_.expansion_params.rotation_enabled, eroded_by_id_);
        }
    }
    start_node_ = pool_.create();
    start_node_->patch_vertices = {config_.start_position};
    start_node_->stance_foot = config_.start_stance_foot;
    start_node_->centroid = config_.start_position;
    start_node_->foot_yaw = config_.expansion_params.rotation_enabled ? config_.start_foot_yaw : 0.0;
    // The only place a float is ever converted to a yaw bin by division: start_foot_yaw is an
    // arbitrary config-provided angle, not itself produced by an integer number of expansion
    // steps, so there is no exact integer to inherit here (unlike every descendant node, whose
    // foot_yaw_bin is parent's own + the same integer offset used for its foot_yaw -- see
    // core/expansion.cpp's candidate_yaws()). std::llround (nearest), not truncation, and
    // wrapped into [0, bins) -- both deviations from the old cell_of()'s int(x / increment) that
    // docs/patchindex-scalability-note.md flagged as the source of the doubled-width bin at 0.
    if (config_.expansion_params.rotation_enabled) {
        const int bins = yaw_bins_per_revolution(config_.expansion_params.yaw_angle_increment);
        long long idx = std::llround(start_node_->foot_yaw / config_.expansion_params.yaw_angle_increment);
        start_node_->foot_yaw_bin = static_cast<int>(((idx % bins) + bins) % bins);
    }
    start_node_->depth = 0;
    // scene_cubes non-empty: start empty-handed (must pick one up) rather than carrying
    // one from the start -- with scene_cubes empty, this reduces exactly to the original
    // expression, unchanged behavior for every caller that predates this extension.
    start_node_->cube_state =
        (config_.cube_half_extent > 0.0 && config_.scene_cubes.empty()) ? CubeState::InHand : CubeState::None;
    start_node_->cubes_picked_up = 0;
    // The start foot stands on some surface: give the start node that surface's frame (its normal
    // orients the reachability polytope of the first step). surface_id stays -1 (the start is not a
    // visited surface for the cycle detection). No surface within reach: flat.
    {
        const Surface* best = nullptr;
        double best_dist = 0.05; // metres from the surface plane
        for (const Surface& s : surfaces_) {
            double a = CGAL::to_double(s.plane.a()), b = CGAL::to_double(s.plane.b()), c = CGAL::to_double(s.plane.c()), d = CGAL::to_double(s.plane.d());
            double norm = std::sqrt(a * a + b * b + c * c);
            double dist = std::abs(a * CGAL::to_double(config_.start_position.x()) + b * CGAL::to_double(config_.start_position.y()) +
                                   c * CGAL::to_double(config_.start_position.z()) + d) / norm;
            if (dist >= best_dist) continue;
            std::vector<Point_2> local = transform_3d_points_to_surface_plane({config_.start_position}, s.transform_to_surface);
            // inside the (foot-shrunk) footprint, or within 0.2 m of it: the start foot may stand near the edge
            bool near_footprint = s.polygon_2d.bounded_side(local[0]) != CGAL::ON_UNBOUNDED_SIDE;
            if (!near_footprint) {
                for (size_t k = 0; k < s.polygon_2d.size(); ++k) {
                    Segment_2 e(s.polygon_2d.vertex(k), s.polygon_2d.vertex((k + 1) % s.polygon_2d.size()));
                    if (std::sqrt(CGAL::to_double(CGAL::squared_distance(local[0], e))) < 0.2) { near_footprint = true; break; }
                }
            }
            if (near_footprint) { best = &s; best_dist = dist; }
        }
        if (best) {
            start_node_->transformation_to_3d = best->transform_to_3d;
            start_node_->transformation_to_2d = best->transform_to_surface;
        }
    }
    start_node_->g_score = 0.0;
    if (has_left_goal && has_right_goal) {
        // Both feet targeted: sum both remaining distances, exactly like process_child does for
        // every later node (see search()) — the start node is not a special case for this mode.
        start_node_->h_score = weight_if_epa(distance_to_goal(*start_node_, StanceFoot::Left) +
                                              distance_to_goal(*start_node_, StanceFoot::Right));
    } else {
        // A single-point patch has no area for EPA (it needs >=3 points): foot_goal_distance's own
        // point/surface_id branches already fall back to a plain Euclidean distance in that case
        // (see below), so the start node needs no special case here.
        StanceFoot targeted = has_left_goal ? StanceFoot::Left : StanceFoot::Right;
        start_node_->h_score = weight_if_epa(distance_to_goal(*start_node_, targeted));
    }
    start_node_->f_score = start_node_->g_score + start_node_->h_score;
    start_node_->parent = nullptr;
}

namespace {
// A FootGoal's representative point, for goal_point()'s dual-target approximation below: the point
// itself, a surface's centroid, or a polytope's centroid.
Point_3 foot_goal_representative_point(const AstarSearchConfig::FootGoal& g,
                                       const std::vector<std::optional<Surface>>& eroded_by_id) {
    if (std::holds_alternative<Point_3>(g.region)) return std::get<Point_3>(g.region);
    if (std::holds_alternative<int>(g.region)) return eroded_by_id[static_cast<size_t>(std::get<int>(g.region))]->centroid;
    return get_centroid(std::get<std::vector<Point_3>>(g.region));
}
} // namespace

Point_3 AstarSearch::goal_point() const {
    // At least one slot is always set (validated at construction).
    if (config_.foot_goals[0] && config_.foot_goals[1]) {
        // Only consumed by heading_weight's edge cost when foot_goals is also in use — an
        // approximation (not tuned specifically for two targets), documented at heading_weight.
        return CGAL::midpoint(foot_goal_representative_point(*config_.foot_goals[0], eroded_by_id_),
                               foot_goal_representative_point(*config_.foot_goals[1], eroded_by_id_));
    }
    return foot_goal_representative_point(config_.foot_goals[0] ? *config_.foot_goals[0] : *config_.foot_goals[1], eroded_by_id_);
}

double AstarSearch::weight_if_epa(double raw_distance) const {
    return config_.distance_metric == DistanceMetric::Epa ? config_.heuristic_weight * raw_distance : raw_distance;
}

double foot_goal_distance(const Node& node, const AstarSearchConfig::FootGoal& goal, DistanceMetric metric,
                           const std::vector<std::optional<Surface>>& eroded_by_id) {
    if (std::holds_alternative<Point_3>(goal.region)) {
        const Point_3& target = std::get<Point_3>(goal.region);
        if (metric == DistanceMetric::Euclidean || node.patch_vertices.size() < 3) {
            // A single-point patch (only ever the start node) has no area for EPA.
            return compute_euclidean_distance(node.centroid, target);
        }
        return calculate_epa_distance_point_to_patch(node.patch_vertices, target);
    }
    if (std::holds_alternative<int>(goal.region)) {
        // Never nullopt: a goal on a collapsed surface is rejected at construction.
        const Surface& target = *eroded_by_id[static_cast<size_t>(std::get<int>(goal.region))];
        if (metric == DistanceMetric::Euclidean || node.patch_vertices.size() < 3) {
            return compute_euclidean_distance(node.centroid, target.centroid);
        }
        return calculate_epa_distance_patch_to_patch(node.patch_vertices, target.vertices_3d);
    }
    // Polytope target: not necessarily flat or given in a fan-triangulable vertex order (unlike
    // surfaces_[*].vertices_3d, which the surface_id branch above relies on) — the exact shape
    // test lives in foot_goal_satisfied() (a real plane slice against the cached hull); the
    // heuristic only needs a reasonable estimate, and this codebase's own EPA helper already falls
    // back to centroid distance whenever it can't compute a true patch distance
    // (calculate_epa_distance_patch_to_patch's catch block, geometry.cpp) — using that same
    // approximation proactively here is the same judgment call, not a new one.
    Point_3 target_centroid = get_centroid(std::get<std::vector<Point_3>>(goal.region));
    return compute_euclidean_distance(node.centroid, target_centroid);
}

namespace {

// The node's own patch, clipped against cached_hull's polytope (a FootGoal region's precomputed
// convex hull), in the node's own surface frame. nullopt when the node has no plane to clip
// against (a bare-point node) or the intersection is empty/degenerate (<=2 vertices).
//
// Factored out of foot_goal_satisfied's own polytope branch, which used to compute exactly this
// and throw it away, keeping only a boolean. That was fine for a TERMINAL goal (nothing happens
// after it) and for foot_goals' own use here, but wrong for expand_cube_pickup's single-slot
// case (below): the search still has to keep exploring FROM the pickup node afterward, and a
// child that inherits the parent's whole, un-narrowed patch lets every later node's own patch
// (built via Minkowski sum from this one) overstate what's really reachable - the true, tight
// requirement lives only in this discarded intersection. See docs/cube-pickup-spec.md's own note
// on this, added alongside this fix, for the concrete case that surfaced it.
struct ClippedPatch {
    std::vector<Point_3> vertices_3d;
    Polygon_2 polygon_2d;
    Point_3 centroid;
};

std::optional<ClippedPatch> clip_patch_to_hull(const Node& node, const Polyhedron& cached_hull) {
    if (node.patch_vertices.size() < 3) return std::nullopt;
    Plane_3 node_plane(node.patch_vertices[0], node.up_normal());

    // compute_polytope_plane_intersection finds edges that CROSS the plane (some endpoint above,
    // some below) — a target that is flat and exactly coincident with the node's own plane (e.g. a
    // copy of some surface's own vertices, sitting on the same floor the foot is on: the common
    // case, not a rare one) has every vertex exactly ON the plane, no crossing edge at all, and the
    // slicer returns nothing even though this is precisely the well-defined "target flush with this
    // surface" case — found empirically (a hand test on a flat Flat-scenario target always failed
    // before this check was added). Detected by testing the hull's own vertices against the plane
    // equation directly, not relied on as an exact predicate (a genuinely 3D target that merely
    // grazes the plane along one face should still take this branch, not just a bit-exact match).
    double pa = CGAL::to_double(node_plane.a()), pb = CGAL::to_double(node_plane.b());
    double pc = CGAL::to_double(node_plane.c()), pd = CGAL::to_double(node_plane.d());
    double pnorm = std::sqrt(pa * pa + pb * pb + pc * pc);
    std::vector<Point_3> hull_verts;
    bool coplanar = true;
    for (auto v = cached_hull.vertices_begin(); v != cached_hull.vertices_end(); ++v) {
        hull_verts.push_back(v->point());
        double sd = std::abs(pa * CGAL::to_double(v->point().x()) + pb * CGAL::to_double(v->point().y()) +
                              pc * CGAL::to_double(v->point().z()) + pd) / pnorm;
        if (sd > 1e-6) coplanar = false;
    }
    std::vector<Point_3> sliced_3d = coplanar ? hull_verts : compute_polytope_plane_intersection(node_plane, cached_hull);
    if (sliced_3d.size() <= 2) return std::nullopt;

    std::vector<Point_2> sliced_2d = transform_3d_points_to_surface_plane(sliced_3d, node.transformation_to_2d);
    Polygon_2 sliced_hull;
    CGAL::convex_hull_2(sliced_2d.begin(), sliced_2d.end(), std::back_inserter(sliced_hull));
    std::vector<Point_2> sliced_hull_pts(sliced_hull.vertices_begin(), sliced_hull.vertices_end());
    std::vector<Point_2> node_patch_pts(node.patch_polygon_2d.vertices_begin(), node.patch_polygon_2d.vertices_end());
    std::vector<Point_2> clipped_2d = compute_2d_polygon_intersection(sliced_hull_pts, node_patch_pts);
    if (clipped_2d.size() <= 2) return std::nullopt;

    Polygon_2 clipped_polygon(clipped_2d.begin(), clipped_2d.end());
    std::vector<Point_3> clipped_3d = transform_2d_points_to_world(clipped_2d, node.transformation_to_3d);
    Point_3 centroid = area_centroid(clipped_2d, node.transformation_to_3d, clipped_3d);
    return ClippedPatch{clipped_3d, clipped_polygon, centroid};
}

} // namespace

bool foot_goal_satisfied(const Node& node, const AstarSearchConfig::FootGoal& goal, const Polyhedron* cached_hull) {
    bool shape_ok;
    if (std::holds_alternative<Point_3>(goal.region)) {
        shape_ok = node.check_if_node_contains_point(std::get<Point_3>(goal.region));
    } else if (std::holds_alternative<int>(goal.region)) {
        // Index equality, not geometric containment: "standing anywhere on this surface" -- the
        // same test goal_surface_id used before foot_goals absorbed it. Needs no cached_hull.
        shape_ok = node.surface_id == std::get<int>(goal.region);
    } else {
        const auto& polytope_verts = std::get<std::vector<Point_3>>(goal.region);
        if (node.patch_vertices.size() < 3) {
            // Degenerate bare-point node (only ever the start node, reached as current_node->parent
            // in the closing-stance mode's very first possible termination): no plane/patch to clip
            // against, so fall back to "is this point on/in the target" via the same EPA-touches-zero
            // test the point-to-patch heuristic itself uses (1e-6 sphere radius baked into it).
            shape_ok = calculate_epa_distance_point_to_patch(polytope_verts, node.centroid) <= 1e-6;
        } else {
            // Same pipeline expand_node already runs against real scene surfaces (slice the
            // candidate region by the node's own contact plane, clip against the node's patch) —
            // reused here against an arbitrary target polytope instead of a registered Surface.
            shape_ok = clip_patch_to_hull(node, *cached_hull).has_value();
        }
    }
    if (!shape_ok || !goal.yaw_range) return shape_ok;

    double center = (goal.yaw_range->first + goal.yaw_range->second) / 2.0;
    double half_width = (goal.yaw_range->second - goal.yaw_range->first) / 2.0;
    return angdiff(node.foot_yaw, center) <= half_width;
}

double AstarSearch::distance_to_goal(const Node& node, StanceFoot which) const {
    return foot_goal_distance(node, *config_.foot_goals[static_cast<size_t>(which)], config_.distance_metric, eroded_by_id_);
}

bool AstarSearch::goal_satisfied(const Node& node, StanceFoot which) const {
    size_t idx = static_cast<size_t>(which);
    return foot_goal_satisfied(node, *config_.foot_goals[idx], target_polyhedra_[idx] ? &*target_polyhedra_[idx] : nullptr);
}

std::vector<Node*> expand_cube_pickup(Node* parent,
                                       const std::vector<AstarSearchConfig::SceneCube>& scene_cubes,
                                       const std::vector<std::array<std::optional<Polyhedron>, 2>>& scene_cube_polyhedra,
                                       NodePool& pool) {
    std::vector<Node*> children;
    if (parent->cube_state != CubeState::None && parent->cube_state != CubeState::PlacedInactive) return children;

    for (size_t i = 0; i < scene_cubes.size(); ++i) {
        if (parent->cubes_picked_up & (std::uint64_t(1) << i)) continue; // already taken on this path

        const auto& aff = scene_cubes[i].pickup_affordance;
        bool ready;
        // Mode 1 only: the child's patch gets narrowed to parent's patch (cap) affordance region
        // (see clip_patch_to_hull's own doc comment for why) -- nullopt when the affordance isn't
        // polytope-shaped (point/surface_id: no narrowing needed there, see below) or mode 2 is
        // used (retroactively narrowing parent->parent, a node shared with sibling branches that
        // don't take this pickup, isn't done here -- see the comment at its use below).
        std::optional<ClippedPatch> narrowed;
        if (aff[0].has_value() != aff[1].has_value()) {
            // Mode 1: only one foot's slot is filled -- this foot alone must satisfy it, same
            // convention as foot_goals mode 1 (the other foot is irrelevant to this test).
            StanceFoot which = aff[0] ? StanceFoot::Left : StanceFoot::Right;
            size_t w = static_cast<size_t>(which);
            if (parent->stance_foot != which) continue;
            const Polyhedron* hull = scene_cube_polyhedra[i][w] ? &*scene_cube_polyhedra[i][w] : nullptr;
            ready = foot_goal_satisfied(*parent, *aff[w], hull);
            if (ready && hull && std::holds_alternative<std::vector<Point_3>>(aff[w]->region)) {
                // foot_goal_satisfied already confirmed this clip is non-empty (same computation,
                // recomputed here since it discards its own result) -- a point-shaped affordance
                // needs no narrowing: the QP already pins that node to it exactly (an equality, not
                // a region), same as any other point-shaped foot_goals slot, so nothing downstream
                // needs a tighter patch than the parent's own to make that guarantee hold.
                narrowed = clip_patch_to_hull(*parent, *hull);
            }
        } else if (aff[0] && aff[1]) {
            // Mode 2: both slots filled -- symmetric stance, tested on (parent, parent->parent)
            // together, same pairing as foot_goals' own closing-stance termination test. No parent
            // yet (the very start node): only one foot has ever been placed, no pair to test.
            //
            // Not narrowed (both here and via parent->parent, unlike mode 1 above): parent->parent
            // is an already-expanded node other branches of the search may still be exploring from
            // (siblings that don't pick up this cube need its own, un-narrowed patch) -- narrowing
            // it in place would corrupt those; narrowing only this action's own child would still
            // leave the OTHER foot's true position under-constrained downstream. Properly fixing
            // this needs its own zero-displacement pseudo-node for the trailing foot too, not
            // attempted here. Left as a known gap -- see this function's own doc comment.
            if (parent->parent == nullptr) continue;
            size_t pw = static_cast<size_t>(parent->stance_foot), ow = static_cast<size_t>(other_foot(parent->stance_foot));
            const Polyhedron* phull = scene_cube_polyhedra[i][pw] ? &*scene_cube_polyhedra[i][pw] : nullptr;
            const Polyhedron* ohull = scene_cube_polyhedra[i][ow] ? &*scene_cube_polyhedra[i][ow] : nullptr;
            ready = foot_goal_satisfied(*parent, *aff[pw], phull) && foot_goal_satisfied(*parent->parent, *aff[ow], ohull);
        } else {
            continue; // neither slot set -- rejected at AstarSearch construction, defensive here
        }
        if (!ready) continue;

        // Zero-displacement pseudo-action, same shape as expand_cube_placement's own children:
        // same foot/position/yaw/surface as parent, only the cube-related state differs. The patch
        // is parent's own UNLESS mode 1 narrowed it above -- deliberately not written back onto
        // parent itself, which keeps its own full patch for its other children (ordinary steps that
        // don't pick up this cube, or a different scene cube's own pickup).
        Node* child = pool.create();
        child->parent_ptrs.push_back(parent);
        child->patch_vertices = narrowed ? narrowed->vertices_3d : parent->patch_vertices;
        child->patch_polygon_2d = narrowed ? narrowed->polygon_2d : parent->patch_polygon_2d;
        child->transformation_to_2d = parent->transformation_to_2d;
        child->transformation_to_3d = parent->transformation_to_3d;
        child->stance_foot = parent->stance_foot;
        child->surface_id = parent->surface_id;
        child->depth = parent->depth + 1;
        child->centroid = narrowed ? narrowed->centroid : parent->centroid;
        child->foot_yaw = parent->foot_yaw;
        child->foot_yaw_bin = parent->foot_yaw_bin;
        child->pred_surface_ids = parent->pred_surface_ids; // no footstep was taken

        child->cube_state = CubeState::InHand;
        child->cube = std::nullopt;
        // parent->cubes_picked_up itself is never mutated (each sibling child below gets its own
        // independent value with just its own bit added).
        child->cubes_picked_up = parent->cubes_picked_up | (std::uint64_t(1) << i);

        children.push_back(child);
    }
    return children;
}

void AstarSearch::search() {
    using OpenSet = boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>>;
    OpenSet open_set;
    // open_index answers "is an equivalent node already open?"; the heap handle
    // of an open node is looked up by the node itself.
    PatchIndex open_index(config_.node_similarity_threshold, config_.patch_index_cell_size);
    PatchIndex closed_index(config_.node_similarity_threshold, config_.patch_index_cell_size);
    std::unordered_map<Node*, OpenSet::handle_type> node_handles;

    node_handles[start_node_] = open_set.push(start_node_);
    open_index.insert(start_node_);

    while (!open_set.empty()) {
        if (config_.max_expansions > 0 && expansion_count_ >= config_.max_expansions) return;
        Node* current_node = open_set.top();
        open_set.pop();
        ++expansion_count_;
        node_handles.erase(current_node);
        open_index.erase(current_node);
        if (config_.on_expand) config_.on_expand(expansion_count_, *current_node);

        const bool has_left_goal = config_.foot_goals[0].has_value();
        const bool has_right_goal = config_.foot_goals[1].has_value();
        bool at_closing_stance = false;
        if (has_left_goal != has_right_goal) {
            // One foot_goals slot filled: this foot must satisfy its own slot, the other foot stays free.
            StanceFoot targeted = has_left_goal ? StanceFoot::Left : StanceFoot::Right;
            at_closing_stance = current_node->stance_foot == targeted && goal_satisfied(*current_node, targeted);
        } else if (current_node->parent != nullptr) {
            // Both slots filled (the only way has_left_goal == has_right_goal can be true here --
            // "neither" is rejected at construction): closing stance -- the last two consecutive
            // footsteps (node + its immediate predecessor, always the other foot on this non-cube
            // expansion path) must each satisfy their own slot at once. current_node->parent ==
            // nullptr (the start node) can never close here: only one foot is placed yet, there is
            // no pair to test.
            at_closing_stance = goal_satisfied(*current_node, current_node->stance_foot) &&
                                 goal_satisfied(*current_node->parent, current_node->parent->stance_foot);
        }
        if (at_closing_stance) {
            Node* current = current_node;
            while (current != nullptr) {
                result_path_.push_back(current);
                current = current->parent;
            }
            std::reverse(result_path_.begin(), result_path_.end());
            return;
        }

        closed_index.insert(current_node);

        // Mode 2 only: the trailing foot's own remaining distance doesn't change across this node's
        // children (they all share the same current_node), so it is computed once here instead of
        // once per child. child->parent is not usable for this — it is only assigned after this
        // node's children are scored below (see process_child), always nullptr before that.
        const double current_own_term =
            (has_left_goal && has_right_goal) ? distance_to_goal(*current_node, current_node->stance_foot) : 0.0;

        // Shared by every action source below (expand_node, and -- when the cube
        // extension is enabled -- expand_cube_placement/expand_onto_cube): same
        // edge-cost formula (only the base action cost differs), same open/closed-set
        // bookkeeping. Extracted so the cube actions don't duplicate this ~30-line block
        // twice more (docs/cube-implementation-plan.md's search-integration step).
        auto process_child = [&](Node* child, double base_cost) {
            if (closed_index.find(child) != nullptr) {
                if (config_.on_child) config_.on_child(expansion_count_, *child, ChildAction::SkippedClosed);
                return;
            }

            double edge_cost = base_cost;
            if (config_.yaw_change_weight > 0.0 && config_.expansion_params.rotation_enabled) {
                edge_cost += config_.yaw_change_weight * angdiff(child->foot_yaw, current_node->foot_yaw);
            }
            if (config_.heading_weight > 0.0 && config_.expansion_params.rotation_enabled) {
                // the rough direction of travel: from the parent's centroid to the goal
                const Point_3 goal = goal_point();
                double gx = CGAL::to_double(goal.x() - current_node->centroid.x()), gy = CGAL::to_double(goal.y() - current_node->centroid.y());
                if (std::hypot(gx, gy) > 1e-6) {
                    edge_cost += config_.heading_weight * angdiff(child->foot_yaw, std::atan2(gy, gx));
                }
            }
            if (config_.goal_yaw_weight > 0.0) {
                // Validated at construction: exactly one slot filled, and it has a yaw_range -- its
                // center is the target, its half-width the dead zone (same convention as
                // foot_goal_satisfied's own yaw_range check above).
                const AstarSearchConfig::FootGoal& targeted = has_left_goal ? *config_.foot_goals[0] : *config_.foot_goals[1];
                double center = (targeted.yaw_range->first + targeted.yaw_range->second) / 2.0;
                double half_width = (targeted.yaw_range->second - targeted.yaw_range->first) / 2.0;
                double d = angdiff(child->foot_yaw, center);
                edge_cost += config_.goal_yaw_weight * std::max(0.0, d - half_width);
            }
            double tentative_g_score = current_node->g_score + edge_cost;
            double tentative_h_score;
            if (has_left_goal != has_right_goal) {
                tentative_h_score = weight_if_epa(distance_to_goal(*child, has_left_goal ? StanceFoot::Left : StanceFoot::Right));
            } else {
                // Both slots filled ("neither" rejected at construction): sum both feet's remaining
                // distance (this child's own + the trailing foot's, cached above) rather than just the
                // foot being placed right now: the search must be pulled toward closing the pair, not
                // just toward whichever foot happens to move next — a per-foot-only estimate could
                // converge one foot onto its target while never pulling the other, since nothing else
                // pushes the trailing foot toward its own slot.
                tentative_h_score = weight_if_epa(distance_to_goal(*child, child->stance_foot) + current_own_term);
            }
            double tentative_f_score = tentative_g_score + tentative_h_score;

            Node* existing = open_index.find(child);
            if (existing == nullptr) {
                child->g_score = tentative_g_score;
                child->h_score = tentative_h_score;
                child->f_score = tentative_f_score;
                child->parent = current_node;
                node_handles[child] = open_set.push(child);
                open_index.insert(child);
                if (config_.on_child) config_.on_child(expansion_count_, *child, ChildAction::Pushed);
            } else if (tentative_g_score < existing->g_score) {
                Node* existing_node = existing;
                existing_node->g_score = tentative_g_score;
                existing_node->h_score = tentative_h_score;
                existing_node->f_score = tentative_f_score;
                existing_node->parent = current_node;
                open_set.increase(node_handles.at(existing_node));
                if (config_.on_child) config_.on_child(expansion_count_, *child, ChildAction::ImprovedExisting);
                // `child` itself is simply left unreferenced in the pool —
                // unlike the old code's `delete child`, NodePool has no
                // per-element free. Not a leak (freed with the pool), just
                // a few extra unused Nodes for the search's lifetime; see
                // docs/paper-deltas.md.
            } else if (config_.on_child) {
                config_.on_child(expansion_count_, *child, ChildAction::MergedWorse);
            }
        };

        for (Node* child : expand_node(current_node, surfaces_, reachability_, ReachabilityDirection::Forward, config_.expansion_params, pool_)) {
            process_child(child, config_.step_weight);
        }

        if (config_.cube_half_extent > 0.0) {
            switch (current_node->cube_state) {
                case CubeState::None:
                case CubeState::PlacedInactive:
                    // Hands free: try picking up any scene cube not yet taken on this path (a no-op
                    // when scene_cubes is empty, e.g. every caller of the "carried from the start"
                    // cube mode that predates this extension).
                    for (Node* child : expand_cube_pickup(current_node, config_.scene_cubes, scene_cube_polyhedra_, pool_)) {
                        process_child(child, config_.cube_pickup_cost);
                    }
                    break;
                case CubeState::InHand:
                    for (Node* child : expand_cube_placement(current_node, cube_support_, reachability_, config_.expansion_params, pool_)) {
                        process_child(child, config_.cube_place_cost);
                    }
                    break;
                case CubeState::PlacedActive:
                    for (Node* child : expand_onto_cube(current_node, reachability_, config_.cube_height, config_.cube_half_extent - config_.inner_margin, config_.expansion_params, pool_)) {
                        process_child(child, config_.cube_step_cost);
                    }
                    break;
            }
        }
    }
}

} // namespace nas
