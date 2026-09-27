#pragma once

// AstarSearch — CASSR, on core/{node,expansion,reachability,surface} (see
// PLAN.md phase 5). Nothing is read from globals: start/goal, heuristic weight
// and node-similarity threshold are AstarSearchConfig fields.
//
// A weighted A* (g counts steps, h = heuristic_weight x EPA distance from the
// goal to the node's patch), the expansion delegated to core/expansion's
// expand_node(). Where it departs from the old code (open-set tie order,
// node-similarity test, clip, patch cleaning) and why is in
// docs/paper-deltas.md, "Profil retenu".

#include "nas/core/expansion.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/core/surface.hpp"

#include <array>
#include <functional>
#include <optional>
#include <variant>
#include <vector>

namespace nas {

// Heuristic distance from the goal. Epa: distance from the goal to the node's
// patch (COAL EPA), weighted by heuristic_weight - the CASSR heuristic.
// Euclidean: distance from the goal to the node's patch centroid, unweighted
// (as in the old code). The old code also offered a GJK variant of the patch
// distance; it was dropped.
enum class DistanceMetric { Euclidean, Epa };

// What the search did with a freshly expanded child (see AstarSearchConfig::on_child).
enum class ChildAction {
    SkippedClosed,     // an equal node was already expanded
    Pushed,            // no equal node in the open set: added
    ImprovedExisting,  // equal node in the open set, this path is shorter: replaced its scores/parent
    MergedWorse,       // equal node in the open set, not better: dropped
};

struct AstarSearchConfig {
    Point_3 start_position;
    StanceFoot start_stance_foot = StanceFoot::Right;
    double start_foot_yaw = 0.0;

    // The goal is EITHER a position (goal_location, the default) OR a surface (goal_surface_id >= 0).
    // Position: the search ends on a node of the goal stance foot whose patch contains goal_location; the
    // heuristic measures the distance to that point. Surface: it ends on a node of the goal stance foot standing
    // on that surface (an index into the surfaces the search was given); the heuristic measures the distance to
    // the surface's patch (EPA between two polytopes) or, for the Euclidean metric, to its centroid.
    Point_3 goal_location;
    int goal_surface_id = -1;
    StanceFoot goal_stance_foot = StanceFoot::Left;

    DistanceMetric distance_metric = DistanceMetric::Epa;

    // Weight of the constant cost of a step (the paper's edge cost 1). 0 ignores it: only the optional costs
    // below (and the heuristic) then drive the search.
    double step_weight = 1.0;

    // Optional edge cost on rotating: the cost of an edge is 1 + yaw_change_weight * |yaw of the child - yaw of the
    // parent| (wrapped to [0, pi]). 0 (default) = the paper's cost, always 1 ("we do not impose a penalty on the
    // rotation", V-B.5). 0.1 is the weight of the old code's unused penalty (its grid baseline, commented out) and
    // the paper mentions a small cost penalising foot rotation for its videos. Only with rotation enabled.
    double yaw_change_weight = 0.0;

    // Optional edge cost on the heading: + heading_weight * |yaw of the child - direction from the parent's patch centroid
    // to the goal| (wrapped to [0, pi], in the horizontal plane; the goal is the goal position or the goal surface's
    // centroid). It favours feet aligned with the rough direction of travel, unlike yaw_change_weight, which penalises
    // rotating from one step to the next whatever the direction. 0 (default) = off. Only with rotation enabled.
    double heading_weight = 0.0;

    // Optional constraint + optional edge cost on the FINAL foot yaw (as opposed to yaw_change_weight/heading_weight,
    // which shape the whole path). Unset (default) = no constraint at all, nothing below is evaluated: the goal
    // stance foot's yaw at termination can be anything, exactly like before this option existed.
    //
    // Set goal_yaw_target: the search now only terminates on a node whose foot_yaw is within goal_yaw_tolerance
    // (wrapped, radians) of the target — same position/surface + stance-foot condition as before, plus this one.
    // A too-tight, unreachable combination (e.g. a target the yaw discretization can never actually land in) fails
    // like today's "no path" (see max_expansions), not a crash or a hang.
    //
    // goal_yaw_weight (0 by default, like the other optional costs) additionally biases the search toward that
    // target throughout the path — same mechanism as heading_weight (edge cost, wrapped angular distance) but
    // against the fixed target instead of the dynamic direction-to-goal, and with a dead zone of goal_yaw_tolerance
    // (no cost once already within the accepted range). At 0, only the hard constraint above applies (no bias, the
    // search may need more expansions to happen upon a node that satisfies it). Requires expansion_params.rotation_enabled
    // — AstarSearch's constructor throws if goal_yaw_target is set without it (foot_yaw is always 0 otherwise, so any
    // target other than 0 would be silently unreachable).
    std::optional<double> goal_yaw_target;
    double goal_yaw_tolerance = 0.0;
    double goal_yaw_weight = 0.0;

    // A goal for ONE foot: either a precise point (like goal_location above) or an arbitrary
    // polytope (>= 3 vertices, world frame, validated at construction) -- the two shapes
    // goal_location/goal_surface_id already use separately, unified into one representation.
    // Independent of the shape, an optional accepted yaw range narrows the target further (unset =
    // no yaw constraint, generalizing goal_yaw_target's own "unset = free" convention). Unwrap the
    // range if it would otherwise cross +/-pi (e.g. {170deg, 190deg}, not {170deg, -170deg}).
    struct FootGoal {
        std::variant<Point_3, std::vector<Point_3>> region;
        std::optional<std::pair<double, double>> yaw_range;
    };

    // Indexed by StanceFoot, each slot INDEPENDENTLY optional (not an array that must be filled in
    // full):
    //   - neither slot set (default): this mechanism is off, goal_location/goal_surface_id/
    //     goal_stance_foot/goal_yaw_target above govern exactly as before this option existed;
    //   - exactly one slot set: direct generalization of today's behaviour -- that foot must satisfy
    //     its own slot (shape + optional yaw), the other foot stays as free as it is today;
    //   - both slots set: "closing stance" -- terminates only when the last two consecutive
    //     footsteps (one per foot, whichever arrives last) each satisfy their own slot at once. The
    //     naive approach (aim for one target, then the other) does not work here: the heuristic must
    //     track both feet's remaining distance at every node, not just the one currently being
    //     placed, or the search can converge one foot onto its target while never pulling the
    //     trailing foot toward its own (see astar_search.cpp for the corrected heuristic).
    // Mutually exclusive with goal_surface_id/goal_yaw_target and with the cube extension below
    // (cube_half_extent > 0), all validated at construction: expand_cube_placement gives its child
    // the SAME stance foot as its parent (placing a cube doesn't move a foot), breaking the
    // foot-alternation invariant the closing-stance mode (both slots filled) relies on for its
    // node+parent termination test and heuristic. A single-slot goal never reads the parent's own
    // foot, so it doesn't actually have this problem, but the exclusion applies to foot_goals as a
    // whole for simplicity -- relax later if single-slot + cube together proves useful.
    // goal_location/goal_stance_foot have no "unset" sentinel to validate against (already the case
    // today for goal_location whenever goal_surface_id is used) and are simply ignored.
    std::array<std::optional<FootGoal>, 2> foot_goals;

    // Cube-extension config (see docs/cube-extension-spec.md, docs/cube-implementation-plan.md).
    // 0 (default) = extension entirely off: no cube actions are ever candidates, the search
    // behaves exactly as before this extension existed. > 0 enables it: the search starts with
    // the cube "in hand" (CubeState::InHand on the start node) and considers
    // expand_cube_placement/expand_onto_cube as extra candidate actions alongside expand_node.
    // Requires the reachability model passed to AstarSearch's constructor to also have a "Cube"
    // entry for each stance foot (validated at construction, same style as goal_yaw_target's
    // rotation_enabled check). v1 scope: at most one cube, used once (docs/cube-implementation-plan.md
    // §4) -- expand_onto_cube always deactivates it immediately after a single on-cube step.
    double cube_half_extent = 0.0;
    double cube_height = 0.0;

    // Edge costs for the two cube actions. Both default to step_weight's own default (1.0): a
    // cube action costs as much as an ordinary step unless configured otherwise. Kept separate
    // from step_weight itself (not reused directly) since the heuristic never rewards placing a
    // cube (spec §5.4 -- it doesn't move the foot, so EPA-to-goal doesn't improve), so these are
    // the only lever to make the search prefer/avoid using the cube when a plan is possible both
    // ways.
    double cube_place_cost = 1.0;
    double cube_step_cost = 1.0;

    // Cube-pickup extension (see docs/cube-pickup-spec.md): a cube that rests in the
    // scene from the very start of the search, at a fixed, known position -- as opposed
    // to the "carried from the start" cube modeled by cube_half_extent/cube_height alone
    // above. NOT a Surface (never a surface_id, never injected into the scene's surfaces,
    // never seen by cycle_path_detection): same precedent as FootGoal::region above,
    // which already lives directly in this config rather than in the scene.
    //
    // pickup_affordance reuses FootGoal's own per-foot region+yaw-range shape and 0/1/2-
    // slot convention verbatim -- 1 slot filled: that foot alone must reach in and satisfy
    // it; 2 slots filled: a symmetric two-footed stance is required, tested against the
    // last two consecutive footsteps exactly like foot_goals' own closing-stance mode --
    // but evaluated as a trigger DURING the search (at every expansion), not as a
    // termination condition. Inspired by (not consuming) g1motion's own in-progress,
    // uncommitted "grasp affordance polytope" prototype: a per-foot (x, y, yaw) region
    // around a seed stance, the same shape as FootGoal.
    struct SceneCube {
        std::array<std::optional<FootGoal>, 2> pickup_affordance;
    };

    // Empty (default): this mechanism is entirely off, start_node_->cube_state keeps
    // today's exact convention (cube_half_extent > 0 ? InHand : None). Non-empty: the
    // search starts with EMPTY hands (None) and must pick one of these up before
    // expand_cube_placement/expand_onto_cube ever become candidate actions. Requires
    // cube_half_extent > 0 (validated at construction) -- v1: every scene cube shares the
    // same half_extent/height, no per-cube geometry (nobody asked for differently-sized
    // cubes in one scene).
    std::vector<SceneCube> scene_cubes;

    // Edge cost of the pickup pseudo-action (zero displacement, same rationale as
    // cube_place_cost). Defaults to 1.0, same convention as cube_place_cost/cube_step_cost.
    double cube_pickup_cost = 1.0;

    // Safety limit: the search gives up (empty path) after this many expansions. 0 = no limit.
    // Unweighted heuristics (Euclidean) can expand a very large number of nodes on a continuous
    // state space, and nodes are never freed during a search.
    int max_expansions = 0;

    // Weight on the EPA distance from the goal to a node's patch: what makes
    // CASSR a *weighted* A*, sacrificing the admissibility guarantee (the paper
    // confirms this but not the value; see docs/paper-deltas.md). The start
    // node, a single point, uses the plain Euclidean distance.
    double heuristic_weight = 10.0;

    // Two nodes are "the same" (only the better one is kept) when they share
    // surface, stance foot and yaw bin and their patches are within this
    // distance of each other (paper: "set empirically to 2cm"): the largest
    // vertex-to-boundary distance between the two polygons, both ways. The old
    // code compared int(centroid / t) and int(perimeter / t) instead, which
    // splits patches a few mm apart across a cell boundary and merges nothing
    // else (docs/paper-deltas.md, "Audit de similarité").
    double node_similarity_threshold = 0.02;

    // Side length of PatchIndex's spatial hash cell over the node centroid (x, y, z), in metres.
    // Was a hardcoded 0.1 (10cm); made configurable per docs/patchindex-scalability-note.md, whose
    // "Experience faite" section measured 0.05 (5cm, now the default) at -40 to -42% search time on
    // the scenario that exposed PatchIndex's cost-grows-with-bucket-population issue (StairsGap+
    // scene_cubes), with expansions/path identical to 0.1 on that scenario and on the standard
    // 11-scenario suite -- also checked at 0.2 (worse: bigger cells, bigger buckets, +11%) and 0.02
    // (still zero missed merges, measured with an independent check, but with node_similarity_threshold
    // also at 0.02 there is no margin left, so 0.05 is kept as the default rather than pushing to the
    // edge of that guarantee). Must stay well above node_similarity_threshold (same margin argument as
    // that note's "position" section: a cell smaller than ~2.5x the similarity tolerance could let two
    // centroids within tolerance fall more than 1 cell index apart, outside PatchIndex::find()'s
    // 27-neighbour scan).
    double patch_index_cell_size = 0.05;

    ExpansionParams expansion_params;

    // Test/diagnostic seams, unset in production. on_expand: right after a node
    // is popped (1-based expansion index). on_child: after each child's dedup
    // decision, with the expansion index of its parent. Used by
    // tests/golden_all/trace_divergence.cpp to find where two runs first differ.
    std::function<void(int expansion_index, const Node& node)> on_expand;
    std::function<void(int parent_expansion_index, const Node& child, ChildAction action)> on_child;
};

// Shared shape+yaw test (point containment, or polytope plane-slice + 2D clip against the
// node's own patch): the geometric core of AstarSearch::goal_satisfied/distance_to_goal,
// generalized to take their FootGoal and pre-built hull explicitly instead of reading them
// off `config_`/a cached member, so the same logic backs both AstarSearchConfig::foot_goals
// and the cube-pickup trigger below without duplicating it. `cached_hull` must be non-null
// whenever goal.region holds a polytope (built once, see AstarSearch's target_polyhedra_/
// scene_cube_polyhedra_) -- ignored for a point-shaped region.
bool foot_goal_satisfied(const Node& node, const AstarSearchConfig::FootGoal& goal, const Polyhedron* cached_hull);
double foot_goal_distance(const Node& node, const AstarSearchConfig::FootGoal& goal, DistanceMetric metric);

// Cube-pickup trigger (docs/cube-pickup-spec.md): for every config-level scene cube not yet
// marked picked-up on `parent`'s own path (parent->cubes_picked_up), tests its
// pickup_affordance against `parent` alone (one slot filled) or against (parent,
// parent->parent) together (both slots filled -- same node+parent pairing as foot_goals'
// closing-stance mode) -- a candidate only when parent->cube_state is None or PlacedInactive
// (hands free; InHand/PlacedActive mean a cube is already in play). A free function, not a
// private AstarSearch method, so it stays directly unit-testable like expand_cube_placement,
// and so core/expansion.* never needs to know AstarSearchConfig::SceneCube exists. Produces
// one zero-displacement child per satisfied cube (same foot/position/yaw as parent,
// cube_state -> InHand, cube -> nullopt, that cube's bit set on the CHILD's own
// cubes_picked_up -- parent's own vector is never mutated).
std::vector<Node*> expand_cube_pickup(Node* parent,
                                       const std::vector<AstarSearchConfig::SceneCube>& scene_cubes,
                                       const std::vector<std::array<std::optional<Polyhedron>, 2>>& scene_cube_polyhedra,
                                       NodePool& pool);

class AstarSearch {
public:
    AstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, AstarSearchConfig config);

    void search();

    const std::vector<Node*>& result_path() const { return result_path_; }
    int expansion_count() const { return expansion_count_; }

private:
    std::vector<Surface> surfaces_;
    ReachabilityModel reachability_;
    AstarSearchConfig config_;
    NodePool pool_;

    Node* start_node_ = nullptr;
    std::vector<Node*> result_path_;
    int expansion_count_ = 0;

    // Convex hull of foot_goals[i]'s polytope, built once at construction -- only populated for a
    // slot whose region actually holds a polytope (unused/nullopt for a point-shaped or empty slot).
    std::array<std::optional<Polyhedron>, 2> target_polyhedra_;

    // One entry per config_.scene_cubes[i] (index-aligned), same "nullopt unless
    // polytope-shaped" convention as target_polyhedra_ above, built once at construction.
    std::vector<std::array<std::optional<Polyhedron>, 2>> scene_cube_polyhedra_;

    double heuristic(const Node* node) const;
    Point_3 goal_point() const; // the goal position, or the goal surface's centroid

    // foot_goals support (see AstarSearchConfig::foot_goals). `which` selects config_.foot_goals[which]
    // (must be set). distance_to_goal is unweighted (see weight_if_epa); goal_satisfied tests shape
    // (point containment or polytope overlap) and yaw range together.
    double distance_to_goal(const Node& node, StanceFoot which) const;
    bool goal_satisfied(const Node& node, StanceFoot which) const;
    double weight_if_epa(double raw_distance) const;
};

} // namespace nas
