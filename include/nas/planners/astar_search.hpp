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

#include <functional>
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

    ExpansionParams expansion_params;

    // Test/diagnostic seams, unset in production. on_expand: right after a node
    // is popped (1-based expansion index). on_child: after each child's dedup
    // decision, with the expansion index of its parent. Used by
    // tests/golden_all/trace_divergence.cpp to find where two runs first differ.
    std::function<void(int expansion_index, const Node& node)> on_expand;
    std::function<void(int parent_expansion_index, const Node& child, ChildAction action)> on_child;
};

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

    double heuristic(const Node* node) const;
    Point_3 goal_point() const; // the goal position, or the goal surface's centroid
};

} // namespace nas
