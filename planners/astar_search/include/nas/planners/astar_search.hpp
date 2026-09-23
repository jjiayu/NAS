#pragma once

// AstarSearch — CASSR, ported onto core/{node,expansion,reachability,
// surface} (see PLAN.md phase 5). De-globalized: every old global read
// from constants.hpp (start/goal, distance metric, node similarity
// threshold, ...) is now a constructor parameter via AstarSearchConfig,
// instead of the old code reading nas:: globals directly inside the class.
//
// Behavior is a faithful port of the old AstarSearch — same priority
// queue, same node-similarity dedup by (surface_id, stance_foot, yaw,
// quantized centroid+perimeter), same heuristic weighting (x10 for
// gjk/epa, unweighted for euclidean, see docs/paper-deltas.md) — with the
// actual expansion delegated entirely to core/expansion's expand_node(),
// which is the whole point of this rewrite (no more duplicated
// get_children between this and planners/tree_search).

#include "nas/core/expansion.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/core/surface.hpp"

#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <vector>

namespace nas {

enum class DistanceMetric { Euclidean, Gjk, Epa };

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

    Point_3 goal_location;
    StanceFoot goal_stance_foot = StanceFoot::Left;

    DistanceMetric distance_metric = DistanceMetric::Epa;
    // Applied to the gjk/epa heuristic only — NOT to euclidean, matching
    // the old code exactly (see docs/paper-deltas.md: this weight is what
    // makes CASSR a *weighted* A*, sacrificing the admissibility
    // guarantee; the paper confirms this but not the value itself).
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
};

} // namespace nas
