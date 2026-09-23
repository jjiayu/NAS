#pragma once

// Node — de-globalized, biped-only port (see PLAN.md phase 4: 2 effectors
// only for now, no N-effector generalization, decided 2026-09-18). One
// concrete Node type shared by both search strategies (NAS/Tree and
// CASSR/AstarSearch), same as the original code — fields only meaningful
// to one strategy are commented as such rather than split into a type
// hierarchy nobody asked for yet.
//
// Ownership: Nodes are never `new`'d directly — NodePool below owns them
// (via a std::deque, which keeps element addresses stable across
// insertions, unlike std::vector) and hands out non-owning Node* valid for
// the pool's lifetime. See PLAN.md's "Node ownership" decision: this is
// what makes repeated planning calls from a long-lived process (the
// eventual Python bindings) safe, instead of the old code's `new` that was
// only ever freed once in one spot (AstarSearch) and never in Tree.

#include "nas/core/types.hpp"

#include <array>
#include <deque>
#include <vector>

namespace nas {

enum class StanceFoot : int { Left = 0, Right = 1 };

inline StanceFoot other_foot(StanceFoot foot) {
    return foot == StanceFoot::Left ? StanceFoot::Right : StanceFoot::Left;
}

class Node {
public:
    int node_id = 0;

    // For NAS/Tree: a node can have several parents (it's a DAG once
    // geometrically-similar nodes are merged). CASSR/AstarSearch populates
    // this too (mirroring the old code) but only ever reads the single
    // `parent` field below.
    std::vector<Node*> parent_ptrs;

    // For NAS/Tree's own KD-tree spatial index only (planners/tree_search,
    // phase 6) — irrelevant to CASSR/AstarSearch.
    Node* kd_left_ptr = nullptr;
    Node* kd_right_ptr = nullptr;

    std::vector<Point_3> patch_vertices;
    Polygon_2 patch_polygon_2d;
    Transformation transformation_to_2d;
    Transformation transformation_to_3d;

    StanceFoot stance_foot = StanceFoot::Left;

    // -1 = "not on any surface" (the start/root node). The original Node
    // never gave this a default, which meant AstarSearch's start node read
    // as uninitialized memory — see docs/paper-deltas.md.
    int surface_id = -1;

    int depth = 0;
    Point_3 centroid{0.0, 0.0, 0.0};

    // Per-stance-foot history of visited surface ids, one entry per depth
    // layer — used by cycle_path_detection() below. Indexed by StanceFoot,
    // hence exactly 2 slots (see PLAN.md's "2 effecteurs seulement").
    std::array<std::vector<std::vector<int>>, 2> pred_surface_ids;

    double foot_yaw = 0.0;

    // For CASSR/AstarSearch only: single parent + open-set bookkeeping.
    // Deliberately NOT initialized to any sentinel here (the old code's
    // g_score=inf/h_score=0/f_score=inf init in get_children was dead code
    // — search() always overwrote it immediately, see docs/paper-deltas.md)
    // — whichever search algorithm creates/visits a node owns these.
    Node* parent = nullptr;
    double g_score = 0.0;
    double h_score = 0.0;
    double f_score = 0.0;

    // True if `point`, expressed in this node's surface frame, falls
    // inside (or on the boundary of) patch_polygon_2d.
    bool check_if_node_contains_point(const Point_3& point) const;
};

// Prevents re-visiting the same surface with the same foot after having
// left it once (Sec. IV / hasContactSurfaceNotBeenLeft() in the CASSR
// paper). Walks `parent`'s history for `current_stance_foot`.
bool cycle_path_detection(const Node* parent, StanceFoot current_stance_foot, int surface_id);

// Owns every Node created for one search run. std::deque guarantees
// pointer/reference stability across insertions, so handed-out Node*
// remain valid for the pool's whole lifetime without per-node heap
// allocation bookkeeping (no unique_ptr indirection needed).
class NodePool {
public:
    // Assigns node_id sequentially so planners don't each need their own
    // counter (the old code had AstarSearch/Tree each own one separately).
    Node* create() {
        nodes_.emplace_back();
        nodes_.back().node_id = next_id_++;
        return &nodes_.back();
    }

    size_t size() const { return nodes_.size(); }

private:
    std::deque<Node> nodes_;
    int next_id_ = 0;
};

} // namespace nas
