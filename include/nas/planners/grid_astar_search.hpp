#pragma once

// GridAstarSearch — the discretized grid baseline (see PLAN.md phase 13:
// low priority, direct port of the old code's algorithm). Same overall
// shape as planners/astar_search's AstarSearch (fibonacci_heap open set,
// hash-deduplicated closed/open sets, node-per-candidate-yaw expansion),
// but the search space is a uniform 2D grid (GridEnvironment) instead of
// continuous patches/polygons, and reachability is checked directly
// against the Forward polytope's H-representation (a point-in-halfspace
// test) rather than by intersecting patches.
//
// Deliberately not ported from the old code: the "mid-pose interpolation"
// distance-cost calculation in the old get_grid_children/search() (~60
// lines) was dead code — tentative_g_score was always just
// `current_node->g_score + 1.0`, the interpolated cost was computed and
// discarded. See docs/paper-deltas.md.

#include "nas/core/geometry.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/grid_environment.hpp"

#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace nas {

struct GridExpansionParams {
    bool rotation_enabled = false;
    int yaw_discretization_num = 3;
    double yaw_angle_increment = 10.0 / 180.0 * M_PI;
};

struct GridAstarSearchConfig {
    Point_3 start_position;
    StanceFoot start_stance_foot = StanceFoot::Right;
    double start_foot_yaw = 0.0;

    Point_3 goal_location;
    StanceFoot goal_stance_foot = StanceFoot::Left;

    double cell_size = 0.05; // meters, matches the old a_star_grid_resolution default
    // Candidate cells are searched in a square of this half-width around
    // the parent, in meters (old code: "search_radius", commented "3
    // meter radius" but actually used as a 1.5m half-width — misleading
    // comment in the original, see docs/paper-deltas.md).
    double search_radius_m = 1.5;
    double heuristic_weight = 10.0;

    GridExpansionParams expansion_params;
};

class GridAstarSearch {
public:
    GridAstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, GridAstarSearchConfig config);

    void search();

    const std::vector<Node*>& result_path() const { return result_path_; }
    int expansion_count() const { return expansion_count_; }
    const GridEnvironment& grid_environment() const { return grid_env_; }

private:
    std::vector<Surface> surfaces_;
    ReachabilityModel reachability_;
    GridAstarSearchConfig config_;
    GridEnvironment grid_env_;
    NodePool pool_;

    // Cached once at construction (old code's own approach) rather than
    // rebuilt on every get_grid_children() call — convert_polytope_to_half_space_constraint
    // recomputes every facet's plane, not a cheap thing to redo per node
    // expansion. Indexed by the *support* foot.
    HalfSpacePolytopeConstraint constraint_when_support_left_;
    HalfSpacePolytopeConstraint constraint_when_support_right_;

    Node* start_node_ = nullptr;
    std::vector<Node*> result_path_;
    int expansion_count_ = 0;

    // `should_skip(probe, tentative_g)` is asked about a stack-allocated
    // probe node *before* anything is allocated in pool_ — NodePool never
    // frees individual nodes, so allocating every candidate first and
    // discarding duplicates afterwards (what the old code did with
    // new/delete) would grow memory with every already-seen candidate. On
    // a scene the grid can't cross (e.g. NarrowPassage's ~2cm shrunk
    // passage at 5cm cells) the whole reachable set gets exhausted, which
    // made that pattern exhaust RAM.
    std::vector<Node*> get_grid_children(Node* parent, const std::function<bool(Node*, double)>& should_skip);
    double heuristic(const Point_3& position) const;
};

} // namespace nas
