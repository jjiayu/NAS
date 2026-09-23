#include "nas/planners/astar_search.hpp"
#include "nas/core/geometry.hpp"

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
// (surface, stance, yaw bin, 10 cm centroid cell) and a query scans the 27
// neighbouring cells, so similar nodes are found across cell boundaries and the
// cost is independent of the set size. Used for both the open and closed sets.
class PatchIndex {
public:
    PatchIndex(double tol, bool rotation_enabled, double yaw_increment)
        : tol_(tol), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

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
        int surface, stance, yaw, x, y, z;
        bool operator==(const Cell& o) const { return surface == o.surface && stance == o.stance && yaw == o.yaw && x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash {
        size_t operator()(const Cell& c) const {
            size_t seed = 0;
            for (int v : {c.surface, c.stance, c.yaw, c.x, c.y, c.z}) boost::hash_combine(seed, v);
            return seed;
        }
    };
    static constexpr double CELL = 0.1;

    Cell cell_of(const Node& n) const {
        return {n.surface_id, static_cast<int>(n.stance_foot),
                rotation_enabled_ ? static_cast<int>(n.foot_yaw / yaw_increment_) : 0,
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.x()) / CELL)),
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.y()) / CELL)),
                static_cast<int>(std::floor(CGAL::to_double(n.centroid.z()) / CELL))};
    }
    bool similar(const Node& a, const Node& b) const {
        if (a.surface_id != b.surface_id || a.stance_foot != b.stance_foot) return false;
        if (rotation_enabled_ && static_cast<int>(a.foot_yaw / yaw_increment_) != static_cast<int>(b.foot_yaw / yaw_increment_)) return false;
        return patch_distance(a, b) < tol_;
    }

    double tol_;
    bool rotation_enabled_;
    double yaw_increment_;
    std::unordered_map<Cell, std::vector<Node*>, CellHash> cells_;
};

} // namespace

AstarSearch::AstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, AstarSearchConfig config)
    : surfaces_(std::move(surfaces)), reachability_(std::move(reachability)), config_(std::move(config)) {

    if (config_.goal_surface_id >= static_cast<int>(surfaces_.size())) {
        throw std::invalid_argument("AstarSearch: goal_surface_id " + std::to_string(config_.goal_surface_id) + " is not a surface of the scenario");
    }
    start_node_ = pool_.create();
    start_node_->patch_vertices = {config_.start_position};
    start_node_->stance_foot = config_.start_stance_foot;
    start_node_->centroid = config_.start_position;
    start_node_->foot_yaw = config_.expansion_params.rotation_enabled ? config_.start_foot_yaw : 0.0;
    start_node_->depth = 0;
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
    // A single-point patch has no area for EPA (it needs >=3 points), so the
    // start node's heuristic is the plain Euclidean distance — matches the old
    // code exactly (see docs/paper-deltas.md).
    start_node_->h_score = compute_euclidean_distance(config_.start_position, goal_point());
    start_node_->f_score = start_node_->g_score + start_node_->h_score;
    start_node_->parent = nullptr;
}

Point_3 AstarSearch::goal_point() const {
    return config_.goal_surface_id >= 0 ? surfaces_[static_cast<size_t>(config_.goal_surface_id)].centroid : config_.goal_location;
}

double AstarSearch::heuristic(const Node* node) const {
    const bool surface_goal = config_.goal_surface_id >= 0;
    switch (config_.distance_metric) {
        case DistanceMetric::Euclidean:
            return compute_euclidean_distance(node->centroid, goal_point());
        case DistanceMetric::Epa:
        default:
            if (surface_goal) {
                return config_.heuristic_weight *
                       calculate_epa_distance_patch_to_patch(node->patch_vertices, surfaces_[static_cast<size_t>(config_.goal_surface_id)].vertices_3d);
            }
            return config_.heuristic_weight * calculate_epa_distance_point_to_patch(node->patch_vertices, config_.goal_location);
    }
}

void AstarSearch::search() {
    using OpenSet = boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>>;
    OpenSet open_set;
    // open_index answers "is an equivalent node already open?"; the heap handle
    // of an open node is looked up by the node itself.
    const auto& ep = config_.expansion_params;
    PatchIndex open_index(config_.node_similarity_threshold, ep.rotation_enabled, ep.yaw_angle_increment);
    PatchIndex closed_index(config_.node_similarity_threshold, ep.rotation_enabled, ep.yaw_angle_increment);
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

        const bool at_goal = config_.goal_surface_id >= 0 ? current_node->surface_id == config_.goal_surface_id
                                                           : current_node->check_if_node_contains_point(config_.goal_location);
        if (current_node->stance_foot == config_.goal_stance_foot && at_goal) {
            Node* current = current_node;
            while (current != nullptr) {
                result_path_.push_back(current);
                current = current->parent;
            }
            std::reverse(result_path_.begin(), result_path_.end());
            return;
        }

        closed_index.insert(current_node);

        std::vector<Node*> children = expand_node(current_node, surfaces_, reachability_,
                                                   ReachabilityDirection::Forward, config_.expansion_params, pool_);

        for (Node* child : children) {
            if (closed_index.find(child) != nullptr) {
                if (config_.on_child) config_.on_child(expansion_count_, *child, ChildAction::SkippedClosed);
                continue;
            }

            double edge_cost = 1.0;
            if (config_.yaw_change_weight > 0.0 && config_.expansion_params.rotation_enabled) {
                double dyaw = std::fmod(std::abs(child->foot_yaw - current_node->foot_yaw), 2.0 * M_PI);
                if (dyaw > M_PI) dyaw = 2.0 * M_PI - dyaw;
                edge_cost += config_.yaw_change_weight * dyaw;
            }
            if (config_.heading_weight > 0.0 && config_.expansion_params.rotation_enabled) {
                // the rough direction of travel: from the parent's centroid to the goal
                const Point_3 goal = goal_point();
                double gx = CGAL::to_double(goal.x() - current_node->centroid.x()), gy = CGAL::to_double(goal.y() - current_node->centroid.y());
                if (std::hypot(gx, gy) > 1e-6) {
                    double d = std::fmod(std::abs(child->foot_yaw - std::atan2(gy, gx)), 2.0 * M_PI);
                    if (d > M_PI) d = 2.0 * M_PI - d;
                    edge_cost += config_.heading_weight * d;
                }
            }
            double tentative_g_score = current_node->g_score + edge_cost;
            double tentative_h_score = heuristic(child);
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
        }
    }
}

} // namespace nas
