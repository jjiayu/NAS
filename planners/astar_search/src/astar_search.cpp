#include "nas/planners/astar_search.hpp"
#include "nas/core/geometry.hpp"

#include <algorithm>

#include <cmath>
#include <memory>

namespace nas {

namespace {

// Default: matches the old CompareNodes exactly (f only; equal f = arbitrary
// order). deterministic_ties: f is compared after rounding to 1 nm, and nodes
// that tie are ordered by creation order (node_id, lower first). Scores that
// are equal in exact arithmetic (many nodes: distance 0 to the goal patch) differ
// by last-bit noise that changes with heap state, and that noise alone decided
// which node was expanded first.
struct CompareNodes {
    bool deterministic_ties = false;
    bool ties_lifo = false; // among ties, most recently created first
    bool operator()(const Node* a, const Node* b) const {
        if (!deterministic_ties) return a->f_score > b->f_score;
        long long qa = std::llround(a->f_score * 1e9), qb = std::llround(b->f_score * 1e9);
        if (qa != qb) return qa > qb;
        return ties_lifo ? a->node_id < b->node_id : a->node_id > b->node_id;
    }
};

// Stateful port of the old NodeHash/NodeEqual — they used to read
// node_similarity_threshold/foot_yaw_rotation_flag/foot_yaw_angle_increment
// as globals; here they carry the equivalent AstarSearchConfig values as
// constructor state instead, and are passed as instances to the
// unordered_map/unordered_set constructors below (both types support that
// overload without requiring Hash/Equal to be default-constructible).
class NodeHash {
public:
    NodeHash(double threshold, bool rotation_enabled, double yaw_increment)
        : threshold_(threshold), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

    size_t operator()(const Node* node) const {
        int x = static_cast<int>(CGAL::to_double(node->centroid.x()) / threshold_);
        int y = static_cast<int>(CGAL::to_double(node->centroid.y()) / threshold_);
        int z = static_cast<int>(CGAL::to_double(node->centroid.z()) / threshold_);
        int quantized_perimeter = static_cast<int>(node->perimeter / threshold_);

        size_t seed = 0;
        boost::hash_combine(seed, x);
        boost::hash_combine(seed, y);
        boost::hash_combine(seed, z);
        boost::hash_combine(seed, quantized_perimeter);
        boost::hash_combine(seed, node->surface_id);
        boost::hash_combine(seed, static_cast<int>(node->stance_foot));
        if (rotation_enabled_) {
            int quantized_yaw = static_cast<int>(node->foot_yaw / yaw_increment_);
            boost::hash_combine(seed, quantized_yaw);
        }
        return seed;
    }

private:
    double threshold_;
    bool rotation_enabled_;
    double yaw_increment_;
};

class NodeEqual {
public:
    NodeEqual(double threshold, bool rotation_enabled, double yaw_increment)
        : threshold_(threshold), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

    bool operator()(const Node* a, const Node* b) const {
        int xa = static_cast<int>(CGAL::to_double(a->centroid.x()) / threshold_);
        int ya = static_cast<int>(CGAL::to_double(a->centroid.y()) / threshold_);
        int za = static_cast<int>(CGAL::to_double(a->centroid.z()) / threshold_);
        int xb = static_cast<int>(CGAL::to_double(b->centroid.x()) / threshold_);
        int yb = static_cast<int>(CGAL::to_double(b->centroid.y()) / threshold_);
        int zb = static_cast<int>(CGAL::to_double(b->centroid.z()) / threshold_);

        int perim_a = static_cast<int>(a->perimeter / threshold_);
        int perim_b = static_cast<int>(b->perimeter / threshold_);

        bool basic_equal = (xa == xb && ya == yb && za == zb && perim_a == perim_b &&
                             a->surface_id == b->surface_id && a->stance_foot == b->stance_foot);

        if (rotation_enabled_ && basic_equal) {
            int yaw_a = static_cast<int>(a->foot_yaw / yaw_increment_);
            int yaw_b = static_cast<int>(b->foot_yaw / yaw_increment_);
            return yaw_a == yaw_b;
        }
        return basic_equal;
    }

private:
    double threshold_;
    bool rotation_enabled_;
    double yaw_increment_;
};

// Set of nodes with a "find an equivalent node" query, one implementation per
// DedupMode. Used for both the open set (alongside the heap handles) and the
// closed set.
class NodeIndex {
public:
    virtual ~NodeIndex() = default;
    virtual Node* find(const Node* n) const = 0;
    virtual void insert(Node* n) = 0;
    virtual void erase(const Node* n) = 0;
};

// The old unordered_set semantics, unchanged.
class LegacyIndex : public NodeIndex {
public:
    LegacyIndex(double threshold, bool rotation_enabled, double yaw_increment)
        : set_(16, NodeHash(threshold, rotation_enabled, yaw_increment), NodeEqual(threshold, rotation_enabled, yaw_increment)) {}
    Node* find(const Node* n) const override {
        auto it = set_.find(const_cast<Node*>(n));
        return it == set_.end() ? nullptr : *it;
    }
    void insert(Node* n) override { set_.insert(n); }
    void erase(const Node* n) override { set_.erase(const_cast<Node*>(n)); }

private:
    std::unordered_set<Node*, NodeHash, NodeEqual> set_;
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

// Tolerance-based similarity with a spatial index: nodes are bucketed by
// (surface, stance, yaw bin, 10 cm centroid cell) and a query scans the 27
// neighbouring cells, so similar nodes are found across cell boundaries and the
// cost stays independent of the open-set size.
class SpatialIndex : public NodeIndex {
public:
    SpatialIndex(DedupMode mode, double tol, bool rotation_enabled, double yaw_increment)
        : mode_(mode), tol_(tol), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

    Node* find(const Node* n) const override {
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
    void insert(Node* n) override { cells_[cell_of(*n)].push_back(n); }
    void erase(const Node* n) override {
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
        if (mode_ == DedupMode::CentroidPerimeterTolerance) {
            return CGAL::to_double(CGAL::squared_distance(a.centroid, b.centroid)) < tol_ * tol_ && std::abs(a.perimeter - b.perimeter) < tol_;
        }
        return patch_distance(a, b) < tol_;
    }

    DedupMode mode_;
    double tol_;
    bool rotation_enabled_;
    double yaw_increment_;
    std::unordered_map<Cell, std::vector<Node*>, CellHash> cells_;
};

std::unique_ptr<NodeIndex> make_index(const AstarSearchConfig& c) {
    const auto& ep = c.expansion_params;
    if (c.dedup_mode == DedupMode::LegacyCells)
        return std::make_unique<LegacyIndex>(c.node_similarity_threshold, ep.rotation_enabled, ep.yaw_angle_increment);
    return std::make_unique<SpatialIndex>(c.dedup_mode, c.node_similarity_threshold, ep.rotation_enabled, ep.yaw_angle_increment);
}

} // namespace

AstarSearch::AstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, AstarSearchConfig config)
    : surfaces_(std::move(surfaces)), reachability_(std::move(reachability)), config_(std::move(config)) {

    start_node_ = pool_.create();
    start_node_->patch_vertices = {config_.start_position};
    start_node_->stance_foot = config_.start_stance_foot;
    start_node_->centroid = config_.start_position;
    start_node_->foot_yaw = config_.expansion_params.rotation_enabled ? config_.start_foot_yaw : 0.0;
    start_node_->depth = 0;
    start_node_->perimeter = 0.0;
    start_node_->g_score = 0.0;
    // A single-point patch has no area for gjk/epa (they need >=3 points),
    // so the start node's heuristic is always euclidean regardless of the
    // configured metric — matches the old code exactly (see
    // docs/paper-deltas.md).
    start_node_->h_score = compute_euclidean_distance(config_.start_position, config_.goal_location);
    start_node_->f_score = start_node_->g_score + start_node_->h_score;
    start_node_->parent = nullptr;
}

double AstarSearch::heuristic(const Node* node) const {
    switch (config_.distance_metric) {
        case DistanceMetric::Gjk:
            return config_.heuristic_weight *
                   calculate_gjk_distance_point_to_patch(node->patch_vertices, config_.goal_location);
        case DistanceMetric::Epa:
            return config_.heuristic_weight *
                   calculate_epa_distance_point_to_patch(node->patch_vertices, config_.goal_location);
        case DistanceMetric::Euclidean:
        default:
            return compute_euclidean_distance(node->centroid, config_.goal_location);
    }
}

void AstarSearch::search() {
    using OpenSet = boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>>;
    OpenSet open_set(CompareNodes{config_.deterministic_ties, config_.ties_lifo});
    // open_index answers "is an equivalent node already open?"; the heap handle
    // of an open node is looked up by the node itself.
    std::unique_ptr<NodeIndex> open_index = make_index(config_);
    std::unique_ptr<NodeIndex> closed_index = make_index(config_);
    std::unordered_map<Node*, OpenSet::handle_type> node_handles;

    node_handles[start_node_] = open_set.push(start_node_);
    open_index->insert(start_node_);

    while (!open_set.empty()) {
        Node* current_node = open_set.top();
        open_set.pop();
        ++expansion_count_;
        node_handles.erase(current_node);
        open_index->erase(current_node);
        if (config_.on_expand) config_.on_expand(expansion_count_, *current_node);

        if (current_node->stance_foot == config_.goal_stance_foot &&
            current_node->check_if_node_contains_point(config_.goal_location)) {
            Node* current = current_node;
            while (current != nullptr) {
                result_path_.push_back(current);
                current = current->parent;
            }
            std::reverse(result_path_.begin(), result_path_.end());
            return;
        }

        closed_index->insert(current_node);

        std::vector<Node*> children = expand_node(current_node, surfaces_, reachability_,
                                                   ReachabilityDirection::Forward, config_.expansion_params, pool_);

        for (Node* child : children) {
            if (closed_index->find(child) != nullptr) {
                if (config_.on_child) config_.on_child(expansion_count_, *child, ChildAction::SkippedClosed);
                continue;
            }

            double tentative_g_score = current_node->g_score + 1.0;
            double tentative_h_score = heuristic(child);
            double tentative_f_score = tentative_g_score + tentative_h_score;

            Node* existing = open_index->find(child);
            if (existing == nullptr) {
                child->g_score = tentative_g_score;
                child->h_score = tentative_h_score;
                child->f_score = tentative_f_score;
                child->parent = current_node;
                node_handles[child] = open_set.push(child);
                open_index->insert(child);
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
