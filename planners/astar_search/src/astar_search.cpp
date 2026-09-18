#include "nas/planners/astar_search.hpp"
#include "nas/core/geometry.hpp"

#include <algorithm>

namespace nas {

namespace {

// Stateless: no config needed, matches the old CompareNodes exactly.
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const { return a->f_score > b->f_score; }
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
    NodeHash node_hash(config_.node_similarity_threshold, config_.expansion_params.rotation_enabled,
                        config_.expansion_params.yaw_angle_increment);
    NodeEqual node_equal(config_.node_similarity_threshold, config_.expansion_params.rotation_enabled,
                          config_.expansion_params.yaw_angle_increment);

    using OpenSet = boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>>;
    OpenSet open_set;
    std::unordered_map<Node*, OpenSet::handle_type, NodeHash, NodeEqual> node_handles(16, node_hash, node_equal);
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set(16, node_hash, node_equal);

    node_handles[start_node_] = open_set.push(start_node_);

    while (!open_set.empty()) {
        Node* current_node = open_set.top();
        open_set.pop();
        ++expansion_count_;
        node_handles.erase(current_node);

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

        closed_set.insert(current_node);

        std::vector<Node*> children = expand_node(current_node, surfaces_, reachability_,
                                                   ReachabilityDirection::Forward, config_.expansion_params, pool_);

        for (Node* child : children) {
            if (closed_set.find(child) != closed_set.end()) {
                continue;
            }

            double tentative_g_score = current_node->g_score + 1.0;
            double tentative_h_score = heuristic(child);
            double tentative_f_score = tentative_g_score + tentative_h_score;

            auto handle_it = node_handles.find(child);
            if (handle_it == node_handles.end()) {
                child->g_score = tentative_g_score;
                child->h_score = tentative_h_score;
                child->f_score = tentative_f_score;
                child->parent = current_node;
                node_handles[child] = open_set.push(child);
            } else if (tentative_g_score < handle_it->first->g_score) {
                Node* existing_node = handle_it->first;
                existing_node->g_score = tentative_g_score;
                existing_node->h_score = tentative_h_score;
                existing_node->f_score = tentative_f_score;
                existing_node->parent = current_node;
                open_set.increase(handle_it->second);
                // `child` itself is simply left unreferenced in the pool —
                // unlike the old code's `delete child`, NodePool has no
                // per-element free. Not a leak (freed with the pool), just
                // a few extra unused Nodes for the search's lifetime; see
                // docs/paper-deltas.md.
            }
        }
    }
}

} // namespace nas
