#include "nas/planners/grid_astar_search.hpp"
#include "nas/core/geometry.hpp"

#include <algorithm>

namespace nas {

namespace {

// Duplicated from core/expansion::effector_name rather than depending on
// the whole core/expansion module for one two-line helper — this baseline
// doesn't use expand_node's Minkowski-sum/clip approach at all.
std::string effector_name(StanceFoot foot) {
    return foot == StanceFoot::Left ? "LF" : "RF";
}

struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const { return a->f_score > b->f_score; }
};

// Two nodes are "the same" if they land in the same grid cell with the
// same stance foot (and, if rotation is enabled, the same quantized foot
// yaw) — grid_env.world_to_grid() is exact here since every candidate
// child's centroid is itself produced by grid_env.grid_to_world() for some
// integer cell, so there's no separate rounding step to disagree with it.
class GridNodeHash {
public:
    GridNodeHash(const GridEnvironment* grid_env, bool rotation_enabled, double yaw_increment)
        : grid_env_(grid_env), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

    size_t operator()(const Node* node) const {
        auto [gx, gy] = grid_env_->world_to_grid(node->centroid);
        size_t seed = 0;
        boost::hash_combine(seed, gx);
        boost::hash_combine(seed, gy);
        boost::hash_combine(seed, static_cast<int>(node->stance_foot));
        boost::hash_combine(seed, node->surface_id);
        if (rotation_enabled_) {
            boost::hash_combine(seed, static_cast<int>(node->foot_yaw / yaw_increment_));
        }
        return seed;
    }

private:
    const GridEnvironment* grid_env_;
    bool rotation_enabled_;
    double yaw_increment_;
};

class GridNodeEqual {
public:
    GridNodeEqual(const GridEnvironment* grid_env, bool rotation_enabled, double yaw_increment)
        : grid_env_(grid_env), rotation_enabled_(rotation_enabled), yaw_increment_(yaw_increment) {}

    bool operator()(const Node* a, const Node* b) const {
        auto [ax, ay] = grid_env_->world_to_grid(a->centroid);
        auto [bx, by] = grid_env_->world_to_grid(b->centroid);
        bool basic_equal = ax == bx && ay == by && a->stance_foot == b->stance_foot && a->surface_id == b->surface_id;
        if (rotation_enabled_ && basic_equal) {
            return static_cast<int>(a->foot_yaw / yaw_increment_) == static_cast<int>(b->foot_yaw / yaw_increment_);
        }
        return basic_equal;
    }

private:
    const GridEnvironment* grid_env_;
    bool rotation_enabled_;
    double yaw_increment_;
};

} // namespace

GridAstarSearch::GridAstarSearch(std::vector<Surface> surfaces, ReachabilityModel reachability, GridAstarSearchConfig config)
    : surfaces_(std::move(surfaces)), reachability_(std::move(reachability)), config_(std::move(config)),
      grid_env_(config_.cell_size) {
    grid_env_.initialize_from_surfaces(surfaces_);

    start_node_ = pool_.create();
    start_node_->patch_vertices = {config_.start_position};
    start_node_->stance_foot = config_.start_stance_foot;
    start_node_->centroid = config_.start_position;
    start_node_->foot_yaw = config_.expansion_params.rotation_enabled ? config_.start_foot_yaw : 0.0;
    start_node_->depth = 0;
    start_node_->g_score = 0.0;

    constraint_when_support_left_ = convert_polytope_to_half_space_constraint(
        reachability_.query(effector_name(StanceFoot::Right), effector_name(StanceFoot::Left), ReachabilityDirection::Forward));
    constraint_when_support_right_ = convert_polytope_to_half_space_constraint(
        reachability_.query(effector_name(StanceFoot::Left), effector_name(StanceFoot::Right), ReachabilityDirection::Forward));
}

double GridAstarSearch::heuristic(const Point_3& position) const {
    return config_.heuristic_weight * compute_euclidean_distance(position, config_.goal_location);
}

std::vector<Node*> GridAstarSearch::get_grid_children(Node* parent, const std::function<bool(Node*, double)>& should_skip) {
    std::vector<Node*> children;

    StanceFoot child_stance_foot = other_foot(parent->stance_foot);
    const HalfSpacePolytopeConstraint& constraint =
        parent->stance_foot == StanceFoot::Left ? constraint_when_support_left_ : constraint_when_support_right_;

    auto [parent_gx, parent_gy] = grid_env_.world_to_grid(parent->centroid);
    int search_radius_cells = static_cast<int>(std::ceil(config_.search_radius_m / grid_env_.cell_size()));

    for (int dy = -search_radius_cells; dy <= search_radius_cells; ++dy) {
        for (int dx = -search_radius_cells; dx <= search_radius_cells; ++dx) {
            if (dx == 0 && dy == 0) continue;

            int target_gx = parent_gx + dx;
            int target_gy = parent_gy + dy;
            if (!grid_env_.is_traversable(target_gx, target_gy)) continue;

            const GridEnvironment::GridCell& cell = grid_env_.get_cell(target_gx, target_gy);
            Point_3 target_xy = grid_env_.grid_to_world(target_gx, target_gy);
            Point_3 target_world_pos(target_xy.x(), target_xy.y(), cell.height);

            double rel_x = CGAL::to_double(target_world_pos.x() - parent->centroid.x());
            double rel_y = CGAL::to_double(target_world_pos.y() - parent->centroid.y());
            double rel_z = CGAL::to_double(target_world_pos.z() - parent->centroid.z());

            if (config_.expansion_params.rotation_enabled && parent->foot_yaw != 0.0) {
                double cos_yaw = std::cos(-parent->foot_yaw);
                double sin_yaw = std::sin(-parent->foot_yaw);
                double rotated_x = cos_yaw * rel_x - sin_yaw * rel_y;
                double rotated_y = sin_yaw * rel_x + cos_yaw * rel_y;
                rel_x = rotated_x;
                rel_y = rotated_y;
            }

            Eigen::Vector3d relative_pos(rel_x, rel_y, rel_z);
            Eigen::VectorXd lhs = constraint.A * relative_pos;
            bool reachable = true;
            for (int i = 0; i < lhs.size() && reachable; ++i) {
                reachable = lhs(i) <= constraint.b(i) + 1e-6;
            }
            if (!reachable) continue;

            std::vector<double> yaw_angles;
            if (config_.expansion_params.rotation_enabled) {
                for (int i = -config_.expansion_params.yaw_discretization_num; i <= config_.expansion_params.yaw_discretization_num; ++i) {
                    yaw_angles.push_back(parent->foot_yaw + i * config_.expansion_params.yaw_angle_increment);
                }
            } else {
                yaw_angles.push_back(0.0);
            }

            for (double yaw : yaw_angles) {
                double normalized_yaw = yaw;
                while (normalized_yaw > M_PI) normalized_yaw -= 2.0 * M_PI;
                while (normalized_yaw < -M_PI) normalized_yaw += 2.0 * M_PI;
                double child_yaw = config_.expansion_params.rotation_enabled ? normalized_yaw : 0.0;

                Node probe; // stack-allocated: only the fields hash/equal look at
                probe.centroid = target_world_pos;
                probe.stance_foot = child_stance_foot;
                probe.surface_id = cell.surface_id;
                probe.foot_yaw = child_yaw;
                if (should_skip(&probe, parent->g_score + 1.0)) continue;

                Node* child = pool_.create();
                child->parent_ptrs.push_back(parent);
                child->patch_vertices = {target_world_pos};
                child->stance_foot = child_stance_foot;
                child->surface_id = cell.surface_id;
                child->depth = parent->depth + 1;
                child->centroid = target_world_pos;
                child->foot_yaw = child_yaw;

                children.push_back(child);
            }
        }
    }

    return children;
}

void GridAstarSearch::search() {
    GridNodeHash node_hash(&grid_env_, config_.expansion_params.rotation_enabled, config_.expansion_params.yaw_angle_increment);
    GridNodeEqual node_equal(&grid_env_, config_.expansion_params.rotation_enabled, config_.expansion_params.yaw_angle_increment);

    using OpenSet = boost::heap::fibonacci_heap<Node*, boost::heap::compare<CompareNodes>>;
    OpenSet open_set;
    std::unordered_map<Node*, OpenSet::handle_type, GridNodeHash, GridNodeEqual> node_handles(16, node_hash, node_equal);
    std::unordered_set<Node*, GridNodeHash, GridNodeEqual> closed_set(16, node_hash, node_equal);

    start_node_->h_score = heuristic(start_node_->centroid);
    start_node_->f_score = start_node_->g_score + start_node_->h_score;
    node_handles[start_node_] = open_set.push(start_node_);

    auto goal_grid = grid_env_.world_to_grid(config_.goal_location);

    while (!open_set.empty()) {
        Node* current_node = open_set.top();
        open_set.pop();
        ++expansion_count_;
        node_handles.erase(current_node);

        if (current_node->stance_foot == config_.goal_stance_foot) {
            auto current_grid = grid_env_.world_to_grid(current_node->centroid);
            if (current_grid == goal_grid) {
                Node* current = current_node;
                while (current != nullptr) {
                    result_path_.push_back(current);
                    current = current->parent;
                }
                std::reverse(result_path_.begin(), result_path_.end());
                return;
            }
        }

        closed_set.insert(current_node);

        // Already closed, or already open with an equal/better g: it could
        // never change the search, so don't even allocate it.
        auto should_skip = [&](Node* probe, double tentative_g) {
            if (closed_set.find(probe) != closed_set.end()) return true;
            auto it = node_handles.find(probe);
            return it != node_handles.end() && tentative_g >= it->first->g_score;
        };
        std::vector<Node*> children = get_grid_children(current_node, should_skip);
        for (Node* child : children) {
            if (closed_set.find(child) != closed_set.end()) continue;

            double tentative_g_score = current_node->g_score + 1.0;
            double tentative_h_score = heuristic(child->centroid);
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
            }
        }
    }
}

} // namespace nas
