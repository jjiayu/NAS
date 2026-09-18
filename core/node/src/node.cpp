#include "nas/core/node.hpp"
#include "nas/core/geometry.hpp"

#include <algorithm>

namespace nas {

bool Node::check_if_node_contains_point(const Point_3& point) const {
    std::vector<Point_2> point_in_surface_coord =
        transform_3d_points_to_surface_plane({point}, this->transformation_to_2d);

    CGAL::Bounded_side result = this->patch_polygon_2d.bounded_side(point_in_surface_coord[0]);
    return result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY;
}

bool cycle_path_detection(const Node* parent, StanceFoot current_stance_foot, int surface_id) {
    if (parent == nullptr) {
        return false;
    }

    const auto& foot_history = parent->pred_surface_ids[static_cast<size_t>(current_stance_foot)];
    if (foot_history.empty()) {
        return false;
    }

    bool left_surface = false;
    for (int i = static_cast<int>(foot_history.size()) - 1; i >= 0; --i) {
        const auto& layer = foot_history[i];
        bool surface_in_layer = std::find(layer.begin(), layer.end(), surface_id) != layer.end();

        if (!surface_in_layer) {
            left_surface = true;
        } else if (left_surface && surface_in_layer) {
            return true;
        }
    }
    return false;
}

} // namespace nas
