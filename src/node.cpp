#include "node.hpp"
#include <iostream>

using namespace nas;

Node::Node() {    // surface_id = 0;
    node_id = 0;
    parent_ptrs = std::vector<Node*>();
}

bool Node::check_if_node_contains_point(const Point_3& point) {

    // Transform the point to the surface coordinate system
    std::vector<Point_2> point_in_surface_coord = transform_3d_points_to_surface_plane(std::vector<Point_3>{point}, this->transform_to_2d);

    // Check if the point is in the polygon or on its boundary
    CGAL::Bounded_side result = this->patch_polygon.bounded_side(point_in_surface_coord[0]);//take the first point of the transformed point (the list only have one component)
    
    return result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY;
}