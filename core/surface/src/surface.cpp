#include "nas/core/surface.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <stdexcept>

namespace nas {

Surface::Surface(const std::vector<Point_3>& points, int surface_idx, double foot_length, double foot_width) {
    if (points.size() < 3) throw std::invalid_argument("At least 3 points are required to fit a surface");

    surface_id = surface_idx;

    CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

    norm = plane.orthogonal_vector();
    norm = norm / std::sqrt(norm.squared_length());

    centroid = get_centroid(points);

    establish_surface_coordinate_system(points);

    // Transform points to surface plane and get 2d polygon (also sort
    // points counterclockwise with convex hull further down).
    vertices_2d = transform_3d_points_to_surface_plane(points, transform_to_surface);

    // Shrink patches by foot_length and foot_width.
    std::vector<Point_2> vertices_2d_shrinked;
    for (const auto& vertex : vertices_2d) {
        double new_x = vertex.x();
        double new_y = vertex.y();
        if (vertex.x() > 0) new_x -= foot_length / 2.0;
        if (vertex.x() < 0) new_x += foot_length / 2.0;
        if (vertex.y() > 0) new_y -= foot_width / 2.0;
        if (vertex.y() < 0) new_y += foot_width / 2.0;
        vertices_2d_shrinked.emplace_back(new_x, new_y);
    }

    CGAL::convex_hull_2(vertices_2d_shrinked.begin(), vertices_2d_shrinked.end(), std::back_inserter(polygon_2d));

    vertices_2d.clear();
    for (auto it = polygon_2d.vertices_begin(); it != polygon_2d.vertices_end(); ++it) {
        vertices_2d.push_back(Point_2(it->x(), it->y()));
    }

    vertices_3d = transform_2d_points_to_world(vertices_2d, transform_to_3d);

    // Vertices are coplanar, use the safe wrapper (CGAL::convex_hull_3
    // asserts on coplanar input).
    polyhedron_3d = convex_hull_3_from_coplanar_points(vertices_3d, norm);
}

void Surface::establish_surface_coordinate_system(const std::vector<Point_3>& points) {
    Vector_3 world_x(1, 0, 0);
    Vector_3 world_y(0, 1, 0);

    Vector_3 x_axis = world_x - (world_x * norm) * norm;
    x_axis = x_axis / std::sqrt(x_axis.squared_length());

    Vector_3 y_axis = world_y - (world_y * norm) * norm;
    y_axis = y_axis - (x_axis * y_axis) * x_axis; // Gram-Schmidt
    y_axis = y_axis / std::sqrt(y_axis.squared_length());

    transform_to_3d = Transformation(
        x_axis.x(), y_axis.x(), norm.x(), centroid.x(),
        x_axis.y(), y_axis.y(), norm.y(), centroid.y(),
        x_axis.z(), y_axis.z(), norm.z(), centroid.z()
    );

    transform_to_surface = transform_to_3d.inverse();
}

} // namespace nas
