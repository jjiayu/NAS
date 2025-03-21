#include "types.hpp"
#include "geometry.hpp"

namespace nas {

std::vector<Point_2> transform_3d_points_to_surface_plane(const std::vector<Point_3>& points, const Transformation& transformation) {
    std::vector<Point_2> transformed_points;
    for (const auto& point : points) {
        Point_3 transformed = transformation(point);
        transformed_points.push_back(Point_2(transformed.x(), transformed.y()));
    }
    return transformed_points;
}

std::vector<Point_3> transform_2d_points_to_world(const std::vector<Point_2>& points, const Transformation& inverse_transformation) {
    std::vector<Point_3> transformed_points;
    for (const auto& point : points) {
        Point_3 transformed = inverse_transformation(Point_3(point.x(), point.y(), 0));
        transformed_points.push_back(transformed);
    }
    return transformed_points;
}
} // namespace nas