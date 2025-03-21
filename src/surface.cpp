#include "surface.hpp"


namespace nas {

Surface::Surface(const std::vector<Point_3>& points) {
    // Check if surfaces are valid
    if (points.size() < 3) throw std::invalid_argument("At least 3 points are required to fit a surface");

    // Fit the plane
    CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());  // 0 for points

    // Get the normal vector of the plane
    norm = plane.orthogonal_vector();
    norm = norm / std::sqrt(norm.squared_length());

    // Get centroid
    centroid = get_centroid(points);

    // Establish the surface coordinate system at the centroid
    establish_surface_coordinate_system(points);

    // Transform points to surface plane and get 2d polygon (also sort points counterclockwise with convex hull)
    std::vector<Point_2> surf_pts_2d = transform_3d_points_to_surface_plane(points, transform);
    CGAL::convex_hull_2(surf_pts_2d.begin(), surf_pts_2d.end(), std::back_inserter(polygon_2d));

    // Get the 2d polygon vertices
    std::vector<Point_2> surf_pts_2d_sorted;
    for (auto it = polygon_2d.vertices_begin(); it != polygon_2d.vertices_end(); ++it) {
        surf_pts_2d_sorted.push_back(Point_2(it->x(), it->y()));
    }

    // Convert back to 3D
    vertices = transform_2d_points_to_world(surf_pts_2d_sorted, transform.inverse());

    // Print Surface Information
    std::cout << "\n[ Surface Information ]" << std::endl;
    std::cout << "  - Plane: " << plane << std::endl;
    std::cout << "  - Norm: " << norm << std::endl;
    std::cout << "  - Centroid: " << centroid << std::endl;
    std::cout << "  - Transform: " << transform << std::endl;
    
}

void Surface::establish_surface_coordinate_system(const std::vector<Point_3>& points) {
    
    // 1. Find the point furthest from centroid for robust X-axis
    double max_dist = 0;
    Vector_3 x_axis_candidate;
    for (const auto& point : points) {
        Vector_3 vec(point.x() - centroid.x(),
                     point.y() - centroid.y(),
                     point.z() - centroid.z());
        // Project vector onto plane
        Vector_3 projected = vec - (vec * norm) * norm;
        double dist = projected.squared_length();
        if (dist > max_dist) {
            max_dist = dist;
            x_axis_candidate = projected;
        }
    }
    
    // 2. Normalize X-axis
    Vector_3 x_axis = x_axis_candidate / std::sqrt(x_axis_candidate.squared_length());
    
    // 3. Create Y-axis using cross product (ensures right-handed system)
    Vector_3 y_axis = CGAL::cross_product(norm, x_axis);
    y_axis = y_axis / std::sqrt(y_axis.squared_length());
    
    // 4. Double check X-axis is perpendicular (optional but safer)
    x_axis = CGAL::cross_product(y_axis, norm);
    x_axis = x_axis / std::sqrt(x_axis.squared_length());
    
    // Create transformation matrix from world to surface coordinates
    transform = Transformation(
        x_axis.x(), y_axis.x(), norm.x(), centroid.x(),
        x_axis.y(), y_axis.y(), norm.y(), centroid.y(),
        x_axis.z(), y_axis.z(), norm.z(), centroid.z()
    );
}

} // namespace nas