#include "surface.hpp"
#include <iomanip>
#include "constants.hpp"

namespace nas {

Surface::Surface(const std::vector<Point_3>& points, int& surface_idx) {
    // Check if surfaces are valid
    if (points.size() < 3) throw std::invalid_argument("At least 3 points are required to fit a surface");

    // Assign surface ID
    surface_id = surface_idx;

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
    vertices_2d = transform_3d_points_to_surface_plane(points, transform_to_surface); //Note: the vertices_2d are not sorted now

    // Shrink patches by foot_length and foot_width
    std::vector<Point_2> vertices_2d_shrinked;
    for (const auto& vertex : vertices_2d) {
        double new_x = vertex.x();
        double new_y = vertex.y();
        // Example logic: shrink left/right edges along x, top/bottom along y
        if (vertex.x() > 0) new_x -= foot_length/2.0;
        if (vertex.x() < 0) new_x += foot_length/2.0;
        if (vertex.y() > 0) new_y -= foot_width/2.0;
        if (vertex.y() < 0) new_y += foot_width/2.0;
        vertices_2d_shrinked.emplace_back(new_x, new_y);
    }

    // Sort with convex hull
    CGAL::convex_hull_2(vertices_2d_shrinked.begin(), vertices_2d_shrinked.end(), std::back_inserter(polygon_2d));

    // Update vertices_2d with sorted vertices from polygon_2d
    vertices_2d.clear();  // Clear the unsorted vertices
    for (auto it = polygon_2d.vertices_begin(); it != polygon_2d.vertices_end(); ++it) {
        vertices_2d.push_back(Point_2(it->x(), it->y()));
    }

    // Convert back to 3D
    vertices_3d = transform_2d_points_to_world(vertices_2d, transform_to_3d);

    // Create 3D polyhedron
    CGAL::convex_hull_3(vertices_3d.begin(), vertices_3d.end(), polyhedron_3d);

    // Print Surface Information
    std::cout << "\n- Surface Information " << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  - Surface ID: " << surface_id << std::endl;
    std::cout << "  - Plane: " << plane << std::endl;
    std::cout << "  - Norm: " << norm << std::endl;
    std::cout << "  - Centroid: " << centroid << std::endl;
    std::cout << "  - Transform Matrix:" << std::endl;
    std::cout << "    [" << std::setw(10) << transform_to_3d.m(0,0) << " " << std::setw(10) << transform_to_3d.m(0,1) << " " << std::setw(10) << transform_to_3d.m(0,2) << " " << std::setw(10) << transform_to_3d.m(0,3) << "]" << std::endl;
    std::cout << "    [" << std::setw(10) << transform_to_3d.m(1,0) << " " << std::setw(10) << transform_to_3d.m(1,1) << " " << std::setw(10) << transform_to_3d.m(1,2) << " " << std::setw(10) << transform_to_3d.m(1,3) << "]" << std::endl;
    std::cout << "    [" << std::setw(10) << transform_to_3d.m(2,0) << " " << std::setw(10) << transform_to_3d.m(2,1) << " " << std::setw(10) << transform_to_3d.m(2,2) << " " << std::setw(10) << transform_to_3d.m(2,3) << "]" << std::endl;
    std::cout << "    [" << std::setw(10) << 0.0 << " " << std::setw(10) << 0.0 << " " << std::setw(10) << 0.0 << " " << std::setw(10) << 1.0 << "]" << std::endl;
    std::cout << "  - 3D Vertices:" << std::endl;
    for (const auto& vertex : vertices_3d) {
        std::cout << "    (" << std::setw(8) << vertex.x() << ", " 
                             << std::setw(8) << vertex.y() << ", " 
                             << std::setw(8) << vertex.z() << ")" << std::endl;
    }
}

void Surface::establish_surface_coordinate_system(const std::vector<Point_3>& points) {
    // 1. Find the point furthest from centroid for robust X-axis

    // World axes
    Vector_3 world_x(1, 0, 0);
    Vector_3 world_y(0, 1, 0);

    // Project world_x onto the plane
    Vector_3 x_axis = world_x - (world_x * norm) * norm;
    x_axis = x_axis / std::sqrt(x_axis.squared_length());

    // Project world_y onto the plane
    Vector_3 y_axis = world_y - (world_y * norm) * norm;
    y_axis = y_axis - (x_axis * y_axis) * x_axis; // Gram-Schmidt
    y_axis = y_axis / std::sqrt(y_axis.squared_length());
    
    // Create transformation matrix from the surface coordinates to the world
    transform_to_3d = Transformation(
        x_axis.x(), y_axis.x(), norm.x(), centroid.x(),
        x_axis.y(), y_axis.y(), norm.y(), centroid.y(),
        x_axis.z(), y_axis.z(), norm.z(), centroid.z()
    );

    // Cache the inverse transformation (from world to surface)
    transform_to_surface = transform_to_3d.inverse();
}

} // namespace nas