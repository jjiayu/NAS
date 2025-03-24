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

Vector_3 get_centroid(const std::vector<Point_3>& points) {
    if (points.empty()) {
        return Vector_3(0, 0, 0);
    }

    Vector_3 sum(0, 0, 0);
    for (const auto& point : points) {
        sum = sum + (point - CGAL::ORIGIN);
    }
    return sum / static_cast<double>(points.size());
}

Polyhedron minkowski_sum(const std::vector<Vector_3>& patch_vertices, 
                         const Polyhedron& polytope) {
    // Store all vertices of the transformed polytopes
    std::vector<Point_3> all_vertices;

    // Build the list of all transformed polytopes and collect vertices
    for (size_t i = 0; i < patch_vertices.size(); ++i) {
        Transformation translation(CGAL::TRANSLATION, patch_vertices[i]);
        
        // Transform each vertex of the polytope and collect them
        for (auto v = polytope.vertices_begin(); v != polytope.vertices_end(); ++v) {
            Point_3 transformed_point = translation(v->point());
            all_vertices.push_back(transformed_point);
        }
    }

    // Compute convex hull of all vertices
    Polyhedron P_union;
    CGAL::convex_hull_3(all_vertices.begin(), all_vertices.end(), P_union);
    
    return P_union;
}

} // namespace nas