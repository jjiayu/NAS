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

Point_3 get_centroid(const std::vector<Point_3>& points) {
    if (points.empty()) {
        return Point_3(0, 0, 0);
    }

    Vector_3 sum(0, 0, 0);
    for (const auto& point : points) {
        sum = sum + (point - CGAL::ORIGIN);  // Convert Point_3 to Vector_3 for addition
    }
    return CGAL::ORIGIN + (sum / static_cast<double>(points.size()));  // Convert back to Point_3
}

Polyhedron minkowski_sum(const std::vector<Point_3>& patch_vertices, 
                         const Polyhedron& polytope) {
    // Store all vertices of the transformed polytopes
    std::vector<Point_3> all_vertices;

    // Build the list of all transformed polytopes and collect vertices
    for (size_t i = 0; i < patch_vertices.size(); ++i) {
        // Convert Point_3 to Vector_3 for translation
        Transformation translation(CGAL::TRANSLATION, patch_vertices[i] - CGAL::ORIGIN);
        
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

std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope){
    // Find intersection points with the plane
    std::vector<Point_3> intersection_points;
    for (auto edge = polytope.edges_begin(); edge != polytope.edges_end(); ++edge) {
        Point_3 p1 = edge->vertex()->point();
        Point_3 p2 = edge->opposite()->vertex()->point();
        Kernel::Segment_3 segment(p1, p2);

        auto intersection = CGAL::intersection(plane, segment);
        if (intersection) {
            Point_3 intersection_point;
            if (CGAL::assign(intersection_point, *intersection)) {
                intersection_points.push_back(intersection_point);
            }
        }
    }
    return intersection_points;
}

double is_leftside_of_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end){
    return ((edge_end.x() - edge_start.x()) * (point.y() - edge_start.y()) -
            (edge_end.y() - edge_start.y()) * (point.x() - edge_start.x()));
}


// Compute the intersection between two 2d polygons using Sutherland-Hodgman algorithm
// Subject polygon is the interseciton result between the polytope and the plans in 2d), clipping polygon is the surface polygon in 2d
std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& subject_polygon, const std::vector<Point_2>& clip_polygon) {
    
    if (subject_polygon.empty() || clip_polygon.empty()) {
        std::cout << "One of the input polygons is empty" << std::endl;
        return std::vector<Point_2>();
    }

    // Initialize outputList to subjectPlygon
    std::vector<Point_2> output_list = subject_polygon;

    // for (Edge clipEdge in clipPolygon) do
    auto clip_end = clip_polygon.end();
    for (auto clip_it = clip_polygon.begin(); clip_it != clip_end; ++clip_it) {
        if (output_list.empty()) {
            return std::vector<Point_2>();
        }

        // Get clip edge
        Point_2 edge_start = *clip_it;
        Point_2 edge_end = (std::next(clip_it) == clip_end) ? 
                            clip_polygon.front() : *std::next(clip_it);

        // List inputList = outputList
        std::vector<Point_2> input_list = output_list;
        output_list.clear();

        for (size_t i = 0; i < input_list.size(); i++) {
            Point_2 current_point = input_list[i];
            Point_2 prev_point = input_list[(i + input_list.size() - 1) % input_list.size()];//if current point is the first point, prev point is the last point
            
            // Create segments for intersection check
            Segment_2 edge(edge_start, edge_end);
            Segment_2 line(prev_point, current_point);

            bool current_inside = is_leftside_of_edge(current_point, edge_start, edge_end) >= 0;
            bool prev_inside = is_leftside_of_edge(prev_point, edge_start, edge_end) >= 0;

            if (current_inside) {
                if (!prev_inside) {
                    auto result = CGAL::intersection(edge, line);
                    if (result) {
                        Point_2 intersection_point;
                        if (CGAL::assign(intersection_point, *result)) {
                            output_list.push_back(intersection_point);
                        }
                    }
                }
                output_list.push_back(current_point);
            }
            else if (prev_inside) {
                auto result = CGAL::intersection(edge, line);
                if (result) {
                    Point_2 intersection_point;
                    if (CGAL::assign(intersection_point, *result)) {
                        output_list.push_back(intersection_point);
                    }
                }
            }
        }
    }
    return output_list;
}

bool is_point_in_polygon(const Point_3& point, const std::vector<Point_3>& polygon, const Vector_3& normal) {
    return true;
}


} // namespace nas