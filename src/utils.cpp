#include "utils.hpp"
#include "tree.hpp"
#include "constants.hpp"
#include "visualizer.hpp"
#include "types.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <fstream>
#include <iostream>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron) {
    if (!CGAL::IO::read_polygon_mesh(filename, polyhedron)) {
        throw std::runtime_error("Failed to load polyhedron from: " + filename);
    }
    std::cout << "Successfully loaded polytope from: " << filename << std::endl;
}

std::vector<Polyhedron> convert_surf_pts_to_polyhedron(std::vector<std::vector<Point_3>> surface_list) {
    std::vector<Polyhedron> polyhedra; // Changed to store polyhedra
    
    for (const auto& surface : surface_list) {
        // Get four points from the surface
        Point_3 p1(surface[0][0], surface[0][1], surface[0][2]); // bottom left
        Point_3 p2(surface[1][0], surface[1][1], surface[1][2]); // bottom right
        Point_3 p3(surface[2][0], surface[2][1], surface[2][2]); // top right
        Point_3 p4(surface[3][0], surface[3][1], surface[3][2]); // top left
        
        // Create a polyhedron from the four points
        Polyhedron polyhedron;
        polyhedron.make_tetrahedron(p1, p2, p3, p4); // Assuming a tetrahedron for the surface
        polyhedra.push_back(polyhedron);
    }
    
    return polyhedra; // Return the vector of polyhedra
}

Vector_3 get_centroid(const Polyhedron& polyhedron) {
    Vector_3 centroid_vector(0.0, 0.0, 0.0); // Initialize a Vector_3 for the centroid
    int vertex_count = 0;

    for (auto v = polyhedron.vertices_begin(); v != polyhedron.vertices_end(); ++v) {
        const Point_3& point = v->point();
        centroid_vector = centroid_vector + Vector_3(point.x(), point.y(), point.z()); // Accumulate the vector
        vertex_count++;
    }

    // Calculate the centroid as a Vector_3
    if (vertex_count > 0) {
        return Vector_3(centroid_vector.x() / vertex_count, 
                        centroid_vector.y() / vertex_count, 
                        centroid_vector.z() / vertex_count);
    } else {
        // Handle the case where there are no vertices
        return Vector_3(0.0, 0.0, 0.0); // Or throw an exception, or handle as needed
    }
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


Transformation create_transfomation_to_surface_center(const std::vector<Point_3>& surf_pts, const Plane_3& plane) {
    // 1. Get and normalize the normal vector (Z-axis)
    Vector_3 normal = plane.orthogonal_vector();
    normal = normal / std::sqrt(normal.squared_length());
    
    // 2. Calculate centroid as origin
    Point_3 centroid(0, 0, 0);
    for (const auto& point : surf_pts) {
        centroid = Point_3(centroid.x() + point.x(),
                          centroid.y() + point.y(),
                          centroid.z() + point.z());
    }
    centroid = Point_3(centroid.x() / surf_pts.size(),
                      centroid.y() / surf_pts.size(),
                      centroid.z() / surf_pts.size());
    
    // 3. Find the point furthest from centroid for robust X-axis
    double max_dist = 0;
    Vector_3 x_axis_candidate;
    for (const auto& point : surf_pts) {
        Vector_3 vec(point.x() - centroid.x(),
                    point.y() - centroid.y(),
                    point.z() - centroid.z());
        // Project vector onto plane
        Vector_3 projected = vec - (vec * normal) * normal;
        double dist = projected.squared_length();
        if (dist > max_dist) {
            max_dist = dist;
            x_axis_candidate = projected;
        }
    }
    
    // 4. Normalize X-axis
    Vector_3 x_axis = x_axis_candidate / std::sqrt(x_axis_candidate.squared_length());
    
    // 5. Create Y-axis using cross product (ensures right-handed system)
    Vector_3 y_axis = CGAL::cross_product(normal, x_axis);
    y_axis = y_axis / std::sqrt(y_axis.squared_length());
    
    // 6. Double check X-axis is perpendicular (optional but safer)
    x_axis = CGAL::cross_product(y_axis, normal);
    x_axis = x_axis / std::sqrt(x_axis.squared_length());
    
    // Create transformation matrix from world to surface coordinates
    return Transformation(
        x_axis.x(), y_axis.x(), normal.x(), centroid.x(),
        x_axis.y(), y_axis.y(), normal.y(), centroid.y(),
        x_axis.z(), y_axis.z(), normal.z(), centroid.z()
    );
}

// Helper function to check if a point is inside a polygon
bool is_point_inside_polygon(const Point_2& point, const Polygon_2& polygon) {
    bool inside = false;
    auto j = polygon.vertices_end() - 1;
    
    for (auto i = polygon.vertices_begin(); i != polygon.vertices_end(); ++i) {
        if (((i->y() > point.y()) != (j->y() > point.y())) &&
            (point.x() < (j->x() - i->x()) * (point.y() - i->y()) / (j->y() - i->y()) + i->x())) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

// Helper function to compute intersection between two line segments using CGAL
bool compute_2d_edge_intersection(const Point_2& p1, const Point_2& p2,
                                const Point_2& p3, const Point_2& p4,
                                Point_2& intersection) {
    Kernel::Segment_2 segment1(p1, p2);
    Kernel::Segment_2 segment2(p3, p4);
    
    auto result = CGAL::intersection(segment1, segment2);
    
    if (result) {
        try {
            if (const Point_2* p = std::get_if<Point_2>(&*result)) {
                // Intersection is a point
                intersection = *p;
                return true;
            }
            else if (const Kernel::Segment_2* s = std::get_if<Kernel::Segment_2>(&*result)) {
                // Segments overlap, take midpoint
                intersection = CGAL::midpoint(s->source(), s->target());
                return true;
            }
        } catch (const std::bad_variant_access&) {
            // Handle variant access error
            return false;
        }
    }
    return false;
}

// Helper function to check if point is inside edge
bool is_point_inside_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end) {
    return ((edge_end.x() - edge_start.x()) * (point.y() - edge_start.y()) -
            (edge_end.y() - edge_start.y()) * (point.x() - edge_start.x())) >= 0;
}

// Helper function to get intersection point
Point_2 compute_intersection(const Point_2& prev_point, const Point_2& current_point,
                           const Point_2& edge_start, const Point_2& edge_end) {
    Point_2 intersection;
    compute_2d_edge_intersection(prev_point, current_point, edge_start, edge_end, intersection);
    return intersection;
}

// Function to compute intersection between two polygons in 2d using Sutherland-Hodgman
Polygon_2 compute_polygon_intersection(const Polygon_2& intersection_polygon, const Polygon_2& surf_polygon) {
    if (intersection_polygon.is_empty() || surf_polygon.is_empty()) {
        std::cout << "One of the input polygons is empty" << std::endl;
        return Polygon_2();
    }

    std::cout << "\nStarting Sutherland-Hodgman clipping:" << std::endl;
    std::cout << "Subject polygon vertices: " << intersection_polygon.size() << std::endl;
    std::cout << "Clip polygon vertices: " << surf_polygon.size() << std::endl;

    // List outputList = subjectPolygon
    std::vector<Point_2> output_list;
    for (auto it = intersection_polygon.vertices_begin(); it != intersection_polygon.vertices_end(); ++it) {
        output_list.push_back(*it);
    }

    // for (Edge clipEdge in clipPolygon) do
    auto clip_end = surf_polygon.vertices_end();
    int edge_count = 0;
    for (auto clip_it = surf_polygon.vertices_begin(); clip_it != clip_end; ++clip_it) {
        if (output_list.empty()) {
            std::cout << "Output list became empty after edge " << edge_count << std::endl;
            return Polygon_2();
        }

        // Get clip edge
        Point_2 edge_start = *clip_it;
        Point_2 edge_end = (std::next(clip_it) == clip_end) ? 
                          *surf_polygon.vertices_begin() : *std::next(clip_it);

        std::cout << "\nProcessing clip edge " << edge_count << ": (" 
                  << edge_start.x() << "," << edge_start.y() << ") -> ("
                  << edge_end.x() << "," << edge_end.y() << ")" << std::endl;

        // List inputList = outputList
        std::vector<Point_2> input_list = output_list;
        output_list.clear();

        // for (int i = 0; i < inputList.count; i += 1) do
        for (size_t i = 0; i < input_list.size(); i++) {
            Point_2 current_point = input_list[i];
            Point_2 prev_point = input_list[(i + input_list.size() - 1) % input_list.size()];

            bool current_inside = is_point_inside_edge(current_point, edge_start, edge_end);
            bool prev_inside = is_point_inside_edge(prev_point, edge_start, edge_end);

            std::cout << "  Point " << i << " (" << current_point.x() << "," << current_point.y() 
                      << ") inside: " << current_inside << std::endl;

            if (current_inside) {
                if (!prev_inside) {
                    Point_2 intersecting_point = compute_intersection(prev_point, current_point, edge_start, edge_end);
                    output_list.push_back(intersecting_point);
                    std::cout << "    Added intersection point: (" << intersecting_point.x() 
                              << "," << intersecting_point.y() << ")" << std::endl;
                }
                output_list.push_back(current_point);
                std::cout << "    Added current point" << std::endl;
            }
            else if (prev_inside) {
                Point_2 intersecting_point = compute_intersection(prev_point, current_point, edge_start, edge_end);
                output_list.push_back(intersecting_point);
                std::cout << "    Added intersection point: (" << intersecting_point.x() 
                          << "," << intersecting_point.y() << ")" << std::endl;
            }
        }

        std::cout << "Points after edge " << edge_count << ": " << output_list.size() << std::endl;
        edge_count++;
    }

    // Create final polygon
    Polygon_2 result;
    for (const auto& point : output_list) {
        result.push_back(point);
    }

    std::cout << "Final polygon vertices: " << result.size() << std::endl;
    return result;
}

void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope) {
    // Fit plane to surface points
    Plane_3 plane;
    CGAL::linear_least_squares_fitting_3(
        surf_pts.begin(),
        surf_pts.end(),
        plane,
        CGAL::Dimension_tag<0>() // 0 for points
    );
    
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
    
    // Visualize results
    if (intersection_points.size() >= 3) {
        Polyhedron intersection_polygon;
        CGAL::convex_hull_3(intersection_points.begin(), intersection_points.end(), intersection_polygon);

        std::cout << intersection_polygon.is_valid() << std::endl;

        Visualizer::show_scene(plane, polytope, intersection_polygon);
        Visualizer::show_polyhedron(intersection_polygon);
    }

    // Create transformation matrix to transform points to surface plane
    Transformation transform = create_transfomation_to_surface_center(surf_pts, plane);

    // Transform surface points to plane coordinates
    std::vector<Point_2> surf_pts_2d;
    for (const auto& point : surf_pts) {
        Point_3 transformed = transform(point);
        surf_pts_2d.push_back(Point_2(transformed.x(), transformed.y()));
    }

    // Transform intersection points to plane coordinates
    std::vector<Point_2> intersection_pts_2d;
    for (const auto& point : intersection_points) {
        Point_3 transformed = transform(point);
        intersection_pts_2d.push_back(Point_2(transformed.x(), transformed.y()));
    }

    // Create 2D polygons directly using convex hull
    Polygon_2 surf_polygon_2d;
    CGAL::convex_hull_2(surf_pts_2d.begin(), surf_pts_2d.end(), std::back_inserter(surf_polygon_2d));

    Polygon_2 intersection_polygon_2d;
    CGAL::convex_hull_2(intersection_pts_2d.begin(), intersection_pts_2d.end(), std::back_inserter(intersection_polygon_2d));

    // Print points for visualization
    std::cout << "\nSurface Points (2D):" << std::endl;
    for (auto it = surf_polygon_2d.vertices_begin(); it != surf_polygon_2d.vertices_end(); ++it) {
        std::cout << "(" << it->x() << ", " << it->y() << ")" << std::endl;
    }

    std::cout << "\nIntersection Points (2D):" << std::endl;
    for (auto it = intersection_polygon_2d.vertices_begin(); it != intersection_polygon_2d.vertices_end(); ++it) {
        std::cout << "(" << it->x() << ", " << it->y() << ")" << std::endl;
    }

    // Visualize 2D polygons
    std::cout << "\nSurface Polygon:" << std::endl;
    std::cout << "Is simple: " << surf_polygon_2d.is_simple() << std::endl;
    std::cout << "Is convex: " << surf_polygon_2d.is_convex() << std::endl;
    std::cout << "Area: " << surf_polygon_2d.area() << std::endl;

    std::cout << "\nIntersection Polygon:" << std::endl;
    std::cout << "Is simple: " << intersection_polygon_2d.is_simple() << std::endl;
    std::cout << "Is convex: " << intersection_polygon_2d.is_convex() << std::endl;
    std::cout << "Area: " << intersection_polygon_2d.area() << std::endl;

    // Compute intersection of 2D polygons
    Polygon_2 intersection_result = compute_polygon_intersection(intersection_polygon_2d, surf_polygon_2d);
    
    // Print intersection result
    std::cout << "\nIntersection Result:" << std::endl;
    std::cout << "Number of vertices: " << std::distance(intersection_result.vertices_begin(), 
                                                        intersection_result.vertices_end()) << std::endl;
    std::cout << "Area: " << intersection_result.area() << std::endl;
    
    // Visualize the 2D result
    Visualizer::plot_2d_points_and_polygons(surf_polygon_2d, intersection_result);

    // Convert intersection polygon back to 3D
    std::vector<Point_3> intersection_3d_points;
    std::vector<Point_3> surface_3d_points;
    Transformation inverse_transform = transform.inverse();
    
    std::cout << "\nIntersection Points (3D):" << std::endl;
    for (auto it = intersection_result.vertices_begin(); it != intersection_result.vertices_end(); ++it) {
        // Convert 2D point to 3D (z=0 since it's on the plane)
        Point_3 point_3d(it->x(), it->y(), 0);
        // Apply inverse transformation
        Point_3 transformed_point = inverse_transform(point_3d);
        intersection_3d_points.push_back(transformed_point);
        std::cout << "(" << transformed_point.x() << ", " 
                  << transformed_point.y() << ", " 
                  << transformed_point.z() << ")" << std::endl;
    }

    std::cout << "\nSurface Points (3D):" << std::endl;
    for (auto it = surf_polygon_2d.vertices_begin(); it != surf_polygon_2d.vertices_end(); ++it) {
        // Convert 2D point to 3D (z=0 since it's on the plane)
        Point_3 point_3d(it->x(), it->y(), 0);
        // Apply inverse transformation
        Point_3 transformed_point = inverse_transform(point_3d);
        surface_3d_points.push_back(transformed_point);
        std::cout << "(" << transformed_point.x() << ", " 
                  << transformed_point.y() << ", " 
                  << transformed_point.z() << ")" << std::endl;
    }

    // Create 3D polygons from the points
    if (surface_3d_points.size() >= 3 && intersection_3d_points.size() >= 3) {
        Polyhedron surface_3d;
        Polyhedron intersection_3d;
        CGAL::convex_hull_3(surface_3d_points.begin(), surface_3d_points.end(), surface_3d);
        CGAL::convex_hull_3(intersection_3d_points.begin(), intersection_3d_points.end(), intersection_3d);
        
        // Visualize all components
        std::cout << "\nVisualizing 3D scene:" << std::endl;
        std::cout << "1. Original polytope and surface polygon" << std::endl;
        Visualizer::show_scene(plane, polytope, surface_3d);
        
        std::cout << "\n2. Original polytope and intersection result" << std::endl;
        Visualizer::show_scene(plane, polytope, intersection_3d);
        
        // Show detailed views of each component
        std::cout << "\nDetailed views:" << std::endl;
        std::cout << "Surface polygon:" << std::endl;
        Visualizer::show_polyhedron(surface_3d);
        std::cout << "Intersection polygon:" << std::endl;
        Visualizer::show_polyhedron(intersection_3d);
    }
}

Vector_3 compute_centroid(const std::vector<Point_3>& points) {
    if (points.empty()) {
        return Vector_3(0.0, 0.0, 0.0);  // Return zero vector for empty input
    }

    Vector_3 centroid(0.0, 0.0, 0.0);
    for (const Point_3& point : points) {
        centroid = centroid + Vector_3(point.x(), point.y(), point.z());
    }

    return Vector_3(
        centroid.x() / points.size(),
        centroid.y() / points.size(),
        centroid.z() / points.size()
    );
}

} // namespace nas