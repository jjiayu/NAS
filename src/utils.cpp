#include "utils.hpp"
#include "tree.hpp"
#include "constants.hpp"
#include "visualizer.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include <iostream>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/aff_transformation_tags.h>  // For TRANSLATION tag
#include <CGAL/convex_hull_3.h>  // Add this instead of minkowski_sum_3
#include <CGAL/linear_least_squares_fitting_3.h> // This wasn't enough
#include <CGAL/Linear_algebraCd.h>              // Need this
#include <CGAL/linear_least_squares_fitting_points_3.h> // And this specific header
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <boost/variant.hpp>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <libqhull_r/qhull_ra.h>  // Add QHULL header

typedef CGAL::Simple_cartesian<double> Kernel; // Using Simple Geometry Kernel
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Vector_3<Kernel> Vector_3;
typedef CGAL::Point_3<Kernel> Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Gps_segment_traits_2<Kernel> Traits_2;
typedef CGAL::General_polygon_set_2<Traits_2> Polygon_set_2;
typedef Traits_2::Polygon_2 General_polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;  // Add this line

namespace nas {

bool load_obj(const std::string& filename, Polyhedron& polyhedron) {
    if (!CGAL::IO::read_polygon_mesh(filename, polyhedron)) {
        std::cerr << "Failed to load polyhedron from: " << filename << std::endl;
        return false;
    }
    return true;
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
    // Get the normal vector and normalize it
    Vector_3 normal = plane.orthogonal_vector();
    normal = normal / std::sqrt(normal.squared_length());
    
    // Calculate centroid of surface points to use as origin
    Point_3 centroid(0, 0, 0);
    for (const auto& point : surf_pts) {
        centroid = Point_3(centroid.x() + point.x(),
                          centroid.y() + point.y(),
                          centroid.z() + point.z());
    }
    centroid = Point_3(centroid.x() / surf_pts.size(),
                      centroid.y() / surf_pts.size(),
                      centroid.z() / surf_pts.size());
    
    // Create basis vectors for the plane coordinate system
    // Use the first surface point to create x-axis (projected onto plane)
    Vector_3 first_point_vec = Vector_3(surf_pts[0].x() - centroid.x(),
                                      surf_pts[0].y() - centroid.y(),
                                      surf_pts[0].z() - centroid.z());
    // Project first point vector onto plane
    first_point_vec = first_point_vec - (first_point_vec * normal) * normal;
    first_point_vec = first_point_vec / std::sqrt(first_point_vec.squared_length());
    
    // Create y-axis using cross product
    Vector_3 y_axis = CGAL::cross_product(normal, first_point_vec);
    y_axis = y_axis / std::sqrt(y_axis.squared_length());
    
    // Create and return transformation matrix from world to plane coordinates
    return Transformation(
        first_point_vec.x(), y_axis.x(), normal.x(), centroid.x(),
        first_point_vec.y(), y_axis.y(), normal.y(), centroid.y(),
        first_point_vec.z(), y_axis.z(), normal.z(), centroid.z()
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

// Helper function to compute intersection between two line segments
bool compute_2d_edge_intersection(const Point_2& p1, const Point_2& p2,
                                const Point_2& p3, const Point_2& p4,
                                Point_2& intersection) {
    double denominator = (p1.x() - p2.x()) * (p3.y() - p4.y()) - 
                        (p1.y() - p2.y()) * (p3.x() - p4.x());
    
    if (std::abs(denominator) < 1e-10) return false;  // Lines are parallel or coincident
    
    double t = ((p1.x() - p3.x()) * (p3.y() - p4.y()) - 
                (p1.y() - p3.y()) * (p3.x() - p4.x())) / denominator;
    
    double u = -((p1.x() - p2.x()) * (p1.y() - p3.y()) - 
                 (p1.y() - p2.y()) * (p1.x() - p3.x())) / denominator;
    
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        intersection = Point_2(p1.x() + t * (p2.x() - p1.x()),
                             p1.y() + t * (p2.y() - p1.y()));
        return true;
    }
    return false;
}

// Function to compute intersection between two polygons in 2d
Polygon_2 compute_polygon_intersection(const Polygon_2& poly1, const Polygon_2& poly2) {
    std::vector<Point_2> intersection_points;
    
    // Step 1: Find vertices of poly1 that are inside poly2
    for (auto it = poly1.vertices_begin(); it != poly1.vertices_end(); ++it) {
        if (is_point_inside_polygon(*it, poly2)) {
            intersection_points.push_back(*it);
        }
    }
    
    // Step 2: Find vertices of poly2 that are inside poly1
    for (auto it = poly2.vertices_begin(); it != poly2.vertices_end(); ++it) {
        if (is_point_inside_polygon(*it, poly1)) {
            intersection_points.push_back(*it);
        }
    }
    
    // Step 3: Find intersection points between edges
    for (auto it1 = poly1.vertices_begin(); it1 != poly1.vertices_end(); ++it1) {
        auto next1 = std::next(it1);
        if (next1 == poly1.vertices_end()) next1 = poly1.vertices_begin();
        
        for (auto it2 = poly2.vertices_begin(); it2 != poly2.vertices_end(); ++it2) {
            auto next2 = std::next(it2);
            if (next2 == poly2.vertices_end()) next2 = poly2.vertices_begin();
            
            Point_2 intersection;
            if (compute_2d_edge_intersection(*it1, *next1, *it2, *next2, intersection)) {
                intersection_points.push_back(intersection);
            }
        }
    }
    
    // Step 4: Create and sort points using convex hull
    Polygon_2 result;
    if (intersection_points.size() >= 3) {
        // Use CGAL's convex hull to sort points counterclockwise
        CGAL::convex_hull_2(intersection_points.begin(), intersection_points.end(), std::back_inserter(result));
    } else {
        // If we have less than 3 points, just add them to the result
        for (const auto& p : intersection_points) {
            result.push_back(p);
        }
    }
    
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
    
    // Find intersection points
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
    Polygon_2 intersection_result = compute_polygon_intersection(surf_polygon_2d, intersection_polygon_2d);
    
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

} // namespace nas