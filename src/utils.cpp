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

// Add helper function for QHULL intersection
std::vector<Point_2> compute_intersection_qhull(const std::vector<Point_2>& points1, 
                                              const std::vector<Point_2>& points2) {
    std::vector<Point_2> result;
    
    // Combine points from both polygons
    std::vector<double> points;
    // First add points from the intersection polygon
    for (const auto& p : points1) {
        points.push_back(CGAL::to_double(p.x()));
        points.push_back(CGAL::to_double(p.y()));
    }
    // Then add points from the surface polygon
    for (const auto& p : points2) {
        points.push_back(CGAL::to_double(p.x()));
        points.push_back(CGAL::to_double(p.y()));
    }

    if (points.empty() || points.size() < 4) return result;  // Need at least 2 points

    // Initialize QHULL
    qhT qh_qh;
    qhT* qh = &qh_qh;
    
    // Initialize qhull with stdout/stderr redirected to nullptr to suppress output
    qh_init_A(qh, nullptr, nullptr, nullptr, 0, nullptr);
    
    // Set QHULL options:
    // v - Voronoi diagram (not used, but needed for 2D)
    // Qt - triangulate output
    // Qbb - scale input to unit cube
    // Qc - keep coplanar points
    char* options = const_cast<char*>("qhull v Qt Qbb Qc");
    int exitcode = qh_new_qhull(qh, 2, points.size()/2, points.data(), false,
                               options, nullptr, nullptr);
    
    if (!exitcode) {
        // Get vertices from QHULL
        vertexT* vertex = qh->vertex_list;
        while (vertex) {  // Changed condition to catch all vertices
            double* point = vertex->point;
            result.push_back(Point_2(point[0], point[1]));
            vertex = vertex->next;
        }
        
        // Sort points in counter-clockwise order
        if (result.size() >= 3) {
            Point_2 centroid(0, 0);
            for (const auto& p : result) {
                centroid = Point_2(centroid.x() + p.x(), centroid.y() + p.y());
            }
            centroid = Point_2(centroid.x() / result.size(), centroid.y() / result.size());
            
            std::sort(result.begin(), result.end(),
                [&centroid](const Point_2& a, const Point_2& b) {
                    return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
                           std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
                });
        }
    }
    
    // Clean up QHULL
    qh_freeqhull(qh, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(qh, &curlong, &totlong);
    
    return result;
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

    Visualizer::plot_2d_points_and_polygons(surf_polygon_2d, intersection_polygon_2d);

}

} // namespace nas