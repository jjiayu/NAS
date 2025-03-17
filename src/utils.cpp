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


typedef CGAL::Simple_cartesian<double> Kernel; // Using Simple Geometry Kernel
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Vector_3<Kernel> Vector_3;
typedef CGAL::Point_3<Kernel> Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;

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

void polytope_surf_intersection(const std::vector<Point_3>& surf_pts, const Polyhedron& polytope) {
    // Get Plane Parameters
    // Fit plane to all vertices
    Plane_3 plane;
    CGAL::linear_least_squares_fitting_3(
        surf_pts.begin(),
        surf_pts.end(),
        plane,
        CGAL::Dimension_tag<0>() // 0 for points
    );
    
    // Get the normal vector
    Vector_3 normal = plane.orthogonal_vector();
    
    // Normalize the normal vector
    double length = std::sqrt(normal.squared_length());
    normal = normal / length;
    
    // Calculate d (converting from ax + by + cz + d = 0 to n⋅p = d form)
    double d = -plane.d() / length;

    std::cout << "Plane Equation: " << plane << std::endl;
    std::cout << "Normal vector (n): " << normal << std::endl;
    std::cout << "d value: " << d << std::endl;

    // Store intersection points
    std::vector<Point_3> intersection_points;

    // Iterate through all edges of the polytope (using edges instead of halfedges)
    for (auto edge = polytope.edges_begin(); edge != polytope.edges_end(); ++edge) {
        // Get the vertices of the edge
        Point_3 p1 = edge->vertex()->point();
        Point_3 p2 = edge->opposite()->vertex()->point();

        // Create a segment from the edge
        Kernel::Segment_3 segment(p1, p2);

        // Compute intersection
        auto intersection = CGAL::intersection(plane, segment);
        
        // Check if intersection exists and is a point
        if (intersection) {
            Point_3 intersection_point;
            if (CGAL::assign(intersection_point, *intersection)) {
                intersection_points.push_back(intersection_point);
            }
        }
    }

    // Output results
    std::cout << "Found " << intersection_points.size() << " intersection points:" << std::endl;
    for (const auto& point : intersection_points) {
        std::cout << "Intersection point: " << point << std::endl;
    }

    if (!intersection_points.empty()) {
        // Find centroid of intersection points
        Point_3 centroid(0, 0, 0);
        for (const auto& p : intersection_points) {
            centroid = Point_3(centroid.x() + p.x(), 
                             centroid.y() + p.y(), 
                             centroid.z() + p.z());
        }
        centroid = Point_3(centroid.x() / intersection_points.size(),
                          centroid.y() / intersection_points.size(),
                          centroid.z() / intersection_points.size());

        // Create a coordinate system in the plane
        Vector_3 v1 = plane.base1();
        Vector_3 v2 = plane.base2();
        
        // Ensure v1 and v2 are normalized
        double v1_length = std::sqrt(v1.squared_length());
        double v2_length = std::sqrt(v2.squared_length());
        v1 = v1 / v1_length;
        v2 = v2 / v2_length;
        
        // Make sure v1 and v2 are perpendicular to the normal
        normal = plane.orthogonal_vector();
        normal = normal / std::sqrt(normal.squared_length());
        v2 = CGAL::cross_product(normal, v1); // Ensure perpendicular basis
        
        // Sort points based on their polar angle
        std::sort(intersection_points.begin(), intersection_points.end(),
            [&](const Point_3& p1, const Point_3& p2) {
                // Get vectors from centroid to points
                Vector_3 vec1(centroid, p1);
                Vector_3 vec2(centroid, p2);
                
                // Project vectors onto the plane
                double x1 = vec1 * v1;
                double y1 = vec1 * v2;
                double x2 = vec2 * v1;
                double y2 = vec2 * v2;
                
                // Compare polar angles using atan2
                double angle1 = std::atan2(y1, x1);
                double angle2 = std::atan2(y2, x2);
                
                // Handle the -π to π transition
                if (angle1 - angle2 > M_PI)
                    angle2 += 2 * M_PI;
                else if (angle2 - angle1 > M_PI)
                    angle1 += 2 * M_PI;
                    
                return angle1 < angle2;
            });

        // Verify the orientation
        double total_angle = 0.0;
        for (size_t i = 0; i < intersection_points.size(); ++i) {
            size_t j = (i + 1) % intersection_points.size();
            Vector_3 vec1(centroid, intersection_points[i]);
            Vector_3 vec2(centroid, intersection_points[j]);
            
            double x1 = vec1 * v1;
            double y1 = vec1 * v2;
            double x2 = vec2 * v1;
            double y2 = vec2 * v2;
            
            double angle = std::atan2(y2, x2) - std::atan2(y1, x1);
            if (angle < -M_PI) angle += 2 * M_PI;
            if (angle > M_PI) angle -= 2 * M_PI;
            total_angle += angle;
        }

        // If total angle is negative, reverse the order
        if (total_angle < 0) {
            std::reverse(intersection_points.begin() + 1, intersection_points.end());
        }
    }

    // Create a polygon from the sorted intersection points
    if (intersection_points.size() >= 3) {
        Polyhedron intersection_polygon;
        CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> builder(intersection_polygon.hds());
        
        // Start building with one face, n vertices
        builder.begin_surface(intersection_points.size(), 1);
        
        // Add vertices
        for (const auto& p : intersection_points) {
            builder.add_vertex(p);
        }
        
        // Add the face (one polygon using all vertices)
        builder.begin_facet();
        for (size_t i = 0; i < intersection_points.size(); ++i) {
            builder.add_vertex_to_facet(i);
        }
        builder.end_facet();
        
        // Finish building
        builder.end_surface();

        // Show the visualizations
        Visualizer::show_plane(plane);
        Visualizer::show_polyhedron(polytope);
        Visualizer::show_polyhedron(intersection_polygon);
    }

    // Output results
    std::cout << "Created polygon from " << intersection_points.size() << " intersection points" << std::endl;
}

} // namespace nas  