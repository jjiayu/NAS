#include "types.hpp"
#include "geometry.hpp"
#include "node.hpp"
#include <limits>
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
            Line_2 line(edge_start, edge_end);
            Segment_2 edge(prev_point, current_point);

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

    // Add surface corners to the intersection result (not used for now)
    // for (const auto& clip_vertex : clip_polygon) {
    // if (CGAL::bounded_side_2(subject_polygon.begin(), subject_polygon.end(), clip_vertex, Kernel()) == CGAL::ON_BOUNDED_SIDE ||
    //     CGAL::bounded_side_2(subject_polygon.begin(), subject_polygon.end(), clip_vertex, Kernel()) == CGAL::ON_BOUNDARY) {
    //     // Insert the clip_vertex into the intersection_result at the best place
    //     // For simplicity, you can just add it (optionally, you can insert it after the closest edge)
    //         output_list.push_back(clip_vertex);
    //     }
    // }

    // // Remove duplicates if needed (Segmentation fault with Convex hull will need this)
    // // Sort points first to group duplicates together
    // std::sort(output_list.begin(), output_list.end(), [](const Point_2& a, const Point_2& b) {
    //     if (a.x() != b.x()) return a.x() < b.x();
    //     return a.y() < b.y();
    // });

    // // Now std::unique will work because duplicates are consecutive
    // auto end = std::unique(output_list.begin(), output_list.end());
    // output_list.erase(end, output_list.end());

    return output_list;
}

double compute_polygon_perimeter(const Polyhedron& polyhedron){
    double perimeter = 0.0;
    for (auto edge = polyhedron.edges_begin(); edge != polyhedron.edges_end(); ++edge) {
        // No need to check vertices as they are guaranteed to exist in a valid polyhedron
        perimeter += CGAL::sqrt(CGAL::squared_distance(edge->vertex()->point(), 
                                                     edge->opposite()->vertex()->point()));
    }
    return perimeter;
}

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location){
    return CGAL::sqrt(CGAL::squared_distance(start_location, end_location));
}

// Convert half-space polytope constraint to H-representation
HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope){
    HalfSpacePolytopeConstraint constraint;
    
    // Calculate centroid for normal orientation
    Point_3 centroid = std::accumulate(
        polytope.vertices_begin(), polytope.vertices_end(), Point_3(0, 0, 0),
        [](const Point_3& acc, const auto& vertex) {
            return Point_3(acc.x() + vertex.point().x(),
                         acc.y() + vertex.point().y(),
                         acc.z() + vertex.point().z());
        });
    
    int vertex_count = std::distance(polytope.vertices_begin(), polytope.vertices_end());
    if (vertex_count == 0) {
        //exception handling
        throw std::runtime_error("Error: Empty polytope");
    }
    
    centroid = Point_3(centroid.x() / vertex_count,
                      centroid.y() / vertex_count,
                      centroid.z() / vertex_count);
    
    // Each facet represents one half-space constraint
    int num_facets = polytope.size_of_facets();
    constraint.A = Eigen::MatrixXd::Zero(num_facets, 3);
    constraint.b = Eigen::VectorXd::Zero(num_facets);
    
    int facet_index = 0;
    for (auto facet = polytope.facets_begin(); facet != polytope.facets_end(); ++facet) {
        // Get three points defining the facet
        auto h = facet->facet_begin();
        const Point_3& p1 = h->vertex()->point();
        const Point_3& p2 = (++h)->vertex()->point();
        const Point_3& p3 = (++h)->vertex()->point();
        
        // Create CGAL plane from three points
        Plane_3 plane(p1, p2, p3);
        
        // Check for degenerate plane
        if (plane.is_degenerate()) {
            std::cout << "\033[1;33mWarning: Degenerate facet detected, skipping...\033[0m" << std::endl;
            continue;
        }
        
        // Get plane coefficients: ax + by + cz + d = 0
        double a = CGAL::to_double(plane.a());
        double b = CGAL::to_double(plane.b());
        double c = CGAL::to_double(plane.c());
        double d = CGAL::to_double(plane.d());
        
        // Normalize the coefficients
        double norm = std::sqrt(a*a + b*b + c*c);
        if (norm <= 1e-12) {
            std::cout << "\033[1;33mWarning: Zero normal vector detected, skipping...\033[0m" << std::endl;
            continue;
        }
        a /= norm;
        b /= norm;
        c /= norm;
        d /= norm;
        
        // Check orientation: for half-space Ax <= b, we want normal pointing inward
        // CGAL plane equation: ax + by + cz + d = 0
        // For half-space: ax + by + cz <= -d (interior points satisfy this)
        double centroid_side = a * CGAL::to_double(centroid.x()) + 
                              b * CGAL::to_double(centroid.y()) + 
                              c * CGAL::to_double(centroid.z()) + d;
        
        // If centroid_side > 0, the normal points away from centroid (outward)
        // We want inward normals, so flip if needed
        if (centroid_side > 0) {
            a = -a; b = -b; c = -c; d = -d;
        }
        
        // Store in constraint matrices: Ax <= b format
        // From ax + by + cz + d = 0, we get ax + by + cz <= -d
        constraint.A(facet_index, 0) = a;
        constraint.A(facet_index, 1) = b;
        constraint.A(facet_index, 2) = c;
        constraint.b(facet_index) = -d;  // Note the negative sign!
        
        facet_index++;
    }
    
    // Resize matrices in case some facets were skipped due to degeneracy
    if (facet_index < num_facets) {
        constraint.A.conservativeResize(facet_index, 3);
        constraint.b.conservativeResize(facet_index);
    }
    
    std::cout << "- Successfully converted polytope to half-space representation with " 
              << facet_index << " constraints" << std::endl;
    

    return constraint;
}

// Convert surface constraint to H-representation with plane equality and boundary inequalities
// The first row is equality constraint defines the plane
// Other rows are inequality constraints that define the boundary of the surface
SurfaceConstraint generate_surface_constraint(const Polyhedron& surface_3d){
    // std::cout << "- Converting surface to H-representation with plane equality and boundary constraints" << std::endl;
    SurfaceConstraint constraint;

    // PART 1: Get the plane equation of the surface (equality constraint)
    // First, extract all vertices from the polyhedron
    std::vector<Point_3> vertices;
    for (auto v = surface_3d.vertices_begin(); v != surface_3d.vertices_end(); ++v) {
        vertices.push_back(v->point());
    }
    
    if (vertices.size() < 3) {
        throw std::runtime_error("Error: Need at least 3 vertices to fit a plane");
    }
    
    // Fit a plane to the vertices using least squares
    Plane_3 plane;
    CGAL::linear_least_squares_fitting_3(vertices.begin(), vertices.end(), plane, CGAL::Dimension_tag<0>());

    // Get the plane coefficients
    double a = CGAL::to_double(plane.a());
    double b = CGAL::to_double(plane.b());
    double c = CGAL::to_double(plane.c());
    double d = CGAL::to_double(plane.d());

    // Normalize the coefficients
    double norm = std::sqrt(a*a + b*b + c*c);
    if (norm <= 1e-12) {
        std::cout << "\033[1;33mWarning: Zero normal vector detected, skipping...\033[0m" << std::endl;
        return SurfaceConstraint();
    }
    a /= norm;
    b /= norm;
    c /= norm;
    d /= norm;
    
    // PART 2: Get boundary constraints from surface vertices (vertical half-space planes in 3D)
    int num_vertices = vertices.size();
    if (num_vertices < 3) {
        throw std::runtime_error("Error: Surface has fewer than 3 vertices");
    }
    
    // PART 3: Create half-space constraints from edges
    // For a closed polygon: num_edges = num_vertices
    // Total: 1 equality constraint (plane) + num_vertices inequality constraints (edge boundaries)
    constraint.A = Eigen::MatrixXd::Zero(1 + num_vertices, 3);
    constraint.b = Eigen::VectorXd::Zero(1 + num_vertices);
    
    // Row 0: Plane constraint (equality: ax + by + cz = -d)
    constraint.A(0, 0) = a;
    constraint.A(0, 1) = b;
    constraint.A(0, 2) = c;
    constraint.b(0) = -d;
    
    // Rows 1 to num_vertices: Vertical boundary constraints from each edge
    for (int i = 0; i < num_vertices; i++) {
        Point_3 p1 = vertices[i];
        Point_3 p2 = vertices[(i + 1) % num_vertices];
        
        // Edge vector in x-y plane
        double edge_x = CGAL::to_double(p2.x() - p1.x());
        double edge_y = CGAL::to_double(p2.y() - p1.y());
        
        // Inward normal vector in x-y plane (perpendicular to edge, pointing inside)
        // For counterclockwise ordering, inward normal is (-edge_y, edge_x)
        double normal_x = -edge_y;  // inward normal for counterclockwise vertices
        double normal_y = edge_x;
        
        // Normalize the normal vector
        double normal_length = std::sqrt(normal_x * normal_x + normal_y * normal_y);
        if (normal_length > 1e-12) {
            normal_x /= normal_length;
            normal_y /= normal_length;
        }
        
        // VERIFICATION: Check if normal points inward by testing against polygon centroid
        // Calculate 2D centroid of the polygon
        double centroid_x = 0.0, centroid_y = 0.0;
        for (const auto& v : vertices) {
            centroid_x += CGAL::to_double(v.x());
            centroid_y += CGAL::to_double(v.y());
        }
        centroid_x /= vertices.size();
        centroid_y /= vertices.size();
        
        // Vector from edge midpoint to centroid
        double midpoint_x = (CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) / 2.0;
        double midpoint_y = (CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) / 2.0;
        double to_centroid_x = centroid_x - midpoint_x;
        double to_centroid_y = centroid_y - midpoint_y;
        
        // Dot product: if positive, normal points toward centroid (inward)
        double dot_product = normal_x * to_centroid_x + normal_y * to_centroid_y;
        
        // If dot product is positive, normal points inward - we need outward normals for ≤ constraints
        if (dot_product > 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }
        
        // DEBUG: Verify RHS calculation consistency
        double p1_x = CGAL::to_double(p1.x());
        double p1_y = CGAL::to_double(p1.y());
        double p2_x = CGAL::to_double(p2.x());
        double p2_y = CGAL::to_double(p2.y());
        
        double rhs_p1 = normal_x * p1_x + normal_y * p1_y;
        double rhs_p2 = normal_x * p2_x + normal_y * p2_y;
        
        // Test the boundary constraint with the polygon centroid
        double centroid_value = normal_x * centroid_x + normal_y * centroid_y;
        // std::cout << "    Edge " << i << ": Normal=[" << normal_x << "," << normal_y << "], RHS=" << rhs_p1 << std::endl;
        // std::cout << "    Centroid test: " << centroid_value << " <= " << rhs_p1 << " ? " << (centroid_value <= rhs_p1 ? "PASS" : "FAIL") << std::endl;
        
        // Use p1 for RHS (both should give the same value if normal is correct)
        double rhs = rhs_p1;
        
        constraint.A(1 + i, 0) = normal_x;  // x coefficient
        constraint.A(1 + i, 1) = normal_y;  // y coefficient
        constraint.A(1 + i, 2) = 0.0;       // z coefficient (vertical plane)
        constraint.b(1 + i) = rhs;
    }
    
    // std::cout << "- Successfully created combined surface constraint:" << std::endl;
    // std::cout << "  Row 0: Plane equation (ax + by + cz = " << -d << ")" << std::endl;
    // std::cout << "  Rows 1-" << num_vertices << ": Vertical edge boundary constraints (" << num_vertices << " edges)" << std::endl;

    return constraint;
}


} // namespace nas