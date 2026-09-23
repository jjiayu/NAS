#include "nas/core/geometry.hpp"

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/convex.h>
#include <coal/distance.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

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
        sum = sum + (point - CGAL::ORIGIN);
    }
    return CGAL::ORIGIN + (sum / static_cast<double>(points.size()));
}

Polyhedron minkowski_sum(const std::vector<Point_3>& patch_vertices,
                         const Polyhedron& polytope) {
    std::vector<Point_3> all_vertices;
    for (size_t i = 0; i < patch_vertices.size(); ++i) {
        Transformation translation(CGAL::TRANSLATION, patch_vertices[i] - CGAL::ORIGIN);
        for (auto v = polytope.vertices_begin(); v != polytope.vertices_end(); ++v) {
            all_vertices.push_back(translation(v->point()));
        }
    }
    Polyhedron P_union;
    CGAL::convex_hull_3(all_vertices.begin(), all_vertices.end(), P_union);
    return P_union;
}

EdgeList polytope_edges(const Polyhedron& polytope) {
    EdgeList edges;
    for (auto edge = polytope.edges_begin(); edge != polytope.edges_end(); ++edge) {
        edges.emplace_back(edge->vertex()->point(), edge->opposite()->vertex()->point());
    }
    return edges;
}

std::vector<Point_3> compute_edges_plane_intersection(const Plane_3& plane, const EdgeList& edges) {
    std::vector<Point_3> intersection_points;
    for (const auto& [p1, p2] : edges) {
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

std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope) {
    return compute_edges_plane_intersection(plane, polytope_edges(polytope));
}

double is_leftside_of_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end) {
    return ((edge_end.x() - edge_start.x()) * (point.y() - edge_start.y()) -
            (edge_end.y() - edge_start.y()) * (point.x() - edge_start.x()));
}

// Sutherland-Hodgman clip. See the header comment: a CGAL-native
// implementation was tried and reverted here because it crashed on the
// real NarrowPassage scenario's near-degenerate shrunk "Passage" polygon.
std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& subject_polygon, const std::vector<Point_2>& clip_polygon) {
    // Both polygons come from a convex hull upstream (expand_node,
    // Surface's own constructor) and are never empty in practice — an
    // empty polygon here means a caller broke that invariant, not a
    // recoverable geometric edge case (see docs/paper-deltas.md, 8d-1).
    if (subject_polygon.empty() || clip_polygon.empty()) {
        throw std::invalid_argument("compute_2d_polygon_intersection: subject/clip polygon must not be empty");
    }

    std::vector<Point_2> output_list = subject_polygon;

    auto clip_end = clip_polygon.end();
    for (auto clip_it = clip_polygon.begin(); clip_it != clip_end; ++clip_it) {
        if (output_list.empty()) {
            return std::vector<Point_2>();
        }

        Point_2 edge_start = *clip_it;
        Point_2 edge_end = (std::next(clip_it) == clip_end) ? clip_polygon.front() : *std::next(clip_it);

        std::vector<Point_2> input_list = output_list;
        output_list.clear();

        for (size_t i = 0; i < input_list.size(); i++) {
            Point_2 current_point = input_list[i];
            Point_2 prev_point = input_list[(i + input_list.size() - 1) % input_list.size()];

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
            } else if (prev_inside) {
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

double compute_polygon_perimeter(const Polyhedron& polyhedron) {
    double perimeter = 0.0;
    for (auto edge = polyhedron.edges_begin(); edge != polyhedron.edges_end(); ++edge) {
        perimeter += CGAL::sqrt(CGAL::squared_distance(edge->vertex()->point(),
                                                     edge->opposite()->vertex()->point()));
    }
    return perimeter;
}

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location) {
    return CGAL::sqrt(CGAL::squared_distance(start_location, end_location));
}

double calculate_gjk_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal) {
    // A real patch always has >=3 vertices by construction (expand_node
    // checks this before creating a Node) — this guards a caller contract,
    // not a recoverable geometric edge case.
    if (patch_points.size() < 3) {
        throw std::invalid_argument("calculate_gjk_distance_point_to_patch: patch must have at least 3 points");
    }

    coal::Vec3s coal_goal(goal.x(), goal.y(), goal.z());
    std::vector<coal::Vec3s> coal_patch_vertices;
    coal_patch_vertices.reserve(patch_points.size());
    for (const auto& point : patch_points) {
        coal_patch_vertices.emplace_back(point.x(), point.y(), point.z());
    }

    auto point_shape = std::make_shared<coal::Sphere>(1e-6);
    coal::CollisionObject point_obj(point_shape);
    coal::Transform3s point_tf = coal::Transform3s::Identity();
    point_tf.translation() = coal_goal;
    point_obj.setTransform(point_tf);

    try {
        auto vertices_ptr = std::make_shared<std::vector<coal::Vec3s>>(coal_patch_vertices);
        std::vector<coal::Triangle> triangles;
        for (size_t i = 1; i < coal_patch_vertices.size() - 1; ++i) {
            triangles.push_back(coal::Triangle(0, i, i + 1));
        }
        auto triangles_ptr = std::make_shared<std::vector<coal::Triangle>>(triangles);

        auto convex_shape = std::make_shared<coal::Convex<coal::Triangle>>(
            vertices_ptr, coal_patch_vertices.size(),
            triangles_ptr, triangles.size()
        );

        coal::CollisionObject convex_obj(convex_shape);
        coal::Transform3s convex_tf = coal::Transform3s::Identity();
        convex_obj.setTransform(convex_tf);

        coal::DistanceRequest request;
        coal::DistanceResult result;
        coal::distance(&point_obj, &convex_obj, request, result);
        return result.min_distance;

    } catch (const std::exception&) {
        // Unlike the precondition above, this is coal itself failing on
        // otherwise-valid input (e.g. a numerically thin/degenerate
        // triangle fan) — a genuine runtime condition, not a caller error.
        // Falling back to centroid distance is a deliberate, kept-from-the-
        // old-code choice rather than propagating the failure — see
        // docs/paper-deltas.md, 8d-1.
        Point_3 centroid = get_centroid(patch_points);
        return compute_euclidean_distance(goal, centroid);
    }
}

double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal) {
    // Same reasoning as calculate_gjk_distance_point_to_patch above.
    if (patch_points.size() < 3) {
        throw std::invalid_argument("calculate_epa_distance_point_to_patch: patch must have at least 3 points");
    }

    coal::Vec3s coal_goal(goal.x(), goal.y(), goal.z());
    std::vector<coal::Vec3s> coal_patch_vertices;
    coal_patch_vertices.reserve(patch_points.size());
    for (const auto& point : patch_points) {
        coal_patch_vertices.emplace_back(point.x(), point.y(), point.z());
    }

    auto point_shape = std::make_shared<coal::Sphere>(1e-6);
    coal::CollisionObject point_obj(point_shape);
    coal::Transform3s point_tf = coal::Transform3s::Identity();
    point_tf.translation() = coal_goal;
    point_obj.setTransform(point_tf);

    try {
        auto vertices_ptr = std::make_shared<std::vector<coal::Vec3s>>(coal_patch_vertices);
        std::vector<coal::Triangle> triangles;
        for (size_t i = 1; i < coal_patch_vertices.size() - 1; ++i) {
            triangles.push_back(coal::Triangle(0, i, i + 1));
        }
        auto triangles_ptr = std::make_shared<std::vector<coal::Triangle>>(triangles);

        auto convex_shape = std::make_shared<coal::Convex<coal::Triangle>>(
            vertices_ptr, coal_patch_vertices.size(),
            triangles_ptr, triangles.size()
        );

        coal::CollisionObject convex_obj(convex_shape);
        coal::Transform3s convex_tf = coal::Transform3s::Identity();
        convex_obj.setTransform(convex_tf);

        coal::DistanceRequest distance_request;
        coal::DistanceResult distance_result;
        distance_request.enable_signed_distance = false;

        coal::distance(&point_obj, &convex_obj, distance_request, distance_result);
        return std::abs(distance_result.min_distance);

    } catch (const std::exception&) {
        // Same reasoning as the catch block in calculate_gjk_distance_point_to_patch.
        Point_3 centroid = get_centroid(patch_points);
        return compute_euclidean_distance(goal, centroid);
    }
}

HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope) {
    HalfSpacePolytopeConstraint constraint;

    Point_3 centroid = std::accumulate(
        polytope.vertices_begin(), polytope.vertices_end(), Point_3(0, 0, 0),
        [](const Point_3& acc, const auto& vertex) {
            return Point_3(acc.x() + vertex.point().x(),
                         acc.y() + vertex.point().y(),
                         acc.z() + vertex.point().z());
        });

    int vertex_count = std::distance(polytope.vertices_begin(), polytope.vertices_end());
    if (vertex_count == 0) {
        throw std::runtime_error("Error: Empty polytope");
    }

    centroid = Point_3(centroid.x() / vertex_count,
                      centroid.y() / vertex_count,
                      centroid.z() / vertex_count);

    int num_facets = polytope.size_of_facets();
    constraint.A = Eigen::MatrixXd::Zero(num_facets, 3);
    constraint.b = Eigen::VectorXd::Zero(num_facets);

    int facet_index = 0;
    for (auto facet = polytope.facets_begin(); facet != polytope.facets_end(); ++facet) {
        auto h = facet->facet_begin();
        const Point_3& p1 = h->vertex()->point();
        const Point_3& p2 = (++h)->vertex()->point();
        const Point_3& p3 = (++h)->vertex()->point();

        Plane_3 plane(p1, p2, p3);

        // A single degenerate facet in an otherwise-valid polytope is
        // tolerated by skipping it (the constraint just gets one fewer
        // row) — this can legitimately happen from floating-point noise
        // in the source .obj mesh, it isn't a caller error (8d-1).
        if (plane.is_degenerate()) {
            continue;
        }

        double a = CGAL::to_double(plane.a());
        double b = CGAL::to_double(plane.b());
        double c = CGAL::to_double(plane.c());
        double d = CGAL::to_double(plane.d());

        double norm = std::sqrt(a*a + b*b + c*c);
        if (norm <= 1e-12) {
            // Same reasoning as the degenerate-facet check above.
            continue;
        }
        a /= norm; b /= norm; c /= norm; d /= norm;

        double centroid_side = a * CGAL::to_double(centroid.x()) +
                              b * CGAL::to_double(centroid.y()) +
                              c * CGAL::to_double(centroid.z()) + d;

        if (centroid_side > 0) {
            a = -a; b = -b; c = -c; d = -d;
        }

        constraint.A(facet_index, 0) = a;
        constraint.A(facet_index, 1) = b;
        constraint.A(facet_index, 2) = c;
        constraint.b(facet_index) = -d;

        facet_index++;
    }

    if (facet_index < num_facets) {
        constraint.A.conservativeResize(facet_index, 3);
        constraint.b.conservativeResize(facet_index);
    }

    return constraint;
}

SurfaceConstraint generate_surface_constraint(const Polyhedron& surface_3d) {
    SurfaceConstraint constraint;

    std::vector<Point_3> vertices;
    for (auto v = surface_3d.vertices_begin(); v != surface_3d.vertices_end(); ++v) {
        vertices.push_back(v->point());
    }

    if (vertices.size() < 3) {
        throw std::runtime_error("Error: Need at least 3 vertices to fit a plane");
    }

    Plane_3 plane;
    CGAL::linear_least_squares_fitting_3(vertices.begin(), vertices.end(), plane, CGAL::Dimension_tag<0>());

    double a = CGAL::to_double(plane.a());
    double b = CGAL::to_double(plane.b());
    double c = CGAL::to_double(plane.c());
    double d = CGAL::to_double(plane.d());

    double norm = std::sqrt(a*a + b*b + c*c);
    if (norm <= 1e-12) {
        // Unlike a single degenerate facet on a polytope (tolerated in
        // convert_polytope_to_half_space_constraint above), a zero-normal
        // fit here means the *entire* surface's plane fit degenerated —
        // silently returning an empty constraint would let a surface
        // vanish from the QP without anyone noticing. Throw instead (8d-1).
        throw std::runtime_error("generate_surface_constraint: degenerate plane fit (zero normal vector)");
    }
    a /= norm; b /= norm; c /= norm; d /= norm;

    int num_vertices = vertices.size();
    if (num_vertices < 3) {
        throw std::runtime_error("Error: Surface has fewer than 3 vertices");
    }

    constraint.A = Eigen::MatrixXd::Zero(1 + num_vertices, 3);
    constraint.b = Eigen::VectorXd::Zero(1 + num_vertices);

    constraint.A(0, 0) = a;
    constraint.A(0, 1) = b;
    constraint.A(0, 2) = c;
    constraint.b(0) = -d;

    for (int i = 0; i < num_vertices; i++) {
        Point_3 p1 = vertices[i];
        Point_3 p2 = vertices[(i + 1) % num_vertices];

        double edge_x = CGAL::to_double(p2.x() - p1.x());
        double edge_y = CGAL::to_double(p2.y() - p1.y());

        double normal_x = -edge_y;
        double normal_y = edge_x;

        double normal_length = std::sqrt(normal_x * normal_x + normal_y * normal_y);
        if (normal_length > 1e-12) {
            normal_x /= normal_length;
            normal_y /= normal_length;
        }

        double centroid_x = 0.0, centroid_y = 0.0;
        for (const auto& v : vertices) {
            centroid_x += CGAL::to_double(v.x());
            centroid_y += CGAL::to_double(v.y());
        }
        centroid_x /= vertices.size();
        centroid_y /= vertices.size();

        double midpoint_x = (CGAL::to_double(p1.x()) + CGAL::to_double(p2.x())) / 2.0;
        double midpoint_y = (CGAL::to_double(p1.y()) + CGAL::to_double(p2.y())) / 2.0;
        double to_centroid_x = centroid_x - midpoint_x;
        double to_centroid_y = centroid_y - midpoint_y;

        double dot_product = normal_x * to_centroid_x + normal_y * to_centroid_y;

        if (dot_product > 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        double rhs = normal_x * CGAL::to_double(p1.x()) + normal_y * CGAL::to_double(p1.y());

        constraint.A(1 + i, 0) = normal_x;
        constraint.A(1 + i, 1) = normal_y;
        constraint.A(1 + i, 2) = 0.0;
        constraint.b(1 + i) = rhs;
    }

    return constraint;
}

Polyhedron rotate_polyhedron_z(const Polyhedron& polytope, double yaw_angle) {
    double cos_yaw = std::cos(yaw_angle);
    double sin_yaw = std::sin(yaw_angle);

    Transformation rotation(
        cos_yaw, -sin_yaw, 0.0, 0.0,
        sin_yaw,  cos_yaw, 0.0, 0.0,
        0.0,      0.0,     1.0, 0.0,
        1.0
    );

    Polyhedron rotated_polytope = polytope;
    for (auto v_it = rotated_polytope.vertices_begin(); v_it != rotated_polytope.vertices_end(); ++v_it) {
        v_it->point() = rotation(v_it->point());
    }
    return rotated_polytope;
}

// Helper: build a prism Polyhedron from ordered 2D hull vertices reprojected
// to 3D — CGAL::convex_hull_3 asserts on coplanar input, so this avoids
// calling it at all for the "flat patch" case that's pervasive here.
template <class HDS>
class Build_prism : public CGAL::Modifier_base<HDS> {
    std::vector<Point_3> top_, bot_;
public:
    Build_prism(std::vector<Point_3> top, std::vector<Point_3> bot)
        : top_(std::move(top)), bot_(std::move(bot)) {}
    void operator()(HDS& hds) {
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        const int n = static_cast<int>(top_.size());
        B.begin_surface(2 * n, 2 * n + 2 * (n - 2), 0);
        for (int i = 0; i < n; ++i) B.add_vertex(top_[i]);
        for (int i = 0; i < n; ++i) B.add_vertex(bot_[i]);
        for (int i = 1; i < n - 1; ++i) {
            B.begin_facet();
            B.add_vertex_to_facet(0);
            B.add_vertex_to_facet(i);
            B.add_vertex_to_facet(i + 1);
            B.end_facet();
        }
        for (int i = 1; i < n - 1; ++i) {
            B.begin_facet();
            B.add_vertex_to_facet(n);
            B.add_vertex_to_facet(n + i + 1);
            B.add_vertex_to_facet(n + i);
            B.end_facet();
        }
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            B.begin_facet();
            B.add_vertex_to_facet(j);
            B.add_vertex_to_facet(i);
            B.add_vertex_to_facet(n + i);
            B.end_facet();
            B.begin_facet();
            B.add_vertex_to_facet(j);
            B.add_vertex_to_facet(n + i);
            B.add_vertex_to_facet(n + j);
            B.end_facet();
        }
        B.end_surface();
    }
};

Polyhedron convex_hull_3_from_coplanar_points(const std::vector<Point_3>& points, const Vector_3& normal) {
    Vector_3 n = normal / std::sqrt(normal.squared_length());

    Vector_3 t = (std::abs(n.x()) < 0.9) ? Vector_3(1, 0, 0) : Vector_3(0, 1, 0);
    Vector_3 u = CGAL::cross_product(n, t);
    u = u / std::sqrt(u.squared_length());
    Vector_3 v = CGAL::cross_product(n, u);

    Point_3 origin = get_centroid(points);

    std::vector<Point_2> pts2d;
    pts2d.reserve(points.size());
    for (const auto& p : points) {
        Vector_3 d = p - origin;
        pts2d.emplace_back(d * u, d * v);
    }

    std::vector<Point_2> hull2d;
    CGAL::convex_hull_2(pts2d.begin(), pts2d.end(), std::back_inserter(hull2d));

    const double eps = 1e-6;
    std::vector<Point_3> top, bot;
    top.reserve(hull2d.size());
    bot.reserve(hull2d.size());
    for (const auto& p2 : hull2d) {
        Point_3 p3 = origin + p2.x() * u + p2.y() * v;
        top.push_back(p3 + eps * n);
        bot.push_back(p3 - eps * n);
    }

    Polyhedron poly;
    Build_prism<Polyhedron::HalfedgeDS> builder(std::move(top), std::move(bot));
    poly.delegate(builder);
    return poly;
}

} // namespace nas
