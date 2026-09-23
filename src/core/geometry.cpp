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
#include <memory>
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

std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope) {
    std::vector<Point_3> intersection_points;
    for (auto edge = polytope.edges_begin(); edge != polytope.edges_end(); ++edge) {
        Kernel::Segment_3 segment(edge->vertex()->point(), edge->opposite()->vertex()->point());
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

            double current_side = is_leftside_of_edge(current_point, edge_start, edge_end);
            double prev_side = is_leftside_of_edge(prev_point, edge_start, edge_end);
            bool current_inside = current_side >= 0;
            bool prev_inside = prev_side >= 0;

            // Called only when the segment prev->current straddles the clip line
            // according to the inside test above: the two signed values have
            // opposite signs, so the denominator is non-zero and the point exists.
            auto push_crossing = [&]() {
                double t = prev_side / (prev_side - current_side);
                output_list.emplace_back(prev_point.x() + t * (current_point.x() - prev_point.x()),
                                         prev_point.y() + t * (current_point.y() - prev_point.y()));
            };

            if (current_inside) {
                if (!prev_inside) push_crossing();
                output_list.push_back(current_point);
            } else if (prev_inside) {
                push_crossing();
            }
        }
    }

    return output_list;
}

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location) {
    return CGAL::sqrt(CGAL::squared_distance(start_location, end_location));
}

double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal) {
    // A real patch always has >=3 vertices by construction (expand_node checks
    // this before creating a Node) — this guards a caller contract, not a
    // recoverable geometric edge case.
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
        // Unlike the precondition above, this is coal itself failing on
        // otherwise-valid input (e.g. a numerically thin/degenerate triangle
        // fan) — a genuine runtime condition, not a caller error. Falling back
        // to centroid distance is a deliberate, kept-from-the-old-code choice
        // rather than propagating the failure — see docs/paper-deltas.md, 8d-1.
        Point_3 centroid = get_centroid(patch_points);
        return compute_euclidean_distance(goal, centroid);
    }
}

namespace {
// A planar convex polygon as a coal convex shape (triangle fan), like the point-to-patch distance does.
std::shared_ptr<coal::Convex<coal::Triangle>> fan_convex(const std::vector<Point_3>& pts) {
    auto vertices = std::make_shared<std::vector<coal::Vec3s>>();
    for (const auto& p : pts) vertices->emplace_back(p.x(), p.y(), p.z());
    auto triangles = std::make_shared<std::vector<coal::Triangle>>();
    for (size_t i = 1; i + 1 < pts.size(); ++i) triangles->push_back(coal::Triangle(0, i, i + 1));
    return std::make_shared<coal::Convex<coal::Triangle>>(vertices, vertices->size(), triangles, triangles->size());
}
} // namespace

double calculate_epa_distance_patch_to_patch(const std::vector<Point_3>& patch_a, const std::vector<Point_3>& patch_b) {
    if (patch_a.size() < 3 || patch_b.size() < 3) {
        throw std::invalid_argument("calculate_epa_distance_patch_to_patch: both patches must have at least 3 points");
    }
    try {
        coal::CollisionObject obj_a(fan_convex(patch_a));
        coal::CollisionObject obj_b(fan_convex(patch_b));
        coal::DistanceRequest request;
        coal::DistanceResult result;
        request.enable_signed_distance = false;
        coal::distance(&obj_a, &obj_b, request, result);
        return std::max(0.0, result.min_distance);
    } catch (const std::exception&) {
        // as in the point-to-patch distance: fall back to the distance between centroids
        return compute_euclidean_distance(get_centroid(patch_a), get_centroid(patch_b));
    }
}

HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& mesh) {
    HalfSpacePolytopeConstraint constraint;

    // The polytope IS the convex hull of the mesh's vertices (that is what the search's
    // Minkowski sum uses). The reachability .obj files also contain quadrilateral faces
    // that are not planar (22 of them, up to 29 cm from the plane of their first three
    // vertices): taking each face's plane from its first three vertices, as the old code
    // did, gives a region that differs from the polytope - footsteps satisfying it left
    // the true polytope by up to 7 mm. Facets of the hull are exact triangles.
    std::vector<Point_3> mesh_vertices;
    for (auto v = mesh.vertices_begin(); v != mesh.vertices_end(); ++v) mesh_vertices.push_back(v->point());
    if (mesh_vertices.empty()) {
        throw std::runtime_error("Error: Empty polytope");
    }
    Polyhedron polytope;
    CGAL::convex_hull_3(mesh_vertices.begin(), mesh_vertices.end(), polytope);

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

SurfaceConstraint generate_surface_constraint(const std::vector<Point_3>& vertices) {
    SurfaceConstraint constraint;

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

bool is_vertical_normal(const Vector_3& normal) {
    double nx = CGAL::to_double(normal.x()), ny = CGAL::to_double(normal.y());
    return std::abs(nx) < 1e-9 && std::abs(ny) < 1e-9;
}

Eigen::Matrix3d foot_frame_rotation(const Vector_3& surface_normal, double yaw) {
    double nx = CGAL::to_double(surface_normal.x()), ny = CGAL::to_double(surface_normal.y()), nz = CGAL::to_double(surface_normal.z());
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-12) { nx = 0; ny = 0; nz = 1; } else { nx /= len; ny /= len; nz /= len; }
    if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; } // up normal

    Eigen::Matrix3d Rz;
    double c = std::cos(yaw), s = std::sin(yaw);
    Rz << c, -s, 0,
          s,  c, 0,
          0,  0, 1;

    // Rodrigues: minimal rotation taking e_z to n, axis = e_z x n = (-ny, nx, 0), sin = |axis|, cos = nz.
    double sin_t = std::hypot(nx, ny);
    Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity();
    if (sin_t > 1e-12) {
        // skew matrix of the axis (ax, ay, az) = (-ny, nx, 0): [[0,-az,ay],[az,0,-ax],[-ay,ax,0]]
        Eigen::Matrix3d K;
        K << 0, 0, nx,
             0, 0, ny,
             -nx, -ny, 0;
        Rt = Eigen::Matrix3d::Identity() + K + K * K * ((1.0 - nz) / (sin_t * sin_t));
    }
    return Rt * Rz;
}

Polyhedron rotate_polyhedron(const Polyhedron& polytope, const Eigen::Matrix3d& R) {
    Polyhedron rotated = polytope;
    for (auto v = rotated.vertices_begin(); v != rotated.vertices_end(); ++v) {
        Eigen::Vector3d p(CGAL::to_double(v->point().x()), CGAL::to_double(v->point().y()), CGAL::to_double(v->point().z()));
        Eigen::Vector3d q = R * p;
        v->point() = Point_3(q.x(), q.y(), q.z());
    }
    return rotated;
}

namespace {
// Hull vertices from CGAL::convex_hull_3/2 are always literal copies of specific
// input points (never synthesized) -- linear scan is exact and simple at the
// sizes these polytopes have (tens of vertices, not thousands).
template <typename Point>
const Point& find_payload(const Point& p, const std::vector<Point>& points, const std::vector<Point>& payloads) {
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i] == p) return payloads[i];
    }
    throw std::logic_error("find_payload: hull vertex not found among inputs (unexpected)");
}
} // namespace

TaggedPolyhedron minkowski_sum_tagged(const std::vector<Point_3>& patch_vertices,
                                       const std::vector<Point_3>& payloads,
                                       const Polyhedron& polytope) {
    if (patch_vertices.size() != payloads.size()) {
        throw std::invalid_argument("minkowski_sum_tagged: patch_vertices and payloads must have the same size");
    }
    std::vector<Point_3> all_vertices;
    std::vector<Point_3> all_payloads;
    for (size_t i = 0; i < patch_vertices.size(); ++i) {
        Transformation translation(CGAL::TRANSLATION, patch_vertices[i] - CGAL::ORIGIN);
        for (auto v = polytope.vertices_begin(); v != polytope.vertices_end(); ++v) {
            all_vertices.push_back(translation(v->point()));
            all_payloads.push_back(payloads[i]);
        }
    }

    TaggedPolyhedron result;
    CGAL::convex_hull_3(all_vertices.begin(), all_vertices.end(), result.mesh);
    for (auto v = result.mesh.vertices_begin(); v != result.mesh.vertices_end(); ++v) {
        result.vertices.push_back(v->point());
        result.payloads.push_back(find_payload(v->point(), all_vertices, all_payloads));
    }
    return result;
}

std::vector<TaggedPoint3> compute_polytope_plane_intersection_tagged(const Plane_3& plane, const TaggedPolyhedron& tp) {
    std::vector<TaggedPoint3> intersection_points;
    for (auto edge = tp.mesh.edges_begin(); edge != tp.mesh.edges_end(); ++edge) {
        const Point_3& p0 = edge->vertex()->point();
        const Point_3& p1 = edge->opposite()->vertex()->point();
        double d0 = CGAL::to_double(plane.a() * p0.x() + plane.b() * p0.y() + plane.c() * p0.z() + plane.d());
        double d1 = CGAL::to_double(plane.a() * p1.x() + plane.b() * p1.y() + plane.c() * p1.z() + plane.d());
        if ((d0 > 0) == (d1 > 0)) continue; // both on the same side: this edge doesn't cross the plane
        if (d0 == d1) continue; // degenerate (both exactly on-plane): same as the untagged function, not special-cased

        double t = d0 / (d0 - d1);
        const Point_3& pay0 = find_payload(p0, tp.vertices, tp.payloads);
        const Point_3& pay1 = find_payload(p1, tp.vertices, tp.payloads);

        auto lerp = [t](const Point_3& a, const Point_3& b) {
            return Point_3(CGAL::to_double(a.x()) + t * CGAL::to_double(b.x() - a.x()),
                           CGAL::to_double(a.y()) + t * CGAL::to_double(b.y() - a.y()),
                           CGAL::to_double(a.z()) + t * CGAL::to_double(b.z() - a.z()));
        };
        intersection_points.push_back({lerp(p0, p1), lerp(pay0, pay1)});
    }
    return intersection_points;
}

std::vector<TaggedPoint2> convex_hull_2_tagged(const std::vector<TaggedPoint2>& points) {
    std::vector<Point_2> pts;
    pts.reserve(points.size());
    for (const auto& tp : points) pts.push_back(tp.point);

    Polygon_2 hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));

    std::vector<TaggedPoint2> result;
    for (auto v = hull.vertices_begin(); v != hull.vertices_end(); ++v) {
        for (const auto& tp : points) {
            if (tp.point == *v) {
                result.push_back(tp);
                break;
            }
        }
    }
    return result;
}

std::vector<TaggedPoint2> compute_2d_polygon_intersection_tagged(
    const std::vector<TaggedPoint2>& subject_polygon,
    const std::vector<Point_2>& clip_polygon,
    const std::function<Point_2(const TaggedPoint2&)>& classify_coord) {
    if (subject_polygon.empty() || clip_polygon.empty()) {
        throw std::invalid_argument("compute_2d_polygon_intersection_tagged: subject/clip polygon must not be empty");
    }

    std::vector<TaggedPoint2> output_list = subject_polygon;

    auto clip_end = clip_polygon.end();
    for (auto clip_it = clip_polygon.begin(); clip_it != clip_end; ++clip_it) {
        if (output_list.empty()) return std::vector<TaggedPoint2>();

        Point_2 edge_start = *clip_it;
        Point_2 edge_end = (std::next(clip_it) == clip_end) ? clip_polygon.front() : *std::next(clip_it);

        std::vector<TaggedPoint2> input_list = output_list;
        output_list.clear();

        for (size_t i = 0; i < input_list.size(); i++) {
            const TaggedPoint2& current = input_list[i];
            const TaggedPoint2& prev = input_list[(i + input_list.size() - 1) % input_list.size()];

            double current_side = is_leftside_of_edge(classify_coord(current), edge_start, edge_end);
            double prev_side = is_leftside_of_edge(classify_coord(prev), edge_start, edge_end);
            bool current_inside = current_side >= 0;
            bool prev_inside = prev_side >= 0;

            auto push_crossing = [&]() {
                double t = prev_side / (prev_side - current_side);
                Point_2 pt(prev.point.x() + t * (current.point.x() - prev.point.x()),
                           prev.point.y() + t * (current.point.y() - prev.point.y()));
                Point_2 pay(prev.payload.x() + t * (current.payload.x() - prev.payload.x()),
                            prev.payload.y() + t * (current.payload.y() - prev.payload.y()));
                output_list.push_back({pt, pay});
            };

            if (current_inside) {
                if (!prev_inside) push_crossing();
                output_list.push_back(current);
            } else if (prev_inside) {
                push_crossing();
            }
        }
    }

    return output_list;
}

std::vector<TaggedPoint2WithOrigin> convex_hull_2_with_origin(const std::vector<TaggedPoint2WithOrigin>& points) {
    std::vector<Point_2> pts;
    pts.reserve(points.size());
    for (const auto& tp : points) pts.push_back(tp.point);

    Polygon_2 hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));

    std::vector<TaggedPoint2WithOrigin> result;
    for (auto v = hull.vertices_begin(); v != hull.vertices_end(); ++v) {
        for (const auto& tp : points) {
            if (tp.point == *v) {
                result.push_back(tp);
                break;
            }
        }
    }
    return result;
}

std::vector<TaggedPoint2WithOrigin> compute_2d_polygon_intersection_with_origin(
    const std::vector<TaggedPoint2WithOrigin>& subject_polygon,
    const std::vector<Point_2>& clip_polygon) {
    if (subject_polygon.empty() || clip_polygon.empty()) {
        throw std::invalid_argument("compute_2d_polygon_intersection_with_origin: subject/clip polygon must not be empty");
    }

    std::vector<TaggedPoint2WithOrigin> output_list = subject_polygon;

    auto clip_end = clip_polygon.end();
    for (auto clip_it = clip_polygon.begin(); clip_it != clip_end; ++clip_it) {
        if (output_list.empty()) return std::vector<TaggedPoint2WithOrigin>();

        Point_2 edge_start = *clip_it;
        Point_2 edge_end = (std::next(clip_it) == clip_end) ? clip_polygon.front() : *std::next(clip_it);

        std::vector<TaggedPoint2WithOrigin> input_list = output_list;
        output_list.clear();

        for (size_t i = 0; i < input_list.size(); i++) {
            const TaggedPoint2WithOrigin& current = input_list[i];
            const TaggedPoint2WithOrigin& prev = input_list[(i + input_list.size() - 1) % input_list.size()];

            double current_side = is_leftside_of_edge(current.point, edge_start, edge_end);
            double prev_side = is_leftside_of_edge(prev.point, edge_start, edge_end);
            bool current_inside = current_side >= 0;
            bool prev_inside = prev_side >= 0;

            auto push_crossing = [&]() {
                double t = prev_side / (prev_side - current_side);
                Point_2 pt(prev.point.x() + t * (current.point.x() - prev.point.x()),
                           prev.point.y() + t * (current.point.y() - prev.point.y()));
                Point_3 pay(CGAL::to_double(prev.payload.x()) + t * CGAL::to_double(current.payload.x() - prev.payload.x()),
                            CGAL::to_double(prev.payload.y()) + t * CGAL::to_double(current.payload.y() - prev.payload.y()),
                            CGAL::to_double(prev.payload.z()) + t * CGAL::to_double(current.payload.z() - prev.payload.z()));
                output_list.push_back({pt, pay});
            };

            if (current_inside) {
                if (!prev_inside) push_crossing();
                output_list.push_back(current);
            } else if (prev_inside) {
                push_crossing();
            }
        }
    }

    return output_list;
}

} // namespace nas
