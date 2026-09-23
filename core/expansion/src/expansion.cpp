#include "nas/core/expansion.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nas {

std::string effector_name(StanceFoot foot) {
    return foot == StanceFoot::Left ? "LF" : "RF";
}

namespace {

struct NodeKeys {
    double perimeter = 0.0;
    Point_3 centroid{0.0, 0.0, 0.0};
};

// Area centroid and edge-length perimeter of the patch's convex polygon (in the
// surface's orthonormal 2D frame, so lengths are true lengths). Adding a
// collinear point to the polygon changes neither. Falls back to the vertex
// average when the polygon has (near) zero area.
NodeKeys canonical_keys(const Polygon_2& hull, const Transformation& to_3d, const std::vector<Point_3>& raw_patch_3d) {
    NodeKeys keys;
    const size_t n = hull.size();
    double area2 = 0.0, cx = 0.0, cy = 0.0, perimeter = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Point_2& p = hull.vertex(i);
        const Point_2& q = hull.vertex((i + 1) % n);
        double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y());
        double qx = CGAL::to_double(q.x()), qy = CGAL::to_double(q.y());
        double cross = px * qy - qx * py;
        area2 += cross;
        cx += (px + qx) * cross;
        cy += (py + qy) * cross;
        perimeter += std::hypot(qx - px, qy - py);
    }
    keys.perimeter = perimeter;
    if (std::abs(area2) > 1e-12) {
        keys.centroid = to_3d(Point_3(cx / (3.0 * area2), cy / (3.0 * area2), 0.0));
    } else {
        keys.centroid = get_centroid(raw_patch_3d);
    }
    return keys;
}

} // namespace

std::vector<Node*> expand_node(Node* parent,
                                const std::vector<Surface>& surfaces,
                                const ReachabilityModel& reachability,
                                ReachabilityDirection direction,
                                const ExpansionParams& params,
                                NodePool& pool) {
    std::vector<Node*> children;

    StanceFoot child_stance_foot = other_foot(parent->stance_foot);

    // moving = the foot about to make the next contact (the child's
    // stance foot); support = the foot currently planted (the parent's).
    // Queried by reference and only copied when rotation actually needs to
    // mutate it (rotation_enabled is false for every NAS/Tree call site
    // today) — a Polyhedron copy rebuilds a CGAL halfedge data structure,
    // not worth paying on every expansion for the common no-rotation case.
    const Polyhedron& queried_polytope = reachability.query(
        effector_name(child_stance_foot), effector_name(parent->stance_foot), direction);

    Polyhedron rotated_polytope;
    const Polyhedron* base_polytope = &queried_polytope;
    if (params.rotation_enabled) {
        rotated_polytope = rotate_polyhedron_z(queried_polytope, parent->foot_yaw);
        base_polytope = &rotated_polytope;
    }

    // One edge list per expansion, shared by all surfaces.
    const EdgeList union_edges = params.union_edges_override
        ? params.union_edges_override(*parent)
        : polytope_edges(minkowski_sum(parent->patch_vertices, *base_polytope));

    for (const auto& surface : surfaces) {
        std::vector<Point_3> plane_intersect_3d = compute_edges_plane_intersection(surface.plane, union_edges);
        if (plane_intersect_3d.size() <= 2) {
            continue;
        }

        std::vector<Point_2> plane_intersect_2d =
            transform_3d_points_to_surface_plane(plane_intersect_3d, surface.transform_to_surface);

        Polygon_2 plane_hull_2d;
        CGAL::convex_hull_2(plane_intersect_2d.begin(), plane_intersect_2d.end(), std::back_inserter(plane_hull_2d));
        std::vector<Point_2> plane_hull_pts(plane_hull_2d.vertices_begin(), plane_hull_2d.vertices_end());

        std::vector<Point_2> polygon_intersect_2d = compute_2d_polygon_intersection(
            plane_hull_pts, surface.vertices_2d, params.legacy_clip ? ClipMode::Legacy : ClipMode::Robust);
        if (polygon_intersect_2d.size() <= 2) {
            continue;
        }

        // Only patch_polygon_2d is the convex hull of the clip output, as in
        // the old get_children. patch_vertices, the polyhedron and the
        // centroid (an average of these vertices) are built from the raw
        // Sutherland-Hodgman points, near-collinear ones included: the old
        // code keeps them, and the hull would drop some, shifting centroids
        // by up to ~7cm and the polyhedron's edge-length sum (used as
        // "perimeter", a node dedup key) - found by tests/golden_all's
        // expansion differential test, see docs/paper-deltas.md.
        Polygon_2 final_hull_2d;
        CGAL::convex_hull_2(polygon_intersect_2d.begin(), polygon_intersect_2d.end(), std::back_inserter(final_hull_2d));

        std::vector<Point_3> patch_3d;
        if (params.convex_patch) {
            // A patch that degenerates to a segment/point has no polygon to keep.
            if (final_hull_2d.size() < 3) continue;
            std::vector<Point_2> hull_pts(final_hull_2d.vertices_begin(), final_hull_2d.vertices_end());
            if (params.convex_patch_simplify_tol > 0.0) {
                bool removed = true;
                while (removed && hull_pts.size() > 3) {
                    removed = false;
                    for (size_t i = 0; i < hull_pts.size(); ++i) {
                        const Point_2& a = hull_pts[(i + hull_pts.size() - 1) % hull_pts.size()];
                        const Point_2& b = hull_pts[i];
                        const Point_2& c = hull_pts[(i + 1) % hull_pts.size()];
                        double ex = CGAL::to_double(c.x() - a.x()), ey = CGAL::to_double(c.y() - a.y());
                        double len = std::hypot(ex, ey);
                        double dist = len > 0 ? std::abs(ex * CGAL::to_double(b.y() - a.y()) - ey * CGAL::to_double(b.x() - a.x())) / len
                                              : std::hypot(CGAL::to_double(b.x() - a.x()), CGAL::to_double(b.y() - a.y()));
                        if (dist < params.convex_patch_simplify_tol) {
                            hull_pts.erase(hull_pts.begin() + static_cast<std::ptrdiff_t>(i));
                            removed = true;
                            break;
                        }
                    }
                }
            }
            if (params.canonical_prism_start && !hull_pts.empty()) {
                // Same vertex order in every run: the hull starts at a tie-broken
                // vertex that flips with 1e-16 noise, and this order feeds the
                // EPA heuristic and the next Minkowski sum.
                auto key = [](const Point_2& p) {
                    return std::make_pair(std::llround(CGAL::to_double(p.x()) * 1e9), std::llround(CGAL::to_double(p.y()) * 1e9));
                };
                size_t start = 0;
                for (size_t i = 1; i < hull_pts.size(); ++i)
                    if (key(hull_pts[i]) < key(hull_pts[start])) start = i;
                std::rotate(hull_pts.begin(), hull_pts.begin() + static_cast<std::ptrdiff_t>(start), hull_pts.end());
            }
            final_hull_2d = Polygon_2(hull_pts.begin(), hull_pts.end());
            patch_3d = transform_2d_points_to_world(hull_pts, surface.transform_to_3d);
        } else {
            patch_3d = transform_2d_points_to_world(polygon_intersect_2d, surface.transform_to_3d);
        }
        Polyhedron patch_polyhedron = convex_hull_3_from_coplanar_points(patch_3d, surface.norm, params.canonical_prism_start);

        if (params.cycle_detection_enabled &&
            cycle_path_detection(parent, child_stance_foot, surface.surface_id)) {
            continue;
        }

        // Same for every yaw variant of this surface's patch.
        NodeKeys keys;
        if (params.canonical_centroid || params.canonical_perimeter) {
            keys = canonical_keys(final_hull_2d, surface.transform_to_3d, patch_3d);
        }
        if (params.hull_prism_perimeter) {
            std::vector<Point_2> hull_pts(final_hull_2d.vertices_begin(), final_hull_2d.vertices_end());
            keys.perimeter = compute_polygon_perimeter(
                convex_hull_3_from_coplanar_points(transform_2d_points_to_world(hull_pts, surface.transform_to_3d), surface.norm));
        } else if (!params.canonical_perimeter) {
            keys.perimeter = compute_polygon_perimeter(patch_polyhedron);
        }
        if (!params.canonical_centroid) keys.centroid = get_centroid(patch_3d);

        std::vector<double> yaw_angles;
        if (params.rotation_enabled) {
            for (int i = -params.yaw_discretization_num; i <= params.yaw_discretization_num; ++i) {
                yaw_angles.push_back(parent->foot_yaw + i * params.yaw_angle_increment);
            }
        } else {
            yaw_angles.push_back(0.0);
        }

        for (double yaw : yaw_angles) {
            Node* child = pool.create();
            child->parent_ptrs.push_back(parent);
            child->patch_vertices = patch_3d;
            child->stance_foot = child_stance_foot;
            child->surface_id = surface.surface_id;
            child->depth = parent->depth + 1;
            child->patch_polygon_2d = final_hull_2d;
            child->patch_polyhedron_3d = patch_polyhedron;
            child->transformation_to_2d = surface.transform_to_surface;
            child->transformation_to_3d = surface.transform_to_3d;
            child->perimeter = keys.perimeter;
            child->centroid = keys.centroid;

            if (params.rotation_enabled) {
                double normalized_yaw = yaw;
                while (normalized_yaw > M_PI) normalized_yaw -= 2.0 * M_PI;
                while (normalized_yaw < -M_PI) normalized_yaw += 2.0 * M_PI;
                child->foot_yaw = normalized_yaw;
            } else {
                child->foot_yaw = 0.0;
            }

            child->pred_surface_ids = parent->pred_surface_ids;
            child->pred_surface_ids[static_cast<size_t>(parent->stance_foot)].push_back({parent->surface_id});

            children.push_back(child);
        }
    }

    return children;
}

} // namespace nas
