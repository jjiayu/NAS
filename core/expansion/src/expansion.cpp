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

// Area centroid of a convex polygon given in the surface's 2D frame, mapped to
// world. Adding a collinear point to the polygon does not change it. Falls back
// to the vertex average when the polygon has (near) zero area.
Point_3 area_centroid(const std::vector<Point_2>& polygon, const Transformation& to_3d, const std::vector<Point_3>& vertices_3d) {
    const size_t n = polygon.size();
    double area2 = 0.0, cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Point_2& p = polygon[i];
        const Point_2& q = polygon[(i + 1) % n];
        double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y());
        double qx = CGAL::to_double(q.x()), qy = CGAL::to_double(q.y());
        double cross = px * qy - qx * py;
        area2 += cross;
        cx += (px + qx) * cross;
        cy += (py + qy) * cross;
    }
    if (std::abs(area2) > 1e-12) return to_3d(Point_3(cx / (3.0 * area2), cy / (3.0 * area2), 0.0));
    return get_centroid(vertices_3d);
}

// Removes vertices within 1 nm of the line through their neighbours, then
// rotates the list to start at the vertex minimal after rounding to 1 nm.
// The exact convex hull keeps a vertex that is collinear only up to 1e-16
// noise, and starts at a lexicographic-minimum vertex that is an x-tie decided
// by that same noise for any axis-aligned edge; both change from run to run
// with heap state, and the search's decisions with them (docs/paper-deltas.md,
// "Bilan : périmètre, clés de dédoublonnage et déterminisme").
void clean_polygon(std::vector<Point_2>& pts) {
    constexpr double TOL = 1e-9;
    bool removed = true;
    while (removed && pts.size() > 3) {
        removed = false;
        for (size_t i = 0; i < pts.size(); ++i) {
            const Point_2& a = pts[(i + pts.size() - 1) % pts.size()];
            const Point_2& b = pts[i];
            const Point_2& c = pts[(i + 1) % pts.size()];
            double ex = CGAL::to_double(c.x() - a.x()), ey = CGAL::to_double(c.y() - a.y());
            double len = std::hypot(ex, ey);
            double dist = len > 0 ? std::abs(ex * CGAL::to_double(b.y() - a.y()) - ey * CGAL::to_double(b.x() - a.x())) / len
                                  : std::hypot(CGAL::to_double(b.x() - a.x()), CGAL::to_double(b.y() - a.y()));
            if (dist < TOL) {
                pts.erase(pts.begin() + static_cast<std::ptrdiff_t>(i));
                removed = true;
                break;
            }
        }
    }
    if (pts.empty()) return;
    auto key = [](const Point_2& p) {
        return std::make_pair(std::llround(CGAL::to_double(p.x()) * 1e9), std::llround(CGAL::to_double(p.y()) * 1e9));
    };
    size_t start = 0;
    for (size_t i = 1; i < pts.size(); ++i)
        if (key(pts[i]) < key(pts[start])) start = i;
    std::rotate(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(start), pts.end());
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

    // The polytope is in the support (parent) foot's frame: rotate it by the paper's Q (Eq. 2),
    // the parent's yaw composed with the tilt of the surface it stands on. Flat surfaces take
    // the exact yaw-only path (identical to what horizontal scenes always did).
    Polyhedron rotated_polytope;
    const Polyhedron* base_polytope = &queried_polytope;
    const Vector_3 support_normal = parent->up_normal();
    if (!is_vertical_normal(support_normal)) {
        rotated_polytope = rotate_polyhedron(queried_polytope, foot_frame_rotation(support_normal, params.rotation_enabled ? parent->foot_yaw : 0.0));
        base_polytope = &rotated_polytope;
    } else if (params.rotation_enabled) {
        rotated_polytope = rotate_polyhedron_z(queried_polytope, parent->foot_yaw);
        base_polytope = &rotated_polytope;
    }

    Polyhedron P_union = minkowski_sum(parent->patch_vertices, *base_polytope);

    for (const auto& surface : surfaces) {
        std::vector<Point_3> plane_intersect_3d = compute_polytope_plane_intersection(surface.plane, P_union);
        if (plane_intersect_3d.size() <= 2) {
            continue;
        }

        std::vector<Point_2> plane_intersect_2d =
            transform_3d_points_to_surface_plane(plane_intersect_3d, surface.transform_to_surface);

        Polygon_2 plane_hull_2d;
        CGAL::convex_hull_2(plane_intersect_2d.begin(), plane_intersect_2d.end(), std::back_inserter(plane_hull_2d));
        std::vector<Point_2> plane_hull_pts(plane_hull_2d.vertices_begin(), plane_hull_2d.vertices_end());

        std::vector<Point_2> polygon_intersect_2d = compute_2d_polygon_intersection(plane_hull_pts, surface.vertices_2d);
        if (polygon_intersect_2d.size() <= 2) {
            continue;
        }

        // The patch is the convex polygon of the clip output, cleaned (see clean_polygon).
        Polygon_2 hull_2d;
        CGAL::convex_hull_2(polygon_intersect_2d.begin(), polygon_intersect_2d.end(), std::back_inserter(hull_2d));
        std::vector<Point_2> patch_2d(hull_2d.vertices_begin(), hull_2d.vertices_end());
        clean_polygon(patch_2d);
        if (patch_2d.size() < 3) {
            continue; // degenerated to a segment/point: no patch
        }

        if (params.cycle_detection_enabled &&
            cycle_path_detection(parent, child_stance_foot, surface.surface_id)) {
            continue;
        }

        std::vector<Point_3> patch_3d = transform_2d_points_to_world(patch_2d, surface.transform_to_3d);
        Polygon_2 patch_polygon(patch_2d.begin(), patch_2d.end());
        Point_3 centroid = area_centroid(patch_2d, surface.transform_to_3d, patch_3d);

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
            child->patch_polygon_2d = patch_polygon;
            child->transformation_to_2d = surface.transform_to_surface;
            child->transformation_to_3d = surface.transform_to_3d;
            child->centroid = centroid;

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
