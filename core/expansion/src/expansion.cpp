#include "nas/core/expansion.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <stdexcept>

namespace nas {

std::string effector_name(StanceFoot foot) {
    return foot == StanceFoot::Left ? "LF" : "RF";
}

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

        Polygon_2 final_hull_2d;
        CGAL::convex_hull_2(polygon_intersect_2d.begin(), polygon_intersect_2d.end(), std::back_inserter(final_hull_2d));
        std::vector<Point_2> final_hull_pts(final_hull_2d.vertices_begin(), final_hull_2d.vertices_end());

        std::vector<Point_3> patch_3d = transform_2d_points_to_world(final_hull_pts, surface.transform_to_3d);
        Polyhedron patch_polyhedron = convex_hull_3_from_coplanar_points(patch_3d, surface.norm);

        if (params.cycle_detection_enabled &&
            cycle_path_detection(parent, child_stance_foot, surface.surface_id)) {
            continue;
        }

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
            child->perimeter = compute_polygon_perimeter(patch_polyhedron);
            child->centroid = get_centroid(patch_3d);

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
