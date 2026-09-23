#pragma once

// Pure geometric operations shared by every search strategy (NAS/Tree,
// CASSR/AstarSearch, the grid baseline) and by the footstep QP stage. No
// dependency on Node, on a search strategy, or on global configuration —
// every function here takes exactly the data it needs as parameters. See
// PLAN.md's "core/geometry" entry.

#include "nas/core/types.hpp"
#include <vector>
#include <Eigen/Dense>

namespace nas {

// Half-space polytope constraint representation: Ax <= b
struct HalfSpacePolytopeConstraint {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};

// Surface constraint representation (the first row is the on-surface plane
// equality constraint, remaining rows are the boundary inequalities)
struct SurfaceConstraint {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};

std::vector<Point_2> transform_3d_points_to_surface_plane(const std::vector<Point_3>& points, const Transformation& transformation);

std::vector<Point_3> transform_2d_points_to_world(const std::vector<Point_2>& points, const Transformation& inverse_transformation);

Point_3 get_centroid(const std::vector<Point_3>& points);

Polyhedron minkowski_sum(const std::vector<Point_3>& patch_vertices, const Polyhedron& polytope);

// Points where the plane crosses the polytope's edges (CGAL::intersection on
// each edge segment). Measured equivalent to CGAL::Polygon_mesh_slicer, a
// half-space cut and an exact-arithmetic cut on the cut polygon (docs/paper-deltas.md).
std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope);

// 2D polygon intersection (Sutherland-Hodgman clip). A CGAL-native
// replacement (CGAL::intersection on Polygon_2) was tried here first but
// reverted: it crashed (segfault inside CGAL's arrangement/surface-sweep
// code) on the real NarrowPassage scenario, whose "Passage" surface
// shrinks to a ~2cm-wide near-degenerate rectangle after the foot-size
// margin — see docs/paper-deltas.md "Tentatives abandonnées".
//
// Each crossing point is computed from the signed values that classified its
// two endpoints, so a segment classified as straddling the clip line always
// yields a point. The old code asked CGAL::intersection (exact predicates) for
// it and silently dropped the point when that disagreed with the double
// inside-test on a near-parallel edge (measured: 23 wrong patches in 19952
// cuts, up to 1 m; docs/paper-deltas.md).
std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& subject_polygon, const std::vector<Point_2>& clip_polygon);

double is_leftside_of_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end);

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location);

// Distance from a point to a convex patch (COAL EPA): the CASSR heuristic.
// The paper compares GJK/Euclidean variants; EPA is the one kept.
double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// Convert half-space polytope constraint to H-representation
HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope);

// Surface constraint in H-representation: plane equality (row 0) + one
// boundary inequality per polygon edge. The vertices are the patch polygon's,
// in order (either winding: each edge normal is oriented towards the vertex
// average).
SurfaceConstraint generate_surface_constraint(const std::vector<Point_3>& polygon_vertices);

// Rotate polyhedron around Z-axis by given angle in radians
Polyhedron rotate_polyhedron_z(const Polyhedron& polytope, double yaw_angle);

} // namespace nas
