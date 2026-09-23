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

// Ordered edge list of a polyhedron (edges_begin order, each as
// (vertex(), opposite()->vertex())). The plane/polytope intersection below is
// a pure function of this sequence, so it is exposed to let tests replay the
// exact hull triangulation an older run produced.
using EdgeList = std::vector<std::pair<Point_3, Point_3>>;
EdgeList polytope_edges(const Polyhedron& polytope);
std::vector<Point_3> compute_edges_plane_intersection(const Plane_3& plane, const EdgeList& edges);

std::vector<Point_3> compute_polytope_plane_intersection(const Plane_3& plane, const Polyhedron& polytope);

// 2D polygon intersection (Sutherland-Hodgman clip). A CGAL-native
// replacement (CGAL::intersection on Polygon_2) was tried here first but
// reverted: it crashed (segfault inside CGAL's arrangement/surface-sweep
// code) on the real NarrowPassage scenario, whose "Passage" surface
// shrinks to a ~2cm-wide near-degenerate rectangle after the foot-size
// margin — see docs/paper-deltas.md "Tentatives abandonnées".
std::vector<Point_2> compute_2d_polygon_intersection(const std::vector<Point_2>& subject_polygon, const std::vector<Point_2>& clip_polygon);

double is_leftside_of_edge(const Point_2& point, const Point_2& edge_start, const Point_2& edge_end);

double compute_polygon_perimeter(const Polyhedron& polyhedron);

double compute_euclidean_distance(const Point_3& start_location, const Point_3& end_location);

// COAL GJK distance computation function
double calculate_gjk_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// COAL EPA distance computation function
double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// Convert half-space polytope constraint to H-representation
HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope);

// Convert surface constraint to H-representation
SurfaceConstraint generate_surface_constraint(const Polyhedron& surface_3d);

// Rotate polyhedron around Z-axis by given angle in radians
Polyhedron rotate_polyhedron_z(const Polyhedron& polytope, double yaw_angle);

// Build a polyhedron from coplanar points (convex_hull_3 fails on degenerate input)
Polyhedron convex_hull_3_from_coplanar_points(const std::vector<Point_3>& points, const Vector_3& normal);

} // namespace nas
