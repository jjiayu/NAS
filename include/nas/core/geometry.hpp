#pragma once

// Pure geometric operations shared by every search strategy (NAS/Tree,
// CASSR/AstarSearch, the grid baseline) and by the footstep QP stage. No
// dependency on Node, on a search strategy, or on global configuration —
// every function here takes exactly the data it needs as parameters. See
// PLAN.md's "core/geometry" entry.

#include "nas/core/types.hpp"
#include <functional>
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
// (The old code also had a GJK variant; dropped.)
double calculate_epa_distance_point_to_patch(const std::vector<Point_3>& patch_points, const Point_3& goal);

// Same between two convex planar patches (the paper, V-B.5: "either the target or the node can be described as a
// polytope"): 0 when they touch or overlap, the minimum distance otherwise. Used when the goal is a surface.
double calculate_epa_distance_patch_to_patch(const std::vector<Point_3>& patch_a, const std::vector<Point_3>& patch_b);

// Convert half-space polytope constraint to H-representation
HalfSpacePolytopeConstraint convert_polytope_to_half_space_constraint(const Polyhedron& polytope);

// Surface constraint in H-representation: plane equality (row 0) + one
// boundary inequality per polygon edge. The vertices are the patch polygon's,
// in order (either winding: each edge normal is oriented towards the vertex
// average).
SurfaceConstraint generate_surface_constraint(const std::vector<Point_3>& polygon_vertices);

// Rotate polyhedron around Z-axis by given angle in radians
Polyhedron rotate_polyhedron_z(const Polyhedron& polytope, double yaw_angle);

// Rotation of a foot's frame in the world: the paper's Q (Eq. 2, "the rotation matrix that matches the
// current yaw and contact surface rotation"). The reachability polytopes are expressed in the frame of the
// support foot, z up along the contact surface's normal: Q = R_tilt * R_z(yaw), with R_tilt the minimal rotation
// taking the world z axis to the surface's up normal (the normal is flipped to point up if needed). For a
// horizontal surface Q = R_z(yaw).
Eigen::Matrix3d foot_frame_rotation(const Vector_3& surface_normal, double yaw);

// True when a normal is (numerically) vertical: Q is then a pure yaw rotation, and callers use the exact
// rotate_polyhedron_z path (bit-identical to the flat-scene behaviour).
bool is_vertical_normal(const Vector_3& normal);

// Rotates every vertex of a polyhedron by R (general counterpart of rotate_polyhedron_z).
Polyhedron rotate_polyhedron(const Polyhedron& polytope, const Eigen::Matrix3d& R);

// --- Cube-extension support: payload-carrying primitives ---
// See docs/cube-extension-spec.md §2-3 and docs/cube-implementation-plan.md §2:
// the joint-state polytope needed to keep a foot position and a derived cube
// position correlated (instead of losing that correlation the way two
// independently-computed Minkowski sums would, per the spec's own 1D
// counterexample) never needs real n-dimensional convex-hull/halfspace
// machinery. Every stage of the existing footstep-patch pipeline
// (minkowski_sum -> compute_polytope_plane_intersection -> 2D hull ->
// compute_2d_polygon_intersection -> 2D hull) is either a subset selection
// (a convex hull's vertices are always among its input points, never
// synthesized) or an explicit affine interpolation already parameterized by a
// scalar t (a plane/segment or clip-edge crossing) -- both generalize
// cleanly to carry an extra "payload" point through unchanged. These sibling
// functions do exactly that, next to the existing (unmodified) ones so every
// other call site is untouched.

struct TaggedPoint2 {
    Point_2 point;
    Point_2 payload;
};

struct TaggedPoint3 {
    Point_3 point;
    Point_3 payload;
};

// A 3D convex hull (as minkowski_sum computes) together with, for every
// surviving vertex, the payload of the specific (patch_vertex, polytope_vertex)
// pair it was translated from.
struct TaggedPolyhedron {
    Polyhedron mesh;
    std::vector<Point_3> vertices; // parallel arrays: mesh's vertex points ...
    std::vector<Point_3> payloads; // ... and their payload, same order/size
};

// minkowski_sum, but payloads[i] is carried onto every hull vertex descended
// from patch_vertices[i] (patch_vertices and payloads must be the same size).
TaggedPolyhedron minkowski_sum_tagged(const std::vector<Point_3>& patch_vertices,
                                       const std::vector<Point_3>& payloads,
                                       const Polyhedron& polytope);

// compute_polytope_plane_intersection, but interpolating tp's payload along
// each cut edge with the same parameter t as the position itself. Uses an
// explicit double-precision t (not CGAL::intersection) on purpose, like
// compute_2d_polygon_intersection's own push_crossing below -- consistent
// with this file's established preference for that style over CGAL-native
// intersection on this kind of cut (see that function's header comment).
std::vector<TaggedPoint3> compute_polytope_plane_intersection_tagged(const Plane_3& plane, const TaggedPolyhedron& tp);

// CGAL::convex_hull_2, but keeping the payload of whichever input point
// survives onto the hull (2D convex hull never synthesizes new points, same
// subset property as the 3D case above).
std::vector<TaggedPoint2> convex_hull_2_tagged(const std::vector<TaggedPoint2>& points);

// compute_2d_polygon_intersection (Sutherland-Hodgman), but classifying/cutting
// against classify_coord(subject_point) instead of subject_point itself (e.g.
// point - payload for the spec's on-cube-step cut, §3.3), while carrying (and,
// on a cut edge, interpolating with the exact same t as the classify
// coordinate) both point and payload. Pass `[](const TaggedPoint2& p){ return
// p.point; }` to clip on the position itself, same behaviour as the untagged
// function.
std::vector<TaggedPoint2> compute_2d_polygon_intersection_tagged(
    const std::vector<TaggedPoint2>& subject_polygon,
    const std::vector<Point_2>& clip_polygon,
    const std::function<Point_2(const TaggedPoint2&)>& classify_coord);

} // namespace nas
