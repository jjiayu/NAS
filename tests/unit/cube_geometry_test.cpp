// Directed tests for the cube-extension geometry primitives (see
// docs/cube-extension-spec.md §2-3, docs/cube-implementation-plan.md §2):
// minkowski_sum_tagged, compute_polytope_plane_intersection_tagged,
// convex_hull_2_tagged, compute_2d_polygon_intersection_tagged.

#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_3.h>
#include <iostream>
#include <string>

using namespace nas;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::cerr << "FAIL: " << what << "\n";
        ++g_failures;
    } else {
        std::cout << "ok: " << what << "\n";
    }
}

bool close(double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; }

Polyhedron make_box(double hx, double hy, double hz) {
    std::vector<Point_3> pts;
    for (double sx : {-hx, hx})
        for (double sy : {-hy, hy})
            for (double sz : {-hz, hz}) pts.push_back(Point_3(sx, sy, sz));
    Polyhedron box;
    CGAL::convex_hull_3(pts.begin(), pts.end(), box);
    return box;
}

// --- The spec's own §2 counterexample (docs/cube-extension-spec.md), ported as-is ---
//
// Z = [0,1] (foot patch). For a given z: cube top spans [z, z+0.4], next step spans
// [z+0.5, z+0.6] -- for every single z these are disjoint (0.4 < 0.5), so the action
// is genuinely always infeasible. The naive approach (sum each independently over all
// z, i.e. project away z first, THEN intersect) reports a false positive.
//
// The fix does not need any new "tagged" primitive here: it needs the *existing*
// compute_2d_polygon_intersection (2D polygon-vs-polygon clip), applied to the two
// regions {(z,a): z in Z, a-z in [0,0.4]} and {(z,b): z in Z, b-z in [0.5,0.6]} in the
// SHARED (z, position) plane -- i.e. never projecting z away before intersecting. That
// is the whole lesson of the counterexample: the bug is in projecting the coupling
// variable away too early, not a deficiency of the existing 2D clip itself.
int test_counterexample() {
    // J_A = {(z,a): 0<=z<=1, 0<=a-z<=0.4}: vertices (0,0),(1,1),(1,1.4),(0,0.4)
    std::vector<Point_2> j_a = {Point_2(0, 0), Point_2(1, 1), Point_2(1, 1.4), Point_2(0, 0.4)};
    // J_B = {(z,b): 0<=z<=1, 0.5<=b-z<=0.6}: vertices (0,0.5),(1,1.5),(1,1.6),(0,0.6)
    std::vector<Point_2> j_b = {Point_2(0, 0.5), Point_2(1, 1.5), Point_2(1, 1.6), Point_2(0, 0.6)};

    // Naive: project each region onto the position axis alone (throw away z), then
    // intersect the two resulting 1D intervals -- exactly what independently computing
    // "all reachable cube tops" and "all reachable next-step positions" and intersecting
    // them amounts to.
    auto project_a = [](const std::vector<Point_2>& poly) {
        double lo = 1e9, hi = -1e9;
        for (const auto& p : poly) { lo = std::min(lo, CGAL::to_double(p.y())); hi = std::max(hi, CGAL::to_double(p.y())); }
        return std::make_pair(lo, hi);
    };
    auto [a_lo, a_hi] = project_a(j_a);
    auto [b_lo, b_hi] = project_a(j_b);
    double naive_lo = std::max(a_lo, b_lo), naive_hi = std::min(a_hi, b_hi);
    check(naive_lo <= naive_hi, "naive (project-then-intersect) gives a false-positive non-empty overlap");
    check(close(naive_lo, 0.5) && close(naive_hi, 1.4), "naive false positive matches the spec's own numbers ([0.5, 1.4])");

    // Correct: keep z explicit, intersect the two regions as 2D polygons in the shared
    // (z, position) plane. No new geometry code -- this is the point.
    std::vector<Point_2> correct = compute_2d_polygon_intersection(j_a, j_b);
    check(correct.size() <= 2, "correct (joint 2D intersection) is empty or degenerate -- no feasible shared z");

    return g_failures;
}

// --- minkowski_sum_tagged: payload survives translate+hull ---
int test_minkowski_sum_tagged() {
    int before = g_failures;
    // Two "patch" vertices (distinguishable by a payload each), translated by a small box.
    std::vector<Point_3> patch = {Point_3(0, 0, 0), Point_3(10, 0, 0)};
    std::vector<Point_3> payloads = {Point_3(-1, -1, -1), Point_3(-2, -2, -2)}; // arbitrary markers
    Polyhedron box = make_box(0.5, 0.5, 0.5);

    TaggedPolyhedron result = minkowski_sum_tagged(patch, payloads, box);
    check(result.vertices.size() == result.payloads.size(), "minkowski_sum_tagged: one payload per surviving vertex");
    check(std::distance(result.mesh.vertices_begin(), result.mesh.vertices_end()) == static_cast<long>(result.vertices.size()),
          "minkowski_sum_tagged: parallel arrays match the hull's actual vertex count");

    // The box is small (radius 0.5) relative to the 10-unit gap between the two patch
    // points, so the hull cleanly separates into two clusters: every vertex near x=0
    // must carry payloads[0], every vertex near x=10 must carry payloads[1].
    bool all_correct = true;
    for (size_t i = 0; i < result.vertices.size(); ++i) {
        double x = CGAL::to_double(result.vertices[i].x());
        const Point_3& expected = (x < 5.0) ? payloads[0] : payloads[1];
        if (result.payloads[i] != expected) all_correct = false;
    }
    check(all_correct, "minkowski_sum_tagged: every hull vertex carries the payload of the patch vertex it descends from");

    return g_failures - before;
}

// --- compute_polytope_plane_intersection_tagged: payload interpolates correctly ---
int test_plane_intersection_tagged() {
    int before = g_failures;
    // A single patch vertex at the origin with payload (100,100,100), translated by a
    // unit box. Slicing z=0 through the box's vertical edges must interpolate the
    // (constant, since there's only one patch vertex) payload back to itself exactly.
    std::vector<Point_3> patch = {Point_3(0, 0, 0)};
    std::vector<Point_3> payloads = {Point_3(100, 100, 100)};
    Polyhedron box = make_box(1.0, 1.0, 1.0); // spans z in [-1, 1]

    TaggedPolyhedron tp = minkowski_sum_tagged(patch, payloads, box);
    Plane_3 z0(0, 0, 1, 0); // z = 0
    std::vector<TaggedPoint3> cut = compute_polytope_plane_intersection_tagged(z0, tp);
    // CGAL::convex_hull_3 triangulates the box's 6 square faces into 12 triangles (Euler:
    // V=8, F=12 => E=18 distinct edges, not the 12 of the untriangulated box), so z=0
    // crosses more than just the 4 "vertical" edges an untriangulated box would suggest --
    // matches what the existing (untagged) compute_polytope_plane_intersection gives on
    // the same box+plane (measured: 8, not 4).
    check(cut.size() == 8, "plane_intersection_tagged: z=0 cut count matches the untagged function on the same box+plane");
    bool all_on_plane = true, all_payload_preserved = true;
    for (const auto& tpt : cut) {
        if (!close(CGAL::to_double(tpt.point.z()), 0.0)) all_on_plane = false;
        if (tpt.payload != Point_3(100, 100, 100)) all_payload_preserved = false;
    }
    check(all_on_plane, "plane_intersection_tagged: every cut point actually lies on the cutting plane");
    check(all_payload_preserved, "plane_intersection_tagged: a single-source payload survives the cut unchanged");

    // The real invariant this primitive exists to preserve (spec §3.1: c in z (+) K_cube,
    // i.e. c - z in K_cube): use payload = patch vertex itself (payload IS the originating
    // z, not an arbitrary label), with patch vertices far enough apart that the hull of
    // their two box-sweeps includes genuine new "bridging" vertices between them (not just
    // the original 8+8 box corners) -- exactly the kind of point a real, spread-out foot
    // patch would produce. For EVERY resulting point, including bridging ones, point -
    // payload must land back inside the original box: that's the coupling invariant, and
    // it must hold whether or not the point is an original box corner.
    std::vector<Point_3> patch2 = {Point_3(0, 0, 0), Point_3(10, 0, 0)};
    std::vector<Point_3> payloads2 = patch2; // payload = z itself
    TaggedPolyhedron tp2 = minkowski_sum_tagged(patch2, payloads2, box);
    std::vector<TaggedPoint3> cut2 = compute_polytope_plane_intersection_tagged(z0, tp2);
    bool coupling_holds = true;
    bool found_bridging_point = false;
    for (const auto& tpt : cut2) {
        double dx = CGAL::to_double(tpt.point.x() - tpt.payload.x());
        double dy = CGAL::to_double(tpt.point.y() - tpt.payload.y());
        double dz = CGAL::to_double(tpt.point.z() - tpt.payload.z());
        if (dx < -1.0 - 1e-9 || dx > 1.0 + 1e-9 || dy < -1.0 - 1e-9 || dy > 1.0 + 1e-9 || dz < -1.0 - 1e-9 || dz > 1.0 + 1e-9)
            coupling_holds = false;
        // A "bridging" point: its payload (the z it's coupled to) is neither original patch
        // vertex, i.e. it's an interpolated z strictly between them.
        if (tpt.payload != payloads2[0] && tpt.payload != payloads2[1]) found_bridging_point = true;
    }
    check(coupling_holds, "plane_intersection_tagged: point - payload stays inside K_cube for every point, including bridging ones");
    check(found_bridging_point, "plane_intersection_tagged: at least one point genuinely has an interpolated (non-original) z payload");

    return g_failures - before;
}

// --- convex_hull_2_tagged: payload survives 2D hull ---
int test_convex_hull_2_tagged() {
    int before = g_failures;
    // A square with one interior point that must NOT survive the hull.
    std::vector<TaggedPoint2> pts = {
        {Point_2(0, 0), Point_2(0, 0)},
        {Point_2(1, 0), Point_2(1, 0)},
        {Point_2(1, 1), Point_2(1, 1)},
        {Point_2(0, 1), Point_2(0, 1)},
        {Point_2(0.5, 0.5), Point_2(9, 9)}, // interior point, distinctive payload
    };
    std::vector<TaggedPoint2> hull = convex_hull_2_tagged(pts);
    check(hull.size() == 4, "convex_hull_2_tagged: drops the interior point, keeps the 4 corners");
    bool interior_dropped = true;
    for (const auto& h : hull) {
        if (h.payload == Point_2(9, 9)) interior_dropped = false;
    }
    check(interior_dropped, "convex_hull_2_tagged: the interior point's payload does not leak onto the hull");
    bool payload_matches_point = true;
    for (const auto& h : hull) {
        if (h.payload != h.point) payload_matches_point = false; // for the 4 corners, payload was set equal to point
    }
    check(payload_matches_point, "convex_hull_2_tagged: surviving corners keep their own (point-equal) payload");

    return g_failures - before;
}

// --- compute_2d_polygon_intersection_tagged with a derived classify coordinate ---
// Mirrors the spec's §3.3 "step on the cube" cut: clip on (point - payload) instead of
// point itself, matching x' - c against carre_cube, while still outputting/interpolating
// the real (point, payload) pair.
int test_clip_tagged_derived_coordinate() {
    int before = g_failures;
    // Subject: a thin rectangle in "point" space, x from 0 to 2. Payload is offset from
    // point so that the derived coordinate (point - payload) sweeps from (-0.7, 0) at
    // x=0 (outside the clip square below) to (0.3, 0) at x=2 (inside it) -- forcing a
    // genuine crossing to interpolate, like the spec's x' - c crossing carre_cube (§3.3).
    std::vector<TaggedPoint2> subject = {
        {Point_2(0, -0.1), Point_2(0.7, -0.1)},  // point - payload = (-0.7, 0) : outside
        {Point_2(2, -0.1), Point_2(1.7, -0.1)},  // point - payload = (0.3, 0)  : inside
        {Point_2(2, 0.1), Point_2(1.7, 0.1)},    // point - payload = (0.3, 0)  : inside
        {Point_2(0, 0.1), Point_2(0.7, 0.1)},    // point - payload = (-0.7, 0) : outside
    };
    // Clip square: [-0.5, 0.5] x [-0.5, 0.5] in the derived (point-payload) space.
    std::vector<Point_2> clip_square = {Point_2(-0.5, -0.5), Point_2(0.5, -0.5), Point_2(0.5, 0.5), Point_2(-0.5, 0.5)};

    auto classify = [](const TaggedPoint2& tp) { return Point_2(tp.point.x() - tp.payload.x(), tp.point.y() - tp.payload.y()); };
    std::vector<TaggedPoint2> clipped = compute_2d_polygon_intersection_tagged(subject, clip_square, classify);

    check(clipped.size() == 4, "clip_tagged (derived coord): 2 original inside points + 2 new interpolated crossings");
    bool all_inside_derived = true;
    for (const auto& tp : clipped) {
        Point_2 d = classify(tp);
        if (CGAL::to_double(d.x()) > 0.5 + 1e-9 || CGAL::to_double(d.x()) < -0.5 - 1e-9) all_inside_derived = false;
    }
    check(all_inside_derived, "clip_tagged (derived coord): every surviving point's derived coordinate is within the clip square");

    // The two original x=2 vertices must survive completely unmodified (both point and
    // payload), and the two new crossing points must land where hand computation puts
    // them: derived x crosses -0.5 at t=(-0.5-(-0.7))/(0.3-(-0.7))=0.2 along each edge,
    // i.e. point.x = 0 + 0.2*(2-0) = 0.4, point.y unchanged (edges are horizontal), and
    // payload interpolates the same way: 0.7 + 0.2*(1.7-0.7) = 0.9.
    bool survivors_exact = false, new_point_correct = false, new_payload_correct = false;
    for (const auto& tp : clipped) {
        if (tp.point == Point_2(2, -0.1) && tp.payload == Point_2(1.7, -0.1)) survivors_exact = true;
        if (close(CGAL::to_double(tp.point.x()), 0.4) && close(CGAL::to_double(tp.point.y()), -0.1)) new_point_correct = true;
        if (close(CGAL::to_double(tp.payload.x()), 0.9) && close(CGAL::to_double(tp.payload.y()), -0.1)) new_payload_correct = true;
    }
    check(survivors_exact, "clip_tagged (derived coord): an untouched inside vertex keeps its exact point and payload");
    check(new_point_correct, "clip_tagged (derived coord): the interpolated crossing's point matches hand computation (x=0.4)");
    check(new_payload_correct, "clip_tagged (derived coord): the interpolated crossing's payload uses the same t as the point (x=0.9)");

    return g_failures - before;
}

} // namespace

int run_cube_geometry() {
    test_counterexample();
    test_minkowski_sum_tagged();
    test_plane_intersection_tagged();
    test_convex_hull_2_tagged();
    test_clip_tagged_derived_coordinate();

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All cube-geometry tests passed\n";
    return 0;
}
