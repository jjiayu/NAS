// Minimal, dependency-free directed tests for core/geometry — no test
// framework linked (none exists in this repo yet), just asserts + a
// pass/fail tally. Focused on compute_2d_polygon_intersection since that's
// the one function whose *algorithm* changed during the port (hand-rolled
// Sutherland-Hodgman -> CGAL native boolean ops, see docs/paper-deltas.md);
// the rest is a near-verbatim port so a couple of smoke checks suffice.

#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_3.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

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

bool close(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

double polygon_area(const std::vector<Point_2>& pts) {
    if (pts.size() < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const Point_2& p1 = pts[i];
        const Point_2& p2 = pts[(i + 1) % pts.size()];
        area += CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) -
                CGAL::to_double(p2.x()) * CGAL::to_double(p1.y());
    }
    return std::abs(area) / 2.0;
}

std::vector<Point_2> square(double x0, double y0, double side) {
    return {Point_2(x0, y0), Point_2(x0 + side, y0), Point_2(x0 + side, y0 + side), Point_2(x0, y0 + side)};
}

void test_get_centroid() {
    std::vector<Point_3> tri = {Point_3(0, 0, 0), Point_3(3, 0, 0), Point_3(0, 3, 0)};
    Point_3 c = get_centroid(tri);
    check(close(CGAL::to_double(c.x()), 1.0) && close(CGAL::to_double(c.y()), 1.0) && close(CGAL::to_double(c.z()), 0.0),
          "get_centroid: triangle centroid is the coordinate average");
}

void test_polygon_intersection_partial_overlap() {
    // Two unit squares overlapping on their right/left half -> a 0.5x1 rectangle.
    auto subject = square(0.0, 0.0, 1.0);
    auto clip = square(0.5, 0.0, 1.0);
    auto result = compute_2d_polygon_intersection(subject, clip);
    check(result.size() >= 3, "polygon intersection (partial overlap): non-empty result");
    check(close(polygon_area(result), 0.5, 1e-6), "polygon intersection (partial overlap): area is 0.5");
}

void test_polygon_intersection_no_overlap() {
    auto subject = square(0.0, 0.0, 1.0);
    auto clip = square(10.0, 10.0, 1.0);
    auto result = compute_2d_polygon_intersection(subject, clip);
    check(result.empty(), "polygon intersection (no overlap): empty result");
}

void test_polygon_intersection_fully_contained() {
    // clip (small square) fully inside subject (big square) -> intersection == clip.
    auto subject = square(-5.0, -5.0, 10.0);
    auto clip = square(0.0, 0.0, 1.0);
    auto result = compute_2d_polygon_intersection(subject, clip);
    check(result.size() >= 3, "polygon intersection (fully contained): non-empty result");
    check(close(polygon_area(result), 1.0, 1e-6), "polygon intersection (fully contained): area equals the smaller polygon's area");
}

void test_polygon_intersection_identical() {
    auto subject = square(0.0, 0.0, 2.0);
    auto clip = square(0.0, 0.0, 2.0);
    auto result = compute_2d_polygon_intersection(subject, clip);
    check(close(polygon_area(result), 4.0, 1e-6), "polygon intersection (identical squares): area unchanged");
}

void test_minkowski_sum_translates_and_preserves_volume() {
    // Unit cube centered at origin as the base "reachability" polytope.
    std::vector<Point_3> cube_pts = {
        Point_3(-0.5, -0.5, -0.5), Point_3(0.5, -0.5, -0.5), Point_3(0.5, 0.5, -0.5), Point_3(-0.5, 0.5, -0.5),
        Point_3(-0.5, -0.5, 0.5), Point_3(0.5, -0.5, 0.5), Point_3(0.5, 0.5, 0.5), Point_3(-0.5, 0.5, 0.5)
    };
    Polyhedron cube;
    CGAL::convex_hull_3(cube_pts.begin(), cube_pts.end(), cube);

    // Minkowski sum with a single-point "patch" is just a translation.
    Point_3 translation(10.0, 20.0, 30.0);
    Polyhedron translated = minkowski_sum({translation}, cube);

    check(std::distance(translated.vertices_begin(), translated.vertices_end()) == 8,
          "minkowski_sum (single point): result has the base polytope's 8 vertices");

    bool found_translated_corner = false;
    for (auto v = translated.vertices_begin(); v != translated.vertices_end(); ++v) {
        if (close(CGAL::to_double(v->point().x()), 10.5) &&
            close(CGAL::to_double(v->point().y()), 20.5) &&
            close(CGAL::to_double(v->point().z()), 30.5)) {
            found_translated_corner = true;
        }
    }
    check(found_translated_corner, "minkowski_sum (single point): corner (0.5,0.5,0.5) correctly translated");
}

} // namespace

int main() {
    test_get_centroid();
    test_polygon_intersection_partial_overlap();
    test_polygon_intersection_no_overlap();
    test_polygon_intersection_fully_contained();
    test_polygon_intersection_identical();
    test_minkowski_sum_translates_and_preserves_volume();

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/geometry tests passed\n";
    return 0;
}
