// Directed tests for core/surface — same lightweight no-framework style as
// core/geometry/tests/test_geometry.cpp.

#include "nas/core/surface.hpp"

#include <cmath>
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

bool close(double a, double b, double eps = 1e-6) { return std::abs(a - b) < eps; }

} // namespace

int run_core_surface() {
    // A flat 2x2 square on the ground plane (z=0), centered at the origin.
    std::vector<Point_3> square = {
        Point_3(-1.0, -1.0, 0.0), Point_3(1.0, -1.0, 0.0),
        Point_3(1.0, 1.0, 0.0), Point_3(-1.0, 1.0, 0.0)
    };

    // Surface itself is csp::Surface (tested in cspplusplus); what NAS adds is make_surfaces.
    std::vector<Surface> surfaces = make_surfaces({square, square}, /*inner_margin=*/0.1);
    check(surfaces.size() == 2, "one Surface per raw list");
    const Surface& surf = surfaces[1];

    check(surf.surface_id == 1, "surface_id is the index in the returned vector");

    check(close(CGAL::to_double(surf.centroid.x()), 0.0) &&
          close(CGAL::to_double(surf.centroid.y()), 0.0) &&
          close(CGAL::to_double(surf.centroid.z()), 0.0),
          "centroid of a square centered at the origin is the origin");

    check(close(std::abs(CGAL::to_double(surf.norm.z())), 1.0),
          "normal of a flat ground-plane square points along +/-Z");

    // Footprint eroded by 0.1 on every side -> a (2 - 0.2) x (2 - 0.2) square.
    check(surf.vertices_2d.size() == 4, "eroded footprint of a square patch has 4 vertices");

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& v : surf.vertices_2d) {
        min_x = std::min(min_x, CGAL::to_double(v.x()));
        max_x = std::max(max_x, CGAL::to_double(v.x()));
        min_y = std::min(min_y, CGAL::to_double(v.y()));
        max_y = std::max(max_y, CGAL::to_double(v.y()));
    }
    check(close(max_x - min_x, 1.8, 1e-6), "footprint eroded by the margin on each side of x (2.0 -> 1.8)");
    check(close(max_y - min_y, 1.8, 1e-6), "footprint eroded by the margin on each side of y (2.0 -> 1.8)");

    // A list thinner than 2*margin is dropped, later ids shift down (surface_id == index).
    std::vector<Point_3> sliver = {Point_3(0, 0, 0), Point_3(3, 0, 0), Point_3(3, 0.1, 0), Point_3(0, 0.1, 0)};
    std::vector<Surface> kept = make_surfaces({sliver, square}, 0.1);
    check(kept.size() == 1 && kept[0].surface_id == 0, "a surface thinner than 2*margin is dropped and ids are re-numbered");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/surface tests passed\n";
    return 0;
}
