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

    // Surface itself is csp::Surface (tested in cspplusplus); what NAS adds is make_surfaces
    // (raw fit, id == index) and erode_by_id/erode_surfaces (margin).
    std::vector<Surface> raw = make_surfaces({square, square});
    check(raw.size() == 2 && raw[1].surface_id == 1, "make_surfaces: one raw Surface per list, id is the index");

    auto extents = [](const Surface& s) {
        double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
        for (const auto& v : s.vertices_2d) {
            min_x = std::min(min_x, CGAL::to_double(v.x()));
            max_x = std::max(max_x, CGAL::to_double(v.x()));
            min_y = std::min(min_y, CGAL::to_double(v.y()));
            max_y = std::max(max_y, CGAL::to_double(v.y()));
        }
        return std::make_pair(max_x - min_x, max_y - min_y);
    };
    check(close(extents(raw[0]).first, 2.0) && close(extents(raw[0]).second, 2.0), "a raw surface keeps its full footprint (2 x 2)");
    check(close(CGAL::to_double(raw[1].centroid.x()), 0.0) && close(CGAL::to_double(raw[1].centroid.y()), 0.0) &&
              close(CGAL::to_double(raw[1].centroid.z()), 0.0),
          "centroid of a square centered at the origin is the origin");
    check(close(std::abs(CGAL::to_double(raw[1].norm.z())), 1.0), "normal of a flat ground-plane square points along +/-Z");

    // Footprint eroded by 0.1 on every side -> a (2 - 0.2) x (2 - 0.2) square.
    auto eroded = erode_by_id(raw, 0.1);
    check(eroded.size() == 2 && eroded[1] && eroded[1]->surface_id == 1, "erode_by_id keeps the id");
    check(eroded[1]->vertices_2d.size() == 4, "eroded footprint of a square patch has 4 vertices");
    check(close(extents(*eroded[1]).first, 1.8) && close(extents(*eroded[1]).second, 1.8),
          "footprint eroded by the margin on each side (2.0 -> 1.8)");

    // A surface thinner than 2*margin collapses: nullopt (slot kept) / dropped (compact), ids untouched.
    std::vector<Point_3> sliver = {Point_3(0, 0, 0), Point_3(3, 0, 0), Point_3(3, 0.1, 0), Point_3(0, 0.1, 0)};
    std::vector<Surface> mixed = make_surfaces({sliver, square});
    auto by_id = erode_by_id(mixed, 0.1);
    check(by_id.size() == 2 && !by_id[0] && by_id[1], "erode_by_id: a surface thinner than 2*margin is nullopt, its slot is kept");
    std::vector<Surface> compact = erode_surfaces(mixed, 0.1);
    check(compact.size() == 1 && compact[0].surface_id == 1, "erode_surfaces drops it and the others keep their id");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/surface tests passed\n";
    return 0;
}
