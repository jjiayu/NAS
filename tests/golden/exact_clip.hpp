#pragma once
// Test oracle: the expansion's downstream of the plane cut (2D projection, hull,
// Sutherland-Hodgman clip against the surface) done in EXACT arithmetic.
// Independent of core/geometry's clip, so a robust clip can be checked
// against it.
#include "nas/core/geometry.hpp"
#include "nas/core/surface.hpp"

#include <CGAL/Cartesian_converter.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

#include <variant>
#include <vector>

namespace nas::oracle {

using EK = CGAL::Exact_predicates_exact_constructions_kernel;

// Same Sutherland-Hodgman as compute_2d_polygon_intersection, in exact
// arithmetic (inside test >= 0 and segment/line intersection both exact).
inline std::vector<Point_3> patch_from_cut_exact_clip(const std::vector<Point_3>& cut, const Surface& s) {
    if (cut.size() <= 2) return {};
    static CGAL::Cartesian_converter<Kernel, EK> conv;
    auto p2 = transform_3d_points_to_surface_plane(cut, s.transform_to_surface);
    Polygon_2 hull;
    CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
    std::vector<EK::Point_2> out;
    for (auto v = hull.vertices_begin(); v != hull.vertices_end(); ++v) out.push_back(conv(*v));
    std::vector<EK::Point_2> clip;
    for (const auto& v : s.vertices_2d) clip.push_back(conv(v));
    for (size_t k = 0; k < clip.size() && !out.empty(); ++k) {
        const auto& a = clip[k];
        const auto& b = clip[(k + 1) % clip.size()];
        std::vector<EK::Point_2> in = out;
        out.clear();
        EK::Line_2 line(a, b);
        for (size_t i = 0; i < in.size(); ++i) {
            const auto& cur = in[i];
            const auto& prev = in[(i + in.size() - 1) % in.size()];
            bool cin = CGAL::orientation(a, b, cur) != CGAL::RIGHT_TURN;
            bool pin = CGAL::orientation(a, b, prev) != CGAL::RIGHT_TURN;
            auto cross = [&] {
                auto r = CGAL::intersection(EK::Segment_2(prev, cur), line);
                if (r) if (const EK::Point_2* q = std::get_if<EK::Point_2>(&*r)) out.push_back(*q);
            };
            if (cin) { if (!pin) cross(); out.push_back(cur); }
            else if (pin) cross();
        }
    }
    if (out.size() <= 2) return {};
    std::vector<Point_2> back;
    for (const auto& q : out) back.emplace_back(CGAL::to_double(q.x()), CGAL::to_double(q.y()));
    return transform_2d_points_to_world(back, s.transform_to_3d);
}

} // namespace nas::oracle
