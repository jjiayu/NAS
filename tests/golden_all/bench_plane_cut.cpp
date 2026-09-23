// Compares three ways of cutting P_union with a surface plane (step 1 of
// expand_node), on the REAL parent states dumped by old_expansion_dump:
//   EDGE   - ours/old code: CGAL::intersection(plane, segment) on every edge
//   SLICER - CGAL::Polygon_mesh_slicer (AABB tree + triangle/plane cut)
//   EXACT  - the EDGE method in exact arithmetic (Epeck): the arbiter, since
//            inexact predicates are what make near-coplanar edges unstable
// All three see the SAME P_union (computed once per parent in this process),
// and each cut goes through the same downstream pipeline (2D projection,
// hull, Sutherland-Hodgman clip) so what is compared is the final patch
// polygon, like the real expansion. Also times each method.
// Usage: nas_bench_plane_cut <dump_dir> [repeat]

#include "nas/config/scenario.hpp"
#include "nas/core/geometry.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"

#include <CGAL/Cartesian_converter.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/convex_hull_2.h>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;
using json = nlohmann::json;
using EK = CGAL::Exact_predicates_exact_constructions_kernel;
using Clock = std::chrono::steady_clock;

namespace {

Point_3 to_pt(const json& j) { return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>()); }

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    return ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
}

// Downstream of step 1, identical to expand_node.
std::vector<Point_3> patch_from_cut(const std::vector<Point_3>& cut, const Surface& s) {
    if (cut.size() <= 2) return {};
    auto p2 = transform_3d_points_to_surface_plane(cut, s.transform_to_surface);
    Polygon_2 hull;
    CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
    std::vector<Point_2> hp(hull.vertices_begin(), hull.vertices_end());
    auto inter = compute_2d_polygon_intersection(hp, s.vertices_2d);
    if (inter.size() <= 2) return {};
    return transform_2d_points_to_world(inter, s.transform_to_3d);
}

std::vector<Point_3> cut_edge(const Surface& s, const EdgeList& edges) { return compute_edges_plane_intersection(s.plane, edges); }

std::vector<Point_3> cut_slicer(const Surface& s, CGAL::Polygon_mesh_slicer<Polyhedron, Kernel>& slicer) {
    std::vector<std::vector<Point_3>> polylines;
    slicer(s.plane, std::back_inserter(polylines));
    std::vector<Point_3> pts;
    for (const auto& pl : polylines) pts.insert(pts.end(), pl.begin(), pl.end());
    return pts;
}

std::vector<Point_3> cut_exact(const Surface& s, const std::vector<std::pair<EK::Point_3, EK::Point_3>>& edges) {
    EK::Plane_3 plane(s.plane.a(), s.plane.b(), s.plane.c(), s.plane.d());
    std::vector<Point_3> out;
    for (const auto& [p, q] : edges) {
        auto r = CGAL::intersection(plane, EK::Segment_3(p, q));
        if (!r) continue;
        if (const EK::Point_3* pt = std::get_if<EK::Point_3>(&*r))
            out.emplace_back(CGAL::to_double(pt->x()), CGAL::to_double(pt->y()), CGAL::to_double(pt->z()));
    }
    return out;
}

// Shape deviation of two patches (two-way Hausdorff of xy-hull boundaries,
// area difference); 1e300 when exactly one of them is empty.
double dev(const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
    if (a.empty() && b.empty()) return 0.0;
    if (a.empty() || b.empty()) return 1e300;
    auto hull = [](const std::vector<Point_3>& v) {
        std::vector<Point_2> p, h;
        for (const auto& q : v) p.emplace_back(CGAL::to_double(q.x()), CGAL::to_double(q.y()));
        CGAL::convex_hull_2(p.begin(), p.end(), std::back_inserter(h));
        return h;
    };
    auto area = [](const std::vector<Point_2>& h) {
        double s = 0;
        for (size_t i = 0; i < h.size(); ++i) {
            const auto& p = h[i]; const auto& q = h[(i + 1) % h.size()];
            s += CGAL::to_double(p.x() * q.y() - q.x() * p.y());
        }
        return std::abs(s) / 2;
    };
    auto bd = [](const Point_2& p, const std::vector<Point_2>& h) {
        if (h.size() == 1) return std::sqrt(CGAL::to_double(CGAL::squared_distance(p, h[0])));
        double best = 1e300;
        for (size_t i = 0; i < h.size(); ++i)
            best = std::min(best, std::sqrt(CGAL::to_double(CGAL::squared_distance(p, Segment_2(h[i], h[(i + 1) % h.size()])))));
        return best;
    };
    auto ha = hull(a), hb = hull(b);
    double d = 0;
    for (const auto& p : ha) d = std::max(d, bd(p, hb));
    for (const auto& p : hb) d = std::max(d, bd(p, ha));
    return std::max(d, std::abs(area(ha) - area(hb)));
}

struct Acc {
    long cases = 0, differ_from_exact = 0, presence_differs = 0;
    double max_dev = 0;
    double seconds = 0;
    void add(double d) {
        ++cases;
        if (d >= 1e299) { ++presence_differs; ++differ_from_exact; }
        else if (d > 1e-7) { ++differ_from_exact; max_dev = std::max(max_dev, d); }
    }
};

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " <dump_dir> [repeat]\n"; return 1; }
    int repeat = argc > 2 ? std::stoi(argv[2]) : 3;
    ReachabilityModel reach = make_forward_reachability();
    CGAL::Cartesian_converter<Kernel, EK> conv;
    const std::vector<std::string> scenes = {"NarrowPassage", "Stairs", "TwoFlatSurfaces", "LongStairs", "LongLongStairs", "Flat",
                                             "LongStairsComplete", "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down", "ThreePathsNAS"};
    std::printf("%-19s %6s | %-27s | %-27s | %s\n", "scene", "cuts", "EDGE vs EXACT  differ/max dev", "SLICER vs EXACT differ/max dev", "time per cut: edge / slicer / exact (us)");
    Acc tot_edge, tot_slicer;
    double tot_te = 0, tot_ts = 0, tot_tx = 0; long tot_n = 0;
    for (const auto& scene : scenes) {
        std::ifstream f(std::string(argv[1]) + "/" + scene + ".json");
        if (!f) continue;
        json dump; f >> dump;
        config::Scenario sc = config::load_scenario(scene);
        Acc ae, as;
        double te = 0, ts = 0, tx = 0; long n = 0;
        for (const auto& e : dump["expansions"]) {
            const json& pj = e["parent"];
            StanceFoot stance = pj["stance_foot"].get<int>() == 0 ? StanceFoot::Left : StanceFoot::Right;
            std::vector<Point_3> patch;
            for (const auto& v : pj["patch_vertices"]) patch.push_back(to_pt(v));
            Polyhedron base = reach.query(stance == StanceFoot::Left ? "RF" : "LF", stance == StanceFoot::Left ? "LF" : "RF", ReachabilityDirection::Forward);
            base = rotate_polyhedron_z(base, pj["foot_yaw"].get<double>());
            Polyhedron P = minkowski_sum(patch, base);
            EdgeList edges = polytope_edges(P);
            std::vector<std::pair<EK::Point_3, EK::Point_3>> xedges;
            for (const auto& [p, q] : edges) xedges.emplace_back(conv(p), conv(q));

            for (const auto& s : sc.surfaces) {
                // results (one pass)
                CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer(P);
                auto pe = patch_from_cut(cut_edge(s, edges), s);
                auto ps = patch_from_cut(cut_slicer(s, slicer), s);
                auto px = patch_from_cut(cut_exact(s, xedges), s);
                ae.add(dev(pe, px));
                as.add(dev(ps, px));
                ++n;
                // timings: step 1 only (the cut), repeated; slicer includes its per-P AABB build amortised per surface
                auto t0 = Clock::now();
                for (int r = 0; r < repeat; ++r) { volatile auto sz = cut_edge(s, edges).size(); (void)sz; }
                auto t1 = Clock::now();
                for (int r = 0; r < repeat; ++r) { CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> sl(P); volatile auto sz = cut_slicer(s, sl).size(); (void)sz; }
                auto t2 = Clock::now();
                for (int r = 0; r < repeat; ++r) { volatile auto sz = cut_exact(s, xedges).size(); (void)sz; }
                auto t3 = Clock::now();
                te += std::chrono::duration<double>(t1 - t0).count() / repeat;
                ts += std::chrono::duration<double>(t2 - t1).count() / repeat;
                tx += std::chrono::duration<double>(t3 - t2).count() / repeat;
            }
        }
        std::printf("%-19s %6ld | %5ld (%ld empty/non-empty) %.1e | %5ld (%ld empty/non-empty) %.1e | %.0f / %.0f / %.0f\n", scene.c_str(), n,
                    ae.differ_from_exact, ae.presence_differs, ae.max_dev, as.differ_from_exact, as.presence_differs, as.max_dev, te / n * 1e6, ts / n * 1e6, tx / n * 1e6);
        tot_edge.cases += ae.cases; tot_edge.differ_from_exact += ae.differ_from_exact; tot_edge.presence_differs += ae.presence_differs; tot_edge.max_dev = std::max(tot_edge.max_dev, ae.max_dev);
        tot_slicer.cases += as.cases; tot_slicer.differ_from_exact += as.differ_from_exact; tot_slicer.presence_differs += as.presence_differs; tot_slicer.max_dev = std::max(tot_slicer.max_dev, as.max_dev);
        tot_te += te; tot_ts += ts; tot_tx += tx; tot_n += n;
    }
    std::printf("TOTAL %ld cuts: EDGE differs from exact on %ld (%ld presence), max %.2e | SLICER differs on %ld (%ld presence), max %.2e | mean us/cut edge %.1f slicer %.1f exact %.1f\n",
                tot_n, tot_edge.differ_from_exact, tot_edge.presence_differs, tot_edge.max_dev, tot_slicer.differ_from_exact, tot_slicer.presence_differs,
                tot_slicer.max_dev, tot_te / tot_n * 1e6, tot_ts / tot_n * 1e6, tot_tx / tot_n * 1e6);
    return 0;
}
