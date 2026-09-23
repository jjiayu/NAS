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
//   HALF   - direct cut: intersect the plane with the polytope's facet
//            half-spaces (2D Sutherland-Hodgman of a big square, in the surface
//            frame, against each facet's half-plane), no edge walking, so no
//            dependence on how the hull triangulates a facet. A facet plane is
//            normalised so the inside test uses a tolerance in metres.
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

#include <array>
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

// Same Sutherland-Hodgman as compute_2d_polygon_intersection, in exact
// arithmetic (inside test >= 0 and segment/line intersection both exact).
std::vector<Point_3> patch_from_cut_exact_clip(const std::vector<Point_3>& cut, const Surface& s) {
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

// Instrumented copy of compute_2d_polygon_intersection (same logic) that
// reports what happens at each clip edge, to find the failing mechanism.
struct ClipTrace { long crossings = 0, crossing_no_point = 0, crossing_segment = 0, crossing_empty = 0; };
std::vector<Point_2> traced_clip(const std::vector<Point_2>& subject, const std::vector<Point_2>& clip, ClipTrace& tr, bool verbose) {
    std::vector<Point_2> out = subject;
    for (size_t k = 0; k < clip.size(); ++k) {
        if (out.empty()) return {};
        const Point_2 a = clip[k], b = clip[(k + 1) % clip.size()];
        std::vector<Point_2> in = out;
        out.clear();
        Line_2 line(a, b);
        for (size_t i = 0; i < in.size(); ++i) {
            Point_2 cur = in[i], prev = in[(i + in.size() - 1) % in.size()];
            bool cin = is_leftside_of_edge(cur, a, b) >= 0, pin = is_leftside_of_edge(prev, a, b) >= 0;
            auto cross = [&] {
                ++tr.crossings;
                auto r = CGAL::intersection(Segment_2(prev, cur), line);
                Point_2 q;
                if (!r) { ++tr.crossing_empty; ++tr.crossing_no_point; if (verbose) std::printf("   clip edge %zu: prev(%.17g,%.17g,%s) cur(%.17g,%.17g,%s): intersection EMPTY -> point lost\n", k, CGAL::to_double(prev.x()), CGAL::to_double(prev.y()), pin ? "in" : "out", CGAL::to_double(cur.x()), CGAL::to_double(cur.y()), cin ? "in" : "out"); }
                else if (CGAL::assign(q, *r)) out.push_back(q);
                else { ++tr.crossing_segment; ++tr.crossing_no_point; if (verbose) std::printf("   clip edge %zu: intersection is a SEGMENT -> point lost\n", k); }
            };
            if (cin) { if (!pin) cross(); out.push_back(cur); }
            else if (pin) cross();
        }
    }
    return out;
}

// Candidate fix: same Sutherland-Hodgman, but the crossing point is computed
// from the very signed values that decided "inside/outside", so a crossing
// classified by the double test always yields a point (no second, exact
// predicate that can disagree).
std::vector<Point_3> patch_from_cut_fixed_clip(const std::vector<Point_3>& cut, const Surface& s) {
    if (cut.size() <= 2) return {};
    auto p2 = transform_3d_points_to_surface_plane(cut, s.transform_to_surface);
    Polygon_2 hull;
    CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
    std::vector<Point_2> out(hull.vertices_begin(), hull.vertices_end());
    const auto& clip = s.vertices_2d;
    for (size_t k = 0; k < clip.size() && !out.empty(); ++k) {
        const Point_2 a = clip[k], b = clip[(k + 1) % clip.size()];
        std::vector<Point_2> in = out;
        out.clear();
        for (size_t i = 0; i < in.size(); ++i) {
            const Point_2& cur = in[i];
            const Point_2& prev = in[(i + in.size() - 1) % in.size()];
            double fc = is_leftside_of_edge(cur, a, b), fp = is_leftside_of_edge(prev, a, b);
            bool cin = fc >= 0, pin = fp >= 0;
            auto cross = [&] {
                double t = fp / (fp - fc);
                out.emplace_back(CGAL::to_double(prev.x()) + t * CGAL::to_double(cur.x() - prev.x()),
                                 CGAL::to_double(prev.y()) + t * CGAL::to_double(cur.y() - prev.y()));
            };
            if (cin) { if (!pin) cross(); out.push_back(cur); }
            else if (pin) cross();
        }
    }
    if (out.size() <= 2) return {};
    return transform_2d_points_to_world(out, s.transform_to_3d);
}

struct FacetPlanes { std::vector<std::array<double, 4>> p; }; // unit normal (a,b,c), d: inside <=> a x+b y+c z+d <= eps

FacetPlanes facet_planes(const Polyhedron& P) {
    FacetPlanes out;
    for (auto f = P.facets_begin(); f != P.facets_end(); ++f) {
        auto h = f->halfedge();
        const Point_3& a = h->vertex()->point();
        const Point_3& b = h->next()->vertex()->point();
        const Point_3& c = h->next()->next()->vertex()->point();
        // hull facets are seen counter-clockwise from outside -> outward normal
        Vector_3 n = CGAL::cross_product(b - a, c - a);
        double len = std::sqrt(CGAL::to_double(n.squared_length()));
        if (len < 1e-14) continue; // degenerate sliver
        double nx = CGAL::to_double(n.x()) / len, ny = CGAL::to_double(n.y()) / len, nz = CGAL::to_double(n.z()) / len;
        double d = -(nx * CGAL::to_double(a.x()) + ny * CGAL::to_double(a.y()) + nz * CGAL::to_double(a.z()));
        bool dup = false;
        for (const auto& q : out.p)
            if (std::abs(q[0] - nx) < 1e-12 && std::abs(q[1] - ny) < 1e-12 && std::abs(q[2] - nz) < 1e-12 && std::abs(q[3] - d) < 1e-12) { dup = true; break; }
        if (!dup) out.p.push_back({nx, ny, nz, d});
    }
    return out;
}

constexpr double HALF_EPS = 1e-9; // metres

std::vector<Point_3> cut_halfspace(const Surface& s, const FacetPlanes& fp) {
    // 2D frame of the surface: 3D point = T(x, y, 0). Each half-space is
    // affine in (x, y): coefficients from the images of the origin and axes.
    const Transformation& T = s.transform_to_3d;
    Point_3 o = T(Point_3(0, 0, 0)), ex = T(Point_3(1, 0, 0)), ey = T(Point_3(0, 1, 0));
    double ox = CGAL::to_double(o.x()), oy = CGAL::to_double(o.y()), oz = CGAL::to_double(o.z());
    double ux = CGAL::to_double(ex.x()) - ox, uy = CGAL::to_double(ex.y()) - oy, uz = CGAL::to_double(ex.z()) - oz;
    double vx = CGAL::to_double(ey.x()) - ox, vy = CGAL::to_double(ey.y()) - oy, vz = CGAL::to_double(ey.z()) - oz;
    const double R = 20.0; // larger than any reachability polytope
    std::vector<std::array<double, 2>> poly = {{-R, -R}, {R, -R}, {R, R}, {-R, R}};
    for (const auto& q : fp.p) {
        double c0 = q[0] * ox + q[1] * oy + q[2] * oz + q[3];
        double cu = q[0] * ux + q[1] * uy + q[2] * uz, cv = q[0] * vx + q[1] * vy + q[2] * vz;
        std::vector<std::array<double, 2>> next;
        size_t n = poly.size();
        for (size_t i = 0; i < n; ++i) {
            const auto& A = poly[(i + n - 1) % n];
            const auto& B = poly[i];
            double fa = c0 + cu * A[0] + cv * A[1], fb = c0 + cu * B[0] + cv * B[1];
            bool ina = fa <= HALF_EPS, inb = fb <= HALF_EPS;
            if (inb != ina) {
                double t = fa / (fa - fb);
                next.push_back({A[0] + t * (B[0] - A[0]), A[1] + t * (B[1] - A[1])});
            }
            if (inb) next.push_back(B);
        }
        poly.swap(next);
        if (poly.empty()) return {};
    }
    std::vector<Point_3> out;
    for (const auto& v : poly) out.push_back(T(Point_3(v[0], v[1], 0)));
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
    long cases = 0, differ = 0, presence = 0;
    double max_dev = 0;
    void add(double d) {
        ++cases;
        if (d >= 1e299) { ++presence; ++differ; }
        else if (d > 1e-7) { ++differ; max_dev = std::max(max_dev, d); }
    }
    void merge(const Acc& o) { cases += o.cases; differ += o.differ; presence += o.presence; max_dev = std::max(max_dev, o.max_dev); }
};

// Two measurements per method, both against EXACT:
//   cut   - the step-1 output itself (hull of the cut points), isolates the cut;
//   patch - the final patch after the shared downstream (hull + Sutherland-Hodgman
//           clip against the surface), which adds the clip's own sensitivity
//           (its inside test has no tolerance, so a patch edge lying on a
//           surface edge can flip on rounding alone).
constexpr int NM = 4;
long g_fixed_wrong = 0, g_fixed_presence = 0; double g_fixed_max = 0;
long g_differs_without_lost_point = 0, g_differs_with_lost_point = 0, g_ok_with_lost_point = 0;
const char* kNames[NM] = {"EDGE", "SLICER", "EXACT", "HALF"};

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " <dump_dir> [repeat]\n"; return 1; }
    int repeat = argc > 2 ? std::stoi(argv[2]) : 3;
    ReachabilityModel reach = make_forward_reachability();
    CGAL::Cartesian_converter<Kernel, EK> conv;
    const std::vector<std::string> scenes = {"NarrowPassage", "Stairs", "TwoFlatSurfaces", "LongStairs", "LongLongStairs", "Flat",
                                             "LongStairsComplete", "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down", "ThreePathsNAS"};
    auto cutdev = [](const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
        return dev(a.size() > 2 ? a : std::vector<Point_3>{}, b.size() > 2 ? b : std::vector<Point_3>{});
    };
    std::printf("wrong vs EXACT, as: cut differs / patch differs (max cut dev in m)\n");
    std::printf("%-19s %6s | %-22s %-22s %-22s | us per cut: edge slicer exact half(+facet build)\n", "scene", "cuts", "EDGE", "SLICER", "HALF");
    Acc tcut[NM], tpatch[NM], tpatchx[NM];
    double ttime[NM] = {0, 0, 0, 0}; long tn = 0;
    for (const auto& scene : scenes) {
        std::ifstream f(std::string(argv[1]) + "/" + scene + ".json");
        if (!f) continue;
        json dump; f >> dump;
        config::Scenario sc = config::load_scenario(scene);
        Acc cut[NM], patch[NM], patchx[NM];
        double tm[NM] = {0, 0, 0, 0}; long n = 0;
        for (const auto& e : dump["expansions"]) {
            const json& pj = e["parent"];
            StanceFoot stance = pj["stance_foot"].get<int>() == 0 ? StanceFoot::Left : StanceFoot::Right;
            std::vector<Point_3> pv;
            for (const auto& v : pj["patch_vertices"]) pv.push_back(to_pt(v));
            Polyhedron base = reach.query(stance == StanceFoot::Left ? "RF" : "LF", stance == StanceFoot::Left ? "LF" : "RF", ReachabilityDirection::Forward);
            base = rotate_polyhedron_z(base, pj["foot_yaw"].get<double>());
            Polyhedron P = minkowski_sum(pv, base);
            EdgeList edges = polytope_edges(P);
            std::vector<std::pair<EK::Point_3, EK::Point_3>> xedges;
            for (const auto& [p, q] : edges) xedges.emplace_back(conv(p), conv(q));
            FacetPlanes fplanes = facet_planes(P);
            double build_share;
            {
                auto b0 = Clock::now();
                for (int r = 0; r < repeat; ++r) { volatile auto sz = facet_planes(P).p.size(); (void)sz; }
                build_share = std::chrono::duration<double>(Clock::now() - b0).count() / repeat / sc.surfaces.size();
            }
            for (const auto& s : sc.surfaces) {
                CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> slicer(P);
                std::vector<Point_3> c[NM] = {cut_edge(s, edges), cut_slicer(s, slicer), cut_exact(s, xedges), cut_halfspace(s, fplanes)};
                std::vector<Point_3> pt[NM];
                for (int m = 0; m < NM; ++m) pt[m] = patch_from_cut(c[m], s);
                {
                    static ClipTrace all, bad; static long nb = 0, nbad = 0;
                    if (c[0].size() > 2) {
                        auto p2 = transform_3d_points_to_surface_plane(c[0], s.transform_to_surface);
                        Polygon_2 hull; CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
                        std::vector<Point_2> hp(hull.vertices_begin(), hull.vertices_end());
                        ClipTrace t;
                        bool differs = dev(pt[0], patch_from_cut_exact_clip(c[0], s)) > 1e-7;
                        traced_clip(hp, s.vertices_2d, t, differs && nbad < 3);
                        ++nb; if (differs) ++nbad;
                        all.crossing_no_point += t.crossing_no_point;
                        if (differs) bad.crossing_no_point += t.crossing_no_point;
                        if (differs && t.crossing_no_point == 0) g_differs_without_lost_point++;
                        if (differs && t.crossing_no_point > 0) g_differs_with_lost_point++;
                        if (!differs && t.crossing_no_point > 0) g_ok_with_lost_point++;
                    }
                }
                {
                    double d = dev(patch_from_cut_fixed_clip(c[0], s), patch_from_cut_exact_clip(c[0], s));
                    if (d >= 1e299) { ++g_fixed_presence; ++g_fixed_wrong; } else if (d > 1e-7) { ++g_fixed_wrong; g_fixed_max = std::max(g_fixed_max, d); }
                }
                std::vector<Point_3> px[NM];
                for (int m = 0; m < NM; ++m) px[m] = patch_from_cut_exact_clip(c[m], s);
                for (int m = 0; m < NM; ++m) if (m != 2) patchx[m].add(dev(px[m], px[2]));
                if (std::getenv("BENCH_DEBUG") && dev(pt[0], px[0]) > 1e-7) {
                    static int shown = 0;
                    if (shown++ < 3) {
                        std::printf("\nDEBUG %s surface %d: double-clip patch vs exact-clip patch differ by %.3g\n", scene.c_str(), s.surface_id, dev(pt[0], px[0]));
                        auto p2 = transform_3d_points_to_surface_plane(c[0], s.transform_to_surface);
                        Polygon_2 hull; CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
                        std::vector<Point_2> hp(hull.vertices_begin(), hull.vertices_end());
                        std::printf(" subject hull (%zu pts):", hp.size());
                        for (auto& q : hp) std::printf(" (%.17g,%.17g)", CGAL::to_double(q.x()), CGAL::to_double(q.y()));
                        std::printf("\n clip polygon (%zu pts):", s.vertices_2d.size());
                        for (auto& q : s.vertices_2d) std::printf(" (%.17g,%.17g)", CGAL::to_double(q.x()), CGAL::to_double(q.y()));
                        // signed distance of every subject vertex to every clip edge (metres), smallest |d| first
                        std::printf("\n subject vertices closest to a clip edge line (signed, + = inside):\n");
                        for (size_t k = 0; k < s.vertices_2d.size(); ++k) {
                            const auto& a = s.vertices_2d[k]; const auto& b = s.vertices_2d[(k + 1) % s.vertices_2d.size()];
                            double ex = CGAL::to_double(b.x() - a.x()), ey = CGAL::to_double(b.y() - a.y()), L = std::hypot(ex, ey);
                            for (auto& q : hp) {
                                double d = (ex * CGAL::to_double(q.y() - a.y()) - ey * CGAL::to_double(q.x() - a.x())) / L;
                                if (std::abs(d) < 1e-9) std::printf("   clip edge %zu, vertex (%.17g,%.17g): signed dist %.3e\n", k, CGAL::to_double(q.x()), CGAL::to_double(q.y()), d);
                            }
                        }
                        auto show = [&](const char* l, const std::vector<Point_3>& v) {
                            std::printf(" %s (%zu pts):", l, v.size());
                            for (auto& q : v) std::printf(" (%.6f,%.6f)", CGAL::to_double(q.x()), CGAL::to_double(q.y()));
                            std::printf("\n");
                        };
                        show("double clip", pt[0]); show("exact clip ", px[0]);
                    }
                }
                for (int m = 0; m < NM; ++m) if (m != 2) { cut[m].add(cutdev(c[m], c[2])); patch[m].add(dev(pt[m], pt[2])); }
                ++n;
                auto time_it = [&](auto&& fn) {
                    auto t0 = Clock::now();
                    for (int r = 0; r < repeat; ++r) { volatile auto sz = fn().size(); (void)sz; }
                    return std::chrono::duration<double>(Clock::now() - t0).count() / repeat;
                };
                tm[0] += time_it([&] { return cut_edge(s, edges); });
                tm[1] += time_it([&] { CGAL::Polygon_mesh_slicer<Polyhedron, Kernel> sl(P); return cut_slicer(s, sl); });
                tm[2] += time_it([&] { return cut_exact(s, xedges); });
                tm[3] += time_it([&] { return cut_halfspace(s, fplanes); }) + build_share;
            }
        }
        auto cell = [&](int m) { char b[64]; std::snprintf(b, sizeof b, "%ld / %ld (%.1e)", cut[m].differ, patch[m].differ, cut[m].max_dev); return std::string(b); };
        std::printf("%-19s %6ld | %-22s %-22s %-22s | %.0f %.0f %.0f %.0f\n", scene.c_str(), n, cell(0).c_str(), cell(1).c_str(), cell(3).c_str(),
                    tm[0] / n * 1e6, tm[1] / n * 1e6, tm[2] / n * 1e6, tm[3] / n * 1e6);
        for (int m = 0; m < NM; ++m) { tcut[m].merge(cut[m]); tpatch[m].merge(patch[m]); tpatchx[m].merge(patchx[m]); ttime[m] += tm[m]; }
        tn += n;
    }
    std::printf("TOTAL %ld cuts\n", tn);
    std::printf("  candidate fixed clip (crossing point from the same signed values) vs exact clip, EDGE cuts: wrong on %ld (%ld presence), max %.2e m\n", g_fixed_wrong, g_fixed_presence, g_fixed_max);
    std::printf("  clip trace (EDGE cuts): wrong patch AND a lost intersection point: %ld | wrong patch WITHOUT lost point: %ld | correct patch but a lost point: %ld\n", g_differs_with_lost_point, g_differs_without_lost_point, g_ok_with_lost_point);
    for (int m = 0; m < NM; ++m) {
        if (m == 2) { std::printf("  %-7s reference, %.1f us/cut\n", kNames[m], ttime[m] / tn * 1e6); continue; }
        std::printf("  %-7s cut wrong on %ld (max %.2e m, %ld empty/non-empty), final patch wrong on %ld (%ld empty/non-empty) with the current double clip, %ld (%ld empty/non-empty, max %.1e m) with an exact clip, %.1f us/cut\n", kNames[m],
                    tcut[m].differ, tcut[m].max_dev, tcut[m].presence, tpatch[m].differ, tpatch[m].presence, tpatchx[m].differ, tpatchx[m].presence, tpatchx[m].max_dev, ttime[m] / tn * 1e6);
    }
    return 0;
}
