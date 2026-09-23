// Checks core/expansion against an independent oracle, on real states.
//
// Parents are the nodes a real CASSR search expands (AstarSearchConfig::on_expand),
// on every scenario. For each parent and each surface, the child patch produced
// by expand_node must equal the patch recomputed by exact_clip.hpp: the same
// plane cut, then the 2D projection, hull and clip against the surface done in
// EXACT arithmetic (independent of core/geometry's clip). It also checks that
// children exist exactly when the oracle patch is non-empty and the cycle
// detection allows them, and that every child's patch is a clean convex polygon
// (>= 3 vertices, counter-clockwise, no near-collinear vertex).
// This is what proves the corrected clip: the old code's clip dropped an
// intersection point on near-parallel edges (docs/paper-deltas.md).
#include "exact_clip.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"
#include "nas/core/geometry.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <CGAL/convex_hull_2.h>
#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

constexpr double POLY_TOL = 1e-7;
constexpr int MAX_EXPANSIONS_PER_SCENE = 80;

std::vector<Point_2> hull_xy(const std::vector<Point_3>& pts) {
    std::vector<Point_2> p2, h;
    for (const auto& p : pts) p2.emplace_back(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(h));
    return h;
}

double boundary_distance(const Point_2& p, const std::vector<Point_2>& poly) {
    if (poly.size() == 1) return std::sqrt(CGAL::to_double(CGAL::squared_distance(p, poly[0])));
    double best = 1e300;
    for (size_t i = 0; i < poly.size(); ++i) {
        Segment_2 seg(poly[i], poly[(i + 1) % poly.size()]);
        best = std::min(best, std::sqrt(CGAL::to_double(CGAL::squared_distance(p, seg))));
    }
    return best;
}

double polygon_area(const std::vector<Point_2>& h) {
    double a = 0;
    for (size_t i = 0; i < h.size(); ++i) {
        const Point_2& p = h[i];
        const Point_2& q = h[(i + 1) % h.size()];
        a += CGAL::to_double(p.x()) * CGAL::to_double(q.y()) - CGAL::to_double(q.x()) * CGAL::to_double(p.y());
    }
    return std::abs(a) / 2.0;
}

// Two-way Hausdorff distance of the xy hull boundaries, area difference and z
// difference (all scenes of environments.hpp are horizontal).
double polygon_deviation(const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
    auto ha = hull_xy(a), hb = hull_xy(b);
    double dev = 0;
    for (const auto& p : ha) dev = std::max(dev, boundary_distance(p, hb));
    for (const auto& p : hb) dev = std::max(dev, boundary_distance(p, ha));
    dev = std::max(dev, std::abs(polygon_area(ha) - polygon_area(hb)));
    if (!a.empty() && !b.empty()) dev = std::max(dev, std::abs(CGAL::to_double(a[0].z() - b[0].z())));
    return dev;
}

struct Tally {
    int parents = 0, cuts = 0, patch_mismatch = 0, presence_mismatch = 0, not_clean = 0;
    double max_dev = 0;
    std::vector<std::string> examples;
};

struct SceneSetup { const char* name; Point_3 start; Vector_3 goal_offset; };
const std::vector<SceneSetup> kScenes = {
    {"NarrowPassage", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs", Point_3(0.1, 0, 0), Vector_3(0, 0, 0)},
    {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0), Vector_3(0, 0, 0)}, {"LongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"LongLongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Flat", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"LongStairsComplete", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"LongStairsExp", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"ThreePathsScene", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs_Up_Down", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"ThreePathsNAS", Point_3(0, 0, 0), Vector_3(0, 1, 0)},
};

bool clean_polygon(const Polygon_2& poly) {
    const size_t n = poly.size();
    if (n < 3 || !poly.is_counterclockwise_oriented()) return false;
    for (size_t i = 0; i < n; ++i) {
        const Point_2& a = poly.vertex((i + n - 1) % n);
        const Point_2& b = poly.vertex(i);
        const Point_2& c = poly.vertex((i + 1) % n);
        double ex = CGAL::to_double(c.x() - a.x()), ey = CGAL::to_double(c.y() - a.y());
        double len = std::hypot(ex, ey);
        double dist = len > 0 ? std::abs(ex * CGAL::to_double(b.y() - a.y()) - ey * CGAL::to_double(b.x() - a.x())) / len : 0.0;
        if (dist < 1e-9) return false;
    }
    return true;
}

} // namespace

int main() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    bool all_ok = true;
    long total_cuts = 0;

    for (const auto& s : kScenes) {
        config::Scenario sc = config::load_scenario(s.name);
        AstarSearchConfig cfg;
        cfg.start_position = s.start;
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.goal_location = sc.surfaces.back().centroid + s.goal_offset;
        cfg.goal_stance_foot = StanceFoot::Left;
        cfg.expansion_params.rotation_enabled = true;
        cfg.expansion_params.yaw_discretization_num = 3;
        cfg.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
        cfg.expansion_params.cycle_detection_enabled = true;

        Tally t;
        cfg.on_expand = [&](int index, const Node& node) {
            if (index > MAX_EXPANSIONS_PER_SCENE) return;
            ++t.parents;
            // an independent copy of the parent, expanded with the very same code path
            NodePool pool;
            Node* parent = pool.create();
            parent->patch_vertices = node.patch_vertices;
            parent->stance_foot = node.stance_foot;
            parent->foot_yaw = node.foot_yaw;
            parent->depth = node.depth;
            parent->surface_id = node.surface_id;
            parent->pred_surface_ids = node.pred_surface_ids;
            std::vector<Node*> kids = expand_node(parent, sc.surfaces, reach, ReachabilityDirection::Forward, cfg.expansion_params, pool);
            std::map<int, std::vector<const Node*>> by_surface;
            for (const Node* k : kids) by_surface[k->surface_id].push_back(k);

            // the oracle's polytope: same construction, done here from scratch
            StanceFoot child_stance = other_foot(parent->stance_foot);
            Polyhedron base = rotate_polyhedron_z(
                reach.query(effector_name(child_stance), effector_name(parent->stance_foot), ReachabilityDirection::Forward), parent->foot_yaw);
            Polyhedron P = minkowski_sum(parent->patch_vertices, base);
            for (const Surface& surf : sc.surfaces) {
                ++t.cuts;
                std::vector<Point_3> oracle = oracle::patch_from_cut_exact_clip(compute_polytope_plane_intersection(surf.plane, P), surf);
                bool blocked = cycle_path_detection(parent, child_stance, surf.surface_id);
                bool expect_children = !oracle.empty() && !blocked;
                auto it = by_surface.find(surf.surface_id);
                bool have = it != by_surface.end();
                if (have != expect_children) {
                    ++t.presence_mismatch;
                    if (t.examples.size() < 4) t.examples.push_back("expansion " + std::to_string(index) + " surface " + std::to_string(surf.surface_id) +
                                                                     ": children present " + std::to_string(have) + ", oracle expects " + std::to_string(expect_children));
                    continue;
                }
                if (!have) continue;
                for (const Node* k : it->second) {
                    double d = polygon_deviation(k->patch_vertices, oracle);
                    t.max_dev = std::max(t.max_dev, d);
                    if (d > POLY_TOL) {
                        ++t.patch_mismatch;
                        if (t.examples.size() < 4) t.examples.push_back("expansion " + std::to_string(index) + " surface " + std::to_string(surf.surface_id) +
                                                                         ": patch differs from the exact oracle by " + std::to_string(d) + " m");
                    }
                    if (!clean_polygon(k->patch_polygon_2d)) ++t.not_clean;
                }
            }
        };
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();

        bool ok = t.patch_mismatch == 0 && t.presence_mismatch == 0 && t.not_clean == 0;
        all_ok = all_ok && ok;
        total_cuts += t.cuts;
        std::printf("%-19s %3d expansions, %5d cuts: patch mismatches %d, presence mismatches %d, unclean patches %d (max dev %.1e m)  %s\n", s.name, t.parents,
                    t.cuts, t.patch_mismatch, t.presence_mismatch, t.not_clean, t.max_dev, ok ? "ok" : "FAIL");
        for (const auto& e : t.examples) std::printf("      e.g. %s\n", e.c_str());
    }
    std::printf("%s (%ld surface cuts checked against the exact-arithmetic oracle)\n", all_ok ? "PASS" : "FAIL", total_cuts);
    return all_ok ? 0 : 1;
}
