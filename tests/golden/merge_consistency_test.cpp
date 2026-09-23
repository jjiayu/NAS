// Do the merges happen correctly, also when the search explodes (Euclidean heuristic)? Checked from the outside with the
// on_child hook, which reports every generated child and what the search did with it:
//   Pushed (new open node), ImprovedExisting / MergedWorse (merged into an open node), SkippedClosed (equal to an
//   already-expanded node).
// Consistency: among all the nodes that were PUSHED, no two may be "the same" (same surface, stance foot and yaw bin, patches
// within 2 cm of each other, recomputed here independently): if two were, the second should have been merged into the first,
// which is in the open or the closed index. Reported: the merge statistics and how the pushed nodes spread by depth.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

struct Pushed { int surface, stance, yawbin, depth; std::vector<Point_2> poly; };

double point_to_boundary(const Point_2& p, const std::vector<Point_2>& poly) {
    double best = 1e300;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Point_2 &a = poly[i], &b = poly[(i + 1) % poly.size()];
        double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
        double ex = CGAL::to_double(b.x()) - ax, ey = CGAL::to_double(b.y()) - ay, l2 = ex * ex + ey * ey;
        double px = CGAL::to_double(p.x()) - ax, py = CGAL::to_double(p.y()) - ay;
        double t = l2 > 0 ? std::max(0.0, std::min(1.0, (px * ex + py * ey) / l2)) : 0.0;
        best = std::min(best, std::hypot(px - t * ex, py - t * ey));
    }
    return best;
}
double patch_distance(const std::vector<Point_2>& a, const std::vector<Point_2>& b) {
    double d = 0;
    for (const auto& p : a) d = std::max(d, point_to_boundary(p, b));
    for (const auto& p : b) d = std::max(d, point_to_boundary(p, a));
    return d;
}

} // namespace

int run_merge_consistency() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    struct Case { const char* scene; Point_3 start; DistanceMetric metric; int cap; };
    const std::vector<Case> cases = {
        {"Stairs", Point_3(0.1, 0, 0), DistanceMetric::Epa, 0},           {"LongStairs", Point_3(0, 0, 0), DistanceMetric::Epa, 0},
        {"NarrowPassage", Point_3(0, 0, 0), DistanceMetric::Epa, 0},
        {"Stairs", Point_3(0.1, 0, 0), DistanceMetric::Euclidean, 0},     {"Flat", Point_3(0, 0, 0), DistanceMetric::Euclidean, 600},
        {"LongStairs", Point_3(0, 0, 0), DistanceMetric::Euclidean, 700}, {"NarrowPassage", Point_3(0, 0, 0), DistanceMetric::Euclidean, 700},
    };
    int failures = 0;
    for (const Case& c : cases) {
        config::Scenario sc = config::load_scenario(c.scene);
        AstarSearchConfig cfg;
        cfg.start_position = c.start; cfg.start_stance_foot = StanceFoot::Right; cfg.goal_stance_foot = StanceFoot::Left;
        cfg.goal_location = sc.surfaces.back().centroid;
        cfg.distance_metric = c.metric; cfg.max_expansions = c.cap;
        cfg.expansion_params.rotation_enabled = true;
        long counts[4] = {0, 0, 0, 0};
        std::vector<Pushed> pushed;
        const double inc = cfg.expansion_params.yaw_angle_increment;
        cfg.on_child = [&](int, const Node& n, ChildAction a) {
            ++counts[static_cast<int>(a)];
            if (a == ChildAction::Pushed)
                pushed.push_back({n.surface_id, static_cast<int>(n.stance_foot), static_cast<int>(n.foot_yaw / inc), n.depth,
                                  std::vector<Point_2>(n.patch_polygon_2d.vertices_begin(), n.patch_polygon_2d.vertices_end())});
        };
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        // missed merges: two pushed nodes that are "the same"
        std::map<std::tuple<int, int, int>, std::vector<const Pushed*>> groups;
        for (const Pushed& p : pushed) groups[{p.surface, p.stance, p.yawbin}].push_back(&p);
        long missed = 0;
        for (const auto& [key, g] : groups)
            for (size_t i = 0; i < g.size(); ++i)
                for (size_t j = i + 1; j < g.size(); ++j)
                    if (patch_distance(g[i]->poly, g[j]->poly) < 0.02) ++missed;
        std::map<int, long> by_depth;
        for (const Pushed& p : pushed) ++by_depth[p.depth];
        long children = counts[0] + counts[1] + counts[2] + counts[3];
        bool ok = missed == 0;
        failures += ok ? 0 : 1;
        std::printf("%s %-13s %-9s: %5d expansions, %6ld children = %5ld pushed + %5ld merged into an open node (%ld improving it) + %5ld equal to an expanded one; missed merges: %ld; pushed per depth:",
                    ok ? "ok:  " : "FAIL:", c.scene, c.metric == DistanceMetric::Epa ? "EPA" : "Euclidean", search.expansion_count(), children, counts[1],
                    counts[2] + counts[3], counts[2], counts[0], missed);
        for (const auto& [d, n] : by_depth) std::printf(" %d:%ld", d, n);
        std::printf("%s\n", search.result_path().empty() ? "  [no path within the cap]" : "");
    }
    return failures == 0 ? 0 : 1;
}
