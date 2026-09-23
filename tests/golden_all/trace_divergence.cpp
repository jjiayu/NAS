// Finds where two runs of the SAME search first differ when only the heap
// state differs (the second run happens after a seed-dependent heap
// fragmentation, like NAS_SCRAMBLE). Records every expansion (pop) and every
// child dedup decision, then reports the first event where the runs disagree
// and classifies it:
//   GRID   - same node up to < 1e-9 in centroid/perimeter but different dedup
//            cell (int(x/threshold) flipped across a cell boundary);
//   TIE    - the two runs pop different nodes whose f-scores are equal to 1e-9
//            (order decided by noise/tie-breaking);
//   GEOMETRY - the nodes' centroid/perimeter really differ (> 1e-9).
// Also: nas_trace_divergence --audit <scene> <variant>  (similarity audit of the dedup key)
// Usage: nas_trace_divergence <scene> <seed> [variant]   variant: old|hc (default hc)
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

constexpr double THR = 0.02;

struct Event {
    char kind;          // 'P' pop, 'C' child
    int index;          // expansion index (pop) / parent's expansion index (child)
    int action;         // ChildAction for 'C', -1 for 'P'
    int surface, stance, depth, yawq;
    int kx, ky, kz, kp; // dedup cell
    double f, cx, cy, cz, per, yaw;
    std::vector<std::array<double, 3>> verts; // patch_vertices
};

Event make(char kind, int idx, int action, const Node& n) {
    Event e;
    e.kind = kind; e.index = idx; e.action = action;
    e.surface = n.surface_id; e.stance = static_cast<int>(n.stance_foot); e.depth = n.depth;
    e.yaw = n.foot_yaw; e.yawq = static_cast<int>(n.foot_yaw / (10.0 / 180.0 * M_PI));
    e.cx = CGAL::to_double(n.centroid.x()); e.cy = CGAL::to_double(n.centroid.y()); e.cz = CGAL::to_double(n.centroid.z());
    e.per = n.perimeter; e.f = n.f_score;
    for (const auto& v : n.patch_vertices) e.verts.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y()), CGAL::to_double(v.z())});
    e.kx = static_cast<int>(e.cx / THR); e.ky = static_cast<int>(e.cy / THR); e.kz = static_cast<int>(e.cz / THR); e.kp = static_cast<int>(e.per / THR);
    return e;
}

bool same_identity(const Event& a, const Event& b) {
    return a.kind == b.kind && a.index == b.index && a.action == b.action && a.surface == b.surface && a.stance == b.stance && a.depth == b.depth &&
           a.yawq == b.yawq && a.kx == b.kx && a.ky == b.ky && a.kz == b.kz && a.kp == b.kp;
}

std::vector<Event> run(const std::string& scene, std::string variant, int& expansions) {
    config::Scenario sc = config::load_scenario(scene);
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    Point_3 start(0.0, 0.0, 0.0);
    if (scene == "Stairs") start = Point_3(0.1, 0.0, 0.0);
    if (scene == "TwoFlatSurfaces") start = Point_3(2.2, 0.7, 0.0);
    Vector_3 goal_offset = scene == "ThreePathsNAS" ? Vector_3(0.0, 1.0, 0.0) : Vector_3(0.0, 0.0, 0.0);
    AstarSearchConfig c;
    c.start_position = start;
    c.start_stance_foot = StanceFoot::Right;
    c.goal_location = sc.surfaces.back().centroid + goal_offset;
    c.goal_stance_foot = StanceFoot::Left;
    c.distance_metric = DistanceMetric::Epa;
    c.heuristic_weight = 10.0;
    c.node_similarity_threshold = THR;
    c.expansion_params.rotation_enabled = true;
    c.expansion_params.yaw_discretization_num = 3;
    c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    c.expansion_params.cycle_detection_enabled = true;
    // variant letters: h convex patch, t simplify tol 1e-9, s canonical prism start, d deterministic ties, c area centroid ("old" = none)
    // "none" (or any word without these letters) = the old keys and ordering
    if (variant == "none" || variant == "old") variant = "";
    c.expansion_params.canonical_perimeter = variant.find('p') != std::string::npos;
    c.expansion_params.convex_patch = variant.find('h') != std::string::npos;
    c.expansion_params.convex_patch_simplify_tol = variant.find('t') != std::string::npos ? 1e-9 : 0.0;
    if (variant.find('g') != std::string::npos) c.dedup_mode = DedupMode::PatchDistance;
    if (variant.find('k') != std::string::npos) c.dedup_mode = DedupMode::CentroidPerimeterTolerance;
    c.deterministic_ties = variant.find('d') != std::string::npos;
    c.expansion_params.canonical_prism_start = variant.find('s') != std::string::npos;
    c.expansion_params.canonical_centroid = variant.find('c') != std::string::npos;
    std::vector<Event> events;
    c.on_expand = [&](int i, const Node& n) { events.push_back(make('P', i, -1, n)); };
    c.on_child = [&](int i, const Node& n, ChildAction a) { events.push_back(make('C', i, static_cast<int>(a), n)); };
    AstarSearch search(sc.surfaces, reach, c);
    search.search();
    expansions = search.expansion_count();
    return events;
}

void scramble(unsigned seed) {
    std::srand(seed);
    std::vector<void*> blocks;
    for (int i = 0; i < 4000; ++i) blocks.push_back(std::malloc(16 + std::rand() % 1500));
    for (size_t i = blocks.size(); i > 1; --i) std::swap(blocks[i - 1], blocks[std::rand() % i]);
    for (size_t i = 0; i < blocks.size(); i += 2) std::free(blocks[i]);
}

const char* kActions[] = {"SkippedClosed", "Pushed", "ImprovedExisting", "MergedWorse"};

void show(const char* tag, const Event& e) {
    std::printf("   %s %c #%d %s surf %d stance %d depth %d yaw %.4f | centroid (%.12f, %.12f, %.12f) perim %.12f | cell (%d,%d,%d,p%d) f %.9f\n", tag, e.kind, e.index,
                e.kind == 'C' ? kActions[e.action] : "pop", e.surface, e.stance, e.depth, e.yaw, e.cx, e.cy, e.cz, e.per, e.kx, e.ky, e.kz, e.kp, e.f);
}

// distance (m) of a coordinate to the nearest cell boundary
double boundary_dist(double v) {
    double t = v / THR;
    return std::abs(t - std::round(t)) * THR;
}

} // namespace

// --- similarity audit ------------------------------------------------------
// Is the dedup key a good similarity criterion? Over every patch a search
// generates (all children, merged or not), compares "same dedup cell" with the
// true geometric distance between the two patches (two-way max vertex-to-
// boundary distance of their polygons, in metres; scenes are horizontal).
double point_to_boundary(double px, double py, const std::vector<std::array<double, 3>>& poly) {
    double best = 1e300;
    for (size_t i = 0; i < poly.size(); ++i) {
        const auto &a = poly[i], &b = poly[(i + 1) % poly.size()];
        double ex = b[0] - a[0], ey = b[1] - a[1], L2 = ex * ex + ey * ey;
        double t = L2 > 0 ? std::max(0.0, std::min(1.0, ((px - a[0]) * ex + (py - a[1]) * ey) / L2)) : 0.0;
        best = std::min(best, std::hypot(px - (a[0] + t * ex), py - (a[1] + t * ey)));
    }
    return best;
}
double patch_distance(const Event& a, const Event& b) {
    double d = 0;
    for (const auto& v : a.verts) d = std::max(d, point_to_boundary(v[0], v[1], b.verts));
    for (const auto& v : b.verts) d = std::max(d, point_to_boundary(v[0], v[1], a.verts));
    return d;
}

int audit(const std::string& scene, const std::string& variant) {
    int expansions = 0;
    std::vector<Event> ev = run(scene, variant, expansions);
    long children = 0, pushed = 0, merged = 0, closed = 0;
    std::vector<const Event*> uniq; // one per (surface, stance, yawq, polygon)
    for (const auto& e : ev) {
        if (e.kind != 'C') continue;
        ++children;
        if (e.action == 1) ++pushed; else if (e.action == 0) ++closed; else ++merged;
        bool dup = false;
        for (const Event* u : uniq) {
            if (u->surface != e.surface || u->stance != e.stance || u->yawq != e.yawq || u->verts.size() != e.verts.size()) continue;
            double d = 0;
            for (size_t i = 0; i < e.verts.size(); ++i) d = std::max(d, std::hypot(u->verts[i][0] - e.verts[i][0], u->verts[i][1] - e.verts[i][1]));
            if (d < 1e-9) { dup = true; break; }
        }
        if (!dup) uniq.push_back(&e);
    }
    long same_key = 0, same_key_far2 = 0, same_key_far5 = 0, diff_key_close = 0, diff_key_close2 = 0, diff_key_near_pairs = 0;
    double same_key_max = 0;
    for (size_t i = 0; i < uniq.size(); ++i)
        for (size_t j = i + 1; j < uniq.size(); ++j) {
            const Event &a = *uniq[i], &b = *uniq[j];
            if (a.surface != b.surface || a.stance != b.stance || a.yawq != b.yawq) continue;
            bool key = a.kx == b.kx && a.ky == b.ky && a.kz == b.kz && a.kp == b.kp;
            double d = patch_distance(a, b);
            if (key) {
                ++same_key; same_key_max = std::max(same_key_max, d);
                if (d > 0.02) ++same_key_far2;
                if (d > 0.05) ++same_key_far5;
            } else {
                if (d < 0.005) ++diff_key_close;
                if (d < 0.02) ++diff_key_close2;
            }
        }
    std::printf("%-16s %-6s: %d expansions; %ld children generated: %ld pushed, %ld merged into an open node, %ld skipped (already expanded)\n", scene.c_str(), variant.c_str(),
                expansions, children, pushed, merged, closed);
    std::printf("    %zu distinct patches | pairs with the SAME key: %ld (max patch distance %.3f m; > 2 cm: %ld, > 5 cm: %ld) | pairs with DIFFERENT keys but patches < 5 mm apart: %ld, < 2 cm apart: %ld\n",
                uniq.size(), same_key, same_key_max, same_key_far2, same_key_far5, diff_key_close, diff_key_close2);
    return 0;
}

int main(int argc, char** argv) {
    if (argc >= 4 && std::string(argv[1]) == "--audit") return audit(argv[2], argv[3]);
    if (argc < 3) { std::fprintf(stderr, "usage: %s <scene> <seed> [variant letters h,t,c or old]\n", argv[0]); return 1; }
    std::string scene = argv[1];
    unsigned seed = static_cast<unsigned>(std::stoul(argv[2]));
    std::string variant = argc < 4 ? "hc" : argv[3];
    int ea = 0, eb = 0;
    std::vector<Event> A = run(scene, variant, ea);
    scramble(seed);
    std::vector<Event> B = run(scene, variant, eb);
    std::printf("%-16s seed %u %-4s: run A %d expansions (%zu events), run B %d expansions (%zu events)", scene.c_str(), seed, variant.c_str(), ea, A.size(), eb, B.size());
    size_t n = std::min(A.size(), B.size()), i = 0;
    double max_prefix_dev = 0;
    for (; i < n; ++i) {
        if (!same_identity(A[i], B[i])) break;
        max_prefix_dev = std::max({max_prefix_dev, std::abs(A[i].cx - B[i].cx), std::abs(A[i].cy - B[i].cy), std::abs(A[i].cz - B[i].cz), std::abs(A[i].per - B[i].per)});
    }
    if (i == n && A.size() == B.size()) { std::printf("  -> IDENTICAL (max centroid/perimeter deviation over all events %.2e)\n", max_prefix_dev); return 0; }
    std::printf("\n  first divergence at event %zu (identical before, max centroid/perimeter deviation in that prefix %.2e)\n", i, max_prefix_dev);
    if (i < n) {
        show("A", A[i]); show("B", B[i]);
        const Event &a = A[i], &b = B[i];
        double dc = std::max({std::abs(a.cx - b.cx), std::abs(a.cy - b.cy), std::abs(a.cz - b.cz), std::abs(a.per - b.per)});
        bool same_node = a.kind == b.kind && a.surface == b.surface && a.stance == b.stance && a.depth == b.depth && a.yawq == b.yawq;
        const char* verdict;
        if (same_node && dc < 1e-9) verdict = "GRID: the same node (centroid/perimeter equal to 1e-9) lands in a different dedup cell";
        else if (a.kind == 'P' && b.kind == 'P' && std::abs(a.f - b.f) < 1e-9) verdict = "TIE: different nodes popped with equal f-scores";
        else if (a.kind == 'P' && b.kind == 'P') verdict = "ORDER: different nodes popped, f-scores differ (check earlier events: a merge/push difference upstream would show first)";
        else if (same_node) verdict = "GEOMETRY: same node id but centroid/perimeter differ by more than 1e-9";
        else verdict = "STRUCTURE: a different child/action sequence";
        for (int w = 0; w < 2; ++w) {
            const Event& e = w == 0 ? a : b;
            std::printf("  %c patch: %zu vertices:", w == 0 ? 'A' : 'B', e.verts.size());
            for (auto& v : e.verts) std::printf(" (%.9f,%.9f)", v[0], v[1]);
            std::printf("\n");
        }
        std::printf("  deviation A-B: %.3e m;  A distance to nearest cell boundary: cx %.2e cy %.2e per %.2e\n  VERDICT: %s\n", dc,
                    boundary_dist(a.cx), boundary_dist(a.cy), boundary_dist(a.per), verdict);
    } else {
        std::printf("  one run is a prefix of the other\n");
    }
    return 0;
}
