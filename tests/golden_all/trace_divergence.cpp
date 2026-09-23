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
// Usage: nas_trace_divergence <scene> <seed>
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

std::vector<Event> run(const std::string& scene, int& expansions) {
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

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: %s <scene> <seed>\n", argv[0]); return 1; }
    std::string scene = argv[1];
    unsigned seed = static_cast<unsigned>(std::stoul(argv[2]));
    int ea = 0, eb = 0;
    std::vector<Event> A = run(scene, ea);
    scramble(seed);
    std::vector<Event> B = run(scene, eb);
    std::printf("%-16s seed %u: run A %d expansions (%zu events), run B %d expansions (%zu events)", scene.c_str(), seed, ea, A.size(), eb, B.size());
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
