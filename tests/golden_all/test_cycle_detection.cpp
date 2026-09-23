// Are cycles really excluded? The rule (paper IV and V-B.4, "hasContactSurfaceNotBeenLeft"): once a foot has left a
// surface it never steps on it again. Checked here from the outside, on the parent chain (Node::parent) of EVERY node the
// search expands, not from the node's own history (pred_surface_ids), which is what the detection uses:
//   for each foot, the sequence of surfaces of its successive contacts must never read S ... T ... S with T != S.
// Also checked on the final path, on all scenes, with the EPA heuristic and (capped) with the Euclidean one, and on a scene
// with duplicated coplanar surfaces ("similar surfaces": the same ground given twice, ids 0 and 1).
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

// true when the chain root -> node revisits a surface with the same foot after leaving it
bool has_cycle(const Node* node) {
    std::vector<const Node*> chain;
    for (const Node* n = node; n != nullptr; n = n->parent) chain.push_back(n);
    std::map<int, std::vector<int>> per_foot; // foot -> surfaces of its successive contacts, oldest first
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        if ((*it)->surface_id < 0) continue; // the start node: not on a scene surface
        per_foot[static_cast<int>((*it)->stance_foot)].push_back((*it)->surface_id);
    }
    for (const auto& [foot, seq] : per_foot) {
        std::set<int> left;
        for (size_t i = 0; i < seq.size(); ++i) {
            if (left.count(seq[i])) return true;
            if (i + 1 < seq.size() && seq[i + 1] != seq[i]) left.insert(seq[i]);
        }
    }
    return false;
}

} // namespace

int main() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    struct Case { std::string scene; Point_3 start; DistanceMetric metric; int cap; std::vector<Surface> extra; };
    std::vector<Case> cases;
    for (const char* n : {"NarrowPassage", "Stairs", "LongStairs", "LongLongStairs", "Flat", "LongStairsComplete", "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down", "ThreePathsNAS"})
        cases.push_back({n, std::string(n) == "Stairs" ? Point_3(0.1, 0, 0) : Point_3(0, 0, 0), DistanceMetric::Epa, 3000, {}});
    for (const char* n : {"Flat", "Stairs", "LongStairs"}) // Euclidean: many more expansions, capped
        cases.push_back({n, std::string(n) == "Stairs" ? Point_3(0.1, 0, 0) : Point_3(0, 0, 0), DistanceMetric::Euclidean, 4000, {}});

    int failures = 0;
    long total_nodes = 0;
    for (const Case& c : cases) {
        config::Scenario sc = config::load_scenario(c.scene);
        AstarSearchConfig cfg;
        cfg.start_position = c.start; cfg.start_stance_foot = StanceFoot::Right; cfg.goal_stance_foot = StanceFoot::Left;
        cfg.goal_location = sc.surfaces.back().centroid;
        cfg.distance_metric = c.metric; cfg.max_expansions = c.cap;
        cfg.expansion_params.rotation_enabled = true;
        long nodes = 0, cycles = 0;
        cfg.on_expand = [&](int, const Node& n) { ++nodes; if (has_cycle(&n)) ++cycles; };
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        bool path_ok = search.result_path().empty() || !has_cycle(search.result_path().back());
        bool ok = cycles == 0 && path_ok;
        failures += ok ? 0 : 1;
        total_nodes += nodes;
        std::printf("%s %-19s %-9s: %6ld expanded nodes, %ld with a revisited surface, final path %s\n", ok ? "ok:  " : "FAIL:", c.scene.c_str(),
                    c.metric == DistanceMetric::Epa ? "EPA" : "Euclidean", nodes, cycles, path_ok ? "clean" : "HAS A CYCLE");
    }

    // "similar surfaces": the same ground given twice (two coplanar overlapping squares, ids 0 and 1). Two goals:
    //   - reachable step (must find a path, clean);
    //   - unreachable surface (2 m up): the search explores 3000 nodes, the situation where cycles would pile up.
    // The unreachable case is run with the detection on (must be clean) and off (negative control: the checker must then
    // find cycles, otherwise it could not see any).
    std::vector<Point_3> ground = {Point_3(-1.5, -1, 0), Point_3(1.5, -1, 0), Point_3(1.5, 1, 0), Point_3(-1.5, 1, 0)};
    std::vector<Point_3> ground_again = {Point_3(-1.4, -0.9, 0), Point_3(1.6, -0.9, 0), Point_3(1.6, 1.1, 0), Point_3(-1.4, 1.1, 0)};
    long control_cycles = 0;
    for (int variant = 0; variant < 3; ++variant) {
        const bool reachable = variant == 0, detection = variant != 2;
        const double z = reachable ? 0.1 : 2.0;
        std::vector<Point_3> far = {Point_3(1.6, -1, z), Point_3(3.0, -1, z), Point_3(3.0, 1, z), Point_3(1.6, 1, z)};
        std::vector<Surface> surfaces = {Surface(ground, 0, 0.22, 0.22), Surface(ground_again, 1, 0.22, 0.22), Surface(far, 2, 0.22, 0.22)};
        AstarSearchConfig cfg;
        cfg.start_position = Point_3(-1.2, 0, 0); cfg.start_stance_foot = StanceFoot::Right; cfg.goal_stance_foot = StanceFoot::Left;
        cfg.goal_location = surfaces.back().centroid;
        cfg.expansion_params.rotation_enabled = true;
        cfg.expansion_params.cycle_detection_enabled = detection;
        cfg.max_expansions = 3000;
        long nodes = 0, cycles = 0;
        cfg.on_expand = [&](int, const Node& n) { ++nodes; if (has_cycle(&n)) ++cycles; };
        AstarSearch search(surfaces, reach, cfg);
        search.search();
        if (detection) {
            bool ok = cycles == 0 && (!reachable || !search.result_path().empty());
            failures += ok ? 0 : 1;
            std::printf("%s duplicated ground, %s goal: %5ld expanded nodes, %ld with a revisited surface\n", ok ? "ok:  " : "FAIL:", reachable ? "reachable  " : "unreachable", nodes, cycles);
        } else {
            control_cycles = cycles;
            std::printf("control: duplicated ground, unreachable goal, detection OFF: %ld of %ld expanded nodes revisit a surface\n", cycles, nodes);
        }
    }
    {
        bool ok = control_cycles > 0; // the checker is able to see a cycle
        failures += ok ? 0 : 1;
        std::printf("%s negative control: with the detection off the checker finds cycles (%ld)\n", ok ? "ok:  " : "FAIL:", control_cycles);
    }
    std::printf("%s (%ld expanded nodes checked)\n", failures == 0 ? "PASS" : "FAIL", total_nodes);
    return failures == 0 ? 0 : 1;
}
