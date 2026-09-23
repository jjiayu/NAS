// The Euclidean heuristic (distance from the goal to the node's patch centroid,
// unweighted) is an alternative to the default EPA one. Unweighted, it expands
// far more nodes (the EPA one is x10 weighted; measured: Flat 1568 expansions vs 11,
// Stairs 101 vs 34, and more than 20000 on LongStairsComplete where EPA needs 26),
// so this checks it only on the two scenes where it stays small: it must find a
// path ending on a patch that contains the goal, and be deterministic.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <iostream>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

int run_euclidean_metric() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    struct Scene { const char* name; Point_3 start; };
    int failures = 0;
    for (const Scene& s : std::vector<Scene>{{"Flat", Point_3(0, 0, 0)}, {"Stairs", Point_3(0.1, 0, 0)} }) {
        config::Scenario sc = config::load_scenario(s.name);
        AstarSearchConfig cfg;
        cfg.start_position = s.start;
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.goal_location = sc.surfaces.back().centroid;
        cfg.goal_stance_foot = StanceFoot::Left;
        cfg.expansion_params.rotation_enabled = true;
        AstarSearchConfig euclid = cfg;
        euclid.distance_metric = DistanceMetric::Euclidean;
        euclid.max_expansions = 20000;

        AstarSearch a(sc.surfaces, reach, euclid), b(sc.surfaces, reach, euclid), epa(sc.surfaces, reach, cfg);
        a.search(); b.search(); epa.search();
        bool found = !a.result_path().empty();
        bool goal = found && a.result_path().back()->check_if_node_contains_point(euclid.goal_location);
        bool det = a.expansion_count() == b.expansion_count() && a.result_path().size() == b.result_path().size();
        std::cout << (found && goal && det ? "ok: " : "FAIL: ") << s.name << ": Euclidean " << a.expansion_count() << " expansions / "
                  << a.result_path().size() << " nodes (path found " << found << ", ends on the goal " << goal << ", deterministic " << det
                  << "); EPA " << epa.expansion_count() << " / " << epa.result_path().size() << "\n";
        failures += (found && goal && det) ? 0 : 1;
    }
    return failures == 0 ? 0 : 1;
}
