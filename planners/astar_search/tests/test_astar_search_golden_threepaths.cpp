// Second golden comparison (see test_astar_search_golden.cpp) —
// ThreePathsNAS exercises a different topology (branching / local minima).
// Scenario + comparison logic live in tests/fixtures (phase 8d-2).

#include "nas/fixtures/golden_compare.hpp"
#include "nas/fixtures/scenarios.hpp"
#include "nas/planners/astar_search.hpp"

#include <iostream>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

int main() {
    fixtures::Scenario scenario = fixtures::make_three_paths_nas();
    ReachabilityModel reachability = fixtures::make_forward_reachability_model(TALOS_REACHABILITY_DATA_DIR);

    AstarSearch search(scenario.surfaces, reachability, scenario.astar_config);
    search.search();

    bool ok = fixtures::check_path_matches_golden(search.result_path(),
                                                   std::string(GOLDEN_DATA_DIR) + "/ThreePathsNAS_astar.json");
    if (!ok) {
        return 1;
    }
    std::cout << "AstarSearch matches the ThreePathsNAS golden reference exactly ("
              << search.result_path().size() << " nodes)\n";
    return 0;
}
