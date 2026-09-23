// Phase 5's actual deliverable: compare the ported AstarSearch against the
// golden reference captured from the OLD repo in phase 0
// (tests/golden/NarrowPassage_astar.json), on the exact same scenario.
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

int run_astar_search_golden() {
    fixtures::Scenario scenario = fixtures::make_narrow_passage();
    ReachabilityModel reachability = fixtures::make_forward_reachability_model(TALOS_REACHABILITY_DATA_DIR);

    AstarSearch search(scenario.surfaces, reachability, scenario.astar_config);
    search.search();

    bool ok = fixtures::check_path_matches_golden(search.result_path(),
                                                   std::string(GOLDEN_DATA_DIR) + "/NarrowPassage_astar.json");
    if (!ok) {
        return 1;
    }
    std::cout << "AstarSearch matches the NarrowPassage golden reference (depth, stance, surfaces; yaw ties are free) ("
              << search.result_path().size() << " nodes)\n";
    return 0;
}
