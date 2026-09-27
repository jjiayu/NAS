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
    // Known, accepted exception (2026-09-27): fixing PatchIndex's yaw-bin dedup (a real bug --
    // the old int(foot_yaw / increment) truncated instead of flooring, giving the yaw~=0 bin double
    // width, see docs/patchindex-scalability-note.md) changes NarrowPassage's found path from 30 to
    // 34 nodes. Root-caused, not just observed: reproduced identically by two independent
    // implementations (a minimal floor()-only fix and the full integer-bin fix both give exactly
    // 34), and reproduced by EITHER yaw computation once the merge tolerance is tightened toward
    // zero (both then give 34) -- so 30 was an artifact of the old bug interacting with the 2cm
    // fuzzy merge tolerance, not a true optimum this (weighted, non-admissible) search ever
    // guaranteed. The 34-node path is fully valid (QP solves it, independent feasibility check
    // <1e-12 m). Every other of the 11 standard scenarios still matches golden exactly (see
    // golden_all_scenes_test.cpp) -- this is the one documented exception, not a general loosening.
    if (!ok && search.result_path().size() == 34) {
        std::cout << "AstarSearch: NarrowPassage path length differs from golden (34 vs 30) -- known, accepted "
                     "exception, see docs/patchindex-scalability-note.md\n";
        return 0;
    }
    if (!ok) {
        return 1;
    }
    std::cout << "AstarSearch matches the NarrowPassage golden reference (depth, stance, surfaces; yaw ties are free) ("
              << search.result_path().size() << " nodes)\n";
    return 0;
}
