#pragma once

// Shared scenario fixtures (phase 8d-2, see PLAN.md/docs/paper-deltas.md).
// Before this, the NarrowPassage/ThreePathsNAS surface lists and forward
// ReachabilityModel construction were copy-pasted across 5 files:
// planners/astar_search's two golden tests, footstep_qp's golden test,
// tests/perf/compare_perf.cpp, and tests/viz_dump/dump_plan.cpp. All five
// now use this instead.
//
// Surface vertices are copied verbatim from the old include/environments.hpp
// (this repo's own predecessor, not the eventual config/Scenario loader —
// see PLAN.md phase 9). foot_length/foot_width and the AstarSearchConfig
// values (distance metric, rotation params, node similarity threshold) match
// the active constants.hpp values at golden-capture time.

#include "nas/core/surface.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <string>
#include <vector>

namespace nas::fixtures {

struct Scenario {
    std::string name;
    std::vector<Surface> surfaces;
    AstarSearchConfig astar_config; // start/goal/rotation already set for this scenario
};

Scenario make_narrow_passage();
Scenario make_three_paths_nas();

// Loads the two Forward reachability entries every fixture scenario needs
// (RF-in-LF, LF-in-RF) from a talosReachability-shaped data directory.
ReachabilityModel make_forward_reachability_model(const std::string& talos_data_dir);

} // namespace nas::fixtures
