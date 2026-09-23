#pragma once

// Shared golden-comparison helper (phase 8d-2) — the depth/stance_foot/
// foot_yaw/surface_id comparison loop against a phase-0 golden capture was
// duplicated between test_astar_search_golden.cpp and
// test_astar_search_golden_threepaths.cpp.

#include "nas/core/node.hpp"

#include <string>
#include <vector>

namespace nas::fixtures {

// Prints "ok"/"FAIL" lines (same style as the rest of this repo's
// dependency-free tests) and per-mismatch detail to stderr. Returns true
// iff the golden file opens, records success, has the same path length,
// and every node matches on (depth, stance_foot, surface_id) - foot_yaw is
// only reported, not compared: among equal-cost plans the yaw is a tie-break -
// (surface_id skipped at index 0 — the old code's
// start node had it uninitialized, see docs/paper-deltas.md).
bool check_path_matches_golden(const std::vector<Node*>& path, const std::string& golden_json_path);

} // namespace nas::fixtures
