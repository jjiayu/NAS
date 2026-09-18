// Phase 5's actual deliverable: compare the ported AstarSearch against the
// golden reference captured from the OLD repo in phase 0
// (tests/golden/NarrowPassage_astar.json), on the exact same scenario.
//
// The NarrowPassage surfaces below are copied verbatim from the old
// include/environments.hpp — scenario data is still hardcoded here rather
// than loaded from config, since config-driven scenario loading is
// phase 9's job (RobotModel/Scenario), not phase 5's. foot_length/
// foot_width/goal are copied from the active values in the old
// constants.hpp at golden-capture time (see docs/paper-deltas.md re: the
// foot_width=0.22-vs-0.12 comment inconsistency — 0.22 is what was
// actually compiled in and captured).

#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::cerr << "FAIL: " << what << "\n";
        ++g_failures;
    } else {
        std::cout << "ok: " << what << "\n";
    }
}

bool close(double a, double b, double eps = 1e-6) { return std::abs(a - b) < eps; }

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

std::vector<Surface> make_narrow_passage_surfaces() {
    std::vector<std::vector<Point_3>> raw = {
        // Starting Floor
        {Point_3(-2.0, -2.0, 0.0), Point_3(2.0, -2.0, 0.0), Point_3(2.0, 2.0, 0.0), Point_3(-2.0, 2.0, 0.0)},
        // Passage
        {Point_3(2.0, -0.12, 0.0), Point_3(6.0, -0.12, 0.0), Point_3(6.0, 0.12, 0.0), Point_3(2.0, 0.12, 0.0)},
        // Last Floor
        {Point_3(6.0, -2.0, 0.0), Point_3(10.0, -2.0, 0.0), Point_3(10.0, 2.0, 0.0), Point_3(6.0, 2.0, 0.0)},
    };

    std::vector<Surface> surfaces;
    for (size_t i = 0; i < raw.size(); ++i) {
        surfaces.emplace_back(raw[i], static_cast<int>(i), /*foot_length=*/0.22, /*foot_width=*/0.22);
    }
    return surfaces;
}

ReachabilityModel make_forward_model() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    std::vector<ReachabilityEntry> entries = {
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

} // namespace

int main() {
    std::vector<Surface> surfaces = make_narrow_passage_surfaces();
    ReachabilityModel reachability = make_forward_model();

    AstarSearchConfig config;
    config.start_position = Point_3(0.0, 0.0, 0.0);
    config.start_stance_foot = StanceFoot::Right;   // current_stance_foot_flag = RIGHT_FOOT
    config.goal_stance_foot = StanceFoot::Left;      // stance_foot_at_goal = LEFT_FOOT
    config.goal_location = surfaces.back().centroid; // goal_offset = (0,0,0) in the old constants.hpp
    config.distance_metric = DistanceMetric::Epa;
    config.heuristic_weight = 10.0;
    config.node_similarity_threshold = 0.02;
    config.expansion_params.rotation_enabled = true;
    config.expansion_params.yaw_discretization_num = 3;
    config.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    config.expansion_params.cycle_detection_enabled = true;

    AstarSearch search(surfaces, reachability, config);
    search.search();

    const auto& path = search.result_path();
    check(!path.empty(), "search finds a path on NarrowPassage");

    std::string golden_path = std::string(GOLDEN_DATA_DIR) + "/NarrowPassage_astar.json";
    std::ifstream golden_file(golden_path);
    check(golden_file.is_open(), "golden reference file opens: " + golden_path);
    if (!golden_file.is_open()) {
        return 1; // nothing else to compare against
    }
    json golden;
    golden_file >> golden;

    check(golden.value("success", false), "golden reference itself recorded success=true (sanity check)");

    const auto& golden_nodes = golden["nodes"];
    check(path.size() == golden_nodes.size(),
          "path length matches golden exactly (" + std::to_string(path.size()) +
          " vs golden " + std::to_string(golden_nodes.size()) + ")");

    size_t compare_count = std::min(path.size(), golden_nodes.size());
    bool all_match = true;
    for (size_t i = 0; i < compare_count; ++i) {
        const auto& gnode = golden_nodes[i];
        const Node* n = path[i];

        bool depth_ok = n->depth == gnode.value("depth", -999);
        bool stance_ok = static_cast<int>(n->stance_foot) == gnode.value("stance_foot", -999);
        bool yaw_ok = close(n->foot_yaw, gnode.value("foot_yaw", 1e9), 1e-3);
        // The start node's surface_id was nulled in the golden capture
        // (uninitialized memory in the old code); the new Node correctly
        // defaults to -1 there instead — not a mismatch to flag, see
        // docs/paper-deltas.md. Compare surface_id only for depth >= 1.
        bool surface_ok = (i == 0) || (n->surface_id == gnode.value("surface_id", -999));

        if (!(depth_ok && stance_ok && yaw_ok && surface_ok)) {
            all_match = false;
            std::cerr << "  mismatch at index " << i
                      << ": depth(" << depth_ok << ") stance(" << stance_ok
                      << ") yaw(" << yaw_ok << ") surface(" << surface_ok << ")"
                      << " -- new: depth=" << n->depth << " stance=" << static_cast<int>(n->stance_foot)
                      << " yaw=" << n->foot_yaw << " surface_id=" << n->surface_id
                      << " -- golden: " << gnode.dump() << "\n";
        }
    }
    check(all_match, "every node's (depth, stance_foot, foot_yaw, surface_id) matches the golden reference exactly");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "AstarSearch matches the NarrowPassage golden reference exactly (" << path.size() << " nodes)\n";
    return 0;
}
