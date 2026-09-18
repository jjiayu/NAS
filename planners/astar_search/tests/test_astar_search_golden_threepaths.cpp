// Second golden comparison (see test_astar_search_golden.cpp for the
// rationale) — ThreePathsNAS exercises a different topology (branching /
// local minima) than NarrowPassage, for extra confidence in the port.
// Surfaces copied verbatim from the old include/environments.hpp.

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

std::vector<Surface> make_three_paths_nas_surfaces() {
    std::vector<std::vector<Point_3>> raw = {
        {Point_3(-0.3, -4.0, 0.0), Point_3(0.3, -4.0, 0.0), Point_3(0.3, 2.0, 0.0), Point_3(-0.3, 2.0, 0.0)},
        {Point_3(0.32, 1.4, 0.0), Point_3(0.72, 1.4, 0.0), Point_3(0.72, 2.0, 0.0), Point_3(0.32, 2.0, 0.0)},
        {Point_3(0.74, 1.4, 0.0), Point_3(1.32, 1.4, 0.0), Point_3(1.32, 2.0, 0.0), Point_3(0.74, 2.0, 0.0)},
        {Point_3(1.34, 1.4, 0.0), Point_3(2.52, 1.4, 0.0), Point_3(2.52, 2.0, 0.0), Point_3(1.34, 2.0, 0.0)},
        {Point_3(2.54, 1.4, 0.0), Point_3(3.12, 1.4, 0.0), Point_3(3.12, 2.0, 0.0), Point_3(2.54, 2.0, 0.0)},
        {Point_3(3.14, 1.4, 0.0), Point_3(3.72, 1.4, 0.0), Point_3(3.72, 2.0, 0.0), Point_3(3.14, 2.0, 0.0)},
        {Point_3(3.74, 1.4, 0.0), Point_3(4.32, 1.4, 0.0), Point_3(4.32, 2.0, 0.0), Point_3(3.74, 2.0, 0.0)},
        {Point_3(0.32, -3.2, 0.0), Point_3(1.2, -3.2, 0.0), Point_3(1.2, -4.0, 0.0), Point_3(0.32, -4.0, 0.0)},
        {Point_3(1.22, -3.2, 0.0), Point_3(2.2, -3.2, 0.0), Point_3(2.2, -4.0, 0.0), Point_3(1.22, -4.0, 0.0)},
        {Point_3(2.22, -3.2, 0.0), Point_3(3.2, -3.2, 0.0), Point_3(3.2, -4.0, 0.0), Point_3(2.22, -4.0, 0.0)},
        {Point_3(3.22, -3.2, 0.0), Point_3(4.32, -3.2, 0.0), Point_3(4.32, -4.0, 0.0), Point_3(3.22, -4.0, 0.0)},
        {Point_3(0.32, -1.0, 0.0), Point_3(1.5, -1.0, 0.0), Point_3(1.5, -2.0, 0.0), Point_3(0.32, -2.0, 0.0)},
        {Point_3(4.34, -4.0, 0.0), Point_3(4.94, -4.0, 0.0), Point_3(4.94, 2.0, 0.0), Point_3(4.34, 2.0, 0.0)},
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
    std::vector<Surface> surfaces = make_three_paths_nas_surfaces();
    ReachabilityModel reachability = make_forward_model();

    AstarSearchConfig config;
    config.start_position = Point_3(0.0, 0.0, 0.0);
    config.start_stance_foot = StanceFoot::Right;
    config.goal_stance_foot = StanceFoot::Left;
    // goal_offset = (0.0, 1.0, 0.0) for this scenario specifically, per the
    // old constants.hpp comment ("for 3path NAS goal offset...") and the
    // phase-0 capture script.
    Point_3 c = surfaces.back().centroid;
    config.goal_location = Point_3(CGAL::to_double(c.x()), CGAL::to_double(c.y()) + 1.0, CGAL::to_double(c.z()));
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
    check(!path.empty(), "search finds a path on ThreePathsNAS");

    std::string golden_path = std::string(GOLDEN_DATA_DIR) + "/ThreePathsNAS_astar.json";
    std::ifstream golden_file(golden_path);
    check(golden_file.is_open(), "golden reference file opens: " + golden_path);
    if (!golden_file.is_open()) {
        return 1;
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
    std::cout << "AstarSearch matches the ThreePathsNAS golden reference exactly (" << path.size() << " nodes)\n";
    return 0;
}
