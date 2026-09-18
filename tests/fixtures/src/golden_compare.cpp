#include "nas/fixtures/golden_compare.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>

namespace nas::fixtures {

namespace {
bool close(double a, double b, double eps) { return std::abs(a - b) < eps; }
} // namespace

bool check_path_matches_golden(const std::vector<Node*>& path, const std::string& golden_json_path) {
    using json = nlohmann::json;
    bool all_ok = true;

    auto check = [&](bool cond, const std::string& what) {
        if (!cond) {
            std::cerr << "FAIL: " << what << "\n";
            all_ok = false;
        } else {
            std::cout << "ok: " << what << "\n";
        }
    };

    check(!path.empty(), "search finds a path");

    std::ifstream golden_file(golden_json_path);
    check(golden_file.is_open(), "golden reference file opens: " + golden_json_path);
    if (!golden_file.is_open() || path.empty()) {
        return false;
    }

    json golden;
    golden_file >> golden;
    check(golden.value("success", false), "golden reference itself recorded success=true (sanity check)");

    const auto& golden_nodes = golden["nodes"];
    check(path.size() == golden_nodes.size(),
          "path length matches golden exactly (" + std::to_string(path.size()) +
          " vs golden " + std::to_string(golden_nodes.size()) + ")");

    size_t n = std::min(path.size(), golden_nodes.size());
    bool all_match = true;
    for (size_t i = 0; i < n; ++i) {
        const auto& gnode = golden_nodes[i];
        const Node* node = path[i];

        bool depth_ok = node->depth == gnode.value("depth", -999);
        bool stance_ok = static_cast<int>(node->stance_foot) == gnode.value("stance_foot", -999);
        bool yaw_ok = close(node->foot_yaw, gnode.value("foot_yaw", 1e9), 1e-3);
        // The old code's start node had surface_id uninitialized; the
        // golden capture nulled it deliberately (see docs/paper-deltas.md).
        // The new Node correctly defaults to -1 instead — skip index 0.
        bool surface_ok = (i == 0) || (node->surface_id == gnode.value("surface_id", -999));

        if (!(depth_ok && stance_ok && yaw_ok && surface_ok)) {
            all_match = false;
            std::cerr << "  mismatch at index " << i
                      << ": depth(" << depth_ok << ") stance(" << stance_ok
                      << ") yaw(" << yaw_ok << ") surface(" << surface_ok << ")"
                      << " -- new: depth=" << node->depth << " stance=" << static_cast<int>(node->stance_foot)
                      << " yaw=" << node->foot_yaw << " surface_id=" << node->surface_id
                      << " -- golden: " << gnode.dump() << "\n";
        }
    }
    check(all_match, "every node's (depth, stance_foot, foot_yaw, surface_id) matches the golden reference exactly");

    return all_ok;
}

} // namespace nas::fixtures
