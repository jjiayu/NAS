// golden_capture.cpp — Stage A phase 0 (see PLAN.md).
//
// Headless capture tool for the CURRENT (pre-rewrite) NAS/CASSR implementation.
// Runs AstarSearch, Tree (NAS) and FootstepPlanner exactly as astar_plan.cpp /
// nas_plan.cpp do, but dumps structured JSON instead of opening a VTK window,
// so it can run unattended across every scenario in environments.hpp.
//
// Temporary tool: lives only in the pre-rewrite repo, not carried into the
// rewrite (same status as test_bench_operations.cpp / test_print_paths.cpp).
// Scenario/start position are still selected at compile time via
// constants.hpp, exactly like the rest of this repo today — this tool does
// not change that, it is driven by capture_golden_references.sh which edits
// constants.hpp, rebuilds, and re-runs this binary per scenario.
//
// Usage: golden_capture <scenario_name> <git_sha> <output_dir>

#include "tree.hpp"
#include "astar_search.hpp"
#include "footstep_planner.hpp"
#include "node.hpp"
#include "constants.hpp"

#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

double elapsed_ms(std::chrono::high_resolution_clock::time_point t0,
                   std::chrono::high_resolution_clock::time_point t1) {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

json node_to_json(const Node* n) {
    return json{
        {"node_id", n->node_id},
        {"depth", n->depth},
        {"surface_id", n->surface_id},
        {"stance_foot", n->stance_foot},
        {"foot_yaw", n->foot_yaw},
        {"centroid", {CGAL::to_double(n->centroid.x()),
                      CGAL::to_double(n->centroid.y()),
                      CGAL::to_double(n->centroid.z())}}
    };
}

json path_to_json(const std::vector<Node*>& path) {
    json arr = json::array();
    for (const Node* n : path) arr.push_back(node_to_json(n));
    return arr;
}

// Runs FootstepPlanner::plan on one path and records its result, timing the
// call independently since FootstepPlanner does not expose the QP duration
// it already measures internally (see docs/paper-deltas.md).
json run_footstep_plan(FootstepPlanner& fp,
                        int start_foot, const Point_3& start_pos,
                        int goal_foot, const Point_3& goal_pos,
                        const std::vector<Node*>& path) {
    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok = fp.plan(start_foot, start_pos, goal_foot, goal_pos, path);
    auto t1 = std::chrono::high_resolution_clock::now();

    json j;
    j["success"] = ok;
    j["qp_time_ms"] = elapsed_ms(t0, t1);
    json footsteps = json::array();
    if (ok) {
        for (const auto& p : fp.get_computed_footsteps()) {
            footsteps.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
        }
    }
    j["footsteps"] = footsteps;
    return j;
}

json capture_astar(const std::string& scenario_name, const std::string& git_sha) {
    AstarSearch astar_search;
    FootstepPlanner footstep_planner;

    auto t0 = std::chrono::high_resolution_clock::now();
    astar_search.search();
    auto t1 = std::chrono::high_resolution_clock::now();

    json j;
    j["scenario"] = scenario_name;
    j["planner"] = "astar";
    j["git_sha"] = git_sha;
    j["success"] = !astar_search.result_path.empty();

    json nodes = path_to_json(astar_search.result_path);
    // AstarSearch's start node (index 0) never has surface_id set by the
    // constructor — reading it is uninitialized memory, not reproducible.
    // Null it out explicitly rather than capture non-deterministic noise.
    // Logged as a real gap in docs/paper-deltas.md.
    if (!nodes.empty()) nodes[0]["surface_id"] = nullptr;
    j["nodes"] = nodes;

    j["timing_ms"] = {
        {"search_total", elapsed_ms(t0, t1)},
        {"expansion_count", astar_search.expansion_coount},
        {"minkowski", astar_search.total_minkowski_time},
        {"clipping", astar_search.total_clipping_time},
        {"plane_polytope_intersect", astar_search.total_plane_polytope_intersect_time},
        {"polygon_2d_intersect", astar_search.total_polygon_2d_intersect_time}
    };

    if (!astar_search.result_path.empty()) {
        j["footstep_plan"] = run_footstep_plan(
            footstep_planner,
            current_stance_foot_flag, current_foot_pos,
            astar_search.goal_stance_foot, astar_search.goal_location,
            astar_search.result_path);
    }
    return j;
}

json capture_nas(const std::string& scenario_name, const std::string& git_sha) {
    Tree tree;
    FootstepPlanner footstep_planner;

    auto t0 = std::chrono::high_resolution_clock::now();
    tree.expand(tree.num_steps);
    auto t1 = std::chrono::high_resolution_clock::now();

    // Forced to bruteforce regardless of constants.hpp's node_search_method:
    // "knn" is a documented no-op and "kdtree" is a separate, less-trusted
    // code path — bruteforce is the only fully exercised one (see
    // docs/paper-deltas.md).
    std::vector<Node*> start_nodes =
        tree.find_nodes_containing_current_stance_foot_brute_force(current_stance_foot_flag, current_foot_pos);

    json j;
    j["scenario"] = scenario_name;
    j["planner"] = "nas";
    j["git_sha"] = git_sha;
    j["success"] = !start_nodes.empty();
    j["timing_ms"] = {
        {"expand_total", elapsed_ms(t0, t1)},
        {"node_count", tree.node_counter}
    };

    // Capture ALL paths at the minimal depth, not just one — free to do now
    // and makes this golden already usable for Stage B's completeness check
    // (B4) without a re-capture later.
    json paths_json = json::array();
    for (Node* node : start_nodes) {
        for (const auto& path : tree.find_paths_to_root(node)) {
            json entry;
            entry["nodes"] = path_to_json(path);
            entry["footstep_plan"] = run_footstep_plan(
                footstep_planner,
                current_stance_foot_flag, current_foot_pos,
                tree.goal_stance_foot, tree.goal_location,
                path);
            paths_json.push_back(entry);
        }
    }
    j["num_paths"] = paths_json.size();
    j["paths"] = paths_json;
    return j;
}

void write_json(const json& j, const fs::path& path) {
    std::ofstream out(path);
    out << j.dump(2);
}

} // namespace

// Runs one capture function and writes its JSON, or writes an error record
// instead of letting an exception (e.g. a missing/misnamed .obj asset —
// see docs/paper-deltas.md) kill the whole unattended multi-scenario run.
void run_and_write(const std::string& label,
                    const std::function<json()>& capture_fn,
                    const fs::path& out_path) {
    try {
        json j = capture_fn();
        write_json(j, out_path);
        std::cout << "[golden_capture] " << label << ": captured -> " << out_path << "\n";
    } catch (const std::exception& e) {
        json err;
        err["success"] = false;
        err["error"] = e.what();
        write_json(err, out_path);
        std::cerr << "[golden_capture] " << label << ": EXCEPTION - " << e.what() << "\n";
    }
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_name> <git_sha> <output_dir>\n";
        return 1;
    }
    const std::string scenario_name = argv[1];
    const std::string git_sha = argv[2];
    const fs::path output_dir = argv[3];
    fs::create_directories(output_dir);

    run_and_write("astar",
                  [&]() { return capture_astar(scenario_name, git_sha); },
                  output_dir / (scenario_name + "_astar.json"));

    run_and_write("nas",
                  [&]() { return capture_nas(scenario_name, git_sha); },
                  output_dir / (scenario_name + "_nas.json"));

    return 0;
}
