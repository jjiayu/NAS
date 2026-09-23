// astar_plan — thin CLI driver for CASSR/AstarSearch (see PLAN.md phase
// 10: "drivers CLI minces"). Where tests/viz_dump/dump_plan.cpp is a test
// tool restricted to tests/fixtures's 2 hardcoded scenarios/configs, this
// is the production entrypoint: any scenario from config::available_scenarios()
// (11 today, plus whatever an STL import adds later — PLAN.md phase 9d)
// and any AstarSearchConfig/FootstepQPConfig loaded from a JSON file (see
// config::load_planner_config). Same output JSON shape as dump_plan.cpp on
// purpose, so both feed the same downstream SVG/plot render.

#include "nas/config/planner_config.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <chrono>
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;

namespace {

json point_json(const Point_3& p) {
    return json::array({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
}

// Talos's own forward polytopes — the only ones AstarSearch/footstep_qp
// need (see core/reachability's README: Antecedent is NAS/Tree-only).
ReachabilityModel load_forward_reachability(const std::string& talos_data_dir) {
    std::vector<ReachabilityEntry> entries = {
        {talos_data_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {talos_data_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <scenario_name> <planner_config.json> <talos_reachability_data_dir> <output.json>\n";
        std::cerr << "  scenario_name: one of config::available_scenarios() — see config/README.md\n";
        return 1;
    }
    std::string scenario_name = argv[1];
    std::string planner_config_path = argv[2];
    std::string talos_data_dir = argv[3];
    std::string out_path = argv[4];

    config::Scenario scenario;
    config::PlannerConfig planner_config;
    try {
        scenario = config::load_scenario(scenario_name);
        planner_config = config::load_planner_config(planner_config_path);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (planner_config.goal_offset) {
        planner_config.astar.goal_location = scenario.surfaces.back().centroid + *planner_config.goal_offset;
    }

    ReachabilityModel reachability = load_forward_reachability(talos_data_dir);

    using Clock = std::chrono::steady_clock;
    AstarSearch search(scenario.surfaces, reachability, planner_config.astar);
    auto t0 = Clock::now();
    search.search();
    double search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    const auto& path = search.result_path();

    QuadprogBackend backend;
    FootstepPlan plan;
    double qp_ms = 0.0;
    if (!path.empty()) {
        auto q0 = Clock::now();
        plan = solve_footstep_qp(path, planner_config.astar.start_position, planner_config.astar.goal_location,
                                 reachability, planner_config.qp, backend);
        qp_ms = std::chrono::duration<double, std::milli>(Clock::now() - q0).count();
    }

    json out;
    out["scenario"] = scenario_name;
    out["path_found"] = !path.empty();
    out["qp_success"] = plan.success;
    out["start"] = point_json(planner_config.astar.start_position);
    out["goal"] = point_json(planner_config.astar.goal_location);
    out["expansions"] = search.expansion_count();
    out["search_ms"] = search_ms;
    out["qp_ms"] = qp_ms;

    json surfaces_json = json::array();
    for (const auto& s : scenario.surfaces) {
        json verts = json::array();
        for (const auto& v : s.vertices_3d) verts.push_back(point_json(v));
        surfaces_json.push_back(verts);
    }
    out["surfaces"] = surfaces_json;

    json path_json = json::array();
    for (const auto* n : path) {
        json node_j;
        node_j["depth"] = n->depth;
        node_j["stance_foot"] = static_cast<int>(n->stance_foot);
        node_j["foot_yaw"] = n->foot_yaw;
        node_j["surface_id"] = n->surface_id;
        node_j["centroid"] = point_json(n->centroid);
        json patch = json::array();
        for (const auto& v : n->patch_vertices) patch.push_back(point_json(v));
        node_j["patch_vertices"] = patch;
        path_json.push_back(node_j);
    }
    out["path"] = path_json;

    json footsteps_json = json::array();
    if (plan.success) {
        for (size_t i = 0; i < plan.footsteps.size(); ++i) {
            json fs;
            fs["position"] = point_json(plan.footsteps[i]);
            fs["stance_foot"] = static_cast<int>(path[i]->stance_foot);
            fs["foot_yaw"] = path[i]->foot_yaw;
            footsteps_json.push_back(fs);
        }
    }
    out["footsteps"] = footsteps_json;

    std::ofstream file(out_path);
    file << out.dump(2);
    std::cout << "Wrote " << out_path << " (scenario=" << scenario_name << ", path size " << path.size()
              << ", qp_success=" << plan.success << ", " << search.expansion_count() << " expansions, " << search_ms << " ms)\n";
    if (path.empty()) std::cerr << "No path found.\n";
    return path.empty() ? 1 : 0;
}
