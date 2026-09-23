// Visual-sanity export tool — the seed of phase 11's "static export for
// figures" half (viz/ also gets a meshcat-cpp interactive half later, see
// PLAN.md). Runs the new AstarSearch + solve_footstep_qp on a named
// fixtures::Scenario and dumps surfaces/path/footsteps as JSON, for a
// quick SVG/plot render — not the real-time viewer.
//
// Generalized in phase 8d-2 to take the scenario by name instead of being
// hardcoded to NarrowPassage — reuses the same fixtures every test/perf
// tool already depends on, so a new scenario only needs adding there.

#include "nas/fixtures/scenarios.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

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

fixtures::Scenario make_scenario(const std::string& name) {
    if (name == "NarrowPassage") return fixtures::make_narrow_passage();
    if (name == "ThreePathsNAS") return fixtures::make_three_paths_nas();
    throw std::invalid_argument("Unknown scenario '" + name + "' — known: NarrowPassage, ThreePathsNAS "
                                 "(add more to tests/fixtures/scenarios.{hpp,cpp} first)");
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_name> <talos_reachability_data_dir> <output.json>\n";
        std::cerr << "  scenario_name: NarrowPassage | ThreePathsNAS\n";
        return 1;
    }
    std::string scenario_name = argv[1];
    std::string talos_dir = argv[2];
    std::string out_path = argv[3];

    fixtures::Scenario scenario;
    try {
        scenario = make_scenario(scenario_name);
    } catch (const std::invalid_argument& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    ReachabilityModel reachability = fixtures::make_forward_reachability_model(talos_dir);

    AstarSearch search(scenario.surfaces, reachability, scenario.astar_config);
    search.search();
    const auto& path = search.result_path();
    if (path.empty()) {
        std::cerr << "No path found.\n";
        return 1;
    }

    FootstepQPConfig qp_config;
    qp_config.alpha_weight = 10.0;
    qp_config.rotation_enabled = true;
    QuadprogBackend backend;
    FootstepPlan plan = solve_footstep_qp(path, scenario.astar_config.start_position,
                                           scenario.astar_config.goal_location, reachability, qp_config, backend);

    json out;
    out["scenario"] = scenario_name;
    out["qp_success"] = plan.success;

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
              << ", qp_success=" << plan.success << ")\n";
    return 0;
}
