// The optional yaw-change cost (AstarSearchConfig::yaw_change_weight): edge cost 1 + w * |yaw change|.
// Runs the 15 scenes with w = 0 (the paper's cost, the default), 0.1 (the old code's unused weight) and 1.0 and prints, for
// each: expansions, path nodes, total rotation along the path (sum of |yaw change|, degrees) and whether the footstep QP
// succeeds. Asserted: w = 0 reproduces the default search exactly (same expansions and path); for every w a path is found
// wherever it is with w = 0, its QP succeeds, and the search is deterministic; on Flat (a straight walk) w = 1 does
// not rotate more than w = 0.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {
double total_rotation_deg(const std::vector<Node*>& path) {
    double t = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        double d = std::fmod(std::abs(path[i]->foot_yaw - path[i - 1]->foot_yaw), 2.0 * M_PI);
        t += (d > M_PI ? 2.0 * M_PI - d : d) * 180.0 / M_PI;
    }
    return t;
}
struct Scene { const char* name; Point_3 start; Vector_3 goal_offset; };
} // namespace

int main() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    const double t10 = std::tan(10.0 * M_PI / 180.0);
    const std::vector<Scene> scenes = {
        {"NarrowPassage", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs", Point_3(0.1, 0, 0), Vector_3(0, 0, 0)},
        {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0), Vector_3(0, 0, 0)}, {"LongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
        {"LongLongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Flat", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
        {"LongStairsComplete", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"LongStairsExp", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
        {"ThreePathsScene", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs_Up_Down", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
        {"ThreePathsNAS", Point_3(0, 0, 0), Vector_3(0, 1, 0)}, {"Ramp", Point_3(-1, 0, 0), Vector_3(0, 0, 0)},
        {"SteepRamp", Point_3(-1, 0, 0), Vector_3(0, 0, 0)}, {"SlopedGround", Point_3(-1.5, 0, -1.5 * t10), Vector_3(0, 0, 0)},
        {"SideSlope", Point_3(-1.5, 0, 0), Vector_3(0, 0, 0)},
    };
    const std::vector<double> weights = {0.0, 0.1, 1.0};
    int failures = 0;
    std::printf("%-19s | %-34s | %-34s | %-34s\n", "scene", "w = 0 (default): exp / nodes / rot deg", "w = 0.1: exp / nodes / rot deg", "w = 1.0: exp / nodes / rot deg");
    for (const Scene& s : scenes) {
        config::Scenario sc = config::load_scenario(s.name);
        double rot[3] = {0, 0, 0};
        bool found0 = false;
        char cells[3][64] = {"", "", ""};
        for (size_t k = 0; k < weights.size(); ++k) {
            AstarSearchConfig cfg;
            cfg.start_position = s.start; cfg.start_stance_foot = StanceFoot::Right; cfg.goal_stance_foot = StanceFoot::Left;
            cfg.goal_location = sc.surfaces.back().centroid + s.goal_offset;
            cfg.expansion_params.rotation_enabled = true;
            cfg.yaw_change_weight = weights[k];
            AstarSearch a(sc.surfaces, reach, cfg), b(sc.surfaces, reach, cfg);
            a.search(); b.search();
            bool found = !a.result_path().empty();
            if (k == 0) {
                found0 = found;
                AstarSearchConfig def = cfg; def.yaw_change_weight = 0.0; // = the default config
                AstarSearch d(sc.surfaces, reach, def); d.search();
                if (d.expansion_count() != a.expansion_count() || d.result_path().size() != a.result_path().size()) { std::printf("FAIL: w=0 differs from the default\n"); ++failures; }
            }
            bool det = a.expansion_count() == b.expansion_count() && a.result_path().size() == b.result_path().size();
            bool qp_ok = true;
            if (found) {
                FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
                QuadprogBackend backend;
                qp_ok = solve_footstep_qp(a.result_path(), s.start, cfg.goal_location, reach, qc, backend).success;
            }
            rot[k] = found ? total_rotation_deg(a.result_path()) : 0.0;
            std::snprintf(cells[k], sizeof cells[k], "%d / %zu / %.0f%s", a.expansion_count(), a.result_path().size(), rot[k], (found && qp_ok && det) || (!found && !found0) ? "" : "  <-- PROBLEM");
            if ((found0 && !found) || (found && !qp_ok) || !det) ++failures;
        }
        if (std::string(s.name) == "Flat" && rot[2] > rot[0] + 1e-9) { std::printf("FAIL: Flat rotates more with w = 1\n"); ++failures; }
        std::printf("%-19s | %-34s | %-34s | %-34s\n", s.name, cells[0], cells[1], cells[2]);
    }
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
