// Search + QP timing on every scenario, same configuration as nas_golden_all_scenes.
// Written so the same source builds in a checkout of an older commit (e.g. the tag
// legacy-replay-verified, whose defaults are the old behaviour): that gives a baseline
// measured on the same machine with the same tool. Usage: nas_perf_all_scenes [runs]
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace nas;
using Clock = std::chrono::steady_clock;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {
struct SceneSetup { const char* name; Point_3 start; Vector_3 goal_offset; };
const std::vector<SceneSetup> kScenes = {
    {"NarrowPassage", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs", Point_3(0.1, 0, 0), Vector_3(0, 0, 0)},
    {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0), Vector_3(0, 0, 0)}, {"LongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"LongLongStairs", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Flat", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"LongStairsComplete", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"LongStairsExp", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"ThreePathsScene", Point_3(0, 0, 0), Vector_3(0, 0, 0)}, {"Stairs_Up_Down", Point_3(0, 0, 0), Vector_3(0, 0, 0)},
    {"ThreePathsNAS", Point_3(0, 0, 0), Vector_3(0, 1, 0)},
};
} // namespace

int main(int argc, char** argv) {
    int runs = argc > 1 ? std::stoi(argv[1]) : 10;
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    std::printf("%-19s %10s %8s %6s %10s\n", "scene", "search ms", "+/-", "exp", "QP ms");
    double total = 0;
    for (const auto& s : kScenes) {
        config::Scenario sc = config::load_scenario(s.name);
        Point_3 goal = sc.surfaces.back().centroid + s.goal_offset;
        AstarSearchConfig c;
        c.start_position = s.start; c.start_stance_foot = StanceFoot::Right; c.goal_location = goal; c.goal_stance_foot = StanceFoot::Left;
        c.heuristic_weight = 10.0; c.node_similarity_threshold = 0.02;
        c.expansion_params.rotation_enabled = true; c.expansion_params.yaw_discretization_num = 3;
        c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI; c.expansion_params.cycle_detection_enabled = true;
        std::vector<double> t, q;
        int exp = 0;
        for (int r = 0; r < runs; ++r) {
            AstarSearch search(sc.surfaces, reach, c);
            auto t0 = Clock::now();
            search.search();
            t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
            exp = search.expansion_count();
            if (!search.result_path().empty()) {
                FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
                QuadprogBackend backend;
                auto q0 = Clock::now();
                solve_footstep_qp(search.result_path(), s.start, goal, reach, qc, backend);
                q.push_back(std::chrono::duration<double, std::milli>(Clock::now() - q0).count());
            }
        }
        auto mean = [](const std::vector<double>& v) { double a = 0; for (double x : v) a += x; return v.empty() ? 0.0 : a / v.size(); };
        double m = mean(t), sd = 0;
        for (double x : t) sd += (x - m) * (x - m);
        sd = std::sqrt(sd / t.size());
        total += m;
        std::printf("%-19s %10.2f %8.2f %6d %10.2f\n", s.name, m, sd, exp, mean(q));
    }
    std::printf("TOTAL search %.1f ms\n", total);
    return 0;
}
