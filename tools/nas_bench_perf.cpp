// Search + QP timing on every scenario, same configuration as nas_golden_all_scenes.
// Written so the same source builds in a checkout of an older commit (e.g. the tag
// legacy-replay-verified, whose defaults are the old behaviour): that gives a baseline
// measured on the same machine with the same tool. Usage: nas_perf_all_scenes [runs]
//
// Each scene is now timed TWICE: once as before (single target, "1 cible" — the timed loop and its
// config are byte-for-byte what they were before foot_goals existed, so this half of the table stays
// a valid before/after comparison across this feature's introduction), and once with
// AstarSearchConfig::foot_goals' "closing stance" mode (both feet targeted, "2 cibles"), to measure
// this feature's own cost where it's actually used. The 2-cibles targets are built the same way
// tests/golden/dual_target_all_scenes_test.cpp does: a small region around each of the last two
// nodes of the (already computed, untimed) 1-cible path — a solution is then guaranteed to exist by
// construction, so this measures the harder joint search, not a search for something unreachable.
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

AstarSearchConfig base_config(const SceneSetup& s, const Point_3& goal) {
    AstarSearchConfig c;
    c.start_position = s.start; c.start_stance_foot = StanceFoot::Right; c.goal_location = goal; c.goal_stance_foot = StanceFoot::Left;
    c.heuristic_weight = 10.0; c.node_similarity_threshold = 0.02;
    c.expansion_params.rotation_enabled = true; c.expansion_params.yaw_discretization_num = 3;
    c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI; c.expansion_params.cycle_detection_enabled = true;
    return c;
}

// Same helper as dual_target_all_scenes_test.cpp: a flat square centered exactly on an
// already-reachable node's own centroid/yaw, generous margins.
AstarSearchConfig::FootGoal target_around(const Node* n, double half_extent, double yaw_half_width_deg) {
    double x = CGAL::to_double(n->centroid.x()), y = CGAL::to_double(n->centroid.y()), z = CGAL::to_double(n->centroid.z());
    AstarSearchConfig::FootGoal g;
    g.region = std::vector<Point_3>{Point_3(x - half_extent, y - half_extent, z), Point_3(x + half_extent, y - half_extent, z),
                                     Point_3(x + half_extent, y + half_extent, z), Point_3(x - half_extent, y + half_extent, z)};
    double yaw_deg = n->foot_yaw * 180.0 / M_PI;
    g.yaw_range = std::make_pair((yaw_deg - yaw_half_width_deg) / 180.0 * M_PI, (yaw_deg + yaw_half_width_deg) / 180.0 * M_PI);
    return g;
}

double mean_of(const std::vector<double>& v) {
    double a = 0;
    for (double x : v) a += x;
    return v.empty() ? 0.0 : a / v.size();
}
double stddev_of(const std::vector<double>& v, double m) {
    double sd = 0;
    for (double x : v) sd += (x - m) * (x - m);
    return v.empty() ? 0.0 : std::sqrt(sd / v.size());
}

} // namespace

int main(int argc, char** argv) {
    int runs = argc > 1 ? std::stoi(argv[1]) : 10;
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    std::printf("%-19s %5s %10s %8s %6s %10s\n", "scene", "buts", "search ms", "+/-", "exp", "QP ms");
    double total = 0;
    for (const auto& s : kScenes) {
        config::Scenario sc = config::load_scenario(s.name);
        Point_3 goal = sc.surfaces.back().centroid + s.goal_offset;
        AstarSearchConfig c = base_config(s, goal);

        std::vector<double> t, q;
        int exp = 0;
        Node* last = nullptr;
        Node* prev = nullptr;
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
                if (search.result_path().size() >= 2 && r == runs - 1) {
                    // Kept from the LAST timed run only, to derive the 2-cibles targets below — this
                    // does not affect the 1-cible timing loop above in any way (nothing here is timed).
                    last = search.result_path().back();
                    prev = search.result_path()[search.result_path().size() - 2];
                }
            }
        }
        double m = mean_of(t);
        total += m;
        std::printf("%-19s %5s %10.2f %8.2f %6d %10.2f\n", s.name, "1", m, stddev_of(t, m), exp, mean_of(q));

        if (last == nullptr) {
            std::printf("%-19s %5s %10s\n", s.name, "2", "n/a (pas de chemin a 1 cible pour en construire 2)");
            continue;
        }

        AstarSearchConfig c2 = base_config(s, goal);
        c2.max_expansions = 50000; // safety net only, see dual_target_all_scenes_test.cpp
        c2.foot_goals[static_cast<size_t>(last->stance_foot)] = target_around(last, 0.15, 40.0);
        c2.foot_goals[static_cast<size_t>(prev->stance_foot)] = target_around(prev, 0.15, 40.0);

        std::vector<double> t2, q2;
        int exp2 = 0;
        for (int r = 0; r < runs; ++r) {
            AstarSearch search(sc.surfaces, reach, c2);
            auto t0 = Clock::now();
            search.search();
            t2.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
            exp2 = search.expansion_count();
            if (!search.result_path().empty()) {
                FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
                QuadprogBackend backend;
                auto q0 = Clock::now();
                solve_footstep_qp(search.result_path(), s.start, std::nullopt, reach, qc, backend);
                q2.push_back(std::chrono::duration<double, std::milli>(Clock::now() - q0).count());
            }
        }
        double m2 = mean_of(t2);
        std::printf("%-19s %5s %10.2f %8.2f %6d %10.2f\n", s.name, "2", m2, stddev_of(t2, m2), exp2, mean_of(q2));
    }
    std::printf("TOTAL search (1 cible) %.1f ms\n", total);
    return 0;
}
