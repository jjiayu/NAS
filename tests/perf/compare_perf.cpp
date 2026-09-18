// Phase 8b — performance comparison: runs the new AstarSearch +
// solve_footstep_qp several times per scenario (for a stable mean/stddev,
// same spirit as the old test_bench_operations.cpp) and prints them next
// to phase 0's golden timing_ms for a human-readable comparison. Not a
// pass/fail test — "perf comparable" per PLAN.md's Stage A exit criterion
// is a judgment call, not an exact-match assertion.

#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace nas;

namespace {

struct Stats {
    double mean_ms = 0.0;
    double stddev_ms = 0.0;
};

Stats compute_stats(const std::vector<double>& samples_ms) {
    Stats s;
    s.mean_ms = std::accumulate(samples_ms.begin(), samples_ms.end(), 0.0) / samples_ms.size();
    double sq_sum = 0.0;
    for (double v : samples_ms) sq_sum += (v - s.mean_ms) * (v - s.mean_ms);
    s.stddev_ms = std::sqrt(sq_sum / samples_ms.size());
    return s;
}

ReachabilityModel make_forward_model(const std::string& talos_dir) {
    std::vector<ReachabilityEntry> entries = {
        {talos_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {talos_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

std::vector<Surface> make_narrow_passage_surfaces() {
    std::vector<std::vector<Point_3>> raw = {
        {Point_3(-2.0, -2.0, 0.0), Point_3(2.0, -2.0, 0.0), Point_3(2.0, 2.0, 0.0), Point_3(-2.0, 2.0, 0.0)},
        {Point_3(2.0, -0.12, 0.0), Point_3(6.0, -0.12, 0.0), Point_3(6.0, 0.12, 0.0), Point_3(2.0, 0.12, 0.0)},
        {Point_3(6.0, -2.0, 0.0), Point_3(10.0, -2.0, 0.0), Point_3(10.0, 2.0, 0.0), Point_3(6.0, 2.0, 0.0)},
    };
    std::vector<Surface> surfaces;
    for (size_t i = 0; i < raw.size(); ++i) surfaces.emplace_back(raw[i], static_cast<int>(i), 0.22, 0.22);
    return surfaces;
}

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
    for (size_t i = 0; i < raw.size(); ++i) surfaces.emplace_back(raw[i], static_cast<int>(i), 0.22, 0.22);
    return surfaces;
}

AstarSearchConfig make_config(const Point_3& goal_offset_applied_goal) {
    AstarSearchConfig config;
    config.start_position = Point_3(0.0, 0.0, 0.0);
    config.start_stance_foot = StanceFoot::Right;
    config.goal_stance_foot = StanceFoot::Left;
    config.goal_location = goal_offset_applied_goal;
    config.distance_metric = DistanceMetric::Epa;
    config.heuristic_weight = 10.0;
    config.node_similarity_threshold = 0.02;
    config.expansion_params.rotation_enabled = true;
    config.expansion_params.yaw_discretization_num = 3;
    config.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    config.expansion_params.cycle_detection_enabled = true;
    return config;
}

void run_scenario(const std::string& name, const std::vector<Surface>& surfaces, const ReachabilityModel& reachability,
                   double golden_search_ms, int golden_expansions, double golden_qp_ms, int num_runs) {
    Point_3 c = surfaces.back().centroid;
    Point_3 goal = (name == "ThreePathsNAS")
                       ? Point_3(CGAL::to_double(c.x()), CGAL::to_double(c.y()) + 1.0, CGAL::to_double(c.z()))
                       : c;
    AstarSearchConfig config = make_config(goal);

    std::vector<double> search_times_ms;
    std::vector<double> qp_times_ms;
    int expansions = 0;
    bool qp_success = false;

    for (int run = 0; run < num_runs; ++run) {
        AstarSearch search(surfaces, reachability, config);
        auto t0 = std::chrono::high_resolution_clock::now();
        search.search();
        auto t1 = std::chrono::high_resolution_clock::now();
        search_times_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        expansions = search.expansion_count();

        const auto& path = search.result_path();
        if (!path.empty()) {
            FootstepQPConfig qp_config;
            qp_config.alpha_weight = 10.0;
            qp_config.rotation_enabled = true;
            QuadprogBackend backend;

            auto qt0 = std::chrono::high_resolution_clock::now();
            FootstepPlan plan = solve_footstep_qp(path, config.start_position, config.goal_location, reachability, qp_config, backend);
            auto qt1 = std::chrono::high_resolution_clock::now();
            if (plan.success) {
                qp_times_ms.push_back(std::chrono::duration<double, std::milli>(qt1 - qt0).count());
                qp_success = true;
            }
        }
    }

    Stats search_stats = compute_stats(search_times_ms);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== " << name << " (n=" << num_runs << " runs) ===\n";
    std::cout << "  search: new " << search_stats.mean_ms << " +/- " << search_stats.stddev_ms
              << " ms  vs  golden " << golden_search_ms << " ms  (ratio "
              << search_stats.mean_ms / golden_search_ms << "x)\n";
    std::cout << "  expansions: new " << expansions << "  vs  golden " << golden_expansions << "\n";
    if (qp_success) {
        Stats qp_stats = compute_stats(qp_times_ms);
        std::cout << "  QP (quadprog): new " << qp_stats.mean_ms << " +/- " << qp_stats.stddev_ms
                  << " ms  vs  golden (casadi/qpoases) " << golden_qp_ms << " ms  (ratio "
                  << qp_stats.mean_ms / golden_qp_ms << "x)\n";
    } else {
        std::cout << "  QP: no successful solve to time in this run\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <talos_reachability_data_dir>\n";
        return 1;
    }
    std::string talos_dir = argv[1];
    ReachabilityModel reachability = make_forward_model(talos_dir);
    const int num_runs = 20;

    run_scenario("NarrowPassage", make_narrow_passage_surfaces(), reachability,
                 /*golden_search_ms=*/159.896173, /*golden_expansions=*/90, /*golden_qp_ms=*/0.0, num_runs);
    run_scenario("ThreePathsNAS", make_three_paths_nas_surfaces(), reachability,
                 /*golden_search_ms=*/68.122904, /*golden_expansions=*/91, /*golden_qp_ms=*/41.510454, num_runs);

    std::cout << "\nNote: NarrowPassage's golden QP was infeasible in the old code too "
                 "(see tests/golden/NarrowPassage_astar.json) — its QP timing isn't compared.\n";
    return 0;
}
