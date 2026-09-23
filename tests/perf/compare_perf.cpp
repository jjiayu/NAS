// Phase 8b — performance comparison: runs the new AstarSearch +
// solve_footstep_qp several times per scenario (for a stable mean/stddev,
// same spirit as the old test_bench_operations.cpp) and prints them next
// to phase 0's golden timing_ms for a human-readable comparison. Not a
// pass/fail test — "perf comparable" per PLAN.md's Stage A exit criterion
// is a judgment call, not an exact-match assertion. Scenario setup lives
// in tests/fixtures (phase 8d-2).

#include "nas/fixtures/scenarios.hpp"
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

void run_scenario(const fixtures::Scenario& scenario, const ReachabilityModel& reachability,
                   double golden_search_ms, int golden_expansions, double golden_qp_ms, int num_runs) {
    std::vector<double> search_times_ms;
    std::vector<double> qp_times_ms;
    int expansions = 0;
    bool qp_success = false;

    for (int run = 0; run < num_runs; ++run) {
        AstarSearch search(scenario.surfaces, reachability, scenario.astar_config);
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
            FootstepPlan plan = solve_footstep_qp(path, scenario.astar_config.start_position,
                                                   scenario.astar_config.goal_location, reachability, qp_config, backend);
            auto qt1 = std::chrono::high_resolution_clock::now();
            if (plan.success) {
                qp_times_ms.push_back(std::chrono::duration<double, std::milli>(qt1 - qt0).count());
                qp_success = true;
            }
        }
    }

    Stats search_stats = compute_stats(search_times_ms);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=== " << scenario.name << " (n=" << num_runs << " runs) ===\n";
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

// A/B of the 2D clip: same search, legacy clip (old behaviour) vs corrected
// clip, runs interleaved so machine drift hits both equally.
void compare_clip(const fixtures::Scenario& scenario, const ReachabilityModel& reachability, int num_runs) {
    std::vector<double> legacy_ms, robust_ms;
    int legacy_exp = 0, robust_exp = 0;
    for (int run = 0; run < num_runs; ++run) {
        for (bool legacy : {true, false}) {
            AstarSearchConfig cfg = scenario.astar_config;
            cfg.expansion_params.legacy_clip = legacy;
            AstarSearch search(scenario.surfaces, reachability, cfg);
            auto t0 = std::chrono::high_resolution_clock::now();
            search.search();
            auto t1 = std::chrono::high_resolution_clock::now();
            (legacy ? legacy_ms : robust_ms).push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            (legacy ? legacy_exp : robust_exp) = search.expansion_count();
        }
    }
    Stats l = compute_stats(legacy_ms), r = compute_stats(robust_ms);
    std::cout << "  clip A/B " << scenario.name << ": legacy " << l.mean_ms << " +/- " << l.stddev_ms << " ms (" << legacy_exp
              << " expansions)  vs  robust " << r.mean_ms << " +/- " << r.stddev_ms << " ms (" << robust_exp << " expansions)  ratio "
              << r.mean_ms / l.mean_ms << "x\n";
}

// A/B of the node-key variants (ExpansionParams::convex_patch / canonical_*),
// interleaved, on the default (robust) clip.
void compare_keys(const fixtures::Scenario& scenario, const ReachabilityModel& reachability, int num_runs) {
    struct Variant { const char* name; bool convex, centroid, det; std::vector<double> ms; int exp = 0; };
    std::vector<Variant> variants = {{"old keys", false, false, false, {}},
                                     {"convex patch + area centroid", true, true, false, {}},
                                     {"... + simplify/canonical start (hts c)", true, true, false, {}},
                                     {"... + deterministic ties (htscd)", true, true, true, {}}};
    for (int run = 0; run < num_runs; ++run) {
        for (size_t k = 0; k < variants.size(); ++k) {
            Variant& v = variants[k];
            AstarSearchConfig cfg = scenario.astar_config;
            cfg.expansion_params.convex_patch = v.convex;
            cfg.expansion_params.canonical_centroid = v.centroid;
            if (k >= 2) {
                cfg.expansion_params.convex_patch_simplify_tol = 1e-9;
                cfg.expansion_params.canonical_prism_start = true;
            }
            cfg.deterministic_ties = v.det;
            AstarSearch search(scenario.surfaces, reachability, cfg);
            auto t0 = std::chrono::high_resolution_clock::now();
            search.search();
            auto t1 = std::chrono::high_resolution_clock::now();
            v.ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            v.exp = search.expansion_count();
        }
    }
    double base = compute_stats(variants[0].ms).mean_ms;
    for (auto& v : variants) {
        Stats st = compute_stats(v.ms);
        std::cout << "  keys A/B " << scenario.name << " [" << v.name << "]: " << st.mean_ms << " +/- " << st.stddev_ms << " ms (" << v.exp
                  << " expansions, " << st.mean_ms / base << "x)  " << st.mean_ms / v.exp << " ms/expansion\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <talos_reachability_data_dir>\n";
        return 1;
    }
    ReachabilityModel reachability = fixtures::make_forward_reachability_model(argv[1]);
    const int num_runs = 20;

    run_scenario(fixtures::make_narrow_passage(), reachability,
                 /*golden_search_ms=*/159.896173, /*golden_expansions=*/90, /*golden_qp_ms=*/0.0, num_runs);
    run_scenario(fixtures::make_three_paths_nas(), reachability,
                 /*golden_search_ms=*/68.122904, /*golden_expansions=*/91, /*golden_qp_ms=*/41.510454, num_runs);

    std::cout << "\n--- 2D clip: legacy vs corrected (interleaved, n=" << num_runs << ") ---\n";
    compare_clip(fixtures::make_narrow_passage(), reachability, num_runs);
    compare_clip(fixtures::make_three_paths_nas(), reachability, num_runs);

    std::cout << "\n--- node keys: old vs convex patch (interleaved, n=" << num_runs << ") ---\n";
    compare_keys(fixtures::make_narrow_passage(), reachability, num_runs);
    compare_keys(fixtures::make_three_paths_nas(), reachability, num_runs);

    std::cout << "\nNote: NarrowPassage's golden QP was infeasible in the old code too "
                 "(see tests/golden/NarrowPassage_astar.json) — its QP timing isn't compared.\n";
    return 0;
}
