// Compares the two FootstepQP backends (QuadprogBackend/eiquadprog, ProxqpBackend/proxsuite) on
// the same 15 scenes and the same CASSR path per scene (searched once, solved with each backend):
// optimum (objective value, paper Eq. 6, unregularized H), feasibility (success + max_violation,
// FootstepQPConfig::feasibility_tolerance), and timing (mean +/- stddev of N alternated runs).
// Also a ctest: both backends solve the exact same convex QP (same H, g, constraints — the
// formulation is backend-agnostic), so they must agree on success and land on the same optimum;
// a mismatch means one of them is not actually solving the problem it was given.
// Informational for the backend choice too (see PLAN.md TODO: purge the library not kept once
// decided) — read the printed table for the numbers. Usage: nas_compare_qp_backends [runs]
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/proxqp_backend.hpp"
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
struct Scene { const char* name; Point_3 start; Vector_3 goal_offset; };

struct Stat { double mean = 0, sd = 0; };
Stat stats(const std::vector<double>& v) {
    Stat s;
    for (double x : v) s.mean += x;
    if (!v.empty()) s.mean /= v.size();
    for (double x : v) s.sd += (x - s.mean) * (x - s.mean);
    s.sd = v.empty() ? 0.0 : std::sqrt(s.sd / v.size());
    return s;
}

// One backend's result on one scene: feasibility, optimum, timing.
struct BackendResult {
    bool success = false;
    double max_violation = 0.0;
    double objective = 0.0;
    Stat time_ms;
};

BackendResult run_backend(QPBackend& backend, const std::vector<Node*>& path, const Point_3& start,
                           const Point_3& goal, const ReachabilityModel& reach, int runs) {
    BackendResult r;
    std::vector<double> t;
    FootstepQPConfig qc;
    qc.alpha_weight = 10.0;
    qc.rotation_enabled = true;
    for (int i = 0; i < runs; ++i) {
        auto t0 = Clock::now();
        FootstepPlan plan = solve_footstep_qp(path, start, goal, reach, qc, backend);
        t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
        if (i == 0) {
            r.success = plan.success;
            r.max_violation = plan.max_violation;
            r.objective = plan.objective;
        }
    }
    r.time_ms = stats(t);
    return r;
}
} // namespace

int main(int argc, char** argv) {
    int runs = argc > 1 ? std::stoi(argv[1]) : 10;
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

    QuadprogBackend quadprog;
    ProxqpBackend proxqp;

    std::printf("%-19s %6s | %-8s %-11s %-10s | %-8s %-11s %-10s | %-10s\n", "scene", "exp",
                "quadprog", "obj", "ms", "proxqp", "obj", "ms", "|d obj|");
    int mismatches = 0;
    for (const Scene& s : scenes) {
        config::Scenario sc = config::load_scenario(s.name);
        Point_3 goal = sc.surfaces.back().centroid + s.goal_offset;
        AstarSearchConfig cfg;
        cfg.start_position = s.start; cfg.start_stance_foot = StanceFoot::Right;
        cfg.goal_location = goal; cfg.goal_stance_foot = StanceFoot::Left;
        cfg.expansion_params.rotation_enabled = true;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        const auto& path = search.result_path();
        if (path.empty()) {
            std::printf("%-19s %6d | %-8s\n", s.name, search.expansion_count(), "no path");
            continue;
        }
        BackendResult q = run_backend(quadprog, path, s.start, goal, reach, runs);
        BackendResult p = run_backend(proxqp, path, s.start, goal, reach, runs);
        double dobj = (q.success && p.success) ? std::abs(q.objective - p.objective) : 0.0;
        // Both succeeding on the same path/formulation but landing on visibly different optima
        // would mean at least one backend is not actually solving the problem it was given.
        bool mismatch = q.success && p.success && dobj > 1e-3 * std::max(1.0, std::abs(q.objective));
        if (mismatch) ++mismatches;
        if (q.success != p.success) ++mismatches;
        std::printf("%-19s %6d | %-8s %-11.6f %5.2f+-%.2f | %-8s %-11.6f %5.2f+-%.2f | %-10.2e%s\n",
                    s.name, search.expansion_count(),
                    q.success ? "ok" : "FAIL", q.objective, q.time_ms.mean, q.time_ms.sd,
                    p.success ? "ok" : "FAIL", p.objective, p.time_ms.mean, p.time_ms.sd,
                    dobj, mismatch ? "  <-- MISMATCH" : "");
    }
    std::printf("%s (%d mismatch%s)\n", mismatches == 0 ? "PASS" : "FAIL", mismatches, mismatches == 1 ? "" : "es");
    return mismatches == 0 ? 0 : 1;
}
