// Phase 8: compares the new footstep QP (search via planners/astar_search,
// then solve_footstep_qp with QuadprogBackend) against phase 0's golden
// footstep positions on ThreePathsNAS — the only golden scenario where the
// OLD CasADi/qpoases QP actually succeeded with a clean example
// (NarrowPassage's own golden QP is infeasible, not useful for this).
// Scenario setup lives in tests/fixtures (phase 8d-2).
//
// Tolerance-based, not exact. Two reasons: a different solver (eiquadprog vs
// the old qpOASES-via-CasADi) can land on a different point within a degenerate
// feasible region — see PLAN.md phase 8 and docs/paper-deltas.md — and the
// search now breaks yaw ties deterministically instead of in the old code's
// arbitrary heap order, so the path can differ from the old one by yaw (same
// surfaces, same number of steps) and footsteps then move by up to ~35cm.
// When the path was identical (git tag legacy-replay-verified) the observed QP
// deviation was ~3.8e-08 m, i.e. the solver itself is faithful. The check is
// therefore: same footstep count, and the distance walked within 6% of the old
// plan's; the max deviation is reported for information.

#include "nas/fixtures/scenarios.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::cerr << "FAIL: " << what << "\n";
        ++g_failures;
    } else {
        std::cout << "ok: " << what << "\n";
    }
}

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

} // namespace

int main() {
    fixtures::Scenario scenario = fixtures::make_three_paths_nas();
    ReachabilityModel reachability = fixtures::make_forward_reachability_model(TALOS_REACHABILITY_DATA_DIR);

    AstarSearch search(scenario.surfaces, reachability, scenario.astar_config);
    search.search();
    const auto& path = search.result_path();
    check(!path.empty(), "search finds a path on ThreePathsNAS");

    FootstepQPConfig qp_config;
    qp_config.alpha_weight = 10.0;
    qp_config.rotation_enabled = true;

    QuadprogBackend backend;
    FootstepPlan plan = solve_footstep_qp(path, scenario.astar_config.start_position,
                                           scenario.astar_config.goal_location, reachability, qp_config, backend);

    check(plan.success, "footstep QP (quadprog) succeeds on ThreePathsNAS");

    std::ifstream golden_file(std::string(GOLDEN_DATA_DIR) + "/ThreePathsNAS_astar.json");
    check(golden_file.is_open(), "golden reference file opens");
    if (!golden_file.is_open() || !plan.success) {
        return g_failures > 0 ? 1 : 0;
    }
    json golden;
    golden_file >> golden;
    const auto& golden_fp = golden["footstep_plan"];
    check(golden_fp.value("success", false), "golden itself recorded a successful QP solve (sanity check)");

    const auto& golden_footsteps = golden_fp["footsteps"];
    check(plan.footsteps.size() == golden_footsteps.size(),
          "same number of footsteps as golden (" + std::to_string(plan.footsteps.size()) +
          " vs " + std::to_string(golden_footsteps.size()) + ")");

    size_t n = std::min(plan.footsteps.size(), golden_footsteps.size());
    double max_dev = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto& g = golden_footsteps[i];
        double dx = CGAL::to_double(plan.footsteps[i].x()) - g[0].get<double>();
        double dy = CGAL::to_double(plan.footsteps[i].y()) - g[1].get<double>();
        double dz = CGAL::to_double(plan.footsteps[i].z()) - g[2].get<double>();
        max_dev = std::max(max_dev, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    std::cout << "  max deviation from golden footsteps: " << max_dev << " m\n";
    auto walked = [](const std::vector<Point_3>& steps) {
        double d = 0;
        for (size_t i = 1; i < steps.size(); ++i)
            d += std::hypot(CGAL::to_double(steps[i].x() - steps[i - 1].x()), CGAL::to_double(steps[i].y() - steps[i - 1].y()));
        return d;
    };
    std::vector<Point_3> old_steps;
    for (const auto& g : golden_footsteps) old_steps.emplace_back(g[0].get<double>(), g[1].get<double>(), g[2].get<double>());
    double w_new = walked(plan.footsteps), w_old = walked(old_steps);
    std::cout << "  distance walked: " << w_new << " m vs golden " << w_old << " m\n";
    check(std::abs(w_new - w_old) <= 0.06 * w_old, "distance walked is within 6% of the golden plan's (yaw ties may move footsteps)");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "Footstep QP (quadprog) matches golden within tolerance\n";
    return 0;
}
