// Phase 8: compares the new footstep QP (search via planners/astar_search,
// then solve_footstep_qp with QuadprogBackend) against phase 0's golden
// footstep positions on ThreePathsNAS — the only golden scenario where the
// OLD CasADi/qpoases QP actually succeeded with a clean example
// (NarrowPassage's own golden QP is infeasible, not useful for this).
//
// Tolerance-based, not exact: a different solver (eiquadprog vs the old
// qpOASES-via-CasADi) can land on a different point within a degenerate
// feasible region (e.g. ties in the stride-length objective) — see
// PLAN.md phase 8 and docs/paper-deltas.md.

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
    for (size_t i = 0; i < raw.size(); ++i) {
        surfaces.emplace_back(raw[i], static_cast<int>(i), 0.22, 0.22);
    }
    return surfaces;
}

ReachabilityModel make_forward_model() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    std::vector<ReachabilityEntry> entries = {
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

} // namespace

int main() {
    std::vector<Surface> surfaces = make_three_paths_nas_surfaces();
    ReachabilityModel reachability = make_forward_model();

    AstarSearchConfig config;
    config.start_position = Point_3(0.0, 0.0, 0.0);
    config.start_stance_foot = StanceFoot::Right;
    config.goal_stance_foot = StanceFoot::Left;
    Point_3 c = surfaces.back().centroid;
    config.goal_location = Point_3(CGAL::to_double(c.x()), CGAL::to_double(c.y()) + 1.0, CGAL::to_double(c.z()));
    config.distance_metric = DistanceMetric::Epa;
    config.heuristic_weight = 10.0;
    config.node_similarity_threshold = 0.02;
    config.expansion_params.rotation_enabled = true;
    config.expansion_params.yaw_discretization_num = 3;
    config.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    config.expansion_params.cycle_detection_enabled = true;

    AstarSearch search(surfaces, reachability, config);
    search.search();
    const auto& path = search.result_path();
    check(!path.empty(), "search finds a path on ThreePathsNAS");

    FootstepQPConfig qp_config;
    qp_config.alpha_weight = 10.0;
    qp_config.rotation_enabled = true;

    QuadprogBackend backend;
    FootstepPlan plan = solve_footstep_qp(path, config.start_position, config.goal_location, reachability, qp_config, backend);

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
        double dev = std::sqrt(dx * dx + dy * dy + dz * dz);
        max_dev = std::max(max_dev, dev);
    }
    std::cout << "  max deviation from golden footsteps: " << max_dev << " m\n";
    // Loose tolerance on purpose: different solver, different active-set
    // path through a QP whose objective has real degrees of freedom (the
    // stride objective doesn't uniquely pin every intermediate point when
    // reachability/surface constraints leave slack) — see PLAN.md phase 8.
    check(max_dev < 0.05, "footstep positions are within 5cm of golden (loose tolerance, different solver)");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "Footstep QP (quadprog) matches golden within tolerance\n";
    return 0;
}
