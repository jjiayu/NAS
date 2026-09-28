// Perf non-regression, systematized (previously only ever measured by hand with tools/nas_bench_perf
// and compared visually against a prior session's printed numbers). Every run of this suite prints
// timing for the 11 standard scenarios + StairsGap+scene_cubes AND checks it against a committed
// reference (tests/golden_data/perf_baseline.json): expansions must match exactly (a real behaviour
// change, same bar as the other golden tests), search time must stay within a generous factor of the
// baseline (not a tight threshold -- this machine has constrained RAM and shares load with other
// processes, so +/-20-30% is noise; only a genuine multi-x regression should fail this).
//
// Scenario/config setup is a deliberate 3rd copy of tools/nas_bench_perf.cpp's own kScenes/
// base_config (itself a deliberate copy of tests/common/src/scenarios.cpp's smaller 2-scenario set):
// nas_bench_perf.cpp must stay buildable standalone against an older checked-out commit (its own doc
// comment), so it can't be refactored to share code with a test file that only exists on this branch.
// The 2-cibles ("closing stance") timing nas_bench_perf.cpp also measures is NOT repeated here, to
// keep this suite's own runtime reasonable -- it stays a manual/exploratory measurement.
//
// The reference is updated deliberately, same discipline as expected_*_expansions elsewhere in this
// codebase: only when a real perf change is measured and accepted (see docs/patchindex-scalability-
// note.md for the process followed for the CELL=5cm change this suite's baseline reflects).
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

using namespace nas;
using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what.c_str());
    }
}

constexpr int kRuns = 5;
// Generous on purpose (see file header): only a real multi-x regression should trip this, not
// machine-load noise.
constexpr double kSlowdownFactor = 2.0;

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
    c.start_position = s.start; c.start_stance_foot = StanceFoot::Right;
    c.foot_goals[static_cast<size_t>(StanceFoot::Left)] = AstarSearchConfig::FootGoal{goal, std::nullopt};
    c.heuristic_weight = 10.0; c.node_similarity_threshold = 0.02;
    c.expansion_params.rotation_enabled = true; c.expansion_params.yaw_discretization_num = 3;
    c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI; c.expansion_params.cycle_detection_enabled = true;
    return c;
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

// Times `runs` fresh searches of `make_search()`, returns {mean_ms, stddev_ms, expansions of the
// last run (identical across runs -- determinism is checked elsewhere, golden_all_scenes_test.cpp)}.
struct Timing { double mean_ms; double stddev_ms; int expansions; };
Timing time_search(const std::function<AstarSearch()>& make_search) {
    std::vector<double> t;
    int exp = 0;
    for (int r = 0; r < kRuns; ++r) {
        AstarSearch search = make_search();
        auto t0 = Clock::now();
        search.search();
        t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
        exp = search.expansion_count();
    }
    double m = mean_of(t);
    return {m, stddev_of(t, m), exp};
}

// Prints the row and checks expansions (exact) + timing (within kSlowdownFactor of baseline),
// against `baseline["scenarios"][name]` = {"expansions": int, "search_ms": double}. Missing entry:
// reported and failed loudly (a scenario with nothing to compare against defeats the point of this
// suite), not silently skipped.
void check_against_baseline(const json& baseline, const char* name, const Timing& timing) {
    if (!baseline.contains(name)) {
        std::printf("%-19s: %5d exp, %8.2f +/- %6.2f ms  (PAS DE REFERENCE dans perf_baseline.json)\n", name,
                    timing.expansions, timing.mean_ms, timing.stddev_ms);
        check(false, std::string(name) + ": entree de reference manquante dans perf_baseline.json");
        return;
    }
    int expected_exp = baseline[name]["expansions"].get<int>();
    double expected_ms = baseline[name]["search_ms"].get<double>();
    double ratio = expected_ms > 0.0 ? timing.mean_ms / expected_ms : 0.0;
    std::printf("%-19s: %5d exp (ref %5d) | %8.2f +/- %6.2f ms (ref %8.2f ms, x%.2f)\n", name, timing.expansions,
                expected_exp, timing.mean_ms, timing.stddev_ms, expected_ms, ratio);
    check(timing.expansions == expected_exp, std::string(name) + ": expansions inchangees vs reference");
    char factor_msg[96];
    std::snprintf(factor_msg, sizeof factor_msg, "%s: temps dans le facteur x%.1f de la reference", name, kSlowdownFactor);
    check(timing.mean_ms <= expected_ms * kSlowdownFactor, factor_msg);
}

} // namespace

int run_perf_regression() {
    std::ifstream bf(std::string(GOLDEN_DATA_DIR) + "/perf_baseline.json");
    if (!bf.is_open()) {
        std::fprintf(stderr, "perf_regression: tests/golden_data/perf_baseline.json introuvable\n");
        return 1;
    }
    json full_baseline;
    bf >> full_baseline;
    const json& baseline = full_baseline.at("scenarios");

    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });

    std::printf("%-19s   %-25s   %-30s\n", "scene", "expansions", "search ms");
    for (const auto& s : kScenes) {
        config::Scenario sc = config::load_scenario(s.name);
        Point_3 goal = sc.surfaces.back().centroid + s.goal_offset;
        AstarSearchConfig c = base_config(s, goal);
        Timing timing = time_search([&]() { return AstarSearch(sc.surfaces, reach, c); });
        check_against_baseline(baseline, s.name, timing);
    }

    // StairsGap+scene_cubes: the scenario that originally exposed PatchIndex's cost-grows-with-
    // bucket-population issue (docs/patchindex-scalability-note.md) -- the one scenario in this
    // benchmark where patch_index_cell_size actually matters, kept here permanently.
    {
        ReachabilityModel cube_reach = ReachabilityModel::load({
            {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED_clamp_z18.obj", "RF", "LF", ReachabilityDirection::Forward},
            {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED_clamp_z18.obj", "LF", "RF", ReachabilityDirection::Forward},
            {dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
            {dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
        });
        config::Scenario sc = config::load_scenario("StairsGap");

        AstarSearchConfig c;
        c.start_position = Point_3(0.1, 0.0, 0.0);
        c.start_stance_foot = StanceFoot::Right;
        c.expansion_params.rotation_enabled = true;
        c.expansion_params.yaw_discretization_num = 3;
        c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
        c.expansion_params.cycle_detection_enabled = true;
        c.node_similarity_threshold = 0.02;
        c.cube_half_extent = 0.075;
        c.cube_height = 0.15;
        c.foot_goals[static_cast<size_t>(StanceFoot::Left)] = AstarSearchConfig::FootGoal{sc.surfaces.back().centroid, std::nullopt};
        c.max_expansions = 5000;

        AstarSearchConfig::FootGoal g;
        g.region = std::vector<Point_3>{Point_3(0.15, -0.3, 0.0), Point_3(0.45, -0.3, 0.0), Point_3(0.45, 0.3, 0.0),
                                         Point_3(0.15, 0.3, 0.0)};
        AstarSearchConfig::SceneCube cube;
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
        c.scene_cubes = {cube};

        Timing timing = time_search([&]() { return AstarSearch(sc.surfaces, cube_reach, c); });
        check_against_baseline(baseline, "StairsGap+cube", timing);
    }

    if (g_failures > 0) {
        std::fprintf(stderr, "%d perf check(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All perf_regression checks passed\n");
    return 0;
}
