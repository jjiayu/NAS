// Single entry point for every nas test — replaces the ~20 standalone `int main()` binaries this
// project used to build one per test file (see docs/architecture.md). Each test file keeps its own
// body verbatim (including its own anonymous-namespace helpers, e.g. check()/g_failures — those
// have internal linkage per translation unit, so they don't collide across files linked together
// here); only its `int main()` was mechanically renamed to a unique `int run_<name>()`, declared
// below and dispatched from this file.
//
// `--suite=unit` runs only the fast (<1s) directed tests; `--suite=golden` runs the slower
// regression suite against golden/scenario data (some take tens of seconds); no argument runs both
// — see tests/CMakeLists.txt for how ctest registers these as two separate entries from this one
// binary.
//
// Known trade-off: tests/unit/config_*_test.cpp use assert() internally (ported as-is from when
// they were their own binaries) rather than the check()/g_failures idiom the others use — an
// assert() failure there aborts this whole process instead of just failing its own suite, so a
// crash here can hide whether later-registered suites would have passed. Not a live problem (every
// suite passes as of this writing), flagged for whoever next touches those three files.

#include <cstdio>
#include <cstring>
#include <string>

int run_core_geometry();
int run_core_node();
int run_core_surface();
int run_core_reachability();
int run_core_expansion();
int run_footstep_qp_backend();
int run_config_planner();
int run_config_scenario();
int run_config_stl();
int run_config_cube();
int run_cube_geometry();
int run_cube_expansion();

int run_astar_search_golden();
int run_astar_search_golden_threepaths();
int run_grid_astar_search();
int run_footstep_qp_golden();
int run_golden_all_scenes();
int run_expansion_oracle();
int run_euclidean_metric();
int run_inclined_scenes();
int run_goal_surface();
int run_cycle_detection();
int run_merge_consistency();
int run_edge_costs();
int run_goal_yaw();
int run_foot_goals();
int run_dual_target_all_scenes();

namespace {

struct Suite {
    const char* name;
    const char* group; // "unit" or "golden"
    int (*fn)();
};

const Suite kSuites[] = {
    {"core_geometry", "unit", run_core_geometry},
    {"core_node", "unit", run_core_node},
    {"core_surface", "unit", run_core_surface},
    {"core_reachability", "unit", run_core_reachability},
    {"core_expansion", "unit", run_core_expansion},
    {"footstep_qp_backend", "unit", run_footstep_qp_backend},
    {"config_planner", "unit", run_config_planner},
    {"config_scenario", "unit", run_config_scenario},
    {"config_stl", "unit", run_config_stl},
    {"config_cube", "unit", run_config_cube},
    {"cube_geometry", "unit", run_cube_geometry},
    {"cube_expansion", "unit", run_cube_expansion},

    {"astar_search_golden", "golden", run_astar_search_golden},
    {"astar_search_golden_threepaths", "golden", run_astar_search_golden_threepaths},
    {"grid_astar_search", "golden", run_grid_astar_search},
    {"footstep_qp_golden", "golden", run_footstep_qp_golden},
    {"golden_all_scenes", "golden", run_golden_all_scenes},
    {"expansion_oracle", "golden", run_expansion_oracle},
    {"euclidean_metric", "golden", run_euclidean_metric},
    {"inclined_scenes", "golden", run_inclined_scenes},
    {"goal_surface", "golden", run_goal_surface},
    {"cycle_detection", "golden", run_cycle_detection},
    {"merge_consistency", "golden", run_merge_consistency},
    {"edge_costs", "golden", run_edge_costs},
    {"goal_yaw", "golden", run_goal_yaw},
    {"foot_goals", "golden", run_foot_goals},
    {"dual_target_all_scenes", "golden", run_dual_target_all_scenes},
};

} // namespace

int main(int argc, char** argv) {
    std::string filter; // empty = run everything
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--suite=", 0) == 0) filter = arg.substr(8);
    }

    int ran = 0, failed = 0;
    for (const Suite& s : kSuites) {
        if (!filter.empty() && filter != s.group) continue;
        std::printf("\n=== %s (%s) ===\n", s.name, s.group);
        ++ran;
        if (s.fn() != 0) {
            std::fprintf(stderr, "FAIL: %s\n", s.name);
            ++failed;
        }
    }

    std::printf("\n%d/%d suites passed\n", ran - failed, ran);
    if (ran == 0) {
        std::fprintf(stderr, "no suite matched --suite=%s\n", filter.c_str());
        return 1;
    }
    return failed == 0 ? 0 : 1;
}
