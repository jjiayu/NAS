// Replays EVERY scenario captured in phase 0 (tests/golden/*_astar.json)
// through the new AstarSearch + footstep QP and compares against what the
// old code produced. The two dedicated golden tests only cover
// NarrowPassage and ThreePathsNAS; this closes the gap for the other
// scenes the old code solved.
//
// Per scene, using exactly the configuration tests/capture_golden_references.sh
// ran the old code with (start position / goal offset per scene, everything
// else from the old constants.hpp: right stance start, left stance goal,
// EPA heuristic x10, 2cm node similarity, rotation on with 3 yaws of 10deg,
// cycle detection on):
//   - path: same length and (depth, stance_foot, foot_yaw, surface_id) per
//     node. Node centroids are only REPORTED (max deviation), not asserted:
//     re-running the old astar binary today reproduces the path fields of
//     its own stored golden but not its centroids (up to 5cm off on
//     ThreePathsNAS, identical across 3 reruns) - the stored golden came
//     from a slightly different old build, so 1e-6 would be a criterion
//     the old code itself fails;
//   - scenes where the OLD code is itself unstable (Stairs, LongStairs,
//     LongLongStairs, LongStairsComplete, LongStairsExp, ThreePathsScene,
//     Stairs_Up_Down: re-running the old expansion with only its heap order
//     scrambled changes patch polygons on 0.3-1.8% of children, see
//     tests/golden_all/test_expansion_differential.cpp): a differing path is
//     reported, not failed, as long as the new search finds one; where the
//     path is identical the QP is still compared;
//   - QP: where the old QP succeeded, same footstep count and positions
//     within 5cm (different solver, same loose tolerance as
//     footstep_qp's own golden test); where the old QP failed, the new
//     result is only reported, not asserted.
// Scenes where the old search found no path are reported, not asserted.

#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/fixtures/golden_compare.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;
using json = nlohmann::json;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif
#ifndef GOLDEN_DATA_DIR
#error "GOLDEN_DATA_DIR must be defined by CMake"
#endif

namespace {

struct SceneSetup {
    const char* name;
    Point_3 start;
    Vector_3 goal_offset;
};

// Scenes where the old code is itself unstable (see header comment).
bool is_unstable_in_old(const std::string& name) {
    static const std::vector<std::string> names = {"Stairs", "LongStairs", "LongLongStairs", "LongStairsComplete",
                                                   "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down"};
    return std::find(names.begin(), names.end(), name) != names.end();
}

// From tests/capture_golden_references.sh's SCENARIOS table.
const std::vector<SceneSetup> kScenes = {
    {"NarrowPassage", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Stairs", Point_3(0.1, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"TwoFlatSurfaces", Point_3(2.2, 0.7, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongLongStairs", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Flat", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairsComplete", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"LongStairsExp", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"ThreePathsScene", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"Stairs_Up_Down", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 0.0, 0.0)},
    {"ThreePathsNAS", Point_3(0.0, 0.0, 0.0), Vector_3(0.0, 1.0, 0.0)},
};

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    return ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
}

AstarSearchConfig old_constants_config(const SceneSetup& setup, const Point_3& goal) {
    AstarSearchConfig c;
    c.start_position = setup.start;
    c.start_stance_foot = StanceFoot::Right;
    c.start_foot_yaw = 0.0;
    c.goal_location = goal;
    c.goal_stance_foot = StanceFoot::Left;
    c.distance_metric = DistanceMetric::Epa;
    c.heuristic_weight = 10.0;
    c.node_similarity_threshold = 0.02;
    c.expansion_params.rotation_enabled = true;
    c.expansion_params.yaw_discretization_num = 3;
    c.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    c.expansion_params.cycle_detection_enabled = true;
    // NAS_CANONICAL=c|p|q|h|hc: canonical centroid / perimeter dedup keys (see ExpansionParams).
    if (const char* k = std::getenv("NAS_CANONICAL")) {
        std::string keys = k;
        c.expansion_params.canonical_centroid = keys.find('c') != std::string::npos;
        c.expansion_params.canonical_perimeter = keys.find('p') != std::string::npos;
        c.expansion_params.hull_prism_perimeter = keys.find('q') != std::string::npos;
        c.expansion_params.convex_patch = keys.find('h') != std::string::npos;
    }
    return c;
}

struct Outcome {
    std::string name;
    bool old_found = false;
    bool path_ok = true;       // meaningful when old_found
    bool same_length = false;
    bool known_tie_divergence = false;
    double max_centroid_dev = 0.0;
    std::string qp_status;     // human-readable
    bool qp_ok = true;
    bool new_found = false;
};

} // namespace

int main() {
    // NAS_SCRAMBLE=<seed>: fragment the heap in a seed-dependent way so that
    // allocation-order-dependent behaviour (CGAL::convex_hull_3's triangulation)
    // changes between runs; used to check the search is independent of it.
    if (const char* seed = std::getenv("NAS_SCRAMBLE")) {
        std::srand(std::stoul(seed));
        std::vector<void*> blocks;
        for (int i = 0; i < 4000; ++i) blocks.push_back(std::malloc(16 + std::rand() % 1500));
        for (size_t i = blocks.size(); i > 1; --i) std::swap(blocks[i - 1], blocks[std::rand() % i]);
        for (size_t i = 0; i < blocks.size(); i += 2) std::free(blocks[i]);
    }
    ReachabilityModel reachability = make_forward_reachability();
    std::vector<Outcome> outcomes;

    for (const SceneSetup& setup : kScenes) {
        Outcome out;
        out.name = setup.name;
        std::cout << "\n=== " << setup.name << " ===\n";

        std::ifstream gf(std::string(GOLDEN_DATA_DIR) + "/" + setup.name + "_astar.json");
        json golden;
        gf >> golden;
        out.old_found = golden.value("success", false);

        config::Scenario scenario = config::load_scenario(setup.name);
        Point_3 goal = scenario.surfaces.back().centroid + setup.goal_offset;
        AstarSearchConfig cfg = old_constants_config(setup, goal);

        AstarSearch search(scenario.surfaces, reachability, cfg);
        search.search();
        const auto& path = search.result_path();
        out.new_found = !path.empty();
        {   // path signature, to compare runs (see NAS_SCRAMBLE above)
            std::string sig;
            for (const auto* n : path) sig += std::to_string(n->surface_id) + ":" + std::to_string(static_cast<int>(n->stance_foot)) + ":" + std::to_string(n->foot_yaw) + " ";
            std::cout << "SIG " << setup.name << " expansions=" << search.expansion_count() << " " << sig << "\n";
        }

        if (!out.old_found) {
            out.qp_status = "n/a (old search found no path)";
            std::cout << "old code found no path; new code " << (out.new_found ? "FOUND one (differs)" : "also finds none") << "\n";
            outcomes.push_back(out);
            continue;
        }

        out.path_ok = fixtures::check_path_matches_golden(path, std::string(GOLDEN_DATA_DIR) + "/" + setup.name + "_astar.json");

        const auto& gnodes = golden["nodes"];
        out.same_length = path.size() == gnodes.size();
        out.known_tie_divergence = is_unstable_in_old(setup.name);
        if (out.same_length) {
            for (size_t i = 0; i < path.size(); ++i) {
                const auto& gc = gnodes[i]["centroid"];
                double d = std::max({std::abs(CGAL::to_double(path[i]->centroid.x()) - gc[0].get<double>()),
                                     std::abs(CGAL::to_double(path[i]->centroid.y()) - gc[1].get<double>()),
                                     std::abs(CGAL::to_double(path[i]->centroid.z()) - gc[2].get<double>())});
                out.max_centroid_dev = std::max(out.max_centroid_dev, d);
            }
        }
        std::cout << "info: max node-centroid deviation vs stored golden = " << out.max_centroid_dev << " m (reported only)\n";

        if (path.empty()) {
            out.qp_status = "n/a (new search found no path)";
            out.qp_ok = false;
            outcomes.push_back(out);
            continue;
        }

        FootstepQPConfig qp_config;
        qp_config.alpha_weight = 10.0;
        qp_config.rotation_enabled = true;
        QuadprogBackend backend;
        FootstepPlan plan = solve_footstep_qp(path, cfg.start_position, cfg.goal_location, reachability, qp_config, backend);

        const auto& gfp = golden["footstep_plan"];
        bool old_qp_ok = gfp.value("success", false);
        if (!out.path_ok) {
            out.qp_status = "skipped (different path, positions not comparable)";
        } else if (old_qp_ok) {
            const auto& gsteps = gfp["footsteps"];
            double max_dev = 0.0;
            bool same_count = plan.footsteps.size() == gsteps.size();
            for (size_t i = 0; same_count && i < gsteps.size(); ++i) {
                for (int k = 0; k < 3; ++k) {
                    double v = k == 0 ? CGAL::to_double(plan.footsteps[i].x())
                             : k == 1 ? CGAL::to_double(plan.footsteps[i].y())
                                      : CGAL::to_double(plan.footsteps[i].z());
                    max_dev = std::max(max_dev, std::abs(v - gsteps[i][k].get<double>()));
                }
            }
            out.qp_ok = plan.success && same_count && max_dev < 0.05;
            out.qp_status = std::string(plan.success ? "solved" : "FAILED") + ", max dev vs old QP " + std::to_string(max_dev) + " m";
        } else {
            out.qp_status = std::string("old QP failed; new QP ") + (plan.success ? "SOLVES it" : "also fails") + " (reported only)";
        }
        std::cout << (out.qp_ok ? "ok" : "FAIL") << ": QP - " << out.qp_status << "\n";
        outcomes.push_back(out);
    }

    std::cout << "\n================ SUMMARY ================\n";
    bool all_ok = true;
    for (const Outcome& o : outcomes) {
        bool path_acceptable = o.path_ok || (o.known_tie_divergence && o.new_found);
        bool ok = !o.old_found || (path_acceptable && o.qp_ok);
        all_ok = all_ok && ok;
        std::cout << (ok ? "PASS  " : "FAIL  ") << o.name << ": ";
        if (!o.old_found) {
            std::cout << "old found no path; new " << (o.new_found ? "finds one" : "finds none") << "\n";
        } else {
            std::cout << "path " << (o.path_ok ? "identical" : (o.known_tie_divergence ? "differs (old code unstable in this scene)" : "DIFFERS"))
                      << ", centroid dev " << o.max_centroid_dev << " m, QP: " << o.qp_status << "\n";
        }
    }
    return all_ok ? 0 : 1;
}
