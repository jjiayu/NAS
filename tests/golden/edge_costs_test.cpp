// The optional edge costs on rotation (AstarSearchConfig::yaw_change_weight: 1 + w*|yaw change|, and heading_weight:
// + w*|yaw - direction to the goal|). Runs the 15 scenes for four settings: none (the paper's cost, the default), a yaw-change
// cost 0.1, a heading cost 0.1 and a heading cost 1.0. For each: expansions / path nodes / total rotation (sum of |yaw
// change|, degrees) / mean heading error (mean over the path of |yaw - direction to the goal|, degrees).
// Asserted: weights 0 reproduce the default search exactly; wherever the default finds a path every setting does, its footstep
// QP succeeds and the search is deterministic; on Flat (a straight walk) the heading cost 1.0 does not misalign more than the
// default.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {
double heading_error_deg(const std::vector<Node*>& path, const Point_3& goal) {
    double t = 0; int n = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        double gx = CGAL::to_double(goal.x() - path[i - 1]->centroid.x()), gy = CGAL::to_double(goal.y() - path[i - 1]->centroid.y());
        if (std::hypot(gx, gy) < 1e-6) continue;
        double d = std::fmod(std::abs(path[i]->foot_yaw - std::atan2(gy, gx)), 2.0 * M_PI);
        t += (d > M_PI ? 2.0 * M_PI - d : d) * 180.0 / M_PI; ++n;
    }
    return n ? t / n : 0.0;
}
double total_rotation_deg(const std::vector<Node*>& path) {
    double t = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        double d = std::fmod(std::abs(path[i]->foot_yaw - path[i - 1]->foot_yaw), 2.0 * M_PI);
        t += (d > M_PI ? 2.0 * M_PI - d : d) * 180.0 / M_PI;
    }
    return t;
}
struct Scene { const char* name; Point_3 start; Vector_3 goal_offset; };
} // namespace

int run_edge_costs() {
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
    struct Setting { const char* label; double yaw_change, heading; };
    const std::vector<Setting> settings = {{"default", 0.0, 0.0}, {"yaw change 0.1", 0.1, 0.0}, {"heading 0.1", 0.0, 0.1}, {"heading 1.0", 0.0, 1.0}};
    int failures = 0;
    std::printf("%-19s", "scene");
    for (const auto& st : settings) std::printf(" | %-34s", st.label);
    std::printf("\n%-19s", "");
    for (size_t k = 0; k < settings.size(); ++k) std::printf(" | %-34s", "exp / nodes / rot deg / heading err deg");
    std::printf("\n");
    for (const Scene& s : scenes) {
        config::Scenario sc = config::load_scenario(s.name);
        double heading_err[4] = {0, 0, 0, 0};
        bool found0 = false;
        char cells[4][64] = {"", "", "", ""};
        for (size_t k = 0; k < settings.size(); ++k) {
            AstarSearchConfig cfg;
            cfg.start_position = s.start; cfg.start_stance_foot = StanceFoot::Right; cfg.goal_stance_foot = StanceFoot::Left;
            cfg.goal_location = sc.surfaces.back().centroid + s.goal_offset;
            cfg.expansion_params.rotation_enabled = true;
            cfg.yaw_change_weight = settings[k].yaw_change;
            cfg.heading_weight = settings[k].heading;
            AstarSearch a(sc.surfaces, reach, cfg), b(sc.surfaces, reach, cfg);
            a.search(); b.search();
            bool found = !a.result_path().empty();
            if (k == 0) {
                found0 = found;
                AstarSearchConfig def;  // everything default
                def.start_position = s.start; def.start_stance_foot = StanceFoot::Right; def.goal_stance_foot = StanceFoot::Left;
                def.goal_location = cfg.goal_location; def.expansion_params.rotation_enabled = true;
                AstarSearch d(sc.surfaces, reach, def); d.search();
                if (d.expansion_count() != a.expansion_count() || d.result_path().size() != a.result_path().size()) { std::printf("FAIL: weights 0 differ from the default\n"); ++failures; }
            }
            bool det = a.expansion_count() == b.expansion_count() && a.result_path().size() == b.result_path().size();
            bool qp_ok = true;
            if (found) {
                FootstepQPConfig qc; qc.alpha_weight = 10.0; qc.rotation_enabled = true;
                QuadprogBackend backend;
                qp_ok = solve_footstep_qp(a.result_path(), s.start, cfg.goal_location, reach, qc, backend).success;
            }
            heading_err[k] = found ? heading_error_deg(a.result_path(), cfg.goal_location) : 0.0;
            std::snprintf(cells[k], sizeof cells[k], "%d / %zu / %.0f / %.0f%s", a.expansion_count(), a.result_path().size(),
                          found ? total_rotation_deg(a.result_path()) : 0.0, heading_err[k], (found && qp_ok && det) || (!found && !found0) ? "" : "  <-- PROBLEM");
            if ((found0 && !found) || (found && !qp_ok) || !det) ++failures;
        }
        if (std::string(s.name) == "Flat" && heading_err[3] > heading_err[0] + 1e-9) { std::printf("FAIL: Flat is less aligned with heading 1.0\n"); ++failures; }
        std::printf("%-19s", s.name);
        for (int k = 0; k < 4; ++k) std::printf(" | %-34s", cells[k]);
        std::printf("\n");
    }
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
