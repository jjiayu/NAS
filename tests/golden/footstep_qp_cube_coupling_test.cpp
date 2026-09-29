// FootstepQPConfig::cube_placements: couples a cube's base center into the SAME footstep QP as
// the ordinary footsteps, instead of leaving g1motion's cube_plan.py::place_box to fit one to the
// QP's already-fixed footsteps afterward (see the struct's own doc comment in footstep_qp.hpp).
// Reuses cube_pickup_and_placement_test.cpp's exact scenario (StairsGap, empty-handed start,
// scene_cubes) - the search there already proves a full pickup/place/cross path exists; this test
// only adds the QP layer on top and checks the coupling constraint actually holds by construction.

#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
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

// A cube-carrying path has zero-displacement pseudo-nodes (place, pickup: same stance_foot as
// their own parent - the only way two consecutive nodes share one, ordinary steps always
// alternate) that solve_footstep_qp never sees - same filtering nas_tools/cube_plan.cpp does.
std::vector<Node*> filter_pseudo_nodes(const std::vector<Node*>& path) {
    std::vector<Node*> out;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0 && path[i]->stance_foot == path[i - 1]->stance_foot) continue;
        out.push_back(path[i]);
    }
    return out;
}

} // namespace

int run_footstep_qp_cube_coupling() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED_clamp_z18.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED_clamp_z18.obj", "LF", "RF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("StairsGap");

    AstarSearchConfig cfg;
    cfg.start_position = Point_3(0.1, 0.0, 0.0);
    cfg.start_stance_foot = StanceFoot::Right;
    cfg.expansion_params.rotation_enabled = true;
    cfg.expansion_params.yaw_discretization_num = 3;
    cfg.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    cfg.cube_half_extent = 0.15;
    cfg.cube_height = 0.15;
    cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = AstarSearchConfig::FootGoal{sc.surfaces.back().centroid, std::nullopt};
    cfg.max_expansions = 5000;

    AstarSearchConfig::SceneCube cube;
    AstarSearchConfig::FootGoal g;
    g.region = std::vector<Point_3>{Point_3(0.15, -0.3, 0.0), Point_3(0.45, -0.3, 0.0), Point_3(0.45, 0.3, 0.0),
                                     Point_3(0.15, 0.3, 0.0)};
    cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = g;
    cfg.scene_cubes = {cube};

    AstarSearch search(sc.surfaces, reach, cfg);
    search.search();
    const auto& path = search.result_path();
    check(!path.empty(), "a full pickup-then-place-then-cross path is found (same as cube_pickup_and_placement_test)");
    if (path.empty()) {
        if (g_failures > 0) { std::fprintf(stderr, "%d test(s) FAILED\n", g_failures); return 1; }
        return 0;
    }

    // locate: the "place" node (PlacedActive, parent InHand), its parent (the support footstep),
    // and the first "onto the cube" node after it (surface_id == kOnCubeSurfaceId) - the exact same
    // logic g1motion's cube_plan.py::choose_positions uses.
    int idx_place = -1;
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i]->cube_state == CubeState::PlacedActive && path[i - 1]->cube_state == CubeState::InHand) {
            idx_place = static_cast<int>(i);
            break;
        }
    }
    check(idx_place > 0, "found the placement node");
    int idx_onto = -1;
    if (idx_place > 0) {
        for (size_t i = static_cast<size_t>(idx_place) + 1; i < path.size(); ++i) {
            if (path[i]->surface_id == kOnCubeSurfaceId) { idx_onto = static_cast<int>(i); break; }
        }
    }
    check(idx_onto > idx_place, "found the onto-the-cube node after it");
    if (idx_place <= 0 || idx_onto <= idx_place) {
        if (g_failures > 0) { std::fprintf(stderr, "%d test(s) FAILED\n", g_failures); return 1; }
        return 0;
    }
    check(path[static_cast<size_t>(idx_place)]->cube.has_value(), "the placement node carries a CubePlacement (its own cube_vertices)");

    std::vector<Node*> qp_path = filter_pseudo_nodes(path);
    auto qp_index_of = [&](Node* n) -> int {
        for (size_t i = 0; i < qp_path.size(); ++i) if (qp_path[i] == n) return static_cast<int>(i);
        return -1;
    };
    // the support footstep: place's parent, UNLESS that parent is itself a pseudo-node (e.g. the
    // pickup that immediately preceded this placement, with no ordinary step in between) - walk
    // back through any such chain to the nearest ancestor that actually survived the filter.
    int support_raw = static_cast<int>(idx_place) - 1;
    while (support_raw > 0 && qp_index_of(path[static_cast<size_t>(support_raw)]) < 0) --support_raw;
    int support_qi = support_raw >= 0 ? qp_index_of(path[static_cast<size_t>(support_raw)]) : -1;
    int onto_qi = qp_index_of(path[static_cast<size_t>(idx_onto)]);
    check(support_qi >= 0, "the support footstep maps to a qp_path index");
    check(onto_qi >= 0, "the onto footstep maps to a qp_path index");
    if (support_qi < 0 || onto_qi < 0) {
        if (g_failures > 0) { std::fprintf(stderr, "%d test(s) FAILED\n", g_failures); return 1; }
        return 0;
    }

    FootstepQPConfig qp_config;
    qp_config.alpha_weight = 10.0;
    qp_config.rotation_enabled = true;
    qp_config.cube_half_extent = cfg.cube_half_extent;
    FootstepQPConfig::CubePlacement cp;
    cp.support_index = static_cast<size_t>(support_qi);
    cp.onto_index = static_cast<size_t>(onto_qi);
    cp.placement_patch = path[static_cast<size_t>(idx_place)]->cube->vertices_3d;
    qp_config.cube_placements = {cp};

    QuadprogBackend backend;
    FootstepPlan plan = solve_footstep_qp(qp_path, cfg.start_position, sc.surfaces.back().centroid, reach, qp_config, backend);
    check(plan.success, "the coupled QP (footsteps + cube center, same problem) succeeds");
    check(plan.cube_centers.size() == 1, "exactly one cube center comes back, matching cube_placements");

    if (plan.success && plan.cube_centers.size() == 1) {
        // the coupling constraint itself, checked independently of the QP's own residual: the onto
        // footstep must land within the box's own half_extent square, in the support footstep's frame.
        Node* support_node = qp_path[static_cast<size_t>(support_qi)];
        double yaw = support_node->foot_yaw;
        double cy = std::cos(yaw), sy = std::sin(yaw);
        Point_3 c = plan.cube_centers[0];
        Point_3 p = plan.footsteps[static_cast<size_t>(onto_qi)];
        double dx = CGAL::to_double(p.x() - c.x()), dy = CGAL::to_double(p.y() - c.y());
        double local_x = cy * dx + sy * dy;
        double local_y = -sy * dx + cy * dy;
        double tol = 1e-4;
        check(std::abs(local_x) <= cfg.cube_half_extent + tol,
              "onto footstep is within the box's own half_extent in the support foot's local x");
        check(std::abs(local_y) <= cfg.cube_half_extent + tol,
              "onto footstep is within the box's own half_extent in the support foot's local y");
        std::printf("cube center: (%.3f, %.3f, %.3f), onto local offset: (%.4f, %.4f), half_extent %.3f\n",
                    CGAL::to_double(c.x()), CGAL::to_double(c.y()), CGAL::to_double(c.z()), local_x, local_y,
                    cfg.cube_half_extent);
    }

    // Regression guard: an EMPTY cube_placements (every existing caller) must behave exactly as
    // before this feature existed - no cube columns, no coupling rows, cube_centers stays empty.
    FootstepQPConfig plain_config;
    plain_config.alpha_weight = 10.0;
    plain_config.rotation_enabled = true;
    FootstepPlan plain_plan = solve_footstep_qp(qp_path, cfg.start_position, sc.surfaces.back().centroid, reach, plain_config, backend);
    check(plain_plan.success, "the SAME path/goal, without cube_placements, still solves (today's behavior)");
    check(plain_plan.cube_centers.empty(), "and returns no cube centers when none were asked for");

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All footstep_qp_cube_coupling tests passed\n");
    return 0;
}
