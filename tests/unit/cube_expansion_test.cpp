// Directed tests for expand_cube_placement (spec §3.1, core/expansion.*, see
// docs/cube-implementation-plan.md Etape 3): the cube-placement action built on top of
// the tagged geometry primitives from Etape 2.

#include "nas/config/cube_model.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"

#include <cmath>
#include <iostream>
#include <string>

using namespace nas;

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

} // namespace

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

int run_cube_expansion() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    });
    config::CubeConfig cube_cfg;

    config::Scenario sc = config::load_scenario("Flat");
    check(sc.surfaces.size() == 1, "Flat scenario is a single surface, as this test assumes");
    const Surface& flat = sc.surfaces[0];

    NodePool pool;
    Node* parent = pool.create();
    parent->patch_vertices = {Point_3(0, 0, 0)};
    // patch_polygon_2d left default (empty): expand_cube_placement never reads it, only
    // patch_vertices (the 3D x-values) and the two transforms below.
    parent->transformation_to_2d = flat.transform_to_surface;
    parent->transformation_to_3d = flat.transform_to_3d;
    parent->stance_foot = StanceFoot::Right;
    parent->surface_id = flat.surface_id;
    parent->centroid = Point_3(0, 0, 0);
    parent->foot_yaw = 0.0;
    parent->depth = 0;
    parent->cube_state = CubeState::InHand;

    ExpansionParams params;
    params.rotation_enabled = false;

    // --- Negative control: not InHand => no candidate children at all ---
    {
        Node* not_carrying = pool.create();
        *not_carrying = *parent;
        not_carrying->cube_state = CubeState::None;
        auto none_children = expand_cube_placement(not_carrying, sc.surfaces, reach, cube_cfg.half_extent, params, pool);
        check(none_children.empty(), "expand_cube_placement: no children when cube_state != InHand");
    }

    // --- The real placement action ---
    auto children = expand_cube_placement(parent, sc.surfaces, reach, cube_cfg.half_extent, params, pool);
    check(!children.empty(), "expand_cube_placement: at least one child on a large flat surface with the cube in hand");

    for (Node* child : children) {
        check(child->cube_state == CubeState::PlacedActive, "child cube_state is PlacedActive");
        check(child->cube.has_value(), "child carries a CubePlacement");
        if (!child->cube.has_value()) continue;

        // Foot side is untouched in identity (stance foot, yaw, surface, depth advances by 1).
        check(child->stance_foot == parent->stance_foot, "child keeps the parent's stance foot (no foot moved)");
        check(child->surface_id == parent->surface_id, "child keeps the parent's own surface_id (foot didn't move)");
        check(std::abs(child->foot_yaw - parent->foot_yaw) < 1e-12, "child keeps the parent's foot_yaw");
        check(child->depth == parent->depth + 1, "child is one depth level below the parent");

        // The joint-state invariant this whole extension exists to preserve (spec §3.1,
        // docs/cube-implementation-plan.md §2): patch_vertices (x) and cube->vertices_3d
        // (c) are index-aligned, and for every i, c_i - x_i lands inside K_cube (here,
        // for a flat scene with foot_yaw=0, K_cube's own documented box bounds apply
        // directly in world coordinates: X in [-0.15,0.15], Y in [0.15,0.40], Z in
        // [-0.05,0.10] -- see talosReachability/.../Cube_constraints_in_RF.obj).
        check(child->patch_vertices.size() == child->cube->vertices_3d.size(),
              "patch_vertices (x) and cube->vertices_3d (c) are index-aligned (same size)");

        bool coupling_holds = true;
        for (size_t i = 0; i < child->patch_vertices.size(); ++i) {
            double dx = CGAL::to_double(child->cube->vertices_3d[i].x() - child->patch_vertices[i].x());
            double dy = CGAL::to_double(child->cube->vertices_3d[i].y() - child->patch_vertices[i].y());
            double dz = CGAL::to_double(child->cube->vertices_3d[i].z() - child->patch_vertices[i].z());
            if (dx < -0.15 - 1e-6 || dx > 0.15 + 1e-6 || dy < 0.15 - 1e-6 || dy > 0.40 + 1e-6 || dz < -0.05 - 1e-6 || dz > 0.10 + 1e-6)
                coupling_holds = false;
        }
        check(coupling_holds, "every c_i - x_i lands inside K_cube's documented box (the §3.1 coupling invariant)");

        check(std::abs(child->cube->placement_yaw - parent->foot_yaw) < 1e-12,
              "cube's placement_yaw is frozen to the foot_yaw at placement time");
    }

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All cube-expansion tests passed\n";
    return 0;
}
