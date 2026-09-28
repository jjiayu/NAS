// Directed tests for expand_cube_placement (spec §3.1, core/expansion.*, see
// docs/cube-implementation-plan.md Etape 3): the cube-placement action built on top of
// the tagged geometry primitives from Etape 2.

#include "nas/config/cube_model.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"

#include <algorithm>
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
    // expand_node/expand_cube_placement take eroded footprints (AstarSearch erodes its raw scene itself)
    const std::vector<Surface> foot_surfaces = erode_surfaces(sc.surfaces, kDefaultInnerMargin);
    // Where the cube's center may go: the raw footprint eroded by the cube's half-diagonal (AstarSearch builds this).
    std::vector<Surface> cube_support;
    for (const Surface& s : sc.surfaces) {
        if (auto e = s.inner_margin(cube_cfg.half_extent * std::sqrt(2.0))) cube_support.push_back(std::move(*e));
    }
    // Usable half side of the cube top: same inner margin as any surface.
    const double top_half_extent = cube_cfg.half_extent - kDefaultInnerMargin;

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

    // Flat is an axis-aligned rectangle at z=0: its world bounding box is the raw footprint.
    double sx0 = 1e9, sx1 = -1e9, sy0 = 1e9, sy1 = -1e9;
    for (const Point_3& v : flat.vertices_3d) {
        sx0 = std::min(sx0, CGAL::to_double(v.x())); sx1 = std::max(sx1, CGAL::to_double(v.x()));
        sy0 = std::min(sy0, CGAL::to_double(v.y())); sy1 = std::max(sy1, CGAL::to_double(v.y()));
    }

    // --- Negative control: not InHand => no candidate children at all ---
    {
        Node* not_carrying = pool.create();
        *not_carrying = *parent;
        not_carrying->cube_state = CubeState::None;
        auto none_children = expand_cube_placement(not_carrying, cube_support, reach, params, pool);
        check(none_children.empty(), "expand_cube_placement: no children when cube_state != InHand");
    }

    // --- The real placement action ---
    auto children = expand_cube_placement(parent, cube_support, reach, params, pool);
    check(!children.empty(), "expand_cube_placement: at least one child on a large flat surface with the cube in hand");

    // The cube must rest ENTIRELY on the raw surface wherever its center is put: every center in the
    // placement region is at least half_extent (the square's own half side, even a yaw-0 cube) from
    // the raw footprint's border, indeed at least the half-diagonal (any yaw).
    {
        const double need = cube_cfg.half_extent * std::sqrt(2.0) - 1e-6;
        bool whole_cube_on_surface = !children.empty();
        for (Node* child : children) {
            if (!child->cube.has_value()) continue;
            for (const Point_3& c : child->cube->vertices_3d) {
                double cx = CGAL::to_double(c.x()), cy = CGAL::to_double(c.y());
                if (cx - sx0 < need || sx1 - cx < need || cy - sy0 < need || sy1 - cy < need) whole_cube_on_surface = false;
            }
        }
        check(whole_cube_on_surface, "expand_cube_placement: the whole cube rests on the raw surface (centers >= half-diagonal from its border)");

        // Foot-eroded footprints (the old behaviour) would let the center get within the foot margin
        // of the border; the cube-support region must be strictly tighter than them.
        check(cube_support.size() == 1 && cube_support[0].vertices_2d.size() >= 3 &&
                  foot_surfaces.size() == 1,
              "cube support region exists next to the foot-eroded footprint");
    }

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
        // (c) are index-aligned, and for every i, c_i - x_i lands inside K_cube (here, for
        // a flat scene with foot_yaw=0 and stance_foot=Right, Cube_constraints_in_RF.obj's
        // own documented box bounds apply directly in world coordinates: X in [0.0,0.15]
        // forward, Y in [0.245,0.395] lateral (offset for stance clearance, see that file's
        // header), Z in [-0.05,0.10]).
        check(child->patch_vertices.size() == child->cube->vertices_3d.size(),
              "patch_vertices (x) and cube->vertices_3d (c) are index-aligned (same size)");

        bool coupling_holds = true;
        for (size_t i = 0; i < child->patch_vertices.size(); ++i) {
            double dx = CGAL::to_double(child->cube->vertices_3d[i].x() - child->patch_vertices[i].x());
            double dy = CGAL::to_double(child->cube->vertices_3d[i].y() - child->patch_vertices[i].y());
            double dz = CGAL::to_double(child->cube->vertices_3d[i].z() - child->patch_vertices[i].z());
            if (dx < -1e-6 || dx > 0.15 + 1e-6 || dy < 0.245 - 1e-6 || dy > 0.395 + 1e-6 || dz < -0.05 - 1e-6 || dz > 0.10 + 1e-6)
                coupling_holds = false;
        }
        check(coupling_holds, "every c_i - x_i lands inside K_cube's documented box (the §3.1 coupling invariant)");

        check(std::abs(child->cube->placement_yaw - parent->foot_yaw) < 1e-12,
              "cube's placement_yaw is frozen to the foot_yaw at placement time");
    }

    // --- §3.2: a normal step (expand_node) transports an active cube unchanged ---
    // Reuse one of the placement children above as the new parent, then take one more,
    // ordinary (non-cube) step from it via the *same* expand_node used everywhere else
    // in the codebase (Etape 4: expand_node itself carries the cube when cube_state ==
    // PlacedActive, every other caller -- cube_state == None -- hits the exact same code
    // path it always has).
    {
        ReachabilityModel foot_reach = ReachabilityModel::load({
            {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
            {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
        });

        Node* placed = children.front();
        check(placed->cube_state == CubeState::PlacedActive, "transport test: starting parent really is PlacedActive");

        auto stepped = expand_node(placed, foot_surfaces, foot_reach, ReachabilityDirection::Forward, params, pool);
        check(!stepped.empty(), "expand_node: a normal step is still possible from a cube-carrying node");

        for (Node* child : stepped) {
            check(child->cube_state == CubeState::PlacedActive, "transport: child keeps cube_state PlacedActive");
            check(child->cube.has_value(), "transport: child still carries a CubePlacement");
            if (!child->cube.has_value()) continue;
            check(child->stance_foot != placed->stance_foot, "transport: the OTHER foot moved this time (normal step, not placement)");
            check(child->patch_vertices.size() == child->cube->vertices_3d.size(),
                  "transport: x and c stay index-aligned after a normal step too");

            // c was never modified by this step (spec §3.2), only carried/interpolated
            // among placed->cube's own vertices -- so it must stay within THEIR bounding
            // box (a convex hull's interpolations/subsets never leave the hull of their
            // inputs), i.e. still within K_cube's box relative to the ORIGINAL (0,0,0)
            // placement foot position, same bounds as the placement invariant above.
            bool still_within_original_cube_bounds = true;
            for (const auto& c : child->cube->vertices_3d) {
                double x = CGAL::to_double(c.x()), y = CGAL::to_double(c.y()), z = CGAL::to_double(c.z());
                if (x < -1e-6 || x > 0.15 + 1e-6 || y < 0.245 - 1e-6 || y > 0.395 + 1e-6 || z < -0.05 - 1e-6 || z > 0.10 + 1e-6)
                    still_within_original_cube_bounds = false;
            }
            check(still_within_original_cube_bounds, "transport: c stays within the original K_cube placement bounds (never modified, only carried)");
        }
    }

    // --- §3.3: stepping onto the cube, and §3.4's v1 simplification (single-use) ---
    {
        ReachabilityModel foot_reach = ReachabilityModel::load({
            {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
            {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
        });

        // Negative control: not PlacedActive => no candidate children.
        {
            Node* inactive = pool.create();
            *inactive = *children.front();
            inactive->cube_state = CubeState::PlacedInactive;
            inactive->cube = std::nullopt;
            auto none = expand_onto_cube(inactive, foot_reach, cube_cfg.height, top_half_extent, params, pool);
            check(none.empty(), "expand_onto_cube: no children when cube_state != PlacedActive");
        }

        Node* placed = children.front();
        auto onto = expand_onto_cube(placed, foot_reach, cube_cfg.height, top_half_extent, params, pool);
        check(!onto.empty(), "expand_onto_cube: stepping onto a nearby cube on a flat scene is reachable");

        for (Node* child : onto) {
            check(child->cube_state == CubeState::PlacedInactive, "on-cube step: v1 always deactivates the cube afterward");
            check(!child->cube.has_value(), "on-cube step: cube is dropped (nullopt) once used");
            check(child->surface_id == kOnCubeSurfaceId, "on-cube step: surface_id is the on-cube sentinel, not a real surface");
            check(child->surface_id != -1, "on-cube step: sentinel is distinct from the root/no-surface value -1");
            check(child->stance_foot != placed->stance_foot, "on-cube step: the other foot is the one landing on the cube");
            check(child->depth == placed->depth + 1, "on-cube step: child is one depth level below the parent");

            // Landed exactly on top of the cube: z = placement surface z (0, Flat) + cube
            // height (0.15m) -- the whole geometric point of §3.3.
            bool landed_at_cube_height = true;
            for (const auto& p : child->patch_vertices) {
                if (std::abs(CGAL::to_double(p.z()) - cube_cfg.height) > 1e-6) landed_at_cube_height = false;
            }
            check(landed_at_cube_height, "on-cube step: every landing point is exactly at the cube's own height (0.15m above Flat)");

            // The landing patch stays within the cube's top eroded by the inner margin: never closer than
            // (half_extent - top_half_extent) = inner margin to the cube's own edge. Bounding check
            // against the extent of the possible cube centers.
            double cmin_x = 1e9, cmax_x = -1e9, cmin_y = 1e9, cmax_y = -1e9;
            for (const Point_3& c : placed->cube->vertices_3d) {
                cmin_x = std::min(cmin_x, CGAL::to_double(c.x())); cmax_x = std::max(cmax_x, CGAL::to_double(c.x()));
                cmin_y = std::min(cmin_y, CGAL::to_double(c.y())); cmax_y = std::max(cmax_y, CGAL::to_double(c.y()));
            }
            bool within_eroded_top = true;
            for (const auto& p : child->patch_vertices) {
                double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y());
                if (px < cmin_x - top_half_extent - 1e-6 || px > cmax_x + top_half_extent + 1e-6 ||
                    py < cmin_y - top_half_extent - 1e-6 || py > cmax_y + top_half_extent + 1e-6)
                    within_eroded_top = false;
            }
            check(within_eroded_top, "on-cube step: landing stays within the cube top eroded by the inner margin (not the full cube side)");
        }
    }

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All cube-expansion tests passed\n";
    return 0;
}
