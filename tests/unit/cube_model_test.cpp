// Directed test for the cube-extension groundwork (see
// docs/cube-extension-spec.md / docs/cube-implementation-plan.md, Etape 1):
// the hand-authored K_cube placement polytope loads through the existing
// ReachabilityModel unchanged (moving_effector="Cube"), and CubeConfig's
// defaults match the 15cm cube decided on 2026-09-23.

#include "nas/config/cube_model.hpp"
#include "nas/core/reachability.hpp"

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

int run_config_cube() {
    const std::string data_dir = TALOS_REACHABILITY_DATA_DIR;

    std::vector<ReachabilityEntry> entries = {
        {data_dir + "/Cube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {data_dir + "/Cube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    };
    ReachabilityModel model = ReachabilityModel::load(entries);
    check(model.size() == 2, "loaded both Cube-in-LF and Cube-in-RF entries");

    // LF and RF are mirrored in Y (not identical): the cube must be offset past the real
    // reachability's own ~0.19m minimum lateral stance-width clearance, toward whichever
    // side the OTHER foot (the one that will eventually step onto it) naturally reaches --
    // +y when RF supports (LF steps), -y when LF supports (RF steps). Found empirically
    // validating against StairsGap: a Y-centered box placed the cube inside that dead zone,
    // unreachable by either foot (see docs/cube-implementation-plan.md).
    struct Expected { const char* support; double y_min, y_max; };
    for (const Expected& e : {Expected{"RF", 0.20, 0.35}, Expected{"LF", -0.35, -0.20}}) {
        const Polyhedron& k_cube = model.query("Cube", e.support, ReachabilityDirection::Forward);
        long n_verts = std::distance(k_cube.vertices_begin(), k_cube.vertices_end());
        check(n_verts == 8, std::string("Cube-in-") + e.support + " loads as the 8-vertex hand-authored box");
        check(k_cube.is_closed(), std::string("Cube-in-") + e.support + " is a closed (valid) polyhedron");

        // Sanity-check the conservative box bounds documented in the .obj file itself:
        // catches an accidental unit/scale mistake (e.g. cm instead of m) early, and pins
        // down the deliberate LF/RF Y-mirroring so it can't silently drift back to centered.
        double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9, min_z = 1e9, max_z = -1e9;
        for (auto v = k_cube.vertices_begin(); v != k_cube.vertices_end(); ++v) {
            double x = CGAL::to_double(v->point().x());
            double y = CGAL::to_double(v->point().y());
            double z = CGAL::to_double(v->point().z());
            min_x = std::min(min_x, x); max_x = std::max(max_x, x);
            min_y = std::min(min_y, y); max_y = std::max(max_y, y);
            min_z = std::min(min_z, z); max_z = std::max(max_z, z);
        }
        check(std::abs(min_x) < 1e-9 && std::abs(max_x - 0.15) < 1e-9,
              std::string("Cube-in-") + e.support + " X (forward) bounds match the documented [0.0, 0.15]");
        check(std::abs(min_y - e.y_min) < 1e-9 && std::abs(max_y - e.y_max) < 1e-9,
              std::string("Cube-in-") + e.support + " Y (lateral) bounds match the documented, mirrored range");
        check(std::abs(min_z + 0.05) < 1e-9 && std::abs(max_z - 0.10) < 1e-9,
              std::string("Cube-in-") + e.support + " Z bounds match the documented [-0.05, 0.10]");
    }

    config::CubeConfig cube_cfg;
    check(std::abs(cube_cfg.half_extent - 0.075) < 1e-12, "CubeConfig default half_extent is 7.5cm (15cm cube)");
    check(std::abs(cube_cfg.height - 0.15) < 1e-12, "CubeConfig default height is 15cm");

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All cube-model tests passed\n";
    return 0;
}
