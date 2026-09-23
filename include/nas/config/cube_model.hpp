#pragma once

// CubeConfig -- physical dimensions of the manipulable cube for the cube-
// placement extension (see docs/cube-extension-spec.md,
// docs/cube-implementation-plan.md). Same spirit as RobotModel: a plain
// struct with a couche-0 default, no loader here.

namespace nas::config {

struct CubeConfig {
    // Cube is square in plan; half_extent is half its side length (0.075m
    // for the 15cm cube decided on 2026-09-23 -- see cube-implementation-plan.md
    // Etape 1). Used both for the placement surface erosion (S~_j, spec §3.1)
    // and the on-cube-step footprint square (spec §3.3).
    double half_extent = 0.075;

    // Cube height (h in the spec's x' - c - h*n cut, §3.3).
    double height = 0.15;
};

} // namespace nas::config
