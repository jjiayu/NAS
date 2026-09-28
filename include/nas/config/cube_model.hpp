#pragma once

// CubeConfig -- physical dimensions of the manipulable cube for the cube-
// placement extension (see docs/cube-extension-spec.md,
// docs/cube-implementation-plan.md). Same spirit as RobotModel: a plain
// struct with a couche-0 default, no loader here.

namespace nas::config {

struct CubeConfig {
    // Cube is square in plan; half_extent is half its side length: 0.15 m for the 30 cm cube (was
    // 0.075 for a 15 cm one, too small to step on with the same inner margin as any surface:
    // usable top = 2 x (half_extent - inner_margin)). Used for the cube's placement support
    // (half-diagonal erosion) and the on-cube-step footprint square (spec §3.3).
    double half_extent = 0.15;

    // Cube height (h in the spec's x' - c - h*n cut, §3.3).
    double height = 0.15;
};

} // namespace nas::config
