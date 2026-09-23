#include "nas/config/scenario.hpp"

#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace nas::config {

namespace {

using RawScenario = std::vector<std::vector<Point_3>>;

// Every raw vertex list below is copied verbatim from the old code's
// include/environments.hpp (see PLAN.md phase 9b) — same points, same
// winding order, same comments where useful. This is deliberately *not*
// deduplicated against tests/fixtures's own copies of NarrowPassage/
// ThreePathsNAS (see PLAN.md): that module is the test harness bundling an
// AstarSearchConfig too, this one is the production scenario registry.

const RawScenario kStairs = {
    // Floor
    {Point_3(-1.8, -1., 0.0), Point_3(0.3, -1., 0.0), Point_3(0.3, 1., 0.0), Point_3(-1.8, 1., 0.0)},
    // Step 1
    {Point_3(0.3, -0.16, 0.1), Point_3(0.6, -0.16, 0.1), Point_3(0.6, 0.6, 0.1), Point_3(0.3, 0.6, 0.1)},
    // Step 2
    {Point_3(0.6, -0.16, 0.2), Point_3(0.9, -0.16, 0.2), Point_3(0.9, 0.6, 0.2), Point_3(0.6, 0.6, 0.2)},
    // Step 3
    {Point_3(0.9, -0.16, 0.3), Point_3(1.2, -0.16, 0.3), Point_3(1.2, 0.6, 0.3), Point_3(0.9, 0.6, 0.3)},
    // Step 4
    {Point_3(1.2, -0.16, 0.4), Point_3(1.5, -0.16, 0.4), Point_3(1.5, 0.6, 0.4), Point_3(1.2, 0.6, 0.4)},
};

// Same as kStairs but with Step 1 removed: floor -> (small gap) -> Step 2/3/4.
// Cube-extension test scenario (see docs/cube-extension-spec.md /
// docs/cube-implementation-plan.md): a normal footstep can't climb the
// resulting 0.2m double riser directly, but a 15cm cube split into two
// smaller rises (0m->0.15m, then 0.15m->0.2m) can.
//
// REVISED 2026-09-24 (see docs/cube-implementation-plan.md): the original
// version kept the floor at its pre-removal edge (x<=0.3, a ~0.5m gap to
// Step 2), making the challenge a mix of horizontal reach AND height -- hard
// to bridge with a plausibly-sized cube, and hard to reason about which
// limit was actually binding. The floor now extends to x<=0.55 (5cm short of
// Step 2's edge at x=0.6): with a foot reachability height-capped at 0.18m
// (see *_clamp_z18.obj below), the challenge is now purely about height, not
// horizontal reach -- a direct floor->Step2 step needs 0.2m of vertical
// reach (blocked, > 0.18m cap) regardless of the (now small) horizontal
// distance, while floor->cube-top (0.15m) and cube-top->Step2 (0.05m) each
// fit comfortably under the cap.
//
// Whether the direct climb is actually infeasible depends on the
// reachability data: the default quasi_flat_REDUCED polytopes are generous
// enough (up to ~0.9m local Y, ~0.6m local Z once yaw is exploited) that even
// two missing risers stayed reachable directly under the original geometry --
// measured empirically with a standalone AstarSearch call before committing
// to this scenario, not assumed. With the conservative *_clamp_z18.obj
// polytopes (X<=0.2, Y<=0.3, Z<=0.18 -- talosReachability/data/
// reachability_constraints/), the direct climb is infeasible. That is the
// reachability the cube golden test must load explicitly (this scenario's
// own geometry doesn't encode a reachability choice, same as every other
// scenario here).
const RawScenario kStairsGap = {
    // Floor -- extended to x<=0.55 (was 0.3): 5cm short of Step 2's edge, see the
    // header comment above on why this makes height, not horizontal reach, the
    // binding constraint.
    {Point_3(-1.8, -1., 0.0), Point_3(0.55, -1., 0.0), Point_3(0.55, 1., 0.0), Point_3(-1.8, 1., 0.0)},
    // Step 2
    {Point_3(0.6, -0.16, 0.2), Point_3(0.9, -0.16, 0.2), Point_3(0.9, 0.6, 0.2), Point_3(0.6, 0.6, 0.2)},
    // Step 3
    {Point_3(0.9, -0.16, 0.3), Point_3(1.2, -0.16, 0.3), Point_3(1.2, 0.6, 0.3), Point_3(0.9, 0.6, 0.3)},
    // Step 4
    {Point_3(1.2, -0.16, 0.4), Point_3(1.5, -0.16, 0.4), Point_3(1.5, 0.6, 0.4), Point_3(1.2, 0.6, 0.4)},
};

const RawScenario kTwoFlatSurfaces = {
    {Point_3(0.0, 0.0, 0.0), Point_3(5.45, 0.0, 0.0), Point_3(5.45, 1.0, 0.0), Point_3(0.0, 1.0, 0.0)},
    {Point_3(5.5, 0.0, 0.0), Point_3(7.0, 0.0, 0.0), Point_3(7.0, 1.0, 0.0), Point_3(5.5, 1.0, 0.0)},
};

const RawScenario kFlat = {
    {Point_3(-2.0, -1.0, 0.0), Point_3(5.45, -1.0, 0.0), Point_3(5.45, 1.0, 0.0), Point_3(-2.0, 1.0, 0.0)},
};

const RawScenario kLongStairs = {
    // Floor
    {Point_3(0.12, 1.0, 0.0), Point_3(-0.5, 1.0, 0.0), Point_3(-0.5, -1.0, 0.0), Point_3(0.12, -1.0, 0.0)},
    // 1st stair
    {Point_3(0.42, 1.0, 0.1), Point_3(0.12, 1.0, 0.1), Point_3(0.12, 0.5, 0.1), Point_3(0.42, 0.5, 0.1)},
    {Point_3(0.42, -0.5, 0.1), Point_3(0.12, -0.5, 0.1), Point_3(0.12, -1.0, 0.1), Point_3(0.42, -1.0, 0.1)},
    // 2nd stair
    {Point_3(0.725, 0.8, 0.2), Point_3(0.425, 0.8, 0.2), Point_3(0.425, 0, 0.2), Point_3(0.725, 0, 0.2)},
    {Point_3(0.725, -0.5, 0.2), Point_3(0.425, -0.5, 0.2), Point_3(0.425, -1.0, 0.2), Point_3(0.725, -1.0, 0.2)},
    // 3rd stair
    {Point_3(1.03, 1.0, 0.3), Point_3(0.73, 1.0, 0.3), Point_3(0.73, -1.0, 0.3), Point_3(1.03, -1.0, 0.3)},
    // 4th stair
    {Point_3(1.33, 1.0, 0.4), Point_3(1.03, 1.0, 0.4), Point_3(1.03, 0, 0.4), Point_3(1.33, 0, 0.4)},
    {Point_3(1.33, -0.5, 0.4), Point_3(1.03, -0.5, 0.4), Point_3(1.03, -1.0, 0.4), Point_3(1.33, -1.0, 0.4)},
    // 5th stair
    {Point_3(1.63, 1.0, 0.5), Point_3(1.33, 1.0, 0.5), Point_3(1.33, 0.5, 0.5), Point_3(1.63, 0.5, 0.5)},
    {Point_3(1.63, -0.5, 0.5), Point_3(1.33, -0.5, 0.5), Point_3(1.33, -1.0, 0.5), Point_3(1.63, -1.0, 0.5)},
    // 6th stair
    {Point_3(1.93, 1.0, 0.6), Point_3(1.63, 1.0, 0.6), Point_3(1.63, -1.0, 0.6), Point_3(1.93, -1.0, 0.6)},
    // 7th stair
    {Point_3(2.23, 0.75, 0.7), Point_3(1.93, 0.75, 0.7), Point_3(1.93, 0.25, 0.7), Point_3(2.23, 0.25, 0.7)},
    {Point_3(2.23, -0.5, 0.7), Point_3(1.93, -0.5, 0.7), Point_3(1.93, -1.0, 0.7), Point_3(2.23, -1.0, 0.7)},
    // 8th stair
    {Point_3(2.53, 0.75, 0.8), Point_3(2.23, 0.75, 0.8), Point_3(2.23, -0.7, 0.8), Point_3(2.53, -0.7, 0.8)},
    {Point_3(2.53, -0.5, 0.8), Point_3(2.23, -0.5, 0.8), Point_3(2.23, -1.0, 0.8), Point_3(2.53, -1.0, 0.8)},
    // 9th stair (skip breaking)
    {Point_3(2.83, 0.75, 0.9), Point_3(2.53, 0.75, 0.9), Point_3(2.53, 0.25, 0.9), Point_3(2.83, 0.25, 0.9)},
    {Point_3(2.83, -0.5, 0.9), Point_3(2.53, -0.5, 0.9), Point_3(2.53, -1.0, 0.9), Point_3(2.83, -1.0, 0.9)},
    // 10th stair
    {Point_3(3.13, 1.0, 1.0), Point_3(2.83, 1.0, 1.0), Point_3(2.83, 0.0, 1.0), Point_3(3.13, 0.0, 1.0)},
    {Point_3(3.13, -0.5, 1.0), Point_3(2.83, -0.5, 1.0), Point_3(2.83, -1.0, 1.0), Point_3(3.13, -1.0, 1.0)},
    // 11th stair
    {Point_3(3.43, 0.5, 1.1), Point_3(3.13, 0.5, 1.1), Point_3(3.13, 0.0, 1.1), Point_3(3.43, 0.0, 1.1)},
    {Point_3(3.43, -0.5, 1.1), Point_3(3.13, -0.5, 1.1), Point_3(3.13, -1.0, 1.1), Point_3(3.43, -1.0, 1.1)},
    // Target floor (12th stair)
    {Point_3(3.73, 1.0, 1.2), Point_3(3.43, 1.0, 1.2), Point_3(3.43, -1.0, 1.2), Point_3(3.73, -1.0, 1.2)},
};

const RawScenario kLongLongStairs = {
    {Point_3(0.12, 1.0, 0.0), Point_3(-0.5, 1.0, 0.0), Point_3(-0.5, -1.0, 0.0), Point_3(0.12, -1.0, 0.0)},
    {Point_3(0.42, 1.0, 0.1), Point_3(0.12, 1.0, 0.1), Point_3(0.12, 0.5, 0.1), Point_3(0.42, 0.5, 0.1)},
    {Point_3(0.42, -0.5, 0.1), Point_3(0.12, -0.5, 0.1), Point_3(0.12, -1.0, 0.1), Point_3(0.42, -1.0, 0.1)},
    {Point_3(0.725, 0.8, 0.2), Point_3(0.425, 0.8, 0.2), Point_3(0.425, 0, 0.2), Point_3(0.725, 0, 0.2)},
    {Point_3(0.725, -0.5, 0.2), Point_3(0.425, -0.5, 0.2), Point_3(0.425, -1.0, 0.2), Point_3(0.725, -1.0, 0.2)},
    {Point_3(1.03, 0.5, 0.3), Point_3(0.73, 0.5, 0.3), Point_3(0.73, -1.0, 0.3), Point_3(1.03, -1.0, 0.3)},
    {Point_3(1.33, 1.0, 0.4), Point_3(1.03, 1.0, 0.4), Point_3(1.03, 0, 0.4), Point_3(1.33, 0, 0.4)},
    {Point_3(1.33, -0.5, 0.4), Point_3(1.03, -0.5, 0.4), Point_3(1.03, -1.0, 0.4), Point_3(1.33, -1.0, 0.4)},
    {Point_3(1.63, 1.0, 0.5), Point_3(1.33, 1.0, 0.5), Point_3(1.33, 0.5, 0.5), Point_3(1.63, 0.5, 0.5)},
    {Point_3(1.63, -0.5, 0.5), Point_3(1.33, -0.5, 0.5), Point_3(1.33, -1.0, 0.5), Point_3(1.63, -1.0, 0.5)},
    {Point_3(1.93, 1.0, 0.6), Point_3(1.63, 1.0, 0.6), Point_3(1.63, -1.0, 0.6), Point_3(1.93, -1.0, 0.6)},
    {Point_3(2.23, 0.75, 0.7), Point_3(1.93, 0.75, 0.7), Point_3(1.93, 0.25, 0.7), Point_3(2.23, 0.25, 0.7)},
    {Point_3(2.23, -0.5, 0.7), Point_3(1.93, -0.5, 0.7), Point_3(1.93, -1.0, 0.7), Point_3(2.23, -1.0, 0.7)},
    {Point_3(2.53, 0.75, 0.8), Point_3(2.23, 0.75, 0.8), Point_3(2.23, -0.7, 0.8), Point_3(2.53, -0.7, 0.8)},
    {Point_3(2.53, -0.5, 0.8), Point_3(2.23, -0.5, 0.8), Point_3(2.23, -1.0, 0.8), Point_3(2.53, -1.0, 0.8)},
    {Point_3(2.83, 0.75, 0.9), Point_3(2.53, 0.75, 0.9), Point_3(2.53, 0.25, 0.9), Point_3(2.83, 0.25, 0.9)},
    {Point_3(2.83, -0.5, 0.9), Point_3(2.53, -0.5, 0.9), Point_3(2.53, -1.0, 0.9), Point_3(2.83, -1.0, 0.9)},
    {Point_3(3.13, 1.0, 1.0), Point_3(2.83, 1.0, 1.0), Point_3(2.83, 0.0, 1.0), Point_3(3.13, 0.0, 1.0)},
    {Point_3(3.13, -0.5, 1.0), Point_3(2.83, -0.5, 1.0), Point_3(2.83, -1.0, 1.0), Point_3(3.13, -1.0, 1.0)},
    {Point_3(3.43, 0.5, 1.1), Point_3(3.13, 0.5, 1.1), Point_3(3.13, 0.0, 1.1), Point_3(3.43, 0.0, 1.1)},
    {Point_3(3.43, -0.5, 1.1), Point_3(3.13, -0.5, 1.1), Point_3(3.13, -1.0, 1.1), Point_3(3.43, -1.0, 1.1)},
    {Point_3(3.73, 1.0, 1.2), Point_3(3.43, 1.0, 1.2), Point_3(3.43, -1.0, 1.2), Point_3(3.73, -1.0, 1.2)},
    // Stage 2 surfaces
    {Point_3(4.03, 1.0, 1.3), Point_3(3.73, 1.0, 1.3), Point_3(3.73, 0.5, 1.3), Point_3(4.03, 0.5, 1.3)},
    {Point_3(4.03, -0.5, 1.3), Point_3(3.73, -0.5, 1.3), Point_3(3.73, -1.0, 1.3), Point_3(4.03, -1.0, 1.3)},
    {Point_3(4.335, 0.8, 1.4), Point_3(4.035, 0.8, 1.4), Point_3(4.035, 0, 1.4), Point_3(4.335, 0, 1.4)},
    {Point_3(4.335, -0.25, 1.4), Point_3(4.035, -0.25, 1.4), Point_3(4.035, -1.0, 1.4), Point_3(4.335, -1.0, 1.4)},
    {Point_3(4.64, 0.5, 1.5), Point_3(4.34, 0.5, 1.5), Point_3(4.34, -1.0, 1.5), Point_3(4.64, -1.0, 1.5)},
    {Point_3(4.94, 1.0, 1.6), Point_3(4.64, 1.0, 1.6), Point_3(4.64, 0, 1.6), Point_3(4.94, 0, 1.6)},
    {Point_3(4.94, -0.5, 1.6), Point_3(4.64, -0.5, 1.6), Point_3(4.64, -1.0, 1.6), Point_3(4.94, -1.0, 1.6)},
    {Point_3(5.24, 1.0, 1.7), Point_3(4.94, 1.0, 1.7), Point_3(4.94, 0.5, 1.7), Point_3(5.24, 0.5, 1.7)},
    {Point_3(5.24, -0.5, 1.7), Point_3(4.94, -0.5, 1.7), Point_3(4.94, -1.0, 1.7), Point_3(5.24, -1.0, 1.7)},
    {Point_3(5.54, 1.0, 1.8), Point_3(5.24, 1.0, 1.8), Point_3(5.24, -1.0, 1.8), Point_3(5.54, -1.0, 1.8)},
    {Point_3(5.84, 0.75, 1.9), Point_3(5.54, 0.75, 1.9), Point_3(5.54, 0.25, 1.9), Point_3(5.84, 0.25, 1.9)},
    {Point_3(5.84, -0.5, 1.9), Point_3(5.54, -0.5, 1.9), Point_3(5.54, -1.0, 1.9), Point_3(5.84, -1.0, 1.9)},
    {Point_3(6.14, 0.75, 2.0), Point_3(5.84, 0.75, 2.0), Point_3(5.84, -0.45, 2.0), Point_3(6.14, -0.45, 2.0)},
    {Point_3(6.14, -0.5, 2.0), Point_3(5.84, -0.5, 2.0), Point_3(5.84, -1.0, 2.0), Point_3(6.14, -1.0, 2.0)},
    {Point_3(6.44, 0.75, 2.1), Point_3(6.14, 0.75, 2.1), Point_3(6.14, 0.25, 2.1), Point_3(6.44, 0.25, 2.1)},
    {Point_3(6.44, -0.5, 2.1), Point_3(6.14, -0.5, 2.1), Point_3(6.14, -1.0, 2.1), Point_3(6.44, -1.0, 2.1)},
    {Point_3(6.74, 1.0, 2.2), Point_3(6.44, 1.0, 2.2), Point_3(6.44, 0.0, 2.2), Point_3(6.74, 0.0, 2.2)},
    {Point_3(6.74, -0.5, 2.2), Point_3(6.44, -0.5, 2.2), Point_3(6.44, -1.0, 2.2), Point_3(6.74, -1.0, 2.2)},
    {Point_3(7.04, 0.5, 2.3), Point_3(6.74, 0.5, 2.3), Point_3(6.74, 0.0, 2.3), Point_3(7.04, 0.0, 2.3)},
    {Point_3(7.04, -0.5, 2.3), Point_3(6.74, -0.5, 2.3), Point_3(6.74, -1.0, 2.3), Point_3(7.04, -1.0, 2.3)},
    // Target floor stage 2
    {Point_3(7.34, 1.0, 2.4), Point_3(7.04, 1.0, 2.4), Point_3(7.04, -1.0, 2.4), Point_3(7.34, -1.0, 2.4)},
};

const RawScenario kLongStairsComplete = {
    {Point_3(0.12, 1.0, 0.0), Point_3(-0.5, 1.0, 0.0), Point_3(-0.5, -1.0, 0.0), Point_3(0.12, -1.0, 0.0)},
    {Point_3(0.42, 1.0, 0.1), Point_3(0.12, 1.0, 0.1), Point_3(0.12, -1.0, 0.1), Point_3(0.42, -1.0, 0.1)},
    {Point_3(0.725, 1.0, 0.2), Point_3(0.425, 1.0, 0.2), Point_3(0.425, -1.0, 0.2), Point_3(0.725, -1.0, 0.2)},
    {Point_3(1.03, 1.0, 0.3), Point_3(0.73, 1.0, 0.3), Point_3(0.73, -1.0, 0.3), Point_3(1.03, -1.0, 0.3)},
    {Point_3(1.33, 1.0, 0.4), Point_3(1.03, 1.0, 0.4), Point_3(1.03, -1.0, 0.4), Point_3(1.33, -1.0, 0.4)},
    {Point_3(1.63, 1.0, 0.5), Point_3(1.33, 1.0, 0.5), Point_3(1.33, -1.0, 0.5), Point_3(1.63, -1.0, 0.5)},
    {Point_3(1.93, 1.0, 0.6), Point_3(1.63, 1.0, 0.6), Point_3(1.63, -1.0, 0.6), Point_3(1.93, -1.0, 0.6)},
    {Point_3(2.23, 1.0, 0.7), Point_3(1.93, 1.0, 0.7), Point_3(1.93, -1.0, 0.7), Point_3(2.23, -1.0, 0.7)},
    {Point_3(2.53, 1.0, 0.8), Point_3(2.23, 1.0, 0.8), Point_3(2.23, -1.0, 0.8), Point_3(2.53, -1.0, 0.8)},
    {Point_3(2.83, 1.0, 0.9), Point_3(2.53, 1.0, 0.9), Point_3(2.53, -1.0, 0.9), Point_3(2.83, -1.0, 0.9)},
    {Point_3(3.13, 1.0, 1.0), Point_3(2.83, 1.0, 1.0), Point_3(2.83, -1.0, 1.0), Point_3(3.13, -1.0, 1.0)},
    {Point_3(3.43, 1.0, 1.1), Point_3(3.13, 1.0, 1.1), Point_3(3.13, -1.0, 1.1), Point_3(3.43, -1.0, 1.1)},
    {Point_3(3.73, 1.0, 1.2), Point_3(3.43, 1.0, 1.2), Point_3(3.43, -1.0, 1.2), Point_3(3.73, -1.0, 1.2)},
};

const RawScenario kLongStairsExp = {
    {Point_3(0.12, 1.0, 0.0), Point_3(-0.5, 1.0, 0.0), Point_3(-0.5, -1.0, 0.0), Point_3(0.12, -1.0, 0.0)},
    {Point_3(0.42, 1.0, 0.1), Point_3(0.12, 1.0, 0.1), Point_3(0.12, 0.5, 0.1), Point_3(0.42, 0.5, 0.1)},
    {Point_3(0.42, -0.5, 0.1), Point_3(0.12, -0.5, 0.1), Point_3(0.12, -1.0, 0.1), Point_3(0.42, -1.0, 0.1)},
};

const RawScenario kThreePathsScene = {
    // Floor (start)
    {Point_3(0.15, 0.3, 0.0), Point_3(-0.15, 0.3, 0.0), Point_3(-0.15, -0.15, 0.0), Point_3(0.15, -0.15, 0.0)},
    // Path Up
    {Point_3(0.15, 0.62, 0.0), Point_3(-0.15, 0.62, 0.0), Point_3(-0.15, 0.32, 0.0), Point_3(0.15, 0.32, 0.0)},
    {Point_3(0.15, 0.94, 0.0), Point_3(-0.15, 0.94, 0.0), Point_3(-0.15, 0.64, 0.0), Point_3(0.15, 0.64, 0.0)},
    {Point_3(0.25, 1.175, 0.0), Point_3(-0.05, 1.175, 0.0), Point_3(-0.05, 0.875, 0.0), Point_3(0.25, 0.875, 0.0)},
    {Point_3(0.55, 1.175, 0.0), Point_3(0.3, 1.175, 0.0), Point_3(0.3, 0.875, 0.0), Point_3(0.55, 0.875, 0.0)},
    {Point_3(0.9, 1.175, 0.0), Point_3(0.6, 1.175, 0.0), Point_3(0.6, 0.875, 0.0), Point_3(0.9, 0.875, 0.0)},
    {Point_3(1.25, 1.175, 0.0), Point_3(0.95, 1.175, 0.0), Point_3(0.95, 0.875, 0.0), Point_3(1.25, 0.875, 0.0)},
    {Point_3(1.6, 1.175, 0.0), Point_3(1.3, 1.175, 0.0), Point_3(1.3, 0.875, 0.0), Point_3(1.6, 0.875, 0.0)},
    {Point_3(1.95, 1.175, 0.0), Point_3(1.65, 1.175, 0.0), Point_3(1.65, 0.875, 0.0), Point_3(1.95, 0.875, 0.0)},
    {Point_3(2.05, 0.85, 0.0), Point_3(1.75, 0.85, 0.0), Point_3(1.75, 0.55, 0.0), Point_3(2.05, 0.55, 0.0)},
    {Point_3(2.05, 0.5, 0.0), Point_3(1.75, 0.5, 0.0), Point_3(1.75, 0.2, 0.0), Point_3(2.05, 0.2, 0.0)},
    // Path Down
    {Point_3(0.15, -0.5, 0.0), Point_3(-0.15, -0.5, 0.0), Point_3(-0.15, -0.2, 0.0), Point_3(0.15, -0.2, 0.0)},
    {Point_3(0.15, -0.85, 0.0), Point_3(-0.15, -0.85, 0.0), Point_3(-0.15, -0.55, 0.0), Point_3(0.15, -0.55, 0.0)},
    {Point_3(0.25, -1.175, 0.0), Point_3(-0.05, -1.175, 0.0), Point_3(-0.05, -0.875, 0.0), Point_3(0.25, -0.875, 0.0)},
    {Point_3(0.55, -1.175, 0.0), Point_3(0.3, -1.175, 0.0), Point_3(0.3, -0.875, 0.0), Point_3(0.55, -0.875, 0.0)},
    {Point_3(0.9, -1.175, 0.0), Point_3(0.6, -1.175, 0.0), Point_3(0.6, -0.875, 0.0), Point_3(0.9, -0.875, 0.0)},
    {Point_3(1.25, -1.175, 0.0), Point_3(0.95, -1.175, 0.0), Point_3(0.95, -0.875, 0.0), Point_3(1.25, -0.875, 0.0)},
    {Point_3(1.6, -1.175, 0.0), Point_3(1.3, -1.175, 0.0), Point_3(1.3, -0.875, 0.0), Point_3(1.6, -0.875, 0.0)},
    {Point_3(1.95, -1.175, 0.0), Point_3(1.65, -1.175, 0.0), Point_3(1.65, -0.875, 0.0), Point_3(1.95, -0.875, 0.0)},
    {Point_3(2.05, -0.85, 0.0), Point_3(1.75, -0.85, 0.0), Point_3(1.75, -0.55, 0.0), Point_3(2.05, -0.55, 0.0)},
    {Point_3(2.05, -0.5, 0.0), Point_3(1.75, -0.5, 0.0), Point_3(1.75, -0.2, 0.0), Point_3(2.05, -0.2, 0.0)},
    // Dead end path
    {Point_3(0.5, 0.15, 0.0), Point_3(0.2, 0.15, 0.0), Point_3(0.2, -0.15, 0.0), Point_3(0.5, -0.15, 0.0)},
    // Target
    {Point_3(2.05, 0.15, 0.0), Point_3(1.75, 0.15, 0.0), Point_3(1.75, -0.15, 0.0), Point_3(2.05, -0.15, 0.0)},
};

const RawScenario kStairsUpDown = {
    {Point_3(-1.8, -1., 0.0), Point_3(0.6, -1., 0.0), Point_3(0.6, 1., 0.0), Point_3(-1.8, 1., 0.0)},
    {Point_3(0.6, -0.16, 0.1), Point_3(1.2, -0.16, 0.1), Point_3(1.2, 0.6, 0.1), Point_3(0.6, 0.6, 0.1)},
    {Point_3(1.2, -0.16, 0.2), Point_3(1.8, -0.16, 0.2), Point_3(1.8, 0.6, 0.2), Point_3(1.2, 0.6, 0.2)},
    {Point_3(1.8, -0.16, 0.3), Point_3(2.4, -0.16, 0.3), Point_3(2.4, 0.6, 0.3), Point_3(1.8, 0.6, 0.3)},
    {Point_3(2.4, -0.16, 0.2), Point_3(3.0, -0.16, 0.2), Point_3(3.0, 0.6, 0.2), Point_3(2.4, 0.6, 0.2)},
    {Point_3(3.0, -0.16, 0.1), Point_3(3.6, -0.16, 0.1), Point_3(3.6, 0.6, 0.1), Point_3(3.0, 0.6, 0.1)},
    {Point_3(3.6, -0.16, 0.0), Point_3(4.2, -0.16, 0.0), Point_3(4.2, 0.6, 0.0), Point_3(3.6, 0.6, 0.0)},
};

const RawScenario kThreePathsNAS = {
    // Starting platform (larger for initial stance)
    {Point_3(-0.3, -4.0, 0.0), Point_3(0.3, -4.0, 0.0), Point_3(0.3, 2.0, 0.0), Point_3(-0.3, 2.0, 0.0)},
    // Up Path 1-5
    {Point_3(0.32, 1.4, 0.0), Point_3(0.72, 1.4, 0.0), Point_3(0.72, 2.0, 0.0), Point_3(0.32, 2.0, 0.0)},
    {Point_3(0.74, 1.4, 0.0), Point_3(1.32, 1.4, 0.0), Point_3(1.32, 2.0, 0.0), Point_3(0.74, 2.0, 0.0)},
    {Point_3(1.34, 1.4, 0.0), Point_3(2.52, 1.4, 0.0), Point_3(2.52, 2.0, 0.0), Point_3(1.34, 2.0, 0.0)},
    {Point_3(2.54, 1.4, 0.0), Point_3(3.12, 1.4, 0.0), Point_3(3.12, 2.0, 0.0), Point_3(2.54, 2.0, 0.0)},
    {Point_3(3.14, 1.4, 0.0), Point_3(3.72, 1.4, 0.0), Point_3(3.72, 2.0, 0.0), Point_3(3.14, 2.0, 0.0)},
    {Point_3(3.74, 1.4, 0.0), Point_3(4.32, 1.4, 0.0), Point_3(4.32, 2.0, 0.0), Point_3(3.74, 2.0, 0.0)},
    // Down
    {Point_3(0.32, -3.2, 0.0), Point_3(1.2, -3.2, 0.0), Point_3(1.2, -4.0, 0.0), Point_3(0.32, -4.0, 0.0)},
    {Point_3(1.22, -3.2, 0.0), Point_3(2.2, -3.2, 0.0), Point_3(2.2, -4.0, 0.0), Point_3(1.22, -4.0, 0.0)},
    {Point_3(2.22, -3.2, 0.0), Point_3(3.2, -3.2, 0.0), Point_3(3.2, -4.0, 0.0), Point_3(2.22, -4.0, 0.0)},
    {Point_3(3.22, -3.2, 0.0), Point_3(4.32, -3.2, 0.0), Point_3(4.32, -4.0, 0.0), Point_3(3.22, -4.0, 0.0)},
    // Dead end
    {Point_3(0.32, -1.0, 0.0), Point_3(1.5, -1.0, 0.0), Point_3(1.5, -2.0, 0.0), Point_3(0.32, -2.0, 0.0)},
    // End Surface
    {Point_3(4.34, -4.0, 0.0), Point_3(4.94, -4.0, 0.0), Point_3(4.94, 2.0, 0.0), Point_3(4.34, 2.0, 0.0)},
};

const RawScenario kNarrowPassage = {
    {Point_3(-2.0, -2.0, 0.0), Point_3(2.0, -2.0, 0.0), Point_3(2.0, 2.0, 0.0), Point_3(-2.0, 2.0, 0.0)},
    {Point_3(2.0, -0.12, 0.0), Point_3(6.0, -0.12, 0.0), Point_3(6.0, 0.12, 0.0), Point_3(2.0, 0.12, 0.0)},
    {Point_3(6.0, -2.0, 0.0), Point_3(10.0, -2.0, 0.0), Point_3(10.0, 2.0, 0.0), Point_3(6.0, 2.0, 0.0)},
};

// ---- Inclined scenes (NOT from the old environments.hpp, which is all horizontal) ----
// Ramp helpers: a plane rising at `slope_deg` along x, y in [-1, 1].
RawScenario make_ramp(double slope_deg, double ramp_length) {
    const double dz = ramp_length * std::tan(slope_deg * M_PI / 180.0);
    return {
        {Point_3(-1.5, -1.0, 0.0), Point_3(0.0, -1.0, 0.0), Point_3(0.0, 1.0, 0.0), Point_3(-1.5, 1.0, 0.0)},          // floor
        {Point_3(0.0, -1.0, 0.0), Point_3(ramp_length, -1.0, dz), Point_3(ramp_length, 1.0, dz), Point_3(0.0, 1.0, 0.0)}, // ramp
        {Point_3(ramp_length, -1.0, dz), Point_3(ramp_length + 1.5, -1.0, dz), Point_3(ramp_length + 1.5, 1.0, dz), Point_3(ramp_length, 1.0, dz)}, // top
    };
}
// One inclined plane over the whole scene, x in [-2, 3], y in [-1, 1]: z = tan(a) * x (slope along the walk)
// or z = tan(a) * y (cross slope).
RawScenario make_sloped_plane(double slope_deg, bool cross_slope) {
    const double t = std::tan(slope_deg * M_PI / 180.0);
    auto z = [&](double x, double y) { return cross_slope ? t * y : t * x; };
    return {{Point_3(-2.0, -1.0, z(-2.0, -1.0)), Point_3(3.0, -1.0, z(3.0, -1.0)), Point_3(3.0, 1.0, z(3.0, 1.0)), Point_3(-2.0, 1.0, z(-2.0, 1.0))}};
}
const RawScenario kRamp = make_ramp(12.0, 3.0);
const RawScenario kSteepRamp = make_ramp(20.0, 2.0);
const RawScenario kSlopedGround = make_sloped_plane(10.0, false);
const RawScenario kSideSlope = make_sloped_plane(10.0, true);

// Start corridor -> one enlarged room (box sits near its far +y edge, well
// past where the robot actually stands to grasp it - not on top of any
// planned footstep) -> a short flight of stairs -> a goal landing. Revised
// from the first version (2026-09-23): that one had a separate small 0.5 x
// 0.6 alcove whose centroid ended up ~0.1m from the goal footstep - the box
// and the standing foot nearly coincided. Single bigger room instead (1.1 x
// 1.9), box pushed to its far edge, goal stance placed ~0.45m short of it
// along y. Also gives the goal-yaw-target leg (see BoxRoomStairs_leg1.json)
// enough room to actually turn back out afterward at a tight tolerance
// around 90deg (facing the box squarely) - the old cramped alcove couldn't
// (0 expansions, no path at 90deg; backed off to 45deg/15deg tolerance,
// itself a symptom of too little room, not addressed at the geometry level
// until now). See apps/astar_plan/examples/BoxRoomStairs_leg1/2.json.
const RawScenario kBoxRoomStairs = {
    // Start corridor
    {Point_3(0.0, -0.5, 0.0), Point_3(1.3, -0.5, 0.0), Point_3(1.3, 0.5, 0.0), Point_3(0.0, 0.5, 0.0)},
    // Room (enlarged; box sits near y=1.3, goal stance around y=0.65)
    {Point_3(1.3, -0.6, 0.0), Point_3(2.4, -0.6, 0.0), Point_3(2.4, 1.3, 0.0), Point_3(1.3, 1.3, 0.0)},
    // Stair tread 1
    {Point_3(2.4, -0.35, 0.1), Point_3(2.7, -0.35, 0.1), Point_3(2.7, 0.35, 0.1), Point_3(2.4, 0.35, 0.1)},
    // Stair tread 2
    {Point_3(2.7, -0.35, 0.2), Point_3(3.0, -0.35, 0.2), Point_3(3.0, 0.35, 0.2), Point_3(2.7, 0.35, 0.2)},
    // Stair tread 3
    {Point_3(3.0, -0.35, 0.3), Point_3(3.3, -0.35, 0.3), Point_3(3.3, 0.35, 0.3), Point_3(3.0, 0.35, 0.3)},
    // Goal landing
    {Point_3(3.3, -0.6, 0.3), Point_3(4.2, -0.6, 0.3), Point_3(4.2, 0.6, 0.3), Point_3(3.3, 0.6, 0.3)},
};

const std::unordered_map<std::string, const RawScenario*>& registry() {
    static const std::unordered_map<std::string, const RawScenario*> kRegistry = {
        {"Stairs", &kStairs},
        {"StairsGap", &kStairsGap},
        {"TwoFlatSurfaces", &kTwoFlatSurfaces},
        {"Flat", &kFlat},
        {"LongStairs", &kLongStairs},
        {"LongLongStairs", &kLongLongStairs},
        {"LongStairsComplete", &kLongStairsComplete},
        {"LongStairsExp", &kLongStairsExp},
        {"ThreePathsScene", &kThreePathsScene},
        {"Stairs_Up_Down", &kStairsUpDown},
        {"ThreePathsNAS", &kThreePathsNAS},
        {"NarrowPassage", &kNarrowPassage},
        {"Ramp", &kRamp},
        {"SteepRamp", &kSteepRamp},
        {"SlopedGround", &kSlopedGround},
        {"SideSlope", &kSideSlope},
        {"BoxRoomStairs", &kBoxRoomStairs},
    };
    return kRegistry;
}

} // namespace

std::vector<std::string> available_scenarios() {
    std::vector<std::string> names;
    for (const auto& [name, raw] : registry()) {
        names.push_back(name);
    }
    return names;
}

Scenario load_scenario(const std::string& name, const RobotModel& robot_model) {
    auto it = registry().find(name);
    if (it == registry().end()) {
        throw std::out_of_range("load_scenario: unknown scenario '" + name + "'");
    }
    Scenario scenario;
    scenario.name = name;
    const RawScenario& raw = *it->second;
    for (size_t i = 0; i < raw.size(); ++i) {
        scenario.surfaces.emplace_back(raw[i], static_cast<int>(i), robot_model.foot_length, robot_model.foot_width);
    }
    return scenario;
}

} // namespace nas::config
