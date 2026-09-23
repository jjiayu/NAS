#include "nas/config/scenario.hpp"

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

const std::unordered_map<std::string, const RawScenario*>& registry() {
    static const std::unordered_map<std::string, const RawScenario*> kRegistry = {
        {"Stairs", &kStairs},
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
