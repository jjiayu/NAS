#pragma once

// STL scene import (see PLAN.md phase 9d) — an alternative to
// config::load_scenario()'s hardcoded vertex lists: reads an STL mesh
// (ASCII or binary), groups its triangles into convex, (near-)planar
// surfaces, and builds the same Surface objects load_scenario() does.
//
// Grouping: adjacent triangles whose plane (normal direction + offset from
// origin) matches within a small tolerance are merged into one surface's
// raw vertex list — see stl_import.cpp for the exact quantization. This
// assumes the input mesh already has each flat face split into triangles
// with a consistent per-face normal (true of any STL exporter), not an
// arbitrary triangle soup that happens to be locally coplanar.

#include "nas/config/robot_model.hpp"
#include "nas/config/scenario.hpp"

#include <string>

namespace nas::config {

// Throws std::runtime_error if the file can't be opened, doesn't parse as
// a recognized ASCII/binary STL, or has fewer than 3 triangles.
Scenario load_scenario_from_stl(const std::string& stl_path, const RobotModel& robot_model = RobotModel{});

} // namespace nas::config
