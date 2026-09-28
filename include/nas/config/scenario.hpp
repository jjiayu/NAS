#pragma once

// Scenario + named scenario registry — the "production" counterpart to
// tests/fixtures::Scenario (see PLAN.md phase 9b). tests/fixtures stays the
// test harness (2 scenarios, bundled with an AstarSearchConfig); this
// module builds Surface objects for any of the raw vertex lists the old
// code's environments.hpp defined as globals, from just a name and an inner
// margin. No AstarSearchConfig here — a scenario's surfaces don't
// determine a search config, that stays the caller's job.

#include "nas/core/surface.hpp"

#include <string>
#include <vector>

namespace nas::config {

struct Scenario {
    std::string name;
    std::vector<Surface> surfaces;
};

// Names of every scene ported from the old code's environments.hpp.
std::vector<std::string> available_scenarios();

// Builds a Scenario's Surface objects from the named raw vertex list, each footprint eroded by
// `inner_margin` metres (see make_surfaces in core/surface.hpp: a too-thin surface is dropped).
// Throws std::out_of_range if `name` isn't in available_scenarios().
Scenario load_scenario(const std::string& name, double inner_margin = kDefaultInnerMargin);

} // namespace nas::config
