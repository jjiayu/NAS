#pragma once

// RobotModel — the foot/CoM dimensions old code read from constants.hpp
// globals (see PLAN.md phase 9). Couche 0 only, like core/reachability: no
// file loader here, just a plain struct with Talos's own values as
// defaults — a loader belongs with bindings/ (phase 12) if/when it's
// needed from Python.

namespace nas::config {

struct RobotModel {
    double foot_length = 0.22;
    double foot_width = 0.22;
    double com_z_height = 0.75;
};

} // namespace nas::config
