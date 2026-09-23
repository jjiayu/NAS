// nas_bindings — Python entrypoint for CASSR (see PLAN.md phase 12).
// Couche 0 only, same contract as the rest of the rewrite (core/reachability,
// config/): every input is an explicit path/name, nothing is discovered or
// guessed. A convenience layer (building a planner from a package that
// extracts its own files) was explicitly descoped, same reasoning as
// talosReachability's own couche 0/couche 1 split — see PLAN.md "Différé".
//
// Returns a flat DTO (FootstepResult) instead of exposing Node/Surface/CGAL
// types to Python — those are C++-internal implementation details (CGAL
// kernel types in particular have no sane nanobind binding), not a stable
// public API surface. The actual planning call (search + QP) releases the
// GIL, since it can take tens of milliseconds and has no need to touch
// Python objects while running.

#include "nas/config/planner_config.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <array>

namespace nb = nanobind;
using namespace nas;

namespace {

struct FootstepResult {
    bool success = false;
    // One entry per path node: world-frame (x, y, z), stance foot
    // (0=Left, 1=Right, matching StanceFoot's own values), foot yaw (rad).
    std::vector<std::array<double, 3>> positions;
    std::vector<int> stance_feet;
    std::vector<double> foot_yaws;
};

ReachabilityModel load_forward_reachability(const std::string& talos_data_dir) {
    std::vector<ReachabilityEntry> entries = {
        {talos_data_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {talos_data_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

// Every argument is an explicit path/name — see the module-level couche 0
// note above. Throws (propagated to Python as a RuntimeError/ValueError by
// nanobind) on a bad scenario name, malformed config, or missing .obj file,
// same as apps/astar_plan's own error handling.
FootstepResult plan(const std::string& scenario_name, const std::string& planner_config_path,
                     const std::string& talos_reachability_data_dir) {
    config::Scenario scenario = config::load_scenario(scenario_name);
    config::PlannerConfig planner_config = config::load_planner_config(planner_config_path);
    config::resolve_goal(planner_config, scenario); // "goal_offset" configs (see planner_config.hpp)
    ReachabilityModel reachability = load_forward_reachability(talos_reachability_data_dir);

    FootstepResult result;
    std::vector<Node*> path;
    FootstepPlan plan_result;

    {
        // The search and QP solve are the only parts worth releasing the
        // GIL for — no Python object is touched until this block exits.
        nb::gil_scoped_release release;

        AstarSearch search(scenario.surfaces, reachability, planner_config.astar);
        search.search();
        path = search.result_path();

        if (!path.empty()) {
            QuadprogBackend backend;
            plan_result = solve_footstep_qp(path, planner_config.astar.start_position, planner_config.astar.goal_location,
                                             reachability, planner_config.qp, backend);
        }
    }

    if (path.empty() || !plan_result.success) {
        return result; // success = false, empty vectors
    }

    result.success = true;
    result.positions.reserve(path.size());
    result.stance_feet.reserve(path.size());
    result.foot_yaws.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        const Point_3& p = plan_result.footsteps[i];
        result.positions.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
        result.stance_feet.push_back(static_cast<int>(path[i]->stance_foot));
        result.foot_yaws.push_back(path[i]->foot_yaw);
    }
    return result;
}

} // namespace

NB_MODULE(nas_bindings, m) {
    m.doc() = "CASSR footstep planning — couche 0 Python entrypoint (see PLAN.md phase 12)";

    nb::class_<FootstepResult>(m, "FootstepResult")
        .def_ro("success", &FootstepResult::success)
        .def_ro("positions", &FootstepResult::positions)
        .def_ro("stance_feet", &FootstepResult::stance_feet)
        .def_ro("foot_yaws", &FootstepResult::foot_yaws);

    m.def("plan", &plan, nb::arg("scenario_name"), nb::arg("planner_config_path"), nb::arg("talos_reachability_data_dir"),
          "Run CASSR (AstarSearch + footstep QP) on a named config::available_scenarios() scenario.");

    m.def("available_scenarios", &config::available_scenarios,
          "Names of every scenario config::load_scenario() can build.");
}
