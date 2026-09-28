// nas_bindings — Python entrypoint for CASSR (see PLAN.md phase 12).
// Couche 0 for scenario/reachability discovery: every scenario name and data directory is an
// explicit argument, nothing is discovered or guessed (a convenience layer that builds a planner
// from a package extracting its own files was explicitly descoped, same reasoning as
// talosReachability's own couche 0/couche 1 split — see PLAN.md "Différé"). The *config* itself can
// be built two ways: from a JSON file (plan(), the original path) or directly in Python
// (plan_with_config(), added later — see PyPlannerConfig/PyFootGoal below), both converging on the
// same nas::config::PlannerConfig before the shared run_plan() takes over.
//
// Returns a flat DTO (FootstepResult) instead of exposing Node/Surface/CGAL types to Python — those
// are C++-internal implementation details (CGAL kernel types in particular have no sane nanobind
// binding), not a stable public API surface. Same reasoning motivates PyFootGoal/PyPlannerConfig
// below: Python-friendly mirrors (plain doubles/ints/arrays) of AstarSearchConfig::FootGoal/
// AstarSearchConfig/FootstepQPConfig, converted to the real (CGAL-using) types inside this file —
// Python never touches a Point_3. The actual planning call (search + QP) releases the GIL, since it
// can take tens of milliseconds and has no need to touch Python objects while running.

#include "nas/config/planner_config.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/footstep_qp/quadprog_backend.hpp"
#include "nas/planners/astar_search.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

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
    // Timing/search stats, same fields as apps/astar_plan's own JSON output -- Python callers had no
    // way to see these before (only the CLI did).
    int expansion_count = 0;
    double search_ms = 0.0;
    double qp_ms = 0.0;
};

// ---- Python-facing mirror of AstarSearchConfig::FootGoal/PlannerConfig (CGAL-free) ----
//
// Unlike config::parse_foot_goal_slot (src/config/planner_config.cpp), which must defend against
// untyped JSON text ("exactly one of point/surface/polytope/offset"), PyFootGoal makes an invalid
// region unrepresentable: private-ish storage (one Kind), only reachable through the 5 named static
// factories below, each setting exactly one field. No runtime "exactly one region" check needed.
struct PyFootGoal {
    enum class Kind { Point, Surface, Polytope, Offset, Polygon2D };
    Kind kind = Kind::Point;
    std::array<double, 3> point{};                // Point
    int surface = -1;                              // Surface, and the target surface for Polygon2D
    std::vector<std::array<double, 3>> polytope;    // Polytope (world frame)
    std::array<double, 3> offset{};                 // Offset
    std::vector<std::array<double, 2>> polygon_2d;  // Polygon2D (that surface's own local 2D frame)
    std::optional<std::pair<double, double>> yaw_range_deg;

    static PyFootGoal make_point(std::array<double, 3> p) {
        PyFootGoal g;
        g.kind = Kind::Point;
        g.point = p;
        return g;
    }
    static PyFootGoal make_surface(int surface_id) {
        PyFootGoal g;
        g.kind = Kind::Surface;
        g.surface = surface_id;
        return g;
    }
    static PyFootGoal make_polytope(std::vector<std::array<double, 3>> vertices) {
        PyFootGoal g;
        g.kind = Kind::Polytope;
        g.polytope = std::move(vertices);
        return g;
    }
    static PyFootGoal make_offset(std::array<double, 3> offset) {
        PyFootGoal g;
        g.kind = Kind::Offset;
        g.offset = offset;
        return g;
    }
    static PyFootGoal make_polygon_2d(int surface_id, std::vector<std::array<double, 2>> polygon) {
        PyFootGoal g;
        g.kind = Kind::Polygon2D;
        g.surface = surface_id;
        g.polygon_2d = std::move(polygon);
        return g;
    }
};

struct PyFootGoals {
    std::optional<PyFootGoal> left;
    std::optional<PyFootGoal> right;
};

// Same field coverage as config::parse_astar_config/parse_qp_config (src/config/planner_config.cpp)
// — not more, not less: a direct alternative front-end to the same options the JSON schema exposes,
// not a reduced subset. Defaults match AstarSearchConfig/ExpansionParams/FootstepQPConfig's own
// default member initializers exactly, so an untouched PlannerConfig behaves like a minimal JSON
// config ({"astar": {"start_position": ..., "foot_goals": ...}}).
struct PyPlannerConfig {
    std::array<double, 3> start_position;
    StanceFoot start_stance_foot = StanceFoot::Right;
    double start_foot_yaw = 0.0;
    PyFootGoals foot_goals;
    DistanceMetric distance_metric = DistanceMetric::Epa;
    double heading_weight = 0.0;
    double yaw_change_weight = 0.0;
    double heuristic_weight = 10.0;
    double step_weight = 1.0;
    double goal_yaw_weight = 0.0;
    double node_similarity_threshold = 0.02;
    double patch_index_cell_size = 0.05;
    bool rotation_enabled = false;
    int yaw_discretization_num = 3;
    double yaw_angle_increment_deg = 10.0;
    bool cycle_detection_enabled = true;
    double qp_alpha_weight = 10.0;
    bool qp_rotation_enabled = false;
    double qp_hessian_regularization = 1e-8;

    explicit PyPlannerConfig(std::array<double, 3> start) : start_position(start) {}
};

// PyFootGoal -> AstarSearchConfig::FootGoal. Offset/Polygon2D need the Scenario to resolve (same
// split as config::PendingFootGoalRegion/config::resolve_goal): `pending`, if set, is picked up by
// resolve_goal() exactly like a JSON "offset"/"surface"+"polygon_2d" region would be.
AstarSearchConfig::FootGoal foot_goal_from_py(const PyFootGoal& g, std::optional<config::PendingFootGoalRegion>& pending) {
    AstarSearchConfig::FootGoal out;
    switch (g.kind) {
        case PyFootGoal::Kind::Point:
            out.region = Point_3(g.point[0], g.point[1], g.point[2]);
            break;
        case PyFootGoal::Kind::Surface:
            out.region = g.surface;
            break;
        case PyFootGoal::Kind::Polytope: {
            std::vector<Point_3> verts;
            verts.reserve(g.polytope.size());
            for (const auto& v : g.polytope) verts.emplace_back(v[0], v[1], v[2]);
            out.region = std::move(verts);
            break;
        }
        case PyFootGoal::Kind::Offset: {
            out.region = Point_3(0, 0, 0); // placeholder -- resolve_goal fills this in
            config::PendingFootGoalRegion p;
            p.offset = Vector_3(g.offset[0], g.offset[1], g.offset[2]);
            pending = std::move(p);
            break;
        }
        case PyFootGoal::Kind::Polygon2D: {
            out.region = std::vector<Point_3>{}; // placeholder -- resolve_goal lifts this into 3D
            std::vector<Point_2> poly;
            poly.reserve(g.polygon_2d.size());
            for (const auto& v : g.polygon_2d) poly.emplace_back(v[0], v[1]);
            config::PendingFootGoalRegion p;
            p.polygon_on_surface = std::make_pair(g.surface, std::move(poly));
            pending = std::move(p);
            break;
        }
    }
    if (g.yaw_range_deg) {
        out.yaw_range = std::make_pair(g.yaw_range_deg->first / 180.0 * M_PI, g.yaw_range_deg->second / 180.0 * M_PI);
    }
    return out;
}

config::PlannerConfig planner_config_from_py(const PyPlannerConfig& py) {
    config::PlannerConfig out;
    out.astar.start_position = Point_3(py.start_position[0], py.start_position[1], py.start_position[2]);
    out.astar.start_stance_foot = py.start_stance_foot;
    out.astar.start_foot_yaw = py.start_foot_yaw;

    if (py.foot_goals.left) out.astar.foot_goals[0] = foot_goal_from_py(*py.foot_goals.left, out.pending_foot_goal_regions[0]);
    if (py.foot_goals.right) out.astar.foot_goals[1] = foot_goal_from_py(*py.foot_goals.right, out.pending_foot_goal_regions[1]);
    if (!out.astar.foot_goals[0] && !out.astar.foot_goals[1]) {
        throw std::invalid_argument("PlannerConfig: foot_goals needs at least \"left\" or \"right\"");
    }

    out.astar.distance_metric = py.distance_metric;
    out.astar.heading_weight = py.heading_weight;
    out.astar.yaw_change_weight = py.yaw_change_weight;
    out.astar.heuristic_weight = py.heuristic_weight;
    out.astar.step_weight = py.step_weight;
    out.astar.goal_yaw_weight = py.goal_yaw_weight;
    out.astar.node_similarity_threshold = py.node_similarity_threshold;
    out.astar.patch_index_cell_size = py.patch_index_cell_size;
    out.astar.expansion_params.rotation_enabled = py.rotation_enabled;
    out.astar.expansion_params.yaw_discretization_num = py.yaw_discretization_num;
    out.astar.expansion_params.yaw_angle_increment = py.yaw_angle_increment_deg / 180.0 * M_PI;
    out.astar.expansion_params.cycle_detection_enabled = py.cycle_detection_enabled;

    out.qp.alpha_weight = py.qp_alpha_weight;
    out.qp.rotation_enabled = py.qp_rotation_enabled;
    out.qp.hessian_regularization = py.qp_hessian_regularization;
    return out;
}

ReachabilityModel load_forward_reachability(const std::string& talos_data_dir) {
    std::vector<ReachabilityEntry> entries = {
        {talos_data_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {talos_data_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

// Shared by plan()/plan_with_config() -- the only difference between them is how planner_config was
// built (JSON file vs. Python objects); resolve_goal() must already have run on it.
FootstepResult run_plan(const config::Scenario& scenario, const config::PlannerConfig& planner_config,
                         const ReachabilityModel& reachability) {
    FootstepResult result;
    std::vector<Node*> path;
    FootstepPlan plan_result;

    {
        // The search and QP solve are the only parts worth releasing the
        // GIL for — no Python object is touched until this block exits.
        nb::gil_scoped_release release;

        using Clock = std::chrono::steady_clock;
        AstarSearch search(scenario.surfaces, reachability, planner_config.astar);
        auto t0 = Clock::now();
        search.search();
        result.search_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        result.expansion_count = search.expansion_count();
        path = search.result_path();

        if (!path.empty()) {
            QuadprogBackend backend;
            auto q0 = Clock::now();
            plan_result = solve_footstep_qp(path, planner_config.astar.start_position, config::qp_goal(planner_config),
                                             reachability, planner_config.qp, backend);
            result.qp_ms = std::chrono::duration<double, std::milli>(Clock::now() - q0).count();
        }
    }

    if (path.empty() || !plan_result.success) {
        return result; // success = false, empty vectors, timing fields still populated
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

// Every argument is an explicit path/name — see the module-level couche 0
// note above. Throws (propagated to Python as a RuntimeError/ValueError by
// nanobind) on a bad scenario name, malformed config, or missing .obj file,
// same as apps/astar_plan's own error handling.
FootstepResult plan(const std::string& scenario_name, const std::string& planner_config_path,
                     const std::string& talos_reachability_data_dir) {
    config::Scenario scenario = config::load_scenario(scenario_name);
    config::PlannerConfig planner_config = config::load_planner_config(planner_config_path);
    config::resolve_goal(planner_config, scenario); // "offset"/"polygon_2d" regions (see planner_config.hpp)
    ReachabilityModel reachability = load_forward_reachability(talos_reachability_data_dir);
    return run_plan(scenario, planner_config, reachability);
}

// Same contract as plan(), but the config is built directly in Python (PyPlannerConfig/PyFootGoal)
// instead of loaded from a JSON file -- no file needed to iterate on a goal from a REPL/notebook.
FootstepResult plan_with_config(const std::string& scenario_name, const PyPlannerConfig& python_config,
                                 const std::string& talos_reachability_data_dir) {
    config::Scenario scenario = config::load_scenario(scenario_name);
    config::PlannerConfig planner_config = planner_config_from_py(python_config);
    config::resolve_goal(planner_config, scenario);
    ReachabilityModel reachability = load_forward_reachability(talos_reachability_data_dir);
    return run_plan(scenario, planner_config, reachability);
}

} // namespace

NB_MODULE(nas_bindings, m) {
    m.doc() = "CASSR footstep planning — couche 0 Python entrypoint (see PLAN.md phase 12)";

    nb::class_<FootstepResult>(m, "FootstepResult")
        .def_ro("success", &FootstepResult::success)
        .def_ro("positions", &FootstepResult::positions)
        .def_ro("stance_feet", &FootstepResult::stance_feet)
        .def_ro("foot_yaws", &FootstepResult::foot_yaws)
        .def_ro("expansion_count", &FootstepResult::expansion_count)
        .def_ro("search_ms", &FootstepResult::search_ms)
        .def_ro("qp_ms", &FootstepResult::qp_ms);

    nb::enum_<StanceFoot>(m, "StanceFoot")
        .value("Left", StanceFoot::Left)
        .value("Right", StanceFoot::Right);

    nb::enum_<DistanceMetric>(m, "DistanceMetric")
        .value("Epa", DistanceMetric::Epa)
        .value("Euclidean", DistanceMetric::Euclidean);

    nb::class_<PyFootGoal>(m, "FootGoal",
        "A goal for ONE foot: build with exactly one of the static factories below, optionally set "
        "yaw_range_deg afterward. Mirrors AstarSearchConfig::FootGoal (see docs/tutorial-running-a-"
        "scenario.md for the region shapes' full semantics).")
        .def_static("point", &PyFootGoal::make_point, nb::arg("point"), "A precise (x, y, z) target, world frame.")
        .def_static("surface", &PyFootGoal::make_surface, nb::arg("surface_id"),
                    "Anywhere on this surface (index equality against the landing node's own surface_id, not geometric containment).")
        .def_static("polytope", &PyFootGoal::make_polytope, nb::arg("vertices"),
                    "An arbitrary polytope target (world frame, >= 3 vertices).")
        .def_static("offset", &PyFootGoal::make_offset, nb::arg("offset"),
                    "The scenario's LAST surface centroid + this vector, resolved once the scenario is loaded.")
        .def_static("polygon_2d", &PyFootGoal::make_polygon_2d, nb::arg("surface_id"), nb::arg("polygon"),
                    "A polygon in that surface's own local 2D frame, lifted to a world-frame polytope at resolve time.")
        .def_rw("yaw_range_deg", &PyFootGoal::yaw_range_deg, "Optional accepted [min, max] yaw window, degrees.");

    nb::class_<PyFootGoals>(m, "FootGoals", "Indexed by foot: at least one of left/right must be set.")
        .def(nb::init<>())
        .def_rw("left", &PyFootGoals::left)
        .def_rw("right", &PyFootGoals::right);

    nb::class_<PyPlannerConfig>(m, "PlannerConfig",
        "Direct Python-native alternative to load_planner_config()'s JSON file — same fields as the "
        "\"astar\"/\"qp\" JSON sections, same defaults, see docs/tutorial-running-a-scenario.md.")
        .def(nb::init<std::array<double, 3>>(), nb::arg("start_position"))
        .def_rw("start_position", &PyPlannerConfig::start_position)
        .def_rw("start_stance_foot", &PyPlannerConfig::start_stance_foot)
        .def_rw("start_foot_yaw", &PyPlannerConfig::start_foot_yaw)
        .def_rw("foot_goals", &PyPlannerConfig::foot_goals)
        .def_rw("distance_metric", &PyPlannerConfig::distance_metric)
        .def_rw("heading_weight", &PyPlannerConfig::heading_weight)
        .def_rw("yaw_change_weight", &PyPlannerConfig::yaw_change_weight)
        .def_rw("heuristic_weight", &PyPlannerConfig::heuristic_weight)
        .def_rw("step_weight", &PyPlannerConfig::step_weight)
        .def_rw("goal_yaw_weight", &PyPlannerConfig::goal_yaw_weight)
        .def_rw("node_similarity_threshold", &PyPlannerConfig::node_similarity_threshold)
        .def_rw("patch_index_cell_size", &PyPlannerConfig::patch_index_cell_size)
        .def_rw("rotation_enabled", &PyPlannerConfig::rotation_enabled)
        .def_rw("yaw_discretization_num", &PyPlannerConfig::yaw_discretization_num)
        .def_rw("yaw_angle_increment_deg", &PyPlannerConfig::yaw_angle_increment_deg)
        .def_rw("cycle_detection_enabled", &PyPlannerConfig::cycle_detection_enabled)
        .def_rw("qp_alpha_weight", &PyPlannerConfig::qp_alpha_weight)
        .def_rw("qp_rotation_enabled", &PyPlannerConfig::qp_rotation_enabled)
        .def_rw("qp_hessian_regularization", &PyPlannerConfig::qp_hessian_regularization);

    m.def("plan", &plan, nb::arg("scenario_name"), nb::arg("planner_config_path"), nb::arg("talos_reachability_data_dir"),
          "Run CASSR (AstarSearch + footstep QP) on a named config::available_scenarios() scenario, config from a JSON file.");

    m.def("plan_with_config", &plan_with_config, nb::arg("scenario_name"), nb::arg("config"), nb::arg("talos_reachability_data_dir"),
          "Run CASSR (AstarSearch + footstep QP) on a named scenario, config built directly in Python (PlannerConfig/FootGoal) -- no JSON file.");

    m.def("available_scenarios", &config::available_scenarios,
          "Names of every scenario config::load_scenario() can build.");
}
