// Old footstep QP (CasADi + qpOASES) started COLD (its own behaviour: x0 = 0) and WARM
// (x0 = the rewritten pipeline's solution), on the same plan dump, to tell a solver-side failure
// from a real infeasibility. The old planner's source is not edited: the CMake build copies
// src/footstep_planner.cpp, adds a global `g_qp_warm_start` and one line passing it as
// arg["x0"] to the solver call (see CMakeLists.txt, target old_qp_warmstart).
// With OLD_QP_REG=<value> in the environment the objective also gets <value> * |x|^2 (a Hessian
// regularization, like the rewritten QP's 1e-8), to test whether the old failure comes from the
// semi-definite Hessian (alpha has no quadratic term).
// Variable order in the old QP: 3*n footstep coordinates (x, y, z per footstep), then alpha.
//
// Usage: old_qp_warmstart <plan.json>
#include "footstep_planner.hpp"
#include "constants.hpp"
#include "geometry.hpp"
#include "node.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <vector>

extern std::vector<double> g_qp_warm_start;

using namespace nas;
using json = nlohmann::json;

static Point_3 pt(const json& j) { return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>()); }

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: " << argv[0] << " <plan.json>\n"; return 1; }
    json plan;
    std::ifstream(argv[1]) >> plan;
    if (!plan.value("path_found", false) || !plan.value("qp_success", false)) { std::cout << "no plan to check\n"; return 2; }
    std::vector<Node*> path;
    for (const auto& nj : plan["path"]) {
        Node* n = new Node();
        n->depth = nj["depth"]; n->stance_foot = nj["stance_foot"]; n->foot_yaw = nj["foot_yaw"]; n->surface_id = nj["surface_id"];
        n->centroid = pt(nj["centroid"]);
        for (const auto& v : nj["patch_vertices"]) n->patch_vertices.push_back(pt(v));
        if (n->patch_vertices.size() >= 3) n->patch_polyhedron_3d = convex_hull_3_from_coplanar_points(n->patch_vertices, Vector_3(0, 0, 1));
        path.push_back(n);
    }
    const std::string name = plan["scenario"].get<std::string>();

    g_qp_warm_start.clear();
    FootstepPlanner cold;
    bool cold_ok = cold.plan(current_stance_foot_flag, pt(plan["start"]), stance_foot_at_goal, pt(plan["goal"]), path);

    // warm start: the new QP's footsteps, alpha = 0 (a feasible margin: constraints A x + alpha <= b hold with alpha = 0)
    for (const auto& f : plan["footsteps"])
        for (int k = 0; k < 3; ++k) g_qp_warm_start.push_back(f["position"][k].get<double>());
    g_qp_warm_start.push_back(0.0);
    FootstepPlanner warm;
    bool warm_ok = warm.plan(current_stance_foot_flag, pt(plan["start"]), stance_foot_at_goal, pt(plan["goal"]), path);

    double dev = 0.0;
    if (warm_ok) {
        const auto& fs = warm.get_computed_footsteps();
        for (size_t i = 0; i < fs.size() && i < plan["footsteps"].size(); ++i)
            for (int k = 0; k < 3; ++k) {
                double a = k == 0 ? CGAL::to_double(fs[i].x()) : k == 1 ? CGAL::to_double(fs[i].y()) : CGAL::to_double(fs[i].z());
                dev = std::max(dev, std::abs(a - plan["footsteps"][i]["position"][k].get<double>()));
            }
    }
    std::cout << "OLD_QP_WARM " << name << ": cold " << (cold_ok ? "solved" : "FAILED") << ", warm-started with the new solution "
              << (warm_ok ? "solved" : "FAILED") << (warm_ok ? " (max deviation from the new QP's footsteps " + std::to_string(dev * 1000) + " mm)" : "") << "\n";
    return 0;
}
