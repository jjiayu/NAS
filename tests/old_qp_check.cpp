// Feasibility check with the OLD footstep QP (CasADi + qpOASES, src/footstep_planner.cpp):
// reads a plan dump written by the rewritten apps/astar_plan (path nodes: patch
// vertices, stance foot, yaw, surface id, centroid; start and goal), rebuilds the old
// nas::Node path from it - including each patch's old thin-prism polyhedron - and runs
// FootstepPlanner::plan on it. Prints whether the old QP finds a solution and writes
// its footsteps to <out.json>, so they can be compared with the new QP's.
//
// Usage: old_qp_check <plan.json> <out.json>
#include "footstep_planner.hpp"
#include "constants.hpp"
#include "geometry.hpp"
#include "node.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <vector>

using namespace nas;
using json = nlohmann::json;

static Point_3 pt(const json& j) { return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>()); }

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <plan.json> <out.json>\n";
        return 1;
    }
    json plan;
    std::ifstream(argv[1]) >> plan;
    if (!plan.value("path_found", false)) {
        std::cout << "no path in the dump: nothing to check\n";
        return 2;
    }
    std::vector<Node*> path;
    for (const auto& nj : plan["path"]) {
        Node* n = new Node();
        n->depth = nj["depth"];
        n->stance_foot = nj["stance_foot"];
        n->foot_yaw = nj["foot_yaw"];
        n->surface_id = nj["surface_id"];
        n->centroid = pt(nj["centroid"]);
        for (const auto& v : nj["patch_vertices"]) n->patch_vertices.push_back(pt(v));
        // the old code's patch polyhedron: a thin prism on the (horizontal) patch;
        // the root node is a point and keeps an empty one (never read by the QP).
        if (n->patch_vertices.size() >= 3) n->patch_polyhedron_3d = convex_hull_3_from_coplanar_points(n->patch_vertices, Vector_3(0, 0, 1));
        path.push_back(n);
    }
    FootstepPlanner planner;
    bool ok = planner.plan(current_stance_foot_flag, pt(plan["start"]), stance_foot_at_goal, pt(plan["goal"]), path);
    json out;
    out["old_qp_success"] = ok;
    json fs = json::array();
    for (const auto& p : planner.get_computed_footsteps()) fs.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
    out["footsteps"] = fs;
    std::ofstream(argv[2]) << out.dump(1);
    std::cout << "OLD_QP " << plan["scenario"].get<std::string>() << " " << (ok ? "FEASIBLE" : "INFEASIBLE") << " (" << fs.size() << " footsteps)\n";
    return ok ? 0 : 3;
}
