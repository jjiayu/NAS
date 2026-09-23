// Is a plan produced by the rewritten CASSR (+ its QP) valid for the OLD A*?
// Built in a checkout of the tag cassr-stage-a-validated, whose expand_node still has
// legacy_clip and legacy_node_keys: bit-identical to the old get_children (proved by the
// old-binary replay, tag legacy-replay-verified). For a plan dump written by apps/astar_plan:
//   1. the path must exist in the OLD expansion graph: from the start node, at each step the
//      old expansion must produce a child with the plan's surface AND yaw (the chain continues
//      from the OLD child, so its patches are the old ones);
//   2. each footstep the QP returned must lie inside the OLD child's patch (on its plane,
//      inside its polygon, tolerance 1e-6 m).
// Usage: nas_check_old_astar <scene> <plan.json>
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"
#include "nas/core/geometry.hpp"
#include "nas/core/reachability.hpp"

#include <CGAL/squared_distance_2.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

static Point_3 pt(const json& j) { return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>()); }

int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "Usage: " << argv[0] << " <scene> <plan.json>\n"; return 1; }
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario(argv[1]);
    json plan; std::ifstream(argv[2]) >> plan;
    if (!plan.value("path_found", false) || !plan.value("qp_success", false)) { std::cout << argv[1] << ": no plan to check\n"; return 2; }

    ExpansionParams params;
    params.rotation_enabled = true; params.yaw_discretization_num = 3; params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    params.cycle_detection_enabled = true;
    params.legacy_clip = true; params.legacy_node_keys = true;   // the old code's expansion

    NodePool pool;
    Node* cur = pool.create();
    cur->patch_vertices = {pt(plan["start"])};
    cur->stance_foot = static_cast<StanceFoot>(plan["path"][0]["stance_foot"].get<int>());
    cur->foot_yaw = plan["path"][0]["foot_yaw"];
    cur->surface_id = -1; cur->depth = 0;

    const auto& path = plan["path"]; const auto& feet = plan["footsteps"];
    size_t n = path.size();
    int missing = 0, outside = 0;
    double worst = 0.0;
    for (size_t i = 1; i < n; ++i) {
        std::vector<Node*> kids = expand_node(cur, sc.surfaces, reach, ReachabilityDirection::Forward, params, pool);
        Node* match = nullptr;
        for (Node* k : kids)
            if (k->surface_id == path[i]["surface_id"].get<int>() && std::abs(k->foot_yaw - path[i]["foot_yaw"].get<double>()) < 1e-6) { match = k; break; }
        if (!match) {
            ++missing;
            std::cout << "  step " << i << ": the old expansion has NO child on surface " << path[i]["surface_id"] << " with yaw " << path[i]["foot_yaw"] << "\n";
            break; // cannot continue the chain
        }
        if (i + 1 < n) { // intermediate footsteps must be on their patch (the last one is the goal, not on a patch)
            Point_3 f = pt(feet[i]["position"]);
            std::vector<Point_2> p2 = transform_3d_points_to_surface_plane({f}, match->transformation_to_2d);
            const Polygon_2& poly = match->patch_polygon_2d;
            double d = 0.0;
            if (poly.bounded_side(p2[0]) == CGAL::ON_UNBOUNDED_SIDE) {
                d = 1e300;
                for (size_t k = 0; k < poly.size(); ++k)
                    d = std::min(d, std::sqrt(CGAL::to_double(CGAL::squared_distance(p2[0], Segment_2(poly.vertex(k), poly.vertex((k + 1) % poly.size()))))));
            }
            worst = std::max(worst, d);
            if (d > 1e-6) { ++outside; std::cout << "  step " << i << ": footstep is " << d * 1000 << " mm outside the OLD patch\n"; }
        }
        cur = match;
    }
    bool ok = missing == 0 && outside == 0;
    std::cout << argv[1] << ": " << (ok ? "VALID for the old A*" : "NOT valid for the old A*") << " (" << n - 1 << " steps, missing children " << missing
              << ", footsteps outside the old patch " << outside << ", worst " << worst << " m)\n";
    return ok ? 0 : 3;
}
