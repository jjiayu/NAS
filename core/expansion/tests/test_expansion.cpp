// Directed tests for core/expansion — exercises expand_node against the
// real talosReachability Forward assets (same ones CASSR uses), since
// that's the direction we can actually validate today (see PLAN.md: the
// Antecedent/NAS assets are broken/ambiguous). A large flat surface is
// used so whatever the polytope's actual reach is, it lands inside it —
// this is a mechanics test, not a scenario-replication test (that's
// phase 5's job once planners/astar_search exists).

#include "nas/core/expansion.hpp"

#include <iostream>
#include <string>

using namespace nas;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::cerr << "FAIL: " << what << "\n";
        ++g_failures;
    } else {
        std::cout << "ok: " << what << "\n";
    }
}

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

ReachabilityModel make_forward_model() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    std::vector<ReachabilityEntry> entries = {
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

std::vector<Surface> make_single_flat_surface() {
    // A large (10x10 m) flat ground surface centered at the origin —
    // big enough that any real reach distance lands inside it.
    std::vector<Point_3> square = {
        Point_3(-5, -5, 0), Point_3(5, -5, 0), Point_3(5, 5, 0), Point_3(-5, 5, 0)
    };
    std::vector<Surface> surfaces;
    surfaces.emplace_back(square, /*surface_idx=*/0, /*foot_length=*/0.2, /*foot_width=*/0.12);
    return surfaces;
}

Node* make_start_node(NodePool& pool) {
    Node* start = pool.create();
    start->patch_vertices = {Point_3(0, 0, 0)};
    start->centroid = Point_3(0, 0, 0);
    start->stance_foot = StanceFoot::Left;
    start->surface_id = 0;
    start->depth = 0;
    return start;
}

void test_basic_expansion_forward_no_rotation() {
    ReachabilityModel model = make_forward_model();
    std::vector<Surface> surfaces = make_single_flat_surface();
    NodePool pool;
    Node* start = make_start_node(pool);

    ExpansionParams params;
    params.rotation_enabled = false;

    std::vector<Node*> children = expand_node(start, surfaces, model, ReachabilityDirection::Forward, params, pool);

    check(!children.empty(), "expand_node (forward, no rotation) finds at least one child on a large flat surface");
    if (!children.empty()) {
        check(children[0]->stance_foot == StanceFoot::Right, "child alternates stance foot (Left parent -> Right child)");
        check(children[0]->depth == 1, "child depth is parent depth + 1");
        check(children[0]->surface_id == 0, "child surface_id matches the surface it landed on");
        check(children[0]->foot_yaw == 0.0, "child foot_yaw is 0 when rotation is disabled");
        check(children.size() == 1, "exactly one child per surface when rotation is disabled (no fan-out)");
    }
}

void test_rotation_fans_out_children() {
    ReachabilityModel model = make_forward_model();
    std::vector<Surface> surfaces = make_single_flat_surface();
    NodePool pool;
    Node* start = make_start_node(pool);

    ExpansionParams params;
    params.rotation_enabled = true;
    params.yaw_discretization_num = 1; // -> 3 candidate yaws: -inc, 0, +inc

    std::vector<Node*> children = expand_node(start, surfaces, model, ReachabilityDirection::Forward, params, pool);

    check(children.size() == 3,
          "expand_node (rotation enabled, discretization=1) fans out into 2*1+1 = 3 children on one surface");
}

void test_cycle_detection_blocks_revisit() {
    ReachabilityModel model = make_forward_model();
    std::vector<Surface> surfaces = make_single_flat_surface();

    NodePool pool;
    Node* start = make_start_node(pool);
    // Pretend the left foot already visited surface 0 and left it, so
    // revisiting surface 0 with the left foot should be flagged.
    start->pred_surface_ids[static_cast<size_t>(StanceFoot::Left)] = {{0}, {1}};
    // The child about to be created here is the RIGHT foot though (parent
    // is Left, expand alternates) — cycle detection checks the CHILD's
    // stance foot history, which is empty, so nothing should be blocked
    // in this exact setup. To actually exercise the block, put the
    // history on the foot that will become the child's stance foot.
    // Child stance foot = other_foot(parent->stance_foot) = Right here,
    // so seed the Right-foot history instead:
    start->pred_surface_ids[static_cast<size_t>(StanceFoot::Right)] = {{0}, {1}};

    ExpansionParams params_with_detection;
    params_with_detection.rotation_enabled = false;
    params_with_detection.cycle_detection_enabled = true;
    std::vector<Node*> blocked = expand_node(start, surfaces, model, ReachabilityDirection::Forward, params_with_detection, pool);
    check(blocked.empty(), "cycle detection enabled: revisiting surface 0 with the same (child) foot is blocked");

    ExpansionParams params_without_detection;
    params_without_detection.rotation_enabled = false;
    params_without_detection.cycle_detection_enabled = false;
    std::vector<Node*> allowed = expand_node(start, surfaces, model, ReachabilityDirection::Forward, params_without_detection, pool);
    check(!allowed.empty(), "cycle detection disabled: the same surface is reachable again");
}

} // namespace

int main() {
    test_basic_expansion_forward_no_rotation();
    test_rotation_fans_out_children();
    test_cycle_detection_blocks_revisit();

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/expansion tests passed\n";
    return 0;
}
