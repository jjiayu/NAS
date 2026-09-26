// Directed tests for expand_cube_pickup (docs/cube-pickup-spec.md): the geometric reverse of
// expand_cube_placement -- a cube already resting at a fixed, known scene position (no
// minkowski/tagged coupling needed, unlike placement: there is no second family of candidates to
// keep correlated once the cube's own position isn't itself an unknown -- see the spec's
// rationale), tested via the same foot_goal_satisfied() shape+yaw test AstarSearchConfig::
// foot_goals already uses. Same hand-built-Node pattern as cube_expansion_test.cpp: build a
// minimal parent, a negative control cloning it with one gating field flipped, call the function
// under test directly (no AstarSearch involved), assert on the children.
#include "nas/config/scenario.hpp"
#include "nas/core/geometry.hpp"
#include "nas/planners/astar_search.hpp"

#include <CGAL/convex_hull_3.h>
#include <cmath>
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

std::vector<Point_3> square(double cx, double cy, double cz, double half) {
    return {Point_3(cx - half, cy - half, cz), Point_3(cx + half, cy - half, cz),
            Point_3(cx + half, cy + half, cz), Point_3(cx - half, cy + half, cz)};
}

} // namespace

int run_cube_pickup() {
    config::Scenario sc = config::load_scenario("Flat");
    check(sc.surfaces.size() == 1, "Flat scenario is a single surface, as this test assumes");
    const Surface& flat = sc.surfaces[0];

    NodePool pool;

    // A real 10cm-square patch centered on (cx, cy, 0), on Flat's own surface frame -- unlike
    // cube_expansion_test.cpp's single-point patches (expand_cube_placement never reads
    // patch_polygon_2d), foot_goal_satisfied's point-shaped-target branch DOES test containment
    // against a real patch_polygon_2d, and its polytope-shaped branch needs a real patch plane.
    auto make_node = [&](double cx, double cy, StanceFoot foot, double yaw, Node* parent_ptr) {
        Node* n = pool.create();
        n->patch_vertices = square(cx, cy, 0.0, 0.05);
        std::vector<Point_2> p2d = transform_3d_points_to_surface_plane(n->patch_vertices, flat.transform_to_surface);
        n->patch_polygon_2d = Polygon_2(p2d.begin(), p2d.end());
        n->transformation_to_2d = flat.transform_to_surface;
        n->transformation_to_3d = flat.transform_to_3d;
        n->stance_foot = foot;
        n->surface_id = flat.surface_id;
        n->centroid = Point_3(cx, cy, 0.0);
        n->foot_yaw = yaw;
        n->depth = parent_ptr ? parent_ptr->depth + 1 : 0;
        n->cube_state = CubeState::None;
        n->parent = parent_ptr;
        return n;
    };

    // No-hull polyhedra table for point-shaped-only affordances (n cubes, both slots nullopt).
    auto no_hulls = [](size_t n) { return std::vector<std::array<std::optional<Polyhedron>, 2>>(n); };

    // --- (1) point affordance, mode 1 (right foot only), satisfied: the basic positive case ---
    {
        Node* parent = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(1.0, 0.0, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;

        auto children = expand_cube_pickup(parent, {cube}, no_hulls(1), pool);
        check(children.size() == 1, "(1) point affordance satisfied: exactly one child produced");
        if (children.size() == 1) {
            Node* c = children[0];
            check(c->cube_state == CubeState::InHand, "(1) child cube_state is InHand");
            check(!c->cube.has_value(), "(1) child carries no CubePlacement yet");
            check(c->cubes_picked_up.size() == 1 && c->cubes_picked_up[0], "(1) child marks cube 0 as picked up");
            check(c->stance_foot == parent->stance_foot, "(1) child keeps the parent's stance foot (no foot moved)");
            check(std::abs(CGAL::to_double(c->centroid.x() - parent->centroid.x())) < 1e-12 &&
                      std::abs(CGAL::to_double(c->centroid.y() - parent->centroid.y())) < 1e-12,
                  "(1) child keeps the parent's position (zero-displacement pseudo-action)");
            check(std::abs(c->foot_yaw - parent->foot_yaw) < 1e-12, "(1) child keeps the parent's foot_yaw");
            check(c->depth == parent->depth + 1, "(1) child is one depth level below the parent");
        }
    }

    // --- (2) negative controls, one gating field flipped at a time from the (1) setup ---
    {
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(1.0, 0.0, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;

        Node* wrong_foot = make_node(1.0, 0.0, StanceFoot::Left, 0.0, nullptr);
        check(expand_cube_pickup(wrong_foot, {cube}, no_hulls(1), pool).empty(),
              "(2) wrong foot (affordance is for Right, node is Left): no children");

        Node* outside = make_node(5.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        check(expand_cube_pickup(outside, {cube}, no_hulls(1), pool).empty(),
              "(2) node far outside the affordance region: no children");

        Node* already_taken = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        already_taken->cubes_picked_up = {true};
        check(expand_cube_pickup(already_taken, {cube}, no_hulls(1), pool).empty(),
              "(2) cube already picked up on this path: no children");

        Node* in_hand = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        in_hand->cube_state = CubeState::InHand;
        check(expand_cube_pickup(in_hand, {cube}, no_hulls(1), pool).empty(),
              "(2) cube_state == InHand (hands already full): no children");

        Node* placed_active = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        placed_active->cube_state = CubeState::PlacedActive;
        check(expand_cube_pickup(placed_active, {cube}, no_hulls(1), pool).empty(),
              "(2) cube_state == PlacedActive (a different cube already in play): no children");
    }

    // --- (3) cube_state == PlacedInactive is ALSO an eligible gate (hands free, sequential
    // pickup of a second scene cube), and an already-taken cube is skipped in favor of one that
    // isn't -- both cubes' affordances are satisfied by the same node on purpose, isolating the
    // "already taken" bookkeeping from the geometric test itself. ---
    {
        Node* parent = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        parent->cube_state = CubeState::PlacedInactive;
        parent->cubes_picked_up = {true, false}; // cube 0 already used earlier on this path

        AstarSearchConfig::SceneCube cube0, cube1;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(1.0, 0.0, 0.0);
        cube0.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;
        cube1.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;

        auto children = expand_cube_pickup(parent, {cube0, cube1}, no_hulls(2), pool);
        check(children.size() == 1, "(3) PlacedInactive + sequential pickup: exactly one child (cube 1 only)");
        if (children.size() == 1) {
            check(children[0]->cubes_picked_up.size() == 2 && children[0]->cubes_picked_up[0] && children[0]->cubes_picked_up[1],
                  "(3) child's cubes_picked_up is {taken, taken}: cube 0 preserved, cube 1 newly set");
        }
    }

    // --- (4) polytope affordance + yaw_range: satisfied inside the range, rejected outside it ---
    {
        std::vector<Point_3> verts = square(1.0, 0.0, 0.0, 0.1);
        Polyhedron hull;
        CGAL::convex_hull_3(verts.begin(), verts.end(), hull);

        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal g;
        g.region = verts;
        g.yaw_range = std::make_pair(-10.0 / 180.0 * M_PI, 10.0 / 180.0 * M_PI);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;

        std::vector<std::array<std::optional<Polyhedron>, 2>> hulls(1);
        hulls[0][static_cast<size_t>(StanceFoot::Right)] = hull;

        Node* in_range = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        check(expand_cube_pickup(in_range, {cube}, hulls, pool).size() == 1,
              "(4) polytope affordance, yaw inside range: satisfied");

        Node* out_of_range = make_node(1.0, 0.0, StanceFoot::Right, 45.0 / 180.0 * M_PI, nullptr);
        check(expand_cube_pickup(out_of_range, {cube}, hulls, pool).empty(),
              "(4) polytope affordance, yaw outside range: rejected");
    }

    // --- (5) mode 2 (both slots filled): symmetric stance tested on (node, node->parent) ---
    {
        AstarSearchConfig::SceneCube cube;
        AstarSearchConfig::FootGoal left_g, right_g;
        left_g.region = Point_3(1.0, 0.1, 0.0);
        right_g.region = Point_3(1.0, -0.1, 0.0);
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cube.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = right_g;

        Node* left_node = make_node(1.0, 0.1, StanceFoot::Left, 0.0, nullptr);
        Node* right_node = make_node(1.0, -0.1, StanceFoot::Right, 0.0, left_node);
        check(expand_cube_pickup(right_node, {cube}, no_hulls(1), pool).size() == 1,
              "(5) mode 2, both feet satisfy their own slot: triggers");

        Node* left_off = make_node(1.0, 0.9, StanceFoot::Left, 0.0, nullptr); // left slot NOT satisfied
        Node* right_only = make_node(1.0, -0.1, StanceFoot::Right, 0.0, left_off);
        check(expand_cube_pickup(right_only, {cube}, no_hulls(1), pool).empty(),
              "(5) mode 2, only one foot satisfies its slot: does not trigger");

        Node* no_parent = make_node(1.0, -0.1, StanceFoot::Right, 0.0, nullptr); // parent == nullptr
        check(expand_cube_pickup(no_parent, {cube}, no_hulls(1), pool).empty(),
              "(5) mode 2, node->parent == nullptr (only one foot ever placed): does not trigger");
    }

    // --- (6) two scene cubes satisfied at once from the same parent: two independent children,
    // and the parent's own cubes_picked_up is never mutated (the per-node-state requirement this
    // whole mechanism exists to satisfy -- each branch of the search must see its own copy). ---
    {
        Node* parent = make_node(1.0, 0.0, StanceFoot::Right, 0.0, nullptr);
        parent->cubes_picked_up = {false, false};

        AstarSearchConfig::SceneCube cube0, cube1;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(1.0, 0.0, 0.0);
        cube0.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;
        cube1.pickup_affordance[static_cast<size_t>(StanceFoot::Right)] = g;

        auto children = expand_cube_pickup(parent, {cube0, cube1}, no_hulls(2), pool);
        check(children.size() == 2, "(6) two independently-satisfied scene cubes: two children");
        if (children.size() == 2) {
            check(children[0]->cubes_picked_up == std::vector<bool>{true, false}, "(6) first child took cube 0 only");
            check(children[1]->cubes_picked_up == std::vector<bool>{false, true}, "(6) second child took cube 1 only");
        }
        check(parent->cubes_picked_up == std::vector<bool>{false, false},
              "(6) parent's own cubes_picked_up is untouched after producing both children");
    }

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All cube-pickup tests passed\n";
    return 0;
}
