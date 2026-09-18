#include "nas/core/node.hpp"

#include <iostream>
#include <string>
#include <vector>

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

void test_other_foot() {
    check(other_foot(StanceFoot::Left) == StanceFoot::Right, "other_foot(Left) == Right");
    check(other_foot(StanceFoot::Right) == StanceFoot::Left, "other_foot(Right) == Left");
}

void test_node_pool_pointer_stability() {
    NodePool pool;
    std::vector<Node*> handles;
    // Push well past any small deque chunk size to exercise growth.
    for (int i = 0; i < 5000; ++i) {
        Node* n = pool.create();
        n->node_id = i;
        handles.push_back(n);
    }
    check(pool.size() == 5000, "NodePool::size() reflects every create()");

    bool all_stable = true;
    for (int i = 0; i < 5000; ++i) {
        if (handles[static_cast<size_t>(i)]->node_id != i) {
            all_stable = false;
            break;
        }
    }
    check(all_stable, "NodePool: earlier Node* pointers stay valid and correct after many more create()s");
}

void test_node_defaults() {
    NodePool pool;
    Node* n = pool.create();
    check(n->surface_id == -1, "Node::surface_id defaults to -1 (was uninitialized in the old code)");
    check(n->stance_foot == StanceFoot::Left, "Node::stance_foot defaults to Left");
    check(n->parent == nullptr, "Node::parent defaults to nullptr");
}

void test_node_pool_assigns_sequential_ids() {
    NodePool pool;
    Node* a = pool.create();
    Node* b = pool.create();
    Node* c = pool.create();
    check(a->node_id == 0 && b->node_id == 1 && c->node_id == 2,
          "NodePool::create() assigns sequential node_id (0, 1, 2, ...)");
}

void test_contains_point() {
    NodePool pool;
    Node* n = pool.create();
    // A flat unit square on the ground plane, in world == surface frame
    // (identity transform), so points can be tested directly.
    std::vector<Point_2> square = {Point_2(-1, -1), Point_2(1, -1), Point_2(1, 1), Point_2(-1, 1)};
    n->patch_polygon_2d = Polygon_2(square.begin(), square.end());
    n->transformation_to_2d = Transformation(CGAL::IDENTITY);

    check(n->check_if_node_contains_point(Point_3(0, 0, 0)), "point at the origin is inside the unit square patch");
    check(!n->check_if_node_contains_point(Point_3(5, 5, 0)), "point far outside the patch is not contained");
}

void test_cycle_path_detection_no_parent() {
    check(!cycle_path_detection(nullptr, StanceFoot::Left, 0), "cycle_path_detection: null parent -> false");
}

void test_cycle_path_detection_empty_history() {
    NodePool pool;
    Node* parent = pool.create();
    check(!cycle_path_detection(parent, StanceFoot::Left, 0), "cycle_path_detection: empty history -> false");
}

void test_cycle_path_detection_revisit() {
    NodePool pool;
    Node* parent = pool.create();
    // Left foot was on surface 0, then surface 1 (left surface 0), now
    // revisiting surface 0 -> should be flagged as a cycle.
    parent->pred_surface_ids[static_cast<size_t>(StanceFoot::Left)] = {{0}, {1}};
    check(cycle_path_detection(parent, StanceFoot::Left, 0),
          "cycle_path_detection: revisiting a surface left earlier -> true");
}

void test_cycle_path_detection_no_revisit() {
    NodePool pool;
    Node* parent = pool.create();
    parent->pred_surface_ids[static_cast<size_t>(StanceFoot::Left)] = {{0}, {1}};
    check(!cycle_path_detection(parent, StanceFoot::Left, 2),
          "cycle_path_detection: a genuinely new surface -> false");
}

} // namespace

int main() {
    test_other_foot();
    test_node_pool_pointer_stability();
    test_node_defaults();
    test_node_pool_assigns_sequential_ids();
    test_contains_point();
    test_cycle_path_detection_no_parent();
    test_cycle_path_detection_empty_history();
    test_cycle_path_detection_revisit();
    test_cycle_path_detection_no_revisit();

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All core/node tests passed\n";
    return 0;
}
