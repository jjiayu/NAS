// old_expansion_dump.cpp — differential-testing ground truth for the
// rewrite's core/expansion (see PLAN.md, "tests/golden_all").
//
// The stored golden files only record a search's *final path*, and re-running
// the old binary today doesn't even reproduce their centroids (see
// docs/paper-deltas.md), so they can't prove the rewrite computes the same
// intersections. This tool calls the OLD AstarSearch::get_children directly
// on real parent states (breadth-first from the start node, no dedup, no
// heuristic — pure expansion) and dumps, per expansion, the exact parent
// state and every child: surface, yaw, stance, depth, perimeter, centroid,
// raw patch_vertices (in the old code's order, duplicates included) and the
// predecessor-surface history. A companion test in the rewrite
// (tests/golden_all/test_expansion_differential.cpp) replays each parent
// through expand_node and compares exactly.
//
// Any scene of environments.hpp can be requested at runtime: the scene the
// binary was compiled with (constants.hpp) is only used by the AstarSearch
// constructor; `surfaces` is public, so it is overwritten here.
//
// Usage: old_expansion_dump <scene> <start_x> <start_y> <start_z> <max_expansions> <out.json>

#include "astar_search.hpp"
#include "constants.hpp"
#include "geometry.hpp"
#include "environments.hpp"
#include "node.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

using namespace nas;
using json = nlohmann::json;

namespace {

json pt(const Point_3& p) {
    return json::array({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
}

json node_json(const Node* n) {
    json j;
    j["depth"] = n->depth;
    j["stance_foot"] = n->stance_foot;
    j["foot_yaw"] = n->foot_yaw;
    j["surface_id"] = n->surface_id;
    j["perimeter"] = n->perimeter;
    j["centroid"] = pt(n->centroid);
    json verts = json::array();
    for (const auto& v : n->patch_vertices) verts.push_back(pt(v));
    j["patch_vertices"] = verts;
    j["pred_surface_ids"] = n->pred_surface_ids;
    return j;
}

const std::map<std::string, const std::vector<std::vector<Point_3>>*>& scenes() {
    static const std::map<std::string, const std::vector<std::vector<Point_3>>*> m = {
        {"Stairs", &Stairs}, {"TwoFlatSurfaces", &TwoFlatSurfaces}, {"Flat", &Flat},
        {"LongStairs", &LongStairs}, {"LongLongStairs", &LongLongStairs},
        {"LongStairsComplete", &LongStairsComplete}, {"LongStairsExp", &LongStairsExp},
        {"ThreePathsScene", &ThreePathsScene}, {"Stairs_Up_Down", &Stairs_Up_Down},
        {"ThreePathsNAS", &ThreePathsNAS}, {"NarrowPassage", &NarrowPassage},
    };
    return m;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::cerr << "Usage: " << argv[0] << " <scene> <start_x> <start_y> <start_z> <max_expansions> <out.json>\n";
        return 1;
    }
    // Heap-layout probe: NAS_PAD_ALLOC=<bytes> makes a leaked allocation before
    // anything else, shifting later addresses. If the old code's results change
    // with it, something in the pipeline depends on pointer order.
    if (const char* pad = std::getenv("NAS_PAD_ALLOC")) {
        static volatile char* leak = static_cast<char*>(std::malloc(std::stoul(pad)));
        (void)leak;
    }
    // Heap-order probe: NAS_SCRAMBLE=<seed> allocates many blocks and frees
    // them in a shuffled order, so later allocations come back in a different
    // *relative* address order (a constant shift like NAS_PAD_ALLOC cannot
    // change relative order). If the old code's results depend on it, some
    // step (e.g. a pointer-keyed container inside CGAL's hull) is
    // address-order dependent.
    if (const char* seed = std::getenv("NAS_SCRAMBLE")) {
        std::srand(std::stoul(seed));
        std::vector<void*> blocks;
        for (int i = 0; i < 4000; ++i) blocks.push_back(std::malloc(16 + std::rand() % 1500));
        for (size_t i = blocks.size(); i > 1; --i) std::swap(blocks[i - 1], blocks[std::rand() % i]);
        for (size_t i = 0; i < blocks.size(); i += 2) std::free(blocks[i]);
        // keep the other half allocated (leaked) so the free list stays fragmented
    }
    std::string scene = argv[1];
    auto it = scenes().find(scene);
    if (it == scenes().end()) {
        std::cerr << "unknown scene " << scene << "\n";
        return 1;
    }
    Point_3 start(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
    int max_expansions = std::stoi(argv[5]);

    AstarSearch search; // loads the forward polytopes; scene below overrides surfaces
    search.surfaces.clear();
    for (size_t i = 0; i < it->second->size(); ++i) {
        int idx = static_cast<int>(i);
        search.surfaces.push_back(Surface((*it->second)[i], idx));
    }

    // Same start node as AstarSearch's constructor (surface_id -1 instead of
    // its uninitialized value: only affects cycle detection when read, and
    // pred_surface_ids for the root is empty anyway).
    Node* root = new Node();
    root->patch_vertices = {start};
    root->stance_foot = current_stance_foot_flag;
    root->centroid = start;
    root->depth = 0;
    root->foot_yaw = foot_yaw_rotation_flag ? current_foot_yaw : 0.0;
    root->perimeter = 0.0;
    root->surface_id = -1;

    json out;
    out["scene"] = scene;
    out["expansions"] = json::array();

    std::deque<Node*> queue{root};
    int expanded = 0;
    while (!queue.empty() && expanded < max_expansions) {
        Node* parent = queue.front();
        queue.pop_front();
        std::vector<Node*> children = search.get_children(parent);
        json e;
        e["parent"] = node_json(parent);
        if (expanded < 15) {
            // Intermediate stages, computed with the OLD functions on the very
            // same inputs get_children just used, to localise any divergence.
            Polyhedron base = parent->stance_foot == 0 ? search.rf_in_lf_polytope : search.lf_in_rf_polytope;
            if (foot_yaw_rotation_flag) base = rotate_polyhedron_z(base, parent->foot_yaw);
            json rot = json::array();
            for (auto v = base.vertices_begin(); v != base.vertices_end(); ++v) rot.push_back(pt(v->point()));
            Polyhedron P = minkowski_sum(parent->patch_vertices, base);
            json pu = json::array();
            for (auto v = P.vertices_begin(); v != P.vertices_end(); ++v) pu.push_back(pt(v->point()));
            json plane_coeffs = json::array();
            for (const auto& surf : search.surfaces) {
                plane_coeffs.push_back(json::array({CGAL::to_double(surf.plane.a()), CGAL::to_double(surf.plane.b()),
                                                    CGAL::to_double(surf.plane.c()), CGAL::to_double(surf.plane.d())}));
            }
            json planes = json::array();
            for (const auto& surf : search.surfaces) {
                json pl = json::array();
                for (const auto& q : compute_polytope_plane_intersection(surf.plane, P)) pl.push_back(pt(q));
                planes.push_back(pl);
            }
            e["stages"] = {{"rotated_polytope", rot}, {"p_union", pu}, {"plane_intersections", planes}, {"plane_coeffs", plane_coeffs}};
        }
        json cj = json::array();
        for (Node* c : children) {
            cj.push_back(node_json(c));
            queue.push_back(c);
        }
        e["children"] = cj;
        out["expansions"].push_back(e);
        ++expanded;
    }

    std::ofstream(argv[6]) << out.dump(1);
    std::cout << "wrote " << argv[6] << ": " << expanded << " expansions from scene " << scene << "\n";
    return 0;
}
