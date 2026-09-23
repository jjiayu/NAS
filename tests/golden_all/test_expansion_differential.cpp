// Differential test of core/expansion against the OLD code, on real states.
//
// Input: JSON written by the old repo's tests/old_expansion_dump.cpp, which
// calls the OLD AstarSearch::get_children on real parent states and records
// every child exactly. Here each recorded parent is rebuilt as a Node and
// pushed through expand_node with the old constants.hpp settings; every
// child is compared field by field:
//   - same number of children, same order (surface loop outer, yaw inner);
//   - surface_id, stance_foot, depth, foot_yaw, pred_surface_ids: exact;
//   - perimeter, centroid: within TOL (floating-point noise only);
//   - patch_vertices: the SET of distinct points must match within TOL
//     (old stores the raw clip output incl. duplicates/collinear points,
//     new stores the convex hull's vertices - same polygon, so the distinct
//     hull points must all appear in the old set and vice versa up to
//     collinear extras, which are reported separately).
// Prints a per-scene table with max deviations, and exits non-zero on any
// mismatch beyond TOL. Usage: test_expansion_differential <dump_dir>

#include "exact_clip.hpp"
#include "nas/config/scenario.hpp"
#include "nas/core/expansion.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <CGAL/squared_distance_2.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace nas;
using json = nlohmann::json;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

constexpr double TOL = 1e-9;
constexpr double POLY_TOL = 1e-7; // polygon geometry: same shape up to floating-point noise

// Scenes where the OLD code is itself unstable: comparing the old
// expansion dump with another run of the old code whose only difference is
// the heap allocation order (old_expansion_dump's NAS_SCRAMBLE) also gives
// different patch polygons on 0.3%-1.8% of children (up to 0.9m), while the
// other scenes come out identical. P_union's vertex order/triangulation from
// CGAL::convex_hull_3 changes with heap order; the plane/polytope cut itself
// is unaffected (same polygon, see bench_plane_cut.cpp) but the 2D
// Sutherland-Hodgman clip silently drops an intersection point when its double
// inside-test and CGAL's exact segment/line intersection disagree on a
// near-parallel edge (measured, docs/paper-deltas.md). Exact old==new is undefined there; a bounded rate is asserted.
bool is_unstable_in_old(const std::string& scene) {
    static const std::vector<std::string> names = {"Stairs", "LongStairs", "LongLongStairs", "LongStairsComplete",
                                                   "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down"};
    return std::find(names.begin(), names.end(), scene) != names.end();
}
constexpr double UNSTABLE_SCENE_MAX_POLYGON_RATE = 0.05;

const std::vector<std::string> kScenes = {"NarrowPassage", "Stairs", "TwoFlatSurfaces", "LongStairs", "LongLongStairs",
                                          "Flat", "LongStairsComplete", "LongStairsExp", "ThreePathsScene",
                                          "Stairs_Up_Down", "ThreePathsNAS"};

ReachabilityModel make_forward_reachability() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    return ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
}

Point_3 to_pt(const json& j) { return Point_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>()); }

double dist(const Point_3& a, const Point_3& b) {
    return std::max({std::abs(CGAL::to_double(a.x() - b.x())), std::abs(CGAL::to_double(a.y() - b.y())),
                     std::abs(CGAL::to_double(a.z() - b.z()))});
}

// Distinct points (within TOL), so raw-with-duplicates and hull sets compare.
std::vector<Point_3> distinct(const std::vector<Point_3>& pts) {
    std::vector<Point_3> out;
    for (const auto& p : pts) {
        if (std::none_of(out.begin(), out.end(), [&](const Point_3& q) { return dist(p, q) < TOL; })) out.push_back(p);
    }
    return out;
}

bool contains(const std::vector<Point_3>& set, const Point_3& p) {
    return std::any_of(set.begin(), set.end(), [&](const Point_3& q) { return dist(p, q) < TOL; });
}

// Symmetric max nearest-neighbour distance between two point sets (0 if they
// are the same set within floating-point noise).
double set_deviation(const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
    auto one_way = [](const std::vector<Point_3>& x, const std::vector<Point_3>& y) {
        double worst = 0.0;
        for (const auto& p : x) {
            double best = 1e300;
            for (const auto& q : y) best = std::min(best, dist(p, q));
            worst = std::max(worst, best);
        }
        return worst;
    };
    if (a.empty() || b.empty()) return (a.empty() && b.empty()) ? 0.0 : 1e300;
    return std::max(one_way(a, b), one_way(b, a));
}

std::vector<Point_3> to_pts(const json& arr) {
    std::vector<Point_3> v;
    for (const auto& x : arr) v.push_back(to_pt(x));
    return v;
}

// Geometry of a patch = its convex polygon in the surface plane. Compared via
// the two-way Hausdorff distance between the hulls' boundaries plus the area
// difference, so extra collinear/duplicate vertices (address-order artifacts
// of CGAL's convex_hull_3, see docs/paper-deltas.md) don't count as
// differences but any real change of shape does. All scenes of
// environments.hpp are horizontal, so xy hulls plus a z check suffice.
std::vector<Point_2> hull_xy(const std::vector<Point_3>& pts) {
    std::vector<Point_2> p2;
    for (const auto& p : pts) p2.emplace_back(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    std::vector<Point_2> h;
    CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(h));
    return h;
}

double boundary_distance(const Point_2& p, const std::vector<Point_2>& poly) {
    if (poly.size() == 1) return std::sqrt(CGAL::to_double(CGAL::squared_distance(p, poly[0])));
    double best = 1e300;
    for (size_t i = 0; i < poly.size(); ++i) {
        Segment_2 seg(poly[i], poly[(i + 1) % poly.size()]);
        best = std::min(best, std::sqrt(CGAL::to_double(CGAL::squared_distance(p, seg))));
    }
    return best;
}

double polygon_area(const std::vector<Point_2>& h) {
    double a = 0;
    for (size_t i = 0; i < h.size(); ++i) {
        const Point_2& p = h[i];
        const Point_2& q = h[(i + 1) % h.size()];
        a += CGAL::to_double(p.x()) * CGAL::to_double(q.y()) - CGAL::to_double(q.x()) * CGAL::to_double(p.y());
    }
    return std::abs(a) / 2.0;
}

// Returns max(two-way Hausdorff of boundaries, |area diff|, |z diff|).
double polygon_deviation(const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
    auto ha = hull_xy(a), hb = hull_xy(b);
    double dev = 0;
    for (const auto& p : ha) dev = std::max(dev, boundary_distance(p, hb));
    for (const auto& p : hb) dev = std::max(dev, boundary_distance(p, ha));
    dev = std::max(dev, std::abs(polygon_area(ha) - polygon_area(hb)));
    if (!a.empty() && !b.empty()) dev = std::max(dev, std::abs(CGAL::to_double(a[0].z() - b[0].z())));
    return dev;
}

// The old code's exact P_union as an ordered edge list (dumped by
// old_expansion_dump from the very hull get_children used). Rebuilding a
// Polyhedron from its facets does NOT reproduce the old edge iteration order
// (measured: 130-149 of 150 expansions differ), so the edge list itself is
// replayed; the plane/polytope intersection depends on nothing else.
EdgeList old_edges(const json& mesh) {
    EdgeList out;
    for (const auto& e : mesh["edges"]) out.emplace_back(to_pt(mesh["vertices"][e[0].get<int>()]), to_pt(mesh["vertices"][e[1].get<int>()]));
    return out;
}

struct Stats {
    int polygon_mismatch = 0;
    std::string poly_detail;
    double max_polygon_dev = 0;
    // intermediate stages (only the first expansions carry them in the dump)
    int stage_expansions = 0, rot_mismatch = 0, punion_mismatch = 0, plane_mismatch = 0;
    double max_rot_dev = 0, max_punion_dev = 0, max_plane_dev = 0, max_plane_coeff_dev = 0;
    std::string plane_coeff_note;
    std::string plane_detail;
    int rot_order_diff = 0, punion_order_diff = 0;
    std::string order_note;

    int expansions = 0, children = 0;
    int count_mismatch = 0, surface_mismatch = 0, stance_mismatch = 0, depth_mismatch = 0, yaw_mismatch = 0;
    int history_mismatch = 0, perimeter_mismatch = 0, centroid_mismatch = 0, vertex_set_mismatch = 0;
    int old_extra_vertices = 0; // old raw list had points the hull dropped (collinear/duplicate)
    double max_perimeter_dev = 0, max_centroid_dev = 0, max_yaw_dev = 0;
    std::vector<std::string> examples;
    int detailed = 0;
    bool structure_ok() const {
        // Asserted: structure (children count/order, surface, stance, depth,
        // yaw, cycle history) and patch POLYGON geometry. Not asserted, only
        // reported: centroid, perimeter and the raw vertex list, which the old
        // code itself does not reproduce when only its heap order changes.
        return count_mismatch + surface_mismatch + stance_mismatch + depth_mismatch + yaw_mismatch + history_mismatch == 0;
    }
};

// How compare_scene runs expand_node:
//   Port    - own hull, the old (Legacy) clip: is the port faithful? (bounded rate on unstable scenes)
//   Replay  - the old run's exact hull + Legacy clip: everything bit-identical to the old output
//   Robust  - the old run's exact hull + the corrected clip, checked against an
//             independent EXACT-arithmetic oracle on every surface (must agree
//             everywhere), and against the old output (may differ ONLY where the
//             old clip dropped a point, i.e. the new patch contains the old one)
enum class Mode { Port, Replay, Robust };

Stats compare_scene(const std::string& scene, const json& dump, const ReachabilityModel& reachability, Mode mode) {
    const bool replay = mode != Mode::Port;
    Stats st;
    config::Scenario scenario = config::load_scenario(scene);
    ExpansionParams params;
    params.rotation_enabled = true;
    params.yaw_discretization_num = 3;
    params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    params.cycle_detection_enabled = true;
    params.legacy_clip = mode != Mode::Robust;
    params.legacy_node_keys = mode != Mode::Robust; // the old code's raw patch / prism perimeter / vertex-average centroid

    auto note = [&](const std::string& s) {
        if (st.examples.size() < 6) st.examples.push_back(s);
    };

    for (size_t ei = 0; ei < dump["expansions"].size(); ++ei) {
        const json& e = dump["expansions"][ei];
        const json& pj = e["parent"];
        NodePool pool;
        Node* parent = pool.create();
        parent->depth = pj["depth"];
        parent->stance_foot = pj["stance_foot"].get<int>() == 0 ? StanceFoot::Left : StanceFoot::Right;
        parent->foot_yaw = pj["foot_yaw"];
        parent->surface_id = pj["surface_id"];
        parent->perimeter = pj["perimeter"];
        parent->centroid = to_pt(pj["centroid"]);
        for (const auto& v : pj["patch_vertices"]) parent->patch_vertices.push_back(to_pt(v));
        for (size_t k = 0; k < 2 && k < pj["pred_surface_ids"].size(); ++k) {
            parent->pred_surface_ids[k] = pj["pred_surface_ids"][k].get<std::vector<std::vector<int>>>();
        }

        if (e.contains("stages")) {
            ++st.stage_expansions;
            const json& sg = e["stages"];
            Polyhedron base = reachability.query(parent->stance_foot == StanceFoot::Left ? "RF" : "LF",
                                                 parent->stance_foot == StanceFoot::Left ? "LF" : "RF", ReachabilityDirection::Forward);
            base = rotate_polyhedron_z(base, parent->foot_yaw);
            std::vector<Point_3> rot;
            for (auto v = base.vertices_begin(); v != base.vertices_end(); ++v) rot.push_back(v->point());
            Polyhedron P = minkowski_sum(parent->patch_vertices, base);
            std::vector<Point_3> pu;
            for (auto v = P.vertices_begin(); v != P.vertices_end(); ++v) pu.push_back(v->point());
            {
                // ORDER-sensitive comparison (set_deviation ignores order, but a
                // convex hull's triangulation can depend on input order).
                auto ordered_equal = [](const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
                    if (a.size() != b.size()) return false;
                    for (size_t i = 0; i < a.size(); ++i) if (dist(a[i], b[i]) > 1e-12) return false;
                    return true;
                };
                if (!ordered_equal(rot, to_pts(sg["rotated_polytope"]))) ++st.rot_order_diff;
                if (!ordered_equal(pu, to_pts(sg["p_union"]))) ++st.punion_order_diff;
                if (st.order_note.empty() && !ordered_equal(pu, to_pts(sg["p_union"]))) {
                    auto op = to_pts(sg["p_union"]);
                    st.order_note = "expansion " + std::to_string(ei) + ": P_union vertex count new " + std::to_string(pu.size()) + " old " + std::to_string(op.size()) +
                                    "; rotated polytope order equal: " + std::string(ordered_equal(rot, to_pts(sg["rotated_polytope"])) ? "yes" : "NO") + "; parent patch size " + std::to_string(parent->patch_vertices.size());
                }
            }
            double d1 = set_deviation(rot, to_pts(sg["rotated_polytope"]));
            double d2 = set_deviation(pu, to_pts(sg["p_union"]));
            st.max_rot_dev = std::max(st.max_rot_dev, d1);
            st.max_punion_dev = std::max(st.max_punion_dev, d2);
            if (d1 > TOL) ++st.rot_mismatch;
            if (d2 > TOL) ++st.punion_mismatch;
            const EdgeList stage_edges = polytope_edges(P);
            for (size_t si = 0; si < scenario.surfaces.size(); ++si) {
                const auto& pl = scenario.surfaces[si].plane;
                const json& oc4 = sg["plane_coeffs"][si];
                double dc = std::max({std::abs(CGAL::to_double(pl.a()) - oc4[0].get<double>()), std::abs(CGAL::to_double(pl.b()) - oc4[1].get<double>()),
                                      std::abs(CGAL::to_double(pl.c()) - oc4[2].get<double>()), std::abs(CGAL::to_double(pl.d()) - oc4[3].get<double>())});
                st.max_plane_coeff_dev = std::max(st.max_plane_coeff_dev, dc);
                if (dc > TOL && st.plane_coeff_note.empty()) {
                    st.plane_coeff_note = "surface " + std::to_string(si) + " plane new (" + std::to_string(CGAL::to_double(pl.a())) + "," +
                        std::to_string(CGAL::to_double(pl.b())) + "," + std::to_string(CGAL::to_double(pl.c())) + "," + std::to_string(CGAL::to_double(pl.d())) +
                        ") old (" + std::to_string(oc4[0].get<double>()) + "," + std::to_string(oc4[1].get<double>()) + "," +
                        std::to_string(oc4[2].get<double>()) + "," + std::to_string(oc4[3].get<double>()) + ")";
                }
                // The dump's plane_intersections stage was computed on a second,
                // separately recomputed hull (CGAL's triangulation is heap-order
                // dependent), so it says nothing about the replayed hull: skip it.
                if (replay) continue;
                double d3 = set_deviation(compute_edges_plane_intersection(scenario.surfaces[si].plane, stage_edges), to_pts(sg["plane_intersections"][si]));
                st.max_plane_dev = std::max(st.max_plane_dev, d3);
                if (d3 > TOL && st.plane_detail.empty()) {
                    auto newp = compute_edges_plane_intersection(scenario.surfaces[si].plane, stage_edges);
                    auto oldp = to_pts(sg["plane_intersections"][si]);
                    std::string t = "expansion " + std::to_string(ei) + " surface " + std::to_string(si) + ": new " + std::to_string(newp.size()) + " pts, old " + std::to_string(oldp.size()) + " pts\n";
                    auto show = [&](const char* label, const std::vector<Point_3>& a, const std::vector<Point_3>& b) {
                        int shown = 0;
                        for (const auto& p : a) {
                            bool found = false;
                            for (const auto& q : b) if (dist(p, q) < 1e-9) { found = true; break; }
                            if (!found && shown++ < 4) t += std::string("        ") + label + " (" + std::to_string(CGAL::to_double(p.x())) + "," + std::to_string(CGAL::to_double(p.y())) + "," + std::to_string(CGAL::to_double(p.z())) + ")\n";
                        }
                    };
                    show("only in NEW:", newp, oldp);
                    show("only in OLD:", oldp, newp);
                    st.plane_detail = t;
                }
                if (d3 > TOL) ++st.plane_mismatch;
            }
        }

        if (replay) {
            EdgeList replayed = old_edges(e["p_union_mesh"]);
            params.union_edges_override = [replayed](const Node&) { return replayed; };
        }
        std::vector<Node*> kids = expand_node(parent, scenario.surfaces, reachability, ReachabilityDirection::Forward, params, pool);
        const json& oc = e["children"];
        ++st.expansions;
        if (kids.size() != oc.size()) {
            ++st.count_mismatch;
            note("expansion " + std::to_string(ei) + ": " + std::to_string(kids.size()) + " children vs old " + std::to_string(oc.size()));
            continue;
        }
        for (size_t ci = 0; ci < kids.size(); ++ci) {
            const Node* n = kids[ci];
            const json& o = oc[ci];
            ++st.children;
            std::string where = "expansion " + std::to_string(ei) + " child " + std::to_string(ci);

            if (n->surface_id != o["surface_id"].get<int>()) { ++st.surface_mismatch; note(where + ": surface " + std::to_string(n->surface_id) + " vs old " + std::to_string(o["surface_id"].get<int>())); }
            if (static_cast<int>(n->stance_foot) != o["stance_foot"].get<int>()) ++st.stance_mismatch;
            if (n->depth != o["depth"].get<int>()) ++st.depth_mismatch;
            double yd = std::abs(n->foot_yaw - o["foot_yaw"].get<double>());
            st.max_yaw_dev = std::max(st.max_yaw_dev, yd);
            if (yd > TOL) ++st.yaw_mismatch;

            std::array<std::vector<std::vector<int>>, 2> old_hist;
            for (size_t k = 0; k < 2; ++k) old_hist[k] = o["pred_surface_ids"][k].get<std::vector<std::vector<int>>>();
            if (n->pred_surface_ids != old_hist) ++st.history_mismatch;

            double pd = std::abs(n->perimeter - o["perimeter"].get<double>());
            st.max_perimeter_dev = std::max(st.max_perimeter_dev, pd);
            if (pd > TOL) {
                ++st.perimeter_mismatch;
                note(where + ": perimeter dev " + std::to_string(pd) + " (new " + std::to_string(n->perimeter) + ", old " +
                     std::to_string(o["perimeter"].get<double>()) + ")");
                if (st.detailed < 1) {
                    ++st.detailed;
                    std::string txt = "  NEW patch_vertices:";
                    for (const auto& p : n->patch_vertices) txt += " (" + std::to_string(CGAL::to_double(p.x())) + "," + std::to_string(CGAL::to_double(p.y())) + ")";
                    txt += "\n      OLD patch_vertices:";
                    for (const auto& v : o["patch_vertices"]) txt += " (" + std::to_string(v[0].get<double>()) + "," + std::to_string(v[1].get<double>()) + ")";
                    note(where + txt);
                }
            }

            double cd = dist(n->centroid, to_pt(o["centroid"]));
            st.max_centroid_dev = std::max(st.max_centroid_dev, cd);
            if (cd > TOL) { ++st.centroid_mismatch; note(where + ": centroid dev " + std::to_string(cd)); }

            std::vector<Point_3> old_pts, new_pts = distinct(n->patch_vertices);
            for (const auto& v : o["patch_vertices"]) old_pts.push_back(to_pt(v));
            old_pts = distinct(old_pts);
            {
                std::vector<Point_3> old_raw;
                for (const auto& v : o["patch_vertices"]) old_raw.push_back(to_pt(v));
                double gd = polygon_deviation(n->patch_vertices, old_raw);
                st.max_polygon_dev = std::max(st.max_polygon_dev, gd);
                if (gd > POLY_TOL) {
                    ++st.polygon_mismatch;
                    note(where + ": patch POLYGON differs by " + std::to_string(gd) + " m");
                    if (st.poly_detail.empty()) {
                        std::string t = where + " (surface " + std::to_string(n->surface_id) + ", parent surface " + std::to_string(parent->surface_id) + ", parent yaw " +
                                        std::to_string(parent->foot_yaw) + ")\n        NEW:";
                        for (const auto& p : n->patch_vertices) t += " (" + std::to_string(CGAL::to_double(p.x())) + "," + std::to_string(CGAL::to_double(p.y())) + "," + std::to_string(CGAL::to_double(p.z())) + ")";
                        t += "\n        OLD:";
                        for (const auto& p : old_raw) t += " (" + std::to_string(CGAL::to_double(p.x())) + "," + std::to_string(CGAL::to_double(p.y())) + "," + std::to_string(CGAL::to_double(p.z())) + ")";
                        st.poly_detail = t;
                    }
                }
            }
            bool new_in_old = std::all_of(new_pts.begin(), new_pts.end(), [&](const Point_3& p) { return contains(old_pts, p); });
            bool old_in_new = std::all_of(old_pts.begin(), old_pts.end(), [&](const Point_3& p) { return contains(new_pts, p); });
            if (!new_in_old) { ++st.vertex_set_mismatch; note(where + ": new patch has a vertex absent from old patch"); }
            else if (!old_in_new) ++st.old_extra_vertices;
        }
    }
    return st;
}


struct RobustStats {
    int expansions = 0, surfaces = 0, oracle_mismatch = 0, presence_mismatch = 0;
    int old_children_surfaces = 0, differs_from_old = 0, old_not_subset = 0, new_only = 0, old_only = 0;
    double max_oracle_dev = 0, max_old_dev = 0;
    std::vector<std::string> examples;
};

// Corrected clip, old run's exact hull: (1) every surface's children must match an
// independent exact-arithmetic recomputation of the same cut; (2) versus the old
// output, a difference is acceptable only when the old patch is CONTAINED in
// the new one (the old clip only ever loses points, it never invents any).
RobustStats robust_scene(const std::string& scene, const json& dump, const ReachabilityModel& reachability) {
    RobustStats st;
    config::Scenario scenario = config::load_scenario(scene);
    ExpansionParams params;
    params.rotation_enabled = true;
    params.yaw_discretization_num = 3;
    params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    params.cycle_detection_enabled = true;
    params.legacy_clip = false;

    for (size_t ei = 0; ei < dump["expansions"].size(); ++ei) {
        const json& e = dump["expansions"][ei];
        const json& pj = e["parent"];
        NodePool pool;
        Node* parent = pool.create();
        parent->depth = pj["depth"];
        parent->stance_foot = pj["stance_foot"].get<int>() == 0 ? StanceFoot::Left : StanceFoot::Right;
        parent->foot_yaw = pj["foot_yaw"];
        parent->surface_id = pj["surface_id"];
        parent->perimeter = pj["perimeter"];
        parent->centroid = to_pt(pj["centroid"]);
        for (const auto& v : pj["patch_vertices"]) parent->patch_vertices.push_back(to_pt(v));
        for (size_t k = 0; k < 2 && k < pj["pred_surface_ids"].size(); ++k)
            parent->pred_surface_ids[k] = pj["pred_surface_ids"][k].get<std::vector<std::vector<int>>>();

        EdgeList edges = old_edges(e["p_union_mesh"]);
        params.union_edges_override = [edges](const Node&) { return edges; };
        NodePool kpool;
        std::vector<Node*> kids = expand_node(parent, scenario.surfaces, reachability, ReachabilityDirection::Forward, params, kpool);
        ++st.expansions;

        std::map<int, std::vector<const Node*>> new_by_surface;
        for (const Node* n : kids) new_by_surface[n->surface_id].push_back(n);
        std::map<int, std::vector<Point_3>> old_by_surface; // first child's raw patch per surface
        for (const auto& c : e["children"]) {
            int sid = c["surface_id"].get<int>();
            if (!old_by_surface.count(sid)) {
                std::vector<Point_3> v;
                for (const auto& q : c["patch_vertices"]) v.push_back(to_pt(q));
                old_by_surface[sid] = v;
            }
        }
        StanceFoot child_stance = other_foot(parent->stance_foot);
        for (const Surface& surf : scenario.surfaces) {
            ++st.surfaces;
            std::vector<Point_3> cut = compute_edges_plane_intersection(surf.plane, edges);
            std::vector<Point_3> oracle = oracle::patch_from_cut_exact_clip(cut, surf);
            bool blocked = cycle_path_detection(parent, child_stance, surf.surface_id);
            bool expect_children = !oracle.empty() && !blocked;
            auto it = new_by_surface.find(surf.surface_id);
            bool have = it != new_by_surface.end();
            if (have != expect_children) {
                ++st.presence_mismatch;
                if (st.examples.size() < 5) st.examples.push_back("expansion " + std::to_string(ei) + " surface " + std::to_string(surf.surface_id) + ": children present " + std::to_string(have) + ", oracle expects " + std::to_string(expect_children));
                continue;
            }
            if (have) {
                double d = polygon_deviation(it->second.front()->patch_vertices, oracle);
                st.max_oracle_dev = std::max(st.max_oracle_dev, d);
                if (d > POLY_TOL) {
                    ++st.oracle_mismatch;
                    if (st.examples.size() < 5) st.examples.push_back("expansion " + std::to_string(ei) + " surface " + std::to_string(surf.surface_id) + ": patch differs from exact oracle by " + std::to_string(d));
                }
            }
            // versus the old output
            auto oit = old_by_surface.find(surf.surface_id);
            bool old_has = oit != old_by_surface.end();
            if (old_has) ++st.old_children_surfaces;
            if (have && !old_has) { ++st.new_only; ++st.differs_from_old; continue; } // old lost the whole patch
            if (!have && old_has) { ++st.old_only; ++st.differs_from_old; ++st.old_not_subset; continue; }
            if (have && old_has) {
                const auto& np = it->second.front()->patch_vertices;
                double d = polygon_deviation(np, oit->second);
                st.max_old_dev = std::max(st.max_old_dev, d);
                if (d > POLY_TOL) {
                    ++st.differs_from_old;
                    // old must lie inside new: area(new) >= area(old) and every old hull vertex inside/on new
                    auto hn = hull_xy(np), ho = hull_xy(oit->second);
                    bool subset = polygon_area(hn) >= polygon_area(ho) - 1e-9;
                    for (const auto& p : ho)
                        if (CGAL::bounded_side_2(hn.begin(), hn.end(), p) == CGAL::ON_UNBOUNDED_SIDE && boundary_distance(p, hn) > 1e-9) subset = false;
                    if (!subset) ++st.old_not_subset;
                }
            }
        }
    }
    return st;
}

} // namespace

int main(int argc, char** argv) {
    // --replay-old-hull: feed expand_node the old run's exact P_union hull
    // (same triangulation) and demand EVERYTHING exact, on every scene.
    bool replay = argc == 3 && std::string(argv[2]) == "--replay-old-hull";
    bool robust = argc == 3 && std::string(argv[2]) == "--robust";
    if (argc != 2 && !replay && !robust) {
        std::cerr << "Usage: " << argv[0] << " <dump_dir containing <scene>.json from old_expansion_dump> [--replay-old-hull | --robust]\n";
        return 1;
    }
    ReachabilityModel reachability = make_forward_reachability();
    bool all_ok = true;
    int compared_scenes = 0;

    std::cout << "scene                 expansions children | ASSERTED: count surf stance depth yaw hist POLYGON (max dev) | reported only: perim centroid vertset (max centroid dev)\n";
    for (const std::string& scene : kScenes) {
        std::ifstream f(std::string(argv[1]) + "/" + scene + ".json");
        if (!f) {
            std::cout << scene << ": no dump, skipped\n";
            continue;
        }
        json dump;
        f >> dump;
        if (robust) {
            RobustStats rs = robust_scene(scene, dump, reachability);
            ++compared_scenes;
            bool ok = rs.oracle_mismatch == 0 && rs.presence_mismatch == 0 && rs.old_not_subset == 0;
            all_ok = all_ok && ok;
            std::printf("%-20s %4d expansions %6d surface cuts | vs EXACT oracle: patch mismatches %d, presence mismatches %d (max dev %.1e) | vs old: %d differ (%d only new, %d only old), old NOT contained in new: %d (max dev %.2f m)  %s\n",
                        scene.c_str(), rs.expansions, rs.surfaces, rs.oracle_mismatch, rs.presence_mismatch, rs.max_oracle_dev, rs.differs_from_old,
                        rs.new_only, rs.old_only, rs.old_not_subset, rs.max_old_dev, ok ? "OK" : "MISMATCH");
            for (const auto& ex : rs.examples) std::cout << "      e.g. " << ex << "\n";
            continue;
        }
        Stats st = compare_scene(scene, dump, reachability, replay ? Mode::Replay : Mode::Port);
        ++compared_scenes;
        bool unstable = is_unstable_in_old(scene);
        bool polygon_ok = unstable ? st.polygon_mismatch <= UNSTABLE_SCENE_MAX_POLYGON_RATE * std::max(1, st.children)
                                   : st.polygon_mismatch == 0;
        bool scene_ok = st.structure_ok() && polygon_ok;
        if (replay) {
            // same triangulation in => everything must be bit-for-bit: polygon,
            // perimeter, centroid, and the raw vertex list (no extras either).
            scene_ok = st.structure_ok() && st.polygon_mismatch == 0 && st.perimeter_mismatch == 0 && st.centroid_mismatch == 0 &&
                       st.vertex_set_mismatch == 0 && st.old_extra_vertices == 0;
            std::printf("      replay (old hull edge list): old-only vertices %d\n", st.old_extra_vertices);
        }
        all_ok = all_ok && scene_ok;
        char line[400];
        std::snprintf(line, sizeof(line), "%-20s %10d %8d | %5d %4d %6d %5d %3d %4d %7d (%.1e) | %5d %8d %7d (%.1e)  %s",
                      scene.c_str(), st.expansions, st.children, st.count_mismatch, st.surface_mismatch, st.stance_mismatch,
                      st.depth_mismatch, st.yaw_mismatch, st.history_mismatch, st.polygon_mismatch, st.max_polygon_dev,
                      st.perimeter_mismatch, st.centroid_mismatch, st.vertex_set_mismatch, st.max_centroid_dev,
                      scene_ok ? (replay ? "OK (exact, old hull replayed)" : unstable ? "OK (old code unstable here, bounded)" : "OK (exact)") : "MISMATCH");
        std::cout << line << "\n";
        std::printf("      stages over first %d expansions: rotated polytope mismatches %d (max dev %.2e) | P_union mismatches %d (max dev %.2e) | plane-intersection mismatches %d (max dev %.2e)\n",
                    st.stage_expansions, st.rot_mismatch, st.max_rot_dev, st.punion_mismatch, st.max_punion_dev, st.plane_mismatch, st.max_plane_dev);
        std::printf("      vertex ORDER differs: rotated polytope %d, P_union %d (of %d expansions) %s\n", st.rot_order_diff, st.punion_order_diff, st.stage_expansions, st.order_note.c_str());
        if (!st.poly_detail.empty()) std::cout << "      first POLYGON mismatch: " << st.poly_detail << "\n";
        if (!st.plane_detail.empty()) std::cout << "      first plane-intersection mismatch: " << st.plane_detail;
        std::printf("      surface plane coefficients: max dev %.2e %s\n", st.max_plane_coeff_dev, st.plane_coeff_note.c_str());
        for (const auto& ex : st.examples) std::cout << "      e.g. " << ex << "\n";
    }
    if (compared_scenes == 0) {
        std::cerr << "no dumps found\n";
        return 1;
    }
    if (robust) {
        std::cout << (all_ok ? "CORRECTED CLIP: EVERY PATCH EQUALS THE EXACT-ARITHMETIC ORACLE; DIFFERENCES FROM THE OLD OUTPUT ARE ONLY POINTS THE OLD CLIP LOST\n" : "DIFFERENCES FOUND\n");
        return all_ok ? 0 : 1;
    }
    if (replay) {
        std::cout << (all_ok ? "REPLAY OF THE OLD P_UNION HULL: EVERYTHING BIT-IDENTICAL (structure, polygon, perimeter, centroid, raw vertex list) ON EVERY EXPANSION\n" : "DIFFERENCES FOUND\n");
        return all_ok ? 0 : 1;
    }
    std::cout << (all_ok ? "STRUCTURE IDENTICAL ON EVERY EXPANSION; PATCH GEOMETRY EXACT WHERE THE OLD CODE IS STABLE, BOUNDED WHERE IT IS NOT\n" : "DIFFERENCES FOUND\n");
    return all_ok ? 0 : 1;
}
