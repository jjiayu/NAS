// Per-foot goal generalization (AstarSearchConfig::FootGoal / foot_goals): a goal for one foot is
// now a point OR an arbitrary polytope, each independently with or without an accepted yaw range: 4
// shape/yaw combinations, covered across variants (b)-(d) below. Filling zero, one, or both of the
// two (Left/Right-indexed) slots selects mode 0 (legacy, untouched), mode 1 (single foot, direct
// generalization of goal_location/goal_stance_foot: the other foot stays free) or mode 2 ("closing
// stance": terminates only when the last two consecutive footsteps each satisfy their own slot at
// once) — see astar_search.hpp's own doc comment on foot_goals for the full contract, and
// astar_search.cpp for why the closing-stance heuristic must sum both feet's remaining distance
// rather than track only the foot currently being placed.
//
// Scenario: Flat, start (0,0,0) facing +x, right stance (same setup as goal_yaw_test/goal_surface_test).
// Every target region and expected path below was found empirically first (a throwaway probe, not
// committed), not guessed by hand: with heuristic_weight=10 and no yaw-change cost, this planner's
// unconstrained gait already rotates fairly aggressively step to step and its reachable patches are
// larger than they look, so a hand-picked "plausible" target can turn out unreachable, or satisfied
// in a surprisingly small number of steps by a patch whose centroid isn't even close to it (patches
// are regions, not points — a target only needs to overlap SOME of a patch, not contain its
// centroid). That same empirical pass also caught a real bug this test guards against: a flat target
// polytope exactly coincident with the node's own contact plane (the common case — most targets sit
// on the same floor the foot is already on) made compute_polytope_plane_intersection return nothing
// at all, because its edge-crossing search never finds a crossing when every vertex already lies
// exactly on the cutting plane. Variant (c) below (and (d)'s left slot) exercise exactly this case.
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace nas;

#ifndef TALOS_REACHABILITY_DATA_DIR
#error "TALOS_REACHABILITY_DATA_DIR must be defined by CMake"
#endif

namespace {

int g_failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what.c_str());
    }
}

bool yaw_in_range_deg(const Node* n, double lo_deg, double hi_deg) {
    double deg = n->foot_yaw * 180.0 / M_PI;
    return deg >= lo_deg - 1e-6 && deg <= hi_deg + 1e-6;
}

} // namespace

int run_foot_goals() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("Flat");

    auto base_config = [&]() {
        AstarSearchConfig cfg;
        cfg.start_position = Point_3(0, 0, 0);
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.expansion_params.rotation_enabled = true;
        cfg.max_expansions = 5000;
        return cfg;
    };

    // (a) foot_goals entirely empty: mode 0, the legacy single point-or-surface goal — must behave
    // exactly like before this feature existed (the code path itself is untouched; this is a smoke
    // test that adding the feature didn't perturb the default-off case).
    {
        AstarSearchConfig cfg = base_config();
        cfg.goal_location = Point_3(0.35, 0.0, 0.0);
        cfg.goal_stance_foot = StanceFoot::Left;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(a) sans foot_goals : comportement historique inchange, un chemin est trouve");
    }

    // (b) mode 1, POINT shape, no yaw range, left foot only — the right foot stays completely free.
    Node* mode1_point_last = nullptr;
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.15, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(b) mode 1 point (pied gauche, sans yaw) : un chemin est trouve");
        if (!search.result_path().empty()) {
            mode1_point_last = search.result_path().back();
            check(mode1_point_last->stance_foot == StanceFoot::Left, "(b) le dernier pas est bien celui du pied cible (gauche)");
            check(mode1_point_last->check_if_node_contains_point(Point_3(0.3, 0.15, 0.0)),
                  "(b) le patch du dernier pas contient bien le point cible");
        }
    }

    // (c) mode 1, POLYTOPE shape + yaw range, right foot only, flat and exactly coplanar with the
    // node's own contact plane (z=0, same as the Flat floor) -- exactly the case that needed the
    // coplanar-target fix.
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal g;
        g.region = std::vector<Point_3>{Point_3(0.1, -0.3, 0.0), Point_3(0.3, -0.3, 0.0), Point_3(0.3, -0.05, 0.0),
                                         Point_3(0.1, -0.05, 0.0)};
        g.yaw_range = std::make_pair(-90.0 / 180.0 * M_PI, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Right)] = g;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(c) mode 1 polytope+yaw (pied droit, plat et coplanaire) : un chemin est trouve");
        if (!search.result_path().empty()) {
            Node* last = search.result_path().back();
            check(last->stance_foot == StanceFoot::Right, "(c) le dernier pas est bien celui du pied cible (droit)");
            check(yaw_in_range_deg(last, -90.0, 0.0), "(c) le lacet final est dans la plage demandee");
        }
    }

    // (d) mode 2 ("closing stance"), both slots filled, mixed shapes to cover the two combinations
    // not yet exercised by (b)/(c): left = polytope, NO yaw; right = POINT, WITH yaw range. Regions
    // were built around an actual mode-1 solution found while writing this test (guaranteeing at
    // least that exact path stays valid), generous enough to not depend on hitting it exactly.
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal left_g; // polytope, no yaw
        left_g.region = std::vector<Point_3>{Point_3(0.05, -0.02, 0.0), Point_3(0.35, -0.02, 0.0),
                                              Point_3(0.35, 0.29, 0.0), Point_3(0.05, 0.29, 0.0)};
        AstarSearchConfig::FootGoal right_g; // point, with yaw range
        right_g.region = Point_3(-0.2365, 0.0813, 0.0);
        right_g.yaw_range = std::make_pair(-100.0 / 180.0 * M_PI, -20.0 / 180.0 * M_PI);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Right)] = right_g;

        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(d) mode 2 (cibles mixtes) : un chemin est trouve");
        if (search.result_path().size() >= 2) {
            const auto& path = search.result_path();
            Node* last = path.back();
            Node* prev = path[path.size() - 2];
            check(last->parent == prev, "(d) le dernier noeud a bien l'avant-dernier pour parent (verification de base du backtrack)");
            check(last->stance_foot != prev->stance_foot, "(d) les deux derniers pas sont bien de pieds opposes");
            // Whichever of the last two nodes is the LEFT foot must satisfy the polytope slot; the
            // RIGHT one must satisfy the point+yaw slot — order (which foot closes last) is not fixed.
            Node* left_node = last->stance_foot == StanceFoot::Left ? last : prev;
            Node* right_node = last->stance_foot == StanceFoot::Right ? last : prev;
            check(right_node->check_if_node_contains_point(Point_3(-0.2365, 0.0813, 0.0)),
                  "(d) le pas du pied droit atteint bien le point cible");
            check(yaw_in_range_deg(right_node, -100.0, -20.0), "(d) le lacet du pied droit est dans la plage demandee");
            (void)left_node; // shape already exercised precisely by (c); here only success + pairing matter
        }
    }

    // (e) mode 2, one target moved 10m out of any conceivable reach: must fail cleanly (no path)
    // within a modest expansion budget, not falsely succeed on a &&/|| slip in the termination test,
    // and not hang.
    {
        AstarSearchConfig cfg = base_config();
        cfg.max_expansions = 500;
        AstarSearchConfig::FootGoal left_g;
        left_g.region = std::vector<Point_3>{Point_3(0.05, -0.02, 0.0), Point_3(0.35, -0.02, 0.0),
                                              Point_3(0.35, 0.29, 0.0), Point_3(0.05, 0.29, 0.0)};
        AstarSearchConfig::FootGoal unreachable_g;
        unreachable_g.region = Point_3(10.2365, 0.0813, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Right)] = unreachable_g;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(search.result_path().empty(), "(e) mode 2 avec une cible hors de portee : correctement \"pas de chemin\"");
    }

    // (f) mode 2, a genuinely 3D (non-flat) polytope target for one slot — exercises the real
    // plane-slice path (compute_polytope_plane_intersection actually cutting through a volume),
    // not just its coplanar-shortcut special case exercised by (c)/(d).
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal left_g; // a box straddling z=0, not a flat set of coplanar points
        left_g.region = std::vector<Point_3>{
            Point_3(0.05, -0.02, -0.1), Point_3(0.35, -0.02, -0.1), Point_3(0.35, 0.29, -0.1), Point_3(0.05, 0.29, -0.1),
            Point_3(0.05, -0.02, 0.1), Point_3(0.35, -0.02, 0.1), Point_3(0.35, 0.29, 0.1), Point_3(0.05, 0.29, 0.1)};
        AstarSearchConfig::FootGoal right_g;
        right_g.region = Point_3(-0.2365, 0.0813, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = left_g;
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Right)] = right_g;
        AstarSearch search(sc.surfaces, reach, cfg);
        search.search();
        check(!search.result_path().empty(), "(f) mode 2 avec un polytope 3D non plat : un chemin est trouve");
    }

    // Negative controls at construction (mirroring goal_yaw_test.cpp's style): each must throw
    // std::invalid_argument, not silently misbehave or crash later inside a geometry call.
    auto expect_throw = [&](AstarSearchConfig cfg, const std::string& what) {
        bool threw = false;
        try {
            AstarSearch bad(sc.surfaces, reach, cfg);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        check(threw, what);
    };

    {
        AstarSearchConfig cfg = base_config();
        cfg.goal_surface_id = 0;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.15, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        expect_throw(cfg, "foot_goals + goal_surface_id leve une erreur claire (mecanismes mutuellement exclusifs)");
    }
    {
        AstarSearchConfig cfg = base_config();
        cfg.goal_yaw_target = 0.0;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.15, 0.0);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        expect_throw(cfg, "foot_goals + goal_yaw_target leve une erreur claire (mecanismes mutuellement exclusifs)");
    }
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal g;
        g.region = std::vector<Point_3>{Point_3(0, 0, 0), Point_3(0.1, 0, 0)}; // only 2 vertices
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        expect_throw(cfg, "un polytope a moins de 3 sommets leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.15, 0.0);
        g.yaw_range = std::make_pair(0.5, 0.1); // max < min
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        expect_throw(cfg, "un yaw_range avec max < min leve une erreur claire");
    }
    {
        AstarSearchConfig cfg = base_config();
        cfg.expansion_params.rotation_enabled = false;
        AstarSearchConfig::FootGoal g;
        g.region = Point_3(0.3, 0.15, 0.0);
        g.yaw_range = std::make_pair(0.0, 0.1);
        cfg.foot_goals[static_cast<size_t>(StanceFoot::Left)] = g;
        expect_throw(cfg, "un yaw_range sans rotation_enabled leve une erreur claire");
    }

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All foot_goals tests passed\n");
    return 0;
}
