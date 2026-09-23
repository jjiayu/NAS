// Optional final-yaw constraint/cost on the goal (AstarSearchConfig::goal_yaw_target/tolerance/weight,
// added on request — see docs/paper-deltas.md).
//
// Scenario: Flat, a straight corridor (start (0,0,0), goal along +x). Left alone, the search arrives
// facing roughly the direction of travel (~0 deg) — never intentionally, just whatever the tie-break
// happens to produce. goal_yaw_target = 180 deg forces the plan to end facing the OPPOSITE direction,
// a full reversal from that natural heading: a deliberately hard case, chosen so the with/without
// difference (and the effect of also adding a guiding cost) shows up clearly rather than being lost in
// noise a small turn would produce.
//
// Four variants compared: (a) no constraint at all (today's behaviour, unaffected by this feature
// existing) ; (b) the hard constraint alone (goal_yaw_weight = 0: search unguided, only filtered at
// termination) ; (c) constraint + a guiding edge cost toward the target ; (d) (c) plus heading_weight
// (the existing, unrelated "align with the direction of travel" cost) at the same time, to show the two
// costs pulling in different directions during the walk while agreeing at the very end.
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

double angdiff_deg(double a, double b) {
    double d = std::fmod(std::abs(a - b), 2.0 * M_PI);
    return (d > M_PI ? 2.0 * M_PI - d : d) * 180.0 / M_PI;
}

double total_rotation_deg(const std::vector<Node*>& path) {
    double t = 0.0;
    for (size_t i = 1; i < path.size(); ++i) t += angdiff_deg(path[i]->foot_yaw, path[i - 1]->foot_yaw);
    return t;
}

} // namespace

int run_goal_yaw() {
    std::string dir = TALOS_REACHABILITY_DATA_DIR;
    ReachabilityModel reach = ReachabilityModel::load({
        {dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("Flat");
    Point_3 start(0, 0, 0);
    Point_3 goal = sc.surfaces.back().centroid;

    const double target = M_PI; // 180 deg: a full reversal from the natural ~0 deg arrival
    // 30 deg = one full yaw-discretization arc (yaw_angle_increment_deg x yaw_discretization_num, the
    // span a single expansion's children already cover) — chosen empirically, not just "a reasonable
    // number": tightening it to 10 deg (one third as wide) measured ~10x more expansions needed (~3300
    // instead of ~300) to find a node that jointly satisfies position AND that narrow a yaw window, and
    // per-expansion cost itself grows with the total node count on this kind of hard-to-satisfy search
    // (super-linearly, not measured further here) — so 10 deg made this one test alone take minutes.
    // That is itself the answer to "what does a tight final-orientation window cost": a lot, in a way
    // that compounds, well before it becomes literally unreachable. Left at 30 deg so this test stays
    // fast and reliable; the finding is recorded here and in docs/paper-deltas.md.
    const double tolerance = 30.0 / 180.0 * M_PI;

    auto base_config = [&]() {
        AstarSearchConfig cfg;
        cfg.start_position = start;
        cfg.start_stance_foot = StanceFoot::Right;
        cfg.goal_location = goal;
        cfg.goal_stance_foot = StanceFoot::Left;
        cfg.expansion_params.rotation_enabled = true;
        cfg.max_expansions = 1000; // safety cap — see the tolerance comment above on why this can grow fast
        return cfg;
    };

    AstarSearchConfig cfg_a = base_config(); // (a) no constraint at all — today's behaviour
    AstarSearchConfig cfg_b = base_config(); // (b) hard constraint, no guiding cost
    cfg_b.goal_yaw_target = target;
    cfg_b.goal_yaw_tolerance = tolerance;
    AstarSearchConfig cfg_c = base_config(); // (c) constraint + guiding cost
    cfg_c.goal_yaw_target = target;
    cfg_c.goal_yaw_tolerance = tolerance;
    cfg_c.goal_yaw_weight = 0.5; // 0.1 measured too weak to change anything at this tolerance (see diagnosis above)
    AstarSearchConfig cfg_d = cfg_c; // (d) + heading_weight (aligns with direction of travel) at the same time
    cfg_d.heading_weight = 0.1; // measured: >=0.2 here fights goal_yaw_weight hard enough to blow the expansion budget

    AstarSearch search_a(sc.surfaces, reach, cfg_a);
    AstarSearch search_b(sc.surfaces, reach, cfg_b);
    AstarSearch search_c(sc.surfaces, reach, cfg_c);
    AstarSearch search_d(sc.surfaces, reach, cfg_d);
    search_a.search();
    search_b.search();
    search_c.search();
    search_d.search();

    struct Variant { const char* name; AstarSearch& search; };
    std::vector<Variant> variants = {
        {"(a) sans contrainte", search_a},
        {"(b) contrainte seule (poids 0)", search_b},
        {"(c) contrainte + cout guide (0,5)", search_c},
        {"(d) + heading_weight (0,1)", search_d},
    };

    std::printf("%-38s %6s %6s %8s %14s %10s\n", "variante", "trouve", "exp", "pas", "lacet final deg", "rotation deg");
    for (const Variant& v : variants) {
        const auto& path = v.search.result_path();
        bool found = !path.empty();
        double yaw_deg = found ? path.back()->foot_yaw * 180.0 / M_PI : 0.0;
        double rot = found ? total_rotation_deg(path) : 0.0;
        std::printf("%-38s %6s %6d %8zu %14.1f %10.0f\n", v.name, found ? "oui" : "non", v.search.expansion_count(),
                    path.size(), yaw_deg, rot);
    }

    check(!search_a.result_path().empty(), "(a) sans contrainte : un chemin est trouve");
    check(!search_b.result_path().empty(), "(b) contrainte seule : un chemin est trouve malgre le demi-tour impose");
    check(!search_c.result_path().empty(), "(c) contrainte + cout : un chemin est trouve");
    check(!search_d.result_path().empty(), "(d) + heading_weight : un chemin est trouve");

    for (const Variant& v : {variants[1], variants[2], variants[3]}) { // (b), (c), (d): must end within tolerance
        const auto& path = v.search.result_path();
        if (path.empty()) continue;
        double d = angdiff_deg(path.back()->foot_yaw, target); // both in radians; angdiff_deg returns degrees
        check(d <= tolerance * 180.0 / M_PI + 1e-6, std::string(v.name) + " : le lacet final est dans la tolerance de la cible (180 deg)");
    }

    // The whole point of the constraint: (b) must differ meaningfully from the unconstrained (a) — a
    // plan that ends facing backwards cannot look like one that was never asked to.
    if (!search_a.result_path().empty() && !search_b.result_path().empty()) {
        double yaw_a_rad = search_a.result_path().back()->foot_yaw;
        check(angdiff_deg(yaw_a_rad, target) > tolerance * 180.0 / M_PI,
              "(a) sans contrainte n'atterrit pas deja pres de la cible (sinon le test ne prouve rien)");
        double rot_a = total_rotation_deg(search_a.result_path());
        double rot_b = total_rotation_deg(search_b.result_path());
        check(rot_b > rot_a + 30.0, "(b) tourne nettement plus que (a) — la contrainte a un effet reel sur le plan");
    }

    // The guiding cost (c) should not need MORE expansions than the unguided hard filter (b) to reach an
    // equally-constrained goal — that's the entire motivation for adding it instead of the filter alone.
    if (!search_b.result_path().empty() && !search_c.result_path().empty()) {
        check(search_c.expansion_count() <= search_b.expansion_count(),
              "(c) le cout guide n'expanse pas plus que (b) le filtre seul pour satisfaire la meme contrainte");
    }

    // (d) combines two costs pulling in different directions while walking (heading_weight wants the
    // foot aligned with the direction of travel — roughly 0 deg on this straight corridor — while
    // goal_yaw_weight pulls toward 180 deg): the resulting plan is expected to differ from (c) (not
    // necessarily "better" or "worse", just a different compromise), while still respecting the hard
    // constraint at the end (checked above).
    if (!search_c.result_path().empty() && !search_d.result_path().empty()) {
        bool same_path = search_c.result_path().size() == search_d.result_path().size();
        if (same_path) {
            for (size_t i = 0; i < search_c.result_path().size() && same_path; ++i) {
                if (std::abs(search_c.result_path()[i]->foot_yaw - search_d.result_path()[i]->foot_yaw) > 1e-9) same_path = false;
            }
        }
        check(!same_path, "(d) le compromis avec heading_weight donne un plan different de (c)");
    }

    // Negative control: goal_yaw_target set without rotation enabled must throw at construction, not
    // silently search a state space where foot_yaw is always 0 (any target other than 0 would be
    // unreachable without any indication why).
    {
        bool threw = false;
        AstarSearchConfig cfg_bad = base_config();
        cfg_bad.expansion_params.rotation_enabled = false;
        cfg_bad.goal_yaw_target = target;
        try {
            AstarSearch bad(sc.surfaces, reach, cfg_bad);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        check(threw, "goal_yaw_target sans rotation_enabled leve une erreur claire");
    }

    // Negative control: a target/tolerance window the 10-degree discretization can never land in (175
    // +/- 1 deg, unreachable from multiples of 10 deg starting at foot_yaw 0) correctly reports "no
    // path" within the expansion cap, not a hang.
    {
        AstarSearchConfig cfg_unreach = base_config();
        cfg_unreach.goal_yaw_target = 175.0 / 180.0 * M_PI;
        cfg_unreach.goal_yaw_tolerance = 1.0 / 180.0 * M_PI;
        // Small cap on purpose: an unreachable target/tolerance never terminates on its own — the point
        // here is only that the search gives up cleanly within budget, not that it's fast; (b)/(c)/(d)
        // above needed up to ~3300 expansions to satisfy a REACHABLE constraint, so this one (which by
        // design never succeeds) would otherwise burn through the same 5000-expansion budget every time.
        cfg_unreach.max_expansions = 500;
        AstarSearch search_unreach(sc.surfaces, reach, cfg_unreach);
        search_unreach.search();
        check(search_unreach.result_path().empty(), "une cible/tolerance inatteignable par la discretisation rend correctement \"pas de chemin\"");
    }

    if (g_failures > 0) {
        std::fprintf(stderr, "%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("All goal_yaw tests passed\n");
    return 0;
}
