#pragma once

// ReachabilityModel — replaces the hardcoded pairs of global polytope paths
// duplicated across the old Tree/AstarSearch/FootstepPlanner (see PLAN.md
// phase 3). Generalizes the old binary "stance_foot==0 ? A : B" lookup into
// a query keyed by (moving_effector, support_effector, direction), which
// already fits N effectors without further change (see the quadruped
// extension notes in PLAN.md) — the biped case just never has more than 2
// possible keys.
//
// Loading contract — "couche 0" only (see PLAN.md phase 3): the caller
// supplies an explicit list of ReachabilityEntry {path, moving_effector,
// support_effector, direction}. There is deliberately no filename-parsing
// or package-discovery convenience layer here. Two reasons:
//   1. It was explicitly descoped for now ("j'aime pas trop le sucre pour
//      l'instant") — a Python-side convenience is planned separately later.
//   2. Talos's antecedent file naming turned out to be genuinely ambiguous
//      (RF_antecedent_CUTZ_2.obj is used for the "lf_in_rf" direction in
//      the old constants.hpp, and the LF_antecedent counterpart doesn't
//      even exist — see docs/paper-deltas.md). Auto-inferring (mover,
//      support, direction) from a filename would have silently guessed
//      wrong here; an explicit manifest pushes that judgment call to
//      whoever configures it, not to a heuristic.

#include "nas/core/geometry.hpp"
#include "nas/core/types.hpp"

#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace nas {

enum class ReachabilityDirection {
    // Where the moving effector can go next, given the support effector's
    // current position (used by CASSR/AstarSearch and the footstep QP).
    Forward,
    // Where the support effector must have been, given the moving
    // effector's current position (used by NAS/Tree's backward search).
    Antecedent
};

struct ReachabilityEntry {
    std::string path;
    std::string moving_effector;
    std::string support_effector;
    ReachabilityDirection direction;
};

class ReachabilityModel {
public:
    // Loads every entry's .obj file. Throws std::runtime_error (propagated
    // from the underlying CGAL polygon-mesh reader) if any file cannot be
    // read — fails the whole load rather than silently skipping an entry,
    // since a partially-loaded model is a worse failure mode than a loud
    // one at startup.
    static ReachabilityModel load(const std::vector<ReachabilityEntry>& entries);

    bool has(const std::string& moving_effector,
             const std::string& support_effector,
             ReachabilityDirection direction) const;

    // Throws std::out_of_range if the (moving, support, direction) key was
    // never loaded — check has() first if that's expected (e.g. Talos's
    // incomplete antecedent set today).
    const Polyhedron& query(const std::string& moving_effector,
                             const std::string& support_effector,
                             ReachabilityDirection direction) const;

    // The H-rep (convert_polytope_to_half_space_constraint) of a queried polytope, computed once
    // and cached (lazily, on first request) instead of rebuilt from a fresh CGAL::convex_hull_3 on
    // every call — the footstep QP was doing exactly that once per path step, so ~9 identical
    // rebuilds of the same 2 polytopes per solve on a 10-step path (see docs/paper-deltas.md, the
    // ProxQP backend comparison entry). The rotation applied per use (yaw, or yaw+tilt) still
    // happens on the caller's side, on this cached, unrotated H-rep — a rotation is cheap (a
    // matrix multiply per row) next to rebuilding the hull, and the row count/geometry here is
    // untouched: same convert_polytope_to_half_space_constraint, same rows, just computed once.
    // Throws std::out_of_range under the same condition as query().
    const HalfSpacePolytopeConstraint& half_space_constraint(const std::string& moving_effector,
                                                               const std::string& support_effector,
                                                               ReachabilityDirection direction) const;

    size_t size() const { return polytopes_.size(); }

private:
    using Key = std::tuple<std::string, std::string, ReachabilityDirection>;
    std::map<Key, Polyhedron> polytopes_;
    mutable std::map<Key, HalfSpacePolytopeConstraint> hrep_cache_;
};

} // namespace nas
