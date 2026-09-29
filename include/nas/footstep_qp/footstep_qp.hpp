#pragma once

// Footstep QP formulation (see PLAN.md phase 7) — builds the QPProblem
// once, backend-agnostic, from a CASSR result path. Direct Eigen matrix
// construction replaces the old code's CasADi symbolic expression tree
// (never actually needed for autodiff — every Jacobian there was already
// hand-built, see qp_backend.hpp).
//
// Only reproduces the constraint blocks the old FootstepPlanner actually
// used: reachability_constraints_prev and com_constraints_next were
// computed in the old code but never included in its final constraint set
// (dead code, confirmed by reading it — see docs/paper-deltas.md), so they
// are not ported here at all.
//
// Numerical note: the stride-length part of the objective is a graph
// Laplacian (each stride term only depends on x_i - x_j), which is only
// positive *semi*-definite — it has a 3D null space (shifting every
// footstep by the same constant vector). The old CasADi/qpOASES pipeline
// tolerated that; eiquadprog's Cholesky-based active-set method requires a
// strictly positive-definite Hessian. A tiny Tikhonov regularization
// (FootstepQPConfig::hessian_regularization) is added to fix this — the
// constrained problem is well-posed regardless (the initial/final equality
// constraints pin down the translational null space), the regularization
// only makes eiquadprog's Cholesky step well-defined without materially
// changing the optimum.

#include "nas/core/geometry.hpp"
#include "nas/core/node.hpp"
#include "nas/core/reachability.hpp"
#include "nas/footstep_qp/qp_backend.hpp"

#include <optional>
#include <variant>
#include <vector>

namespace nas {

struct FootstepQPConfig {
    // Matches the paper's Eq. 6 (Sec. VI): "we empirically scale alpha by a
    // factor 10" — confirmed by the paper itself, see docs/paper-deltas.md.
    double alpha_weight = 10.0;
    // Must match whatever ExpansionParams::rotation_enabled the path was
    // searched with (selects identity vs. per-step yaw rotation matrices
    // in the reachability constraints).
    bool rotation_enabled = false;
    double hessian_regularization = 1e-8;
    // A solution is only reported as a success when every constraint holds within this
    // tolerance (metres). The active-set solver can return "optimal" with a residual of
    // ~5e-5 on the ill-conditioned problems the regularization creates; such a result is
    // re-solved with a 100x stronger regularization (up to 4 attempts), then declared a failure.
    double feasibility_tolerance = 1e-6;

    // Additional, purely opt-in per-node goal constraints - for a path whose foot_goals had more
    // than one slot filled (AstarSearchConfig's "closing stance" mode: the last TWO path nodes,
    // one per foot, each satisfy their own slot - "which foot closes last" isn't fixed, so either
    // could be path_nodes.size()-1 or -2). goal_position below only ever pins path_nodes.back() to
    // an exact point; this generalizes to an arbitrary node index, and to a REGION rather than
    // always a point: a Point_3 constrains that node the same way goal_position does (equality); a
    // polytope constrains it to stay inside that region (same alpha-coupled boundary inequalities
    // as an intermediate footstep's own patch, generate_surface_constraint on the region's own
    // vertices instead of the node's patch_vertices - the node's actual patch is only ever a
    // superset of a foot_goals polytope slot, since that's what let the search terminate there).
    // Empty by default: every existing caller is unaffected. A node_index covered here is excluded
    // from the default "free on its own patch" handling of the last node (see goal_position) and
    // from the ordinary intermediate-patch loop, so it is constrained exactly once, not twice.
    struct GoalConstraint {
        std::size_t node_index;
        std::variant<Point_3, std::vector<Point_3>> region;
    };
    std::vector<GoalConstraint> goal_constraints;

    // Couples a cube's placement into the SAME QP, instead of leaving it to a caller-side LP run
    // afterward on the already-fixed footsteps (g1motion's cube_plan.py::place_box - see its own
    // updated doc comment). The box's base center becomes 3 more QP variables (excluded from the
    // stride objective, like alpha), constrained by: (a) the "Cube_in_<support foot>" reachability
    // polytope (reachability.half_space_constraint, the SAME mechanism an ordinary footstep's own
    // reachability constraint already uses just above, with "Cube" as the moving effector - see
    // NAS's docs/cube-extension-mechanism.md), (b) the search's own placement_patch (the same
    // mechanism an intermediate footstep's own patch gets), and (c) the coupling constraint
    // itself: onto_index's footstep must land within the box's own half_extent x half_extent top
    // face, oriented like the support footstep - LINEAR jointly in (x_onto, c) since the
    // footstep's yaw is fixed by the search (not a QP unknown), with the SAME alpha robustness
    // margin as every other boundary constraint (no separate margin variable, unlike place_box's
    // own maximized t).
    //
    // Why this exists: place_box takes the onto footstep's position as already fixed (this QP,
    // upstream, chose it purely to minimize stride length - genuinely unaware a cube is even
    // involved) and only then tries to fit a box under it. The search's own continuous patches
    // guarantee SOME mutually consistent (box, onto-foot) pair exists, but that guarantee is lost
    // once two separate, uncoupled steps (this QP, then place_box) each independently pick one
    // specific discrete value from those patches. It usually still works out (plenty of slack),
    // until it doesn't (g1motion's boxcube_discover_two.py, a two-cube scenario: the first cube's
    // spacious patch left enough slack, the second's tight one didn't - place_box's LP came back
    // infeasible even though the search had already found a fully valid path). Solving both in the
    // same QP guarantees compatibility by construction instead of hoping for it.
    struct CubePlacement {
        // Indices into path_nodes (this function's own indexing, matching plan.footsteps): the
        // footstep supporting the placement (the foot on the ground when the box was set down -
        // the search's own "place" pseudo-node shares its position, so this is that pseudo-node's
        // PARENT, since the pseudo-node itself is never passed to this function - see
        // cube_plan.cpp's zero_displacement filtering), and the footstep landing on top of the
        // box (the first "step onto the cube" event after the placement - not necessarily
        // support_index + 1, the search can take ordinary steps in between before actually
        // stepping onto the cube).
        std::size_t support_index;
        std::size_t onto_index;
        // The search's own candidate region for the box's base center (world frame, flat/coplanar
        // - a placement node's own cube_vertices).
        std::vector<Point_3> placement_patch;
    };
    // Empty (default): today's exact behavior, the box is not a QP variable at all - every
    // existing caller (including every prior test) is unaffected.
    std::vector<CubePlacement> cube_placements;
    double cube_half_extent = 0.0;  // required (and validated) when cube_placements is non-empty
};

struct FootstepPlan {
    bool success = false;
    std::vector<Point_3> footsteps; // one per path_nodes entry, world frame
    double alpha = 0.0;
    double max_violation = 0.0;     // largest constraint residual of the returned solution (<= feasibility_tolerance when success)
    double objective = 0.0;         // 0.5*x'*H*x + g'*x at the solution, H unregularized (paper Eq. 6) — lets backends be compared on the value they actually optimize, not just feasibility
    std::vector<Point_3> cube_centers; // one per config.cube_placements entry, same order, only when success
};

// `path_nodes` is a full CASSR result path (path_nodes[0] is the start
// node, path_nodes.back() is the goal-containing node) — the same object
// AstarSearch::result_path() returns. `reachability` must hold Forward
// entries for every (moving, support) pair the path exercises.
// The goal is either a position (the last footstep is fixed to it: an equality) or, with `goal_position` empty, a
// surface: the last footstep is then free on the last patch (the same surface constraint, with the margin alpha, as the
// intermediate footsteps). A Point_3 converts to the position case.
FootstepPlan solve_footstep_qp(const std::vector<Node*>& path_nodes,
                                const Point_3& start_position,
                                const std::optional<Point_3>& goal_position,
                                const ReachabilityModel& reachability,
                                const FootstepQPConfig& config,
                                QPBackend& backend);

} // namespace nas
