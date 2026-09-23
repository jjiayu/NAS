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
};

struct FootstepPlan {
    bool success = false;
    std::vector<Point_3> footsteps; // one per path_nodes entry, world frame
    double alpha = 0.0;
    double max_violation = 0.0;     // largest constraint residual of the returned solution (<= feasibility_tolerance when success)
    double objective = 0.0;         // 0.5*x'*H*x + g'*x at the solution, H unregularized (paper Eq. 6) — lets backends be compared on the value they actually optimize, not just feasibility
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
