#include "nas/footstep_qp/proxqp_backend.hpp"

#include <proxsuite/proxqp/dense/dense.hpp>

#include <limits>

namespace nas {

QPSolution ProxqpBackend::solve(const QPProblem& problem) {
    using namespace proxsuite::proxqp;

    const dense::isize n = static_cast<dense::isize>(problem.H.rows());
    const dense::isize n_eq = static_cast<dense::isize>(problem.A_eq.rows());
    const dense::isize n_in = static_cast<dense::isize>(problem.A_ineq.rows());

    dense::QP<double> qp(n, n_eq, n_in);
    qp.settings.verbose = false;
    // proxsuite's default eps_abs (1e-5) is coarser than FootstepQPConfig::feasibility_tolerance
    // (1e-6) — solve_footstep_qp's residual check would then reject an already-converged
    // solution as "not accurate enough". Tighten it so both backends are compared on equal
    // footing (eiquadprog's active-set method solves equality/active constraints exactly).
    qp.settings.eps_abs = 1e-9;
    // The footstep formulation's surface constraints come from patches with many boundary
    // vertices (core/expansion's clipped polygons), which can make A_ineq's rows nearly
    // parallel/redundant. Confirmed on LongStairsComplete (10 path nodes, 550 inequality rows
    // for 31 variables): with the default eps_primal_inf (1e-4), proxqp's infeasibility
    // certificate check fired a false positive (PROXQP_PRIMAL_INFEASIBLE at pri_res ~1.6e-5,
    // after ~30 iterations) on a problem eiquadprog solves with a 7e-9 residual — i.e. actually
    // feasible. Tightening eps_primal_inf (and eps_dual_inf, symmetrically) to match eps_abs
    // removes the false positive: proxqp then converges to the same optimum as eiquadprog
    // (verified: status PROXQP_SOLVED, pri_res 2.4e-11, objective matching to 1e-6).
    qp.settings.eps_primal_inf = 1e-9;
    qp.settings.eps_dual_inf = 1e-9;

    // Convention conversion — see the header comment. QPProblem is one-sided
    // (A_ineq*x <= b_ineq); proxsuite wants l <= C*x <= u, so l is -infinity.
    const double inf = std::numeric_limits<double>::infinity();
    Eigen::VectorXd l = Eigen::VectorXd::Constant(n_in, -inf);

    qp.init(problem.H, problem.g, problem.A_eq, problem.b_eq, problem.A_ineq, l, problem.b_ineq);
    qp.solve();

    QPSolution solution;
    solution.success = (qp.results.info.status == QPSolverOutput::PROXQP_SOLVED);
    if (solution.success) {
        solution.x = qp.results.x;
    }
    return solution;
}

} // namespace nas
