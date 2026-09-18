#include "nas/footstep_qp/quadprog_backend.hpp"

#include <eiquadprog/eiquadprog-fast.hpp>

namespace nas {

QPSolution QuadprogBackend::solve(const QPProblem& problem) {
    using namespace eiquadprog::solvers;

    const int n = static_cast<int>(problem.H.rows());
    const int n_eq = static_cast<int>(problem.A_eq.rows());
    const int n_ineq = static_cast<int>(problem.A_ineq.rows());

    // Convention conversion — see the header comment.
    Eigen::MatrixXd Hess = 0.5 * problem.H;
    Eigen::VectorXd g0 = 0.5 * problem.g;
    Eigen::MatrixXd CE = problem.A_eq;
    Eigen::VectorXd ce0 = -problem.b_eq;
    Eigen::MatrixXd CI = -problem.A_ineq;
    Eigen::VectorXd ci0 = problem.b_ineq;

    EiquadprogFast solver;
    solver.reset(static_cast<size_t>(n), static_cast<size_t>(n_eq), static_cast<size_t>(n_ineq));

    Eigen::VectorXd x(n);
    EiquadprogFast_status status = solver.solve_quadprog(Hess, g0, CE, ce0, CI, ci0, x);

    QPSolution solution;
    solution.success = (status == EIQUADPROG_FAST_OPTIMAL);
    if (solution.success) {
        solution.x = x;
    }
    return solution;
}

} // namespace nas
