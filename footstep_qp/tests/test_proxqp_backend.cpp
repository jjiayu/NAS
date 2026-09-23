// Validates the QPProblem <-> proxsuite convention conversion on
// hand-solvable problems, before this backend is ever trusted on the real
// footstep formulation.

#include "nas/footstep_qp/proxqp_backend.hpp"

#include <iostream>
#include <string>

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

bool close(double a, double b, double eps = 1e-6) { return std::abs(a - b) < eps; }

// minimize (x-1)^2 + (y-2)^2, unconstrained optimum is exactly (1,2).
void test_unconstrained_optimum_is_feasible() {
    QPProblem p;
    p.H = Eigen::MatrixXd::Identity(2, 2) * 2.0;
    p.g = Eigen::Vector2d(-2.0, -4.0);
    // x + y <= 10 (not active at the unconstrained optimum), x >= 0, y >= 0.
    p.A_ineq.resize(3, 2);
    p.A_ineq << 1, 1, -1, 0, 0, -1;
    p.b_ineq.resize(3);
    p.b_ineq << 10, 0, 0;

    ProxqpBackend backend;
    QPSolution sol = backend.solve(p);
    check(sol.success, "solve succeeds (inactive inequality)");
    if (sol.success) {
        // proxsuite is an iterative (ADMM) solver, accurate to eps_abs (tightened to 1e-9 in
        // the backend, see proxqp_backend.cpp) rather than exactly, unlike eiquadprog's
        // active-set method — hence the slightly looser tolerance than QuadprogBackend's test.
        check(close(sol.x(0), 1.0, 1e-6) && close(sol.x(1), 2.0, 1e-6),
              "matches the analytical unconstrained optimum (1, 2)");
    }
}

// Same objective, but x + y <= 2 is now active -> analytical optimum is
// (0.5, 1.5) (projection of (1,2) onto the constraint line via Lagrange
// multipliers, worked out by hand).
void test_active_inequality_constraint() {
    QPProblem p;
    p.H = Eigen::MatrixXd::Identity(2, 2) * 2.0;
    p.g = Eigen::Vector2d(-2.0, -4.0);
    p.A_ineq.resize(3, 2);
    p.A_ineq << 1, 1, -1, 0, 0, -1;
    p.b_ineq.resize(3);
    p.b_ineq << 2, 0, 0;

    ProxqpBackend backend;
    QPSolution sol = backend.solve(p);
    check(sol.success, "solve succeeds (active inequality)");
    if (sol.success) {
        check(close(sol.x(0), 0.5, 1e-4) && close(sol.x(1), 1.5, 1e-4),
              "matches the analytical constrained optimum (0.5, 1.5)");
    }
}

// Same objective under an equality constraint x = y (forces x=y=1.5 by
// symmetry of the two quadratic terms... actually the weighted optimum
// under x=y for min (x-1)^2+(x-2)^2 is x=1.5 by direct calculus).
void test_equality_constraint() {
    QPProblem p;
    p.H = Eigen::MatrixXd::Identity(2, 2) * 2.0;
    p.g = Eigen::Vector2d(-2.0, -4.0);
    p.A_eq.resize(1, 2);
    p.A_eq << 1, -1;
    p.b_eq.resize(1);
    p.b_eq << 0; // x - y = 0 -> x == y

    ProxqpBackend backend;
    QPSolution sol = backend.solve(p);
    check(sol.success, "solve succeeds (equality constraint)");
    if (sol.success) {
        check(close(sol.x(0), 1.5, 1e-6) && close(sol.x(1), 1.5, 1e-6),
              "matches the analytical equality-constrained optimum (1.5, 1.5)");
    }
}

// x >= 5 and x <= 1 simultaneously -> infeasible.
void test_infeasible_problem_reports_failure() {
    QPProblem p;
    p.H = Eigen::MatrixXd::Identity(1, 1) * 2.0;
    p.g = Eigen::VectorXd::Zero(1);
    p.A_ineq.resize(2, 1);
    p.A_ineq << -1, 1; // -x <= -5 (x>=5), x <= 1
    p.b_ineq.resize(2);
    p.b_ineq << -5, 1;

    ProxqpBackend backend;
    QPSolution sol = backend.solve(p);
    check(!sol.success, "infeasible problem is reported as failure, not garbage output");
}

} // namespace

int main() {
    test_unconstrained_optimum_is_feasible();
    test_active_inequality_constraint();
    test_equality_constraint();
    test_infeasible_problem_reports_failure();

    if (g_failures > 0) {
        std::cerr << g_failures << " test(s) FAILED\n";
        return 1;
    }
    std::cout << "All ProxqpBackend tests passed\n";
    return 0;
}
