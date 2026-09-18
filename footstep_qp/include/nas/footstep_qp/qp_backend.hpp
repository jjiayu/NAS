#pragma once

// QPBackend — solver-agnostic QP interface (see PLAN.md phase 7). The old
// FootstepPlanner built a CasADi symbolic expression tree and always solved
// it via qpoases; CasADi's autodiff was never actually exploited (every
// Jacobian in the old formulation was already hand-built), so here the
// formulation (footstep_qp.hpp) builds plain Eigen matrices once and any
// backend can solve them.
//
// Convention: minimize 0.5 x'Hx + g'x subject to A_eq x = b_eq and
// A_ineq x <= b_ineq. Variable bounds (e.g. alpha >= 0) are just rows of
// A_ineq/b_ineq — no separate bounds vector, so every backend only needs to
// handle one constraint shape.

#include <Eigen/Dense>

namespace nas {

struct QPProblem {
    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;
};

struct QPSolution {
    bool success = false;
    Eigen::VectorXd x;
};

class QPBackend {
public:
    virtual ~QPBackend() = default;
    virtual QPSolution solve(const QPProblem& problem) = 0;
};

} // namespace nas
