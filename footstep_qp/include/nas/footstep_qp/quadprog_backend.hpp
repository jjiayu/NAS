#pragma once

// QuadprogBackend — the default/safe QPBackend (see PLAN.md "Décisions
// clés": quadprog first, ProxQP optional later, CasADi temporary). Wraps
// eiquadprog (LAAS/Gepetto, Eigen-native, already a transitive dependency
// via tsid in this conda env — same lineage as Pinocchio/coal).
//
// eiquadprog's solve_quadprog has a different convention than QPProblem's:
//   min x'*Hess*x + 2*g0'*x   s.t. CE*x + ce0 = 0, CI*x + ci0 >= 0
// vs. QPProblem's min 0.5*x'*H*x + g'*x s.t. A_eq*x = b_eq, A_ineq*x <= b_ineq.
// The conversion (Hess=0.5*H, g0=0.5*g, CE=A_eq, ce0=-b_eq, CI=-A_ineq,
// ci0=b_ineq) is done once in the .cpp and covered by
// tests/test_quadprog_backend.cpp on a hand-solvable problem before it's
// ever trusted on the real footstep formulation.

#include "nas/footstep_qp/qp_backend.hpp"

namespace nas {

class QuadprogBackend : public QPBackend {
public:
    QPSolution solve(const QPProblem& problem) override;
};

} // namespace nas
