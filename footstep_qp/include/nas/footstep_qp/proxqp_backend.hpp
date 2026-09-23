#pragma once

// ProxqpBackend — alternative QPBackend, compared against QuadprogBackend
// (see PLAN.md "décisions clés" and the TODO on library cleanup once a
// backend is chosen). Wraps proxsuite's dense QP (already a transitive
// conda dependency via tsid/pinocchio's solver lineage).
//
// proxsuite's dense::QP uses l <= Cx <= u for inequalities, vs. QPProblem's
// one-sided A_ineq*x <= b_ineq. The conversion (C = A_ineq, u = b_ineq,
// l = -infinity) is done once in the .cpp and covered by
// tests/test_proxqp_backend.cpp on the same hand-solvable problems as
// QuadprogBackend before it's ever trusted on the real footstep formulation.

#include "nas/footstep_qp/qp_backend.hpp"

namespace nas {

class ProxqpBackend : public QPBackend {
public:
    QPSolution solve(const QPProblem& problem) override;
};

} // namespace nas
