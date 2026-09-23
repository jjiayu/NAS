#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/core/expansion.hpp"

#include <cmath>

namespace nas {

namespace {

// 3x3 world-frame index of the block for path node i, coordinate c.
inline int idx(int i, int c) { return 3 * i + c; }

Eigen::Matrix3d yaw_rotation_matrix(double yaw, bool rotation_enabled) {
    if (!rotation_enabled) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix3d R;
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

Eigen::Vector3d to_eigen(const Point_3& p) {
    return Eigen::Vector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
}

} // namespace

FootstepPlan solve_footstep_qp(const std::vector<Node*>& path_nodes,
                                const Point_3& start_position,
                                const Point_3& goal_position,
                                const ReachabilityModel& reachability,
                                const FootstepQPConfig& config,
                                QPBackend& backend) {
    const int n = static_cast<int>(path_nodes.size());
    const int alpha_idx = 3 * n;
    const int dim = 3 * n + 1;

    QPProblem qp;
    qp.H = Eigen::MatrixXd::Zero(dim, dim);
    qp.g = Eigen::VectorXd::Zero(dim);

    // --- Objective: minimize sum ||stride_i||^2 - alpha_weight * alpha ---
    // stride_1 = x_1 - x_0; stride_i (i>=2) = x_i - x_{i-2}.
    for (int i = 1; i < n; ++i) {
        int j = (i == 1) ? 0 : i - 2;
        for (int c = 0; c < 3; ++c) {
            qp.H(idx(i, c), idx(i, c)) += 2.0;
            qp.H(idx(j, c), idx(j, c)) += 2.0;
            qp.H(idx(i, c), idx(j, c)) += -2.0;
            qp.H(idx(j, c), idx(i, c)) += -2.0;
        }
    }
    qp.H += config.hessian_regularization * Eigen::MatrixXd::Identity(dim, dim);
    qp.g(alpha_idx) = -config.alpha_weight;

    std::vector<Eigen::RowVectorXd> eq_rows;
    std::vector<double> eq_rhs;
    std::vector<Eigen::RowVectorXd> ineq_rows;
    std::vector<double> ineq_rhs;

    auto add_eq = [&](const Eigen::RowVectorXd& row, double rhs) {
        eq_rows.push_back(row);
        eq_rhs.push_back(rhs);
    };
    auto add_ineq = [&](const Eigen::RowVectorXd& row, double rhs) {
        ineq_rows.push_back(row);
        ineq_rhs.push_back(rhs);
    };

    // --- Reachability constraints: A * R_yaw^T * (x_i - x_{i-1}) <= b ---
    // (next foot in previous foot's polytope), for i = 1..n-1.
    for (int i = 1; i < n; ++i) {
        StanceFoot moving = path_nodes[i]->stance_foot;
        StanceFoot support = path_nodes[i - 1]->stance_foot;
        const Polyhedron& poly = reachability.query(effector_name(moving), effector_name(support),
                                                      ReachabilityDirection::Forward);
        HalfSpacePolytopeConstraint hrep = convert_polytope_to_half_space_constraint(poly);
        Eigen::Matrix3d R_yaw = yaw_rotation_matrix(path_nodes[i - 1]->foot_yaw, config.rotation_enabled);

        for (int r = 0; r < hrep.A.rows(); ++r) {
            Eigen::RowVector3d a_rotated = hrep.A.row(r) * R_yaw.transpose();
            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
            row.segment<3>(idx(i, 0)) = a_rotated;
            row.segment<3>(idx(i - 1, 0)) = -a_rotated;
            add_ineq(row, hrep.b(r));
        }
    }

    // --- Surface constraints for intermediate steps (1..n-2) ---
    // Row 0 of generate_surface_constraint is the plane equality, the rest
    // are boundary inequalities with the alpha robustness margin.
    for (int i = 1; i <= n - 2; ++i) {
        SurfaceConstraint sc = generate_surface_constraint(path_nodes[i]->patch_vertices);

        for (int r = 0; r < sc.A.rows(); ++r) {
            double row_norm = sc.A.row(r).norm();
            Eigen::RowVector3d a_normalized = (row_norm > 1e-12) ? (sc.A.row(r) / row_norm).eval() : sc.A.row(r).eval();
            double b_normalized = (row_norm > 1e-12) ? sc.b(r) / row_norm : sc.b(r);

            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
            row.segment<3>(idx(i, 0)) = a_normalized;

            if (r == 0) {
                add_eq(row, b_normalized);
            } else {
                row(alpha_idx) = 1.0;
                add_ineq(row, b_normalized);
            }
        }
    }

    // --- Initial/final footstep equality constraints ---
    Eigen::Vector3d start_eigen = to_eigen(start_position);
    Eigen::Vector3d goal_eigen = to_eigen(goal_position);
    for (int c = 0; c < 3; ++c) {
        Eigen::RowVectorXd row0 = Eigen::RowVectorXd::Zero(dim);
        row0(idx(0, c)) = 1.0;
        add_eq(row0, start_eigen(c));

        Eigen::RowVectorXd rowN = Eigen::RowVectorXd::Zero(dim);
        rowN(idx(n - 1, c)) = 1.0;
        add_eq(rowN, goal_eigen(c));
    }

    // --- alpha >= 0 ---
    {
        Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
        row(alpha_idx) = -1.0;
        add_ineq(row, 0.0);
    }

    qp.A_eq = Eigen::MatrixXd(eq_rows.size(), dim);
    qp.b_eq = Eigen::VectorXd(eq_rows.size());
    for (size_t r = 0; r < eq_rows.size(); ++r) {
        qp.A_eq.row(static_cast<int>(r)) = eq_rows[r];
        qp.b_eq(static_cast<int>(r)) = eq_rhs[r];
    }

    qp.A_ineq = Eigen::MatrixXd(ineq_rows.size(), dim);
    qp.b_ineq = Eigen::VectorXd(ineq_rows.size());
    for (size_t r = 0; r < ineq_rows.size(); ++r) {
        qp.A_ineq.row(static_cast<int>(r)) = ineq_rows[r];
        qp.b_ineq(static_cast<int>(r)) = ineq_rhs[r];
    }

    QPSolution solution = backend.solve(qp);

    FootstepPlan plan;
    plan.success = solution.success;
    if (solution.success) {
        plan.alpha = solution.x(alpha_idx);
        plan.footsteps.reserve(n);
        for (int i = 0; i < n; ++i) {
            plan.footsteps.emplace_back(solution.x(idx(i, 0)), solution.x(idx(i, 1)), solution.x(idx(i, 2)));
        }
    }
    return plan;
}

} // namespace nas
