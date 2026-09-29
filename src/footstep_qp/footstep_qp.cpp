#include "nas/footstep_qp/footstep_qp.hpp"
#include "nas/core/expansion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

// Q of the paper (Eq. 2) for the support foot of path node `node`: yaw composed with the tilt of its contact
// surface. Flat surface: the yaw-only matrix, exactly as before.
Eigen::Matrix3d support_frame_rotation(const Node& node, bool rotation_enabled) {
    const Vector_3 n = node.up_normal();
    if (is_vertical_normal(n)) return yaw_rotation_matrix(node.foot_yaw, rotation_enabled);
    return foot_frame_rotation(n, rotation_enabled ? node.foot_yaw : 0.0);
}

Eigen::Vector3d to_eigen(const Point_3& p) {
    return Eigen::Vector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
}

} // namespace

FootstepPlan solve_footstep_qp(const std::vector<Node*>& path_nodes,
                                const Point_3& start_position,
                                const std::optional<Point_3>& goal_position,
                                const ReachabilityModel& reachability,
                                const FootstepQPConfig& config,
                                QPBackend& backend) {
    const int n = static_cast<int>(path_nodes.size());
    const int num_cubes = static_cast<int>(config.cube_placements.size());
    if (num_cubes > 0 && config.cube_half_extent <= 0.0) {
        throw std::invalid_argument("solve_footstep_qp: cube_placements given without a positive cube_half_extent");
    }
    for (const auto& cp : config.cube_placements) {
        if (cp.support_index >= static_cast<std::size_t>(n) || cp.onto_index >= static_cast<std::size_t>(n)) {
            throw std::invalid_argument("solve_footstep_qp: a CubePlacement index is out of range");
        }
    }
    const int alpha_idx = 3 * n;
    const int cube_col0 = 3 * n + 1;  // first cube's own 3 columns start here, one cube after another
    const int dim = cube_col0 + 3 * num_cubes;
    auto cube_col = [&](int k) { return cube_col0 + 3 * k; };

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
    const Eigen::MatrixXd H_base = qp.H;
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
        // Cached (built once per (moving, support) pair, not rebuilt from convex_hull_3 on every
        // step) — see ReachabilityModel::half_space_constraint.
        const HalfSpacePolytopeConstraint& hrep =
            reachability.half_space_constraint(effector_name(moving), effector_name(support), ReachabilityDirection::Forward);
        Eigen::Matrix3d R_yaw = support_frame_rotation(*path_nodes[i - 1], config.rotation_enabled);

        for (int r = 0; r < hrep.A.rows(); ++r) {
            Eigen::RowVector3d a_rotated = hrep.A.row(r) * R_yaw.transpose();
            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
            row.segment<3>(idx(i, 0)) = a_rotated;
            row.segment<3>(idx(i - 1, 0)) = -a_rotated;
            add_ineq(row, hrep.b(r));
        }
    }

    // Row 0 of generate_surface_constraint is the plane equality, the rest are boundary
    // inequalities with the alpha robustness margin - shared by an intermediate footstep's own
    // patch, a config.goal_constraints polytope slot, and a config.cube_placements patch further
    // down (col: the 3-column block the constraint applies to - a footstep's own idx(i, 0), or a
    // cube's cube_col(k)).
    auto add_region_constraint_at = [&](int col, const std::vector<Point_3>& vertices) {
        SurfaceConstraint sc = generate_surface_constraint(vertices);
        for (int r = 0; r < sc.A.rows(); ++r) {
            double row_norm = sc.A.row(r).norm();
            Eigen::RowVector3d a_normalized = (row_norm > 1e-12) ? (sc.A.row(r) / row_norm).eval() : sc.A.row(r).eval();
            double b_normalized = (row_norm > 1e-12) ? sc.b(r) / row_norm : sc.b(r);

            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
            row.segment<3>(col) = a_normalized;

            if (r == 0) {
                add_eq(row, b_normalized);
            } else {
                row(alpha_idx) = 1.0;
                add_ineq(row, b_normalized);
            }
        }
    };
    auto add_region_constraint = [&](int i, const std::vector<Point_3>& vertices) {
        add_region_constraint_at(idx(i, 0), vertices);
    };

    // A node claimed by goal_position (path_nodes.back(), when set) or by a config.goal_constraints
    // entry is constrained exactly once, by that mechanism - not also by its own raw search patch
    // here (a strict superset of any foot_goals polytope slot, or an unrelated conflicting plane fit
    // for an exact-point slot).
    std::vector<bool> claimed(static_cast<size_t>(n), false);
    if (goal_position) claimed[static_cast<size_t>(n - 1)] = true;
    for (const auto& gc : config.goal_constraints) claimed[gc.node_index] = true;

    // --- Surface constraints for every intermediate step not claimed above ---
    for (int i = 1; i < n; ++i) {
        if (claimed[static_cast<size_t>(i)]) continue;
        add_region_constraint(i, path_nodes[i]->patch_vertices);
    }

    // --- Initial/final footstep equality constraints ---
    Eigen::Vector3d start_eigen = to_eigen(start_position);
    for (int c = 0; c < 3; ++c) {
        Eigen::RowVectorXd row0 = Eigen::RowVectorXd::Zero(dim);
        row0(idx(0, c)) = 1.0;
        add_eq(row0, start_eigen(c));
    }
    if (goal_position) {
        Eigen::Vector3d goal_eigen = to_eigen(*goal_position);
        for (int c = 0; c < 3; ++c) {
            Eigen::RowVectorXd rowN = Eigen::RowVectorXd::Zero(dim);
            rowN(idx(n - 1, c)) = 1.0;
            add_eq(rowN, goal_eigen(c));
        }
    }

    // --- config.goal_constraints: a point (equality, same as goal_position but at an arbitrary
    // node index) or a polytope (region membership, same mechanism as an intermediate patch above,
    // just on the goal polytope's own vertices instead of that node's full search patch) ---
    for (const auto& gc : config.goal_constraints) {
        int i = static_cast<int>(gc.node_index);
        if (std::holds_alternative<Point_3>(gc.region)) {
            Eigen::Vector3d p = to_eigen(std::get<Point_3>(gc.region));
            for (int c = 0; c < 3; ++c) {
                Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
                row(idx(i, c)) = 1.0;
                add_eq(row, p(c));
            }
        } else {
            add_region_constraint(i, std::get<std::vector<Point_3>>(gc.region));
        }
    }

    // --- config.cube_placements: couples each cube's base center into this same QP (see the
    // struct's own doc comment in footstep_qp.hpp for why) ---
    for (int k = 0; k < num_cubes; ++k) {
        const auto& cp = config.cube_placements[static_cast<std::size_t>(k)];
        int support_i = static_cast<int>(cp.support_index);
        int onto_i = static_cast<int>(cp.onto_index);
        int col = cube_col(k);

        // (a) placement polytope, in the support footstep's own frame - same mechanism as the
        // ordinary reachability constraint above, just "Cube" as the moving effector instead of a
        // StanceFoot, and the child variable is this cube's column block instead of idx(i, 0).
        StanceFoot support_foot = path_nodes[support_i]->stance_foot;
        const HalfSpacePolytopeConstraint& cube_hrep =
            reachability.half_space_constraint("Cube", effector_name(support_foot), ReachabilityDirection::Forward);
        Eigen::Matrix3d R_support = support_frame_rotation(*path_nodes[support_i], config.rotation_enabled);
        for (int r = 0; r < cube_hrep.A.rows(); ++r) {
            Eigen::RowVector3d a_rotated = cube_hrep.A.row(r) * R_support.transpose();
            Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
            row.segment<3>(col) = a_rotated;
            row.segment<3>(idx(support_i, 0)) = -a_rotated;
            add_ineq(row, cube_hrep.b(r));
        }

        // (b) the search's own placement patch (same mechanism as an intermediate footstep's own
        // patch: a plane equality pinning this cube's z, plus alpha-margined boundary inequalities).
        add_region_constraint_at(col, cp.placement_patch);

        // (c) the coupling itself: onto_i's footstep must land within the box's own
        // half_extent x half_extent top face, oriented like the support footstep -
        // |R_support^T (x_onto - c)| <= half_extent - alpha, axis by axis (place_box's own
        // formula, generalized to a QP variable c instead of an LP-fixed x_onto).
        for (int axis = 0; axis < 2; ++axis) {
            for (double sign : {1.0, -1.0}) {
                Eigen::RowVector3d row_dir = sign * R_support.col(axis).transpose();
                Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
                row.segment<3>(idx(onto_i, 0)) = row_dir;
                row.segment<3>(col) = -row_dir;
                row(alpha_idx) = 1.0;
                add_ineq(row, config.cube_half_extent);
            }
        }
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

    // Solve, then check the residuals: "optimal" from the solver is not enough (see
    // FootstepQPConfig::feasibility_tolerance).
    auto max_residual = [&](const Eigen::VectorXd& x) {
        double v = 0.0;
        if (qp.A_ineq.rows() > 0) v = std::max(v, (qp.A_ineq * x - qp.b_ineq).maxCoeff());
        if (qp.A_eq.rows() > 0) v = std::max(v, (qp.A_eq * x - qp.b_eq).cwiseAbs().maxCoeff());
        return v;
    };
    QPSolution solution;
    double violation = std::numeric_limits<double>::infinity();
    double regularization = config.hessian_regularization;
    for (int attempt = 0; attempt < 4; ++attempt) {
        qp.H = H_base + regularization * Eigen::MatrixXd::Identity(dim, dim);
        solution = backend.solve(qp);
        if (solution.success) {
            violation = max_residual(solution.x);
            if (violation <= config.feasibility_tolerance) break;
        }
        regularization *= 100.0;
    }
    const bool feasible = solution.success && violation <= config.feasibility_tolerance;

    FootstepPlan plan;
    plan.success = feasible;
    plan.max_violation = std::isfinite(violation) ? violation : 0.0;
    if (feasible) {
        plan.objective = 0.5 * solution.x.dot(H_base * solution.x) + qp.g.dot(solution.x);
        plan.alpha = solution.x(alpha_idx);
        plan.footsteps.reserve(n);
        for (int i = 0; i < n; ++i) {
            plan.footsteps.emplace_back(solution.x(idx(i, 0)), solution.x(idx(i, 1)), solution.x(idx(i, 2)));
        }
        plan.cube_centers.reserve(static_cast<std::size_t>(num_cubes));
        for (int k = 0; k < num_cubes; ++k) {
            int col = cube_col(k);
            plan.cube_centers.emplace_back(solution.x(col), solution.x(col + 1), solution.x(col + 2));
        }
    }
    return plan;
}

} // namespace nas
