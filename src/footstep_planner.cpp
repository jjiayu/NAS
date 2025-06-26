#include "footstep_planner.hpp"
#include <casadi/casadi.hpp>
#include <iostream>

namespace nas {

FootstepPlanner::FootstepPlanner() {

    std::cout << "\n[ Initializing Footstep Planner ]" << std::endl;

    load_obj(rf_in_lf_path_forward, this->rf_in_lf_polytope);
    this->rf_in_lf_constraint = convert_polytope_to_half_space_constraint(this->rf_in_lf_polytope);

    // Convert Eigen matrices to CasADi SX (for mtimes compatibility)
    //   Right foot in Left Foot constraint polytope
    this->A_rf_in_lf_casadi = casadi::SX::zeros(rf_in_lf_constraint.A.rows(), rf_in_lf_constraint.A.cols());
    this->b_rf_in_lf_casadi = casadi::SX::zeros(rf_in_lf_constraint.b.size());

    for (int i = 0; i < rf_in_lf_constraint.A.rows(); ++i) {
        for (int j = 0; j < rf_in_lf_constraint.A.cols(); ++j) {
            this->A_rf_in_lf_casadi(i, j) = rf_in_lf_constraint.A(i, j);
        }
        this->b_rf_in_lf_casadi(i) = rf_in_lf_constraint.b(i);
    }
    std::cout << "- Right foot in Left Foot constraint polytope: " << A_rf_in_lf_casadi.rows() << " x " << A_rf_in_lf_casadi.columns() << std::endl;

    // Left foot in Right Foot constraint polytope
    
    load_obj(lf_in_rf_path_forward, this->lf_in_rf_polytope);
    this->lf_in_rf_constraint = convert_polytope_to_half_space_constraint(this->lf_in_rf_polytope);

    this->A_lf_in_rf_casadi = casadi::SX::zeros(lf_in_rf_constraint.A.rows(), lf_in_rf_constraint.A.cols());
    this->b_lf_in_rf_casadi = casadi::SX::zeros(lf_in_rf_constraint.b.size());
    
    for (int i = 0; i < lf_in_rf_constraint.A.rows(); ++i) {
        for (int j = 0; j < lf_in_rf_constraint.A.cols(); ++j) {
            this->A_lf_in_rf_casadi(i, j) = lf_in_rf_constraint.A(i, j);
        }
        this->b_lf_in_rf_casadi(i) = lf_in_rf_constraint.b(i);
    }

    std::cout << "- Left foot in Right Foot constraint polytope: " << A_lf_in_rf_casadi.rows() << " x " << A_lf_in_rf_casadi.columns() << std::endl;

}

FootstepPlanner::~FootstepPlanner() {
    // Destructor implementation (can be empty if no special cleanup needed)
}

void FootstepPlanner::plan(const int& stance_foot_flag_at_start, 
                           const Point_3& stance_foot_position_at_start, 
                           const int& stance_foot_flag_at_goal,
                           const Point_3& stance_foot_position_at_goal,
                           const std::vector<Node*>& path_nodes) {
                             
    std::cout << "\n[ Planning Footsteps ]" << std::endl;

    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size() - 1; footstep_cnt++) {
    }

    // // Decision variables: [x, y, z] position of footstep
    // casadi::SX x = casadi::SX::sym("x");
    // casadi::SX y = casadi::SX::sym("y");
    // casadi::SX z = casadi::SX::sym("z");
    // casadi::SX vars = vertcat(x, y, z);
    
    // // Quadratic objective: minimize deviation from target position
    // // For demo: minimize x^2 + y^2 + z^2 (minimize distance from origin)
    // casadi::SX objective = x*x + y*y + z*z;
    
    // // Create constraint: A*x <= b  becomes  A*x - b <= 0
    // // Now all SX types - no ambiguity!
    // casadi::SX constraints = mtimes(A_rf_in_lf_casadi, vars) - b_rf_in_lf_casadi;
    
    // // Formulate QP problem
    // casadi::SXDict qp;
    // qp["x"] = vars;                    // Decision variables [x, y, z]
    // qp["f"] = objective;               // Quadratic objective
    // qp["g"] = constraints;             // Inequality constraints: A*x <= b
    
    // // Create QP solver
    // casadi::Function solver = casadi::qpsol("footstep_qp", "qpoases", qp);
    // std::cout << "QP Solver created with polytope constraints!" << std::endl;
    
    // try {
    //     // Setup solver arguments
    //     std::map<std::string, casadi::DM> arg;
    //     arg["x0"] = casadi::DM::zeros(3);                           // Initial guess [x0, y0, z0]
    //     arg["lbg"] = -casadi::DM::inf(constraints.size1());         // Lower bounds: -inf (no lower bounds on Ax <= b)
    //     arg["ubg"] = casadi::DM::zeros(constraints.size1());        // Upper bounds: 0 (for Ax <= b)
        
    //     // Add variable bounds if needed (optional)
    //     arg["lbx"] = casadi::DM::vertcat({-10, -10, -10});          // Variable lower bounds
    //     arg["ubx"] = casadi::DM::vertcat({10, 10, 10});             // Variable upper bounds
        
    //     // Solve the QP
    //     std::map<std::string, casadi::DM> result = solver(arg);
        
    //     // Check solver success
    //     bool solve_successful = true;
    //     std::string solver_status = "unknown";
        
    //     // Method 1: Check solver statistics
    //     auto stats = solver.stats();
    //     if (stats.find("return_status") != stats.end()) {
    //         solver_status = static_cast<std::string>(stats.at("return_status"));
    //         std::cout << "Solver status: " << solver_status << std::endl;
            
    //         // qpoases success indicators
    //         if (solver_status == "SUCCESS" || 
    //             solver_status == "SOLVED" || 
    //             solver_status == "Successful return." ||
    //             solver_status.find("Successful") != std::string::npos) {
    //             solve_successful = true;
    //             std::cout << "✅ Optimization SUCCESSFUL!" << std::endl;
    //         } else {
    //             solve_successful = false;
    //             std::cout << "❌ Optimization FAILED: " << solver_status << std::endl;
    //         }
    //     }
        
    //     // Method 2: Check if solution exists and is finite
    //     if (result.find("x") == result.end()) {
    //         solve_successful = false;
    //         std::cout << "❌ No solution returned!" << std::endl;
    //     } else {
    //         casadi::DM x_sol = result["x"];
    //         bool solution_finite = true;
    //         for (int i = 0; i < x_sol.size1(); ++i) {
    //             double val = static_cast<double>(x_sol(i));
    //             if (!std::isfinite(val)) {
    //                 solution_finite = false;
    //                 break;
    //             }
    //         }
    //         if (!solution_finite) {
    //             solve_successful = false;
    //             std::cout << "❌ Solution contains infinite/NaN values!" << std::endl;
    //         }
    //     }
        
    //     // Method 3: Display solver statistics
    //     std::cout << "\n=== Solver Statistics ===" << std::endl;
    //     for (const auto& stat : stats) {
    //         std::cout << stat.first << ": " << stat.second << std::endl;
    //     }
        
    //     if (!solve_successful) {
    //         std::cout << "\n❌ QP Solve FAILED - cannot proceed" << std::endl;
    //         return;
    //     }
        
    //     // Extract and display results (only if successful)
    //     casadi::DM x_opt = result["x"];
    //     double obj_val = static_cast<double>(result["f"]);
        
    //     std::cout << "\n=== QP Solution ===" << std::endl;
    //     std::cout << "Optimal footstep position:" << std::endl;
    //     std::cout << "  x = " << x_opt(0) << std::endl;
    //     std::cout << "  y = " << x_opt(1) << std::endl;
    //     std::cout << "  z = " << x_opt(2) << std::endl;
    //     std::cout << "Objective value: " << obj_val << std::endl;
        
    //     // Verify constraints - convert to DM for numerical evaluation
    //     casadi::DM A_rf_num = casadi::DM::zeros(rf_in_lf_constraint.A.rows(), rf_in_lf_constraint.A.cols());
    //     casadi::DM b_rf_num = casadi::DM::zeros(rf_in_lf_constraint.b.size());
        
    //     for (int i = 0; i < rf_in_lf_constraint.A.rows(); ++i) {
    //         for (int j = 0; j < rf_in_lf_constraint.A.cols(); ++j) {
    //             A_rf_num(i, j) = rf_in_lf_constraint.A(i, j);
    //         }
    //         b_rf_num(i) = rf_in_lf_constraint.b(i);
    //     }
        
    //     // Use proper matrix multiplication for DM - need to be careful about dimensions
    //     // A_rf_num is (n_constraints x 3), x_opt is (3 x 1), result should be (n_constraints x 1)
    //     std::cout << "Matrix dimensions: A=" << A_rf_num.rows() << "x" << A_rf_num.columns() 
    //               << ", x=" << x_opt.rows() << "x" << x_opt.columns() 
    //               << ", b=" << b_rf_num.rows() << "x" << b_rf_num.columns() << std::endl;
        
    //     casadi::DM constraint_values = casadi::DM::mtimes(A_rf_num, x_opt) - b_rf_num;
        
    //     // Method 4: Check constraint satisfaction
    //     std::cout << "\n=== Constraint Verification ===" << std::endl;
    //     bool constraints_satisfied = true;
    //     double constraint_tolerance = 1e-6;
        
    //     for (int i = 0; i < constraint_values.size1(); ++i) {
    //         double g_val = static_cast<double>(constraint_values(i));
    //         std::cout << "  g[" << i << "] = " << g_val;
            
    //         if (g_val > constraint_tolerance) {
    //             std::cout << " ❌ VIOLATED (should be ≤ 0)";
    //             constraints_satisfied = false;
    //         } else {
    //             std::cout << " ✅ satisfied";
    //         }
    //         std::cout << std::endl;
    //     }
        
    //     if (constraints_satisfied) {
    //         std::cout << "✅ All constraints satisfied within tolerance " << constraint_tolerance << std::endl;
    //     } else {
    //         std::cout << "❌ Some constraints violated! Solution may be infeasible." << std::endl;
    //     }
        
    // } catch (const std::exception& e) {
    //     std::cout << "QP Optimization failed: " << e.what() << std::endl;
    // }
}

}