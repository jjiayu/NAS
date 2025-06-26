#include "footstep_planner.hpp"
#include <casadi/casadi.hpp>
#include <iostream>

namespace nas {

FootstepPlanner::FootstepPlanner() {

    std::cout << "\n[ Initializing Footstep Planner ]" << std::endl;

    // Load the reachability polytope and convert to half-space constraint
    load_obj(rf_in_lf_path_forward, this->rf_in_lf_polytope);
    this->rf_in_lf_constraint = convert_polytope_to_half_space_constraint(this->rf_in_lf_polytope);

    load_obj(lf_in_rf_path_forward, this->lf_in_rf_polytope);
    this->lf_in_rf_constraint = convert_polytope_to_half_space_constraint(this->lf_in_rf_polytope);

}

FootstepPlanner::~FootstepPlanner() {
    // Destructor implementation (can be empty if no special cleanup needed)
}

void FootstepPlanner::plan(const std::vector<Node*>& path_nodes) {
    std::cout << "\n[ Planning Footsteps ]" << std::endl;

    // Create the QP problem using the direct qpsol interface
    using namespace casadi;

    // Create symbolic variables
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    
    // Create QP problem: minimize x^2 + y^2 subject to x + y = 10
    SXDict qp;
    qp["x"] = vertcat(x, y);           // Decision variables [x, y]
    qp["f"] = x*x + y*y;               // Quadratic objective
    qp["g"] = x + y - 10;              // Constraint: x + y - 10 = 0

    // Create QP solver using qpoases
    Function S = qpsol("S", "qpoases", qp);
    
    std::cout << "QP Solver (qpoases) created successfully!" << std::endl;

    try {
        // Solve the problem
        std::map<std::string, DM> arg;
        arg["x0"] = DM::zeros(2);     // Initial guess [x0, y0] = [0, 0]
        arg["lbg"] = DM::zeros(1);    // Lower bound on constraint: x + y - 10 = 0
        arg["ubg"] = DM::zeros(1);    // Upper bound on constraint: x + y - 10 = 0
        
        std::map<std::string, DM> res = S(arg);

        // Print results
        std::cout << "Solution: x = " << res["x"](0) << ", y = " << res["x"](1) << std::endl;
        std::cout << "Objective value: " << res["f"](0) << std::endl;
        std::cout << "Constraint check: x + y = " << (static_cast<double>(res["x"](0)) + static_cast<double>(res["x"](1))) << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Optimization failed: " << e.what() << std::endl;
    }
}

}