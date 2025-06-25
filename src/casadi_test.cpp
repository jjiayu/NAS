#include <casadi/casadi.hpp>
#include <iostream>

int main() {
    using namespace casadi;

    // Create symbolic variables
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    
    // Create QP problem: minimize x^2 + y^2 subject to x + y = 10
    SXDict qp;
    qp["x"] = vertcat(x, y);
    qp["f"] = x*x + y*y;
    qp["g"] = x + y - 10;
    
    // Create QP solver
    Function S = qpsol("S", "qpoases", qp);
    
    // Print solver info
    std::cout << "QP Solver created successfully!" << std::endl;
    std::cout << "Solver: " << S << std::endl;

    // Solve the problem
    std::map<std::string, DM> arg;
    arg["x0"] = DM::zeros(2);  // Initial guess
    arg["lbg"] = DM::zeros(1); // Lower bound on constraint: x + y - 10 = 0
    arg["ubg"] = DM::zeros(1); // Upper bound on constraint: x + y - 10 = 0
    
    std::map<std::string, DM> res = S(arg);
    
    // Print results
    std::cout << "\nSolution:" << std::endl;
    std::cout << "x = " << res["x"](0) << std::endl;
    std::cout << "y = " << res["x"](1) << std::endl;
    std::cout << "Objective value = " << res["f"] << std::endl;
    std::cout << "Constraint value = " << res["g"] << std::endl;
    
    // Verify the solution
    double x_val = res["x"](0).scalar();
    double y_val = res["x"](1).scalar();
    std::cout << "\nVerification:" << std::endl;
    std::cout << "x + y = " << x_val + y_val << " (should be 10)" << std::endl;
    std::cout << "x^2 + y^2 = " << x_val*x_val + y_val*y_val << " (should equal objective)" << std::endl;

    return 0;
}