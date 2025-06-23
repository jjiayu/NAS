#include <casadi/casadi.hpp>
#include <iostream>

int main() {
    std::cout << "CasADi C++ Test" << std::endl;

    // Create a CasADi symbolic variable
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");

    // Define a simple function: f(x, y) = x^2 + y^2
    casadi::SX f = x*x + y*y;
    casadi::Function fun("fun", {x, y}, {f});

    // Evaluate at (3, 4) using explicit DM vector
    std::vector<casadi::DM> input = {casadi::DM(3), casadi::DM(4)};
    std::vector<casadi::DM> result = fun(input);
    std::cout << "f(3, 4) = " << result[0] << std::endl; // Should print 25

    return 0;
} 