#include "footstep_planner.hpp"

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

void FootstepPlanner::plan() {
    std::cout << "\n[ Planning Footsteps ]" << std::endl;
}

}