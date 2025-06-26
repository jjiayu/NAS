#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "constants.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "tree.hpp"
#include "astar_search.hpp"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include <casadi/casadi.hpp>

namespace nas {

class FootstepPlanner {
    public:
        // Reachability Polytope
        Polyhedron rf_in_lf_polytope;
        Polyhedron lf_in_rf_polytope;

        // Half-space polytope constraint
        HalfSpacePolytopeConstraint rf_in_lf_constraint;
        HalfSpacePolytopeConstraint lf_in_rf_constraint;

        // CasADi matrices will be created in the implementation file
        // No need to expose CasADi types in the header
        casadi::SX A_rf_in_lf_casadi;
        casadi::SX b_rf_in_lf_casadi;
        casadi::SX A_lf_in_rf_casadi;
        casadi::SX b_lf_in_rf_casadi;
        
        // Store computed footsteps for visualization
        std::vector<Point_3> computed_footsteps;

        FootstepPlanner();
        ~FootstepPlanner();

        void plan(const int& stance_foot_flag_at_start, 
                  const Point_3& stance_foot_position_at_start, 
                  const int& stance_foot_flag_at_goal,
                  const Point_3& stance_foot_position_at_goal,
                  const std::vector<Node*>& path_nodes);
                  
        // Get the computed footsteps for visualization
        const std::vector<Point_3>& get_computed_footsteps() const { return computed_footsteps; }

};

}