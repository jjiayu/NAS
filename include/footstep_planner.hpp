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

namespace nas {

class FootstepPlanner {
    public:
        // Reachability Polytope
        Polyhedron rf_in_lf_polytope;
        Polyhedron lf_in_rf_polytope;

        // Half-space polytope constraint
        HalfSpacePolytopeConstraint rf_in_lf_constraint;
        HalfSpacePolytopeConstraint lf_in_rf_constraint;

        FootstepPlanner();
        ~FootstepPlanner();

        void plan(const std::vector<Node*>& path_nodes);

};

}