#include "footstep_planner.hpp"
#include "constants.hpp"
#include <casadi/casadi.hpp>
#include <iostream>
#include <limits>

namespace nas {

FootstepPlanner::FootstepPlanner() {

    std::cout << "\n[ Initializing Footstep Planner ]" << std::endl;

    // Left foot in Right Foot constraint forward polytope
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

    // Left foot in Right Foot constraint forward polytope
    
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

    // CoM in Left Foot constraint forward polytope
    load_obj(com_in_lf_path_forward, this->com_in_lf_polytope);
    this->com_in_lf_constraint = convert_polytope_to_half_space_constraint(this->com_in_lf_polytope);

    // CoM in Right Foot constraint forward polytope
    load_obj(com_in_rf_path_forward, this->com_in_rf_polytope);
    this->com_in_rf_constraint = convert_polytope_to_half_space_constraint(this->com_in_rf_polytope);

    // Convert Eigen matrices to CasADi SX (for mtimes compatibility)
    //   CoM in Left Foot constraint polytope
    this->A_com_in_lf_casadi = casadi::SX::zeros(com_in_lf_constraint.A.rows(), com_in_lf_constraint.A.cols());
    this->b_com_in_lf_casadi = casadi::SX::zeros(com_in_lf_constraint.b.size());

    for (int i = 0; i < com_in_lf_constraint.A.rows(); ++i) {
        for (int j = 0; j < com_in_lf_constraint.A.cols(); ++j) {
            this->A_com_in_lf_casadi(i, j) = com_in_lf_constraint.A(i, j);
        }
        this->b_com_in_lf_casadi(i) = com_in_lf_constraint.b(i);
    }
    
    // CoM in Right Foot constraint forward polytope
    this->A_com_in_rf_casadi = casadi::SX::zeros(com_in_rf_constraint.A.rows(), com_in_rf_constraint.A.cols());
    this->b_com_in_rf_casadi = casadi::SX::zeros(com_in_rf_constraint.b.size());

    for (int i = 0; i < com_in_rf_constraint.A.rows(); ++i) {
        for (int j = 0; j < com_in_rf_constraint.A.cols(); ++j) {
            this->A_com_in_rf_casadi(i, j) = com_in_rf_constraint.A(i, j);
        }
        this->b_com_in_rf_casadi(i) = com_in_rf_constraint.b(i);
    }

}

FootstepPlanner::~FootstepPlanner() {
    // Destructor implementation (can be empty if no special cleanup needed)
}

bool FootstepPlanner::plan(const int& stance_foot_flag_at_start, 
                           const Point_3& stance_foot_position_at_start, 
                           const int& stance_foot_flag_at_goal,
                           const Point_3& stance_foot_position_at_goal,
                           const std::vector<Node*>& path_nodes) {
                             
    // std::cout << "\n[ Planning Footsteps ]" << std::endl;
    std::cout << "CoM Constraint disabled for now" << std::endl;

    // Generate desired footstep positions
    std::vector<casadi::SX> desired_footstep_positions;
    for (int footstep_cnt = 0; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        if (footstep_cnt == 0) {
            desired_footstep_positions.push_back(casadi::SX::vertcat({
                CGAL::to_double(stance_foot_position_at_start.x()),
                CGAL::to_double(stance_foot_position_at_start.y()),
                CGAL::to_double(stance_foot_position_at_start.z())
            }));
        } else if (footstep_cnt == path_nodes.size() - 1) {
            desired_footstep_positions.push_back(casadi::SX::vertcat({
                CGAL::to_double(stance_foot_position_at_goal.x()),
                CGAL::to_double(stance_foot_position_at_goal.y()),
                CGAL::to_double(stance_foot_position_at_goal.z())
            }));
        } else {
            desired_footstep_positions.push_back(casadi::SX::vertcat({
                CGAL::to_double(path_nodes[footstep_cnt]->centroid.x()),
                CGAL::to_double(path_nodes[footstep_cnt]->centroid.y()),
                CGAL::to_double(path_nodes[footstep_cnt]->centroid.z())
            }));
        }
    }

    // Create decision variables for each footstep
    std::vector<casadi::SX> footstep_pos_vars;
    for (int footstep_cnt = 0; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        footstep_pos_vars.push_back(casadi::SX::sym("step"+std::to_string(footstep_cnt),3));
    }

    // Create objective function (empty)
    casadi::SX objective = 0;
    for (int footstep_cnt = 0; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        casadi::SX deviation = footstep_pos_vars[footstep_cnt] - desired_footstep_positions[footstep_cnt];
        objective += casadi::SX::dot(deviation, deviation);
    }

    // Create Footstep Reachability constraints (next foot in previous foot's polytope)
    std::vector<casadi::SX> reachability_constraints;
    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        casadi::SX A_matrix;
        casadi::SX b_vector;

        if (path_nodes[footstep_cnt]->stance_foot == 0) { // Left foot for making current step "footstep_cnt", then lf in rf
            A_matrix = A_lf_in_rf_casadi;
            b_vector = b_lf_in_rf_casadi;
        } else if (path_nodes[footstep_cnt]->stance_foot == 1) { // Right foot for making current step "footstep_cnt", then rf in lf
            A_matrix = A_rf_in_lf_casadi;
            b_vector = b_rf_in_lf_casadi;
        }
        else {
            throw std::runtime_error("Invalid stance foot flag");
        }
        reachability_constraints.push_back(mtimes(A_matrix, (footstep_pos_vars[footstep_cnt] - footstep_pos_vars[footstep_cnt-1])) - b_vector);
    }
    // Concatenate reachability constraints into a single vector
    casadi::SX reachability_constraints_vec = casadi::SX::vertcat(reachability_constraints);


    // Create Footstep Reachability constraints (previous foot in next foot's polytope)
    // Constraint: previous_foot must be reachable from current_foot
    // Polytope selection: based on previous foot's stance type
    std::vector<casadi::SX> reachability_constraints_prev;
    
    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        casadi::SX A_matrix;
        casadi::SX b_vector;

        // Select polytope based on PREVIOUS foot's stance type
        // If previous foot is LF, use "LF in RF" polytope
        // If previous foot is RF, use "RF in LF" polytope
        if (path_nodes[footstep_cnt-1]->stance_foot == 0) { // Previous was LF, so use LF in RF polytope
            A_matrix = A_lf_in_rf_casadi;
            b_vector = b_lf_in_rf_casadi;
        } else if (path_nodes[footstep_cnt-1]->stance_foot == 1) { // Previous was RF, so use RF in LF polytope
            A_matrix = A_rf_in_lf_casadi;
            b_vector = b_rf_in_lf_casadi;
        } else {
            throw std::runtime_error("Invalid previous stance foot flag for reverse reachability");
        }
        
        // Constraint: A * (previous_foot - current_foot) <= b
        casadi::SX relative_position = footstep_pos_vars[footstep_cnt-1] - footstep_pos_vars[footstep_cnt];
        casadi::SX constraint_expr = mtimes(A_matrix, relative_position) - b_vector;
        reachability_constraints_prev.push_back(constraint_expr);
        
        // std::cout << "  Step " << footstep_cnt << ": Previous foot (" 
        //           << (path_nodes[footstep_cnt-1]->stance_foot == 0 ? "LF" : "RF") 
        //           << ") reachable from current foot, " 
        //           << A_matrix.size1() << " constraints" << std::endl;
    }
    
    // Concatenate reachability constraints into a single vector
    casadi::SX reachability_constraints_prev_vec;
    casadi::DM reachability_constraints_prev_lb, reachability_constraints_prev_ub;
    
    if (!reachability_constraints_prev.empty()) {
        reachability_constraints_prev_vec = casadi::SX::vertcat(reachability_constraints_prev);
        reachability_constraints_prev_lb = -casadi::DM::inf(reachability_constraints_prev_vec.size1());
        reachability_constraints_prev_ub = casadi::DM::zeros(reachability_constraints_prev_vec.size1());
        // std::cout << "  Total reverse reachability constraints: " << reachability_constraints_prev_vec.size1() << std::endl;
    } else {
        reachability_constraints_prev_vec = casadi::SX::zeros(0, 1);
        reachability_constraints_prev_lb = casadi::DM::zeros(0);
        reachability_constraints_prev_ub = casadi::DM::zeros(0);
        // std::cout << "  No reverse reachability constraints" << std::endl;
    }

    // Create CoM constraints (next foot in previous foot's polytope)
    std::vector<casadi::SX> com_constraints_next;
    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        casadi::SX A_matrix;
        casadi::SX b_vector;
        
        if (path_nodes[footstep_cnt]->stance_foot == 0) { // Left foot for making current step "footstep_cnt", then lf in rf
            A_matrix = A_lf_in_rf_casadi;
            b_vector = b_lf_in_rf_casadi;
        } else if (path_nodes[footstep_cnt]->stance_foot == 1) { // Right foot for making current step "footstep_cnt", then rf in lf
            A_matrix = A_rf_in_lf_casadi;
            b_vector = b_rf_in_lf_casadi;
        }   
        else {
            throw std::runtime_error("Invalid stance foot flag");
        }
        com_constraints_next.push_back(mtimes(A_matrix, (footstep_pos_vars[footstep_cnt] - footstep_pos_vars[footstep_cnt-1])) - b_vector);
    }
    // Concatenate com constraints into a single vector
    casadi::SX com_constraints_next_vec = casadi::SX::vertcat(com_constraints_next);

    // Create CoM constraints starting from footstep 1
    // CoM position is above each footstep at com_z_height
    // CoM must stay inside the polytope based on the previous footstep's stance foot
    std::vector<casadi::SX> com_constraints;
    // std::cout << "\n[ Creating CoM Constraints ]" << std::endl;
    
    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size(); footstep_cnt++) {
        // CoM position: above current footstep at com_z_height
        casadi::SX com_position = casadi::SX::vertcat({
            footstep_pos_vars[footstep_cnt](0),  // x
            footstep_pos_vars[footstep_cnt](1),  // y
            footstep_pos_vars[footstep_cnt](2) + com_z_height  // z (footstep_z + com_height)
        });
        
        // Previous footstep position (stance foot)
        casadi::SX previous_footstep_position = footstep_pos_vars[footstep_cnt-1];
        
        // Select polytope based on previous footstep's stance foot
        casadi::SX A_com_matrix;
        casadi::SX b_com_vector;
        
        if (path_nodes[footstep_cnt-1]->stance_foot == 0) { // Previous was LF, so CoM constrained by LF polytope
            A_com_matrix = A_com_in_lf_casadi;
            b_com_vector = b_com_in_lf_casadi;
        } else if (path_nodes[footstep_cnt-1]->stance_foot == 1) { // Previous was RF, so CoM constrained by RF polytope
            A_com_matrix = A_com_in_rf_casadi;
            b_com_vector = b_com_in_rf_casadi;
        } else {
            throw std::runtime_error("Invalid previous stance foot flag for CoM constraint");
        }
        
        // Create constraint: A_com * (com_position - previous_footstep_position) <= b_com
        casadi::SX com_relative_position = com_position - previous_footstep_position;
        casadi::SX com_constraint_expr = mtimes(A_com_matrix, com_relative_position) - b_com_vector;
        com_constraints.push_back(com_constraint_expr);
        
        // std::cout << "  Step " << footstep_cnt << ": CoM constraint using " 
        //           << (path_nodes[footstep_cnt-1]->stance_foot == 0 ? "LF" : "RF") 
        //           << " polytope (previous stance foot), " 
        //           << A_com_matrix.size1() << " constraints" << std::endl;
    }
    
    // Concatenate CoM constraints into a single vector
    casadi::SX com_constraints_vec;
    casadi::DM com_constraints_lb, com_constraints_ub;
    
    if (!com_constraints.empty()) {
        com_constraints_vec = casadi::SX::vertcat(com_constraints);
        // CoM constraints are inequalities: A_com * com_position <= b_com
        com_constraints_lb = -casadi::DM::inf(com_constraints_vec.size1());
        // com_constraints_ub = casadi::DM::zeros(com_constraints_vec.size1());
        com_constraints_ub = casadi::DM::inf(com_constraints_vec.size1());
        // std::cout << " (Disabled) Total CoM constraints: " << com_constraints_vec.size1() << std::endl;
    } else {
        com_constraints_vec = casadi::SX::zeros(0, 1);
        com_constraints_lb = casadi::DM::zeros(0);
        com_constraints_ub = casadi::DM::zeros(0);
        // std::cout << "  No CoM constraints (only one footstep)" << std::endl;
    }


    // Create Surface constraints for intermediate steps (steps 1 to n-2)
    // Skip first step (0) and last step (n-1) as they are already constrained
    std::vector<casadi::SX> surface_constraints;
    std::vector<SurfaceConstraint> surface_constraint_cache; // Cache to avoid redundant calls
    // std::cout << "\n[ Creating Surface Constraints for Intermediate Steps ]" << std::endl;
    
    for (int footstep_cnt = 1; footstep_cnt < path_nodes.size() - 1; footstep_cnt++) {
        // Get the surface constraint for this patch (store in cache)
        SurfaceConstraint surface_constraint = generate_surface_constraint(path_nodes[footstep_cnt]->patch_polyhedron_3d);
        surface_constraint_cache.push_back(surface_constraint);
        
        // Convert Eigen matrices to CasADi format
        casadi::SX A_surface_casadi = casadi::SX::zeros(surface_constraint.A.rows(), surface_constraint.A.cols());
        casadi::SX b_surface_casadi = casadi::SX::zeros(surface_constraint.b.size());
        
        for (int i = 0; i < surface_constraint.A.rows(); ++i) {
            for (int j = 0; j < surface_constraint.A.cols(); ++j) {
                A_surface_casadi(i, j) = surface_constraint.A(i, j);
            }
            b_surface_casadi(i) = surface_constraint.b(i);
        }
        
        // Create constraint: A_surface * footstep_pos <= b_surface
        casadi::SX surface_constraint_expr = mtimes(A_surface_casadi, footstep_pos_vars[footstep_cnt]) - b_surface_casadi;
        surface_constraints.push_back(surface_constraint_expr);
        
        // std::cout << "  Step " << footstep_cnt << ": Added surface constraint with " 
        //           << surface_constraint.A.rows() << " constraints (1 plane + " 
        //           << (surface_constraint.A.rows() - 1) << " edge boundaries)" << std::endl;
    }
    
    // Concatenate surface constraints into a single vector (if any exist)
    casadi::SX surface_constraints_vec;
    casadi::DM surface_constraints_lb, surface_constraints_ub;
    
    if (!surface_constraints.empty()) {
        surface_constraints_vec = casadi::SX::vertcat(surface_constraints);
        
        // Create bounds: first row of each patch is equality (plane), rest are inequalities (edges)
        int total_surface_constraints = surface_constraints_vec.size1();
        surface_constraints_lb = casadi::DM::zeros(total_surface_constraints);
        surface_constraints_ub = casadi::DM::zeros(total_surface_constraints);
        
        // Set bounds for each surface constraint block using cached constraints
        int constraint_offset = 0;
        for (size_t cache_idx = 0; cache_idx < surface_constraint_cache.size(); cache_idx++) {
            // Use cached surface constraint instead of calling generate_surface_constraint again
            const SurfaceConstraint& surface_constraint = surface_constraint_cache[cache_idx];
            int num_constraints = surface_constraint.A.rows();
            
            // First constraint: plane equality (lb = ub = 0)
            surface_constraints_lb(constraint_offset) = 0.0;
            surface_constraints_ub(constraint_offset) = 0.0;
            
            // Remaining constraints: edge inequalities (lb = -inf, ub = 0)
            for (int i = 1; i < num_constraints; i++) {
                surface_constraints_lb(constraint_offset + i) = -casadi::DM::inf();
                surface_constraints_ub(constraint_offset + i) = 0.0;
            }
            
            constraint_offset += num_constraints;
        }
        
        // std::cout << "  Total surface constraints: " << surface_constraints_vec.size1() << std::endl;
        // std::cout << "  (Each patch: 1 plane equality + multiple edge inequalities)" << std::endl;
    } else {
        surface_constraints_vec = casadi::SX::zeros(0, 1);
        surface_constraints_lb = casadi::DM::zeros(0);
        surface_constraints_ub = casadi::DM::zeros(0);
        // std::cout << "  No intermediate steps - no surface constraints added" << std::endl;
    }

    // Concatenate all footstep position variables into a single vector
    casadi::SX all_vars = casadi::SX::vertcat(footstep_pos_vars);

    // Create bounds for reachability constraints
    casadi::DM reachability_constraints_lb = -casadi::DM::inf(reachability_constraints_vec.size1());
    casadi::DM reachability_constraints_ub = casadi::DM::zeros(reachability_constraints_vec.size1());
    
    // Generate Initial and Final footstep constraints
    // Convert Point_3 to CasADi SX format
    casadi::SX initial_stance_pos = casadi::SX::vertcat({
        CGAL::to_double(stance_foot_position_at_start.x()),
        CGAL::to_double(stance_foot_position_at_start.y()),
        CGAL::to_double(stance_foot_position_at_start.z())
    });
    casadi::SX final_stance_pos = casadi::SX::vertcat({
        CGAL::to_double(stance_foot_position_at_goal.x()),
        CGAL::to_double(stance_foot_position_at_goal.y()),
        CGAL::to_double(stance_foot_position_at_goal.z())
    });
    
    casadi::SX initial_footstep_constraints = footstep_pos_vars[0] - initial_stance_pos;
    casadi::SX final_footstep_constraints = footstep_pos_vars[path_nodes.size() - 1] - final_stance_pos;
    casadi::DM initial_footstep_constraints_lb = casadi::DM::zeros(initial_footstep_constraints.size1());
    casadi::DM initial_footstep_constraints_ub = casadi::DM::zeros(initial_footstep_constraints.size1());
    casadi::DM final_footstep_constraints_lb = casadi::DM::zeros(final_footstep_constraints.size1());
    casadi::DM final_footstep_constraints_ub = casadi::DM::zeros(final_footstep_constraints.size1());

    // Concatenate all constraint functions into a single vector
    std::vector<casadi::SX> all_constraint_vectors = {reachability_constraints_vec, reachability_constraints_prev_vec, com_constraints_vec, surface_constraints_vec, initial_footstep_constraints, final_footstep_constraints};
    casadi::SX all_constraints = casadi::SX::vertcat(all_constraint_vectors);

    // Create QP problem
    casadi::SXDict qp;
    qp["x"] = all_vars;
    qp["f"] = objective;
    qp["g"] = all_constraints;

    // Create QP solver
    casadi::Function solver = casadi::qpsol("footstep_qp", "qpoases", qp);
    std::cout << "QP Solver created with polytope constraints!" << std::endl;

    // Create solver arguments with proper types
    casadi::DMDict arg;
    arg["x0"] = casadi::DM::zeros(all_vars.size1());
    
    // Collect the bounds for all constraints
    std::vector<casadi::DM> lbg_parts = {reachability_constraints_lb, reachability_constraints_prev_lb, com_constraints_lb, surface_constraints_lb, initial_footstep_constraints_lb, final_footstep_constraints_lb};
    std::vector<casadi::DM> ubg_parts = {reachability_constraints_ub, reachability_constraints_prev_ub, com_constraints_ub, surface_constraints_ub, initial_footstep_constraints_ub, final_footstep_constraints_ub};
    
    arg["lbg"] = casadi::DM::vertcat(lbg_parts);
    arg["ubg"] = casadi::DM::vertcat(ubg_parts);

    // Solve QP with error handling
    try {
        casadi::DMDict result = solver(arg);
        
        // Print QP solver status
        casadi::Dict stats = solver.stats();
        std::cout << "\nSuccess: " << stats["success"] << std::endl;

        // Check if solver actually succeeded
        if (static_cast<bool>(stats["success"])) {
            // Extract and display results
            casadi::DM x_opt = result.at("x");
            double obj_val = static_cast<double>(result.at("f"));

            std::cout << "\n✓ QP solved successfully!" << std::endl;

            std::cout << "\n=== QP Solution ===" << std::endl;

            // Print the footstep positions (extract from solution)
            std::cout << "\n[ Footstep Positions ]" << std::endl;
            for (int i = 0; i < footstep_pos_vars.size(); i++) {
                // Extract the 3D position for footstep i from the solution vector
                // Each footstep has 3 coordinates: [x, y, z]
                casadi::DM footstep_x = x_opt(i*3 + 0);
                casadi::DM footstep_y = x_opt(i*3 + 1); 
                casadi::DM footstep_z = x_opt(i*3 + 2);
                
                std::cout << "Footstep " << i << " position: [" << footstep_x << ", " << footstep_y << ", " << footstep_z << "]" << std::endl;
                std::cout << "Footstep " << i << " desired:  " << desired_footstep_positions[i] << std::endl;
                std::cout << std::endl;
            }

            // Store the computed footsteps for later use
            computed_footsteps.clear();
            for (int i = 0; i < footstep_pos_vars.size(); i++) {
                double x = static_cast<double>(x_opt(i*3 + 0));
                double y = static_cast<double>(x_opt(i*3 + 1));
                double z = static_cast<double>(x_opt(i*3 + 2));
                computed_footsteps.push_back(Point_3(x, y, z));
            }
            
            return true;  // Success
        } else {
            std::cout << "\n✗ QP solver returned success=false (infeasible/failed)" << std::endl;
            // Clear computed footsteps to indicate failure
            computed_footsteps.clear();
            return false;  // Failure
        }
        
    } catch (const casadi::CasadiException& e) {
        std::cout << "\n✗ QP solver failed with CasADi exception: " << e.what() << std::endl;
        std::cout << "   This path is infeasible - skipping to next iteration" << std::endl;
        
        // Clear computed footsteps to indicate failure
        computed_footsteps.clear();
        
        // Don't re-throw the exception - let the caller handle the failure gracefully
        return false;  // Failure
    } catch (const std::exception& e) {
        std::cout << "\n✗ QP solver failed with exception: " << e.what() << std::endl;
        std::cout << "   This path is infeasible - skipping to next iteration" << std::endl;
        
        // Clear computed footsteps to indicate failure
        computed_footsteps.clear();
        return false;  // Failure
    }

}


}