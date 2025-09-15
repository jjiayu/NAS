#include "tree.hpp"
#include "types.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "geometry.hpp"
#include "constants.hpp"
#include <iostream>
#include "astar_search.hpp"
#include "footstep_planner.hpp"
#include <vtkRendererCollection.h>

int main() {
    using namespace nas;

    std::cout << "=== A* Path Planning with Footstep Optimization (using Discretization) ===" << std::endl;

    // Create A* search instance
    AstarSearch astar_search;

    // Create FootstepPlanner instance
    FootstepPlanner footstep_planner;

    // Search for the path
    std::cout << "\n=== Running A* Search ===" << std::endl;
    astar_search.search();

    // Check if path was found
    if (astar_search.result_path.empty()) {
        std::cout << "No path found by A* search. Exiting..." << std::endl;
        return 1;
    }

    // std::cout << "\n=== A* Path Found with " << astar_search.result_path.size() << " nodes ===" << std::endl;
    // for (size_t i = 0; i < astar_search.result_path.size(); i++) {
    //     const auto& node = astar_search.result_path[i];
    //     std::cout << "Step " << i << ": Node ID " << node->node_id 
    //               << ", Surface ID " << node->surface_id 
    //               << ", Stance Foot " << (node->stance_foot == LEFT_FOOT ? "LEFT" : "RIGHT") << std::endl;
    // }

    astar_search.plot_path();

    // Plan the footsteps
    std::cout << "\n=== Planning Footsteps with Optimization ===" << std::endl;
    bool planning_success = footstep_planner.plan(current_stance_foot_flag, current_foot_pos, 
                                                   astar_search.goal_stance_foot, 
                                                   astar_search.goal_location,
                                                   astar_search.result_path);
    
    if (!planning_success) {
        std::cout << "⚠️  Footstep planning failed! Continuing with visualization of A* path only." << std::endl;
    }

    // Create comprehensive visualization
    // std::cout << "\n=== Creating Comprehensive Visualization ===" << std::endl;
    auto renderWindow = Visualizer::create_figure("A* Path with Footstep Planning and Reachability Polytopes");
    auto renderer = renderWindow->GetRenderers()->GetFirstRenderer();

    // Add coordinate axes
    Visualizer::add_coordinate_axes(renderer);

    // Add environment surfaces
    // std::cout << "Adding environment surfaces..." << std::endl;
    for (const auto& surface : astar_search.surfaces) {
        double surface_color[3] = {0.7, 0.9, 1.0};  // Light blue
        Visualizer::add_polyhedron(renderer, surface.polyhedron_3d, surface_color, 0.3);
    }

    // Add start and goal positions
    // std::cout << "Adding start and goal positions..." << std::endl;
    double start_color[3] = {1.0, 0.0, 0.0};  // Red
    double goal_color[3] = {0.0, 1.0, 0.0};   // Green
    Visualizer::add_points(renderer, {current_foot_pos}, start_color, 0.1);
    Visualizer::add_points(renderer, {astar_search.goal_location}, goal_color, 0.1);

    // Add A* path patches with stance foot coloring
    // std::cout << "Adding A* path patches..." << std::endl;
    for (const auto& node : astar_search.result_path) {
        double patch_color[3];
        if (node->stance_foot == LEFT_FOOT) {
            patch_color[0] = 1.0; patch_color[1] = 0.0; patch_color[2] = 0.0; // Red for left foot
        } else {
            patch_color[0] = 0.0; patch_color[1] = 0.0; patch_color[2] = 1.0; // Blue for right foot
        }
        Visualizer::add_polyhedron(renderer, node->patch_polyhedron_3d, patch_color, 0.5);
    }

    // Add computed footsteps if available
    const auto& computed_footsteps = footstep_planner.get_computed_footsteps();
    if (!computed_footsteps.empty()) {
        // std::cout << "Adding optimized footsteps..." << std::endl;
        
        // Extract foot yaw angles from A* path nodes
        std::vector<double> foot_yaw_angles;
        for (const auto& node : astar_search.result_path) {
            foot_yaw_angles.push_back(node->foot_yaw);
        }
        
        // Add optimized footsteps with red/blue coloring based on stance foot
        for (size_t j = 0; j < computed_footsteps.size(); j++) {
            double footstep_color[3];
            if (j < astar_search.result_path.size()) {
                if (astar_search.result_path[j]->stance_foot == LEFT_FOOT) {
                    footstep_color[0] = 1.0; footstep_color[1] = 0.0; footstep_color[2] = 0.0; // Red for left foot
                } else {
                    footstep_color[0] = 0.0; footstep_color[1] = 0.0; footstep_color[2] = 1.0; // Blue for right foot
                }
            } else {
                // Default color for any extra footsteps
                footstep_color[0] = 0.5; footstep_color[1] = 0.5; footstep_color[2] = 0.5; // Gray
            }
            
            double yaw_angle = (j < foot_yaw_angles.size()) ? foot_yaw_angles[j] : 0.0;
            Visualizer::add_footsteps(renderer, {computed_footsteps[j]}, footstep_color, 0.22, 0.12, {yaw_angle});
        }

        // Add reachability polytopes at each footstep position
        // std::cout << "\n=== Adding reachability polytopes at footstep positions ===" << std::endl;
        for (size_t j = 0; j < computed_footsteps.size() - 1; j++) {  // Don't show polytope for last step
            const auto& current_footstep = computed_footsteps[j];
            
            // Determine which polytope to show based on current step's stance foot
            Polyhedron polytope_to_show;
            double polytope_color[3];
            std::string polytope_description;
            
            if (j < astar_search.result_path.size()) {
                if (astar_search.result_path[j]->stance_foot == RIGHT_FOOT) {
                    // Current step is right foot stance, show LF in RF polytope (reachable region for left foot)
                    polytope_to_show = footstep_planner.lf_in_rf_polytope;
                    polytope_color[0] = 1.0; polytope_color[1] = 0.5; polytope_color[2] = 0.5;  // Light red
                    // polytope_description = "LF in RF (left foot reachable region)";
                } else {
                    // Current step is left foot stance, show RF in LF polytope (reachable region for right foot)
                    polytope_to_show = footstep_planner.rf_in_lf_polytope;
                    polytope_color[0] = 0.5; polytope_color[1] = 0.5; polytope_color[2] = 1.0;  // Light blue
                    // polytope_description = "RF in LF (right foot reachable region)";
                }
                
                // Get translation offsets
                double tx = CGAL::to_double(current_footstep.x());
                double ty = CGAL::to_double(current_footstep.y());
                double tz = CGAL::to_double(current_footstep.z());
                
                // Get foot yaw angle for rotation
                double foot_yaw = 0.0;
                if (j < foot_yaw_angles.size()) {
                    foot_yaw = foot_yaw_angles[j];
                }
                
                // Create a transformed copy of the polytope
                Polyhedron transformed_polytope = polytope_to_show;  // Copy the original
                
                // Apply rotation around Z-axis followed by translation
                Transformation rotation(cos(foot_yaw), -sin(foot_yaw), 0, 0,
                                      sin(foot_yaw),  cos(foot_yaw), 0, 0,
                                      0,             0,            1, 0,
                                      1);
                Transformation translation(CGAL::TRANSLATION, Vector_3(tx, ty, tz));
                Transformation combined_transform = translation * rotation;
                
                // Apply transformation to all vertices
                for (auto v_it = transformed_polytope.vertices_begin(); v_it != transformed_polytope.vertices_end(); ++v_it) {
                    v_it->point() = combined_transform(v_it->point());
                }
                
                // Add the transformed polytope to visualization
                Visualizer::add_polyhedron(renderer, transformed_polytope, polytope_color, 0.2);
                
                // std::cout << "Step " << j << " (" << polytope_description << ") at position [" 
                        //   << tx << ", " << ty << ", " << tz << "]" << std::endl;
            }
        }
    } else {
        std::cout << "No computed footsteps available for visualization." << std::endl;
    }

    // Show the comprehensive visualization
    std::cout << "\n=== Showing Comprehensive Visualization ===" << std::endl;
    std::cout << "Visualization includes:" << std::endl;
    std::cout << "  - Environment surfaces (light blue)" << std::endl;
    std::cout << "  - Start position (red point)" << std::endl;
    std::cout << "  - Goal position (green point)" << std::endl;
    std::cout << "  - A* path patches (red=left foot stance, blue=right foot stance)" << std::endl;
    std::cout << "  - Optimized footsteps (red=left foot stance, blue=right foot stance)" << std::endl;
    std::cout << "  - Reachability polytopes at each step (alternating colors)" << std::endl;
    std::cout << "Close the visualization window to exit." << std::endl;
    
    Visualizer::show(renderWindow);
    
    return 0;
}