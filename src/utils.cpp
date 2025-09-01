#include "utils.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include "geometry.hpp"
#include "types.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <iostream>
#include <algorithm>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron) {
    if (!CGAL::IO::read_polygon_mesh(filename, polyhedron)) {
        throw std::runtime_error("Failed to load polyhedron from: " + filename);
    }
    std::cout << "- Successfully loaded polytope from: " << filename << std::endl;
}

bool cycle_path_detection(const Node* parent, const int current_stance_foot, const int surface_id) {
    // Safety check for null pointer
    if (parent == nullptr) {
        return false;
    }
    
    // Get the history for the current stance foot
    const auto& foot_history = parent->pred_surface_ids[current_stance_foot];
    
    // If no history, allow the move
    if (foot_history.empty()) {
        return false;
    }
    
    bool left_surface = false;
    
    // Loop backward through history to detect cycle
    for (int i = foot_history.size() - 1; i >= 0; --i) {
        const auto& layer = foot_history[i];
        bool surface_in_layer = std::find(layer.begin(), layer.end(), surface_id) != layer.end();
        
        if (surface_in_layer == false) {
            // We found a layer where we were not on the target surface - we left it
            left_surface = true;
        } else if (left_surface == true && surface_in_layer == true) {
            // We left the surface and now found it again in history - cycle detected
            return true;
        }
    }
    return false; // No cycle detected
}

} // namespace nas