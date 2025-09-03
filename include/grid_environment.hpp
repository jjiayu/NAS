#pragma once

#include "types.hpp"
#include "surface.hpp"
#include <vector>
#include "constants.hpp"

namespace nas {

// Grid environment representation
class GridEnvironment {
public:
    struct GridCell {
        bool is_traversable;
        int surface_id;
        double height;
        Point_3 world_position;
        
        GridCell() : is_traversable(false), surface_id(-1), height(0.0) {}
    };
    
private:
    int width_, height_;
    double cell_size_;
    Point_3 world_min_, world_max_;
    std::vector<std::vector<GridCell>> grid_;
    
public:
    GridEnvironment(double cell_size = a_star_grid_resolution);
    
    // Initialize grid from surfaces
    void initialize_from_surfaces(const std::vector<Surface>& surfaces);
    
    // Coordinate conversion
    std::pair<int, int> world_to_grid(const Point_3& world_pos) const;
    Point_3 grid_to_world(int grid_x, int grid_y) const;
    
    // Grid queries
    bool is_valid_cell(int grid_x, int grid_y) const;
    bool is_traversable(int grid_x, int grid_y) const;
    const GridCell& get_cell(int grid_x, int grid_y) const;
    
    // Get neighboring cells for A* expansion
    std::vector<std::pair<int, int>> get_neighbors(int grid_x, int grid_y) const;
    
    // Getters
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    double get_cell_size() const { return cell_size_; }
};

} // namespace nas
