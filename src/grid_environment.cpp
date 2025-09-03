#include "grid_environment.hpp"
#include <limits>
#include <cmath>

namespace nas {

// GridEnvironment implementation
GridEnvironment::GridEnvironment(double cell_size) : cell_size_(cell_size) {
    width_ = 0;
    height_ = 0;
}

void GridEnvironment::initialize_from_surfaces(const std::vector<Surface>& surfaces) {
    if (surfaces.empty()) return;
    
    // Calculate world bounds from all surfaces
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    
    for (const auto& surface : surfaces) {
        for (const auto& vertex : surface.vertices_3d) {
            min_x = std::min(min_x, vertex.x());
            max_x = std::max(max_x, vertex.x());
            min_y = std::min(min_y, vertex.y());
            max_y = std::max(max_y, vertex.y());
        }
    }
    
    // Add padding
    double padding = cell_size_ * 2.0;
    world_min_ = Point_3(min_x - padding, min_y - padding, 0.0);
    world_max_ = Point_3(max_x + padding, max_y + padding, 0.0);
    
    // Calculate grid dimensions (num of grid cells)
    width_ = static_cast<int>(std::ceil((max_x - min_x + 2 * padding) / cell_size_));
    height_ = static_cast<int>(std::ceil((max_y - min_y + 2 * padding) / cell_size_));
    
    // Initialize grid
    grid_.resize(height_, std::vector<GridCell>(width_));
    
    // Mark cells as traversable if they overlap with surfaces
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Point_3 cell_center = grid_to_world(x, y);
            
            // Check if this cell overlaps with any surface
            for (size_t surf_id = 0; surf_id < surfaces.size(); ++surf_id) {
                const auto& surface = surfaces[surf_id];
                
                // Check if point is inside the surface projected to XY plane
                std::vector<Point_2> surface_2d_projection;
                for (const auto& vertex : surface.vertices_3d) {
                    surface_2d_projection.push_back(Point_2(vertex.x(), vertex.y()));
                }
                
                // Ensure convex hull with counter-clockwise ordering
                std::vector<Point_2> convex_hull_2d;
                CGAL::convex_hull_2(surface_2d_projection.begin(), surface_2d_projection.end(), 
                                   std::back_inserter(convex_hull_2d));
                
                Point_2 cell_center_2d(cell_center.x(), cell_center.y());
                CGAL::Bounded_side result = CGAL::bounded_side_2(
                    convex_hull_2d.begin(), 
                    convex_hull_2d.end(),
                    cell_center_2d, 
                    Kernel()
                );
                bool inside = (result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY);
                
                if (inside == true) {
                    // Calculate exact Z-coordinate using surface plane equation
                    // Plane equation: ax + by + cz + d = 0, solve for z: z = -(ax + by + d) / c
                    double a = surface.plane.a();
                    double b = surface.plane.b(); 
                    double c = surface.plane.c();
                    double d = surface.plane.d();
                    
                    double z_coord;
                    if (std::abs(c) > 1e-9) { // Check if plane is not vertical
                        z_coord = -(a * cell_center.x() + b * cell_center.y() + d) / c;
                    } else {
                        // For vertical surfaces, use centroid Z as fallback
                        z_coord = surface.centroid.z();
                    }
                    
                    grid_[y][x].is_traversable = true;
                    grid_[y][x].surface_id = surf_id;
                    grid_[y][x].height = z_coord;
                    grid_[y][x].world_position = Point_3(cell_center.x(), cell_center.y(), z_coord);
                    break; // Use first matching surface
                }
            }
        }
    }
    
    std::cout << "Grid initialized: " << width_ << "x" << height_ 
              << " cells, cell_size=" << cell_size_ << std::endl;
}

std::pair<int, int> GridEnvironment::world_to_grid(const Point_3& world_pos) const {
    int grid_x = static_cast<int>((world_pos.x() - world_min_.x()) / cell_size_);
    int grid_y = static_cast<int>((world_pos.y() - world_min_.y()) / cell_size_);
    return {grid_x, grid_y};
}

Point_3 GridEnvironment::grid_to_world(int grid_x, int grid_y) const {
    double world_x = world_min_.x() + (grid_x + 0.5) * cell_size_;
    double world_y = world_min_.y() + (grid_y + 0.5) * cell_size_;
    return Point_3(world_x, world_y, 0.0); // Z will be set based on surface
}

bool GridEnvironment::is_valid_cell(int grid_x, int grid_y) const {
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

bool GridEnvironment::is_traversable(int grid_x, int grid_y) const {
    if (!is_valid_cell(grid_x, grid_y)) return false;
    return grid_[grid_y][grid_x].is_traversable;
}

const GridEnvironment::GridCell& GridEnvironment::get_cell(int grid_x, int grid_y) const {
    static GridCell invalid_cell;
    if (!is_valid_cell(grid_x, grid_y)) return invalid_cell;
    return grid_[grid_y][grid_x];
}

std::vector<std::pair<int, int>> GridEnvironment::get_neighbors(int grid_x, int grid_y) const {
    std::vector<std::pair<int, int>> neighbors;
    
    // 8-connected neighbors (including diagonals)
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue; // Skip current cell
            
            int nx = grid_x + dx;
            int ny = grid_y + dy;
            
            if (is_traversable(nx, ny)) {
                neighbors.push_back({nx, ny});
            }
        }
    }
    
    return neighbors;
}

} // namespace nas
