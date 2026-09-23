#include "nas/planners/grid_environment.hpp"

#include <CGAL/convex_hull_2.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace nas {

GridEnvironment::GridEnvironment(double cell_size) : cell_size_(cell_size) {}

void GridEnvironment::initialize_from_surfaces(const std::vector<Surface>& surfaces) {
    if (surfaces.empty()) return;

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& surface : surfaces) {
        for (const auto& vertex : surface.vertices_3d) {
            min_x = std::min(min_x, CGAL::to_double(vertex.x()));
            max_x = std::max(max_x, CGAL::to_double(vertex.x()));
            min_y = std::min(min_y, CGAL::to_double(vertex.y()));
            max_y = std::max(max_y, CGAL::to_double(vertex.y()));
        }
    }

    double padding = cell_size_ * 2.0;
    world_min_ = Point_3(min_x - padding, min_y - padding, 0.0);

    width_ = static_cast<int>(std::ceil((max_x - min_x + 2 * padding) / cell_size_));
    height_ = static_cast<int>(std::ceil((max_y - min_y + 2 * padding) / cell_size_));

    grid_.assign(height_, std::vector<GridCell>(width_));

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Point_3 cell_center = grid_to_world(x, y);

            for (size_t surf_id = 0; surf_id < surfaces.size(); ++surf_id) {
                const Surface& surface = surfaces[surf_id];

                std::vector<Point_2> footprint_2d;
                footprint_2d.reserve(surface.vertices_3d.size());
                for (const auto& vertex : surface.vertices_3d) {
                    footprint_2d.emplace_back(vertex.x(), vertex.y());
                }
                std::vector<Point_2> hull_2d;
                CGAL::convex_hull_2(footprint_2d.begin(), footprint_2d.end(), std::back_inserter(hull_2d));

                Point_2 cell_center_2d(cell_center.x(), cell_center.y());
                CGAL::Bounded_side side = CGAL::bounded_side_2(hull_2d.begin(), hull_2d.end(), cell_center_2d, Kernel());
                if (side != CGAL::ON_BOUNDED_SIDE && side != CGAL::ON_BOUNDARY) {
                    continue;
                }

                // Exact height from the surface's own plane equation
                // (ax + by + cz + d = 0), not just its centroid Z — matters
                // for sloped surfaces (e.g. Stairs' angled steps, if any).
                double a = CGAL::to_double(surface.plane.a());
                double b = CGAL::to_double(surface.plane.b());
                double c = CGAL::to_double(surface.plane.c());
                double d = CGAL::to_double(surface.plane.d());

                double z = (std::abs(c) > 1e-9) ? -(a * CGAL::to_double(cell_center.x()) + b * CGAL::to_double(cell_center.y()) + d) / c
                                                 : CGAL::to_double(surface.centroid.z());

                GridCell& cell = grid_[y][x];
                cell.is_traversable = true;
                cell.surface_id = static_cast<int>(surf_id);
                cell.height = z;
                cell.world_position = Point_3(cell_center.x(), cell_center.y(), z);
                break; // first matching surface wins, matches the old code
            }
        }
    }
}

std::pair<int, int> GridEnvironment::world_to_grid(const Point_3& world_pos) const {
    int grid_x = static_cast<int>((CGAL::to_double(world_pos.x()) - CGAL::to_double(world_min_.x())) / cell_size_);
    int grid_y = static_cast<int>((CGAL::to_double(world_pos.y()) - CGAL::to_double(world_min_.y())) / cell_size_);
    return {grid_x, grid_y};
}

Point_3 GridEnvironment::grid_to_world(int grid_x, int grid_y) const {
    double x = CGAL::to_double(world_min_.x()) + (grid_x + 0.5) * cell_size_;
    double y = CGAL::to_double(world_min_.y()) + (grid_y + 0.5) * cell_size_;
    return Point_3(x, y, 0.0); // height is set by initialize_from_surfaces per cell
}

bool GridEnvironment::is_valid_cell(int grid_x, int grid_y) const {
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

bool GridEnvironment::is_traversable(int grid_x, int grid_y) const {
    return is_valid_cell(grid_x, grid_y) && grid_[grid_y][grid_x].is_traversable;
}

const GridEnvironment::GridCell& GridEnvironment::get_cell(int grid_x, int grid_y) const {
    static const GridCell invalid_cell;
    return is_valid_cell(grid_x, grid_y) ? grid_[grid_y][grid_x] : invalid_cell;
}

} // namespace nas
