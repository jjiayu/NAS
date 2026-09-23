#pragma once

// GridEnvironment — rasterizes a set of Surfaces into a 2D grid (see
// PLAN.md phase 13: grid_astar_search is a low-priority, direct-port
// discretized baseline, faithful to the old code's algorithm). Each
// traversable cell caches which surface it belongs to and its exact
// world-space height (evaluated from that surface's own plane equation,
// not just the surface's centroid Z) — cheap since it's computed once at
// construction, not per query.
//
// Unlike core/expansion's continuous patch/polygon representation (Minkowski
// sum + convex clip), this is a plain uniform grid: cheaper to reason about,
// cheaper to query, but only as precise as its cell_size and blind to
// anything smaller than a cell.

#include "nas/core/surface.hpp"

#include <utility>
#include <vector>

namespace nas {

class GridEnvironment {
public:
    struct GridCell {
        bool is_traversable = false;
        int surface_id = -1;
        double height = 0.0;
        Point_3 world_position{0.0, 0.0, 0.0};
    };

    explicit GridEnvironment(double cell_size);

    // Rasterizes every surface's footprint (its own vertices_3d, i.e. the
    // already foot-shrunk patch, projected to the XY plane) into the grid.
    // A cell is traversable if its center falls inside any surface's
    // footprint; ties (overlapping surfaces) go to whichever surface comes
    // first in `surfaces`.
    void initialize_from_surfaces(const std::vector<Surface>& surfaces);

    std::pair<int, int> world_to_grid(const Point_3& world_pos) const;
    Point_3 grid_to_world(int grid_x, int grid_y) const;

    bool is_valid_cell(int grid_x, int grid_y) const;
    bool is_traversable(int grid_x, int grid_y) const;
    const GridCell& get_cell(int grid_x, int grid_y) const;

    int width() const { return width_; }
    int height() const { return height_; }
    double cell_size() const { return cell_size_; }

private:
    int width_ = 0;
    int height_ = 0;
    double cell_size_;
    Point_3 world_min_{0.0, 0.0, 0.0};
    std::vector<std::vector<GridCell>> grid_;
};

} // namespace nas
