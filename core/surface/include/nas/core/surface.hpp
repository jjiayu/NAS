#pragma once

// De-globalized port of the original Surface class (see PLAN.md phase 2).
// Foot dimensions used to shrink the surface footprint into a patch are now
// constructor parameters instead of globals read from constants.hpp — this
// is the module that will eventually take them from RobotModel (phase 9).

#include "nas/core/types.hpp"
#include <vector>

namespace nas {

class Surface {
public:
    int surface_id;
    std::vector<Point_3> vertices_3d;   // sorted counterclockwise
    std::vector<Point_2> vertices_2d;   // sorted counterclockwise
    Plane_3 plane;
    Vector_3 norm;
    Point_3 centroid;
    Transformation transform_to_3d;      // surface coordinate system -> world
    Transformation transform_to_surface; // cached inverse (world -> surface)
    Polyhedron polyhedron_3d;
    Polygon_2 polygon_2d;

    // foot_length/foot_width shrink the surface footprint on each axis by
    // half their value, so a footstep placed on the shrunk boundary still
    // has the full foot resting on the original surface.
    Surface(const std::vector<Point_3>& points, int surface_idx, double foot_length, double foot_width);

    void establish_surface_coordinate_system(const std::vector<Point_3>& points);
};

} // namespace nas
