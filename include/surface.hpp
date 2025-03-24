#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "utils.hpp"
#include <vector>
#include <memory>
#include <CGAL/convex_hull_3.h>


namespace nas {

class Surface {
public:
    int surf_id;
    std::vector<Point_3> vertices_3d;
    std::vector<Point_2> vertices_2d;
    Plane_3 plane;
    Vector_3 norm;
    Vector_3 centroid;
    Transformation transform; // Transformation to align with plane (center)
    Polygon_2 polygon_2d;

    
    Surface(const std::vector<Point_3>& points, int& surface_idx);    // Constructor
    void establish_surface_coordinate_system(const std::vector<Point_3>& points);     // Establish the surface coordinate system at the centroid
    
};


} // namespace nas  
