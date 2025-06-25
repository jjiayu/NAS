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
    int surface_id;
    std::vector<Point_3> vertices_3d; //sorted counterclockwise
    std::vector<Point_2> vertices_2d; //sorted counterclockwise
    Plane_3 plane;
    Vector_3 norm;
    Point_3 centroid;
    Transformation transform_to_3d; // Transformation from surface to the world (representing the surface coordinate system in the world coordinate system)
    Transformation transform_to_surface; // Cached inverse transformation (from world to surface)
    Polyhedron polyhedron_3d;
    Polygon_2 polygon_2d;

    
    Surface(const std::vector<Point_3>& points, int& surface_idx);    // Constructor
    void establish_surface_coordinate_system(const std::vector<Point_3>& points);     // Establish the surface coordinate system at the centroid
    
};


} // namespace nas  
