#ifndef NAS_VISUALIZER_HPP
#define NAS_VISUALIZER_HPP

#include "types.hpp"
#include <CGAL/Surface_mesh.h>
#include <vector>

namespace nas {

typedef CGAL::Surface_mesh<Point_3> Mesh;

class Visualizer {
public:
    static void show_plane(const Plane_3& plane, double size = 5.0);
    static void show_polyhedron(const Polyhedron& P);
    static void show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon);
    static void show_2d_polygon(const Polygon_2& polygon);
    static void show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2);
    static void plot_2d_points_and_polygons(const Polygon_2& surf_polygon, 
                                          const Polygon_2& intersection_polygon);
};

} // namespace nas

#endif