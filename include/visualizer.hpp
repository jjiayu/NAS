#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Plane_3.h>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Point_3<Kernel> Point_3;
typedef CGAL::Plane_3<Kernel> Plane_3;
typedef Kernel::Point_2 Point_2;

namespace nas {

class Visualizer {
public:
    static void show_plane(const Plane& plane, double size = 5.0);
    static void show_polyhedron(const Polyhedron& P);
    static void show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon);
    static void show_2d_polygon(const Polygon_2& polygon);
    static void show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2);
    static void plot_2d_points_and_polygons(const Polygon_2& surf_polygon, 
                                          const Polygon_2& intersection_polygon);
};

} // namespace nas

#endif