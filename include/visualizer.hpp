#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Polygon_2.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>
#include <vtkCamera.h>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Polygon_2<Kernel> Polygon_2;

namespace nas {

class Visualizer {
public:
    // Create a new figure/window
    static vtkSmartPointer<vtkRenderWindow> create_figure(const std::string& title = "Visualization");
    
    // Add objects to the current figure
    static void add_polyhedron(vtkSmartPointer<vtkRenderer> renderer, 
                              const Polyhedron& polyhedron,
                              const double color[3] = nullptr,
                              double opacity = 0.7);
    
    static void add_plane(vtkSmartPointer<vtkRenderer> renderer,
                         const Plane_3& plane,
                         const double color[3] = nullptr,
                         double opacity = 0.3);
    
    static void add_points(vtkSmartPointer<vtkRenderer> renderer,
                          const std::vector<Point_3>& points,
                          const double color[3] = nullptr,
                          double radius = 0.02);
    
    static void add_coordinate_axes(vtkSmartPointer<vtkRenderer> renderer);
    
    // Show the current figure
    static void show(vtkSmartPointer<vtkRenderWindow> renderWindow);
    
    // Legacy functions (can be removed later)
    static void show_scene(const Plane_3& plane, const Polyhedron& polytope, const Polyhedron& intersection_polygon);
    static void show_polyhedron(const Polyhedron& P);
    static void show_2d_polygon(const Polygon_2& polygon);
    static void show_2d_polygons(const Polygon_2& polygon1, const Polygon_2& polygon2);
    static void plot_2d_points_and_polygons(const Polygon_2& surf_polygon, const Polygon_2& intersection_polygon);
};

} // namespace nas