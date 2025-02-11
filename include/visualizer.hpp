#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <matplot/matplot.h>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

class Visualizer {
public:
    static void show_mesh(const Mesh& mesh) {
        std::vector<double> x, y, z;
        
        // Extract vertex coordinates
        for(auto v : mesh.vertices()) {
            Point p = mesh.point(v);
            x.push_back(p.x());
            y.push_back(p.y());
            z.push_back(p.z());
        }
        
        // Create 3D scatter plot
        auto f = matplot::figure(true);  // Create a new figure
        auto ax = f->current_axes();     // Get current axes
        
        // Plot points and lines
        auto p = ax->plot3(x, y, z, "o-");
        p->line_width(2);
        
        // Customize appearance
        ax->grid(true);
        ax->xlabel("X");
        ax->ylabel("Y");
        ax->zlabel("Z");
        
        // Display the plot
        f->show();
        matplot::show();
    }
};

#endif