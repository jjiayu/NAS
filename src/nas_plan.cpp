#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

using namespace nas;
using namespace std;

int main() {
    // Create a plane (ax + by + cz + d = 0)
    // This creates a plane z = 1
    Plane plane(0, 0, 1, -1);
    
    // Visualize the plane
    Visualizer::show_plane(plane);
    
    return 0;
}
