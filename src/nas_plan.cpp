#include <iostream>
#include "nas_plan.hpp"
#include "node.hpp"
#include "visualizer.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

using namespace nas;
using namespace std;

int main() {
    // Create a mesh
    Mesh mesh;
    
    // Add vertices
    Point p1(0,0,0);
    Point p2(1,0,0);
    Point p3(0,1,0);
    
    // Add vertices to mesh
    Mesh::Vertex_index v1 = mesh.add_vertex(p1);
    Mesh::Vertex_index v2 = mesh.add_vertex(p2);
    Mesh::Vertex_index v3 = mesh.add_vertex(p3);
    
    // Add face (triangle)
    mesh.add_face(v1, v2, v3);
    
    // Visualize the mesh
    Visualizer::show_mesh(mesh);
    
    return 0;
}
