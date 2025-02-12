#include "tree.hpp"
#include "constants.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include "utils.hpp"

typedef CGAL::Simple_cartesian<double> Kernel; // Using Simple Geometry Kernel
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
namespace nas {

Tree::Tree() {

    // Load the OBJ file into the polyhedron
    if (load_obj(rf_in_lf_path, rf_in_lf_polytope)) {
        std::cout << "Successfully loaded the OBJ file." << std::endl;
        std::cout << "Number of vertices: " << rf_in_lf_polytope.size_of_vertices() << std::endl;
        // std::cout << "Number of faces: " << rf_in_lf_polytope.size_of_faces() << std::endl;
    } else {
        std::cerr << "Failed to load the OBJ file." << std::endl;
    }

}



} // namespace nas  

