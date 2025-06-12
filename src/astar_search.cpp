#include "astar_search.hpp"
#include "tree.hpp"
#include "types.hpp"
#include "visualizer.hpp"
#include "utils.hpp"
#include "geometry.hpp"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <vtkRendererCollection.h>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iomanip>

namespace nas {

AstarSearch::AstarSearch() {

    std::cout << "Initializing A* search algorithm" << std::endl;

    // Initialize polytopes
    load_obj(rf_in_lf_path, this->rf_in_lf_polytope);
    load_obj(lf_in_rf_path, this->lf_in_rf_polytope);

    // Initialize goal parameters
    this->goal_stance_foot = stance_foot_at_goal;
    this->goal_location = Point_3(0.0, 0.0, 0.0);  // Will be set during search
}

void AstarSearch::search() {
    // TODO: Implement A* search algorithm
    std::cout << "A* search not yet implemented" << std::endl;
}

} // namespace nas