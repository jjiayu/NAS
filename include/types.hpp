#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Line_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/property_map.h>

namespace nas {

// Forward declaration
class Node;

// Basic kernel
typedef CGAL::Simple_cartesian<double> Kernel;

// 3D types
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Vector_3<Kernel> Vector_3;
typedef CGAL::Point_3<Kernel> Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;
typedef Kernel::Vector_2 Vector_2;

// 2D types
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Gps_segment_traits_2<Kernel> Traits_2;
typedef CGAL::General_polygon_set_2<Traits_2> Polygon_set_2;
typedef Traits_2::Polygon_2 General_polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
typedef CGAL::Segment_2<Kernel> Segment_2;
typedef CGAL::Line_2<Kernel> Line_2;

// KD-tree types for spatial searching
typedef CGAL::Search_traits_3<Kernel> TreeTraits;
typedef std::pair<Point_3, Node*> Centroid_and_Node;
typedef CGAL::Search_traits_adapter<Centroid_and_Node,
                                  CGAL::First_of_pair_property_map<Centroid_and_Node>,
                                  TreeTraits> Search_traits_adapted;
typedef CGAL::Orthogonal_k_neighbor_search<Search_traits_adapted> Neighbor_search;
typedef Neighbor_search::Tree KD_Tree;

} // namespace nas 