#pragma once

// Pure geometric type aliases (CGAL-based). Deliberately excludes anything
// robot/search-specific (no Node forward-declaration, no KD-tree types) —
// those belong to core/node once it's ported (see PLAN.md phase 4). This
// header has no dependency on core/node, core/reachability, or any global
// configuration: it is the base of the "core" dependency graph.

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Line_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>

namespace nas {

// Basic kernel — EPICK gives exact predicates (robust convex hull,
// orientation tests) while keeping double-precision constructions (fast
// arithmetic).
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

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

} // namespace nas
