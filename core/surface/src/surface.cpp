#include "nas/core/surface.hpp"
#include "nas/core/geometry.hpp"

#include <CGAL/convex_hull_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <stdexcept>

namespace nas {

Surface::Surface(const std::vector<Point_3>& points, int surface_idx, double foot_length, double foot_width) {
    if (points.size() < 3) throw std::invalid_argument("At least 3 points are required to fit a surface");

    surface_id = surface_idx;

    CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());

    norm = plane.orthogonal_vector();
    norm = norm / std::sqrt(norm.squared_length());

    // norm keeps the raw sign the fit returned, exactly like the old code:
    // it is passed to convex_hull_3_from_coplanar_points, whose prism
    // triangulation - and therefore compute_polygon_perimeter, which sums
    // *every polyhedron edge* including the triangulation diagonals - depends
    // on it. Flipping it changed node perimeters (and the dedup keys built
    // from them) versus the old code; found by tests/golden_all's
    // expansion differential test, see docs/paper-deltas.md.

    centroid = get_centroid(points);

    establish_surface_coordinate_system(points);

    // Transform points to surface plane and get 2d polygon (also sort
    // points counterclockwise with convex hull further down).
    vertices_2d = transform_3d_points_to_surface_plane(points, transform_to_surface);

    // Shrink patches by foot_length and foot_width.
    std::vector<Point_2> vertices_2d_shrinked;
    for (const auto& vertex : vertices_2d) {
        double new_x = vertex.x();
        double new_y = vertex.y();
        if (vertex.x() > 0) new_x -= foot_length / 2.0;
        if (vertex.x() < 0) new_x += foot_length / 2.0;
        if (vertex.y() > 0) new_y -= foot_width / 2.0;
        if (vertex.y() < 0) new_y += foot_width / 2.0;
        vertices_2d_shrinked.emplace_back(new_x, new_y);
    }

    CGAL::convex_hull_2(vertices_2d_shrinked.begin(), vertices_2d_shrinked.end(), std::back_inserter(polygon_2d));

    vertices_2d.clear();
    for (auto it = polygon_2d.vertices_begin(); it != polygon_2d.vertices_end(); ++it) {
        vertices_2d.push_back(Point_2(it->x(), it->y()));
    }

    vertices_3d = transform_2d_points_to_world(vertices_2d, transform_to_3d);

    // Vertices are coplanar, use the safe wrapper (CGAL::convex_hull_3
    // asserts on coplanar input).
    polyhedron_3d = convex_hull_3_from_coplanar_points(vertices_3d, norm);
}

void Surface::establish_surface_coordinate_system(const std::vector<Point_3>& points) {
    // In-plane axes are built from a sign-canonicalised copy of norm (largest
    // |component| positive): the fit's sign is arbitrary, and a cross product
    // is sign-sensitive, so building the frame from the raw sign would mirror
    // the local 2D frame (reversing the polygon winding Sutherland-Hodgman
    // relies on) on roughly half of all surfaces. norm itself stays raw (see
    // the constructor). For the horizontal surfaces of environments.hpp this
    // yields exactly the old code's x_axis=(1,0,0), y_axis=(0,1,0).
    double nx = CGAL::to_double(norm.x());
    double ny = CGAL::to_double(norm.y());
    double nz = CGAL::to_double(norm.z());
    double dominant = (std::abs(nx) >= std::abs(ny) && std::abs(nx) >= std::abs(nz)) ? nx
                     : (std::abs(ny) >= std::abs(nz))                                ? ny
                                                                                      : nz;
    Vector_3 frame_norm = dominant < 0.0 ? -norm : norm;

    // Project whichever world axis is *least* aligned with the normal
    // (smallest |component|) instead of always projecting world_x then
    // world_y, which degenerates to the zero vector (0/0 -> NaN) when the
    // normal equals that axis exactly - never hit by environments.hpp's
    // near-horizontal scenes, hit by an STL import's vertical faces
    // (docs/paper-deltas.md). y_axis is the exact cross product: orthonormal
    // by construction.
    Vector_3 reference = (std::abs(nx) <= std::abs(ny) && std::abs(nx) <= std::abs(nz)) ? Vector_3(1, 0, 0)
                        : (std::abs(ny) <= std::abs(nz))                                ? Vector_3(0, 1, 0)
                                                                                         : Vector_3(0, 0, 1);

    Vector_3 x_axis = reference - (reference * frame_norm) * frame_norm;
    x_axis = x_axis / std::sqrt(x_axis.squared_length());

    Vector_3 y_axis = CGAL::cross_product(frame_norm, x_axis);

    transform_to_3d = Transformation(
        x_axis.x(), y_axis.x(), norm.x(), centroid.x(),
        x_axis.y(), y_axis.y(), norm.y(), centroid.y(),
        x_axis.z(), y_axis.z(), norm.z(), centroid.z()
    );

    transform_to_surface = transform_to_3d.inverse();
}

} // namespace nas
