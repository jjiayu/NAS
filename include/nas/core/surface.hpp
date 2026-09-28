#pragma once

// Surface is csp::Surface (Convex Surface Processing, github.com/ipab-rwa/cspplusplus): a convex
// planar patch fitted from 3D points, with its own frame (transform_to_3d/transform_to_surface),
// footprint (polygon_2d/vertices_2d/vertices_3d) and surface_id. This header only names it inside
// nas:: and adds the one thing NAS does with it: build a scene's surfaces with an inner margin.
//
// The margin replaces the old foot-size hack (each vertex moved foot_length/2 in x and foot_width/2
// in y in the local frame, right only for a rectangle aligned with its frame -- it also mirrored a
// surface narrower than the foot instead of removing it). Measured identical to the hack, to 1e-9,
// on every surface of every built-in scenario at the default margin.

#include "nas/core/types.hpp"

#include <csp/surface.hpp>

#include <optional>
#include <vector>

namespace nas {

using Surface = csp::Surface;

// Footprint erosion applied to every scene surface by default, in metres: half of Talos's 0.22 m
// foot, so a footstep placed on the eroded boundary keeps the whole foot on the real surface.
constexpr double kDefaultInnerMargin = 0.11;

// One Surface per raw point list, each eroded inward by `inner_margin` (Surface::inner_margin, a
// parallel-edge offset). Ids are the indices in the returned vector: a list whose margin collapses
// it (thinner than 2*inner_margin) is dropped with a warning on stderr and later ids shift down, so
// surface_id == index always holds. Throws std::invalid_argument (from csp) on a list with fewer
// than 3 points or all collinear.
std::vector<Surface> make_surfaces(const std::vector<std::vector<Point_3>>& raw_surfaces, double inner_margin);

} // namespace nas
