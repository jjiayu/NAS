#pragma once

// Surface is csp::Surface (Convex Surface Processing, github.com/ipab-rwa/cspplusplus): a convex
// planar patch fitted from 3D points, with its own frame (transform_to_3d/transform_to_surface),
// footprint (polygon_2d/vertices_2d/vertices_3d) and surface_id. This header only names it inside
// nas:: and adds what NAS does with it: fit a scene's surfaces, and erode them by a margin.
//
// The margin replaced the old foot-size hack (each vertex moved foot_length/2 in x and foot_width/2
// in y in the local frame, right only for a rectangle aligned with its frame -- it also mirrored a
// surface narrower than the foot instead of removing it). Measured identical to the hack, to 1e-9,
// on every surface of every built-in scenario at the default margin.

#include "nas/core/types.hpp"

#include <csp/surface.hpp>

#include <optional>
#include <vector>

namespace nas {

using Surface = csp::Surface;

// Default footprint erosion, in metres: half of Talos's 0.22 m foot, so a footstep placed on the
// eroded boundary keeps the whole foot on the real surface.
constexpr double kDefaultInnerMargin = 0.11;

// One RAW (un-eroded) Surface per point list; surface_id == index in the returned vector. This is
// what a Scenario holds: the scene's true geometry, no margin applied. Throws std::invalid_argument
// (from csp) on a list with fewer than 3 points or all collinear.
std::vector<Surface> make_surfaces(const std::vector<std::vector<Point_3>>& raw_surfaces);

// Each surface eroded inward by `margin` (Surface::inner_margin, a parallel-edge offset), indexed by
// surface id: entry i is nullopt when the margin collapses surface i (thinner than 2 x margin). A
// collapsed surface keeps its index, so ids never shift. Warns on stderr for each collapsed one.
std::vector<std::optional<Surface>> erode_by_id(const std::vector<Surface>& surfaces, double margin);

// The same, compacted: collapsed surfaces are dropped, the others keep their surface_id (so ids may
// have gaps). For callers that just iterate over usable footprints (expand_node, GridEnvironment).
std::vector<Surface> erode_surfaces(const std::vector<Surface>& surfaces, double margin);

} // namespace nas
