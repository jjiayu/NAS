#include "nas/core/surface.hpp"

#include <cstdio>

namespace nas {

std::vector<Surface> make_surfaces(const std::vector<std::vector<Point_3>>& raw_surfaces) {
    std::vector<Surface> surfaces;
    surfaces.reserve(raw_surfaces.size());
    for (size_t i = 0; i < raw_surfaces.size(); ++i) {
        surfaces.emplace_back(raw_surfaces[i], static_cast<int>(i));
    }
    return surfaces;
}

std::vector<std::optional<Surface>> erode_by_id(const std::vector<Surface>& surfaces, double margin) {
    std::vector<std::optional<Surface>> eroded;
    eroded.reserve(surfaces.size());
    for (const Surface& s : surfaces) {
        eroded.push_back(s.inner_margin(margin));
        if (!eroded.back()) {
            std::fprintf(stderr, "erode_by_id: surface %d is thinner than 2 x margin (%g m), unusable\n", s.surface_id, margin);
        }
    }
    return eroded;
}

std::vector<Surface> erode_surfaces(const std::vector<Surface>& surfaces, double margin) {
    std::vector<Surface> kept;
    for (std::optional<Surface>& s : erode_by_id(surfaces, margin)) {
        if (s) kept.push_back(std::move(*s));
    }
    return kept;
}

} // namespace nas
