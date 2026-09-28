#include "nas/core/surface.hpp"

#include <cstdio>

namespace nas {

std::vector<Surface> make_surfaces(const std::vector<std::vector<Point_3>>& raw_surfaces, double inner_margin) {
    std::vector<Surface> surfaces;
    surfaces.reserve(raw_surfaces.size());
    for (size_t i = 0; i < raw_surfaces.size(); ++i) {
        // The id it will have if it survives: keeps surface_id == index in `surfaces` even when an
        // earlier list was dropped.
        Surface fitted(raw_surfaces[i], static_cast<int>(surfaces.size()));
        std::optional<Surface> eroded = fitted.inner_margin(inner_margin);
        if (!eroded) {
            std::fprintf(stderr, "make_surfaces: surface list %zu is thinner than 2 x inner_margin (%g m), dropped\n", i, inner_margin);
            continue;
        }
        surfaces.push_back(std::move(*eroded));
    }
    return surfaces;
}

} // namespace nas
