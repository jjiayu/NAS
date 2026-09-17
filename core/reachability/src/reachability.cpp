#include "nas/core/reachability.hpp"

#include <CGAL/IO/polygon_mesh_io.h>

namespace nas {

namespace {

Polyhedron load_obj(const std::string& path) {
    Polyhedron polyhedron;
    if (!CGAL::IO::read_polygon_mesh(path, polyhedron)) {
        throw std::runtime_error("ReachabilityModel: failed to load polyhedron from: " + path);
    }
    return polyhedron;
}

} // namespace

ReachabilityModel ReachabilityModel::load(const std::vector<ReachabilityEntry>& entries) {
    ReachabilityModel model;
    for (const auto& entry : entries) {
        Key key{entry.moving_effector, entry.support_effector, entry.direction};
        model.polytopes_[key] = load_obj(entry.path);
    }
    return model;
}

bool ReachabilityModel::has(const std::string& moving_effector,
                             const std::string& support_effector,
                             ReachabilityDirection direction) const {
    return polytopes_.find(Key{moving_effector, support_effector, direction}) != polytopes_.end();
}

const Polyhedron& ReachabilityModel::query(const std::string& moving_effector,
                                            const std::string& support_effector,
                                            ReachabilityDirection direction) const {
    auto it = polytopes_.find(Key{moving_effector, support_effector, direction});
    if (it == polytopes_.end()) {
        throw std::out_of_range(
            "ReachabilityModel::query: no entry for moving_effector='" + moving_effector +
            "', support_effector='" + support_effector + "'");
    }
    return it->second;
}

} // namespace nas
