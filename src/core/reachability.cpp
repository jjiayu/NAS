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

const HalfSpacePolytopeConstraint& ReachabilityModel::half_space_constraint(
    const std::string& moving_effector, const std::string& support_effector, ReachabilityDirection direction) const {
    Key key{moving_effector, support_effector, direction};
    auto cached = hrep_cache_.find(key);
    if (cached != hrep_cache_.end()) return cached->second;
    // query() throws out_of_range with the right message if the key is missing — reuse it
    // instead of duplicating the check here.
    const Polyhedron& poly = query(moving_effector, support_effector, direction);
    return hrep_cache_.emplace(key, convert_polytope_to_half_space_constraint(poly)).first->second;
}

} // namespace nas
