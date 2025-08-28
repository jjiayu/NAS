#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <stdexcept>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron);

bool cycle_path_detection(const Node* parent, const int current_stance_foot, const int surface_id);

} // namespace nas