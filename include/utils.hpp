#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <stdexcept>

namespace nas {

void load_obj(const std::string& filename, Polyhedron& polyhedron);

} // namespace nas