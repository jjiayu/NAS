// stl_preview — dumps an STL-imported (or named) scenario's surfaces to
// JSON, with no search/QP involved (see PLAN.md phase 9d). Exists purely
// to let a human eyeball whether config::load_scenario_from_stl() grouped
// triangles into the right surfaces before trusting it in a real planning
// run — astar_plan's own scenario loading isn't a substitute, since a
// grouping bug (e.g. a face split into two surfaces, or two faces merged
// into one) wouldn't necessarily make the search fail loudly.

#include "nas/config/scenario.hpp"
#include "nas/config/stl_import.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>

using namespace nas;
using json = nlohmann::json;

namespace {

json point_json(const Point_3& p) {
    return json::array({CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())});
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <stl_path> <output.json>\n";
        return 1;
    }
    std::string stl_path = argv[1];
    std::string out_path = argv[2];

    config::Scenario scenario;
    try {
        scenario = config::load_scenario_from_stl(stl_path);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    json out;
    out["name"] = scenario.name;
    json surfaces_json = json::array();
    for (const auto& s : scenario.surfaces) {
        json surface_j;
        surface_j["surface_id"] = s.surface_id;
        surface_j["centroid"] = point_json(s.centroid);
        json verts = json::array();
        for (const auto& v : s.vertices_3d) verts.push_back(point_json(v));
        surface_j["vertices_3d"] = verts;
        surfaces_json.push_back(surface_j);
    }
    out["surfaces"] = surfaces_json;

    std::ofstream file(out_path);
    file << out.dump(2);
    std::cout << "Wrote " << out_path << " (" << scenario.surfaces.size() << " surfaces)\n";
    return 0;
}
