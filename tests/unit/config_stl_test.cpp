#include "nas/config/stl_import.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace nas::config;
using nas::Point_3;

namespace {

bool near(double a, double b, double eps = 1e-6) { return std::abs(a - b) < eps; }

std::string data_path(const std::string& filename) {
    return std::string(TEST_DATA_DIR) + "/" + filename;
}

// A 1x1x1 cube's 6 faces (2 triangles each, 12 total) must be grouped back
// into exactly 6 surfaces — one per face — not 12.
void test_ascii_cube_groups_into_six_faces(const std::string& filename) {
    Scenario s = load_scenario_from_stl(data_path(filename));
    assert(s.surfaces.size() == 6);
    for (const auto& surface : s.surfaces) {
        // Every face of a unit cube has centroid coordinates that are
        // either 0, 0.5, or 1 on each axis.
        double cx = CGAL::to_double(surface.centroid.x());
        double cy = CGAL::to_double(surface.centroid.y());
        double cz = CGAL::to_double(surface.centroid.z());
        auto is_face_coord = [](double v) { return near(v, 0.0) || near(v, 0.5) || near(v, 1.0); };
        assert(is_face_coord(cx) && is_face_coord(cy) && is_face_coord(cz));
    }
    std::cout << "test_ascii_cube_groups_into_six_faces(" << filename << ") passed\n";
}

void test_binary_and_ascii_agree() {
    Scenario ascii_scenario = load_scenario_from_stl(data_path("cube_ascii.stl"));
    Scenario binary_scenario = load_scenario_from_stl(data_path("cube_binary.stl"));
    assert(ascii_scenario.surfaces.size() == binary_scenario.surfaces.size());
    std::cout << "test_binary_and_ascii_agree passed\n";
}

void test_scenario_name_from_filename() {
    Scenario s = load_scenario_from_stl(data_path("cube_ascii.stl"));
    assert(s.name == "cube_ascii");
    std::cout << "test_scenario_name_from_filename passed\n";
}

void test_too_small_file_throws() {
    bool threw = false;
    try {
        load_scenario_from_stl(data_path("too_small.stl"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_too_small_file_throws passed\n";
}

void test_nonexistent_file_throws() {
    bool threw = false;
    try {
        load_scenario_from_stl(data_path("does_not_exist.stl"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "test_nonexistent_file_throws passed\n";
}

void test_stl_surfaces_are_raw() {
    // The margin is a planner parameter now: the importer returns the mesh's true faces.
    Scenario s = load_scenario_from_stl(data_path("cube_ascii.stl"));
    assert(s.surfaces.size() == 6);
    for (size_t i = 0; i < s.surfaces.size(); ++i) assert(s.surfaces[i].surface_id == static_cast<int>(i));
    std::cout << "test_stl_surfaces_are_raw passed\n";
}

} // namespace

int run_config_stl() {
    test_ascii_cube_groups_into_six_faces("cube_ascii.stl");
    test_binary_and_ascii_agree();
    test_scenario_name_from_filename();
    test_too_small_file_throws();
    test_nonexistent_file_throws();
    test_stl_surfaces_are_raw();
    std::cout << "All config/stl_import tests passed.\n";
    return 0;
}
