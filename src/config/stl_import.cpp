#include "nas/config/stl_import.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace nas::config {

namespace {

struct Triangle {
    Point_3 v0, v1, v2;
    Vector_3 normal;
};

std::vector<Triangle> parse_binary_stl(std::ifstream& file, uint32_t triangle_count) {
    std::vector<Triangle> triangles;
    triangles.reserve(triangle_count);
    for (uint32_t i = 0; i < triangle_count; ++i) {
        float normal[3];
        float coords[9];
        uint16_t attribute_byte_count;
        file.read(reinterpret_cast<char*>(normal), sizeof(normal));
        file.read(reinterpret_cast<char*>(coords), sizeof(coords));
        file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(attribute_byte_count));

        Triangle t;
        t.normal = Vector_3(normal[0], normal[1], normal[2]);
        t.v0 = Point_3(coords[0], coords[1], coords[2]);
        t.v1 = Point_3(coords[3], coords[4], coords[5]);
        t.v2 = Point_3(coords[6], coords[7], coords[8]);
        triangles.push_back(t);
    }
    return triangles;
}

std::vector<Triangle> parse_ascii_stl(const std::string& path) {
    std::ifstream file(path);
    std::vector<Triangle> triangles;
    Triangle current;
    int vertex_index = 0;
    std::string token;
    while (file >> token) {
        if (token == "facet") {
            file >> token; // "normal"
            double nx, ny, nz;
            file >> nx >> ny >> nz;
            current.normal = Vector_3(nx, ny, nz);
            vertex_index = 0;
        } else if (token == "vertex") {
            double x, y, z;
            file >> x >> y >> z;
            Point_3 p(x, y, z);
            if (vertex_index == 0) current.v0 = p;
            else if (vertex_index == 1) current.v1 = p;
            else if (vertex_index == 2) current.v2 = p;
            ++vertex_index;
        } else if (token == "endfacet") {
            triangles.push_back(current);
        }
    }
    return triangles;
}

std::vector<Triangle> parse_stl(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("load_scenario_from_stl: could not open '" + path + "'");
    }

    file.seekg(0, std::ios::end);
    std::streamoff file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file_size < 84) {
        throw std::runtime_error("load_scenario_from_stl: '" + path + "' is too small to be a valid STL file");
    }

    char header[80];
    file.read(header, sizeof(header));
    uint32_t declared_triangle_count = 0;
    file.read(reinterpret_cast<char*>(&declared_triangle_count), sizeof(declared_triangle_count));

    // Binary STL's size is fully determined by its declared triangle count
    // (84-byte header + 50 bytes/triangle) — an exact match is the
    // standard, reliable way to distinguish it from ASCII (an ASCII file
    // starting with "solid" would almost never happen to match this size).
    std::streamoff expected_binary_size = 84 + static_cast<std::streamoff>(declared_triangle_count) * 50;

    std::vector<Triangle> triangles;
    if (file_size == expected_binary_size) {
        triangles = parse_binary_stl(file, declared_triangle_count);
    } else {
        triangles = parse_ascii_stl(path);
    }

    if (triangles.size() < 3) {
        throw std::runtime_error("load_scenario_from_stl: '" + path + "' has fewer than 3 triangles (or failed to parse)");
    }
    return triangles;
}

using PlaneKey = std::tuple<long, long, long, long>;
constexpr double kPlaneQuantization = 1e4; // ~1e-4 tolerance on (nx, ny, nz, d)

PlaneKey quantize_plane(const Vector_3& unit_normal, double d) {
    return {
        std::lround(CGAL::to_double(unit_normal.x()) * kPlaneQuantization),
        std::lround(CGAL::to_double(unit_normal.y()) * kPlaneQuantization),
        std::lround(CGAL::to_double(unit_normal.z()) * kPlaneQuantization),
        std::lround(d * kPlaneQuantization),
    };
}

// Groups triangles sharing (approximately) the same plane into one raw
// vertex list per group — the same shape config::load_scenario()'s
// hardcoded scenarios already provide to Surface's constructor. Falls back
// to the vertices' own cross product when a triangle's declared normal is
// zero-length (some exporters omit it); triangles that are degenerate
// either way are silently dropped, matching core/geometry's own tolerance
// for a single bad facet (see docs/paper-deltas.md, 8d-1).
std::vector<std::vector<Point_3>> group_by_plane(const std::vector<Triangle>& triangles) {
    std::map<PlaneKey, std::vector<Point_3>> groups;

    for (const auto& t : triangles) {
        Vector_3 normal = t.normal;
        double norm = std::sqrt(CGAL::to_double(normal.squared_length()));
        if (norm < 1e-12) {
            normal = CGAL::cross_product(t.v1 - t.v0, t.v2 - t.v0);
            norm = std::sqrt(CGAL::to_double(normal.squared_length()));
        }
        if (norm < 1e-12) {
            continue; // degenerate triangle (collinear or duplicate vertices)
        }
        normal = normal / norm;
        double d = CGAL::to_double(normal * (t.v0 - CGAL::ORIGIN));

        std::vector<Point_3>& verts = groups[quantize_plane(normal, d)];
        verts.push_back(t.v0);
        verts.push_back(t.v1);
        verts.push_back(t.v2);
    }

    std::vector<std::vector<Point_3>> raw_surfaces;
    raw_surfaces.reserve(groups.size());
    for (auto& [key, verts] : groups) {
        raw_surfaces.push_back(std::move(verts));
    }
    return raw_surfaces;
}

std::string scenario_name_from_path(const std::string& stl_path) {
    size_t slash = stl_path.find_last_of("/\\");
    std::string filename = (slash == std::string::npos) ? stl_path : stl_path.substr(slash + 1);
    size_t dot = filename.find_last_of('.');
    return (dot == std::string::npos) ? filename : filename.substr(0, dot);
}

} // namespace

Scenario load_scenario_from_stl(const std::string& stl_path, const RobotModel& robot_model) {
    std::vector<Triangle> triangles = parse_stl(stl_path);
    std::vector<std::vector<Point_3>> raw_surfaces = group_by_plane(triangles);

    Scenario scenario;
    scenario.name = scenario_name_from_path(stl_path);
    for (size_t i = 0; i < raw_surfaces.size(); ++i) {
        scenario.surfaces.emplace_back(raw_surfaces[i], static_cast<int>(i), robot_model.foot_length, robot_model.foot_width);
    }
    return scenario;
}

} // namespace nas::config
