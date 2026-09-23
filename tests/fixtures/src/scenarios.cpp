#include "nas/fixtures/scenarios.hpp"

namespace nas::fixtures {

namespace {

AstarSearchConfig make_default_config() {
    AstarSearchConfig config;
    config.start_position = Point_3(0.0, 0.0, 0.0);
    config.start_stance_foot = StanceFoot::Right;
    config.goal_stance_foot = StanceFoot::Left;
    config.heuristic_weight = 10.0;
    config.node_similarity_threshold = 0.02;
    config.expansion_params.rotation_enabled = true;
    config.expansion_params.yaw_discretization_num = 3;
    config.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    config.expansion_params.cycle_detection_enabled = true;
    return config;
}

std::vector<Surface> build_surfaces(const std::vector<std::vector<Point_3>>& raw) {
    std::vector<Surface> surfaces;
    for (size_t i = 0; i < raw.size(); ++i) {
        surfaces.emplace_back(raw[i], static_cast<int>(i), /*foot_length=*/0.22, /*foot_width=*/0.22);
    }
    return surfaces;
}

} // namespace

Scenario make_narrow_passage() {
    Scenario s;
    s.name = "NarrowPassage";
    s.surfaces = build_surfaces({
        {Point_3(-2.0, -2.0, 0.0), Point_3(2.0, -2.0, 0.0), Point_3(2.0, 2.0, 0.0), Point_3(-2.0, 2.0, 0.0)},
        {Point_3(2.0, -0.12, 0.0), Point_3(6.0, -0.12, 0.0), Point_3(6.0, 0.12, 0.0), Point_3(2.0, 0.12, 0.0)},
        {Point_3(6.0, -2.0, 0.0), Point_3(10.0, -2.0, 0.0), Point_3(10.0, 2.0, 0.0), Point_3(6.0, 2.0, 0.0)},
    });
    s.astar_config = make_default_config();
    s.astar_config.goal_location = s.surfaces.back().centroid; // goal_offset = (0,0,0)
    return s;
}

Scenario make_three_paths_nas() {
    Scenario s;
    s.name = "ThreePathsNAS";
    s.surfaces = build_surfaces({
        {Point_3(-0.3, -4.0, 0.0), Point_3(0.3, -4.0, 0.0), Point_3(0.3, 2.0, 0.0), Point_3(-0.3, 2.0, 0.0)},
        {Point_3(0.32, 1.4, 0.0), Point_3(0.72, 1.4, 0.0), Point_3(0.72, 2.0, 0.0), Point_3(0.32, 2.0, 0.0)},
        {Point_3(0.74, 1.4, 0.0), Point_3(1.32, 1.4, 0.0), Point_3(1.32, 2.0, 0.0), Point_3(0.74, 2.0, 0.0)},
        {Point_3(1.34, 1.4, 0.0), Point_3(2.52, 1.4, 0.0), Point_3(2.52, 2.0, 0.0), Point_3(1.34, 2.0, 0.0)},
        {Point_3(2.54, 1.4, 0.0), Point_3(3.12, 1.4, 0.0), Point_3(3.12, 2.0, 0.0), Point_3(2.54, 2.0, 0.0)},
        {Point_3(3.14, 1.4, 0.0), Point_3(3.72, 1.4, 0.0), Point_3(3.72, 2.0, 0.0), Point_3(3.14, 2.0, 0.0)},
        {Point_3(3.74, 1.4, 0.0), Point_3(4.32, 1.4, 0.0), Point_3(4.32, 2.0, 0.0), Point_3(3.74, 2.0, 0.0)},
        {Point_3(0.32, -3.2, 0.0), Point_3(1.2, -3.2, 0.0), Point_3(1.2, -4.0, 0.0), Point_3(0.32, -4.0, 0.0)},
        {Point_3(1.22, -3.2, 0.0), Point_3(2.2, -3.2, 0.0), Point_3(2.2, -4.0, 0.0), Point_3(1.22, -4.0, 0.0)},
        {Point_3(2.22, -3.2, 0.0), Point_3(3.2, -3.2, 0.0), Point_3(3.2, -4.0, 0.0), Point_3(2.22, -4.0, 0.0)},
        {Point_3(3.22, -3.2, 0.0), Point_3(4.32, -3.2, 0.0), Point_3(4.32, -4.0, 0.0), Point_3(3.22, -4.0, 0.0)},
        {Point_3(0.32, -1.0, 0.0), Point_3(1.5, -1.0, 0.0), Point_3(1.5, -2.0, 0.0), Point_3(0.32, -2.0, 0.0)},
        {Point_3(4.34, -4.0, 0.0), Point_3(4.94, -4.0, 0.0), Point_3(4.94, 2.0, 0.0), Point_3(4.34, 2.0, 0.0)},
    });
    s.astar_config = make_default_config();
    // goal_offset = (0.0, 1.0, 0.0) for this scenario specifically, per the
    // old constants.hpp comment ("for 3path NAS goal offset...").
    Point_3 c = s.surfaces.back().centroid;
    s.astar_config.goal_location = Point_3(CGAL::to_double(c.x()), CGAL::to_double(c.y()) + 1.0, CGAL::to_double(c.z()));
    return s;
}

ReachabilityModel make_forward_reachability_model(const std::string& talos_data_dir) {
    std::vector<ReachabilityEntry> entries = {
        {talos_data_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {talos_data_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
    };
    return ReachabilityModel::load(entries);
}

} // namespace nas::fixtures
