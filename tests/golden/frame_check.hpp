#pragma once
// Independent implementation of the paper's Q (Eq. 2: yaw composed with the contact surface's
// rotation) for the tests: uses Eigen::AngleAxis, not core/geometry's Rodrigues code, and gets a
// patch's normal from its own vertices, not from the node's transformation.
#include "nas/core/types.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

namespace nas::test {

inline Eigen::Vector3d to_vec(const Point_3& p) {
    return Eigen::Vector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
}

// Unit normal of a planar polygon (its first three vertices), pointing up.
inline Eigen::Vector3d up_normal_of(const std::vector<Point_3>& v) {
    Eigen::Vector3d n = (to_vec(v[1]) - to_vec(v[0])).cross(to_vec(v[2]) - to_vec(v[0])).normalized();
    return n.z() < 0 ? -n : n;
}

// Q = R_tilt * R_z(yaw); R_tilt = the rotation about e_z x n taking e_z to n. `tilted = false` gives the
// yaw-only frame (the negative control: what the code did before the surface rotation was implemented).
inline Eigen::Matrix3d frame_Q(const Eigen::Vector3d& n, double yaw, bool tilted = true) {
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if (!tilted) return Rz;
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(n);
    if (axis.norm() < 1e-12) return Rz;
    double angle = std::acos(std::max(-1.0, std::min(1.0, n.z())));
    return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix() * Rz;
}

} // namespace nas::test
