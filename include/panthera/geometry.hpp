#pragma once
#include "panthera/motion_qp.hpp"
#include <Eigen/Geometry>
#include <array>
#include <vector>
namespace panthera {
using Six = Eigen::Matrix<double, 6, 1>;
using Jacobian = Eigen::Matrix<double, 6, 6>;
struct ArmGeometry {
    std::array<Eigen::Isometry3d, 9> frames;
    Jacobian jacobian;
    Six gravity;
    const Eigen::Isometry3d& tcp() const { return frames.back(); }
};
ArmGeometry arm_geometry(unsigned arm, const Six& q);
MotionBounds default_bounds();
struct CollisionState {
    double closest = std::numeric_limits<double>::infinity();
    double hard_margin = std::numeric_limits<double>::infinity();
    int pair = -1;
    std::vector<CollisionRow> rows;
};
void collision_state(const std::array<ArmGeometry, 2>& arms, CollisionState& state);
} // namespace panthera
