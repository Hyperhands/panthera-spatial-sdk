#pragma once
#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <span>
namespace panthera {
using JointVector = Eigen::Matrix<double, 12, 1>;
using JointMatrix = Eigen::Matrix<double, 12, 12>;
inline constexpr double control_dt = .002;
struct CollisionRow {
    JointVector gradient;
    double lower, clearance_margin;
};
struct MotionBounds {
    JointVector lower, upper, max_speed, max_acceleration, max_lead;
};
struct QpStep {
    JointVector position, velocity;
    bool success{};
    int solves{};
    std::chrono::nanoseconds solve_time{};
};
// N=1 with the known initial state substituted into x1 = reference + dt*u.
class MotionQp {
  public:
    MotionQp();
    ~MotionQp();
    QpStep solve(const JointVector& measured, const JointVector& reference,
                 const JointVector& previous, const JointMatrix& hessian, const JointVector& linear,
                 const MotionBounds& bounds, std::span<const CollisionRow> rows);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace panthera
