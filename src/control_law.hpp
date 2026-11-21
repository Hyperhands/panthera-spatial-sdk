#pragma once
#include "panthera/controller.hpp"
namespace panthera::detail {
class ControlLaw {
  public:
    explicit ControlLaw(ControlConfig config = {});
    void reset(const JointVector& measured);
    void step(const JointVector& measured, const std::array<ArmTarget, 2>& targets);
    void freeze();
    ControlState state;

  private:
    void observe(const JointVector&);
    std::array<ArmGeometry, 2> arms_;
    std::array<Eigen::Isometry3d, 2> interpolated_;
    std::array<TargetType, 2> previous_mode_{};
    JointVector nominal_, measured_lower_;
    CollisionState current_, candidate_;
    ControlConfig config_;
    std::vector<int> startup_pairs_;
    MotionQp qp_;
};
} // namespace panthera::detail
