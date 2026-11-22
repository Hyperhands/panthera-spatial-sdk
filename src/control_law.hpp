#pragma once
#include "panthera/controller.hpp"
namespace panthera::detail {
class ControlLaw {
  public:
    explicit ControlLaw(ControlConfig config = {});
    void reset(const JointVector& measured);
    void step(const JointVector& measured, const std::array<ArmTarget, 2>& targets);
    void set_task_weights(const Six& weights);
    void set_arm_task_weights(unsigned side, const Six& weights);
    void freeze();
    ControlState state;

  private:
    void observe(const JointVector&);
    std::array<ArmGeometry, 2> arms_, ref_arms_;
    std::array<Eigen::Isometry3d, 2> interpolated_;
    std::array<TargetType, 2> previous_mode_{};
    std::array<Six, 2> task_weights_;
    JointVector nominal_, hard_lower_, hard_upper_, tracking_stop_;
    CollisionState current_, ref_collision_, candidate_;
    std::vector<CollisionRow> qp_rows_;
    ControlConfig config_;
    MotionQp qp_;
};
} // namespace panthera::detail
