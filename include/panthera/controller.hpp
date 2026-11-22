#pragma once
#include "panthera/geometry.hpp"
#include "panthera/session.hpp"
#include <optional>

namespace panthera {
enum class TargetType { joint, pose, twist };
struct ArmTarget {
    TargetType type = TargetType::joint;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Six joint = Six::Zero(), twist = Six::Zero();
};
struct ControlConfig {
    MotionBounds joints = default_bounds();
    Six task_weights = (Six() << 1., 1., 1., .15, .15, .15).finished();
    double pose_gain = 10., joint_gain = 8.;
    double regularization = .0001, smoothing = .001, posture_weight = .00001;
    double translation_speed = .2, rotation_speed = 1.2;
    // When unset, retain the per-axis stop at twice the configured lead.
    std::optional<double> tracking_stop_rad;
};
struct ControlState {
    bool enabled{}, frozen{}, expert{};
    int consecutive_failures{}, closest_pair = -1;
    double clearance{}, hard_margin{};
    JointVector measured = JointVector::Zero(), reference = JointVector::Zero(),
                velocity = JointVector::Zero();
    std::array<Eigen::Isometry3d, 2> tcp{Eigen::Isometry3d::Identity(),
                                         Eigen::Isometry3d::Identity()};
    std::array<BoardState, 2> boards;
    std::array<RpcTiming, 2> rpc;
    std::uint64_t cycles{}, missed_cycles{};
    std::chrono::nanoseconds qp_time{};
    std::string warning;
    std::string error;
};
// World-frame TCP twist is ordered [vx,vy,vz,wx,wy,wz].
enum class ExpertAck { IUnderstandNoSafety };
class DualArm {
  public:
    DualArm(BoardAddress left = BoardAddress::left(), BoardAddress right = BoardAddress::right(),
            ControlConfig config = {});
    ~DualArm();
    void enable(std::chrono::milliseconds timeout = std::chrono::seconds(30));
    // check_interrupt may throw while waiting; a partial takeover is rolled back.
    void enable(std::chrono::milliseconds timeout, void (*check_interrupt)());
    void setPoseTarget(Side side, const Eigen::Isometry3d& pose,
                       const Six& feedforward = Six::Zero());
    void setTwistTarget(Side side, const Six& twist);
    void setJointTarget(Side side, const Six& angles);
    void setHandTarget(Side side, const Joints& angles, std::uint16_t duration_ms);
    void setTaskWeights(const Six& weights);
    void setArmTaskWeights(Side side, const Six& weights);
    ControlState state() const;
    void softStop();
    void enterExpertMode(ExpertAck);
    void sendMit(Side side, const std::array<float, 30>& raw);
    std::array<std::uint16_t, 2> local_ports(Side side) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace panthera
