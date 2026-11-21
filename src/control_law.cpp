#include "control_law.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace panthera::detail {
namespace {
using Vec3 = Eigen::Vector3d;
Vec3 bounded(Vec3 v, double limit) {
    double norm = v.norm();
    if (norm > limit)
        v *= limit / norm;
    return v;
}
// Body SE(3) logarithm, rotated into the world-aligned TCP Jacobian frame.
Six pose_error(const Eigen::Isometry3d& current, const Eigen::Isometry3d& target) {
    Eigen::Isometry3d local = current.inverse() * target;
    Eigen::AngleAxisd aa(local.linear());
    Vec3 omega = aa.angle() * aa.axis();
    Vec3 pos = local.translation();
    double theta = aa.angle();
    double coeff =
        theta < 1e-4 ? 1. / 12. : (1. - .5 * theta / std::tan(.5 * theta)) / (theta * theta);
    Six error;
    error.head<3>() =
        current.linear() * (pos - .5 * omega.cross(pos) + coeff * omega.cross(omega.cross(pos)));
    error.tail<3>() = current.linear() * omega;
    return error;
}
} // namespace
ControlLaw::ControlLaw(ControlConfig config) : config_(std::move(config)) {
    const auto& b = config_.joints;
    const auto model = default_bounds();
    if (!b.lower.allFinite() || !b.upper.allFinite() || !b.max_speed.allFinite() ||
        !b.max_acceleration.allFinite() || !b.max_lead.allFinite() ||
        (b.lower.array() < model.lower.array()).any() ||
        (b.upper.array() > model.upper.array()).any() ||
        (b.lower.array() >= b.upper.array()).any() || (b.max_speed.array() <= 0).any() ||
        (b.max_acceleration.array() <= 0).any() || (b.max_lead.array() <= 0).any() ||
        !config_.task_weights.allFinite() || (config_.task_weights.array() < 0).any())
        throw std::invalid_argument("Invalid joint limits or task weights");
    tracking_stop_ = 2 * b.max_lead;
    if (config_.tracking_stop_rad) {
        const double stop = *config_.tracking_stop_rad;
        if (!std::isfinite(stop) || stop <= 0.)
            throw std::invalid_argument("Invalid joint tracking stop");
        tracking_stop_.setConstant(stop);
    }
    measured_lower_ = b.lower.array() - 1e-5;
    for (int j : {1, 7})
        if (b.lower[j] == 0.)
            measured_lower_[j] = -std::numbers::pi / 180.;
    for (auto gain : {config_.pose_gain, config_.joint_gain, config_.regularization,
                      config_.translation_speed, config_.rotation_speed})
        if (!std::isfinite(gain) || gain <= 0)
            throw std::invalid_argument("Invalid control gain or rate");
    for (auto weight : {config_.posture_weight, config_.smoothing})
        if (!std::isfinite(weight) || weight < 0)
            throw std::invalid_argument("Invalid control weight");
}
void ControlLaw::observe(const JointVector& q) {
    if (!q.allFinite())
        throw std::runtime_error("Nonfinite joint feedback");
    if ((q.array() < measured_lower_.array()).any() ||
        (q.array() > config_.joints.upper.array() + 1e-5).any())
        throw std::runtime_error("Measured joint outside configured limits");
    state.measured = q;
    for (unsigned side = 0; side < 2; ++side) {
        arms_[side] = arm_geometry(side, q.segment<6>(6 * side));
        state.tcp[side] = arms_[side].tcp();
    }
    collision_state(arms_, current_);
    state.closest_pair = current_.pair;
    state.clearance = current_.closest;
    state.hard_margin = current_.hard_margin;
    if (!(current_.hard_margin >= 0))
        throw std::runtime_error("Measured collision hard threshold");
}
void ControlLaw::reset(const JointVector& q) {
    state = {};
    observe(q);
    state.reference = nominal_ = q;
    interpolated_ = state.tcp;
    previous_mode_.fill(TargetType::joint);
}
void ControlLaw::freeze() {
    state.frozen = true;
    state.velocity.setZero();
    interpolated_ = state.tcp;
}
void ControlLaw::step(const JointVector& measured, const std::array<ArmTarget, 2>& targets) {
    const auto deadline = HostClock::now() + std::chrono::milliseconds(2);
    observe(measured);
    if (((state.reference - measured).array().abs() > tracking_stop_.array()).any())
        throw std::runtime_error("Joint tracking error exceeds stop threshold");
    state.frozen = false;
    const double regularization = config_.regularization, smoothing = config_.smoothing,
                 posture = config_.posture_weight;
    JointMatrix h = (regularization + smoothing + posture) * JointMatrix::Identity();
    JointVector linear = -smoothing * state.velocity - posture * (nominal_ - state.reference);
    for (size_t s = 0; s < 2; ++s) {
        const int index = 6 * s;
        auto& t = targets[s];
        if (t.type == TargetType::joint) {
            h.block<6, 6>(index, index).diagonal().array() += 1;
            for (int j = 0; j < 6; ++j) {
                const int axis = index + j;
                const double reference = state.reference[axis];
                const double desired = config_.joint_gain * (t.joint[j] - reference);
                const double lead = reference - measured[axis];
                // Keep 5% of the lead window for feedback jitter at rest.
                const double room =
                    .95 * config_.joints.max_lead[axis] - (desired < 0 ? -lead : lead);
                // Budget one sample of travel plus v²/(2a) to brake at the tracking boundary.
                // Shape only the joint objective so collision avoidance can still override it.
                const double a = config_.joints.max_acceleration[axis];
                const double dv = a * control_dt;
                const double speed = std::sqrt(dv * dv + 2 * a * std::max(0., room)) - dv;
                linear[axis] -= std::copysign(std::min(std::abs(desired), speed), desired);
            }
        } else {
            Six v = t.twist;
            if (t.type == TargetType::pose) {
                auto& p = interpolated_[s];
                if (previous_mode_[s] != TargetType::pose)
                    p = state.tcp[s];
                p.translation() += bounded(t.pose.translation() - p.translation(),
                                           config_.translation_speed * control_dt);
                Eigen::AngleAxisd aa(p.linear().transpose() * t.pose.linear());
                p.linear() =
                    p.linear() *
                    Eigen::AngleAxisd(std::min(aa.angle(), config_.rotation_speed * control_dt),
                                      aa.axis())
                        .toRotationMatrix();
                v += config_.pose_gain * pose_error(state.tcp[s], p);
            }
            const auto& j = arms_[s].jacobian;
            const auto& w = config_.task_weights;
            h.block<6, 6>(index, index).noalias() += j.transpose() * w.asDiagonal() * j;
            linear.segment<6>(index).noalias() -= j.transpose() * w.asDiagonal() * v;
        }
        previous_mode_[s] = t.type;
    }
    // Below nominal zero the takeover reference may stay put or rise. The
    // previous reference is the floor, even if the measured arm sags farther.
    MotionBounds step_bounds = config_.joints;
    for (int j : {1, 7})
        if (step_bounds.lower[j] == 0.)
            step_bounds.lower[j] = std::min(0., state.reference[j]);
    QpStep step;
    state.qp_time = {};
    int remaining_solves = 3;
    for (int pass = 0; pass < 2 && remaining_solves > 0; ++pass) {
        step = qp_.solve(measured, state.reference, state.velocity, h, linear, step_bounds,
                         current_.rows, remaining_solves);
        remaining_solves -= step.solves;
        state.qp_time += step.solve_time;
        if (!step.success)
            break;
        std::array<ArmGeometry, 2> predicted{arm_geometry(0, step.position.head<6>()),
                                             arm_geometry(1, step.position.tail<6>())};
        collision_state(predicted, candidate_);
        step.success = candidate_.hard_margin >= 0;
        // A tangential joint step can cross a curved 20 mm surface even
        // when the linearized derivative is zero. Correct the violated row.
        for (auto& predicted_row : candidate_.rows) {
            if (predicted_row.same_arm || predicted_row.clearance_margin >= 0)
                continue;
            bool matched = false;
            for (auto& row : current_.rows)
                if (row.pair == predicted_row.pair) {
                    matched = true;
                    const double deficit =
                        std::min(0., row.clearance_margin) - predicted_row.clearance_margin;
                    if (deficit > 1e-10) {
                        row.lower = std::max(row.lower, row.gradient.dot(step.velocity) +
                                                            deficit / control_dt + 1e-6);
                        step.success = false;
                    }
                    break;
                }
            if (!matched)
                step.success = false;
        }
        if (HostClock::now() > deadline) {
            step.success = false;
            break;
        }
        if (step.success || candidate_.hard_margin < 0)
            break;
    }
    if (!step.success) {
        freeze();
        if (++state.consecutive_failures >= 5)
            throw std::runtime_error("Five consecutive QP failures");
        return;
    }
    state.consecutive_failures = 0;
    state.reference = step.position;
    state.velocity = step.velocity;
}
} // namespace panthera::detail
