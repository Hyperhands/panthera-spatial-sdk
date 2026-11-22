#include "control_law.hpp"
#include "servo_clock.hpp"
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
void add_warning(std::string& dst, const char* msg) {
    if (dst.find(msg) != std::string::npos)
        return;
    if (!dst.empty())
        dst += "; ";
    dst += msg;
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
    // Hard tracking stop only triggers on severe physical stall unless explicitly configured.
    tracking_stop_ = (3. * b.max_lead.array()).max(0.45);
    if (config_.tracking_stop_rad) {
        const double stop = *config_.tracking_stop_rad;
        if (!std::isfinite(stop) || stop <= 0.)
            throw std::invalid_argument("Invalid joint tracking stop");
        tracking_stop_.setConstant(stop);
    }
    constexpr double hard_limit_margin = 10. * std::numbers::pi / 180.;
    hard_lower_ = model.lower.array() - hard_limit_margin;
    hard_upper_ = model.upper.array() + hard_limit_margin;
    for (auto gain : {config_.pose_gain, config_.joint_gain, config_.regularization,
                      config_.translation_speed, config_.rotation_speed})
        if (!std::isfinite(gain) || gain <= 0)
            throw std::invalid_argument("Invalid control gain or rate");
    for (auto weight : {config_.posture_weight, config_.smoothing})
        if (!std::isfinite(weight) || weight < 0)
            throw std::invalid_argument("Invalid control weight");
    task_weights_ = {config_.task_weights, config_.task_weights};
}
void ControlLaw::observe(const JointVector& q) {
    if (!q.allFinite())
        throw std::runtime_error("Nonfinite joint feedback");
    if ((q.array() < hard_lower_.array()).any() || (q.array() > hard_upper_.array()).any())
        throw std::runtime_error("Measured joint outside mechanical limits");
    if ((q.array() < config_.joints.lower.array() - 1e-5).any() ||
        (q.array() > config_.joints.upper.array() + 1e-5).any())
        add_warning(state.warning, "Soft joint limit reached");
    state.measured = q;
    for (unsigned side = 0; side < 2; ++side) {
        arms_[side] = arm_geometry(side, q.segment<6>(6 * side));
        state.tcp[side] = arms_[side].tcp();
    }
    collision_state(arms_, current_);
    state.closest_pair = current_.pair;
    state.clearance = current_.closest;
    state.hard_margin = current_.hard_margin;
    if (!(current_.inter_arm >= 0.005))
        throw std::runtime_error("Inter-arm collision hard threshold");
    if (!(current_.hard_margin >= 0))
        add_warning(state.warning, "Same-arm clearance boundary: holding");
}
void ControlLaw::reset(const JointVector& q) {
    state = {};
    if (config_.bypass_qp) {
        if (!q.allFinite())
            throw std::runtime_error("Nonfinite joint feedback");
        state.measured = q;
        for (unsigned side = 0; side < 2; ++side) {
            arms_[side] = arm_geometry(side, q.segment<6>(6 * side));
            state.tcp[side] = arms_[side].tcp();
        }
    } else {
        observe(q);
    }
    state.reference = nominal_ = q;
    interpolated_ = state.tcp;
    previous_mode_.fill(TargetType::joint);
}
void ControlLaw::set_task_weights(const Six& weights) {
    if (!weights.allFinite() || (weights.array() < 0).any())
        throw std::invalid_argument("Invalid task weights");
    config_.task_weights = weights;
    task_weights_ = {weights, weights};
}
void ControlLaw::set_arm_task_weights(unsigned side, const Six& weights) {
    if (side > 1 || !weights.allFinite() || (weights.array() < 0).any())
        throw std::invalid_argument("Invalid task weights");
    task_weights_[side] = weights;
}
void ControlLaw::freeze() {
    state.frozen = true;
    state.velocity.setZero();
    interpolated_ = state.tcp;
}
void ControlLaw::step(const JointVector& measured, const std::array<ArmTarget, 2>& targets) {
    if (config_.bypass_qp) {
        state.warning.clear();
        if (!measured.allFinite())
            throw std::runtime_error("Nonfinite joint feedback");
        state.measured = measured;
        state.frozen = false;
        state.consecutive_failures = 0;
        state.qp_time = {};
        for (size_t s = 0; s < 2; ++s) {
            const int idx = 6 * static_cast<int>(s);
            const auto& t = targets[s];
            for (int j = 0; j < 6; ++j) {
                const int a = idx + j;
                const double goal =
                    std::clamp(t.joint[j], config_.joints.lower[a], config_.joints.upper[a]);
                const double err = goal - state.reference[a];
                const double vmax = config_.joints.max_speed[a];
                const double amax = config_.joints.max_acceleration[a];
                const double reach = vmax * control_dt;
                if (std::abs(err) <= reach) {
                    state.reference[a] = goal;
                    state.velocity[a] = 0.;
                } else {
                    const double dir = err > 0. ? 1. : -1.;
                    const double brake_v = std::min(vmax, std::sqrt(2. * amax * std::abs(err)));
                    const double dv = amax * control_dt;
                    const double v =
                        state.velocity[a] + std::clamp(dir * brake_v - state.velocity[a], -dv, dv);
                    state.velocity[a] = v;
                    state.reference[a] += v * control_dt;
                    if ((goal - state.reference[a]) * dir < 0.) {
                        state.reference[a] = goal;
                        state.velocity[a] = 0.;
                    }
                }
            }
        }
        return;
    }
    const auto deadline = ServoClock::now() + std::chrono::milliseconds(2);
    state.warning.clear();
    observe(measured);
    if (((state.reference - measured).array().abs() > tracking_stop_.array()).any())
        throw std::runtime_error("Joint tracking error exceeds stop threshold");
    state.frozen = false;
    // Clamp reference back inside max_lead so moderate lag reduces torque instead of deadlocking QP.
    for (int j = 0; j < 12; ++j) {
        const double lead = state.reference[j] - measured[j];
        const double limit = config_.joints.max_lead[j];
        if (std::abs(lead) > limit) {
            state.reference[j] = measured[j] + std::copysign(.95 * limit, lead);
            state.velocity[j] = 0.;
            add_warning(state.warning, "Joint tracking lag clamped");
        }
    }
    const double regularization = config_.regularization, smoothing = config_.smoothing,
                 posture = config_.posture_weight;
    JointMatrix h = (regularization + smoothing) * JointMatrix::Identity();
    JointVector linear = -smoothing * state.velocity;
    const Six wrist_sweet_spot = (Six() << 0., 0., 0., -.5, 0., 0.).finished();
    for (size_t s = 0; s < 2; ++s) {
        const int index = 6 * s;
        auto& t = targets[s];
        if (t.type == TargetType::joint) {
            h.block<6, 6>(index, index).diagonal().array() += 1. + posture;
            linear.segment<6>(index) -=
                posture * (nominal_.segment<6>(index) - state.reference.segment<6>(index));
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
            // In task space, bias wrist joints (J4–J6) more strongly toward their comfortable
            // mid-range sweet spot so position-only dragging avoids wrist joint-limit walls.
            for (int j = 0; j < 6; ++j) {
                const int axis = index + j;
                const double w_posture = j >= 3 ? 5. * posture : posture;
                const double target_q = j >= 3 ? wrist_sweet_spot[j] : nominal_[axis];
                h(axis, axis) += w_posture;
                linear[axis] -= w_posture * (target_q - state.reference[axis]);
            }
            Six v = t.twist;
            const auto& w = task_weights_[s];
            if (t.type == TargetType::pose) {
                auto& p = interpolated_[s];
                if (previous_mode_[s] != TargetType::pose)
                    p = state.tcp[s];
                // Anti-windup: prevent p from wandering deep past a collision wall or joint limit,
                // and reset immediately when the target reverses to the opposite side of current TCP.
                const Vec3 p_err = p.translation() - state.tcp[s].translation();
                const Vec3 t_err = t.pose.translation() - state.tcp[s].translation();
                if (p_err.dot(t_err) < 0.)
                    p.translation() = state.tcp[s].translation();
                else {
                    const double max_windup = std::max(
                        0.015, 1.2 * config_.translation_speed / std::max(config_.pose_gain, 0.5));
                    p.translation() = state.tcp[s].translation() + bounded(p_err, max_windup);
                }
                p.translation() += bounded(t.pose.translation() - p.translation(),
                                           config_.translation_speed * control_dt);
                if ((w.tail<3>().array() <= 1e-12).all()) {
                    p.linear() = state.tcp[s].linear();
                } else {
                    const double max_rot_windup = std::max(
                        0.15, 1.2 * config_.rotation_speed / std::max(config_.pose_gain, 0.5));
                    Eigen::AngleAxisd cur_aa(state.tcp[s].linear().transpose() * p.linear());
                    if (cur_aa.angle() > max_rot_windup)
                        p.linear() =
                            state.tcp[s].linear() *
                            Eigen::AngleAxisd(max_rot_windup, cur_aa.axis()).toRotationMatrix();
                    Eigen::AngleAxisd aa(p.linear().transpose() * t.pose.linear());
                    p.linear() =
                        p.linear() *
                        Eigen::AngleAxisd(std::min(aa.angle(), config_.rotation_speed * control_dt),
                                          aa.axis())
                            .toRotationMatrix();
                }
                v += config_.pose_gain * pose_error(state.tcp[s], p);
            }
            const auto& j = arms_[s].jacobian;
            h.block<6, 6>(index, index).noalias() += j.transpose() * w.asDiagonal() * j;
            linear.segment<6>(index).noalias() -= j.transpose() * w.asDiagonal() * v;
        }
        previous_mode_[s] = t.type;
    }
    // If measured or reference slightly exceeds any configured soft limit, forbid moving
    // further outward while still allowing smooth return inside [lower, upper].
    MotionBounds step_bounds = config_.joints;
    for (int j = 0; j < 12; ++j) {
        if (measured[j] < step_bounds.lower[j] || state.reference[j] < step_bounds.lower[j])
            step_bounds.lower[j] =
                std::min({step_bounds.lower[j], state.reference[j],
                          measured[j] + .95 * step_bounds.max_lead[j]});
        if (measured[j] > step_bounds.upper[j] || state.reference[j] > step_bounds.upper[j])
            step_bounds.upper[j] =
                std::max({step_bounds.upper[j], state.reference[j],
                          measured[j] - .95 * step_bounds.max_lead[j]});
    }
    // Use same-arm rows at measured and inter-arm rows linearized at state.reference
    // (taking the stricter inter-arm damper when measured is closer than reference),
    // preventing reference-vs-measured deadlocks on real motors while keeping actual
    // inter-arm clearance near 20 mm.
    for (unsigned side = 0; side < 2; ++side)
        ref_arms_[side] = arm_geometry(side, state.reference.segment<6>(6 * side));
    collision_state(ref_arms_, ref_collision_);
    qp_rows_.clear();
    for (const auto& r : current_.rows)
        if (r.same_arm)
            qp_rows_.push_back(r);
    for (auto r : ref_collision_.rows) {
        if (r.same_arm)
            continue;
        for (const auto& cur : current_.rows)
            if (!cur.same_arm && cur.pair == r.pair) {
                r.lower = std::max(r.lower, cur.lower);
                break;
            }
        if (r.lower > 0.15)
            r.lower = 0.15;
        qp_rows_.push_back(r);
    }

    QpStep step;
    state.qp_time = {};
    int remaining_solves = 3;
    for (int pass = 0; pass < 2 && remaining_solves > 0; ++pass) {
        step = qp_.solve(measured, state.reference, state.velocity, h, linear, step_bounds,
                         qp_rows_, remaining_solves);
        remaining_solves -= step.solves;
        state.qp_time += step.solve_time;
        if (!step.success)
            break;
        std::array<ArmGeometry, 2> predicted{arm_geometry(0, step.position.head<6>()),
                                             arm_geometry(1, step.position.tail<6>())};
        collision_state(predicted, candidate_);
        const bool same_arm_ok =
            current_.hard_margin >= 0 ? candidate_.hard_margin >= 0
                                      : candidate_.hard_margin >= current_.hard_margin - 1e-6;
        step.success = candidate_.inter_arm >= 0.005 && same_arm_ok;
        // A tangential joint step can cross a curved 20 mm surface even
        // when the linearized derivative is zero. Correct the violated row.
        for (auto& predicted_row : candidate_.rows) {
            if (predicted_row.same_arm || predicted_row.clearance_margin >= 0)
                continue;
            bool matched = false;
            for (auto& row : qp_rows_)
                if (!row.same_arm && row.pair == predicted_row.pair) {
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
        if (ServoClock::now() > deadline) {
            step.success = false;
            break;
        }
        if (step.success || candidate_.inter_arm < 0.005 || !same_arm_ok)
            break;
    }
    if (!step.success) {
        freeze();
        ++state.consecutive_failures;
        add_warning(state.warning, "QP infeasible: holding current pose");
        return;
    }
    state.consecutive_failures = 0;
    state.reference = step.position;
    state.velocity = step.velocity;
}
} // namespace panthera::detail
