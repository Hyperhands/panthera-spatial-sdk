#include "control_law.hpp"
#include <iostream>
#include <numbers>
#include <random>
using namespace panthera;
using panthera::detail::ControlLaw;
void check(bool value, const char* msg) {
    if (!value)
        throw std::runtime_error(msg);
}
void check_j2_takeover() {
    constexpr double deg = std::numbers::pi / 180.;
    JointVector field;
    field << -.406, -.055, .857, -1.203, -.352, .566, -.291, -.022, 1.527, -.989, -.280, .478;
    field *= deg;
    ControlLaw law;
    auto rejected = [](ControlLaw& controller, const JointVector& q) {
        try {
            controller.reset(q);
        } catch (const std::runtime_error& e) {
            return std::string(e.what()) == "Measured joint outside configured limits";
        }
        return false;
    };
    for (int j : {1, 7}) {
        for (double angle : {-1., -.5, 0.}) {
            JointVector q = field;
            q[j] = angle * deg;
            law.reset(q);
            check(law.state.reference.isApprox(q), "takeover changed the measured pose");
        }
        JointVector q = field;
        q[j] = -1.01 * deg;
        check(rejected(law, q), "J2 below -1 degree accepted");
        q = field;
        q[j + 1] = -.05 * deg;
        check(rejected(law, q), "J3 lower limit widened");
        ControlConfig tighter;
        tighter.joints.lower[j] = .5 * deg;
        ControlLaw custom(tighter);
        check(rejected(custom, field), "custom J2 limit widened");
    }
    JointVector q = field;
    q[1] = -deg;
    law.reset(q);
    std::array<ArmTarget, 2> targets{};
    for (int side = 0; side < 2; ++side)
        targets[side].joint = q.segment<6>(6 * side);
    const JointVector start = q;
    for (int k = 0; k < 5000; ++k) {
        law.step(q, targets);
        q = law.state.reference;
        check((q - start).lpNorm<Eigen::Infinity>() < .05 * deg, "hold drifted in 10 seconds");
    }
    for (auto& t : targets)
        t.joint[1] = 2 * deg;
    for (int k = 0; k < 1000; ++k) {
        const auto previous = q;
        law.step(q, targets);
        q = law.state.reference;
        for (int j : {1, 7})
            check(q[j] >= std::min(0., previous[j]) - 1e-9,
                  "negative J2 reference advanced outward");
    }
    check(q[1] > deg && q[7] > deg, "failed to return both J2 to normal range");
}
void check_field_hold() {
    constexpr double deg = std::numbers::pi / 180.;
    JointVector start;
    start << -.406, -.049, .857, -1.203, -.352, .566, -.286, -.022, 1.527, -.989, -.286, .483;
    start *= deg;
    ControlLaw law;
    std::array<ArmTarget, 2> targets{};
    for (int side = 0; side < 2; ++side)
        targets[side].joint = start.segment<6>(6 * side);
    auto hold = [&](const JointVector& measured, int tick) {
        const auto begin = HostClock::now();
        law.step(measured, targets);
        if (law.state.frozen || law.state.consecutive_failures) {
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::microseconds>(HostClock::now() - begin);
            std::cerr << "Field hold tick " << tick << ", step " << elapsed.count()
                      << " us, solver " << law.state.qp_time.count() / 1000. << " us\n";
            throw std::runtime_error("field hold froze");
        }
        check((law.state.reference - start).lpNorm<Eigen::Infinity>() < 1e-4,
              "hold reference drifted");
        check((law.state.reference - measured).lpNorm<Eigen::Infinity>() < .006, "excessive lead");
    };
    for (double bias : {.15, .30}) {
        JointVector measured = start;
        measured[3] -= bias * deg;
        law.reset(start);
        for (int k = 0; k < 5000; ++k)
            hold(measured, k);
    }
    // The initial sag in the recorded takeover settles within 100 ms.
    law.reset(start);
    for (int k = 0; k < 5000; ++k) {
        const double ramp = std::min(1., k / 50.);
        JointVector measured = start;
        measured[3] += (-.115 * ramp + (k % 2 ? .0055 : -.0055)) * deg;
        measured[7] += .110 * ramp * deg;
        measured[9] -= .055 * ramp * deg;
        hold(measured, k);
    }
    for (int side = 0; side < 2; ++side)
        for (double wrist : {-1., 1.}) {
            JointVector contact = start;
            contact.segment<6>(6 * side) = Six{0., -.5, 0., -2.5, wrist, 0.} * deg;
            for (bool takeover : {true, false}) {
                law.reset(start);
                try {
                    if (takeover)
                        law.reset(contact);
                    else
                        law.step(contact, targets);
                } catch (const std::runtime_error& e) {
                    if (std::string(e.what()) == "Measured collision hard threshold")
                        continue;
                    throw;
                }
                throw std::runtime_error("Intersecting feedback did not stop control");
            }
        }
}
void check_delayed_joint_feedback() {
    const ControlConfig cfg;
    ControlLaw law(cfg);
    JointVector start;
    start << 0, 1, 1.3, -.5, 0, 0, 0, 1, 1.3, -.5, 0, 0;
    law.reset(start);
    std::array<ArmTarget, 2> targets{};
    for (int side = 0; side < 2; ++side) {
        targets[side].joint = start.segment<6>(6 * side);
        targets[side].joint[3] += side ? -.4 : .4;
    }
    JointVector measured = start, previous = JointVector::Zero();
    double peak_tracking_speed = 0.;
    for (int k = 0; k < 750; ++k) {
        // Delay motion for 300 ms, with one encoder count of jitter on each wrist.
        if (k < 150) {
            measured[3] = start[3] + (k % 2 ? .0001 : -.0001);
            measured[9] = start[9] - (k % 2 ? .0001 : -.0001);
        } else {
            measured = law.state.reference;
        }
        law.step(measured, targets);
        check(!law.state.frozen, "delayed feedback froze joint motion");
        check(((law.state.reference - measured).array().abs() <= cfg.joints.max_lead.array() + 1e-7)
                  .all(),
              "joint reference exceeded lead window");
        check((law.state.velocity - previous).cwiseAbs().maxCoeff() <= 8. * control_dt + 1e-7,
              "joint acceleration exceeded 8 rad/s^2");
        check(law.state.velocity.cwiseAbs().maxCoeff() <= 1. + 1e-7,
              "joint speed exceeded 1 rad/s");
        if (k >= 150)
            peak_tracking_speed = std::max(peak_tracking_speed,
                                           std::min(law.state.velocity[3], -law.state.velocity[9]));
        previous = law.state.velocity;
    }
    check(peak_tracking_speed > .99, "joint speed did not reach the new cap with feedback");
    for (int side = 0; side < 2; ++side)
        check((law.state.reference.segment<6>(6 * side) - targets[side].joint).norm() < .001,
              "joint motion did not resume after delayed feedback");
}
int main() try {
    ControlConfig stage2;
    stage2.tracking_stop_rad = .10;
    ControlLaw guarded(stage2);
    JointVector q_start;
    q_start << 0, 1, 1.3, -.5, 0, 0, 0, 1, 1.3, -.5, 0, 0;
    guarded.reset(q_start);
    q_start[3] += .101;
    bool tracking_stop = false;
    try {
        guarded.step(q_start, {});
    } catch (const std::runtime_error& e) {
        tracking_stop = std::string(e.what()) == "Joint tracking error exceeds stop threshold";
    }
    check(tracking_stop, "Stage 2 tracking stop must be independent of the lead window");
    check_delayed_joint_feedback();
    check_field_hold();
    check_j2_takeover();
    ControlLaw c;
    std::vector<double> timings;
    JointVector q = JointVector::Zero();
    c.reset(q);
    std::array<ArmTarget, 2> targets{};
    for (auto& t : targets)
        t.joint[1] = t.joint[2] = .08726646;
    for (int k = 0; k < 1000; ++k) {
        c.step(q, targets);
        timings.push_back(c.state.qp_time.count() / 1000.);
        q = c.state.reference;
    }
    check((q.head<6>() - targets[0].joint).norm() < .001, "cannot lift from zero");
    Six ready;
    ready << 0, 1, 1.3, -.5, 0, 0;
    q << ready, ready;
    c.reset(q);
    CollisionState distance;
    // Drive the two palms through one another from opposite sides.
    for (int s = 0; s < 2; ++s) {
        targets[s].type = TargetType::pose;
        targets[s].pose = c.state.tcp[s];
        targets[s].pose.translation().y() += s ? .48 : -.48;
    }
    double minimum = 10;
    for (int k = 0; k < 2500; ++k) {
        c.step(q, targets);
        timings.push_back(c.state.qp_time.count() / 1000.);
        q = c.state.reference;
        collision_state({arm_geometry(0, q.head<6>()), arm_geometry(1, q.tail<6>())}, distance);
        minimum = std::min(minimum, distance.inter_arm);
    }
    std::cout << "Crossing clearance " << minimum << " m\n";
    check(minimum >= .02 - 1e-6 && minimum < .04, "opposing arms did not stop at clearance");
    // Random joint commands on both arms; verify all geometry at every emitted reference.
    std::mt19937 rng(324);
    std::uniform_real_distribution<double> offset(-.22, .22);
    q << ready, ready;
    c.reset(q);
    for (int k = 0; k < 3000; ++k) {
        if (k % 100 == 0)
            for (auto& t : targets) {
                t.type = TargetType::joint;
                for (int j = 0; j < 6; ++j)
                    t.joint[j] = ready[j] + offset(rng);
            }
        c.step(q, targets);
        timings.push_back(c.state.qp_time.count() / 1000.);
        q = c.state.reference;
        collision_state({arm_geometry(0, q.head<6>()), arm_geometry(1, q.tail<6>())}, distance);
        check(distance.hard_margin >= 0 && distance.closest > 0, "random command penetrated");
    }
    std::sort(timings.begin(), timings.end());
    std::cout << "Trajectory solver p99 " << timings[timings.size() * 99 / 100] << " us\n";
    // Construct a measured/reference lag which makes the lead and acceleration bounds incompatible.
    q << ready, ready;
    c.reset(q);
    q[0] += default_bounds().max_lead[0] + .02;
    for (int k = 0; k < 4; ++k) {
        c.step(q, targets);
        check(c.state.frozen && c.state.velocity.isZero(), "failed QP did not hold");
    }
    bool stopped = false;
    try {
        c.step(q, targets);
    } catch (const std::runtime_error&) {
        stopped = true;
    }
    check(stopped && c.state.consecutive_failures == 5, "five-failure stop missing");
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
