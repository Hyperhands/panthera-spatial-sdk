#include "control_law.hpp"
#include <iostream>
#include <random>
using namespace panthera;
using panthera::detail::ControlLaw;
void check(bool value, const char* msg) {
    if (!value)
        throw std::runtime_error(msg);
}
int main() try {
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
    q[0] += .07;
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
