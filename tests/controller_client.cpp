#include "panthera/controller.hpp"
#include <iostream>
#include <thread>
using namespace panthera;
using namespace std::chrono_literals;
namespace {
BoardAddress addr(char** argv, int n) {
    return {{"127.0.0.1", 0, "127.0.0.1", static_cast<std::uint16_t>(std::stoi(argv[0])),
             static_cast<uint16_t>(n)},
            0,
            static_cast<std::uint16_t>(std::stoi(argv[1]))};
}
ControlState check(DualArm& robot) {
    auto state = robot.state();
    if (!state.enabled || state.hard_margin < 0)
        throw std::runtime_error("Controller stopped: " + state.error);
    return state;
}
} // namespace
int main(int argc, char** argv) try {
    if (argc != 6)
        return 2;
    DualArm robot(addr(argv + 1, 11), addr(argv + 3, 12));
    for (auto s : {Side::left, Side::right})
        for (auto port : robot.local_ports(s))
            std::cout << port << " ";
    std::cout << std::endl;
    robot.enable(3s);
    auto initial = check(robot);
    std::string mode = argv[5];
    if (mode == "uplink" || mode == "reboot" || mode == "lag" || mode == "fault") {
        for (auto s : {Side::left, Side::right}) {
            Six goal = initial.measured.segment<6>(s == Side::left ? 0 : 6);
            goal[0] += .15;
            robot.setJointTarget(s, goal);
        }
        auto until = HostClock::now() + 2s;
        while (robot.state().enabled && HostClock::now() < until)
            std::this_thread::sleep_for(1ms);
        auto stopped = robot.state();
        const char* expected = mode == "uplink"   ? "feedback expired"
                               : mode == "reboot" ? "restarted"
                               : mode == "lag"    ? "Five consecutive"
                                                  : "Board fault";
        if (stopped.enabled || stopped.error.find(expected) == std::string::npos)
            throw std::runtime_error("Unexpected controller stop: " + stopped.error);
        if (mode == "lag" && stopped.consecutive_failures != 5)
            throw std::runtime_error("QP failure count");
        if (mode == "reboot") {
            robot.enable(3s);
            auto recovered = check(robot);
            std::this_thread::sleep_for(65ms);
            if ((check(robot).reference - recovered.measured).norm() > .002)
                throw std::runtime_error("Replayed pre-reboot target");
            robot.softStop();
        }
        return 0;
    }
    if (mode == "expert") {
        robot.enterExpertMode(ExpertAck::IUnderstandNoSafety);
        std::array<float, 30> mit{};
        for (int j = 0; j < 6; ++j)
            mit[5 * j] = static_cast<float>(j + .25);
        for (int k = 0; k < 5; ++k) {
            robot.sendMit(Side::left, mit);
            robot.sendMit(Side::right, mit);
            std::this_thread::sleep_for(2ms);
        }
        robot.softStop();
        return 0;
    }
    std::array<Eigen::Isometry3d, 2> end = initial.tcp;
    for (int s = 0; s < 2; ++s) {
        Six goal = initial.measured.segment<6>(6 * s);
        goal[0] += s ? .02 : -.02;
        goal[1] += .035;
        goal[2] += .025;
        goal[3] -= .01;
        end[s] = arm_geometry(s, goal).tcp();
    }
    if (mode == "line") {
        for (int k = 1; k <= 80; ++k) {
            const double t = k / 80.;
            // Quintic time scaling gives a straight Cartesian segment with smooth ends.
            const double f = t * t * t * (10 + t * (-15 + 6 * t));
            for (int s = 0; s < 2; ++s) {
                auto target = initial.tcp[s];
                target.translation() += f * (end[s].translation() - target.translation());
                target.linear() = Eigen::Quaterniond(initial.tcp[s].linear())
                                      .slerp(f, Eigen::Quaterniond(end[s].linear()))
                                      .toRotationMatrix();
                robot.setPoseTarget(s ? Side::right : Side::left, target);
            }
            check(robot);
            std::this_thread::sleep_for(5ms);
        }
    } else
        for (int s = 0; s < 2; ++s)
            robot.setPoseTarget(s ? Side::right : Side::left, end[s]);
    const auto finish = HostClock::now() + 2s;
    while (HostClock::now() < finish) {
        check(robot);
        std::this_thread::sleep_for(10ms);
    }
    auto state = check(robot);
    for (int s = 0; s < 2; ++s) {
        const double p = (state.tcp[s].translation() - end[s].translation()).norm();
        const double angle =
            Eigen::AngleAxisd(state.tcp[s].linear().transpose() * end[s].linear()).angle();
        std::cerr << mode << " arm " << s << " position " << p * 1000 << " mm angle "
                  << angle * 180 / std::numbers::pi << " deg\n";
        if (p >= .002 || angle >= std::numbers::pi / 180)
            throw std::runtime_error("TCP tracking tolerance");
    }
    robot.softStop();
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
}
