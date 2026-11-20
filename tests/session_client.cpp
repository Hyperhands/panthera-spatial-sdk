#include "panthera/session.hpp"
#include <cmath>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using namespace panthera;
namespace {
void require(bool ok, const char* msg) {
    if (!ok)
        throw std::runtime_error(msg);
}
BoardAddress loopback(char** ports, std::uint16_t node) {
    return {{"127.0.0.1", 0, "127.0.0.1", static_cast<std::uint16_t>(std::stoi(ports[0])), node},
            0,
            static_cast<std::uint16_t>(std::stoi(ports[1]))};
}
} // namespace
int main(int argc, char** argv) try {
    if (argc != 6)
        return 2;
    std::string scenario = argv[5];
    DualSession session(loopback(argv + 1, 11), loopback(argv + 3, 12),
                        [](Side s, const Joints& model) {
                            require(std::abs(model[1] - 1.5f) < .001f, "Wrong J2 encoder branch");
                            Joints torque{};
                            torque[1] = s == Side::left ? 1.25f : 2.25f;
                            return torque;
                        });
    for (auto side : {Side::left, Side::right})
        for (auto port : session.local_ports(side))
            std::cout << port << ' ';
    std::cout << std::endl;
    if (scenario == "refused" || scenario == "timeout") {
        try {
            session.enable(2s);
        } catch (const std::runtime_error& e) {
            std::string why = e.what();
            require(why.find(scenario == "refused" ? "status -3" : "timed out") !=
                        std::string::npos,
                    "Unexpected takeover error");
            require(!session.active(), "Partial activation");
            return 0;
        }
        throw std::runtime_error("Takeover should have failed");
    }
    session.enable(3s);
    require(session.rpc_timing(Side::left).completed >= 6, "Missing RPC measurements");
    Joints fingers{.1f, .2f, .3f, .4f, .5f, .6f};
    session.send_hand(Side::left, fingers, 80);
    bool frozen = false, stopped = false, read_hand = false;
    auto deadline = HostClock::now() + 1s;
    while (HostClock::now() < deadline) {
        try {
            frozen |= session.hold() == HoldStatus::frozen;
        } catch (const std::runtime_error& e) {
            std::string why = e.what();
            std::string expected = scenario == "reboot"   ? "restarted"
                                   : scenario == "uplink" ? "feedback expired"
                                   : scenario == "jump"   ? "branch jump"
                                                          : "Board fault";
            if (scenario == "success" || why.find(expected) == std::string::npos)
                throw;
            stopped = true;
            break;
        }
        auto angles = session.state(Side::left).active_hand_angles(HostClock::now(), 30ms);
        read_hand |= angles && *angles == fingers;
        if (scenario == "success" && read_hand &&
            session.take_fault_events(Side::right).size() == 1)
            break;
        std::this_thread::sleep_for(1ms);
    }
    if (scenario == "success") {
        require(read_hand && HostClock::now() < deadline,
                "Hand echo or reliable fault event missing");
        session.stop();
    } else {
        require(stopped && !session.active(), "Session did not stop");
        if (scenario == "uplink")
            require(frozen, "Freeze missing before stop");
        try {
            session.hold();
            return 1;
        } catch (const std::logic_error&) {
        }
        if (scenario == "reboot") {
            session.enable(3s);
            session.hold();
            session.stop();
        }
    }
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
