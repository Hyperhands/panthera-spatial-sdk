#include "panthera/telemetry.hpp"

#include <iostream>
#include <string_view>
#include <thread>

int main(int argc, char** argv) try {
    using namespace std::chrono_literals;
    auto laddr = panthera::RealtimeAddress::left();
    auto raddr = panthera::RealtimeAddress::right();
    if (argc == 2 && std::string_view(argv[1]) == "--loopback") {
        laddr.local_ip = laddr.board_ip = raddr.local_ip = raddr.board_ip = "127.0.0.1";
        laddr.board_port = 15005;
        raddr.board_port = 15015;
    } else if (argc != 1) {
        std::cerr << "Usage: panthera-read-state [--loopback]\n";
        return 2;
    }
    panthera::BoardTelemetry left(laddr);
    panthera::BoardTelemetry right(raddr);
    const auto deadline = panthera::HostClock::now() + 8s;
    do {
        const auto& l = left.poll();
        const auto& r = right.poll();
        const auto now = panthera::HostClock::now();
        if (l.fresh_arm_feedback(now) && r.fresh_arm_feedback(now)) {
            for (const auto* state : {&l, &r}) {
                std::cout << "node " << state->arm->value.header.source_node << " boot "
                          << state->boot_id << " q_rad";
                for (const auto q : state->arm->value.q_rad)
                    std::cout << ' ' << q;
                std::cout << '\n';
            }
            return 0;
        }
        std::this_thread::sleep_for(1ms);
    } while (panthera::HostClock::now() < deadline);
    std::cerr << "Timed out waiting for fresh six-axis feedback from both boards\n";
    return 1;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
