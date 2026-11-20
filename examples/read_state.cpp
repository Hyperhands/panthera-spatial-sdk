#include "panthera/telemetry.hpp"

#include <iostream>
#include <thread>

int main() try {
    using namespace std::chrono_literals;
    panthera::BoardTelemetry left(panthera::RealtimeAddress::left());
    panthera::BoardTelemetry right(panthera::RealtimeAddress::right());
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
