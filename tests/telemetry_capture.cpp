// Receiver for the independent Python board simulator. The parent process supplies
// board ports, reads our ephemeral host ports, then publishes exactly 500 cycles.
#include "panthera/telemetry.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) try {
    using namespace std::chrono_literals;
    if (argc != 3)
        return 2;
    panthera::BoardTelemetry left(
        {"127.0.0.1", 0, "127.0.0.1", static_cast<std::uint16_t>(std::stoi(argv[1])), 11});
    panthera::BoardTelemetry right(
        {"127.0.0.1", 0, "127.0.0.1", static_cast<std::uint16_t>(std::stoi(argv[2])), 12});
    std::cout << left.local_port() << ' ' << right.local_port() << std::endl;
    // Statistical capture test. The deterministic sequence tests live in telemetry_loopback.
    const auto complete = [](const panthera::BoardState& s) {
        return s.restarts == 0 && s.arm && s.arm->received >= 495 && s.hand &&
               s.hand->received >= 99 && s.faults && s.faults->received >= 19 && s.diagnostics &&
               s.diagnostics->received >= 9;
    };
    const auto deadline = panthera::HostClock::now() + 8s;
    std::uint64_t previous = 0;
    do {
        const auto& l = left.poll();
        const auto& r = right.poll();
        if (complete(l) && complete(r) &&
            panthera::HostClock::now() - std::max(l.arm->received_at, r.arm->received_at) > 30ms) {
            std::cerr << "Arm samples (left/right): " << l.arm->received << "/" << r.arm->received
                      << " of 500, internal sequence gaps: " << l.arm->missing << "/"
                      << r.arm->missing << '\n';
            return 0;
        }
        const auto count = (l.arm ? l.arm->received : 0) + (r.arm ? r.arm->received : 0);
        if (count == previous)
            std::this_thread::sleep_for(1ms);
        previous = count;
    } while (panthera::HostClock::now() < deadline);
    std::cerr << "Incomplete realtime streams after " << previous << " total arm samples\n";
    return 1;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
