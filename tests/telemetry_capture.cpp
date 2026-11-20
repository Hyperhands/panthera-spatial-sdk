// Receiver for the independent Python board simulator. The parent process supplies
// board ports, reads our ephemeral host ports, then publishes exactly 500 cycles.
#include "panthera/telemetry.hpp"

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
    const auto complete = [](const panthera::BoardState& s) {
        return s.restarts == 0 && s.arm && s.arm->received == 500 && s.arm->missing == 0 &&
               s.arm->value.header.sequence == 500 && s.hand && s.hand->received == 100 &&
               !s.hand->missing && s.faults && s.faults->received == 20 && !s.faults->missing &&
               s.diagnostics && s.diagnostics->received == 10 && !s.diagnostics->missing;
    };
    const auto deadline = panthera::HostClock::now() + 8s;
    std::uint64_t previous = 0;
    do {
        const auto& l = left.poll();
        const auto& r = right.poll();
        if (complete(l) && complete(r))
            return 0;
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
