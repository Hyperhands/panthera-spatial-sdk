#pragma once
#include <chrono>

namespace panthera::detail {
#ifdef PANTHERA_TEST_SERVO_CLOCK
struct ServoClock {
    using time_point = std::chrono::steady_clock::time_point;
    static inline time_point current{};
    static inline std::chrono::steady_clock::duration advance_per_read{};
    static time_point now() {
        const auto sample = current;
        current += advance_per_read;
        return sample;
    }
};
#else
using ServoClock = std::chrono::steady_clock;
#endif
} // namespace panthera::detail
