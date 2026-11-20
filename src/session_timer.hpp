#pragma once
#include <chrono>
#include <thread>
#ifdef _WIN32
#include <system_error>
#include <windows.h>
#endif

namespace panthera::detail {
// MinGW sleep_for(1ms) can round up to a full Windows scheduler tick. A high
// resolution waitable timer keeps both sockets serviced inside the 20 ms prehold.
class SessionTimer {
    using Clock = std::chrono::steady_clock;
#ifdef _WIN32
    HANDLE timer;

  public:
    SessionTimer()
        : timer(CreateWaitableTimerExW(nullptr, nullptr, 0x2 /* HIGH_RESOLUTION */,
                                       TIMER_MODIFY_STATE | SYNCHRONIZE)) {
        if (!timer)
            throw std::system_error(GetLastError(), std::system_category(),
                                    "Creating session timer");
    }
    ~SessionTimer() { CloseHandle(timer); }
    SessionTimer(const SessionTimer&) = delete;
    SessionTimer& operator=(const SessionTimer&) = delete;
#endif
  public:
    void until(Clock::time_point deadline) {
#ifdef _WIN32
        using Tick = std::chrono::duration<long long, std::ratio<1, 10000000>>;
        const auto ticks = std::chrono::ceil<Tick>(deadline - Clock::now()).count();
        if (ticks <= 0)
            return;
        LARGE_INTEGER due;
        due.QuadPart = -ticks;
        if (!SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE) ||
            WaitForSingleObject(timer, INFINITE) != WAIT_OBJECT_0)
            throw std::system_error(GetLastError(), std::system_category(),
                                    "Waiting for session timer");
#else
        std::this_thread::sleep_until(deadline);
#endif
    }
};
} // namespace panthera::detail
