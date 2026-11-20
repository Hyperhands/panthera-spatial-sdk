#pragma once

#include "robot_messages_values.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace panthera {

struct RealtimeAddress {
    std::string local_ip;
    std::uint16_t local_port;
    std::string board_ip;
    std::uint16_t board_port;
    std::uint16_t board_node;
    std::uint16_t host_node = 10;

    static RealtimeAddress left();
    static RealtimeAddress right();
};

using HostClock = std::chrono::steady_clock;

template <typename T> struct TelemetrySample {
    T value;
    HostClock::time_point received_at;
    std::uint64_t missing = 0;
};

struct BoardState {
    std::uint64_t boot_id = 0;
    std::uint64_t restarts = 0;
    std::optional<TelemetrySample<arm_state_value_t>> arm;
    std::optional<TelemetrySample<hand_state_value_t>> hand;
    std::optional<TelemetrySample<fault_snapshot_value_t>> faults;
    std::optional<TelemetrySample<diagnostics_value_t>> diagnostics;

    bool fresh_arm_feedback(HostClock::time_point now) const;
};

// Single-owner, nonblocking receiver. poll() updates the snapshot in place.
// A new ArmState boot identity discards all samples from the previous boot.
class BoardTelemetry {
  public:
    explicit BoardTelemetry(const RealtimeAddress& address);
    ~BoardTelemetry();
    const BoardState& poll();
    std::uint16_t local_port() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace panthera
