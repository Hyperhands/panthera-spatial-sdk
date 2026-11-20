#pragma once
#include "panthera/telemetry.hpp"
#include <functional>
#include <vector>

namespace panthera {
using Joints = std::array<float, 6>;
enum class Side { left, right };
enum class HoldStatus { holding, frozen };
struct BoardAddress {
    RealtimeAddress realtime;
    std::uint16_t management_local_port, management_board_port;
    static BoardAddress left() { return {RealtimeAddress::left(), 5006, 5006}; }
    static BoardAddress right() { return {RealtimeAddress::right(), 5016, 5016}; }
};
struct RpcTiming {
    std::uint64_t completed{}, retransmissions{};
    std::chrono::microseconds last{}, maximum{};
};

// Single-threaded WP-B transport. The caller supplies model gravity in Nm and drives
// hold() at a nominal 2 ms interval. enable() and stop() service both boards internally.
class DualSession {
  public:
    using Gravity = std::function<Joints(Side, const Joints&)>;
    DualSession(BoardAddress left, BoardAddress right, Gravity gravity);
    ~DualSession();
    void enable(std::chrono::milliseconds timeout);
    HoldStatus hold();
    // Returns false while either board has stale feedback.
    bool send_hand(Side side, const Joints& target_rad, std::uint16_t duration_ms);
    // Sends both stop requests before waiting for confirmation. Throws on unconfirmed stop.
    void stop();
    bool active() const;
    std::array<std::uint16_t, 2> local_ports(Side side) const;
    const BoardState& state(Side side) const;
    const RpcTiming& rpc_timing(Side side) const;
    std::vector<fault_event_value_t> take_fault_events(Side side);

  private:
    friend class DualArm;
    using JointPair = std::array<Joints, 2>;
    using Preflight = std::function<void(const JointPair&)>;
    void enable_checked(std::chrono::milliseconds timeout, const Preflight& preflight,
                        void (*check_interrupt)() = nullptr);
    using Control = std::function<void(const JointPair&, JointPair&, JointPair&)>;
    HoldStatus control(const Control& compute);
    void raw(Side side, const std::array<float, 30>& mit);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace panthera
