#include "panthera/telemetry.hpp"
#include "value_init.h"

#include <asio.hpp>
#include <wirelink/frame.h>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
using asio::ip::udp;
using namespace std::chrono_literals;
using panthera::HostClock;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

template <typename Test> void wait_for(panthera::BoardTelemetry& board, Test predicate) {
    const auto deadline = HostClock::now() + 1s;
    do {
        if (predicate(board.poll()))
            return;
        std::this_thread::sleep_for(1ms);
    } while (HostClock::now() < deadline);
    throw std::runtime_error("Timed out waiting for expected loopback packet");
}

stream_header_value_t header(std::uint16_t node, std::uint64_t boot, std::uint32_t seq) {
    stream_header_value_t h{};
    init_stream_header(&h);
    h.schema_revision = 7;
    h.source_node = node;
    h.target_node = 10;
    h.source_boot_id = boot;
    h.sequence = seq;
    return h;
}
arm_state_value_t arm(std::uint16_t node, std::uint64_t boot, std::uint32_t seq) {
    arm_state_value_t value{};
    init_arm_state(&value);
    value.header = header(node, boot, seq);
    value.valid_q_mask = value.valid_dq_mask = value.valid_tau_mask = value.online_mask =
        value.motor_fault_valid_mask = 63;
    value.feedback_age_us[3] = 2000;
    return value;
}

struct SimBoard {
    udp::socket socket;
    std::uint16_t host_port{};
    SimBoard(asio::io_context& io) : socket(io, {asio::ip::address_v4::loopback(), 0}) {}
    panthera::RealtimeAddress address(std::uint16_t node) const {
        return {"127.0.0.1", 0, "127.0.0.1", socket.local_endpoint().port(), node};
    }
    template <typename T, typename Encode>
    void send(std::uint16_t id, const T& value, Encode encode, bool bad_crc = false) {
        std::array<std::uint8_t, 512> payload{};
        std::size_t length{};
        require(encode(&value, payload.data(), payload.size(), &length) == 0, "encode payload");
        wl_wire_packet_t p{};
        p.type = WL_PACKET_DATA;
        p.integrity = WL_INTEGRITY_CRC32C;
        p.message_id = id;
        p.payload = payload.data();
        p.payload_len = length;
        std::array<std::uint8_t, 544> bytes{};
        require(wl_frame_encode(&p, WL_ENVELOPE_NATIVE_PACKET, bytes.data(), bytes.size(),
                                &length) == 0,
                "encode frame");
        if (bad_crc)
            bytes[length - 1] ^= 1;
        socket.send_to(asio::buffer(bytes.data(), length),
                       {asio::ip::address_v4::loopback(), host_port});
    }
    void send_arm(const arm_state_value_t& value, bool bad_crc = false) {
        send(ARM_STATE_MESSAGE_ID, value, arm_state_value_encode, bad_crc);
    }
};
} // namespace

int main() try {
    asio::io_context io;
    SimBoard left(io), right(io), stranger(io);
    panthera::BoardTelemetry l(left.address(11)), r(right.address(12));
    left.host_port = stranger.host_port = l.local_port();
    right.host_port = r.local_port();
    require(!l.poll().arm && !r.poll().arm, "empty discovery");
    left.send_arm(arm(11, 123, UINT32_MAX));
    right.send_arm(arm(12, 456, 40));
    wait_for(l, [](const auto& s) { return s.boot_id == 123; });
    wait_for(r, [](const auto& s) { return s.boot_id == 456; });

    hand_state_value_t hand{};
    init_hand_state(&hand);
    hand.header = header(11, 123, 9);
    hand.online = true;
    hand.valid_fields = 2;
    hand.field_age_us[1] = 4000;
    for (int i = 0; i < 11; ++i)
        hand.joint_q_rad[i] = static_cast<float>(i);
    left.send(HAND_STATE_MESSAGE_ID, hand, hand_state_value_encode);
    wait_for(l, [](const auto& s) { return s.hand.has_value(); });
    auto hand_state = l.poll();
    const auto at = hand_state.hand->received_at;
    const auto active = hand_state.active_hand_angles(at + 5ms, 10ms);
    require(active && *active == std::array<float, 6>{0, 1, 3, 5, 7, 9}, "active finger mapping");
    require(!hand_state.active_hand_angles(at + 6ms, 10ms), "hand feedback age");
    hand_state.hand->value.valid_fields = 1;
    require(!hand_state.active_hand_angles(at, 10ms), "missing angle field");
    left.send_arm(arm(11, 123, 1)); // Sequence zero was missed across wrap.
    wait_for(l, [](const auto& s) { return s.arm->value.header.sequence == 1; });
    require(l.poll().arm->missing == 1, "wraparound gap count");
    left.send_arm(arm(11, 123, 0)); // old
    left.send_arm(arm(11, 123, 1)); // duplicate
    stranger.send_arm(arm(11, 999, 5));
    left.send_arm(arm(12, 999, 7)); // wrong node on the expected socket
    left.send_arm(arm(11, 999, 8), true);
    left.send_arm(arm(11, 123, 2));
    wait_for(l, [](const auto& s) { return s.arm->value.header.sequence == 2; });
    auto state = l.poll();
    require(state.boot_id == 123 && state.restarts == 0 && state.arm->missing == 1,
            "ignore rejected packets");
    const auto received = state.arm->received_at;
    require(state.fresh_arm_feedback(received + 17ms) && !state.fresh_arm_feedback(received + 18ms),
            "combined age threshold");
    state.arm->value.q_rad[0] = std::numeric_limits<float>::quiet_NaN();
    require(!state.fresh_arm_feedback(received), "non-finite feedback");
    left.send_arm(arm(11, 789, 1));
    wait_for(l, [](const auto& s) { return s.boot_id == 789; });
    left.send_arm(arm(11, 123, 3)); // delayed packet from the retired boot
    left.send(HAND_STATE_MESSAGE_ID, hand, hand_state_value_encode);
    left.send_arm(arm(11, 789, 2));
    wait_for(l,
             [](const auto& s) { return s.boot_id == 789 && s.arm->value.header.sequence == 2; });
    require(l.poll().restarts == 1 && !l.poll().hand && !l.poll().arm->missing,
            "discard pre-reboot samples");
    require(r.poll().boot_id == 456 && r.poll().restarts == 0, "independent board identity");
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
