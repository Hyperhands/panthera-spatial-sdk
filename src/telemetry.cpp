#include "panthera/telemetry.hpp"

#include <wirelink/asio/udp_adapter.hpp>
#include <wirelink/link.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace panthera {
namespace {
void check_link(int result) {
    if (result != WL_OK) {
        throw std::runtime_error(wl_err_str(result));
    }
}

template <typename T>
void publish(std::optional<TelemetrySample<T>>& sample, const T& message,
             HostClock::time_point now) {
    std::uint64_t missing = 0;
    if (sample) {
        const std::uint32_t delta = message.header.sequence - sample->value.header.sequence;
        if (delta == 0 || delta >= 0x80000000U) {
            return;
        }
        missing = sample->missing + delta - 1;
    }
    const auto received = sample ? sample->received + 1 : 1;
    sample.emplace(TelemetrySample<T>{message, now, missing, received});
}
} // namespace

RealtimeAddress RealtimeAddress::left() {
    return {"192.168.10.1", 5005, "192.168.10.11", 5005, 11};
}
RealtimeAddress RealtimeAddress::right() {
    return {"192.168.10.1", 5015, "192.168.10.12", 5015, 12};
}

bool BoardState::fresh_arm_feedback(HostClock::time_point now) const {
    if (!arm || now < arm->received_at) {
        return false;
    }
    const auto& a = arm->value;
    if (a.online_mask != 63 || a.valid_q_mask != 63 || a.valid_dq_mask != 63 ||
        a.valid_tau_mask != 63 || a.motor_fault_valid_mask != 63) {
        return false;
    }
    for (int joint = 0; joint < 6; ++joint) {
        if (a.fault_code_raw[joint] || !std::isfinite(a.q_rad[joint]) ||
            !std::isfinite(a.dq_rad_s[joint]) || !std::isfinite(a.tau_measured_nm[joint])) {
            return false;
        }
    }
    const auto board_age = std::chrono::microseconds(
        *std::max_element(std::begin(a.feedback_age_us), std::end(a.feedback_age_us)));
    return now - arm->received_at + board_age < std::chrono::milliseconds(20);
}

struct BoardTelemetry::Impl {
    RealtimeAddress address;
    wl_ctx_t link{};
    std::array<std::uint8_t, 512> payload{};
    std::array<std::uint8_t, 544> tx{}, control{}, rx{};
    BoardState state;
    std::vector<std::uint64_t> retired_boots;
    std::unique_ptr<wirelink::asio::UdpAdapter> socket;

    explicit Impl(const RealtimeAddress& config) : address(config) {
        if (!config.board_node || !config.host_node || config.board_node == config.host_node ||
            !config.board_port) {
            throw std::invalid_argument("Invalid host/board identity or board port");
        }
        wl_config_t wire{};
        wire.max_payload_len = payload.size();
        wire.envelope = WL_ENVELOPE_NATIVE_PACKET;
        wire.integrity = WL_INTEGRITY_CRC32C;
        // The observer sends no application requests. A nonzero wire session is required by
        // wl_init.
        wire.session_id = 1;
        wire.ack_timeout_ms = 20;
        wire.max_transmission_unit = tx.size();
        wl_storage_t storage{};
        storage.tx_payload = payload.data();
        storage.tx_payload_size = payload.size();
        storage.tx_unit = tx.data();
        storage.tx_unit_size = tx.size();
        storage.control_unit = control.data();
        storage.control_unit_size = control.size();
        storage.rx_fallback = rx.data();
        storage.rx_fallback_size = rx.size();
        check_link(wl_init(&link, &wire, &storage));
        wirelink::asio::UdpAdapterConfig udp;
        udp.bind_address = config.local_ip;
        udp.bind_port = config.local_port;
        udp.receive_slots = 8;
        std::error_code error;
        socket = wirelink::asio::UdpAdapter::open(link, udp, error);
        if (!socket) {
            throw std::system_error(error, "Opening realtime UDP socket");
        }
        check_link(socket->set_peer(config.board_ip, config.board_port));
    }

    template <typename T, typename Decode>
    void receive(const wl_event_t& event, Decode decode,
                 std::optional<TelemetrySample<T>> BoardState::* field) {
        T value{};
        if (decode(event.payload, event.payload_len, &value) != 0) {
            return;
        }
        const auto& h = value.header;
        if (h.schema_revision != 7 || h.source_node != address.board_node ||
            h.target_node != address.host_node || !h.source_boot_id) {
            return;
        }
        if (state.boot_id != h.source_boot_id) {
            if (event.message_id != ARM_STATE_MESSAGE_ID ||
                std::find(retired_boots.begin(), retired_boots.end(), h.source_boot_id) !=
                    retired_boots.end()) {
                return;
            }
            if (state.boot_id) {
                retired_boots.push_back(state.boot_id);
            }
            state = {};
            state.restarts = retired_boots.size();
            state.boot_id = h.source_boot_id;
        }
        publish(state.*field, value, HostClock::now());
    }

    void dispatch(const wl_event_t& event) {
        if (event.type != WL_EVT_UNRELIABLE_RX) {
            return;
        }
        switch (event.message_id) {
        case ARM_STATE_MESSAGE_ID:
            receive(event, arm_state_value_decode, &BoardState::arm);
            break;
        case HAND_STATE_MESSAGE_ID:
            receive(event, hand_state_value_decode, &BoardState::hand);
            break;
        case FAULT_SNAPSHOT_MESSAGE_ID:
            receive(event, fault_snapshot_value_decode, &BoardState::faults);
            break;
        case DIAGNOSTICS_MESSAGE_ID:
            receive(event, diagnostics_value_decode, &BoardState::diagnostics);
            break;
        }
    }
};

BoardTelemetry::BoardTelemetry(const RealtimeAddress& address)
    : impl_(std::make_unique<Impl>(address)) {}
BoardTelemetry::~BoardTelemetry() = default;
std::uint16_t BoardTelemetry::local_port() const { return impl_->socket->local_port(); }

const BoardState& BoardTelemetry::poll() {
    auto& i = *impl_;
    const auto rc = i.socket->service();
    if (rc != WL_ERR_NO_DATA && rc != WL_ERR_WOULD_BLOCK) {
        check_link(rc);
    }
    const auto now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(HostClock::now().time_since_epoch())
            .count();
    for (unsigned slot = 0; slot < 8; ++slot) {
        wl_event_t event{};
        const auto result = wl_poll(&i.link, static_cast<wl_time_ms_t>(now_ms), &event);
        if (result == WL_ERR_NO_DATA) {
            break;
        }
        check_link(result);
        i.dispatch(event);
        wl_event_release(&i.link, &event);
    }
    return i.state;
}
} // namespace panthera
