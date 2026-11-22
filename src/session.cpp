#include "panthera/session.hpp"
#include "session_timer.hpp"
#include "value_init.h"
#include <algorithm>
#include <asio/ip/udp.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <thread>
#include <wirelink/frame.h>

namespace panthera {
using namespace std::chrono_literals;
namespace {
using udp = asio::ip::udp;
using Time = HostClock::time_point;
using Bytes = std::array<std::uint8_t, 512>;
std::uint64_t boot_nonce() {
    std::random_device r;
    const auto n = (std::uint64_t(r()) << 32) | r();
    return n ? n : 1;
}
std::uint32_t next(std::uint32_t& n) {
    if (!++n)
        ++n;
    return n;
}
template <class T, class Encoder> size_t encode(const T& value, Encoder encoder, Bytes& bytes) {
    size_t size{};
    if (encoder(&value, bytes.data(), bytes.size(), &size))
        throw std::runtime_error("WLC encoding failed");
    return size;
}
struct Result {
    int status;
    std::uint32_t revision;
    std::uint64_t lease;
};
// Tuned MIT gains (J2/J3 stiffened with higher damping; J4 matches firmware homing).
constexpr std::array<float, 6> kp{24.48f, 90.f, 90.f, 36.72f, 12.24f, 12.24f},
    kd{1.2f, 2.5f, 2.5f, .8313843876f, .48f, .48f};
constexpr Joints branch{0, std::numbers::pi_v<float> / 2, 2.886410263f, .255182391f, 0, 0};
constexpr float turn = 2 * std::numbers::pi_v<float>;
} // namespace
class SessionPeer {
  public:
    BoardAddress address;
    BoardTelemetry rt;
    const BoardState* state;
    const std::uint64_t host;
    std::uint64_t boot{}, lease{};
    std::uint32_t revision{};
    bool activation_sent{};
    Joints zero{}, reference{}, previous_raw{}, measured{};
    RpcTiming timing;
    std::vector<fault_event_value_t> events;

    SessionPeer(BoardAddress a, std::uint64_t nonce)
        : address(a), rt(a.realtime), state(&rt.poll()), host(nonce),
          socket(io, udp::endpoint(asio::ip::make_address(a.realtime.local_ip),
                                   a.management_local_port)) {
        socket.connect(
            udp::endpoint(asio::ip::make_address(a.realtime.board_ip), a.management_board_port));
        socket.non_blocking(true);
    }
    std::uint16_t management_port() const { return socket.local_endpoint().port(); }
    void discover() {
        if (boot != state->boot_id)
            last_event = 0;
        boot = state->boot_id;
        revision = state->arm->value.policy_revision;
    }
    stream_header_value_t header() {
        stream_header_value_t h;
        init_stream_header(&h);
        h.schema_revision = 7;
        h.source_node = address.realtime.host_node;
        h.target_node = address.realtime.board_node;
        h.source_boot_id = host;
        h.target_boot_id = boot;
        h.sequence = next(sequence);
        h.tx_monotonic_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                HostClock::now().time_since_epoch())
                                .count();
        return h;
    }
    void start(unsigned op) {
        result.reset();
        operation = op;
        if (op) {
            operation_request_value_t q;
            init_operation_request(&q);
            q.header = header();
            q.operation_id = q.header.sequence;
            q.operation = op;
            q.control_lease = lease;
            q.expected_policy_revision = revision;
            q.target_mask = op == 10 ? 500u << 16 : 0;
            request_size = encode(q, operation_request_value_encode, request);
            request_id = q.operation_id;
            message = OPERATION_REQUEST_MESSAGE_ID;
        } else {
            configure_request_value_t q;
            init_configure_request(&q);
            q.header = header();
            q.operation_id = q.header.sequence;
            q.expected_policy_revision = revision;
            q.new_policy_revision = revision + 1;
            if (!q.new_policy_revision)
                throw std::runtime_error("Policy revision exhausted");
            q.hand_mode = 2;
            q.arm_state_period_us = 2000;
            q.hand_state_period_us = 10000;
            std::fill_n(q.force_feedback_gain, 6, 1.f);
            std::fill_n(q.hand_position_current_limit_ma, 6, 1000u);
            request_size = encode(q, configure_request_value_encode, request);
            request_id = q.operation_id;
            message = CONFIGURE_REQUEST_MESSAGE_ID;
        }
        started = HostClock::now();
        transmit();
    }
    void cancel() {
        request_id = 0;
        result.reset();
    }
    std::optional<Result> collect() {
        service();
        if (result) {
            auto out = result;
            cancel();
            return out;
        }
        if (request_id && HostClock::now() - sent >= 20ms) {
            ++timing.retransmissions;
            transmit();
        }
        return {};
    }
    void service() {
        std::array<std::uint8_t, 2048> bytes{};
        for (int i = 0; i < 16; ++i) {
            std::error_code ec;
            auto size = socket.receive(asio::buffer(bytes), 0, ec);
            if (ec == asio::error::would_block || ec == asio::error::try_again)
                break;
            if (ec)
                throw std::system_error(ec, "management receive");
            wl_frame_view_t f{};
            if (wl_frame_decode(bytes.data(), size, WL_INTEGRITY_CRC32C, &f) != 0 ||
                f.type != WL_PACKET_DATA || f.flags != WL_PACKET_FLAG_RELIABLE)
                continue;
            if (f.message_id == FAULT_EVENT_MESSAGE_ID) {
                fault_event_value_t e{};
                if (!fault_event_value_decode(f.payload.data, f.payload.length, &e) &&
                    identity(e.header) && e.event_id > last_event) {
                    events.push_back(e);
                    last_event = e.event_id;
                }
            } else if (request_id && !result && f.message_id == message + 1) {
                if (operation) {
                    operation_result_value_t r{};
                    if (!operation_result_value_decode(f.payload.data, f.payload.length, &r) &&
                        matches(r))
                        result = Result{r.status, r.applied_policy_revision, r.control_lease};
                } else {
                    configure_result_value_t r{};
                    if (!configure_result_value_decode(f.payload.data, f.payload.length, &r) &&
                        matches(r))
                        result = Result{r.status, r.applied_policy_revision, lease};
                }
                if (result) {
                    ++timing.completed;
                    timing.last = std::chrono::duration_cast<std::chrono::microseconds>(
                        HostClock::now() - started);
                    timing.maximum = std::max(timing.maximum, timing.last);
                }
            }
            send_frame(
                {WL_PACKET_ACK, WL_INTEGRITY_CRC32C, 0, 0, f.session_id, f.sequence, nullptr, 0});
        }
    }
    void check_faults() const {
        const auto has_fault = [](const auto& codes, std::uint32_t mask) {
            for (unsigned j = 0; j < 6; ++j)
                if ((mask & (1u << j)) && codes[j])
                    return true;
            return false;
        };
        const auto& a = state->arm->value;
        if (has_fault(a.fault_code_raw, a.motor_fault_valid_mask))
            throw std::runtime_error("Board fault");
        if (state->faults) {
            const auto& f = state->faults->value;
            if ((f.mcu_fault_active & f.mcu_fault_valid_mask) ||
                has_fault(f.arm_fault_raw, f.arm_fault_valid_mask))
                throw std::runtime_error("Board fault");
        }
    }
    void calibrate() {
        initial_jumps = state->encoder_jumps;
        for (size_t j = 0; j < 6; ++j) {
            auto raw = state->arm->value.q_rad[j];
            zero[j] = turn * std::floor((raw - branch[j]) / turn + .5f);
            previous_raw[j] = raw;
            reference[j] = measured[j] = raw - zero[j];
        }
    }
    void update_angles() {
        if (state->encoder_jumps != initial_jumps)
            throw std::runtime_error("Encoder branch jump");
        for (size_t j = 0; j < 6; ++j) {
            auto raw = state->arm->value.q_rad[j];
            if (!std::isfinite(raw) || std::abs(raw - previous_raw[j]) > turn / 2)
                throw std::runtime_error("Encoder branch jump or invalid angle");
            measured[j] = raw - zero[j];
            previous_raw[j] = raw;
        }
    }
    void raw(const std::array<float, 30>& mit) {
        arm_command_value_t q;
        init_arm_command(&q);
        q.header = header();
        q.control_lease = lease;
        q.policy_revision = revision;
        q.joint_mask = 63;
        std::copy(mit.begin(), mit.end(), q.mit);
        Bytes b;
        auto n = encode(q, arm_command_value_encode, b);
        rt.send(ARM_COMMAND_MESSAGE_ID, b.data(), n);
    }
    void hold(const Joints& gravity, const Joints& velocity = {}) {
        std::array<float, 30> mit;
        for (size_t j = 0; j < 6; ++j) {
            if (!std::isfinite(gravity[j]))
                throw std::runtime_error("Nonfinite gravity torque");
            mit[5 * j] = reference[j] + zero[j];
            mit[5 * j + 1] = velocity[j];
            mit[5 * j + 2] = kp[j];
            mit[5 * j + 3] = kd[j];
            mit[5 * j + 4] = gravity[j];
        }
        raw(mit);
    }
    void hand(const Joints& target, std::uint16_t duration) {
        hand_command_value_t q;
        init_hand_command(&q);
        q.header = header();
        q.control_lease = lease;
        q.policy_revision = revision;
        q.duration_ms = duration;
        std::copy(target.begin(), target.end(), q.target_q_rad);
        Bytes b;
        auto n = encode(q, hand_command_value_encode, b);
        rt.send(HAND_COMMAND_MESSAGE_ID, b.data(), n);
    }

  private:
    asio::io_context io;
    udp::socket socket;
    Bytes request;
    size_t request_size{};
    std::uint32_t sequence{}, wire_sequence{}, request_id{};
    std::uint16_t message{}, operation{};
    std::uint64_t last_event{}, initial_jumps{};
    Time started{}, sent{};
    std::optional<Result> result;
    bool identity(const stream_header_value_t& h) const {
        return h.schema_revision == 7 && h.source_node == address.realtime.board_node &&
               h.target_node == address.realtime.host_node && h.source_boot_id == boot;
    }
    template <class R> bool matches(const R& r) const {
        return identity(r.header) && request_id == r.operation_id &&
               r.header.sequence == request_id &&
               (r.header.target_boot_id == host || operation == 19 || operation == 10 ||
                (operation == 1 && r.status != 0));
    }
    void transmit() {
        send_frame({WL_PACKET_DATA, WL_INTEGRITY_CRC32C, WL_PACKET_FLAG_RELIABLE, message, host,
                    next(wire_sequence), request.data(), request_size});
        sent = HostClock::now();
    }
    void send_frame(const wl_wire_packet_t& f) {
        std::array<std::uint8_t, 544> b;
        size_t n{};
        if (wl_frame_encode(&f, WL_ENVELOPE_NATIVE_PACKET, b.data(), b.size(), &n))
            throw std::runtime_error("Wirelink frame encoding failed");
        socket.send(asio::buffer(b.data(), n));
    }
};

struct DualSession::Impl {
    SessionPeer left, right;
    std::array<SessionPeer*, 2> peers{&left, &right};
    Gravity gravity;
    detail::SessionTimer timer;
    bool active{};
    Time last_hold{};
    std::array<bool, 2> claiming{}, releasing{};

    Impl(BoardAddress l, BoardAddress r, Gravity g)
        : left(l, boot_nonce()), right(r, left.host), gravity(std::move(g)) {
        if (!gravity)
            throw std::invalid_argument("Model gravity callback required");
        if (l.realtime.board_node == r.realtime.board_node)
            throw std::invalid_argument("Distinct board nodes required");
    }
    SessionPeer& peer(Side s) { return s == Side::left ? left : right; }
    void refresh(bool check_boot = true) {
        for (auto p : peers)
            p->rt.poll();
        if (check_boot)
            for (auto p : peers)
                if (p->boot != p->state->boot_id)
                    throw std::runtime_error("Board restarted: node " +
                                             std::to_string(p->address.realtime.board_node));
    }
    void wait(Time deadline, void (*check_interrupt)()) {
        if (check_interrupt)
            check_interrupt();
        if (HostClock::now() >= deadline)
            throw std::runtime_error("Dual-board takeover timed out");
        timer.until(std::min(deadline, HostClock::now() + 1ms));
    }
    void feedback(Time deadline, void (*check_interrupt)(), bool after_config = false) {
        const auto after = HostClock::now();
        while (true) {
            refresh();
            bool ready = true;
            for (auto p : peers) {
                p->check_faults();
                auto& s = *p->state;
                ready &= s.fresh_arm_feedback(HostClock::now()) && s.arm->received_at >= after &&
                         (!after_config || (s.arm->value.policy_revision == p->revision &&
                                            s.arm->value.header.target_boot_id == p->host));
            }
            if (ready)
                return;
            wait(deadline, check_interrupt);
        }
    }
    void holds(const JointPair& velocity = {}) {
        auto l = gravity(Side::left, left.measured), r = gravity(Side::right, right.measured);
        left.hold(l, velocity[0]);
        right.hold(r, velocity[1]);
        last_hold = HostClock::now();
    }
    void stage(unsigned op, Time deadline, void (*check_interrupt)()) {
        std::array<bool, 2> done{};
        auto start = [&](size_t i) {
            if (op == 1)
                claiming[i] = true;
            if (op == 3)
                peers[i]->activation_sent = true;
            peers[i]->start(op);
        };
        for (size_t i = 0; i < 2; ++i)
            start(i);
        Time next_hold = HostClock::now() + 2ms;
        while (true) {
            refresh();
            if (op == 3 && HostClock::now() >= next_hold) {
                for (auto p : peers) {
                    p->check_faults();
                    if (!p->state->fresh_arm_feedback(HostClock::now()))
                        throw std::runtime_error("Stale activation feedback");
                    p->update_angles();
                }
                holds();
                next_hold = HostClock::now() + 2ms;
            }
            for (size_t i = 0; i < 2; ++i) {
                auto& p = *peers[i];
                if (done[i]) {
                    p.service();
                    continue;
                }
                auto r = p.collect();
                if (!r)
                    continue;
                if (op == 1)
                    claiming[i] = false;
                if (op == 3 && r->status != 0)
                    p.activation_sent = false;
                if (r->status == -16 &&
                    (op == 1 || (op == 3 && p.state->arm->value.control_owner == 2))) {
                    if (HostClock::now() >= deadline)
                        throw std::runtime_error("Board remained busy during takeover");
                    start(i);
                    continue;
                }
                if (r->status)
                    throw std::runtime_error(
                        "Node " + std::to_string(p.address.realtime.board_node) + " operation " +
                        std::to_string(op) + " status " + std::to_string(r->status));
                if (op == 1) {
                    if (!r->lease)
                        throw std::runtime_error("Board returned an empty lease");
                    p.lease = r->lease;
                }
                if (!op && r->revision != p.revision + 1)
                    throw std::runtime_error("Configure revision mismatch");
                p.revision = r->revision;
                done[i] = true;
            }
            if (done[0] && done[1])
                return;
            wait(deadline, check_interrupt);
        }
    }
    // A pending claim may already have been applied. Recover its cached result
    // with the original operation ID, then release the returned lease.
    bool cleanup() {
        active = false;
        std::array<bool, 2> pending{};
        bool confirmed = true;
        auto release = [&](size_t i) {
            auto& p = *peers[i];
            p.start(p.activation_sent ? 18 : 2);
            releasing[i] = true;
        };
        for (size_t i = 0; i < 2; ++i) {
            auto& p = *peers[i];
            if (p.boot != p.state->boot_id) {
                p.cancel();
                p.lease = 0;
                claiming[i] = false;
            }
            pending[i] = claiming[i] || p.lease;
            if (!pending[i]) {
                p.activation_sent = false;
                releasing[i] = false;
                p.cancel();
                continue;
            }
            if (p.lease && !releasing[i])
                try {
                    release(i);
                } catch (...) {
                    confirmed = false;
                    pending[i] = false;
                }
        }
        const auto deadline = HostClock::now() + 200ms;
        while ((pending[0] || pending[1]) && HostClock::now() < deadline) {
            for (size_t i = 0; i < 2; ++i)
                if (pending[i]) {
                    auto& p = *peers[i];
                    try {
                        auto r = p.collect();
                        if (!r)
                            continue;
                        if (claiming[i]) {
                            claiming[i] = false;
                            if (!r->status && r->lease) {
                                p.lease = r->lease;
                                release(i);
                                continue;
                            }
                        } else {
                            releasing[i] = false;
                            if (r->status)
                                confirmed = false;
                            // Firmware reports its remaining lease even on a failed soft hold.
                            if (r->lease != p.lease || !r->status)
                                p.lease = 0;
                        }
                        p.activation_sent = p.lease && p.activation_sent;
                        pending[i] = false;
                    } catch (...) {
                        confirmed = false;
                        pending[i] = false;
                    }
                }
            if (pending[0] || pending[1])
                timer.until(std::min(deadline, HostClock::now() + 1ms));
        }
        // Keep unresolved RPCs and their operation IDs for an explicit stop retry.
        return confirmed && !pending[0] && !pending[1];
    }
    void enable(Time deadline, const Preflight& preflight, void (*check_interrupt)()) {
        if (left.lease || right.lease || claiming[0] || claiming[1]) {
            refresh(false);
            if (!cleanup())
                throw std::runtime_error("Previous board release unconfirmed");
        }
        do {
            refresh(false);
            if (left.state->arm && right.state->arm)
                break;
            wait(deadline, check_interrupt);
        } while (true);
        for (auto p : peers)
            p->discover();
        stage(19, deadline, check_interrupt);
        stage(10, deadline, check_interrupt);
        feedback(deadline, check_interrupt);
        stage(1, deadline, check_interrupt);
        stage(13, deadline, check_interrupt);
        stage(0, deadline, check_interrupt);
        feedback(deadline, check_interrupt, true);
        for (auto p : peers)
            p->calibrate();
        if (preflight)
            preflight({left.measured, right.measured});
        holds();
        auto activation_deadline = std::min(deadline, HostClock::now() + 100ms);
        stage(3, activation_deadline, check_interrupt);
        // A reliable result may precede the first active realtime sample.
        auto next_hold = HostClock::now() + 2ms;
        while (true) {
            refresh();
            bool ready = true;
            for (auto p : peers) {
                p->check_faults();
                if (!p->state->fresh_arm_feedback(HostClock::now()))
                    throw std::runtime_error("Invalid active feedback");
                p->update_angles();
                ready &=
                    p->state->arm->value.control_owner == 1 && p->state->arm->value.run_state == 1;
            }
            if (ready)
                break;
            if (HostClock::now() >= next_hold) {
                holds();
                next_hold = HostClock::now() + 2ms;
            }
            wait(activation_deadline, check_interrupt);
        }
        active = true;
    }
    HoldStatus status() {
        refresh();
        if (HostClock::now() - last_hold >= 50ms)
            throw std::runtime_error("Host hold deadline expired");
        bool frozen = false;
        for (auto p : peers) {
            p->service();
            auto& s = *p->state;
            auto& a = s.arm->value;
            auto now = HostClock::now();
            auto age = now - s.arm->received_at +
                       std::chrono::microseconds(*std::max_element(std::begin(a.feedback_age_us),
                                                                   std::end(a.feedback_age_us)));
            if (age >= 50ms || a.control_owner != 1 || !a.run_state ||
                a.header.target_boot_id != p->host || a.policy_revision != p->revision)
                throw std::runtime_error("Board feedback expired or host control lost");
            p->check_faults();
            bool valid = s.fresh_arm_feedback(now);
            if (valid)
                p->update_angles();
            frozen |= !valid || age >= 10ms;
        }
        return frozen ? HoldStatus::frozen : HoldStatus::holding;
    }
    [[noreturn]] void rollback() {
        if (!cleanup())
            std::throw_with_nested(std::runtime_error("Board stop or lease release unconfirmed"));
        throw;
    }
};

DualSession::DualSession(BoardAddress l, BoardAddress r, Gravity g)
    : impl_(std::make_unique<Impl>(l, r, std::move(g))) {}
DualSession::~DualSession() {
    if (impl_->active)
        try {
            impl_->cleanup();
        } catch (...) {
        }
}
bool DualSession::active() const { return impl_->active; }
void DualSession::enable(std::chrono::milliseconds timeout) { enable_checked(timeout, {}); }
void DualSession::enable_checked(std::chrono::milliseconds timeout, const Preflight& preflight,
                                 void (*check_interrupt)()) {
    if (active())
        throw std::logic_error("Session already active");
    if (timeout <= 0ms)
        throw std::invalid_argument("Positive takeover timeout required");
    try {
        impl_->enable(HostClock::now() + timeout, preflight, check_interrupt);
    } catch (...) {
        impl_->rollback();
    }
}
HoldStatus DualSession::hold() {
    if (!active())
        throw std::logic_error("Enable both boards first");
    try {
        auto status = impl_->status();
        impl_->holds();
        return status;
    } catch (...) {
        impl_->rollback();
    }
}
HoldStatus DualSession::control(const Control& compute) {
    if (!active())
        throw std::logic_error("Inactive session");
    try {
        auto status = impl_->status();
        JointPair reference{impl_->left.reference, impl_->right.reference}, velocity{};
        if (status == HoldStatus::holding) {
            compute({impl_->left.measured, impl_->right.measured}, reference, velocity);
            status = impl_->status();
            if (status == HoldStatus::holding) {
                impl_->left.reference = reference[0];
                impl_->right.reference = reference[1];
            } else
                velocity = {};
        }
        impl_->holds(velocity);
        return status;
    } catch (...) {
        impl_->rollback();
    }
}
void DualSession::raw(Side side, const std::array<float, 30>& mit) {
    if (!active())
        throw std::logic_error("Inactive session");
    impl_->peer(side).raw(mit);
}
bool DualSession::send_hand(Side side, const Joints& target, std::uint16_t duration) {
    if (!active())
        throw std::logic_error("Enable both boards first");
    if (!std::all_of(target.begin(), target.end(), [](float v) { return std::isfinite(v); }))
        throw std::invalid_argument("Nonfinite hand target");
    try {
        if (impl_->status() == HoldStatus::frozen)
            return false;
        impl_->peer(side).hand(target, duration);
        return true;
    } catch (...) {
        impl_->rollback();
    }
}
void DualSession::stop() {
    if (!impl_->cleanup())
        throw std::runtime_error("Board stop unconfirmed");
}
std::array<std::uint16_t, 2> DualSession::local_ports(Side s) const {
    auto& p = impl_->peer(s);
    return {p.rt.local_port(), p.management_port()};
}
const BoardState& DualSession::state(Side s) const { return *impl_->peer(s).state; }
const RpcTiming& DualSession::rpc_timing(Side s) const { return impl_->peer(s).timing; }
std::vector<fault_event_value_t> DualSession::take_fault_events(Side s) {
    std::vector<fault_event_value_t> out;
    out.swap(impl_->peer(s).events);
    return out;
}
} // namespace panthera
