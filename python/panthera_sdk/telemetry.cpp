#include "panthera/session.hpp"
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace panthera;
using namespace pybind11::literals;

namespace {
template <class T, size_t N> py::array_t<T> copied(const T (&values)[N]) {
    py::array_t<T> result(N);
    std::copy_n(values, N, result.mutable_data());
    return result;
}
template <class T> auto sample(py::module_& m, const char* name) {
    using S = TelemetrySample<T>;
    return py::class_<S>(m, name)
        .def_readonly("received", &S::received)
        .def_readonly("missing", &S::missing)
        .def_property_readonly(
            "received_at",
            [](const S& s) {
                return std::chrono::duration<double>(s.received_at.time_since_epoch()).count();
            })
        .def_property_readonly("sequence", [](const S& s) { return s.value.header.sequence; });
}
} // namespace
void bind_telemetry(py::module_& m) {
    py::class_<RealtimeAddress>(m, "RealtimeAddress")
        .def(py::init<std::string, std::uint16_t, std::string, std::uint16_t, std::uint16_t,
                      std::uint16_t>(),
             "local_ip"_a, "local_port"_a, "board_ip"_a, "board_port"_a, "board_node"_a,
             "host_node"_a = 10)
        .def_static("left", &RealtimeAddress::left)
        .def_static("right", &RealtimeAddress::right)
        .def_readwrite("local_ip", &RealtimeAddress::local_ip)
        .def_readwrite("local_port", &RealtimeAddress::local_port)
        .def_readwrite("board_ip", &RealtimeAddress::board_ip)
        .def_readwrite("board_port", &RealtimeAddress::board_port)
        .def_readwrite("board_node", &RealtimeAddress::board_node)
        .def_readwrite("host_node", &RealtimeAddress::host_node);
    py::class_<BoardAddress>(m, "BoardAddress")
        .def(py::init<RealtimeAddress, std::uint16_t, std::uint16_t>(), "realtime"_a,
             "management_local_port"_a, "management_board_port"_a)
        .def_static("left", &BoardAddress::left)
        .def_static("right", &BoardAddress::right)
        .def_readwrite("realtime", &BoardAddress::realtime)
        .def_readwrite("management_local_port", &BoardAddress::management_local_port)
        .def_readwrite("management_board_port", &BoardAddress::management_board_port);
    sample<arm_state_value_t>(m, "ArmStateSample")
        .def_property_readonly("last_applied_command_sequence",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return s.value.last_applied_command_sequence;
                               })
        .def_property_readonly(
            "command_age_us",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.command_age_us; })
        .def_property_readonly(
            "policy_revision",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.policy_revision; })
        .def_property_readonly(
            "control_owner",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.control_owner; })
        .def_property_readonly(
            "run_state",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.run_state; })
        .def_property_readonly(
            "compensation_mode",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.compensation_mode; })
        .def_property_readonly(
            "online_mask",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.online_mask; })
        .def_property_readonly(
            "valid_q_mask",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.valid_q_mask; })
        .def_property_readonly(
            "valid_dq_mask",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.valid_dq_mask; })
        .def_property_readonly(
            "valid_tau_mask",
            [](const TelemetrySample<arm_state_value_t>& s) { return s.value.valid_tau_mask; })
        .def_property_readonly("feedback_age_us",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return copied(s.value.feedback_age_us);
                               })
        .def_property_readonly(
            "q_rad",
            [](const TelemetrySample<arm_state_value_t>& s) { return copied(s.value.q_rad); })
        .def_property_readonly(
            "dq_rad_s",
            [](const TelemetrySample<arm_state_value_t>& s) { return copied(s.value.dq_rad_s); })
        .def_property_readonly("tau_measured_nm",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return copied(s.value.tau_measured_nm);
                               })
        .def_property_readonly("fault_code_raw",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return copied(s.value.fault_code_raw);
                               })
        .def_property_readonly("motor_mode_raw",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return copied(s.value.motor_mode_raw);
                               })
        .def_property_readonly("motor_fault_valid_mask",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return s.value.motor_fault_valid_mask;
                               })
        .def_property_readonly("last_state_probe_sequence",
                               [](const TelemetrySample<arm_state_value_t>& s) {
                                   return s.value.last_state_probe_sequence;
                               });
    sample<hand_state_value_t>(m, "HandStateSample")
        .def_property_readonly("last_applied_command_sequence",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return s.value.last_applied_command_sequence;
                               })
        .def_property_readonly(
            "command_age_us",
            [](const TelemetrySample<hand_state_value_t>& s) { return s.value.command_age_us; })
        .def_property_readonly(
            "online", [](const TelemetrySample<hand_state_value_t>& s) { return s.value.online; })
        .def_property_readonly(
            "valid_fields",
            [](const TelemetrySample<hand_state_value_t>& s) { return s.value.valid_fields; })
        .def_property_readonly("field_age_us",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.field_age_us);
                               })
        .def_property_readonly("motor_position_percent",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.motor_position_percent);
                               })
        .def_property_readonly("joint_q_rad",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.joint_q_rad);
                               })
        .def_property_readonly("motor_dq_rad_s",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.motor_dq_rad_s);
                               })
        .def_property_readonly("motor_current_ma",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.motor_current_ma);
                               })
        .def_property_readonly("fault_code_raw",
                               [](const TelemetrySample<hand_state_value_t>& s) {
                                   return copied(s.value.fault_code_raw);
                               })
        .def_property_readonly(
            "mode_raw",
            [](const TelemetrySample<hand_state_value_t>& s) { return s.value.mode_raw; })
        .def_property_readonly(
            "fault_valid_mask",
            [](const TelemetrySample<hand_state_value_t>& s) { return s.value.fault_valid_mask; })
        .def_property_readonly(
            "tactile_capable",
            [](const TelemetrySample<hand_state_value_t>& s) { return s.value.tactile_capable; })
        .def_property_readonly("tactile_valid", [](const TelemetrySample<hand_state_value_t>& s) {
            return s.value.tactile_valid;
        });
    sample<fault_snapshot_value_t>(m, "FaultSnapshotSample")
        .def_property_readonly("policy_revision",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.policy_revision;
                               })
        .def_property_readonly("mcu_fault_active",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.mcu_fault_active;
                               })
        .def_property_readonly("mcu_fault_latched",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.mcu_fault_latched;
                               })
        .def_property_readonly("mcu_fault_valid_mask",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.mcu_fault_valid_mask;
                               })
        .def_property_readonly(
            "local_action",
            [](const TelemetrySample<fault_snapshot_value_t>& s) { return s.value.local_action; })
        .def_property_readonly("arm_fault_raw",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return copied(s.value.arm_fault_raw);
                               })
        .def_property_readonly("arm_fault_valid_mask",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.arm_fault_valid_mask;
                               })
        .def_property_readonly("hand_fault_raw",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return copied(s.value.hand_fault_raw);
                               })
        .def_property_readonly("hand_fault_valid_mask",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.hand_fault_valid_mask;
                               })
        .def_property_readonly("latest_event_id",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.latest_event_id;
                               })
        .def_property_readonly("oldest_retained_event_id",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.oldest_retained_event_id;
                               })
        .def_property_readonly("event_overflow_count",
                               [](const TelemetrySample<fault_snapshot_value_t>& s) {
                                   return s.value.event_overflow_count;
                               });
    sample<diagnostics_value_t>(m, "DiagnosticsSample")
        .def_property_readonly(
            "can_tx_completed",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_tx_completed; })
        .def_property_readonly(
            "can_tx_failed",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_tx_failed; })
        .def_property_readonly(
            "can_rx_valid",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_rx_valid; })
        .def_property_readonly(
            "can_rx_unknown",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_rx_unknown; })
        .def_property_readonly(
            "can_rx_ambiguous",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_rx_ambiguous; })
        .def_property_readonly(
            "can_rx_malformed",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_rx_malformed; })
        .def_property_readonly(
            "can_bus_state",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.can_bus_state; })
        .def_property_readonly("can_tx_error_count",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.can_tx_error_count;
                               })
        .def_property_readonly("can_rx_error_count",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.can_rx_error_count;
                               })
        .def_property_readonly(
            "udp_rx_valid",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.udp_rx_valid; })
        .def_property_readonly(
            "udp_rx_bad",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.udp_rx_bad; })
        .def_property_readonly(
            "udp_rx_old",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.udp_rx_old; })
        .def_property_readonly(
            "udp_tx_drop",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.udp_tx_drop; })
        .def_property_readonly("control_period_mean_us",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.control_period_mean_us;
                               })
        .def_property_readonly("control_period_max_us",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.control_period_max_us;
                               })
        .def_property_readonly("control_jitter_abs_max_us",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.control_jitter_abs_max_us;
                               })
        .def_property_readonly(
            "command_coalesced",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.command_coalesced; })
        .def_property_readonly("counter_valid_mask",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.counter_valid_mask;
                               })
        .def_property_readonly("motor_feedback_count",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return copied(s.value.motor_feedback_count);
                               })
        .def_property_readonly("hand_field_feedback_count",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return copied(s.value.hand_field_feedback_count);
                               })
        .def_property_readonly("state_probe_received",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.state_probe_received;
                               })
        .def_property_readonly("last_state_probe_sequence",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.last_state_probe_sequence;
                               })
        .def_property_readonly("hand_state_query_mode",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_state_query_mode;
                               })
        .def_property_readonly("hand_mode_guard_phase",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_mode_guard_phase;
                               })
        .def_property_readonly("hand_mode_guard_status",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_mode_guard_status;
                               })
        .def_property_readonly("hand_mode_guard_elapsed_us",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_mode_guard_elapsed_us;
                               })
        .def_property_readonly("hand_mode_guard_token",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_mode_guard_token;
                               })
        .def_property_readonly(
            "hand_control_tx",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.hand_control_tx; })
        .def_property_readonly(
            "hand_control_ack",
            [](const TelemetrySample<diagnostics_value_t>& s) { return s.value.hand_control_ack; })
        .def_property_readonly("hand_query_timeouts",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_query_timeouts;
                               })
        .def_property_readonly("motor_control_tx",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return copied(s.value.motor_control_tx);
                               })
        .def_property_readonly("hand_full_query_tx",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_full_query_tx;
                               })
        .def_property_readonly("hand_control_negative_ack",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_control_negative_ack;
                               })
        .def_property_readonly("hand_control_ack_latest_status",
                               [](const TelemetrySample<diagnostics_value_t>& s) {
                                   return s.value.hand_control_ack_latest_status;
                               });
    py::class_<BoardState>(m, "BoardState")
        .def_readonly("arm", &BoardState::arm)
        .def_readonly("hand", &BoardState::hand)
        .def_readonly("faults", &BoardState::faults)
        .def_readonly("diagnostics", &BoardState::diagnostics)
        .def_readonly("boot_id", &BoardState::boot_id)
        .def_readonly("restarts", &BoardState::restarts)
        .def_readonly("encoder_jumps", &BoardState::encoder_jumps)
        .def("fresh_arm_feedback",
             [](const BoardState& s) { return s.fresh_arm_feedback(HostClock::now()); })
        .def(
            "active_hand_angles",
            [](const BoardState& s, int max_age_us) -> py::object {
                auto angles =
                    s.active_hand_angles(HostClock::now(), std::chrono::microseconds(max_age_us));
                if (!angles)
                    return py::none();
                py::array_t<float> a(angles->size());
                std::copy(angles->begin(), angles->end(), a.mutable_data());
                return a;
            },
            "max_age_us"_a = 30000);
    py::class_<BoardTelemetry>(m, "BoardTelemetry")
        .def(py::init<const RealtimeAddress&>())
        .def("poll", [](BoardTelemetry& rx) -> BoardState { return rx.poll(); })
        .def("local_port", &BoardTelemetry::local_port);
}
