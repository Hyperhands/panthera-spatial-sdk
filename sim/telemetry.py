"""Publish the four realtime streams for two board-owned arms on loopback."""

from itertools import count
import secrets
import socket
import time

from robot_protocol import IDS, encode, frame, zero


class TelemetryBoard:
    def __init__(self, node, port=0):
        self.node = node
        self.boot = secrets.randbits(64) or 1
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.port = self.socket.getsockname()[1]
        self.sequence = {}
        self.arm = zero("ArmState")
        self.arm.update(policy_revision=1, control_owner=2, online_mask=63,
                        valid_q_mask=63, valid_dq_mask=63, valid_tau_mask=63,
                        motor_fault_valid_mask=63, feedback_age_us=[1000] * 6)
        self.hand = zero("HandState")
        self.hand.update(online=True, valid_fields=2)
        self.faults = zero("FaultSnapshot")
        self.faults.update(policy_revision=1, arm_fault_valid_mask=63)
        self.diagnostics = zero("Diagnostics")
        self.diagnostics.update(control_period_mean_us=2000)

    def send(self, host_port, name, message, now_us):
        sequence = (self.sequence.get(name, 0) + 1) & 0xFFFFFFFF
        self.sequence[name] = sequence
        message["header"].update(schema_revision=7, source_node=self.node, target_node=10,
                                 source_boot_id=self.boot, sequence=sequence, tx_monotonic_us=now_us)
        packet = frame(IDS[name], encode(name, message))
        self.socket.sendto(packet, ("127.0.0.1", host_port))

    def tick(self, host_port, cycle):
        now_us = time.monotonic_ns() // 1000
        self.send(host_port, "ArmState", self.arm, now_us)
        if cycle % 5 == 0:
            self.send(host_port, "HandState", self.hand, now_us)
        if cycle % 25 == 0:
            self.send(host_port, "FaultSnapshot", self.faults, now_us)
        if cycle % 50 == 0:
            self.send(host_port, "Diagnostics", self.diagnostics, now_us)


def run(left, left_host_port, right, right_host_port, cycles=None):
    period_ns = 2_000_000
    deadline = time.monotonic_ns()
    missed_deadlines = 0
    for cycle in range(cycles) if cycles is not None else count():
        left.tick(left_host_port, cycle)
        right.tick(right_host_port, cycle)
        deadline += period_ns
        remaining = deadline - time.monotonic_ns()
        if remaining <= 0:
            # Skip elapsed deadlines; never burst several stale snapshots.
            missed = 1 + (-remaining) // period_ns
            missed_deadlines += missed
            deadline += missed * period_ns
        time.sleep(max(0, deadline - time.monotonic_ns()) / 1e9)
    return missed_deadlines


if __name__ == "__main__":
    left = TelemetryBoard(11, 15005)
    right = TelemetryBoard(12, 15015)
    try:
        run(left, 5005, right, 5015)
    except KeyboardInterrupt:
        pass
    finally:
        left.socket.close()
        right.socket.close()
