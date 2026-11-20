"""Loopback board model for the host-control takeover and watchdog paths."""

from collections import OrderedDict
import math
import secrets
import socket
import time

from robot_protocol import IDS, ProtocolError, decode, encode, frame, unframe, zero
from telemetry import TelemetryBoard


class SessionBoard(TelemetryBoard):
    def __init__(self, node, rt_port=0, mgmt_port=0):
        super().__init__(node, rt_port)
        self.management = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.management.bind(("127.0.0.1", mgmt_port))
        self.management.setblocking(False)
        self.socket.setblocking(False)
        self.reset()

    def reset(self):
        self.boot = secrets.randbits(64) or 1
        self.sequence.clear()
        self.wire_boot = secrets.randbits(64) or 1
        self.wire_seq = self.host_boot = self.lease = 0
        self.revision = 1
        self.active = self.polling = False
        self.arm['fault_code_raw'] = [0] * 6
        self.homing_until = time.monotonic() + 0.05
        self.arm_at = self.arm_seq = self.hand_seq = 0
        self.mit = None
        self.mask = 0
        self.arm.update(control_owner=2, run_state=0, policy_revision=1,
                        last_applied_command_sequence=0)
        self.hand['last_applied_command_sequence'] = 0
        self.cache = OrderedDict()
        self.pending = OrderedDict()
        self.operations = []

    def close(self):
        self.management.close()
        self.socket.close()

    def stop(self):
        self.active = False
        self.lease = 0
        self.mit = None
        self.arm.update(control_owner=2, run_state=0)

    def operation(self, req):
        op = req["operation"]
        self.operations.append(op)
        source = req["header"]["source_boot_id"]
        if op == 19:
            return 0
        if op == 10:
            self.polling = not (req["target_mask"] & 8)
            return 0
        if op == 1:
            if time.monotonic() < self.homing_until:
                return -16
            if self.lease and source != self.host_boot:
                return -3
            if not self.lease:
                self.lease = secrets.randbits(64) or 1
                self.mit = None
                self.arm_seq = self.hand_seq = 0
            self.host_boot = source
            return 0
        if source != self.host_boot or not self.lease or req["control_lease"] != self.lease:
            return -1
        if op == 13 and req["target_mask"] == 0:
            self.polling = True
        elif op == 3:
            if (not self.polling or self.mit is None or self.mask != 63 or time.monotonic() - self.arm_at >= .020
                    or any(abs(self.mit[5*i] - q) > math.radians(2)
                           or abs(self.mit[5*i+1]) > math.radians(3)
                           or min(self.mit[5*i+2:5*i+4]) < 0
                           for i, q in enumerate(self.arm["q_rad"]))):
                return -16
            self.active = True
            self.arm.update(control_owner=1, run_state=1)
        elif op in (2, 18):
            self.stop()
        else:
            return -2
        return 0

    def rpc(self, mid, payload):
        names = {1025: ("OperationRequest", "OperationResult"),
                 1027: ("ConfigureRequest", "ConfigureResult")}
        if mid not in names:
            return
        name, result_name = names[mid]
        req = decode(name, payload)
        h = req["header"]
        if not self.valid_header(h) or not req["operation_id"]:
            return
        key = (h["source_boot_id"], mid, req["operation_id"])
        if key in self.cache:
            original, response = self.cache[key]
            if payload == original:
                return response
            return
        result = zero(result_name)
        if name == "OperationRequest":
            result["status"] = self.operation(req)
            result.update(control_lease=self.lease, observed_run_state=int(self.active))
        elif (h["source_boot_id"] == self.host_boot and self.lease and
              req["expected_policy_revision"] == self.revision and
              req["new_policy_revision"] > self.revision and req["hand_mode"] in (1, 2, 3)
              and req['arm_state_period_us'] and req['hand_state_period_us']):
            self.revision = self.arm["policy_revision"] = req["new_policy_revision"]
        else:
            result["status"] = -1
        result["header"].update(schema_revision=7, source_node=self.node, target_node=10,
                                source_boot_id=self.boot, target_boot_id=self.host_boot,
                                sequence=h["sequence"], tx_monotonic_us=time.monotonic_ns()//1000)
        result.update(operation_id=req["operation_id"], applied_policy_revision=self.revision)
        response = (IDS[result_name], encode(result_name, result))
        self.cache[key] = payload, response
        if len(self.cache) > 8:
            self.cache.popitem(last=False)
        return response

    def valid_header(self, h):
        return (h["schema_revision"] == 7 and h["source_node"] == 10 and
                h["target_node"] == self.node and h["source_boot_id"] and
                h["target_boot_id"] == self.boot)

    def realtime(self, mid, payload):
        if mid not in (257, 258):
            return
        msg = decode("ArmCommand" if mid == 257 else "HandCommand", payload)
        h = msg["header"]
        if (not self.valid_header(h) or h["source_boot_id"] != self.host_boot
                or not self.lease or msg["control_lease"] != self.lease
                or msg["policy_revision"] != self.revision):
            return
        previous = self.arm_seq if mid == 257 else self.hand_seq
        if previous and not 0 < (h["sequence"] - previous) % 2**32 < 2**31:
            return
        if mid == 257 and msg["joint_mask"] <= 63 and all(map(math.isfinite, msg["mit"])):
            self.arm_seq = self.arm["last_applied_command_sequence"] = h["sequence"]
            self.mit, self.mask, self.arm_at = msg["mit"], msg["joint_mask"], time.monotonic()
        elif mid == 258 and self.active and all(map(math.isfinite, msg["target_q_rad"])):
            self.hand_seq = self.hand["last_applied_command_sequence"] = h["sequence"]
            for index, q in zip((0, 1, 3, 5, 7, 9), msg["target_q_rad"]):
                self.hand["joint_q_rad"][index] = q

    def service(self, rt_host_port, mgmt_host_port):
        for sock, port in ((self.management, mgmt_host_port), (self.socket, rt_host_port)):
            for _ in range(32):
                try:
                    data, peer = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                except (ConnectionResetError, ConnectionRefusedError):
                    # UDP can report a previous send to a closed host port on receive.
                    # Continue servicing the board watchdog after host exit.
                    continue
                if peer != ("127.0.0.1", port):
                    continue
                try:
                    kind, mid, session, seq, payload = unframe(data)
                    if sock is self.socket and kind == 0:
                        self.realtime(mid, payload)
                    elif sock is self.management and kind == 2 and session == self.wire_boot:
                        self.pending.pop(seq, None)
                    elif sock is self.management and kind == 1:
                        sock.sendto(frame(0, b"", session, seq, 2), peer)
                        response = self.rpc(mid, payload)
                        if response and len(self.pending) < 8:
                            self.wire_seq += 1
                            self.pending[self.wire_seq] = (frame(*response, self.wire_boot, self.wire_seq, 1), 0, 0)
                except ProtocolError:
                    continue
        now = time.monotonic()
        if self.pending:
            seq, (packet, sent, tries) = next(iter(self.pending.items()))
            if now - sent >= .020:
                if tries == 6:
                    del self.pending[seq]
                else:
                    self.management.sendto(packet, ("127.0.0.1", mgmt_host_port))
                    self.pending[seq] = packet, now, tries + 1
        if self.active and now - self.arm_at >= .050:
            self.stop()
        for msg in (self.arm, self.hand, self.faults, self.diagnostics):
            msg["header"]["target_boot_id"] = self.host_boot


if __name__ == "__main__":
    left, right = SessionBoard(11, 15005, 15006), SessionBoard(12, 15015, 15016)
    try:
        cycle = 0
        deadline = time.monotonic()
        while True:
            for board, port in ((left, 5005), (right, 5015)):
                board.service(port, port+1)
                board.tick(port, cycle)
            cycle += 1
            deadline += .002
            if deadline <= time.monotonic():
                deadline = time.monotonic()
            time.sleep(max(0, deadline-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        left.close()
        right.close()
