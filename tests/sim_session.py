"""Exercise C++ session semantics over four UDP sockets, with independent board faults."""
from pathlib import Path
import math
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from robot_protocol import decode, encode, frame, zero
from session import SessionBoard


class Board(SessionBoard):
    def __init__(self, node, scenario):
        super().__init__(node)
        self.scenario = scenario
        self.arm["q_rad"][1] = 1.5 + (1 if node == 11 else -1) * math.tau
        self.is_right = node == 12
        self.holds = self.activations = self.soft_stops = self.claim_retries = 0
        self.reordered = self.lost_result = False

    def rpc(self, mid, payload):
        response = super().rpc(mid, payload)
        if mid == 1025 and self.is_right:
            op = decode("OperationRequest", payload)["operation"]
            if op == 3 and self.scenario == "timeout" and self.active:
                return None  # Applied successfully, only the result is lost.
            if op == 19 and not self.lost_result:
                self.lost_result = True
                return None  # Transport ACK alone cannot complete a request.
        return response

    def operation(self, req):
        op = req["operation"]
        if op == 3 and self.is_right:
            if self.scenario == "refused":
                return -3
            if not self.reordered and self.scenario != "timeout":
                self.reordered = True
                return -16
        if op == 1 and not self.claim_retries:
            self.claim_retries += 1
            return -16
        result = super().operation(req)
        if result == 0:
            self.activations += op == 3
            self.soft_stops += op == 18
        return result

    def realtime(self, mid, payload):
        super().realtime(mid, payload)
        if mid == 257 and self.mit:
            assert abs(self.mit[5] - self.arm["q_rad"][1]) < .001
            assert self.mit[6] == 0 and self.mit[9] == (2.25 if self.is_right else 1.25)
            assert abs(self.mit[7] - 61.2) < .001 and abs(self.mit[8] - 1.92) < .001
            self.holds += 1


def run(exe, scenario):
    left, right = Board(11, scenario), Board(12, scenario)
    boards = (left, right)
    ports = [str(port) for b in boards for port in (b.port, b.management.getsockname()[1])]
    with subprocess.Popen([exe, *ports, scenario], stdout=subprocess.PIPE, text=True) as client:
        try:
            lr, lm, rr, rm = map(int, client.stdout.readline().split())
            cycle = 0
            injected = False
            deadline = time.monotonic() + 8
            while client.poll() is None and time.monotonic() < deadline:
                left.service(lr, lm)
                right.service(rr, rm)
                if left.hand_seq and not injected:
                    injected = True
                    if scenario == "reboot":
                        right.reset()
                    elif scenario == "jump":
                        right.arm["q_rad"][3] += math.tau
                    elif scenario == "fault":
                        right.arm["fault_code_raw"][0] = 10
                    elif scenario == "success":
                        msg = zero("FaultEvent")
                        msg.update(event_id=1, active=False)
                        msg["header"].update(right.arm["header"])
                        for _ in range(2):
                            right.wire_seq += 1
                            right.pending[right.wire_seq] = (frame(769, encode("FaultEvent",msg),right.wire_boot,right.wire_seq,1),0,0)
                left.tick(lr, cycle)
                if not (injected and scenario == "uplink"):
                    right.tick(rr, cycle)
                cycle += 1
                time.sleep(.001)
            assert client.wait(timeout=1) == 0, scenario
            assert left.activations == left.soft_stops and left.activations >= 1, scenario
            assert not any(b.active or b.lease for b in boards), scenario
            assert all(11 not in b.operations and b.holds for b in boards)
            if scenario == "reboot":
                assert left.activations == right.activations == 2 and right.soft_stops == 1
            if scenario == "timeout":
                assert right.activations == right.soft_stops == 1
            if scenario == "success":
                assert right.operations.count(19) == 1 and right.reordered and right.lost_result
            print(scenario, "passed", flush=True)
        finally:
            if client.poll() is None:
                client.kill()
            for b in boards:
                b.close()


for case in ("success", "refused", "timeout", "uplink", "reboot", "jump", "fault"):
    run(sys.argv[1], case)
