"""Run a pair of loopback boards on ephemeral ports for Python SDK clients."""
import threading
import time

from motion import MotionBoard


class ObservedBoard(MotionBoard):
    def __init__(self, node):
        super().__init__(node)
        self.stop_request_at = None
        self.publish_feedback = True

    def operation(self, request):
        if request["operation"] == 18:
            self.stop_request_at = time.monotonic()
        return super().operation(request)

    def send(self, host_port, name, message, now_us):
        if self.publish_feedback:
            super().send(host_port, name, message, now_us)


class BoardPair:
    def __init__(self, ready=False):
        self.boards = [ObservedBoard(11), ObservedBoard(12)]
        if ready:
            for board in self.boards:
                board.arm["q_rad"] = [0, 1, 1.3, -.5, 0, 0]
        self.end = threading.Event()
        self.thread = None
        self.error = None

    def addresses(self):
        import panthera_sdk as sdk
        return [sdk.BoardAddress(
            sdk.RealtimeAddress("127.0.0.1", 0, "127.0.0.1", b.port, b.node),
            0, b.management.getsockname()[1]) for b in self.boards]

    def start(self, ports):
        def run():
            cycle = 0
            deadline = time.monotonic()
            try:
                while not self.end.is_set():
                    for b, (rt, mgmt) in zip(self.boards, ports):
                        if mgmt:
                            b.service(rt, mgmt)
                        b.tick(rt, cycle)
                    cycle += 1
                    deadline += .002
                    now = time.monotonic()
                    if deadline < now:
                        deadline += (1 + int((now - deadline) / .002)) * .002
                    self.end.wait(max(0, deadline - now))
            except Exception as exc:
                self.error = exc
        self.thread = threading.Thread(target=run, name="panthera-loopback")
        self.thread.start()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.end.set()
        if self.thread:
            self.thread.join()
        for b in self.boards:
            b.close()
        if self.error:
            raise self.error
