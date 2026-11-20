from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from telemetry import TelemetryBoard, run


left, right = TelemetryBoard(11), TelemetryBoard(12)
with subprocess.Popen([sys.argv[1], str(left.port), str(right.port)],
                      stdout=subprocess.PIPE, text=True) as receiver:
    try:
        left_port, right_port = map(int, receiver.stdout.readline().split())
        start = time.monotonic()
        skipped = run(left, left_port, right, right_port, 500)
        elapsed = time.monotonic() - start
        print(f"500 cycles per board in {elapsed:.3f} s; skipped publisher deadlines: {skipped}")
        if receiver.wait(timeout=10):
            raise RuntimeError("Dual board telemetry capture failed")
    finally:
        if receiver.poll() is None:
            receiver.kill()
        left.socket.close()
        right.socket.close()
