"""Exercise all standalone Python example entry points against the installed wheel."""

from pathlib import Path
import subprocess
import sys

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

for name in (
    "read_state.py",
    "cartesian_move.py",
    "approach_stop.py",
    "hand_passthrough.py",
    "expert_mit.py",
):
    subprocess.run([sys.executable, EXAMPLES / name], check=True, timeout=60)

subprocess.run(
    [sys.executable, EXAMPLES / "dual_gello_teleop.py", "--mock-gello", "--headless"],
    check=True,
    timeout=60,
)
subprocess.run(
    [sys.executable, EXAMPLES / "dual_gello_teleop.py", "--monitor", "--mock-gello", "--headless"],
    check=True,
    timeout=30,
)

rc = subprocess.run([sys.executable, EXAMPLES / "expert_mit.py", "--real"]).returncode
assert rc == 2, f"Expected exit code 2 for real expert mode without ack, got {rc}"
