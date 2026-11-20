"""Exercise all example and WP-H entry points against the installed wheel."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
scripts = [
    "read_state",
    "cartesian_move",
    "approach_stop",
    "hand_passthrough",
    "expert_mit",
]
stages = [
    "stage1-telemetry",
    "stage2-takeover",
    "stage3-single-joint",
    "stage4-hand",
    "stage5-approach",
    "stage6-soak",
]

with tempfile.TemporaryDirectory() as temp:
    for name in scripts + stages:
        args = (
            [EXAMPLES / (name + ".py")]
            if name in scripts
            else [EXAMPLES / "wp_h_accept.py", name]
        )
        subprocess.run(
            [sys.executable, *args, "--smoke", "--output", temp], check=True, timeout=90
        )
        (report,) = Path(temp).glob(f"*/{name}-*.json")
        result = json.loads(report.read_text())
        assert (
            result["run_mode"] == "smoke_loopback"
            and result["automated_checks"] == "passed"
        )
        assert (
            result["actual_duration_s"] > 0
            and result["human_checks"] == "pending_human_confirmation"
        )
        assert report.with_suffix(".csv").stat().st_size > 300

    for args in (
        ["wp_h_accept.py", stages[0], "--real", "--smoke"],
        ["expert_mit.py", "--real"],
    ):
        rc = subprocess.run(
            [sys.executable, EXAMPLES / args[0], *args[1:], "--output", temp]
        ).returncode
        assert rc == 2
