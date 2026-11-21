"""Exercise all example and WP-H entry points against the installed wheel."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
sys.path.insert(0, str(EXAMPLES))
from wp_h_accept import BoardPair, Record, telemetry


def check_can_window(temp, scenario, expected_error=None):
    with BoardPair() as pair:
        right = pair.boards[1]
        original = right.send
        diag_samples = 0

        def send(port, name, message, now):
            nonlocal diag_samples
            if name == "Diagnostics":
                diag_samples += 1
                message["can_tx_failed"] = 9 + int(
                    scenario == "failed" and diag_samples >= 2
                )
                message["can_tx_error_count"] = (
                    12 if diag_samples in (2, 3) or scenario == "unrecovered" else 0
                )
                message["can_rx_error_count"] = 1 if diag_samples == 3 else 0
                if scenario == "degraded" and diag_samples == 2:
                    message["can_bus_state"] = 1
            if name == "ArmState" and scenario == "stale" and diag_samples >= 2:
                message["feedback_age_us"] = [20_000] * 6
            original(port, name, message, now)

        right.send = send
        report = Record(Path(temp), scenario, False)
        try:
            telemetry(pair.addresses(), pair, report, False)
            report.check_can_recovery()
        except RuntimeError as exc:
            report.finish(exc)
            assert expected_error and expected_error in str(exc), str(exc)
        else:
            report.finish(None)
            assert expected_error is None
            metrics = json.loads(report.base.with_suffix(".json").read_text())["can"]
            assert metrics["left"]["warning_samples"] == 0
            right_metrics = metrics["right"]
            assert right_metrics["warning_samples"] == 2
            assert (
                right_metrics["tx_error_peak"] == 12
                and right_metrics["rx_error_peak"] == 1
            )
            assert right_metrics["longest_observed_nonzero_s"] > 0
            assert not right_metrics["last_sample_nonzero"]
            assert (
                right_metrics["tx_failed_start"] == 9
                and right_metrics["tx_failed_delta"] == 0
            )


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
    check_can_window(temp, "recovered")
    check_can_window(temp, "failed", "right: CAN TX failures increased by 1")
    check_can_window(temp, "degraded", "right: CAN bus state 1")
    check_can_window(temp, "unrecovered", "recovery unconfirmed")
    check_can_window(temp, "stale", "Missing or stale six-axis feedback")
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
