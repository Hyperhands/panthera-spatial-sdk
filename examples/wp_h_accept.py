"""Attended WP-H checks and their loopback regression scenarios."""

import argparse
from contextlib import ExitStack
import csv
from datetime import datetime
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
import time

import numpy as np
import panthera_sdk as sdk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from loopback import BoardPair

SIDES = (sdk.Side.left, sdk.Side.right)
READY = np.array([0.0, 1.0, 1.3, -0.5, 0.0, 0.0])
BRANCH = np.array([0, np.pi / 2, 2.886410263, 0.255182391, 0, 0])
STAGES = (
    "stage1-telemetry",
    "stage2-takeover",
    "stage3-single-joint",
    "stage4-hand",
    "stage5-approach",
    "stage6-soak",
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class CanWindow:
    """Evaluate distinct Diagnostics samples for one board during this run."""

    def __init__(self, side):
        self.side = side
        self.last_sequence = None
        self.nonzero_since = None
        self.metrics = dict(
            samples=0,
            warning_samples=0,
            tx_error_peak=0,
            rx_error_peak=0,
            longest_observed_nonzero_s=0.0,
            last_sample_nonzero=False,
            tx_failed_start=None,
            tx_failed_delta=0,
            last_bus_state=None,
        )

    def observe(self, diagnostics):
        if diagnostics is None or self.last_sequence == diagnostics.sequence:
            return
        self.last_sequence = diagnostics.sequence
        metrics = self.metrics
        # Diagnostics validity bits 1, 6, 7, 8 cover TX failures, state, TEC, REC.
        required = 0x1C2
        require(
            diagnostics.counter_valid_mask & required == required,
            f"{self.side}: invalid CAN diagnostics",
        )
        if metrics["tx_failed_start"] is None:
            metrics["tx_failed_start"] = diagnostics.can_tx_failed
        metrics["samples"] += 1
        metrics["tx_failed_delta"] = (
            diagnostics.can_tx_failed - metrics["tx_failed_start"]
        ) & 0xFFFFFFFF
        metrics["last_bus_state"] = diagnostics.can_bus_state
        require(
            diagnostics.can_bus_state == 0,
            f"{self.side}: CAN bus state {diagnostics.can_bus_state} (expected ERROR_ACTIVE)",
        )
        require(
            metrics["tx_failed_delta"] == 0,
            f"{self.side}: CAN TX failures increased by {metrics['tx_failed_delta']}",
        )
        metrics["tx_error_peak"] = max(
            metrics["tx_error_peak"], diagnostics.can_tx_error_count
        )
        metrics["rx_error_peak"] = max(
            metrics["rx_error_peak"], diagnostics.can_rx_error_count
        )
        metrics["last_sample_nonzero"] = bool(
            diagnostics.can_tx_error_count or diagnostics.can_rx_error_count
        )
        if metrics["last_sample_nonzero"]:
            metrics["warning_samples"] += 1
            if self.nonzero_since is None:
                self.nonzero_since = diagnostics.received_at
            metrics["longest_observed_nonzero_s"] = max(
                metrics["longest_observed_nonzero_s"],
                diagnostics.received_at - self.nonzero_since,
            )
        else:
            self.nonzero_since = None


class Record:
    def __init__(self, output, stage, real):
        stamp = datetime.now()
        self.folder = output / stamp.strftime("%Y%m%d")
        self.folder.mkdir(parents=True, exist_ok=True)
        self.base = self.folder / f"{stage}-{stamp:%H%M%S-%f}"
        self.start = time.monotonic()
        self.summary = dict(
            stage=stage,
            run_mode="real" if real else "smoke_loopback",
            human_checks="pending_human_confirmation",
        )
        self.can = [CanWindow("left"), CanWindow("right")]
        self.summary["can"] = {w.side: w.metrics for w in self.can}
        manual = {
            STAGES[0]: ["boot_homing", "encoder_branches_after_power_cycle"],
            STAGES[1]: ["soft_hold_posture", "disconnected_board_stop"],
            STAGES[2]: [
                "joint_directions",
                "wrist_clearance",
                "adapter_dimensions_mass",
            ],
            STAGES[3]: ["finger_order_sign_scale", "physical_endpoints"],
            STAGES[4]: ["measured_inter_arm_gap"],
            STAGES[5]: ["temperature", "link_quality_trend"],
        }
        self.summary["field_checks"] = dict.fromkeys(
            manual.get(stage, []), "pending_human_confirmation"
        )
        self.stream = self.base.with_suffix(".csv").open(
            "x", newline="", encoding="utf-8"
        )
        self.csv = csv.writer(self.stream)
        self.csv.writerow(
            [
                "elapsed_s",
                "side",
                "received_at",
                "sequence",
                "received",
                "missing",
                "owner",
                "q_rad",
                "dq_rad_s",
                "tau_measured_nm",
                "feedback_age_us",
                "boot_id",
                "restarts",
                "hand_q_rad",
                "hard_margin_m",
                "inter_arm_m",
                "qp_us",
                "cycles",
                "missed_cycles",
            ]
        )
        self.last_cycles = None
        self.solve_us = []

    def row(self, i, board, control=None, inter=None):
        a = board.arm
        if a is None:
            return
        hand = board.hand
        self.csv.writerow(
            [
                time.monotonic() - self.start,
                i,
                a.received_at,
                a.sequence,
                a.received,
                a.missing,
                a.control_owner,
                a.q_rad.tolist(),
                a.dq_rad_s.tolist(),
                a.tau_measured_nm.tolist(),
                a.feedback_age_us.tolist(),
                board.boot_id,
                board.restarts,
                hand.joint_q_rad.tolist() if hand else None,
                control.hard_margin if control else None,
                inter,
                control.qp_time * 1e6 if control else None,
                control.cycles if control else None,
                control.missed_cycles if control else None,
            ]
        )

    def check_can_recovery(self):
        for window in self.can:
            require(
                not window.metrics["last_sample_nonzero"],
                f"{window.side}: CAN counters still nonzero at end; recovery unconfirmed",
            )

    def finish(self, error):
        self.stream.close()
        self.summary.update(
            actual_duration_s=time.monotonic() - self.start,
            automated_checks="failed" if error else "passed",
        )
        if error:
            self.summary["error"] = str(error) or type(error).__name__
        path = self.base.with_suffix(".json")
        path.write_text(
            json.dumps(self.summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        print(path, flush=True)


def check_board(board):
    a = board.arm
    require(
        a is not None and board.fresh_arm_feedback(),
        "Missing or stale six-axis feedback",
    )
    require(
        not board.restarts and not board.encoder_jumps,
        "Board restarted or encoder jumped",
    )
    require(not np.any(a.fault_code_raw), "Motor fault")
    if board.faults:
        f = board.faults
        require(
            not f.mcu_fault_active
            and not np.any(f.arm_fault_raw)
            and not np.any(f.hand_fault_raw),
            "Active board, arm or hand fault",
        )


def telemetry(addresses, pair, report, real):
    rx = [sdk.BoardTelemetry(a.realtime) for a in addresses]
    if pair:
        pair.start([(r.local_port(), 0) for r in rx])
    samples = [[], []]
    previous = [None, None]
    last = [None, None]
    duration = 60 if real else 0.5
    start = time.monotonic()
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            pending = any(w.metrics["last_sample_nonzero"] for w in report.can)
            if not pending or elapsed >= duration + 0.5:
                break
        for i, r in enumerate(rx):
            b = r.poll()
            if b.arm is None:
                require(time.monotonic() - start < 0.5, "No telemetry")
                continue
            a = b.arm
            diagnostics = b.diagnostics
            # Allow the 10 Hz diagnostics stream the same startup grace as ArmState.
            diagnostic_age = (
                a.received_at - diagnostics.received_at
                if diagnostics
                else time.monotonic() - start
            )
            require(
                diagnostic_age < 0.5,
                f"{report.can[i].side}: missing or stale CAN diagnostics",
            )
            report.can[i].observe(diagnostics)
            check_board(b)
            if previous[i] != a.sequence:
                samples[i].append((a.received, a.received_at, a.missing))
                previous[i] = a.sequence
                report.row(i, b)
            last[i] = b
        time.sleep(0.0005)
    metrics = report.summary["telemetry"] = []
    for i, b in enumerate(last):
        require(
            b and b.arm and b.diagnostics and b.hand and b.faults,
            "Incomplete telemetry streams",
        )
        rows = np.array(samples[i])
        require(len(rows) > 2, "Insufficient samples")
        received, elapsed, missing = rows[-1] - rows[0]
        rate = received / elapsed
        intervals = np.diff(rows[:, 1])[np.diff(rows[:, 0]) == 1] * 1e3
        require(intervals.size > 0, "No consecutive samples for timing")
        q = b.arm.q_rad
        zero = 2 * np.pi * np.floor((q - BRANCH) / (2 * np.pi) + 0.5)
        metrics.append(
            dict(
                rate_hz=rate,
                missing=int(missing),
                consecutive_interval_ms_p50_p99=np.percentile(
                    intervals, [50, 99]
                ).tolist(),
                encoder_offset_rad=zero.tolist(),
                raw_q_rad=q.tolist(),
            )
        )
        require(np.max(np.abs(q - zero)) < np.deg2rad(2), "Zero branch mismatch")
        require(b.arm.control_owner == 2, "Expected board-owned hold")
        if real:
            require(
                missing / (received + missing) < 0.001 and 475 <= rate <= 525,
                "Telemetry rate/loss",
            )


class Motion:
    def __init__(self, arm, report, real):
        self.arm, self.report, self.real = arm, report, real

    def sample(self):
        s = self.arm.state()
        require(s.enabled and not s.error, s.error or "Controller stopped")
        require(s.hard_margin > 0, "Nonpositive hard margin")
        if s.cycles != self.report.last_cycles:
            self.report.last_cycles = s.cycles
            for i, b in enumerate(s.boards):
                # The native servo owns feedback-age decisions during motion.
                require(
                    not b.restarts and not b.encoder_jumps,
                    "Unexpected board restart or jump",
                )
                self.report.can[i].observe(b.diagnostics)
                self.report.row(i, b, s)
            if s.qp_time:
                self.report.solve_us.append(s.qp_time * 1e6)
        return s

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            state = self.sample()
            if predicate(state):
                return state
            time.sleep(0.006)
        raise RuntimeError(f"Timed out waiting for {label}")

    def hold(self, seconds):
        end = time.monotonic() + seconds
        self.wait(lambda s: time.monotonic() >= end, seconds + 1, "hold")

    def joints(self, goals, tolerance=0.004):
        for side, goal in zip(SIDES, goals):
            self.arm.set_joint_target(side, goal)
        target = np.concatenate(goals)
        return self.wait(
            lambda s: np.max(np.abs(s.measured - target)) < tolerance,
            20,
            "measured joint targets",
        )

    def hands(self, side, q):
        self.arm.set_hand_target(side, q, 1000)

        def reached(s):
            angles = s.boards[SIDES.index(side)].active_hand_angles(100000)
            return angles is not None and np.max(np.abs(q - angles)) < 0.025

        self.wait(reached, 4, "fresh hand feedback")

    def cartesian(self, full=False):
        initial = self.sample().tcp
        goals = [pose.copy() for pose in initial]
        # Positive world x is common to both arms.
        for g in goals:
            g[0, 3] += 0.05 if full else 0.012
        poses = [goals, initial]
        if full:
            angle = np.deg2rad(15)
            for sign in (-1, 1):
                c, v = np.cos(angle), sign * np.sin(angle)
                rotated = [p.copy() for p in initial]
                for p in rotated:
                    p[:3, :3] = np.array([[c, -v, 0], [v, c, 0], [0, 0, 1]]) @ p[:3, :3]
                poses.extend([rotated, initial])
            backward = [p.copy() for p in initial]
            for p in backward:
                p[0, 3] -= 0.05
            poses.extend([backward, initial])
        for targets in poses:
            for side, target in zip(SIDES, targets):
                self.arm.set_pose_target(side, target)

            def reached(s):
                return all(
                    np.linalg.norm(p[:3, 3] - g[:3, 3]) < 0.003
                    and np.linalg.norm(p[:3, :3] - g[:3, :3]) < 0.026
                    for p, g in zip(s.tcp, targets)
                )

            self.wait(reached, 10, "Cartesian pose")

    def approach(self):
        state = self.sample()
        require(
            sdk.collision_state(state.measured).inter_arm > 0.15,
            "Approach starts too close",
        )
        for i, side in enumerate(SIDES):
            target = state.tcp[i].copy()
            target[1, 3] = -0.005 if i == 0 else 0.005
            self.arm.set_pose_target(side, target)
        stable_since = None
        minimum = float("inf")

        def stopped(s):
            nonlocal stable_since, minimum
            distance = sdk.collision_state(s.measured).inter_arm
            minimum = min(distance, minimum)
            self.report.row(0, s.boards[0], s, distance)
            require(distance >= 0.019, "Inter-arm clearance below 19 mm")
            dq = np.concatenate([b.arm.dq_rad_s for b in s.boards])
            if distance < 0.04 and np.linalg.norm(dq) < 0.02 and not s.frozen:
                if stable_since is None:
                    stable_since = time.monotonic()
                return time.monotonic() - stable_since >= 0.2
            stable_since = None
            return False

        self.wait(stopped, 35, "settled inter-arm clearance")
        self.report.summary["minimum_inter_arm_m"] = minimum


def takeover(m, pair):
    require(
        np.max(np.abs(m.sample().measured)) < np.deg2rad(2),
        "Start takeover check at zero",
    )
    m.hold(10 if m.real else 0.1)
    goal = READY if m.real else np.array([0, 0.07, 0.07, 0, 0, 0])
    m.joints([goal, goal])
    zero = np.zeros(6)
    returned = m.joints([zero, zero], tolerance=np.deg2rad(2))
    m.report.summary["return_zero_measured_rad"] = returned.measured.tolist()
    m.joints([goal, goal])
    m.arm.soft_stop()
    m.arm.enable()
    if pair:
        for b in pair.boards:
            b.stop_request_at = None
        pair.boards[1].publish_feedback = False
    else:
        print("在工作区外断开右板上行遥测。等待 60 秒。", flush=True)
    events = dict(
        freeze_observed_s=None,
        stop_observed_s=None,
        op18_received_s=["unconfirmed"] * 2,
    )
    m.report.summary["disconnect"] = events
    events["stop_request_sent_s"] = "unconfirmed"
    end = time.monotonic() + (60 if m.real else 2)
    while time.monotonic() < end:
        s = m.arm.state()
        observed = time.monotonic() - m.report.start
        if s.frozen and events["freeze_observed_s"] is None:
            events["freeze_observed_s"] = observed
        if not s.enabled:
            events["stop_observed_s"] = observed
            require(
                "feedback expired" in s.error or "release unconfirmed" in s.error,
                f"Unexpected stop: {s.error}",
            )
            break
        time.sleep(0.001)
    require(
        all(events[k] is not None for k in ("freeze_observed_s", "stop_observed_s")),
        "Missing freeze/stop transition",
    )
    if pair:
        require(
            all(
                18 in b.operations and not b.active and b.stop_request_at
                for b in pair.boards
            ),
            "Missing board-side soft stop",
        )
        events["op18_received_s"] = [
            b.stop_request_at - m.report.start for b in pair.boards
        ]
    else:
        events["board_phase"] = ["pending_human_confirmation", "unconfirmed"]
        events["sdk_stop_result"] = s.error


def single_joint(m):
    m.joints([READY, READY])
    for i in range(2):
        for j in range(6):
            for delta in (0.09, -0.09) if m.real else (0.025,):
                goals = [READY.copy(), READY.copy()]
                goals[i][j] += delta
                m.joints(goals)
                m.joints([READY, READY])
            if m.real and j == 4:
                for sign in (-1, 1):
                    goals = [READY.copy(), READY.copy()]
                    goals[i][j] = sign * np.deg2rad(60)
                    m.joints(goals)
                    m.joints([READY, READY])


def hand_check(m, endpoints=False):
    if endpoints:
        model = ET.parse(
            ROOT / "model/panthera_dual_description/urdf/panthera_dual_a2l1.urdf"
        )
        drivers = (
            "thumb_cmc",
            "thumb_pip",
            "index_mcp",
            "middle_mcp",
            "ring_mcp",
            "little_mcp",
        )
        observations = m.report.summary["observed_hand_endpoints_rad"] = {}

    for side in SIDES:
        for j in range(6):
            q = np.zeros(6)
            q[j] = 0.10
            m.hands(side, q)
            if endpoints:
                name = (
                    ("left" if side == sdk.Side.left else "right")
                    + "_"
                    + drivers[j]
                    + "_joint"
                )
                maximum = float(model.find(f"joint[@name='{name}']/limit").get("upper"))
                while q[j] < maximum:
                    reply = input(
                        f"{name} 当前 {q[j]:.3f} rad。输入 + 前进最多 0.1 rad，输入 end 记录端点并张开："
                    ).strip()
                    if reply == "end":
                        break
                    require(reply == "+", "Operator ended hand travel check")
                    q[j] = min(maximum, q[j] + 0.1)
                    m.hands(side, q)
                angles = m.sample().boards[SIDES.index(side)].active_hand_angles(100000)
                require(angles is not None, "Stale hand endpoint feedback")
                observations[name] = float(angles[j])
            m.hands(side, np.zeros(6))


def expert(m, pair):
    s = m.sample()
    commands = []
    for i, side in enumerate(SIDES):
        raw = np.zeros((6, 5))
        raw[:, 0] = s.boards[i].arm.q_rad
        raw[:, 2] = [24.48, 61.2, 61.2, 12.24, 12.24, 12.24]
        raw[:, 3] = [1.2, 1.92, 1.92, 0.48, 0.48, 0.48]
        raw[:, 4] = sdk.arm_geometry(side, s.measured[i * 6 : i * 6 + 6]).gravity
        commands.append(raw.ravel())
    m.arm.enter_expert_mode(sdk.ExpertAck.IUnderstandNoSafety)
    for _ in range(25):
        for side, command in zip(SIDES, commands):
            m.arm.send_mit(side, command)
        time.sleep(0.002)
    if pair:
        require(
            all(
                b.active and np.allclose(b.mit, cmd)
                for b, cmd in zip(pair.boards, commands)
            ),
            "Board did not receive the expert hold command",
        )


def soak(m):
    m.joints([READY, READY])
    start = m.sample()
    begin = time.monotonic()
    duration = 3600 if m.real else 1
    direction = 1
    while time.monotonic() - begin < duration:
        goals = [READY.copy(), READY.copy()]
        for q in goals:
            q[1] += direction * 0.04
        m.joints(goals)
        for side in SIDES:
            m.hands(side, np.full(6, 0.08 if direction > 0 else 0))
        if m.real:
            m.joints([READY, READY])
            m.approach()
            m.joints([READY, READY])
        direction *= -1
    stop = m.sample()
    missed = stop.missed_cycles - start.missed_cycles
    fraction = missed / max(1, missed + stop.cycles - start.cycles)
    p99 = float(np.percentile(m.report.solve_us, 99))
    m.report.summary.update(
        soak_s=time.monotonic() - begin, missed_fraction=fraction, sampled_qp_p99_us=p99
    )
    if m.real:
        require(fraction < 0.001 and p99 < 100, "Soak timing limits exceeded")


def main(example=None):
    p = argparse.ArgumentParser(description=__doc__)
    if example is None:
        p.add_argument("stage", choices=STAGES)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true")
    mode.add_argument("--loopback", action="store_true")
    p.add_argument("--smoke", action="store_true", help="short loopback regression")
    p.add_argument("--output", type=Path, default=Path("validation"))
    p.add_argument("--i-understand-no-safety", action="store_true")
    args = p.parse_args()
    stage = example or args.stage
    if args.real and args.smoke:
        p.error("--smoke requires loopback")
    if args.real and stage == "expert_mit" and not args.i_understand_no_safety:
        p.error("Real expert mode requires --i-understand-no-safety")
    record = Record(args.output, stage, args.real)
    try:
        with ExitStack() as stack:
            pair = None
            addresses = (
                [sdk.BoardAddress.left(), sdk.BoardAddress.right()]
                if args.real
                else None
            )
            if not args.real:
                pair = stack.enter_context(
                    BoardPair(ready=stage not in (*STAGES[:2], "read_state"))
                )
                addresses = pair.addresses()
            if stage in ("stage1-telemetry", "read_state"):
                telemetry(addresses, pair, record, args.real)
            else:
                config = sdk.ControlConfig()
                config.joints.max_speed = np.full(12, 0.25)
                config.translation_speed = 0.05
                if stage in (STAGES[4], STAGES[5], "approach_stop"):
                    config.pose_gain = 2.0
                    config.regularization = 0.025
                    config.posture_weight = 0.0001
                arm = stack.enter_context(sdk.DualArm(*addresses, config))
                if pair:
                    pair.start([arm.local_ports(s) for s in SIDES])
                arm.enable(timeout_ms=30000 if args.real else 3000)
                motion = Motion(arm, record, args.real)
                if stage == STAGES[1]:
                    takeover(motion, pair)
                elif stage == STAGES[2]:
                    single_joint(motion)
                elif stage in (STAGES[3], "hand_passthrough"):
                    hand_check(motion, endpoints=args.real and stage == STAGES[3])
                elif stage in (STAGES[4], "approach_stop", "cartesian_move"):
                    motion.joints([READY, READY])
                    if stage != "approach_stop":
                        motion.cartesian(full=args.real and stage == STAGES[4])
                    if stage != "cartesian_move":
                        motion.approach()
                elif stage == STAGES[5]:
                    soak(motion)
                elif stage == "expert_mit":
                    expert(motion, pair)
        record.check_can_recovery()
    except BaseException as exc:
        record.finish(exc)
        raise
    record.finish(None)


if __name__ == "__main__":
    main()
