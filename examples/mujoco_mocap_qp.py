"""Drag the two mocap targets against MuJoCo simulation or real boards.

Interactive shortcuts inside the MuJoCo window:
  - Double-click a target sphere (blue=left, orange=right) to select it
  - Ctrl + right-drag: translate target
  - Ctrl + left-drag:  rotate target (effective in 6D mode)
  - O: toggle Position-Only (free wrist, default) <-> 6D orientation lock
  - S / R: snap both target handles back to the current TCP poses
  - E: re-enable controller if it stopped
"""

import argparse
import copy
import json
import sys
import threading
import time
from contextlib import ExitStack
from pathlib import Path

import mujoco as mj
import numpy as np
import panthera_sdk as sdk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from mujoco_board import HAND, Plant, indices, load_scene

SIDES = (sdk.Side.left, sdk.Side.right)
SIDE_NAMES = ("left", "right")
READY = np.array([0.0, 1.0, 1.3, -0.5, 0.0, 0.0])
READY_TOLERANCE_RAD = np.deg2rad(2.0)

WEIGHTS_POS = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
WEIGHTS_6D = np.array([1.0, 1.0, 1.0, 0.15, 0.15, 0.15])
MAX_ROT_LEASH_RAD = np.deg2rad(35.0)


class ReleaseSnap:
    """Snap a handle back to TCP only after that handle was dragged, released, and settled."""

    def __init__(self, settle_s=0.4):
        self.settle_s = settle_s
        self.dragging = False
        self.anchor = None
        self.since = 0.0

    def update(self, active, pose, now):
        if active:
            self.dragging = True
            self.anchor = None
        elif self.dragging:
            self.dragging = False
            self.anchor = pose.copy()
            self.since = now
        elif self.anchor is not None:
            motion = float(np.linalg.norm(pose[:3, 3] - self.anchor[:3, 3]))
            rotation = rotation_error_rad(pose[:3, :3], self.anchor[:3, :3])
            if motion > 0.0015 or rotation > np.deg2rad(1.0):
                self.anchor = pose.copy()
                self.since = now
            elif self.settle_s > 0 and now - self.since >= self.settle_s:
                self.anchor = None
                return True
        return False

DEFAULT_RGBA = (
    np.array([0.15, 0.70, 1.00, 0.65]),  # left: sky blue
    np.array([1.00, 0.40, 0.18, 0.65]),  # right: orange
)
YELLOW_RGBA = np.array([1.00, 0.85, 0.10, 0.85])
RED_RGBA = np.array([1.00, 0.12, 0.12, 0.90])


def load_pair_labels():
    path = ROOT / "generated" / "model" / "capsules.json"
    if not path.is_file():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    caps = data["capsules"]
    labels = []
    for i, j in data["pairs"]:
        la = caps[i]["links"][0].replace("_base_link", "").replace("_cmc_link", "_thumb")
        lb = caps[j]["links"][0].replace("_base_link", "").replace("_cmc_link", "_thumb")
        labels.append(f"{la} <-> {lb}")
    return labels


def scene_handles(model):
    handles = []
    for name in SIDE_NAMES:
        aq, av = indices(model, [f"{name}_joint{i}" for i in range(1, 7)])
        hq, _ = indices(model, [f"{name}_{joint}_joint" for joint in HAND])
        body_id = model.body(f"{name}_target").id
        geom_id = int(model.body_geomadr[body_id])
        site_ids = np.where(model.site_bodyid == body_id)[0]
        handles.append(
            {
                "aq": aq,
                "av": av,
                "hq": hq,
                "target_id": int(model.body_mocapid[body_id]),
                "geom_id": geom_id,
                "site_id": int(site_ids[0]) if len(site_ids) else -1,
            }
        )
    return handles


def mocap_to_pose(position, quat):
    pose = np.eye(4)
    norm = float(np.linalg.norm(quat))
    unit = quat / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])
    rotation = np.zeros(9)
    mj.mju_quat2Mat(rotation, unit)
    pose[:3, :3] = rotation.reshape(3, 3)
    pose[:3, 3] = position
    return pose


def pose_to_mocap(pose):
    quat = np.zeros(4)
    mj.mju_mat2Quat(quat, np.asarray(pose[:3, :3], dtype=float).ravel())
    return np.asarray(pose[:3, 3], dtype=float).copy(), quat


def leash_pose(target, current_tcp, pos_leash, lock_orientation):
    out = np.asarray(target, dtype=float).copy()
    dp = out[:3, 3] - current_tcp[:3, 3]
    dist = float(np.linalg.norm(dp))
    if pos_leash > 0 and dist > pos_leash:
        out[:3, 3] = current_tcp[:3, 3] + dp * (pos_leash / dist)
    if not lock_orientation:
        out[:3, :3] = current_tcp[:3, :3]
        return out
    rel = current_tcp[:3, :3].T @ out[:3, :3]
    cos_theta = np.clip(0.5 * (float(np.trace(rel)) - 1.0), -1.0, 1.0)
    angle = float(np.arccos(cos_theta))
    if angle > MAX_ROT_LEASH_RAD and angle > 1e-6:
        axis = np.array(
            [rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm > 1e-9:
            axis /= axis_norm
            k = np.array(
                [
                    [0.0, -axis[2], axis[1]],
                    [axis[2], 0.0, -axis[0]],
                    [-axis[1], axis[0], 0.0],
                ]
            )
            step = (
                np.eye(3)
                + np.sin(MAX_ROT_LEASH_RAD) * k
                + (1.0 - np.cos(MAX_ROT_LEASH_RAD)) * (k @ k)
            )
            out[:3, :3] = current_tcp[:3, :3] @ step
    return out


def rotation_error_rad(r_a, r_b):
    rel = r_a.T @ r_b
    cos_theta = np.clip(0.5 * (float(np.trace(rel)) - 1.0), -1.0, 1.0)
    return float(np.arccos(cos_theta))


def sync_display_from_state(model, data, handles, state):
    for i, h in enumerate(handles):
        data.qpos[h["aq"]] = state.measured[i * 6 : i * 6 + 6]
        data.qvel[h["av"]] = state.velocity[i * 6 : i * 6 + 6]
        hand = state.boards[i].hand
        if hand is not None:
            q_hand = np.asarray(hand.joint_q_rad, dtype=float)
            if q_hand.shape == h["hq"].shape and np.all(np.isfinite(q_hand)):
                data.qpos[h["hq"]] = q_hand
    mj.mj_forward(model, data)


def snap_mocap_from_tcp(data, handles, state, side_indices=(0, 1)):
    for i in side_indices:
        h = handles[i]
        pos, quat = pose_to_mocap(state.tcp[i])
        data.mocap_pos[h["target_id"]] = pos
        data.mocap_quat[h["target_id"]] = quat


def set_handle_color(model, handle, rgba):
    model.geom_rgba[handle["geom_id"]] = rgba
    if handle["site_id"] >= 0:
        model.site_rgba[handle["site_id"]] = rgba


def diagnose_status(state, bounds, pair_labels):
    c = sdk.collision_state(state.measured)
    levels = [0, 0]  # 0=normal, 1=yellow, 2=red
    reasons = []
    for i, name in enumerate(SIDE_NAMES):
        for j in range(6):
            idx = i * 6 + j
            lo = float(bounds.lower[idx])
            hi = float(bounds.upper[idx])
            span = max(hi - lo, 1e-6)
            q = float(state.measured[idx])
            margin_lo = (q - lo) / span
            margin_hi = (hi - q) / span
            if margin_lo <= 0.02:
                levels[i] = max(levels[i], 2)
                reasons.append(
                    f"{name}_j{j + 1} at lower ({np.rad2deg(q):.1f}°/{np.rad2deg(lo):.1f}°)"
                )
            elif margin_hi <= 0.02:
                levels[i] = max(levels[i], 2)
                reasons.append(
                    f"{name}_j{j + 1} at upper ({np.rad2deg(q):.1f}°/{np.rad2deg(hi):.1f}°)"
                )
            elif margin_lo <= 0.05:
                levels[i] = max(levels[i], 1)
                reasons.append(
                    f"{name}_j{j + 1} near lower ({np.rad2deg(q):.1f}°/{np.rad2deg(lo):.1f}°)"
                )
            elif margin_hi <= 0.05:
                levels[i] = max(levels[i], 1)
                reasons.append(
                    f"{name}_j{j + 1} near upper ({np.rad2deg(q):.1f}°/{np.rad2deg(hi):.1f}°)"
                )
    pair_name = (
        pair_labels[c.pair] if 0 <= c.pair < len(pair_labels) else f"pair#{c.pair}"
    )
    if c.inter_arm < 0.024:
        levels = [2, 2]
        reasons.append(f"inter_arm={c.inter_arm * 1000:.1f}mm ({pair_name})")
    elif c.inter_arm < 0.035:
        levels = [max(levels[0], 1), max(levels[1], 1)]
        reasons.append(f"inter_arm near {c.inter_arm * 1000:.1f}mm")
    if state.frozen or state.warning:
        levels = [2, 2]
        if state.warning:
            reasons.append(state.warning)
        elif state.frozen:
            reasons.append("controller frozen")
    return c, levels, reasons


def verify_gravity(plant):
    model = plant.model
    data = mj.MjData(model)
    maximum = 0.0
    for q in (
        [0] * 6,
        [0, 1, 1.3, -0.5, 0, 0],
        [0.2, 0.5, 1.0, -0.2, 0.3, 0.1],
        [-0.6, 1.2, 2.0, 0.4, -0.5, -1.0],
    ):
        for b in plant.boards:
            data.qpos[b.aq] = q
        mj.mj_forward(model, data)
        for side, b in zip(SIDES, plant.boards):
            delta = sdk.arm_geometry(side, q).gravity - data.qfrc_bias[b.av]
            maximum = max(maximum, float(np.max(np.abs(delta))))
    print(f"maximum static gravity error {maximum:.3g} Nm", flush=True)
    if maximum >= 0.001:
        raise RuntimeError("Static gravity check failed")


def raise_controller_stop(arm, exc=None):
    state = arm.state()
    c = sdk.collision_state(state.measured)
    reason = state.error or (str(exc) if exc else "Controller stopped")
    detail = (
        f"Controller stopped: {reason} "
        f"(inter_arm={c.inter_arm * 1000:.2f} mm, closest={c.closest * 1000:.2f} mm, "
        f"hard_margin={c.hard_margin * 1000:.2f} mm, pair={c.pair}, "
        f"frozen={state.frozen}, qp_failures={state.consecutive_failures}, "
        f"missed_cycles={state.missed_cycles})"
    )
    raise RuntimeError(detail) from exc


def set_pose_checked(arm, side, pose):
    try:
        arm.set_pose_target(side, pose)
    except Exception as exc:
        if not arm.state().enabled:
            raise_controller_stop(arm, exc)
        raise


def feed_targets(arm, plant):
    with plant.lock:
        positions = plant.data.mocap_pos.copy()
        quats = plant.data.mocap_quat.copy()
    for side, b in zip(SIDES, plant.boards):
        set_pose_checked(
            arm, side, mocap_to_pose(positions[b.target_id], quats[b.target_id])
        )
    state = check_active(arm)
    if plant.error:
        raise plant.error
    return state


def check_active(arm):
    state = arm.state()
    if state.error or not state.enabled:
        raise_controller_stop(arm)
    return state


def move_real_to_ready(arm, model, display, handles, viewer):
    for side in SIDES:
        arm.set_joint_target(side, READY)
    goal = np.tile(READY, 2)
    deadline = time.monotonic() + 30.0
    while viewer.is_running():
        state = arm.state()
        if not state.enabled:
            print(f"[WARN] Controller stopped while moving to ready: {state.error}", flush=True)
            return state
        with viewer.lock():
            sync_display_from_state(model, display, handles, state)
            snap_mocap_from_tcp(display, handles, state)
        viewer.sync()
        if np.max(np.abs(state.measured - goal)) <= READY_TOLERANCE_RAD:
            return state
        if time.monotonic() >= deadline:
            print("[WARN] Moving to ready pose timed out; continuing from current pose", flush=True)
            return state
        time.sleep(0.012)
    return None


def run_interactive(
    stack,
    arm,
    display_model,
    display,
    handles,
    config,
    args,
    plant=None,
    viewer=None,
    key_lock=None,
    key_events=None,
):
    import mujoco.viewer

    pair_labels = load_pair_labels()
    if key_lock is None or key_events is None:
        key_lock = threading.Lock()
        key_events = {"toggle_orientation": False, "snap_now": False, "re_enable": False}

        def on_key(keycode):
            if keycode in (ord("O"), ord("o")):
                with key_lock:
                    key_events["toggle_orientation"] = True
            elif keycode in (ord("S"), ord("s"), ord("R"), ord("r")):
                with key_lock:
                    key_events["snap_now"] = True
            elif keycode in (ord("E"), ord("e")):
                with key_lock:
                    key_events["re_enable"] = True
    else:
        on_key = None

    lock_orientation = bool(args.lock_orientation)
    state = arm.state()
    if state.enabled:
        arm.set_task_weights(WEIGHTS_6D if lock_orientation else WEIGHTS_POS)

    if plant is None:
        sync_display_from_state(display_model, display, handles, state)
        snap_mocap_from_tcp(display, handles, state)
    if viewer is None:
        viewer = stack.enter_context(
            mujoco.viewer.launch_passive(
                display_model,
                display,
                key_callback=on_key,
                show_left_ui=False,
                show_right_ui=False,
            )
        )
    if plant is None and args.move_ready:
        state = move_real_to_ready(arm, display_model, display, handles, viewer)
        if state is None:
            return

    mode_str = "6D pose" if lock_orientation else "3D position"
    print(
        f"[MODE] {mode_str} (symmetric dual-arm avoidance) | "
        f"press O in window to toggle 3D/6D, R or S to snap handles | "
        f"leash={args.leash * 100:.0f} cm, release-snap={args.snap_delay:.2f} s",
        flush=True,
    )

    mocap_ids = [h["target_id"] for h in handles]
    snaps = [ReleaseSnap(args.snap_delay), ReleaseSnap(args.snap_delay)]
    engaged = plant is not None
    ignore_drag = False
    last_overlay = None
    last_reason_line = ""
    next_print = 0.0

    while viewer.is_running():
        state = arm.state()
        if plant is not None and plant.error:
            print(f"[WARN] simulator error: {plant.error}", flush=True)
            plant.error = None
        c, levels, reasons = diagnose_status(state, config.joints, pair_labels)

        with key_lock:
            do_toggle = key_events["toggle_orientation"]
            do_snap = key_events["snap_now"]
            do_reenable = key_events.get("re_enable", False)
            key_events["toggle_orientation"] = False
            key_events["snap_now"] = False
            key_events["re_enable"] = False

        if do_reenable and not state.enabled:
            try:
                arm.enable(timeout_ms=30000 if args.real else 5000)
                arm.set_task_weights(WEIGHTS_6D if lock_orientation else WEIGHTS_POS)
                state = arm.state()
                do_snap = True
                engaged = plant is not None
                print("[OK] controller re-enabled", flush=True)
            except Exception as exc:
                print(f"[WARN] re-enable failed: {exc}", flush=True)

        if not state.enabled:
            levels = [2, 2]
            reasons = [f"STOPPED: {state.error or 'inactive'} (press E to re-enable)"]
            engaged = False

        if do_toggle and state.enabled:
            lock_orientation = not lock_orientation
            arm.set_task_weights(WEIGHTS_6D if lock_orientation else WEIGHTS_POS)
            snaps = [ReleaseSnap(args.snap_delay), ReleaseSnap(args.snap_delay)]
            with viewer.lock():
                ignore_drag = bool(viewer.perturb.active)
                for i, h in enumerate(handles):
                    _, quat = pose_to_mocap(state.tcp[i])
                    display.mocap_quat[h["target_id"]] = quat
            print(
                f"[MODE] {'6D pose LOCKED' if lock_orientation else '3D position (free wrist)'}",
                flush=True,
            )

        overlay_lines = [
            f"{'6D pose' if lock_orientation else '3D position'} (O: toggle, R/S: snap, E: re-enable)",
            f"inter_arm: {c.inter_arm * 1000:.1f} mm",
            *reasons,
        ]
        overlay_text = "\n".join(overlay_lines)
        overlay_changed = overlay_text != last_overlay

        with viewer.lock():
            if plant is not None:
                with plant.lock:
                    display.qpos[:] = plant.data.qpos
                    display.qvel[:] = plant.data.qvel
                    display.time = plant.data.time
                mj.mj_forward(display_model, display)
            else:
                sync_display_from_state(display_model, display, handles, state)

            if do_snap:
                snap_mocap_from_tcp(display, handles, state)
                snaps = [ReleaseSnap(args.snap_delay), ReleaseSnap(args.snap_delay)]
                ignore_drag = bool(viewer.perturb.active)
                print("[SNAP] handles synced to current TCP", flush=True)

            if not lock_orientation:
                for i, h in enumerate(handles):
                    _, quat = pose_to_mocap(state.tcp[i])
                    display.mocap_quat[h["target_id"]] = quat

            for i, h in enumerate(handles):
                color = (
                    RED_RGBA
                    if levels[i] >= 2
                    else YELLOW_RGBA
                    if levels[i] == 1
                    else DEFAULT_RGBA[i]
                )
                set_handle_color(display_model, h, color)

        if overlay_changed:
            viewer.set_texts((None, None, overlay_text, ""))
            last_overlay = overlay_text
        viewer.sync(state_only=not overlay_changed and not do_snap)

        now = time.monotonic()
        with viewer.lock():
            perturb = viewer.perturb
            dragged = -1
            if not perturb.active:
                ignore_drag = False
            if (
                not ignore_drag
                and perturb.active
                and perturb.select > 0
                and display_model.body_mocapid[perturb.select] >= 0
            ):
                mid = int(display_model.body_mocapid[perturb.select])
                if mid in mocap_ids:
                    dragged = mocap_ids.index(mid)
                    mj.mjv_applyPerturbPose(display_model, display, perturb, 0)
                    engaged = True

            for i, h in enumerate(handles):
                tid = h["target_id"]
                if args.leash > 0:
                    dp = display.mocap_pos[tid] - state.tcp[i][:3, 3]
                    dist = float(np.linalg.norm(dp))
                    if dist > args.leash:
                        display.mocap_pos[tid] = state.tcp[i][:3, 3] + dp * (
                            args.leash / dist
                        )
                if snaps[i].update(i == dragged, state.tcp[i], now):
                    pos, quat = pose_to_mocap(state.tcp[i])
                    display.mocap_pos[tid] = pos
                    display.mocap_quat[tid] = quat
                if not lock_orientation:
                    _, quat = pose_to_mocap(state.tcp[i])
                    display.mocap_quat[tid] = quat

            positions = display.mocap_pos.copy()
            quats = display.mocap_quat.copy()

        if engaged and state.enabled:
            for i, (side, h) in enumerate(zip(SIDES, handles)):
                tid = h["target_id"]
                raw_target = mocap_to_pose(positions[tid], quats[tid])
                cmd_pose = leash_pose(
                    raw_target, state.tcp[i], args.leash, lock_orientation
                )
                try:
                    arm.set_pose_target(side, cmd_pose)
                except Exception as exc:
                    print(f"[WARN] set_pose_target({SIDE_NAMES[i]}): {exc}", flush=True)

        reason_line = " | ".join(reasons)
        if reason_line != last_reason_line:
            if reason_line:
                print(f"[LIMIT/WARN] {reason_line}", flush=True)
            else:
                print("[OK] back in free workspace", flush=True)
            last_reason_line = reason_line

        if now >= next_print:
            tag = "6D" if lock_orientation else "POS"
            extra = f" | {reason_line}" if reason_line else ""
            print(
                f"[{tag}] inter_arm={c.inter_arm * 1000:.1f}mm closest={c.closest * 1000:.1f}mm "
                f"qp={state.qp_time * 1e6:.0f}us{extra}",
                flush=True,
            )
            next_print = now + 1.0
        time.sleep(0.012)


def headless(arm, plant, zero):
    if zero:
        # Escape the folded pose through the same joint target interface as WP-H.
        for s in SIDES:
            arm.set_joint_target(s, READY)
        end = time.monotonic() + 20
        while True:
            state = arm.state()
            if not state.enabled or time.monotonic() > end:
                raise RuntimeError(state.error or "Ready pose timed out")
            if np.max(np.abs(state.measured - np.tile(READY, 2))) < 0.004:
                break
            time.sleep(0.01)
        with plant.lock:
            for b in plant.boards:
                plant.data.mocap_pos[b.target_id] = plant.data.xpos[b.tcp]
                plant.data.mocap_quat[b.target_id] = plant.data.xquat[b.tcp]
    b = plant.boards[0]
    with plant.lock:
        plant.data.mocap_pos[b.target_id, 0] += 0.018
        goal = plant.data.mocap_pos[b.target_id].copy()
    for crossing in (False, True):
        if crossing:
            with plant.lock:
                for b, y in zip(plant.boards, (-0.005, 0.005)):
                    plant.data.mocap_pos[b.target_id, 1] = y
        deadline = time.monotonic() + 30
        stable = None
        minimum = float("inf")
        while time.monotonic() < deadline:
            state = feed_targets(arm, plant)
            c = sdk.collision_state(state.measured)
            minimum = min(minimum, c.inter_arm, plant.min_inter_arm)
            if minimum < 0.015 or plant.min_hard_margin <= 0:
                raise RuntimeError(f"Clearance violated: {minimum}, {c.hard_margin}")
            with plant.lock:
                err = np.linalg.norm(plant.data.xpos[plant.boards[0].tcp] - goal)
                speed = np.linalg.norm([plant.data.qvel[b.av] for b in plant.boards])
            reached = c.inter_arm < 0.04 if crossing else err < 0.005
            speed_limit = 0.10 if crossing else 0.02
            if reached and speed < speed_limit and not state.frozen:
                if stable is None:
                    stable = time.monotonic()
                if time.monotonic() - stable >= 0.3:
                    print(
                        f"inter_arm_min={minimum:.5f} m, hard_margin={c.hard_margin:.5f} m, "
                        f"speed={speed:.5f} rad/s, qp_time={state.qp_time * 1e6:.1f} us"
                        if crossing
                        else f"steady TCP error={err * 1000:.2f} mm",
                        flush=True,
                    )
                    break
            else:
                stable = None
            time.sleep(0.003)
        else:
            raise RuntimeError(
                f"Unsettled target: crossing={crossing}, distance={c.inter_arm}, "
                f"speed={speed}, TCP error={err}"
            )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument(
        "--real",
        action="store_true",
        help="connect to real left/right boards and use MuJoCo as live twin + mocap handle",
    )
    mode.add_argument(
        "--loopback",
        action="store_true",
        help="run against local MuJoCo sim (default)",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="run loopback verification without viewer",
    )
    p.add_argument(
        "--start-zero",
        action="store_true",
        help="start loopback simulation in folded zero pose",
    )
    p.add_argument(
        "--move-ready",
        action="store_true",
        help="in --real mode, move both arms to ready before starting mocap tracking",
    )
    p.add_argument(
        "--lock-orientation",
        action="store_true",
        help="start in 6D orientation-locked mode instead of Position-Only (toggle anytime with O)",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=0.25,
        help="Cartesian translation speed limit (m/s) in interactive mode (default: 0.25)",
    )
    p.add_argument(
        "--joint-speed",
        type=float,
        default=1.0,
        help="per-joint speed limit (rad/s) in interactive mode (default: 1.0)",
    )
    p.add_argument(
        "--pose-gain",
        type=float,
        default=6.0,
        help="TCP pose proportional gain in interactive mode (default: 6.0)",
    )
    p.add_argument(
        "--leash",
        type=float,
        default=0.15,
        help="max elastic leash distance (m) from current TCP while dragging (default: 0.15)",
    )
    p.add_argument(
        "--snap-delay",
        type=float,
        default=0.40,
        help="seconds after mouse release & arm settles before snapping handle back to TCP (0 disables)",
    )
    args = p.parse_args()
    if args.real and args.headless:
        p.error("--headless requires loopback simulation")
    if args.real and args.start_zero:
        p.error("--start-zero only applies to loopback simulation")
    if not args.real and args.move_ready:
        p.error("--move-ready only applies to --real mode")

    config = sdk.ControlConfig()
    config.translation_speed = args.speed if not args.headless else 0.05
    config.joints.max_speed = np.full(12, args.joint_speed if not args.headless else 0.25)
    config.joints.max_acceleration = np.full(12, 4.5 if not args.headless else 8.0)
    config.pose_gain = args.pose_gain if not args.headless else 2.0
    config.regularization = 0.003 if not args.headless else 0.025
    config.smoothing = 0.02 if not args.headless else 0.001
    config.posture_weight = 0.002 if not args.headless else 0.0001
    if not args.headless and not args.lock_orientation:
        config.task_weights = WEIGHTS_POS

    with ExitStack() as stack:
        if args.real:
            display_model = load_scene()
            handles = scene_handles(display_model)
            display = mj.MjData(display_model)
            arm = stack.enter_context(
                sdk.DualArm(sdk.BoardAddress.left(), sdk.BoardAddress.right(), config)
            )
            arm.enable(timeout_ms=30000)
            run_interactive(stack, arm, display_model, display, handles, config, args)
            return

        plant = stack.enter_context(
            Plant(args.start_zero, measure_clearance=args.headless)
        )
        if args.headless:
            verify_gravity(plant)
        viewer = None
        display_model = display = handles = key_lock = key_events = None
        if not args.headless:
            import mujoco.viewer

            # Launch the viewer before starting the Python Plant thread so OpenGL/GLFW
            # initialization cannot stall 500 Hz simulated telemetry under the GIL.
            display_model = copy.copy(plant.model)
            handles = scene_handles(display_model)
            display = mj.MjData(display_model)
            display.qpos[:] = plant.data.qpos
            display.mocap_pos[:] = plant.data.mocap_pos
            display.mocap_quat[:] = plant.data.mocap_quat
            key_lock = threading.Lock()
            key_events = {"toggle_orientation": False, "snap_now": False, "re_enable": False}

            def on_key(keycode):
                if keycode in (ord("O"), ord("o")):
                    with key_lock:
                        key_events["toggle_orientation"] = True
                elif keycode in (ord("S"), ord("s"), ord("R"), ord("r")):
                    with key_lock:
                        key_events["snap_now"] = True
                elif keycode in (ord("E"), ord("e")):
                    with key_lock:
                        key_events["re_enable"] = True

            viewer = stack.enter_context(
                mujoco.viewer.launch_passive(
                    display_model,
                    display,
                    key_callback=on_key,
                    show_left_ui=False,
                    show_right_ui=False,
                )
            )
        arm = stack.enter_context(sdk.DualArm(*plant.addresses(), config))
        plant.start([arm.local_ports(s) for s in SIDES])
        arm.enable(timeout_ms=5000)
        if args.headless:
            headless(arm, plant, args.start_zero)
            return
        run_interactive(
            stack,
            arm,
            display_model,
            display,
            handles,
            config,
            args,
            plant=plant,
            viewer=viewer,
            key_lock=key_lock,
            key_events=key_events,
        )


if __name__ == "__main__":
    main()
