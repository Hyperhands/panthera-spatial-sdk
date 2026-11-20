"""Drag the two mocap targets, or exercise the same UDP control path headlessly."""

import argparse
import copy
from contextlib import ExitStack
from pathlib import Path
import sys
import time

import mujoco as mj
import numpy as np
import panthera_sdk as sdk

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from mujoco_board import Plant

SIDES = (sdk.Side.left, sdk.Side.right)


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


def feed_targets(arm, plant):
    with plant.lock:
        positions = plant.data.mocap_pos.copy()
        quats = plant.data.mocap_quat.copy()
    for side, b in zip(SIDES, plant.boards):
        pose = np.eye(4)
        rotation = np.zeros(9)
        mj.mju_quat2Mat(rotation, quats[b.target_id])
        pose[:3, :3] = rotation.reshape(3, 3)
        pose[:3, 3] = positions[b.target_id]
        arm.set_pose_target(side, pose)
    state = arm.state()
    if state.error or not state.enabled:
        raise RuntimeError(state.error or "Controller stopped")
    if plant.error:
        raise plant.error
    return state


def headless(arm, plant, zero):
    if zero:
        # Escape the folded pose through the same joint target interface as WP-H.
        for s in SIDES:
            arm.set_joint_target(s, [0, 1, 1.3, -0.5, 0, 0])
        end = time.monotonic() + 20
        while True:
            state = arm.state()
            if not state.enabled or time.monotonic() > end:
                raise RuntimeError(state.error or "Ready pose timed out")
            if (
                np.max(np.abs(state.measured - np.tile([0, 1, 1.3, -0.5, 0, 0], 2)))
                < 0.004
            ):
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
            if minimum < 0.019 or plant.min_hard_margin <= 0:
                raise RuntimeError(f"Clearance violated: {minimum}, {c.hard_margin}")
            with plant.lock:
                err = np.linalg.norm(plant.data.xpos[plant.boards[0].tcp] - goal)
                speed = np.linalg.norm([plant.data.qvel[b.av] for b in plant.boards])
            reached = c.inter_arm < 0.04 if crossing else err < 0.005
            if reached and speed < 0.02 and not state.frozen:
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
    p.add_argument("--headless", action="store_true")
    p.add_argument("--start-zero", action="store_true")
    args = p.parse_args()
    with ExitStack() as stack:
        plant = stack.enter_context(
            Plant(args.start_zero, measure_clearance=args.headless)
        )
        verify_gravity(plant)
        viewer = None
        if not args.headless:
            import mujoco.viewer

            display_model = copy.copy(plant.model)
            display = mj.MjData(display_model)
            display.qpos[:] = plant.data.qpos
            display.mocap_pos[:] = plant.data.mocap_pos
            display.mocap_quat[:] = plant.data.mocap_quat
            viewer = stack.enter_context(
                mujoco.viewer.launch_passive(
                    display_model, display, show_left_ui=False, show_right_ui=False
                )
            )
        config = sdk.ControlConfig()
        config.translation_speed = 0.05
        config.joints.max_speed = np.full(12, 0.25)
        config.pose_gain, config.regularization, config.posture_weight = (
            2.0,
            0.025,
            0.0001,
        )
        arm = stack.enter_context(sdk.DualArm(*plant.addresses(), config))
        plant.start([arm.local_ports(s) for s in SIDES])
        arm.enable(timeout_ms=5000)
        if viewer is None:
            headless(arm, plant, args.start_zero)
            return
        next_print = 0.0
        while viewer.is_running():
            with plant.lock:
                display.qpos[:] = plant.data.qpos
                display.qvel[:] = plant.data.qvel
                display.time = plant.data.time
            mj.mj_forward(display_model, display)
            viewer.sync()
            with plant.lock:
                plant.data.mocap_pos[:] = display.mocap_pos
                plant.data.mocap_quat[:] = display.mocap_quat
            state = feed_targets(arm, plant)
            if time.monotonic() >= next_print:
                c = sdk.collision_state(state.measured)
                print(
                    f"inter_arm={c.inter_arm:.4f} closest={c.closest:.4f} "
                    f"hard_margin={c.hard_margin:.4f} m qp_time={state.qp_time * 1e6:.1f} us",
                    flush=True,
                )
                next_print = time.monotonic() + 1
            time.sleep(0.012)


if __name__ == "__main__":
    main()
