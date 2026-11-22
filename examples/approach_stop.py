"""Command both arms across y=0 to demonstrate QP self-collision avoidance (~20 mm stop)."""

import argparse
from contextlib import ExitStack
from pathlib import Path
import sys
import time

import numpy as np
import panthera_sdk as sdk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from loopback import BoardPair

SIDES = (sdk.Side.left, sdk.Side.right)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="connect to real boards")
    mode.add_argument("--loopback", action="store_true", help="run against local UDP simulator (default)")
    p.add_argument("--timeout", type=float, default=15.0, help="max wait time in seconds (default: 15.0)")
    args = p.parse_args()

    config = sdk.ControlConfig()
    config.translation_speed = 0.08
    config.pose_gain = 2.0
    config.regularization = 0.025
    config.posture_weight = 0.0001

    with ExitStack() as stack:
        pair = None
        if args.real:
            addresses = [sdk.BoardAddress.left(), sdk.BoardAddress.right()]
        else:
            pair = stack.enter_context(BoardPair(ready=True))
            addresses = pair.addresses()

        arm = stack.enter_context(sdk.DualArm(*addresses, config))
        if pair is not None:
            pair.start([arm.local_ports(s) for s in SIDES])
        arm.enable(timeout_ms=30000 if args.real else 5000)

        state = arm.state()
        for i, side in enumerate(SIDES):
            target = state.tcp[i].copy()
            target[1, 3] = -0.005 if i == 0 else 0.005
            arm.set_pose_target(side, target)

        min_dist = float("inf")
        stable_since = None
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            state = arm.state()
            if not state.enabled:
                print(f"[WARN] controller stopped: {state.error}", flush=True)
                break
            c = sdk.collision_state(state.measured)
            min_dist = min(min_dist, c.inter_arm)
            dq = np.concatenate([b.arm.dq_rad_s for b in state.boards])
            if c.inter_arm < 0.04 and float(np.linalg.norm(dq)) < 0.02 and not state.frozen:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.2:
                    break
            else:
                stable_since = None
            time.sleep(0.01)

        print(
            f"[approach_stop] min_inter_arm={min_dist * 1000:.2f} mm "
            f"final_inter_arm={sdk.collision_state(arm.state().measured).inter_arm * 1000:.2f} mm",
            flush=True,
        )


if __name__ == "__main__":
    main()
