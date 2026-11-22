"""Move both arm TCPs forward along +x and return to the starting pose."""

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


def wait_pose(arm, targets, timeout_s=10.0, pos_tol_m=0.004):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = arm.state()
        if not state.enabled:
            print(f"[WARN] controller stopped: {state.error}", flush=True)
            return state
        errs = [float(np.linalg.norm(p[:3, 3] - g[:3, 3])) for p, g in zip(state.tcp, targets)]
        if max(errs) <= pos_tol_m:
            return state
        time.sleep(0.01)
    print("[WARN] pose target timed out, continuing", flush=True)
    return arm.state()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="connect to real boards")
    mode.add_argument("--loopback", action="store_true", help="run against local UDP simulator (default)")
    p.add_argument("--distance", type=float, default=0.03, help="forward displacement in meters (default: 0.03)")
    args = p.parse_args()

    config = sdk.ControlConfig()
    config.translation_speed = 0.08

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

        initial = [pose.copy() for pose in arm.state().tcp]
        forward = [pose.copy() for pose in initial]
        for pose in forward:
            pose[0, 3] += args.distance

        for label, targets in (("forward", forward), ("return", initial)):
            for side, target in zip(SIDES, targets):
                arm.set_pose_target(side, target)
            state = wait_pose(arm, targets)
            errs_mm = [np.linalg.norm(p[:3, 3] - g[:3, 3]) * 1000.0 for p, g in zip(state.tcp, targets)]
            print(
                f"[{label}] tcp_err=({errs_mm[0]:.2f}, {errs_mm[1]:.2f}) mm "
                f"qp={state.qp_time * 1e6:.1f} us",
                flush=True,
            )


if __name__ == "__main__":
    main()
