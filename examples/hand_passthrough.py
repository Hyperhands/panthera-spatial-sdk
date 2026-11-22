"""Send 6-DOF active hand joint targets and read back live finger angles."""

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


def send_and_print(arm, q_target, duration_ms=600):
    for side in SIDES:
        arm.set_hand_target(side, q_target, duration_ms)
    time.sleep(duration_ms / 1000.0 + 0.15)
    state = arm.state()
    for i, name in enumerate(("left", "right")):
        angles = state.boards[i].active_hand_angles(100_000)
        val = np.round(angles, 3).tolist() if angles is not None else "unavailable"
        print(f"[{name}] target={np.round(q_target, 3).tolist()} measured={val}", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="connect to real boards")
    mode.add_argument("--loopback", action="store_true", help="run against local UDP simulator (default)")
    args = p.parse_args()

    with ExitStack() as stack:
        pair = None
        if args.real:
            addresses = [sdk.BoardAddress.left(), sdk.BoardAddress.right()]
        else:
            pair = stack.enter_context(BoardPair(ready=True))
            addresses = pair.addresses()

        arm = stack.enter_context(sdk.DualArm(*addresses))
        if pair is not None:
            pair.start([arm.local_ports(s) for s in SIDES])
        arm.enable(timeout_ms=30000 if args.real else 5000)

        send_and_print(arm, np.full(6, 0.15))
        send_and_print(arm, np.zeros(6))


if __name__ == "__main__":
    main()
