"""Send direct 30-float MIT commands in expert mode (bypasses QP, collision, and limits)."""

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
    p.add_argument(
        "--i-understand-no-safety",
        action="store_true",
        help="required acknowledgement when using --real in expert mode",
    )
    args = p.parse_args()

    if args.real and not args.i_understand_no_safety:
        p.error("Real expert mode requires --i-understand-no-safety")

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

        state = arm.state()
        commands = []
        for i, side in enumerate(SIDES):
            raw = np.zeros((6, 5))
            raw[:, 0] = state.boards[i].arm.q_rad
            raw[:, 2] = [24.48, 90.0, 90.0, 36.72, 12.24, 12.24]
            raw[:, 3] = [1.2, 2.5, 2.5, 0.8314, 0.48, 0.48]
            raw[:, 4] = sdk.arm_geometry(side, state.measured[i * 6 : i * 6 + 6]).gravity
            commands.append(raw.ravel())

        arm.enter_expert_mode(sdk.ExpertAck.IUnderstandNoSafety)
        for _ in range(25):
            for side, cmd in zip(SIDES, commands):
                arm.send_mit(side, cmd)
            time.sleep(0.002)
        print("[expert_mit] sent 25 MIT hold frames per arm", flush=True)


if __name__ == "__main__":
    main()
