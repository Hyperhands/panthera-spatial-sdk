"""Read live 6-axis arm and hand telemetry from both boards without taking control."""

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


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="read from real boards (192.168.10.11/.12)")
    mode.add_argument("--loopback", action="store_true", help="run against local UDP simulator (default)")
    p.add_argument("--duration", type=float, default=1.0, help="sampling duration in seconds (default: 1.0)")
    args = p.parse_args()

    with ExitStack() as stack:
        pair = None
        if args.real:
            addresses = [sdk.BoardAddress.left(), sdk.BoardAddress.right()]
        else:
            pair = stack.enter_context(BoardPair(ready=True))
            addresses = pair.addresses()

        rx = [sdk.BoardTelemetry(a.realtime) for a in addresses]
        if pair is not None:
            pair.start([(r.local_port(), 0) for r in rx])

        start = time.monotonic()
        last = [None, None]
        while time.monotonic() - start < max(0.2, args.duration):
            for i, r in enumerate(rx):
                last[i] = r.poll()
            time.sleep(0.002)

        for name, state in zip(("left", "right"), last):
            if state is None or state.arm is None:
                print(f"[{name}] no telemetry received", flush=True)
                continue
            q = np.asarray(state.arm.q_rad, dtype=float)
            hand = state.active_hand_angles(100_000)
            hand_str = f" hand={np.round(hand, 3).tolist()}" if hand is not None else ""
            print(
                f"[{name}] rx={state.arm.received} missing={state.arm.missing} "
                f"owner={state.arm.control_owner} q_rad={np.round(q, 3).tolist()}{hand_str}",
                flush=True,
            )


if __name__ == "__main__":
    main()
