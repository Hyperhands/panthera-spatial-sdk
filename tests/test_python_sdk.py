"""Installed-wheel integration tests, including rollback of a partial claim."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import unittest

import numpy as np
import panthera_sdk as sdk

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from loopback import BoardPair

SIDES = (sdk.Side.left, sdk.Side.right)


def interrupt_partial_claim():
    with BoardPair() as pair:
        pair.boards[1].homing_until = float("inf")
        with sdk.DualArm(*pair.addresses()) as robot:
            pair.start([robot.local_ports(s) for s in SIDES])
            sent = []
            def interrupt():
                until = time.monotonic() + 4
                while not pair.boards[0].lease and time.monotonic() < until:
                    time.sleep(.001)
                if pair.boards[0].lease:
                    sent.append(time.monotonic())
                    if os.name == "nt":
                        signal.raise_signal(signal.SIGINT)
                    else:
                        os.kill(os.getpid(), signal.SIGINT)
            worker = threading.Thread(target=interrupt)
            worker.start()
            try:
                robot.enable(timeout_ms=5000)
            except KeyboardInterrupt:
                assert sent and time.monotonic() - sent[0] < 1
            else:
                raise AssertionError("Takeover ignored SIGINT")
            finally:
                worker.join()
        assert 2 in pair.boards[0].operations
        assert all(not b.lease and not b.active for b in pair.boards)


class WheelTests(unittest.TestCase):
    def test_interrupt(self):
        subprocess.run([sys.executable, __file__, "partial-claim"], check=True, timeout=12)

    def test_telemetry(self):
        with BoardPair() as pair:
            readers = [sdk.BoardTelemetry(a.realtime) for a in pair.addresses()]
            pair.start([(r.local_port(), 0) for r in readers])
            until = time.monotonic() + 2
            while time.monotonic() < until:
                first = readers[0].poll()
                if first.arm:
                    break
            self.assertIsNotNone(first.arm)
            count = first.arm.received
            time.sleep(.012)
            self.assertGreater(readers[0].poll().arm.received, count)
            self.assertEqual(first.arm.received, count)
            self.assertTrue(np.isfinite(first.arm.received_at))
            self.assertFalse(pair.boards[0].operations)

    def test_motion(self):
        with BoardPair(ready=True) as pair, sdk.DualArm(*pair.addresses()) as arm:
            pair.start([arm.local_ports(s) for s in SIDES])
            arm.enable(timeout_ms=3000)
            state = arm.state()
            self.assertTrue(state.enabled)
            self.assertGreater(sdk.collision_state(state.measured).inter_arm, .15)
            side = SIDES[0]
            arm.set_pose_target(side, state.tcp[0])
            arm.set_twist_target(side, np.zeros(6))
            arm.set_joint_target(side, state.measured[:6])
            arm.set_hand_target(side, np.full(6, .15), 100)
            until = time.monotonic() + 2
            while time.monotonic() < until:
                hand = arm.state().boards[0].active_hand_angles()
                if hand is not None and np.allclose(hand,.15):
                    break
                time.sleep(.004)
            np.testing.assert_allclose(hand, .15)
            for pose in (np.eye(3), np.zeros((4,4)), np.diag([-1,1,1,1])):
                with self.assertRaises(ValueError):
                    arm.set_pose_target(side,pose)
            for q in (np.zeros(7), np.full(6,np.nan)):
                with self.assertRaises(ValueError):
                    arm.set_joint_target(side,q)
            arm.enter_expert_mode(sdk.ExpertAck.IUnderstandNoSafety)
            self.assertTrue(arm.state().expert)
            arm.soft_stop()
            self.assertFalse(arm.state().enabled)
            self.assertTrue(all(18 in b.operations for b in pair.boards))


if __name__ == "__main__":
    if sys.argv[1:] == ["partial-claim"]:
        interrupt_partial_claim()
    else:
        unittest.main()
