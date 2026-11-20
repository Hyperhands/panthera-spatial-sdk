"""One MuJoCo plant shared by two Wirelink session endpoints."""

from pathlib import Path
import struct
import threading
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from loopback import BoardPair
from session import SessionBoard
from robot_protocol import decode
import panthera_sdk as sdk

SCENE = Path(__file__).with_name("assets") / "panthera_dual_mocap.xml"
HAND = (
    "thumb_cmc",
    "thumb_pip",
    "thumb_dip",
    "index_mcp",
    "index_pip",
    "middle_mcp",
    "middle_pip",
    "ring_mcp",
    "ring_pip",
    "little_mcp",
    "little_pip",
)
ACTIVE = [0, 1, 3, 5, 7, 9]


def load_scene():
    # link2 has 370292 triangles. The STL decoder caps files at 200000.
    # Give MuJoCo the same triangles as an in-memory binary MSH asset.
    scene = ET.parse(SCENE)
    virtual = {}
    for mesh in scene.findall("asset/mesh"):
        path = (SCENE.parent / mesh.get("file")).resolve()
        mesh.set("file", str(path))
        if path.name == "link2.STL":
            raw = path.read_bytes()
            (count,) = struct.unpack_from("<I", raw, 80)
            records = np.ndarray(
                (count, 3, 3), dtype="<f4", buffer=raw, offset=96, strides=(50, 12, 4)
            )
            vertices, indices = np.unique(
                records.reshape(-1, 3), axis=0, return_inverse=True
            )
            key = "panthera_link2.msh"
            virtual[key] = (
                struct.pack("<4i", len(vertices), 0, 0, count)
                + vertices.astype("<f4").tobytes()
                + indices.astype("<i4").tobytes()
            )
            mesh.set("file", key)
    return mj.MjModel.from_xml_string(
        ET.tostring(scene.getroot(), encoding="unicode"), virtual
    )


class ArmBoard(SessionBoard):
    def __init__(self, node, side, model):
        super().__init__(node)
        self.arm_names = [f"{side}_joint{i}" for i in range(1, 7)]
        hand_names = [f"{side}_{name}_joint" for name in HAND]
        self.aq, self.av = indices(model, self.arm_names)
        self.hq, self.hv = indices(model, hand_names)
        self.motor = [model.actuator(j).id for j in self.arm_names]
        self.servo = [model.actuator(hand_names[i]).id for i in ACTIVE]
        self.hand_target = np.zeros(6)
        self.hand_start = self.hand_target.copy()
        self.hand_at = self.hand_duration = 0.0
        self.target_id = model.body(side + "_target").mocapid[0]
        self.tcp = model.body(side + "_tcp").id

    def realtime(self, mid, payload):
        before = self.hand_seq
        super().realtime(mid, payload)
        if self.hand_seq != before:
            self.hand_target = np.asarray(self.hand["joint_q_rad"])[ACTIVE].copy()
            self.hand_duration = decode("HandCommand", payload)["duration_ms"] / 1000.0
            self.hand_at = time.monotonic()
            self.hand_start = self.actual_hand.copy()

    def apply(self, data):
        q, dq = data.qpos[self.aq], data.qvel[self.av]
        if self.active and self.mit is not None:
            ref, vel, kp, kd, ff = np.asarray(self.mit).reshape(6, 5).T
            tau = kp * (ref - q) + kd * (vel - dq) + ff
        else:
            # Model board-owned gravity and damping during takeover and soft hold.
            tau = data.qfrc_bias[self.av] - 2 * dq
        data.ctrl[self.motor] = tau
        t = min(1.0, (time.monotonic() - self.hand_at) / max(self.hand_duration, 1e-9))
        data.ctrl[self.servo] = self.hand_start + t * (
            self.hand_target - self.hand_start
        )

    def feedback(self, data):
        self.arm.update(
            q_rad=data.qpos[self.aq].tolist(),
            dq_rad_s=data.qvel[self.av].tolist(),
            tau_measured_nm=data.actuator_force[self.motor].tolist(),
        )
        self.hand["joint_q_rad"] = data.qpos[self.hq].tolist()
        self.actual_hand = data.qpos[self.hq[ACTIVE]].copy()


def indices(model, names):
    return tuple(
        np.array([getattr(model.joint(j), key)[0] for j in names])
        for key in ("qposadr", "dofadr")
    )


class Plant(BoardPair):
    def __init__(self, zero=False, measure_clearance=False):
        self.measure_clearance = measure_clearance
        self.min_inter_arm = self.min_hard_margin = float("inf")
        self.model = load_scene()
        self.data = mj.MjData(self.model)
        self.boards = [
            ArmBoard(i, name, self.model) for i, name in ((11, "left"), (12, "right"))
        ]
        for b in self.boards:
            self.data.qpos[b.aq] = np.zeros(6) if zero else [0, 1, 1.3, -0.5, 0, 0]
        self.lock = threading.Lock()
        self.end = threading.Event()
        self.thread = self.error = None
        mj.mj_forward(self.model, self.data)
        for b in self.boards:
            self.data.mocap_pos[b.target_id] = self.data.xpos[b.tcp]
            self.data.mocap_quat[b.target_id] = self.data.xquat[b.tcp]
            b.feedback(self.data)
        mj.mj_forward(self.model, self.data)

    def start(self, ports):
        def run():
            deadline = time.monotonic()
            cycle = 0
            try:
                while not self.end.is_set():
                    for b, (rt, mgmt) in zip(self.boards, ports):
                        b.service(rt, mgmt)
                    with self.lock:
                        for _ in range(round(0.002 / self.model.opt.timestep)):
                            for b in self.boards:
                                b.apply(self.data)
                            mj.mj_step(self.model, self.data)
                        for b in self.boards:
                            b.feedback(self.data)
                        if self.measure_clearance:
                            q = np.concatenate(
                                [self.data.qpos[b.aq] for b in self.boards]
                            )
                            c = sdk.collision_state(q)
                            self.min_inter_arm = min(self.min_inter_arm, c.inter_arm)
                            self.min_hard_margin = min(
                                self.min_hard_margin, c.hard_margin
                            )
                    for b, (rt, _) in zip(self.boards, ports):
                        b.tick(rt, cycle)
                    cycle += 1
                    deadline += 0.002
                    now = time.monotonic()
                    if deadline < now:
                        deadline += (1 + int((now - deadline) / 0.002)) * 0.002
                    time.sleep(max(0, deadline - now))
            except Exception as exc:
                self.error = exc

        self.thread = threading.Thread(target=run, name="mujoco-boards")
        self.thread.start()
