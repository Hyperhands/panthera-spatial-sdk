"""Generate FK, world-aligned geometric Jacobian, gravity, and collision frame placements."""

from pathlib import Path
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "model/panthera_dual_description"
OUT = ROOT / "generated/model"
SIDES = ("left", "right")
FRAMES = ["base_link", *(f"link{i}" for i in range(1, 7)), "palm_base_link", "tcp"]


def vector(element, key):
    return np.fromstring(element.get(key, "0 0 0"), sep=" ")


def origin(element):
    t = np.eye(4)
    e = element.find("origin")
    if e is not None:
        t[:3, 3] = vector(e, "xyz")
        t[:3, :3] = pin.rpy.rpyToMatrix(vector(e, "rpy"))
    return t


def generate(side):
    path = PKG / "urdf" / f"panthera_{side}_a2l1.urdf"
    tree = ET.parse(path)
    mount = next(
        j
        for j in ET.parse(PKG / "urdf/panthera_dual_a2l1.urdf").iter("joint")
        if j.find("child").get("link") == side + "_base_link"
    )
    base = origin(mount)
    transforms = {side + "_base_link": ca.DM(base)}
    q = ca.SX.sym("q", 6)
    axes = []
    for j in tree.iter("joint"):
        parent = j.find("parent").get("link")
        t = transforms[parent] @ origin(j)
        if j.get("name") in [f"{side}_joint{i}" for i in range(1, 7)]:
            axis = ca.DM(vector(j.find("axis"), "xyz"))
            axes.append(t[:3, :3] @ axis)
            k = ca.skew(axis)
            angle = q[len(axes) - 1]
            rot = ca.SX.eye(4)
            rot[:3, :3] += ca.sin(angle) * k + (1 - ca.cos(angle)) * (k @ k)
            t = t @ rot
        transforms[j.find("child").get("link")] = t
    energy = 0
    for link in tree.iter("link"):
        inertial = link.find("inertial")
        if inertial is not None:
            com = transforms[link.get("name")] @ origin(inertial)
            energy += 9.81 * float(inertial.find("mass").get("value")) * com[2, 3]
    placements = [transforms[side + "_" + name][:3, :] for name in FRAMES]
    jac = ca.vertcat(ca.jacobian(placements[-1][:, 3], q), ca.horzcat(*axes))
    f = ca.Function(
        "panthera_" + side,
        [q],
        list(map(ca.densify, [ca.horzcat(*placements), jac, ca.gradient(energy, q)])),
    )
    f.generate(f.name() + ".c", {"with_header": True})
    # Load the compiled artifact and compare to independent URDF rigid-body algorithms.
    ref = pin.buildModelFromUrdf(str(path))
    data = ref.createData()
    idx = [ref.joints[ref.getJointId(f"{side}_joint{i}")].idx_q for i in range(1, 7)]
    with tempfile.TemporaryDirectory() as tmp:
        lib = str(Path(tmp) / "model.so")
        subprocess.run(
            ["cc", "-shared", "-fPIC", "-O2", f.name() + ".c", "-lm", "-o", lib], check=True
        )
        evaluate = ca.external(f.name(), lib)
        maximum = 0.0
        for arm in np.random.default_rng(812).uniform(-2, 2, (128, 6)):
            full = pin.neutral(ref)
            full[idx] = arm
            pin.computeJointJacobians(ref, data, full)
            pin.updateFramePlacements(ref, data)
            frames, jac, g = map(np.asarray, evaluate(arm))
            for i, name in enumerate(FRAMES):
                se3 = data.oMf[ref.getFrameId(side + "_" + name)]
                maximum = max(
                    maximum,
                    np.max(abs(frames[:, 4 * i : 4 * i + 4] - (base @ se3.homogeneous)[:3])),
                )
            j = pin.getFrameJacobian(
                ref, data, ref.getFrameId(side + "_tcp"), pin.LOCAL_WORLD_ALIGNED
            )[:, idx]
            j = np.vstack([base[:3, :3] @ j[:3], base[:3, :3] @ j[3:]])
            maximum = max(
                maximum,
                np.max(abs(j - jac)),
                np.max(abs(pin.computeGeneralizedGravity(ref, data, full)[idx] - g.ravel())),
            )
        print(f"{side}: maximum FK/Jacobian/gravity discrepancy {maximum:.3g}")
        if maximum >= 1e-9:
            raise ValueError("Generated model differs from Pinocchio")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True, parents=True)
    os.chdir(OUT)
    for side in SIDES:
        generate(side)
