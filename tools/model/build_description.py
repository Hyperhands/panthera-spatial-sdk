#!/usr/bin/env python3
"""Assemble the dual Panthera-HT + A2/L1 hand description from pristine sources.

Inputs (never modified):
  model/panthera_dual_description/sources/panthera_ht/PantheraHT.urdf
  model/panthera_dual_description/sources/a2l1_hand/urdf/{left,right}_hand.urdf
  model/panthera_dual_description/config/dual_arm.yaml

Outputs (deterministic, committed):
  urdf/panthera_dual_a2l1.urdf     both arms + hands under one `world` link
  urdf/panthera_left_a2l1.urdf     single arm+hand, root = left_base_link
  urdf/panthera_right_a2l1.urdf    single arm+hand, root = right_base_link

Mesh references use package://panthera_dual_description/... so that ROS tools
and Pinocchio (package_dirs=[<repo>/model]) resolve them identically.
Only names, mesh paths and the new mounting joints are introduced; every
source inertial/joint/limit/mimic value is copied verbatim.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

PKG = "panthera_dual_description"
ROOT = Path(__file__).resolve().parents[2]
PKG_DIR = ROOT / "model" / PKG
ARM_SRC = PKG_DIR / "sources" / "panthera_ht" / "PantheraHT.urdf"
HAND_SRC = PKG_DIR / "sources" / "a2l1_hand" / "urdf"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fmt(x: float) -> str:
    v = 0.0 if abs(x) < 1e-15 else float(x)
    return repr(v)  # shortest exact round-trip representation


def vec(xs) -> str:
    return " ".join(fmt(float(x)) for x in xs)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_matrix(rpy):
    r, p, y = rpy
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def matrix_to_rpy(R):
    """URDF convention R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:  # gimbal lock: put everything into yaw
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    out = (roll, pitch, yaw)
    assert np.allclose(rpy_to_matrix(out), R, atol=1e-12), "rpy round-trip failed"
    return out


def fixed_joint(name, parent, child, xyz, rpy):
    j = ET.Element("joint", name=name, type="fixed")
    ET.SubElement(j, "origin", xyz=vec(xyz), rpy=vec(rpy))
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    return j


def frame_link(name):
    return ET.Element("link", name=name)


def rewrite_meshes(elem, mapper):
    for mesh in elem.iter("mesh"):
        mesh.set("filename", mapper(mesh.get("filename")))


def arm_elements(side: str):
    """Prefixed copies of all Panthera links/joints, materials made unique."""
    src = ET.parse(ARM_SRC).getroot()
    rename = {}
    for link in src.findall("link"):
        rename[link.get("name")] = f"{side}_{link.get('name')}"

    def mesh_map(fn: str) -> str:
        assert fn.startswith("meshes/"), fn
        return f"package://{PKG}/meshes/panthera_ht/{fn[len('meshes/'):]}"

    out = []
    for e in src:
        if e.tag not in ("link", "joint"):
            continue
        e = copy.deepcopy(e)
        e.set("name", f"{side}_{e.get('name')}")
        if e.tag == "joint":
            e.find("parent").set("link", rename[e.find("parent").get("link")])
            e.find("child").set("link", rename[e.find("child").get("link")])
        else:
            for vis in e.findall("visual"):
                if vis.find("material") is None:
                    m = ET.SubElement(vis, "material", name="panthera_grey")
                    ET.SubElement(m, "color", rgba="0.55 0.57 0.60 1")
        rewrite_meshes(e, mesh_map)
        out.append(e)
    return out


def hand_elements(hand: str):
    """Copies of the A2/L1 hand (already side-prefixed), materials namespaced."""
    path = HAND_SRC / f"{hand}_hand.urdf"
    src = ET.parse(path).getroot()

    def mesh_map(fn: str) -> str:
        assert fn.startswith("../mesh/"), fn
        return f"package://{PKG}/meshes/a2l1_hand/{fn[len('../mesh/'):]}"

    out = []
    for e in src:
        if e.tag not in ("link", "joint"):
            continue
        e = copy.deepcopy(e)
        for mat in e.iter("material"):
            mat.set("name", f"a2l1_{mat.get('name')}")
        rewrite_meshes(e, mesh_map)
        out.append(e)
    root_links = {l.get("name") for l in src.findall("link")} - {
        j.find("child").get("link") for j in src.findall("joint")}
    assert root_links == {f"{hand}_palm_base_link"}, root_links
    return out, path


def side_chain(side: str, cfg: dict):
    """Arm + flange + hand + tcp for one side. Root link: {side}_base_link."""
    arm_cfg = cfg["arms"][side]
    hand = arm_cfg["hand"]
    elems = arm_elements(side)

    # flange_frame: z = tool axis (link6 +x), x = link6 -z  ->  Ry(+90 deg)
    R_flange = rot_y(math.pi / 2)
    assert np.allclose(R_flange @ [0, 0, 1], [1, 0, 0])
    flange_x = float(cfg["flange"]["offset_along_link6_x_m"])
    elems.append(frame_link(f"{side}_flange_frame"))
    elems.append(fixed_joint(f"{side}_flange_joint", f"{side}_link6", f"{side}_flange_frame",
                             [flange_x, 0, 0], matrix_to_rpy(R_flange)))

    # hand_mount_frame: roll about tool axis + adapter thickness along tool axis
    roll = math.radians(float(arm_cfg["hand_roll_deg"]))
    adapter = float(cfg["hand_mount"]["adapter_thickness_m"])
    elems.append(frame_link(f"{side}_hand_mount_frame"))
    elems.append(fixed_joint(f"{side}_hand_mount_joint", f"{side}_flange_frame",
                             f"{side}_hand_mount_frame", [0, 0, adapter], [0, 0, roll]))

    hand_elems, hand_path = hand_elements(hand)
    elems.append(fixed_joint(f"{side}_hand_joint", f"{side}_hand_mount_frame",
                             f"{hand}_palm_base_link", cfg["hand_mount"]["palm_offset_xyz_m"], [0, 0, 0]))
    elems.extend(hand_elems)

    tcp = cfg["tcp"]
    elems.append(frame_link(f"{side}_tcp"))
    elems.append(fixed_joint(f"{side}_tcp_joint", f"{hand}_palm_base_link", f"{side}_tcp",
                             tcp["xyz_m"], [math.radians(a) for a in tcp["rpy_deg"]]))
    return elems, hand_path


def make_robot(name: str, comment: str, body):
    robot = ET.Element("robot", name=name)
    robot.append(ET.Comment(comment))
    for e in body:
        robot.append(e)
    ET.indent(robot, space="  ")
    return robot


def write(robot: ET.Element, path: Path):
    data = ET.tostring(robot, encoding="unicode", xml_declaration=False)
    path.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + data + "\n", encoding="utf-8", newline="\n")


def validate_cfg(cfg):
    l, r = cfg["arms"]["left"], cfg["arms"]["right"]
    spacing = float(cfg["arms"]["base_spacing_m"])
    d = np.linalg.norm(np.subtract(l["base_xyz_m"], r["base_xyz_m"]))
    if abs(d - spacing) > 1e-9:
        sys.exit(f"config: |left.base - right.base| = {d} m != base_spacing_m = {spacing} m")
    if {l["hand"], r["hand"]} != {"left", "right"}:
        sys.exit("config: each arm needs exactly one of hand: left/right")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=PKG_DIR / "config" / "dual_arm.yaml")
    ap.add_argument("--out", type=Path, default=PKG_DIR / "urdf")
    args = ap.parse_args()
    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    validate_cfg(cfg)
    args.out.mkdir(parents=True, exist_ok=True)

    provenance = (f" GENERATED by tools/model/build_description.py - DO NOT EDIT. "
                  f"config sha256={sha256(args.config)[:16]} "
                  f"arm={ARM_SRC.name} sha256={sha256(ARM_SRC)[:16]} ")

    world_body = [frame_link(cfg["world"]["frame"])]
    for side in ("left", "right"):
        chain, hand_path = side_chain(side, cfg)
        a = cfg["arms"][side]
        world_body.append(fixed_joint(f"{side}_base_mount_joint", cfg["world"]["frame"], f"{side}_base_link",
                                      a["base_xyz_m"], [math.radians(x) for x in a["base_rpy_deg"]]))
        world_body.extend(chain)
        single = make_robot(f"panthera_{side}_a2l1",
                            provenance + f"hand={hand_path.name} sha256={sha256(hand_path)[:16]} ", chain)
        write(single, args.out / f"panthera_{side}_a2l1.urdf")

    hands = " ".join(f"{h}_hand.urdf sha256={sha256(HAND_SRC / f'{h}_hand.urdf')[:16]}" for h in ("left", "right"))
    dual = make_robot("panthera_dual_a2l1", provenance + hands + " ", world_body)
    write(dual, args.out / "panthera_dual_a2l1.urdf")
    for f in sorted(args.out.glob("*.urdf")):
        print(f"wrote {f.relative_to(ROOT)}  sha256={sha256(f)[:16]}")


if __name__ == "__main__":
    main()
