#!/usr/bin/env python3
"""Verify the generated dual description and derive its SRDF collision matrix.

Run after build_description.py:
    python3 tools/model/verify_description.py [--samples 6000] [--no-preview]

Writes:
  srdf/panthera_dual_a2l1.srdf          groups, end effectors, disabled pairs
  verification/verification.json        all numeric checks and pair statistics
  verification/preview.png              collision meshes at zero and ready pose

Disabled-pair policy (deliberately conservative for a dual-arm safety check):
  Adjacent      links joined by one movable joint (fixed chains merged)
  HandInternal  both links inside the same hand (fingers are not controlled here)
  Static        both links rigidly fixed to world (the two base_links)
  Default       intra-side pair already in contact at the zero pose
  Never         arm-link vs own-arm-link pair never in contact in N random samples
                (hand vs own arm is never sampled away)
Cross-arm pairs are NEVER disabled except Static.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import pinocchio as pin

PKG = "panthera_dual_description"
ROOT = Path(__file__).resolve().parents[2]
PKG_DIR = ROOT / "model" / PKG
URDF = PKG_DIR / "urdf" / "panthera_dual_a2l1.urdf"
SINGLE = {s: PKG_DIR / "urdf" / f"panthera_{s}_a2l1.urdf" for s in ("left", "right")}
SIDES = ("left", "right")
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
READY = [0.0, 1.0, 1.3, -0.5, 0.0, 0.0]  # visual check pose (rad): arm raised, tool roughly forward


# ----------------------------------------------------------------- URDF graph
class Tree:
    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.parent, self.jtype, self.mimic = {}, {}, set()
        for j in root.findall("joint"):
            c, p = j.find("child").get("link"), j.find("parent").get("link")
            self.parent[c] = (p, j.get("type"), j.get("name"))
            if j.find("mimic") is not None:
                self.mimic.add(j.get("name"))
        self.links = [l.get("name") for l in root.findall("link")]

    def ancestors(self, link):
        out = [link]
        while link in self.parent:
            link = self.parent[link][0]
            out.append(link)
        return out

    def body(self, link):
        """Representative of the rigid body (merge across fixed joints)."""
        while link in self.parent and self.parent[link][1] == "fixed":
            link = self.parent[link][0]
        return link

    def side(self, link):
        for a in self.ancestors(link):
            for s in SIDES:
                if a == f"{s}_base_link":
                    return s
        return None

    def in_hand(self, link):
        return next((a for a in self.ancestors(link) if a.endswith("_palm_base_link")), None)

    def adjacent(self, a, b):
        ba, bb = self.body(a), self.body(b)
        if ba == bb:
            return True
        for x, y in ((ba, bb), (bb, ba)):
            if x in self.parent and self.body(self.parent[x][0]) == y:
                return True
        return False

    def world_fixed(self, link):
        return self.body(link) == "world"


# --------------------------------------------------------------- mesh loading
def load_mesh(path: str):
    p = Path(path)
    if p.suffix.lower() == ".stl":
        b = p.read_bytes()
        if b[:5] == b"solid" and b"facet" in b[:400]:
            v = [list(map(float, l.split()[1:4])) for l in b.decode().splitlines() if l.strip().startswith("vertex")]
            v = np.array(v)
            return v, np.arange(len(v)).reshape(-1, 3)
        n = struct.unpack("<I", b[80:84])[0]
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        v = a["v"].reshape(-1, 3).astype(float)
        return v, np.arange(len(v)).reshape(-1, 3)
    verts, faces = [], []
    for line in p.read_text(errors="ignore").splitlines():
        if line.startswith("v "):
            verts.append(list(map(float, line.split()[1:4])))
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            faces += [[idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1)]
    return np.array(verts), np.array(faces)


def cluster_decimate(v, f, cell):
    """Vertex-clustering decimation for previews only (keeps the surface closed-looking)."""
    key = np.floor(v / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    centers = np.zeros((inv.max() + 1, 3))
    np.add.at(centers, inv, v)
    centers /= np.bincount(inv)[:, None]
    g = inv[f]
    g = g[(g[:, 0] != g[:, 1]) & (g[:, 1] != g[:, 2]) & (g[:, 0] != g[:, 2])]
    g = np.unique(np.sort(g, axis=1), axis=0)
    return centers, g


# --------------------------------------------------------------------- checks
def frame_pose(model, data, name):
    return data.oMf[model.getFrameId(name)]


def arm_q(model, values: dict):
    q = pin.neutral(model)
    for name, v in values.items():
        jid = model.getJointId(name)
        q[model.joints[jid].idx_q] = v
    return q


def structural_checks(model, data, tree):
    out = {}
    q0 = pin.neutral(model)
    pin.framesForwardKinematics(model, data, q0)
    b = {s: frame_pose(model, data, f"{s}_base_link").translation for s in SIDES}
    out["base_spacing_m"] = float(np.linalg.norm(b["left"] - b["right"]))
    out["base_xyz_m"] = {s: b[s].tolist() for s in SIDES}
    for s in SIDES:
        l6 = frame_pose(model, data, f"{s}_link6")
        fl = frame_pose(model, data, f"{s}_flange_frame")
        palm_name = next(l for l in tree.links if l.endswith("_palm_base_link") and tree.side(l) == s)
        palm = frame_pose(model, data, palm_name)
        tool_axis = fl.rotation[:, 2]
        joint6_axis = l6.rotation[:, 0]
        # Palm normal is +x of palm_base_link: fingers flex about +y towards +x.
        palm_normal = palm.rotation[:, 0]
        ref = fl.rotation[:, 0]
        roll = math.degrees(math.atan2(np.dot(np.cross(ref, palm_normal), tool_axis), np.dot(ref, palm_normal)))
        out[s] = dict(
            hand_root=palm_name,
            tool_axis_world_zero_pose=tool_axis.tolist(),
            tool_axis_parallel_joint6_axis=bool(np.allclose(tool_axis, joint6_axis, atol=1e-12)),
            flange_xyz_world_zero_pose=fl.translation.tolist(),
            palm_origin_along_tool_axis_from_flange_m=float(np.dot(palm.translation - fl.translation, tool_axis)),
            palm_normal_world_zero_pose=np.round(palm_normal, 12).tolist(),
            hand_roll_about_tool_axis_deg=roll,
            tcp_xyz_world_zero_pose=frame_pose(model, data, f"{s}_tcp").translation.tolist(),
        )
    # finger-curl direction sanity: index MCP +0.5 rad must move the tip towards palm +x
    for s in SIDES:
        h = out[s]["hand_root"].split("_")[0]
        tip = f"{h}_index_tip_frame"
        palm = frame_pose(model, data, out[s]["hand_root"])
        p0 = palm.actInv(frame_pose(model, data, tip)).translation
        q1 = q0.copy()
        q1[model.joints[model.getJointId(f"{h}_index_mcp_joint")].idx_q] = 0.5
        pin.framesForwardKinematics(model, data, q1)
        p1 = frame_pose(model, data, out[s]["hand_root"]).actInv(frame_pose(model, data, tip)).translation
        out[s]["index_curl_moves_tip_towards_palm_plus_x"] = bool(p1[0] - p0[0] > 0)
        pin.framesForwardKinematics(model, data, q0)
    # mass budget
    masses = {}
    root = ET.parse(URDF).getroot()
    for l in root.findall("link"):
        m = l.find("inertial/mass")
        if m is not None:
            key = f"{tree.side(l.get('name'))}_{'hand' if tree.in_hand(l.get('name')) else 'arm'}"
            masses[key] = masses.get(key, 0.0) + float(m.get("value"))
    out["mass_kg"] = {k: round(v, 8) for k, v in sorted(masses.items())}
    out["total_mass_kg"] = float(pin.computeTotalMass(model))
    return out


def single_vs_dual(model, data):
    """Each single-arm URDF must equal the dual model up to the base transform."""
    rng = np.random.default_rng(1)
    err = {}
    for s in SIDES:
        ms = pin.buildModelFromUrdf(str(SINGLE[s]), mimic=True)
        ds = ms.createData()
        worst = 0.0
        for _ in range(200):
            qs = pin.randomConfiguration(ms)
            qd = pin.neutral(model)
            for jid in range(1, ms.njoints):
                jd = model.getJointId(ms.names[jid])
                nq = ms.joints[jid].nq
                qd[model.joints[jd].idx_q:model.joints[jd].idx_q + nq] = qs[ms.joints[jid].idx_q:ms.joints[jid].idx_q + nq]
            pin.framesForwardKinematics(ms, ds, qs)
            pin.framesForwardKinematics(model, data, qd)
            base = frame_pose(model, data, f"{s}_base_link")
            for f in (f"{s}_tcp", f"{s}_flange_frame"):
                a = ds.oMf[ms.getFrameId(f)]
                b = base.actInv(frame_pose(model, data, f))
                worst = max(worst, np.abs(a.homogeneous - b.homogeneous).max())
        err[s] = float(worst)
    return err


# ------------------------------------------------------------------ collision
def collision_study(model, geom, tree, samples, seed):
    data = model.createData()
    names = [geom.geometryObjects[i].name for i in range(geom.ngeoms)]
    link_of = [model.frames[o.parentFrame].name for o in geom.geometryObjects]
    pairs = []
    for i in range(geom.ngeoms):
        for j in range(i + 1, geom.ngeoms):
            pairs.append((i, j))

    reason = {}
    for i, j in pairs:
        a, b = link_of[i], link_of[j]
        sa, sb = tree.side(a), tree.side(b)
        if tree.world_fixed(a) and tree.world_fixed(b):
            reason[(i, j)] = "Static"
        elif tree.adjacent(a, b):
            reason[(i, j)] = "Adjacent"
        elif tree.in_hand(a) and tree.in_hand(a) == tree.in_hand(b):
            reason[(i, j)] = "HandInternal"

    tested = [(i, j) for (i, j) in pairs if (i, j) not in reason]
    g = pin.GeometryModel()
    for o in geom.geometryObjects:
        g.addGeometryObject(o)
    for i, j in tested:
        g.addCollisionPair(pin.CollisionPair(i, j))
    gd = pin.GeometryData(g)

    def colliding(q):
        pin.computeCollisions(model, data, g, gd, q, False)
        hit = set()
        for k, cp in enumerate(g.collisionPairs):
            if gd.collisionResults[k].isCollision():
                hit.add((cp.first, cp.second))
        return hit

    q0 = pin.neutral(model)
    zero_hits = colliding(q0)
    qr = arm_q(model, {f"{s}_{j}": v for s in SIDES for j, v in zip(ARM_JOINTS, READY)})
    ready_hits = colliding(qr)

    counts = defaultdict(int)
    np.random.seed(seed)  # pinocchio's randomConfiguration uses the C rand engine
    pin.seed(seed)
    t0 = time.time()
    for _ in range(samples):
        for p in colliding(pin.randomConfiguration(model)):
            counts[p] += 1
    elapsed = time.time() - t0

    for i, j in tested:
        a, b = link_of[i], link_of[j]
        intra = tree.side(a) == tree.side(b)
        # Only arm-vs-own-arm pairs may be sampled away; a hand against its own
        # arm stays enabled even if random sampling never produced a contact.
        if not intra or tree.in_hand(a) or tree.in_hand(b):
            continue
        if (i, j) in zero_hits:
            reason[(i, j)] = "Default"
        elif counts[(i, j)] == 0:
            reason[(i, j)] = "Never"

    enabled = [(i, j) for (i, j) in pairs if (i, j) not in reason]
    stats = dict(
        geometry_objects=geom.ngeoms,
        all_pairs=len(pairs),
        samples=samples,
        sampling_seconds=round(elapsed, 2),
        disabled_by_reason={r: sum(1 for v in reason.values() if v == r) for r in sorted(set(reason.values()))},
        enabled_pairs=len(enabled),
        enabled_cross_arm_pairs=sum(1 for i, j in enabled if tree.side(link_of[i]) != tree.side(link_of[j])),
        zero_pose_contacts=[[link_of[i], link_of[j]] for i, j in sorted(zero_hits)],
        ready_pose_contacts_enabled=[[link_of[i], link_of[j]] for i, j in sorted(ready_hits) if (i, j) not in reason],
        sampled_contact_rate_enabled_top=sorted(
            ([link_of[i], link_of[j], counts[(i, j)] / samples] for i, j in enabled if counts[(i, j)]),
            key=lambda x: -x[2])[:25],
    )
    disabled_links = {}
    for (i, j), r in reason.items():
        key = tuple(sorted((link_of[i], link_of[j])))
        disabled_links[key] = r
    return stats, disabled_links, enabled, link_of, zero_hits


def zero_pose_cross_distance(model, geom, tree, link_of):
    data = model.createData()
    g = pin.GeometryModel()
    for o in geom.geometryObjects:
        g.addGeometryObject(o)
    for i in range(geom.ngeoms):
        for j in range(i + 1, geom.ngeoms):
            if tree.side(link_of[i]) != tree.side(link_of[j]):
                g.addCollisionPair(pin.CollisionPair(i, j))
    gd = pin.GeometryData(g)
    out = {}
    for label, q in (("zero", pin.neutral(model)),
                     ("ready", arm_q(model, {f"{s}_{j}": v for s in SIDES for j, v in zip(ARM_JOINTS, READY)}))):
        pin.computeDistances(model, data, g, gd, q)
        k = int(np.argmin([r.min_distance for r in gd.distanceResults]))
        cp = g.collisionPairs[k]
        out[label] = dict(min_distance_m=float(gd.distanceResults[k].min_distance),
                          pair=[link_of[cp.first], link_of[cp.second]])
    return out


# ----------------------------------------------------------------------- SRDF
def write_srdf(path: Path, disabled: dict, tree: Tree):
    r = ET.Element("robot", name="panthera_dual_a2l1")
    r.append(ET.Comment(" GENERATED by tools/model/verify_description.py - DO NOT EDIT. "
                        "Cross-arm pairs are never disabled (except Static). "))
    for s in SIDES:
        g = ET.SubElement(r, "group", name=f"{s}_arm")
        ET.SubElement(g, "chain", base_link=f"{s}_base_link", tip_link=f"{s}_tcp")
        palm = next(l for l in tree.links if l.endswith("_palm_base_link") and tree.side(l) == s)
        h = ET.SubElement(r, "group", name=f"{s}_hand")
        for l in tree.links:
            if tree.in_hand(l) == palm:
                ET.SubElement(h, "link", name=l)
    both = ET.SubElement(r, "group", name="both_arms")
    for s in SIDES:
        ET.SubElement(both, "group", name=f"{s}_arm")
    for label, vals in (("zero", [0.0] * 6), ("ready", READY)):
        st = ET.SubElement(r, "group_state", name=label, group="both_arms")
        for s in SIDES:
            for j, v in zip(ARM_JOINTS, vals):
                ET.SubElement(st, "joint", name=f"{s}_{j}", value=repr(float(v)))
    for s in SIDES:
        palm = next(l for l in tree.links if l.endswith("_palm_base_link") and tree.side(l) == s)
        ET.SubElement(r, "end_effector", name=f"{s}_hand_ee", parent_link=f"{s}_tcp",
                      group=f"{s}_hand", parent_group=f"{s}_arm")
    for (a, b), why in sorted(disabled.items()):
        ET.SubElement(r, "disable_collisions", link1=a, link2=b, reason=why)
    ET.indent(r, space="  ")
    path.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(r, encoding="unicode") + "\n",
                    encoding="utf-8", newline="\n")


# -------------------------------------------------------------------- preview
def preview(model, geom, path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    data = model.createData()
    gd = pin.GeometryData(geom)
    cache = {}
    rng = np.random.default_rng(0)
    poses = (("zero pose", pin.neutral(model)),
             ("ready pose " + str(READY), arm_q(model, {f"{s}_{j}": v for s in SIDES for j, v in zip(ARM_JOINTS, READY)})))
    views = (("iso", 25, -60), ("top (x fwd, y left)", 90, -90), ("front (from +x)", 0, 0))
    fig = plt.figure(figsize=(18, 11))
    for r, (title, q) in enumerate(poses):
        pin.updateGeometryPlacements(model, data, geom, gd, q)
        tris_by_obj = []
        for k, o in enumerate(geom.geometryObjects):
            if not Path(o.meshPath).is_file():
                continue  # source fingertip marker spheres (r = 1.5 mm): not drawn
            if o.meshPath not in cache:
                v, f = load_mesh(o.meshPath)
                cache[o.meshPath] = cluster_decimate(v, f, 0.002 if "a2l1_hand" in o.meshPath else 0.004)
            v, f = cache[o.meshPath]
            M = gd.oMg[k]
            w = (M.rotation @ (v * o.meshScale).T).T + M.translation
            hand = "palm" in o.name or "thumb" in o.name or any(x in o.name for x in ("index", "middle", "ring", "little"))
            color = ("tab:orange" if hand else "tab:blue") if o.name.startswith("left") else ("tab:red" if hand else "tab:green")
            tris_by_obj.append((w[f], color))
        for c, (vname, elev, azim) in enumerate(views):
            ax = fig.add_subplot(2, 3, r * 3 + c + 1, projection="3d")
            for tris, color in tris_by_obj:
                ax.add_collection3d(Poly3DCollection(tris, facecolor=color, edgecolor="none", alpha=0.8))
            for s in SIDES:
                pin.framesForwardKinematics(model, data, q)
                T = data.oMf[model.getFrameId(f"{s}_tcp")]
                for axis, col in zip(range(3), ("r", "g", "b")):
                    e = T.translation + 0.08 * T.rotation[:, axis]
                    ax.plot(*zip(T.translation, e), color=col, lw=2)
            ax.set_xlim(-0.25, 0.55); ax.set_ylim(-0.45, 0.45); ax.set_zlim(-0.05, 0.6)
            ax.set_box_aspect((0.8, 0.9, 0.65))
            ax.view_init(elev=elev, azim=azim)
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
            ax.set_title(f"{title} - {vname}", fontsize=9)
    fig.suptitle("panthera_dual_a2l1 collision meshes (blue/orange = left arm/hand, green/red = right). "
                 "TCP axes: x red, y green, z blue", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=6000)
    ap.add_argument("--seed", type=int, default=20261120)
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    tree = Tree(URDF)
    model = pin.buildModelFromUrdf(str(URDF), mimic=True)
    data = model.createData()
    geom = pin.buildGeomFromUrdf(model, str(URDF), pin.COLLISION, package_dirs=[str(ROOT / "model")])

    report = dict(
        urdf=str(URDF.relative_to(ROOT)),
        pinocchio=pin.__version__,
        nq=model.nq, nv=model.nv,
        movable_joints=[model.names[i] for i in range(1, model.njoints)],
        mimic_joints=sorted(tree.mimic),
    )
    report["structure"] = structural_checks(model, data, tree)
    report["single_vs_dual_max_abs_error"] = single_vs_dual(model, data)
    stats, disabled, enabled, link_of, _ = collision_study(model, geom, tree, args.samples, args.seed)
    report["collision"] = stats
    report["cross_arm_min_distance"] = zero_pose_cross_distance(model, geom, tree, link_of)

    srdf = PKG_DIR / "srdf" / "panthera_dual_a2l1.srdf"
    srdf.parent.mkdir(parents=True, exist_ok=True)
    write_srdf(srdf, disabled, tree)

    # Round-trip: SRDF loaded through Pinocchio must yield exactly the enabled set.
    g2 = pin.buildGeomFromUrdf(model, str(URDF), pin.COLLISION, package_dirs=[str(ROOT / "model")])
    g2.addAllCollisionPairs()
    pin.removeCollisionPairs(model, g2, str(srdf))
    report["srdf_roundtrip_enabled_pairs"] = len(g2.collisionPairs)
    report["srdf_roundtrip_ok"] = len(g2.collisionPairs) == len(enabled)

    out = PKG_DIR / "verification"
    out.mkdir(parents=True, exist_ok=True)
    (out / "verification.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not args.no_preview:
        preview(model, geom, out / "preview.png")

    s = report["structure"]
    print(f"nq={model.nq}  base spacing={s['base_spacing_m']:.4f} m  total mass={s['total_mass_kg']:.4f} kg")
    for side in SIDES:
        print(f"{side}: roll={s[side]['hand_roll_about_tool_axis_deg']:+.2f} deg  "
              f"palm normal(zero)={np.round(s[side]['palm_normal_world_zero_pose'], 3)}  "
              f"curl ok={s[side]['index_curl_moves_tip_towards_palm_plus_x']}")
    print("single vs dual:", report["single_vs_dual_max_abs_error"])
    print("collision:", json.dumps({k: stats[k] for k in ('all_pairs', 'disabled_by_reason', 'enabled_pairs',
                                                           'enabled_cross_arm_pairs', 'zero_pose_contacts',
                                                           'ready_pose_contacts_enabled', 'sampling_seconds')}))
    print("cross-arm min distance:", report["cross_arm_min_distance"])
    print("srdf round-trip ok:", report["srdf_roundtrip_ok"])


if __name__ == "__main__":
    main()
