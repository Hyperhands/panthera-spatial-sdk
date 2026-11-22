"""Fit conservative capsule envelopes to URDF collision meshes and hand joint sweeps."""

import itertools
import json
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull
from clearance import certify, clearance
from model import PKG, ROOT, OUT, FRAMES, SIDES, origin, vector

sys.path.insert(0, str(ROOT / "tools/model"))
from verify_description import load_mesh, READY


def hull(points):
    return points[ConvexHull(points).vertices]


def fit(points, margin):
    # Minimum enclosing circle perpendicular to the principal axis. At the
    # resulting radius, shorten both ends as far as vertex containment allows.
    p = hull(points)
    centre = p.mean(axis=0)
    _, axes = np.linalg.eigh(np.cov((p - centre).T))
    xy = (p - centre) @ axes[:, :2]
    z = (p - centre) @ axes[:, 2]

    def radius(c):
        return np.max(np.linalg.norm(xy - c, axis=1))

    c = np.zeros(2)
    # Minimize the maximum projected radius by successively finer planar searches.
    directions = np.stack(
        (np.cos(np.arange(24) * np.pi / 12), np.sin(np.arange(24) * np.pi / 12)), axis=1
    )
    step = np.ptp(xy, axis=0).max() / 2
    while step > 1e-9:
        candidates = c + step * directions
        best = min(candidates, key=radius)
        if radius(best) < radius(c):
            c = best
        else:
            step *= 0.5
    r = radius(c) + 1e-9
    slack = np.sqrt(np.maximum(0, r * r - np.sum((xy - c) ** 2, axis=1)))
    lo, hi = np.min(z + slack), np.max(z - slack)
    if lo > hi:
        lo = hi = (lo + hi) / 2
    centre += axes[:, :2] @ c
    a, b = centre + lo * axes[:, 2], centre + hi * axes[:, 2]
    distances = np.linalg.norm(
        points - a - np.outer(np.clip((points - a) @ axes[:, 2], 0, hi - lo), axes[:, 2]), axis=1
    )
    if np.max(distances) > r + 1e-12:
        raise ValueError("Incomplete mesh coverage")
    return a, b, r + margin


def mesh(link):
    geom = link.find("collision")
    obj = geom.find("geometry/mesh")
    if obj is None:
        r = float(geom.find("geometry/sphere").get("radius"))
        t = origin(geom)
        return np.array(list(itertools.product([-r, r], repeat=3))) @ t[:3, :3].T + t[:3, 3], None
    v, f = load_mesh(PKG / obj.get("filename").split("/", 3)[-1])
    t = origin(geom)
    scale = np.fromstring(obj.get("scale", "1 1 1"), sep=" ")
    return (v * scale) @ t[:3, :3].T + t[:3, 3], f


def sweep(points, chain):
    drivers = [j for j in chain if j.find("mimic") is None]
    grids = {}
    for j in drivers:
        low, hi = limits(j)
        grids[j.get("name")] = np.linspace(low, hi, 1 + int(np.ceil((hi - low) / 0.05)))
    # For each ancestor, |dp/dq| <= maximum possible radius about that joint.
    # Clamping a mimic joint cannot increase its driver's angular displacement.
    bound = 0.0
    reach = np.linalg.norm(points, axis=1).max()
    for j in reversed(chain):
        mimic = j.find("mimic")
        driver = j.get("name") if mimic is None else mimic.get("joint")
        gain = 1 if mimic is None else abs(float(mimic.get("multiplier", "1")))
        grid = grids[driver]
        bound += reach * gain * (grid[-1] - grid[0]) / (2 * (len(grid) - 1))
        reach += np.linalg.norm(origin(j)[:3, 3])
    cloud = []
    for vals in itertools.product(*grids.values()):
        angles = dict(zip(grids, vals))
        t = np.eye(4)
        for j in chain:
            mimic = j.find("mimic")
            if mimic is None:
                angle = angles[j.get("name")]
            else:
                angle = np.clip(
                    angles[mimic.get("joint")] * float(mimic.get("multiplier", "1"))
                    + float(mimic.get("offset", "0")),
                    *limits(j),
                )
            t = t @ origin(j)
            t[:3, :3] = t[:3, :3] @ pin.AngleAxis(angle, vector(j.find("axis"), "xyz")).matrix()
        cloud.append(points @ t[:3, :3].T + t[:3, 3])
    return hull(np.vstack(cloud)), bound


def limits(j):
    return tuple(float(j.find("limit").get(k)) for k in ("lower", "upper"))


def make(side):
    tree = ET.parse(PKG / "urdf" / f"panthera_{side}_a2l1.urdf")
    joints = {j.find("child").get("link"): j for j in tree.iter("joint")}
    caps = []

    def add(frame, names, points, margin=0, triangles=None):
        a, b, r = fit(points, margin)
        caps.append(
            dict(
                points=hull(points),
                triangles=triangles,
                frame=FRAMES.index(frame),
                side=SIDES.index(side),
                links=names,
                a=a.tolist(),
                b=b.tolist(),
                r=r,
                step_margin=margin,
            )
        )

    groups = defaultdict(list)
    for link in tree.iter("link"):
        if link.find("collision") is None:
            continue
        name = link.get("name")
        short = name[len(side) + 1 :]
        v, f = mesh(link)
        if short in FRAMES:
            # Assign complete triangles so a face crossing the split remains covered.
            if short in ("link2", "link3", "link4", "link5", "base_link", "link1"):
                axis = np.linalg.eigh(np.cov(v.T))[1][:, -1]
                centres = v[f].mean(axis=1) @ axis
                mask = centres < (0.5 * (centres.min() + centres.max()))
                for part in (mask, ~mask):
                    add(short, [name], v[np.unique(f[part])], triangles=v[f[part]])
            else:
                add(short, [name], v, triangles=v[f])
        else:
            chain = []
            parent = name
            while parent != side + "_palm_base_link":
                j = joints[parent]
                chain.insert(0, j)
                parent = j.find("parent").get("link")
            while chain[-1].get("type") == "fixed":
                t = origin(chain.pop())
                v = v @ t[:3, :3].T + t[:3, 3]
            pts, delta = sweep(hull(v), chain)
            group = "thumb" if "thumb" in short else "fingers"
            groups[group].append((name, pts, delta))
    for items in groups.values():
        add(
            "palm_base_link",
            [x[0] for x in items],
            np.vstack([x[1] for x in items]),
            max(x[2] for x in items),
        )
    return caps


def segment_distance(a, b, c, d):
    u, v, w = b - a, d - c, a - c
    candidates = []
    for p, x, y in ((a, c, d), (b, c, d), (c, a, b), (d, a, b)):
        dr = y - x
        t = np.clip(np.dot(p - x, dr) / max(np.dot(dr, dr), 1e-28), 0, 1)
        candidates.append(np.linalg.norm(p - x - t * dr))
    cross = np.cross(u, v)
    det = cross @ cross
    if det > 1e-24:
        s = np.dot(np.cross(v, w), cross) / det
        t = np.dot(np.cross(u, w), cross) / det
        if 0 <= s <= 1 and 0 <= t <= 1:
            candidates.append(np.linalg.norm(w + s * u - t * v))
    return min(candidates)


def folded_clearance_samples(model, urdf):
    geom = pin.buildGeomFromUrdf(model, str(urdf), pin.COLLISION, package_dirs=[str(PKG.parent)])
    frames = {}
    indices = {}
    for side in SIDES:
        frames[side] = [model.getFrameId(f"{side}_link{k}") for k in (2, 5)]
        objects = [
            next(i for i, o in enumerate(geom.geometryObjects) if o.parentFrame == frame)
            for frame in frames[side]
        ]
        geom.addCollisionPair(pin.CollisionPair(*objects))
        indices[side] = [
            model.joints[model.getJointId(f"{side}_joint{k}")].idx_q for k in (3, 4, 5)
        ]
    data = model.createData()
    geometry_data = geom.createData()
    samples = {s: [] for s in SIDES}
    q = pin.neutral(model)
    for angles in itertools.product(
        (0.5, 1, 1.5, 2, 3), (-2.5, -2, -1.5, -1, -0.5, 0), (-1.5, 0, 1.5)
    ):
        for side in SIDES:
            q[indices[side]] = np.deg2rad(angles)
        pin.framesForwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geom, geometry_data)
        for k, side in enumerate(SIDES):
            distance = pin.computeDistance(geom, geometry_data, k).min_distance
            if distance >= 0.0005:
                ta, tb = (data.oMf[f].copy() for f in frames[side])
                samples[side].append((ta, tb, distance))
    return samples


def supplement_folded_proofs(a, b, proofs, samples):
    for ta, tb, distance in samples:
        raw = (
            segment_distance(
                *(t.act(np.asarray(c[key])) for c, t in ((a, ta), (b, tb)) for key in ("a", "b"))
            )
            - a["r"]
            - b["r"]
        )
        gap = max([raw] + [clearance(group, ta, tb) for group in proofs])
        if gap < max(0.0002, 0.35 * distance):
            extra = certify(
                a["points"],
                b["points"],
                ta,
                tb,
                a["step_margin"],
                b["step_margin"],
                a["triangles"],
                b["triangles"],
            )
            if clearance(extra, ta, tb) > gap + 0.0001:
                proofs.append(extra)
    proofs.sort(key=len)


def wrist_clearance_frames(model):
    # Only (J5, J6) affect the relative pose of link4/link5 against palm_base_link.
    # Extract the fixed transforms along link4 -> joint5 -> link5 -> joint6 -> palm_base_link
    # so we can sweep a dense 2.5 deg (J5, J6) grid in milliseconds.
    data = model.createData()
    pin.framesForwardKinematics(model, data, pin.neutral(model))
    chains = {}
    for side in SIDES:
        f4 = data.oMf[model.getFrameId(f"{side}_link4")]
        f5 = data.oMf[model.getFrameId(f"{side}_link5")]
        f6 = data.oMf[model.getFrameId(f"{side}_link6")]
        f7 = data.oMf[model.getFrameId(f"{side}_palm_base_link")]
        t45 = f4.inverse() * f5
        t56 = f5.inverse() * f6
        t67 = f6.inverse() * f7
        ax5 = np.array([0.0, 0.0, -1.0])
        ax6 = np.array([1.0, 0.0, 0.0])
        chains[side] = (t45, t56, t67, ax5, ax6)

    identity = pin.SE3.Identity()
    j5_vals = np.deg2rad(np.linspace(-60, 60, 49))
    j6_vals = np.deg2rad(np.linspace(-160, 160, 129))
    frames = []
    for j5, j6 in itertools.product(j5_vals, j6_vals):
        entry = {}
        for side, (t45, t56, t67, ax5, ax6) in chains.items():
            r5 = pin.SE3(pin.AngleAxis(j5, ax5).matrix(), np.zeros(3))
            r6 = pin.SE3(pin.AngleAxis(j6, ax6).matrix(), np.zeros(3))
            p5 = t45 * r5
            p7 = p5 * t56 * r6 * t67
            entry[(side, 4)] = identity
            entry[(side, 5)] = p5
            entry[(side, 7)] = p7
        frames.append(entry)
    return frames


def supplement_wrist_proofs(a, b, proofs, wrist_frames, target=0.0014):
    side = SIDES[a["side"]]
    ta_list = [f[(side, a["frame"])] for f in wrist_frames]
    tb_list = [f[(side, b["frame"])] for f in wrist_frames]
    raws = np.array(
        [
            segment_distance(
                ta.act(np.asarray(a["a"])),
                ta.act(np.asarray(a["b"])),
                tb.act(np.asarray(b["a"])),
                tb.act(np.asarray(b["b"])),
            )
            - a["r"]
            - b["r"]
            for ta, tb in zip(ta_list, tb_list)
        ]
    )
    best = raws.copy()
    for group in proofs:
        best = np.maximum(best, [clearance(group, ta, tb) for ta, tb in zip(ta_list, tb_list)])
    while True:
        idx = int(np.argmin(best))
        if best[idx] >= target:
            break
        extra = certify(
            a["points"],
            b["points"],
            ta_list[idx],
            tb_list[idx],
            a["step_margin"],
            b["step_margin"],
            a["triangles"],
            b["triangles"],
        )
        covered = np.array([clearance(extra, ta, tb) for ta, tb in zip(ta_list, tb_list)])
        if covered[idx] <= best[idx] + 0.0001:
            break
        proofs.append(extra)
        best = np.maximum(best, covered)
    proofs.sort(key=len)
    return float(raws.min()), float(best.min())


def run():
    caps = sum((make(s) for s in SIDES), [])
    srdf = ET.parse(PKG / "srdf/panthera_dual_a2l1.srdf")
    disabled = {
        frozenset([p.get("link1"), p.get("link2")]) for p in srdf.iter("disable_collisions")
    }
    pairs = [
        (i, j)
        for j, b in enumerate(caps)
        for i, a in enumerate(caps[:j])
        if any(x != y and frozenset([x, y]) not in disabled for x in a["links"] for y in b["links"])
    ]
    urdf = PKG / "urdf/panthera_dual_a2l1.urdf"
    m = pin.buildModelFromUrdf(str(urdf))
    data = m.createData()
    dist = {}
    poses = {}
    for pose, angles in dict(
        zero=[0] * 6,
        ready=READY,
        home_waypoint=[
            -0.030775491,
            0.434116572,
            0.752034128,
            -0.051771853,
            -0.036911413,
            0.132018223,
        ],
        lift=[0, np.deg2rad(5), np.deg2rad(5), 0, 0, 0],
    ).items():
        q = pin.neutral(m)
        for side in SIDES:
            for k, x in enumerate(angles, 1):
                q[m.joints[m.getJointId(f"{side}_joint{k}")].idx_q] = x
        pin.framesForwardKinematics(m, data, q)
        poses[pose] = [
            data.oMf[m.getFrameId(SIDES[c["side"]] + "_" + FRAMES[c["frame"]])].copy() for c in caps
        ]
        segments = []
        for c in caps:
            t = data.oMf[m.getFrameId(SIDES[c["side"]] + "_" + FRAMES[c["frame"]])]
            segments.append([t.act(np.asarray(c[k])) for k in ("a", "b")])
        dist[pose] = [
            segment_distance(*segments[i], *segments[j]) - caps[i]["r"] - caps[j]["r"]
            for i, j in pairs
        ]
    folded_samples = folded_clearance_samples(m, urdf)
    wrist_frames = wrist_clearance_frames(m)
    thresholds = []
    for k, (i, j) in enumerate(pairs):
        a, b = caps[i], caps[j]
        baseline = min(dist[pose][k] for pose in poses)
        inter = a["side"] != b["side"]
        wrist = not inter and a["frame"] in (4, 5) and b["frame"] == 7
        proofs = []
        for pose, frames in poses.items():
            if inter or dist[pose][k] > 0.012:
                continue
            ta, tb = frames[i], frames[j]
            if any(clearance(group, ta, tb) > 0.001 for group in proofs):
                continue
            proofs.append(
                certify(
                    a["points"],
                    b["points"],
                    ta,
                    tb,
                    a["step_margin"],
                    b["step_margin"],
                    a["triangles"],
                    b["triangles"],
                )
            )
        wrist_min = float("inf")
        if wrist:
            wrist_raw_min, wrist_min = supplement_wrist_proofs(a, b, proofs, wrist_frames)
            baseline = min(baseline, wrist_raw_min)
        effective = {
            pose: max(
                [dist[pose][k]] + [clearance(group, frames[i], frames[j]) for group in proofs]
            )
            for pose, frames in poses.items()
        }
        minimum = min(min(effective.values()), wrist_min)
        if minimum <= 0:
            raise ValueError((i, j, effective, wrist_min))
        hard = 0.005 if inter else min(0.005, minimum * 0.4)
        capsule_margin = 0.005 if wrist else 0.001
        thresholds.append(
            dict(
                hard=hard,
                safe=0.02 if inter else min(0.02, max(0.001, minimum * 0.8)),
                capsule_hard=0.005 if inter else min(hard, baseline - capsule_margin),
                certified=effective,
                proofs=proofs,
            )
        )
        side = SIDES[a["side"]]
        if a["links"] == [f"{side}_link2"] and b["links"] == [f"{side}_link5"]:
            supplement_folded_proofs(a, b, proofs, folded_samples[side])
    for c in caps:
        del c["points"], c["triangles"]
    emit_tables(caps, pairs, thresholds)
    for t in thresholds:
        t["witness_counts"] = [len(g) for g in t.pop("proofs")]
    out = {"capsules": caps, "pairs": pairs, "clearances_m": dist, "thresholds": thresholds}
    (OUT / "capsules.json").write_text(json.dumps(out, indent=1) + "\n")
    print(len(caps), "capsules,", len(pairs), "pairs")
    for pose, ds in dist.items():
        worst = int(np.argmin(ds))
        i, j = pairs[worst]
        print(pose, ds[worst], caps[i]["links"][0], caps[j]["links"][0])


def emit_tables(caps, pairs, thresholds):
    def cpp(x):
        if isinstance(x, (list, tuple)):
            return "{" + ",".join(map(cpp, x)) + "}"
        return format(x, ".17g")

    lines = ["// Generated by tools/codegen/capsules.py"]

    def table(name, rows):
        lines.append(name + "[] = {")
        lines.extend(cpp(row) + "," for row in rows)
        lines.append("};")

    table(
        "constexpr Capsule capsules",
        [[c["side"], c["frame"], c["a"], c["b"], c["r"]] for c in caps],
    )
    witnesses = []
    groups = []
    spec = []
    for (i, j), t in zip(pairs, thresholds):
        spec.append([i, j, t["hard"], t["safe"], t["capsule_hard"], len(groups), len(t["proofs"])])
        for group in t["proofs"]:
            groups.append([len(witnesses), len(group)])
            witnesses.extend(
                [[p["normal"], p["offset"], p["center"], p["half_axes"]] for p in group]
            )
    table("constexpr Witness witnesses", witnesses)
    table("constexpr Group groups", groups)
    table("constexpr Pair pairs", spec)
    tree = ET.parse(PKG / "urdf/panthera_left_a2l1.urdf")
    lim = [limits(tree.find(f"joint[@name='left_joint{j}']")) for j in range(1, 7)]
    lim[4] = (-np.pi / 3, np.pi / 3)
    table("constexpr double lower", [p[0] for p in lim])
    table("constexpr double upper", [p[1] for p in lim])
    (OUT / "geometry.inc").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    run()
