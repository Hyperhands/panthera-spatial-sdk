"""Generate the dual-arm MJCF using the checked-in URDF inertias and collision meshes."""

from pathlib import Path
import os
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "model/panthera_dual_description"
OUTPUT = ROOT / "sim/assets/panthera_dual_mocap.xml"
ZERO = "0 0 0"


def origin(element):
    value = element.find("origin")
    if value is None:
        return {}
    return dict(pos=value.get("xyz", ZERO), euler=value.get("rpy", ZERO))


def generate():
    urdf = ET.parse(PACKAGE / "urdf/panthera_dual_a2l1.urdf")
    model = ET.Element("mujoco", model="Panthera dual-arm mocap")
    ET.SubElement(
        model, "compiler", angle="radian", eulerseq="XYZ", inertiafromgeom="false"
    )
    ET.SubElement(
        model,
        "option",
        timestep="0.0005",
        integrator="implicitfast",
        gravity="0 0 -9.81",
    )
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", azimuth="140", elevation="-20")
    ET.SubElement(model, "statistic", center=".16 0 .28", extent=".85")
    default = ET.SubElement(model, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0")
    assets = ET.SubElement(model, "asset")
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 1.5", dir="0 0.5 -1", directional="true")
    ET.SubElement(
        world,
        "geom",
        type="plane",
        size="1 1 .015",
        pos="0 0 -.003",
        rgba=".15 .18 .2 1",
    )
    equality = ET.SubElement(model, "equality")
    actuators = ET.SubElement(model, "actuator")
    links = {link.get("name"): link for link in urdf.findall("link")}
    children = {}
    for j in urdf.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    meshes = {}

    def add_children(body, parent):
        for j in children.get(parent, []):
            name = j.find("child").get("link")
            link = links[name]
            node = ET.SubElement(body, "body", name=name, **origin(j))
            inertial = link.find("inertial")
            if inertial is not None and float(inertial.find("mass").get("value")) > 0:
                frame = origin(inertial)
                if any(float(x) for x in frame.get("euler", ZERO).split()):
                    raise ValueError(f"Expected link-aligned inertia for {name}")
                tensor = inertial.find("inertia")
                ET.SubElement(
                    node,
                    "inertial",
                    pos=frame.get("pos", ZERO),
                    mass=inertial.find("mass").get("value"),
                    fullinertia=" ".join(
                        tensor.get(k)
                        for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                    ),
                )
            if j.get("type") != "fixed":
                joint = j.get("name")
                limits = j.find("limit")
                arm = joint.split("_", 1)[1] in {f"joint{i}" for i in range(1, 7)}
                ET.SubElement(
                    node,
                    "joint",
                    name=joint,
                    type="hinge",
                    axis=j.find("axis").get("xyz"),
                    range=f"{limits.get('lower')} {limits.get('upper')}",
                    damping=".02" if arm else ".1",
                    armature=".012" if arm else ".0001",
                )
                mimic = j.find("mimic")
                if mimic is not None:
                    ET.SubElement(
                        equality,
                        "joint",
                        joint1=joint,
                        joint2=mimic.get("joint"),
                        polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                    )
                elif arm:
                    ET.SubElement(actuators, "motor", name=joint, joint=joint)
                else:
                    ET.SubElement(
                        actuators, "position", name=joint, joint=joint, kp="4"
                    )
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    continue
                relative = mesh.get("filename").split(
                    "package://panthera_dual_description/"
                )[1]
                if relative not in meshes:
                    meshes[relative] = Path(relative).stem
                    ET.SubElement(
                        assets,
                        "mesh",
                        name=meshes[relative],
                        file=Path(
                            os.path.relpath(PACKAGE / relative, OUTPUT.parent)
                        ).as_posix(),
                    )
                color = ".62 .72 .80 1" if name.startswith("left_") else ".76 .66 .53 1"
                ET.SubElement(
                    node,
                    "geom",
                    type="mesh",
                    mesh=meshes[relative],
                    rgba=color,
                    **origin(collision),
                )
            add_children(node, name)

    add_children(world, "world")
    for side, color in (("left", ".15 .7 1 .65"), ("right", "1 .4 .18 .65")):
        body = ET.SubElement(world, "body", name=f"{side}_target", mocap="true")
        ET.SubElement(
            body, "geom", type="sphere", size=".024", pos="0 0 .085", rgba=color
        )
        ET.SubElement(
            body,
            "site",
            type="capsule",
            fromto="0 0 0 0 0 .085",
            size=".003",
            rgba=color,
        )
    ET.indent(model)
    ET.ElementTree(model).write(OUTPUT, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    generate()
