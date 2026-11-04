"""Physical incline and mounted-payload fixtures, without editing vendor models."""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .registry import ROOT, get_profile


def make_model(spec, directory):
    profile = get_profile(spec.robot)
    root = ET.parse(profile.model).getroot()
    package = profile.model.parent.parent
    dependencies = {profile.model}
    for mesh in root.iter("mesh"):
        name = mesh.get("filename", "")
        if name.startswith("package://"):
            pkg, relative = name[len("package://"):].split("/", 1)
            if pkg != package.name:
                raise ValueError(f"Unresolved external mesh package: {pkg}")
            target = package / relative
        else:
            target = profile.model.parent / name
        target = target.resolve()
        if not target.is_file():
            raise ValueError(f"Missing model mesh: {target}")
        dependencies.add(target)
        mesh.set("filename", target.as_posix())
    base_mass = sum(float(m.get("value")) for m in root.findall("./link/inertial/mass"))
    links = {link.get("name"): link for link in root.findall("link")}
    roots = set(links) - {c.get("link") for c in root.findall("./joint/child")}
    if len(roots) != 1:
        raise ValueError("Assessment requires a unique URDF root")
    mount = next(iter(roots))
    # Preserve the importer's pure-frame rerooting even when mounting cargo.
    while all(links[mount].find(tag) is None for tag in ("visual", "collision", "inertial")):
        outgoing = [j for j in root.findall("joint") if j.find("parent").get("link") == mount]
        if len(outgoing) != 1 or outgoing[0].get("type") != "fixed":
            break
        mount = outgoing[0].find("child").get("link")
    importer_mass = .1 if links[mount].find("inertial") is None and base_mass > 0 else 0.
    # This is the tyre/surface CONTACT coefficient. Imported Gazebo tyre tags
    # otherwise override the world's coefficient; preserve no such hidden override.
    for gazebo in root.findall("gazebo"):
        for tag in ("mu1", "mu2"):
            element = gazebo.find(tag)
            if element is not None:
                element.set("value", str(spec.ground_mu))
                element.text = str(spec.ground_mu)
    if spec.payload_kg:
        mass = spec.payload_kg
        size = (.30, .25, .20)
        payload = ET.SubElement(root, "link", name="assessment_payload")
        inertial = ET.SubElement(payload, "inertial")
        ET.SubElement(inertial, "mass", value=str(mass))
        x, y, z = size
        ET.SubElement(inertial, "inertia", ixx=str(mass*(y*y+z*z)/12),
                      iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                      ixy="0", ixz="0", iyz="0")
        for kind in ("visual", "collision"):
            shape = ET.SubElement(payload, kind)
            geom = ET.SubElement(shape, "geometry")
            ET.SubElement(geom, "box", size=".30 .25 .20")
            if kind == "visual":
                material = ET.SubElement(shape, "material", name="assessment_cargo")
                ET.SubElement(material, "color", rgba="0.85 0.36 0.08 1")
        joint = ET.SubElement(root, "joint", name="assessment_payload_mount", type="fixed")
        ET.SubElement(joint, "parent", link=mount)
        ET.SubElement(joint, "child", link="assessment_payload")
        ET.SubElement(joint, "origin", xyz=f"{spec.payload_offset_x_m} 0 {spec.payload_height_m}", rpy="0 0 0")
    model = directory / "model.urdf"
    ET.ElementTree(root).write(model, encoding="utf-8", xml_declaration=True)
    return model, dependencies, base_mass, importer_mass, mount


def make_world(spec, directory):
    profile = get_profile(spec.robot)
    if spec.target_speed_m_s > profile.config["max_wheel_speed_radps"] * profile.config["wheel_radius_m"]:
        raise ValueError(f"Requested speed exceeds {spec.robot}'s registered wheel command envelope")
    model, dependencies, base_mass, importer_mass, mount = make_model(spec, directory)
    theta = math.radians(spec.slope_deg)
    sn, cs = math.sin(theta), math.cos(theta)
    height = profile.spawn_height_m + .03
    # Positive slope goes downhill along the robot's +X. The upper face passes
    # through the origin, and robot attitude matches the surface at spawn.
    world = f'''#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
WorldInfo {{ basicTimeStep 16 title "{spec.robot} mobility assessment"
  newtonSolver "mujoco" newtonGroundMu {spec.ground_mu} newtonSubsteps 8 }}
Viewpoint {{ position 3.964 -4.219 4.158 orientation -0.278945 0.120681 0.952694 2.359689 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
DEF SUN_MARKER OmniSimSunMarker {{ }}
DEF FLOOR Solid {{
  translation {-sn*.1} 0 {-cs*.1} rotation 0 1 0 {theta}
  children [ Shape {{ appearance PBRAppearance {{ baseColor .23 .27 .30 roughness .9 metalness 0 }}
    geometry Box {{ size 40 6 .2 }} }} ]
  boundingObject Box {{ size 40 6 .2 }}
}}
DEF ASSESSMENT_ROBOT URDFRobot {{
  url "{model.as_posix()}"
  translation {sn*height} 0 {cs*height} rotation 0 1 0 {theta}
  name "{spec.robot}" supervisor TRUE controller "husky_braking"
}}
'''
    (directory / "world.omniworld").write_text(world, encoding="utf-8")
    return world, dependencies, {"original_model": profile.model.relative_to(ROOT).as_posix(),
        "base_urdf_mass_kg": base_mass, "importer_root_mass_kg": importer_mass,
        "expected_total_mass_kg": base_mass + importer_mass + spec.payload_kg,
        "payload_mount": f"rigid fixed joint, {mount}-frame offset; not loose cargo",
        "downhill_tangent": [cs, 0, -sn], "ground_normal": [sn, 0, cs],
        "slope_deg": spec.slope_deg, "ground_mu": spec.ground_mu,
        "payload_kg": spec.payload_kg, "wheel_count": len(profile.wheel_names)}
