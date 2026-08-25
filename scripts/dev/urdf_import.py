#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""URDF → OmniSim VRML converter.

Converts a URDF robot description into the equivalent VRML node tree
that the OmniSim simulator can load directly. Kept in sync with the C++
runtime importer at src/omnisim/vrml/OmUrdfImporter.cpp; the two
implementations must produce equivalent output for the same URDF input.

Supported URDF features:
  - <link> with <visual>, <collision>, <inertial>
  - geometry: <box>, <cylinder>, <sphere>, <mesh>
  - <inertia ixx ixy ixz iyy iyz izz> (full tensor, positive-definite check)
  - <joint> types: fixed, revolute, continuous, prismatic
  - <origin xyz="x y z" rpy="r p y"/>
  - <material> with <color rgba="r g b a"/>
  - <gazebo reference="LINK"><sensor> for imu/gps/camera/ray
  - <gazebo><plugin filename="..."> for legacy hector_gazebo IMU/GPS

Usage:
  python scripts/dev/urdf_import.py path/to/robot.urdf > robot.wbt.snippet
  python scripts/dev/urdf_import.py path/to/robot.urdf --to robot.wbt.snippet
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass
class Origin:
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Material:
    name: str = ""
    rgba: tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)


@dataclass
class Geometry:
    kind: str  # box, cylinder, sphere, mesh, unsupported
    box_size: tuple[float, float, float] | None = None
    cylinder_radius: float = 0.0
    cylinder_length: float = 0.0
    sphere_radius: float = 0.0
    detail: str = ""           # mesh filename as written in the URDF
    mesh_path: str = ""        # resolved absolute filesystem path, empty if unresolvable
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass
class Visual:
    origin: Origin = field(default_factory=Origin)
    geometry: Geometry | None = None
    material: Material | None = None


@dataclass
class Collision:
    origin: Origin = field(default_factory=Origin)
    geometry: Geometry | None = None


@dataclass
class Inertial:
    origin: Origin = field(default_factory=Origin)
    mass: float = 0.0
    has_inertia_matrix: bool = False  # True iff the URDF declared a PD <inertia> tensor
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0


@dataclass
class Sensor:
    """Sensor extracted from <gazebo> blocks. URDF has no native schema; we read
    both the modern <gazebo reference><sensor> form and the legacy
    <gazebo><plugin filename="..."> form (used by Clearpath URDFs)."""

    kind: str  # imu, gps, camera, lidar
    name: str
    link_name: str
    update_rate: float = 100.0
    width: int = 320
    height: int = 240
    horizontal_fov: float = 1.047
    min_range: float = 0.1
    max_range: float = 30.0
    lidar_horizontal_resolution: int = 360


@dataclass
class Link:
    name: str
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)
    inertial: Inertial | None = None


@dataclass
class Joint:
    name: str
    type: str  # fixed, revolute, continuous, prismatic
    parent: str
    child: str
    origin: Origin = field(default_factory=Origin)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)
    lower: float | None = None
    upper: float | None = None
    velocity: float | None = None
    effort: float | None = None
    damping: float | None = None
    friction: float | None = None


@dataclass
class UrdfRobot:
    name: str
    materials: dict[str, Material] = field(default_factory=dict)
    links: dict[str, Link] = field(default_factory=dict)
    link_order: list[str] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    sensors: list[Sensor] = field(default_factory=list)


def inertia_is_positive_definite(ixx: float, ixy: float, ixz: float,
                                 iyy: float, iyz: float, izz: float) -> bool:
    """Match the C++ importer's quick analytical PD check on the 3x3 tensor."""
    if ixx <= 0.0:
        return False
    minor2 = ixx * iyy - ixy * ixy
    if minor2 <= 0.0:
        return False
    det = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    return det > 0.0


def _resolve_mesh_path(filename_attr: str, urdf_dir: Path) -> str:
    """Replica of OmUrdfImporter::resolveMeshPath. Returns absolute forward-slash
    path on success, empty string otherwise. Supports package://, file://,
    absolute, and relative-to-URDF-dir references."""
    if not filename_attr:
        return ""
    s = filename_attr.replace("\\", "/")

    def _abs(p: Path) -> str:
        return str(p.resolve()).replace("\\", "/") if p.is_file() else ""

    if s.startswith("file:///"):
        return _abs(Path(s[7:]))  # keep leading slash for POSIX
    if s.startswith("file://"):
        return _abs(Path(s[7:]))
    if s.startswith("package://"):
        rest = s[len("package://"):]
        slash = rest.find("/")
        if slash <= 0:
            return ""
        pkg = rest[:slash]
        tail = rest[slash + 1:]
        d = urdf_dir
        for _ in range(8):
            hit = _abs(d / pkg / tail)
            if hit:
                return hit
            if d.name == pkg:
                hit = _abs(d / tail)
                if hit:
                    return hit
            if d.parent == d:
                break
            d = d.parent
        return _abs(urdf_dir / tail)
    p = Path(s)
    if p.is_absolute():
        return _abs(p)
    return _abs(urdf_dir / s)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def _parse_floats(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    parts = text.split()
    if len(parts) != count:
        return default
    return tuple(float(p) for p in parts)


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        return (1.0, 0.0, 0.0)
    return (x / norm, y / norm, z / norm)


def parse_origin(elem: ET.Element | None) -> Origin:
    if elem is None:
        return Origin()
    return Origin(
        xyz=_parse_floats(elem.get("xyz"), 3, (0.0, 0.0, 0.0)),
        rpy=_parse_floats(elem.get("rpy"), 3, (0.0, 0.0, 0.0)),
    )


def parse_geometry(elem: ET.Element, urdf_dir: Path) -> Geometry | None:
    box = elem.find("box")
    if box is not None:
        size = _parse_floats(box.get("size"), 3, (0.1, 0.1, 0.1))
        return Geometry(kind="box", box_size=size)

    cyl = elem.find("cylinder")
    if cyl is not None:
        return Geometry(
            kind="cylinder",
            cylinder_radius=float(cyl.get("radius", "0.05")),
            cylinder_length=float(cyl.get("length", "0.1")),
        )

    sph = elem.find("sphere")
    if sph is not None:
        return Geometry(kind="sphere", sphere_radius=float(sph.get("radius", "0.05")))

    mesh = elem.find("mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        scale = _parse_floats(mesh.get("scale"), 3, (1.0, 1.0, 1.0)) if mesh.get("scale") else (1.0, 1.0, 1.0)
        return Geometry(
            kind="mesh",
            detail=filename,
            mesh_path=_resolve_mesh_path(filename, urdf_dir),
            mesh_scale=scale,
        )

    for child in list(elem):
        if isinstance(child.tag, str):
            return Geometry(kind="unsupported", detail=child.tag)

    return None


def parse_material(elem: ET.Element, materials: dict[str, Material]) -> Material:
    name = elem.get("name", "")
    color_elem = elem.find("color")
    if color_elem is not None:
        rgba = _parse_floats(color_elem.get("rgba"), 4, (0.5, 0.5, 0.5, 1.0))
        m = Material(name=name, rgba=rgba)
        if name:
            materials[name] = m
        return m
    if name and name in materials:
        return materials[name]
    return Material(name=name)


def parse_visual(elem: ET.Element, materials: dict[str, Material], urdf_dir: Path) -> Visual:
    visual = Visual()
    visual.origin = parse_origin(elem.find("origin"))
    geom_elem = elem.find("geometry")
    if geom_elem is not None:
        visual.geometry = parse_geometry(geom_elem, urdf_dir)
    mat_elem = elem.find("material")
    if mat_elem is not None:
        visual.material = parse_material(mat_elem, materials)
    return visual


def parse_collision(elem: ET.Element, urdf_dir: Path) -> Collision:
    coll = Collision()
    coll.origin = parse_origin(elem.find("origin"))
    geom_elem = elem.find("geometry")
    if geom_elem is not None:
        coll.geometry = parse_geometry(geom_elem, urdf_dir)
    return coll


def parse_inertial(elem: ET.Element) -> Inertial:
    inertial = Inertial()
    inertial.origin = parse_origin(elem.find("origin"))
    mass_elem = elem.find("mass")
    if mass_elem is not None:
        inertial.mass = float(mass_elem.get("value", "0.0"))
    inertia_elem = elem.find("inertia")
    if inertia_elem is not None:
        ixx = float(inertia_elem.get("ixx", "0.0"))
        ixy = float(inertia_elem.get("ixy", "0.0"))
        ixz = float(inertia_elem.get("ixz", "0.0"))
        iyy = float(inertia_elem.get("iyy", "0.0"))
        iyz = float(inertia_elem.get("iyz", "0.0"))
        izz = float(inertia_elem.get("izz", "0.0"))
        if inertia_is_positive_definite(ixx, ixy, ixz, iyy, iyz, izz):
            inertial.ixx = ixx
            inertial.ixy = ixy
            inertial.ixz = ixz
            inertial.iyy = iyy
            inertial.iyz = iyz
            inertial.izz = izz
            inertial.has_inertia_matrix = True
        # else: non-PD tensor will be flagged in build_report; the run-time
        # importer falls back to bounding-object-derived inertia.
    return inertial


def parse_link(elem: ET.Element, materials: dict[str, Material], urdf_dir: Path) -> Link:
    link = Link(name=elem.get("name", ""))
    for v in elem.findall("visual"):
        link.visuals.append(parse_visual(v, materials, urdf_dir))
    for c in elem.findall("collision"):
        link.collisions.append(parse_collision(c, urdf_dir))
    inert_elem = elem.find("inertial")
    if inert_elem is not None:
        link.inertial = parse_inertial(inert_elem)
    return link


def _parse_gazebo_sensor(sensor_el: ET.Element, link_name: str) -> Sensor | None:
    type_attr = sensor_el.get("type", "")
    name_attr = sensor_el.get("name", "")
    if not type_attr or not link_name:
        return None
    update_el = sensor_el.find("update_rate")
    update_rate = float(update_el.text) if update_el is not None and update_el.text else 100.0
    s = Sensor(
        kind="",
        name=name_attr or link_name,
        link_name=link_name,
        update_rate=update_rate,
    )
    if type_attr == "imu":
        s.kind = "imu"
    elif type_attr == "gps":
        s.kind = "gps"
    elif type_attr in ("camera", "depth"):
        s.kind = "camera"
        cam = sensor_el.find("camera")
        if cam is not None:
            hfov = cam.find("horizontal_fov")
            if hfov is not None and hfov.text:
                s.horizontal_fov = float(hfov.text)
            image = cam.find("image")
            if image is not None:
                w = image.find("width")
                h = image.find("height")
                if w is not None and w.text:
                    s.width = int(w.text)
                if h is not None and h.text:
                    s.height = int(h.text)
    elif type_attr in ("ray", "gpu_ray", "lidar", "gpu_lidar"):
        s.kind = "lidar"
        source = sensor_el.find("ray") or sensor_el.find("lidar")
        if source is not None:
            scan = source.find("scan")
            if scan is not None:
                horizontal = scan.find("horizontal")
                if horizontal is not None:
                    samples = horizontal.find("samples")
                    min_a = horizontal.find("min_angle")
                    max_a = horizontal.find("max_angle")
                    if samples is not None and samples.text:
                        s.lidar_horizontal_resolution = int(samples.text)
                    if min_a is not None and max_a is not None and min_a.text and max_a.text:
                        s.horizontal_fov = float(max_a.text) - float(min_a.text)
            r = source.find("range")
            if r is not None:
                mn = r.find("min")
                mx = r.find("max")
                if mn is not None and mn.text:
                    s.min_range = float(mn.text)
                if mx is not None and mx.text:
                    s.max_range = float(mx.text)
    else:
        return None
    return s


def _parse_gazebo_plugin_as_sensor(plugin_el: ET.Element) -> Sensor | None:
    filename = plugin_el.get("filename", "")
    if not filename:
        return None
    body_el = plugin_el.find("bodyName")
    body = (body_el.text or "").strip() if body_el is not None else ""
    if not body:
        return None
    rate_el = plugin_el.find("updateRate")
    update_rate = float(rate_el.text) if rate_el is not None and rate_el.text else 100.0
    topic_el = plugin_el.find("topicName")
    if topic_el is not None and topic_el.text:
        topic = topic_el.text.strip().lstrip("/")
        name = topic.replace("/", "_") if topic else body
    else:
        name = plugin_el.get("name", body)
    if "hector_gazebo_ros_imu" in filename or "gazebo_ros_imu" in filename:
        return Sensor(kind="imu", name=name, link_name=body, update_rate=update_rate)
    if "hector_gazebo_ros_gps" in filename or "gazebo_ros_gps" in filename:
        return Sensor(kind="gps", name=name, link_name=body, update_rate=update_rate)
    return None


def parse_gazebo_extensions(root: ET.Element, robot: UrdfRobot) -> None:
    for g in root.findall("gazebo"):
        reference = g.get("reference", "")
        if reference:
            for sensor_el in g.findall("sensor"):
                s = _parse_gazebo_sensor(sensor_el, reference)
                if s is not None:
                    robot.sensors.append(s)
        for plugin_el in g.findall("plugin"):
            s = _parse_gazebo_plugin_as_sensor(plugin_el)
            if s is not None:
                robot.sensors.append(s)
    # Dedupe by (kind, link, name)
    seen: set[tuple[str, str, str]] = set()
    deduped: list[Sensor] = []
    for s in robot.sensors:
        key = (s.kind, s.link_name, s.name)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(s)
    robot.sensors = deduped


def parse_joint(elem: ET.Element) -> Joint:
    joint = Joint(
        name=elem.get("name", ""),
        type=elem.get("type", "fixed"),
        parent=elem.find("parent").get("link") if elem.find("parent") is not None else "",
        child=elem.find("child").get("link") if elem.find("child") is not None else "",
    )
    joint.origin = parse_origin(elem.find("origin"))
    axis_elem = elem.find("axis")
    if axis_elem is not None:
        joint.axis = normalize_vector(_parse_floats(axis_elem.get("xyz"), 3, (1.0, 0.0, 0.0)))
    limit_elem = elem.find("limit")
    if limit_elem is not None:
        if limit_elem.get("lower") is not None:
            joint.lower = float(limit_elem.get("lower"))
        if limit_elem.get("upper") is not None:
            joint.upper = float(limit_elem.get("upper"))
        if limit_elem.get("velocity") is not None:
            joint.velocity = float(limit_elem.get("velocity"))
        if limit_elem.get("effort") is not None:
            joint.effort = float(limit_elem.get("effort"))
    dynamics_elem = elem.find("dynamics")
    if dynamics_elem is not None:
        if dynamics_elem.get("damping") is not None:
            joint.damping = float(dynamics_elem.get("damping"))
        if dynamics_elem.get("friction") is not None:
            joint.friction = float(dynamics_elem.get("friction"))
    return joint


def parse_urdf(path: Path) -> UrdfRobot:
    # Strip unexpanded xacro placeholders (e.g. TurtleBot3's literal
    # ${namespace} in joint/link names). Keeps the C++ importer in sync.
    text = path.read_text(encoding="utf-8")
    xacro_re = re.compile(r"\$\{[^}]*\}")
    if xacro_re.search(text):
        text = xacro_re.sub("", text)
    root = ET.fromstring(text)
    if root.tag != "robot":
        raise ValueError(f"Expected <robot> root, got <{root.tag}>")
    robot = UrdfRobot(name=root.get("name", "robot"))
    urdf_dir = path.resolve().parent

    # Top-level materials
    for m in root.findall("material"):
        parse_material(m, robot.materials)

    for link_elem in root.findall("link"):
        link = parse_link(link_elem, robot.materials, urdf_dir)
        robot.links[link.name] = link
        robot.link_order.append(link.name)

    for joint_elem in root.findall("joint"):
        robot.joints.append(parse_joint(joint_elem))

    parse_gazebo_extensions(root, robot)

    return robot


# ---------------------------------------------------------------------------
# VRML / WBT emission
# ---------------------------------------------------------------------------


def is_supported_primitive(geometry: Geometry | None) -> bool:
    """Pure geometric primitive: box/cylinder/sphere. Excludes mesh."""
    return geometry is not None and geometry.kind in {"box", "cylinder", "sphere"}


def is_supported_geometry(geometry: Geometry | None) -> bool:
    """Anything emittable into VRML: primitives, or meshes with a resolved path."""
    if geometry is None:
        return False
    if is_supported_primitive(geometry):
        return True
    return geometry.kind == "mesh" and bool(geometry.mesh_path)


def rpy_to_matrix(
    rpy: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate_vector_by_rpy(vector: tuple[float, float, float], rpy: tuple[float, float, float]) -> tuple[float, float, float]:
    matrix = rpy_to_matrix(rpy)
    x, y, z = vector
    rotated = (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )
    return normalize_vector(rotated)


def joint_axis_in_parent_frame(joint: Joint) -> tuple[float, float, float]:
    return rotate_vector_by_rpy(joint.axis, joint.origin.rpy)


def rpy_to_axis_angle(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Convert roll-pitch-yaw (XYZ Euler) to axis-angle (x, y, z, angle)."""
    r, p, y = rpy
    if r == 0.0 and p == 0.0 and y == 0.0:
        return (0.0, 0.0, 1.0, 0.0)

    ((m00, m01, m02), (m10, m11, m12), (m20, m21, m22)) = rpy_to_matrix(rpy)

    # Convert rotation matrix to axis-angle
    trace = m00 + m11 + m22
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))

    if abs(angle) < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)

    if abs(angle - math.pi) < 1e-6:
        # Edge case: 180 degree rotation
        if m00 > m11 and m00 > m22:
            x = math.sqrt(max(0.0, m00 - m11 - m22 + 1.0)) / 2.0
            y = m01 / (2.0 * x) if x else 0.0
            z = m02 / (2.0 * x) if x else 0.0
        elif m11 > m22:
            y = math.sqrt(max(0.0, m11 - m00 - m22 + 1.0)) / 2.0
            x = m01 / (2.0 * y) if y else 0.0
            z = m12 / (2.0 * y) if y else 0.0
        else:
            z = math.sqrt(max(0.0, m22 - m00 - m11 + 1.0)) / 2.0
            x = m02 / (2.0 * z) if z else 0.0
            y = m12 / (2.0 * z) if z else 0.0
        return (x, y, z, math.pi)

    s = 2.0 * math.sin(angle)
    x = (m21 - m12) / s
    y = (m02 - m20) / s
    z = (m10 - m01) / s
    return (x, y, z, angle)


def emit_geometry(geom: Geometry, indent: str) -> str:
    if geom.kind == "box":
        sx, sy, sz = geom.box_size
        return f"{indent}geometry Box {{ size {sx} {sy} {sz} }}\n"
    if geom.kind == "cylinder":
        return f"{indent}geometry Cylinder {{ height {geom.cylinder_length} radius {geom.cylinder_radius} }}\n"
    if geom.kind == "sphere":
        return f"{indent}geometry Sphere {{ radius {geom.sphere_radius} }}\n"
    if geom.kind == "mesh" and geom.mesh_path:
        return f'{indent}geometry Mesh {{ url "{geom.mesh_path}" }}\n'
    return ""


def emit_raw_geometry(geom: Geometry, indent: str) -> str:
    """Bare geometry node, no `geometry` keyword wrapper. Used inside Pose."""
    if geom.kind == "box":
        sx, sy, sz = geom.box_size
        return f"{indent}Box {{ size {sx} {sy} {sz} }}\n"
    if geom.kind == "cylinder":
        return f"{indent}Cylinder {{ height {geom.cylinder_length} radius {geom.cylinder_radius} }}\n"
    if geom.kind == "sphere":
        return f"{indent}Sphere {{ radius {geom.sphere_radius} }}\n"
    if geom.kind == "mesh" and geom.mesh_path:
        return f'{indent}Mesh {{ url "{geom.mesh_path}" }}\n'
    return ""


def emit_visual(visual: Visual, indent: str) -> str:
    if not is_supported_geometry(visual.geometry):
        return ""
    out = []
    ax, ay, az, ang = rpy_to_axis_angle(visual.origin.rpy)
    has_transform = (visual.origin.xyz != (0.0, 0.0, 0.0) or ang != 0.0)

    if has_transform:
        out.append(f"{indent}Pose {{\n")
        out.append(f"{indent}  translation {visual.origin.xyz[0]} {visual.origin.xyz[1]} {visual.origin.xyz[2]}\n")
        if ang != 0.0:
            out.append(f"{indent}  rotation {ax} {ay} {az} {ang}\n")
        out.append(f"{indent}  children [\n")
        inner_indent = indent + "    "
    else:
        inner_indent = indent

    out.append(f"{inner_indent}Shape {{\n")
    if visual.material is not None:
        r, g, b, _ = visual.material.rgba
        out.append(f"{inner_indent}  appearance PBRAppearance {{\n")
        out.append(f"{inner_indent}    baseColor {r} {g} {b}\n")
        out.append(f"{inner_indent}    roughness 0.5\n")
        out.append(f"{inner_indent}    metalness 0\n")
        out.append(f"{inner_indent}  }}\n")
    out.append(emit_geometry(visual.geometry, inner_indent + "  "))
    out.append(f"{inner_indent}}}\n")

    if has_transform:
        out.append(f"{indent}  ]\n")
        out.append(f"{indent}}}\n")

    return "".join(out)


def _emit_collision_inside_group(collision: Collision, indent: str) -> str:
    """Emit one collision suitable for placement inside a Group's children."""
    if not is_supported_geometry(collision.geometry):
        return ""
    geom = collision.geometry
    ax, ay, az, ang = rpy_to_axis_angle(collision.origin.rpy)
    has_transform = (collision.origin.xyz != (0.0, 0.0, 0.0) or ang != 0.0)
    if not has_transform:
        return emit_raw_geometry(geom, indent)
    out = [f"{indent}Pose {{\n"]
    out.append(f"{indent}  translation {collision.origin.xyz[0]} {collision.origin.xyz[1]} {collision.origin.xyz[2]}\n")
    if ang != 0.0:
        out.append(f"{indent}  rotation {ax} {ay} {az} {ang}\n")
    out.append(f"{indent}  children [\n")
    out.append(emit_raw_geometry(geom, indent + "    "))
    out.append(f"{indent}  ]\n")
    out.append(f"{indent}}}\n")
    return "".join(out)


def emit_collision_bounding_object(collision: Collision, indent: str) -> str:
    """Emit a single boundingObject for a link with exactly one supported collision."""
    if not is_supported_geometry(collision.geometry):
        return ""
    geom = collision.geometry
    ax, ay, az, ang = rpy_to_axis_angle(collision.origin.rpy)
    has_transform = (collision.origin.xyz != (0.0, 0.0, 0.0) or ang != 0.0)

    if not has_transform:
        if geom.kind == "box":
            sx, sy, sz = geom.box_size
            return f"{indent}boundingObject Box {{ size {sx} {sy} {sz} }}\n"
        if geom.kind == "cylinder":
            return f"{indent}boundingObject Cylinder {{ height {geom.cylinder_length} radius {geom.cylinder_radius} }}\n"
        if geom.kind == "sphere":
            return f"{indent}boundingObject Sphere {{ radius {geom.sphere_radius} }}\n"
        if geom.kind == "mesh":
            return f'{indent}boundingObject Mesh {{ url "{geom.mesh_path}" }}\n'
        return ""

    out = [f"{indent}boundingObject Pose {{\n"]
    out.append(f"{indent}  translation {collision.origin.xyz[0]} {collision.origin.xyz[1]} {collision.origin.xyz[2]}\n")
    if ang != 0.0:
        out.append(f"{indent}  rotation {ax} {ay} {az} {ang}\n")
    out.append(f"{indent}  children [\n")
    out.append(emit_raw_geometry(geom, indent + "    "))
    out.append(f"{indent}  ]\n")
    out.append(f"{indent}}}\n")
    return "".join(out)


def emit_collision_group(collisions: list[Collision], indent: str) -> str:
    """Group multiple collisions into a single boundingObject Group."""
    out = [f"{indent}boundingObject Group {{\n", f"{indent}  children [\n"]
    for c in collisions:
        if is_supported_geometry(c.geometry):
            out.append(_emit_collision_inside_group(c, indent + "    "))
    out.append(f"{indent}  ]\n")
    out.append(f"{indent}}}\n")
    return "".join(out)


def emit_sensors_for_link(link_name: str, indent: str,
                          sensors_by_link: dict[str, list[Sensor]]) -> str:
    """Emit OmniSim device nodes for sensors attached to this link."""
    if link_name not in sensors_by_link:
        return ""
    out = []
    for s in sensors_by_link[link_name]:
        if s.kind == "imu":
            # OmniSim has no single IMU node — emit the three companion devices
            # a typical IMU consumer wants (orientation, angular rate, accel).
            out.append(f'{indent}InertialUnit {{ name "{s.name}" }}\n')
            out.append(f'{indent}Gyro {{ name "{s.name}_gyro" }}\n')
            out.append(f'{indent}Accelerometer {{ name "{s.name}_accel" }}\n')
        elif s.kind == "gps":
            out.append(f'{indent}GPS {{ name "{s.name}" }}\n')
        elif s.kind == "camera":
            out.append(f"{indent}Camera {{\n")
            out.append(f'{indent}  name "{s.name}"\n')
            out.append(f"{indent}  width {s.width}\n")
            out.append(f"{indent}  height {s.height}\n")
            out.append(f"{indent}  fieldOfView {s.horizontal_fov}\n")
            out.append(f"{indent}}}\n")
        elif s.kind == "lidar":
            out.append(f"{indent}Lidar {{\n")
            out.append(f'{indent}  name "{s.name}"\n')
            out.append(f"{indent}  horizontalResolution {s.lidar_horizontal_resolution}\n")
            out.append(f"{indent}  fieldOfView {s.horizontal_fov}\n")
            out.append(f"{indent}  minRange {s.min_range}\n")
            out.append(f"{indent}  maxRange {s.max_range}\n")
            out.append(f"{indent}}}\n")
    return "".join(out)


def emit_link_children(link: Link, indent: str,
                       sensors_by_link: dict[str, list[Sensor]]) -> str:
    out = []
    for visual in link.visuals:
        out.append(emit_visual(visual, indent))
    out.append(emit_sensors_for_link(link.name, indent, sensors_by_link))
    return "".join(out)


def emit_link_physics_and_bounding(link: Link, indent: str) -> str:
    out = []
    supported = [c for c in link.collisions if is_supported_geometry(c.geometry)]
    if len(supported) == 1:
        out.append(emit_collision_bounding_object(supported[0], indent))
    elif len(supported) > 1:
        out.append(emit_collision_group(supported, indent))
    if link.inertial is not None:
        out.append(f"{indent}physics Physics {{\n")
        out.append(f"{indent}  density -1\n")
        out.append(f"{indent}  mass {link.inertial.mass}\n")
        # OmniSim requires centerOfMass when inertiaMatrix is set. Emit it
        # unconditionally when we have a URDF tensor, even when the URDF
        # origin is (0 0 0).
        has_urdf_inertia = link.inertial.has_inertia_matrix and link.inertial.mass > 0.0
        has_com_offset = link.inertial.origin.xyz != (0.0, 0.0, 0.0)
        if has_urdf_inertia or has_com_offset:
            cx, cy, cz = link.inertial.origin.xyz
            out.append(f"{indent}  centerOfMass [{cx} {cy} {cz}]\n")
        if has_urdf_inertia:
            i = link.inertial
            out.append(f"{indent}  inertiaMatrix [\n")
            out.append(f"{indent}    {i.ixx} {i.iyy} {i.izz}\n")
            out.append(f"{indent}    {i.ixy} {i.ixz} {i.iyz}\n")
            out.append(f"{indent}  ]\n")
        out.append(f"{indent}}}\n")
    return "".join(out)


def append_joint_physics_parameters(out: list[str], joint: Joint, indent: str) -> None:
    if joint.lower is not None and joint.upper is not None:
        out.append(f"{indent}minStop {joint.lower}\n")
        out.append(f"{indent}maxStop {joint.upper}\n")
    if joint.damping is not None:
        out.append(f"{indent}dampingConstant {joint.damping}\n")
    if joint.friction is not None:
        out.append(f"{indent}staticFriction {joint.friction}\n")


def emit_solid_for_link(link: Link, joint: Joint, indent: str, joints_by_parent: dict, links: dict,
                        sensors_by_link: dict[str, list[Sensor]]) -> str:
    """Emit a Solid node for a child link, including its sub-tree of joints."""
    out = []
    out.append(f"{indent}endPoint Solid {{\n")
    tx, ty, tz = joint.origin.xyz
    out.append(f"{indent}  translation {tx} {ty} {tz}\n")
    ax, ay, az, ang = rpy_to_axis_angle(joint.origin.rpy)
    if ang != 0.0:
        out.append(f"{indent}  rotation {ax} {ay} {az} {ang}\n")
    out.append(f"{indent}  name \"{link.name}\"\n")
    out.append(f"{indent}  children [\n")
    out.append(emit_link_children(link, indent + "    ", sensors_by_link))
    # Recurse into child joints
    for child_joint in joints_by_parent.get(link.name, []):
        out.append(emit_joint(child_joint, indent + "    ", joints_by_parent, links, sensors_by_link))
    out.append(f"{indent}  ]\n")
    out.append(emit_link_physics_and_bounding(link, indent + "  "))
    out.append(f"{indent}}}\n")
    return "".join(out)


def emit_joint(joint: Joint, indent: str, joints_by_parent: dict, links: dict,
               sensors_by_link: dict[str, list[Sensor]]) -> str:
    child_link = links.get(joint.child)
    if child_link is None:
        return ""

    out = []
    axis_parent = joint_axis_in_parent_frame(joint)

    if joint.type == "fixed":
        # Fixed joints become a child Solid directly attached to the parent
        out.append(f"{indent}Solid {{\n")
        tx, ty, tz = joint.origin.xyz
        out.append(f"{indent}  translation {tx} {ty} {tz}\n")
        ax, ay, az, ang = rpy_to_axis_angle(joint.origin.rpy)
        if ang != 0.0:
            out.append(f"{indent}  rotation {ax} {ay} {az} {ang}\n")
        out.append(f"{indent}  name \"{child_link.name}\"\n")
        out.append(f"{indent}  children [\n")
        out.append(emit_link_children(child_link, indent + "    ", sensors_by_link))
        for cj in joints_by_parent.get(child_link.name, []):
            out.append(emit_joint(cj, indent + "    ", joints_by_parent, links, sensors_by_link))
        out.append(f"{indent}  ]\n")
        out.append(emit_link_physics_and_bounding(child_link, indent + "  "))
        out.append(f"{indent}}}\n")

    elif joint.type in ("revolute", "continuous"):
        out.append(f"{indent}HingeJoint {{\n")
        out.append(f"{indent}  jointParameters HingeJointParameters {{\n")
        out.append(f"{indent}    axis {axis_parent[0]} {axis_parent[1]} {axis_parent[2]}\n")
        out.append(f"{indent}    anchor {joint.origin.xyz[0]} {joint.origin.xyz[1]} {joint.origin.xyz[2]}\n")
        append_joint_physics_parameters(out, joint, indent + "    ")
        out.append(f"{indent}  }}\n")
        out.append(f"{indent}  device [\n")
        out.append(f"{indent}    RotationalMotor {{\n")
        out.append(f"{indent}      name \"{joint.name}_motor\"\n")
        if joint.velocity is not None:
            out.append(f"{indent}      maxVelocity {joint.velocity}\n")
        else:
            out.append(f"{indent}      maxVelocity 10\n")
        out.append(f"{indent}    }}\n")
        out.append(f"{indent}  ]\n")
        out.append(emit_solid_for_link(child_link, joint, indent + "  ", joints_by_parent, links, sensors_by_link))
        out.append(f"{indent}}}\n")

    elif joint.type == "prismatic":
        out.append(f"{indent}SliderJoint {{\n")
        out.append(f"{indent}  jointParameters JointParameters {{\n")
        out.append(f"{indent}    axis {axis_parent[0]} {axis_parent[1]} {axis_parent[2]}\n")
        append_joint_physics_parameters(out, joint, indent + "    ")
        out.append(f"{indent}  }}\n")
        out.append(f"{indent}  device [\n")
        out.append(f"{indent}    LinearMotor {{\n")
        out.append(f"{indent}      name \"{joint.name}_motor\"\n")
        out.append(f"{indent}    }}\n")
        out.append(f"{indent}  ]\n")
        out.append(emit_solid_for_link(child_link, joint, indent + "  ", joints_by_parent, links, sensors_by_link))
        out.append(f"{indent}}}\n")

    return "".join(out)


def emit_robot(robot: UrdfRobot, controller: str = "<none>") -> str:
    """Emit a OmniSim Robot { ... } node tree from a parsed URDF."""
    # Find root link (one not appearing as a child of any joint)
    child_links = {j.child for j in robot.joints}
    root_links = [name for name in robot.link_order if name not in child_links]
    if not root_links:
        raise ValueError("URDF has no root link (every link is a child of some joint)")
    root_name = root_links[0]
    root_link = robot.links[root_name]

    joints_by_parent: dict[str, list[Joint]] = {}
    for j in robot.joints:
        joints_by_parent.setdefault(j.parent, []).append(j)

    sensors_by_link: dict[str, list[Sensor]] = {}
    for s in robot.sensors:
        if s.link_name in robot.links:
            sensors_by_link.setdefault(s.link_name, []).append(s)

    out = []
    out.append("Robot {\n")
    out.append(f"  name \"{robot.name}\"\n")
    out.append("  children [\n")
    out.append(emit_link_children(root_link, "    ", sensors_by_link))
    for j in joints_by_parent.get(root_name, []):
        out.append(emit_joint(j, "    ", joints_by_parent, robot.links, sensors_by_link))
    out.append("  ]\n")
    out.append(emit_link_physics_and_bounding(root_link, "  "))
    out.append(f"  controller \"{controller}\"\n")
    out.append("}\n")
    return "".join(out)


def _vectors_close(a: tuple[float, float, float], b: tuple[float, float, float], eps: float = 1e-9) -> bool:
    return all(abs(x - y) <= eps for x, y in zip(a, b))


def build_report(robot: UrdfRobot) -> dict:
    child_links = {joint.child for joint in robot.joints}
    root_links = [name for name in robot.link_order if name not in child_links]
    warnings: list[str] = []

    if not root_links:
        warnings.append("Robot has no root link.")
    elif len(root_links) > 1:
        warnings.append(
            f"Multiple root links found: {', '.join(root_links)}. The importer uses '{root_links[0]}'."
        )

    link_entries = []
    for link_name in robot.link_order:
        link = robot.links[link_name]
        link_notes: list[str] = []
        # A geometry is "imported" if it's a primitive or a resolved mesh.
        supported_visuals = sum(1 for visual in link.visuals if is_supported_geometry(visual.geometry))
        unresolved_meshes_v = [
            visual.geometry.detail
            for visual in link.visuals
            if visual.geometry and visual.geometry.kind == "mesh" and not visual.geometry.mesh_path
        ]
        unsupported_visuals = [
            visual.geometry.detail
            for visual in link.visuals
            if visual.geometry and visual.geometry.kind == "unsupported"
        ]
        supported_collisions = sum(1 for collision in link.collisions if is_supported_geometry(collision.geometry))
        unresolved_meshes_c = [
            collision.geometry.detail
            for collision in link.collisions
            if collision.geometry and collision.geometry.kind == "mesh" and not collision.geometry.mesh_path
        ]
        unsupported_collisions = [
            collision.geometry.detail
            for collision in link.collisions
            if collision.geometry and collision.geometry.kind == "unsupported"
        ]

        if link.collisions and supported_collisions == 0:
            link_notes.append("No supported collision geometry is available; the imported link will have no boundingObject.")
        if link.inertial and not link.inertial.has_inertia_matrix:
            # has_inertia_matrix is only False if either no <inertia> tag was
            # present (silent — most leaf links) or the tensor wasn't PD (worth
            # flagging). We can't distinguish without keeping the raw element,
            # but a tensor that parsed all zeros most likely means "no tag".
            tensor_all_zero = (
                link.inertial.ixx == 0.0 and link.inertial.iyy == 0.0 and link.inertial.izz == 0.0
            )
            if not tensor_all_zero:
                link_notes.append(
                    "URDF inertia tensor is not positive definite; falling back to bounding-object-derived inertia."
                )
        if unresolved_meshes_v:
            link_notes.append(
                f"Visual mesh(es) could not be resolved on disk and will be skipped: {', '.join(unresolved_meshes_v)}"
            )
        if unresolved_meshes_c:
            link_notes.append(
                f"Collision mesh(es) could not be resolved on disk and will be skipped: {', '.join(unresolved_meshes_c)}"
            )
        if unsupported_visuals:
            link_notes.append(f"Unsupported visual geometries are skipped: {', '.join(unsupported_visuals)}")
        if unsupported_collisions:
            link_notes.append(f"Unsupported collision geometries are skipped: {', '.join(unsupported_collisions)}")

        link_entries.append(
            {
                "name": link.name,
                "visual_count": len(link.visuals),
                "supported_visual_count": supported_visuals,
                "unresolved_meshes_visual": unresolved_meshes_v,
                "unsupported_visuals": unsupported_visuals,
                "collision_count": len(link.collisions),
                "supported_collision_count": supported_collisions,
                "unresolved_meshes_collision": unresolved_meshes_c,
                "unsupported_collisions": unsupported_collisions,
                "has_inertial": link.inertial is not None,
                "has_inertia_matrix": bool(link.inertial and link.inertial.has_inertia_matrix),
                "notes": link_notes,
            }
        )
        warnings.extend(f"link {link.name}: {note}" for note in link_notes)

    joint_entries = []
    for joint in robot.joints:
        axis_parent = joint_axis_in_parent_frame(joint)
        joint_notes: list[str] = []
        joint_warnings: list[str] = []
        if not _vectors_close(joint.axis, axis_parent):
            joint_notes.append(
                "Joint axis is rotated by the joint origin before emission because OmniSim expects the axis in the parent frame."
            )
        if joint.type == "revolute" and (joint.lower is None or joint.upper is None):
            joint_warning = "Revolute joint has incomplete limits; the imported joint will be free unless limits are added."
            joint_notes.append(joint_warning)
            joint_warnings.append(joint_warning)

        joint_entries.append(
            {
                "name": joint.name,
                "type": joint.type,
                "parent": joint.parent,
                "child": joint.child,
                "origin_xyz": list(joint.origin.xyz),
                "origin_rpy": list(joint.origin.rpy),
                "axis_joint_frame": list(joint.axis),
                "axis_parent_frame": list(axis_parent),
                "lower": joint.lower,
                "upper": joint.upper,
                "velocity": joint.velocity,
                "effort": joint.effort,
                "damping": joint.damping,
                "friction": joint.friction,
                "notes": joint_notes,
            }
        )
        warnings.extend(f"joint {joint.name}: {note}" for note in joint_warnings)

    sensor_entries = [
        {
            "name": s.name,
            "kind": s.kind,
            "link": s.link_name,
            "update_rate": s.update_rate,
        }
        for s in robot.sensors
    ]
    for s in robot.sensors:
        if s.link_name not in robot.links:
            warnings.append(f"sensor {s.name}: references unknown link '{s.link_name}'; will be skipped at import time.")

    return {
        "robot": robot.name,
        "root_links": root_links,
        "link_count": len(robot.links),
        "joint_count": len(robot.joints),
        "sensor_count": len(robot.sensors),
        "warnings": warnings,
        "links": link_entries,
        "joints": joint_entries,
        "sensors": sensor_entries,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert URDF to OmniSim VRML")
    parser.add_argument("urdf", type=Path, help="Path to .urdf file")
    parser.add_argument("--to", type=Path, default=None, help="Output file (default: stdout)")
    parser.add_argument("--controller", default="<none>", help="Controller name to set on the Robot node")
    parser.add_argument("--report", type=Path, default=None, help="Write a JSON import report for debugging")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a non-zero exit code if the import report contains warnings",
    )
    args = parser.parse_args()

    if not args.urdf.exists():
        print(f"URDF file not found: {args.urdf}", file=sys.stderr)
        return 1

    robot = parse_urdf(args.urdf)
    vrml = emit_robot(robot, controller=args.controller)
    report = build_report(robot)

    if args.to:
        args.to.write_text(vrml)
        print(f"[urdf-import] Wrote {args.to} ({len(vrml)} bytes)", file=sys.stderr)
    else:
        sys.stdout.write(vrml)

    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"[urdf-import] Wrote report {args.report}", file=sys.stderr)

    for warning in report["warnings"]:
        print(f"[urdf-import] warning: {warning}", file=sys.stderr)

    return 2 if args.strict and report["warnings"] else 0


if __name__ == "__main__":
    sys.exit(main())
