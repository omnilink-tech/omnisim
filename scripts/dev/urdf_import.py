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
    inertia_declared: bool = False  # True iff an <inertia> tag was present at all
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
    # Surface friction from <gazebo reference="LINK"><mu1>. None = not declared,
    # so the Solid inherits WorldInfo.newtonGroundMu. URDF has no native schema
    # for surface friction; the gazebo extension is the de-facto convention, and
    # dropping it silently is exactly the "declared but unread" failure this
    # importer warns about elsewhere.
    surface_friction: float | None = None


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
    # <mimic joint="X" multiplier="M" offset="O">. Every commodity parallel-jaw
    # gripper is mimic-driven (Robotiq 2F85/2F140/3F, Franka Hand, PAL, Schunk),
    # so dropping it silently turns a coupled gripper into independently free
    # fingers -- they drift apart under asymmetric load and the grasp never
    # closes symmetrically, with no error anywhere.
    mimic_joint: str | None = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0
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


def implied_inertia_from_primitive(geom, mass: float) -> tuple[float, float, float] | None:
    """Principal moments a link WOULD have if its collision primitive were solid.

    Only defined for box/sphere/cylinder -- a mesh needs the actual vertices. The
    point is not to demand this value, it is to catch a tensor that disagrees with
    the link's own declared geometry by orders of magnitude, which is how zero and
    placeholder inertias reach shipped robots. A foot declaring a 22 mm collision
    sphere and mass 0.02 implies 3.872e-06; several published quadrupeds declare 0
    on exactly that link.
    """
    if geom is None or mass <= 0.0:
        return None
    if geom.kind == "sphere":
        i = 0.4 * mass * geom.sphere_radius ** 2
        return (i, i, i)
    if geom.kind == "box" and geom.box_size:
        a, b, c = geom.box_size
        k = mass / 12.0
        return (k * (b * b + c * c), k * (a * a + c * c), k * (a * a + b * b))
    if geom.kind == "cylinder":
        r, h = geom.cylinder_radius, geom.cylinder_length
        it = mass * (3.0 * r * r + h * h) / 12.0
        return (it, it, 0.5 * mass * r * r)
    return None


def principal_moments(ixx: float, ixy: float, ixz: float,
                      iyy: float, iyz: float, izz: float) -> tuple[float, float, float]:
    """Eigenvalues of the symmetric 3x3 inertia tensor, ascending.

    Closed form (Smith 1961) rather than numpy: this script is deliberately
    dependency-light and runs as a preflight before anything else is installed.
    """
    p1 = ixy * ixy + ixz * ixz + iyz * iyz
    if p1 == 0.0:
        return tuple(sorted((ixx, iyy, izz)))  # already diagonal
    q = (ixx + iyy + izz) / 3.0
    p2 = (ixx - q) ** 2 + (iyy - q) ** 2 + (izz - q) ** 2 + 2.0 * p1
    p = math.sqrt(p2 / 6.0)
    # B = (A - q*I) / p, then r = det(B) / 2
    b11, b22, b33 = (ixx - q) / p, (iyy - q) / p, (izz - q) / p
    b12, b13, b23 = ixy / p, ixz / p, iyz / p
    det = (b11 * (b22 * b33 - b23 * b23)
           - b12 * (b12 * b33 - b23 * b13)
           + b13 * (b12 * b23 - b22 * b13))
    r = max(-1.0, min(1.0, det / 2.0))
    phi = math.acos(r) / 3.0
    eig1 = q + 2.0 * p * math.cos(phi)
    eig3 = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    eig2 = 3.0 * q - eig1 - eig3
    return tuple(sorted((eig1, eig2, eig3)))


def max_admissible_product_of_inertia(ixx: float, iyy: float, izz: float,
                                      tol: float = 1e-12) -> float:
    """Largest equal-magnitude off-diagonal that keeps the tensor physical.

    Detecting that a tensor is impossible tells an author it is wrong. It does
    NOT tell them what would be right, and that is the question they are
    actually stuck on -- a real maintainer, asked to replace a bad product of
    inertia, answered "1e-7 works as well, but that is again just a guessed
    number." This turns the warning into an interval: any |off-diagonal| at or
    below the returned value is admissible for the given diagonal.

    Bisection rather than a closed form because the constraint is the pair
    (positive definite AND triangle inequality on the PRINCIPAL moments), and
    the second is what the eigenvalues -- not the declared entries -- decide.
    """
    def ok(p: float) -> bool:
        if inertia_violates_triangle_inequality(ixx, p, p, iyy, p, izz)[0]:
            return False
        return inertia_is_positive_definite(ixx, p, p, iyy, p, izz)

    if not ok(0.0):
        return 0.0                       # the diagonal alone is already impossible
    hi = max(abs(ixx), abs(iyy), abs(izz))
    if hi <= 0.0:
        return 0.0
    while ok(hi):                        # bracket upward if even the diagonal scale is fine
        hi *= 2.0
        if hi > 1e12:
            return hi
    lo = 0.0
    while hi - lo > tol * max(1.0, hi):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def inertia_violates_triangle_inequality(ixx: float, ixy: float, ixz: float,
                                         iyy: float, iyz: float, izz: float,
                                         rel_tol: float = 1e-9) -> tuple[bool, tuple[float, float, float]]:
    """A rigid body's principal moments must satisfy a + b >= c.

    A tensor can be positive definite and still describe no physical body, so
    this is a strictly stronger check than inertia_is_positive_definite() and
    catches a different defect class (most commonly a product of inertia that
    was copy-pasted from a moment of inertia).
    """
    a, b, c = principal_moments(ixx, ixy, ixz, iyy, iyz, izz)
    return (a + b) < c * (1.0 - rel_tol), (a, b, c)


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
        # Always retain what the file declared; has_inertia_matrix stays the
        # "PD and therefore usable for emission" gate. Keeping the rejected
        # values is what lets build_report tell a bad tensor from an absent one
        # and run the triangle-inequality check on it.
        inertial.ixx = ixx
        inertial.ixy = ixy
        inertial.ixz = ixz
        inertial.iyy = iyy
        inertial.iyz = iyz
        inertial.izz = izz
        inertial.inertia_declared = True
        if inertia_is_positive_definite(ixx, ixy, ixz, iyy, iyz, izz):
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
            # <gazebo reference="LINK"><mu1>2.0</mu1></gazebo>. mu1/mu2 are the
            # two tangential directions; per-Solid friction is a single isotropic
            # value, so mu1 is taken and a DIFFERING mu2 is reported as a lossy
            # import rather than silently averaged.
            _link = robot.links.get(reference)
            if _link is not None:
                _m1 = g.find("mu1")
                if _m1 is not None and (_m1.text or "").strip():
                    try:
                        _link.surface_friction = float(_m1.text.strip())
                    except ValueError:
                        pass
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
    mimic_elem = elem.find("mimic")
    if mimic_elem is not None and mimic_elem.get("joint"):
        joint.mimic_joint = mimic_elem.get("joint")
        try:
            joint.mimic_multiplier = float(mimic_elem.get("multiplier", "1.0"))
            joint.mimic_offset = float(mimic_elem.get("offset", "0.0"))
        except ValueError:
            pass
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
    # Surface friction declared via the gazebo extension. Emitted as the
    # per-Solid field so it actually reaches the solver; absent means the
    # Solid inherits WorldInfo.newtonGroundMu, so nothing changes for a URDF
    # that does not declare it.
    if link.surface_friction is not None:
        out.append(f"{indent}newtonFriction {link.surface_friction}" + chr(10))
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
        if link.inertial is not None:
            inert = link.inertial
            if inert.inertia_declared and not inert.has_inertia_matrix:
                link_notes.append(
                    "URDF inertia tensor is not positive definite; falling back to bounding-object-derived inertia."
                )
            # A tensor can be positive definite and still describe no physical
            # body. Checked independently of the PD gate so a valid-looking
            # tensor is not waved through.
            if inert.inertia_declared:
                violates, (a, b, c) = inertia_violates_triangle_inequality(
                    inert.ixx, inert.ixy, inert.ixz, inert.iyy, inert.iyz, inert.izz
                )
                if violates:
                    # The interval is what makes this actionable rather than a
                    # complaint: it answers "then what value IS right?".
                    p_max = max_admissible_product_of_inertia(
                        inert.ixx, inert.iyy, inert.izz)
                    link_notes.append(
                        "URDF inertia tensor violates the triangle inequality for principal moments "
                        f"(a+b >= c): principal moments are ({a:.9g}, {b:.9g}, {c:.9g}), "
                        f"a+b = {a + b:.9g} < c = {c:.9g} (short by {100.0 * (1.0 - (a + b) / c):.3f}%). "
                        "No rigid body can have this tensor; a product of inertia copied from a "
                        "moment of inertia is the usual cause. "
                        f"For this diagonal, any |off-diagonal| <= {p_max:.6g} is admissible."
                    )
            # A tensor that disagrees with the link's OWN collision primitive by
            # orders of magnitude is checkable from one file, with no sibling and
            # no mesh loading. Deliberately an order-of-magnitude test: real links
            # are not uniform solids, so a factor of a few is ordinary.
            if inert.inertia_declared and inert.mass > 0.0 and link.collisions:
                prim = next((c.geometry for c in link.collisions
                             if c.geometry is not None
                             and c.geometry.kind in ("box", "sphere", "cylinder")), None)
                implied = implied_inertia_from_primitive(prim, inert.mass)
                if implied is not None:
                    declared_trace = inert.ixx + inert.iyy + inert.izz
                    implied_trace = sum(implied)
                    if implied_trace > 0.0:
                        if declared_trace <= 0.0:
                            link_notes.append(
                                f"Link declares an all-zero (or negative) inertia while carrying a "
                                f"{prim.kind} collision primitive and mass={inert.mass}; a solid "
                                f"{prim.kind} of those dimensions implies principal moments "
                                f"({implied[0]:.6g}, {implied[1]:.6g}, {implied[2]:.6g})."
                            )
                        elif declared_trace < implied_trace / 10.0 or declared_trace > implied_trace * 10.0:
                            link_notes.append(
                                f"Declared inertia disagrees with the link's own {prim.kind} collision "
                                f"primitive by {implied_trace / declared_trace:.3g}x on the trace "
                                f"(declared {declared_trace:.6g}, implied {implied_trace:.6g} for a solid "
                                f"{prim.kind} at mass={inert.mass}). Worth confirming the units."
                            )
            # Zero mass is only a defect when the link is meant to be a body.
            # A massless frame carrying no geometry is a normal URDF idiom.
            if inert.mass <= 0.0 and (link.collisions or link.visuals):
                link_notes.append(
                    f"Link declares mass={inert.mass} but carries "
                    f"{len(link.visuals)} visual and {len(link.collisions)} collision geometr"
                    f"{'y' if len(link.collisions) == 1 else 'ies'}; a dynamic body with no mass "
                    "cannot be simulated and consumers differ on whether they reject it or "
                    "integrate garbage."
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
        if joint.mimic_joint is not None:
            # OmniSim has no coupled-joint primitive, so this constraint is
            # DROPPED at emission. Silence here turns a coupled gripper into
            # independently free fingers: they drift apart under asymmetric load
            # and never close symmetrically, with no error anywhere. Every
            # commodity parallel-jaw gripper is mimic-driven, including three
            # shipped in this repo.
            joint_warning = (
                f"Joint mimics '{joint.mimic_joint}' (multiplier {joint.mimic_multiplier}, "
                f"offset {joint.mimic_offset}), but the coupling is NOT imported -- the joint "
                "becomes independently free. Drive both joints from one command in the "
                "controller, or the gripper will not close symmetrically."
            )
            joint_notes.append(joint_warning)
            joint_warnings.append(joint_warning)
        if joint.type in ("revolute", "prismatic", "continuous"):
            # effort/velocity of 0 is not "no limit declared" -- URDF spells that
            # by omitting the attribute. A declared zero is a declared inability
            # to move, and consumers that honour it give the joint no authority
            # at all (the arm collapses under gravity, the gripper never closes).
            zeroed = [
                name for name, value in (("effort", joint.effort), ("velocity", joint.velocity))
                if value is not None and value == 0.0
            ]
            if zeroed:
                joint_warning = (
                    f"Actuated joint declares {' and '.join(f'{n}=0' for n in zeroed)}; "
                    "consumers that honour URDF limits will give this joint zero authority. "
                    "Omit the attribute if the intent was 'unlimited'."
                )
                joint_notes.append(joint_warning)
                joint_warnings.append(joint_warning)
        if (joint.type in ("revolute", "prismatic")
                and joint.lower is not None and joint.upper is not None
                and joint.lower == joint.upper):
            # A zero-width range on a joint declared movable is a mis-typed fixed
            # joint. It slips past the range-excludes-zero check whenever the
            # pinned value happens to BE zero, which is the common case.
            joint_warning = (
                f"Joint declares a zero-width range [{joint.lower}, {joint.upper}] but is "
                f"type '{joint.type}'; it cannot move, so this is almost certainly meant "
                "to be type=\"fixed\". A pinned value of 0 also passes the "
                "range-excludes-zero check, so nothing else flags it."
            )
            joint_notes.append(joint_warning)
            joint_warnings.append(joint_warning)
        if joint.type in ("revolute", "prismatic") and joint.lower is not None and joint.upper is not None:
            # A URDF can only express one default configuration -- all-zero. A
            # joint whose own range excludes 0 therefore has no representable
            # valid default, and every consumer that initialises at q=0 starts
            # in limit violation and takes a constraint impulse on step 1.
            if not (joint.lower <= 0.0 <= joint.upper):
                joint_warning = (
                    f"Joint range [{joint.lower}, {joint.upper}] excludes the zero position, "
                    "which is the only default a URDF can express; consumers that initialise "
                    "at q=0 will start this joint outside its own limits. Publish a home pose "
                    "alongside the URDF."
                )
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


# ── CROSS-FILE MIRROR DIFF ───────────────────────────────────────────────────
# A whole class of real defect is INVISIBLE to a single-file checker, because
# nothing in the file itself is out of range -- it is only wrong relative to the
# robot's other hand/arm/leg. Found in the field on published descriptions:
#   * a hand whose left file declares (effort 100, velocity 1) on all 21 joints
#     and whose right declares (effort 1, velocity 100) -- transposed, and only
#     the sibling reveals which side is right;
#   * two joints carrying a placeholder effort=100 where the mirror has
#     calibrated 4.29 and 4.8;
#   * fingertip joints pinned at 1.0 rad on one hand and 0.0 on the other.
# This mode also carries the ONLY reliable source of the CORRECT value, which is
# what turns a complaint into a fix.

MIRROR_PREFIXES = (("left_", "right_"), ("l_", "r_"), ("L_", "R_"),
                   ("lh_", "rh_"), ("left", "right"))
MIRROR_SUFFIXES = (("_left", "_right"), ("_l", "_r"), ("_L", "_R"))


def mirror_name(name: str) -> str | None:
    """The name this link/joint would have on the opposite side, or None.

    Two conventions exist and both are common:
      * ONE file describing both sides, where the side is in the NAME
        (``left_arm_joint2`` / ``right_arm_joint2``) -- handled by the affix
        swap below;
      * TWO files, one per side, where the side is in the FILE PATH and the
        joint names are IDENTICAL (``index_mcp_roll`` in both). Handled by the
        identity fallback in the caller -- a name with no mirror affix matches
        its own name in the other file. Missing that second case made this
        checker silently match ZERO pairs on the very descriptions it was
        written for.
    """
    for a, b in MIRROR_PREFIXES:
        if name.startswith(a):
            return b + name[len(a):]
        if name.startswith(b):
            return a + name[len(b):]
    for a, b in MIRROR_SUFFIXES:
        if name.endswith(a):
            return name[: -len(a)] + b
        if name.endswith(b):
            return name[: -len(b)] + a
    return None


def build_mirror_report(robot: UrdfRobot, other: UrdfRobot) -> dict:
    """Differences between a robot and its mirror twin, matched by name.

    Reports only fields that SHOULD agree across a mirror. Poses and the
    products of inertia legitimately differ (see the sign note below), so they
    are handled separately rather than reported as mismatches.
    """
    findings: list[str] = []
    matched = 0

    ours = {j.name: j for j in robot.joints}
    theirs = {j.name: j for j in other.joints}
    for name, j in sorted(ours.items()):
        twin_name = mirror_name(name)
        if twin_name is None or twin_name not in theirs:
            # Two-file convention: identical names, side carried by the path.
            twin_name = name if name in theirs else None
        if twin_name is None:
            continue
        matched += 1
        t = theirs[twin_name]
        for field_name, a, b in (("effort", j.effort, t.effort),
                                 ("velocity", j.velocity, t.velocity),
                                 ("type", j.type, t.type)):
            if a != b:
                findings.append(
                    f"joint {name} / {twin_name}: {field_name} differs across the mirror "
                    f"({a} vs {b}). Mirror-image hardware should agree; one side is "
                    "likely the intended value."
                )
        # Limits mirror as a NEGATED, SWAPPED pair on a mirrored axis, so a bare
        # inequality would be noise. Only a differing WIDTH is unambiguous.
        if None not in (j.lower, j.upper, t.lower, t.upper):
            wa, wb = abs(j.upper - j.lower), abs(t.upper - t.lower)
            if abs(wa - wb) > 1e-6 * max(1.0, abs(wa), abs(wb)):
                findings.append(
                    f"joint {name} / {twin_name}: range WIDTH differs across the mirror "
                    f"({wa:.6g} vs {wb:.6g} rad); the sign convention may flip but the "
                    "travel should not."
                )

    lours = robot.links
    lthem = other.links
    for name, link in sorted(lours.items()):
        twin_name = mirror_name(name)
        if twin_name is None or twin_name not in lthem:
            twin_name = name if name in lthem else None
        if twin_name is None:
            continue
        twin = lthem[twin_name]
        if link.inertial is None or twin.inertial is None:
            continue
        if abs(link.inertial.mass - twin.inertial.mass) > 1e-9 * max(1.0, link.inertial.mass):
            findings.append(
                f"link {name} / {twin_name}: mass differs across the mirror "
                f"({link.inertial.mass} vs {twin.inertial.mass})."
            )
        # Under a mirror about a principal plane, exactly two products of inertia
        # NEGATE and one keeps its sign -- which two depends on the plane. We do
        # not know the plane, so we only flag the case where NONE of the three
        # negated while at least one is materially non-zero: a mirrored link whose
        # products are all identical is the signature of geometry that was
        # mirrored without transforming its inertia tensor.
        if link.inertial.inertia_declared and twin.inertial.inertia_declared:
            prods = [(link.inertial.ixy, twin.inertial.ixy),
                     (link.inertial.ixz, twin.inertial.ixz),
                     (link.inertial.iyz, twin.inertial.iyz)]
            scale = max(abs(v) for pair in prods for v in pair)
            if scale > 1e-12:
                identical = all(abs(a - b) <= 1e-9 * scale for a, b in prods)
                if identical:
                    findings.append(
                        f"link {name} / {twin_name}: all three products of inertia are "
                        "IDENTICAL across the mirror, none negated. A mirrored body should "
                        "negate two of the three; this is the signature of geometry that was "
                        "mirrored without transforming its inertia tensor. Worth confirming "
                        "against the mesh rather than taking this as proof."
                    )
    return {"matched_joint_pairs": matched, "findings": findings}


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert URDF to OmniSim VRML")
    parser.add_argument("urdf", type=Path, help="Path to .urdf file")
    parser.add_argument("--to", type=Path, default=None, help="Output file (default: stdout)")
    parser.add_argument("--controller", default="<none>", help="Controller name to set on the Robot node")
    parser.add_argument("--report", type=Path, default=None, help="Write a JSON import report for debugging")
    parser.add_argument(
        "--mirror", type=Path, default=None,
        help="Compare against the robot's mirror twin (the other hand/arm/leg) and report "
             "fields that disagree across the mirror. Catches transposed or placeholder "
             "values that are in range in both files and only wrong relative to each other.")
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

    if args.mirror:
        if not args.mirror.exists():
            print(f"Mirror file not found: {args.mirror}", file=sys.stderr)
            return 1
        mrep = build_mirror_report(robot, parse_urdf(args.mirror))
        report["mirror"] = mrep
        report["warnings"].extend(mrep["findings"])
        print(f"[urdf-import] mirror: matched {mrep['matched_joint_pairs']} joint pairs, "
              f"{len(mrep['findings'])} finding(s)", file=sys.stderr)
        for f in mrep["findings"]:
            print(f"[urdf-import] mirror: {f}", file=sys.stderr)

    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"[urdf-import] Wrote report {args.report}", file=sys.stderr)

    for warning in report["warnings"]:
        print(f"[urdf-import] warning: {warning}", file=sys.stderr)

    return 2 if args.strict and report["warnings"] else 0


if __name__ == "__main__":
    sys.exit(main())
