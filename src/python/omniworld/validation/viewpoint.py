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

"""Viewpoint-framing validator — "does this world open looking at its subject?"

The convention is documented in ``docs/developer/viewpoint-convention.md`` and
implemented by :mod:`omniworld.viewpoint`. Until now nothing *enforced* it, and
worlds drifted: an audit of the tree found 160 worlds sharing the literal
``orientation -0.5773 0.5773 0.5773 2.0944`` (which decodes to forward
``(0, 0, -1)`` — straight down) pasted into worlds whose ``position`` had since
been moved to an oblique eye point, so the camera stares at empty floor metres
from the robot.

This module answers, for one ``.wbt``:

1. **Where does the camera look?** Parse the ``Viewpoint`` block for
   ``position`` / ``orientation`` / ``fieldOfView``. The camera's local frame is
   +X forward, +Y left, +Z up; ``orientation`` is axis-angle.
2. **What is the subject?** Every ``Robot`` / ``Supervisor`` / ``URDFRobot``
   node — including PROTO instances whose base node resolves to one of those
   (``Husky { ... }``) — with parent transforms accumulated so nested robots
   land at their true world position. Scene helpers that merely *derive* from
   ``Robot`` (the draggable sun marker, conveyor belts, mirrors) and bodiless
   supervisor shims are excluded. A world with no subject falls back to framing
   the scene extent.
3. **Is the subject in the frustum, and how much of the frame does it fill?**
   Reported as a signed angular miss (degrees outside the frustum edge) and a
   fill fraction (subject angular radius / the tight half-FOV).

``fieldOfView`` follows VRML semantics — it is the angle on the **larger**
viewport dimension, so on a 16:9 window it is the horizontal FOV and the
binding (tighter) axis is vertical. Getting that backwards makes every tall
subject look fine on paper and overflow in practice.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field as dc_field
from pathlib import Path

from ..viewpoint import DEFAULT_ASPECT, DEFAULT_FOV, SUBJECT_PRESETS
from ._wbt_tree import (
    IDENTITY,
    WbtNode,
    axis_angle_to_matrix,
    iter_tree,
    mat_apply,
    mat_mul,
    parse_wbt,
    proto_index_for,
)

__all__ = [
    "Subject",
    "ViewpointVerdict",
    "analyze_viewpoint",
    "check_viewpoint",
    "guess_subject_class",
    "SUBJECT_BASE_TYPES",
    "NON_SUBJECT_TYPES",
]

Vec3 = tuple[float, float, float]

# Built-in node types that make a node a framing subject.
SUBJECT_BASE_TYPES = frozenset({"Robot", "Supervisor", "URDFRobot", "DifferentialWheels"})

# Robot-derived nodes that are scene furniture, not the world's subject.
# (Each of these resolves to ``Robot`` only because it needs a controller.)
NON_SUBJECT_TYPES = frozenset({
    "OmniSimSunMarker",     # the draggable sun gizmo every world carries
    "ConveyorBelt",         # factory prop with a belt controller
    "Mirror",               # rendering prop
    "ControlledStreetLight",
    "TrafficLight",
    "StreetLight",
})

# A ``Robot`` whose controller is one of these is a passive wrapper -- the
# node exists only to host devices/fixtures (city traffic lights, the sun
# gizmo, harness plumbing), never to be looked at. ``<extern>`` is NOT here:
# an extern-driven robot is a real subject.
_SHIM_CONTROLLERS = frozenset({
    "", "<none>", "none", "sun_marker",
    "harness_supervisor", "capture_supervisor",
})

# Nodes that draw something. A ``Robot`` whose ``children`` contain none of
# these is a *device mount* -- a bodiless carrier for a Camera / Lidar / GPS
# (``CAMBOT``, ``BIN_CAM``, ``CEILING_CAM``, ``SCANNERS`` in this tree). They
# are Robots, but framing one means framing a point in mid-air.
_BODY_TYPES = frozenset({"Shape", "CadShape"})

# Grouping + device nodes that a device mount is allowed to contain.
_TRANSPARENT_TYPES = frozenset({
    "Pose", "Transform", "Group", "Solid", "Slot", "Track", "USE",
    "Camera", "Lidar", "RangeFinder", "Radar", "GPS", "Compass", "InertialUnit",
    "Accelerometer", "Gyro", "Receiver", "Emitter", "Display", "Speaker",
    "Microphone", "LightSensor", "DistanceSensor", "TouchSensor", "Connector",
    "Pen", "LED", "Altimeter", "Skin", "Recognition", "Lens", "Focus",
    "Zoom", "Charger",
})

# Nodes that never contribute to a "scene extent" fallback (they are not
# geometry the camera should frame).
_NON_GEOMETRY_TYPES = frozenset({
    "WorldInfo", "Viewpoint", "Background", "TexturedBackground",
    "TexturedBackgroundLight", "OmniSimSky", "OmniSimSun", "OmniSimSunMarker",
    "DirectionalLight", "PointLight", "SpotLight", "Fog", "NightSky",
})

# Keyword -> subject class, used to size the framing sphere. Longest match wins.
_CLASS_KEYWORDS: tuple[tuple[str, str], ...] = (
    ("g1", "humanoid"), ("h1", "humanoid"), ("humanoid", "humanoid"),
    ("atlas", "humanoid"), ("nao", "humanoid"), ("talos", "humanoid"),
    ("omniquad", "quadruped"), ("spot", "quadruped"),
    ("go2", "quadruped"), ("go1", "quadruped"),
    ("b2", "quadruped"), ("anymal", "quadruped"), ("a1", "quadruped"),
    ("laikago", "quadruped"), ("aliengo", "quadruped"), ("quadruped", "quadruped"),
    ("husky", "mobile"), ("jackal", "mobile"), ("rosbot", "mobile"),
    ("turtlebot", "mobile"), ("tb3", "mobile"), ("burger", "mobile"),
    ("waffle", "mobile"), ("tiago", "mobile"), ("fetch", "mobile"),
    ("pr2", "mobile"), ("summit", "mobile"), ("ridgeback", "mobile"),
    ("warthog", "mobile"), ("robotino", "mobile"), ("battlebot", "mobile"),
    ("ur5", "arm"), ("ur10", "arm"), ("ur3", "arm"),
    ("panda", "arm"), ("franka", "arm"), ("kinova", "arm"), ("gen3", "arm"),
    ("iiwa", "arm"), ("kuka", "arm"), ("xarm", "arm"), ("arm", "arm"),
    ("manipulator", "arm"), ("gripper", "arm"),
)


def guess_subject_class(*hints: str | None) -> str:
    """Best-effort subject class from a robot's name / DEF / URDF url.

    Only used to pick a framing radius, so a wrong guess costs a slightly
    loose or tight frame, never a wrong direction. Defaults to ``mobile``.
    """
    blob = " ".join(h.lower() for h in hints if h)
    blob = re.sub(r"[^a-z0-9]+", " ", blob)
    tokens = set(blob.split())
    best: tuple[int, str] | None = None
    for key, klass in _CLASS_KEYWORDS:
        if key in tokens or key in blob.replace(" ", ""):
            score = len(key) + (5 if key in tokens else 0)
            if best is None or score > best[0]:
                best = (score, klass)
    return best[1] if best else "mobile"


# --------------------------------------------------------------------------
# Data
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Subject:
    """One framing subject with its world-space placement."""

    label: str
    type_name: str
    klass: str
    center: Vec3          # world position, already raised by the class look-height
    radius: float
    def_name: str | None = None
    name: str | None = None

    @property
    def handle(self) -> str | None:
        """The identifier ``set_viewpoint.py --subject`` would accept."""
        return self.def_name or self.name


@dataclass
class ViewpointVerdict:
    """Structured outcome of the framing check on one world."""

    world: Path
    status: str                     # "ok" | "borderline" | "broken"
    subject_kind: str               # "robot" | "scene" | "none"
    reason: str
    has_viewpoint: bool = True
    subjects: tuple[Subject, ...] = ()
    n_subjects: int = 0
    n_in_frame: int = 0
    angular_miss_deg: float = 0.0   # 0 when the subject sphere is in-frustum
    fill_frac: float = 0.0          # subject angular radius / tight half-FOV
    center: Vec3 | None = None
    radius: float = 0.0
    eye: Vec3 | None = None
    follow: str | None = None
    suggestion: str = ""

    @property
    def ok(self) -> bool:
        return self.status != "broken"

    def summary(self) -> str:
        return f"{self.status}: {self.reason}"


# --------------------------------------------------------------------------
# Scene walk
# --------------------------------------------------------------------------


def _node_transform(node: WbtNode) -> tuple[Vec3, tuple]:
    t = node.floats("translation", 3) or (0.0, 0.0, 0.0)
    r = node.floats("rotation", 4)
    return t, (axis_angle_to_matrix(*r) if r else IDENTITY)


def _is_shim_robot(node: WbtNode, base: str) -> bool:
    """Is this ``Robot`` scene plumbing rather than the world's subject?

    Two shapes: a *passive wrapper* (``controller "<none>"`` -- e.g. the node
    that hosts a junction's traffic lights in ``environments/city.omniworld``), and a
    *bodiless shim* (no ``children`` at all, so there is literally nothing to
    look at).

    The bodiless test applies only to a **literal** ``Robot``/``Supervisor``.
    A PROTO instance (``DemoBot { translation ... }``) carries its body in the
    PROTO, not at the call site, so "no children here" proves nothing.
    """
    if base not in ("Robot", "Supervisor"):
        return False
    ctrl = node.string("controller")
    if ctrl is not None and ctrl.strip() in _SHIM_CONTROLLERS:
        return True
    if node.type_name != base:
        return False
    return not _has_body(node)


def _has_body(node: WbtNode) -> bool:
    """Does this node's visible scene graph draw anything?

    Walks ``children`` only (a ``boundingObject`` is collision geometry, not a
    body). Anything that is neither a grouping node nor a device is assumed to
    draw -- an unresolved PROTO child is a body, not a sensor.
    """
    stack = list(node.children)
    while stack:
        n = stack.pop()
        if n.type_name in _BODY_TYPES:
            return True
        if n.type_name not in _TRANSPARENT_TYPES:
            return True
        stack.extend(n.children)
    return False


# Half-extent of a primitive geometry node, in its own local frame.
def _geometry_extent(node: WbtNode) -> float | None:
    t = node.type_name
    if t == "Box":
        s = node.floats("size", 3) or (2.0, 2.0, 2.0)
        return 0.5 * math.sqrt(sum(c * c for c in s))
    if t == "Sphere":
        r = node.floats("radius", 1)
        return r[0] if r else 1.0
    if t in ("Cylinder", "Cone"):
        r = node.floats("radius", 1) or node.floats("bottomRadius", 1) or (1.0,)
        h = node.floats("height", 1) or (2.0,)
        return math.hypot(r[0], h[0] / 2.0)
    if t == "Capsule":
        r = node.floats("radius", 1) or (1.0,)
        h = node.floats("height", 1) or (2.0,)
        return r[0] + h[0] / 2.0
    if t == "Plane":
        s = node.floats("size", 2) or (1.0, 1.0)
        return 0.5 * math.hypot(*s)
    return None


def _measure_body(node: WbtNode) -> tuple[Vec3, float] | None:
    """Bounding (offset, radius) of a robot's inline geometry, in its own frame.

    Hand-built ``Robot { children [ ... ] }`` bodies vary from a 5 cm e-puck to
    a 3 m gantry, so a class preset radius is a bad guess for them. URDF robots
    have no inline geometry (their body comes from the ``url``) and fall back to
    the preset. Returns ``None`` when nothing measurable was found.
    """
    lo = [math.inf] * 3
    hi = [-math.inf] * 3

    def walk(n: WbtNode, pos: Vec3, rot) -> None:
        if n.type_name == "USE":
            return
        t, r = _node_transform(n)
        wt = mat_apply(rot, t)
        world = (pos[0] + wt[0], pos[1] + wt[1], pos[2] + wt[2])
        ext = _geometry_extent(n)
        if ext is not None:
            for i in range(3):
                lo[i] = min(lo[i], world[i] - ext)
                hi[i] = max(hi[i], world[i] + ext)
        for c in n.child_nodes():
            walk(c, world, mat_mul(rot, r))

    for c in node.child_nodes():
        walk(c, (0.0, 0.0, 0.0), IDENTITY)
    if not all(math.isfinite(v) for v in lo + hi):
        return None
    center = tuple((lo[i] + hi[i]) / 2.0 for i in range(3))
    radius = 0.5 * math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(3)))
    return center, max(radius, 0.05)


def _collect(nodes: list[WbtNode], proto_index) -> tuple[list[Subject], list[Vec3]]:
    """Walk the tree accumulating transforms; return (subjects, geometry points)."""
    subjects: list[Subject] = []
    points: list[Vec3] = []

    def walk(node: WbtNode, pos: Vec3, rot) -> None:
        if node.type_name in ("USE",):
            return
        t, r = _node_transform(node)
        wt = mat_apply(rot, t)
        world = (pos[0] + wt[0], pos[1] + wt[1], pos[2] + wt[2])
        world_rot = mat_mul(rot, r)

        base = node.type_name
        if base not in SUBJECT_BASE_TYPES:
            base = proto_index.resolve(node.type_name)

        if node.type_name not in _NON_GEOMETRY_TYPES:
            points.append(world)

        if (base in SUBJECT_BASE_TYPES
                and node.type_name not in NON_SUBJECT_TYPES
                and not _is_shim_robot(node, base)):
            name = node.string("name")
            url = node.string("url")
            klass = guess_subject_class(node.def_name, name, url, node.type_name)
            preset = SUBJECT_PRESETS.get(klass) or SUBJECT_PRESETS["mobile"]
            measured = _measure_body(node)
            if measured is not None:
                off, radius = measured
                off = mat_apply(world_rot, off)
                center = (world[0] + off[0], world[1] + off[1], world[2] + off[2])
            else:
                radius = float(preset["radius"] or 1.2)
                center = (world[0], world[1], world[2] + preset["look_z"])
            subjects.append(Subject(
                label=node.def_name or name or node.type_name,
                type_name=node.type_name,
                klass=klass,
                center=center,
                radius=float(radius),
                def_name=node.def_name,
                name=name,
            ))
            # Do not descend into a robot: its links are part of the same body.
            return

        for child in node.child_nodes():
            walk(child, world, world_rot)

    for n in nodes:
        walk(n, (0.0, 0.0, 0.0), IDENTITY)
    return subjects, points


def _bounding(centers: list[Vec3], radii: list[float]) -> tuple[Vec3, float]:
    xs = [c[0] for c in centers]
    ys = [c[1] for c in centers]
    zs = [c[2] for c in centers]
    mid = ((min(xs) + max(xs)) / 2.0,
           (min(ys) + max(ys)) / 2.0,
           (min(zs) + max(zs)) / 2.0)
    rad = 0.0
    for c, r in zip(centers, radii):
        d = math.dist(c, mid)
        rad = max(rad, d + r)
    return mid, max(rad, 1e-3)


# --------------------------------------------------------------------------
# Frustum test
# --------------------------------------------------------------------------


def _half_angles(fov: float, aspect: float) -> tuple[float, float]:
    """(horizontal, vertical) half-FOV from the VRML larger-dimension ``fov``."""
    half = max(min(fov, math.pi - 1e-3), 1e-3) / 2.0
    if aspect >= 1.0:
        return half, math.atan(math.tan(half) / aspect)
    return math.atan(math.tan(half) * aspect), half


def _angles(eye: Vec3, rot, target: Vec3) -> tuple[float, float, float, float]:
    """(h_angle, v_angle, forward_distance, range) of ``target`` from the camera."""
    fwd = mat_apply(rot, (1.0, 0.0, 0.0))
    left = mat_apply(rot, (0.0, 1.0, 0.0))
    up = mat_apply(rot, (0.0, 0.0, 1.0))
    d = (target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    fd = sum(a * b for a, b in zip(d, fwd))
    rd = -sum(a * b for a, b in zip(d, left))   # +right
    ud = sum(a * b for a, b in zip(d, up))
    # atan2 against the signed forward distance folds "behind the camera" into
    # an angle > 90 deg, so no separate behind-test is needed.
    return math.atan2(rd, fd), math.atan2(ud, fd), fd, math.dist(eye, target)


# Engine defaults when a Viewpoint omits the field (resources/nodes/Viewpoint.wrl).
DEFAULT_EYE: Vec3 = (-10.0, 0.0, 0.0)
DEFAULT_ORIENTATION = (0.0, 0.0, 1.0, 0.0)

# Verdict thresholds. Deliberately loose: this gate exists to catch cameras
# pointed at empty floor, not to police composition.
_SPECK_FILL = 0.015      # below this the subject is < 1.5% of the frame half-height
_TIGHT_FILL = 0.08       # below this it is a distant dot -> borderline
_OVERFLOW_FILL = 2.5     # above this the camera is practically inside the subject


def analyze_viewpoint(path: Path | str, text: str | None = None, *,
                      aspect: float = DEFAULT_ASPECT,
                      repo_root: Path | str | None = None) -> ViewpointVerdict:
    """Frame-check one world. Never raises on malformed input."""
    path = Path(path)
    if text is None:
        text = path.read_text(encoding="utf-8", errors="replace")
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[3]

    try:
        nodes = parse_wbt(text)
    except Exception as exc:                     # pragma: no cover - defensive
        return ViewpointVerdict(world=path, status="borderline", subject_kind="none",
                                reason=f"could not parse world ({exc})")

    proto_index = proto_index_for(repo_root)
    subjects, points = _collect(nodes, proto_index)

    if subjects:
        kind = "robot"
        center, radius = _bounding([s.center for s in subjects],
                                   [s.radius for s in subjects])
    elif points:
        kind = "scene"
        center, radius = _bounding(points, [0.5] * len(points))
    else:
        return ViewpointVerdict(world=path, status="ok", subject_kind="none",
                                reason="no robot and no placed geometry - nothing to frame")

    vp = next((n for n in nodes if n.type_name == "Viewpoint"), None)
    if vp is None:
        return ViewpointVerdict(
            world=path, status="broken", subject_kind=kind, has_viewpoint=False,
            reason=("no Viewpoint node - the engine falls back to position -10 0 0 "
                    "looking down +X"),
            subjects=tuple(subjects), n_subjects=len(subjects), n_in_frame=0,
            angular_miss_deg=180.0, center=center, radius=radius,
            suggestion=_suggest(path, subjects, center, radius, kind, repo_root),
        )

    # Missing fields are not an error: the engine substitutes the schema
    # defaults from resources/nodes/Viewpoint.wrl (position -10 0 0,
    # orientation identity = looking down +X). Evaluate what the user will
    # actually see, which is usually still a miss but occasionally fine.
    eye = vp.floats("position", 3) or DEFAULT_EYE
    orient = vp.floats("orientation", 4) or DEFAULT_ORIENTATION
    fov_v = vp.floats("fieldOfView", 1)
    fov = fov_v[0] if fov_v else DEFAULT_FOV
    follow = vp.string("follow")

    # A dangling `follow` is the silent version of this whole class of bug: the
    # engine resolves it via WbSolid::findSolidFromUniqueName and, when nothing
    # matches, simply does not follow -- with NO warning in the log (verified).
    # So the camera quietly stops tracking and the robot drives out of frame,
    # which is exactly the symptom this check exists to prevent. Note `follow`
    # takes a Solid's `name`, NOT its DEF -- the single most common mistake.
    if follow:
        names = {n.string("name") for n, _depth in iter_tree(nodes) if n.string("name")}
        if follow not in names:
            near = sorted(nm for nm in names if nm and (
                follow.lower() in nm.lower() or nm.lower() in follow.lower()))
            hint = f" Did you mean {near[0]!r}?" if near else ""
            return ViewpointVerdict(
                world=path, status="broken", subject_kind=kind, has_viewpoint=True,
                reason=(f"Viewpoint.follow is {follow!r} but no Solid in this world "
                        f"has that name, so the camera silently never tracks."
                        f"{hint} (follow takes a Solid's `name`, not its DEF)"),
                subjects=tuple(subjects), n_subjects=len(subjects), n_in_frame=0,
                center=center, radius=radius, eye=eye, follow=follow,
                suggestion=(f"set Viewpoint.follow to one of: "
                            f"{', '.join(repr(n) for n in sorted(names)[:6])}"),
            )

    rot = axis_angle_to_matrix(*orient)
    half_h, half_v = _half_angles(fov, aspect)

    ah, av, fd, rng = _angles(eye, rot, center)
    alpha = math.asin(min(1.0, radius / rng)) if rng > radius else math.pi / 2.0
    miss = max(abs(ah) - alpha - half_h, abs(av) - alpha - half_v, 0.0)
    clipped = (abs(ah) + alpha > half_h) or (abs(av) + alpha > half_v)
    fill = alpha / half_v

    n_in = 0
    for s in subjects:
        sh, sv, sfd, srng = _angles(eye, rot, s.center)
        sa = math.asin(min(1.0, s.radius / srng)) if srng > s.radius else math.pi / 2.0
        if abs(sh) - sa <= half_h and abs(sv) - sa <= half_v:
            n_in += 1

    noun = "robot" if kind == "robot" else "scene"
    if miss > 0.0:
        status = "broken"
        reason = (f"{noun} is completely out of frame: "
                  f"{math.degrees(miss):.1f} deg outside the frustum edge "
                  f"(half-FOV {math.degrees(half_h):.1f}x{math.degrees(half_v):.1f} deg)")
    elif rng <= radius:
        # An extreme close-up. The framing radius is an estimate, so this is
        # never a hard failure -- it just means the composition is very tight.
        status = "borderline"
        reason = f"camera sits inside the {noun}'s framing sphere (range {rng:.2f} m)"
    elif fill < _SPECK_FILL:
        status = "broken"
        reason = (f"{noun} is a speck - fills {fill * 100:.1f}% of the frame "
                  f"half-height at {rng:.1f} m")
    elif fill < _TIGHT_FILL:
        status = "borderline"
        reason = f"{noun} is small - fills {fill * 100:.1f}% of the frame half-height"
    elif fill > _OVERFLOW_FILL:
        status = "borderline"
        reason = f"camera is very close - {noun} overflows the frame ({fill:.1f}x)"
    elif clipped:
        status = "borderline"
        reason = f"{noun} is centred but clipped by the frame edge"
    else:
        status = "ok"
        reason = (f"{noun} centred, fills {fill * 100:.0f}% of the frame half-height"
                  + (f" ({n_in}/{len(subjects)} robots in frame)" if len(subjects) > 1 else ""))

    return ViewpointVerdict(
        world=path, status=status, subject_kind=kind, reason=reason,
        subjects=tuple(subjects), n_subjects=len(subjects), n_in_frame=n_in,
        angular_miss_deg=math.degrees(miss), fill_frac=fill,
        center=center, radius=radius, eye=eye, follow=follow,
        suggestion=_suggest(path, subjects, center, radius, kind, repo_root),
    )


def _suggest(path: Path, subjects: list[Subject], center: Vec3, radius: float,
             kind: str, repo_root) -> str:
    """The exact ``set_viewpoint.py`` command that would fix this world."""
    try:
        rel = path.resolve().relative_to(Path(repo_root).resolve()).as_posix()
    except (ValueError, OSError):
        rel = path.as_posix()
    head = f"python scripts/dev/set_viewpoint.py {rel}"
    if kind == "robot" and len(subjects) == 1 and subjects[0].handle:
        return f"{head} --subject {subjects[0].handle} --class {subjects[0].klass}"
    mode = "topdown" if (kind == "scene" or len(subjects) > 2) else "hero"
    return (f"{head} --mode {mode} --center {center[0]:.2f} {center[1]:.2f} "
            f"{center[2]:.2f} --radius {radius:.2f}")


# --------------------------------------------------------------------------
# omniworld validate integration
# --------------------------------------------------------------------------


def check_viewpoint(path: Path, text: str):
    """``omniworld validate`` check: the world must open looking at its subject.

    Only a **robot** subject can fail the check. Scene-extent framing is
    advisory (a "scene" is a proxy built from node translations, not real
    geometry bounds) and worlds with nothing placed always pass.
    """
    from . import CheckResult

    verdict = analyze_viewpoint(path, text)
    if verdict.status == "broken" and verdict.subject_kind == "robot":
        detail = verdict.reason
        if verdict.suggestion:
            detail += f"; fix: {verdict.suggestion}"
        return CheckResult(name="viewpoint_framing", passed=False, detail=detail)
    detail = "" if verdict.status == "ok" else verdict.summary()
    return CheckResult(name="viewpoint_framing", passed=True, detail=detail)


# --------------------------------------------------------------------------
# CLI -- ``python -m omniworld.validation.viewpoint <world.omniworld> ...``
# --------------------------------------------------------------------------

# Worlds the convention deliberately leaves alone. Reframing these would
# destroy the thing they exist to show; see
# docs/developer/viewpoint-convention.md "What is intentionally left alone".
#
# Entries are ``(path fragment, why)``; a world matches if its POSIX path
# contains the fragment.
EXEMPT_FRAGMENTS: tuple[tuple[str, str], ...] = (
    ("generated_worlds/earth_night.wbt",
     "sky showcase - the low-horizon framing IS the subject"),
    ("generated_worlds/mars_night.wbt",
     "sky showcase - the low-horizon framing IS the subject"),
    ("demos/worlds/rendering/",
     "wgpu/lidar render-oracle fixture - the camera is the test instrument"),
)


def is_exempt(path: Path | str) -> str | None:
    """Reason this world is exempt from the framing gate, or ``None``."""
    p = Path(path).as_posix()
    for frag, why in EXEMPT_FRAGMENTS:
        if frag in p:
            return why
    return None


def _iter_worlds(paths, repo_root: Path):
    for raw in paths:
        p = Path(raw)
        if not p.is_absolute():
            p = (repo_root / p) if not p.exists() else p
        if p.is_dir():
            yield from sorted([*p.rglob("*.omniworld"), *p.rglob("*.wbt")])
        elif p.suffix in (".wbt", ".omniworld") and p.is_file():
            yield p


def main(argv=None) -> int:
    import argparse
    import json

    repo_root = Path(__file__).resolve().parents[3]
    ap = argparse.ArgumentParser(
        prog="python -m omniworld.validation.viewpoint",
        description="Check that worlds open looking at their subject "
                    "(docs/developer/viewpoint-convention.md).")
    ap.add_argument("paths", nargs="*", default=["projects", "tests", "distribution"],
                    help="worlds or directories (default: projects tests distribution)")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument("--summary", action="store_true", help="counts only")
    ap.add_argument("--quiet", action="store_true", help="print only failures")
    ap.add_argument("--aspect", type=float, default=DEFAULT_ASPECT)
    ap.add_argument("--fail-on", choices=["never", "broken", "borderline"],
                    default="never", help="exit non-zero on this severity "
                    "(robot-subject worlds only)")
    args = ap.parse_args(argv)

    verdicts = []
    for w in _iter_worlds(args.paths, repo_root):
        exempt = is_exempt(w)
        v = analyze_viewpoint(w, aspect=args.aspect, repo_root=repo_root)
        verdicts.append((v, exempt))

    if args.json:
        print(json.dumps([{
            "world": v.world.as_posix(),
            "status": "exempt" if ex else v.status,
            "subject_kind": v.subject_kind,
            "reason": ex or v.reason,
            "n_subjects": v.n_subjects,
            "n_in_frame": v.n_in_frame,
            "angular_miss_deg": round(v.angular_miss_deg, 3),
            "fill_frac": round(v.fill_frac, 4),
            "has_viewpoint": v.has_viewpoint,
            "follow": v.follow or None,
            "suggestion": v.suggestion,
        } for v, ex in verdicts], indent=2))
        return 0

    counts = {"ok": 0, "borderline": 0, "broken": 0, "exempt": 0}
    failures = []
    for v, ex in verdicts:
        key = "exempt" if ex else v.status
        counts[key] += 1
        bad = (not ex) and v.subject_kind == "robot" and (
            v.status == "broken"
            or (args.fail_on == "borderline" and v.status == "borderline"))
        if bad:
            failures.append(v)
        if args.summary:
            continue
        if args.quiet and not bad:
            continue
        try:
            rel = v.world.resolve().relative_to(repo_root).as_posix()
        except ValueError:
            rel = v.world.as_posix()
        print(f"[{key:10s}] {rel}")
        print(f"             {ex or v.reason}")
        if bad and v.suggestion:
            print(f"             fix: {v.suggestion}")

    total = len(verdicts)
    robot_worlds = [v for v, ex in verdicts if v.subject_kind == "robot" and not ex]
    broken_robot = [v for v in robot_worlds if v.status == "broken"]
    print("")
    print(f"{total} world(s): {counts['ok']} ok, {counts['borderline']} borderline, "
          f"{counts['broken']} broken, {counts['exempt']} exempt")
    if robot_worlds:
        pct = 100.0 * len(broken_robot) / len(robot_worlds)
        print(f"{len(broken_robot)}/{len(robot_worlds)} worlds with a robot open "
              f"with it out of frame ({pct:.1f}%)")
    if args.fail_on != "never" and failures:
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
