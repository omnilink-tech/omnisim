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

"""roll_probe -- measure whether a wheeled robot's WHEELS account for its motion.

THE DEFECT THIS EXISTS FOR. A hand-authored 4-wheel rover in this repo drove
its body forward at 1.0-1.6 m/s while its four wheel hinges turned at ~0.14
rad/s. At the authored 0.08 m wheel radius, rolling that fast would need 13-20
rad/s. The body was SLIDING -- motor torque going into the chassis through the
joint reaction instead of into the ground through the tyre -- and it had been
that way for the world's whole life, because **every check in this repo asks
only whether the body MOVED**. A headless PASS, a displacement assertion, a
"drove to the goal" verdict: a sliding robot passes all of them.

WHAT IT MEASURES, and why it is a MEASUREMENT and not a formula. Do not try to
predict this from stiction/torque algebra -- it does not survive contact with
the engine. The rover above had 0.4 N.m on each of four wheels against 3.6 kg,
which pencils out to ~5.6 m/s^2 of available traction and *should* have been
plenty; it still failed, and `maxTorque 12` fixed it. The mechanism is not
established, so the check is behavioural:

    v_roll  =  omega_spin * r        (what the wheels would carry the body at)
    v_body  =  body velocity projected on that same direction

and the residual between them is the slip. Nothing here assumes a friction
model, a solver, or a torque budget.

HOW omega_spin IS OBTAINED, and why not from a PositionSensor. Both work, but a
PositionSensor belongs to the robot that owns it and only that robot's own
controller may read it (`/robot/<def>/sensor/<name>` is 501 by design), so a
sensor-based probe would have to be injected into every robot in the corpus.
Instead this reads the wheel Solid's own rigid-body angular velocity through
the supervisor API and subtracts the chassis's:

    omega_spin = (omega_wheel - omega_body) . a_world

with `a_world` the authored hinge axis rotated into world coordinates by the
ROBOT's orientation. Subtracting the chassis is not optional -- a robot yawing
at 2 rad/s would otherwise read 2 rad/s of "wheel spin" while its wheels were
welded solid. One supervisor, appended to a throwaway sibling copy of the
world, measures every robot in the scene without touching any of them.

DIRECTION. Each wheel defines its own forward unit vector

    f = a_world  x  up

(hinge axis crossed with world up: for the ENU-standard `axis 0 1 0` wheel on a
+X-forward robot this gives +X, and it stays correct for a wheel authored on any
other axis or a robot spawned at any yaw). `v_roll = omega_spin * r` is compared
against `v_body . f`, so both sides of the comparison live on the same axis and
the SIGN is meaningful: a wheel spinning backwards relative to the body's
travel is not "nearly rolling", it is 200% slip.

WHAT IT DOES NOT DO. It does not judge. It writes measurements, and
`scripts/dev/roll_check.py` applies the tolerance -- one place for the number,
so the sweep and any future gate cannot drift apart. It also never quits the
simulation: some worlds in the corpus ignore `simulationQuit`, so the caller
bounds every run with a timeout instead (and this probe writes its JSON
incrementally, so a killed run still reports).

ARGS: ``--out=<path>``  where the JSON goes (required to be useful)
      ``--settle-steps=N``  steps to discard before driving is credited
      ``--drive-steps=N``   steps of measurement after settle
      ``--sample-every=N``  basic steps between samples (default 1)
"""

import json
import math
import os
import sys

from omnisim import Supervisor

# Rolling colliders. A wheel authored with a Box collider cannot roll by
# construction; it is reported with radius None rather than silently dropped.
ROLLING_GEOMS = ("Cylinder", "Sphere")
WRAPPERS = ("Pose", "Transform", "Group", "Shape", "Solid")


def arg(name, default):
    prefix = "--%s=" % name
    for item in sys.argv[1:]:
        if item.startswith(prefix):
            return item[len(prefix):]
    return default


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return [x / n for x in a] if n > 1e-12 else [0.0, 0.0, 0.0]


def rotate(orientation, v):
    """Rotate v by a row-major 3x3 orientation matrix (Node.getOrientation)."""
    return [orientation[0] * v[0] + orientation[1] * v[1] + orientation[2] * v[2],
            orientation[3] * v[0] + orientation[4] * v[1] + orientation[5] * v[2],
            orientation[6] * v[0] + orientation[7] * v[1] + orientation[8] * v[2]]


def sf_node(node, field_name):
    field = node.getField(field_name) if node is not None else None
    if field is None:
        return None
    try:
        return field.getSFNode()
    except Exception:  # pragma: no cover - field is not an SFNode
        return None


def mf_nodes(node, field_name):
    field = node.getField(field_name) if node is not None else None
    if field is None:
        return []
    out = []
    try:
        count = field.getCount()
    except Exception:  # pragma: no cover - field is not MF
        return []
    for index in range(count):
        try:
            child = field.getMFNode(index)
        except Exception:  # pragma: no cover
            child = None
        if child is not None:
            out.append(child)
    return out


def sf_float(node, field_name, default=None):
    field = node.getField(field_name) if node is not None else None
    if field is None:
        return default
    try:
        return field.getSFFloat()
    except Exception:  # pragma: no cover
        return default


def sf_vec3(node, field_name, default=None):
    field = node.getField(field_name) if node is not None else None
    if field is None:
        return default
    try:
        return list(field.getSFVec3f())
    except Exception:  # pragma: no cover
        return default


def sf_string(node, field_name, default=None):
    field = node.getField(field_name) if node is not None else None
    if field is None:
        return default
    try:
        return field.getSFString()
    except Exception:  # pragma: no cover
        return default


def collider_radius(node):
    """(geom_type, radius) for a Solid's boundingObject, unwrapping Pose/Shape."""
    current = sf_node(node, "boundingObject")
    for _ in range(8):
        if current is None:
            return (None, None)
        type_name = current.getTypeName()
        if type_name in ROLLING_GEOMS:
            return (type_name, sf_float(current, "radius"))
        if type_name not in WRAPPERS:
            return (type_name, None)
        nxt = sf_node(current, "geometry")
        if nxt is None:
            kids = mf_nodes(current, "children")
            nxt = kids[0] if kids else None
        current = nxt
    return (None, None)


def walk(node, depth=0):
    """Every node reachable from `node` through children/endPoint/device."""
    if node is None or depth > 24:
        return
    yield node
    for field_name in ("children", "device"):
        for child in mf_nodes(node, field_name):
            yield from walk(child, depth + 1)
    for field_name in ("endPoint",):
        child = sf_node(node, field_name)
        if child is not None:
            yield from walk(child, depth + 1)


def find_wheels(robot_node):
    """Driven-wheel hinges under a Robot: (wheel Solid, radius, local axis)."""
    wheels = []
    for node in walk(robot_node):
        if node.getTypeName() != "HingeJoint":
            continue
        devices = mf_nodes(node, "device")
        if not any(d.getTypeName() == "RotationalMotor" for d in devices):
            continue
        end = sf_node(node, "endPoint")
        if end is None:
            continue
        geom, radius = collider_radius(end)
        if geom not in ROLLING_GEOMS or not radius:
            continue
        params = sf_node(node, "jointParameters")
        axis = sf_vec3(params, "axis", [1.0, 0.0, 0.0]) if params else [1.0, 0.0, 0.0]
        wheels.append({
            "node": end,
            "radius": float(radius),
            "axis": axis,
            "def": end.getDef() or None,
            "name": sf_string(end, "name"),
        })
    return wheels


def world_up(supervisor):
    """World up axis from WorldInfo.coordinateSystem (ENU default -> +Z)."""
    children = supervisor.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is not None and node.getTypeName() == "WorldInfo":
            system = (sf_string(node, "coordinateSystem", "ENU") or "ENU").upper()
            # The up axis is the SECOND letter's complement in Webots' naming:
            # ENU = East/North/Up -> +Z; NUE = North/Up/East -> +Y.
            if system == "NUE":
                return [0.0, 1.0, 0.0]
            if system == "EUN":
                return [0.0, 1.0, 0.0]
            return [0.0, 0.0, 1.0]
    return [0.0, 0.0, 1.0]


def main():
    supervisor = Supervisor()
    step_ms = int(supervisor.getBasicTimeStep())
    out_path = arg("out", "roll_probe.json")
    settle_steps = int(arg("settle-steps", "80"))
    drive_steps = int(arg("drive-steps", "400"))
    sample_every = max(1, int(arg("sample-every", "1")))

    up = world_up(supervisor)
    self_node = supervisor.getSelf()

    robots = []
    children = supervisor.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is None:
            continue
        for candidate in walk(node):
            if candidate.getTypeName() != "Robot":
                continue
            if self_node is not None and candidate.getId() == self_node.getId():
                continue
            wheels = find_wheels(candidate)
            if len(wheels) < 2:
                continue
            robots.append({"node": candidate, "wheels": wheels,
                           "def": candidate.getDef() or None,
                           "name": sf_string(candidate, "name")})

    doc = {
        "dt_s": step_ms / 1000.0,
        "settle_steps": settle_steps,
        "drive_steps": drive_steps,
        "sample_every": sample_every,
        "world_up": up,
        "robots": [
            {
                "def": r["def"],
                "name": r["name"],
                "n_wheels": len(r["wheels"]),
                "radius": sorted(w["radius"] for w in r["wheels"])[len(r["wheels"]) // 2],
                "wheels": [{"def": w["def"], "name": w["name"],
                            "radius": w["radius"], "axis": w["axis"]}
                           for w in r["wheels"]],
                "samples": [],
            }
            for r in robots
        ],
        "columns": ["t_s", "v_fwd_mps", "v_roll_mps", "speed_mps",
                    "omega_spin_rad_s", "yaw_rate_rad_s", "x", "y", "z"],
        "complete": False,
    }

    if not robots:
        doc["note"] = ("no robot in this world has >= 2 driven hinge wheels with a "
                       "Cylinder/Sphere boundingObject -- nothing to measure")
        _dump(doc, out_path)
        return

    total = settle_steps + drive_steps
    for k in range(total):
        if supervisor.step(step_ms) == -1:
            break
        if k % sample_every or k < settle_steps:
            continue
        t = supervisor.getTime()
        for slot, r in zip(doc["robots"], robots):
            node = r["node"]
            body_v = node.getVelocity()
            if not body_v or len(body_v) < 6:
                continue
            body_lin = body_v[0:3]
            body_ang = body_v[3:6]
            orientation = node.getOrientation()
            # Per-wheel roll speed, averaged. Averaging is right for a straight
            # drive and is what makes a single number meaningful; per-wheel
            # spread is recoverable because the spin rates are recorded too.
            v_roll_sum = 0.0
            v_fwd_sum = 0.0
            spin_sum = 0.0
            used = 0
            for wheel in r["wheels"]:
                wheel_v = wheel["node"].getVelocity()
                if not wheel_v or len(wheel_v) < 6:
                    continue
                axis_world = unit(rotate(orientation, wheel["axis"]))
                spin = dot([wheel_v[3] - body_ang[0],
                            wheel_v[4] - body_ang[1],
                            wheel_v[5] - body_ang[2]], axis_world)
                forward = unit(cross(axis_world, up))
                if norm(forward) < 1e-9:
                    continue  # hinge axis is vertical: a castor/steer, not a drive wheel
                v_roll_sum += spin * wheel["radius"]
                v_fwd_sum += dot(body_lin, forward)
                spin_sum += spin
                used += 1
            if not used:
                continue
            position = node.getPosition()
            slot["samples"].append([
                round(t, 4),
                round(v_fwd_sum / used, 6),
                round(v_roll_sum / used, 6),
                round(norm(body_lin), 6),
                round(spin_sum / used, 6),
                round(dot(body_ang, up), 6),
                round(position[0], 4), round(position[1], 4), round(position[2], 4),
            ])
        if k % 200 == 0:
            _dump(doc, out_path)  # incremental: a killed run still reports

    doc["complete"] = True
    _dump(doc, out_path)


def _dump(doc, out_path):
    try:
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(doc, handle)
    except OSError:
        pass


main()
