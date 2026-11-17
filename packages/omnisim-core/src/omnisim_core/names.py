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

"""Names for an engine-compiled model.

The engine's model has bodies and joints but not the world's names. The world
file (and its URDFs) has the names and where everything stood when it was
written. `world_names` reads the second; `match` joins it to the first by
position and axis, so no engine build rule is re-implemented here; `retarget`
applies what a job's world changes on an already compiled machine.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

from . import worldfile as wf
from .runtime import ensure_runtime
from .worldfile import IDQ, Node, Str, add3, q_axis_angle, q_mul, q_rot, q_rpy

# Controllers that are not a script the core could run.
NO_CONTROLLER = ("<none>", "<generic>", "")

# Device nodes the core does not model. A controller that asks for one is refused, not handed nothing.
UNMODELLED_DEVICES = frozenset((
    "Camera", "RangeFinder", "Lidar", "Radar", "DistanceSensor", "TouchSensor", "GPS", "InertialUnit", "Gyro",
    "Accelerometer", "Compass", "Emitter", "Receiver", "Display", "LED", "Connector", "VacuumGripper", "Pen",
    "Speaker", "Brake", "LightSensor", "Altimeter", "Skin", "Propeller", "Track", "Muscle"))

# A DEF of one of these is part of the physics or of the cut: the core either has a handle for it or refuses.
# A DEF of any other type (Pose, Shape, Group ...) only changes the picture.
PHYSICAL_TYPES = frozenset(("Solid", "Robot", "URDFRobot", "SliderJoint", "HingeJoint", "Hinge2Joint", "BallJoint",
                            "MillingStock", "Physics", "Connector", "Track"))

# Controllers that only move what is drawn (the sun marker follows the sun light). The core does not run them.
INERT_CONTROLLERS = frozenset(("sun_marker",))


def _vec(node, key, n, default):
    v = node.get(key, default)
    return v if isinstance(v, list) and len(v) == n and all(isinstance(x, float) for x in v) else default


def _string(node, key, default=""):
    v = node.get(key, None)
    return str(v[0]) if v else default


def world_names(world_path):
    """Robots, joints, DEF'd nodes and milling stocks, with world positions at the authored pose. Kinematics only."""
    world_path = os.path.abspath(str(world_path)).replace("\\", "/")
    wdir = os.path.dirname(world_path)
    with open(world_path, encoding="utf-8") as f:
        ext, nodes = wf.parse_file(f.read())
    wi = next((n for n in nodes if n.type == "WorldInfo"), None)
    out = {"world": world_path, "dt_ms": (wi.get("basicTimeStep", [32.0])[0] if wi is not None else 32.0),
           "robots": [], "defs": {}, "skipped": [], "milling": [], "all_defs": {}, "extern_controllers": []}

    def note_defs(n):
        if n.defname:
            out["all_defs"].setdefault(n.defname, n.type)
        for c in wf.child_nodes(n):
            note_defs(c)

    def walk(n, ext, base, ppos, pq, moving=False):
        try:
            n, ext, base = wf.expand(n, ext, base)
        except Exception as exc:
            out["skipped"].append("%s: %s" % (n.type, str(exc)[:80]))
            return
        note_defs(n)
        t = _vec(n, "translation", 3, [0.0, 0.0, 0.0])
        r = _vec(n, "rotation", 4, [0.0, 0.0, 1.0, 0.0])
        pos, q = add3(ppos, q_rot(pq, t)), q_mul(pq, q_axis_angle(*r))
        if n.type == "URDFRobot":
            out["robots"].append(urdf_names(n, wdir, pos, q))
            return
        if n.type == "MillingStock":
            g = n.get
            out["milling"].append({
                "def": n.defname, "translation": list(t), "size": list(g("size", [0.1, 0.1, 0.01])),
                "resolution": g("resolution", [0.0002])[0], "toolSolid": _string(n, "toolSolid"),
                "toolTip": list(g("toolTip", [0.0, 0.0, 0.0])), "toolShape": _string(n, "toolShape", "flat"),
                "toolDiameter": g("toolDiameter", [0.006])[0], "toolAngle": g("toolAngle", [1.0472])[0],
                "cutting": bool(g("cutting", [True])[0]), "carrierSolid": _string(n, "carrierSolid")})
            return
        if n.type == "Robot":
            rob = vrml_robot_names(n, pos, q)
            if rob["joints"] or rob["controller"] not in NO_CONTROLLER:
                out["robots"].append(rob)
                return
        dynamic = "physics" in n.fields
        if n.type in ("Solid", "Robot") and n.defname:
            out["defs"][n.defname] = {"pos": list(pos), "dynamic": dynamic}
            if n.type == "Solid" and not dynamic and not moving:
                # a static Solid nothing carries: it stands at its authored pose for the whole run
                out["defs"][n.defname].update(static=True, quat=list(q))
        for c in n.get("children", []):
            if isinstance(c, Node):
                walk(c, ext, base, pos, q, moving or dynamic)

    for n in nodes:
        if n.type not in ("WorldInfo", "Viewpoint"):
            walk(n, ext, wdir, (0.0, 0.0, 0.0), IDQ)
    for rob in out["robots"]:
        if rob["controller"] == "<extern>":
            out["extern_controllers"].append(rob["name"])
    return out


def vrml_robot_names(n, rpos, rq):
    """A Robot written in the world file: its joints (with the motor and sensor names it declares) and its Solids."""
    rob = {"name": _string(n, "name", "robot"), "def": n.defname, "controller": _string(n, "controller", "<generic>"),
           "custom_data": _string(n, "customData"), "args": [str(a) for a in n.get("controllerArgs", [])],
           "root_pos": list(rpos), "joints": [], "solids": {}, "kind": "vrml", "other_devices": [], "other_joints": [],
           "frames": {}}

    def pose(node, ppos, pq):
        t = _vec(node, "translation", 3, [0.0, 0.0, 0.0])
        r = _vec(node, "rotation", 4, [0.0, 0.0, 1.0, 0.0])
        return add3(ppos, q_rot(pq, t)), q_mul(pq, q_axis_angle(*r))

    def devices(node):
        for c in wf.child_nodes(node):
            if c.type in UNMODELLED_DEVICES:
                rob["other_devices"].append(_string(c, "name", c.type))
            if c.type in ("Hinge2Joint", "BallJoint"):
                rob["other_joints"].append(c.type)
            devices(c)

    def _has_physical(node):
        # a frame whose subtree holds nothing physical: writing its pose only changes the picture
        for c in wf.child_nodes(node):
            if c.type in PHYSICAL_TYPES or _has_physical(c):
                return True
        return False

    def frames(node, owner, lp, lq):
        # DEF'd Pose / Transform nodes: a frame fixed to the Solid that owns them (a tool point, a camera mount)
        for c in node.get("children", []):
            if isinstance(c, Node) and c.type in ("Pose", "Transform"):
                t = _vec(c, "translation", 3, [0.0, 0.0, 0.0])
                r = _vec(c, "rotation", 4, [0.0, 0.0, 1.0, 0.0])
                cp, cq = add3(lp, q_rot(lq, t)), q_mul(lq, q_axis_angle(*r))
                if c.defname and owner is not None:
                    rob["frames"][c.defname] = {"solid": owner, "pos": list(cp), "quat": list(cq),
                                                "drawn_only": not _has_physical(c)}
                frames(c, owner, cp, cq)

    def walk(node, pos, q, owner=""):
        frames(node, owner, (0.0, 0.0, 0.0), IDQ)
        for c in node.get("children", []):
            if not isinstance(c, Node):
                continue
            if c.type in ("SliderJoint", "HingeJoint"):
                jp = c.get("jointParameters", [None])[0]
                ep = c.get("endPoint", [None])[0]
                if not isinstance(ep, Node):
                    continue
                slider = c.type == "SliderJoint"
                axis = (jp.get("axis", None) if isinstance(jp, Node) else None) or ([0.0, 0.0, 1.0] if slider else [1.0, 0.0, 0.0])
                anchor = (jp.get("anchor", None) if isinstance(jp, Node) else None) or [0.0, 0.0, 0.0]
                cpos, cq = pose(ep, pos, q)
                an = math.sqrt(sum(x * x for x in axis)) or 1.0
                motor = sensor = None
                for d in c.get("device", []):
                    if isinstance(d, Node) and d.type in ("LinearMotor", "RotationalMotor"):
                        motor = d
                    if isinstance(d, Node) and d.type == "PositionSensor":
                        sensor = d
                sname = _string(ep, "name", "solid")
                j = {"name": sname, "type": "prismatic" if slider else "revolute", "child": sname,
                     "anchor": list(cpos if slider else add3(pos, q_rot(q, anchor))),
                     "axis": list(q_rot(q, (axis[0] / an, axis[1] / an, axis[2] / an))),
                     "vmax": 10.0, "fmax": 10.0, "servo": False, "mimic": False,
                     "local_pos": list(_vec(ep, "translation", 3, [0.0, 0.0, 0.0])), "local_identity": "rotation" not in ep.fields}
                if motor is not None:
                    lo, hi = motor.get("minPosition", [0.0])[0], motor.get("maxPosition", [0.0])[0]
                    j["lo"], j["hi"] = lo, hi
                    j.update(motor=_string(motor, "name", "linear motor" if slider else "rotational motor"),
                             vmax=motor.get("maxVelocity", [10.0])[0],
                             fmax=motor.get("maxForce" if slider else "maxTorque", [10.0])[0], servo=lo != hi)
                if sensor is not None:
                    j["sensor"] = _string(sensor, "name", "position sensor")
                rob["joints"].append(j)
                rob["solids"][sname] = {"def": ep.defname, "joint": len(rob["joints"]) - 1}
                walk(ep, cpos, cq, sname)
            elif c.type == "Solid":
                cpos, cq = pose(c, pos, q)
                walk(c, cpos, cq, None)     # fused into the body above: its frames are not recorded (no offset kept)

    devices(n)
    walk(n, rpos, rq)
    return rob


def urdf_names(n, wdir, rpos, rq):
    path = os.path.normpath(os.path.join(wdir, str(n.get("url")[0])))
    rob = {"name": _string(n, "name", "robot"), "def": n.defname, "controller": _string(n, "controller", "<generic>"),
           "custom_data": _string(n, "customData"), "args": [str(a) for a in n.get("controllerArgs", [])],
           "root_pos": list(rpos), "joints": [], "urdf": path.replace("\\", "/"), "kind": "urdf",
           "other_devices": [], "other_joints": []}
    root = ET.parse(path).getroot()

    def floats(s):
        return [float(x) for x in s.split()]

    joints = []
    for j in root.findall("joint"):
        o, lim, ax, typ = j.find("origin"), j.find("limit"), j.find("axis"), j.get("type")
        L = {"lower": 0.0, "upper": 0.0, "effort": 10.0, "velocity": 10.0}
        if lim is not None:
            L = {k: float(lim.get(k, str(L[k]))) for k in L}
        if typ == "continuous":
            L["lower"] = L["upper"] = 0.0
        joints.append({"name": j.get("name"), "type": typ, "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                       "xyz": tuple(floats(o.get("xyz", "0 0 0"))) if o is not None else (0.0, 0.0, 0.0),
                       "q": q_rpy(*(floats(o.get("rpy", "0 0 0")) if o is not None else [0.0, 0.0, 0.0])),
                       "axis": tuple(floats(ax.get("xyz"))) if ax is not None else (1.0, 0.0, 0.0), "lim": L,
                       "mimic": j.find("mimic") is not None})
    children = {}
    for j in joints:
        children.setdefault(j["parent"], []).append(j)
    links = [link.get("name") for link in root.findall("link")]
    root_link = (set(links) - {j["child"] for j in joints}).pop()

    def go(link, pos, q):
        for j in children.get(link, []):
            p2, q2 = add3(pos, q_rot(q, j["xyz"])), q_mul(q, j["q"])
            if j["type"] in ("revolute", "continuous", "prismatic"):
                a = j["axis"]
                an = math.sqrt(sum(x * x for x in a)) or 1.0
                rob["joints"].append({"name": j["name"], "type": j["type"], "child": j["child"], "anchor": list(p2),
                                      "axis": list(q_rot(q2, (a[0] / an, a[1] / an, a[2] / an))),
                                      "vmax": j["lim"]["velocity"], "fmax": j["lim"]["effort"],
                                      "servo": j["lim"]["lower"] != j["lim"]["upper"], "mimic": j["mimic"]})
            go(j["child"], p2, q2)

    go(root_link, rpos, rq)
    return rob


def match(names, model_path):
    """Give every named joint and DEF its MuJoCo id, by position (and axis) at the model's reference pose."""
    ok, why = ensure_runtime()
    if not ok:
        raise RuntimeError(why)
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_binary_path(str(model_path))
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    mujoco.mj_kinematics(m, d)
    cand = []
    for j in range(m.njnt):
        if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            b = m.jnt_bodyid[j]
            R = d.xmat[b].reshape(3, 3)
            cand.append((j, d.xpos[b] + R @ m.jnt_pos[j], R @ m.jnt_axis[j]))
    used, report = set(), {"joints": 0, "unmatched": [], "worst_mm": 0.0}
    for rob in names["robots"]:
        rob["motors"] = []
        root_bodies = set()
        for jn in rob["joints"]:
            best = None
            for j, anc, ax in cand:
                if j in used:
                    continue
                dv = anc - np.array(jn["anchor"])
                if jn["type"] == "prismatic":            # a slider parked elsewhere along its own axis is the same slider
                    dv = dv - np.dot(dv, ax) * ax
                dist = float(np.linalg.norm(dv))
                al = abs(float(np.dot(ax, np.array(jn["axis"]))))
                if al > 0.99 and (best is None or dist < best[1]):
                    best = (j, dist)
            if best is None or best[1] > 2e-3:
                report["unmatched"].append("%s/%s" % (rob["name"], jn["name"]))
                continue
            used.add(best[0])
            report["joints"] += 1
            report["worst_mm"] = max(report["worst_mm"], best[1] * 1e3)
            j = best[0]
            body = int(m.jnt_bodyid[j])
            root_bodies.add(int(m.body_rootid[body]))
            acts = [a for a in range(m.nu) if m.actuator_trnid[a][0] == j]
            jn["body"] = body
            if len(acts) >= 2 and (rob.get("kind") != "vrml" or "motor" in jn):
                rob["motors"].append({"name": jn.get("motor") or jn["name"] + "_motor", "sensor": jn.get("sensor") or jn["name"] + "_sensor",
                                      "jid": j, "a_pos": acts[0], "a_vel": acts[1], "vmax": jn["vmax"], "fmax": jn["fmax"],
                                      "servo": jn["servo"], "child_link": jn["child"], "body": body})
            elif not acts and (rob.get("kind") != "vrml" or "motor" in jn):
                # an EFFORT joint (the world compiled with OMNISIM_NEWTON_TORQUE_MODE=1): a motor that takes torque only
                rob["motors"].append({"name": jn.get("motor") or jn["name"] + "_motor", "sensor": jn.get("sensor") or jn["name"] + "_sensor",
                                      "jid": j, "a_pos": None, "a_vel": None, "vmax": jn["vmax"], "fmax": jn["fmax"],
                                      "servo": jn["servo"], "effort": True, "child_link": jn["child"], "body": body})
        # a URDF robot with no moving joint: its root is the body nearest its origin
        # (a jointless Robot written in the world file is usually a pure supervisor with no body at all)
        if not root_bodies and m.nbody > 1 and rob.get("kind") != "vrml":
            bd = min(range(1, m.nbody), key=lambda b: float(np.linalg.norm(d.xpos[b] - np.array(rob["root_pos"]))))
            root_bodies = {int(m.body_rootid[bd])}
        rob["root_body"] = sorted(root_bodies)[0] if root_bodies else 0
        rob["bodies"] = [b for b in range(1, m.nbody) if int(m.body_rootid[b]) in root_bodies]
        rob["link_of_body"] = {str(mo["body"]): mo["child_link"] for mo in rob["motors"]}
    # Solids by name (a MillingStock names its cutter's Solid) and by DEF
    names["solid_body"] = {}
    for rob in names["robots"]:
        for sname, info in rob.get("solids", {}).items():
            b = rob["joints"][info["joint"]].get("body")
            if b is not None:
                names["solid_body"][sname] = b
                if info.get("def"):
                    names["defs"].setdefault(info["def"], {"pos": [0, 0, 0], "dynamic": False})["body"] = b
    for ms in names.get("milling", []):
        ms["tool_body"] = names["solid_body"].get(ms["toolSolid"])
    robot_bodies = {b for r in names["robots"] for b in r["bodies"]}
    for info in names["defs"].values():
        if not info["dynamic"]:
            continue
        free = [b for b in range(1, m.nbody) if b not in robot_bodies and m.body_jntadr[b] >= 0]
        if free:
            b = min(free, key=lambda b: float(np.linalg.norm(d.xpos[b] - np.array(info["pos"]))))
            if float(np.linalg.norm(d.xpos[b] - np.array(info["pos"]))) < 2e-3:
                info["body"] = int(b)
    return report


def _sliders(names):
    for rob in names["robots"]:
        if rob.get("kind") != "vrml":
            continue
        for jn in rob["joints"]:
            if jn["type"] == "prismatic" and jn.get("body") is not None:
                yield jn


def retarget_is_exact(names, model_path):
    """True when the model the engine compiled for THIS world holds, bit for bit, what `retarget` would write.

    Only then may another job's slider rest heights and travel limits be written into this model instead of
    asking the engine to compile again.
    """
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_binary_path(str(model_path))
    for jn in _sliders(names):
        b = jn["body"]
        jid = int(m.body_jntadr[b])
        if not jn.get("local_identity", True):
            return False
        if not np.array_equal(m.body_pos[b], np.array(jn["local_pos"], dtype=np.float32).astype(np.float64)):
            return False
        if "lo" in jn and jn["lo"] != jn["hi"]:
            if not np.array_equal(m.jnt_range[jid], np.array([jn["lo"], jn["hi"]], dtype=np.float32).astype(np.float64)):
                return False
    return True


def retarget(names, cdir):
    """Write this job's slider rest positions and travel limits into the machine's model and clamp tables."""
    import mujoco
    import numpy as np
    cdir = str(cdir)
    model_path, clamp_path = os.path.join(cdir, "model.mjb"), os.path.join(cdir, "clamp.npz")
    m = mujoco.MjModel.from_binary_path(model_path)
    changed = []
    clamp = dict(np.load(clamp_path)) if os.path.exists(clamp_path) else None
    for jn in _sliders(names):
        b = jn["body"]
        jid = int(m.body_jntadr[b])
        want = np.array(jn["local_pos"], dtype=np.float32).astype(np.float64)
        if jn.get("local_identity", True) and not np.array_equal(m.body_pos[b], want):
            changed.append("%s rest %s -> %s" % (jn["name"], np.round(m.body_pos[b], 6).tolist(), np.round(want, 6).tolist()))
            m.body_pos[b] = want
        if "lo" in jn and jn["lo"] != jn["hi"]:
            rng = np.array([jn["lo"], jn["hi"]], dtype=np.float32).astype(np.float64)
            if not np.array_equal(m.jnt_range[jid], rng):
                changed.append("%s limits -> %s" % (jn["name"], np.round(rng, 6).tolist()))
                m.jnt_range[jid] = rng
            if clamp is not None:
                k = np.nonzero(clamp["qi"] == m.jnt_qposadr[jid])[0]
                if len(k):
                    clamp["lo"][k[0]] = jn["lo"]
                    clamp["hi"][k[0]] = jn["hi"]
    if changed:
        mujoco.mj_saveModel(m, model_path, None)
        if clamp is not None:
            np.savez(clamp_path, **clamp)
    return changed
