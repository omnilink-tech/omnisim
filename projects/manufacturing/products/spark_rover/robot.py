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

"""SparkRover Mini as a robot: the four subsystems joined into an OmniSim world that drives on Newton.

Everything comes from the design (integrate.load(): shell, chassis, arms, electrical):

- the BODIES: the base (chassis, shell, electronics, the arms' static brackets, posts and
  shoulder horns), four wheels (tyre and hub, the hex adapter, its M4 screw, the motor's
  output shaft) and, per arm, the upper arm and the forearm (arms.links()' parts: each link
  carries the servo of the joint above it);
- the MASS: every part's mass_g at its centre_mm (parts()), each part's inertia from its own
  solid (uniform density at the part's mass), summed about each body's centre of mass;
  parts drawn without a solid are point masses, and the battery is a box at its volume;
- the VISUALS: every part's solid (solids()), merged per body and colour into meshes/sim/;
- the COLLISIONS: the tyres as cylinders (65 x 26 mm, chassis.BOUGHT["wheel"]), the body as
  the skeleton's three outer boxes (lower body, upper body, top slot), each arm link as its
  parts' box in its own frame;
- the JOINTS: four wheel hinges, each driven by the Pololu #4844's DC line (34:1, 12 V
  rated) applied by the controller every step at the pack's nominal voltage
  (electrical.PACK["v_nom"]): torque = stall x |1 - speed / no-load speed|, braking above
  no-load speed; and the four arm hinges at arms.joints()' limits, with the XC330-M288-T's
  stall torque and no-load speed at its 5.0 V rail (0.93 N m, 81 rpm: ROBOTIS e-manual,
  https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/, as arms.SERVO records it);
- the FACE: U9's camera ("camera") at its lens and U8's screen ("display") on its active area
  (face_specs(), below). The mics are not simulated: OmniSim retired its Microphone node.

The battery: its collider is electrical.PACK_VOLUME's box, and its 200 g sits at that box's
centre, as electrical.parts() has put it since 2026-11-04 (before, at its lead exit,
(-30, -36, 45)).

    python projects/manufacturing/products/spark_rover/robot.py
        # writes meshes/sim/ and projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pickle
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import skeleton as sk  # noqa: E402

ROOT = HERE.parents[3]
MANUFACTURING = HERE.parents[1]
MESHES = HERE / "meshes" / "sim"
WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld"
CACHE = ROOT / ".tmp" / "spark_rover_robot"
KGCM = 0.0980665
G = 9.81
WHEEL_TAGS = {"fl": "front_left", "fr": "front_right", "rl": "rear_left", "rr": "rear_right"}
SIDES = {"left": 1, "right": -1}
SOURCES = ("skeleton.py", "integrate.py", "shell.py", "chassis.py", "arms.py", "electrical.py")
DESIGN_SCHEMA = "face-devices-1"      # what design() returns; a change re-reads the design


def _f(v: float) -> str:
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _v(*vals: float) -> str:
    return " ".join(_f(v) for v in vals)


# ---- the design, loaded once (the shell's solids take ~30 s to build) --------------------------

def _key() -> str:
    h = hashlib.sha256()
    desk = HERE.parent / "desk_robot"
    for p in [HERE / s for s in SOURCES] + [desk / "design.py", desk / "bought_parts.py"]:
        if p.exists():
            h.update(p.read_bytes())
    h.update(DESIGN_SCHEMA.encode())
    return h.hexdigest()[:16]


def design(use_cache: bool = True) -> dict:
    """Every part of the robot with its owner, mass, centre and solid, and the joints."""
    CACHE.mkdir(parents=True, exist_ok=True)
    f = CACHE / f"design_{_key()}.pkl"
    if use_cache and f.exists():
        with f.open("rb") as fh:
            return pickle.load(fh)
    sys.path.insert(0, str(HERE))
    import integrate
    mods = integrate.load()
    wrong = {k: m.__file__ for k, m in mods.items() if Path(m.__file__).resolve().parent != HERE}
    if wrong:      # a same-named module (desk_robot's electrical.py) imported earlier in this process
        raise RuntimeError(f"integrate.load() returned modules from elsewhere: {wrong}")
    ch, arms, el = mods["chassis"], mods["arms"], mods["electrical"]
    parts, solids = [], {}
    for owner, mod in mods.items():
        sol = dict(mod.solids(None))
        if owner == "chassis":
            sol.update(ch.interface_solids(None))       # the interface screws are drawn there
        for p in mod.parts(None):
            parts.append(dict(p, owner=owner))
        solids.update({k: np.asarray(v, float) for k, v in sol.items()})
    d = {"parts": parts, "solids": solids,
         "arm_joints": arms.joints(None), "arm_links": arms.links(None), "arm": {
             "L1": arms.A.L1, "fore_len": arms.A.fore_len, "servo": {k: arms.SERVO[k] for k in (
                 "spec", "stall_nm", "no_load_rpm", "stall_a", "kt_nm_per_a", "source")}},
         "gearmotor": dict(ch.BOUGHT["gearmotor"]), "wheel": dict(ch.BOUGHT["wheel"]),
         "chassis": {"mu_floor": ch.Chassis().mu_floor, "crr": ch.Chassis().crr, "battery_v": ch.Chassis().battery_v},
         "drive": ch.drive(None),
         "pack": {k: el.PACK[k] for k in ("v_full", "v_nom", "v_end", "g", "outline_mm", "series", "parallel")},
         "pack_box": [el.PACK_VOLUME["box"].x0, el.PACK_VOLUME["box"].x1, el.PACK_VOLUME["box"].y0,
                      el.PACK_VOLUME["box"].y1, el.PACK_VOLUME["box"].z0, el.PACK_VOLUME["box"].z1],
         "el_motor": dict(el.MOTOR), "integrate_mass": integrate.mass(mods),
         "spin_budget": {s: el.operating_point("spin", el.PACK[k]) for s, k in
                         (("full", "v_full"), ("nominal", "v_nom"), ("end", "v_end"))}}
    for v in d["spin_budget"].values():
        v.pop("p5_items", None)
    d["face"] = face_specs(el)
    with f.open("wb") as fh:
        pickle.dump(d, fh)
    return d


# ---- bodies ------------------------------------------------------------------------------------

def body_of(p: dict) -> str:
    """Which rigid body of the simulated robot a part rides on."""
    lab = p["label"]
    if p["owner"] == "chassis":
        for t in WHEEL_TAGS:
            if lab in (f"wheel_{t}", f"hex_adapter_{t}", f"screw_wheel_{t}", f"motor_{t}_shaft"):
                return f"wheel_{t}"
    if p["owner"] == "arms":
        link = p.get("link", "body")
        if link.startswith("upper_"):
            return "upper_arm_" + link.split("_", 1)[1]
        if link.startswith("fore_"):
            return "forearm_" + link.split("_", 1)[1]
    return "base"


def _arm_anchor(d: dict, joint: str) -> np.ndarray:
    return np.asarray(next(j for j in d["arm_joints"] if j["name"] == joint)["anchor_mm"], float)


def body_origins(d: dict) -> Dict[str, np.ndarray]:
    """Each body's frame origin in the robot frame (mm): the base at the robot origin, a wheel
    on its axle, an arm link on its joint's axis (arms.joints()' anchors). No body is rotated
    at the neutral pose."""
    out = {"base": np.zeros(3)}
    for t, name in WHEEL_TAGS.items():
        out[f"wheel_{t}"] = np.asarray(sk.WHEELS[name], float)
    for side in SIDES:
        out[f"upper_arm_{side}"] = _arm_anchor(d, f"shoulder_{side}")
        out[f"forearm_{side}"] = _arm_anchor(d, f"elbow_{side}")
    return out


def battery_box(d: dict) -> np.ndarray:
    x0, x1, y0, y1, z0, z1 = d["pack_box"]
    return box_triangles((x0, y0, z0), (x1, y1, z1))


def box_triangles(lo, hi) -> np.ndarray:
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.asarray(f)]


def part_table(d: dict) -> List[dict]:
    """Every part: label, owner, step, body, mass (g), centre (mm, robot frame), its solid (or
    None) and its colour. The battery is moved to its volume (see the module docstring)."""
    out = []
    for p in d["parts"]:
        lab = p["label"]
        tris = d["solids"].get(lab)
        if lab == "shoulder_post_left" or lab == "shoulder_post_right":
            side = lab.rsplit("_", 1)[1]
            tris = np.concatenate([d["solids"][f"shoulder_post_{side}_in_body"], d["solids"][f"shoulder_post_{side}_outside"]])
        centre = np.asarray(p.get("centre_mm") or (0, 0, 0), float)
        note = ""
        if lab == "el_BT1":
            tris = battery_box(d)
            x0, x1, y0, y1, z0, z1 = d["pack_box"]
            note = f"the pack's volume (parts() gives {tuple(centre)})"
            centre = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        out.append({"label": lab, "owner": p["owner"], "step": p["step"], "body": body_of(p),
                    "mass_g": float(p.get("mass_g") or 0.0), "centre_mm": centre, "tris": tris,
                    "rgb": colour(p), "transparency": 0.75 if p.get("colour") == "clear" else 0.0, "note": note,
                    "kind": p.get("kind"), "material": p.get("material")})
    return out


def colour(p: dict) -> Tuple[float, float, float]:
    c = p.get("colour")
    if isinstance(c, str) and c.startswith("#") and len(c) == 7:
        return tuple(int(c[i:i + 2], 16) / 255.0 for i in (1, 3, 5))
    if isinstance(c, str) and len(c.split()) == 3:
        try:
            return tuple(float(x) for x in c.split())
        except ValueError:
            pass
    if c == "clear":
        return (0.85, 0.92, 0.95)
    if isinstance(c, str) and "natural" in c:
        return (0.93, 0.93, 0.90)
    m = (p.get("material") or "").lower()
    lab = p["label"]
    if "plywood" in m:
        return (0.84, 0.70, 0.50)
    if "rubber" in m:
        return (0.07, 0.07, 0.07)
    if "brass" in m:
        return (0.78, 0.62, 0.30)
    if "steel" in m:
        return (0.55, 0.56, 0.58)
    if "aluminium" in m:
        return (0.80, 0.81, 0.83)
    if m == "mixed":
        return (0.62, 0.62, 0.64)
    if "nylon" in m:
        return (0.10, 0.10, 0.11)
    if "petg" in m:
        return (0.35, 0.37, 0.40)
    if lab == "el_BT1":
        return (0.16, 0.32, 0.62)
    if lab.startswith("el_"):
        return (0.10, 0.36, 0.22)          # a board
    return (0.5, 0.5, 0.5)


# ---- mass properties ---------------------------------------------------------------------------

def _solid_inertia(tris: Optional[np.ndarray], mass_kg: float, centre_mm: np.ndarray) -> np.ndarray:
    """The part's inertia (kg m^2, robot axes) about `centre_mm`: its solid at uniform density
    (trimesh), shifted from the solid's centroid to the stated centre. No solid: a point."""
    if tris is None or mass_kg <= 0 or len(tris) == 0:
        return np.zeros((3, 3))
    import trimesh
    mp = trimesh.triangles.mass_properties(np.asarray(tris, float), density=1.0)
    vol = float(mp["volume"])
    if not np.isfinite(vol) or vol <= 1e-6:
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        a, b, c = (hi - lo) * 1e-3
        I = mass_kg / 12 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
        cs = (lo + hi) / 2
    else:
        I = np.asarray(mp["inertia"], float) * (mass_kg / vol) * 1e-6
        cs = np.asarray(mp["center_mass"], float)
    dd = (cs - centre_mm) * 1e-3
    return I + mass_kg * (dd @ dd * np.eye(3) - np.outer(dd, dd))


def bodies(d: dict) -> Dict[str, dict]:
    """Each body: mass (kg), centre of mass and inertia about it (robot axes), in the body's
    own frame (metres), and its parts."""
    origins = body_origins(d)
    out: Dict[str, dict] = {}
    for p in part_table(d):
        out.setdefault(p["body"], {"parts": []})["parts"].append(p)
    for name, b in out.items():
        ps = b["parts"]
        M = sum(p["mass_g"] for p in ps) / 1000.0
        com_mm = sum(p["mass_g"] * p["centre_mm"] for p in ps) / (M * 1000.0)
        I = np.zeros((3, 3))
        for p in ps:
            m = p["mass_g"] / 1000.0
            I += _solid_inertia(p["tris"], m, p["centre_mm"])
            dd = (p["centre_mm"] - com_mm) * 1e-3
            I += m * (dd @ dd * np.eye(3) - np.outer(dd, dd))
        b.update(mass_kg=M, com_mm=com_mm, origin_mm=origins[name],
                 com_local_m=(com_mm - origins[name]) * 1e-3, inertia_kgm2=I)
    return out


def mass_report(d: dict) -> dict:
    bs = bodies(d)
    total = sum(b["mass_kg"] for b in bs.values())
    com = sum(b["mass_kg"] * b["com_mm"] for b in bs.values()) / total
    im = d["integrate_mass"]
    return {"total_g": round(total * 1000, 1), "com_mm": [round(float(v), 2) for v in com],
            "integrate_total_g": im["total_g"], "integrate_com_mm": im["com_mm"],
            "battery_note": "the 200 g pack at its volume's centre, as integrate has it",
            "bodies": {k: {"mass_g": round(b["mass_kg"] * 1000, 2), "com_mm": [round(float(v), 2) for v in b["com_mm"]],
                           "parts": len(b["parts"])} for k, b in sorted(bs.items())}}


# ---- meshes ------------------------------------------------------------------------------------

def write_stl(path: Path, tris: np.ndarray, name: str = "omnisim") -> None:
    """Binary STL (whatever units `tris` is in: metres here)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tris = np.asarray(tris, float)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with path.open("wb") as f:
        f.write(name.encode()[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def _cname(rgb, tr) -> str:
    return "c" + "".join(f"{int(round(c * 255)):02x}" for c in rgb) + ("t" if tr else "")


def visual_groups(d: dict) -> Dict[str, List[dict]]:
    """Per body, the merged meshes by colour: [{file, rgb, transparency, tris (m, body frame)}]."""
    origins = body_origins(d)
    groups: Dict[Tuple[str, str], dict] = {}
    for p in part_table(d):
        if p["tris"] is None:
            continue
        key = (p["body"], _cname(p["rgb"], p["transparency"]))
        g = groups.setdefault(key, {"rgb": p["rgb"], "transparency": p["transparency"], "tris": []})
        g["tris"].append((p["tris"] - origins[p["body"]]) * 1e-3)
    out: Dict[str, List[dict]] = {}
    for (body, cn), g in sorted(groups.items()):
        out.setdefault(body, []).append({"file": f"{body}__{cn}.stl", "rgb": g["rgb"], "transparency": g["transparency"],
                                         "tris": np.concatenate(g["tris"])})
    return out


def write_meshes(d: dict, out: Path = MESHES) -> Dict[str, List[dict]]:
    out.mkdir(parents=True, exist_ok=True)
    vg = visual_groups(d)
    keep = set()
    for body, gs in vg.items():
        for g in gs:
            write_stl(out / g["file"], g["tris"], f"sparkrover {body}")
            keep.add(g["file"])
    for f in out.glob("*.stl"):
        if f.name not in keep:
            f.unlink()
    return vg


# ---- the motor model -----------------------------------------------------------------------------

def motor_at(gm: dict, v: float) -> dict:
    """The Pololu #4844's line at v volts (speed, stall torque and stall current proportional to
    v: chassis.motor_at), at the wheel."""
    k = v / gm["rated_v"]
    return {"v": v, "no_load_rad_s": gm["no_load_rpm"] * k * 2 * math.pi / 60, "stall_nm": gm["stall_kgcm"] * k * KGCM,
            "stall_a": gm["stall_a"] * k, "no_load_a": gm["no_load_a"]}


def current_at(gm: dict, v: float, w: float) -> float:
    """The motor current at wheel speed w on the controller's line (torque = stall x
    (1 - w / no-load)), with chassis.operating's current model: I0 + (Is - I0) x torque fraction."""
    m = motor_at(gm, v)
    f = abs(1.0 - abs(w) / m["no_load_rad_s"])
    return m["no_load_a"] + (m["stall_a"] - m["no_load_a"]) * f


def predictions(d: dict, v: float = None) -> dict:
    """What the design predicts the drive test will measure, at the pack's nominal voltage, or at
    v (the bus voltage the battery model gave during the run: the drive result's pack_v_mean)."""
    gm, v = d["gearmotor"], (d["pack"]["v_nom"] if v is None else v)
    m = motor_at(gm, v)
    rw = sk.WHEEL_D / 2000.0
    mass = mass_report(d)["total_g"] / 1000.0
    W = mass * G
    out = {"pack_v_nom": v, "wheel_radius_mm": sk.WHEEL_D / 2, "track_mm": sk.TRACK, "wheelbase_mm": sk.WHEELBASE,
           "mass_g": round(mass * 1000, 1), "no_load_rad_s": round(m["no_load_rad_s"], 4),
           "stall_nm": round(m["stall_nm"], 4), "stall_a": round(m["stall_a"], 3),
           "no_load_speed_m_s": round(m["no_load_rad_s"] * rw, 4)}
    crr = d["chassis"]["crr"]
    t_cruise = crr * W / 4 * rw
    w_cruise = m["no_load_rad_s"] * (1 - t_cruise / m["stall_nm"])
    out["cruise_speed_m_s_crr"] = {"crr": crr, "m_s": round(w_cruise * rw, 4),
                                   "note": "with chassis.drive()'s rolling resistance; the simulated tyre is a rigid cylinder (no rolling resistance)"}
    for mu in sorted(set(d["chassis"]["mu_floor"])):
        t = mu * W * sk.WHEELBASE / (4 * sk.TRACK) * rw
        f = t / m["stall_nm"]
        w = m["no_load_rad_s"] * (1 - f)
        i = m["no_load_a"] + (m["stall_a"] - m["no_load_a"]) * f
        out[f"spin_mu_{mu:g}"] = {"torque_nm": round(t, 4), "torque_frac": round(f, 4), "wheel_rad_s": round(w, 3),
                                  "current_a": round(i, 3), "current_frac_of_stall": round(i / m["stall_a"], 4),
                                  "yaw_rate_rad_s_if_no_slip": round(2 * w * rw / (sk.TRACK / 1000.0), 3)}
    out["design_spin_at_2400g_7.2V"] = d["drive"]["operating"]["spin_mu_hi"]["7.2V"]
    return out


# ---- the face's devices: U9's camera and U8's screen ---------------------------------------------
#
# The Raspberry Pi Camera Module 3 (standard lens): 66 deg horizontal and 41 deg vertical field
# of view, a Sony IMX708 of 4608 x 2592 (11.9 MP, 16:9), focus 10 cm to infinity
# (electrical.CAMERA_MOD["source"], the product brief). The simulated camera keeps the 66 deg
# and the 16:9, at 640 x 360 so a frame stays cheap: 2 atan(tan(33 deg) x 9/16) = 40.1 deg
# vertical, against the brief's 41 deg for the full sensor.
#
# The mount: electrical.placements()' U9 is the board turned 180 deg about the lens axis from
# the head's other boards (FPC edge DOWN, eu -y, ev -z). The firmware sets the camera's
# rotation for that mount (rpicam --rotation 180, Picamera2 Transform(hflip=1, vflip=1): the
# IMX708 flips on the sensor, at no cost), so the frame it delivers is upright. The simulated
# camera IS that delivered frame: it looks along the lens axis (+x) with the robot's +z up,
# whatever way round the board sits. Which way up the bare module's image comes out in this
# mount was not checked against a real module; if it comes out inverted the fix is that
# rotation setting, not the hardware.
#
# The Waveshare 5" DSI panel (50H-800480-IPS): 800 x 480 pixels on a 108 x 64.8 mm active area
# (electrical.DISPLAY, Waveshare's page), 0.135 mm a pixel both ways. The simulated screen is a
# Display device drawing onto a quad of exactly that area, centred on the module's front face
# (electrical.checks()' screen_active_area_centred_in_face_window, ASSUMED there) and
# DISPLAY_GAP_MM in front of it, behind the 1.5 mm acrylic face cover.

CAMERA_SIM = {"name": "camera", "width": 640, "height": 360, "near_m": 0.01, "far_m": 10.0,
              "note": "near 0.01 m clips the clear face cover 4.2-5.7 mm ahead of the lens: the lens "
                      "focuses from 10 cm and cast acrylic passes ~92 %, where the cover's visual is drawn "
                      "75 % transparent"}
CAMERA_SENSOR_PX = (4608, 2592)        # IMX708, the CM3 product brief
CAMERA_LENS_TOP_MM = 8.9               # the lens top above the board's back face (electrical.checks()' lens point)
DISPLAY_PX = (800, 480)                # Waveshare 50H-800480-IPS
DISPLAY_SIM = {"name": "display"}
DISPLAY_GAP_MM = 0.2                   # the screen's quad in front of the module's solid (no z-fighting)


def face_specs(el) -> dict:
    """The camera's and the screen's poses and specs, from electrical (placements(), CAMERA_MOD,
    DISPLAY), in the robot frame (mm)."""
    pl = el.placements(None)
    cam, cm = pl["U9"], el.CAMERA_MOD
    lens = cam.at(cm["board_mm"][0] / 2, cm["lens_from_far_edge"], CAMERA_LENS_TOP_MM)
    scr, dp = pl["U8"], el.DISPLAY
    dw, dh, dt = dp["outline_mm"]
    front = np.asarray(scr.at(dw / 2, dh / 2, dt), float)
    normal = scr.R[:, 2]
    return {
        "camera": {"lens_mm": [float(v) for v in lens], "axis": [float(v) for v in cam.R[:, 2]],
                   "board_v": [float(v) for v in cam.R[:, 1]], "fov_deg": list(cm["fov_deg"]),
                   "sensor_px": list(CAMERA_SENSOR_PX), "source": cm["source"]},
        "display": {"centre_mm": [float(v) for v in front + DISPLAY_GAP_MM * normal],
                    "normal": [float(v) for v in normal], "active_mm": list(dp["active_mm"]),
                    "outline_mm": list(dp["outline_mm"]), "px": list(DISPLAY_PX), "source": dp["source"]}}


def camera_node(face: dict) -> str:
    c = face["camera"]
    if not np.allclose(c["axis"], (1, 0, 0)) or abs(c["board_v"][2]) < 0.999:
        raise ValueError(f"the camera node assumes the lens looks +x with the board upright or inverted: {c}")
    s = CAMERA_SIM
    return (f'Camera {{ name "{s["name"]}" translation {_v(*(np.asarray(c["lens_mm"]) * 1e-3))} '
            f'fieldOfView {_f(math.radians(c["fov_deg"][0]))} width {s["width"]} height {s["height"]} '
            f'near {_f(s["near_m"])} far {_f(s["far_m"])} }}')


def display_node(face: dict) -> str:
    d = face["display"]
    if not np.allclose(d["normal"], (1, 0, 0)):
        raise ValueError(f"the display node assumes the screen faces +x: {d}")
    w, h = (v * 1e-3 / 2 for v in d["active_mm"])
    # Display pixel (0, 0) is the top left as seen from outside, looking at the robot (-x): the
    # viewer's right is the robot's +y, so u runs -y -> +y and v bottom -> top.
    pts = [(0, -w, h), (0, -w, -h), (0, w, -h), (0, w, h)]
    # The screen is a lit texture, not an emitter: the wgpu renderer has no emissiveColorMap, and a
    # constant emissiveColor would add the same white to every pixel and wash the face out.
    return (f'Display {{ name "{DISPLAY_SIM["name"]}" width {d["px"][0]} height {d["px"][1]} '
            f'translation {_v(*(np.asarray(d["centre_mm"]) * 1e-3))} children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor 1 1 1 baseColorMap ImageTexture {{ filtering 0 }} '
            f'roughness 1 metalness 0 }} '
            f'geometry IndexedFaceSet {{ coord Coordinate {{ point [ {", ".join(_v(*p) for p in pts)} ] }} '
            f'texCoord TextureCoordinate {{ point [ 0 1, 0 0, 1 0, 1 1 ] }} '
            f'coordIndex [ 0 1 2 3 -1 ] texCoordIndex [ 0 1 2 3 -1 ] }} castShadows FALSE }} ] }}')


# ---- the world ----------------------------------------------------------------------------------

def _linear(c: float) -> float:
    """sRGB (the design's colours, e.g. shell.MADE's #5A5F66) to the linear value PBRAppearance reads."""
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _shape(url: str, rgb, tr: float = 0.0, n_tris: int = 0) -> str:
    """A part group's Shape. rgb is the design's sRGB colour (it also names the mesh file); the
    renderer takes baseColor as linear, and written as sRGB every part rendered lighter than its
    design colour: the dark grey lower shell and bumpers came out light grey (until 2026-11-04)."""
    rough = 0.5
    t = f" transparency {_f(tr)}" if tr else ""
    shadow = " castShadows FALSE" if n_tris > 21845 else ""     # the renderer's shadow-volume limit
    lin = tuple(round(_linear(c), 4) for c in rgb)
    return (f'Shape {{ appearance PBRAppearance {{ baseColor {_v(*lin)} roughness {rough} metalness 0{t} }} '
            f'geometry Mesh {{ url "{url}" }}{shadow} }}')


def _physics(b: dict) -> str:
    I = b["inertia_kgm2"]
    return (f"physics Physics {{ density -1 mass {_f(b['mass_kg'])} centerOfMass [ {_v(*b['com_local_m'])} ] "
            f"inertiaMatrix [ {I[0, 0]:.6g} {I[1, 1]:.6g} {I[2, 2]:.6g} {I[0, 1]:.6g} {I[0, 2]:.6g} {I[1, 2]:.6g} ] }}")


def _box_bo(lo_mm, hi_mm) -> str:
    lo, hi = np.asarray(lo_mm, float) * 1e-3, np.asarray(hi_mm, float) * 1e-3
    return f"Pose {{ translation {_v(*((lo + hi) / 2))} children [ Box {{ size {_v(*(hi - lo))} }} ] }}"


def base_collision_boxes() -> List[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """The skeleton's outer boxes: lower body, upper body, top slot (mm, robot frame)."""
    out = []
    for b in (sk.LOWER_BODY, sk.UPPER_BODY, sk.TOP_SLOT):
        out.append(((b.x0, b.y0, b.z0), (b.x1, b.y1, b.z1)))
    return out


def link_box(d: dict, body: str) -> Tuple[np.ndarray, np.ndarray]:
    """An arm link's parts' bounding box in the link's own frame (mm), at the neutral pose."""
    o = body_origins(d)[body]
    v = np.concatenate([p["tris"].reshape(-1, 3) for p in part_table(d) if p["body"] == body and p["tris"] is not None])
    return v.min(0) - o, v.max(0) - o


def robot_node(d: dict, mesh_url: str, controller: str = "spark_rover_drive", args: Sequence[str] = (),
               name: str = "SPARK_ROVER", translation=(0.0, 0.0, 0.0)) -> str:
    bs = bodies(d)
    vg = visual_groups(d)
    gm = d["gearmotor"]
    mmax = motor_at(gm, d["pack"]["v_full"])          # the joint's own limits: the motor at a full pack
    o = body_origins(d)

    def vis(body):
        return " ".join(_shape(f"{mesh_url}/{g['file']}", g["rgb"], g["transparency"], len(g["tris"])) for g in vg.get(body, []))

    kids = [vis("base"), camera_node(d["face"]), display_node(d["face"])]
    w = d["wheel"]
    for t in WHEEL_TAGS:
        body = f"wheel_{t}"
        a = o[body] * 1e-3
        kids.append(
            f"HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor {_v(*a)} }}\n"
            f'  device [ RotationalMotor {{ name "wheel_{t}_motor" maxVelocity {_f(mmax["no_load_rad_s"])} '
            f'maxTorque {_f(mmax["stall_nm"])} }} PositionSensor {{ name "wheel_{t}_sensor" }} ]\n'
            f'  endPoint Solid {{ name "{body}" translation {_v(*a)}\n'
            f"    children [ {vis(body)} ]\n"
            f"    boundingObject Pose {{ rotation 1 0 0 1.5708 children [ Cylinder {{ radius {_f(w['od'] / 2000)} "
            f"height {_f(w['width'] / 1000)} subdivision 48 }} ] }}\n"
            f"    {_physics(bs[body])} }} }}")
    for side in SIDES:
        js = {j["name"]: j for j in d["arm_joints"]}
        sh, el = js[f"shoulder_{side}"], js[f"elbow_{side}"]
        ua, fa = f"upper_arm_{side}", f"forearm_{side}"
        rel = (o[fa] - o[ua]) * 1e-3

        def motor(j):
            lo, hi = j["limits_rad"]
            return (f'device [ RotationalMotor {{ name "{j["name"]}" minPosition {_f(lo)} maxPosition {_f(hi)} '
                    f'maxTorque {_f(j["max_torque_nm"])} maxVelocity {_f(j["max_velocity_rad_s"])} }} '
                    f'PositionSensor {{ name "{j["name"]}_sensor" }} ]')
        lo_f, hi_f = link_box(d, fa)
        lo_u, hi_u = link_box(d, ua)
        fore = (f"HingeJoint {{ jointParameters HingeJointParameters {{ axis {_v(*el['axis'])} anchor {_v(*rel)} }}\n"
                f"      {motor(el)}\n"
                f'      endPoint Solid {{ name "{fa}" translation {_v(*rel)} children [ {vis(fa)} ]\n'
                f"        boundingObject {_box_bo(lo_f, hi_f)}\n        {_physics(bs[fa])} }} }}")
        kids.append(
            f"HingeJoint {{ jointParameters HingeJointParameters {{ axis {_v(*sh['axis'])} anchor {_v(*(o[ua] * 1e-3))} }}\n"
            f"  {motor(sh)}\n"
            f'  endPoint Solid {{ name "{ua}" translation {_v(*(o[ua] * 1e-3))} children [ {vis(ua)}\n      {fore} ]\n'
            f"    boundingObject {_box_bo(lo_u, hi_u)}\n    {_physics(bs[ua])} }} }}")
    bo = "Group { children [ " + " ".join(_box_bo(lo, hi) for lo, hi in base_collision_boxes()) + " ] }"
    arg_list = " ".join(f'"{a}"' for a in args)
    return (f'DEF {name} Robot {{ name "spark-rover" translation {_v(*translation)} controller "{controller}" '
            f"controllerArgs [ {arg_list} ] supervisor TRUE\n"
            f"  children [\n    " + "\n    ".join(kids) + " ]\n"
            f"  boundingObject {bo}\n  {_physics(bs['base'])} }}")


START_X = -1.0          # the robot starts here and drives +x: 1 m and more fits the 3 m floor
TARGET_X = 0.7          # the camera's target: a board 1.7 m ahead of the start, 0.6 m past the stop


def target_node(x: float = TARGET_X) -> str:
    """A white board facing the robot with a red square high on the robot's left (+y) and a blue
    one low on its right (-y): in an upright, unmirrored frame, red is top left and blue bottom
    right. Visual only (no boundingObject, no physics): nothing the drive test touches."""
    def sq(y, z, rgb):
        return (f"Pose {{ translation {_v(-0.011, y, z)} children [ Shape {{ appearance PBRAppearance "
                f"{{ baseColor {_v(*rgb)} roughness .9 metalness 0 }} geometry Box {{ size .002 .12 .12 }} }} ] }}")
    return (f'DEF TARGET Solid {{ name "camera target" translation {_v(x, 0, 0)} children [ '
            "Pose { translation 0 0 .3 children [ Shape { appearance PBRAppearance { baseColor .95 .95 .95 "
            "roughness .9 metalness 0 } geometry Box { size .02 .5 .6 } } ] } "
            f"{sq(0.14, 0.48, (0.85, 0.1, 0.1))} {sq(-0.14, 0.12, (0.1, 0.2, 0.85))} ] }}")


def world(d: dict, mesh_url: str, args: Sequence[str] = (), mu: Optional[float] = None,
          contact_ke: Optional[float] = None) -> str:
    eye, target = (-0.2, -1.6, 0.75), (-0.2, 0.0, 0.12)
    sys.path.insert(0, str(MANUFACTURING / "core"))
    from worldkit import look_at
    mu_s = f" newtonGroundMu {_f(mu)}" if mu is not None else " newtonGroundMu 1"
    ke_s = f" newtonContactKe {_f(contact_ke)}" if contact_ke else ""
    return "\n".join([
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        "# Generated by projects/manufacturing/products/spark_rover/robot.py from the SparkRover design. "
        "Regenerate; do not hand-edit.",
        f'WorldInfo {{ basicTimeStep 2 title "OmniSim SparkRover Mini - drive test" newtonSolver "mujoco" '
        f"newtonSubsteps 2{mu_s}{ke_s} FPS 30 }}",
        f"Viewpoint {{ position {_v(*eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF FLOOR Solid { name "floor" translation 0 0 -0.01 children [ Shape { appearance PBRAppearance '
        '{ baseColor .62 .63 .60 roughness .8 metalness 0 } geometry Box { size 3 2 .02 } } ] '
        'boundingObject Box { size 3 2 .02 } }',
        target_node(),
        robot_node(d, mesh_url, args=args, translation=(START_X, 0.0, 0.001)),
        "",
    ])


def controller_args(d: dict, result: Optional[str] = None, extra: Sequence[str] = ()) -> List[str]:
    gm = d["gearmotor"]
    a = [f"--pack-v={_f(d['pack']['v_nom'])}", f"--rated-v={_f(gm['rated_v'])}",
         f"--no-load-rpm={_f(gm['no_load_rpm'])}", f"--stall-kgcm={_f(gm['stall_kgcm'])}"]
    if result:
        a.append(f"--result={result}")
    return a + list(extra)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write SparkRover Mini's meshes and its drive-test world.")
    p.add_argument("--out", default=str(WORLD))
    p.add_argument("--result", help="where the drive test writes its result (default: .tmp/spark_rover/drive.json)")
    p.add_argument("--mu", type=float, help="newtonGroundMu (default 1, the design's mu_hi)")
    p.add_argument("--contact-ke", type=float, help="newtonContactKe (default: the engine's)")
    p.add_argument("--quit", action="store_true", help="the controller ends the simulation when it has written its result")
    p.add_argument("--controller-arg", action="append", default=[], metavar="ARG",
                   help="one more controller argument, e.g. --controller-arg=--frames=<dir> (repeatable)")
    p.add_argument("--no-cache", action="store_true")
    a = p.parse_args(argv)
    d = design(use_cache=not a.no_cache)
    write_meshes(d)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    url = Path(os.path.relpath(MESHES, out.resolve().parent)).as_posix()
    args = controller_args(d, a.result, (["--quit"] if a.quit else []) + a.controller_arg)
    out.write_text(world(d, url, args, mu=a.mu, contact_ke=a.contact_ke), encoding="utf-8", newline="\n")
    print(json.dumps({"mass": mass_report(d), "predictions": predictions(d)}, indent=1, default=float))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
