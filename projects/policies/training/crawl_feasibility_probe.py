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

"""Phase-0 feasibility gate for the G1 CRAWL campaign.

QUESTION (the Shadowing thesis, applied): before designing any crawl ghost,
can the G1 *physically* hold a hands-and-knees posture on the crawl-contact
colliders we just added? A feasible reference is worthless if the body can't
bear the pose. This probe answers GO / NO-GO with numbers, no training.

WHAT IT DOES
  1. Compiles `g1_23dof_omnisim_crawl.urdf` through the project's own
     converter (`backends/_urdf_to_mjcf.load_or_convert`) so the contact
     model matches training. Clamps each actuator's force to the URDF
     effort limit — an HONEST test (the arms only get their real ~25 N.m).
  2. Poses the robot hands-and-knees (base pitched forward + folded legs +
     arms reaching down), auto-grounds it (drops base-z so the lowest
     contact just touches), runs FK, and prints where every key body lands.
  3. Settles under gravity with PD holding the pose, then reports:
       - pelvis height trajectory (held vs collapsed),
       - which bodies actually carry ground load (hands+knees == GO;
         face/torso == the robot fell on its face),
       - peak actuator torque vs the effort limit per joint (saturation).

Run:  python projects/policies/training/crawl_feasibility_probe.py
Tune the POSE dict until FK shows a clean all-fours, then read the verdict.
"""
from __future__ import annotations

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco

REPO = Path(__file__).resolve().parents[3]
URDF = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_crawl.urdf"

JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
]

# ── the two crawl postures (select via argv: "high" | "low") ──────────
import sys

# HIGH crawl: hands-and-knees, torso horizontal, arms support the front.
BASE_PITCH_HIGH = 90.0
POSE_HIGH = {
    "left_hip_pitch_joint": -1.30, "right_hip_pitch_joint": -1.30,
    "left_hip_roll_joint":  0.00,  "right_hip_roll_joint":  0.00,
    "left_hip_yaw_joint":   0.00,  "right_hip_yaw_joint":   0.00,
    "left_knee_joint":      1.90,  "right_knee_joint":      1.90,
    "left_ankle_pitch_joint": -0.50, "right_ankle_pitch_joint": -0.50,
    "left_ankle_roll_joint":  0.00,  "right_ankle_roll_joint":  0.00,
    "waist_yaw_joint": 0.00,
    "left_shoulder_pitch_joint": -1.50, "right_shoulder_pitch_joint": -1.50,
    "left_shoulder_roll_joint":   0.18, "right_shoulder_roll_joint": -0.18,
    "left_shoulder_yaw_joint":    0.00, "right_shoulder_yaw_joint":   0.00,
    "left_elbow_joint":  1.20, "right_elbow_joint":  1.20,
    "left_wrist_roll_joint": 0.00, "right_wrist_roll_joint": 0.00,
}

# LOW crawl: belly + forearms on the ground; torso rests (bears the weight),
# forearms lie flat and pull. This is the owner's "elbows / under-the-wire"
# target — and the hypothesis is it loads the weak arms LESS than high crawl.
BASE_PITCH_LOW = 90.0
POSE_LOW = {
    # legs drawn up froggy; knees/shins splayed out and back, resting
    "left_hip_pitch_joint": -0.55, "right_hip_pitch_joint": -0.55,
    "left_hip_roll_joint":  0.28,  "right_hip_roll_joint": -0.28,
    "left_hip_yaw_joint":   0.00,  "right_hip_yaw_joint":  0.00,
    "left_knee_joint":      1.30,  "right_knee_joint":     1.30,
    "left_ankle_pitch_joint": -0.30, "right_ankle_pitch_joint": -0.30,
    "left_ankle_roll_joint":  0.00,  "right_ankle_roll_joint":  0.00,
    "waist_yaw_joint": 0.00,
    # forearms flat ahead: upper arm down, elbow bent ~90 (0 == 90-deg bent)
    "left_shoulder_pitch_joint": -0.80, "right_shoulder_pitch_joint": -0.80,
    "left_shoulder_roll_joint":   0.35, "right_shoulder_roll_joint": -0.35,
    "left_shoulder_yaw_joint":    0.00, "right_shoulder_yaw_joint":   0.00,
    "left_elbow_joint":  0.35, "right_elbow_joint":  0.35,
    "left_wrist_roll_joint": 0.00, "right_wrist_roll_joint": 0.00,
}

# COMMANDO crawl (owner-chosen): forearms flat on the ground (distributed
# contact, low shoulder torque) + knees; hips up. The arm-feasibility sweet
# spot and the literal "on its elbows" look. This static pose anchors the ghost.
BASE_PITCH_COMMANDO = 90.0        # torso ~horizontal, low profile (under the wire)
POSE_COMMANDO = {
    # legs: hips flexed, knees folded out/back — shins/knees rest, hips low
    "left_hip_pitch_joint": -0.60, "right_hip_pitch_joint": -0.60,
    "left_hip_roll_joint":  0.22,  "right_hip_roll_joint": -0.22,
    "left_hip_yaw_joint":   0.00,  "right_hip_yaw_joint":  0.00,
    "left_knee_joint":      1.35,  "right_knee_joint":     1.35,
    "left_ankle_pitch_joint": -0.30, "right_ankle_pitch_joint": -0.30,
    "left_ankle_roll_joint":  0.00,  "right_ankle_roll_joint":  0.00,
    "waist_yaw_joint": 0.00,
    # arms: upper arm ~vertical down from shoulder, elbow bent so the FOREARM
    # lies flat forward on the ground (distributed front support = low shoulder
    # torque). shoulder_pitch ~ -90 deg points the upper arm down from a
    # horizontal torso.
    "left_shoulder_pitch_joint": -1.50, "right_shoulder_pitch_joint": -1.50,
    "left_shoulder_roll_joint":   0.25, "right_shoulder_roll_joint": -0.25,
    "left_shoulder_yaw_joint":    0.00, "right_shoulder_yaw_joint":   0.00,
    "left_elbow_joint":  0.35, "right_elbow_joint":  0.35,
    "left_wrist_roll_joint": 0.00, "right_wrist_roll_joint": 0.00,
}

MODE = sys.argv[1] if len(sys.argv) > 1 else "commando"
POSE = {"low": POSE_LOW, "high": POSE_HIGH, "commando": POSE_COMMANDO}[MODE]
BASE_PITCH_DEG = {"low": BASE_PITCH_LOW, "high": BASE_PITCH_HIGH,
                  "commando": BASE_PITCH_COMMANDO}[MODE]

# NB: fixed-joint child links (head_link, waist_support, pelvis_contour, imu...)
# are merged by MuJoCo into their parent; head_link -> torso_link.
KEY_BODIES = [
    "pelvis", "torso_link",
    "left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand",
    "left_elbow_link", "right_elbow_link",
    "left_knee_link", "right_knee_link",
    "left_ankle_roll_link", "right_ankle_roll_link",
]


def load_converter():
    p = REPO / "projects/policies/research/backends/_urdf_to_mjcf.py"
    s = importlib.util.spec_from_file_location("_u2m", p)
    m = importlib.util.module_from_spec(s)
    s.loader.exec_module(m)
    return m.load_or_convert


def urdf_limits():
    """(lo, hi, effort) per joint name from the URDF."""
    root = ET.parse(URDF).getroot()
    lim = {}
    for j in root.iter("joint"):
        n = j.get("name"); L = j.find("limit")
        if n and L is not None:
            lim[n] = (float(L.get("lower", -3.14)),
                      float(L.get("upper", 3.14)),
                      float(L.get("effort", 100.0)))
    return lim


def quat_pitch(deg):
    a = math.radians(deg)
    return np.array([math.cos(a / 2), 0.0, math.sin(a / 2), 0.0])  # about +y


def abs_mesh_urdf():
    """Rewrite relative `meshes/X.STL` mesh URIs to absolute paths so the
    MuJoCo compiler resolves them (the shared converter only handles
    package:// URIs). Meshes are visual-only here, but the compiler still
    loads them. Returns a sibling temp URDF path."""
    meshdir = (REPO / "projects/robots/unitree/g1/urdf/meshes").as_posix()
    text = URDF.read_text().replace('filename="meshes/', f'filename="{meshdir}/')
    tmp = URDF.with_name("g1_23dof_omnisim_crawl_abs.urdf")
    tmp.write_text(text)
    return tmp


def main():
    load_or_convert = load_converter()
    lim = urdf_limits()
    jl = [(lim[j][0], lim[j][1]) for j in JOINTS]
    model = load_or_convert(abs_mesh_urdf(), actuator_joints=JOINTS,
                            joint_limits=jl, kp=120.0, kv=4.0)
    # Match the TRAINED contact model: only the URDF <collision> geoms (boxes:
    # feet + our added knee/hand boxes) collide with the ground. The converter
    # forces every geom collidable incl. visual MESH geoms; disable those so
    # the robot can't rest on non-physical mesh surfaces (pelvis/belly/thigh).
    n_mesh_off = 0
    for g in range(model.ngeom):
        if model.geom(g).type == mujoco.mjtGeom.mjGEOM_MESH:
            model.geom_contype[g] = 0
            model.geom_conaffinity[g] = 0
            n_mesh_off += 1

    data = mujoco.MjData(model)

    # Clamp actuator force to the URDF effort limit — the honest test.
    for i, j in enumerate(JOINTS):
        eff = lim[j][2]
        aid = model.actuator(f"{j}_motor").id
        model.actuator_forcelimited[aid] = 1
        model.actuator_forcerange[aid] = [-eff, eff]

    jadr = {j: model.joint(j).qposadr[0] for j in JOINTS}
    vadr = {j: model.joint(j).dofadr[0] for j in JOINTS}

    def set_pose(base_z):
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qpos[0:3] = [0.0, 0.0, base_z]
        data.qpos[3:7] = quat_pitch(BASE_PITCH_DEG)
        for j, a in POSE.items():
            data.qpos[jadr[j]] = a
        mujoco.mj_forward(model, data)

    # Auto-ground exactly: drop the base so the lowest collision-box corner
    # sits ~3 mm above the floor (works for any posture, high or low).
    box_geoms = [g for g in range(model.ngeom)
                 if model.geom(g).type == mujoco.mjtGeom.mjGEOM_BOX
                 and model.geom_contype[g] != 0]
    corners = np.array([[sx, sy, sz] for sx in (-1, 1)
                        for sy in (-1, 1) for sz in (-1, 1)], dtype=float)

    def lowest_corner(only_body=None):
        bid = model.body(only_body).id if only_body else None
        zmin = 1e9
        for g in box_geoms:
            if bid is not None and int(model.geom_bodyid[g]) != bid:
                continue
            pos = data.geom_xpos[g]
            R = data.geom_xmat[g].reshape(3, 3)
            s = model.geom_size[g]
            cz = (pos + (corners * s) @ R.T)[:, 2]
            zmin = min(zmin, float(cz.min()))
        return zmin

    # Low crawl: ground on the belly (torso box) so it rests and bears weight.
    # High crawl: co-ground the hands and knees (bring the lower of the two
    # body origins ~7 cm up so both colliders engage together, not just the
    # lowest-hanging shank box), then engage under the anchor.
    set_pose(1.0)
    if MODE == "low":
        base_z = 1.0 - lowest_corner("torso_link") - 0.006
    elif MODE == "commando":
        # co-ground the forearms (elbow_link) + knees
        fk = ["left_elbow_link", "right_elbow_link",
              "left_knee_link", "right_knee_link"]
        base_z = 1.0 - min(lowest_corner(b) for b in fk) - 0.004
    else:
        hk = ["left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand",
              "left_knee_link", "right_knee_link"]
        base_z = 1.0 - min(float(data.body(b).xpos[2]) for b in hk) + 0.055
    set_pose(base_z)
    bpos = {b: data.body(b).xpos.copy() for b in KEY_BODIES}

    print("=" * 66)
    print(f"POSE CHECK  (base_pitch={BASE_PITCH_DEG} deg, base_z={base_z:.3f})")
    print("-" * 66)
    print(f"  {'body':<32}{'x':>7}{'y':>7}{'z':>7}")
    for b in KEY_BODIES:
        p = data.body(b).xpos
        print(f"  {b:<32}{p[0]:7.3f}{p[1]:7.3f}{p[2]:7.3f}")

    # Settle with the base held as a kinematic anchor (== a perfect pelvis
    # harness, the role the lambda=0.9 puppet plays for the walk). This
    # isolates the real question: with training-level base support, can the
    # LIMBS hold the crawl posture within their torque limits and bear load
    # on hands+knees? Free self-support (no harness) is a later durability
    # goal, exactly as it is for the walk.
    base_qpos = data.qpos[0:7].copy()
    ctrl = np.array([POSE[j] for j in JOINTS])
    data.ctrl[:] = ctrl
    dt = model.opt.timestep
    T = 2.0
    n = int(T / dt)
    peak_tau = np.zeros(len(JOINTS))
    ss_tau = np.zeros(len(JOINTS))     # steady-state (mean over last 0.3 s)
    ss_n = 0
    zs = []
    for k in range(n):
        mujoco.mj_step(model, data)
        data.qpos[0:7] = base_qpos     # freeze the base (harness anchor)
        data.qvel[0:6] = 0.0
        mujoco.mj_forward(model, data)
        peak_tau = np.maximum(peak_tau, np.abs(data.actuator_force))
        if k > n - int(0.3 / dt):
            ss_tau += np.abs(data.actuator_force); ss_n += 1
        if k % max(1, n // 20) == 0:
            zs.append(data.body("pelvis").xpos[2])
    ss_tau /= max(1, ss_n)
    z0 = data.body("pelvis").xpos[2]

    # Which bodies carry ground load?
    ground_id = None
    for g in range(model.ngeom):
        if model.geom(g).type == mujoco.mjtGeom.mjGEOM_PLANE:
            ground_id = g
            break
    load = {}
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = c.geom1, c.geom2
        if ground_id in (g1, g2):
            other = g2 if g1 == ground_id else g1
            bid = int(model.geom_bodyid[other])
            bname = model.body(bid).name
            mujoco.mj_contactForce(model, data, i, f6)
            load[bname] = load.get(bname, 0.0) + abs(f6[0])

    print("-" * 66)
    print("SETTLE  (2.0 s, base anchored == harness; force clamped to URDF effort)")
    print(f"  pelvis-z (held at anchor): {z0:.3f} m")
    print("  ground load by body (N, normal):")
    tot = sum(load.values()) or 1.0
    for b, f in sorted(load.items(), key=lambda kv: -kv[1]):
        print(f"    {b:<32}{f:8.1f}  ({100*f/tot:4.0f}%)")
    print("  steady-state actuator torque vs effort limit (peak in parens):")
    any_sat = False
    for i, j in enumerate(JOINTS):
        eff = lim[j][2]
        frac = ss_tau[i] / eff
        sat = frac > 0.97
        any_sat = any_sat or (sat and ("shoulder" in j or "elbow" in j or "hip" in j or "knee" in j))
        flag = "  <-- SATURATED" if sat else ""
        if frac > 0.4 or sat:
            print(f"    {j:<30}{ss_tau[i]:7.1f} / {eff:6.1f}  ({frac:4.0%})  peak {peak_tau[i]:5.1f}{flag}")

    # Verdict: with base support, do hands AND knees bear load, and do the
    # support-critical limb joints stay within torque limits?
    hands = load.get("left_wrist_roll_rubber_hand", 0) + load.get("right_wrist_roll_rubber_hand", 0)
    knees = load.get("left_knee_link", 0) + load.get("right_knee_link", 0)
    feet = load.get("left_ankle_roll_link", 0) + load.get("right_ankle_roll_link", 0)
    four_point = hands > 10 and knees > 10
    print("=" * 66)
    print(f"  contact: hands {hands:.0f}N | knees {knees:.0f}N | feet {feet:.0f}N")
    verdict = ("GO (limbs bear hands-and-knees under harness support)"
               if four_point and not any_sat
               else "NEEDS TUNING (see saturated joints / missing contacts)")
    print(f"  VERDICT --> {verdict}")
    print("=" * 66)


if __name__ == "__main__":
    main()
