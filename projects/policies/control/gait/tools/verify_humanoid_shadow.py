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

"""Verify a humanoid walking SHADOW is dynamically FEASIBLE (achievable by an
RL policy) using the project's ghost_verifier -- the per-step contact-wrench LP
(base-wrench feasibility + friction cone + actuator torque limits).

The URDFs (H1 bad mesh path, Valkyrie .dae) will not load in MuJoCo, so this
tool emits a self-contained WALK-MJCF straight from the URDF kinematic tree:
real per-link masses/inertias, the leg joints as actuated hinges with the
URDF effort limits, every non-leg joint welded (rigid upper body), and the feet
as collision boxes. It then rolls the robot's gait model into a ghost NPZ (the
floating base striding forward at vx over a few cycles) and runs the verifier.

NOTE: the inertia is approximated by the URDF diagonal (products dropped) and
the upper body is rigid -- this is a feasibility TRIAGE on a faithful-enough
model, not a deploy-exact certificate. Validated by running it on G1 first
(whose walk shadow is independently known-feasible).

Usage: python projects/policies/control/gait/tools/verify_humanoid_shadow.py g1|h1|valkyrie
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

# slot order in the gait's 13-vector: HP,HR,HY,KN,AP,AR (L), then R, then waist.
CFG = {
    "g1": dict(
        urdf="projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf",
        gait="g1_human_gait",
        legs=["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
              "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
              "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
              "waist_yaw_joint"],
        feet=["left_ankle_roll_link", "right_ankle_roll_link"],
        gait_kwargs=dict(style="winter", lateral="human", yaw="human"),
    ),
    "h1": dict(
        urdf="projects/robots/unitree/h1/urdf/h1.urdf",
        gait="h1_human_gait",
        legs=["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
              "left_knee_joint", "left_ankle_joint", None,
              "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_joint", None,
              "torso_joint"],
        feet=["left_ankle_link", "right_ankle_link"],
        gait_kwargs=dict(),
    ),
    "valkyrie": dict(
        urdf="projects/robots/nasa/valkyrie/urdf/valkyrie.urdf",
        gait="valkyrie_human_gait",
        legs=["leftHipPitch", "leftHipRoll", "leftHipYaw",
              "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
              "rightHipPitch", "rightHipRoll", "rightHipYaw",
              "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
              "torsoYaw"],
        feet=["leftFoot", "rightFoot"],
        gait_kwargs=dict(),
    ),
}


def _rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, yq, z)


def _vec(el, attr, n=3):
    s = el.get(attr) if el is not None else None
    return [float(v) for v in s.split()] if s else [0.0] * n


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        ine = l.find("inertial")
        d = dict(mass=0.0, ixyz=[0, 0, 0], irpy=[0, 0, 0], inertia=[0, 0, 0],
                 foot=None)
        if ine is not None:
            d["mass"] = float(ine.find("mass").get("value"))
            o = ine.find("origin")
            d["ixyz"] = _vec(o, "xyz")
            d["irpy"] = _vec(o, "rpy")
            I = ine.find("inertia")
            d["inertia"] = [float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))]
        col = l.find("collision")
        if col is not None and col.find("geometry").find("box") is not None:
            box = col.find("geometry").find("box")
            d["foot"] = dict(size=_vec(box, "size"),
                             xyz=_vec(col.find("origin"), "xyz"))
        links[l.get("name")] = d
    children = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        jd = dict(name=j.get("name"), type=j.get("type"),
                  parent=j.find("parent").get("link"),
                  child=j.find("child").get("link"),
                  xyz=_vec(o, "xyz"), rpy=_vec(o, "rpy"),
                  axis=_vec(j.find("axis"), "xyz") if j.find("axis") is not None else [0, 0, 1])
        lim = j.find("limit")
        if lim is not None:
            jd["lower"] = float(lim.get("lower", -3.14))
            jd["upper"] = float(lim.get("upper", 3.14))
            jd["effort"] = float(lim.get("effort", 100))
        else:
            jd["lower"], jd["upper"], jd["effort"] = -3.14, 3.14, 100.0
        joints[jd["child"]] = jd      # index by child link
        children.setdefault(jd["parent"], []).append(jd["child"])
    all_children = set(joints.keys())
    root_link = [n for n in links if n not in all_children][0]
    return links, joints, children, root_link


def emit_mjcf(robot):
    c = CFG[robot]
    links, joints, children, root_link = parse_urdf(ROOT / c["urdf"])
    leg_set = set(j for j in c["legs"] if j)
    feet = set(c["feet"])
    order = []                          # MJCF leg-joint order (qpos order)

    def body_xml(link, indent):
        sp = "  " * indent
        out = []
        d = links[link]
        # inertial (diagonal approx; clamp to valid positive)
        m = max(d["mass"], 1e-4)
        ii = [max(v, 1e-6) for v in d["inertia"]]
        # enforce triangle inequality for a valid inertia box
        ii = [min(ii[k], ii[(k + 1) % 3] + ii[(k + 2) % 3] - 1e-9) for k in range(3)]
        ii = [max(v, 1e-6) for v in ii]
        out.append(f'{sp}<inertial pos="{d["ixyz"][0]} {d["ixyz"][1]} {d["ixyz"][2]}" '
                   f'mass="{m:.5f}" diaginertia="{ii[0]:.6f} {ii[1]:.6f} {ii[2]:.6f}"/>')
        if link in feet and d["foot"]:
            s = d["foot"]["size"]; fx = d["foot"]["xyz"]
            side = "left" if "left" in link.lower() else "right"
            botz = fx[2] - s[2] / 2.0                 # foot sole height
            # 4 corner contact points (a contact PATCH, not a single point) so
            # the feet can support the base roll/pitch moment -- a single box
            # gives one contact point per foot and spuriously inflates the
            # base-wrench residual ~10%mg.
            for ci, (cx, cy) in enumerate([(+1, +1), (+1, -1), (-1, +1), (-1, -1)]):
                px = fx[0] + cx * s[0] / 2.0 * 0.85
                py = fx[1] + cy * s[1] / 2.0 * 0.85
                out.append(f'{sp}<geom name="{side}_foot{ci}" type="sphere" '
                           f'size="0.012" pos="{px:.4f} {py:.4f} {botz:.4f}" '
                           f'rgba="0.3 0.5 0.9 1"/>')
        for ch in children.get(link, []):
            j = joints[ch]
            q = _rpy_to_quat(*j["rpy"])
            # The verifier matches foot contacts by BODY name containing lowercase
            # 'foot'/'ankle_roll'. Valkyrie's 'leftFoot' (camelCase) matches neither,
            # so give every foot body a canonical lowercase '*_foot_link' name.
            bname = ch
            if ch in feet:
                bname = ("left_foot_link" if "left" in ch.lower() else "right_foot_link")
            out.append(f'{sp}<body name="{bname}" pos="{j["xyz"][0]} {j["xyz"][1]} {j["xyz"][2]}" '
                       f'quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}">')
            if j["name"] in leg_set and j["type"] in ("revolute", "continuous"):
                a = j["axis"]
                out.append(f'{"  "*(indent+1)}<joint name="{j["name"]}" type="hinge" '
                           f'axis="{a[0]} {a[1]} {a[2]}" range="{j["lower"]} {j["upper"]}" '
                           f'actuatorfrcrange="{-j["effort"]} {j["effort"]}" '
                           f'actuatorfrclimited="true"/>')
                order.append(j["name"])
            out.extend(body_xml(ch, indent + 1))
            out.append(f'{sp}</body>')
        return out

    body = body_xml(root_link, 3)
    mjcf = ['<mujoco model="%s_walk">' % robot,
            '  <compiler angle="radian" balanceinertia="true"/>',
            '  <option timestep="0.004" gravity="0 0 -9.81"/>',
            '  <worldbody>',
            '    <geom name="floor" type="plane" size="0 0 0.1" pos="0 0 0"/>',
            '    <body name="%s" pos="0 0 0" quat="1 0 0 0">' % root_link,
            '      <freejoint/>']
    mjcf += body
    mjcf += ['    </body>', '  </worldbody>',
             '  <actuator>']
    for jn in order:
        mjcf.append(f'    <motor name="{jn}_m" joint="{jn}"/>')
    mjcf += ['  </actuator>', '</mujoco>']
    return "\n".join(mjcf), order


def roll_ghost(robot, mjcf_path, leg_order, out_npz, dt=0.004, secs=3.0):
    import importlib
    import mujoco
    c = CFG[robot]
    gait = importlib.import_module(f"projects.policies.control.gait.{c['gait']}")
    GP = gait.GaitParams(**c["gait_kwargs"])
    name2slot = {n: i for i, n in enumerate(c["legs"]) if n}

    m = mujoco.MjModel.from_xml_path(mjcf_path)        # compiles -> validates the MJCF
    d = mujoco.MjData(m)
    nq, nv = m.nq, m.nv
    foot_gids = [g for g in range(m.ngeom)
                 if "foot" in (m.geom(g).name or "")]
    T = int(secs / dt)
    q = np.zeros((T, nq))
    for k in range(T):
        t = 1.0 + k * dt                               # steady state (ramp done)
        phi = (gait.DS_PHASE + 2 * math.pi * GP.freq * t) % (2 * math.pi)
        legs, _, _ = gait.targets_np(phi, GP, t_since_start=1e6)
        d.qpos[:] = 0
        d.qpos[2] = GP.pelvis_height                   # provisional base z
        d.qpos[3] = 1.0
        for i, jn in enumerate(leg_order):
            d.qpos[7 + i] = legs[name2slot[jn]]
        mujoco.mj_forward(m, d)
        # auto-ground: drop the base so the LOWEST foot point rests on z=0 (a
        # few mm into the floor so contact is detected). The gait's 2-link IK
        # approximates the real offset leg geometry, so the exact base height
        # is set here from the model FK, not assumed.
        min_fz = min(d.geom_xpos[g][2] for g in foot_gids)
        q[k, 0] = GP.vx * (k * dt)                     # x forward
        q[k, 2] = GP.pelvis_height - min_fz - 0.004
        q[k, 3] = 1.0                                  # quat w (upright)
        q[k, 7:] = d.qpos[7:]
    qvel = np.zeros((T, nv))
    qvel[1:-1, 0] = (q[2:, 0] - q[:-2, 0]) / (2 * dt)  # base x vel
    qvel[1:-1, 2] = (q[2:, 2] - q[:-2, 2]) / (2 * dt)  # base z vel
    qvel[1:-1, 6:] = (q[2:, 7:] - q[:-2, 7:]) / (2 * dt)
    np.savez(out_npz, q=q, qvel=qvel, dt=dt)
    return m


def main(robot):
    out = ROOT / "_scratch"
    out.mkdir(exist_ok=True)
    mjcf_path = str(out / f"{robot}_walk.mjcf.xml")
    npz_path = str(out / f"{robot}_shadow.npz")
    mjcf, order = emit_mjcf(robot)
    Path(mjcf_path).write_text(mjcf, encoding="utf-8")
    print(f"[{robot}] emitted MJCF ({len(order)} leg DOF): {mjcf_path}")
    roll_ghost(robot, mjcf_path, order, npz_path)
    print(f"[{robot}] rolled ghost NPZ: {npz_path}")

    from projects.policies.research.shadowing.ghost_verifier import verify
    # mu=1.5 matches the worlds' contactProperties coulombFriction (deploy ground
    # mu is 2.0); 1.0 was over-conservative and made the friction cone bind.
    mu = float(os.environ.get("VERIFY_MU", "1.5"))
    res = verify(npz_path, mjcf_path, contacts="foot", mu=mu, contact_z=0.06,
                 verbose=True)
    print(f"\n=== {robot.upper()} SHADOW FEASIBILITY: "
          f"{'PASS' if res.get('passed') else 'FAIL'}  "
          f"score={res.get('score', 0):.3f} ===")
    return res


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "g1")
