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

"""Build g1_full_kp100.mjcf.xml: the committed full-body g1_full.mjcf.xml
(correct 23-DOF structure/inertia from the Newton dump pipeline) with every
actuator rewritten to the deploy joint drive kp=100/kv=5 and ground mu=2 --
matching g1_legs_kp100.mjcf.xml and OMNISIM_NEWTON_TARGET_KE=100/KD=5.

Cross-validates the leg portion (masses, joint limits) against the
known-good legs MJCF before writing.
"""
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

SRC = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_full.mjcf.xml")
LEGS = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_legs_kp100.mjcf.xml")
DST = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_full_kp100.mjcf.xml")

text = open(SRC, encoding="utf-8").read()

# Rewrite every <general ...> actuator: *_pos -> kp=100, *_vel -> kv=5.
def fix_actuator(m):
    tag = m.group(0)
    name = re.search(r'name="([^"]+)"', tag)
    if not name:
        return tag
    n = name.group(1)
    joint = re.search(r'joint="([^"]+)"', tag).group(1)
    if n.endswith("_pos"):
        return (f'<general name="{n}" joint="{joint}" biastype="affine" '
                f'gainprm="100" biasprm="0 -100"/>')
    if n.endswith("_vel"):
        return (f'<general name="{n}" joint="{joint}" biastype="affine" '
                f'gainprm="5" biasprm="0 0 -5"/>')
    return tag

text2, nsub = re.subn(r'<general [^>]*/>', fix_actuator, text)
print(f"actuators rewritten: {nsub}")

# Ground friction -> 2 (match deploy GROUND_MU=2.0).
text2 = re.sub(r'(type="plane"[^>]*friction=")[^ "]+',
               lambda m: m.group(1) + "2", text2)

open(DST, "w", encoding="utf-8").write(text2)

mf = mujoco.MjModel.from_xml_path(DST)
ml = mujoco.MjModel.from_xml_path(LEGS)
print(f"full: nq={mf.nq} nv={mf.nv} nu={mf.nu} nbody={mf.nbody}")
print(f"kp sample: {mf.actuator_gainprm[0,0]} kv: {mf.actuator_gainprm[1,0]} "
      f"ground mu: {mf.geom_friction[0,0]}")

# Cross-check the leg portion against the legs model.
LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
]
ok = True
for jn in LEG_JOINTS:
    jf = mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT, jn)
    jl = mujoco.mj_name2id(ml, mujoco.mjtObj.mjOBJ_JOINT, jn)
    if jf < 0 or jl < 0:
        print(f"MISSING {jn}"); ok = False; continue
    rf = mf.jnt_range[jf]; rl = ml.jnt_range[jl]
    if not np.allclose(rf, rl, atol=1e-5):
        print(f"RANGE MISMATCH {jn}: {rf} vs {rl}"); ok = False
LEG_BODIES = ["pelvis", "left_knee_link", "right_ankle_roll_link", "torso_link"]
for bn in LEG_BODIES:
    bf = mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_BODY, bn)
    bl = mujoco.mj_name2id(ml, mujoco.mjtObj.mjOBJ_BODY, bn)
    if bf < 0 or bl < 0:
        print(f"MISSING body {bn}"); ok = False; continue
    if not np.isclose(mf.body_mass[bf], ml.body_mass[bl], atol=1e-5):
        print(f"MASS MISMATCH {bn}: {mf.body_mass[bf]} vs {ml.body_mass[bl]}"); ok = False
ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
]
for jn in ARM_JOINTS:
    if mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT, jn) < 0:
        print(f"MISSING arm joint {jn}"); ok = False
    for sfx in ("_pos", "_vel"):
        if mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + sfx) < 0:
            print(f"MISSING actuator {jn}{sfx}"); ok = False
total_mass = mf.body_mass.sum()
legs_mass = ml.body_mass.sum()
print(f"total mass full={total_mass:.2f} legs={legs_mass:.2f} arms+hands delta={total_mass-legs_mass:.2f} kg")
print("CROSS-CHECK", "OK" if ok else "FAILED")
sys.exit(0 if ok else 1)
