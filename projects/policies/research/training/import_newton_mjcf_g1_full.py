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

"""Convert OmniSim's full-body (23-DOF) G1 Newton-dumped MJCF to one
with controller-readable joint/body names. Full-body sibling of
`import_newton_mjcf.py` (which does the legs-only 13-DOF subset) — see
that file for the full rationale.

Produces: projects/robots/unitree/g1/urdf/g1_full.mjcf.xml

Source: dump produced by setting OMNISIM_NEWTON_SAVE_MJCF=<path> on an
OmniSim launch with projects/policies/worlds/g1_stand_arms_deploy.wbt (the
full 23-DOF body). Newton's ModelBuilder emits anonymous body_N /
joint_N names; we re-attach readable names by walking the dumped
kinematic tree and matching it to the URDF structure.

Mapping (verified against the dump tree + per-link masses):
  body_0  pelvis (free joint_1)
  body_1..6   left leg   (hip_pitch → ankle_roll)
  body_7..12  right leg
  body_13     torso (waist_yaw joint_4)
  body_14..18 left arm   (shoulder_pitch → wrist_roll)
  body_19..23 right arm
The leg portion is identical to import_newton_mjcf.py — same joint_N
ids — which is the cross-check that the dump ordering is stable.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
SRC = REPO / "_scratch/g1_full_newton.mjcf.xml"
DST = REPO / "projects/robots/unitree/g1/urdf/g1_full.mjcf.xml"


JOINT_RENAME = {
    "joint_1":  "root_free",
    # Left leg.
    "joint_2":  "left_hip_pitch_joint",
    "joint_5":  "left_hip_roll_joint",
    "joint_9":  "left_hip_yaw_joint",
    "joint_13": "left_knee_joint",
    "joint_17": "left_ankle_pitch_joint",
    "joint_21": "left_ankle_roll_joint",
    # Right leg.
    "joint_3":  "right_hip_pitch_joint",
    "joint_6":  "right_hip_roll_joint",
    "joint_10": "right_hip_yaw_joint",
    "joint_14": "right_knee_joint",
    "joint_18": "right_ankle_pitch_joint",
    "joint_22": "right_ankle_roll_joint",
    # Waist.
    "joint_4":  "waist_yaw_joint",
    # Left arm.
    "joint_7":  "left_shoulder_pitch_joint",
    "joint_11": "left_shoulder_roll_joint",
    "joint_15": "left_shoulder_yaw_joint",
    "joint_19": "left_elbow_joint",
    "joint_23": "left_wrist_roll_joint",
    # Right arm.
    "joint_8":  "right_shoulder_pitch_joint",
    "joint_12": "right_shoulder_roll_joint",
    "joint_16": "right_shoulder_yaw_joint",
    "joint_20": "right_elbow_joint",
    "joint_24": "right_wrist_roll_joint",
}

BODY_RENAME = {
    "body_0":  "pelvis",
    "body_1":  "left_hip_pitch_link",
    "body_2":  "left_hip_roll_link",
    "body_3":  "left_hip_yaw_link",
    "body_4":  "left_knee_link",
    "body_5":  "left_ankle_pitch_link",
    "body_6":  "left_ankle_roll_link",
    "body_7":  "right_hip_pitch_link",
    "body_8":  "right_hip_roll_link",
    "body_9":  "right_hip_yaw_link",
    "body_10": "right_knee_link",
    "body_11": "right_ankle_pitch_link",
    "body_12": "right_ankle_roll_link",
    "body_13": "torso_link",
    "body_14": "left_shoulder_pitch_link",
    "body_15": "left_shoulder_roll_link",
    "body_16": "left_shoulder_yaw_link",
    "body_17": "left_elbow_link",
    "body_18": "left_wrist_roll_rubber_hand",
    "body_19": "right_shoulder_pitch_link",
    "body_20": "right_shoulder_roll_link",
    "body_21": "right_shoulder_yaw_link",
    "body_22": "right_elbow_link",
    "body_23": "right_wrist_roll_rubber_hand",
}


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: {SRC} missing. Run OmniSim once with\n"
              f"  OMNISIM_NEWTON_SAVE_MJCF={SRC}\n"
              f"  OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1\n"
              f"  OMNISIM_URDF_USE_INERTIA=1\n"
              f"loading projects/policies/worlds/g1_stand_arms_deploy.wbt",
              file=sys.stderr)
        return 1
    text = SRC.read_text(encoding="utf-8")

    # Longest-first so joint_11 isn't shadowed by the joint_1 rule.
    for old in sorted(JOINT_RENAME, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', JOINT_RENAME[old], text)
    for old in sorted(BODY_RENAME, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', BODY_RENAME[old], text)

    # Name the alternating pos/vel actuators (Newton emits them unnamed).
    out_lines = []
    pos_or_vel = "pos"
    actuator_block = False
    for line in text.splitlines():
        s = line.strip()
        if s == "<actuator>":
            actuator_block = True
            out_lines.append(line)
            pos_or_vel = "pos"
            continue
        if s == "</actuator>":
            actuator_block = False
            out_lines.append(line)
            continue
        if actuator_block and s.startswith("<general "):
            m = re.search(r'joint="([^"]+)"', s)
            if m:
                jn = m.group(1)
                name = f"{jn}_{pos_or_vel}"
                line2 = re.sub(r'<general ', f'<general name="{name}" ', line)
                out_lines.append(line2)
                pos_or_vel = "vel" if pos_or_vel == "pos" else "pos"
                continue
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"

    DST.write_text(text, encoding="utf-8")
    print(f"saved {DST} ({DST.stat().st_size} bytes)")

    # Round-trip verification.
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(DST))
    print(f"nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt}")
    joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
              for i in range(m.njnt)]
    print(f"joints ({len(joints)}): {joints}")
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(m.nu)]
    print(f"actuators ({len(acts)}): first 4 = {acts[:4]} ... last 2 = {acts[-2:]}")
    # Sanity: every controller joint name resolves.
    need = [
        "left_hip_pitch_joint", "waist_yaw_joint", "left_elbow_joint",
        "right_wrist_roll_joint",
    ]
    for jn in need:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        assert jid >= 0, f"{jn} missing after rename"
    print("all spot-check joints resolve OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
