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

"""Convert OmniSim's Newton-dumped MJCF (joint_N anonymous names) to
the same MJCF with controller-readable joint names — so the GPU
trainer's name-based joint lookups still work, but the physics is
exactly what OmniSim Newton runs at deploy.

Produce: projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml

Source: dump produced by setting OMNISIM_NEWTON_SAVE_MJCF=<path> on
OmniSim launch with the G1 deploy world (see WbNewtonBackend.cpp:696).

The Newton dump uses anonymous body_N / joint_N naming because Newton's
ModelBuilder doesn't preserve URDF names. We re-attach names by
structural inspection: the URDF expansion adds bodies depth-first
left-leg → right-leg → torso, so the mapping is deterministic.

Plus: adds position+velocity actuators per joint matching Newton's
internal kp=20/kv=3 (which the dump already shows in its actuator
section but with anonymous joint targets).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
SRC = REPO / "_scratch/g1_newton_exact.mjcf.xml"
DST = REPO / "projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml"


# Mapping derived by reading the dump's body chain:
#   body_0 = root (pelvis), free joint = joint_1
#   body_1 → body_6 = left leg (hip_pitch → ankle_roll)
#   body_7 → body_12 = right leg
#   body_13 = torso (waist_yaw)
JOINT_RENAME = {
    "joint_1":  "root_free",
    "joint_2":  "left_hip_pitch_joint",
    "joint_5":  "left_hip_roll_joint",
    "joint_7":  "left_hip_yaw_joint",
    "joint_9":  "left_knee_joint",
    "joint_11": "left_ankle_pitch_joint",
    "joint_13": "left_ankle_roll_joint",
    "joint_3":  "right_hip_pitch_joint",
    "joint_6":  "right_hip_roll_joint",
    "joint_8":  "right_hip_yaw_joint",
    "joint_10": "right_knee_joint",
    "joint_12": "right_ankle_pitch_joint",
    "joint_14": "right_ankle_roll_joint",
    "joint_4":  "waist_yaw_joint",
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
}


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: {SRC} missing. Run OmniSim once with\n"
              f"  OMNISIM_NEWTON_SAVE_MJCF={SRC}\n"
              f"  OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1\n"
              f"  OMNISIM_URDF_USE_INERTIA=1\n"
              f"loading projects/policies/worlds/g1_stand_deploy.wbt", file=sys.stderr)
        return 1
    text = SRC.read_text(encoding="utf-8")

    # Rename joints (longest-first so e.g. joint_11 isn't shadowed by joint_1).
    for old in sorted(JOINT_RENAME, key=len, reverse=True):
        new = JOINT_RENAME[old]
        # Match the exact token bounded by word boundaries so joint_11 isn't
        # caught by the joint_1 rule.
        text = re.sub(rf'\b{old}\b', new, text)
    for old in sorted(BODY_RENAME, key=len, reverse=True):
        new = BODY_RENAME[old]
        text = re.sub(rf'\b{old}\b', new, text)

    # The Newton dump uses one position + one velocity actuator per
    # joint, but the actuators are unnamed (`<general joint="..." .../>`
    # with no name attr). The trainer looks up actuators by name
    # ("<joint>_pos" / "<joint>_vel"), so we have to inject names.
    # Each <general> block under <actuator> is alternating pos, vel.
    out_lines = []
    pos_or_vel = "pos"
    cur_joint = None
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
                # Inject name attribute right after `<general`.
                line2 = re.sub(r'<general ', f'<general name="{name}" ', line)
                out_lines.append(line2)
                pos_or_vel = "vel" if pos_or_vel == "pos" else "pos"
                continue
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"

    DST.write_text(text, encoding="utf-8")
    print(f"saved {DST} ({DST.stat().st_size} bytes)")

    # Round-trip verification: load via MuJoCo, list joints + actuators.
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(DST))
    print(f"nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt}")
    joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    print(f"joints: {joints}")
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    print(f"actuators ({len(acts)}): first 4 = {acts[:4]} ... last 2 = {acts[-2:]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
