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

"""Convert OmniSim's Newton-dumped H1 MJCF (anonymous joint_N / body_N names)
to the trainer MJCF with controller-readable names -- so the GPU walk trainer's
name-based joint lookups work but the PHYSICS is EXACTLY what OmniSim Newton runs
at deploy (zero sim-to-deploy gap, same approach as the G1).

Produce: projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml

Source dump (OMNISIM_NEWTON_SAVE_MJCF):
  $env:OMNISIM_NEWTON_SAVE_MJCF = "_scratch/h1_newton_dump.mjcf.xml"
  $env:HW_NO_WARMUP = "1"
  scripts/dev/run_humanoid_walk_deploy.ps1 -Robot h1 -Duration 16 -Ke 800 -Kd 70
(needs --duration >= 14 so the Newton MuJoCo solver FINALISES before the dump).

The joint_N -> name map was derived from the dump's per-joint axis + range +
body offset signatures (each H1 leg/arm joint is unique), cross-checked against
h1.json joint_order. The dump bakes the deploy kp/kv (TARGET_KE/KD) into the
actuators -- train at that kp and the stiffness matches deploy.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
SRC = REPO / "_scratch/h1_newton_dump.mjcf.xml"
DST = REPO / "projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"

# Verified from the dump signatures (PowerShell jnt_axis/jnt_range/body_pos):
JOINT_RENAME = {
    "joint_1":  "root_free",
    "joint_2":  "left_hip_yaw_joint",     # axis z, body (0,+.087,-.174)
    "joint_5":  "left_hip_roll_joint",    # axis x, body (.039,0,0)
    "joint_9":  "left_hip_pitch_joint",   # axis y, range [-3.13,2.53]
    "joint_13": "left_knee_joint",        # axis y, range [-0.26,2.05]
    "joint_17": "left_ankle_joint",       # axis y, range [-0.87,0.52]
    "joint_3":  "right_hip_yaw_joint",    # axis z, body (0,-.087,-.174)
    "joint_6":  "right_hip_roll_joint",
    "joint_10": "right_hip_pitch_joint",
    "joint_14": "right_knee_joint",
    "joint_18": "right_ankle_joint",
    "joint_4":  "torso_joint",            # axis z, body (0,0,0)
    "joint_7":  "left_shoulder_pitch_joint",   # axis y, body (.005,+.155,.43)
    "joint_11": "left_shoulder_roll_joint",    # axis x
    "joint_15": "left_shoulder_yaw_joint",     # axis z
    "joint_19": "left_elbow_joint",            # axis y, body (.018,0,-.198)
    "joint_8":  "right_shoulder_pitch_joint",  # axis y, body (.005,-.155,.43)
    "joint_12": "right_shoulder_roll_joint",
    "joint_16": "right_shoulder_yaw_joint",
    "joint_20": "right_elbow_joint",
}


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: {SRC} missing. Dump it first:\n"
              f"  $env:OMNISIM_NEWTON_SAVE_MJCF='_scratch/h1_newton_dump.mjcf.xml'; "
              f"$env:HW_NO_WARMUP='1'; "
              f"scripts/dev/run_humanoid_walk_deploy.ps1 -Robot h1 -Duration 16 -Ke 800 -Kd 70",
              file=sys.stderr)
        return 1
    text = SRC.read_text(encoding="utf-8")

    # Auto-derive body names from the joint map: each hinge joint's CHILD body
    # is that joint's link (read the body->joint nesting from the dump in MuJoCo),
    # so body_N -> <joint base>_link, and the free-joint body -> pelvis.
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(SRC))
    body_rename = {}
    for ji in range(m.njnt):
        jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, ji)
        new = JOINT_RENAME.get(jn)
        if not new:
            continue
        bid = int(m.jnt_bodyid[ji])
        bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid)
        if new == "root_free":
            body_rename[bname] = "pelvis"
        else:
            link = new.replace("_joint", "_link")
            body_rename[bname] = link

    for old in sorted(JOINT_RENAME, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', JOINT_RENAME[old], text)
    for old in sorted(body_rename, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', body_rename[old], text)

    # Inject actuator names: alternating pos/vel <general> per joint (matches the
    # G1 trainer convention so name lookups find <joint>_pos / <joint>_vel).
    out_lines = []
    pos_or_vel = "pos"
    in_act = False
    for line in text.splitlines():
        s = line.strip()
        if s == "<actuator>":
            in_act = True
            out_lines.append(line)
            pos_or_vel = "pos"
            continue
        if s == "</actuator>":
            in_act = False
            out_lines.append(line)
            continue
        if in_act and s.startswith("<general "):
            mm = re.search(r'joint="([^"]+)"', s)
            if mm:
                nm = f'{mm.group(1)}_{pos_or_vel}'
                out_lines.append(re.sub(r'<general ', f'<general name="{nm}" ', line))
                pos_or_vel = "vel" if pos_or_vel == "pos" else "pos"
                continue
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"

    DST.write_text(text, encoding="utf-8")
    print(f"saved {DST} ({DST.stat().st_size} bytes)")

    m2 = mujoco.MjModel.from_xml_path(str(DST))
    joints = [mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m2.njnt)]
    acts = [mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m2.nu)]
    print(f"nq={m2.nq} nv={m2.nv} nu={m2.nu} njnt={m2.njnt}")
    print(f"joints: {joints}")
    print(f"actuators: first4={acts[:4]} last2={acts[-2:]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
