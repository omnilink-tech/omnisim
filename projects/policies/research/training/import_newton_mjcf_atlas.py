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

"""Convert OmniSim's Newton-dumped Atlas MJCF (anonymous body_N/joint_N
names) to a trainer-friendly MJCF with readable Atlas URDF joint/body
names. Mirrors `import_newton_mjcf.py` (G1) — see that file's docstring
for the full rationale.

Mapping derived by walking the kinematic tree in the dumped MJCF and
matching each joint to its Atlas URDF counterpart via:
  - parent body (kinematic chain position)
  - joint axis
  - joint limit range
  - body mass (sanity check; e.g. the 84.6 kg utorso is unmistakable)

Source: dump produced by setting OMNISIM_NEWTON_SAVE_MJCF=<path> on
OmniSim launch with `projects/policies/worlds/atlas_stand_deploy.wbt`.

Output: projects/robots/boston_dynamics/atlas/urdf/atlas.mjcf.xml
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
SRC = REPO / "_scratch/atlas_newton_exact.mjcf.xml"
DST = REPO / "projects/robots/boston_dynamics/atlas/urdf/atlas.mjcf.xml"


# Walk-the-tree mapping from the Newton dump (see file docstring).
JOINT_RENAME = {
    "joint_1":  "root_free",
    # Back chain (3 joints up through utorso).
    "joint_2":  "back_bkz",        # body_2 → body_3, axis (0,0,1)
    "joint_5":  "back_bky",        # body_3 → body_4, axis (0,1,0)
    "joint_8":  "back_bkx",        # body_4 → body_5 (utorso, 84.6 kg)
    # Neck.
    "joint_12": "neck_ay",         # body_5 → body_13
    # Left arm (7).
    "joint_11": "l_arm_shz",       # body_5 → body_6
    "joint_16": "l_arm_shx",
    "joint_20": "l_arm_ely",
    "joint_24": "l_arm_elx",
    "joint_26": "l_arm_uwy",
    "joint_28": "l_arm_mwx",
    "joint_30": "l_arm_lwy",
    # Right arm (7).
    "joint_13": "r_arm_shz",       # body_5 → body_14
    "joint_17": "r_arm_shx",
    "joint_21": "r_arm_ely",
    "joint_25": "r_arm_elx",
    "joint_27": "r_arm_uwy",
    "joint_29": "r_arm_mwx",
    "joint_31": "r_arm_lwy",
    # Left leg (6).
    "joint_3":  "l_leg_hpz",       # body_2 → body_21
    "joint_6":  "l_leg_hpx",
    "joint_9":  "l_leg_hpy",
    "joint_14": "l_leg_kny",
    "joint_18": "l_leg_aky",
    "joint_22": "l_leg_akx",
    # Right leg (6).
    "joint_4":  "r_leg_hpz",       # body_2 → body_27
    "joint_7":  "r_leg_hpx",
    "joint_10": "r_leg_hpy",
    "joint_15": "r_leg_kny",
    "joint_19": "r_leg_aky",
    "joint_23": "r_leg_akx",
    # Floating placeholder bodies' free joints (no controller use).
    "joint_32": "_placeholder_free_0",
    "joint_33": "_placeholder_free_1",
}

BODY_RENAME = {
    "body_2":  "pelvis",
    "body_3":  "ltorso",
    "body_4":  "mtorso",
    "body_5":  "utorso",
    # Neck/head.
    "body_13": "head",
    # Left arm.
    "body_6":  "l_clav",
    "body_7":  "l_scap",
    "body_8":  "l_uarm",
    "body_9":  "l_larm",
    "body_10": "l_ufarm",
    "body_11": "l_lfarm",
    "body_12": "l_hand",
    # Right arm.
    "body_14": "r_clav",
    "body_15": "r_scap",
    "body_16": "r_uarm",
    "body_17": "r_larm",
    "body_18": "r_ufarm",
    "body_19": "r_lfarm",
    "body_20": "r_hand",
    # Left leg.
    "body_21": "l_uglut",
    "body_22": "l_lglut",
    "body_23": "l_uleg",
    "body_24": "l_lleg",
    "body_25": "l_talus",
    "body_26": "l_foot",
    # Right leg.
    "body_27": "r_uglut",
    "body_28": "r_lglut",
    "body_29": "r_uleg",
    "body_30": "r_lleg",
    "body_31": "r_talus",
    "body_32": "r_foot",
    # Placeholder bodies.
    "body_0":  "_placeholder_0",
    "body_1":  "_placeholder_1",
}


def main() -> int:
    if not SRC.exists():
        print(f"ERROR: {SRC} missing. Dump it via:\n"
              f"  OMNISIM_NEWTON_SAVE_MJCF={SRC} \\\n"
              f"  OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1 \\\n"
              f"  OMNISIM_URDF_USE_INERTIA=1 \\\n"
              f"  msys64/mingw64/bin/omnisim-bin.exe \\\n"
              f"  projects/policies/worlds/atlas_stand_deploy.wbt",
              file=sys.stderr)
        return 1
    text = SRC.read_text(encoding="utf-8")

    # Longest-first rename so joint_30 isn't shadowed by joint_3.
    for old in sorted(JOINT_RENAME, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', JOINT_RENAME[old], text)
    for old in sorted(BODY_RENAME, key=len, reverse=True):
        text = re.sub(rf'\b{old}\b', BODY_RENAME[old], text)

    # Inject readable actuator names. Newton writes alternating
    # pos/vel `<general joint="…"/>` entries with no name attr.
    out_lines = []
    in_actuator = False
    next_kind = "pos"
    for line in text.splitlines():
        s = line.strip()
        if s == "<actuator>":
            in_actuator = True
            next_kind = "pos"
            out_lines.append(line)
            continue
        if s == "</actuator>":
            in_actuator = False
            out_lines.append(line)
            continue
        if in_actuator and s.startswith("<general "):
            m = re.search(r'joint="([^"]+)"', s)
            if m:
                jn = m.group(1)
                name = f"{jn}_{next_kind}"
                line2 = re.sub(r'<general ', f'<general name="{name}" ', line)
                out_lines.append(line2)
                next_kind = "vel" if next_kind == "pos" else "pos"
                continue
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"

    # Bump njmax / nconmax so mujoco_warp doesn't spam "nefc overflow"
    # under DR. The Newton-dumped MJCF has no <size> tag, so it falls
    # back to small defaults; with 8 foot-corner contacts + DR pushes
    # + transient falls hitting other limbs on ground, 65-70 constraint
    # rows are routine. 256 is generous headroom.
    if "<size " not in text:
        text = re.sub(
            r"(<compiler[^/]*/>)",
            lambda m: m.group(1) + '\n  <size njmax="256" nconmax="128"/>',
            text, count=1,
        )

    # Disable contacts on arm-chain geoms. The Newton-dumped MJCF uses
    # convex-hull collision shapes that vastly overstate arm volumes;
    # at any standing pose the l_scap/l_uarm/l_larm hulls intersect the
    # utorso hull by 10-26 cm, producing massive constraint forces on
    # the FIRST physics step that punch the pelvis with vang > 14 rad/s.
    # The source URDF defines hand/forearm physics as NULL anyway
    # ("As 'physics' is set to NULL, collisions will have no effect"
    # in the OmniSim load log) — Newton's dump roundtripped them as
    # ordinary geoms. Zeroing contype/conaffinity restores the URDF's
    # intent: arms have inertia + actuators but no contact forces.
    NO_COLLIDE_BODIES = {
        "l_clav", "l_scap", "l_uarm", "l_larm",
        "l_ufarm", "l_lfarm", "l_hand",
        "r_clav", "r_scap", "r_uarm", "r_larm",
        "r_ufarm", "r_lfarm", "r_hand",
        "head",
    }
    out_lines = []
    cur_body = None
    body_stack = []
    for line in text.splitlines():
        s = line.strip()
        # Track the innermost <body name="..."> we're inside.
        m_body = re.match(r'<body\s+name="([^"]+)"', s)
        if m_body:
            body_stack.append(m_body.group(1))
            cur_body = m_body.group(1)
        elif s.startswith("</body>"):
            if body_stack:
                body_stack.pop()
            cur_body = body_stack[-1] if body_stack else None
        if cur_body in NO_COLLIDE_BODIES and s.startswith("<geom "):
            line = re.sub(r'contype="\d+"', 'contype="0"', line)
            line = re.sub(r'conaffinity="\d+"', 'conaffinity="0"', line)
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(text, encoding="utf-8")
    print(f"saved {DST} ({DST.stat().st_size} bytes)")

    # Round-trip verification.
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(DST))
    print(f"nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt}")
    joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
              for i in range(m.njnt)]
    print(f"joints ({len(joints)}): {joints[:6]} ... {joints[-3:]}")
    acts = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(m.nu)]
    print(f"actuators ({len(acts)}): first 4 = {acts[:4]} ... last 2 = {acts[-2:]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
