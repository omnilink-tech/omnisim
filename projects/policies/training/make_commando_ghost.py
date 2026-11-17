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

"""FK-build the G1 COMMANDO-CRAWL ghost (draft v1) — the Shadowing reference.

Owner-chosen posture (probe-validated in crawl_feasibility_probe.py): forearms +
knees down, torso ~horizontal, pelvis low (~0.28 m, under-the-wire), arms
comfortable (~50-70% of the 25 N.m limit). This builds a CYCLIC gait around
that anchor pose: a contralateral reach-and-pull, the natural crawl coordination
  diagonal A = {left arm, right knee}   swing while
  diagonal B = {right arm, left knee}   plant, then alternate.
Each limb sweeps fore/aft at the cycle frequency and lifts during its swing to
clear the ground; the body advances at `vx` (a slow, deliberate crawl).

This is a DRAFT reference per the owner's "FK-build then RECORD" plan: train a
tracker to shadow it, then REC_FOLD-record what the robot actually does as the
achievable ghost (ghost-design rule 1/4). It is NOT expected to be perfect.

Schema matches the cyclic walk-ghost family (leg_lut/arm_lut/elbow_lut/att_lut,
nb bins on the gait clock, vx = commanded crawl speed). NOTE the nominal crawl
base attitude (torso ~horizontal, ~90 deg pitch) is a LAUNCH/spawn concern for
the prone training mode (Phase 2), NOT att_lut — att_lut here carries only the
small attitude deviations the corridor machinery expects, exactly like the walk.

Run:  python projects/policies/training/make_commando_ghost.py
Then: python projects/policies/training/build_keypoints.py <out>.json --links
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "projects/policies/ghosts/g1/ghost_crawl_v1_lut.json"

WB_JOINTS = [
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
IDX = {j: i for i, j in enumerate(WB_JOINTS)}

# Anchor pose = the probe-validated commando static pose.
ANCHOR = {
    "left_hip_pitch_joint": -0.80, "right_hip_pitch_joint": -0.80,
    "left_hip_roll_joint":  0.22,  "right_hip_roll_joint": -0.22,
    "left_hip_yaw_joint":   0.00,  "right_hip_yaw_joint":  0.00,
    "left_knee_joint":      0.80,  "right_knee_joint":     0.80,
    "left_ankle_pitch_joint": -0.55, "right_ankle_pitch_joint": -0.55,
    "left_ankle_roll_joint":  0.00,  "right_ankle_roll_joint":  0.00,
    "waist_yaw_joint": 0.00,
    "left_shoulder_pitch_joint": -1.50, "right_shoulder_pitch_joint": -1.50,
    "left_shoulder_roll_joint":   0.25, "right_shoulder_roll_joint": -0.25,
    "left_shoulder_yaw_joint":    0.00, "right_shoulder_yaw_joint":   0.00,
    "left_elbow_joint":  0.35, "right_elbow_joint":  0.35,
    "left_wrist_roll_joint": 0.00, "right_wrist_roll_joint": 0.00,
}

# ── cycle parameters ─────────────────────────────────────────────────
NB = 64
CYCLE_S = 2.0                 # slow, deliberate crawl stroke
VX = 0.06                     # m/s forward (draft; refine after RECORD)
A_ARM_FORE = 0.20             # shoulder-pitch fore/aft sweep (rad)
A_ARM_LIFT = 0.35             # elbow lift during swing (straighten to clear)
A_LEG_FORE = 0.16             # hip-pitch fore/aft sweep (rad)
A_LEG_LIFT = 0.20             # knee lift during swing (rad)
BASE_PITCH_DEG = 90.0         # nominal crawl attitude (handled at spawn, Phase 2)


def swing_lift(phi_local):
    """Positive half-sine bump over a limb's swing window [0,1)."""
    return max(0.0, math.sin(math.pi * phi_local))


def build():
    wb = np.zeros((NB, 23), dtype=float)
    for b in range(NB):
        phi = b / NB
        q = dict(ANCHOR)
        # diagonal A (left arm + right knee): stance [0,0.5) sweeping back,
        # swing [0.5,1) reaching forward + lifting. p in [-1(back),+1(front)].
        pA = math.cos(2 * math.pi * phi)          # +1 front @0, -1 back @0.5
        pB = -pA                                   # diagonal B, 180 deg out
        # swing windows: A swings when it is travelling back->front = [0.5,1)
        swA = swing_lift((phi - 0.5) % 1.0) if phi >= 0.5 else 0.0
        swB = swing_lift((phi - 0.0) % 1.0) if phi < 0.5 else 0.0

        # arms: more-negative shoulder_pitch = reach forward
        q["left_shoulder_pitch_joint"]  = ANCHOR["left_shoulder_pitch_joint"]  - A_ARM_FORE * pA
        q["right_shoulder_pitch_joint"] = ANCHOR["right_shoulder_pitch_joint"] - A_ARM_FORE * pB
        q["left_elbow_joint"]  = ANCHOR["left_elbow_joint"]  + A_ARM_LIFT * swA
        q["right_elbow_joint"] = ANCHOR["right_elbow_joint"] + A_ARM_LIFT * swB

        # legs (contralateral to the arms): more-negative hip_pitch = knee fwd
        q["right_hip_pitch_joint"] = ANCHOR["right_hip_pitch_joint"] - A_LEG_FORE * pA
        q["left_hip_pitch_joint"]  = ANCHOR["left_hip_pitch_joint"]  - A_LEG_FORE * pB
        q["right_knee_joint"] = ANCHOR["right_knee_joint"] - A_LEG_LIFT * swA
        q["left_knee_joint"]  = ANCHOR["left_knee_joint"]  - A_LEG_LIFT * swB

        for j, v in q.items():
            wb[b, IDX[j]] = v

    lut = {
        "nb": NB, "freq": 1.0 / CYCLE_S, "cycle_s": CYCLE_S, "vx": VX,
        "arm_A": A_ARM_FORE,
        "source": ("COMMANDO-CRAWL ghost v1 (FK-built draft, owner posture "
                   "2026-07-07): forearms+knees, torso~horizontal, contralateral "
                   "reach-pull; anchor = probe-validated commando pose. Nominal "
                   f"base pitch {BASE_PITCH_DEG} deg handled at spawn (Phase 2). "
                   "DRAFT: train->REC_FOLD to make achievable (rule 1/4)."),
        "wb_joints": WB_JOINTS,
        "wb_lut": wb.tolist(),
        "leg_lut": wb[:, 0:12].tolist(),
        "arm_lut": wb[:, [IDX["left_shoulder_pitch_joint"],
                          IDX["right_shoulder_pitch_joint"]]].tolist(),
        "elbow_lut": wb[:, [IDX["left_elbow_joint"],
                            IDX["right_elbow_joint"]]].tolist(),
        "att_lut": [[0.0, 0.0] for _ in range(NB)],
        # nominal crawl base attitude + height for the hologram (att_lut stays
        # the small per-bin deviation; base_pitch is the horizontal torso).
        "base_pitch": math.radians(BASE_PITCH_DEG),
        "ghost_z": 0.30,
        "ghost_y": 0.0,
    }
    OUT.write_text(json.dumps(lut))
    return lut


def fk_sanity(lut):
    """Confirm the cycle is sane: support limbs stay near a ground plane and
    swing limbs lift. Uses the crawl model + the same converter as the probe."""
    import importlib.util
    import mujoco
    u = REPO / "projects/policies/research/backends/_urdf_to_mjcf.py"
    s = importlib.util.spec_from_file_location("_u2m", u)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    urdf = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_crawl.urdf"
    md = (REPO / "projects/robots/unitree/g1/urdf/meshes").as_posix()
    tmp = urdf.with_name("g1_23dof_omnisim_crawl_abs.urdf")
    tmp.write_text(urdf.read_text().replace('filename="meshes/', f'filename="{md}/'))
    model = m.load_or_convert(tmp, actuator_joints=WB_JOINTS)
    data = mujoco.MjData(model)
    q = math.radians(BASE_PITCH_DEG)
    quat = [math.cos(q / 2), 0.0, math.sin(q / 2), 0.0]
    jadr = {j: model.joint(j).qposadr[0] for j in WB_JOINTS}
    print(f"{'bin':>4} {'L-hand':>7}{'R-hand':>7}{'L-knee':>7}{'R-knee':>7}"
          f"{'L-elb':>7}{'R-elb':>7}   (world z of body origins)")
    for b in range(0, NB, NB // 8):
        data.qpos[:] = 0.0
        data.qpos[3:7] = quat
        for i, j in enumerate(WB_JOINTS):
            data.qpos[jadr[j]] = lut["wb_lut"][b][i]
        mujoco.mj_forward(model, data)
        z = {k: float(data.body(v).xpos[2]) for k, v in {
            "lh": "left_wrist_roll_rubber_hand", "rh": "right_wrist_roll_rubber_hand",
            "lk": "left_knee_link", "rk": "right_knee_link",
            "le": "left_elbow_link", "re": "right_elbow_link"}.items()}
        print(f"{b:>4} {z['lh']:7.3f}{z['rh']:7.3f}{z['lk']:7.3f}{z['rk']:7.3f}"
              f"{z['le']:7.3f}{z['re']:7.3f}")


if __name__ == "__main__":
    lut = build()
    print(f"wrote {OUT.relative_to(REPO)}  (nb={NB}, cycle_s={CYCLE_S}, vx={VX})")
    fk_sanity(lut)
