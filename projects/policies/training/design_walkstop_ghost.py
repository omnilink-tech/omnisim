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

"""WALK-STOP-WALK sequence ghost -- the right-sized TIME-AXIS test (owner direction 2026-07-04).

After parking the dance campaign: sequences failed on dynamics content (weight placement,
limb momentum), never on the time axis itself. This ghost isolates the time axis with ZERO
new dynamics content: it is composed entirely from ALREADY-ACHIEVED references --
  walk : the owner-approved recorded walking ghost (ghost_v3c_lut.json, the reference the
         champion tracks at WBMATCH 0.913 / survival 1.0)
  stand: the proven deterministic stand crouch (the launch/settle layer)
Timeline (finite, hold_end):  [accel | walk ~5s | decel | stand 5s | accel | walk ~5s | decel |
stand tail] -- then the player PINS on the final stand bin forever (lut flag "hold_end").
Rule-4 compliant by construction; the only synthetic parts are two 0.4s cosine blends, entered
and exited at gait phase 0 (integer cycle count, double-support-adjacent).

Emits the standard GHOST_SEQ lut (leg/arm/att/root + synthesized wb for the balance gate and
the whole-body hologram). Run GATE 1d after; expect near-clean (both regimes are achieved).

Usage: python design_walkstop_ghost.py [out_lut.json] [--walk-s 5] [--stand-s 5]
       [--walk-lut .../ghost_v3c_lut.json] [--robot-spec specs/retarget_g1.json]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.environ.get("OMNISIM_HOME", os.getcwd())
G = os.path.join(RT, "projects", "policies", "controllers", "g1_ghost")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=os.path.join(G, "ghost_walkstop_lut.json"))
    ap.add_argument("--walk-lut", default=os.path.join(G, "ghost_v3c_lut.json"))
    ap.add_argument("--walk-s", type=float, default=5.0)
    ap.add_argument("--stand-s", type=float, default=5.0)
    ap.add_argument("--blend-s", type=float, default=0.4)
    ap.add_argument("--z", type=float, default=0.74)
    ap.add_argument("--robot-spec", default=os.path.join(HERE, "specs", "retarget_g1.json"))
    a = ap.parse_args()

    spec = json.load(open(a.robot_spec))
    w = json.load(open(a.walk_lut))
    leg = np.array(w["leg_lut"], np.float32)          # (nbw, 12), one gait cycle
    arm = np.array(w["arm_lut"], np.float32)          # (nbw, 2)
    att = np.array(w.get("att_lut", np.zeros((len(leg), 2))), np.float32)
    nbw = len(leg)
    gait_T = 1.0 / float(w["freq"])                   # one gait cycle, s
    vx = float(w.get("vx", 0.4))

    n_cyc = max(1, int(round(a.walk_s / gait_T)))     # integer cycles: exit at gait phase 0
    walk_s = n_cyc * gait_T
    DT = 0.016                                        # sequence bin ~ trainer tick
    nb_walk = int(round(walk_s / DT))
    nb_blend = int(round(a.blend_s / DT))
    nb_stand = int(round(a.stand_s / DT))
    nb_tail = int(round(2.0 / DT))
    nb = 2 * (nb_blend + nb_walk + nb_blend) + nb_stand + nb_tail
    cyc_s = nb * DT

    # stand references: the walking lut's own phase-0 pose blended toward the nominal crouch
    stand_leg = np.array([spec["nominal"].get("hip_pitch", -0.10), 0, 0,
                          spec["nominal"].get("knee", 0.30),
                          spec["nominal"].get("ankle_pitch", -0.20), 0] * 2, np.float32)
    stand_arm = np.array([0.30, 0.30], np.float32)

    LEG = np.zeros((nb, 12), np.float32); ARM = np.zeros((nb, 2), np.float32)
    ATT = np.zeros((nb, 2), np.float32); VX = np.zeros(nb, np.float32)
    i0 = 0

    def _bout(i0):
        # accel blend: stand -> gait phase 0.., then N integer cycles, then decel -> stand
        for i in range(nb_blend):
            u = 0.5 * (1 - np.cos(np.pi * (i + 1) / nb_blend))
            gi = int((((i + 1) * DT / gait_T) % 1.0) * nbw) % nbw
            LEG[i0 + i] = (1 - u) * stand_leg + u * leg[gi]
            ARM[i0 + i] = (1 - u) * stand_arm + u * arm[gi]
            ATT[i0 + i] = u * att[gi]
            VX[i0 + i] = u * vx
        i0 += nb_blend
        for i in range(nb_walk):
            gi = int((((nb_blend + i + 1) * DT / gait_T) % 1.0) * nbw) % nbw
            LEG[i0 + i] = leg[gi]; ARM[i0 + i] = arm[gi]; ATT[i0 + i] = att[gi]; VX[i0 + i] = vx
        i0 += nb_walk
        for i in range(nb_blend):
            u = 0.5 * (1 - np.cos(np.pi * (i + 1) / nb_blend))
            gi = int((((nb_blend + nb_walk + i + 1) * DT / gait_T) % 1.0) * nbw) % nbw
            LEG[i0 + i] = (1 - u) * leg[gi] + u * stand_leg
            ARM[i0 + i] = (1 - u) * arm[gi] + u * stand_arm
            ATT[i0 + i] = (1 - u) * att[gi]
            VX[i0 + i] = (1 - u) * vx
        return i0 + nb_blend

    i0 = _bout(i0)                       # walk bout 1
    for i in range(nb_stand):            # stand 5 s
        LEG[i0 + i] = stand_leg; ARM[i0 + i] = stand_arm
    i0 += nb_stand
    i0 = _bout(i0)                       # walk bout 2
    for i in range(nb_tail):             # final stand tail -- hold_end pins here forever
        LEG[i0 + i] = stand_leg; ARM[i0 + i] = stand_arm

    x = np.cumsum(VX) * DT
    x -= x[0]
    root = np.stack([x, np.zeros(nb), np.full(nb, a.z), np.zeros(nb)], 1)

    # synthesized whole-body track (for the balance gate + whole-body hologram): legs + shoulder
    # pitch from the composition, remaining joints at nominal
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(RT, spec["model"]))
    JN = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
    WB = np.zeros((nb, len(JN)), np.float32)
    for j, nm in enumerate(JN):
        if nm in spec["legs12"]:
            WB[:, j] = LEG[:, spec["legs12"].index(nm)]
        elif nm == "left_shoulder_pitch_joint":
            WB[:, j] = ARM[:, 0]
        elif nm == "right_shoulder_pitch_joint":
            WB[:, j] = ARM[:, 1]
        elif "elbow" in nm:
            WB[:, j] = spec["nominal"].get("elbow", 0.5)

    out = {"nb": nb, "freq": 1.0 / cyc_s, "cycle_s": cyc_s, "vx": 0.0, "seq": True,
           "hold_end": True,
           "arm_A": float(w.get("arm_A", 0.3)),
           "leg_lut": [[float(v) for v in r] for r in LEG],
           "arm_lut": [[float(v) for v in r] for r in ARM],
           "att_lut": [[float(v) for v in r] for r in ATT],
           "root_lut": [[float(v) for v in r] for r in root],
           "wb_joints": JN, "wb_lut": [[float(v) for v in r] for r in WB],
           "source": "WALK-STOP-WALK time-axis ghost: %d gait cycles (%.1fs) walk + %.1fs stand, "
                     "0.4s cosine blends at gait phase 0; composed from %s (achieved walking "
                     "reference) + nominal stand; TWO bouts, hold_end pins the final stand. "
                     "Rule-4 by construction." % (n_cyc, walk_s, a.stand_s, os.path.basename(a.walk_lut))}
    json.dump(out, open(a.out, "w"))
    print("walk-stop-walk: nb=%d total=%.2fs [walk %.2fs (%d cyc, vx %.2f) | stand %.1f | "
          "walk %.2fs | stand PERMANENT (hold_end)] -> %s"
          % (nb, cyc_s, walk_s, n_cyc, vx, a.stand_s, walk_s, a.out))


if __name__ == "__main__":
    main()
