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

"""WALK -> STAND -> CLIMB composed sequence ghost (owner 2026-07-08: "walk in from a
bit far away, stand in front of the stairs, then climb one by one" -- so the robot
STEPS ONTO the treads from a stable approach instead of being thrown at the risers
from a standstill 12cm away and collapsing into them).

Composed (walk-stop-walk pattern, design_walkstop_ghost.py) from:
  walk  : the parametric human-gait walk (g1_human_gait) advancing on the flat
  stand : a brief nominal settle square in front of the first riser
  climb : the ALREADY-DESIGNED stair-climb ghost (ghost_stair_climb_lut.json) --
          its legs + rising root, translated to start where the stand ends.
Cosine blends at the seams. Emits the standard GHOST_SEQ lut (seq + hold_end, rising
root_lut z over the climb) + synthesized wb. Train ONE policy on this (warm from the
walker) so it walks in, stands, and climbs as one behaviour; deploy solo (no BATON
seq-specialist surgery). Run build_keypoints.py --links after.

  python projects/policies/training/design_walk_climb_ghost.py [out] \
     [--approach-m 2.0] [--stand-s 1.0] [--walk-vx 0.40] \
     [--climb-lut ghost_stair_climb_lut.json]
"""
from __future__ import annotations
import argparse, json, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.environ.get("OMNISIM_HOME", os.path.abspath(os.path.join(HERE, "..", "..", "..")))
sys.path.insert(0, RT)
G = os.path.join(RT, "projects", "policies", "controllers", "g1_ghost")
from projects.policies.control.gait.g1_human_gait import GaitParams, targets_np  # noqa: E402

DT = 0.016
LEG12 = list(range(12))                      # targets_np returns leg[0:12] in canonical order


def _cos(u):
    return 0.5 * (1.0 - math.cos(math.pi * max(0.0, min(1.0, u))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=os.path.join(G, "ghost_walk_climb_lut.json"))
    ap.add_argument("--climb-lut", default=os.path.join(G, "ghost_stair_climb_lut.json"))
    ap.add_argument("--approach-m", type=float, default=2.0, help="flat walk-in distance (m)")
    ap.add_argument("--stand-s", type=float, default=1.0, help="settle in front of the stairs (s)")
    ap.add_argument("--walk-vx", type=float, default=0.40)
    ap.add_argument("--walk-freq", type=float, default=1.3)
    ap.add_argument("--blend-s", type=float, default=0.4)
    ap.add_argument("--robot-spec", default=os.path.join(HERE, "specs", "retarget_g1.json"))
    a = ap.parse_args()

    spec = json.load(open(a.robot_spec))
    cl = json.load(open(a.climb_lut))
    cLEG = np.array(cl["leg_lut"], np.float32)          # (Nc,12) climb legs
    cARM = np.array(cl.get("arm_lut", np.zeros((len(cLEG), 2))), np.float32)
    cATT = np.array(cl.get("att_lut", np.zeros((len(cLEG), 2))), np.float32)
    cROOT = np.array(cl["root_lut"], np.float32)        # (Nc,4) x y z yaw -- rising
    nb_c = len(cLEG)

    gp = GaitParams(vx=a.walk_vx, freq=a.walk_freq)
    walk_z = gp.pelvis_height                            # ~0.755
    climb_z0 = float(cROOT[0, 2])                        # climb starts crouched (~0.68)
    # --- WALK: integer gait cycles covering approach-m at walk_vx ---
    cyc_T = 1.0 / gp.freq
    walk_s = a.approach_m / a.walk_vx
    n_cyc = max(1, int(round(walk_s / cyc_T)))
    walk_s = n_cyc * cyc_T
    nb_w = int(round(walk_s / DT))
    nb_st = int(round(a.stand_s / DT))
    nb_bl = int(round(a.blend_s / DT))

    stand_leg = np.array([spec["nominal"]["hip_pitch"], 0, 0, spec["nominal"]["knee"],
                          spec["nominal"]["ankle_pitch"], 0] * 2, np.float32)
    stand_arm = np.array([0.30, 0.30], np.float32)

    LEG, ARM, ATT, VX = [], [], [], []
    # walk (ramp stride from a standing start via targets_np t_since_start)
    for i in range(nb_w):
        t = i * DT
        ph = 2 * math.pi * gp.freq * t
        legs, arms, _ = targets_np(ph, gp, t_since_start=t)
        LEG.append(np.asarray(legs)[LEG12]); ARM.append(np.array([arms[0], arms[5]], np.float32))
        ATT.append([0.0, 0.0]); VX.append(a.walk_vx * min(1.0, t / max(1e-3, gp.ramp_s)))
    # blend walk -> stand
    for i in range(nb_bl):
        u = _cos((i + 1) / nb_bl)
        LEG.append((1 - u) * LEG[-1] + u * stand_leg)
        ARM.append((1 - u) * ARM[-1] + u * stand_arm); ATT.append([0.0, 0.0]); VX.append((1 - u) * a.walk_vx)
    # stand
    for i in range(nb_st):
        LEG.append(stand_leg.copy()); ARM.append(stand_arm.copy()); ATT.append([0.0, 0.0]); VX.append(0.0)
    n_pre = len(LEG)                                     # bins before the climb
    # blend stand -> climb[0]
    for i in range(nb_bl):
        u = _cos((i + 1) / nb_bl)
        LEG.append((1 - u) * stand_leg + u * cLEG[0])
        ARM.append((1 - u) * stand_arm + u * cARM[0]); ATT.append([0.0, float(u * cATT[0, 1])]); VX.append(0.0)
    # climb (verbatim legs/arm/att)
    for i in range(nb_c):
        LEG.append(cLEG[i]); ARM.append(cARM[i]); ATT.append([float(cATT[i, 0]), float(cATT[i, 1])]); VX.append(0.0)

    LEG = np.array(LEG, np.float32); ARM = np.array(ARM, np.float32); ATT = np.array(ATT, np.float32)
    VX = np.array(VX, np.float32); nb = len(LEG)

    # --- ROOT: x = cumulative (walk vx during walk, then the climb's own dx); z blends
    #     walk_z -> climb_z0 across the stand, then follows the climb's rising z ---
    x = np.zeros(nb); z = np.full(nb, walk_z, np.float32)
    for i in range(1, n_pre):
        x[i] = x[i - 1] + VX[i] * DT
    x_base = x[n_pre - 1]                                # x at the base of the stairs
    # pre-climb z ramp to the crouch over the last part of the stand
    for i in range(nb_bl):
        z[n_pre + i] = walk_z + (climb_z0 - walk_z) * _cos((i + 1) / nb_bl)
    # climb: shift the climb root to start at x_base, z from the climb lut
    c0x = float(cROOT[0, 0])
    for i in range(nb_c):
        j = n_pre + nb_bl + i
        x[j] = x_base + (float(cROOT[i, 0]) - c0x)
        z[j] = float(cROOT[i, 2])
    # backfill z during the stand-blend already done; hold x during stand
    for i in range(n_pre, n_pre + nb_bl):
        x[i] = x_base
    ROOT = np.stack([x, np.zeros(nb), z, np.zeros(nb)], 1).astype(np.float32)

    # --- synthesized whole-body (legs + shoulder pitch + nominal elbow) ---
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(RT, spec["model"]))
    JN = [n for n in [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)] if n]
    elbow = float(cl.get("elbow_lut", [[1.2, 1.2]])[0][0])
    WB = np.zeros((nb, len(JN)), np.float32)
    for j, nm in enumerate(JN):
        if nm in spec["legs12"]:
            WB[:, j] = LEG[:, spec["legs12"].index(nm)]
        elif nm == "left_shoulder_pitch_joint":
            WB[:, j] = ARM[:, 0]
        elif nm == "right_shoulder_pitch_joint":
            WB[:, j] = ARM[:, 1]
        elif "elbow" in nm:
            WB[:, j] = elbow

    cyc_s = nb * DT
    out = {"nb": nb, "freq": 1.0 / cyc_s, "cycle_s": cyc_s, "vx": 0.0, "seq": True, "hold_end": True,
           "arm_A": float(cl.get("arm_A", 0.2)), "shroll": float(cl.get("shroll", 0.15)),
           "leg_lut": LEG.tolist(), "arm_lut": ARM.tolist(), "att_lut": ATT.tolist(),
           "elbow_lut": [[elbow, elbow]] * nb, "root_lut": ROOT.tolist(),
           "wb_joints": JN, "wb_lut": WB.tolist(),
           "source": ("WALK->STAND->CLIMB composed seq (owner 2026-07-08): %.1fm flat walk-in @%.2f m/s "
                      "(%d cyc) + %.1fs stand + climb from %s. Robot steps ONTO the treads from a stable "
                      "approach. seq+hold_end, rising root over the climb." %
                      (a.approach_m, a.walk_vx, n_cyc, a.stand_s, os.path.basename(a.climb_lut)))}
    json.dump(out, open(a.out, "w"))
    print("WALK->STAND->CLIMB: nb=%d (%.1fs)  walk %.1fm (%d cyc, %d bins) + stand %.1fs + climb %d bins" %
          (nb, cyc_s, a.approach_m, n_cyc, nb_w, a.stand_s, nb_c))
    print("  root x: 0 -> %.2f (base) -> %.2f (top);  z: %.2f (walk) -> %.2f (crouch) -> %.2f (top)" %
          (x_base, x[-1], walk_z, climb_z0, z[-1]))
    print("  base-of-stairs at x=%.2f -> put StairProfile x_start there (deploy world)" % x_base)
    print("  wrote %s  (wb_joints=%d)" % (a.out, len(JN)))


if __name__ == "__main__":
    main()
