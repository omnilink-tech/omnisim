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

"""STAIR-CLIMB sequence ghost -- the feasible reference the climb specialist shadows.

A G1 ascending a flight of stairs is a NON-periodic, MONOTONICALLY-RISING motion, so
(per the ghost-authoring map) it is a TIMED SEQUENCE ghost (seq:true, hold_end:true) whose
root_lut[:,2] carries the base rise -- a periodic leg_lut has no z channel and cannot express
the climb. The geometry is the shared StairProfile (hill_terrain.StairProfile), so the ghost,
the in-engine trainer terrain, and the deploy world all agree on riser/going by construction.

Feasibility-first design (the G1's legs are only L1+L2 = 0.636 m, so a human step-OVER-step
climb -- 2 treads = 0.52 m between successive same-foot placements -- over-extends the trailing
leg). This ghost uses a STEP-TO gait: the lead foot rises one tread, the trailing foot joins it
on the same tread, lead alternates, so the two feet are never more than ONE tread (going x, riser
z) apart -- every pose stays inside the reach envelope. Feet land ON the treads (explicit stair
IK via g1_human_gait._leg_ik, the calibrated 2-link sagittal solver), the pelvis rides above the
lower/support foot and rises tread-by-tread, the torso holds a small forward climb lean, and the
arms counter-swing gently. Timeline:  [settle | (lead-up | join) x n_steps | stand on landing];
hold_end pins the final standing bin.

Emits the standard GHOST_SEQ lut (leg/arm/att/root + synthesized wb). Run build_keypoints.py
--links afterward for the kp/lp/lo ruler tables, then ghost_doctor.py (read T0 = URDF limits as
the hard gate; T1/T2/T3 and ghost_validator are periodic-walk-calibrated -- informational only).

Usage:
  python projects/policies/training/design_stair_climb_ghost.py \
      [out_lut.json] [--riser 0.10] [--going 0.26] [--n-steps 5] [--step-s 0.7] \
      [--clear 0.07] [--ride 0.72] [--lean 0.12] [--settle-s 0.8] [--top-s 2.0] \
      [--arm-A 0.35] [--robot-spec specs/retarget_g1.json]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RT = os.environ.get("OMNISIM_HOME", os.path.abspath(os.path.join(HERE, "..", "..", "..")))
sys.path.insert(0, RT)
G = os.path.join(RT, "projects", "policies", "controllers", "g1_ghost")

from projects.policies.control.gait.g1_human_gait import (  # noqa: E402
    _leg_ik, _ankle_for_foot_pitch, _quintic, L1, L2, HIP_DROP, SOLE_DROP,
)
sys.path.insert(0, os.path.join(RT, "projects", "policies", "research", "shadowing"))
from hill_terrain import StairProfile  # noqa: E402

DT = 0.016                      # sequence bin == trainer tick
REACH = (L1 + L2)               # 0.636 m max leg length (hip anchor -> ankle anchor)
STEP_WIDTH = 0.14               # lateral foot separation (m): feet under the hips
ANK_LO, ANK_HI = -0.87, 0.52    # G1 ankle_pitch URDF limits (ghost_doctor T0)


def _leg_from_foot(px, pz, pitch, fx, fz_sole, foot_pitch):
    """Stair IK: given the pelvis-origin world (px,pz) sagittal position, pelvis
    forward-lean `pitch` (rad, + = torso top forward), and a foot SOLE world
    target (fx, fz_sole), return (q_hip_pitch, q_knee, q_ankle_pitch, reach_D).
    Feet aim flat in WORLD; the ankle is CLIPPED to its URDF range -- when a deep
    crouch + lean demands more dorsiflexion than the G1 has, the foot pitches
    (honest) and the RL tracker resolves exact tread contact."""
    ax, az = fx, fz_sole + SOLE_DROP                       # ankle-pitch anchor world
    c, s = math.cos(pitch), math.sin(pitch)
    hx = px + HIP_DROP * s                                 # hip anchor world (pelvis frame (0,-HIP_DROP) rotated)
    hz = pz - HIP_DROP * c
    dxw, dzw = ax - hx, az - hz                            # foot rel hip, world (forward, up)
    dx_p = dxw * c + dzw * s                               # rotate into pelvis frame
    dz_up = -dxw * s + dzw * c
    D = math.hypot(dx_p, dz_up)
    qh, qk = _leg_ik(dx_p, -dz_up)                         # _leg_ik dz is DOWNWARD
    qa = _ankle_for_foot_pitch(qh, qk, foot_pitch + pitch)  # keep sole flat in world
    qa = max(ANK_LO, min(ANK_HI, qa))                       # clip to URDF ankle range
    return qh, qk, qa, D


def build(a):
    # x_start = the FOOT_PAD (0.12) so the pelvis STARTS at world x=0: the recipe
    # zeroes spawn x, so the training stairs (StairProfile x_start=0.12) must sit
    # right in front of x=0. The ghost's absolute x is otherwise irrelevant to
    # training (only its root DERIVATIVE feeds the command) but must match the
    # preview + world for the hologram to line up.
    sp = StairProfile(riser=a.riser, going=a.going, n_steps=a.n_steps, x_start=a.x_start)
    x_start = sp.x_start

    def tread_x(k):
        # foot x on tread k (k>=1); k==0 is the floor pad just before the first riser
        if k <= 0:
            return x_start - 0.12
        if k >= a.n_steps:
            return x_start + a.n_steps * a.going + 0.12     # onto the top landing
        return x_start + (k - 0.5) * a.going

    def tread_z(k):
        return max(0, min(k, a.n_steps)) * a.riser

    # ---- footstep plan: step-to, alternating lead --------------------------
    feet = {"L": 0, "R": 0}
    lead = "L"
    moves = []                                             # (foot, from_k, to_k, t0, t1)
    t = a.settle_s
    for k in range(1, a.n_steps + 1):
        other = "R" if lead == "L" else "L"
        moves.append((lead, feet[lead], k, t, t + a.step_s)); feet[lead] = k; t += a.step_s
        moves.append((other, feet[other], k, t, t + a.step_s)); feet[other] = k; t += a.step_s
        lead = other
    climb_end = t
    total_t = climb_end + a.top_s
    nb = int(round(total_t / DT))

    fy = {"L": +STEP_WIDTH / 2.0, "R": -STEP_WIDTH / 2.0}
    LEG = np.zeros((nb, 12), np.float32)
    ARM = np.zeros((nb, 2), np.float32)
    ATT = np.zeros((nb, 2), np.float32)
    ROOT = np.zeros((nb, 4), np.float32)
    maxD = 0.0
    footlog = []

    def _foot_at(foot, ti):
        """(x, z_sole, swing_weight) for one foot at time ti."""
        cur = 0
        for (mf, fk, tk, t0, t1) in moves:
            if mf != foot:
                continue
            if ti >= t1:
                cur = tk
            elif ti >= t0:                                 # swinging
                s = (ti - t0) / (t1 - t0)
                x = tread_x(fk) + (tread_x(tk) - tread_x(fk)) * _quintic(s)
                zb = tread_z(fk) + (tread_z(tk) - tread_z(fk)) * _quintic(s)
                apex = max(tread_z(fk), tread_z(tk)) + a.clear
                z = zb + (apex - max(tread_z(fk), tread_z(tk))) * (math.sin(math.pi * s) ** 2)
                return x, z, math.sin(math.pi * s)
            else:
                break
        return tread_x(cur), tread_z(cur), 0.0

    for i in range(nb):
        ti = i * DT
        xL, zL, swL = _foot_at("L", ti)
        xR, zR, swR = _foot_at("R", ti)
        # lean ramps in over the settle, out over the top hold
        lean = a.lean
        if ti < a.settle_s:
            lean = a.lean * _quintic(min(1.0, ti / max(1e-3, a.settle_s)))
        elif ti > climb_end:
            lean = a.lean * (1.0 - _quintic(min(1.0, (ti - climb_end) / max(1e-3, a.top_s))))
        pelvis_x = 0.5 * (xL + xR)
        pelvis_z = min(zL, zR) + a.ride                    # ride above the SUPPORT (lower) foot
        # per-leg stair IK (toes-up while swinging to clear the tread nosing)
        qhL, qkL, qaL, DL = _leg_from_foot(pelvis_x, pelvis_z, lean, xL, zL, -a.toe_up * swL)
        qhR, qkR, qaR, DR = _leg_from_foot(pelvis_x, pelvis_z, lean, xR, zR, -a.toe_up * swR)
        maxD = max(maxD, DL, DR)
        LEG[i] = [qhL, 0.0, 0.0, qkL, qaL, 0.0, qhR, 0.0, 0.0, qkR, qaR, 0.0]
        ATT[i] = [0.0, lean]                               # roll 0, pitch = forward climb lean
        # arms counter-swing the same-side leg's forward foot offset
        ARM[i] = [-a.arm_A * (xL - pelvis_x) / max(1e-3, a.going),
                  -a.arm_A * (xR - pelvis_x) / max(1e-3, a.going)]
        ROOT[i] = [pelvis_x, 0.0, pelvis_z, 0.0]
        footlog.append((ti, xL, zL, xR, zR, pelvis_x, pelvis_z, lean))

    return sp, nb, total_t, climb_end, LEG, ARM, ATT, ROOT, maxD, footlog


def synth_wb(spec, LEG, ARM, elbow):
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(RT, spec["model"]))
    JN = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
    JN = [n for n in JN if n]                              # drop unnamed (free) joints
    nb = len(LEG)
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
    return JN, WB


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=os.path.join(G, "ghost_stair_climb_lut.json"))
    ap.add_argument("--riser", type=float, default=0.10)
    ap.add_argument("--going", type=float, default=0.26)
    ap.add_argument("--n-steps", type=int, default=5)
    ap.add_argument("--x-start", type=float, default=0.12,
                    help="world x of the first riser (=foot pad, so pelvis starts at x=0)")
    ap.add_argument("--step-s", type=float, default=0.7, help="time per single-foot step (s)")
    ap.add_argument("--clear", type=float, default=0.07, help="swing-foot clearance above the higher tread (m)")
    ap.add_argument("--ride", type=float, default=0.72, help="pelvis height above the support foot (m)")
    ap.add_argument("--lean", type=float, default=0.12, help="forward climb lean (rad)")
    ap.add_argument("--toe-up", type=float, default=0.15, help="swing-phase toe-up (rad) to clear the nosing")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--top-s", type=float, default=2.0, help="stand-on-landing hold (hold_end pins the last bin)")
    ap.add_argument("--arm-A", type=float, default=0.20)
    ap.add_argument("--elbow", type=float, default=1.2, help="constant elbow (G1: 0=90deg bent, 1.6=straight)")
    ap.add_argument("--shroll", type=float, default=0.15)
    ap.add_argument("--robot-spec", default=os.path.join(HERE, "specs", "retarget_g1.json"))
    a = ap.parse_args()

    spec = json.load(open(a.robot_spec))
    sp, nb, total_t, climb_end, LEG, ARM, ATT, ROOT, maxD, footlog = build(a)
    JN, WB = synth_wb(spec, LEG, ARM, a.elbow)

    cyc_s = nb * DT
    out = {
        "nb": nb, "freq": 1.0 / cyc_s, "cycle_s": cyc_s, "vx": 0.0, "seq": True,
        "hold_end": True, "arm_A": float(a.arm_A), "shroll": float(a.shroll),
        "leg_lut": [[float(v) for v in r] for r in LEG],
        "arm_lut": [[float(v) for v in r] for r in ARM],
        "att_lut": [[float(v) for v in r] for r in ATT],
        "elbow_lut": [[float(a.elbow), float(a.elbow)] for _ in range(nb)],
        "root_lut": [[float(v) for v in r] for r in ROOT],
        "wb_joints": JN, "wb_lut": [[float(v) for v in r] for r in WB],
        "source": ("STAIR-CLIMB sequence ghost (BATON climb specialist): step-to gait, feet land "
                   "ON treads via g1_human_gait 2-link IK, pelvis rides %.2fm above the support "
                   "foot and climbs %d treads (riser %.2f, going %.2f = %.2fm total), %.2f rad "
                   "forward lean, arms counter-swing; settle %.1fs + %d steps + %.1fs landing "
                   "stand (hold_end). FK-feasible by construction (max reach %.1f%% of leg). "
                   "2026-07-07." % (a.ride, a.n_steps, a.riser, a.going, sp.rise, a.lean,
                                    a.settle_s, 2 * a.n_steps, a.top_s, 100.0 * maxD / REACH)),
    }
    json.dump(out, open(a.out, "w"))

    # ---- feasibility report -------------------------------------------------
    print("STAIR-CLIMB ghost: nb=%d  total=%.2fs  climb_end=%.2fs  rise=%.2fm over %.2fm" %
          (nb, total_t, climb_end, sp.rise, a.n_steps * a.going))
    print("  max leg reach = %.4f m = %.1f%% of L1+L2 (%.4f m) %s" %
          (maxD, 100.0 * maxD / REACH, REACH, "OK" if maxD < 0.98 * REACH else "!! OVER-EXTENDED"))
    lo = LEG.min(0); hi = LEG.max(0)
    names = ["L_hipP", "L_hipR", "L_hipY", "L_knee", "L_ankP", "L_ankR",
             "R_hipP", "R_hipR", "R_hipY", "R_knee", "R_ankP", "R_ankR"]
    for j in (0, 3, 4, 6, 9, 10):
        print("  %-7s range [% .3f, % .3f] rad" % (names[j], lo[j], hi[j]))
    print("  base z range [%.3f, %.3f]  (spawn %.3f -> landing %.3f)" %
          (ROOT[:, 2].min(), ROOT[:, 2].max(), ROOT[0, 2], ROOT[-1, 2]))
    print("  wrote %s  (wb_joints=%d)" % (a.out, len(JN)))
    print("  NEXT: build_keypoints.py %s --links --elbow %.2f --shroll %.2f ; "
          "ghost_doctor.py %s  (read T0=limits)" % (a.out, a.elbow, a.shroll, a.out))


if __name__ == "__main__":
    main()
