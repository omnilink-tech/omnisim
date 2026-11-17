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

"""g1_sitstand_trajopt.py -- the PLANNER (Stage B of the ghost-tracking pipeline).

Produces a DYNAMICALLY-FEASIBLE G1 sit->stand reference by trajectory optimization
(gradient-free predictive sampling / MPPI over the MuJoCo dynamics), instead of a
hand-drawn kinematic ghost. The optimizer keeps the center of mass over the foot
support through the chair->feet contact handoff with feasible torques, so the
resulting reference has built-in stability margin (no seated-start knife-edge).

Output (the "feasible ghost"): a reference table of the ACHIEVED trajectory --
per control step: 23 joint angles + base (x, z, pitch) -- saved to an .npz that the
RL tracker consumes. See docs/developer/ghost-tracking-pipeline.md.

Method (predictive sampling):
  - parameterize the 13 leg+waist position-target trajectory by KNOTS (low-dim,
    smooth), seeded from the hand-drawn IK reference (a good warm start);
  - each iteration: sample K perturbed knot-sets, roll each out in MuJoCo from the
    seated start, score by a cost (reach+hold an upright stand, CoM-over-feet
    balance, alive, effort, smoothness), softmax-weight, update the nominal;
  - repeat; then roll out the best nominal and record the achieved trajectory.

Run:  python projects/policies/control/planner/g1_sitstand_trajopt.py [--iters 40 --samples 96]

STATUS (2026-06-20): the RISE-only converges CLEAN (cost ~58, stable seated start ->
upright 0.777, lean ~2deg). Two findings make the full cycle hard, both captured here:
  (1) CONTACT MUST MATCH THE TRAINER. Softening the contact de-chaoses the optimizer
      but the reference then FLIES on the trainer's stiff contact (sim-to-sim gap). So
      we plan on the SAME stiff contact the trainer uses (do NOT soften here).
  (2) STIFF-CONTACT PREDICTIVE SAMPLING IS CHAOTIC at the seated knife-edge: the SAME
      nominal can score 217 or blow up to ~29000. Mitigation = keep-BEST-ever (record
      the lowest-cost sample's rollout, never the final nominal) + minimal seated dwell.
      The best stiff references are still mediocre (stand sags to ~0.68, seated leans
      back into the backrest), and RL-tracking them still drifts at the dead-seated start.
NEXT: a gradient-based / MuJoCo-MPC (mjpc) planner for a clean reliable stiff reference,
or start the reference already weight-shifted off the chair. See project_g1_sitstand.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from projects.policies.control.gait.g1_sitstand import full_targets  # warm-start + arm reference

MJCF = str(REPO / "projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml")
LEGS = ("left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint")
ARMS = ("left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
        "right_wrist_roll_joint")
ALL23 = LEGS + ARMS

DT = 0.004            # physics step (PHYS_DT)
# Phase 1: sit -> stand -> HOLD (the hard part: the seated start + rise to upright).
# The sit-DOWN is added in a second pass once the rise tracks (predictive sampling
# struggles with the high-dim full cycle; the rise+hold converges crisply).
T_SIT0, T_RISE, T_STAND, T_SITDOWN, T_SIT1 = 0.2, 2.6, 1.0, 0.0, 0.0
T_TOTAL = T_SIT0 + T_RISE + T_STAND + T_SITDOWN + T_SIT1     # 3.8 s (minimal seated dwell)
N_STEPS = int(round(T_TOTAL / DT))
N_KNOTS = 13          # leg-target knots (interpolated)
END_STANDING = True   # terminal = a stable UPRIGHT stand (rise+hold mode)
SPAWN_Z = 0.55        # seated spawn (a stable seated config; chair at 0.40)
Z_STAND = 0.78
FOOT_HALF = 0.10      # support half-width around the foot center (m), for CoM cost


def _ease(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def _z_target(t):
    """Phased reference pelvis height over the full cycle."""
    if t < T_SIT0:
        return SPAWN_Z
    t -= T_SIT0
    if t < T_RISE:
        return SPAWN_Z + _ease(t / T_RISE) * (Z_STAND - SPAWN_Z)
    t -= T_RISE
    if t < T_STAND:
        return Z_STAND
    t -= T_STAND
    if t < T_SITDOWN:
        return Z_STAND - _ease(t / T_SITDOWN) * (Z_STAND - SPAWN_Z)
    return SPAWN_Z


def _bfrac(t):
    """Stand fraction in [0,1] from the height schedule."""
    return (_z_target(t) - SPAWN_Z) / (Z_STAND - SPAWN_Z)


def _model():
    m = mujoco.MjModel.from_xml_path(MJCF)
    m.opt.timestep = DT
    return m


def _ids(m):
    legq = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in LEGS])
    legv = np.array([m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in LEGS])
    armq = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARMS])
    leg_act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos") for j in LEGS])
    arm_act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos") for j in ARMS])
    bodies = {b: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
              for b in ("pelvis", "left_ankle_roll_link", "right_ankle_roll_link")}
    return legq, legv, armq, leg_act, arm_act, bodies


_SEAT = full_targets(0.0)
_STAND = full_targets(6.0)   # IK standing pose (mid stand-hold)
_SEAT_LEGS = np.array([_SEAT[j] for j in LEGS])
_STAND_LEGS = np.array([_STAND[j] for j in LEGS])
_SEAT_ARMS = np.array([_SEAT[j] for j in ARMS])
_STAND_ARMS = np.array([_STAND[j] for j in ARMS])


def _seed_knots(knot_t):
    """Warm-start: blend the IK seated<->standing leg poses by the stand fraction."""
    seed = np.zeros((N_KNOTS, 13), dtype=np.float64)
    for k, tk in enumerate(knot_t):
        b = _bfrac(tk)
        seed[k] = _SEAT_LEGS + b * (_STAND_LEGS - _SEAT_LEGS)
    return seed


def _interp_targets(knots, knot_t):
    """Linear-interp the (N_KNOTS,13) knots to (N_STEPS,13) per-step leg targets."""
    ts = np.arange(N_STEPS) * DT
    out = np.zeros((N_STEPS, 13))
    for j in range(13):
        out[:, j] = np.interp(ts, knot_t, knots[:, j])
    return out


def _arm_targets():
    """Fixed-schedule arm targets: blend seated<->standing arms by the stand fraction."""
    out = np.zeros((N_STEPS, 10))
    for i in range(N_STEPS):
        b = _bfrac(i * DT)
        out[i] = _SEAT_ARMS + b * (_STAND_ARMS - _SEAT_ARMS)
    return out


def _rollout(m, d, leg_tgt, arm_tgt, ids, record=False):
    """Roll out one leg-target trajectory from the seated spawn; return (cost, traj?)."""
    legq, legv, armq, leg_act, arm_act, bodies = ids
    # seated spawn
    mujoco.mj_resetData(m, d)
    d.qpos[2] = SPAWN_Z
    seat = full_targets(0.0)
    for k, j in enumerate(LEGS):
        d.qpos[legq[k]] = seat[j]
    for k, j in enumerate(ARMS):
        d.qpos[armq[k]] = seat[j]
    mujoco.mj_forward(m, d)
    cost = 0.0
    traj = np.zeros((N_STEPS, 23 + 3)) if record else None
    prev = leg_tgt[0]
    for i in range(N_STEPS):
        d.ctrl[leg_act] = leg_tgt[i]
        d.ctrl[arm_act] = arm_tgt[i]
        mujoco.mj_step(m, d)
        pz = d.qpos[2]; px = d.qpos[0]
        w, x, y, z = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
        tilt = abs(np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))   # pitch ~ fwd lean
        roll = abs(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        com = d.subtree_com[0][0]
        foot = 0.5 * (d.xpos[bodies["left_ankle_roll_link"]][0] + d.xpos[bodies["right_ankle_roll_link"]][0])
        t = i * DT
        z_tgt = _z_target(t); b = _bfrac(t)
        off_chair = max(0.0, min(1.0, (pz - 0.57) / 0.10))               # 0 seated -> 1 risen
        tph = t - T_SIT0                                                 # time since the seated hold
        in_stand = (T_RISE <= tph < T_RISE + T_STAND)                    # the stand-hold window
        # costs
        cost += 12.0 * (pz - z_tgt) ** 2                                 # track the height schedule
        cost += 8.0 * off_chair * max(0.0, abs(com - foot) - FOOT_HALF) ** 2  # CoM over feet (off chair)
        cost += 1.5 * (roll ** 2)                                        # no sideways tip
        cost += 0.6 * b * (tilt ** 2)                                    # upright torso when standing
        if in_stand:                                                     # strong pull to a FULL stand (by TIME)
            cost += 40.0 * ((pz - Z_STAND) ** 2 + tilt ** 2)
        if b < 0.15:                                                     # seated: stay on the chair (no drift)
            cost += 6.0 * (px ** 2)
        cost += 0.002 * float(np.sum((leg_tgt[i] - prev) ** 2)) / DT     # smoothness
        if pz < 0.33 or roll > 1.0 or tilt > 1.3:                        # fallen
            cost += 50.0
        prev = leg_tgt[i]
        if record:
            jvals = [d.qpos[legq[k]] for k in range(13)] + [d.qpos[armq[k]] for k in range(10)]
            traj[i, :23] = jvals
            traj[i, 23] = px; traj[i, 24] = pz; traj[i, 25] = -np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    # terminal
    pz = d.qpos[2]; w, x, y, z = d.qpos[3:7]
    tilt = abs(np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    vel = float(np.linalg.norm(d.qvel[0:6]))
    if END_STANDING:    # rise+hold: a stable UPRIGHT stand
        cost += 200.0 * (pz - Z_STAND) ** 2 + 120.0 * tilt ** 2 + 20.0 * vel ** 2
    else:               # full cycle: ended seated + settled
        cost += 100.0 * (pz - SPAWN_Z) ** 2 + 30.0 * tilt ** 2 + 20.0 * vel ** 2
    return cost, traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--samples", type=int, default=96)
    ap.add_argument("--sigma", type=float, default=0.15)   # initial knot noise (rad)
    ap.add_argument("--temp", type=float, default=8.0)     # softmax temperature
    ap.add_argument("--out", default=str(REPO / "_scratch/g1_sitstand_feasible.npz"))
    args = ap.parse_args()

    m = _model(); d = mujoco.MjData(m); ids = _ids(m)
    # raise the chair so the seated robot is supported (matches the trainer MJCF)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "chair_seat")
    m.geom_pos[gid][2] = 0.40
    # NOTE: plan on the SAME (stiff) contact as the trainer/deploy. Softening
    # de-chaoses planning but makes the reference infeasible on stiff contact (the
    # seated start then flies in the trainer). The chaos comes from DWELLING in the
    # knife-edge seated state, so we keep the seated dwell minimal instead.
    knot_t = np.linspace(0.0, T_RISE, N_KNOTS)
    nominal = _seed_knots(knot_t)          # (N_KNOTS, 13)
    arm_tgt = _arm_targets()
    lo = np.array([-2.5, -0.5, -2.7, -0.08, -0.87, -0.26, -2.5, -2.9, -2.7, -0.08, -0.87, -0.26, -2.6])
    hi = np.array([2.8, 2.9, 2.7, 2.8, 0.52, 0.26, 2.8, 0.52, 2.7, 2.8, 0.52, 0.26, 2.6])

    c0, _ = _rollout(m, d, _interp_targets(nominal, knot_t), arm_tgt, ids)
    print(f"[trajopt] seed cost = {c0:.1f}  (knots={N_KNOTS} steps={N_STEPS} samples={args.samples})")
    rng = np.random.default_rng(0)
    best_cost, best_knots = c0, nominal.copy()                       # keep the BEST ever seen
    for it in range(args.iters):
        sigma = args.sigma * (1.0 - 0.7 * it / max(1, args.iters))   # anneal noise
        samples = nominal[None] + rng.normal(0, sigma, size=(args.samples, N_KNOTS, 13))
        samples[0] = nominal; samples[1] = best_knots                # keep the current + best-ever
        samples = np.clip(samples, lo, hi)
        costs = np.array([_rollout(m, d, _interp_targets(s, knot_t), arm_tgt, ids)[0] for s in samples])
        jmin = int(costs.argmin())
        if costs[jmin] < best_cost:                                  # track the best sample (chaos-robust)
            best_cost = float(costs[jmin]); best_knots = samples[jmin].copy()
        wts = np.exp(-(costs - costs.min()) / args.temp); wts /= wts.sum()
        nominal = np.clip(np.einsum("k,kij->ij", wts, samples), lo, hi)
        if it % 5 == 0 or it == args.iters - 1:
            print(f"[trajopt] it {it:3d}  sigma={sigma:.3f}  min sample={costs.min():.1f}  best-ever={best_cost:.1f}")

    # record the BEST-ever trajectory (NOT the final nominal -- chaos can corrupt it)
    leg_tgt = _interp_targets(best_knots, knot_t)
    fc, traj = _rollout(m, d, leg_tgt, arm_tgt, ids, record=True)
    nominal = best_knots
    print(f"[trajopt] BEST cost={best_cost:.1f}  (recorded rollout cost={fc:.1f})")
    # diagnostics across the achieved full cycle
    for ts, lbl in [(0.4, "SIT"), (1.5, "rise"), (2.5, "rise"), (3.2, "STAND"),
                    (3.9, "STAND"), (4.4, "STAND")]:
        i = min(N_STEPS - 1, int(ts / DT))
        print(f"   {lbl:5s} t={ts:4.2f}s  pelvis_x={traj[i,23]:+.3f} z={traj[i,24]:.3f}  lean={np.degrees(traj[i,25]):4.0f}deg")
    np.savez(args.out, traj=traj, dt=DT, joints=np.array(ALL23), knots=nominal, knot_t=knot_t)
    print(f"[trajopt] saved feasible reference -> {args.out}")


if __name__ == "__main__":
    main()
