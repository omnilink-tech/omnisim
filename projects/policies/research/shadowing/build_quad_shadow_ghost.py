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

"""Build + GATE a QUADRUPED SHADOWING walk ghost (lut) -- Component 1+2, ROBOT-GENERAL.

Generalized from build_go2_shadow_ghost.py (2026-07-13) for the "Shadowing is
robot-general" campaign: --robot go2|omniquad|b2. Everything robot-specific lives in
the ROBOTS table at the top -- gait model, deploy-matched MJCF dump, champion
ONNX, joint names, kp, limits, fall height. NOTHING robot-specific is scattered
through the body; the gates, the fold, the corridor law and the replay are
literally the same code for all three. That IS the claim being tested.

    go2   gait go2_trot_gait   dump go2_newton.xml           champion gpu_go2_walk_main
    omniquad  gait omniquad_trot_gait  dump omniquad_newton_fixed2.xml   champion gpu_omniquad_walk_main
    b2    gait b2_trot_gait    dump b2_newton.xml            champion gpu_b2_walk_main

The three robots differ in exactly the ways a *library* must not care about:
Go2/B2 name their joints FL/FR/RL/RR x hip/thigh/calf with a `_joint` suffix and
OmniQuad names them front_left/... x hip_x/hip_y/knee with none; kp is 250 / 500 /
1400; the knee ranges are (-2.72,-0.84) / (-1.20,-0.01) / (-2.82,-0.43). The
CONTROLLER ORDER is identical for all three (leg-major, 4 legs x 3 joints =
abduction, pitch, knee), which is what makes one fold/replay/certify path work.

TWO SOURCES (--source):

  champion (DEFAULT, doctrine class 1a 'recorded'): roll the RL champion
      (<robot>'s policy.onnx riding the analytic trot baseline) on the
      deploy-matched MJCF dump (kp/kv exactly as the standalone trainer,
      DT=16 ms x 8 substeps), with the exact 48-dim trainer obs contract
      (identical for all three quads -- OBS_DIM=48, RES_SCALE=0.15),
      deterministic (mu) actions. A stable policy phase-locks to its clock, so
      the phase-fold is clean -- the same record->fold recipe as the G1
      flagship's official-walk ghost. The ghost is then literally *the
      incumbent's own achieved gait*, certified.

  mppi ('recorded' via the Ghost Generator): receding-horizon MPPI executing the
      analytic trot prior. KEPT for provenance/BASELINE-less robots -- but
      MEASURED 2026-07-12: MPPI's per-cycle exploration jitter misaligns cycles
      on the clock fold and eats ~30% of the stride amplitude; the folded lut
      PD-replays as a SHUFFLE (gmatch 0.998, vx 0.011 -- the stair campaign's
      shuffle tell). Prefer champion whenever a stable walker exists.
      ⚠ mppi needs a NAMED planner MJCF. go2 and b2 ship one; OMNIQUAD DOES NOT
      (feasibility_certificate itself points OmniQuad at the training dump), so
      `--robot omniquad --source mppi` raises instead of guessing.

SHADOW ITERATION (--policy/--baseline/--baseline-lut) is preserved verbatim and
is likewise robot-general: point --policy at a champion that ITSELF rides a ghost
and --baseline ghost --baseline-lut <that ghost>, and the recorder reproduces the
deploy's decode (ghost as command centre + its declared feedforward) so the
recording is in-distribution. That is how you ask whether Shadowing COMPOUNDS.

  analytic ('recorded' from the plant, NO champion needed -- added 2026-07-17 for
      the TURN ghost): execute the analytic gait open-loop through the bare
      deploy-grade PD servo on the deploy-matched dump and fold the ACHIEVED
      joint trajectory. Deterministic (no MPPI jitter -> clean fold), and honest:
      the lut is the plant's own response to the reference, not the reference.
      This is the source for motions that have no champion yet -- the first
      being the TURN-IN-PLACE (--wz), for which no turn champion exists.

TURN GHOSTS (--wz <rad/s>, default 0.0 = walk, bit-identical): the gait models
already carry a yaw-rate parameter (tangential stance sweep, self-tested
reachable) -- a turn-in-place is simply vx=0 + wz!=0. When wz != 0 the builder:
  * passes wz into the gait model for the rollout,
  * measures the ACHIEVED steady yaw rate (wz_meas) from the base quaternion,
  * swaps the lut-replay PROGRESS criterion from forward speed to yaw rate
    (replay |yaw rate| >= 0.6 * |wz_meas of the source|, same sign) -- closure,
    torque and fall checks are unchanged (planted feet stay planted in the world
    during a turn-in-place; the body rotates, the contacts do not),
  * writes `wz` (commanded) and `wz_meas` (achieved) into the lut json, which is
    what the trainer's yaw reward targets and the deploy's wz obs slot carry.

GATES (printed + embedded in the lut json; ALL must pass before training):
  [T0-limits]   joint limits vs the REAL <robot> ranges (the deploy's own
                JOINT_LIMITS, which match the dump's jnt_range).
  [1+4 LUT-REPLAY] the BINDING gate: the folded lut + its declared feedforward
                (ffdq_lut), tracked by the bare deploy-grade position servo (no
                crane -- quads use none) from the settled stand, must WALK:
                no fall, real forward progress, planted-foot world drift
                (CLOSURE on the reference-as-executed) p95 < 20 mm, torques in
                limits. Its gmatch vs the lut = the exam's SELF-MATCH CEILING.
  [2+3 SUPPORT+FWP] feasibility_certificate.certify(motion='walk') on the replay
                trajectory -- the per-step LP (friction-pyramid contact forces +
                torque box must supply the base wrench). This is the DYNAMIC form
                of the COM/FWP gates (a trot's support is a diagonal pair; static
                COM-in-hull is the wrong ruler).
                ⚠ the scalar score pins to 0 whenever one DOF rides its torque
                limit (known artifact, same as the repertoire's G1 row) -- trust
                the VERDICT, quote base_frac95.
  [CORRIDOR LAW] peak |cmd - q_achieved| per joint over the steady segment == the
                corridor floor WITHOUT GHOST-FF; emitted as ffdq_lut so
                QUAD_GHOST_FF=1 can shift the corridor CENTRE instead (rewards
                keep scoring the pose).

OUTPUT <out>_lut.json (self-describing for ghost_validator.py, schema 2):
  robot       : "go2" | "omniquad" | "b2"  -- the registry key; the validator loads
                THIS robot's URDF for its model gates (an unregistered robot used
                to SKIP every model gate and still PASS -- it now fails closed).
  joints      : the REAL deploy motor names in controller order. The trainer, the
                deploy AND the validator all assert against THIS (the WBMATCH
                name-trap rule: never the positional fallback).
  leg_lut(nb,12), ffdq_lut(nb,12), ffdq_kp, nb, freq, vx, source, gait, gates.

Usage:
    python projects/policies/research/shadowing/build_quad_shadow_ghost.py \
        --robot go2|omniquad|b2 [--source champion|mppi] [--reuse <rollout.npz>] \
        [--T 12] [--nb 64] [--fold-from 3.0] [--out _scratch/<robot>_shadow_ghost]
"""
from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.feasibility_certificate import certify  # noqa: E402


# ══════════════════════════════════════════════════════════════════════════════
# THE ROBOT TABLE -- the ONLY place a robot-specific constant is allowed to live.
# ══════════════════════════════════════════════════════════════════════════════
#
# fall_z    : recording/replay abort height (m). go2 0.18 == the go2 trainer's own
#             BZ_FAIL (and the original builder's literal, preserved bit-exact).
#             b2 0.30 == the b2 trainer's BZ_FAIL. omniquad 0.35 is deliberately
#             STRICTER than the omniquad trainer's BZ_FAIL (0.30) -- a recording abort
#             should fire before the trainer's terminal, and omniquad stands tall
#             (body_height 0.55).
# fall_z_reroll : the construction re-roll uses fall_z + 0.02 (the go2 original
#             used 0.20 against a 0.18 record threshold; preserved as the rule).
ROBOTS = {
    "go2": dict(
        display="Go2",
        gait="projects.policies.control.gait.go2_trot_gait",
        dump_mjcf="projects/policies/research/training/mjcf/go2_newton.xml",
        planner_mjcf="projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml",
        champion="projects/policies/research/inference/policies/gpu_go2_walk_main/policy.onnx",
        legs=("FL", "FR", "RL", "RR"),
        parts=("hip", "thigh", "calf"),
        joint_fmt="{leg}_{part}_joint",          # deploy motor names
        kp=250.0,                                 # go2_newton.xml gainprm
        lim_lo=(-1.0472, -0.5236, -2.7227),
        lim_hi=(+1.0472, +3.1316, -0.83776),
        fall_z=0.18,
        foot_r=0.022,                             # planner MJCF foot sphere (mppi only)
    ),
    "omniquad": dict(
        display="OmniQuad",
        gait="projects.policies.control.gait.omniquad_trot_gait",
        dump_mjcf="projects/policies/research/training/mjcf/omniquad_newton_fixed2.xml",
        planner_mjcf=None,                        # OmniQuad ships NO planner MJCF -> no mppi
        champion="projects/policies/research/inference/policies/gpu_omniquad_walk_main/policy.onnx",
        legs=("front_left", "front_right", "rear_left", "rear_right"),
        parts=("hip_x", "hip_y", "knee"),
        joint_fmt="{leg}_{part}",                 # NO _joint suffix (omniquad_walk_deploy)
        kp=500.0,                                 # omniquad_newton_fixed2.xml gainprm
        lim_lo=(-1.50, -0.50, -1.20),
        lim_hi=(+1.50, +3.13, -0.01),
        fall_z=0.35,
        foot_r=0.0,
    ),
    "b2": dict(
        display="B2",
        gait="projects.policies.control.gait.b2_trot_gait",
        dump_mjcf="projects/policies/research/training/mjcf/b2_newton.xml",
        planner_mjcf="projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml",
        champion="projects/policies/research/inference/policies/gpu_b2_walk_main/policy.onnx",
        legs=("FL", "FR", "RL", "RR"),
        parts=("hip", "thigh", "calf"),
        joint_fmt="{leg}_{part}_joint",
        kp=1400.0,                                # b2_newton.xml gainprm
        lim_lo=(-0.87, -0.94, -2.82),             # == b2_walk_deploy JOINT_LIMITS == dump
        lim_hi=(+0.87, +3.1316, -0.43),
        fall_z=0.30,
        foot_r=0.032,
    ),
}

# Shared by every quad (the trainer + deploy tick; identical across go2/omniquad/b2).
DT_CTRL = 0.016      # the trainer/deploy tick
SUBSTEPS = 8         # x 2 ms
PHYS_DT = 0.002

# Canonical joint ROLES, in controller order within a leg. These are ROLES, not
# names: every quad gait model emits (abduction, pitch, knee) per leg, leg-major
# over (FL, FR, RL, RR). The anonymous-MJCF classifier labels by role, and the
# controller index i maps to (CANON_LEGS[i//3], CANON_PARTS[i%3]) -- so nothing
# in the classifier ever parses a robot's joint NAME.
CANON_LEGS = ("FL", "FR", "RL", "RR")
CANON_PARTS = ("hip", "thigh", "calf")   # abduction/roll, pitch, knee


def repo_rel(p) -> str:
    """Repo-relative posix path, or the absolute path if it is outside the repo
    (e.g. --out on another drive -- os.path.relpath RAISES across Windows mounts)."""
    p = os.path.abspath(str(p))
    try:
        return os.path.relpath(p, REPO).replace("\\", "/")
    except ValueError:
        return p.replace("\\", "/")


class Spec:
    """One quadruped, resolved: paths made absolute, gait module imported."""

    def __init__(self, robot: str):
        if robot not in ROBOTS:
            raise SystemExit(f"unknown --robot {robot!r} (have: {', '.join(ROBOTS)})")
        r = ROBOTS[robot]
        self.robot = robot
        self.display = r["display"]          # prose name ("Go2" / "OmniQuad" / "B2")
        self.stg = importlib.import_module(r["gait"])
        self.gait_module = r["gait"]
        self.dump_mjcf = os.path.join(REPO, r["dump_mjcf"])
        self.planner_mjcf = (os.path.join(REPO, r["planner_mjcf"])
                             if r["planner_mjcf"] else None)
        self.champion = os.path.join(REPO, r["champion"])
        self.legs = tuple(r["legs"])
        self.parts = tuple(r["parts"])
        self.foot_r = float(r["foot_r"])
        self.kp = float(r["kp"])
        self.fall_z = float(r["fall_z"])
        self.fall_z_reroll = self.fall_z + 0.02
        self.lim_lo = np.array(list(r["lim_lo"]) * 4, dtype=np.float64)
        self.lim_hi = np.array(list(r["lim_hi"]) * 4, dtype=np.float64)
        # The REAL deploy motor names, controller order (leg-major).
        self.joints = [r["joint_fmt"].format(leg=leg, part=p)
                       for leg in self.legs for p in self.parts]
        # Short labels: what the NAMED planner MJCF calls these joints (go2/b2:
        # "FL_hip"; omniquad: no planner, so the deploy name doubles as the label).
        self.jnames = [f"{leg}_{p}" for leg in self.legs for p in self.parts]
        if not os.path.exists(self.dump_mjcf):
            raise SystemExit(
                f"no deploy-matched MJCF dump for {robot}: {self.dump_mjcf} is missing. "
                "Refusing to guess a model -- the ghost MUST be recorded on the physics "
                "the deploy actually runs (train == deploy).")

    def planner_or_die(self):
        if not self.planner_mjcf or not os.path.exists(self.planner_mjcf):
            raise SystemExit(
                f"--source mppi needs a NAMED planner MJCF and {self.robot} has none "
                f"({self.planner_mjcf or '<not declared>'}). Use --source champion "
                f"(the {self.robot} RL champion exists) -- it is the preferred source "
                "anyway (MPPI folds into a shuffle, measured 2026-07-12).")
        return self.planner_mjcf

    def champion_or_die(self, override: str | None):
        p = override or self.champion
        if not os.path.exists(p):
            raise SystemExit(f"champion ONNX not found for {self.robot}: {p}")
        return p


def _model_maps(sp: Spec, m):
    """(jq, jdof, ctrl_pos_idx, calf_bid, dof_lim) in controller order, for BOTH
    the named planner MJCF and the anonymous deploy dump.

    Classification for the dump == gpu_mjwarp_<robot>_walk_trainer's env (all
    three quads share the class): the leg comes from the joint body's world XY
    quadrant, the role from the axis test first (abduction = the X-hinge), then
    knee = the Y-hinge whose UPPER bound is NEGATIVE, else pitch.

    VERIFIED robot-general 2026-07-13 -- all three dumps are anonymous (joint_N)
    and all three satisfy the test unchanged:
        go2  hip  x-axis          thigh  y (-1.57..3.13)   calf y (-2.72..-0.84)
        omniquad hip_x x-axis         hip_y  y (-0.50..3.13)   knee y (-1.20..-0.01)
        b2   hip  x-axis          thigh  y (-0.94..3.13)   calf y (-2.82..-0.43)
    OmniQuad's knee upper bound (-0.01) is negative, so the SAME classifier resolves
    it; no omniquad-specific branch is needed.
    """
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    named = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, sp.jnames[0]) >= 0
    jq = np.zeros(12, int); jdof = np.zeros(12, int)
    ctrl_idx = np.zeros(12, int); calf_bid = np.zeros(4, int)
    if named:
        for i, nm in enumerate(sp.jnames):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
            jq[i] = m.jnt_qposadr[jid]; jdof[i] = m.jnt_dofadr[jid]
            if i % 3 == 2:
                calf_bid[i // 3] = m.jnt_bodyid[jid]
        # actuators named <joint>_pos
        act = {}
        for ai in range(m.nu):
            an = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, ai) or ""
            if an.endswith("_pos"):
                act[an[:-4]] = ai
        for i, nm in enumerate(sp.jnames):
            ctrl_idx[i] = act[nm]
    else:
        lab = {}
        k_hinge = 0
        for j in range(m.njnt):
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            ax = m.jnt_axis[j]; rng_j = m.jnt_range[j]
            pos = d.xpos[m.jnt_bodyid[j]]
            leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
                   "FR" if (pos[0] > 0) else
                   "RL" if (pos[1] > 0) else "RR")
            role = ("hip" if abs(ax[0]) > 0.5 else
                    "calf" if rng_j[1] < 0.0 else "thigh")
            lab[(leg, role)] = (int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]),
                                k_hinge, int(m.jnt_bodyid[j]))
            k_hinge += 1
        missing = [(lg, rl) for lg in CANON_LEGS for rl in CANON_PARTS if (lg, rl) not in lab]
        if missing or k_hinge != 12:
            raise SystemExit(
                f"[{sp.robot}] anonymous-MJCF classifier resolved {k_hinge}/12 hinges and is "
                f"missing {missing} -- refusing to build a ghost on a mis-mapped model. "
                "(The classifier is axis+range based: abduction = the X-hinge, knee = the "
                "Y-hinge with a NEGATIVE upper bound. Check this robot's dump.)")
        for i in range(12):
            # controller order is leg-major (leg = i//3) x (abduct, pitch, knee)
            qa, va, hidx, bid = lab[(CANON_LEGS[i // 3], CANON_PARTS[i % 3])]
            jq[i] = qa; jdof[i] = va
            ctrl_idx[i] = 2 * hidx          # interleaved [pos, vel] per hinge
            if i % 3 == 2:
                calf_bid[i // 3] = bid
    dof_lim = np.full(m.nv, 1e9)
    for j in range(m.njnt):
        if m.jnt_actfrclimited[j]:
            dof_lim[m.jnt_dofadr[j]] = max(1e-3, np.abs(m.jnt_actfrcrange[j]).max())
    return jq, jdof, ctrl_idx, calf_bid, dof_lim


def _feet_fk(sp: Spec, m, d, calf_bid):
    out = np.zeros((4, 3))
    for f in range(4):
        R3 = d.xmat[calf_bid[f]].reshape(3, 3)
        out[f] = d.xpos[calf_bid[f]] + R3 @ np.array([0.0, 0.0, -sp.stg.L2])
    return out


def _lut_at(table, phase):
    """Circular linear interpolation -- identical to the deploy's _lut_interp."""
    nb = table.shape[0]
    x = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * nb
    b0 = int(math.floor(x)) % nb
    b1 = (b0 + 1) % nb
    f = x - math.floor(x)
    return table[b0] * (1.0 - f) + table[b1] * f


def _yaw_unwrapped(quats):
    """Unwrapped base yaw (rad) from an (n, 4) wxyz quaternion track."""
    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.unwrap(yaw)


def record_analytic(sp: Spec, T: float, out_npz: str, gp, wz: float = 0.0):
    """Deterministic OPEN-LOOP rollout of the analytic gait through the deploy-grade PD.

    No champion, no MPPI: cmd = gait_model(phase, wz). The recording is the plant's
    ACHIEVED response to the reference on the deploy-matched dump -- 'recorded' provenance
    with zero exploration jitter, so the phase fold is clean. This is the source for
    motions that have no champion yet (the turn-in-place is the first).
    """
    stg = sp.stg
    print(f"[{sp.robot}-shadow] rolling the ANALYTIC gait open-loop "
          f"(vx={gp.vx:g}, wz={wz:g} rad/s, freq={gp.freq:g} Hz)")
    m = mujoco.MjModel.from_xml_path(sp.dump_mjcf)
    m.opt.timestep = PHYS_DT
    d = mujoco.MjData(m)
    jq, jdof, ctrl_idx, calf_bid, dof_lim = _model_maps(sp, m)
    nominal = stg.standing_pose(gp).astype(np.float64)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qpos[2] = gp.body_height + 0.03
    d.qpos[3] = 1.0
    d.qpos[jq] = nominal
    full = np.zeros(m.nu)
    full[ctrl_idx] = nominal
    d.ctrl[:] = full
    mujoco.mj_forward(m, d)
    for _ in range(int(1.5 / PHYS_DT)):        # the deploy's rest-start settle
        mujoco.mj_step(m, d)
    n = int(T / DT_CTRL)
    Q = np.zeros((n, m.nq)); QV = np.zeros((n, m.nv)); C = np.zeros((n, 12))
    B = np.zeros((n, 7)); F = np.zeros((n, 4, 3)); TAU = np.zeros((n, 12))
    omega = 2.0 * math.pi * gp.freq
    for k in range(n):
        t = k * DT_CTRL
        phase = stg.QS_PHASE + omega * t
        if wz != 0.0:
            legs, _ = stg.targets_np(phase, gp, t_since_start=t, wz=wz)
        else:
            legs, _ = stg.targets_np(phase, gp, t_since_start=t)
        cmd = np.clip(legs, sp.lim_lo, sp.lim_hi)
        full[ctrl_idx] = cmd
        d.ctrl[:] = full
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
        Q[k] = d.qpos; QV[k] = d.qvel; C[k] = cmd; B[k] = d.qpos[0:7]
        F[k] = _feet_fk(sp, m, d, calf_bid)
        TAU[k] = np.abs(d.qfrc_actuator[jdof])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < sp.fall_z or tilt > 60.0:
            raise RuntimeError(f"analytic rollout FELL at t={t:.2f}s -- the reference is "
                               "not plant-achievable open-loop; redesign before folding")
    os.makedirs(os.path.dirname(out_npz) or ".", exist_ok=True)
    np.savez(out_npz, q=Q, qvel=QV, ctrl=C, base=B, feet=F, tau=TAU, dt=DT_CTRL)
    fwd = B[-1, 0] - B[0, 0]
    dyaw = _yaw_unwrapped(B[:, 3:7])
    print(f"[{sp.robot}-shadow-rec] ANALYTIC recorded {out_npz}: {n} ticks ({T:.0f}s), "
          f"forward {fwd:+.2f} m, yaw {math.degrees(dyaw[-1] - dyaw[0]):+.1f} deg "
          f"({(dyaw[-1] - dyaw[0]) / T:+.3f} rad/s achieved vs {wz:+.3f} commanded), no fall")
    return sp.dump_mjcf


def record_champion(sp: Spec, T: float, out_npz: str, gp,
                    policy_onnx: str | None = None, baseline: str = "trot",
                    baseline_lut: str | None = None, use_ff: bool = True,
                    res_scale_in: float = 0.15):
    """Deterministic rollout of A champion on the deploy dump, exact obs/action contract.

    ⭐ SHADOW ITERATION (2026-07-13). This used to roll exactly one policy (the legacy champion)
    on exactly one baseline (the analytic trot). That is the round-1 operator, and it hides the
    interesting question: Shadowing turns a champion's OWN achieved gait into a reference and
    trains a better champion against it (+12.6% speed, 5x straighter, measured). **Does it
    compound?** To even ask, the recorder has to be able to roll a champion that ITSELF rides a
    ghost -- otherwise you record it on the wrong baseline and capture out-of-distribution junk.

        baseline="trot"  : cmd = trot(phase)                       + a*res_scale   (legacy champ)
        baseline="ghost" : cmd = ghost(phase) [+ ffdq(phase)]      + a*res_scale   (shadow champ)

    The second line is exactly <robot>_shadow_deploy's decode. Same obs family either way
    (48-dim, identical across go2/omniquad/b2); the ghost enters through the clock and the command
    centre, never through new obs.
    """
    import onnxruntime as ort
    stg = sp.stg
    onnx = sp.champion_or_die(policy_onnx)
    sess = ort.InferenceSession(onnx, providers=["CPUExecutionProvider"])
    print(f"[{sp.robot}-shadow] rolling {os.path.basename(os.path.dirname(onnx))} "
          f"on the {baseline.upper()} baseline (ff={'on' if use_ff else 'off'}, "
          f"corridor={res_scale_in})")
    _blut = _bffdq = None
    if baseline == "ghost":
        if not baseline_lut:
            raise SystemExit("--baseline ghost needs --baseline-lut <the ghost this champion rides>")
        _bg = json.loads(open(baseline_lut).read())
        got_robot = _bg.get("robot")
        if got_robot and got_robot != sp.robot:
            raise SystemExit(f"baseline lut is for robot {got_robot!r}, not {sp.robot!r}")
        _blut = np.asarray(_bg["leg_lut"], np.float64)
        if use_ff and "ffdq_lut" in _bg:
            _bffdq = np.asarray(_bg["ffdq_lut"], np.float64)
        got = list(_bg.get("joints") or [])
        if got != sp.joints:
            raise SystemExit(f"baseline lut joint-order mismatch: {got} != {sp.joints}")
    m = mujoco.MjModel.from_xml_path(sp.dump_mjcf)
    m.opt.timestep = PHYS_DT
    d = mujoco.MjData(m)
    jq, jdof, ctrl_idx, calf_bid, dof_lim = _model_maps(sp, m)
    nominal = stg.standing_pose(gp).astype(np.float64)
    mujoco.mj_resetData(m, d)
    d.qpos[:] = 0
    d.qpos[2] = gp.body_height + 0.03
    d.qpos[3] = 1.0
    d.qpos[jq] = nominal
    full = np.zeros(m.nu)
    full[ctrl_idx] = nominal
    d.ctrl[:] = full
    mujoco.mj_forward(m, d)
    # settle 1.5 s at the standing pose (the deploy's rest-start)
    for _ in range(int(1.5 / PHYS_DT)):
        mujoco.mj_step(m, d)
    n = int(T / DT_CTRL)
    Q = np.zeros((n, m.nq)); QV = np.zeros((n, m.nv)); C = np.zeros((n, 12))
    B = np.zeros((n, 7)); F = np.zeros((n, 4, 3)); TAU = np.zeros((n, 12))
    last_action = np.zeros(12, dtype=np.float32)
    last_q = d.qpos[jq].copy()
    omega = 2.0 * math.pi * gp.freq
    res_scale = float(res_scale_in)        # the champion's training corridor
    for k in range(n):
        t = k * DT_CTRL
        phase = stg.QS_PHASE + omega * t
        q12 = d.qpos[jq].copy()
        qd12 = (q12 - last_q) / DT_CTRL    # deploy-faithful finite-diff qd
        last_q = q12.copy()
        w, x, y, z = d.qpos[3:7]
        pg = np.array([-2 * (x * z - w * y), -2 * (y * z + w * x),
                       -(1 - 2 * (x * x + y * y))])
        obs = np.concatenate([
            d.qvel[0:3], d.qvel[3:6], pg, q12 - nominal, qd12, last_action,
            [math.sin(phase), math.cos(phase)], [0.0]]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs), -10.0, 10.0)
        a = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        a = np.clip(a, -1.0, 1.0).astype(np.float32)
        if _blut is None:
            legs, _ = stg.targets_np(phase, gp, t_since_start=t)      # LEGACY: the analytic trot
        else:
            # SHADOW: the exact <robot>_shadow_deploy decode -- the ghost is the command centre,
            # and its declared feedforward shifts that centre (the corridor law: without FF the
            # ghost is untrackable by a 0.15 corridor BY CONSTRUCTION).
            _rr = min(1.0, t / gp.ramp_s) if gp.ramp_s > 0 else 1.0
            legs = nominal + _rr * (_lut_at(_blut, phase) - nominal)
            if _bffdq is not None:
                legs = legs + _rr * _lut_at(_bffdq, phase)
        cmd = np.clip(legs + res_scale * a, sp.lim_lo, sp.lim_hi)
        full[ctrl_idx] = cmd
        d.ctrl[:] = full
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
        last_action = a
        Q[k] = d.qpos; QV[k] = d.qvel; C[k] = cmd; B[k] = d.qpos[0:7]
        F[k] = _feet_fk(sp, m, d, calf_bid)
        TAU[k] = np.abs(d.qfrc_actuator[jdof])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < sp.fall_z or tilt > 60.0:
            raise RuntimeError(f"champion FELL at t={t:.2f}s during recording -- "
                               "cannot record an unstable source (doctrine rule 1)")
    os.makedirs(os.path.dirname(out_npz) or ".", exist_ok=True)
    np.savez(out_npz, q=Q, qvel=QV, ctrl=C, base=B, feet=F, tau=TAU, dt=DT_CTRL)
    fwd = B[-1, 0] - B[0, 0]
    print(f"[{sp.robot}-shadow-rec] CHAMPION recorded {out_npz}: {n} ticks ({T:.0f}s), "
          f"forward {fwd:+.2f} m ({fwd / T:.3f} m/s), no fall")
    return sp.dump_mjcf


def generate_mppi(sp: Spec, T: float, seed: int, out_npz: str, n_samples: int,
                  noise: float, temperature: float):
    from projects.policies.research.shadowing.ghost_generator import GhostGenerator, Intent
    stg = sp.stg
    planner = sp.planner_or_die()
    gp = stg.GaitParams()
    control_dt = 0.02
    gen = GhostGenerator(planner, sim_dt=0.004, control_dt=control_dt)
    mujoco.mj_resetData(gen.m, gen.d)
    qp = gen.d.qpos
    qp[0:3] = [0.0, 0.0, gp.body_height + sp.foot_r]
    qp[3:7] = [1.0, 0.0, 0.0, 0.0]
    stand = stg.standing_pose(gp)
    name2adr = {nm: a for nm, a in zip(
        [n.replace("_pos", "") for n in gen.act_name], gen.act_jqpos)}
    for i, nm in enumerate(sp.jnames):
        qp[name2adr[nm]] = stand[i]
    gen.d.qpos[:] = qp
    full = np.zeros(gen.m.nu)
    full[gen.pos_act] = gen.d.qpos[gen.act_jqpos]
    gen.d.ctrl[:] = full
    for _ in range(int(0.3 / gen.sim_dt)):
        mujoco.mj_step(gen.m, gen.d)
    init = gen.d.qpos.copy()
    n_keys = int(T / control_dt) + 1
    joint_keys, base_keys = [], []
    x = 0.0
    for k in range(n_keys):
        t = k * control_dt
        phase = stg.QS_PHASE + 2.0 * np.pi * gp.freq * t
        legs, _ = stg.targets_np(phase, gp, t_since_start=t)
        joint_keys.append((t, {sp.jnames[i]: float(legs[i]) for i in range(12)}))
        ramp = min(1.0, t / gp.ramp_s)
        x += gp.vx * ramp * control_dt
        base_keys.append((t, {"x": x, "z": gp.body_height + sp.foot_r, "pitch": 0.0}))
    intent = Intent(
        total_time=T, joint_keys=joint_keys, base_keys=base_keys,
        weights=dict(joint=8.0, base_xy=6.0, base_z=8.0, pitch=2.0,
                     upright=1.5, balance=0.3, effort=0.01, smooth=0.05, alive=0.5))
    g = gen.generate(intent, init, n_samples=n_samples, horizon=12, noise=noise,
                     temperature=temperature, seed=seed)
    os.makedirs(os.path.dirname(out_npz) or ".", exist_ok=True)
    np.savez(out_npz, **{k: v for k, v in g.items() if k != "joints"},
             joints=np.array(g["joints"]))
    base = g["base"]
    print(f"[{sp.robot}-shadow-gen] MPPI saved {out_npz} steps={g['q'].shape[0]} "
          f"forward={base[-1, 0] - base[0, 0]:+.2f} m")
    return planner


def main():
    ap = argparse.ArgumentParser(
        description="Build + gate a quadruped Shadowing walk ghost (go2 | omniquad | b2).")
    ap.add_argument("--robot", choices=tuple(ROBOTS), default="go2",
                    help="which quadruped (drives the gait model, the deploy-matched MJCF "
                         "dump, the champion, the joint names, kp and the limits)")
    ap.add_argument("--source", choices=("champion", "mppi", "analytic"), default="champion",
                    help="champion (default) | mppi | analytic (open-loop PD rollout of the "
                         "gait model -- for motions with no champion yet, e.g. the turn)")
    ap.add_argument("--wz", type=float, default=0.0,
                    help="yaw-rate command (rad/s) fed into the gait model's tangential "
                         "stance sweep. Non-zero builds a TURN ghost: the lut-replay "
                         "progress gate switches from forward speed to yaw rate, and the "
                         "lut json gains `wz` (commanded) + `wz_meas` (achieved). "
                         "Default 0.0 keeps every existing walk path bit-identical.")
    ap.add_argument("--vx", type=float, default=None,
                    help="override the gait model's forward speed (m/s). --vx 0 --wz 0.6 "
                         "is a turn-in-place. Default: the robot's GaitParams default.")
    ap.add_argument("--policy", default=None,
                    help="the champion ONNX to roll (default: this robot's champion). "
                         "SHADOW ITERATION: point this at a champion that itself rides a ghost.")
    ap.add_argument("--baseline", choices=("trot", "ghost"), default="trot",
                    help="what the champion's residual composes around: the analytic trot "
                         "(legacy) or a ghost lut (a shadow champion)")
    ap.add_argument("--baseline-lut", default=None,
                    help="the ghost lut the champion rides (required for --baseline ghost)")
    ap.add_argument("--no-ff", action="store_true",
                    help="do NOT add the baseline ghost's declared feedforward")
    ap.add_argument("--corridor", type=float, default=0.15,
                    help="the champion's training corridor (its action scale; RES_SCALE=0.15 "
                         "for all three quad trainers)")
    ap.add_argument("--reuse", default=None, help="skip rollout; use this npz")
    ap.add_argument("--reuse-mjcf", default=None,
                    help="model the reused npz was rolled on (default: by --robot/--source)")
    ap.add_argument("--T", type=float, default=12.0)
    ap.add_argument("--nb", type=int, default=64)
    ap.add_argument("--fold-from", type=float, default=3.0,
                    help="fold only t >= this (past ramp + launch transient)")
    ap.add_argument("--harmonics", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--noise", type=float, default=0.10)
    ap.add_argument("--temp", type=float, default=0.4)
    ap.add_argument("--out", default=None,
                    help="output stem (default: _scratch/<robot>_shadow_ghost)")
    args = ap.parse_args()

    sp = Spec(args.robot)
    stg = sp.stg
    tag = sp.robot
    JN = sp.jnames                      # short labels, for the printouts
    KP = sp.kp
    LIM_LO, LIM_HI = sp.lim_lo, sp.lim_hi
    out_stem = args.out or os.path.join(REPO, "_scratch", f"{sp.robot}_shadow_ghost")

    gp = stg.GaitParams()
    if args.vx is not None:
        gp.vx = float(args.vx)
    wz = float(args.wz)
    if wz != 0.0 and args.source == "champion":
        raise SystemExit(
            "--wz needs --source analytic: no quadruped TURN champion exists to record, "
            "and the walk champion has never seen a non-zero wz command (its wz obs slot "
            "was 0 for its whole training). Roll the analytic gait through the PD instead.")
    npz_path = args.reuse or (out_stem + ".npz")
    if args.reuse:
        mjcf = args.reuse_mjcf or (sp.dump_mjcf if args.source in ("champion", "analytic")
                                   else sp.planner_or_die())
        print(f"[{tag}-shadow] reusing rollout {npz_path} (model {os.path.basename(mjcf)})")
    elif args.source == "champion":
        mjcf = record_champion(sp, args.T, npz_path, gp, policy_onnx=args.policy,
                               baseline=args.baseline, baseline_lut=args.baseline_lut,
                               use_ff=not args.no_ff, res_scale_in=args.corridor)
    elif args.source == "analytic":
        mjcf = record_analytic(sp, args.T, npz_path, gp, wz=wz)
    else:
        mjcf = generate_mppi(sp, args.T, args.seed, npz_path, args.samples,
                             args.noise, args.temp)

    g = np.load(npz_path, allow_pickle=True)
    q, qvel, ctrl, base, feet = g["q"], g["qvel"], g["ctrl"], g["base"], g["feet"]
    dt = float(g["dt"])
    T_total = q.shape[0] * dt
    gates = {}

    m = mujoco.MjModel.from_xml_path(mjcf)
    jq, jdof, ctrl_idx, calf_bid, dof_lim = _model_maps(sp, m)
    qj = q[:, jq]
    assert ctrl.shape[1] == 12

    steady = np.arange(q.shape[0]) * dt >= args.fold_from
    n_cycles = (T_total - args.fold_from) * gp.freq
    vx_steady = float((base[-1, 0] - base[steady][0, 0]) / (T_total - args.fold_from))
    wz_src = 0.0
    if wz != 0.0:
        yaw_src = _yaw_unwrapped(base[:, 3:7])
        i0s = int(steady.argmax())
        wz_src = float((yaw_src[-1] - yaw_src[i0s]) / (T_total - args.fold_from))
    print(f"[{tag}-shadow] rollout T={T_total:.2f}s steady from {args.fold_from}s "
          f"({n_cycles:.1f} cycles), steady vx={vx_steady:.3f} m/s"
          + (f", steady wz={wz_src:+.3f} rad/s (cmd {wz:+.3f})" if wz != 0.0 else "")
          + f", source={args.source}")

    # ── GATE T0-<robot>: the REAL joint-limit check ──
    over_lo = float((LIM_LO - qj.min(0)).max())
    over_hi = float((qj.max(0) - LIM_HI).max())
    ok_lim = over_lo <= 1e-6 and over_hi <= 1e-6
    gates[f"t0_{tag}_limits"] = {"pass": bool(ok_lim), "over_lo": over_lo, "over_hi": over_hi}
    print(f"[GATE T0-{tag}] joint limits: {'PASS' if ok_lim else 'FAIL'} "
          f"(worst under-lo {over_lo:+.4f}, over-hi {over_hi:+.4f} rad)")

    # ── INFO: raw-rollout stance closure ──
    drifts = []
    for f in range(4):
        z = feet[:, f, 2]
        planted = z < (z.min() + 0.015)
        k = 0
        while k < len(planted):
            if planted[k]:
                j = k
                while j + 1 < len(planted) and planted[j + 1]:
                    j += 1
                if j - k >= 3 and k * dt >= args.fold_from:
                    seg = feet[k:j + 1, f, 0:2]
                    drifts.append(float(np.linalg.norm(seg[-1] - seg[0])))
                k = j + 1
            else:
                k += 1
    drift_mean = float(np.mean(drifts)) if drifts else 0.0
    drift_p95 = float(np.percentile(drifts, 95)) if drifts else 0.0
    gates["closure_raw_rollout"] = {"informational": True, "n_stances": len(drifts),
                                    "drift_mean_m": drift_mean, "drift_p95_m": drift_p95}
    print(f"[INFO raw-rollout closure] {len(drifts)} steady stances, drift mean "
          f"{drift_mean * 1000:.1f} mm p95 {drift_p95 * 1000:.1f} mm (the binding "
          f"closure gate runs on the folded lut's PD replay below)")

    # ── CONSTRUCTION RE-ROLL of the raw commands (+ torque capture) ──
    m.opt.timestep = 0.004
    n_sub = max(1, int(round(dt / 0.004)))
    d = mujoco.MjData(m)
    d.qpos[:] = q[0]; d.qvel[:] = qvel[0]
    mujoco.mj_forward(m, d)
    full = np.zeros(m.nu)
    maxfrac, sat, fell = 0.0, 0, False
    tau_hist = np.zeros((ctrl.shape[0], 12))
    x0_reroll = float(d.qpos[0])
    for k in range(ctrl.shape[0]):
        full[ctrl_idx] = ctrl[k]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        tau = np.abs(d.qfrc_actuator[jdof])
        tau_hist[k] = tau
        frac = float((tau / dof_lim[jdof]).max())
        maxfrac = max(maxfrac, frac)
        sat += int(frac > 0.99)
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < sp.fall_z_reroll or tilt > 60.0:
            fell = True
            break
    reroll_fwd = float(d.qpos[0]) - x0_reroll
    ok_reroll = (not fell) and maxfrac <= 1.001
    gates["construction_reroll"] = {"pass": bool(ok_reroll), "fell": bool(fell),
                                    "peak_torque_frac": maxfrac, "saturated_steps": sat,
                                    "of_steps": int(ctrl.shape[0]), "fwd_m": reroll_fwd}
    print(f"[GATE construction re-roll] {'PASS' if ok_reroll else 'FAIL'}: "
          f"{'NO FALL' if not fell else 'FELL'}, open-loop fwd {reroll_fwd:+.2f} m, "
          f"peak torque {maxfrac * 100:.0f}%, saturated {sat}/{ctrl.shape[0]}")

    # ── CORRIDOR-vs-TORQUE LAW ──
    ffdq_raw = ctrl - qj
    ff_peak = np.abs(ffdq_raw[steady]).max(0)
    taukp_peak = tau_hist[steady[:tau_hist.shape[0]]].max(0) / KP
    print("[CORRIDOR LAW] per-joint peak |cmd - q| (rad), steady segment "
          "(= corridor floor without GHOST-FF):")
    for grp in range(4):
        print("   " + "  ".join(
            f"{JN[grp * 3 + j]:16s}={ff_peak[grp * 3 + j]:.3f} (tau/kp {taukp_peak[grp * 3 + j]:.3f})"
            for j in range(3)))
    print(f"[CORRIDOR LAW] overall peak dq = {ff_peak.max():.3f} rad")
    gates["corridor_law"] = {"peak_dq_rad": float(ff_peak.max()),
                             "per_joint_peak_dq": [float(x) for x in ff_peak], "kp": KP}

    # ── FOLD on the absolute gait clock ──
    nb = args.nb
    tt = np.arange(q.shape[0]) * dt
    phase_all = (stg.QS_PHASE + 2.0 * np.pi * gp.freq * tt) % (2.0 * np.pi)
    bins = np.minimum((phase_all / (2.0 * np.pi) * nb).astype(int), nb - 1)
    leg_lut = np.zeros((nb, 12)); ffdq_lut = np.zeros((nb, 12))
    counts = np.zeros(nb, int); spread = np.zeros((nb, 12))
    for b in range(nb):
        sel = steady & (bins == b)
        counts[b] = sel.sum()
        if counts[b]:
            leg_lut[b] = qj[sel].mean(0)
            ffdq_lut[b] = ffdq_raw[sel].mean(0)
            spread[b] = qj[sel].std(0)
    for b in np.where(counts == 0)[0]:
        leg_lut[b] = 0.5 * (leg_lut[(b - 1) % nb] + leg_lut[(b + 1) % nb])
        ffdq_lut[b] = 0.5 * (ffdq_lut[(b - 1) % nb] + ffdq_lut[(b + 1) % nb])
    print(f"[fold] nb={nb}: samples/bin min={counts.min()} mean={counts.mean():.1f}; "
          f"fold spread mean {spread[counts > 0].mean() * 1000:.1f} mrad")

    def _smooth(lut):
        F = np.fft.rfft(lut, axis=0)
        F[args.harmonics + 1:] = 0.0
        return np.fft.irfft(F, n=nb, axis=0)
    leg_lut_s = np.clip(_smooth(leg_lut), LIM_LO, LIM_HI)
    ffdq_lut_s = _smooth(ffdq_lut)
    print(f"[fold] harmonic smoothing (<= {args.harmonics}): max change "
          f"{np.abs(leg_lut_s - np.clip(leg_lut, LIM_LO, LIM_HI)).max() * 1000:.1f} mrad")

    th = 2.0 * np.pi * np.arange(nb) / nb
    if wz != 0.0:
        trot = np.stack([stg.targets_np(t, gp, t_since_start=1e6, wz=wz)[0] for t in th])
    else:
        trot = np.stack([stg.targets_np(t, gp, t_since_start=1e6)[0] for t in th])
    sag = np.abs(leg_lut_s - trot)
    print(f"[preview] achieved-vs-analytic |sag|: mean {sag.mean():.3f} rad, max "
          f"{sag.max():.3f} rad (why the deploy needs the lut baseline)")
    amp_raw = []
    cyc = int(round((1.0 / gp.freq) / dt))
    k = int(args.fold_from / dt)
    while k + cyc <= q.shape[0]:
        w = qj[k:k + cyc]
        amp_raw.append(w.max(0) - w.min(0))
        k += cyc
    amp_raw = np.array(amp_raw).mean(0) if amp_raw else np.zeros(12)
    amp_lut = leg_lut_s.max(0) - leg_lut_s.min(0)
    print(f"[preview] amplitude retention lut/raw (pitch cols): "
          + "  ".join(f"{JN[i]}={amp_lut[i] / max(amp_raw[i], 1e-6):.2f}"
                      for i in (1, 4, 7, 10)))

    # ── THE BINDING GATE: PD REPLAY OF THE FOLDED LUT (gate-4 style, no crane) ──
    T_rep = 8.0
    mr = mujoco.MjModel.from_xml_path(mjcf)
    mr.opt.timestep = PHYS_DT
    dr_ = mujoco.MjData(mr)
    mujoco.mj_resetData(mr, dr_)
    nominal = stg.standing_pose(gp)
    dr_.qpos[:] = 0
    dr_.qpos[2] = gp.body_height + 0.03
    dr_.qpos[3] = 1.0
    dr_.qpos[jq] = nominal
    fullr = np.zeros(mr.nu)
    fullr[ctrl_idx] = nominal
    dr_.ctrl[:] = fullr
    for _ in range(int(1.5 / PHYS_DT)):
        mujoco.mj_step(mr, dr_)

    def _lint(tab, ph):
        x = (ph % (2.0 * np.pi)) / (2.0 * np.pi) * nb
        b0 = int(x) % nb
        f = x - int(x)
        return tab[b0] * (1.0 - f) + tab[(b0 + 1) % nb] * f

    n_rep = int(T_rep / DT_CTRL)
    n_sub_rep = int(round(DT_CTRL / PHYS_DT))
    rq = np.zeros((n_rep, mr.nq))
    rfeet = np.zeros((n_rep, 4, 3))
    rqv = np.zeros((n_rep, mr.nv))
    rctrl = np.zeros((n_rep, 12))
    rgm = np.zeros(n_rep)
    rfell = False
    rmaxfrac = 0.0
    for k in range(n_rep):
        t = k * DT_CTRL
        ph = stg.QS_PHASE + 2.0 * np.pi * gp.freq * t
        rr = min(1.0, t / gp.ramp_s) if gp.ramp_s > 0 else 1.0
        ref_pose = nominal + rr * (_lint(leg_lut_s, ph) - nominal)
        cmd = ref_pose + rr * _lint(ffdq_lut_s, ph)
        fullr[ctrl_idx] = cmd
        dr_.ctrl[:] = fullr
        for _ in range(n_sub_rep):
            mujoco.mj_step(mr, dr_)
        rq[k] = dr_.qpos; rqv[k] = dr_.qvel; rctrl[k] = cmd
        rfeet[k] = _feet_fk(sp, mr, dr_, calf_bid)
        rgm[k] = np.exp(-float(np.mean((dr_.qpos[jq] - ref_pose) ** 2)) / (0.35 ** 2))
        tau = np.abs(dr_.qfrc_actuator[jdof])
        rmaxfrac = max(rmaxfrac, float((tau / dof_lim[jdof]).max()))
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (dr_.qpos[4] ** 2 + dr_.qpos[5] ** 2)))))
        if dr_.qpos[2] < sp.fall_z or tilt > 60.0:
            rfell = True
            break
    k_end = k + 1
    rep_t = np.arange(k_end) * DT_CTRL
    rep_steady = rep_t >= 2.0
    rep_dist = float(rq[k_end - 1, 0] - rq[0, 0])
    if rep_steady.any():
        i0 = int(rep_steady.argmax())
        rep_vx = float((rq[k_end - 1, 0] - rq[i0, 0]) / max(1e-6, (k_end - 1 - i0) * DT_CTRL))
        rep_gm = float(rgm[rep_steady].mean())
    else:
        rep_vx, rep_gm = 0.0, 0.0
    rdrifts = []
    for f4 in range(4):
        z = rfeet[:k_end, f4, 2]
        planted = z < (z.min() + 0.015)
        kk = 0
        while kk < k_end:
            if planted[kk]:
                jj = kk
                while jj + 1 < k_end and planted[jj + 1]:
                    jj += 1
                if jj - kk >= 3 and kk * DT_CTRL >= 2.0:
                    seg = rfeet[kk:jj + 1, f4, 0:2]
                    rdrifts.append(float(np.linalg.norm(seg[-1] - seg[0])))
                kk = jj + 1
            else:
                kk += 1
    rd_p95 = float(np.percentile(rdrifts, 95)) if rdrifts else 0.0
    rd_mean = float(np.mean(rdrifts)) if rdrifts else 0.0
    if wz != 0.0:
        # TURN ghost: PROGRESS is yaw rate, not forward speed (a turn-in-place has
        # vx ~ 0 by design, so the vx criterion is vacuous). Same 0.6x-of-source
        # rule, same sign; every other check (fall, closure, torque) is unchanged.
        rep_yaw = _yaw_unwrapped(rq[:k_end, 3:7])
        if rep_steady.any():
            i0y = int(rep_steady.argmax())
            rep_wz = float((rep_yaw[k_end - 1] - rep_yaw[i0y])
                           / max(1e-6, (k_end - 1 - i0y) * DT_CTRL))
        else:
            rep_wz = 0.0
        ok_prog = (rep_wz * wz_src > 0) and abs(rep_wz) > 0.6 * abs(wz_src)
        prog_txt = f"steady wz {rep_wz:+.3f} vs source {wz_src:+.3f} rad/s"
    else:
        rep_wz = None
        ok_prog = rep_vx > 0.6 * vx_steady
        prog_txt = f"steady vx {rep_vx:.3f} vs source {vx_steady:.3f}"
    ok_replay = (not rfell) and ok_prog and rd_p95 < 0.02 \
        and rmaxfrac <= 1.001
    gates["lut_replay"] = {
        "pass": bool(ok_replay), "fell": bool(rfell), "dist_m": rep_dist,
        "steady_vx": rep_vx, "closure_drift_mean_m": rd_mean,
        "closure_drift_p95_m": rd_p95, "n_stances": len(rdrifts),
        "peak_torque_frac": rmaxfrac, "gmatch_selfmatch_ceiling": rep_gm}
    if rep_wz is not None:
        gates["lut_replay"]["steady_wz"] = rep_wz
        gates["lut_replay"]["source_wz"] = wz_src
    print(f"[GATE 1+4 LUT-REPLAY] {'PASS' if ok_replay else 'FAIL'}: "
          f"{'NO FALL' if not rfell else 'FELL'}, dist {rep_dist:+.2f} m over "
          f"{k_end * DT_CTRL:.0f}s ({prog_txt}), "
          f"CLOSURE drift mean {rd_mean * 1000:.1f} mm p95 {rd_p95 * 1000:.1f} mm / "
          f"{len(rdrifts)} stances (gate < 20 mm), peak torque {rmaxfrac * 100:.0f}%, "
          f"SELF-MATCH gmatch ceiling {rep_gm:.3f}")

    replay_npz = out_stem + "_replay.npz"
    np.savez(replay_npz, q=rq[:k_end], qvel=rqv[:k_end], ctrl=rctrl[:k_end],
             feet=rfeet[:k_end], dt=DT_CTRL)
    rpassed, rscore, rmets = certify(replay_npz, mjcf, motion="walk", verbose=False)
    gates["support_fwp_certificate_replay"] = {
        "pass": bool(rpassed), "score": float(rscore),
        "base_frac95": float(rmets.get("base_frac95", -1))}
    print(f"[GATE 2+3 SUPPORT+FWP on the REPLAY] {'PASS' if rpassed else 'FAIL'} "
          f"score={rscore:.3f} base_frac95={rmets.get('base_frac95')} "
          f"(score pins to 0 when one DOF rides its torque limit -- known artifact; "
          f"the VERDICT + base_frac95 are the signal)")

    out_json = out_stem + "_lut.json"
    if args.source == "champion":
        _src_mid = (f"the {sp.display} RL champion "
                    f"({os.path.basename(os.path.dirname(sp.champion_or_die(args.policy)))}"
                    f"/policy.onnx on the {args.baseline.upper()} baseline) rolled "
                    f"DETERMINISTICALLY on the deploy-matched MJCF dump "
                    f"({os.path.basename(sp.dump_mjcf)}, kp={KP:g}, 16ms x 8 substeps, "
                    f"exact 48-dim trainer obs contract)")
    elif args.source == "analytic":
        _src_mid = (f"the analytic {sp.display} gait model "
                    f"(vx={gp.vx:g}, wz={wz:g} rad/s tangential stance sweep) executed "
                    f"OPEN-LOOP and DETERMINISTICALLY through the bare deploy-grade PD "
                    f"servo on the deploy-matched MJCF dump "
                    f"({os.path.basename(sp.dump_mjcf)}, kp={KP:g}, 16ms x 8 substeps); "
                    f"the folded track is the plant's ACHIEVED response, not the reference")
    else:
        _src_mid = ("receding-horizon MPPI (ghost_generator) executing the analytic "
                    "trot prior over the real force-limited dynamics")
    _verb = "TURNS IN PLACE" if wz != 0.0 else "WALKS"
    doc = {
        "robot": sp.robot,
        "nb": nb,
        "freq": gp.freq,
        "vx": round(vx_steady, 4),
        "joints": sp.joints,
        "joint_order": (f"{','.join(sp.legs)} x ({sp.parts[0]}=abduction/roll, "
                        f"{sp.parts[1]}=pitch, {sp.parts[2]}=knee)"),
        "leg_lut": [[round(float(v), 6) for v in row] for row in leg_lut_s],
        "ffdq_lut": [[round(float(v), 6) for v in row] for row in ffdq_lut_s],
        "ffdq_kp": KP,
        "source": (f"recorded/achieved ({args.source}): " + _src_mid
                   + f"; steady segment (t>={args.fold_from}s, {n_cycles:.1f} cycles) "
                     f"phase-folded at {nb} bins on the gait clock, harmonic-smoothed "
                     f"(<= {args.harmonics}). Replay-verified: the folded lut + its "
                     "declared feedforward, tracked by the bare deploy-grade PD (no "
                     f"crane -- quads use none), {_verb} with closed stance contacts."),
        "gait": dict(vx=gp.vx, freq=gp.freq, duty=gp.duty, step_height=gp.step_height,
                     body_height=gp.body_height, ramp_s=gp.ramp_s),
        "gates": gates,
        "provenance": {"rollout_npz": os.path.basename(npz_path), "dt": dt,
                       "T": T_total, "source": args.source, "robot": sp.robot,
                       "gait_model": sp.gait_module,
                       "builder": "projects/policies/research/shadowing/build_quad_shadow_ghost.py",
                       "mjcf": repo_rel(mjcf)},
    }
    if wz != 0.0:
        # The turn contract: `wz` is the COMMAND fed to the gait model; `wz_meas` is what
        # the plant ACHIEVED (the honest number -- the trainer's yaw reward should target
        # wz_meas, and the deploy's wz obs slot should carry it during the turn leg).
        doc["wz"] = round(wz, 4)
        doc["wz_meas"] = round(wz_src, 4)
        doc["gait"]["wz"] = wz
    with open(out_json, "w") as f:
        json.dump(doc, f)
    print(f"[{tag}-shadow] wrote {out_json}")

    prev = out_stem + "_preview.csv"
    with open(prev, "w") as f:
        f.write("bin,phase_rad," + ",".join(sp.joints) + "\n")
        for b in range(nb):
            f.write(f"{b},{th[b]:.4f}," + ",".join(f"{v:.5f}" for v in leg_lut_s[b]) + "\n")
    amp = leg_lut_s.max(0) - leg_lut_s.min(0)
    print("[preview] per-joint amplitude (rad): " + "  ".join(
        f"{JN[i]}={amp[i]:.3f}" for i in range(12)))
    print(f"[preview] wrote {prev}")

    hard_gates = [f"t0_{tag}_limits", "lut_replay", "support_fwp_certificate_replay",
                  "construction_reroll"]
    all_pass = all(gates[k]["pass"] for k in hard_gates)
    print(f"\n[{tag}-shadow] GATES {'ALL PASS' if all_pass else 'FAILED'} -- "
          f"{ {k: gates[k]['pass'] for k in hard_gates} }")
    print("[{}-shadow] next: python projects/policies/training/ghost_validator.py {}".format(
        tag, out_json))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
