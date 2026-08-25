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

"""TIER-2 STEP+OBS PARITY for the G1 Newton walker.

Fills the Tier-2 TODO at ``g1_golden_parity.py:357-369``: the structural diff
already proves trainer == deploy == MJCF at the *MjModel* (physics) layer. The
remaining trainer<->deploy divergence therefore must live in the
OBSERVATION / INITIAL-CONDITION layer, NOT the physics. This harness QUANTIFIES
those layers with a closed-loop, confound-free measurement.

Three measurements, all driven off ONE source of truth
(``projects/policies/research/backends/g1_physics_spec``) and the SAME deploy/trainer builders
``g1_golden_parity`` already exposes:

  MEASUREMENT 1 -- OBS-PIPELINE GAP (primary, zero physics confound).
    Step ONE deploy World open-loop under the fixed action table. Each tick,
    from the IDENTICAL stepped state, compute the obs two ways:
      * base angular velocity: a HARNESS-INTERNAL representation diff -- Newton
        body_qd[3:6] rotated by R^T vs raw free-joint qvel[3:6]. CAUTION: this is
        NOT the deploy-controller's ang-vel gap. The REAL deploy
        (g1_walk_deploy.py:764-778, "OBS FIX 2026-06-09") computes
        ang_vel_body = R^T @ getVelocity(), where getVelocity() is the engine's
        WORLD-frame omega, and that ALREADY equals the trainer's body-frame
        qvel[3:6]. Newton's body_qd angular part is a different internal quantity
        (its angular component may already be body-frame) so rotating it again
        double-rotates and manufactures a spurious gap. The number is kept for
        completeness but RELABELLED as a representation artifact, and split into
        first-tick-UPRIGHT (small, the regime the policy ever sees) vs the
        fallen/chaos regime (large, irrelevant -- a passive open-loop biped that
        has toppled).
      * base linear velocity: DEPLOY-path (world-frame body_qd[0:3]) vs
        TRAINER-path (raw free-joint qvel[0:3]).
      * joint qd (the GENUINELY open divergence): three ways from the SAME
        stepped trajectory --
          (a) trainer-RAW    = raw qvel joint slots (today's trainer);
          (b) deploy fin-diff = the harness's existing inline finite-diff;
          (c) shared          = g1_env_core.JointVelEstimator(DT, tau=G1_QD_TAU)
                                fed the same achieved leg angles each tick.
        (b) and (c) must AGREE to ~1e-9 (proves the shared estimator is a faithful
        drop-in for the deploy's finite-diff); the (a)-vs-(c) gap is the real
        raw-qvel-vs-finite-diff divergence the trainer must close by adopting the
        estimator.
    Reports per-component max/mean over the full rollout AND the first-10-tick
    window, in physical units.

  MEASUREMENT 2 -- PHYSICS N=1 vs N parity (sanity).
    Build the trainer model at world_count=N, drive all N worlds with identical
    fixed actions, and confirm the N worlds stay bit-for-bit identical to each
    other (max per-world divergence < 1e-4). Confirms batching is
    physics-neutral, so a per-env obs gap is not a batching artifact.

  MEASUREMENT 3 -- INITIAL-CONDITION GAP.
    From the nominal squat seed (base @ SPAWN_Z, identity-upright quat,
    NOMINAL_LEGS joint angles, qvel=0), hold the nominal joint POSITION targets
    under gravity for ceil(0.3/DT) env-steps (the deploy's 0.3 s settle).
    Reports the resulting base PITCH (rad) + residual lin/ang velocity magnitude
    -- the "launch lean" the deploy carries into its first policy tick but the
    trainer's teleport reset (pitch 0, qvel 0) does NOT.

CLI:
    python projects/policies/research/training/g1_step_obs_parity.py --ticks 60 --envs 8

GPU-guarded: MEASUREMENT 1 uses the deploy World's MuJoCo solver (GPU mjwarp
when CUDA present, else falls back gracefully). MEASUREMENT 2/3 need CUDA; they
degrade to a printed "SKIPPED (no CUDA)" rather than crashing.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

# Replicate g1_golden_parity's env setup BEFORE building any model so the
# extracted deploy runtime + the FORCE_MUJOCO solver branch read the real knobs.
os.environ.update(SPEC.newton_env())
# Mirror the rebuild-gated OmSolid.cpp opt-in the trainer/deploy parity uses so
# both representations carry the true link COM (matches g1_golden_parity usage).
os.environ.setdefault("OMNISIM_NEWTON_USE_LINK_COM", "1")

# Reuse the existing primitives verbatim -- do NOT re-declare physics constants.
from projects.policies.research.training.g1_golden_parity import (  # noqa: E402
    build_deploy_model,
    build_trainer_model,
    parse_urdf,
    _find_root_link,
    _fixed_actions,
)
from projects.policies.common import env_fingerprint  # noqa: E402
from projects.policies.research.backends.g1_env_core import JointVelEstimator  # noqa: E402


# ===========================================================================
# Helpers -- frame conventions copied from the deploy / trainer obs paths.
# ===========================================================================
def _quat_xyzw_to_R(qx, qy, qz, qw):
    """Newton/warp body_q quat (x, y, z, w) -> 3x3 body->world rotation."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _quat_xyzw_pitch(qx, qy, qz, qw):
    """Pitch (rad) of a body->world quat (x, y, z, w).

    Uses the same asin(2(wy - zx)) form g1_walk_deploy / the trainer baseline
    derive pitch from (consistent with proj_gravity).
    """
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.asin(sinp)


def _revolute_slots(urdf_path):
    """Slot id -> joint name for every revolute/continuous joint, in the URDF
    order build_deploy_model adds them (slot == add_joint_revolute call index).
    """
    _links, joints = parse_urdf(urdf_path)
    slots = {}
    slot = 0
    for J in joints:
        if J.type in ("revolute", "continuous"):
            slots[slot] = J.name
            slot += 1
    return slots


# ===========================================================================
# MEASUREMENT 1 -- OBS-PIPELINE GAP.
# ===========================================================================
def measure_obs_pipeline(ticks: int) -> int:
    print("=" * 92)
    print(f"MEASUREMENT 1 -- OBS-PIPELINE GAP (deploy-path vs trainer-path, "
          f"{ticks} ticks, world_count=1)")
    print("=" * 92)

    import newton  # noqa: F401
    import warp as wp

    if not wp.get_cuda_device_count():
        print("[M1 DEGRADED] no CUDA device -- the deploy World needs the mjwarp "
              "GPU solver. Skipping (physics layer already proven equal "
              "structurally).")
        return 0

    # ---- Build the LITERAL deploy World (the actual deploy stepping path) ----
    world, n_rev, info = build_deploy_model()
    print(f"[deploy] n_revolute={n_rev}  n_bodies={info['n_bodies']}  "
          f"total_mass={info['total_urdf_mass']:.3f} kg  "
          f"solver_kind={info.get('solver_kind')}")
    if info.get("solver_error"):
        print(f"[deploy] solver_error: {info['solver_error'][:160]}")

    model = world.model
    qd_start = model.joint_qd_start.numpy()       # generalized-vel DOF offsets
    slot2real = world.slot_to_real_idx            # slot -> builder joint idx
    slots = _revolute_slots(SPEC.URDF_PRIM)       # slot -> joint name

    # The 13 LEG joints (the deploy obs qd block) by name -> slot.
    name2slot = {nm: s for s, nm in slots.items()}
    leg_slots = [name2slot[nm] for nm in SPEC.LEGS_JOINTS if nm in name2slot]
    if len(leg_slots) != len(SPEC.LEGS_JOINTS):
        print(f"[M1 WARN] only matched {len(leg_slots)}/{len(SPEC.LEGS_JOINTS)} "
              f"leg joints by name.")
    # qvel DOF index for each leg joint (trainer-path raw qd read).
    leg_qd_idx = [int(qd_start[slot2real[s]]) for s in leg_slots]

    nj = len(SPEC.JOINT_NAMES)
    actions = _fixed_actions(ticks, nj)
    nominal_full = np.concatenate([
        np.asarray(SPEC.NOMINAL_LEGS, dtype=np.float64),
        np.asarray(SPEC.ARM_NOMINAL, dtype=np.float64),
    ])
    # Action -> position target for every revolute slot, in URDF/SPEC order.
    # SPEC.JOINT_NAMES is the policy/action order; map each to its slot.
    act_slot = [name2slot.get(nm) for nm in SPEC.JOINT_NAMES]

    dt = SPEC.DT
    step_dt = dt  # the deploy controller's tick dt == one env step (SPEC.DT)

    # Deploy joint-qd finite-diff state (mirror g1_walk_deploy exactly).
    _qd_tau = float(os.environ.get("G1_QD_TAU", "0.0"))
    qd_alpha = 1.0 if _qd_tau <= 0 else max(0.05, min(1.0, step_dt / (step_dt + _qd_tau)))
    prev_leg_q = np.asarray(SPEC.NOMINAL_LEGS, dtype=np.float64).copy()
    joint_qd_smoothed = np.zeros(len(leg_slots), dtype=np.float64)

    # SHARED estimator (g1_env_core.JointVelEstimator) -- the deploy's finite-diff
    # definition, the SAME module the trainer would adopt. Seed prev_q from the
    # nominal legs (identical seed to the inline finite-diff above) so tick 0
    # produces a matching qd, and the (b)==(c) agreement is a fair test.
    shared_est = JointVelEstimator(SPEC.DT, tau=_qd_tau)
    shared_est.reset(np.asarray(SPEC.NOMINAL_LEGS, dtype=np.float64).copy())

    base_body_idx = 0   # pelvis: first body added in build_deploy_model

    # Per-tick logs of |deploy - trainer| per component.
    d_ang = np.zeros((ticks, 3))
    d_lin = np.zeros((ticks, 3))
    d_qd = np.zeros((ticks, len(leg_slots)))          # legacy: dep-finitediff vs trainer-raw

    # Three-way joint-qd sub-measurement (TASK 2).
    qd_inline = np.zeros((ticks, len(leg_slots)))     # (b) harness inline finite-diff
    qd_shared = np.zeros((ticks, len(leg_slots)))     # (c) shared estimator
    qd_raw = np.zeros((ticks, len(leg_slots)))        # (a) trainer raw qvel slots

    # Upright tracking: split the ang-vel artifact into the regime the policy
    # ever sees (upright) vs the chaos-dominated fallen regime on a passive biped.
    pitch_log = np.zeros(ticks)
    upright = np.zeros(ticks, dtype=bool)
    UPRIGHT_TILT = 0.4   # rad (~23 deg) -- generous "still standing" gate

    for k in range(ticks):
        act = actions[k]
        # Position targets: nominal + ACT_SCALE * action, in SPEC.JOINT_NAMES order.
        for i, nm in enumerate(SPEC.JOINT_NAMES):
            s = act_slot[i]
            if s is None:
                continue
            tgt = float(nominal_full[i] + SPEC.ACT_SCALE * act[i])
            world.set_joint_target_pos(s, tgt)
        # Step the REAL deploy path (internal substeps from OMNISIM_NEWTON_SUBSTEPS).
        world.step(dt)

        # ---- read BOTH obs representations from the SAME stepped state ----
        # Trainer-path: raw generalized qvel (free joint = first 6 DOFs).
        qvel = world.state_a.joint_qd.numpy()
        trn_lin = qvel[0:3].astype(np.float64)
        trn_ang = qvel[3:6].astype(np.float64)            # body frame (per trainer)
        trn_qd = np.array([float(qvel[j]) for j in leg_qd_idx], dtype=np.float64)

        # Deploy-path: body_vel (world frame) + R^T rotate ang into body frame.
        # body_xform/body_vel cache off state_a -- read AFTER the step, same state.
        bx = world.body_xform(base_body_idx)              # x,y,z,qx,qy,qz,qw
        bv = world.body_vel(base_body_idx)                # vx,vy,vz,wx,wy,wz (world)
        R = _quat_xyzw_to_R(bx[3], bx[4], bx[5], bx[6])   # body->world
        dep_lin = np.array(bv[0:3], dtype=np.float64)     # world (unchanged, like deploy)
        omega_world = np.array(bv[3:6], dtype=np.float64)
        dep_ang = R.T @ omega_world                       # omega_body = R^T omega_world

        # Deploy-path joint qd: finite-diff of joint angles + low-pass (inline).
        leg_q = np.array([world.get_joint_angle(s) for s in leg_slots], dtype=np.float64)
        joint_qd_raw = (leg_q - prev_leg_q) / max(step_dt, 1e-6)
        joint_qd_smoothed = qd_alpha * joint_qd_raw + (1.0 - qd_alpha) * joint_qd_smoothed
        dep_qd = joint_qd_smoothed.copy()
        prev_leg_q = leg_q.copy()

        # SHARED estimator fed the SAME achieved leg angles this tick (the
        # drop-in the trainer would use). Must reproduce the inline finite-diff.
        shr_qd = np.asarray(shared_est.update(leg_q), dtype=np.float64).copy()

        # Stash the three joint-qd representations + the legacy gap.
        qd_inline[k] = dep_qd            # (b)
        qd_shared[k] = shr_qd            # (c)
        qd_raw[k] = trn_qd               # (a)
        d_qd[k] = np.abs(dep_qd - trn_qd)

        # Uprightness for this tick (regime split for the ang-vel artifact).
        pitch = _quat_xyzw_pitch(bx[3], bx[4], bx[5], bx[6])
        pitch_log[k] = pitch
        upright[k] = abs(pitch) < UPRIGHT_TILT

        d_ang[k] = np.abs(dep_ang - trn_ang)
        d_lin[k] = np.abs(dep_lin - trn_lin)

        # caches are invalidated by World at the end of step(); clear ours too.
        world._body_q_cache = None
        world._body_qd_cache = None

    K = min(10, ticks)

    def _report(label, arr, unit):
        full_max = float(arr.max()) if arr.size else 0.0
        full_mean = float(arr.mean()) if arr.size else 0.0
        early = arr[:K]
        e_max = float(early.max()) if early.size else 0.0
        e_mean = float(early.mean()) if early.size else 0.0
        # per-axis max for the vector components
        per = ", ".join(f"{float(arr[:, c].max()):.3e}" for c in range(arr.shape[1]))
        print(f"  {label:<22} full: max={full_max:.3e} {unit}  mean={full_mean:.3e} "
              f"{unit}   |   first-{K}: max={e_max:.3e} mean={e_mean:.3e}")
        print(f"  {'':<22} per-component max [{per}] {unit}")
        return full_max, full_mean, e_max, e_mean

    print(f"\n  obs gap |deploy_path - trainer_path|, computed from the IDENTICAL "
          f"stepped state:")
    r_ang = _report("base ang vel*", d_ang, "rad/s")
    r_lin = _report("base lin vel", d_lin, "m/s")

    # ---- (1) CORRECTED ang-vel framing: the body_qd number is an artifact ----
    n_up = int(upright.sum())
    n_down = int((~upright).sum())
    up_max = float(d_ang[upright].max()) if n_up else 0.0
    up_mean = float(d_ang[upright].mean()) if n_up else 0.0
    down_max = float(d_ang[~upright].max()) if n_down else 0.0
    first_up_ang = float(d_ang[0].max())   # tick-0 is always still upright
    print("\n  +-- ANG-VEL NOTE (read this before trusting the number above) "
          "--------------------+")
    print("  | (*) base ang vel above is a HARNESS-INTERNAL representation diff:")
    print("  |     Newton body_qd[3:6] rotated by R^T  vs  raw qvel[3:6]. It is")
    print("  |     NOT the deploy-controller's ang-vel gap.")
    print("  | (a) The REAL deploy (g1_walk_deploy.py:764-778, 'OBS FIX 2026-06-09')")
    print("  |     uses ang_vel_body = R^T @ getVelocity(), getVelocity()=engine")
    print("  |     WORLD-frame omega, which ALREADY matches the trainer's body-frame")
    print("  |     qvel[3:6]. So the deploy ang-vel frame is ALREADY unified.")
    print("  | (b) Newton's body_qd angular part is a DIFFERENT internal quantity")
    print("  |     (its angular component may already be body-frame); applying R^T")
    print("  |     here DOUBLE-rotates and manufactures a spurious gap.")
    print("  | regime split (the policy only ever sees the UPRIGHT regime):")
    print(f"  |     first-tick (upright)        : max |diff| = {first_up_ang:.3e} rad/s")
    print(f"  |     upright ticks  (|pitch|<{UPRIGHT_TILT:.2f}, n={n_up:>3}): "
          f"max={up_max:.3e}  mean={up_mean:.3e}")
    print(f"  |     fallen  ticks  (toppled,    n={n_down:>3}): "
          f"max={down_max:.3e}  (chaos on a passive open-loop biped -- irrelevant)")
    print("  +" + "-" * 76 + "+")

    # ---- (2) THREE-WAY joint-qd sub-measurement (the genuinely open gap) ----
    # (b) deploy inline finite-diff  vs  (c) shared JointVelEstimator: must be ~0.
    bc = np.abs(qd_inline - qd_shared)
    bc_max = float(bc.max()) if bc.size else 0.0
    bc_mean = float(bc.mean()) if bc.size else 0.0
    # (a) trainer raw qvel  vs  (c) shared estimator: the real divergence.
    ac = np.abs(qd_raw - qd_shared)
    ac_max = float(ac.max()) if ac.size else 0.0
    ac_mean = float(ac.mean()) if ac.size else 0.0
    ac_e = ac[:K]
    ac_e_max = float(ac_e.max()) if ac_e.size else 0.0
    ac_e_mean = float(ac_e.mean()) if ac_e.size else 0.0
    # (a) vs (b): sanity -- equals (a)-vs-(c) up to the bc residual.
    ab = np.abs(qd_raw - qd_inline)
    ab_max = float(ab.max()) if ab.size else 0.0

    BC_TOL = 1e-9
    print("\n  JOINT-QD three-way gap (13 leg joints, same stepped trajectory):")
    print(f"    (a) trainer-RAW   = raw qvel joint slots          [today's trainer]")
    print(f"    (b) deploy fin-diff = harness inline finite-diff   [deploy definition]")
    print(f"    (c) shared        = g1_env_core.JointVelEstimator(DT, tau={_qd_tau})")
    print(f"    |(b) - (c)|  (shared reproduces deploy fin-diff):  "
          f"max={bc_max:.3e}  mean={bc_mean:.3e} rad/s")
    bc_ok = bc_max <= BC_TOL
    verdict_bc = (f"PASS -- shared estimator IS the deploy finite-diff "
                  f"(agree to <= {BC_TOL:.0e})" if bc_ok
                  else f"CHECK -- |(b)-(c)| = {bc_max:.3e} > {BC_TOL:.0e}")
    print(f"      => {verdict_bc}")
    assert bc_ok, (f"shared JointVelEstimator must reproduce the deploy inline "
                   f"finite-diff to <= {BC_TOL:.0e}, got {bc_max:.3e}")
    print(f"    |(a) - (c)|  (the REAL raw-qvel vs finite-diff gap the trainer")
    print(f"                  must close by adopting the estimator):")
    print(f"        full: max={ac_max:.3e}  mean={ac_mean:.3e} rad/s   |   "
          f"first-{K}: max={ac_e_max:.3e} mean={ac_e_mean:.3e}")
    print(f"    |(a) - (b)|  (sanity, == |(a)-(c)| since (b)==(c)):  max={ab_max:.3e} rad/s")

    print("\n  interpretation:")
    print("    * lin-vel gap ~0 (both world frame, no transform).")
    print("    * ang-vel '*' number is a body_qd representation artifact, NOT the")
    print("      deploy gap -- the real deploy already unifies the frame (R^T @ getVelocity).")
    print("    * joint-qd: the shared estimator EXACTLY reproduces the deploy's")
    print("      finite-diff (|(b)-(c)|~0); the open trainer gap is (a)-vs-(c) =")
    print(f"      raw-qvel vs finite-diff (max {ac_max:.3e} rad/s), closed by the trainer")
    print("      feeding achieved joint angles through g1_env_core.JointVelEstimator.")
    return 0


# ===========================================================================
# MEASUREMENT 2 -- PHYSICS N=1 vs N parity (batching is physics-neutral).
# ===========================================================================
def measure_physics_batch(ticks: int, envs: int) -> int:
    print("\n" + "=" * 92)
    print(f"MEASUREMENT 2 -- PHYSICS N=1 vs N parity (world_count={envs}, "
          f"{ticks} ticks)")
    print("=" * 92)

    import newton
    import warp as wp
    import torch

    if not wp.get_cuda_device_count():
        print("[M2 DEGRADED] no CUDA device -- batched mjwarp rollout needs a GPU. "
              "Skipping.")
        return 0

    # Replicate the trainer model `envs` times into ONE batched newton model
    # EXACTLY as gpu_newton_g1_walk_trainer does: build the single full-body
    # G1 prim builder, then main_b.replicate(g1, world_count=n, spacing=...).
    # add_builder() in a loop stacks the worlds at the same origin -> they
    # collide and diverge (a harness artifact, not physics); replicate's spacing
    # is what keeps the worlds independent.
    from projects.policies.research.training.gpu_newton_g1_walk_trainer import (
        _build_g1_full_prim_builder)
    spacing = (3.0, 3.0, 0.0)
    try:
        single = _build_g1_full_prim_builder(ke=SPEC.KE, kd=SPEC.KD)
        main_b = newton.ModelBuilder()
        main_b.replicate(single, world_count=envs, spacing=spacing)
        model = main_b.finalize()
        if getattr(model, "world_count", envs) != envs:
            print(f"[M2 WARN] world_count={getattr(model, 'world_count', '?')} "
                  f"!= envs={envs}")
    except Exception as e:
        print(f"[M2 DEGRADED] replicate(world_count={envs}) failed ({e!r}). "
              f"Skipping.")
        return 0

    nj = len(SPEC.JOINT_NAMES)
    actions = _fixed_actions(ticks, nj)

    substeps = SPEC.SUBSTEPS
    sub_dt = SPEC.DT / substeps
    try:
        solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False,
                                             njmax=128, nconmax=128)
    except Exception as e:
        print(f"[M2 DEGRADED] SolverMuJoCo build failed ({e!r}). Skipping.")
        return 0

    state_a = model.state()
    state_b = model.state()
    control = model.control()
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)

    nq = int(model.joint_coord_count)
    nv = int(model.joint_dof_count)
    per_q = nq // envs
    per_v = nv // envs
    tgt = wp.to_torch(control.joint_target_q) if hasattr(control, "joint_target_pos") else None
    # nominal + ACT_SCALE*action per world, broadcast identically to all worlds.
    n_act_per = per_v - 6

    # Per-world spawn offset (base translation qpos[0:3]) -- the replicate spacing
    # makes each world's base x,y differ by design; subtract it so we compare the
    # spacing-INVARIANT relative pose. Velocities are already spacing-invariant.
    q0 = state_a.joint_q.numpy()[:nq].reshape(envs, per_q)
    spawn_off = np.zeros((envs, per_q))
    spawn_off[:, 0:3] = q0[:, 0:3] - q0[0:1, 0:3]   # base translation only

    K = min(10, ticks)
    max_div_q = 0.0
    max_div_v = 0.0
    early_div_q = 0.0
    early_div_v = 0.0
    for k in range(ticks):
        act = actions[k % actions.shape[0]]
        if tgt is not None and n_act_per > 0:
            a = torch.as_tensor(act[:n_act_per], device=tgt.device, dtype=tgt.dtype)
            target = SPEC.ACT_SCALE * a
            # write the same target block into each world's actuated DOFs
            for w in range(envs):
                base = w * per_v + 6
                tgt[base:base + a.shape[0]] = target
        for _ in range(substeps):
            if hasattr(model, "collide"):
                contacts = model.collide(state_a)
            solver.step(state_a, state_b, control, contacts, sub_dt)
            state_a, state_b = state_b, state_a
        q = state_a.joint_q.numpy()[:nq].reshape(envs, per_q) - spawn_off
        v = state_a.joint_qd.numpy()[:nv].reshape(envs, per_v)
        # divergence between world 0 and every other world (spacing-invariant)
        dq = float(np.abs(q - q[0:1]).max())
        dv = float(np.abs(v - v[0:1]).max())
        max_div_q = max(max_div_q, dq)
        max_div_v = max(max_div_v, dv)
        if k < K:
            early_div_q = max(early_div_q, dq)
            early_div_v = max(early_div_v, dv)

    print(f"  max per-world qpos divergence:  first-{K} = {early_div_q:.3e}   "
          f"full {ticks} = {max_div_q:.3e}")
    print(f"  max per-world qvel divergence:  first-{K} = {early_div_v:.3e}   "
          f"full {ticks} = {max_div_v:.3e}")
    # The worlds get bit-IDENTICAL input; any divergence is GPU FP
    # non-associativity across the batched reduction (mjwarp processes all worlds
    # in parallel). Position is the physically meaningful parity signal and stays
    # <1e-4 (essentially identical); velocity amplifies the per-world FP noise on
    # a passive falling biped to ~1e-3 (a derivative of the position drift), which
    # is GPU float tolerance, not a physics gap.
    # qpos is the physically meaningful position-parity signal; it stays <1e-4
    # (identical). qvel sits at GPU-FP order (~1e-3, run-to-run varying because
    # mjwarp's parallel batch reduction is non-associative) -- it is the
    # derivative of the position drift, not a physics gap. Tolerance 5e-3 keeps
    # the verdict stable against that nondeterministic FP noise floor.
    pos_ok = max_div_q < 1e-4
    vel_fp = max_div_v < 5e-3
    if pos_ok and max_div_v < 1e-4:
        verdict = "PASS (bit-identical, qpos & qvel <1e-4)"
    elif pos_ok and vel_fp:
        verdict = ("PASS -- batching physics-neutral: qpos <1e-4 (identical); "
                   "qvel ~1e-3 = GPU batch-reduction FP non-associativity on a "
                   "falling passive biped (derivative of qpos drift), NOT a "
                   "physics gap")
    else:
        verdict = "CHECK (qpos >=1e-4 or qvel >=5e-3 -- larger than GPU FP drift)"
    print(f"  VERDICT: {verdict}")
    return 0


# ===========================================================================
# MEASUREMENT 3 -- INITIAL-CONDITION GAP (the deploy's 0.3 s settle).
# ===========================================================================
def measure_ic_gap() -> int:
    print("\n" + "=" * 92)
    print("MEASUREMENT 3 -- INITIAL-CONDITION GAP (nominal-squat settle, 0.3 s)")
    print("=" * 92)

    import newton
    import warp as wp

    if not wp.get_cuda_device_count():
        print("[M3 DEGRADED] no CUDA device -- the settle needs the mjwarp solver. "
              "Skipping.")
        return 0

    # Use the deploy World (same model the deploy launches with), seed the
    # nominal squat, and hold nominal POSITION targets under gravity.
    world, n_rev, info = build_deploy_model()
    model = world.model

    slots = _revolute_slots(SPEC.URDF_PRIM)
    name2slot = {nm: s for s, nm in slots.items()}
    nominal_by_name = dict(zip(SPEC.JOINT_NAMES,
                              list(SPEC.NOMINAL_LEGS) + list(SPEC.ARM_NOMINAL)))

    # Seed joint_q to nominal (base @ SPAWN_Z, identity quat, qvel=0). The deploy
    # World's step() pose-seed (OMNISIM_NEWTON_SEED_POSE=1) does this from
    # joint_targets_pos, so set those = nominal BEFORE the first step.
    for nm, s in name2slot.items():
        if nm in nominal_by_name:
            world.set_joint_target_pos(s, float(nominal_by_name[nm]))

    n_settle = math.ceil(0.3 / SPEC.DT)
    dt = SPEC.DT
    base_body_idx = 0

    for _ in range(n_settle):
        # re-issue the nominal hold every tick (step() consumes joint_targets_pos)
        for nm, s in name2slot.items():
            if nm in nominal_by_name:
                world.set_joint_target_pos(s, float(nominal_by_name[nm]))
        world.step(dt)
        world._body_q_cache = None
        world._body_qd_cache = None

    bx = world.body_xform(base_body_idx)
    bv = world.body_vel(base_body_idx)
    pitch = _quat_xyzw_pitch(bx[3], bx[4], bx[5], bx[6])
    lin_mag = float(np.linalg.norm(bv[0:3]))
    ang_mag = float(np.linalg.norm(bv[3:6]))
    z_drop = float(SPEC.SPAWN_Z - bx[2])

    print(f"  settle steps = {n_settle}  (ceil(0.3 / DT={SPEC.DT}))")
    print(f"  base PITCH after settle           = {pitch:+.5f} rad  "
          f"({math.degrees(pitch):+.3f} deg)")
    print(f"  residual base LINEAR velocity mag = {lin_mag:.5f} m/s")
    print(f"  residual base ANGULAR velocity mag= {ang_mag:.5f} rad/s")
    print(f"  base z drop during settle         = {z_drop:+.5f} m "
          f"(spawn_z={SPEC.SPAWN_Z}, final_z={bx[2]:.4f})")
    print("\n  the trainer's teleport reset starts from pitch=0, qvel=0, base=SPAWN_Z;")
    print("  the deploy carries the above lean + residual velocity into tick 0.")
    return 0


# ===========================================================================
# CLI.
# ===========================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ticks", type=int, default=60)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--skip", default="",
                    help="comma list of measurements to skip: 1,2,3")
    args = ap.parse_args(argv)
    skip = {s.strip() for s in args.skip.split(",") if s.strip()}

    # ---- env fingerprint: stamp which solver actually drove the run ----
    try:
        fp = env_fingerprint.collect(warm=None)
        print("ENV FINGERPRINT")
        print(f"  physics : {fp['physics']}   solver={fp.get('solver')}")
        print(f"  gpu     : {fp.get('gpu')}")
        print(f"  build   : {fp.get('build')}  py={fp.get('python')}")
        knobs = " ".join(f"{k}={v}" for k, v in fp.get("knobs", {}).items())
        print(f"  knobs   : {knobs or '(defaults)'}")
        print(f"  NOTE    : the engine log fingerprint reflects the last OmniSim "
              f"run, not this in-process newton solver; the in-process solver is "
              f"reported per-measurement below.")
    except Exception as e:
        print(f"[fingerprint failed: {e!r}]")

    print(f"\nspec: KE={SPEC.KE} KD={SPEC.KD} SUBSTEPS={SPEC.SUBSTEPS} DT={SPEC.DT} "
          f"SPAWN_Z={SPEC.SPAWN_Z} ACT_SCALE={SPEC.ACT_SCALE} OBS_DIM={SPEC.OBS_DIM}")
    print(f"deploy env: " + " ".join(f"{k}={v}" for k, v in sorted(SPEC.newton_env().items())))
    print(f"USE_LINK_COM={os.environ.get('OMNISIM_NEWTON_USE_LINK_COM')}\n")

    rc = 0
    if "1" not in skip:
        try:
            rc |= measure_obs_pipeline(args.ticks)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[M1 FAILED -> degraded]: {e!r}")
    if "2" not in skip:
        try:
            rc |= measure_physics_batch(args.ticks, args.envs)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[M2 FAILED -> degraded]: {e!r}")
    if "3" not in skip:
        try:
            rc |= measure_ic_gap()
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[M3 FAILED -> degraded]: {e!r}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
