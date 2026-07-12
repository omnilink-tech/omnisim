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

"""GPU mujoco_warp trainer using the model+residual recipe.

Same physics/PPO machinery as `gpu_mjwarp_trainer.py` but the
per-step "model layer" is replaced from a simple CPG (sin/cos hip_y
oscillation) to the full **gait → inverse kinematics → joint
targets + balance** stack that produced the SB3+Webots
`spot_residual_main` walker in 20k steps.

Per step, for each of the N parallel envs:

    1. Foot targets in body frame come from the analytic gait engine
       (`spot_gait_np.foot_targets_batched`) given the commanded
       (vx, vy, wz) and an internal phase clock.
    2. The 12-dim policy action is interpreted as ±RES_SCALE m foot
       offsets in body frame (NOT as ±0.15 rad joint deltas like the
       CPG trainer).
    3. Foot target + residual → joint angles via the vectorized
       analytic IK (`spot_kinematics_np.inverse_kinematics_batched`).
    4. Joint angles are written into mjData.ctrl as the position
       target for each MuJoCo position+velocity actuator; the
       physics step runs as before.

This matches the recipe documented in
`docs/developer/spot-residual-rl.md` and proven on SB3+Webots.

Same CLI args as `gpu_mjwarp_trainer.py` -- this file deliberately
keeps the rest (PPO loop, obs vector, reward shape) identical so we
can A/B between recipes at the same speed (~60 k env-steps/s on a
single RTX 5070).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "projects" / "rl" / "tools"))

from projects.policies.control.spot_gait import GaitParams  # noqa: E402
from projects.policies.control.spot_gait_np import (  # noqa: E402
    foot_targets_batched,
)
from projects.policies.control.spot_kinematics_np import (  # noqa: E402
    inverse_kinematics_batched,
)


# ---- joint / obs layout (quad: free root + 12 leg joints) ----
NJ = 12
QPOS_J0 = 7      # leg joints start in qpos
QVEL_J0 = 6      # leg joints start in qvel
OBS_DIM = 49     # match CPG trainer for shared PPO machinery
# LEG_SIGNS[i] = (front_sign, left_sign) for leg i in order FL, FR, RL, RR.
# hip_x sign is LEFT/RIGHT (+0.30 for left legs, -0.30 for right legs),
# NOT front/rear. Earlier this used the front sign by mistake, which
# swapped hip_x values for FR and RL during IK NaN fallbacks.
LEG_SIGNS = [(+1, +1), (+1, -1), (-1, +1), (-1, -1)]
NOMINAL = np.array(
    [v for (_, ay) in LEG_SIGNS
     for v in ((0.30 if ay > 0 else -0.30), 0.30, -0.60)],
    dtype=np.float32,
)

# Residual is interpreted as a 12-dim JOINT-SPACE delta (radians),
# NOT a foot-position offset. Joint-space gives the policy direct
# authority over the actuator targets, which was needed once the
# model layer's gait+IK baseline turned out to produce nearly zero
# net forward thrust in the MJCF/mujoco_warp physics (the policy
# has to learn the entire forward push from the residual). The
# SB3+Webots variant used 0.03 m foot offsets because there the
# baseline already walked; here we mirror the CPG trainer's
# ±0.15 rad joint deltas.
RES_SCALE = 0.15

DT = 0.016
STAND_Z = 0.7
ROLL_FAIL = 1.0
PITCH_FAIL = 1.0
BZ_FAIL = 0.45    # was 0.30 — too lenient; policies would belly-crawl
MAX_EP = 1024

# Joint-target clamps matching the ACTUAL MJCF joint ranges on disk
# (NOT the widened URDF ranges -- the MJCF was generated before the
# widening). hip_x sign flips between left/right legs; trainer keeps
# a single ±1.5 band and lets MJCF physics enforce the per-leg sign.
JOINT_LIMITS_LO = np.array([
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
    -1.50, +0.00, -1.20,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
    +1.50, +0.60, -0.01,
], dtype=np.float32)


def quat_to_rp(q):  # q: [N,4] (w,x,y,z) -> roll, pitch
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def proj_gravity(q):  # body-frame gravity unit vector, [N,3]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    gx = -2 * (x * z - w * y)
    gy = -2 * (y * z + w * x)
    gz = -(1 - 2 * (x * x + y * y))
    return np.stack([gx, gy, gz], axis=1).astype(np.float32)


class BatchedQuadResidualEnv:
    """Batched Spot env using gait+IK model layer + foot-offset residual."""

    def __init__(self, n, mjcf, fixed_vx=0.5, device="cuda:0",
                 reward_cfg=None, sim_dt=0.0,
                 gait_params: GaitParams = None, init_noise=0.0,
                 wz_range=0.0):
        self.init_noise = float(init_noise)
        self.wz_range = float(wz_range)
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.device = wp.get_device(device)
        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        if sim_dt and sim_dt > 0:
            self.mjm.opt.timestep = sim_dt
        # Disable upper-body geom self-collisions. The MJCF auto-generated
        # from the URDF only excludes parent-child body pairs in the
        # contact graph -- the body-chassis geom (idx 1) overlaps the
        # hip_x geoms (2,5,8,11) and the hip_y "thigh" geoms (3,6,9,12)
        # and contact forces from those phantom collisions cause the
        # body to drift backward regardless of gait direction. Only the
        # 4 shin geoms (4,7,10,13) need to collide for floor contact.
        for gid in (1, 2, 3, 5, 6, 8, 9, 11, 12):
            self.mjm.geom_contype[gid] = 0
            self.mjm.geom_conaffinity[gid] = 0
        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n)
        # ── Joint-order permutation ──
        # Controller order = [FL_hx, FL_hy, FL_knee, FR_*, RL_*, RR_*].
        # The MJCF on disk happens to be in the same order (joints
        # grouped by leg), so the permutation is identity -- but we
        # derive it explicitly to stay robust to URDFs that group by
        # joint type instead.
        self.controller_to_mjcf, self.nominal_mjcf = self._derive_joint_order(
            mjm=self.mjm, mjd=mjd)
        self.fixed_vx = fixed_vx
        self.r = reward_cfg or {}
        # GaitParams tuned for THIS MJCF (mujoco_warp physics):
        # - neutral_(front_x|rear_x|lateral_y) match FK of nominal
        #   stand pose (hip_x=±0.30, hip_y=+0.30, knee=-0.60).
        # - ground_z = -0.62 (slightly past nominal FK z=-0.559) so
        #   the legs stay slightly compressed against the floor under
        #   body weight -- if equal, the body floats and feet barely
        #   touch; if much more, the legs spring upward.
        # - step_height=0.04 is the smallest swing arc that still
        #   clears the floor under load; larger arcs create backward
        #   reaction force because the actuator can't track the fast
        #   swing trajectory.
        # NOTE the historic ground_z=-0.62 exceeded the legs' reach at this
        # lateral: the IK solved only ~30% of gait phases (0% at stance!),
        # silently falling back to a CONSTANT nominal pose -- so the
        # "gait+IK baseline" never existed and the residual had to invent
        # locomotion from scratch (the unnatural learned shuffle). At
        # ground_z >= -0.57 the trot is 100% IK-reachable across the cycle
        # and the baseline really is a rhythmic trot for the residual to
        # polish. (See _scratch/ik_reach_sweep.py for the reach map.)
        self.gait = gait_params or GaitParams(
            neutral_front_x=0.322, neutral_rear_x=-0.274,
            neutral_lateral_y=0.344, ground_z=-0.57,
            step_height=0.06)
        # Seed body z so the feet rest on the floor at neutral pose.
        # Foot world z = body z + foot body z = body z - 0.559. Set
        # body z = 0.559 + 0.01 (1 cm gap to absorb settle compression).
        seed_body_z = -self.gait.ground_z + 0.01
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0, 0, seed_body_z]
        self.seed_qpos[3:7] = [1, 0, 0, 0]
        self.seed_qpos[QPOS_J0:QPOS_J0 + NJ] = self.nominal_mjcf
        self.nq = self.mjm.nq
        self.nv = self.mjm.nv
        self.nu = self.mjm.nu
        self.t = np.zeros(n, dtype=np.float32)   # per-env gait clock
        self.ep_step = np.zeros(n, dtype=np.int32)
        self.last_action = np.zeros((n, NJ), dtype=np.float32)
        self.vel_cmd = np.zeros((n, 3), dtype=np.float32)
        self.vel_cmd[:, 0] = fixed_vx
        # Pre-allocate the foot-target buffer we mutate each step (avoid
        # per-step allocations of (N, 4, 3) which costs noticeable time
        # at 1024 envs).
        self._foot_buf = np.zeros((n, 4, 3), dtype=np.float32)
        self._reset_all()

    @staticmethod
    def _derive_joint_order(mjm, mjd):
        """Map controller-order joint targets to MJCF qpos order.

        Returns (controller_to_mjcf, nominal_mjcf):
            controller_to_mjcf[i]: MJCF qpos index for controller idx i
                (where controller is FL_hx, FL_hy, FL_knee, FR_*, ...).
            nominal_mjcf: shape (12,), the nominal stand pose in MJCF
                qpos order (so the seed quaternion lines up too).
        """
        import mujoco
        controller_legs = ("FL", "FR", "RL", "RR")
        controller_joints = ("hip_x", "hip_y", "knee")
        # Classify each MJCF hinge by body position (x>0 front, y>0 left)
        # and joint axis (1 0 0 → hip_x; 0 1 0 with [+] range → hip_y;
        # 0 1 0 with [-] range → knee).
        mjcf_to_label = {}
        k_qpos = 7  # skip free-joint qpos
        for j in range(1, mjm.njnt):
            if mjm.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            ax = mjm.jnt_axis[j]
            rng = mjm.jnt_range[j]
            pos = mjd.xpos[mjm.jnt_bodyid[j]]
            front = pos[0] > 0
            left = pos[1] > 0
            leg = ("FL" if (front and left) else
                   "FR" if (front and not left) else
                   "RL" if (not front and left) else "RR")
            # Axis test FIRST: the widened spot.urdf opens hip_x to +/-1.5
            # rad (a599e07e), so a range-first test (rng[0] < -0.9 -> knee)
            # misclassifies hip_x as knee on post-widen MJCF dumps. hip_x is
            # the only X-axis hinge; among Y-axis hinges the knee is the one
            # with the strongly negative lower range.
            if abs(ax[0]) > 0.5:
                joint = "hip_x"
            elif rng[0] < -0.9:
                joint = "knee"
            else:
                joint = "hip_y"
            mjcf_to_label[k_qpos] = (leg, joint)
            k_qpos += 1
        # Build controller->MJCF index map.
        label_to_mjcf = {v: k for k, v in mjcf_to_label.items()}
        controller_to_mjcf = np.zeros(NJ, dtype=np.int32)
        nominal_mjcf = np.zeros(NJ, dtype=np.float32)
        for ci, leg in enumerate(controller_legs):
            for cj, joint in enumerate(controller_joints):
                idx_ctrl = ci * 3 + cj
                idx_mjcf = label_to_mjcf[(leg, joint)] - 7  # zero-relative
                controller_to_mjcf[idx_ctrl] = idx_mjcf
                # nominal: hip_x = +0.30 (L) / -0.30 (R), hip_y = 0.30,
                # knee = -0.60. Put each in MJCF order.
                if joint == "hip_x":
                    nominal_mjcf[idx_mjcf] = 0.30 if leg in ("FL", "RL") else -0.30
                elif joint == "hip_y":
                    nominal_mjcf[idx_mjcf] = 0.30
                else:
                    nominal_mjcf[idx_mjcf] = -0.60
        return controller_to_mjcf, nominal_mjcf

    def _write_qpos_qvel(self, qpos, qvel):
        self.mw_d.qpos.assign(qpos)
        self.mw_d.qvel.assign(qvel)

    def _reset_all(self):
        qpos = np.tile(self.seed_qpos, (self.n, 1)).astype(np.float32)
        qvel = np.zeros((self.n, self.nv), dtype=np.float32)
        # Gentle init DR: jitter the seeded joints/height so the policy
        # has a robustness margin around the nominal start instead of
        # memorizing one deterministic trajectory (deploy's settle state
        # never matches the seed exactly).
        if getattr(self, "init_noise", 0.0) > 0.0:
            s = self.init_noise
            qpos[:, 2] += np.random.uniform(-s * 0.2, s * 0.2, self.n)
            qpos[:, QPOS_J0:QPOS_J0 + NJ] += np.random.uniform(
                -s, s, (self.n, NJ)).astype(np.float32)
            qvel[:, 0:3] = np.random.uniform(
                -s, s, (self.n, 3)).astype(np.float32)
        self._write_qpos_qvel(qpos, qvel)
        self.t[:] = 0.0
        self.ep_step[:] = 0
        self.last_action[:] = 0.0
        # Per-episode yaw-rate command (uniform +/-wz_range, with a third
        # of envs pinned to 0 so straight-walking stays well-covered).
        if getattr(self, "wz_range", 0.0) > 0.0:
            self.vel_cmd[:, 2] = np.random.uniform(
                -self.wz_range, self.wz_range, self.n).astype(np.float32)
            self.vel_cmd[np.random.rand(self.n) < 0.34, 2] = 0.0
        # Deploy-faithful qd: the controller reads joint velocity by
        # finite-differencing 16 ms position-sensor samples, so train on
        # the same signal (previous-control-step q, not physics qvel).
        self._prev_q = np.tile(
            self.seed_qpos[QPOS_J0:QPOS_J0 + NJ], (self.n, 1)
        ).astype(np.float32)
        with self.wp.ScopedDevice(self.device):
            self.mjw.forward(self.mw_m, self.mw_d)

    def _read_state(self):
        return self.mw_d.qpos.numpy(), self.mw_d.qvel.numpy()

    def _build_obs(self, qpos, qvel):
        vlin = qvel[:, 0:3]
        vang = qvel[:, 3:6]
        pg = proj_gravity(qpos[:, 3:7])
        q = qpos[:, QPOS_J0:QPOS_J0 + NJ]
        # Finite-diff qd over the control period -- matches the deploy
        # controller's sensor-based estimate (physics qvel is unavailable
        # to the real controller).
        qd = (q - self._prev_q) / DT
        self._prev_q = q.copy()
        # Gait clock as a (0,1] phase — same role as CPG trainer's
        # `phase` so the obs vector dimensions stay matched.
        clock = (self.t % self.gait.period_s) / self.gait.period_s
        clock = clock[:, None]
        obs = np.concatenate([vlin, vang, pg, q, qd, self.last_action,
                              self.vel_cmd, clock], axis=1).astype(np.float32)
        return np.clip(np.nan_to_num(obs, nan=0.0, posinf=10, neginf=-10),
                       -10, 10)

    def reset(self):
        self._reset_all()
        qpos, qvel = self._read_state()
        return self._build_obs(qpos, qvel)

    # ── The new model layer: gait engine → IK → joint targets ──
    def _model_targets(self, action):
        """Compute joint targets = IK(gait_foot_targets) + residual.
        action: (N, 12) in [-1, 1], interpreted as a JOINT-SPACE delta
        (radians, scaled by RES_SCALE). Returns (N, 12) joint targets
        in MJCF qpos order.
        """
        # gait_foot: (N, 4, 3)
        gait_foot = foot_targets_batched(
            self.t.astype(np.float64),
            self.vel_cmd[:, 0].astype(np.float64),
            self.vel_cmd[:, 1].astype(np.float64),
            self.vel_cmd[:, 2].astype(np.float64),
            self.gait,
        )
        q_target = inverse_kinematics_batched(gait_foot)
        nan_mask = np.isnan(q_target)
        if nan_mask.any():
            nominal_4x3 = NOMINAL.reshape(4, 3)
            broad = np.broadcast_to(nominal_4x3,
                                    q_target.shape).astype(q_target.dtype)
            q_target = np.where(nan_mask, broad, q_target)
        q_ctrl = q_target.reshape(self.n, NJ)
        # Add joint-space residual.
        q_ctrl = q_ctrl + action * RES_SCALE
        q_mjcf = q_ctrl[:, self.controller_to_mjcf]
        return np.clip(q_mjcf, JOINT_LIMITS_LO, JOINT_LIMITS_HI)

    def step(self, action):
        action = np.clip(action, -1, 1).astype(np.float32)
        self.t += DT
        targets = self._model_targets(action)
        # Write interleaved ctrl: [pos, vel] per joint actuator.
        ctrl = np.zeros((self.n, self.nu), dtype=np.float32)
        ctrl[:, 0::2] = targets
        self.mw_d.ctrl.assign(ctrl)
        # Physics DECIMATION: advance mjm.opt.timestep-sized physics steps
        # until one control period (DT=16 ms) has elapsed -- 8 steps at the
        # MuJoCo-default 0.002 s. This matches the OmniSim deploy exactly
        # (OMNISIM_NEWTON_SUBSTEPS=8 -> 8 x 2 ms per 16 ms control tick).
        # The historic single-mjw.step-per-control version advanced physics
        # only 2 ms per 16 ms of gait clock: an 8x time-warped world that no
        # deploy configuration could ever match (a root cause of the May
        # "GPU policy doesn't transfer" mystery).
        n_phys = max(1, int(round(DT / self.mjm.opt.timestep)))
        with self.wp.ScopedDevice(self.device):
            for _ in range(n_phys):
                self.mjw.step(self.mw_m, self.mw_d)
        qpos, qvel = self._read_state()
        self.ep_step += 1
        # Action-rate (smoothness) penalty input: distance from the PREVIOUS
        # action, computed before last_action is overwritten.
        act_rate = np.sum((action - self.last_action) ** 2, axis=1)
        self.last_action = action

        bz = qpos[:, 2]
        roll, pitch = quat_to_rp(qpos[:, 3:7])
        vx = qvel[:, 0]
        r = self.r
        v_xy = qvel[:, 0:2]
        lin_err = np.sum((v_xy - self.vel_cmd[:, 0:2]) ** 2, axis=1)
        r_lin = r.get("lin", 1.0) * np.exp(-lin_err / r.get("lin_scale", 0.25))
        vx_eff = np.maximum(0.0, vx - 0.05)
        cap = r.get("vx_cap", 0.0)
        if cap > 0:
            vx_eff = np.minimum(vx_eff, cap)
        r_vxb = r.get("vx_bonus", 0.0) * vx_eff
        r_alive = r.get("alive", 1.0) * np.ones(self.n, dtype=np.float32)
        r_vz = r.get("vz", -0.3) * qvel[:, 2] ** 2
        r_rp = r.get("rp", -0.1) * (roll ** 2 + pitch ** 2)
        r_wxy = r.get("wxy", -0.05) * (qvel[:, 3] ** 2 + qvel[:, 4] ** 2)
        # Yaw-rate COMMAND tracking: the policy must follow vel_cmd[2]
        # (including holding wz=0 when commanded straight). Without this
        # the obs has no heading information at all and the deployed
        # walker veers in a wide arc; with it, deploy closes a heading
        # loop simply by writing the wz command each tick.
        wz_err = (qvel[:, 5] - self.vel_cmd[:, 2]) ** 2
        r_wzt = r.get("wz_track", 0.0) * np.exp(
            -wz_err / r.get("wz_scale", 0.25))
        r_act = r.get("act", 0.0) * np.sum(action ** 2, axis=1)
        # Smoothness: penalize per-step action CHANGES (twitchy high-frequency
        # residuals read as unnatural even when they locomote well).
        r_actr = r.get("act_rate", 0.0) * act_rate
        # Body-height reward: penalize quadratically when body sinks
        # below `bz_target` (typically the settled stand height ~0.58).
        # Without this, the residual learns to crouch progressively for
        # extra forward momentum and eventually belly-crashes.
        bz_target = r.get("bz_target", 0.55)
        bz_def = np.maximum(0.0, bz_target - bz)
        r_bz = r.get("bz_pen", -5.0) * bz_def ** 2
        reward = (r_lin + r_vxb + r_alive + r_vz + r_rp + r_wxy + r_wzt
                  + r_act + r_actr + r_bz).astype(np.float32)

        fall = ((np.abs(roll) > ROLL_FAIL) | (np.abs(pitch) > PITCH_FAIL) |
                (bz < BZ_FAIL))
        done = fall | (self.ep_step >= MAX_EP)
        reward = reward + fall.astype(np.float32) * r.get("term", -1.0)
        if done.any():
            qpos2 = qpos.copy(); qvel2 = qvel.copy()
            idx = np.where(done)[0]
            qpos2[idx] = self.seed_qpos
            qvel2[idx] = 0.0
            if getattr(self, "init_noise", 0.0) > 0.0:
                s = self.init_noise
                qpos2[idx, 2] += np.random.uniform(-s * 0.2, s * 0.2, len(idx))
                qpos2[idx, QPOS_J0:QPOS_J0 + NJ] += np.random.uniform(
                    -s, s, (len(idx), NJ)).astype(np.float32)
                qvel2[idx, 0:3] = np.random.uniform(
                    -s, s, (len(idx), 3)).astype(np.float32)
            self._write_qpos_qvel(qpos2, qvel2)
            self.t[idx] = 0.0
            self.ep_step[idx] = 0
            self.last_action[idx] = 0.0
            if getattr(self, "wz_range", 0.0) > 0.0:
                self.vel_cmd[idx, 2] = np.random.uniform(
                    -self.wz_range, self.wz_range, len(idx)).astype(np.float32)
                sub = idx[np.random.rand(len(idx)) < 0.34]
                self.vel_cmd[sub, 2] = 0.0
            # Reset the finite-diff qd history for the re-seeded envs.
            self._prev_q[idx] = qpos2[idx, QPOS_J0:QPOS_J0 + NJ]
            with self.wp.ScopedDevice(self.device):
                self.mjw.forward(self.mw_m, self.mw_d)
            qpos, qvel = self._read_state()
        obs = self._build_obs(qpos, qvel)
        return obs, reward, done, {"vx": vx, "bz": bz, "roll": roll,
                                    "pitch": pitch}


# ────────────────────────────────────────────────────────────────────
# PPO loop — same shape as gpu_mjwarp_trainer.main, just swapped env.
# ────────────────────────────────────────────────────────────────────
def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=2048)
    p.add_argument("--iters", type=int, default=150)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--mjcf", default=r"C:\tmp\spot_newton_fixed.xml")
    p.add_argument("--fixed-vx", type=float, default=0.5)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/"
                                      "gpu_spot_residual/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=1024)
    p.add_argument("--vx-bonus", type=float, default=4.0)
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--lin", type=float, default=2.0)
    p.add_argument("--lin-scale", type=float, default=0.1)
    p.add_argument("--act-pen", type=float, default=0.0)
    p.add_argument("--bz-pen", type=float, default=-5.0,
                   help="quadratic penalty per (target - body_z)^2 when "
                        "body sinks below bz_target")
    p.add_argument("--bz-target", type=float, default=0.55)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    p.add_argument("--init-noise", type=float, default=0.0,
                   help="uniform init DR half-width: joints +/-x rad, body z "
                        "+/-0.2x m, body vel +/-x m/s (deploy-robustness)")
    p.add_argument("--term", type=float, default=-1.0,
                   help="terminal fall penalty. The historic -1 was ~nothing "
                        "against the ~+3.7/step income, so PPO settled on a "
                        "surge-crash-reset loop instead of walking.")
    p.add_argument("--wz-range", type=float, default=0.0,
                   help="per-episode yaw-rate command range (rad/s); enables "
                        "the wz_track reward so deploy can steer through "
                        "vel_cmd[2] (heading hold).")
    p.add_argument("--wz-track", type=float, default=1.0,
                   help="weight of the yaw-rate tracking reward when "
                        "--wz-range is set")
    p.add_argument("--act-rate", type=float, default=0.0,
                   help="penalty weight on per-step action CHANGES "
                        "(smoothness / gait naturalness)")
    p.add_argument("--res-scale", type=float, default=None,
                   help="override the joint-space residual half-width "
                        "(rad); smaller keeps the analytic trot shape")
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Run the Spot Newton export "
            f"first (see projects/policies/research/training/spot_native.py).")

    reward_cfg = dict(lin=args.lin, lin_scale=args.lin_scale,
                      vx_bonus=args.vx_bonus, vx_cap=args.fixed_vx,
                      alive=args.alive, vz=-0.3, rp=-0.1, wxy=-0.05,
                      term=args.term, act=args.act_pen,
                      wz_track=(args.wz_track if args.wz_range > 0 else 0.0),
                      act_rate=args.act_rate,
                      bz_pen=args.bz_pen, bz_target=args.bz_target)
    if args.res_scale is not None:
        global RES_SCALE
        RES_SCALE = float(args.res_scale)
    env = BatchedQuadResidualEnv(args.envs, args.mjcf,
                                  fixed_vx=args.fixed_vx,
                                  reward_cfg=reward_cfg, sim_dt=args.sim_dt,
                                  init_noise=args.init_noise,
                                  wz_range=args.wz_range)
    N = args.envs

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    torch.manual_seed(0)
    ac = AC()
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location="cpu"))
        print(f"warm-start from {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.eval()
        ac.load_state_dict(torch.load(args.save, map_location="cpu"))
        obs = env.reset()
        # HONEST metrics. The old `survived += ~done` accumulated alive-steps
        # ACROSS auto-resets, so a policy that crash-looped every ~60 steps
        # scored "survival 1476/1500" (1500 minus 24 deaths) and its vx mean
        # blended takeoff surges across crashes -- the metric that made every
        # crash-looping residual policy (May included) look like a walker.
        # Now: time-to-FIRST-fall per env + vx measured only before it.
        first_fall = np.full(env.n, args.eval_steps, dtype=np.int32)
        deaths = np.zeros(env.n, dtype=np.int32)
        vx_traces = []
        alive = np.ones(env.n, dtype=bool)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(torch.from_numpy(obs))
                act = mu.numpy()
            obs, _, done, info = env.step(act)
            newly = done & alive
            first_fall[newly] = step + 1
            alive &= ~done
            deaths += done.astype(np.int32)
            vx_traces.append(np.where(alive | newly, info["vx"], np.nan))
        vx_arr = np.stack(vx_traces, axis=0)
        vx_pre_fall = np.nanmean(vx_arr, axis=0)
        med = np.median(vx_pre_fall)
        in_band = np.mean((vx_pre_fall > 0.3) & (vx_pre_fall < 0.7))
        print(f"[gpu-eval]  policy={args.save}  envs={env.n}  "
              f"steps={args.eval_steps}")
        print(f"  FIRST fall (s): mean={first_fall.mean() * DT:.2f} "
              f"median={np.median(first_fall) * DT:.2f} "
              f"never-fell={(first_fall >= args.eval_steps).mean():.2f}  "
              f"deaths/env={deaths.mean():.1f}")
        print(f"  forward vx pre-fall (m/s): mean={np.nanmean(vx_pre_fall):.3f} "
              f"median={med:.3f} (target {args.fixed_vx})  "
              f"frac in[0.3,0.7]={in_band:.2f}")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)

    # ── PPO loop (same compact one used in gpu_mjwarp_trainer.main) ──
    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    for it in range(1, args.iters + 1):
        obs_buf = np.zeros((rollout, N, OBS_DIM), dtype=np.float32)
        act_buf = np.zeros((rollout, N, NJ), dtype=np.float32)
        logp_buf = np.zeros((rollout, N), dtype=np.float32)
        rew_buf = np.zeros((rollout, N), dtype=np.float32)
        done_buf = np.zeros((rollout, N), dtype=np.float32)
        val_buf = np.zeros((rollout, N), dtype=np.float32)

        for k in range(rollout):
            with torch.no_grad():
                mu, v, log_std = ac(torch.from_numpy(obs))
                std = log_std.exp()
                dist = torch.distributions.Normal(mu, std)
                a = dist.sample()
                logp = dist.log_prob(a).sum(-1)
            act_np = a.numpy().astype(np.float32)
            obs_buf[k] = obs; act_buf[k] = act_np
            logp_buf[k] = logp.numpy(); val_buf[k] = v.numpy()
            obs, r, done, _ = env.step(act_np)
            rew_buf[k] = r; done_buf[k] = done.astype(np.float32)
            total_steps += N

        with torch.no_grad():
            _, last_v, _ = ac(torch.from_numpy(obs))
            last_v = last_v.numpy()
        # GAE
        gamma, lam = 0.99, 0.95
        adv = np.zeros_like(rew_buf)
        last_gae = np.zeros(N, dtype=np.float32)
        for k in reversed(range(rollout)):
            nonterm = 1.0 - done_buf[k]
            nextv = last_v if k == rollout - 1 else val_buf[k + 1]
            delta = rew_buf[k] + gamma * nextv * nonterm - val_buf[k]
            last_gae = delta + gamma * lam * nonterm * last_gae
            adv[k] = last_gae
        ret = adv + val_buf
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        obs_t = torch.from_numpy(obs_buf.reshape(-1, OBS_DIM))
        act_t = torch.from_numpy(act_buf.reshape(-1, NJ))
        logp_t = torch.from_numpy(logp_buf.reshape(-1))
        adv_t = torch.from_numpy(adv.reshape(-1))
        ret_t = torch.from_numpy(ret.reshape(-1))

        clip_eps = 0.2
        for _epoch in range(4):
            mu, v, log_std = ac(obs_t)
            std = log_std.exp()
            dist = torch.distributions.Normal(mu, std)
            new_logp = dist.log_prob(act_t).sum(-1)
            ratio = (new_logp - logp_t).exp()
            surr1 = ratio * adv_t
            surr2 = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps) * adv_t
            pi_loss = -torch.min(surr1, surr2).mean()
            v_loss = ((v - ret_t) ** 2).mean()
            ent = dist.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - 0.01 * ent
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()

        if it % 5 == 0 or it == 1:
            ep_rps = rew_buf.mean()
            mean_v = val_buf.mean()
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{ep_rps:+.3f}  "
                  f"meanV {mean_v:+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in "
          f"{time.time() - t0:.1f}s)")


if __name__ == "__main__":
    main()
