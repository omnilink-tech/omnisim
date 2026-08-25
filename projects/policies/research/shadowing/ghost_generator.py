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

"""Ghost Generator -- Shadowing Component 1 (robot-agnostic, feasible-by-construction).

Produces a DYNAMICALLY-FEASIBLE ghost from (robot MuJoCo model, intent) via receding-
horizon predictive-sampling MPC (MPPI) over the real dynamics. The control space is the
robot's POSITION-TARGET actuators -- the SAME interface the deploy/RL uses -- so the
generated motion is feasible w.r.t. the actual actuators + torque limits + contacts by
construction.

Why MPC (receding-horizon), not the earlier open-loop full-trajectory predictive sampling:
the earlier planner optimized the WHOLE open-loop knot trajectory at once and went chaotic
at stiff-contact knife-edges. Receding-horizon MPC re-plans a short lookahead each control
step and executes only the first action -- the standard, robust formulation (this is what
MuJoCo-MPC does). Robot-agnostic: depends only on the model + an Intent.

Output ghost (a reference table the verifier certifies + the RL tracker shadows):
  q[t]      (nq)  full qpos incl. free base
  qvel[t]   (nv)
  ctrl[t]   (n_pos) the position targets actually applied
  base[t]   (x, y, z, qw,qx,qy,qz)
  feet[t], com[t]  (for balance/verification)

Usage: see _demo() at the bottom, or projects/policies/research/shadowing/generate_g1_sitstand.py.
"""
from __future__ import annotations

import numpy as np
import mujoco


class Intent:
    """Robot-agnostic task spec for a ghost. All keyframes interpolated (smoothstep).

    joint_keys : list of (t_s, {joint_name: target_rad})   -- desired joint angles
    base_keys  : list of (t_s, {'x':, 'y':, 'z':, 'pitch':})-- desired base pose (any subset)
    weights    : cost weights (sensible defaults; tune per task)
    task       : optional callable (gen, d, t) -> float, ADDED to the per-step cost. Lets a
                 caller inject a task objective the keyframe machinery can't express (e.g. a
                 throw: drive the TCP linear velocity to a target at the release instant).
    task_body  : optional body name whose world position + linear velocity the generator
                 records each step (e.g. "payload" -> the thrown object's release state).
    active_joints : optional list of joint names MPPI is allowed to perturb; all others are
                 held at their nominal/keyframe value (no noise). Use to focus the search on
                 the DOFs that matter (e.g. a sagittal throw uses only the pitch joints) and to
                 stop redundant on-axis joints from spinning chaotically.
    """

    def __init__(self, total_time, joint_keys=None, base_keys=None, weights=None,
                 task=None, task_body=None, vel_limits=None, active_joints=None,
                 prescribed_joints=None):
        self.total_time = float(total_time)
        self.joint_keys = sorted(joint_keys or [], key=lambda kf: kf[0])
        self.base_keys = sorted(base_keys or [], key=lambda kf: kf[0])
        self.task = task
        self.task_body = task_body
        self.active_joints = active_joints
        # Joints whose applied command is FORCED to the keyframe value each step (NOT perturbed,
        # NOT optimized) -- a PRESCRIBED upper-body / end-effector trajectory the MPPI must
        # balance AROUND (e.g. a hand wave on a standing robot). The optimizer then handles only
        # the remaining DOFs (legs/balance), so the prescribed motion is reproduced at full
        # amplitude while the ghost stays feasible. (active_joints, if also set, still gates which
        # of the REMAINING joints get noise.)
        self.prescribed_joints = prescribed_joints
        # Per-actuated-dof joint velocity limit (rad/s), in the generator's pos_act order. When
        # set, the planner is penalized for exceeding it -- ESSENTIAL for dynamic motions
        # (throws): a torque clamp alone lets a light payload spin a low-inertia joint far past
        # its rated speed, so velocity, not torque, is the binding feasibility constraint.
        self.vel_limits = None if vel_limits is None else np.asarray(vel_limits, dtype=float)
        # NOTE: `effort` and `smooth` were historically declared here but never read by the
        # cost functions (dead weights -> there was NO effort/smoothness regularization). They
        # are now wired into _rollout_cost; defaults are 0.0 so the previous behaviour is
        # preserved EXACTLY. Opt in via weights={"smooth": ..., "effort": ...} for a smoother /
        # lower-effort ghost (smooth penalizes control rate; effort penalizes target excursion).
        w = dict(joint=6.0, base_xy=8.0, base_z=8.0, pitch=4.0,
                 upright=2.0, balance=3.0, effort=0.0, smooth=0.0, alive=0.5, vellim=50.0)
        if weights:
            w.update(weights)
        self.w = w


def _smoothstep(a):
    a = min(1.0, max(0.0, a))
    return a * a * (3.0 - 2.0 * a)


def _interp_keys(keys, t, fields):
    """Interpolate a list of (t, dict) keyframes at time t -> dict of the requested fields
    that are present (missing fields return None so the cost can skip them)."""
    if not keys:
        return {}
    if t <= keys[0][0]:
        src = keys[0][1]
    elif t >= keys[-1][0]:
        src = keys[-1][1]
    else:
        src = None
        for i in range(len(keys) - 1):
            t0, d0 = keys[i]
            t1, d1 = keys[i + 1]
            if t0 <= t <= t1:
                f = _smoothstep((t - t0) / (t1 - t0)) if t1 > t0 else 0.0
                out = {}
                for k in fields:
                    if k in d0 and k in d1:
                        out[k] = d0[k] + f * (d1[k] - d0[k])
                    elif k in d1:
                        out[k] = d1[k]
                    elif k in d0:
                        out[k] = d0[k]
                return out
        src = keys[-1][1]
    return {k: src[k] for k in fields if k in src}


def _spline_basis(horizon, n_knots):
    """(horizon, n_knots) linear-interpolation basis. Knots are evenly spaced over the
    horizon; each step is a convex combination of its two surrounding knots. Sampling knot
    perturbations dK (n_samples, n_knots, n_pos) and forming B @ dK yields SMOOTH, temporally-
    correlated control noise whose effective search dimension is n_knots << horizon -- the
    standard MuJoCo-MPC / spline-MPPI control parameterization. Cheaper + smoother than the
    per-timestep white-noise + AR-filter path."""
    B = np.zeros((horizon, n_knots))
    knot_pos = np.linspace(0.0, horizon - 1, n_knots)
    for h in range(horizon):
        j = int(np.searchsorted(knot_pos, h))
        if j <= 0:
            B[h, 0] = 1.0
        elif j >= n_knots:
            B[h, -1] = 1.0
        else:
            lo, hi = knot_pos[j - 1], knot_pos[j]
            f = 0.0 if hi == lo else (h - lo) / (hi - lo)
            B[h, j - 1] = 1.0 - f
            B[h, j] = f
    return B


class GhostGenerator:
    def __init__(self, mjcf_path, sim_dt=0.004, control_dt=0.02):
        self.m = mujoco.MjModel.from_xml_path(mjcf_path)
        self.m.opt.timestep = sim_dt
        self.sim_dt = sim_dt
        self.control_dt = control_dt
        self.n_sub = max(1, int(round(control_dt / sim_dt)))
        self.d = mujoco.MjData(self.m)

        # Fixed base = no free joint at the root (a bolted-down arm). Floating-base costs
        # (base pose, upright, balance, fell-over) read qpos[0:7] as a free-joint pose; on a
        # fixed base those addresses are hinge angles, so we must NOT apply them.
        self.fixed_base = not any(
            self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(self.m.njnt))
        self.task_bid = -1

        # POSITION-target actuators = the control space (names end "_pos"; fall back to all).
        self.pos_act = [a for a in range(self.m.nu)
                        if (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith("_pos")]
        if not self.pos_act:
            self.pos_act = list(range(self.m.nu))
        # actuator -> its joint's qpos address (for keyframe tracking + ctrl limits)
        self.act_jqpos = []
        self.act_name = []
        for a in self.pos_act:
            jid = self.m.actuator_trnid[a, 0]
            self.act_jqpos.append(self.m.jnt_qposadr[jid])
            self.act_name.append(mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or f"act{a}")
        self.act_jqpos = np.array(self.act_jqpos, dtype=int)
        # qvel (dof) address for each actuated joint -- for velocity-limit penalties.
        self.act_jdof = np.array(
            [self.m.jnt_dofadr[self.m.actuator_trnid[a, 0]] for a in self.pos_act], dtype=int)
        jrange = self.m.jnt_range[self.m.actuator_trnid[self.pos_act, 0]]
        self.ctrl_lo = jrange[:, 0].copy()
        self.ctrl_hi = jrange[:, 1].copy()

        # foot bodies (for balance) -- bodies whose name contains "ankle_roll" or "foot"
        self.foot_bodies = [b for b in range(self.m.nbody)
                            if any(s in (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, b) or "")
                                   for s in ("ankle_roll", "foot"))]

    # --- cost on a single state (mjData d), given the interpolated targets ---
    def _state_cost(self, d, jtarget, btarget, w):
        c = 0.0
        # joint-angle tracking (only the joints named in the keyframe)
        if jtarget is not None and len(jtarget):
            qa = d.qpos[self._jt_adr]
            c += w["joint"] * float(np.mean((qa - self._jt_val) ** 2))
        # --- floating-base-only terms (qpos[0:7] is a free-joint pose) ---
        if not self.fixed_base:
            # base pose
            if btarget:
                if "x" in btarget:
                    c += w["base_xy"] * (d.qpos[0] - btarget["x"]) ** 2
                if "y" in btarget:
                    c += w["base_xy"] * (d.qpos[1] - btarget["y"]) ** 2
                if "z" in btarget:
                    c += w["base_z"] * (d.qpos[2] - btarget["z"]) ** 2
                if "pitch" in btarget:
                    wq, xq, yq, zq = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
                    pitch = np.arcsin(max(-1.0, min(1.0, 2 * (wq * yq - zq * xq))))
                    c += w["pitch"] * (pitch - btarget["pitch"]) ** 2
            # upright (penalize total base tilt beyond any commanded pitch)
            R22 = 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)
            tilt = np.arccos(max(-1.0, min(1.0, R22)))
            pitch_cmd = abs(btarget.get("pitch", 0.0)) if btarget else 0.0
            c += w["upright"] * max(0.0, tilt - pitch_cmd - 0.1) ** 2
            # balance: horizontal CoM over the foot-support centroid
            if self.foot_bodies:
                com = d.subtree_com[0]
                fc = np.mean([d.xpos[b] for b in self.foot_bodies], axis=0)
                c += w["balance"] * float((com[0] - fc[0]) ** 2 + (com[1] - fc[1]) ** 2)
            # fell over
            if d.qpos[2] < 0.2 or tilt > 1.4:
                c += 50.0
        return c

    def body_linvel(self, d, bid):
        """World-frame linear velocity (3,) of body `bid`'s origin point."""
        res = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, bid, res, 0)
        return res[3:6].copy()

    def _rollout_cost(self, d, U, intent, t0):
        """Roll a control sequence U (H, n_pos) from state d (a scratch copy); return cost."""
        w = intent.w
        total = 0.0
        full = np.zeros(self.m.nu)
        w_eff = w.get("effort", 0.0)
        w_smo = w.get("smooth", 0.0)
        rest = getattr(self, "_rest", None)
        prev_c = None
        for h in range(U.shape[0]):
            t = t0 + h * self.control_dt
            ctrl_h = np.clip(U[h], self.ctrl_lo, self.ctrl_hi)
            full[self.pos_act] = ctrl_h
            d.ctrl[:] = full
            for _ in range(self.n_sub):
                mujoco.mj_step(self.m, d)
            jt = _interp_keys(intent.joint_keys, t, [n.replace("_pos", "") for n in self.act_name])
            self._set_jtargets(jt)
            bt = _interp_keys(intent.base_keys, t, ["x", "y", "z", "pitch"])
            total += self._state_cost(d, jt, bt, w)
            total += w["alive"] * (-1.0)  # reward staying in the rollout
            # effort: regularize position-target excursion from the rest pose (was inert; default 0)
            if w_eff and rest is not None:
                total += w_eff * float(np.mean((ctrl_h - rest) ** 2))
            # smooth: penalize control rate -> a less jittery, lower-jerk reference (was inert; default 0)
            if w_smo and prev_c is not None:
                total += w_smo * float(np.mean((ctrl_h - prev_c) ** 2))
            prev_c = ctrl_h
            if intent.vel_limits is not None:
                qd = d.qvel[self.act_jdof]
                over = np.maximum(0.0, np.abs(qd) - intent.vel_limits)
                total += w["vellim"] * float(np.sum(over * over))
            if intent.task is not None:
                total += float(intent.task(self, d, t + self.control_dt))
        return total

    def _set_jtargets(self, jt):
        """Cache the joint-target adr/val arrays for the current keyframe dict."""
        if not jt:
            self._jt_adr = np.zeros(0, dtype=int)
            self._jt_val = np.zeros(0)
            return
        adr, val = [], []
        for nm, a in zip([n.replace("_pos", "") for n in self.act_name], self.act_jqpos):
            if nm in jt:
                adr.append(a)
                val.append(jt[nm])
        self._jt_adr = np.array(adr, dtype=int)
        self._jt_val = np.array(val)

    def generate(self, intent, init_qpos, n_samples=64, horizon=15, noise=0.15,
                 temperature=0.4, seed=0, verbose=True, n_knots=None):
        rng = np.random.default_rng(seed)
        np_pos = len(self.pos_act)
        # active-joint mask: which control dims MPPI may perturb (others held at nominal).
        jnames = [n.replace("_pos", "") for n in self.act_name]
        act_mask = np.ones(np_pos, dtype=bool)
        if intent.active_joints is not None:
            act_mask = np.array([jn in intent.active_joints for jn in jnames], dtype=bool)
        # prescribed joints: forced to the keyframe each step, never perturbed/optimized
        presc = np.array([jn in (intent.prescribed_joints or []) for jn in jnames], dtype=bool)
        act_mask &= ~presc
        presc_names = [jnames[i] for i in range(np_pos) if presc[i]]
        # resolve the optional task body (e.g. the thrown payload) for release readout
        self.task_bid = -1
        if intent.task_body:
            self.task_bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, intent.task_body)
            if self.task_bid < 0:
                raise RuntimeError(f"task_body {intent.task_body!r} not found in model")
        # init state
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:] = init_qpos
        mujoco.mj_forward(self.m, self.d)
        # nominal control = current joint angles at the pos-actuator joints
        nominal = np.tile(self.d.qpos[self.act_jqpos], (horizon, 1))
        # rest pose reference for the (opt-in) effort regularizer
        self._rest = self.d.qpos[self.act_jqpos].copy()
        # optional knot-spline control parameterization (smoother + lower-dim search)
        spline_B = None
        nk = 0
        if n_knots is not None and 2 <= int(n_knots) < horizon:
            nk = int(n_knots)
            spline_B = _spline_basis(horizon, nk)
        n_steps = int(round(intent.total_time / self.control_dt))
        scratch = mujoco.MjData(self.m)

        Q, QV, C, BASE, FEET, COM, TCP, TCPV = [], [], [], [], [], [], [], []
        for k in range(n_steps):
            t0 = k * self.control_dt
            # PRESCRIBED joints: overwrite the nominal horizon with the keyframe trajectory so
            # the rollouts evaluate (and the applied action uses) the exact designed motion.
            if presc_names:
                for h in range(horizon):
                    kf = _interp_keys(intent.joint_keys, t0 + h * self.control_dt, presc_names)
                    for nm, val in kf.items():
                        nominal[h, jnames.index(nm)] = val
            # sample K control sequences = nominal + (knot-spline | AR-smoothed) noise
            if spline_B is None:
                dU = rng.normal(0.0, noise, size=(n_samples, horizon, np_pos))
                for h in range(1, horizon):                  # temporal smoothing (AR)
                    dU[:, h] = 0.6 * dU[:, h] + 0.4 * dU[:, h - 1]
            else:
                # sample nk control knots, interpolate to the full horizon: smoother noise,
                # search dim nk<<horizon (MuJoCo-MPC / spline-MPPI parameterization).
                dK = rng.normal(0.0, noise, size=(n_samples, nk, np_pos))
                dU = np.einsum("hk,nkc->nhc", spline_B, dK)
            dU[:, :, ~act_mask] = 0.0                         # freeze inactive joints
            samples = nominal[None] + dU
            samples[0] = nominal                             # keep the nominal as a candidate
            costs = np.empty(n_samples)
            for s in range(n_samples):
                scratch.qpos[:] = self.d.qpos
                scratch.qvel[:] = self.d.qvel
                scratch.time = self.d.time
                mujoco.mj_forward(self.m, scratch)
                costs[s] = self._rollout_cost(scratch, samples[s], intent, t0)
            # softmax (MPPI) update
            beta = costs.min()
            wgt = np.exp(-(costs - beta) / max(1e-6, temperature))
            wgt /= wgt.sum()
            nominal = np.einsum("s,shc->hc", wgt, samples)
            nominal = np.clip(nominal, self.ctrl_lo, self.ctrl_hi)

            # APPLY the first control to the real state, advance one control step
            full = np.zeros(self.m.nu)
            full[self.pos_act] = nominal[0]
            self.d.ctrl[:] = full
            for _ in range(self.n_sub):
                mujoco.mj_step(self.m, self.d)
            # record
            Q.append(self.d.qpos.copy()); QV.append(self.d.qvel.copy())
            C.append(nominal[0].copy()); BASE.append(self.d.qpos[:7].copy())
            COM.append(self.d.subtree_com[0].copy())
            FEET.append(np.array([self.d.xpos[b].copy() for b in self.foot_bodies]) if self.foot_bodies else np.zeros((0, 3)))
            if self.task_bid >= 0:
                TCP.append(self.d.xpos[self.task_bid].copy())
                TCPV.append(self.body_linvel(self.d, self.task_bid))
            # warm-start: shift the horizon
            nominal = np.vstack([nominal[1:], nominal[-1:]])
            if verbose and (k % 25 == 0 or k == n_steps - 1):
                if self.fixed_base:
                    extra = ""
                    if self.task_bid >= 0:
                        tp = self.d.xpos[self.task_bid]
                        tv = self.body_linvel(self.d, self.task_bid)
                        extra = f"  tcp=({tp[0]:.2f},{tp[2]:.2f}) |v|={np.linalg.norm(tv):.2f}"
                    print(f"  [gen] t={t0:4.2f}s{extra}  cost={costs.min():.2f}")
                else:
                    z = self.d.qpos[2]; tl = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (self.d.qpos[4] ** 2 + self.d.qpos[5] ** 2)))))
                    print(f"  [gen] t={t0:4.2f}s  base_z={z:.3f}  tilt={tl:4.1f}deg  cost={costs.min():.2f}")

        out = dict(dt=self.control_dt, joints=[n.replace("_pos", "") for n in self.act_name],
                   q=np.array(Q), qvel=np.array(QV), ctrl=np.array(C),
                   base=np.array(BASE), com=np.array(COM), feet=np.array(FEET))
        if self.task_bid >= 0:
            out["tcp"] = np.array(TCP)
            out["tcpvel"] = np.array(TCPV)
        return out


def _demo():
    """Smoke test: generate a 'hold a small squat' ghost for the G1 (no chair) -- validates
    the MPPI machinery + balance cost produce a feasible, upright, stable trajectory."""
    import os
    REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    mjcf = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
    gen = GhostGenerator(mjcf)
    # init: drop to a settled stand
    mujoco.mj_resetData(gen.m, gen.d)
    init = gen.d.qpos.copy()
    init[2] = 0.78
    # intent: hold upright, slight squat (hips -0.3, knees 0.6), CoM over feet, 2 s
    hold = {"left_hip_pitch_joint": -0.30, "right_hip_pitch_joint": -0.30,
            "left_knee_joint": 0.60, "right_knee_joint": 0.60,
            "left_ankle_pitch_joint": -0.30, "right_ankle_pitch_joint": -0.30}
    intent = Intent(total_time=2.0,
                    joint_keys=[(0.0, hold), (2.0, hold)],
                    base_keys=[(0.0, {"z": 0.74, "pitch": 0.0}), (2.0, {"z": 0.74, "pitch": 0.0})])
    g = gen.generate(intent, init, n_samples=48, horizon=12)
    print(f"[demo] ghost {g['q'].shape[0]} steps; final base_z={g['base'][-1,2]:.3f}")


if __name__ == "__main__":
    _demo()
