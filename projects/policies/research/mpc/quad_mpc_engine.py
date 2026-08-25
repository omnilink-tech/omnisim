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

"""In-engine quad-locomotion MPC. Called as loco_step(self) from OmNewtonBackend's
embedded Python via the OMNISIM_INENGINE_MPC_LOCO hook (the C++ hook just imports +
calls this, so this file is iterable WITHOUT a rebuild).

MPPI-residual-on-trot rolled out in the deploy's OWN mujoco_warp solver (zero
model-gap). The MPC OWNS the leg joint targets: it writes trot(phase) + planned
residual onto control.joint_target_q each tick (no controller phase-sync needed).
Mirrors the validated offline prototype (quad_mpc_offline.py).

Env (set by scripts/dev/run_quad_mpc_engine.ps1):
  MPC_LOCO_GAIT_MODULE  e.g. projects.policies.control.gait.omniquad_trot_gait
  MPC_LOCO_VX/FREQ/DUTY/STEP_H/BODY_H   gait params (match the deploy)
  MPC_LOCO_K/H/EVERY/SIGMA/RESMAX/LAM/WARM   planner knobs
  MPC_LOCO_YAW/WZ   per-robot heading weights (go2 ~12/4; omniquad/b2 0/0)
"""
import importlib
import math
import os

import numpy as np

NJ = 12


def _f(k, d):
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _build(self):
    if getattr(self, "_loco_ready", None) is not None:
        return self._loco_ready
    import mujoco as mj
    sol = self.solver
    mjm = getattr(sol, "mj_model", None)
    m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or m2nd is None:
        self._mpc_log("loco: solver lacks mj_model/mjc_jnt_to_newton_dof")
        self._loco_ready = False; return False
    m2nd = m2nd.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    # joint -> its position actuator (affine pos servo: biasprm[1] != 0)
    act_of = {}
    for a in range(int(mjm.nu)):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            j = int(mjm.actuator_trnid[a, 0])
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of:
                act_of[j] = a
    d = mj.MjData(mjm); mj.mj_forward(mjm, d)
    # classify the 12 leg hinges (controller order FL,FR,RL,RR x hip_x,hip_y,knee)
    label = {}
    for j in range(int(mjm.njnt)):
        if mjm.jnt_type[j] != mj.mjtJoint.mjJNT_HINGE or j not in act_of:
            continue
        ax = mjm.jnt_axis[j]; rng = mjm.jnt_range[j]; pos = d.xpos[mjm.jnt_bodyid[j]]
        leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else "FR" if pos[0] > 0
               else "RL" if pos[1] > 0 else "RR")
        jn = ("hip_x" if abs(ax[0]) > 0.5 else "knee" if rng[1] < 0.0 else "hip_y")
        label[(leg, jn)] = j
    dof, act = [], []
    try:
        for leg in ("FL", "FR", "RL", "RR"):
            for jn in ("hip_x", "hip_y", "knee"):
                j = label[(leg, jn)]
                dof.append(int(m2nd[j])); act.append(int(act_of[j]))
    except KeyError as e:
        self._mpc_log("loco: joint map incomplete %r labels=%s" % (e, sorted(label)))
        self._loco_ready = False; return False
    self._loco_dof = np.array(dof, np.int32)
    self._loco_act = np.array(act, np.int32)
    self._loco_sigma = _f("MPC_LOCO_SIGMA", 0.10) * np.ones(NJ)
    self._loco_nom = np.zeros(NJ)
    self._loco_rng = np.random.default_rng(0)
    self._loco_stg = importlib.import_module(os.environ["MPC_LOCO_GAIT_MODULE"])
    self._loco_gp = self._loco_stg.GaitParams(
        vx=_f("MPC_LOCO_VX", 0.4), freq=_f("MPC_LOCO_FREQ", 1.4),
        duty=_f("MPC_LOCO_DUTY", 0.6), step_height=_f("MPC_LOCO_STEP_H", 0.06),
        body_height=_f("MPC_LOCO_BODY_H", 0.55))
    self._loco_phase = self._loco_stg.QS_PHASE
    self._loco_omega = 2.0 * math.pi * self._loco_gp.freq
    self._loco_dt = float(getattr(self, "_n_substeps", 8)) * float(mjm.opt.timestep)
    self._loco_ctr = 0
    self._mpc_log("loco maps OK: dof=%s act=%s gait=%s dt=%.4f"
                  % (dof, act, os.environ["MPC_LOCO_GAIT_MODULE"], self._loco_dt))
    self._loco_ready = True
    return True


def _trot(self, ph):
    q, _ = self._loco_stg.targets_np(
        ph, self._loco_gp,
        t_since_start=max(0.0, (ph - self._loco_stg.QS_PHASE) / self._loco_omega))
    return np.asarray(q, np.float64)


# MPPI residual is sampled CONSTANT over the horizon, but the gait target
# trot(phase+h) changes every horizon step -> a constant-ctrl CUDA graph (what
# the stand MPC uses) can't express it. This kernel writes ctrl = gait[h] + delta
# from device buffers we refresh each plan, so the WHOLE H*sub rollout captures
# into ONE graph (~30ms/plan vs ~1.5s for the Python step-loop -> realtime).
_CTRL_KERNEL = None


def _ctrl_kernel():
    global _CTRL_KERNEL
    if _CTRL_KERNEL is None:
        import warp as wp

        @wp.kernel
        def _k(ctrl: wp.array2d(dtype=wp.float32),
               gait: wp.array2d(dtype=wp.float32),
               delta: wp.array2d(dtype=wp.float32),
               act: wp.array(dtype=wp.int32), h: int):
            i, j = wp.tid()
            ctrl[i, act[j]] = gait[h, j] + delta[i, j]
        _CTRL_KERNEL = _k
    return _CTRL_KERNEL


def _score(self, rd, K, nq, nv, delta, y0, resmax):
    gp = self._loco_gp
    vxt = gp.vx; zref = gp.body_height - 0.02; floor = 0.40 * gp.body_height / 0.55
    WVX, WUP, WRATE, WY, WH, WVZ, WRES, FALL = 10., 8., 3., 6., 120., 40., 0.2, 300.
    WYAW, WWZ = _f("MPC_LOCO_YAW", 0.0), _f("MPC_LOCO_WZ", 0.0)
    lam = _f("MPC_LOCO_LAM", 0.2)
    q = rd.qpos.numpy().reshape(K, nq); v = rd.qvel.numpy().reshape(K, nv)
    w_, x_, y_, z_ = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    pit = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    yaw = np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
    vx, vz, rr, pr, wz = v[:, 0], v[:, 2], v[:, 3], v[:, 4], v[:, 5]
    J = (WVX * (vx - vxt) ** 2 + WUP * (roll * roll + pit * pit)
         + WRATE * (rr * rr + pr * pr) + WYAW * yaw * yaw + WWZ * wz * wz
         + WY * (q[:, 1] - y0) ** 2 + WH * np.maximum(0, zref - q[:, 2]) ** 2
         + WVZ * np.maximum(0, -vz) ** 2 + WRES * (delta * delta).sum(1))
    J = J + ((q[:, 2] < floor) | (np.abs(roll) > 0.8) | (np.abs(pit) > 0.8)) * FALL
    wts = np.exp(-(J - J.min()) / lam); wts /= wts.sum() + 1e-9
    self._loco_nom = np.clip((wts[:, None] * delta).sum(0), -resmax, resmax)


def _plan(self, K, H):
    import os as _o
    import warp as wp
    buf = self._mpc_rollout_buffers(K)
    if buf is None:
        return
    self._roll_buf = buf
    _mjw, rm, rd, (nq, nv, nu) = buf
    sub = int(_f("MPC_LOCO_ROLL_SUB", getattr(self, "_n_substeps", 8)))
    resmax = _f("MPC_LOCO_RESMAX", 0.35)
    act = self._loco_act
    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype

    noise = self._loco_rng.normal(0, 1, (K, NJ)) * self._loco_sigma
    delta = np.clip(self._loco_nom[None, :] + noise, -resmax, resmax)
    delta[0] = self._loco_nom
    gait_np = np.stack([_trot(self, self._loco_phase + h * self._loco_omega * self._loco_dt)
                        for h in range(H)]).astype(np.float32)            # (H, NJ)

    base = self._mpc_seed_qv(K)                       # seeds rd.qpos/qvel from live
    y0 = rd.qpos.numpy().reshape(K, nq)[:, 1].copy()
    use_graph = (_o.environ.get("MPC_LOCO_NOGRAPH") is None
                 and getattr(self, "_loco_graph", None) is not False
                 and cdt == np.float32)

    if use_graph:
        try:
            dev = getattr(self.model, "device", None)
            if getattr(self, "_loco_gbuf", None) is None or self._loco_gK != (K, H):
                self._loco_gbuf = wp.zeros((H, NJ), dtype=wp.float32, device=dev)
                self._loco_dbuf = wp.zeros((K, NJ), dtype=wp.float32, device=dev)
                self._loco_actw = wp.array(np.asarray(act, np.int32), dtype=wp.int32, device=dev)
                self._loco_gK = (K, H); self._loco_graph = None
            ker = _ctrl_kernel()

            def _seed_ctrl():
                rd.ctrl.assign(np.broadcast_to(base, (K, nu)).astype(cdt).reshape(cshape))
            self._loco_gbuf.assign(gait_np)
            self._loco_dbuf.assign(delta.astype(np.float32))
            _seed_ctrl()
            if self._loco_graph is None:                  # capture ONCE
                wp.launch(ker, dim=(K, NJ), device=dev,
                          inputs=[rd.ctrl, self._loco_gbuf, self._loco_dbuf, self._loco_actw, 0])
                for _ in range(sub):
                    _mjw.step(rm, rd)
                wp.synchronize()
                with wp.ScopedDevice(dev):
                    wp.capture_begin(force_module_load=False)
                    try:
                        for h in range(H):
                            wp.launch(ker, dim=(K, NJ),
                                      inputs=[rd.ctrl, self._loco_gbuf, self._loco_dbuf, self._loco_actw, h])
                            for _ in range(sub):
                                _mjw.step(rm, rd)
                        self._loco_graph = wp.capture_end()
                    except Exception:
                        try:
                            wp.capture_end()
                        except Exception:
                            pass
                        raise
                self._mpc_log("loco rollout CUDA graph captured (K=%d H=%d sub=%d)" % (K, H, sub))
                self._mpc_seed_qv(K); _seed_ctrl()        # capture advanced rd -> reseed
            wp.capture_launch(self._loco_graph)
            wp.synchronize()
            _score(self, rd, K, nq, nv, delta, y0, resmax)
            return
        except Exception as e:
            self._mpc_log("loco graph path failed %r -> plain loop" % (e,))
            self._loco_graph = False

    # plain Python step-loop fallback (~50x slower; the original path)
    for h in range(H):
        c = np.broadcast_to(base, (K, nu)).copy()
        c[:, act] = gait_np[h][None, :] + delta
        rd.ctrl.assign(c.reshape(cshape).astype(cdt))
        for _ in range(sub):
            _mjw.step(rm, rd)
    try:
        wp.synchronize()
    except Exception:
        pass
    _score(self, rd, K, nq, nv, delta, y0, resmax)


def _apply(self):
    g = _trot(self, self._loco_phase)
    tp = self.control.joint_target_q.numpy()
    dof = self._loco_dof
    for i in range(NJ):
        di = int(dof[i])
        if 0 <= di < len(tp):
            tp[di] = float(g[i]) + float(self._loco_nom[i])
    self.control.joint_target_q.assign(tp)
    self._mjc_dirty = True


def loco_step(self):
    if not _build(self):
        return
    K = int(_f("MPC_LOCO_K", 64)); H = int(_f("MPC_LOCO_H", 16))
    every = max(1, int(_f("MPC_LOCO_EVERY", 2)))
    warm = int(_f("MPC_LOCO_WARM", 4))
    self._loco_phase += self._loco_omega * self._loco_dt
    if self._loco_ctr >= warm and (self._loco_ctr - warm) % every == 0:
        try:
            _plan(self, K, H)
        except Exception as e:
            self._mpc_log("loco plan error: %r" % (e,))
    self._loco_ctr += 1
    _apply(self)
