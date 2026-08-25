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

"""In-engine REALTIME MPPI walk driver for the Unitree H1 (deterministic, no RL).

Loaded via OMNISIM_INENGINE_PYMOD on h1_stand_deploy.omniworld (the humanoid_stand_deploy
controller settles H1 standing; this driver then OWNS the leg servo targets = the human
gait + a balance residual). The residual is planned by rolling out K futures in the
engine's OWN mujoco_warp (zero sim-to-deploy gap), CUDA-graph-captured with an (H,K,nu)
ctrl buffer that advances the gait over the horizon (so the rollout SEES the upcoming
swing) -- the wp.copy-per-step trick proven by g1_step_mpc._plan_seg. That keeps it ~realtime
(no per-step Python/launch overhead), unlike the offline Python-loop prototype (~0.1x).

This is the realtime, live-in-OmniSim realization of humanoid_walk_offline.py's H1 walk.

Run: _scratch/stand/run_h1_walk.sh <dur> <tag> projects.policies.research.mpc.h1_walk_mpc:h1_walk_step [ENV=val ...]
Knobs: HWM_K/H/EVERY/WARM_TICKS, HWM_VX/FREQ, HWM_W_* (cost), HWM_SIGMA/RESMAX, HWM_BLEND.
"""
import math
import os
import sys

import numpy as np


def _f(k, d):
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _i(k, d):
    return int(_f(k, d))


def _log(world, msg):
    try:
        world._mpc_log("h1walk: " + msg)
    except Exception:
        sys.stderr.write("[h1walk] " + msg + "\n")


# gait 13-slot leg+waist layout: hip_pitch,hip_roll,hip_yaw,knee,ankle_pitch,ankle_roll
# (L 0-5, R 6-11), waist 12. H1 has no ankle_roll -> slots 5,11 unused.
_GAIT = "projects.policies.control.gait.h1_human_gait"


def _build(world):
    if getattr(world, "_hwm_ready", None) is not None:
        return world._hwm_ready
    world._hwm_ready = False
    try:
        import mujoco as mj
    except Exception as e:
        _log(world, "mujoco import failed %r" % (e,)); return False
    sol = world.solver
    mjm = getattr(sol, "mj_model", None)
    m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or m2nd is None:
        _log(world, "no mj_model/jntmap (ODE?)"); return False
    m2nd = m2nd.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    # position actuator per joint (affine servo); vel actuator is the next index (interleaved)
    act_of = {}
    for a in range(int(mjm.nu)):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            j = int(mjm.actuator_trnid[a, 0])
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of:
                act_of[j] = a
    d = mj.MjData(mjm); mj.mj_forward(mjm, d)
    pelvis_z = 0.0
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            pelvis_z = float(d.xpos[int(mjm.jnt_bodyid[j])][2]); break
    # classify leg joints (body below pelvis) by axis + height + y-sign -> gait slot.
    # H1 leg = hip_yaw(z), hip_roll(x), hip_pitch/knee/ankle(y, top->bottom). slots:
    #   hip_pitch=0/6, hip_roll=1/7, hip_yaw=2/8, knee=3/9, ankle_pitch=4/10. waist=12.
    cand = []
    waist = None
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE) or j not in act_of:
            continue
        nd = int(m2nd[j]); bid = int(mjm.jnt_bodyid[j]); pos = np.array(d.xpos[bid], float)
        ax = np.abs(np.array(mjm.jnt_axis[j], float)); axd = int(np.argmax(ax))
        rec = (nd, act_of[j], int(mjm.jnt_qposadr[j]), pos, axd)
        if pos[2] < pelvis_z - 0.02:
            cand.append(rec)
        elif axd == 2 and abs(pos[1]) < 0.05 and waist is None:   # central z-axis = waist
            waist = rec
    slotmap = {}                     # gait slot -> (newton_dof, act, qadr)
    foot_bid = {}
    for side, ys, base in (("left", 1.0, 0), ("right", -1.0, 6)):
        sj = [c for c in cand if c[3][1] * ys > 0]
        ydom = sorted([c for c in sj if c[4] == 1], key=lambda c: -c[3][2])  # y-axis top->bottom
        xdom = [c for c in sj if c[4] == 0]
        zdom = [c for c in sj if c[4] == 2]
        if len(ydom) < 3 or len(xdom) < 1 or len(zdom) < 1:
            _log(world, "leg %s incomplete y=%d x=%d z=%d" % (side, len(ydom), len(xdom), len(zdom)))
            return False
        slotmap[base + 0] = ydom[0][:3]      # hip_pitch (top y)
        slotmap[base + 3] = ydom[1][:3]      # knee (mid y)
        slotmap[base + 4] = ydom[2][:3]      # ankle_pitch (bottom y)
        slotmap[base + 1] = xdom[0][:3]      # hip_roll (x)
        slotmap[base + 2] = zdom[0][:3]      # hip_yaw (z)
        # foot body = ankle joint's child
        anklend = ydom[2][0]
        for j in range(int(mjm.njnt)):
            if int(m2nd[j]) == anklend:
                foot_bid[side] = int(mjm.jnt_bodyid[j]); break
    if waist is not None:
        slotmap[12] = waist[:3]
    # control order: the mapped slots, sorted by slot index (so it matches gait legs[slot])
    slots = sorted(slotmap)
    world._hwm_slots = slots
    world._hwm_dof = np.array([slotmap[s][0] for s in slots], np.int32)
    world._hwm_act = np.array([slotmap[s][1] for s in slots], np.int32)
    world._hwm_qadr = np.array([slotmap[s][2] for s in slots], np.int32)
    world._hwm_n = len(slots)
    world._hwm_foot = foot_bid
    world._hwm_mj = mj; world._hwm_nq = int(mjm.nq); world._hwm_nv = int(mjm.nv)
    # gait
    import importlib
    stg = importlib.import_module(_GAIT)
    try:
        gp = stg.GaitParams(vx=_f("HWM_VX", 0.14), freq=_f("HWM_FREQ", 0.85),
                            style=os.environ.get("HWM_STYLE", "ik"),
                            lateral=os.environ.get("HWM_LAT", "lipm"), ramp_s=_f("HWM_RAMP", 2.0))
    except TypeError:
        gp = stg.GaitParams(vx=_f("HWM_VX", 0.14), freq=_f("HWM_FREQ", 0.85))
    sw = _f("HWM_STEP_WIDTH", 0.0)
    if sw > 0 and hasattr(gp, "step_width"):
        gp.step_width = sw
    world._hwm_stg = stg; world._hwm_gp = gp
    world._hwm_omega = 2.0 * math.pi * gp.freq
    world._hwm_dt = float(getattr(world, "_n_substeps", 4)) * float(mjm.opt.timestep)
    world._hwm_phase0 = float(stg.DS_PHASE)
    # residual scale
    world._hwm_sigma = _f("HWM_SIGMA", 0.05)
    world._hwm_resmax = _f("HWM_RESMAX", 0.18)
    world._hwm_nom = np.zeros(world._hwm_n)
    world._hwm_rng = np.random.default_rng(0)
    world._hwm_ctr = 0
    world._hwm_started = None
    world._hwm_graph = None; world._hwm_graph_failed = False; world._hwm_ctrlbuf = None
    world._hwm_ready = True
    _log(world, "ready n=%d slots=%s dof=%s dt=%.4f pelvis_z=%.2f vx=%.2f freq=%.2f"
         % (world._hwm_n, slots, list(world._hwm_dof), world._hwm_dt, pelvis_z, gp.vx, gp.freq))
    return True


def _gaitvec(world, phase, tss):
    """Gait targets for the mapped slots (control order) at the gait clock phase."""
    stg = world._hwm_stg; gp = world._hwm_gp
    legs, _arms, _sw = stg.targets_np(phase, gp, t_since_start=tss)
    legs = np.asarray(legs, np.float64)
    return np.array([legs[s] for s in world._hwm_slots])


def _plan(world, K, H):
    import warp as wp
    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        _log(world, "no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    sub = int(getattr(world, "_n_substeps", 4))
    n = world._hwm_n; act = world._hwm_act
    dt = world._hwm_dt; omega = world._hwm_omega
    tss = max(0.0, (world._hwm_ctr - (world._hwm_started or 0)) * dt)
    phase = world._hwm_phase + 0.0
    base = world._mpc_seed_qv(K)              # seed rd.qpos/qvel from live; nominal ctrl
    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype
    q0 = rd.qpos.numpy().reshape(K, nq)
    zref = _f("HWM_ZREF", 0.0) or float(q0[0, 2])
    # sample residual (K,n); candidate 0 = current plan
    noise = world._hwm_rng.normal(0, 1, (K, n)) * world._hwm_sigma
    delta = np.clip(world._hwm_nom[None, :] + noise, -world._hwm_resmax, world._hwm_resmax)
    delta[0] = world._hwm_nom
    # (H,K,nu) ctrl buffer: arms/waist hold the nominal base; leg slots = gait[h] + delta
    full = np.broadcast_to(base, (H, K, nu)).copy()
    for h in range(H):
        gv = _gaitvec(world, phase + h * omega * dt, tss + h * dt)   # (n,)
        full[h][:, act] = gv[None, :] + delta                        # (K,n)
        full[h][:, act + 1] = 0.0                                    # vel actuator target 0
    if world._hwm_ctrlbuf is None:
        world._hwm_ctrlbuf = wp.zeros((H,) + cshape, dtype=rd.ctrl.dtype, device=rd.ctrl.device)
    cbuf = world._hwm_ctrlbuf
    cbuf.assign(full.reshape((H,) + cshape).astype(cdt))

    def _roll():
        for h in range(H):
            wp.copy(rd.ctrl, cbuf[h])
            for _ in range(sub):
                _mjw.step(rm, rd)
    g = world._hwm_graph
    if g is None and not world._hwm_graph_failed:
        try:
            dev = world.model.device
            if "cuda" not in str(dev).lower():
                raise RuntimeError("rollout not on cuda")
            with wp.ScopedDevice(dev):
                wp.synchronize()
                wp.capture_begin(force_module_load=False)
                try:
                    _roll()
                finally:
                    world._hwm_graph = wp.capture_end()
            g = world._hwm_graph
            _log(world, "rollout CUDA graph captured (K=%d H=%d sub=%d horizon=%.2fs)"
                 % (K, H, sub, H * dt))
        except Exception as e:
            world._hwm_graph_failed = True; world._hwm_graph = None; g = None
            _log(world, "graph capture failed (%r) -> python loop" % (e,))
    try:
        if g is not None:
            wp.capture_launch(g)
        else:
            _roll()
        wp.synchronize()
    except Exception as e:
        _log(world, "rollout launch err %r" % (e,)); return
    q = rd.qpos.numpy().reshape(K, nq); v = rd.qvel.numpy().reshape(K, nv)
    w_, x_, y_, z_ = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    pit = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    bz = q[:, 2]; vx, vy = v[:, 0], v[:, 1]; vz = v[:, 2]; wx, wy = v[:, 3], v[:, 4]
    vxt = world._hwm_gp.vx
    WVX = _f("HWM_W_VX", 8.0); WUP = _f("HWM_W_UP", 30.0); WH = _f("HWM_W_H", 150.0)
    WRATE = _f("HWM_W_RATE", 6.0); WY = _f("HWM_W_Y", 60.0); WVZ = _f("HWM_W_VZ", 40.0)
    WOVER = _f("HWM_W_OVER", 40.0); WRES = _f("HWM_W_RES", 0.2); FALL = _f("HWM_W_FALL", 300.0)
    J = (WVX * (vx - vxt) ** 2 + WUP * (roll * roll + pit * pit)
         + WRATE * (wx * wx + wy * wy) + WY * q[:, 1] ** 2
         + WH * np.maximum(0.0, zref - bz) ** 2 + WVZ * np.maximum(0.0, -vz) ** 2
         + WOVER * (np.maximum(0.0, vx - vxt) ** 2 + np.maximum(0.0, pit) ** 2)
         + WRES * (delta * delta).sum(1))
    J += ((bz < 0.72) | (np.abs(roll) > 0.7) | (np.abs(pit) > 0.7)) * FALL
    lam = _f("HWM_LAM", 0.1)
    wts = np.exp(-(J - J.min()) / lam); wts /= wts.sum() + 1e-9
    world._hwm_nom = np.clip((wts[:, None] * delta).sum(0), -world._hwm_resmax, world._hwm_resmax)


def _apply(world):
    tp = world.control.joint_target_q.numpy()
    dt = world._hwm_dt
    tss = max(0.0, (world._hwm_ctr - (world._hwm_started or 0)) * dt)
    blend_s = _f("HWM_BLEND", 1.0)
    blend = min(1.0, tss / max(1e-3, blend_s))   # ease into the gait STANDING pose first
    gv = _gaitvec(world, world._hwm_phase, tss)
    nom = world._hwm_nom
    for i in range(world._hwm_n):
        di = int(world._hwm_dof[i])
        if 0 <= di < len(tp):
            tgt = float(gv[i]) + float(nom[i])
            tp[di] = (1.0 - blend) * float(tp[di]) + blend * tgt
    world.control.joint_target_q.assign(tp)
    world._mjc_dirty = True
    # FREEZE the gait clock during the pose blend-in (stay in double-support at DS_PHASE);
    # only start advancing -> stepping once H1 has eased into the gait's standing squat.
    if blend >= 1.0:
        world._hwm_phase += world._hwm_omega * dt


def h1_walk_step(world):
    if not _build(world):
        return
    warm = _i("HWM_WARM_TICKS", 380)
    world._hwm_ctr += 1
    c = world._hwm_ctr
    if c < warm:
        return                                # let humanoid_stand_deploy settle H1 standing
    if world._hwm_started is None:
        world._hwm_started = c
        world._hwm_phase = world._hwm_phase0
        _log(world, "walk start @tick %d (warm done)" % c)
    K = _i("HWM_K", 64); H = _i("HWM_H", 20); every = max(1, _i("HWM_EVERY", 2))
    try:
        if (c - world._hwm_started) % every == 0:
            import time as _t
            _t0 = _t.perf_counter()
            _plan(world, K, H)
            world._hwm_plan_ms = 0.9 * getattr(world, "_hwm_plan_ms", 0.0) + 0.1 * (_t.perf_counter() - _t0) * 1e3
        _apply(world)
        if (c - world._hwm_started) % 125 == 1:
            live = world.solver.mjw_data
            q = live.qpos.numpy().reshape(-1)
            w_, x_, y_, z_ = q[3], q[4], q[5], q[6]
            roll = math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
            pit = math.asin(max(-1.0, min(1.0, 2 * (w_ * y_ - z_ * x_))))
            # realtime factor = sim-time-per-plan / wall-time-per-plan; tick=dt, replan every `every`
            rt = (every * world._hwm_dt * 1e3) / max(0.01, getattr(world, "_hwm_plan_ms", 1.0))
            _log(world, "t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f |res|=%.2f plan=%.0fms rt~%.2fx"
                 % (c, q[0], q[1], q[2], roll, pit, float(np.abs(world._hwm_nom).max()),
                    getattr(world, "_hwm_plan_ms", 0.0), rt))
    except Exception as e:
        _log(world, "step err %r" % (e,))
