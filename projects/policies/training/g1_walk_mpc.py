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

"""In-engine REALTIME MPPI walk driver for the Unitree G1 (deterministic, no RL).

G1 sibling of h1_walk_mpc.py. Loaded via OMNISIM_INENGINE_PYMOD on a G1 stand world
(humanoid_stand_deploy settles the stand; this driver then OWNS the leg servo targets =
g1 human gait + a balance residual planned by rolling K futures in the engine's OWN
mujoco_warp, CUDA-graph captured). Intended for the BIGFOOT G1 (g1_walk_bigfoot.omniworld):
the offline study showed the small foot can't sustain the walk but the enlarged foot can.

Differences from the H1 driver: g1 gait; the G1 leg has SIX joints incl. ankle_roll, so
the two x-axis leg joints are split by height (top=hip_roll, bottom=ankle_roll); G1 fall
height (~0.55) and stand stiffness (ke 400).

Run: bash _scratch/stand/run_g1_walk.sh <dur> <tag> projects.policies.training.g1_walk_mpc:g1_walk_step [ENV=val ...]
Knobs: GWM_K/H/EVERY/WARM_TICKS, GWM_VX/FREQ/STEP_WIDTH, GWM_W_* (cost), GWM_SIGMA/RESMAX/BLEND/ZFALL.
"""
import math
import os
import sys

import numpy as np

_GAIT = "projects.policies.control.gait.g1_human_gait"


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
        world._mpc_log("g1walk: " + msg)
    except Exception:
        sys.stderr.write("[g1walk] " + msg + "\n")


def _build(world):
    if getattr(world, "_gwm_ready", None) is not None:
        return world._gwm_ready
    world._gwm_ready = False
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
    act_of = {}
    for a in range(int(mjm.nu)):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            j = int(mjm.actuator_trnid[a, 0])
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of:
                act_of[j] = a
    d = mj.MjData(mjm); mj.mj_forward(mjm, d)
    pelvis_z = 0.0; base_bid = -1
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            base_bid = int(mjm.jnt_bodyid[j]); pelvis_z = float(d.xpos[base_bid][2]); break
    # ROTATION-INVARIANT geometry (2026-07-08, direction specialists): classify leg joints in the
    # BASE BODY frame, not the world frame, so a robot spawned facing +y/-x still detects its legs.
    # jnt_axis is already body-local (rotation-invariant); only the POSITIONS were world-frame -> the
    # world-y left/right split and world-z below-pelvis test broke under a base yaw. At the
    # +x/identity default Rb=I and base xy=0, so pos_b == pos except a constant z-shift (base_z), which
    # the below-base test and the height sorts are invariant to -> BYTE-IDENTICAL for every existing walker.
    if base_bid >= 0:
        _Rb = np.array(d.xmat[base_bid], float).reshape(3, 3); _pb0 = np.array(d.xpos[base_bid], float)
    else:
        _Rb = np.eye(3); _pb0 = np.zeros(3)
    # classify leg joints below the pelvis by axis + height + y-sign -> g1 gait slot.
    # g1 leg = hip_pitch(y), hip_roll(x), hip_yaw(z), knee(y), ankle_pitch(y), ankle_roll(x)
    # slots (L base 0 / R base 6): hip_pitch 0, hip_roll 1, hip_yaw 2, knee 3, ankle_pitch 4, ankle_roll 5
    cand = []
    waist = None
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE) or j not in act_of:
            continue
        nd = int(m2nd[j]); bid = int(mjm.jnt_bodyid[j])
        pos = _Rb.T @ (np.array(d.xpos[bid], float) - _pb0)   # base-body frame (rotation-invariant)
        ax = np.abs(np.array(mjm.jnt_axis[j], float)); axd = int(np.argmax(ax))
        rec = (nd, act_of[j], int(mjm.jnt_qposadr[j]), pos, axd)
        if pos[2] < -0.02:            # below the base (== world pos_z < pelvis_z-0.02 at identity)
            cand.append(rec)
        elif axd == 2 and abs(pos[1]) < 0.05 and waist is None:
            waist = rec
    slotmap = {}
    foot_bid = {}
    for side, ys, base in (("left", 1.0, 0), ("right", -1.0, 6)):
        sj = [c for c in cand if c[3][1] * ys > 0]
        ydom = sorted([c for c in sj if c[4] == 1], key=lambda c: -c[3][2])   # y: hip_pitch>knee>ankle_pitch
        xdom = sorted([c for c in sj if c[4] == 0], key=lambda c: -c[3][2])   # x: hip_roll(top)>ankle_roll(bot)
        zdom = [c for c in sj if c[4] == 2]
        if len(ydom) < 3 or len(xdom) < 1 or len(zdom) < 1:
            _log(world, "leg %s incomplete y=%d x=%d z=%d" % (side, len(ydom), len(xdom), len(zdom)))
            return False
        slotmap[base + 0] = ydom[0][:3]      # hip_pitch
        slotmap[base + 3] = ydom[1][:3]      # knee
        slotmap[base + 4] = ydom[2][:3]      # ankle_pitch
        slotmap[base + 1] = xdom[0][:3]      # hip_roll (top x)
        if len(xdom) >= 2:
            slotmap[base + 5] = xdom[-1][:3]  # ankle_roll (bottom x)
        slotmap[base + 2] = zdom[0][:3]      # hip_yaw
        anklend = ydom[2][0]
        for j in range(int(mjm.njnt)):
            if int(m2nd[j]) == anklend:
                foot_bid[side] = int(mjm.jnt_bodyid[j]); break
    if waist is not None:
        slotmap[12] = waist[:3]
    slots = sorted(slotmap)
    world._gwm_slots = slots
    world._gwm_dof = np.array([slotmap[s][0] for s in slots], np.int32)
    world._gwm_act = np.array([slotmap[s][1] for s in slots], np.int32)
    world._gwm_qadr = np.array([slotmap[s][2] for s in slots], np.int32)
    world._gwm_n = len(slots)
    world._gwm_foot = foot_bid
    import importlib
    stg = importlib.import_module(_GAIT)
    try:
        gp = stg.GaitParams(vx=_f("GWM_VX", 0.12), freq=_f("GWM_FREQ", 0.80),
                            style=os.environ.get("GWM_STYLE", "ik"),
                            lateral=os.environ.get("GWM_LAT", "lipm"), ramp_s=_f("GWM_RAMP", 2.0))
    except TypeError:
        gp = stg.GaitParams(vx=_f("GWM_VX", 0.12), freq=_f("GWM_FREQ", 0.80))
    sw = _f("GWM_STEP_WIDTH", 0.16)
    if sw > 0 and hasattr(gp, "step_width"):
        gp.step_width = sw
    world._gwm_stg = stg; world._gwm_gp = gp
    world._gwm_omega = 2.0 * math.pi * gp.freq
    world._gwm_dt = float(getattr(world, "_n_substeps", 4)) * float(mjm.opt.timestep)
    world._gwm_phase0 = float(stg.DS_PHASE)
    world._gwm_zfall = _f("GWM_ZFALL", 0.55 * (gp.pelvis_height / 0.755))
    world._gwm_sigma = _f("GWM_SIGMA", 0.05)
    world._gwm_resmax = _f("GWM_RESMAX", 0.18)
    world._gwm_nom = np.zeros(world._gwm_n)
    world._gwm_rng = np.random.default_rng(0)
    world._gwm_ctr = 0
    world._gwm_started = None
    world._gwm_graph = None; world._gwm_graph_failed = False; world._gwm_ctrlbuf = None
    world._gwm_ready = True
    _log(world, "ready n=%d slots=%s dof=%s dt=%.4f pelvis_z=%.2f zfall=%.2f vx=%.2f freq=%.2f sw=%.2f"
         % (world._gwm_n, slots, list(world._gwm_dof), world._gwm_dt, pelvis_z,
            world._gwm_zfall, gp.vx, gp.freq, getattr(gp, "step_width", 0.0)))
    return True


def _gaitvec(world, phase, tss):
    stg = world._gwm_stg; gp = world._gwm_gp
    legs, _arms, _sw = stg.targets_np(phase, gp, t_since_start=tss)
    legs = np.asarray(legs, np.float64)
    return np.array([legs[s] for s in world._gwm_slots])


def _plan(world, K, H):
    import warp as wp
    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        _log(world, "no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    sub = int(getattr(world, "_n_substeps", 4))
    n = world._gwm_n; act = world._gwm_act
    dt = world._gwm_dt; omega = world._gwm_omega
    tss = max(0.0, (world._gwm_ctr - (world._gwm_started or 0)) * dt)
    phase = world._gwm_phase + 0.0
    base = world._mpc_seed_qv(K)
    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype
    q0 = rd.qpos.numpy().reshape(K, nq)
    zref = _f("GWM_ZREF", 0.0) or float(q0[0, 2])
    noise = world._gwm_rng.normal(0, 1, (K, n)) * world._gwm_sigma
    delta = np.clip(world._gwm_nom[None, :] + noise, -world._gwm_resmax, world._gwm_resmax)
    delta[0] = world._gwm_nom
    full = np.broadcast_to(base, (H, K, nu)).copy()
    for h in range(H):
        gv = _gaitvec(world, phase + h * omega * dt, tss + h * dt)
        full[h][:, act] = gv[None, :] + delta
        full[h][:, act + 1] = 0.0
    if world._gwm_ctrlbuf is None:
        world._gwm_ctrlbuf = wp.zeros((H,) + cshape, dtype=rd.ctrl.dtype, device=rd.ctrl.device)
    cbuf = world._gwm_ctrlbuf
    cbuf.assign(full.reshape((H,) + cshape).astype(cdt))

    def _roll():
        for h in range(H):
            wp.copy(rd.ctrl, cbuf[h])
            for _ in range(sub):
                _mjw.step(rm, rd)
    g = world._gwm_graph
    if g is None and not world._gwm_graph_failed:
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
                    world._gwm_graph = wp.capture_end()
            g = world._gwm_graph
            _log(world, "rollout CUDA graph captured (K=%d H=%d sub=%d horizon=%.2fs)" % (K, H, sub, H * dt))
        except Exception as e:
            world._gwm_graph_failed = True; world._gwm_graph = None; g = None
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
    vxt = world._gwm_gp.vx
    WVX = _f("GWM_W_VX", 8.0); WUP = _f("GWM_W_UP", 30.0); WH = _f("GWM_W_H", 150.0)
    WRATE = _f("GWM_W_RATE", 9.0); WY = _f("GWM_W_Y", 75.0); WVZ = _f("GWM_W_VZ", 40.0)
    WOVER = _f("GWM_W_OVER", 40.0); WRES = _f("GWM_W_RES", 0.2); FALL = _f("GWM_W_FALL", 300.0)
    WYAW = _f("GWM_W_YAW", 13.0)
    yaw = np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
    J = (WVX * (vx - vxt) ** 2 + WUP * (roll * roll + pit * pit)
         + WRATE * (wx * wx + wy * wy) + WY * q[:, 1] ** 2 + WYAW * yaw * yaw
         + WH * np.maximum(0.0, zref - bz) ** 2 + WVZ * np.maximum(0.0, -vz) ** 2
         + WOVER * (np.maximum(0.0, vx - vxt) ** 2 + np.maximum(0.0, pit) ** 2)
         + WRES * (delta * delta).sum(1))
    J += ((bz < world._gwm_zfall) | (np.abs(roll) > 0.7) | (np.abs(pit) > 0.7)) * FALL
    lam = _f("GWM_LAM", 0.1)
    wts = np.exp(-(J - J.min()) / lam); wts /= wts.sum() + 1e-9
    world._gwm_nom = np.clip((wts[:, None] * delta).sum(0), -world._gwm_resmax, world._gwm_resmax)


def _apply(world):
    tp = world.control.joint_target_q.numpy()
    dt = world._gwm_dt
    tss = max(0.0, (world._gwm_ctr - (world._gwm_started or 0)) * dt)
    blend_s = _f("GWM_BLEND", 1.0)
    blend = min(1.0, tss / max(1e-3, blend_s))
    gv = _gaitvec(world, world._gwm_phase, tss)
    nom = world._gwm_nom
    for i in range(world._gwm_n):
        di = int(world._gwm_dof[i])
        if 0 <= di < len(tp):
            tgt = float(gv[i]) + float(nom[i])
            tp[di] = (1.0 - blend) * float(tp[di]) + blend * tgt
    world.control.joint_target_q.assign(tp)
    world._mjc_dirty = True
    if blend >= 1.0:
        world._gwm_phase += world._gwm_omega * dt


def g1_walk_step(world):
    if not _build(world):
        return
    warm = _i("GWM_WARM_TICKS", 380)
    world._gwm_ctr += 1
    c = world._gwm_ctr
    if c < warm:
        return
    if world._gwm_started is None:
        world._gwm_started = c
        world._gwm_phase = world._gwm_phase0
        _log(world, "walk start @tick %d (warm done)" % c)
    K = _i("GWM_K", 64); H = _i("GWM_H", 20); every = max(1, _i("GWM_EVERY", 2))
    try:
        if (c - world._gwm_started) % every == 0:
            import time as _t
            _t0 = _t.perf_counter()
            _plan(world, K, H)
            world._gwm_plan_ms = 0.9 * getattr(world, "_gwm_plan_ms", 0.0) + 0.1 * (_t.perf_counter() - _t0) * 1e3
        _apply(world)
        if (c - world._gwm_started) % 125 == 1:
            live = world.solver.mjw_data
            q = live.qpos.numpy().reshape(-1)
            w_, x_, y_, z_ = q[3], q[4], q[5], q[6]
            roll = math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
            pit = math.asin(max(-1.0, min(1.0, 2 * (w_ * y_ - z_ * x_))))
            rt = (every * world._gwm_dt * 1e3) / max(0.01, getattr(world, "_gwm_plan_ms", 1.0))
            _log(world, "t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f |res|=%.2f plan=%.0fms rt~%.2fx"
                 % (c, q[0], q[1], q[2], roll, pit, float(np.abs(world._gwm_nom).max()),
                    getattr(world, "_gwm_plan_ms", 0.0), rt))
    except Exception as e:
        _log(world, "step err %r" % (e,))
