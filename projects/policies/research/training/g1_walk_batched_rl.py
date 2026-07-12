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

"""BATCHED in-engine residual RL for the G1 walk -- the ~10x accelerator.

Same idea as g1_walk_residual_inengine.py (residual on the deterministic gait, trained in
OmniSim's OWN Newton/mujoco_warp so it transfers), but instead of evaluating the ES
population ONE episode at a time on the live world, it evaluates the WHOLE population as
K = 2*pop PARALLEL mujoco_warp worlds via the engine's batched rollout buffers
(world._mpc_rollout_buffers) -- the GPU steps 64 robots ~as fast as 1. Zero deploy gap
(same solver + same model, just replicated); ~10x faster per member than sequential.

The hook (OMNISIM_INENGINE_PYMOD) waits for the stand to settle, captures the standing
spawn qpos, then runs the ENTIRE batched ES loop in-process (the live world pauses; we step
the rollout buffers). Writes the evolving policy to RES_POLICY each generation. Deploy uses
g1_walk_residual_inengine.py in mode=deploy (same gait + same linear residual).

Knobs: RES_POLICY RES_POP(24) RES_SIGMA(0.05) RES_LR(0.03) RES_ACT_SCALE(0.22) RES_SEED
  WALK_VX WALK_FREQ WALK_STEP_WIDTH WALK_EP_S(5) WALK_BLEND_S(0.5) WALK_WARM_TICKS(360)
  W_VX(8) W_ALIVE(0.7) W_LAT(8) W_UP(4) W_YAW(3) RES_ACT_PEN(0.01)
  FALL_PITCH(0.7) FALL_ROLL(0.6) FALL_BZ(0.5) BATCH_MAXGEN(100) RES_LOG.
"""
import math
import os
import sys

import numpy as np

_RT = os.environ.get("OMNISIM_HOME", ".")
if _RT not in sys.path:
    sys.path.insert(0, _RT)

# reuse the walk gait + leg/actuator map, and the stand module's logger/obs helpers
from projects.policies.training import g1_walk_mpc as WM
from projects.policies.training import g1_residual_inengine as R

OBS_DIM = 11      # proj_g(3)+ang_vel(3)+lin_vel(3)+phase(sin,cos)(2)
N_LEG = 12


def _f(k, d):
    return R._f(k, d)


def _i(k, d):
    return R._i(k, d)


# policy: linear (RES_HIDDEN=0) or 1-hidden-layer tanh MLP (RES_HIDDEN=H).
def _hidden():
    return _i("RES_HIDDEN", 0)


def _nparam():
    h = _hidden()
    if h <= 0:
        return N_LEG * OBS_DIM + N_LEG
    return OBS_DIM * h + h + h * N_LEG + N_LEG     # W1,b1,W2,b2


N_PARAM = _nparam()


def _save(path, theta, meta=None):
    if not path:
        return
    d = {"theta": theta.astype(np.float32)}
    if meta is not None:
        d["meta"] = np.array(meta, dtype=np.float32)
    try:
        np.savez(path, **d)
    except Exception:
        pass


def _load(path):
    if path and os.path.exists(path):
        try:
            return np.load(path)["theta"].astype(np.float64)
        except Exception:
            pass
    return np.zeros(N_PARAM)


def _obs_batched(qp, qv, phase, K):
    """qp:(K,nq) qv:(K,nv) -> obs:(K,11), roll:(K,), pitch:(K,), bz:(K,)."""
    w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    ang = qv[:, 3:6]
    lin = qv[:, 0:3]
    s, c = math.sin(phase), math.cos(phase)
    obs = np.column_stack([gx, gy, gz, ang[:, 0], ang[:, 1], ang[:, 2],
                           lin[:, 0], lin[:, 1], lin[:, 2],
                           np.full(K, s), np.full(K, c)])
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return obs, roll, pitch, qp[:, 2], yaw, lin[:, 0], lin[:, 1]


def _policy_batched(members, obs):
    """members:(K,N_PARAM) obs:(K,11) -> res:(K,12) tanh-bounded. Linear or 1-hidden MLP."""
    K = members.shape[0]
    h = _hidden()
    if h <= 0:
        W = members[:, : N_LEG * OBS_DIM].reshape(K, N_LEG, OBS_DIM)
        b = members[:, N_LEG * OBS_DIM:]
        return np.tanh(np.einsum("kij,kj->ki", W, obs) + b)
    o = 0
    W1 = members[:, o:o + OBS_DIM * h].reshape(K, h, OBS_DIM); o += OBS_DIM * h
    b1 = members[:, o:o + h]; o += h
    W2 = members[:, o:o + h * N_LEG].reshape(K, N_LEG, h); o += h * N_LEG
    b2 = members[:, o:o + N_LEG]
    hid = np.tanh(np.einsum("khj,kj->kh", W1, obs) + b1)
    return np.tanh(np.einsum("kih,kh->ki", W2, hid) + b2)


def _batched_train(world):
    import warp as wp
    if not WM._build(world):
        R._log(world, "batch: WM._build failed"); return
    gp = world._gwm_gp
    slots = world._gwm_slots          # gait slots present (0..12)
    act = world._gwm_act              # actuator idx per slot
    dt = world._gwm_dt
    omega = world._gwm_omega
    sub = int(getattr(world, "_n_substeps", 4))
    # which slots are legs (0..11) -> residual indices; map residual i (0..11) to slot pos
    leg_pos = [p for p, s in enumerate(slots) if s < 12]      # positions into act[]
    leg_slot = [slots[p] for p in leg_pos]                    # gait-slot id for each leg actuator
    # residual vector is indexed by gait-slot 0..11; build slot->residual-index
    res_idx_for_pos = [leg_slot_i for leg_slot_i in leg_slot]  # slot id == residual index

    pop = _i("RES_POP", 24)
    K = 2 * pop
    act_scale = _f("RES_ACT_SCALE", 0.22)
    sigma = _f("RES_SIGMA", 0.05); lr = _f("RES_LR", 0.03)
    rng = np.random.default_rng(_i("RES_SEED", 0))
    path = os.environ.get("RES_POLICY", "")
    theta = _load(path)
    if theta.shape[0] != N_PARAM:
        theta = np.zeros(N_PARAM)
    eplen = max(8, int(_f("WALK_EP_S", 5.0) / dt))
    blend_ticks = max(1, int(_f("WALK_BLEND_S", 0.5) / dt))
    phase0 = world._gwm_phase0
    fr = _f("FALL_ROLL", 0.6); fp = _f("FALL_PITCH", 0.7); fbz = _f("FALL_BZ", 0.5)
    wvx = _f("W_VX", 8.0); walive = _f("W_ALIVE", 0.7); wlat = _f("W_LAT", 8.0)
    wup = _f("W_UP", 4.0); wyaw = _f("W_YAW", 3.0); apen = _f("RES_ACT_PEN", 0.01)
    vxt = gp.vx
    maxgen = _i("BATCH_MAXGEN", 100)

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "batch: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    # spawn = the live standing qpos (captured after settle); broadcast to all K worlds
    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    R._log(world, "BATCH train: K=%d pop=%d eplen=%d N_PARAM=%d act_scale=%.2f spawn_z=%.3f"
           % (K, pop, eplen, N_PARAM, act_scale, spawn[2]))

    # precompute gait leg targets per tick once (shared across worlds) up to eplen
    def gaitvec(phase, tss):
        legs, _a, _s = world._gwm_stg.targets_np(phase, gp, t_since_start=tss)
        return np.asarray(legs, np.float64)

    cnp = rd.ctrl.numpy()
    cshape = cnp.shape; cdt = cnp.dtype
    nq_buf = rd.qpos.numpy().shape  # (K*nq,) or (K,nq)

    for gen in range(maxgen):
        eps = rng.standard_normal((pop, N_PARAM))
        members = np.empty((K, N_PARAM))
        members[0::2] = theta + sigma * eps
        members[1::2] = theta - sigma * eps
        # seed all worlds at spawn + PER-WORLD IC RANDOMIZATION (the key robustness lever):
        # the deterministic single-spawn made policies MEMORIZE one trajectory and overfit;
        # randomizing each world's start (joint angles + base pose/velocity) forces the policy
        # to GENERALIZE -> robust to the deploy's slightly-different start (no more 1.5s fall).
        q = np.broadcast_to(spawn, (K, nq)).astype(np.float32).copy()
        qv0 = np.zeros((K, nv), np.float32)
        # PER-GENERATION IC (SAME for all members this gen, VARIES across gens): fair ES
        # comparison (members differ only by policy, not by start luck) + IC diversity over
        # training -> the policy learns a robust walk, not one memorized trajectory.
        icj = _f("IC_RAND_JOINT", 0.04); icv = _f("IC_RAND_VEL", 0.15); ica = _f("IC_RAND_ANG", 0.06)
        if icj > 0:
            q[:, 7:] += rng.normal(0, icj, (nq - 7)).astype(q.dtype)[None, :]   # joint angles
        if ica > 0:
            q[:, 4:7] += rng.normal(0, ica, (3,)).astype(q.dtype)[None, :]      # base orient
        if icv > 0:
            qv0[:, 0:2] = rng.normal(0, icv, (2,)).astype(np.float32)[None, :]  # base horiz vel
            qv0[:, 3:5] = rng.normal(0, icv, (2,)).astype(np.float32)[None, :]  # base roll/pitch rate
        rd.qpos.assign(q.reshape(rd.qpos.numpy().shape))
        rd.qvel.assign(qv0.reshape(rd.qvel.numpy().shape))
        _mjw.forward(rm, rd)
        phase = phase0
        # CONTACT WARM-UP: hold the captured SPAWN pose (the settled stand the robot is
        # actually in -- NOT the deeper gait pose, which sags) for a few ticks so the foot
        # contacts settle (efc warmstart, penetration) to a WARM state matching the live deploy
        # (which warms via the stand controller before the gait). Closes the fresh-vs-warm gap.
        nset = _i("WALK_SETTLE_TICKS", 0)   # off by default: the episode blend-in already warms contacts
        if nset > 0:
            qadr = world._gwm_qadr
            c0 = rd.ctrl.numpy().reshape(K, nu)
            for p, slot in enumerate(slots):
                a = int(act[p]); c0[:, a] = float(spawn[int(qadr[p])]); c0[:, a + 1] = 0.0
            rd.ctrl.assign(c0.reshape(cshape).astype(cdt))
            for _ in range(nset):
                for _ in range(sub):
                    _mjw.step(rm, rd)
        fit = np.zeros(K); alive = np.ones(K, bool)
        # DR: random horizontal base-velocity kicks -> the policy must learn to RECOVER,
        # giving margin to survive the live deploy's chaotic divergence (robustness, not parity).
        dr_push = _f("DR_PUSH", 0.0)
        dr_every = max(1, int(_f("DR_EVERY_S", 1.2) / dt))
        for t in range(eplen):
            if dr_push > 0 and t > 0 and t % dr_every == 0:
                qv = rd.qvel.numpy().reshape(K, nv)
                ang = rng.uniform(0, 2 * math.pi, K); mag = rng.uniform(0.3, 1.0, K) * dr_push
                qv[:, 0] += (mag * np.cos(ang)).astype(qv.dtype)
                qv[:, 1] += (mag * np.sin(ang)).astype(qv.dtype)
                rd.qvel.assign(qv.reshape(rd.qvel.numpy().shape))
            qp = rd.qpos.numpy().reshape(K, nq); qv = rd.qvel.numpy().reshape(K, nv)
            obs, roll, pitch, bz, yaw, vx, vy = _obs_batched(qp, qv, phase, K)
            res = act_scale * _policy_batched(members, obs)    # (K,12)
            blend = min(1.0, (t + 1) / blend_ticks)
            tss = max(0.0, (t - blend_ticks) * dt)
            gleg = gaitvec(phase, tss)                         # (>=13,)
            ctrl = rd.ctrl.numpy().reshape(K, nu)
            for p, slot in zip(leg_pos, leg_slot):
                a = int(act[p])
                ctrl[:, a] = gleg[slot] + blend * res[:, slot] * alive   # residual only while alive/blended
                ctrl[:, a + 1] = 0.0
            # waist + any non-leg slot: gait only
            for p, slot in enumerate(slots):
                if slot >= 12:
                    a = int(act[p]); ctrl[:, a] = gleg[slot]; ctrl[:, a + 1] = 0.0
            rd.ctrl.assign(ctrl.reshape(cshape).astype(cdt))
            for _ in range(sub):
                _mjw.step(rm, rd)
            if blend >= 1.0:
                phase += omega * dt
            # score
            qp = rd.qpos.numpy().reshape(K, nq); qv = rd.qvel.numpy().reshape(K, nv)
            w_, x_, y_, z_ = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
            roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
            pitch = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
            yaw = np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
            bz = qp[:, 2]; vx = qv[:, 0]; vy = qv[:, 1]
            fell = (np.abs(roll) > fr) | (np.abs(pitch) > fp) | (bz < fbz)
            newly = fell & alive
            fit[newly] -= 30.0
            alive &= ~fell
            if t >= blend_ticks:
                upr = np.maximum(0.0, 1.0 - roll * roll - pitch * pitch)
                r = (wvx * np.clip(vx, -0.1, vxt + 0.1) + walive + wup * (upr - 1.0)
                     - wlat * vy * vy - wyaw * yaw * yaw - apen * (res * res).sum(1))
                fit += r * alive
            if not alive.any():
                break
        # ES update (antithetic, rank-normalized)
        order = np.argsort(fit); ranks = np.empty_like(order, float)
        ranks[order] = np.arange(K); u = ranks / (K - 1) - 0.5
        g = np.zeros(N_PARAM)
        for i in range(pop):
            g += (u[2 * i] - u[2 * i + 1]) * eps[i]
        g /= (2 * pop * sigma)
        # BEST-MEMBER tracking: the ES MEAN can sit in a bad spot between good members
        # (the mean fell ~1.5s while the population survived). Save the single best-scoring
        # member's theta to <path>_best.npz -> deploy THAT, not the mean.
        bj = int(np.argmax(fit))
        if fit[bj] > getattr(_batched_train, "_bestF", -1e18):
            _batched_train._bestF = fit[bj]
            bpath = path.replace(".npz", "_best.npz") if path.endswith(".npz") else path + "_best.npz"
            _save(bpath, members[bj], meta=[gen, float(fit[bj])])
        theta = theta + lr * g
        meanF = float(fit.mean()); maxF = float(fit.max())
        nfell = int((~alive).sum())
        _save(path, theta, meta=[gen, meanF, maxF])
        R._log(world, "GEN %d meanF=%.2f maxF=%.2f fell=%d/%d |theta|=%.3f bestF=%.1f"
               % (gen, meanF, maxF, nfell, K, float(np.linalg.norm(theta)),
                  getattr(_batched_train, "_bestF", 0.0)))
    R._log(world, "BATCH train done (%d gens)" % maxgen)


def _live_obs(world, nq, nv, phase):
    live = world.solver.mjw_data
    qp = live.qpos.numpy().reshape(-1)[:nq]; qv = live.qvel.numpy().reshape(-1)[:nv]
    w, x, y, z = qp[3], qp[4], qp[5], qp[6]
    gx = -2.0 * (x * z - w * y); gy = -2.0 * (y * z + w * x); gz = -(1.0 - 2.0 * (x * x + y * y))
    obs = np.array([gx, gy, gz, qv[3], qv[4], qv[5], qv[0], qv[1], qv[2],
                    math.sin(phase), math.cos(phase)])
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    return obs, roll, pitch, float(qp[2]), float(qp[0]), float(qp[1])


def g1_walk_deploy_step(world):
    """LIVE deploy of the MLP-residual walk (same gait + same policy as the batched trainer).
    OMNISIM_INENGINE_PYMOD=...:g1_walk_deploy_step, RES_POLICY=<npz>, RES_HIDDEN=<H>."""
    if not WM._build(world):
        return
    if not hasattr(world, "_dep_t"):
        world._dep_t = 0
        world._dep_theta = _load(os.environ.get("RES_POLICY", ""))
        world._dep_phase = world._gwm_phase0
        world._dep_nq = int(world.solver.mj_model.nq); world._dep_nv = int(world.solver.mj_model.nv)
        world._dep_acts = float(_f("RES_ACT_SCALE", 0.30))
        world._dep_blend = max(1, int(_f("WALK_BLEND_S", 0.5) / world._gwm_dt))
        R._log(world, "DEPLOY MLP |theta|=%.3f hidden=%d N_PARAM=%d"
               % (float(np.linalg.norm(world._dep_theta)), _hidden(), N_PARAM))
    world._dep_t += 1; t = world._dep_t
    if t < _i("WALK_WARM_TICKS", 360):
        return
    tt = t - _i("WALK_WARM_TICKS", 360)
    obs, roll, pitch, bz, bx, by = _live_obs(world, world._dep_nq, world._dep_nv, world._dep_phase)
    res = world._dep_acts * _policy_batched(world._dep_theta[None, :], obs[None, :])[0]
    blend = min(1.0, (tt + 1) / world._dep_blend)
    tss = max(0.0, (tt - world._dep_blend) * world._gwm_dt)
    legs, _a, _s = world._gwm_stg.targets_np(world._dep_phase, world._gwm_gp, t_since_start=tss)
    legs = np.asarray(legs, np.float64)
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for i, slot in enumerate(world._gwm_slots):
        nd = int(world._gwm_dof[i])
        if 0 <= nd < len(tp):
            r = float(res[slot]) if slot < 12 else 0.0
            tp[nd] = (1.0 - blend) * float(tp[nd]) + blend * (float(legs[slot]) + r)
    world.control.joint_target_pos.assign(tp); world._mjc_dirty = True
    if blend >= 1.0:
        world._dep_phase += world._gwm_omega * world._gwm_dt
    if tt % 125 == 1:
        R._log(world, "deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f |res|=%.2f"
               % (tt, bx, by, bz, roll, pitch, float(np.abs(res).max())))


def g1_walk_diag_step(world):
    """DIAGNOSTIC: run the EXACT deployed (mean) policy in ONE batched-rollout world with full
    per-tick logging, so we can compare the training-side trajectory to the live deploy trajectory
    of the SAME policy. If this SURVIVES (~6s) but deploy falls ~2s -> the gap is the deploy
    ENVIRONMENT (controller/IC/arms), not the policy. Logs x/z/roll/pitch + obs each ~0.5s."""
    if getattr(world, "_diag_done", False):
        return
    if not hasattr(world, "_diag_ctr"):
        world._diag_ctr = 0
    world._diag_ctr += 1
    if world._diag_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._diag_done = True
    try:
        import warp as wp  # noqa
        if not WM._build(world):
            R._log(world, "diag: build failed"); return
        gp = world._gwm_gp; slots = world._gwm_slots; act = world._gwm_act
        dt = world._gwm_dt; omega = world._gwm_omega; sub = int(getattr(world, "_n_substeps", 4))
        phase0 = world._gwm_phase0
        theta = _load(os.environ.get("RES_POLICY", ""))
        act_scale = _f("RES_ACT_SCALE", 0.30)
        blend_ticks = max(1, int(_f("WALK_BLEND_S", 0.5) / dt))
        eplen = max(8, int(_f("WALK_EP_S", 6.0) / dt))
        leg_pos = [p for p, s in enumerate(slots) if s < 12]; leg_slot = [slots[p] for p in leg_pos]
        DK = _i("DIAG_K", 2)
        buf = world._mpc_rollout_buffers(DK)
        _mjw, rm, rd, (nq, nv, nu) = buf
        spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
        spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
        rd.qpos.assign(np.broadcast_to(spawn, (DK, nq)).astype(np.float32).reshape(rd.qpos.numpy().shape))
        rd.qvel.assign(np.zeros((DK, nv), np.float32).reshape(rd.qvel.numpy().shape))
        _mjw.forward(rm, rd)
        members = np.broadcast_to(theta, (DK, theta.shape[0])).copy()
        phase = phase0
        R._log(world, "DIAG rollout of mean policy K=%d |theta|=%.3f hidden=%d spawn_z=%.3f"
               % (DK, float(np.linalg.norm(theta)), _hidden(), spawn[2]))
        for t in range(eplen):
            qp = rd.qpos.numpy().reshape(DK, nq); qv = rd.qvel.numpy().reshape(DK, nv)
            obs, roll, pitch, bz, yaw, vx, vy = _obs_batched(qp, qv, phase, DK)
            res = act_scale * _policy_batched(members, obs)
            blend = min(1.0, (t + 1) / blend_ticks)
            tss = max(0.0, (t - blend_ticks) * dt)
            gleg = gaitvec(phase, tss) if False else None
            legs, _a, _s = world._gwm_stg.targets_np(phase, gp, t_since_start=tss); gleg = np.asarray(legs)
            ctrl = rd.ctrl.numpy().reshape(DK, nu)
            for p, slot in zip(leg_pos, leg_slot):
                a = int(act[p]); ctrl[:, a] = gleg[slot] + blend * res[:, slot]; ctrl[:, a + 1] = 0.0
            for p, slot in enumerate(slots):
                if slot >= 12:
                    a = int(act[p]); ctrl[:, a] = gleg[slot]; ctrl[:, a + 1] = 0.0
            rd.ctrl.assign(ctrl.reshape(rd.ctrl.numpy().shape).astype(rd.ctrl.numpy().dtype))
            for _ in range(sub):
                _mjw.step(rm, rd)
            if blend >= 1.0:
                phase += omega * dt
            qp = rd.qpos.numpy().reshape(DK, nq)
            w_, x_, y_, z_ = qp[0, 3], qp[0, 4], qp[0, 5], qp[0, 6]
            rr = math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
            pp = math.asin(max(-1, min(1, 2 * (w_ * y_ - z_ * x_))))
            if t % 62 == 0 or qp[0, 2] < 0.45:
                R._log(world, "DIAG t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f obs=[%s]"
                       % (t, qp[0, 0], qp[0, 1], qp[0, 2], rr, pp,
                          " ".join("%.2f" % v for v in obs[0])))
            if qp[0, 2] < 0.45:
                R._log(world, "DIAG rollout FELL @ t=%d (=%.2fs)" % (t, t * dt)); return
        R._log(world, "DIAG rollout UPRIGHT through t=%d (=%.2fs)" % (eplen, eplen * dt))
    except Exception as e:
        import traceback
        R._log(world, "diag err %r\n%s" % (e, traceback.format_exc()))


def g1_walk_batched_step(world):
    if getattr(world, "_bwr_started", False):
        return
    if not hasattr(world, "_bwr_ctr"):
        world._bwr_ctr = 0
    world._bwr_ctr += 1
    if world._bwr_ctr < _i("WALK_WARM_TICKS", 360):
        return
    # one-shot: run the whole batched ES training (blocks the live step loop -- fine headless)
    world._bwr_started = True
    try:
        _batched_train(world)
    except Exception as e:
        import traceback
        R._log(world, "batch err %r\n%s" % (e, traceback.format_exc()))
