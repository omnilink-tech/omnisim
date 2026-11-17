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

"""IN-ENGINE PPO for the G1 walk -- gradient-based, full-authority RL, trained WHERE it deploys.

ES choked on the high-authority search (gen 44, fitness still ~0). PPO is gradient-based and
~100x more sample-efficient, so it can actually discover a full-authority walk. The whole point
(the user's insight): train it INSIDE OmniSim's own Newton/mujoco_warp engine -- past PPO walks
were trained out-of-engine and never transferred; here train==deploy by construction.

Vectorized PPO over K batched mujoco_warp worlds (world._mpc_rollout_buffers): each world is an
independent env with its own gait-phase clock and reset-on-fall (+ IC randomization for
robustness). Actor = Gaussian over a residual on the gait (act_scale authority); critic = value.
GAE + clipped PPO. torch autograd runs in the embedded interpreter (the Unitree deploy proves
torch is present). Saves the actor to RES_POLICY (.pt) each iteration; deploy via g1_walk_ppo_deploy.

PHILOSOPHY (owner, 2026-06-30): in-engine training makes the PHYSICS identical (same Newton/mujoco_warp
model, proven by gain + solver-opt probes + the control-path parity probe in g1_walk_ppo_deploy). The
small residual train↔deploy closed-loop differences are NOT a bug to eliminate -- they are FREE domain
randomization, and a policy robust to them is robust by construction (and sim-to-real ready). So do not
chase bit-identical parity; train for ROBUSTNESS: W_ARATE (action-rate smoothing -> gave the first
durable in-engine walk), OBS_NOISE (anti-overfit the exact loop), IC_RAND_*, mild DR. Deploy is
non-monotonic with training (overfitting re-opens the gap); keep the most robust checkpoint, not the
highest train return. See docs/developer/train-deploy-gap.md (OWNER REFRAME callout).

Knobs: RES_POLICY(.pt) PPO_NENV(256) PPO_NSTEPS(32) PPO_EPOCHS(4) PPO_MINIB(4) PPO_LR(3e-4)
  PPO_CLIP(0.2) PPO_GAMMA(0.99) PPO_LAM(0.95) PPO_ENT(0.005) PPO_VF(0.5) PPO_ITERS(100000)
  RES_ACT_SCALE(0.6) PPO_HID(64) WALK_* IC_RAND_* W_* FALL_* RES_LOG.
"""
import glob
import math
import os
import sys

import numpy as np

_RT = os.environ.get("OMNISIM_HOME", ".")
if _RT not in sys.path:
    sys.path.insert(0, _RT)

from projects.policies.training import g1_walk_mpc as WM
from projects.policies.training import g1_residual_inengine as R


def _torch():
    """torch is not in the embedded engine interpreter by default; add the system
    site-packages (same python) so the in-engine hook can use it for PPO autograd."""
    try:
        import torch; return torch
    except ImportError:
        pass
    extra = os.environ.get("PPO_TORCH_SITE", "")
    cands = [extra] if extra else []
    _lad = os.environ.get("LOCALAPPDATA", "")
    if _lad:
        cands += sorted(glob.glob(os.path.join(
            _lad, "Programs", "Python", "Python3*", "Lib", "site-packages")), reverse=True)
    for sp in cands:
        if sp and os.path.isdir(sp) and sp not in sys.path:
            sys.path.append(sp)
    import torch; return torch

ACT_DIM = 12
# obs: proj_g(3)+ang_vel(3)+lin_vel(3)+phase(2)+jpos_rel(12)+jvel(12) = 35
OBS_DIM = 3 + 3 + 3 + 2 + 12 + 12


def _f(k, d):
    return R._f(k, d)


def _i(k, d):
    return R._i(k, d)


def _obs_dim():
    # base 35; +2 (sin/cos of world yaw) when PPO_HEADING_OBS=1 so the policy can SEE its heading
    # and actively steer straight (gravity-projection alone is yaw-blind -> unbounded heading drift).
    return OBS_DIM + (2 if _i("PPO_HEADING_OBS", 0) else 0)


def _make_ac(hid):
    torch = _torch()
    import torch.nn as nn
    od = _obs_dim()

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.pi = nn.Sequential(nn.Linear(od, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                 nn.Linear(hid, ACT_DIM))
            s.vf = nn.Sequential(nn.Linear(od, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                 nn.Linear(hid, 1))
            s.log_std = nn.Parameter(torch.full((ACT_DIM,), -1.0))
            for m in list(s.pi) + list(s.vf):
                if isinstance(m, nn.Linear):
                    nn.init.orthogonal_(m.weight, 1.0); nn.init.zeros_(m.bias)
            nn.init.orthogonal_(s.pi[-1].weight, 0.01)   # small initial policy (near gait)

        def forward(s, o):
            # clamp log_std so the entropy bonus can't inflate action noise to where the
            # deterministic-mean deploy diverges from the stochastic training rollouts.
            lo = float(os.environ.get("PPO_LOGSTD_MIN", "-3.0")); hi = float(os.environ.get("PPO_LOGSTD_MAX", "0.0"))
            return s.pi(o), s.log_std.clamp(lo, hi).exp(), s.vf(o).squeeze(-1)
    return AC()


def _build_legmap(world):
    """leg qpos-adr + dof-adr (12, L6+R6 order matching gait slots 0..11) for proprioception."""
    import mujoco as mj
    mjm = world.solver.mj_model
    qadr = {}; dadr = {}
    m2nd = world.solver.mjc_jnt_to_newton_dof.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    # map gait slot -> the mujoco joint via the newton dof in world._gwm_dof
    nd2j = {}
    for j in range(int(mjm.njnt)):
        nd2j[int(m2nd[j])] = j
    leg_qadr = []; leg_dadr = []
    for i, slot in enumerate(world._gwm_slots):
        if slot < 12:
            nd = int(world._gwm_dof[i]); j = nd2j.get(nd)
            leg_qadr.append(int(mjm.jnt_qposadr[j])); leg_dadr.append(int(mjm.jnt_dofadr[j]))
    return np.array(leg_qadr, np.int32), np.array(leg_dadr, np.int32)


def _ppo_train(world):
    torch = _torch()
    torch.set_num_threads(max(1, (os.cpu_count() or 4) - 2))
    if not WM._build(world):
        R._log(world, "ppo: WM._build failed"); return
    gp = world._gwm_gp; slots = world._gwm_slots; act = world._gwm_act
    dt = world._gwm_dt; omega = world._gwm_omega; sub = int(getattr(world, "_n_substeps", 4))
    phase0 = world._gwm_phase0
    leg_pos = [p for p, s in enumerate(slots) if s < 12]; leg_slot = [slots[p] for p in leg_pos]
    leg_act = np.array([int(act[p]) for p in leg_pos], np.int32)         # actuator idx per leg (12)
    waist_pos = [p for p, s in enumerate(slots) if s >= 12]
    leg_qadr, leg_dadr = _build_legmap(world)
    nom = np.array([gp.__dict__.get("_n", 0)] * 12, float) * 0.0          # rel-to-spawn handled below

    K = _i("PPO_NENV", 256); T = _i("PPO_NSTEPS", 32)
    act_scale = _f("RES_ACT_SCALE", 0.6)
    gamma = _f("PPO_GAMMA", 0.99); lam = _f("PPO_LAM", 0.95)
    clip = _f("PPO_CLIP", 0.2); lr = _f("PPO_LR", 3e-4)
    epochs = _i("PPO_EPOCHS", 4); nmb = _i("PPO_MINIB", 4)
    ent_c = _f("PPO_ENT", 0.005); vf_c = _f("PPO_VF", 0.5)
    iters = _i("PPO_ITERS", 100000)
    fr = _f("FALL_ROLL", 0.7); fp = _f("FALL_PITCH", 0.8); fbz = _f("FALL_BZ", 0.45)
    wvx = _f("W_VX", 5.0); walive = _f("W_ALIVE", 1.0); wlat = _f("W_LAT", 4.0)
    wup = _f("W_UP", 4.0); wyaw = _f("W_YAW", 3.0); apen = _f("RES_ACT_PEN", 0.005); fpen = _f("FALL_PEN", 5.0)
    icj = _f("IC_RAND_JOINT", 0.03); icv = _f("IC_RAND_VEL", 0.1); ica = _f("IC_RAND_ANG", 0.04)
    vxt = gp.vx
    rng = np.random.default_rng(_i("RES_SEED", 0))
    path = os.environ.get("RES_POLICY", "")

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "ppo: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    nom_leg = spawn[leg_qadr].copy()      # nominal leg angles (for obs rel + gait base check)

    # gait lookup table over one cycle (post-ramp full amplitude) -> fast per-world indexing
    NB = 128
    glut = np.zeros((NB, len(slots)))
    for b in range(NB):
        ph = phase0 + 2 * math.pi * b / NB
        legs, _a, _s = world._gwm_stg.targets_np(ph, gp, t_since_start=10.0)
        glut[b] = np.asarray(legs)[[slots[p] for p in range(len(slots))]]
    glut_leg = glut[:, leg_pos]      # (NB, 12) gait targets for the leg actuators, in leg order

    net = _make_ac(_i("PPO_HID", 64))
    if path and os.path.exists(path):
        try:
            net.load_state_dict(torch.load(path)); R._log(world, "ppo: resumed from %s" % path)
        except Exception:
            pass
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    R._log(world, "PPO train: K=%d T=%d obs=%d act=%d act_scale=%.2f hid=%d lr=%.1e spawn_z=%.3f"
           % (K, T, OBS_DIM, ACT_DIM, act_scale, _i("PPO_HID", 64), lr, spawn[2]))

    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype

    def seed_rows(rowmask):
        q = rd.qpos.numpy().reshape(K, nq); v = rd.qvel.numpy().reshape(K, nv)
        n = int(rowmask.sum())
        if n == 0:
            return
        q[rowmask] = spawn.astype(q.dtype)
        q[rowmask][:, 0:3] = 0.0  # (kept relative; x reset so reward vx is local)
        qq = q[rowmask]
        qq[:, 7:] += rng.normal(0, icj, (n, nq - 7)).astype(q.dtype)
        qq[:, 4:7] += rng.normal(0, ica, (n, 3)).astype(q.dtype)
        q[rowmask] = qq
        vv = np.zeros((n, nv), np.float32)
        vv[:, 0:2] = rng.normal(0, icv, (n, 2)); vv[:, 3:5] = rng.normal(0, icv, (n, 2))
        v[rowmask] = vv
        rd.qpos.assign(q.reshape(rd.qpos.numpy().shape)); rd.qvel.assign(v.reshape(rd.qvel.numpy().shape))

    # init all worlds
    seed_rows(np.ones(K, bool)); _mjw.forward(rm, rd)
    phase = np.full(K, phase0)
    ep_ret = np.zeros(K); ep_len = np.zeros(K, np.int32)
    ret_hist = []

    def obs_of():
        qp = rd.qpos.numpy().reshape(K, nq); qv = rd.qvel.numpy().reshape(K, nv)
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        b = ((phase - phase0) / (2 * math.pi) * NB).astype(int) % NB
        jpos = qp[:, leg_qadr] - nom_leg[None, :]
        jvel = qv[:, leg_dadr]
        o = np.concatenate([
            np.stack([gx, gy, gz], 1), 0.25 * qv[:, 3:6], qv[:, 0:3],
            np.stack([np.sin(phase), np.cos(phase)], 1), jpos, 0.1 * jvel], 1).astype(np.float32)
        return o, roll, pitch, qp[:, 2], yaw, qv[:, 0], qv[:, 1], b

    for it in range(iters):
        O = np.zeros((T, K, OBS_DIM), np.float32); A = np.zeros((T, K, ACT_DIM), np.float32)
        LP = np.zeros((T, K), np.float32); V = np.zeros((T, K), np.float32)
        RW = np.zeros((T, K), np.float32); DN = np.zeros((T, K), np.float32)
        for t in range(T):
            o, roll, pitch, bz, yaw, vx, vy, b = obs_of()
            ot = torch.from_numpy(o)
            with torch.no_grad():
                mean, std, val = net(ot)
                d = torch.distributions.Normal(mean, std)
                a = d.sample(); lp = d.log_prob(a).sum(-1)
            an = a.numpy(); env_a = np.clip(an, -1, 1) * act_scale
            ctrl = rd.ctrl.numpy().reshape(K, nu)
            gl = glut_leg[b]                                  # (K,12) gait leg targets
            ctrl[:, leg_act] = gl + env_a; ctrl[:, leg_act + 1] = 0.0
            for p in waist_pos:
                aa = int(act[p]); ctrl[:, aa] = glut[b, p]; ctrl[:, aa + 1] = 0.0
            rd.ctrl.assign(ctrl.reshape(cshape).astype(cdt))
            for _ in range(sub):
                _mjw.step(rm, rd)
            phase = phase + omega * dt
            o2, roll, pitch, bz, yaw, vx, vy, b2 = obs_of()
            upr = np.maximum(0.0, 1 - roll * roll - pitch * pitch)
            rew = (wvx * np.clip(vx, -0.1, vxt + 0.1) + walive + wup * (upr - 1)
                   - wlat * vy * vy - wyaw * yaw * yaw - apen * (env_a * env_a).sum(1)).astype(np.float32)
            fell = (np.abs(roll) > fr) | (np.abs(pitch) > fp) | (bz < fbz)
            rew = rew - fpen * fell
            O[t] = o; A[t] = an; LP[t] = lp.numpy(); V[t] = val.numpy(); RW[t] = rew; DN[t] = fell
            ep_ret += rew; ep_len += 1
            if fell.any():
                for k in np.nonzero(fell)[0]:
                    ret_hist.append(ep_ret[k]); ep_ret[k] = 0; ep_len[k] = 0
                seed_rows(fell); _mjw.forward(rm, rd); phase[fell] = phase0
        # bootstrap + GAE
        with torch.no_grad():
            o, *_ = obs_of(); _, _, lastv = net(torch.from_numpy(o)); lastv = lastv.numpy()
        adv = np.zeros((T, K), np.float32); gae = np.zeros(K, np.float32)
        for t in reversed(range(T)):
            nv_ = lastv if t == T - 1 else V[t + 1]
            nonterm = 1.0 - DN[t]
            delta = RW[t] + gamma * nv_ * nonterm - V[t]
            gae = delta + gamma * lam * nonterm * gae
            adv[t] = gae
        ret = adv + V
        # flatten + normalize adv
        bO = torch.from_numpy(O.reshape(-1, OBS_DIM)); bA = torch.from_numpy(A.reshape(-1, ACT_DIM))
        bLP = torch.from_numpy(LP.reshape(-1)); bRET = torch.from_numpy(ret.reshape(-1))
        bADV = torch.from_numpy(adv.reshape(-1)); bADV = (bADV - bADV.mean()) / (bADV.std() + 1e-8)
        N = bO.shape[0]; idx = np.arange(N)
        for _e in range(epochs):
            rng.shuffle(idx)
            for mb in np.array_split(idx, nmb):
                mb = torch.from_numpy(mb)
                mean, std, val = net(bO[mb])
                d = torch.distributions.Normal(mean, std)
                lp = d.log_prob(bA[mb]).sum(-1); ent = d.entropy().sum(-1).mean()
                ratio = torch.exp(lp - bLP[mb]); a_ = bADV[mb]
                pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                vl = ((val - bRET[mb]) ** 2).mean()
                loss = pl + vf_c * vl - ent_c * ent
                opt.zero_grad(); loss.backward()
                torch.nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
        if it % 10 == 0:
            mret = float(np.mean(ret_hist[-200:])) if ret_hist else 0.0
            R._log(world, "PPO it=%d epret~%.1f stdmean=%.2f neps=%d"
                   % (it, mret, float(net.log_std.exp().mean()), len(ret_hist)))
            try:
                import torch as _t; _t.save(net.state_dict(), path)
            except Exception:
                pass


def _ppo_train_gpu(world):
    """GPU-RESIDENT in-engine PPO. The entire rollout (obs, policy, action, reward, reset) runs on
    CUDA with ZERO per-step host sync: qpos/qvel/ctrl are wp.to_torch zero-copy views of the same
    GPU buffers mujoco_warp steps, the policy is a CUDA net, and the substep loop is CUDA-graph
    captured. This is the ~20x port of _ppo_train (the .numpy()/.assign() host roundtrip is gone).
    Mirrors gpu_mjwarp_g1_walk_trainer.py (proven ~98k steps/s on this GPU) inside the engine."""
    torch = _torch()
    import warp as wp
    dev = os.environ.get("PPO_DEVICE", "cuda")
    if dev == "cuda" and not torch.cuda.is_available():
        R._log(world, "PPO-GPU: cuda not available -> falling back to slow CPU path"); _ppo_train(world); return
    if not WM._build(world):
        R._log(world, "ppo-gpu: WM._build failed"); return
    gp = world._gwm_gp; slots = world._gwm_slots; act = world._gwm_act
    dt = world._gwm_dt; omega = world._gwm_omega; sub = int(getattr(world, "_n_substeps", 4))
    phase0 = world._gwm_phase0
    leg_pos = [p for p, s in enumerate(slots) if s < 12]
    leg_act = np.array([int(act[p]) for p in leg_pos], np.int32)
    waist_pos = [p for p, s in enumerate(slots) if s >= 12]
    waist_act = np.array([int(act[p]) for p in waist_pos], np.int32)
    leg_qadr, leg_dadr = _build_legmap(world)

    K = _i("PPO_NENV", 2048); T = _i("PPO_NSTEPS", 32)
    od = _obs_dim(); heading = _i("PPO_HEADING_OBS", 0)
    act_scale = _f("RES_ACT_SCALE", 0.6)
    gamma = _f("PPO_GAMMA", 0.99); lam = _f("PPO_LAM", 0.95)
    clip = _f("PPO_CLIP", 0.2); lr = _f("PPO_LR", 3e-4)
    epochs = _i("PPO_EPOCHS", 4); nmb = _i("PPO_MINIB", 8)
    ent_c = _f("PPO_ENT", 0.005); vf_c = _f("PPO_VF", 0.5)
    iters = _i("PPO_ITERS", 100000)
    fr = _f("FALL_ROLL", 0.7); fp = _f("FALL_PITCH", 0.8); fbz = _f("FALL_BZ", 0.45)
    wvx = _f("W_VX", 5.0); walive = _f("W_ALIVE", 1.0); wlat = _f("W_LAT", 4.0)
    wup = _f("W_UP", 4.0); wyaw = _f("W_YAW", 3.0); apen = _f("RES_ACT_PEN", 0.005); fpen = _f("FALL_PEN", 5.0)
    wheight = _f("W_HEIGHT", 15.0); z_tgt = _f("Z_TGT", 0.70)   # hold stance height (kills the sink-then-topple)
    wprate = _f("W_PRATE", 0.0)                                  # optional pitch-rate damping
    vtsig = _f("VTRACK_SIG", 0.25)                              # velocity-tracking gaussian width
    warate = _f("W_ARATE", 0.0)                                 # action-RATE penalty: smooth policy (anti-jitter)
    obs_noise = _f("OBS_NOISE", 0.0)                            # gaussian obs noise: anti-overfit the exact train loop
    icj = _f("IC_RAND_JOINT", 0.03); icv = _f("IC_RAND_VEL", 0.1); ica = _f("IC_RAND_ANG", 0.04)
    vxt = float(gp.vx)
    use_graph = _i("PPO_GRAPH", 1) and dev == "cuda"
    path = os.environ.get("RES_POLICY", "")
    torch.manual_seed(_i("RES_SEED", 0))

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "ppo-gpu: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    # zero-copy CUDA views of the SAME buffers mujoco_warp steps (no host sync, ever)
    qpos_t = wp.to_torch(rd.qpos).view(K, nq)
    qvel_t = wp.to_torch(rd.qvel).view(K, nv)
    ctrl_t = wp.to_torch(rd.ctrl).view(K, nu)
    tdev = qpos_t.device
    R._log(world, "PPO-GPU: K=%d T=%d dev=%s graph=%d nq=%d nv=%d nu=%d view_dev=%s"
           % (K, T, dev, int(bool(use_graph)), nq, nv, nu, tdev))
    # TRAIN->DEPLOY GAP PROBE: do the rollout worlds (put_data) carry the same actuator
    # stiffness (KE/KD) the LIVE deploy applies at runtime? a mismatch => trains at one
    # stiffness, deploys at another => survives train, falls deploy.
    try:
        la0 = int(leg_act[0])
        def _arr(a):
            return a.numpy() if hasattr(a, "numpy") else np.asarray(a)
        rg = _arr(rm.actuator_gainprm).reshape(nu, -1); rb = _arr(rm.actuator_biasprm).reshape(nu, -1)
        lm = world.solver.mj_model
        lg = np.asarray(lm.actuator_gainprm).reshape(nu, -1); lb = np.asarray(lm.actuator_biasprm).reshape(nu, -1)
        R._log(world, "PPO-GPU GAINPROBE la0=%d rollout gain=%s bias=%s | live gain=%s bias=%s"
               % (la0, rg[la0][:3], rb[la0][:3], lg[la0][:3], lb[la0][:3]))
        def _opt(m, names):
            o = m.opt; out = {}
            for n in names:
                try:
                    v = getattr(o, n)
                    out[n] = float(v.numpy()) if hasattr(v, "numpy") else (float(v) if not hasattr(v, "__len__") else v)
                except Exception:
                    out[n] = "?"
            return out
        names = ["timestep", "iterations", "ls_iterations", "integrator", "cone", "solver"]
        R._log(world, "PPO-GPU OPTPROBE rollout=%s | live=%s | n_substeps=%s"
               % (_opt(rm, names), _opt(world.solver.mjw_model, names), sub))
    except Exception as e:
        import traceback
        R._log(world, "PPO-GPU GAINPROBE err %r\n%s" % (e, traceback.format_exc()))

    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    spawn_t = torch.tensor(spawn, dtype=torch.float32, device=tdev)
    nom_leg_t = torch.tensor(spawn[leg_qadr], dtype=torch.float32, device=tdev)

    NB = 128
    glut = np.zeros((NB, len(slots)))
    for bb in range(NB):
        legs, _a, _s = world._gwm_stg.targets_np(phase0 + 2 * math.pi * bb / NB, gp, t_since_start=10.0)
        glut[bb] = np.asarray(legs)[[slots[p] for p in range(len(slots))]]
    glut_leg_t = torch.tensor(glut[:, leg_pos], dtype=torch.float32, device=tdev)        # (NB,12)
    glut_waist_t = torch.tensor(glut[:, waist_pos], dtype=torch.float32, device=tdev)    # (NB,nw)
    leg_qadr_t = torch.tensor(leg_qadr, dtype=torch.long, device=tdev)
    leg_dadr_t = torch.tensor(leg_dadr, dtype=torch.long, device=tdev)
    leg_act_t = torch.tensor(leg_act, dtype=torch.long, device=tdev)
    waist_act_t = torch.tensor(waist_act, dtype=torch.long, device=tdev)

    net = _make_ac(_i("PPO_HID", 64)).to(tdev)
    if path and os.path.exists(path):
        try:
            net.load_state_dict(torch.load(path, map_location=tdev)); R._log(world, "ppo-gpu: resumed %s" % path)
        except Exception:
            pass
    opt = torch.optim.Adam(net.parameters(), lr=lr)

    phase_t = torch.full((K,), float(phase0), device=tdev)
    twopi = 2 * math.pi

    def reset_fields(mask):
        """In-place GPU reset of masked worlds (no host sync, no .sum())."""
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, device=tdev) * icj
        rq[:, 4:7] += torch.randn(K, 3, device=tdev) * ica
        rv = torch.zeros(K, nv, device=tdev)
        rv[:, 0:2] = torch.randn(K, 2, device=tdev) * icv
        rv[:, 3:5] = torch.randn(K, 2, device=tdev) * icv
        m = mask.unsqueeze(1)
        qpos_t.copy_(torch.where(m, rq, qpos_t))
        qvel_t.copy_(torch.where(m, rv, qvel_t))

    def obs_of():
        qp = qpos_t; qv = qvel_t
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1, 1))
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        b = ((phase_t - phase0) / twopi * NB).long() % NB
        jpos = qp[:, leg_qadr_t] - nom_leg_t
        jvel = qv[:, leg_dadr_t]
        feats = [torch.stack([gx, gy, gz], 1), 0.25 * qv[:, 3:6], qv[:, 0:3],
                 torch.stack([torch.sin(phase_t), torch.cos(phase_t)], 1), jpos, 0.1 * jvel]
        if heading:
            feats.append(torch.stack([torch.sin(yaw), torch.cos(yaw)], 1))   # world heading -> steer straight
        o = torch.cat(feats, 1)
        return o, roll, pitch, qp[:, 2], yaw, qv[:, 0], qv[:, 1], b

    # init all worlds, then one warm forward
    reset_fields(torch.ones(K, dtype=torch.bool, device=tdev))
    torch.cuda.synchronize() if dev == "cuda" else None
    _mjw.forward(rm, rd); wp.synchronize()

    step_graph = None
    if use_graph:
        try:
            for _ in range(sub):
                _mjw.step(rm, rd)
            wp.synchronize()
            with wp.ScopedDevice(world.model.device):
                wp.capture_begin(force_module_load=False)
                try:
                    for _ in range(sub):
                        _mjw.step(rm, rd)
                finally:
                    step_graph = wp.capture_end()
            R._log(world, "PPO-GPU: CUDA-graph captured (sub=%d)" % sub)
        except Exception as e:
            R._log(world, "PPO-GPU: graph capture failed (%s) -> plain substep loop" % e); step_graph = None

    def physics():
        if step_graph is not None:
            wp.capture_launch(step_graph)
        else:
            for _ in range(sub):
                _mjw.step(rm, rd)

    ep_ret = torch.zeros(K, device=tdev)
    prev_a = torch.zeros(K, ACT_DIM, device=tdev)           # last action, for the action-rate penalty
    vx_acc = torch.zeros((), device=tdev); vx_n = 0          # mean forward speed (walk vs stand?)
    ar_acc = torch.zeros((), device=tdev)                   # mean action-rate (jitter monitor)
    done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
    O = torch.zeros(T, K, od, device=tdev); A = torch.zeros(T, K, ACT_DIM, device=tdev)
    LP = torch.zeros(T, K, device=tdev); V = torch.zeros(T, K, device=tdev)
    RW = torch.zeros(T, K, device=tdev); DN = torch.zeros(T, K, device=tdev)
    fpen_t = float(fpen)
    import time as _time
    t0 = _time.time(); total_steps = 0

    for it in range(iters):
        with torch.no_grad():
            for t in range(T):
                o, roll, pitch, bz, yaw, vx, vy, b = obs_of()
                if obs_noise > 0:
                    o = o + torch.randn_like(o) * obs_noise     # regularize: don't memorize exact obs
                mean, std, val = net(o)
                d = torch.distributions.Normal(mean, std)
                a = d.sample(); lp = d.log_prob(a).sum(-1)
                env_a = torch.clamp(a, -1, 1) * act_scale
                arate = ((env_a - prev_a) ** 2).sum(1)              # tick-to-tick action change (jitter)
                gl = glut_leg_t[b]                                  # (K,12)
                ctrl_t[:, leg_act_t] = gl + env_a
                ctrl_t[:, leg_act_t + 1] = 0.0
                gw = glut_waist_t[b]
                ctrl_t[:, waist_act_t] = gw
                ctrl_t[:, waist_act_t + 1] = 0.0
                physics()
                phase_t.add_(omega * dt)
                o2, roll, pitch, bz, yaw, vx, vy, b2 = obs_of()
                upr = torch.clamp(1 - roll * roll - pitch * pitch, min=0.0)
                sink = torch.clamp(z_tgt - bz, min=0.0)            # how far below target height
                # velocity TRACKING (gaussian at vxt): rewards walking AT target speed, penalizes the
                # forward LUNGE (vx >> vxt) that was diving-then-faceplanting for free vx reward.
                vtrack = torch.exp(-((vx - vxt) ** 2) / (vtsig * vtsig))
                rew = (wvx * vtrack + walive + wup * (upr - 1)
                       - wlat * vy * vy - wyaw * yaw * yaw - apen * (env_a * env_a).sum(1)
                       - wheight * sink * sink - wprate * pitch * pitch - warate * arate)
                fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz)
                rew = rew - fpen_t * fell.float()
                O[t] = o; A[t] = a; LP[t] = lp; V[t] = val; RW[t] = rew; DN[t] = fell.float()
                vx_acc += vx.detach().mean(); ar_acc += arate.detach().mean(); vx_n += 1
                ep_ret = ep_ret + rew
                ff = fell.float()
                done_sum += (ep_ret * ff).sum(); done_cnt += ff.sum()
                ep_ret = ep_ret * (1.0 - ff)
                prev_a = torch.where(fell.unsqueeze(1), torch.zeros_like(env_a), env_a.detach())
                reset_fields(fell)
                phase_t = torch.where(fell, torch.full_like(phase_t, float(phase0)), phase_t)
            # bootstrap + GAE on device
            o, *_ = obs_of(); _, _, lastv = net(o)
            adv = torch.zeros(T, K, device=tdev); gae = torch.zeros(K, device=tdev)
            for t in reversed(range(T)):
                nv_ = lastv if t == T - 1 else V[t + 1]
                nonterm = 1.0 - DN[t]
                delta = RW[t] + gamma * nv_ * nonterm - V[t]
                gae = delta + gamma * lam * nonterm * gae
                adv[t] = gae
            ret = adv + V
        total_steps += T * K
        # PPO update (on device)
        bO = O.reshape(-1, od); bA = A.reshape(-1, ACT_DIM)
        bLP = LP.reshape(-1); bRET = ret.reshape(-1)
        bADV = adv.reshape(-1); bADV = (bADV - bADV.mean()) / (bADV.std() + 1e-8)
        N = bO.shape[0]
        for _e in range(epochs):
            perm = torch.randperm(N, device=tdev)
            for mb in perm.chunk(nmb):
                mean, std, val = net(bO[mb])
                d = torch.distributions.Normal(mean, std)
                lp = d.log_prob(bA[mb]).sum(-1); ent = d.entropy().sum(-1).mean()
                ratio = torch.exp(lp - bLP[mb]); a_ = bADV[mb]
                pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                vl = ((val - bRET[mb]) ** 2).mean()
                loss = pl + vf_c * vl - ent_c * ent
                opt.zero_grad(); loss.backward()
                torch.nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
        if it % 10 == 0:
            cnt = float(done_cnt); mret = float(done_sum / done_cnt) if cnt > 0 else 0.0
            fps = total_steps / max(1e-6, _time.time() - t0)
            mvx = float(vx_acc) / max(1, vx_n); mar = float(ar_acc) / max(1, vx_n)
            R._log(world, "PPO-GPU it=%d epret~%.1f mvx=%.3f(tgt%.2f) jit=%.4f stdmean=%.2f neps=%d steps/s=%.0f"
                   % (it, mret, mvx, vxt, mar, float(net.log_std.exp().mean()), int(cnt), fps))
            try:
                torch.save(net.state_dict(), path)
                ce = _i("CKPT_EVERY", 0)
                if ce > 0 and it % ce == 0 and it > 0 and path:
                    torch.save(net.state_dict(), path.replace(".pt", "_it%d.pt" % it))   # keep-most-robust sweep
            except Exception:
                pass
            t0 = _time.time(); total_steps = 0
            done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)  # recent-window avg
            vx_acc = torch.zeros((), device=tdev); vx_n = 0; ar_acc = torch.zeros((), device=tdev)


def _ppo_deploy_setup(world):
    torch = _torch()
    if not WM._build(world):
        return False
    world._ppo_legact = np.array([int(world._gwm_act[p]) for p, s in enumerate(world._gwm_slots) if s < 12], np.int32)
    world._ppo_legdof = np.array([int(world._gwm_dof[p]) for p, s in enumerate(world._gwm_slots) if s < 12], np.int32)
    world._ppo_qadr, world._ppo_dadr = _build_legmap(world)
    gp = world._gwm_gp; phase0 = world._gwm_phase0; slots = world._gwm_slots
    NB = 128; glut = np.zeros((NB, len(slots)))
    for b in range(NB):
        legs, _a, _s = world._gwm_stg.targets_np(phase0 + 2 * math.pi * b / NB, gp, t_since_start=10.0)
        glut[b] = np.asarray(legs)[[slots[p] for p in range(len(slots))]]
    world._ppo_glut = glut; world._ppo_NB = NB
    world._ppo_legpos = [p for p, s in enumerate(slots) if s < 12]
    world._ppo_waistpos = [p for p, s in enumerate(slots) if s >= 12]
    world._ppo_gdof = [int(world._gwm_dof[p]) for p in range(len(slots))]
    world._ppo_gact = [int(world._gwm_act[p]) for p in range(len(slots))]   # actuator idx per slot pos
    R._log(world, "PPO DEPLOY map: legpos=%s legslots=%s waistpos=%s gdof=%s"
           % (world._ppo_legpos, [int(slots[p]) for p in world._ppo_legpos], world._ppo_waistpos, world._ppo_gdof))
    sp = world.solver.mjw_data.qpos.numpy().reshape(-1).copy()
    world._ppo_nomleg = sp[world._ppo_qadr].copy()
    net = _make_ac(_i("PPO_HID", 64))
    net.load_state_dict(torch.load(os.environ.get("RES_POLICY", ""), map_location="cpu"))
    net.eval(); world._ppo_net = net
    world._ppo_nq = int(world.solver.mj_model.nq); world._ppo_nv = int(world.solver.mj_model.nv)
    world._ppo_phase = phase0; world._ppo_t = 0
    world._ppo_as = _f("RES_ACT_SCALE", 0.6); world._ppo_omega = world._gwm_omega; world._ppo_dt = world._gwm_dt
    R._log(world, "PPO DEPLOY ready act_scale=%.2f" % world._ppo_as)
    return True


def g1_walk_ppo_deploy(world):
    torch = _torch()
    if getattr(world, "_ppo_dep", None) is None:
        world._ppo_dep = False
        if _ppo_deploy_setup(world):
            world._ppo_dep = True
    if not world._ppo_dep:
        return
    if not hasattr(world, "_ppo_dt0"):
        world._ppo_dt0 = 0
    world._ppo_dt0 += 1
    if world._ppo_dt0 < _i("WALK_WARM_TICKS", 360):
        return
    tt = world._ppo_dt0 - _i("WALK_WARM_TICKS", 360)
    live = world.solver.mjw_data; qp = live.qpos.numpy().reshape(-1); qv = live.qvel.numpy().reshape(-1)
    w, x, y, z = qp[3], qp[4], qp[5], qp[6]
    gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
    ph = world._ppo_phase
    jpos = qp[world._ppo_qadr] - world._ppo_nomleg; jvel = qv[world._ppo_dadr]
    parts = [[gx, gy, gz], 0.25 * qv[3:6], qv[0:3], [math.sin(ph), math.cos(ph)], jpos, 0.1 * jvel]
    if _i("PPO_HEADING_OBS", 0):
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        parts.append([math.sin(yaw), math.cos(yaw)])
    o = np.concatenate(parts).astype(np.float32)
    with torch.no_grad():
        mean, _std, _v = world._ppo_net(torch.from_numpy(o[None, :]))
    env_a = np.clip(mean.numpy()[0], -1, 1) * world._ppo_as
    b = int((ph - world._gwm_phase0) / (2 * math.pi) * world._ppo_NB) % world._ppo_NB
    gl = world._ppo_glut[b]
    if _i("PPO_DEPLOY_CTRL", 0):
        # DIAGNOSTIC: write the live mjw ctrl array DIRECTLY at the actuator indices, exactly as
        # training does (bypass joint_target_pos + the Newton bridge). If this survives where the
        # joint_target_pos path falls, the train->deploy gap is the engine deploy control path.
        gact = world._ppo_gact
        cc = world.solver.mjw_data.ctrl.numpy(); cflat = cc.reshape(-1)
        for i, p in enumerate(world._ppo_legpos):
            a = gact[p]
            if 0 <= a < len(cflat):
                cflat[a] = float(gl[p]) + float(env_a[i])
                if a + 1 < len(cflat):
                    cflat[a + 1] = 0.0
        for p in world._ppo_waistpos:
            a = gact[p]
            if 0 <= a < len(cflat):
                cflat[a] = float(gl[p])
                if a + 1 < len(cflat):
                    cflat[a + 1] = 0.0
        world.solver.mjw_data.ctrl.assign(cc)
    else:
        if world.control is None:
            world.control = world.model.control()
        tp = world.control.joint_target_q.numpy()
        gdof = world._ppo_gdof
        for i, p in enumerate(world._ppo_legpos):      # legs: gait[p] + residual (mirrors training ctrl write)
            nd = gdof[p]
            if 0 <= nd < len(tp):
                tp[nd] = float(gl[p]) + float(env_a[i])
        for p in world._ppo_waistpos:                   # waist/torso: gait only (training drives these too)
            nd = gdof[p]
            if 0 <= nd < len(tp):
                tp[nd] = float(gl[p])
        world.control.joint_target_q.assign(tp); world._mjc_dirty = True
    if tt < 4 or tt % 100 == 1:
        # PARITY: did the engine's deploy path apply my commanded leg target to the live mjw ctrl,
        # or transform it? (training writes ctrl directly; this is the deploy path's actual effect.)
        try:
            cc = world.solver.mjw_data.ctrl.numpy().reshape(-1)
            p0 = world._ppo_legpos[0]; a0 = world._ppo_gact[p0]; tgt0 = float(gl[p0]) + float(env_a[0])
            R._log(world, "PARITY tt=%d leg0 target=%.4f live_ctrl[%d]=%.4f d=%.4f"
                   % (tt, tgt0, a0, float(cc[a0]), tgt0 - float(cc[a0])))
        except Exception as e:
            R._log(world, "PARITY err %r" % e)
    world._ppo_phase += world._ppo_omega * world._ppo_dt * _f("PPO_DEPLOY_PHASE_SCALE", 1.0)
    if tt % 20 == 1:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pit = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        R._log(world, "ppo deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f" % (tt, qp[0], qp[1], qp[2], roll, pit))


def g1_walk_ppo_step(world):
    if getattr(world, "_ppo_started", False):
        return
    if not hasattr(world, "_ppo_ctr"):
        world._ppo_ctr = 0
    world._ppo_ctr += 1
    if world._ppo_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._ppo_started = True
    try:
        _ppo_train(world)
    except Exception as e:
        import traceback
        R._log(world, "ppo err %r\n%s" % (e, traceback.format_exc()))


def g1_walk_ppo_gpu_step(world):
    """GPU-resident in-engine PPO train hook (the ~20x fast path). Same warm-up gate, then runs the
    fully-on-CUDA rollout. Launch with OMNISIM_INENGINE_PYMOD=...g1_walk_ppo:g1_walk_ppo_gpu_step."""
    if getattr(world, "_ppo_started", False):
        return
    if not hasattr(world, "_ppo_ctr"):
        world._ppo_ctr = 0
    world._ppo_ctr += 1
    if world._ppo_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._ppo_started = True
    try:
        _ppo_train_gpu(world)
    except Exception as e:
        import traceback
        R._log(world, "ppo-gpu err %r\n%s" % (e, traceback.format_exc()))
