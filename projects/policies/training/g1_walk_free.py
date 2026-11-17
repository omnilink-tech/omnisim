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

"""FREE-POLICY in-engine PPO humanoid walk -- NO fixed-gait anchor. The architecture that actually
produces durable locomotion (the Unitree-style recipe), trained WHERE it deploys (OmniSim Newton/
mujoco_warp), at the 44x GPU-resident throughput of g1_walk_ppo.

Why this exists: the residual-on-a-fixed-gait policy (g1_walk_ppo) plateaued at ~3 m because the gait
anchor constrains the policy to a marginal motion. Here the network outputs the FULL leg joint targets
itself (offsets from the nominal stand pose, act_scale authority), so it can discover a *stable* gait.
A phase clock is provided as an OBSERVATION for rhythm, but it does not dictate the motion.

Obs (47, Unitree-style, NO base-linvel -- the non-transferable cheat is omitted):
  proj_g(3) + ang_vel(3) + cmd_vx(1) + jpos_rel(12) + jvel(12) + last_action(12) + phase sin/cos(2)
  + heading sin/cos(2).
Reward (robust): velocity-tracking to cmd_vx + alive + upright + height-hold + action-rate smoothing
  (anti-jitter -> survives the train↔deploy micro-diffs, which are a FEATURE / free DR, see
  docs/developer/train-deploy-gap.md) - lateral - yaw - action-mag, fall-terminated.

Same GPU-resident core as g1_walk_ppo._ppo_train_gpu: wp.to_torch zero-copy views of the mujoco_warp
rollout buffers, CUDA policy + on-GPU obs/reward/reset, CUDA-graph-captured substeps. NO engine rebuild.

Knobs: FREE_CMD_VX(0.4) FREE_FREQ(1.5) FREE_ACT_SCALE(0.8) PPO_* W_VX/W_UP/W_HEIGHT/W_LAT/W_YAW/W_ARATE
  RES_ACT_PEN OBS_NOISE IC_RAND_* Z_TGT FALL_* PPO_LOGSTD_MAX CKPT_EVERY PPO_DEPLOY_PHASE_SCALE RES_POLICY.
Launch: run_walk_rl.sh <dur> <tag> train|deploy headless ... \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_free:g1_walk_free_step
"""
import math
import os
import sys

import numpy as np

_RT = os.environ.get("OMNISIM_HOME", ".")
if _RT not in sys.path:
    sys.path.insert(0, _RT)

from projects.policies.training import g1_walk_mpc as WM
from projects.policies.training import g1_residual_inengine as R
from projects.policies.training.g1_walk_ppo import _torch, _f, _i, _build_legmap

ACT_DIM = 12
OBS_DIM = 3 + 3 + 1 + 12 + 12 + 12 + 2 + 2   # = 47


def _make_ac(hid):
    torch = _torch()
    import torch.nn as nn

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.pi = nn.Sequential(nn.Linear(OBS_DIM, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                 nn.Linear(hid, ACT_DIM))
            s.vf = nn.Sequential(nn.Linear(OBS_DIM, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                 nn.Linear(hid, 1))
            s.log_std = nn.Parameter(torch.full((ACT_DIM,), -1.0))
            for m in list(s.pi) + list(s.vf):
                if isinstance(m, nn.Linear):
                    nn.init.orthogonal_(m.weight, 1.0); nn.init.zeros_(m.bias)
            nn.init.orthogonal_(s.pi[-1].weight, 0.01)

        def forward(s, o):
            lo = float(os.environ.get("PPO_LOGSTD_MIN", "-3.0")); hi = float(os.environ.get("PPO_LOGSTD_MAX", "0.0"))
            return s.pi(o), s.log_std.clamp(lo, hi).exp(), s.vf(o).squeeze(-1)
    return AC()


def _build_fullmap(world):
    """qadr + dadr + actuator-idx for ALL actuated slot positions (legs first then waist)."""
    import mujoco as mj  # noqa: F401
    mjm = world.solver.mj_model
    m2nd = world.solver.mjc_jnt_to_newton_dof.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    nd2j = {int(m2nd[j]): j for j in range(int(mjm.njnt))}
    slots = world._gwm_slots
    qadr = []; dadr = []; aidx = []; is_leg = []
    for p in range(len(slots)):
        nd = int(world._gwm_dof[p]); j = nd2j.get(nd)
        qadr.append(int(mjm.jnt_qposadr[j])); dadr.append(int(mjm.jnt_dofadr[j]))
        aidx.append(int(world._gwm_act[p])); is_leg.append(1 if slots[p] < 12 else 0)
    return (np.array(qadr, np.int32), np.array(dadr, np.int32),
            np.array(aidx, np.int32), np.array(is_leg, np.int32))


def _free_train_gpu(world):
    torch = _torch()
    import warp as wp
    import time as _time
    dev = os.environ.get("PPO_DEVICE", "cuda")
    if dev == "cuda" and not torch.cuda.is_available():
        R._log(world, "FREE: cuda not available"); return
    if not WM._build(world):
        R._log(world, "free: WM._build failed"); return
    sub = int(getattr(world, "_n_substeps", 4))
    dt = world._gwm_dt
    qadr, dadr, aidx, is_leg = _build_fullmap(world)
    leg_qadr, leg_dadr = _build_legmap(world)             # 12 leg joints (obs order)
    legmask = is_leg.astype(bool)
    leg_aidx = aidx[legmask]                               # 12 leg actuators (action order == _build_legmap order)

    K = _i("PPO_NENV", 2048); T = _i("PPO_NSTEPS", 32)
    cmd_vx = _f("FREE_CMD_VX", 0.4); free_freq = _f("FREE_FREQ", 1.5); act_scale = _f("FREE_ACT_SCALE", 0.8)
    omega = 2.0 * math.pi * free_freq
    gamma = _f("PPO_GAMMA", 0.99); lam = _f("PPO_LAM", 0.95)
    clip = _f("PPO_CLIP", 0.2); lr = _f("PPO_LR", 3e-4)
    epochs = _i("PPO_EPOCHS", 4); nmb = _i("PPO_MINIB", 8)
    ent_c = _f("PPO_ENT", 0.003); vf_c = _f("PPO_VF", 0.5); iters = _i("PPO_ITERS", 100000)
    fr = _f("FALL_ROLL", 0.7); fp = _f("FALL_PITCH", 0.7); fbz = _f("FALL_BZ", 0.45)
    wvx = _f("W_VX", 6.0); walive = _f("W_ALIVE", 1.0); wlat = _f("W_LAT", 4.0)
    wup = _f("W_UP", 6.0); wyaw = _f("W_YAW", 4.0); apen = _f("RES_ACT_PEN", 0.01); fpen = _f("FALL_PEN", 5.0)
    wheight = _f("W_HEIGHT", 15.0); z_tgt = _f("Z_TGT", 0.70); warate = _f("W_ARATE", 1.0)
    vtsig = _f("VTRACK_SIG", 0.25)
    icj = _f("IC_RAND_JOINT", 0.04); icv = _f("IC_RAND_VEL", 0.12); ica = _f("IC_RAND_ANG", 0.06)
    obs_noise = _f("OBS_NOISE", 0.0)
    use_graph = _i("PPO_GRAPH", 1) and dev == "cuda"
    path = os.environ.get("RES_POLICY", "")
    torch.manual_seed(_i("RES_SEED", 0))

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "free: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    qpos_t = wp.to_torch(rd.qpos).view(K, nq)
    qvel_t = wp.to_torch(rd.qvel).view(K, nv)
    ctrl_t = wp.to_torch(rd.ctrl).view(K, nu)
    tdev = qpos_t.device

    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    spawn_t = torch.tensor(spawn, dtype=torch.float32, device=tdev)
    nom_all = torch.tensor(spawn[qadr], dtype=torch.float32, device=tdev)          # nominal target per slot
    nom_leg = torch.tensor(spawn[leg_qadr], dtype=torch.float32, device=tdev)      # 12 leg nominal (obs ref)
    qadr_t = torch.tensor(qadr, dtype=torch.long, device=tdev)
    aidx_t = torch.tensor(aidx, dtype=torch.long, device=tdev)
    leg_qadr_t = torch.tensor(leg_qadr, dtype=torch.long, device=tdev)
    leg_dadr_t = torch.tensor(leg_dadr, dtype=torch.long, device=tdev)
    leg_aidx_t = torch.tensor(leg_aidx, dtype=torch.long, device=tdev)
    legmask_t = torch.tensor(legmask, dtype=torch.bool, device=tdev)
    R._log(world, "FREE-GPU: K=%d T=%d obs=%d act=%d cmd_vx=%.2f freq=%.2f act_scale=%.2f graph=%d nslots=%d"
           % (K, T, OBS_DIM, ACT_DIM, cmd_vx, free_freq, act_scale, int(bool(use_graph)), len(aidx)))

    net = _make_ac(_i("PPO_HID", 128)).to(tdev)
    if path and os.path.exists(path):
        try:
            net.load_state_dict(torch.load(path, map_location=tdev)); R._log(world, "free: resumed %s" % path)
        except Exception:
            pass
    opt = torch.optim.Adam(net.parameters(), lr=lr)

    phase_t = torch.zeros(K, device=tdev)

    def reset_fields(mask):
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, device=tdev) * icj
        rq[:, 4:7] += torch.randn(K, 3, device=tdev) * ica
        rv = torch.zeros(K, nv, device=tdev)
        rv[:, 0:2] = torch.randn(K, 2, device=tdev) * icv
        rv[:, 3:5] = torch.randn(K, 2, device=tdev) * icv
        m = mask.unsqueeze(1)
        qpos_t.copy_(torch.where(m, rq, qpos_t))
        qvel_t.copy_(torch.where(m, rv, qvel_t))

    def obs_of(last_a):
        qp = qpos_t; qv = qvel_t
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1, 1))
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        jpos = qp[:, leg_qadr_t] - nom_leg
        jvel = qv[:, leg_dadr_t]
        cmd = torch.full((K, 1), cmd_vx, device=tdev)
        o = torch.cat([torch.stack([gx, gy, gz], 1), 0.25 * qv[:, 3:6], cmd, jpos, 0.1 * jvel, last_a,
                       torch.stack([torch.sin(phase_t), torch.cos(phase_t)], 1),
                       torch.stack([torch.sin(yaw), torch.cos(yaw)], 1)], 1)
        return o, roll, pitch, qp[:, 2], yaw, qv[:, 0], qv[:, 1]

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
            R._log(world, "FREE-GPU: CUDA-graph captured (sub=%d)" % sub)
        except Exception as e:
            R._log(world, "FREE-GPU: graph capture failed (%s)" % e); step_graph = None

    def physics():
        if step_graph is not None:
            wp.capture_launch(step_graph)
        else:
            for _ in range(sub):
                _mjw.step(rm, rd)

    ep_ret = torch.zeros(K, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
    last_a = torch.zeros(K, ACT_DIM, device=tdev)
    vx_acc = torch.zeros((), device=tdev); ar_acc = torch.zeros((), device=tdev); vx_n = 0
    done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
    O = torch.zeros(T, K, OBS_DIM, device=tdev); A = torch.zeros(T, K, ACT_DIM, device=tdev)
    LP = torch.zeros(T, K, device=tdev); V = torch.zeros(T, K, device=tdev)
    RW = torch.zeros(T, K, device=tdev); DN = torch.zeros(T, K, device=tdev)
    t0 = _time.time(); total_steps = 0

    for it in range(iters):
        with torch.no_grad():
            for t in range(T):
                o, roll, pitch, bz, yaw, vx, vy = obs_of(last_a)
                oin = o + torch.randn_like(o) * obs_noise if obs_noise > 0 else o
                mean, std, val = net(oin)
                d = torch.distributions.Normal(mean, std)
                a = d.sample(); lp = d.log_prob(a).sum(-1)
                ac = torch.clamp(a, -1, 1)
                env_a = ac * act_scale
                # write ALL slot targets: legs = nominal + action; waist = nominal (held)
                tgt = nom_all.unsqueeze(0).repeat(K, 1)
                tgt[:, legmask_t] = nom_leg + env_a
                ctrl_t[:, aidx_t] = tgt
                ctrl_t[:, aidx_t + 1] = 0.0
                physics()
                phase_t.add_(omega * dt)
                arate = ((ac - prev_a) ** 2).sum(1)
                o2, roll, pitch, bz, yaw, vx, vy = obs_of(ac)
                upr = torch.clamp(1 - roll * roll - pitch * pitch, min=0.0)
                sink = torch.clamp(z_tgt - bz, min=0.0)
                vtrack = torch.exp(-((vx - cmd_vx) ** 2) / (vtsig * vtsig))
                rew = (wvx * vtrack + walive + wup * (upr - 1) - wlat * vy * vy - wyaw * yaw * yaw
                       - wheight * sink * sink - warate * arate - apen * (env_a * env_a).sum(1))
                fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz)
                rew = rew - fpen * fell.float()
                O[t] = o; A[t] = a; LP[t] = lp; V[t] = val; RW[t] = rew; DN[t] = fell.float()
                vx_acc += vx.detach().mean(); ar_acc += arate.detach().mean(); vx_n += 1
                ep_ret = ep_ret + rew
                ff = fell.float()
                done_sum += (ep_ret * ff).sum(); done_cnt += ff.sum()
                ep_ret = ep_ret * (1.0 - ff)
                prev_a = torch.where(fell.unsqueeze(1), torch.zeros_like(ac), ac)
                last_a = prev_a
                reset_fields(fell)
                phase_t = torch.where(fell, torch.zeros_like(phase_t), phase_t)
            o, *_ = obs_of(last_a); _, _, lastv = net(o)
            adv = torch.zeros(T, K, device=tdev); gae = torch.zeros(K, device=tdev)
            for t in reversed(range(T)):
                nv_ = lastv if t == T - 1 else V[t + 1]
                nonterm = 1.0 - DN[t]
                delta = RW[t] + gamma * nv_ * nonterm - V[t]
                gae = delta + gamma * lam * nonterm * gae
                adv[t] = gae
            ret = adv + V
        total_steps += T * K
        bO = O.reshape(-1, OBS_DIM); bA = A.reshape(-1, ACT_DIM)
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
            R._log(world, "FREE-GPU it=%d epret~%.1f mvx=%.3f(cmd%.2f) jit=%.4f stdmean=%.2f neps=%d steps/s=%.0f"
                   % (it, mret, mvx, cmd_vx, mar, float(net.log_std.exp().mean()), int(cnt), fps))
            try:
                torch.save(net.state_dict(), path)
                ce = _i("CKPT_EVERY", 0)
                if ce > 0 and it % ce == 0 and it > 0 and path:
                    torch.save(net.state_dict(), path.replace(".pt", "_it%d.pt" % it))
            except Exception:
                pass
            t0 = _time.time(); total_steps = 0
            done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
            vx_acc = torch.zeros((), device=tdev); vx_n = 0; ar_acc = torch.zeros((), device=tdev)


def g1_walk_free_step(world):
    if getattr(world, "_free_started", False):
        return
    if not hasattr(world, "_free_ctr"):
        world._free_ctr = 0
    world._free_ctr += 1
    if world._free_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._free_started = True
    try:
        _free_train_gpu(world)
    except Exception as e:
        import traceback
        R._log(world, "free err %r\n%s" % (e, traceback.format_exc()))


def _free_deploy_setup(world):
    torch = _torch()
    if not WM._build(world):
        return False
    qadr, dadr, aidx, is_leg = _build_fullmap(world)
    world._free_qadr = qadr; world._free_aidx = aidx; world._free_isleg = is_leg
    world._free_legqadr, world._free_legdadr = _build_legmap(world)
    sp = world.solver.mjw_data.qpos.numpy().reshape(-1).copy()
    world._free_nom = sp[qadr].copy(); world._free_nomleg = sp[world._free_legqadr].copy()
    net = _make_ac(_i("PPO_HID", 128))
    net.load_state_dict(torch.load(os.environ.get("RES_POLICY", ""), map_location="cpu"))
    net.eval(); world._free_net = net
    world._free_phase = 0.0; world._free_omega = 2.0 * math.pi * _f("FREE_FREQ", 1.5)
    world._free_dt = world._gwm_dt; world._free_as = _f("FREE_ACT_SCALE", 0.8); world._free_cmd = _f("FREE_CMD_VX", 0.4)
    world._free_last = np.zeros(ACT_DIM, np.float32)
    R._log(world, "FREE DEPLOY ready act_scale=%.2f cmd=%.2f" % (world._free_as, world._free_cmd))
    return True


def g1_walk_free_deploy(world):
    torch = _torch()
    if getattr(world, "_free_dep", None) is None:
        world._free_dep = False
        if _free_deploy_setup(world):
            world._free_dep = True
    if not world._free_dep:
        return
    if not hasattr(world, "_free_dt0"):
        world._free_dt0 = 0
    world._free_dt0 += 1
    if world._free_dt0 < _i("WALK_WARM_TICKS", 360):
        return
    tt = world._free_dt0 - _i("WALK_WARM_TICKS", 360)
    live = world.solver.mjw_data; qp = live.qpos.numpy().reshape(-1); qv = live.qvel.numpy().reshape(-1)
    w, x, y, z = qp[3], qp[4], qp[5], qp[6]
    gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    ph = world._free_phase
    jpos = qp[world._free_legqadr] - world._free_nomleg; jvel = qv[world._free_legdadr]
    o = np.concatenate([[gx, gy, gz], 0.25 * qv[3:6], [world._free_cmd], jpos, 0.1 * jvel, world._free_last,
                        [math.sin(ph), math.cos(ph)], [math.sin(yaw), math.cos(yaw)]]).astype(np.float32)
    with torch.no_grad():
        mean, _std, _v = world._free_net(torch.from_numpy(o[None, :]))
    ac = np.clip(mean.numpy()[0], -1, 1); world._free_last = ac.astype(np.float32)
    env_a = ac * world._free_as
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_q.numpy()
    qa = world._free_qadr; aid = world._free_aidx; isleg = world._free_isleg
    li = 0
    dofmap = [int(world._gwm_dof[p]) for p in range(len(qa))]
    for p in range(len(qa)):
        nd = dofmap[p]
        base = float(world._free_nom[p])
        if isleg[p]:
            base += float(env_a[li]); li += 1
        if 0 <= nd < len(tp):
            tp[nd] = base
    world.control.joint_target_q.assign(tp); world._mjc_dirty = True
    world._free_phase += world._free_omega * world._free_dt * _f("PPO_DEPLOY_PHASE_SCALE", 1.0)
    if tt % 20 == 1:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pit = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        R._log(world, "free deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f" % (tt, qp[0], qp[1], qp[2], roll, pit))
