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

"""AMP (Adversarial Motion Priors) in-engine trainer -- the GENERAL "imitate any motion" system.

The fix for the two walls we hit: (1) from-scratch reward RL can't discover a gait; (2) hard-track
shadowing of a kinematic ghost is impossible because the ghost is dynamically infeasible. AMP (Peng
2021) replaces hard pose-tracking with a STYLE reward: a discriminator D(phi_t, phi_{t+1}) learns
"does this transition look like the REFERENCE motion?", and the policy is rewarded for RESEMBLING the
reference distribution while staying physically valid -- it is FREE to find its own feasible execution.
An infeasible reference still yields a feasible policy. This is how the field trains "any motion".

Pieces on top of the 44x GPU-resident free-policy core (g1_walk_free):
  - motion features phi = [tilt gx,gy, leg jpos(12), leg jvel(12)] = 26  (heading-invariant style, NO task)
  - discriminator D: MLP(2*26 -> hid -> hid -> 1), LSGAN (ref->+1, policy->-1) + gradient penalty
  - style reward r_style = max(0, 1 - 0.25*(D(phi,phi')-1)^2); reward = w_style*r_style + alive - upright
  - reference: a SIMPLE procedural motion to VALIDATE the pipeline first (AMP_MOTION=squat|march),
    generated directly on the G1 (no MoCap/retarget yet); swap in AMASS-retargeted clips later.

Knobs: AMP_MOTION(squat) AMP_AMP(0.30) AMP_FREQ(1.0) W_STYLE(2.0) W_ALIVE(0.5) W_UP(4) W_ARATE(1.0)
  DISC_HID(256) DISC_LR(1e-4) DISC_GP(5.0) DISC_EPOCHS(2) FREE_ACT_SCALE(0.6) PPO_* OBS_NOISE IC_RAND_*
  FALL_* PPO_LOGSTD_MAX CKPT_EVERY PPO_DEPLOY_PHASE_SCALE RES_POLICY.
Launch: OMNISIM_INENGINE_PYMOD=projects.policies.research.training.g1_amp:g1_amp_step
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
from projects.policies.training.g1_walk_free import _build_fullmap

ACT_DIM = 12
# policy obs: proj_g(3)+ang_vel(3)+jpos(12)+jvel(12)+last_act(12)+phase(2) = 44
OBS_DIM = 3 + 3 + 12 + 12 + 12 + 2
FEAT_DIM = 2 + 12 + 12   # motion-feature phi: [gx,gy, jpos_rel(12), 0.1*jvel(12)] = 26


def _make_ac(hid):
    """Actor-critic with a TUNABLE architecture (POLICY_ARCH = mlp | lstm | gru). The recurrent
    variants give the policy MEMORY (hidden state) -- the hypothesis being that memory helps
    durability/robustness under the deploy latency (it can infer the delayed state). forward returns
    (mean, std, value, hidden); MLP ignores/returns hidden=None so recurrent handling is a no-op."""
    torch = _torch()
    import torch.nn as nn
    arch = os.environ.get("POLICY_ARCH", "mlp"); nlayers = _i("POLICY_LAYERS", 1)

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.arch = arch; s.hid = hid; s.nlayers = nlayers
            if arch in ("lstm", "gru"):
                s.enc = nn.Linear(OBS_DIM, hid)
                s.rnn = (nn.LSTM if arch == "lstm" else nn.GRU)(hid, hid, num_layers=nlayers)
                s.pi = nn.Linear(hid, ACT_DIM); s.vf = nn.Linear(hid, 1)
                nn.init.orthogonal_(s.enc.weight, 1.0); nn.init.zeros_(s.enc.bias)
                nn.init.orthogonal_(s.pi.weight, 0.01); nn.init.zeros_(s.pi.bias)
                nn.init.orthogonal_(s.vf.weight, 1.0); nn.init.zeros_(s.vf.bias)
            else:
                s.pi = nn.Sequential(nn.Linear(OBS_DIM, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                     nn.Linear(hid, ACT_DIM))
                s.vf = nn.Sequential(nn.Linear(OBS_DIM, hid), nn.Tanh(), nn.Linear(hid, hid), nn.Tanh(),
                                     nn.Linear(hid, 1))
                for m in list(s.pi) + list(s.vf):
                    if isinstance(m, nn.Linear):
                        nn.init.orthogonal_(m.weight, 1.0); nn.init.zeros_(m.bias)
                nn.init.orthogonal_(s.pi[-1].weight, 0.01)
            s.log_std = nn.Parameter(torch.full((ACT_DIM,), -1.0))

        def init_hidden(s, K, device):
            if s.arch == "lstm":
                return (torch.zeros(s.nlayers, K, s.hid, device=device),
                        torch.zeros(s.nlayers, K, s.hid, device=device))
            if s.arch == "gru":
                return torch.zeros(s.nlayers, K, s.hid, device=device)
            return None

        def forward(s, o, hidden=None):
            lo = float(os.environ.get("PPO_LOGSTD_MIN", "-3.0")); hi = float(os.environ.get("PPO_LOGSTD_MAX", "0.0"))
            std = s.log_std.clamp(lo, hi).exp()
            if s.arch in ("lstm", "gru"):
                y, h2 = s.rnn(torch.tanh(s.enc(o)).unsqueeze(0), hidden)   # (1,K,obs)->(1,K,hid)
                y = y.squeeze(0)
                return s.pi(y), std, s.vf(y).squeeze(-1), h2
            return s.pi(o), std, s.vf(o).squeeze(-1), None

        def forward_seq(s, o_seq, h0):
            """Full T-window in ONE cuDNN call (fast BPTT for the recurrent update). o_seq: (T,K,obs)."""
            lo = float(os.environ.get("PPO_LOGSTD_MIN", "-3.0")); hi = float(os.environ.get("PPO_LOGSTD_MAX", "0.0"))
            std = s.log_std.clamp(lo, hi).exp()
            y, _ = s.rnn(torch.tanh(s.enc(o_seq)), h0)                     # (T,K,hid)
            return s.pi(y), std, s.vf(y).squeeze(-1)                        # (T,K,act),(act),(T,K)
    return AC()


def _reset_hidden(h, done):
    """Zero the recurrent hidden state for worlds that just terminated (done: bool (K,))."""
    if h is None:
        return None
    m = (~done).float()[None, :, None]
    if isinstance(h, tuple):
        return (h[0] * m, h[1] * m)
    return h * m


def _make_disc(hid):
    torch = _torch()
    import torch.nn as nn

    class Disc(nn.Module):
        def __init__(s):
            super().__init__()
            s.net = nn.Sequential(nn.Linear(2 * FEAT_DIM, hid), nn.ReLU(), nn.Linear(hid, hid), nn.ReLU(),
                                  nn.Linear(hid, 1))
            for m in s.net:
                if isinstance(m, nn.Linear):
                    nn.init.orthogonal_(m.weight, 1.0); nn.init.zeros_(m.bias)

        def forward(s, phi, phi2):
            return s.net(torch.cat([phi, phi2], -1)).squeeze(-1)
    return Disc()


def _gen_reference(motion, nom_leg_np, dt, nb=128):
    """Procedural FEASIBLE reference -> phi sequence (nb, FEAT_DIM). Validates the AMP pipeline
    without MoCap/retargeting. g1 leg slots: hip_pitch 0/6, knee 3/9, ankle_pitch 4/10."""
    A = _f("AMP_AMP", 0.30); freq = _f("AMP_FREQ", 1.0)
    q = np.tile(nom_leg_np[None, :], (nb, 1)).astype(np.float64)   # (nb,12) leg angles
    for i in range(nb):
        ph = 2 * math.pi * freq * (i * dt)
        if motion == "march":
            # alternate single-leg knee lift L/R
            sL = A * max(0.0, math.sin(ph)); sR = A * max(0.0, math.sin(ph + math.pi))
            for (kn, hp, an, s) in [(3, 0, 4, sL), (9, 6, 10, sR)]:
                q[i, kn] += s; q[i, hp] += 0.5 * s; q[i, an] += 0.5 * s
        elif motion == "walk":
            # forward gait STYLE (sagittal): each leg swings the thigh FORWARD + flexes the knee for
            # foot clearance during swing, then EXTENDS + pushes during stance. L/R offset pi.
            # Optional LATERAL pelvis-sway (WALK_ROLL, sign tuned empirically) shifts weight onto the
            # stance leg -- the crux of single-support stepping. WALK_ROLL=0 -> policy must learn the
            # lateral balance itself. Forward-velocity TASK reward drives progress; LSTM+latency=durable.
            roll_s = _f("WALK_ROLL", 0.0) * math.sin(ph)   # sway phase-locked to the step (both hips lean)
            q[i, 1] += roll_s; q[i, 7] += roll_s
            for (kn, hp, an, th) in [(3, 0, 4, ph), (9, 6, 10, ph + math.pi)]:
                sw = max(0.0, math.sin(th))              # swing gate (>0 = swing half of the cycle)
                q[i, kn] += A * sw                       # knee flexes to clear the foot in swing
                q[i, hp] += -0.6 * A * math.sin(th)      # thigh forward in swing, extends (push) in stance
                q[i, an] += 0.25 * A * math.sin(th)      # mild ankle coordination
        else:  # squat: COORDINATED knee-bend that keeps the foot FLAT and the CoM over the feet.
            # Ratios copied from the WORKING deterministic squat (specs/g1_squat.json): at the
            # bottom knee +0.68, hip_pitch -0.41 (~ -0.60*knee), ankle_pitch = -(hip+knee) [flat
            # foot]. The old crude bob used hip/ankle = +0.5*knee -- WRONG SIGN -> CoM slides off
            # the feet on descent, so the policy could only follow it shallow without toppling.
            # This balance-preserving reference lets it go DEEP and stay up (an ACHIEVABLE ghost).
            s = A * (1 - math.cos(ph)) / 2
            hp_s = -0.60 * s
            an_s = -(hp_s + s)                      # flat-foot: ankle = -(hip + knee)
            for (kn, hp, an) in [(3, 0, 4), (9, 6, 10)]:
                q[i, kn] += s; q[i, hp] += hp_s; q[i, an] += an_s
    jpos = q - nom_leg_np[None, :]
    jvel = np.zeros_like(q)
    jvel[:-1] = (q[1:] - q[:-1]) / dt; jvel[-1] = (q[0] - q[-1]) / dt
    tilt = np.zeros((nb, 2))
    phi = np.concatenate([tilt, jpos, 0.1 * jvel], axis=1).astype(np.float32)   # (nb, FEAT_DIM)
    return phi


def _amp_train_gpu(world):
    torch = _torch()
    import warp as wp
    import time as _time
    dev = os.environ.get("PPO_DEVICE", "cuda")
    if dev == "cuda" and not torch.cuda.is_available():
        R._log(world, "AMP: cuda not available"); return
    if not WM._build(world):
        R._log(world, "amp: WM._build failed"); return
    sub = int(getattr(world, "_n_substeps", 4)); dt = world._gwm_dt
    qadr, dadr, aidx, is_leg = _build_fullmap(world)
    leg_qadr, leg_dadr = _build_legmap(world)
    legmask = is_leg.astype(bool)

    K = _i("PPO_NENV", 2048); T = _i("PPO_NSTEPS", 32)
    motion = os.environ.get("AMP_MOTION", "squat")
    free_freq = _f("AMP_FREQ", 1.0); act_scale = _f("FREE_ACT_SCALE", 0.6)
    omega = 2.0 * math.pi * free_freq
    gamma = _f("PPO_GAMMA", 0.99); lam = _f("PPO_LAM", 0.95)
    clip = _f("PPO_CLIP", 0.2); lr = _f("PPO_LR", 3e-4)
    epochs = _i("PPO_EPOCHS", 4); nmb = _i("PPO_MINIB", 8)
    ent_c = _f("PPO_ENT", 0.003); vf_c = _f("PPO_VF", 0.5); iters = _i("PPO_ITERS", 100000)
    fr = _f("FALL_ROLL", 0.7); fp = _f("FALL_PITCH", 0.7); fbz = _f("FALL_BZ", 0.45)
    wstyle = _f("W_STYLE", 2.0); walive = _f("W_ALIVE", 0.5); wup = _f("W_UP", 4.0); warate = _f("W_ARATE", 1.0)
    apen = _f("RES_ACT_PEN", 0.005); fpen = _f("FALL_PEN", 5.0)
    wheight = _f("W_HEIGHT", 0.0); z_tgt = _f("Z_TGT", 0.60)   # floor: stop the deep-crouch collapse
    wvel = _f("W_VEL", 0.0); vx_tgt = _f("VX_TGT", 0.6)         # WALK task: reward fwd base speed -> target
    wyaw = _f("W_YAW", 0.0); wlat = _f("W_LAT", 0.0)            # WALK: keep heading straight, no lateral drift
    icj = _f("IC_RAND_JOINT", 0.03); icv = _f("IC_RAND_VEL", 0.08); ica = _f("IC_RAND_ANG", 0.05)
    obs_noise = _f("OBS_NOISE", 0.0); ctrl_lat = _i("AMP_CTRL_LAT", 0)   # train WITH deploy latency
    lat_max = _i("AMP_LAT_MAX", 0)   # >0: RANDOMIZE latency per-iter in [0,lat_max] (anti-overfit a point)
    eval_every = _i("EVAL_EVERY", 0); eval_H = _i("EVAL_H", 1200); eval_ic = _f("EVAL_IC", 0.02)
    disc_hid = _i("DISC_HID", 256); disc_lr = _f("DISC_LR", 1e-4); disc_gp = _f("DISC_GP", 5.0)
    disc_epochs = _i("DISC_EPOCHS", 2)
    use_graph = _i("PPO_GRAPH", 1) and dev == "cuda"
    path = os.environ.get("RES_POLICY", ""); torch.manual_seed(_i("RES_SEED", 0))

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "amp: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    qpos_t = wp.to_torch(rd.qpos).view(K, nq)
    qvel_t = wp.to_torch(rd.qvel).view(K, nv)
    ctrl_t = wp.to_torch(rd.ctrl).view(K, nu)
    tdev = qpos_t.device

    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    spawn_t = torch.tensor(spawn, dtype=torch.float32, device=tdev)
    nom_all = torch.tensor(spawn[qadr], dtype=torch.float32, device=tdev)
    nom_leg_np = spawn[leg_qadr].copy()
    nom_leg = torch.tensor(nom_leg_np, dtype=torch.float32, device=tdev)
    qadr_t = torch.tensor(qadr, dtype=torch.long, device=tdev)
    aidx_t = torch.tensor(aidx, dtype=torch.long, device=tdev)
    leg_qadr_t = torch.tensor(leg_qadr, dtype=torch.long, device=tdev)
    leg_dadr_t = torch.tensor(leg_dadr, dtype=torch.long, device=tdev)
    legmask_t = torch.tensor(legmask, dtype=torch.bool, device=tdev)

    # reference motion -> phi transitions (on GPU)
    ref_phi = _gen_reference(motion, nom_leg_np, dt * 1.0, nb=_i("AMP_NB", 128))
    ref_phi_t = torch.tensor(ref_phi, device=tdev)
    ref_a = ref_phi_t[:-1]; ref_b = ref_phi_t[1:]            # (NB-1, FEAT) transition pairs
    R._log(world, "AMP-GPU: K=%d T=%d obs=%d feat=%d motion=%s amp=%.2f freq=%.2f graph=%d ref_frames=%d"
           % (K, T, OBS_DIM, FEAT_DIM, motion, _f("AMP_AMP", 0.30), free_freq, int(bool(use_graph)), ref_phi.shape[0]))
    # self-describing CONFIG line so the experiment ledger can record what produced this curve
    _pfx = ("AMP_", "PPO_", "W_", "DISC_", "EVAL_", "FREE_", "OBS_NOISE", "IC_RAND", "RES_ACT",
            "Z_TGT", "FALL_", "VTRACK", "POLICY_")
    _cfg = {k: os.environ[k] for k in sorted(os.environ) if any(k.startswith(p) for p in _pfx)}
    R._log(world, "CONFIG " + " ".join("%s=%s" % (k, v) for k, v in _cfg.items()))

    net = _make_ac(_i("PPO_HID", 128)).to(tdev)
    disc = _make_disc(disc_hid).to(tdev)
    if path and os.path.exists(path):
        try:
            net.load_state_dict(torch.load(path, map_location=tdev)); R._log(world, "amp: resumed %s" % path)
        except Exception:
            pass
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    dopt = torch.optim.Adam(disc.parameters(), lr=disc_lr)

    phase_t = torch.zeros(K, device=tdev)

    def reset_fields(mask):
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, device=tdev) * icj
        rq[:, 4:7] += torch.randn(K, 3, device=tdev) * ica
        rv = torch.zeros(K, nv, device=tdev)
        rv[:, 0:2] = torch.randn(K, 2, device=tdev) * icv; rv[:, 3:5] = torch.randn(K, 2, device=tdev) * icv
        m = mask.unsqueeze(1)
        qpos_t.copy_(torch.where(m, rq, qpos_t)); qvel_t.copy_(torch.where(m, rv, qvel_t))

    def feat_of():
        qp = qpos_t; qv = qvel_t
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x)
        jpos = qp[:, leg_qadr_t] - nom_leg; jvel = qv[:, leg_dadr_t]
        return torch.clamp(torch.cat([torch.stack([gx, gy], 1), jpos, 0.1 * jvel], 1), -10.0, 10.0)

    def obs_of(last_a, ph):
        qp = qpos_t; qv = qvel_t
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1, 1))
        jpos = qp[:, leg_qadr_t] - nom_leg; jvel = qv[:, leg_dadr_t]
        o = torch.cat([torch.stack([gx, gy, gz], 1), 0.25 * qv[:, 3:6], jpos, 0.1 * jvel, last_a,
                       torch.stack([torch.sin(ph), torch.cos(ph)], 1)], 1)
        return torch.clamp(o, -10.0, 10.0), roll, pitch, qp[:, 2]

    def style_reward(phi, phi2):
        d = disc(phi, phi2)
        return torch.clamp(1 - 0.25 * (d - 1) ** 2, min=0.0)

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
            R._log(world, "AMP-GPU: CUDA-graph captured")
        except Exception as e:
            R._log(world, "AMP-GPU: graph capture failed (%s)" % e); step_graph = None

    def physics():
        if step_graph is not None:
            wp.capture_launch(step_graph)
        else:
            for _ in range(sub):
                _mjw.step(rm, rd)

    def deploy_eval():
        """DETERMINISTIC DEPLOY PREDICTOR. Runs the MEAN policy (no sampling -- like deploy) for
        eval_H steps (long horizon -- like a real demo) from a FIXED-seed deploy-like IC, and
        measures: surv (fraction of horizon survived before falling = the #1 deploy number),
        fall (fraction that fell), sat (action-saturation fraction = over-drive/collapse risk),
        fidel (motion fidelity under deterministic execution), drift (lateral RMS). Reproducible
        -> a hard good/bad gate. NOTE: clobbers the rollout buffers; caller re-seeds training after."""
        g = torch.Generator(device=tdev).manual_seed(987654321)
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, generator=g, device=tdev) * eval_ic
        qpos_t.copy_(rq); qvel_t.copy_(torch.zeros(K, nv, device=tdev))
        _mjw.forward(rm, rd); wp.synchronize()
        ph = torch.zeros(K, device=tdev); la = torch.zeros(K, ACT_DIM, device=tdev)
        alive = torch.full((K,), float(eval_H), device=tdev); done = torch.zeros(K, dtype=torch.bool, device=tdev)
        sat = torch.zeros((), device=tdev); fid = torch.zeros((), device=tdev); livesteps = torch.zeros((), device=tdev)
        y0 = qpos_t[:, 1].clone(); x0 = qpos_t[:, 0].clone()   # x0 -> forward distance walked
        zmin = torch.full((K,), 1e9, device=tdev); zmax = torch.full((K,), -1e9, device=tdev)
        lat = _i("AMP_CTRL_LAT", 0)
        ebuf = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(lat)]   # control-latency ring (deploy-match)
        heval = net.init_hidden(K, tdev)
        for t in range(eval_H):
            phi = feat_of()
            o, _r, _p, _z = obs_of(la, ph)
            mean, _std, _v, heval = net(o, heval)
            ac = torch.clamp(mean, -1, 1); env_a = ac * act_scale
            if lat > 0:
                ebuf.append(env_a); env_a = ebuf.pop(0)              # apply target delayed by `lat` ticks
            tgt = nom_all.unsqueeze(0).repeat(K, 1); tgt[:, legmask_t] = nom_leg + env_a
            ctrl_t[:, aidx_t] = tgt; ctrl_t[:, aidx_t + 1] = 0.0
            physics(); ph.add_(omega * dt)
            phi2 = feat_of()
            _, roll, pitch, bz = obs_of(ac, ph)
            m = ~done                                              # alive coming into this step
            zmin = torch.where(m, torch.minimum(zmin, bz), zmin)   # pelvis-z excursion -> squat DEPTH
            zmax = torch.where(m, torch.maximum(zmax, bz), zmax)
            live = (~done).float()
            sat += ((ac.abs() > 0.95).float().mean(1) * live).sum()
            fid += (style_reward(phi, phi2) * live).sum(); livesteps += live.sum()
            fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz)
            newly = fell & (~done)
            alive = torch.where(newly, torch.full_like(alive, float(t)), alive)
            done = done | fell; la = ac
            if bool(done.all()):
                break
        ls = float(livesteps)
        drift = float((((qpos_t[:, 1] - y0) ** 2).mean()).sqrt())
        # depth = pelvis-z excursion, but ONLY over worlds that survived the whole horizon --
        # a FALLING robot also has a huge excursion, so unmasked depth is fall-contaminated.
        surv_mask = ~done
        valid = surv_mask & (zmax > zmin)
        depth = float((zmax[valid] - zmin[valid]).mean()) if bool(valid.any()) else 0.0
        # fwd = forward distance walked, over survivors only (a faller's x is meaningless)
        fwd = float((qpos_t[surv_mask, 0] - x0[surv_mask]).mean()) if bool(surv_mask.any()) else 0.0
        return (float(alive.mean()) / eval_H, float(done.float().mean()),
                float(sat) / max(1.0, ls), float(fid) / max(1.0, ls), drift, depth, fwd)

    ep_ret = torch.zeros(K, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
    last_a = torch.zeros(K, ACT_DIM, device=tdev)
    ebuf_tr = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(ctrl_lat)]   # training control-latency ring
    hstate = net.init_hidden(K, tdev)   # recurrent hidden (None for MLP), carried across the rollout

    def _detach_h(h):
        if h is None:
            return None
        return (h[0].detach(), h[1].detach()) if isinstance(h, tuple) else h.detach()
    sty_acc = torch.zeros((), device=tdev); n_acc = 0
    done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
    O = torch.zeros(T, K, OBS_DIM, device=tdev); A = torch.zeros(T, K, ACT_DIM, device=tdev)
    LP = torch.zeros(T, K, device=tdev); V = torch.zeros(T, K, device=tdev)
    RW = torch.zeros(T, K, device=tdev); DN = torch.zeros(T, K, device=tdev)
    PHI = torch.zeros(T, K, FEAT_DIM, device=tdev); PHI2 = torch.zeros(T, K, FEAT_DIM, device=tdev)
    t0 = _time.time(); total_steps = 0

    if _i("EVAL_ONLY", 0):
        with torch.no_grad():
            surv, frate, satv, fidv, drv, dep, fwdv = deploy_eval()
        R._log(world, "DEPLOY-EVAL-ONLY ckpt surv=%.3f fall=%.3f sat=%.3f fidel=%.3f drift=%.2f depth=%.3f fwd=%.3f H=%d"
               % (surv, frate, satv, fidv, drv, dep, fwdv, eval_H))
        return

    for it in range(iters):
        if lat_max > 0:
            lat_this = int(torch.randint(0, lat_max + 1, (1,)).item())       # randomized latency this iter
            ebuf_tr = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(lat_this)]
        else:
            lat_this = ctrl_lat
        h0_stored = _detach_h(hstate)   # hidden at the start of this T-window (for BPTT replay)
        with torch.no_grad():
            for t in range(T):
                phi = feat_of()
                o, roll, pitch, bz = obs_of(last_a, phase_t)
                oin = o + torch.randn_like(o) * obs_noise if obs_noise > 0 else o
                mean, std, val, hstate = net(oin, hstate)
                dist = torch.distributions.Normal(mean, std)
                a = dist.sample(); lp = dist.log_prob(a).sum(-1)
                ac = torch.clamp(a, -1, 1); env_a = ac * act_scale
                if lat_this > 0:
                    ebuf_tr.append(env_a); env_a = ebuf_tr.pop(0)        # apply delayed (deploy-matched)
                tgt = nom_all.unsqueeze(0).repeat(K, 1)
                tgt[:, legmask_t] = nom_leg + env_a
                ctrl_t[:, aidx_t] = tgt; ctrl_t[:, aidx_t + 1] = 0.0
                physics()
                phase_t.add_(omega * dt)
                phi2 = feat_of()
                arate = ((ac - prev_a) ** 2).sum(1)
                _, roll, pitch, bz = obs_of(ac, phase_t)
                rsty = style_reward(phi, phi2)
                upr = roll * roll + pitch * pitch
                sink = torch.clamp(z_tgt - bz, min=0.0)
                # WALK task terms (all default-off -> squat/march unchanged): reward forward base
                # speed toward target, penalize yaw drift + lateral velocity to keep it straight.
                vx = qvel_t[:, 0]; vy = qvel_t[:, 1]
                qw, qx, qy, qz = qpos_t[:, 3], qpos_t[:, 4], qpos_t[:, 5], qpos_t[:, 6]
                yaw = torch.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                # LINEAR forward reward: ~0 at standstill (no reward for just standing), grows with
                # forward speed up to the target, negative for backward. (An exp(-(vx-tgt)^2) shaping
                # pays ~half its max at vx=0 -> the policy just STANDS; linear removes that trap.)
                rvel = wvel * torch.clamp(vx, min=-0.3, max=vx_tgt)
                rew = (wstyle * rsty + walive + rvel - wup * upr - warate * arate
                       - apen * (env_a * env_a).sum(1) - wheight * sink * sink
                       - wyaw * yaw * yaw - wlat * vy * vy)
                fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz)
                rew = rew - fpen * fell.float()
                O[t] = oin; A[t] = a; LP[t] = lp; V[t] = val; RW[t] = rew; DN[t] = fell.float()
                PHI[t] = phi; PHI2[t] = phi2
                sty_acc += rsty.mean(); n_acc += 1
                ep_ret = ep_ret + rew
                ff = fell.float()
                done_sum += (ep_ret * ff).sum(); done_cnt += ff.sum()
                ep_ret = ep_ret * (1.0 - ff)
                prev_a = torch.where(fell.unsqueeze(1), torch.zeros_like(ac), ac); last_a = prev_a
                reset_fields(fell)
                phase_t = torch.where(fell, torch.zeros_like(phase_t), phase_t)
                # (hidden is NOT reset on fall: rollout + BPTT update both treat the T-window as a
                #  continuous sequence -> consistent -> PPO-correct; the LSTM state leak across an
                #  episode boundary decays and is a standard recurrent-PPO approximation.)
            o, *_ = obs_of(last_a, phase_t); _, _, lastv, _ = net(o, hstate)
            adv = torch.zeros(T, K, device=tdev); gae = torch.zeros(K, device=tdev)
            for t in reversed(range(T)):
                nv_ = lastv if t == T - 1 else V[t + 1]
                nonterm = 1.0 - DN[t]
                delta = RW[t] + gamma * nv_ * nonterm - V[t]
                gae = delta + gamma * lam * nonterm * gae
                adv[t] = gae
            ret = adv + V
        total_steps += T * K
        # ---- discriminator update (LSGAN + gradient penalty) ----
        polA = PHI.reshape(-1, FEAT_DIM); polB = PHI2.reshape(-1, FEAT_DIM)
        Np = polA.shape[0]; nref = ref_a.shape[0]
        last_dref = torch.zeros((), device=tdev); last_dpol = torch.zeros((), device=tdev)
        for _de in range(disc_epochs):
            idx = torch.randint(0, Np, (min(4096, Np),), device=tdev)
            ridx = torch.randint(0, nref, (min(4096, Np),), device=tdev)
            ra, rb = ref_a[ridx], ref_b[ridx]
            ra.requires_grad_(True); rb.requires_grad_(True)
            d_ref = disc(ra, rb); d_pol = disc(polA[idx], polB[idx])
            loss_ref = ((d_ref - 1) ** 2).mean(); loss_pol = ((d_pol + 1) ** 2).mean()
            grad = torch.autograd.grad(d_ref.sum(), [ra, rb], create_graph=True)
            gp = (grad[0].pow(2).sum(-1) + grad[1].pow(2).sum(-1)).mean()
            dloss = 0.5 * (loss_ref + loss_pol) + disc_gp * gp
            dopt.zero_grad(); dloss.backward(); dopt.step()
            last_dref = d_ref.mean().detach(); last_dpol = d_pol.mean().detach()
        # ---- PPO update ----
        adv_n = (adv - adv.mean()) / (adv.std() + 1e-8)   # (T,K)
        is_rnn = net.arch in ("lstm", "gru")
        if not is_rnn:
            bO = O.reshape(-1, OBS_DIM); bA = A.reshape(-1, ACT_DIM)
            bLP = LP.reshape(-1); bRET = ret.reshape(-1); bADV = adv_n.reshape(-1)
            Nb = bO.shape[0]
            for _e in range(epochs):
                perm = torch.randperm(Nb, device=tdev)
                for mb in perm.chunk(nmb):
                    mean, std, val, _ = net(bO[mb])
                    dist = torch.distributions.Normal(mean, std)
                    lp = dist.log_prob(bA[mb]).sum(-1); ent = dist.entropy().sum(-1).mean()
                    ratio = torch.exp(lp - bLP[mb]); a_ = bADV[mb]
                    pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                    vl = ((val - bRET[mb]) ** 2).mean()
                    loss = pl + vf_c * vl - ent_c * ent
                    if not torch.isfinite(loss):
                        continue                                # skip a blown-up minibatch (stability guard)
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
        else:
            # RECURRENT PPO: minibatch over WORLDS; replay each T-window in ONE cuDNN call from the
            # stored window-start hidden (truncated BPTT, length T). Fast (no Python step loop).
            for _e in range(epochs):
                perm = torch.randperm(K, device=tdev)
                for mb in perm.chunk(nmb):
                    if isinstance(h0_stored, tuple):
                        h0 = (h0_stored[0][:, mb].contiguous(), h0_stored[1][:, mb].contiguous())
                    else:
                        h0 = h0_stored[:, mb].contiguous()
                    mean, std, val = net.forward_seq(O[:, mb], h0)          # (T,|mb|,act),(act),(T,|mb|)
                    dist = torch.distributions.Normal(mean, std)
                    lp = dist.log_prob(A[:, mb]).sum(-1); ent = dist.entropy().sum(-1).mean()
                    ratio = torch.exp(lp - LP[:, mb]); a_ = adv_n[:, mb]
                    pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                    vl = ((val - ret[:, mb]) ** 2).mean()
                    loss = pl + vf_c * vl - ent_c * ent
                    if not torch.isfinite(loss):
                        continue
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
        if it % 10 == 0:
            cnt = float(done_cnt); mret = float(done_sum / done_cnt) if cnt > 0 else 0.0
            fps = total_steps / max(1e-6, _time.time() - t0); msty = float(sty_acc) / max(1, n_acc)
            R._log(world, "AMP-GPU it=%d epret~%.1f style=%.3f Dref=%.2f Dpol=%.2f stdmean=%.2f neps=%d steps/s=%.0f"
                   % (it, mret, msty, float(last_dref), float(last_dpol), float(net.log_std.exp().mean()), int(cnt), fps))
            try:
                torch.save(net.state_dict(), path)
                ce = _i("CKPT_EVERY", 0)
                if ce > 0 and it % ce == 0 and it > 0 and path:
                    torch.save(net.state_dict(), path.replace(".pt", "_it%d.pt" % it))
            except Exception:
                pass
            t0 = _time.time(); total_steps = 0
            done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
            sty_acc = torch.zeros((), device=tdev); n_acc = 0
            if eval_every > 0 and it > 0 and it % eval_every == 0:
                with torch.no_grad():
                    surv, frate, satv, fidv, drv, dep, fwdv = deploy_eval()
                R._log(world, "  DEPLOY-EVAL it=%d surv=%.3f fall=%.3f sat=%.3f fidel=%.3f drift=%.2f depth=%.3f fwd=%.3f"
                       % (it, surv, frate, satv, fidv, drv, dep, fwdv))
                reset_fields(torch.ones(K, dtype=torch.bool, device=tdev)); _mjw.forward(rm, rd)
                phase_t = torch.zeros(K, device=tdev); ep_ret = torch.zeros(K, device=tdev)
                last_a = torch.zeros(K, ACT_DIM, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)


def g1_amp_step(world):
    if getattr(world, "_amp_started", False):
        return
    if not hasattr(world, "_amp_ctr"):
        world._amp_ctr = 0
    world._amp_ctr += 1
    if world._amp_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._amp_started = True
    try:
        _amp_train_gpu(world)
    except Exception as e:
        import traceback
        R._log(world, "amp err %r\n%s" % (e, traceback.format_exc()))


def _amp_deploy_setup(world):
    torch = _torch()
    if not WM._build(world):
        return False
    qadr, dadr, aidx, is_leg = _build_fullmap(world)
    world._amp_qadr = qadr; world._amp_isleg = is_leg
    world._amp_legqadr, world._amp_legdadr = _build_legmap(world)
    sp = world.solver.mjw_data.qpos.numpy().reshape(-1).copy()
    world._amp_nom = sp[qadr].copy(); world._amp_nomleg = sp[world._amp_legqadr].copy()
    net = _make_ac(_i("PPO_HID", 128))
    net.load_state_dict(torch.load(os.environ.get("RES_POLICY", ""), map_location="cpu"))
    net.eval(); world._amp_net = net; world._amp_h = net.init_hidden(1, "cpu")
    world._amp_phase = 0.0; world._amp_omega = 2.0 * math.pi * _f("AMP_FREQ", 1.0)
    world._amp_dt = world._gwm_dt; world._amp_as = _f("FREE_ACT_SCALE", 0.6); world._amp_last = np.zeros(ACT_DIM, np.float32)
    world._amp_gdof = [int(world._gwm_dof[p]) for p in range(len(qadr))]
    R._log(world, "AMP DEPLOY ready motion=%s" % os.environ.get("AMP_MOTION", "squat"))
    return True


def g1_amp_deploy(world):
    torch = _torch()
    if getattr(world, "_amp_dep", None) is None:
        world._amp_dep = False
        if _amp_deploy_setup(world):
            world._amp_dep = True
    if not world._amp_dep:
        return
    if not hasattr(world, "_amp_dt0"):
        world._amp_dt0 = 0
    world._amp_dt0 += 1
    if world._amp_dt0 < _i("WALK_WARM_TICKS", 360):
        return
    tt = world._amp_dt0 - _i("WALK_WARM_TICKS", 360)
    live = world.solver.mjw_data; qp = live.qpos.numpy().reshape(-1); qv = live.qvel.numpy().reshape(-1)
    w, x, y, z = qp[3], qp[4], qp[5], qp[6]
    gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
    ph = world._amp_phase
    jpos = qp[world._amp_legqadr] - world._amp_nomleg; jvel = qv[world._amp_legdadr]
    o = np.concatenate([[gx, gy, gz], 0.25 * qv[3:6], jpos, 0.1 * jvel, world._amp_last,
                        [math.sin(ph), math.cos(ph)]]).astype(np.float32)
    with torch.no_grad():
        mean, std, _v, world._amp_h = world._amp_net(torch.from_numpy(o[None, :]), world._amp_h)
        if _i("AMP_DEPLOY_STOCH", 0):
            mean = mean + std * torch.randn_like(std)              # diagnostic: deploy stochastically
    ac = np.clip(mean.numpy()[0], -1, 1); world._amp_last = ac.astype(np.float32)
    env_a = ac * world._amp_as
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_q.numpy()
    qa = world._amp_qadr; isleg = world._amp_isleg; gdof = world._amp_gdof
    li = 0
    for p in range(len(qa)):
        nd = gdof[p]; base = float(world._amp_nom[p])
        if isleg[p]:
            base += float(env_a[li]); li += 1
        if 0 <= nd < len(tp):
            tp[nd] = base
    world.control.joint_target_q.assign(tp); world._mjc_dirty = True
    world._amp_phase += world._amp_omega * world._amp_dt * _f("PPO_DEPLOY_PHASE_SCALE", 1.0)
    if tt % 20 == 1:
        R._log(world, "amp deploy t=%d x=%.2f z=%.3f knee_off=%.3f" % (tt, qp[0], qp[2], float(env_a[3])))
