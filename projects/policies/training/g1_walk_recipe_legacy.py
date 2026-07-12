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

"""IN-ENGINE humanoid walk trained on the CONSENSUS deploy-proven RECIPE (Unitree G1 / MuJoCo
Playground / Booster / Berkeley), not the AMP-hand-reference approach that could not walk.

Why this file exists: a 10-paper literature sweep (2026-07-01) found that NOBODY makes a humanoid
walk by imitating a reference for basic locomotion. The deploy-proven recipe is FROM-SCRATCH RL with
a specific structure, and our earlier AMP-style-on-a-hand-gait had the wrong mechanism + was missing
the load-bearing ingredients. This trainer implements the recipe, IN OmniSim's own mujoco_warp engine
(train == deploy, zero sim gap), reusing the batched GPU scaffolding from g1_walk_ppo.

THE RECIPE (what this implements):
  - ACTION = joint POSITION targets over the model PD, offset from the DEFAULT standing pose
    (legs = nom_leg + a * act_scale). NO deterministic-gait base -- the gait EMERGES from the reward.
  - ACTOR OBS (47-dim, proprioception only, NO base linear velocity -- unmeasurable on HW):
    base ang-vel(3), projected gravity(3), velocity COMMAND(3), joint pos(12), joint vel(12),
    prev action(12), gait-phase sin/cos(2).
  - ASYMMETRIC critic: the value net ALSO sees privileged state (base linear velocity, foot heights,
    foot contact) that the actor never gets.
  - REWARD = exp velocity-command TRACKING (lin+ang) + a foot-height-vs-PHASE gait reward (the
    stepping-maker: each foot tracks a clock-driven target height, anti-phase L/R) + feet-slip
    penalty + orientation + base-height + action-rate + alive + termination.
  - DOMAIN RANDOMIZATION for durability: external PUSHES (velocity kicks), per-world MOTOR-STRENGTH
    scaling, observation noise, randomized control LATENCY (the top-credited sim-to-real lever, which
    we independently found is the train->deploy root cause), and IC randomization.
  - VELOCITY-COMMAND CURRICULUM: sample vx per episode, widen the range as the policy improves.
  - MEASURED by deploy_eval (deterministic mean policy, long horizon) -> surv / fall / fwd / vtrack,
    the same deploy-prediction gate we built for the squat.

Knobs (defaults track Unitree G1 where sensible):
  RES_ACT_SCALE(0.5) PPO_HID(256) VX_START(0.3) VX_MAX(0.7) VX_CURR_ITERS(1500) VTRACK_SIG(0.25)
  W_TRACK_LIN(1.5) W_TRACK_ANG(0.5) W_FEET(1.5) FEET_SIG(0.06) SWING_H(0.10) W_SLIP(0.4) W_UP(1.0)
  W_HEIGHT(20) Z_TGT(0.72) W_ARATE(0.8) RES_ACT_PEN(0.01) W_ALIVE(0.5) FALL_PEN(50)
  PUSH_INTERVAL(120) PUSH_VEL(1.0) MOTOR_RAND(0.08) OBS_NOISE(0.02) AMP_CTRL_LAT(0) AMP_LAT_MAX(6)
  IC_RAND_* FALL_* EVAL_EVERY EVAL_H CKPT_EVERY RES_POLICY.
Launch: OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step
Deploy: ...g1_walk_recipe:g1_walk_recipe_deploy
  (!) run_walk_rl.sh DEFAULTS to the legacy ES trainer -- this module only runs if you pass
  the OMNISIM_INENGINE_PYMOD override above. Startup tell-tales that THIS trainer is live:
  "GHOST=RECORDED ...", "GHOST-SEQ: timed sequence mode ..." and "WALK-GPU it=..." in RES_LOG.
  If you see "TRAIN start pop=16 ... N_PARAM=144" instead, the launcher ran the legacy
  trainer and every env this file reads was silently ignored (launcher guard added
  2026-07-04; see projects/policies/training/README.md "Trainer selection").
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
from projects.policies.training import ref_obs as _refobs   # community-tracker Phase 1: reference-in-obs

ACT_DIM = 12
# actor obs: angvel(3)+projg(3)+cmd(3)+jpos(12)+jvel(12)+preva(12)+phase(2)+heading(2) = 49
# heading = sin/cos of world yaw: projected gravity is YAW-BLIND, so without this the policy can't
# sense which way it faces -> unbounded heading drift (it curves sideways and falls). This is the fix.
OBS_DIM = 3 + 3 + 3 + 12 + 12 + 12 + 2 + 2
# privileged extras the CRITIC also gets: base linvel(3)+foot z(2)+foot contact(2) = 7
PRIV_DIM = 3 + 2 + 2


def _make_ac(hid):
    """Asymmetric actor-critic. ACTOR: POLICY_ARCH = mlp | lstm | gru. The recurrent variants give the
    actor MEMORY (hidden state) so it can compensate the deploy CONTROL LATENCY -- a reactive MLP can't
    on a small-margin foot (ablation: LSTM surv~1.0 vs MLP~0.34 under ~6-tick latency). CRITIC: ALWAYS
    an MLP over OBS_DIM+PRIV_DIM (privileged sim-only state -> low-variance value; nothing leaks to the
    deployed actor). act()/act_seq() carry the actor hidden; value() is stateless. [512,256,128] ELU."""
    torch = _torch()
    import torch.nn as nn
    arch = os.environ.get("POLICY_ARCH", "mlp"); nlayers = _i("POLICY_LAYERS", 1)

    def mlp(din, dout, last_gain):
        m = nn.Sequential(nn.Linear(din, 512), nn.ELU(), nn.Linear(512, 256), nn.ELU(),
                          nn.Linear(256, 128), nn.ELU(), nn.Linear(128, dout))
        for layer in m:
            if isinstance(layer, nn.Linear):
                nn.init.orthogonal_(layer.weight, 1.0); nn.init.zeros_(layer.bias)
        nn.init.orthogonal_(m[-1].weight, last_gain)
        return m

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.arch = arch; s.hid = hid; s.nlayers = nlayers
            if arch in ("lstm", "gru"):
                s.enc = nn.Linear(OBS_DIM, hid)
                s.rnn = (nn.LSTM if arch == "lstm" else nn.GRU)(hid, hid, num_layers=nlayers)
                s.pi = nn.Linear(hid, ACT_DIM)
                nn.init.orthogonal_(s.enc.weight, 1.0); nn.init.zeros_(s.enc.bias)
                nn.init.orthogonal_(s.pi.weight, 0.01); nn.init.zeros_(s.pi.bias)
            else:
                s.pi = mlp(OBS_DIM, ACT_DIM, 0.01)          # small initial policy
            s.vf = mlp(OBS_DIM + PRIV_DIM, 1, 1.0)          # critic ALWAYS MLP (asymmetric, sees priv)
            s.log_std = nn.Parameter(torch.full((ACT_DIM,), -1.0))

        def std(s):
            lo = float(os.environ.get("PPO_LOGSTD_MIN", "-2.0"))
            hi = float(os.environ.get("PPO_LOGSTD_MAX", "0.0"))
            return s.log_std.clamp(lo, hi).exp()

        def init_hidden(s, K, device):
            if s.arch == "lstm":
                return (torch.zeros(s.nlayers, K, s.hid, device=device),
                        torch.zeros(s.nlayers, K, s.hid, device=device))
            if s.arch == "gru":
                return torch.zeros(s.nlayers, K, s.hid, device=device)
            return None

        def act(s, o, ha=None):
            if s.arch in ("lstm", "gru"):
                y, ha2 = s.rnn(torch.tanh(s.enc(o)).unsqueeze(0), ha)   # (1,K,obs) -> (1,K,hid)
                return s.pi(y.squeeze(0)), s.std(), ha2
            return s.pi(o), s.std(), None

        def act_seq(s, o_seq, h0):
            """Full T-window in ONE cuDNN call (fast BPTT). o_seq (T,K,obs) -> (mean(T,K,act), std)."""
            y, _ = s.rnn(torch.tanh(s.enc(o_seq)), h0)
            return s.pi(y), s.std()

        def value(s, o, priv):
            return s.vf(torch.cat([o, priv], -1)).squeeze(-1)
    return AC()


def _detach_h(h):
    if h is None:
        return None
    return (h[0].detach(), h[1].detach()) if isinstance(h, tuple) else h.detach()


def _seed_legs():
    """The seeded leg pose (spawn + nominal anchor). STAND_POSE=unitree -> the OFFICIAL model's
    DEFAULT (hip -0.1, knee 0.3, ankle -0.2): the posture the RECORDED ghost cycles around.
    Anchoring the policy's nominal to the ghost's nominal removes the ~0.2 rad persistent
    posture offset that floored gmatch (the mimicry metric) regardless of reward weight.
    Default = the classic deep crouch (hip -0.30, knee 0.52) that bootstrapped the stand."""
    if os.environ.get("STAND_POSE", "") == "unitree":
        return np.array([-0.10, 0.0, 0.0, 0.30, -0.20, 0.0,
                         -0.10, 0.0, 0.0, 0.30, -0.20, 0.0], np.float32)
    return np.array([-0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
                     -0.30, 0.0, 0.0, 0.52, -0.23, 0.0], np.float32)


def _footmap(world):
    """-> (nbody, [foot_body_id_0, foot_body_id_1]) from the gait builder's foot map."""
    nbody = int(world.solver.mj_model.nbody)
    fb = world._gwm_foot                                   # {side: body_id}
    ids = [int(v) for v in fb.values()]
    if len(ids) < 2:                                       # fallback: duplicate (won't happen for G1)
        ids = (ids + ids)[:2]
    return nbody, ids[:2]


def _build_all_pos_act(world):
    """ALL position-servo actuators (all joints incl ARMS) -> (qadr, dadr, aidx). The gait maps only
    cover legs+waist; the arms need this direct model enumeration for TRUE whole-body control. A
    position servo carries its kp in actuator_biasprm[1] (velocity servos put kv in biasprm[2])."""
    mjm = world.solver.mj_model
    nu = int(mjm.nu)
    trnid = np.asarray(mjm.actuator_trnid).reshape(nu, -1)
    bp = np.asarray(mjm.actuator_biasprm).reshape(nu, -1)
    qadr = []; dadr = []; aidx = []
    for a in range(nu):
        if bp.shape[1] > 1 and abs(float(bp[a][1])) < 1e-12:   # skip non-position (velocity/motor) actuators
            continue
        jid = int(trnid[a][0])
        if jid < 0:
            continue
        qadr.append(int(mjm.jnt_qposadr[jid])); dadr.append(int(mjm.jnt_dofadr[jid])); aidx.append(a)
    return np.array(qadr, np.int32), np.array(dadr, np.int32), np.array(aidx, np.int32)


def _walk_recipe_train(world):
    torch = _torch()
    import warp as wp
    import time as _time
    dev = os.environ.get("PPO_DEVICE", "cuda")
    if dev == "cuda" and not torch.cuda.is_available():
        R._log(world, "walk-recipe: cuda not available"); return
    if not WM._build(world):
        R._log(world, "walk-recipe: WM._build failed"); return
    _wr_patch_foot_torsion(world)   # TRAIN with foot spin friction if enabled (turning needs it)
    slots = world._gwm_slots; act = world._gwm_act
    dt = world._gwm_dt; sub = int(getattr(world, "_n_substeps", 4))
    leg_pos = [p for p, sl in enumerate(slots) if sl < 12]
    leg_act = np.array([int(act[p]) for p in leg_pos], np.int32)
    waist_pos = [p for p, sl in enumerate(slots) if sl >= 12]
    waist_act = np.array([int(act[p]) for p in waist_pos], np.int32)
    leg_qadr, leg_dadr = _build_legmap(world)
    # WHOLE_BODY: the actor controls ALL actuated joints (legs + waist + ARMS) instead of legs-only.
    # A real G1 balances a narrow 6cm foot with whole-body control (ankle + hip + arm angular momentum);
    # legs-only can only briefly hold it. The ghost/feet rewards still key off the leg subset (leg_qadr).
    whole = _i("WHOLE_BODY", 0)
    if whole:
        qadr_use, dadr_use, act_use = _build_all_pos_act(world)
        R._log(world, "WHOLE_BODY: %d position actuators (nu=%d) incl arms; leg_act=%s"
               % (len(act_use), int(world.solver.mj_model.nu), np.asarray(leg_act).tolist()))
    else:
        qadr_use = np.asarray(leg_qadr, np.int32); dadr_use = np.asarray(leg_dadr, np.int32); act_use = leg_act
    global ACT_DIM, OBS_DIM
    ACT_DIM = int(len(act_use)); OBS_DIM = 13 + 3 * ACT_DIM   # angvel3+projg3+cmd3+jpos+jvel+preva+phase2+head2

    K = _i("PPO_NENV", 2048); T = _i("PPO_NSTEPS", 32)
    act_scale = _f("RES_ACT_SCALE", 0.5)
    gamma = _f("PPO_GAMMA", 0.99); lam = _f("PPO_LAM", 0.95)
    clip = _f("PPO_CLIP", 0.2); lr = _f("PPO_LR", 3e-4)
    epochs = _i("PPO_EPOCHS", 4); nmb = _i("PPO_MINIB", 8)
    ent_c = _f("PPO_ENT", 0.005); vf_c = _f("PPO_VF", 0.5); iters = _i("PPO_ITERS", 100000)
    fr = _f("FALL_ROLL", 0.8); fp = _f("FALL_PITCH", 0.8); fbz = _f("FALL_BZ", 0.45)
    # reward weights (recipe)
    wtl = _f("W_TRACK_LIN", 1.5); wta = _f("W_TRACK_ANG", 0.5); vtsig = _f("VTRACK_SIG", 0.25)
    # STANDSTILL penalty: the tent reward is 0 at fwd=0, which makes "stand still, never fall" a
    # comfortable local optimum (arm3/arm4 both drifted into it: surv 0.93 with dist ~1). This term
    # makes ignoring a forward command actively NEGATIVE: shortfall below the commanded speed costs
    # up to -W_VSHORT/step at a standstill. Zero-command worlds are unaffected (shortfall 0).
    wvshort = _f("W_VSHORT", 0.0)
    wypos = _f("W_YPOS", 1.0); ypos_dead = _f("YPOS_DEAD", 0.3)   # anti-drift: penalize |y| only BEYOND a
    # deadband -> the ~+-0.3m lateral stepping sway (which walking REQUIRES) is free; only real DRIFT is punished.
    w_wobble = _f("W_WOBBLE", 0.05); w_bounce = _f("W_BOUNCE", 2.0)   # anti-wobble: damp base roll/pitch/yaw
    # angular RATE + vertical bounce -> a smoother, less rocky gait (Unitree/Playground ang_vel_xy + lin_vel_z).
    wfeet = _f("W_FEET", 1.0); feet_sig = _f("FEET_SIG", 0.06); swing_h = _f("SWING_H", 0.10)
    wghost = _f("W_GHOST", 2.5); ghost_sig = _f("GHOST_SIG", 0.4)   # soft NUDGE toward the walking-gait ghost
    w_wb = _f("W_WB", 0.0); wb_sig = _f("WB_SIG", 0.5)   # community-tracker P2: whole-body imitation (arms/waist/elbows track wb_lut)
    mtrack = _i("MTRACK", 0); w_kp = _f("W_KP", 6.0); kp_sig = _f("KP_SIG", 0.15)   # MTRACK: keypoint (feet/hands pos) tracking, balance EMERGENT
    wup = _f("W_UP", 1.0)
    wheight = _f("W_HEIGHT", 20.0); z_tgt = _f("Z_TGT", 0.74)
    warate = _f("W_ARATE", 0.8); apen = _f("RES_ACT_PEN", 0.01)
    walive = _f("W_ALIVE", 0.5); fpen = _f("FALL_PEN", 50.0)
    # velocity command + curriculum
    vx_start = _f("VX_START", 0.3); vx_max = _f("VX_MAX", 0.7); vx_curr = _i("VX_CURR_ITERS", 1500)
    # domain randomization
    push_int = _i("PUSH_INTERVAL", 120); push_vel = _f("PUSH_VEL", 1.0)
    motor_rand = _f("MOTOR_RAND", 0.08); obs_noise = _f("OBS_NOISE", 0.02)
    ctrl_lat = _i("AMP_CTRL_LAT", 0); lat_max = _i("AMP_LAT_MAX", 6)
    icj = _f("IC_RAND_JOINT", 0.03); icv = _f("IC_RAND_VEL", 0.08); ica = _f("IC_RAND_ANG", 0.05)
    # EVAL_H is LONG on purpose: a walk is only "good" if it survives long enough to cover 10-20 m.
    # At ~0.5 m/s a 10 m walk is thousands of control steps -> a short horizon can't judge it (it would
    # pass a policy that walks 3 m then falls). Default 3000; raise to 5000+ to demand a 20 m walk.
    eval_every = _i("EVAL_EVERY", 0); eval_H = _i("EVAL_H", 3000); eval_ic = _f("EVAL_IC", 0.02)
    use_graph = _i("PPO_GRAPH", 1) and dev == "cuda"
    path = os.environ.get("RES_POLICY", ""); torch.manual_seed(_i("RES_SEED", 0))

    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        R._log(world, "walk-recipe: no rollout buffers"); return
    _mjw, rm, rd, (nq, nv, nu) = buf
    # ── DT PARITY (train/eval == live deploy) ──────────────────────────────────
    # The batched buffers advance rm.opt.timestep per _mjw.step (sub steps per action).
    # The LIVE deploy advances sub_dt = dt/n_substeps per solver substep (sub substeps per
    # tick = dt of physics per action). If rm.opt.timestep != sub_dt, the training/eval
    # physics-time-per-action != the deploy's -> a policy tuned to one timescale can't hold
    # balance at the other (indistinguishable from control latency). Force them equal.
    def _ts_get():   # rm.opt.timestep may be a scalar, a warp array (needs .numpy()), or numpy
        x = rm.opt.timestep
        for conv in (lambda v: float(v),
                     lambda v: float(v.numpy().reshape(-1)[0]),
                     lambda v: float(np.asarray(v).reshape(-1)[0])):
            try:
                return conv(x)
            except Exception:
                continue
        try:   # last resort: the compiled CPU model is authoritative and always scalar
            return float(world.solver.mj_model.opt.timestep)
        except Exception:
            return -1.0
    _model_ts = _ts_get()
    if not (1e-5 < _model_ts < 0.1):   # never let a failed read produce a runaway substep count
        _model_ts = float(world.solver.mj_model.opt.timestep)
    # ── DT PARITY (THE 2026-07-02 root-cause fix, v2: EXACT live discretization) ──
    # The LIVE engine advances basicTimeStep (0.016 s) of physics per hook tick, as
    # n_substeps(4) calls to SolverMuJoCo.step(dt=0.004) -- and SolverMuJoCo does
    # `mjw_model.opt.timestep.fill_(dt)` then ONE mjw step per call (newton
    # solver_mujoco.py:3485). So live == 4 mjw steps x 0.004 s. The old trainer stepped
    # 4 x model_ts(0.002) = 0.008 s/action -- HALF the live physics per control step; that
    # factor 2 was the entire "~6-tick latency" / "soft live plant" saga. And 8 x 0.002,
    # while the right TOTAL time, is the wrong DISCRETIZATION (close but not equal).
    # Proof: the tick-aligned open-loop probe (g1_latency_probe, seed at t=1) at
    # LP_SUB=4 LP_TS=0.004 matches the live world BIT-IDENTICALLY at every tick
    # (hold, step response, even the collapse) -- machine-precision train==deploy.
    _tick_s = _f("WALK_TICK_S", 0.016)          # the engine tick (WorldInfo.basicTimeStep / 1000)
    sub = min(64, max(1, _i("WALK_SUB", int(getattr(world, "_n_substeps", 4)))))
    _sd = _tick_s / sub                          # live sub_dt (0.004): the ACTUAL mjw integration step
    try:
        _cur = rm.opt.timestep
        if hasattr(_cur, "fill_"):
            _cur.fill_(_sd)
        elif hasattr(_cur, "shape") and getattr(_cur, "shape", ()) != ():
            _cur[...] = _sd
        else:
            rm.opt.timestep = _sd
    except Exception as _e:
        R._log(world, "DT-PARITY ts-set err %r" % (_e,))
    R._log(world, "DT-PARITY live-exact: %d mjw steps x ts=%.4f = %.4f s/action == live s/tick (model_ts was %.4f)"
           % (sub, _sd, sub * _sd, _model_ts))
    qpos_t = wp.to_torch(rd.qpos).view(K, nq)
    qvel_t = wp.to_torch(rd.qvel).view(K, nv)
    ctrl_t = wp.to_torch(rd.ctrl).view(K, nu)
    tdev = qpos_t.device
    nbody, foot_ids = _footmap(world)
    xpos_t = wp.to_torch(rd.xpos).view(K, nbody, 3)
    try:                                   # WBMATCH v3: body orientations (wxyz), if this mjw exposes them
        xquat_t = wp.to_torch(rd.xquat).view(K, nbody, 4)
    except Exception:
        xquat_t = None
    fL, fRi = int(foot_ids[0]), int(foot_ids[1])

    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    # STAND_SEED: force a KNOWN-GOOD crouched-stand pose (G1 nominal: hip -0.30, knee 0.52,
    # ankle -0.23) as the reset pose, independent of the deterministic controller. On small feet the
    # det. stand collapses during warmup -> RL inherited a falling pose (surv=0, backward eject). A
    # forced upright seed gives RL a survivable start so it can LEARN active balance (real-robot style).
    if _i("STAND_SEED", 0):
        stand_leg = _seed_legs()   # slot order per leg (STAND_POSE selects the anchor posture)
        spawn[leg_qadr] = stand_leg
        spawn[2] = _f("STAND_Z", 0.72)
    spawn_t = torch.tensor(spawn, dtype=torch.float32, device=tdev)
    nom_leg = torch.tensor(spawn[leg_qadr], dtype=torch.float32, device=tdev)
    leg_qadr_t = torch.tensor(leg_qadr, dtype=torch.long, device=tdev)
    leg_dadr_t = torch.tensor(leg_dadr, dtype=torch.long, device=tdev)
    leg_act_t = torch.tensor(leg_act, dtype=torch.long, device=tdev)
    waist_act_t = torch.tensor(waist_act, dtype=torch.long, device=tdev)
    # ── UNITREE_LEG_PD: give the LEGS the official plant (kp=[100,100,100,150,40,40] kd=[2,2,2,4,2,2],
    # Unitree's torque-PD as position-servo gains; paired velocity servos OFF). The official LSTM's
    # closed loop was learned against THESE gains -- under the recipe's stiff uniform servo its
    # rollouts are a different dynamical system (the corrupted-label root cause candidate).
    try:
        # PLANT ECHO (2026-07-06 stride-gap audit): the knee actuator's kp/kd as the batched
        # model ACTUALLY carries them right now -- gain writes (UNITREE_LEG_PD) could persist
        # across jobserver jobs via cached buffers, silently changing the training plant.
        _bpE = np.asarray(rm.actuator_biasprm.numpy()).reshape(-1, np.asarray(world.solver.mj_model.actuator_biasprm).reshape(int(world.solver.mj_model.nu), -1).shape[1])
        _aK = int(leg_act[3])
        R._log(world, "PLANT-ECHO knee actuator: kp=%.1f kd=%.1f (vel-servo kv=%.1f)"
               % (-float(_bpE[_aK][1]), -float(_bpE[_aK][2]),
                  float(np.asarray(rm.actuator_gainprm.numpy()).reshape(_bpE.shape[0], -1)[_aK + 1][0]) if _aK + 1 < _bpE.shape[0] else -1.0))
    except Exception as _eE:
        R._log(world, "PLANT-ECHO err %r" % (_eE,))
    if _i("UNITREE_LEG_PD", 0):
        try:
            _mjmU = world.solver.mj_model
            _nuU = int(_mjmU.nu)
            _bpU = np.asarray(_mjmU.actuator_biasprm).reshape(_nuU, -1).copy()
            _gpU = np.asarray(_mjmU.actuator_gainprm).reshape(_nuU, -1).copy()
            _kpu = [100.0, 100.0, 100.0, 150.0, 40.0, 40.0] * 2
            _kdu = [2.0, 2.0, 2.0, 4.0, 2.0, 2.0] * 2
            for _iU, _aU in enumerate(np.asarray(leg_act).tolist()):
                _gpU[_aU][0] = _kpu[_iU]; _bpU[_aU][1] = -_kpu[_iU]; _bpU[_aU][2] = -_kdu[_iU]
                if _aU + 1 < _nuU and abs(float(_bpU[_aU + 1][1])) < 1e-12:  # paired velocity servo -> off
                    _gpU[_aU + 1][0] = 0.0; _bpU[_aU + 1][2] = 0.0
            rm.actuator_gainprm.assign(_gpU.reshape(rm.actuator_gainprm.numpy().shape))
            rm.actuator_biasprm.assign(_bpU.reshape(rm.actuator_biasprm.numpy().shape))
            R._log(world, "UNITREE-LEG-PD: legs kp=%s kd=%s, vel-servos off (authentic official plant)"
                   % (_kpu[:6], _kdu[:6]))
        except Exception as _eU:
            R._log(world, "UNITREE-LEG-PD err %r" % (_eU,))
    # joint-use set for obs/action (all joints if WHOLE_BODY, else legs); ghost/feet keep leg_qadr_t
    nom_use = torch.tensor(spawn[qadr_use], dtype=torch.float32, device=tdev)
    qadr_use_t = torch.tensor(qadr_use, dtype=torch.long, device=tdev)
    dadr_use_t = torch.tensor(dadr_use, dtype=torch.long, device=tdev)
    act_use_t = torch.tensor(act_use, dtype=torch.long, device=tdev)
    # hold the waist/torso actuators at their spawn-nominal targets (legs-only walk policy)
    # GHOST: the deterministic gait's per-phase leg angles = a kinematic WALKING reference. We do
    # NOT track it hard (impossible for a balancing biped) -- a soft, generous-sigma reward NUDGES
    # the legs toward this stepping pattern, which kills the "dive forward + faceplant" reward-hack
    # (a faceplant's legs are nowhere near the ghost -> ~0 ghost reward). Legs keep full authority.
    NB = 32; phase0 = world._gwm_phase0
    glut = np.zeros((NB, len(slots)))
    for bb in range(NB):
        legs, _a, _s = world._gwm_stg.targets_np(phase0 + 2 * math.pi * bb / NB, world._gwm_gp, t_since_start=10.0)
        glut[bb] = np.asarray(legs)[[slots[p] for p in range(len(slots))]]
    ghost_leg_t = torch.tensor(glut[:, leg_pos], dtype=torch.float32, device=tdev)   # (NB,12) ghost leg angles / phase
    waist_nom_t = torch.tensor(glut[0, waist_pos], dtype=torch.float32, device=tdev)
    # ── RECORD-REPLAY GHOST (owner-approved 2026-07-02) ────────────────────────
    # GHOST_LUT_JSON: replace the hand-designed deterministic gait with a RECORDED cycle of the
    # OFFICIAL Unitree G1 policy walking in this engine (56 cycles phase-folded, 2 mrad spread).
    # Achievable BY CONSTRUCTION (a balancing G1 executed every pose here), smooth, and at the
    # real cadence (1.25 Hz). Overrides: ghost table, gait clock omega (also used by the deploy
    # hook via world._gwm_omega), swing gates (knee-derived, replacing the sin() clock gates),
    # and the arm ghost (opposite-hip-locked, replacing pure sin) -- all from the same recording.
    lut_swing = None; ghost_att_t = None; lut_arm_np = None; ghost_wb_t = None; ref_kp_t = None; kp_ids_t = None
    _glj = os.environ.get("GHOST_LUT_JSON")
    if _glj:
        import json as _json
        _gd = _json.loads(open(_glj).read())
        _lut = np.asarray(_gd["leg_lut"], np.float32)          # (nb, 12) slot order L6+R6
        NB = int(_gd["nb"])
        ghost_leg_t = torch.tensor(_lut, dtype=torch.float32, device=tdev)
        # GHOST v3 (full-body achieved ghost): the LUT may also carry the recorded SHOULDERS
        # (arm_lut, [left,right] absolute rad) and base ATTITUDE (att_lut, [roll,pitch]) --
        # then the arms track the recording directly and WBMATCH judges attitude against the
        # ghost's own sway instead of vs-level (phase-locked sway earns credit, jitter doesn't).
        if "arm_lut" in _gd:
            lut_arm_np = np.asarray(_gd["arm_lut"], np.float32)          # (nb,2)
        if "att_lut" in _gd:
            ghost_att_t = torch.tensor(np.asarray(_gd["att_lut"], np.float32), device=tdev)  # (nb,2)
        if "wb_lut" in _gd:   # (nb,23) whole-body pose in wb_joints order -> imitation reward + whole-body ref
            ghost_wb_t = torch.tensor(np.asarray(_gd["wb_lut"], np.float32), device=tdev)
        if "kp_lut" in _gd:   # (nb, n_kp*3) body-frame keypoints [L_foot,R_foot,L_hand,R_hand]
            ref_kp_t = torch.tensor(np.asarray(_gd["kp_lut"], np.float32), device=tdev)
            # Newton renames bodies (body_5..) so name lookup fails; body ids are FK-MATCHED at it=0
            # (set the sim to the reference pose, find the sim body nearest each reference keypoint).
        omega_lut = 2.0 * math.pi * float(_gd.get("freq", 1.25))
        world._gwm_omega = omega_lut                            # deploy hook reads this too
        # swing gates from the recorded KNEE (flexes for clearance in swing): 0=stance, 1=mid-swing
        def _gate(col):
            k = _lut[:, col]
            return np.clip((k - k.min()) / max(1e-6, (k.max() - k.min())), 0.0, 1.0)
        lut_swing = (torch.tensor(_gate(3), dtype=torch.float32, device=tdev),
                     torch.tensor(_gate(9), dtype=torch.float32, device=tdev))
        R._log(world, "GHOST=RECORDED %s: nb=%d freq=%.2fHz omega=%.3f (clock overridden)"
               % (os.path.basename(_glj), NB, float(_gd.get("freq", 1.25)), omega_lut))
    # ── GHOST-MORPH (2026-07-03): SNAP swaps of the control feedforward collapse a warm-started
    # walker (bisection: v3e-ref-only surv .035, elbow-only surv .25) -- the policy is specialized
    # to its exact references. GHOST_MORPH_JSON=<target> interpolates the CONTROL tables from the
    # loaded (native) ghost to the target over MORPH_ITERS iterations (alpha 0->1, then holds),
    # so the gait is never more than a hair from one the policy masters. Metric should be pinned
    # to the TARGET via GHOST_METRIC_JSON so the score tracks the final goal throughout.
    # ── GHOST-SEQ (2026-07-04): TIMED SEQUENCES (dance -- motion that CHANGES across time).
    # A sequence is a gait with a long, asymmetric cycle: same clock/corridor machinery, plus
    # (1) spawn ON the reference at phase 0 (reset pose == lut(0), base z from root_lut) so the
    #     per-world phase reset in reset_fields keeps pose<->plot consistent;
    # (2) a TIME-VARYING command: cmd_t is fed per tick from the root trajectory's derivative
    #     (body-frame vx, vy + yaw rate) -- the proven velocity-tracking reward/obs machinery
    #     learns "when the routine says lunge, make the contacts that lunge".
    seq_mode = bool(_i("GHOST_SEQ", 0)) and (_glj is not None) and ("root_lut" in _gd)
    # SEQ_TERRAIN (stair-climb 2026-07-08): a rising/terrain sequence (root_lut z varies with
    # forward progress) needs (a) the height reward to track the ghost's RISING root z instead of
    # a fixed Z_TGT that would penalize the climb, and (b) RSI to reset base X *with* Z so a
    # mid-climb respawn lands ON the terrain (the recipe zeroes spawn x, so a z-only respawn would
    # float above the flat floor). Opt-in: flat sequences (dances, walk-stop-walk) are unaffected.
    seq_terrain = seq_mode and bool(_i("SEQ_TERRAIN", 0))
    # PROGRESS-LOCK leash (stair-climb 2026-07-08): cap how far (in bins) the CLOCK reference may run
    # ahead of the robot's ACTUAL position on the climb (base-x mapped to a bin through the ghost's
    # monotonic root-x). 0 = off (pure clock). >0 = the reference WAITS for a slow climber instead of
    # racing to the top out of reach -- kills the clock-lag that stalled the climb past ~2 treads.
    # The lead must exceed the flat settle length so the start isn't deadlocked (settle x is constant).
    _leashlead = _f("SEQ_LEASH_LEAD", 0.0) if seq_terrain else 0.0
    # GHOST-TUBE ET (2026-07-08): the leash CAPS the reference to the robot's x (deadlocks the start);
    # the tube instead TERMINATES an episode when the body sinks GHOST_TUBE_DZ below the phase-advanced
    # ghost climb height. As the phase rises, a robot that cycles its feet at the base (high gmatch,
    # zero climb) drops out of the tube and dies -> "stand at the base" stops being survivable, so the
    # policy must actually step UP to stay alive. Also a clean propulsion probe: if the robot physically
    # can't climb, every world terminates early. Off by default (0.0).
    _tube_dz = _f("GHOST_TUBE_DZ", 0.0) if seq_terrain else 0.0
    seq_cmd_t = None
    if seq_mode:
        _rl = np.asarray(_gd["root_lut"], np.float32)              # (NB,4) x y z yaw
        _dtb = (1.0 / float(_gd.get("freq", 1.25))) / NB
        _vw = (np.roll(_rl, -1, axis=0) - _rl) / _dtb
        _vw[-1] = _vw[-2]                                          # loop seam: no teleport command
        _cy2, _sy2 = np.cos(_rl[:, 3]), np.sin(_rl[:, 3])
        _vbx = _cy2 * _vw[:, 0] + _sy2 * _vw[:, 1]                 # body-frame forward
        _vby = -_sy2 * _vw[:, 0] + _cy2 * _vw[:, 1]                # body-frame lateral
        seq_cmd_np = np.clip(np.stack([_vbx, _vby, _vw[:, 3]], 1), -1.5, 1.5)
        seq_cmd_np[-4:] = seq_cmd_np[-5]        # loop seam: the wrap jump is display, not a command
        seq_cmd_t = torch.tensor(seq_cmd_np, dtype=torch.float32, device=tdev)
        # SEQ TURN FRAME (2026-07-07): the harness lateral-catch + velocity rewards live in the
        # TARGET-heading frame (ytgt). For a TURNING sequence ytgt must follow the reference yaw,
        # else as the robot turns the harness frame stays put and FIGHTS the turn -> NaN (measured
        # twice: turntrack1/2 diverged at it~50-60). Drive ytgt from the reference root yaw.
        seq_yaw_t = torch.tensor(_rl[:, 3], dtype=torch.float32, device=tdev)
        # HOLD_END in TRAINING (2026-07-07): a walk-turn-walk sequence does NOT loop (starts yaw 0,
        # ends -99deg). Cyclic phase wrap snaps the reference (and ytgt) -99->0 at the seam -> NaN
        # (measured: turntrack1/2/3 all diverged at it~50-60). Pin the phase at the final bin so the
        # routine plays ONCE then holds the end pose (the deploy already does this; mirror it here).
        seq_hold_end = bool(_gd.get("hold_end", False))
        _tp5_seq = 2.0 * math.pi * (1.0 - 0.5 / NB)
        spawn[leg_qadr] = _lut[0]                                  # spawn ON the reference
        # +2 cm spawn clearance: the lut's grounding calibration puts soles flush with the floor;
        # teleport-spawning EXACTLY at contact depth on the warm solver explodes in ~6 ticks
        # (the suction-work teleport-bounce trap). Gravity closes 2 cm in ~60 ms harmlessly.
        spawn[2] = float(_rl[0, 2]) + 0.02
        spawn_t = torch.tensor(spawn, dtype=torch.float32, device=tdev)
        nom_leg = torch.tensor(spawn[leg_qadr], dtype=torch.float32, device=tdev)
        # RSI (reference-state initialization, DeepMimic-style): tables for resetting a world at a
        # RANDOM point of the routine in the ghost's pose AND velocity -- without it the policy
        # over-practices the opening and starves at the first hard move (measured: surv flat 0.13,
        # every world dying at the same ~1.1 s kick for 300 iters).
        seq_rootz_t = torch.tensor(_rl[:, 2] + 0.02, dtype=torch.float32, device=tdev)  # RSI clearance
        seq_rootx_t = torch.tensor(_rl[:, 0], dtype=torch.float32, device=tdev)  # SEQ_TERRAIN: RSI base x (respawn on the climb)
        _rootx_lo = float(np.min(_rl[:, 0])); _rootx_hi = float(np.max(_rl[:, 0]))  # PROGRESS-LOCK leash clamp bounds
        seq_yaw_t = torch.tensor(_rl[:, 3], dtype=torch.float32, device=tdev)
        _lv = (np.roll(_lut, -1, axis=0) - _lut) / _dtb
        _lv[-4:] = _lv[-5]                       # loop seam
        seq_legvel_t = torch.tensor(np.clip(_lv, -12, 12), dtype=torch.float32, device=tdev)
        seq_cmdraw_t = torch.tensor(seq_cmd_np, dtype=torch.float32, device=tdev)  # body-frame vx,vy,wz
        leg_dadr_t2 = torch.tensor(leg_dadr, dtype=torch.long, device=tdev)
        R._log(world, "GHOST-SEQ: timed sequence mode -- spawn=lut(0) z=%.3f, cmd=root-velocity profile "
                      "(|vx|<=%.2f |vy|<=%.2f |wz|<=%.2f)" % (spawn[2], np.abs(seq_cmd_np[:, 0]).max(),
                      np.abs(seq_cmd_np[:, 1]).max(), np.abs(seq_cmd_np[:, 2]).max()))
    seq_lo = _i("SEQ_BIN_LO", 0); seq_hi = _i("SEQ_BIN_HI", 0)
    seq_win = seq_mode and (seq_hi > seq_lo)
    if seq_win:
        R._log(world, "GHOST-SEQ WINDOW: segment-isolation bins [%d,%d) of %d" % (seq_lo, seq_hi, NB))
    morph = None; morph_iters = _i("MORPH_ITERS", 400)
    _gmorph = os.environ.get("GHOST_MORPH_JSON")
    if _gmorph and _glj:
        _gt = _json.loads(open(_gmorph).read())
        NBT = int(_gt["nb"])
        def _resamp(a, nbt):
            a = np.asarray(a, np.float32)
            idx = np.arange(nbt, dtype=np.float32) * (a.shape[0] / float(nbt))
            i0 = idx.astype(int) % a.shape[0]; i1 = (i0 + 1) % a.shape[0]
            w = (idx - idx.astype(int))[:, None]
            return a[i0] * (1 - w) + a[i1] * w
        _src_leg = _resamp(_lut, NBT)
        _tgt_leg = np.asarray(_gt["leg_lut"], np.float32)
        morph = {"tgt": _gt, "src_leg": _src_leg, "tgt_leg": _tgt_leg}
        # CADENCE MORPH (hum70/hum40 lesson 2026-07-05): a fixed clock over a growing stride is
        # SELF-INCONSISTENT (surv fell monotonically with morph fraction, robot dragged at 0.26
        # m/s). If the target lut carries a different freq, interpolate omega with the tables.
        morph["omega_src"] = float(world._gwm_omega)
        morph["omega_tgt"] = 2.0 * math.pi * float(_gt.get("freq", world._gwm_omega / (2.0 * math.pi)))
        NB = NBT
        ghost_leg_t = torch.tensor(_src_leg, dtype=torch.float32, device=tdev)   # start = native
        lut_swing = (torch.tensor(np.clip((_src_leg[:, 3] - _src_leg[:, 3].min()) / max(1e-6, float(np.ptp(_src_leg[:, 3]))), 0, 1), dtype=torch.float32, device=tdev),
                     torch.tensor(np.clip((_src_leg[:, 9] - _src_leg[:, 9].min()) / max(1e-6, float(np.ptp(_src_leg[:, 9]))), 0, 1), dtype=torch.float32, device=tdev))
        R._log(world, "GHOST-MORPH: %s -> %s over %d iters (nb=%d)"
               % (os.path.basename(_glj), os.path.basename(_gmorph), morph_iters, NB))
    # ── ARM-SWING GHOST (owner request 2026-07-02) ─────────────────────────────
    # The ghost was LEGS-ONLY, so the arms had no reference and became free balancing flails
    # (gmatch low, awkward look). A natural gait SWINGS the arms anti-phase with the SAME-side
    # leg (left arm forward while the right leg swings) -- that is angular-momentum cancellation,
    # so it should help balance as well as look right. Shoulder-pitch only, on the same gait
    # clock as the legs (left-leg swing gate = sin(phase), matching the foot reward): arm forward
    # = NEGATIVE shoulder pitch, so left_sp = nom + A*sin(phase), right_sp = nom - A*sin(phase).
    # Elbows/rolls stay at nominal. Same nudge philosophy as the leg ghost: soft sigma, no hard track.
    w_armg = _f("W_ARMGHOST", 0.0); armg_sig = _f("ARMGHOST_SIG", 0.3); arm_A = _f("ARM_SWING_A", 0.3)
    # W_ATTGHOST: track the (metric) ghost's base ATTITUDE cycle -- the calm-sway fine-tune lever.
    # Complements W_UP (level pull): this rewards matching the ghost's phase-locked sway exactly,
    # which is what the WBMATCH attitude component measures. 0 = off (exact legacy reward).
    w_attg = _f("W_ATTGHOST", 0.0); attg_sig = _f("ATTGHOST_SIG", 0.15)
    arm_qadr = []; arm_sign = []; elb_qadr = []; elb_sign = []
    if whole and w_armg > 0:
        try:
            import mujoco as _mjn
            _mjm = world.solver.mj_model
            _trn = np.asarray(_mjm.actuator_trnid).reshape(int(_mjm.nu), -1)
            for a in act_use:
                jid = int(_trn[int(a)][0])
                nm = (_mjn.mj_id2name(_mjm, _mjn.mjtObj.mjOBJ_JOINT, jid) or "")
                if "shoulder_pitch" in nm:
                    arm_qadr.append(int(_mjm.jnt_qposadr[jid]))
                    arm_sign.append(1.0 if "left" in nm.lower() else -1.0)
                elif "elbow" in nm:
                    elb_qadr.append(int(_mjm.jnt_qposadr[jid]))
                    elb_sign.append(1.0 if "left" in nm.lower() else -1.0)
            if len(arm_qadr) != 2:
                # names don't survive the newton->MJCF conversion -> GEOMETRIC fallback (same
                # technique as the leg map): shoulder-pitch = the HIGHEST pitch-axis (y) joint on
                # each side ABOVE the pelvis; the ELBOW is the next pitch joint down the arm
                # (~11cm above pelvis, so the height filter is +0.05 not +0.15).
                arm_qadr = []; arm_sign = []; elb_qadr = []; elb_sign = []
                _dcl = _mjn.MjData(_mjm); _mjn.mj_forward(_mjm, _dcl)
                _pelz = 0.7
                for j in range(int(_mjm.njnt)):
                    if int(_mjm.jnt_type[j]) == int(_mjn.mjtJoint.mjJNT_FREE):
                        _pelz = float(_dcl.xpos[int(_mjm.jnt_bodyid[j])][2]); break
                _act_j = set(int(_trn[int(a)][0]) for a in act_use)
                _cands = {1.0: [], -1.0: []}   # side (sign of y) -> [(z, jid)]
                for j in _act_j:
                    ax = np.abs(np.array(_mjm.jnt_axis[j], float))
                    pos = np.array(_dcl.xpos[int(_mjm.jnt_bodyid[j])], float)
                    if int(np.argmax(ax)) == 1 and pos[2] > _pelz + 0.05 and abs(pos[1]) > 0.05:
                        _cands[1.0 if pos[1] > 0 else -1.0].append((float(pos[2]), j))
                for _sd in (1.0, -1.0):
                    if _cands[_sd]:
                        _srt = sorted(_cands[_sd], reverse=True)
                        _z, _j = _srt[0]                   # highest pitch joint on that side = shoulder
                        arm_qadr.append(int(_mjm.jnt_qposadr[_j]))
                        arm_sign.append(_sd)               # +y side = left
                        if len(_srt) > 1:
                            _z2, _j2 = _srt[1]             # next pitch joint down = elbow
                            elb_qadr.append(int(_mjm.jnt_qposadr[_j2]))
                            elb_sign.append(_sd)
            R._log(world, "ARM-GHOST shoulder_pitch qadr=%s sign=%s A=%.2f sig=%.2f w=%.2f"
                   % (arm_qadr, arm_sign, arm_A, armg_sig, w_armg))
        except Exception as _e:
            R._log(world, "ARM-GHOST err %r" % (_e,))
    use_armg = bool(whole and w_armg > 0 and len(arm_qadr) == 2)
    elb_qadr_t2 = torch.tensor(elb_qadr, dtype=torch.long, device=tdev) if len(elb_qadr) == 2 else None
    if use_armg:
        arm_qadr_t = torch.tensor(arm_qadr, dtype=torch.long, device=tdev)
        nom_arm = torch.tensor(spawn[np.array(arm_qadr)], dtype=torch.float32, device=tdev)
        if _glj and lut_arm_np is not None:
            # GHOST v3: shoulders recorded ALONGSIDE the legs (the champion's own swing,
            # harmonic-smoothed) -- absolute angles, recorder order [left(+y), right(-y)];
            # reorder to arm_qadr's side order (arm_sign[0] > 0 <=> index 0 is left).
            _al = torch.tensor(lut_arm_np, dtype=torch.float32, device=tdev)
            ghost_arm_t = _al if arm_sign[0] > 0 else _al[:, [1, 0]]
        elif _glj:
            # RECORDED arm ghost: shoulder pitch proportional to the OPPOSITE side's recorded
            # hip-pitch deviation (left arm forward with the right leg) -- phase-locked to the
            # recorded gait by construction, identical to the preview the owner approved.
            _hl = ghost_leg_t[:, 0]; _hr = ghost_leg_t[:, 6]
            _dl = (_hl - _hl.mean()) / torch.clamp((_hl.max() - _hl.min()) / 2, min=1e-6)
            _dr = (_hr - _hr.mean()) / torch.clamp((_hr.max() - _hr.min()) / 2, min=1e-6)
            _l_sh = arm_A * torch.clamp(_dr, -1, 1)            # arm_sign order: [left, right]?
            _r_sh = arm_A * torch.clamp(_dl, -1, 1)
            _cols = [_l_sh, _r_sh] if arm_sign[0] > 0 else [_r_sh, _l_sh]
            ghost_arm_t = nom_arm.unsqueeze(0) + torch.stack(_cols, 1)          # (NB,2)
        else:
            _phb = torch.arange(NB, dtype=torch.float32, device=tdev) * (2 * math.pi / NB)
            _sgn = torch.tensor(arm_sign, dtype=torch.float32, device=tdev)
            ghost_arm_t = nom_arm.unsqueeze(0) + arm_A * torch.sin(_phb).unsqueeze(1) * _sgn.unsqueeze(0)  # (NB,2)
        if morph is not None and "arm_lut" in morph["tgt"]:
            # arm morph: native table (nom-centered hip-locked or recorded) -> target recorded arms
            morph["src_arm"] = ghost_arm_t.clone()
            _ta = torch.tensor(np.asarray(morph["tgt"]["arm_lut"], np.float32), device=tdev)
            morph["tgt_arm"] = _ta if arm_sign[0] > 0 else _ta[:, [1, 0]]
    # ── GHOST-METRIC split (post-probe architecture, 2026-07-03) ────────────────
    # Open-loop probes showed NO lut walks without feedback (official surv .035, v3c .053), so
    # swapping the champion's CONTROL feedforward is pure risk. Instead: control ghost stays the
    # proven one (GHOST_LUT_JSON); GHOST_METRIC_JSON scores/display a DIFFERENT ghost (v3c = the
    # champion's own smoothed full-body gait). WBMATCH legs/arms/attitude/speed judge against the
    # metric ghost; corridors/rewards keep the control ghost. Unset -> metric == control (as before).
    ghost_leg_M = None; ghost_arm_M = None; NBM = 0; _gvx_m = None
    ghost_lp_M = None; ghost_elb_M = None       # WBMATCH v2 channels (link positions, elbow ref)
    ghost_lo_M = None                           # WBMATCH v3: link orientations (wxyz, heading frame)
    w_elb = _f("W_ELB", 0.0); w_link = _f("W_LINK", 0.0)   # train ON the v2 components
    # REWARD_REFS_METRIC=1: legs/arms IMITATION rewards reference the METRIC ghost instead of the
    # control tables. THE reference-change design that survives the corridor-offsets mechanism
    # (2026-07-05: five failures = center moved; every historical success moved only reward/metric
    # refs). Control corridor NEVER moves; the reward pulls the gait toward the metric ghost within
    # the corridor width (widen smoothly with GHOST_RES_ANNEAL, never step it).
    rr_metric = bool(_i("REWARD_REFS_METRIC", 0))
    _lp_heading = False                          # lp_lut frame: heading (v2.1) vs legacy body
    lp_ids_rt = None                             # rollout-side FK-matched link body ids
    _gmj = os.environ.get("GHOST_METRIC_JSON")
    if _gmj:
        import json as _json2
        _gm2 = _json2.loads(open(_gmj).read())
        NBM = int(_gm2["nb"])
        ghost_leg_M = torch.tensor(np.asarray(_gm2["leg_lut"], np.float32), device=tdev)
        if "arm_lut" in _gm2 and use_armg:
            _alm = torch.tensor(np.asarray(_gm2["arm_lut"], np.float32), device=tdev)  # [left,right]
            ghost_arm_M = _alm if arm_sign[0] > 0 else _alm[:, [1, 0]]
        if "att_lut" in _gm2:
            ghost_att_t = torch.tensor(np.asarray(_gm2["att_lut"], np.float32), device=tdev)
        _gvx_m = float(_gm2.get("vx", 0.45))
        if "lp_lut" in _gm2:    # WBMATCH v2: FK'd link positions of the ruler ghost
            ghost_lp_M = torch.tensor(np.asarray(_gm2["lp_lut"], np.float32), device=tdev)
            _lp_heading = _gm2.get("lp_frame") == "heading"
        if "elbow_lut" in _gm2:  # recorded elbow channel (else the hang constant scores the elbows)
            ghost_elb_M = torch.tensor(np.asarray(_gm2["elbow_lut"], np.float32), device=tdev)
        if "lo_lut" in _gm2:     # WBMATCH v3 (owner 2026-07-05): link ORIENTATIONS of the ruler ghost
            ghost_lo_M = torch.tensor(np.asarray(_gm2["lo_lut"], np.float32), device=tdev)
        R._log(world, "GHOST-METRIC %s: nb=%d vx=%.3f arms=%s att=%s links=%s (control ghost unchanged)"
               % (os.path.basename(_gmj), NBM, _gvx_m, ghost_arm_M is not None, "att_lut" in _gm2,
                  ghost_lp_M is not None))
    omega = world._gwm_omega                                  # LUT mode overrode this to the recorded cadence
    # ── GHOST-RESIDUAL architecture (owner target: gmatch 0.8-0.9 GUARANTEED) ──
    # GHOST_RESIDUAL>0: LEG actions become ghost(phase) + a*GHOST_RESIDUAL (a bounded, so the legs
    # can deviate at most GHOST_RESIDUAL rad from the reference -> gmatch >= exp(-(res/sig_eval)^2)
    # BY CONSTRUCTION; e.g. 0.15 rad -> >=0.83). Arms + waist keep FULL authority for balance.
    # This revives the shadowing residual idea that failed pre-parity: with bit-exact physics and
    # free arms, the only question left is balance, which is what RL learns here.
    gres = _f("GHOST_RESIDUAL", 0.0)
    if gres > 0 or os.environ.get("DISTILL_OFFICIAL"):
        # (DISTILL_OFFICIAL needs the leg index tensors even corridor-free: the teacher writes
        #  leg targets directly during BC while the student learns them as free actions.)
        leg_act_t2 = torch.tensor([int(a) for a in leg_act], dtype=torch.long, device=tdev)
        # position of each leg actuator inside act_use (whole-body action vector), slot order
        _liu = [list(act_use).index(int(a)) for a in leg_act]
        liu_j = torch.tensor(_liu, dtype=torch.long, device=tdev)
        # PER-JOINT corridor widths: the ROLL joints (hipR/ankR, slots 1,5,7,11) are the lateral
        # stabilizers -- clamping them to the recorded angles (recorded at the REFERENCE's states)
        # strips the policy's lateral authority and the base rocks ~3.5x more than the official
        # walker (ours +-9deg vs its 2.6deg rms). GHOST_RESIDUAL_LAT widens ONLY those four; the
        # pitch-plane joints (the visually defining gait) stay at GHOST_RESIDUAL.
        _glat = _f("GHOST_RESIDUAL_LAT", gres)
        _wv = np.full(12, gres, np.float32); _wv[[1, 5, 7, 11]] = _glat
        # GHOST_RESIDUAL_YAW: hip-yaw slack (slots 2, 8). Turning to a commanded heading needs
        # yaw authority the straight-walk corridor doesn't grant (nav1 verdict: the policy
        # physically COULDN'T turn much inside +-0.10 rad of the straight ghost's hip yaw).
        _gyaw = _f("GHOST_RESIDUAL_YAW", gres)
        _wv[[2, 8]] = _gyaw
        gres_w = torch.tensor(_wv, dtype=torch.float32, device=tdev)
    # PHASE-GATED corridor width (round 8: catch-steps are contacts at the WRONG PHASE, and
    # this channel resisted reward -- so corridor the contact timing). During a leg's STANCE
    # window the PITCH-plane corridor tightens to GHOST_STANCE_TIGHT x base (no room to yank
    # the foot into a catch-step); full width returns through swing. ROLL joints stay full
    # width always: lateral balance is applied by the PLANTED leg (gating rolls in stance
    # would re-run the falsified width-snap).
    stance_tight = _f("GHOST_STANCE_TIGHT", 0.0)
    _sg_mask = torch.tensor([1., 0., 1., 1., 1., 0.] * 2, dtype=torch.float32, device=tdev)

    def _gres_gated(_gb):
        if not (0.0 < stance_tight < 1.0) or lut_swing is None:
            return gres_w
        _sL = (stance_tight + (1.0 - stance_tight) * lut_swing[0][_gb]).unsqueeze(1).expand(-1, 6)
        _sR = (stance_tight + (1.0 - stance_tight) * lut_swing[1][_gb]).unsqueeze(1).expand(-1, 6)
        _s = torch.cat([_sL, _sR], 1)
        return gres_w * (1.0 + _sg_mask * (_s - 1.0))
    # ── VELOCITY-CONDITIONED REST (owner architecture 2026-07-04: the transitioning method).
    # VC_REST=1: the corridor FEEDFORWARD blends from the gait to the STAND pose as |cmd|->0
    # (a corridor that marches at cmd=0 forbids standing), swing gates scale with |cmd|, and
    # CMD_RESAMPLE_S resamples the command MID-EPISODE so stop->start transitions happen
    # inside training episodes. sample_cmd already snaps cmd<0.12 to exactly 0 ("learn to
    # stand") -- set VX_START=0 to put rest in the distribution.
    # SPAWN_STATES=<npz>: train FROM a bank of collected states (parts 2/3 of the
    # transitioning method: the stand/transition policy trains on the WALKER'S actual
    # braking-exit states -- skill chaining requires skill A's exit distribution to be
    # inside skill B's entry distribution). Bank comes from COLLECT_STATES in the eval.
    spawn_bank = None
    _sbp = os.environ.get("SPAWN_STATES")
    if _sbp and os.path.exists(_sbp):
        _sb = np.load(_sbp)
        spawn_bank = (torch.tensor(_sb["qpos"], dtype=torch.float32, device=tdev),
                      torch.tensor(_sb["qvel"], dtype=torch.float32, device=tdev))
        R._log(world, "SPAWN-STATES: %d entry states from %s (frac=%.2f)"
               % (int(spawn_bank[0].shape[0]), os.path.basename(_sbp), _f("SPAWN_STATES_FRAC", 0.5)))
    vc_rest = bool(_i("VC_REST", 0)) and gres > 0
    vc_stand_t = None
    vc_glat = torch.ones(K, device=tdev)      # latched gate (see _vc_gate_step)
    vc_vema = torch.zeros(K, device=tdev)     # low-pass |vx| (stride pulsation filter)
    if vc_rest:
        vc_stand_t = torch.tensor(spawn[leg_qadr], dtype=torch.float32, device=tdev)

    def _vc_gate_step():
        """PHASE-LATCHED velocity gate (the deploy DS lesson, in training): when the gate
        target falls (braking), keep the FULL gait until the next DOUBLE SUPPORT, then step
        the reference down in one clean plant-both-feet-and-settle. A gate that decays
        mid-swing collapses gait amplitude while the body still moves -- feet drag, every
        stop trips (vc2..vc7 all capped at surv 0.35-0.42 because of it). Rising targets
        (restart) pass through immediately: accelerating INTO the gait is always safe."""
        # LOW-PASS the measured velocity: a walking G1's instantaneous vx pulsates +-0.2
        # per stride, so a raw-vx gate re-opens between every step and the latch DEADLOCKS
        # (measured live: glat pinned at 1.00 through entire stop windows, robot creeping
        # 0.1-0.4 m/s forever). The EMA decays smoothly through the threshold on a brake.
        vc_vema.mul_(0.95).add_(0.05 * qvel_t[:, 0].abs())
        if _i("VC_GATE_CMDONLY", 0):
            # COMMAND-authoritative target: measured velocity in the target is a POSITIVE
            # FEEDBACK LOOP at the stop (gate open while moving -> corridor commands gait ->
            # robot keeps moving; live draw walked straight through a stop window at 0.51 m/s).
            # The DS latch alone prevents mid-brake amplitude collapse, which is the only job
            # the velocity term ever had (vc7).
            g_tgt = (cmd_t[:, 0].abs() / _f("VC_GATE_V", 0.15)).clamp(0.0, 1.0)
        else:
            g_tgt = (torch.maximum(cmd_t[:, 0].abs(), vc_vema)
                     / _f("VC_GATE_V", 0.15)).clamp(0.0, 1.0)
        _ds = 0.65 * 2.0 * math.pi
        _pm = (phase_t - _ds) % math.pi
        in_ds = torch.minimum(_pm, math.pi - _pm) < 0.35
        # (A restart phase-reset to DS was tried here and measured HARMFUL: 0.589/10.6%
        # completions -> 0.190/0% with the same weights. The free-running clock restart
        # is what the policy handles best; do not re-add without an A/B.)
        _down = _f("VC_GATE_DOWN", 0.0)
        if _down > 0:
            # MORPH RELEASE (ghost-morph doctrine: snaps die, morphs survive): DS only
            # TRIGGERS the settle; the gate then decays smoothly (~0.3 s at 0.06/tick)
            # instead of stepping full-gait -> stand in one tick. A settle already in
            # progress (glat < 1) keeps decaying outside the DS window.
            settling = in_ds | (vc_glat < 0.999)
            vc_glat.copy_(torch.where(g_tgt >= vc_glat, g_tgt,
                                      torch.where(settling,
                                                  torch.maximum(g_tgt, vc_glat - _down), vc_glat)))
        else:
            vc_glat.copy_(torch.where(g_tgt >= vc_glat, g_tgt,
                                      torch.where(in_ds, g_tgt, vc_glat)))
        return vc_glat
        R._log(world, "VC-REST: corridor blends to STAND as |cmd|->0; CMD_RESAMPLE_S=%.1f"
               % _f("CMD_RESAMPLE_S", 0.0))
        R._log(world, "GHOST-RESIDUAL: legs = ghost(phase) +/- %.3f rad (roll joints +/- %.3f); arms/waist free"
               % (gres, _glat))
    # ARM CORRIDOR: bind the SHOULDER-PITCH pair to the arm ghost the same way the legs are bound
    # to the gait -- the arm channel proved immune to reward pressure (flat ~0.36 across three
    # runs); the corridor makes it structural. Elbows/rolls/yaws/wrists + waist stay free.
    ares = _f("ARM_RESIDUAL", 0.0)
    use_armcorr = bool(ares > 0 and use_armg)          # needs the arm ghost (arm_qadr/ghost_arm_t)
    if use_armcorr:
        _arm_act = []
        _trn2 = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
        for q in arm_qadr:                              # find the actuator driving each shoulder joint
            for a in act_use:
                jid = int(_trn2[int(a)][0])
                if int(world.solver.mj_model.jnt_qposadr[jid]) == int(q):
                    _arm_act.append(int(a)); break
        if len(_arm_act) == 2:
            arm_act_t2 = torch.tensor(_arm_act, dtype=torch.long, device=tdev)
            arm_liu = torch.tensor([list(act_use).index(a) for a in _arm_act], dtype=torch.long, device=tdev)
            R._log(world, "ARM-CORRIDOR: shoulders = arm_ghost(phase) +/- %.3f rad" % ares)
        else:
            use_armcorr = False
            R._log(world, "ARM-CORRIDOR: shoulder actuators not resolved -> disabled")
    # ELBOW CORRIDOR (ghost v3e arms package): hold the elbows at ELBOW_TARGET (owner-approved
    # +1.6 rad = near-straight human hang; G1 elbow convention: POSITIVE = extension) +/-
    # ELBOW_RESIDUAL. Elbows were fully policy-free before; without this nothing enforces the look.
    _elbt = os.environ.get("ELBOW_TARGET")
    if _elbt in ("", "off"):        # legacy-era policies: jobs can disable the session corridor
        _elbt = None
    use_elbcorr = False
    if _elbt is not None and whole and len(elb_qadr) == 2:
        elb_tgt_f = float(_elbt); eres = _f("ELBOW_RESIDUAL", 0.15)
        _trn4 = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
        _elb_act = []
        for q in elb_qadr:
            for a4 in act_use:
                jid4 = int(_trn4[int(a4)][0])
                if int(world.solver.mj_model.jnt_qposadr[jid4]) == int(q):
                    _elb_act.append(int(a4)); break
        if len(_elb_act) == 2:
            elb_act_t2 = torch.tensor(_elb_act, dtype=torch.long, device=tdev)
            elb_liu = torch.tensor([list(act_use).index(a4) for a4 in _elb_act], dtype=torch.long, device=tdev)
            use_elbcorr = True
            R._log(world, "ELBOW-CORRIDOR: elbows = %.2f +/- %.2f rad" % (elb_tgt_f, eres))
    # ELBOW-LUT corridor (round 10, owner: "move the elbows... swing them"): when the CONTROL
    # lut carries an elbow_lut, the corridor center follows it per phase bin (breathing elbows)
    # instead of the fixed hang constant.
    elb_lut_t = None
    if use_elbcorr and os.environ.get("GHOST_LUT_JSON"):
        try:
            import json as _json5
            _geld = _json5.loads(open(os.environ["GHOST_LUT_JSON"]).read())
            if "elbow_lut" in _geld:
                elb_lut_t = torch.tensor(np.asarray(_geld["elbow_lut"], np.float32), device=tdev)
                R._log(world, "ELBOW-LUT: corridor center follows the recorded/authored elbow cycle (%.2f-%.2f rad)"
                       % (float(elb_lut_t.min()), float(elb_lut_t.max())))
        except Exception:
            pass
    # SHOULDER ROLL/YAW corridor (WBMATCH v2 links finding 2026-07-05): only shoulder PITCH is
    # ghosted; roll/yaw are FREE channels (straight-elbow arc legacy) and settled ~25-30 deg off
    # the reference zero -> elbows 16cm / hands 20cm off in the LINK metric while the joint
    # metric read 0.88. Reward at W_LINK=10 did not move them (channels that resist reward get
    # CORRIDORS). SHRY_TARGET pins them structurally: target +/- SHRY_RESIDUAL.
    _shryt = os.environ.get("SHRY_TARGET")
    if _shryt in ("", "off"):       # legacy-era policies: jobs can disable the session corridor
        _shryt = None
    use_shrycorr = False
    if _shryt is not None and whole:
        shry_tgt_f = float(_shryt); shry_res = _f("SHRY_RESIDUAL", 0.15)
        _shry_act = []; _shry_sign = []
        _trn5 = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
        import mujoco as _mjn5
        for a5 in act_use:
            jid5 = int(_trn5[int(a5)][0])
            nm5 = (_mjn5.mj_id2name(world.solver.mj_model, _mjn5.mjtObj.mjOBJ_JOINT, jid5) or "")
            if "shoulder_roll" in nm5 or "shoulder_yaw" in nm5:
                _shry_act.append(int(a5))
                _shry_sign.append(1.0 if "left" in nm5.lower() else -1.0)
        if not _shry_act and len(arm_qadr) == 2 and len(elb_qadr) == 2                 and all(int(e) == int(a) + 3 for a, e in zip(sorted(arm_qadr), sorted(elb_qadr))):
            # names don't survive the newton->MJCF conversion (same trap as ARM-GHOST):
            # POSITIONAL fallback -- in the G1 arm chain roll/yaw sit between shoulder-pitch
            # (arm_qadr) and elbow (arm_qadr+3), i.e. qpos adr +1 and +2. Verified by the
            # elbow==pitch+3 sanity check above before trusting the layout.
            _trn5b = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
            _q2a = {}
            for a5 in act_use:
                jid5 = int(_trn5b[int(a5)][0])
                _q2a[int(world.solver.mj_model.jnt_qposadr[jid5])] = int(a5)
            _shry_tgts = []
            _shry_yawt = _f("SHRY_YAW_TARGET", 0.0)
            for qa, sd in zip(arm_qadr, arm_sign):
                for off in (1, 2):
                    if int(qa) + off in _q2a:
                        _shry_act.append(_q2a[int(qa) + off]); _shry_sign.append(float(sd))
                        # SHRY_TARGET is the ROLL target only (mirrored: + = out on the left);
                        # yaw gets its own (default 0 -- a shared target silently TWISTED the arms)
                        _shry_tgts.append(float(sd) * (shry_tgt_f if off == 1 else _shry_yawt))
        if _shry_act:
            shry_act_t2 = torch.tensor(_shry_act, dtype=torch.long, device=tdev)
            shry_liu = torch.tensor([list(act_use).index(a5) for a5 in _shry_act], dtype=torch.long, device=tdev)
            # per-channel signed targets (roll mirrored L/R: positive = arm out on the left, in on the right)
            shry_tgtv_t = torch.tensor(_shry_tgts, dtype=torch.float32, device=tdev)
            shry_sign_t = torch.tensor(_shry_sign, dtype=torch.float32, device=tdev)
            use_shrycorr = True
            R._log(world, "SHRY-CORRIDOR: shoulder roll/yaw (%d ch) roll=%.2f*side yaw=%.2f*side +/- %.2f rad"
                   % (len(_shry_act), shry_tgt_f, _shry_yawt, shry_res))
    # elbow morph start: the pose the policy actually holds (spawn), annealed to ELBOW_TARGET
    elb_start = float(np.mean(spawn[np.array(elb_qadr)])) if (use_elbcorr and len(elb_qadr) == 2) else 0.0
    elb_cur = elb_tgt_f if use_elbcorr else 0.0
    if use_elbcorr and morph is not None:
        elb_cur = elb_start
        R._log(world, "ELBOW-MORPH: %.2f -> %.2f over %d iters" % (elb_start, elb_tgt_f, morph_iters))
    _pk = _f("CARRY_PAYLOAD_KG", 0.0)
    if _pk > 0:
        # CARRY payload (BATON 3rd specialist): add half the box mass to each HAND body -- the
        # payload's balance effect (CoM shift) without contact simulation (the bigfoot precedent:
        # plant modification as a training tool). The engine-compiled model has generic body names,
        # so hands are resolved STRUCTURALLY: elbow joint -> its body -> descend the chain to the leaf.
        try:
            _mjmC = world.solver.mj_model
            _bm = np.asarray(rm.body_mass.numpy()).reshape(-1).copy()
            _sm = np.asarray(rm.body_subtreemass.numpy()).reshape(-1).copy()
            if len(elb_qadr) != 2:
                raise RuntimeError("elbow qadr not resolved (need the arm chain for hand bodies)")
            _hands = []
            for _eq in elb_qadr:
                _jid = next(j for j in range(int(_mjmC.njnt)) if int(_mjmC.jnt_qposadr[j]) == int(_eq))
                _b = int(_mjmC.jnt_bodyid[_jid])
                while True:                          # descend the (linear) forearm chain to the leaf
                    _kids = [c for c in range(int(_mjmC.nbody)) if int(_mjmC.body_parentid[c]) == _b and c != _b]
                    if not _kids:
                        break
                    _b = _kids[0]
                _hands.append(_b)
            for _bidC in _hands:
                _bm[_bidC] += _pk / 2.0
                _p = _bidC
                while _p > 0:                        # subtree mass: this body and every ancestor
                    _sm[_p] += _pk / 2.0
                    _p = int(_mjmC.body_parentid[_p])
                _sm[0] += _pk / 2.0
            rm.body_mass.assign(_bm.reshape(rm.body_mass.numpy().shape))
            rm.body_subtreemass.assign(_sm.reshape(rm.body_subtreemass.numpy().shape))
            R._log(world, "CARRY-PAYLOAD: +%.2f kg per hand (box %.2f kg) on bodies %s (leaf-of-elbow-chain), subtrees updated"
                   % (_pk / 2.0, _pk, _hands))
        except Exception as _eC:
            R._log(world, "CARRY-PAYLOAD err %r" % (_eC,))
    z_stance = float(np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)[fL][2])  # planted foot z

    # ── REFERENCE-IN-OBS (community-tracker Phase 1; opt-in REF_OBS, default off = obs unchanged) ──
    # The ghost currently enters the policy IMPLICITLY (phase clock + corridor + reward). REF_OBS puts
    # the TARGET POSE in the observation (K future lookaheads of ghost_leg deltas + target att) so ONE
    # policy can track ANY motion. Appended at the END of obs_of's vector, so an old (non-REF) champion
    # is a strict obs PREFIX -> load_ref_expand zero-expands it (iter-0 == champion). See ref_obs.py.
    _ref_on, _ref_K, _ref_stride = _refobs.ref_params()
    _ref_use_att = bool(_ref_on and ghost_att_t is not None)
    # REF_OBS_WB: track the WHOLE body (23 joints from wb_lut) instead of just the 12 legs -- needed
    # for dance (arms/waist). Falls back to legs if wb_lut absent or not whole-body.
    _ref_wb = bool(_ref_on and _i("REF_OBS_WB", 0) and ghost_wb_t is not None and whole)
    _ref_tgt = ghost_wb_t if _ref_wb else ghost_leg_t
    _ref_cur_qadr = qadr_use_t if _ref_wb else leg_qadr_t
    if _ref_on:
        OBS_DIM += _refobs.ref_block_dim(int(_ref_tgt.shape[1]), _ref_K, _ref_use_att)
        R._log(world, "REF_OBS: on K=%d stride=%d tgt_w=%d wb=%s att=%s -> OBS_DIM=%d"
               % (_ref_K, _ref_stride, int(_ref_tgt.shape[1]), _ref_wb, _ref_use_att, OBS_DIM))

    net = _make_ac(_i("PPO_HID", 256)).to(tdev)
    if path and os.path.exists(path):
        try:
            if _ref_on:
                _exp = _refobs.load_ref_expand(net, path, tdev)
                R._log(world, "walk-recipe: resumed %s (REF_OBS obs-expand=%s)" % (path, _exp))
            else:
                net.load_state_dict(torch.load(path, map_location=tdev)); R._log(world, "walk-recipe: resumed %s" % path)
        except Exception:
            pass
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    # ── DISTILLATION (owner's Option A: corridor style + NATIVE lateral control) ─
    # DISTILL_TEACHER=<ckpt>: the corridor champion drives the sim exactly as trained (through
    # the GHOST_RESIDUAL/ARM_RESIDUAL corridors), while the STUDENT (this net, FREE action
    # space) is regressed on the teacher's FINAL joint targets converted to free-action
    # coordinates (a_free = (target - nom)/act_scale) -- exact labels, no approximation. After
    # DISTILL_BC iterations the corridors are DROPPED and the normal PPO loop fine-tunes the
    # free student (style stays via reward; attitude/speed gain the state-feedback the corridor
    # coupling blocked). The student net init = teacher weights (same obs; leg/shoulder action
    # dims change meaning, BC re-grounds them).
    teacher = None
    _dt_path = os.environ.get("DISTILL_TEACHER")
    distill_free = bool(_dt_path)
    if distill_free:
        teacher = _make_ac(_i("PPO_HID", 256)).to(tdev)
        teacher.load_state_dict(torch.load(_dt_path, map_location=tdev)); teacher.eval()
        try:
            net.load_state_dict(torch.load(_dt_path, map_location=tdev))   # warm init for the student
        except Exception:
            pass
        R._log(world, "DISTILL: teacher=%s (corridors %s/%s arm %s) -> FREE student, BC=%d iters"
               % (os.path.basename(_dt_path), _f('GHOST_RESIDUAL', 0), _f('GHOST_RESIDUAL_LAT', 0),
                  _f('ARM_RESIDUAL', 0), _i("DISTILL_BC", 300)))
    # ── DISTILL-OFFICIAL (owner 2026-07-06: transplant motion.pt's leg micro-skill onto the
    # puppet). The OFFICIAL Unitree policy (TorchScript, 47-dim unitree obs, 12 leg actions,
    # target = DEFAULT + 0.25*a) drives the legs during BC; the student regresses the exact
    # free-action equivalents. Arms/elbows stay at the corridor centers (the design layer).
    teacher_off = None
    _do_path = os.environ.get("DISTILL_OFFICIAL")
    if _do_path:
        teacher_off = torch.jit.load(_do_path, map_location=tdev)
        teacher_off.eval()
        distill_free = True                     # reuse the BC scaffold's free-student path
        off_default_t = torch.tensor([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2,
                                     dtype=torch.float32, device=tdev)
        off_cmd_scale_t = torch.tensor([2.0, 2.0, 0.25], dtype=torch.float32, device=tdev)
        R._log(world, "DISTILL-OFFICIAL: teacher=%s (unitree 47-dim legs-only) BC=%d iters"
               % (os.path.basename(_do_path), _i("DISTILL_BC", 300)))

    _pfx = ("W_", "VX_", "VTRACK", "FEET_", "SWING_", "PUSH_", "MOTOR_", "OBS_NOISE", "AMP_",
            "RES_ACT", "PPO_", "Z_TGT", "FALL_", "IC_RAND", "WALK_", "EVAL_", "POLICY_")
    _cfg = " ".join("%s=%s" % (k, os.environ[k]) for k in sorted(os.environ)
                    if any(k.startswith(p) for p in _pfx))
    R._log(world, "CONFIG " + _cfg)
    R._log(world, "walk-recipe: K=%d T=%d obs=%d priv=%d act_scale=%.2f feet z_stance=%.3f swing=%.2f"
           % (K, T, OBS_DIM, PRIV_DIM, act_scale, z_stance, swing_h))

    phase_t = torch.zeros(K, device=tdev)
    cmd_t = torch.zeros(K, 3, device=tdev)
    cmd_tgt_t = torch.zeros(K, 3, device=tdev)                    # [vx, vy, wyaw] command per world
    motor_t = torch.ones(K, 1, device=tdev)                  # per-world motor-strength scale
    twopi = 2 * math.pi
    # HEADING CONDITIONING (BATON walkto, 2026-07-06): per-env heading target. The plain heading
    # obs was a DEAD input (trained at yaw~0 only; live steering test: shifted obs = no turn), so
    # the policy must TRAIN with randomized targets: obs heading becomes wrap(yaw - ytgt) and the
    # velocity rewards + harness lateral catch rotate into the target frame.
    ytgt_t = torch.zeros(K, device=tdev)
    _yr = _f("YAW_TGT_RAND", 0.0)
    # ARC CURRICULUM (the TURN specialist): per-episode yaw RATE -- the heading target ROTATES
    # at wz each tick, so the policy learns CONTINUOUS turning (arc walking), not just settling
    # onto a fixed heading. Half the episodes draw wz=0: the straight walk stays in-distribution.
    wz_t = torch.zeros(K, device=tdev)
    _wzr = _f("YAW_RATE_RAND", 0.0)

    def sample_cmd(mask, vx_cap, snap=True):
        n = K
        vx = torch.rand(n, device=tdev) * (vx_cap - vx_start) + vx_start
        vx = torch.where(vx < 0.12, torch.zeros_like(vx), vx)     # deadband -> learn to stand
        _tvx = _f("TURN_VX", -1.0)
        if _tvx >= 0.0:               # TURN specialist: fixed low forward speed -> a tight turning arc
            vx = torch.full_like(vx, _tvx)
        new = torch.stack([vx, torch.zeros(n, device=tdev), torch.zeros(n, device=tdev)], 1)
        # MORPH, NEVER SNAP (GHOST-MORPH doctrine; vc1 collapsed to surv 0.007 when mid-episode
        # command snaps stepped the corridor feedforward gait->stand in ONE tick): mid-episode
        # resamples set the TARGET only; cmd_t slews toward it at CMD_SLEW per tick. Fresh
        # resets (snap=True) set both -- no snap occurs during a lifetime.
        cmd_tgt_t.copy_(torch.where(mask.unsqueeze(1), new, cmd_tgt_t))
        if snap:
            cmd_t.copy_(torch.where(mask.unsqueeze(1), new, cmd_t))
        if _yr > 0:   # heading conditioning: new episodes (and resamples) draw a heading target
            _yn = (torch.rand(K, device=tdev) - 0.5) * 2.0 * _yr
            ytgt_t.copy_(torch.where(mask, _yn, ytgt_t))
        if _wzr > 0:  # arc curriculum: new episodes draw a yaw RATE (half stay straight, wz=0)
            _wn = (torch.rand(K, device=tdev) - 0.5) * 2.0 * _wzr
            _wn = torch.where(torch.rand(K, device=tdev) < 0.5, torch.zeros_like(_wn), _wn)
            wz_t.copy_(torch.where(mask, _wn, wz_t))
        _twz = _f("TURN_WZ", 0.0)
        if _twz != 0.0:               # UNCONDITIONAL turn specialist: EVERY env turns at the SAME
            wz_t.copy_(torch.where(mask, torch.full_like(wz_t, _twz), wz_t))  # fixed rate -> baked-in
        # spin, no live command needed (sidesteps the dead heading channel, like the stander)

    def reset_fields(mask, vx_cap):
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, device=tdev) * icj
        rq[:, 4:7] += torch.randn(K, 3, device=tdev) * ica
        rv = torch.zeros(K, nv, device=tdev)
        rv[:, 0:2] = torch.randn(K, 2, device=tdev) * icv
        rv[:, 3:5] = torch.randn(K, 2, device=tdev) * icv
        new_phase = torch.zeros_like(phase_t)
        if spawn_bank is not None:
            _nb8 = int(spawn_bank[0].shape[0])
            _sel8 = torch.randint(0, _nb8, (K,), device=tdev)
            _use8 = torch.rand(K, device=tdev) < _f("SPAWN_STATES_FRAC", 0.5)
            _bq = spawn_bank[0][_sel8].clone()
            _bq[:, 7:] += torch.randn(K, nq - 7, device=tdev) * icj * 0.5
            rq = torch.where(_use8.unsqueeze(1), _bq, rq)
            rv = torch.where(_use8.unsqueeze(1), spawn_bank[1][_sel8], rv)
        if _i("SPAWN_FACE_YTGT", 0) and _yr > 0 and not seq_mode:
            # HEADING-INVARIANT WALK (owner 2026-07-08): the spawn (nominal + captured-state bank) is
            # always +x-oriented (yaw 0) -> the walker learns an ABSOLUTE +x attractor and reverts to
            # +x live after a footwork turn. Rotate the spawn FACING and base linvel to the (random,
            # full-circle) ytgt so the walker trains to walk STRAIGHT FORWARD in ANY absolute heading;
            # the legs (body-frame) are untouched, so the CLEAN +x gait carries to every axis. The
            # footwork turn changes facing; this walker just holds whatever heading it is handed.
            _cyt, _syt = torch.cos(ytgt_t), torch.sin(ytgt_t)
            _vx0 = rv[:, 0].clone(); _vy0 = rv[:, 1].clone()
            rv[:, 0] = _cyt * _vx0 - _syt * _vy0
            rv[:, 1] = _syt * _vx0 + _cyt * _vy0
            rq[:, 3] = torch.cos(ytgt_t / 2); rq[:, 4] = 0.0; rq[:, 5] = 0.0; rq[:, 6] = torch.sin(ytgt_t / 2)
        if seq_mode:
            # RSI: respawn each resetting world at a RANDOM point of the routine, in the ghost's
            # pose AND velocity there (legs from lut + derivative; base z/yaw/linvel/yaw-rate from
            # the root trajectory). Every part of the dance trains in parallel.
            # SEQ_BIN_LO/HI (segment-isolation mode, owner ask 2026-07-04): confine RSI (and the
            # episode window, see the rollout) to bins [LO, HI) so ONE segment trains alone --
            # per-segment mastery separates "this segment is infeasible" from "the transitions
            # break" in a way the shared-policy feasibility map cannot.
            if seq_win:
                b0 = torch.randint(seq_lo, max(seq_lo + 1, seq_hi - 4), (K,), device=tdev)
            else:
                b0 = torch.randint(0, NB, (K,), device=tdev)
            rq[:, leg_qadr_t] = ghost_leg_t[b0] + torch.randn(K, 12, device=tdev) * icj
            rq[:, 2] = seq_rootz_t[b0]
            if seq_terrain:
                rq[:, 0] = seq_rootx_t[b0]   # respawn ON the terrain (x with z), not floating at x=0
            _y0 = seq_yaw_t[b0]
            rq[:, 3] = torch.cos(_y0 / 2); rq[:, 4] = 0.0; rq[:, 5] = 0.0; rq[:, 6] = torch.sin(_y0 / 2)
            rv[:, leg_dadr_t2] = seq_legvel_t[b0]
            _cv, _sv = torch.cos(_y0), torch.sin(_y0)
            rv[:, 0] = _cv * seq_cmdraw_t[b0, 0] - _sv * seq_cmdraw_t[b0, 1]
            rv[:, 1] = _sv * seq_cmdraw_t[b0, 0] + _cv * seq_cmdraw_t[b0, 1]
            rv[:, 5] = seq_cmdraw_t[b0, 2]
            new_phase = b0.float() * (twopi / NB)
        m = mask.unsqueeze(1)
        if vc_rest:
            _gr0 = (cmd_tgt_t[:, 0].abs() / _f("VC_GATE_V", 0.15)).clamp(0.0, 1.0)
            vc_glat.copy_(torch.where(mask, _gr0, vc_glat))
            vc_vema.copy_(torch.where(mask, cmd_tgt_t[:, 0].abs(), vc_vema))
        qpos_t.copy_(torch.where(m, rq, qpos_t)); qvel_t.copy_(torch.where(m, rv, qvel_t))
        mr = 1.0 + (torch.rand(K, 1, device=tdev) * 2 - 1) * motor_rand
        motor_t.copy_(torch.where(m, mr, motor_t))
        phase_t.copy_(torch.where(mask, new_phase, phase_t))
        sample_cmd(mask, vx_cap)

    def kin(qp, qv):
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1, 1))
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return gx, gy, gz, roll, pitch, yaw

    def _qrot_inv(q, v):   # rotate world vectors v (K,n,3) into the pelvis BODY frame (by conj(q)); q (K,4) wxyz
        w = q[:, 0:1].unsqueeze(1)                       # (K,1,1)
        pu = (-q[:, 1:4]).unsqueeze(1)                   # (K,1,3) vector part of conj(q)
        t = 2.0 * torch.cross(pu.expand_as(v), v, dim=2)
        return v + w * t + torch.cross(pu.expand_as(v), t, dim=2)

    def sim_kp():          # sim keypoints (feet/hands) in the pelvis body frame -> (K, n_kp*3), matches ref_kp_t
        rel = xpos_t[:, kp_ids_t] - qpos_t[:, 0:3].unsqueeze(1)   # world, pelvis-relative
        return _qrot_inv(qpos_t[:, 3:7], rel).reshape(K, -1)

    def _yawrot_inv(q, v):
        """Remove only the YAW of pelvis quat q from world vectors v -> HEADING frame
        (gravity-aligned). Used for lp_frame=="heading" luts: pelvis sway then displaces
        ref and sim links alike instead of re-projecting the leg lever into fake error."""
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, sn = torch.cos(yaw).unsqueeze(1), torch.sin(yaw).unsqueeze(1)
        return torch.stack([c * v[..., 0] + sn * v[..., 1],
                            -sn * v[..., 0] + c * v[..., 1], v[..., 2]], -1)

    def _lp_sim_of(ids):   # sim link positions in the ruler's frame -> (K, n*3)
        rel = xpos_t[:, ids] - qpos_t[:, 0:3].unsqueeze(1)
        rot = _yawrot_inv if _lp_heading else _qrot_inv
        return rot(qpos_t[:, 3:7], rel).reshape(K, -1)

    def obs_of(last_a):
        qp = qpos_t; qv = qvel_t
        gx, gy, gz, roll, pitch, yaw = kin(qp, qv)
        jpos = qp[:, qadr_use_t] - nom_use; jvel = qv[:, dadr_use_t]
        _ye = yaw - ytgt_t
        _ye = torch.atan2(torch.sin(_ye), torch.cos(_ye)).clamp(-0.5, 0.5)   # target-relative heading
        o = torch.cat([0.25 * qv[:, 3:6], torch.stack([gx, gy, gz], 1), cmd_t,
                       jpos, 0.05 * jvel, last_a,
                       torch.stack([torch.sin(phase_t), torch.cos(phase_t)], 1),
                       torch.stack([torch.sin(_ye), torch.cos(_ye)], 1)], 1)   # heading -> steer to ytgt
        if _ref_on:   # append the reference block (future target pose) -> universal-tracker obs
            _gb = (phase_t * (NB / twopi)).long() % NB
            o = torch.cat([o, _refobs.build_ref_block(
                _ref_tgt, ghost_att_t if _ref_use_att else None,
                _gb, qp[:, _ref_cur_qadr], _ref_K, _ref_stride, NB)], 1)
        return torch.clamp(o, -10, 10), roll, pitch, yaw, qp[:, 2]

    def priv_of():
        qv = qvel_t
        zL = xpos_t[:, fL, 2]; zR = xpos_t[:, fRi, 2]
        cL = (zL < z_stance + 0.03).float(); cR = (zR < z_stance + 0.03).float()
        return torch.stack([qv[:, 0], qv[:, 1], qv[:, 2], zL - z_stance, zR - z_stance, cL, cR], 1)

    def foot_track_reward():
        # each foot tracks a clock-driven target height: high mid-swing, low in stance. In LUT
        # mode the gates come from the RECORDED knee flexion (aligned with the recorded gait's
        # actual swing windows); otherwise the classic anti-phase sin() clock gates.
        if lut_swing is not None:
            gb = (phase_t / twopi * NB).long() % NB
            gL = lut_swing[0][gb]; gR = lut_swing[1][gb]
            if vc_rest:
                gL = gL * vc_glat; gR = gR * vc_glat
        else:
            gL = torch.clamp(torch.sin(phase_t), min=0.0)           # left swing gate
            gR = torch.clamp(torch.sin(phase_t + math.pi), min=0.0)  # right swing gate (anti-phase)
        tL = z_stance + swing_h * gL; tR = z_stance + swing_h * gR
        zL = xpos_t[:, fL, 2]; zR = xpos_t[:, fRi, 2]
        err = (zL - tL) ** 2 + (zR - tR) ** 2
        return torch.exp(-err / (feet_sig * feet_sig))

    reset_fields(torch.ones(K, dtype=torch.bool, device=tdev), vx_start)
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
            R._log(world, "walk-recipe: CUDA-graph captured (sub=%d)" % sub)
        except Exception as e:
            R._log(world, "walk-recipe: graph capture failed (%s)" % e); step_graph = None

    def physics():
        if step_graph is not None:
            wp.capture_launch(step_graph)
        else:
            for _ in range(sub):
                _mjw.step(rm, rd)

    # BALANCE HARNESS (owner idea 2026-07-06: the progressive playground). An invisible pelvis
    # wrench -- roll/pitch spring-damper + lateral catch -- scaled by harness_lam, annealed
    # ADAPTIVELY (steps down only when the honest eval says the policy succeeds at this level).
    # Gravity and contacts stay REAL: unlike gravity-scaling, the gait learned under the harness
    # has the correct cadence/contact physics, so each level transfers to the next.
    harness_lam = _f("HARNESS_LAM0", 0.0)
    xfrc_t = None; pelvis_bid = None
    _hkp, _hkd, _hfy = _f("HARNESS_KP", 150.0), _f("HARNESS_KD", 15.0), _f("HARNESS_FY", 80.0)
    if harness_lam > 0:
        import mujoco as _mj9
        _mjm9 = world.solver.mj_model
        pelvis_bid = -1
        for _nm9 in ("pelvis", "base_link", "base", "torso_link", "trunk"):
            pelvis_bid = int(_mj9.mj_name2id(_mjm9, _mj9.mjtObj.mjOBJ_BODY, _nm9))
            if pelvis_bid >= 0:
                break
        if pelvis_bid < 0:
            # floating-base robot: the base is the first body whose parent is the world
            for _b9 in range(1, int(_mjm9.nbody)):
                if int(_mjm9.body_parentid[_b9]) == 0:
                    pelvis_bid = _b9
                    break
        _bn9 = _mj9.mj_id2name(_mjm9, _mj9.mjtObj.mjOBJ_BODY, pelvis_bid) or "?"
        R._log(world, "HARNESS base body resolved: id=%d name=%s" % (pelvis_bid, _bn9))
        xfrc_t = wp.to_torch(rd.xfrc_applied).view(K, nbody, 6)
        R._log(world, "HARNESS: lam0=%.2f KP=%.0f KD=%.0f FY=%.0f pelvis=%d (graduate: -%.2f at eval surv>%.2f)"
               % (harness_lam, _hkp, _hkd, _hfy, pelvis_bid, _f("HARNESS_STEP", 0.1), _f("HARNESS_GRAD_SURV", 0.9)))

    def _harness_apply(_r, _p):
        if xfrc_t is None or harness_lam <= 0:
            return
        xfrc_t[:, pelvis_bid].zero_()
        _alv9 = (qpos_t[:, 2] > 0.35).float()   # never push a FALLEN robot (bungee grinds it into the floor -> NaN)
        _FCAP, _TCAP = 700.0, 350.0   # solver-safe wrench caps (unclamped bungee spikes -> NaN worlds)
        # lateral catch in the TARGET-heading frame (heading conditioning): damping world-Y while
        # walking a non-zero target heading would brake the walk direction itself
        _yc8, _ys8 = torch.cos(ytgt_t), torch.sin(ytgt_t)
        _lat8 = -_ys8 * qvel_t[:, 0] + _yc8 * qvel_t[:, 1]
        _F8 = _alv9 * (harness_lam * (-_hfy * _lat8)).clamp(-_FCAP, _FCAP)
        xfrc_t[:, pelvis_bid, 0] = -_ys8 * _F8
        xfrc_t[:, pelvis_bid, 1] = _yc8 * _F8
        # vertical BUNGEE to standing height: a fresh net doesn't tip over -- it BUCKLES straight
        # down (fall = bz < threshold with perfectly level attitude). The toddler harness holds
        # weight, not just tilt.
        _hkz = _f("HARNESS_KZ", 0.0)
        if _hkz > 0:
            xfrc_t[:, pelvis_bid, 2] = _alv9 * (harness_lam * (_hkz * (_f("HARNESS_Z0", 0.72) - qpos_t[:, 2])
                                                               - _f("HARNESS_DZ", 100.0) * qvel_t[:, 2]).clamp(min=0.0)).clamp(max=_FCAP)
        # HARNESS_ATT_GHOST: spring toward the GHOST's recorded sway (att_lut at the true phase)
        # instead of level. At high lam the harness owns the attitude -- a level-target harness
        # actively FIGHTS the reference's 2.54deg rms roll rhythm and caps the attitude term.
        if _i("HARNESS_ATT_GHOST", 0) and ghost_att_t is not None:
            _gbH = (phase_t / twopi * NBM).long() % NBM
            _rH = _r - ghost_att_t[_gbH, 0]; _pH = _p - ghost_att_t[_gbH, 1]
        else:
            _rH = _r; _pH = _p
        xfrc_t[:, pelvis_bid, 3] = _alv9 * (harness_lam * (-_hkp * _rH - _hkd * qvel_t[:, 3])).clamp(-_TCAP, _TCAP)
        xfrc_t[:, pelvis_bid, 4] = _alv9 * (harness_lam * (-_hkp * _pH - _hkd * qvel_t[:, 4])).clamp(-_TCAP, _TCAP)

    # ---- deploy-prediction metric (deterministic mean policy, long horizon, fixed seed) ----
    def deploy_eval():
        ytgt_t.zero_(); wz_t.zero_()     # evals judge the LEGACY straight walk (heading target 0)
        if _f("EVAL_YTGT", 0.0) != 0.0:
            ytgt_t.fill_(_f("EVAL_YTGT", 0.0))   # steering exam: ydrift readout = the turn verdict
        if xfrc_t is not None:
            xfrc_t.zero_()               # evals are HONEST: no harness assistance, ever
        g = torch.Generator(device=tdev).manual_seed(1234567)
        rq = spawn_t.unsqueeze(0).repeat(K, 1)
        rq[:, 7:] += torch.randn(K, nq - 7, generator=g, device=tdev) * eval_ic
        qpos_t.copy_(rq); qvel_t.copy_(torch.zeros(K, nv, device=tdev))
        motor_t.fill_(1.0); phase_t.zero_()
        cmd_t.copy_(torch.tensor([vx_max, 0.0, 0.0], device=tdev).unsqueeze(0).repeat(K, 1))
        _evcyc = _i("EVAL_CMD_CYCLE", 0)          # walk5/stop5 command alternation exam
        _colp = os.environ.get("COLLECT_STATES")   # harvest braking-entry states at stop edges
        _col_q = []; _col_v = []; _prev_c0 = None; _col_after = -1
        # NOTE: no _mjw.forward here -- it re-runs collision narrowphase (a large fresh CUDA alloc ->
        # OOM at high K). The CUDA-graph step recomputes kinematics from the reset qpos, exactly as
        # the training loop relies on after every fall-reset. One step of stale xpos is negligible.
        la = torch.zeros(K, ACT_DIM, device=tdev)
        alive = torch.full((K,), float(eval_H), device=tdev); done = torch.zeros(K, dtype=torch.bool, device=tdev)
        # EVAL_ACT_RECORD: capture world-0's initial state + per-tick CLAMPED actions + qpos trace
        # for the live ACTION-REPLAY parity probe (the knife that splits plant vs closed-loop).
        _arec_path = os.environ.get("EVAL_ACT_RECORD")
        _arec = {"q0": qpos_t[0].detach().cpu().numpy().copy(),
                 "v0": qvel_t[0].detach().cpu().numpy().copy(),
                 "acts": [], "qpos": [], "ph0": float(phase_t[0])} if _arec_path else None
        # DISTILL_TEVAL: the TEACHER drives the legs during this eval -- scores the official policy
        # ITSELF in the trainer plant (foundation check: are the BC labels the authentic gait?).
        _tev = (teacher_off is not None) and bool(_i("DISTILL_TEVAL", 0))
        if _tev:
            for _bn in ("hidden_state", "cell_state"):
                if hasattr(teacher_off, _bn):
                    _hb = getattr(teacher_off, _bn)
                    setattr(teacher_off, _bn, torch.zeros(_hb.shape[0], K, _hb.shape[-1], device=tdev))
            _tev_prev = torch.zeros(K, 12, device=tdev)
            _tev_tgt = off_default_t.unsqueeze(0).expand(K, -1).clone()
            _tev_acc = 1.0   # >=period so the first tick queries the teacher
            _tev_per = _f("DISTILL_DEC_S", 0.0)   # 0.02 = official 50Hz; 0 = every tick
        # EVAL_RECORD (rule 4, generalized): capture the achieved motion of the best-surviving
        # eval world and re-emit it as the next-generation lut -- the TRUE ghost is what the
        # robot actually does, feasible by construction. Records a subset of worlds (memory),
        # keeps the longest survivor, resamples one routine to the lut bins.
        _rec_path = os.environ.get("EVAL_RECORD")
        _recS = min(K, 64) if _rec_path else 0
        _recQ = torch.zeros(eval_H, _recS, nq, device=tdev) if _recS else None
        x0 = qpos_t[:, 0].clone(); xmax = x0.clone()   # xmax -> furthest forward each world reaches (dist walked)
        y0 = qpos_t[:, 1].clone()                       # y0 -> for lateral-drift measurement
        vt_acc = torch.zeros((), device=tdev); live_n = torch.zeros((), device=tdev)
        stopv_acc = torch.zeros((), device=tdev); stopn_acc = torch.zeros((), device=tdev)  # stop quality
        elb_acc = torch.zeros((), device=tdev)     # WBMATCH v2: elbows folded into the arms term
        lnk_acc = torch.zeros((), device=tdev)     # WBMATCH v2: skeleton link-position match
        # time-resolved profile: the average hides WHEN points are lost -> bucket the composite
        # and the links term by phase bin (16 buckets across the cycle/routine).
        prof_acc = torch.zeros(16, device=tdev); prof_lnk = torch.zeros(16, device=tdev)
        prof_n = torch.zeros(16, device=tdev)
        _nlp_ev = (int(ghost_lp_M.shape[1]) // 3) if ghost_lp_M is not None else 8   # #skeleton links the metric ghost carries (feet+knees=4, full=8)
        lnk_per = torch.zeros(_nlp_ev, device=tdev)      # per-link mean |err| (m): WHICH links carry the deficit
        lori_acc = torch.zeros((), device=tdev)    # WBMATCH v3: link-orientation match
        v4_leg_acc = torch.zeros((), device=tdev); v4_arm_acc = torch.zeros((), device=tdev)
        v4_lp_acc = torch.zeros((), device=tdev); v4_att_acc = torch.zeros((), device=tdev)
        v4_ori_acc = torch.zeros((), device=tdev)  # v4 RAW mean-square errors (shape-only ruler)
        lori_per = torch.zeros(_nlp_ev, device=tdev)     # per-link mean geodesic error (rad)
        # METRIC v3 speed (owner 2026-07-05): STRIDE-AVERAGED velocity. Instantaneous vx pulsates
        # +-0.2 within every stride even for a perfect walker (caps the term ~0.75 and the v2
        # composite at ~0.95 exactly); the eye judges PACE, not intra-stride wobble. EMA tau = one
        # gait cycle. Legacy 4-term WBMATCH keeps the instantaneous term for comparability.
        vx_sema = torch.full((K,), float(vx_max), device=tdev)
        _hipq = int(leg_qadr[0])       # ACHIEVED-SWING readout: the number the score can hide
        # ACHIEVED-STANCE readout (owner round 9: "close the legs, no splay"): mean |hip roll|,
        # mean |hip yaw|, and foot lateral gap over steady live ticks -- leg closure as a number.
        _hrq = torch.tensor([int(leg_qadr[1]), int(leg_qadr[7])], dtype=torch.long, device=tdev)
        _hyq = torch.tensor([int(leg_qadr[2]), int(leg_qadr[8])], dtype=torch.long, device=tdev)
        _st_roll = torch.zeros((), device=tdev); _st_yaw = torch.zeros((), device=tdev)
        _st_gap = torch.zeros((), device=tdev); _st_n = torch.zeros((), device=tdev)
        act_rate_acc = torch.zeros((), device=tdev)   # SMOOTHNESS readouts (owner: "smoother")
        jjerk_acc = torch.zeros((), device=tdev); _prev_jvel = None; _prev_act = None
        # FOOTBEAT readout (owner: "constant cyclic feet, not dancing"): contact-onset stride
        # periods per foot; regularity = CV (humans ~2-4%; reactive stepping >15%).
        _fb_prev = torch.zeros(K, 2, dtype=torch.bool, device=tdev)
        _fb_last = torch.full((K, 2), -1.0, device=tdev)
        _fb_ivals = []
        hip_lo = torch.full((K,), 9.9, device=tdev); hip_hi = torch.full((K,), -9.9, device=tdev)
        _sal = float(np.clip(dt * omega / (2.0 * math.pi), 0.005, 0.2))   # dt/cycle_s
        spd2_acc = torch.zeros((), device=tdev)
        lp_ids_ev = None                            # FK-matched sim body ids for ghost_lp_M columns
        if ghost_lp_M is not None:
            # one-time FK match: strike the ruler's frame-0 pose (legs/arms/elbows refs), read body
            # positions in the pelvis frame, and pair each reference link with the nearest sim body.
            _sq0 = qpos_t.clone()
            qpos_t[:, leg_qadr_t] = (ghost_leg_M[0] if ghost_leg_M is not None else ghost_leg_t[0])
            if use_armg:
                qpos_t[:, arm_qadr_t] = (ghost_arm_M[0] if ghost_arm_M is not None else ghost_arm_t[0])
            if len(elb_qadr) == 2:
                qpos_t[:, torch.tensor(elb_qadr, dtype=torch.long, device=tdev)] =                     (ghost_elb_M[0] if ghost_elb_M is not None else _f("ELB_HANG_REF", 0.30))
            ctrl_t[:, act_use_t] = qpos_t[:, qadr_use_t]; ctrl_t[:, act_use_t + 1] = 0.0
            physics()
            _rot0 = _yawrot_inv if _lp_heading else _qrot_inv
            _bf0 = _rot0(qpos_t[:1, 3:7], xpos_t[:1] - qpos_t[:1, 0:3].unsqueeze(1))[0]
            _r0 = ghost_lp_M[0].reshape(-1, 3)
            _idsL = [int(((_bf0 - _r0[k]) ** 2).sum(1).argmin()) for k in range(_r0.shape[0])]
            lp_ids_ev = torch.tensor(_idsL, dtype=torch.long, device=tdev)
            _errsL = [float(((_bf0[_idsL[k]] - _r0[k]) ** 2).sum() ** 0.5) for k in range(len(_idsL))]
            R._log(world, "LINKMATCH body ids (FK-matched) = %s  match-err(m) = %s"
                   % (_idsL, [round(e, 3) for e in _errsL]))
            qpos_t.copy_(_sq0); qvel_t.copy_(torch.zeros(K, nv, device=tdev))
        gm_acc = torch.zeros((), device=tdev); wob_acc = torch.zeros((), device=tdev)   # wob = base ang-vel (wobble)
        am_acc = torch.zeros((), device=tdev)                                            # arm-swing ghost match
        # WHOLE-BODY match components (owner: the score should mean what the EYE means):
        # legs (gm) + arms (am) + base ATTITUDE vs the ghost's level glide + SPEED vs the ghost's vx.
        att_acc = torch.zeros((), device=tdev); spd_acc = torch.zeros((), device=tdev)
        _gvx_ref = 0.45
        if os.environ.get("GHOST_LUT_JSON"):
            try:
                import json as _json
                _gvx_ref = float(_json.loads(open(os.environ["GHOST_LUT_JSON"]).read()).get("vx", 0.45))
            except Exception:
                pass
        if _gvx_m is not None:
            _gvx_ref = _gvx_m          # metric ghost's pace defines the speed score
        lat = _i("AMP_CTRL_LAT", 0); ring = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(lat)]
        heval = net.init_hidden(K, tdev)   # actor hidden carried, never reset (like the live deploy)
        map_mode = seq_mode and _i("SEQ_EVAL_MAP", 0)
        if map_mode:
            # DYNAMIC FEASIBILITY MAP (owner insight 2026-07-04: "the ghost itself may not be
            # executable"): start world k at routine bin k*NB/K in the ghost's pose+velocity and
            # measure survival PER BEAT -- kinematic gates can't see dynamic infeasibility; this can.
            b0 = (torch.arange(K, device=tdev) * NB) // K
            rq2 = spawn_t.unsqueeze(0).repeat(K, 1)
            rq2[:, leg_qadr_t] = ghost_leg_t[b0]
            rq2[:, 2] = seq_rootz_t[b0]
            if seq_terrain:
                rq2[:, 0] = seq_rootx_t[b0]   # per-beat feasibility map: start each world ON its tread
            _y0 = seq_yaw_t[b0]
            rq2[:, 3] = torch.cos(_y0 / 2); rq2[:, 4] = 0.0; rq2[:, 5] = 0.0; rq2[:, 6] = torch.sin(_y0 / 2)
            rv2 = torch.zeros(K, nv, device=tdev)
            rv2[:, leg_dadr_t2] = seq_legvel_t[b0]
            _cv, _sv = torch.cos(_y0), torch.sin(_y0)
            rv2[:, 0] = _cv * seq_cmdraw_t[b0, 0] - _sv * seq_cmdraw_t[b0, 1]
            rv2[:, 1] = _sv * seq_cmdraw_t[b0, 0] + _cv * seq_cmdraw_t[b0, 1]
            rv2[:, 5] = seq_cmdraw_t[b0, 2]
            qpos_t.copy_(rq2); qvel_t.copy_(rv2)
            phase_t.copy_(b0.float() * (twopi / NB))
        elif seq_win:                      # segment isolation: judged from the WINDOW start
            b0 = torch.full((K,), seq_lo, dtype=torch.long, device=tdev)
            rq2 = spawn_t.unsqueeze(0).repeat(K, 1)
            rq2[:, leg_qadr_t] = ghost_leg_t[b0]
            rq2[:, 2] = seq_rootz_t[b0]
            if seq_terrain:
                rq2[:, 0] = seq_rootx_t[b0]   # segment isolation: start ON the terrain
            _y0 = seq_yaw_t[b0]
            rq2[:, 3] = torch.cos(_y0 / 2); rq2[:, 4] = 0.0; rq2[:, 5] = 0.0; rq2[:, 6] = torch.sin(_y0 / 2)
            rv2 = torch.zeros(K, nv, device=tdev)
            rv2[:, leg_dadr_t2] = seq_legvel_t[b0]
            _cv, _sv = torch.cos(_y0), torch.sin(_y0)
            rv2[:, 0] = _cv * seq_cmdraw_t[b0, 0] - _sv * seq_cmdraw_t[b0, 1]
            rv2[:, 1] = _sv * seq_cmdraw_t[b0, 0] + _cv * seq_cmdraw_t[b0, 1]
            rv2[:, 5] = seq_cmdraw_t[b0, 2]
            qpos_t.copy_(rq2); qvel_t.copy_(rv2)
            phase_t.copy_(b0.float() * (twopi / NB))
        elif seq_mode:                     # sequences are judged from the TOP of the routine
            phase_t.zero_()
        for t in range(eval_H):
            if _evcyc:
                _c0 = vx_max if ((t * dt) % 10.0) < 5.0 else 0.0
                if vc_rest:
                    _sl = _f("CMD_SLEW", 0.008)
                    cmd_t[:, 0] = cmd_t[:, 0] + max(-_sl, min(_sl, _c0 - float(cmd_t[0, 0])))
                else:
                    cmd_t[:, 0] = _c0
                if _colp is not None:
                    if _prev_c0 is not None and ((_c0 == 0.0 and _prev_c0 > 0.0)
                                                 or (_c0 > 0.0 and _prev_c0 == 0.0)):
                        _col_after = 50     # 0..0.4 s after a STOP or RESTART command edge
                    _prev_c0 = _c0
                    if _col_after >= 0:
                        if _col_after % 10 == 0:
                            _alv = ~done
                            _col_q.append(qpos_t[_alv][:256].detach().cpu())
                            _col_v.append(qvel_t[_alv][:256].detach().cpu())
                        _col_after -= 1
            if seq_mode:                   # time-varying command from the root trajectory
                _sbE = (phase_t / twopi * NB).long() % NB
                cmd_t.copy_(seq_cmd_t[_sbE])
                ytgt_t.copy_(seq_yaw_t[_sbE])          # harness/velocity frame FOLLOWS the reference turn
            o, roll, pitch, yaw, bz = obs_of(la)
            mean, _std, heval = net.act(o, heval); a = torch.clamp(mean, -1, 1); env_a = a * act_scale
            if _arec is not None and t < 700:
                _arec["acts"].append(a[0].detach().cpu().numpy().copy())
                _arec["qpos"].append(qpos_t[0].detach().cpu().numpy().copy())
                if t == 699:
                    np.savez(_arec_path, q0=_arec["q0"], v0=_arec["v0"], ph0=_arec["ph0"],
                             acts=np.stack(_arec["acts"]), qpos=np.stack(_arec["qpos"]))
                    R._log(world, "EVAL_ACT_RECORD: 700 ticks of world-0 saved -> %s" % _arec_path)
            _oatE = _i("OBS_AUDIT_T", 0)
            if _oatE > 0 and _oatE <= t < _oatE + 3:   # steady-state FULL-vector audit (trainer side)
                R._log(world, "OBS-AUDIT-EVAL t=%d o=%s act=%s" % (
                    t, [round(float(v), 4) for v in o[0].detach().cpu().numpy()],
                    [round(float(v), 4) for v in a[0].detach().cpu().numpy()]))
            if _i("EVAL_LOG_OBS", 0) and t <= 3:   # world-0 obs/action, mirror of DEP-OBS
                _o0 = o[0].detach().cpu().numpy(); _a0 = a[0].detach().cpu().numpy()
                R._log(world, "EVL-OBS t=%d angv=%s projg=%s jpos6=%s jvel6=%s act6=%s" % (
                    t, [round(float(v), 4) for v in _o0[0:3]], [round(float(v), 4) for v in _o0[3:6]],
                    [round(float(v), 4) for v in _o0[9:15]], [round(float(v), 4) for v in _o0[9 + ACT_DIM:15 + ACT_DIM]],
                    [round(float(v), 3) for v in _a0[0:6]]))
            if lat > 0:
                ring.append(env_a); env_a = ring.pop(0)
            ctrl_t[:, act_use_t] = nom_use + env_a; ctrl_t[:, act_use_t + 1] = 0.0
            _corr_any = (gres > 0 and not distill_free) or (teacher_off is not None)
            if gres > 0 and not distill_free:   # legs: ghost feedforward + bounded residual
                _gb = ((phase_t + _f("PHASE_LEAD", 0.0)) / twopi * NB).long() % NB  # CONTROL lead: cancels PD lag; metric stays on true phase
                if vc_rest:
                    _g6 = _vc_gate_step().unsqueeze(1)
                    _ff6 = _g6 * ghost_leg_t[_gb] + (1.0 - _g6) * vc_stand_t
                    ctrl_t[:, leg_act_t2] = _ff6 + a[:, liu_j].clamp(-1, 1) * (_gres_gated(_gb) * _ann)
                else:
                    ctrl_t[:, leg_act_t2] = ghost_leg_t[_gb] + a[:, liu_j].clamp(-1, 1) * (_gres_gated(_gb) * _ann)
            if _corr_any:   # arm/elbow/shoulder corridors are DESIGN -- they survive distillation (legs go free)
                _gb = ((phase_t + _f("PHASE_LEAD", 0.0)) / twopi * NB).long() % NB
                if use_armcorr:   # shoulders: arm-ghost feedforward + bounded residual
                    _gbA = ((phase_t + _f("PHASE_LEAD_ARM", _f("PHASE_LEAD", 0.0))) / twopi * NB).long() % NB
                    ctrl_t[:, arm_act_t2] = ghost_arm_t[_gbA] + a[:, arm_liu].clamp(-1, 1) * ares
                if use_elbcorr:   # elbows: fixed-angle corridor (human hang; elb_cur morphs to target)
                    _er0 = elb_lut_t[_gb] if elb_lut_t is not None else elb_cur
                    ctrl_t[:, elb_act_t2] = _er0 + a[:, elb_liu].clamp(-1, 1) * eres
                if use_shrycorr:  # shoulder roll/yaw: pin to target (side-mirrored), bounded residual
                    ctrl_t[:, shry_act_t2] = shry_tgtv_t + a[:, shry_liu].clamp(-1, 1) * shry_res
            if _tev:   # teacher-drive: official policy owns the legs (student's leg action ignored)
                _tev_acc += _tick_s
                if _tev_per <= 0.0 or _tev_acc >= _tev_per - 1e-9:
                    if _tev_per > 0.0:
                        _tev_acc -= _tev_per   # 50Hz teacher over 62.5Hz ticks: query on the official period
                    gx9, gy9, gz9, _r9, _p9, _y9 = kin(qpos_t, qvel_t)
                    if _i("DISTILL_HH", 0):   # heading-hold, exactly the recording's condition (YAW_KP=1.5)
                        cmd_t[:, 2] = (-1.5 * _y9).clamp(-0.5, 0.5)
                    _obs47 = torch.cat([
                        qvel_t[:, 3:6] * 0.25,
                        torch.stack([gx9, gy9, gz9], 1),
                        cmd_t * off_cmd_scale_t,
                        qpos_t[:, leg_qadr_t] - off_default_t,
                        qvel_t[:, leg_dadr_t] * 0.05,
                        _tev_prev,
                        torch.stack([torch.sin(phase_t), torch.cos(phase_t)], 1),
                    ], 1).clamp(-100, 100)
                    _a_off = teacher_off(_obs47).clamp(-100, 100)
                    _tev_prev = _a_off
                    _tev_tgt = off_default_t + 0.25 * _a_off
                ctrl_t[:, leg_act_t2] = _tev_tgt
            if not whole:
                ctrl_t[:, waist_act_t] = waist_nom_t; ctrl_t[:, waist_act_t + 1] = 0.0
            _harness_apply(roll, pitch)   # eval WEARS the current level's harness (graduation = surv AT level)
            physics(); phase_t.add_(omega * dt)
            if _leashlead > 0.0:          # PROGRESS-LOCK (mirror the rollout so the eval reflects the trained pacing)
                torch.minimum(phase_t, (torch.searchsorted(seq_rootx_t, qpos_t[:, 0].clamp(_rootx_lo, _rootx_hi)).float() + _leashlead) / NB * twopi, out=phase_t)
            if seq_mode and seq_hold_end:
                phase_t.clamp_(max=_tp5_seq)   # eval: play once then HOLD (mirror the rollout)
            if _recS:
                _recQ[t] = qpos_t[:_recS]
            _o, roll, pitch, yaw, bz = obs_of(a)
            fwd = qvel_t[:, 0]                                       # WORLD +x speed (match training)
            xmax = torch.maximum(xmax, qpos_t[:, 0])                 # track furthest-forward reached
            live = (~done).float()
            vt_acc += (torch.exp(-((fwd - cmd_t[:, 0]) ** 2) / (vtsig * vtsig)) * live).sum(); live_n += live.sum()
            bidx = (phase_t / twopi * NB).long() % NB
            # metric ghost (split architecture): score vs the METRIC lut when present
            bidx_m = (phase_t / twopi * NBM).long() % NBM if ghost_leg_M is not None else bidx
            _leg_ref = ghost_leg_M[bidx_m] if ghost_leg_M is not None else ghost_leg_t[bidx]
            # metric sigma PINNED (GHOST_SIG_EVAL, default 0.35) so gmatch stays comparable across
            # runs even when the REWARD sigma (GHOST_SIG) is tightened as tracking improves.
            _egs = _f("GHOST_SIG_EVAL", 0.35)
            _leg_se = ((qpos_t[:, leg_qadr_t] - _leg_ref) ** 2).mean(1)
            gm = torch.exp(-_leg_se / (_egs * _egs))
            gm_acc += (gm * live).sum()
            # WBMATCH v4 (owner 2026-07-06: SHAPE ONLY, honest sigmas): accumulate RAW mean-square
            # errors; score at print time with discriminating sigmas. The legacy sigmas were
            # exposed as forgiving (7.5deg/joint scored 0.83) and the corridor guarantees a 0.78
            # floor by construction -- v4 exists so the score means what the EYE means.
            v4_leg_acc += (torch.where(torch.isfinite(_leg_se), _leg_se, torch.zeros_like(_leg_se)) * live).sum()
            if use_armg:
                _arm_ref = ghost_arm_M[bidx_m] if ghost_arm_M is not None else ghost_arm_t[bidx]
                am = torch.exp(-((qpos_t[:, arm_qadr_t] - _arm_ref) ** 2).mean(1) / (armg_sig * armg_sig))
                am_acc += (am * live).sum()
                if elb_qadr_t2 is not None:   # v2 arms term: shoulders AND elbows (ref = recorded else hang)
                    _elb_ref = (ghost_elb_M[bidx_m] if ghost_elb_M is not None
                                else _f("ELB_HANG_REF", 0.30))
                    _eq = qpos_t[:, elb_qadr_t2]
                    _ea = ((qpos_t[:, arm_qadr_t] - _arm_ref) ** 2).sum(1)
                    _ee = ((_eq - _elb_ref) ** 2).sum(1)
                    _am2_v = torch.exp(-((_ea + _ee) / 4.0) / (armg_sig * armg_sig))
                    elb_acc += (_am2_v * live).sum()
                    _arm_se = (_ea + _ee) / 4.0
                    v4_arm_acc += (torch.where(torch.isfinite(_arm_se), _arm_se, torch.zeros_like(_arm_se)) * live).sum()
            if ghost_lp_M is not None and lp_ids_ev is not None:   # v2 links term
                _lp_sim = _lp_sim_of(lp_ids_ev)
                _lps = _f("LP_SIG", 0.10)
                _dlp = _lp_sim - ghost_lp_M[bidx_m]
                _lp_se = (_dlp ** 2).mean(1)
                lnk = torch.exp(-_lp_se / (_lps * _lps))
                lnk_acc += (lnk * live).sum()
                v4_lp_acc += (torch.where(torch.isfinite(_lp_se), _lp_se, torch.zeros_like(_lp_se)) * live).sum()
                lnk_per += ((_dlp.reshape(K, -1, 3) ** 2).sum(2).sqrt() * live.unsqueeze(1)).sum(0)
                if ghost_lo_M is not None and xquat_t is not None:   # v3 link-ORIENTATION term
                    _qs = xquat_t[:, lp_ids_ev]                       # (K,n,4) world wxyz
                    _yh = 0.5 * yaw                                   # heading frame: rotate by yaw^-1 about z
                    _cw, _sz = torch.cos(_yh).unsqueeze(1), (-torch.sin(_yh)).unsqueeze(1)
                    _qh = torch.stack([_cw * _qs[..., 0] - _sz * _qs[..., 3],
                                       _cw * _qs[..., 1] - _sz * _qs[..., 2],
                                       _cw * _qs[..., 2] + _sz * _qs[..., 1],
                                       _cw * _qs[..., 3] + _sz * _qs[..., 0]], -1)
                    _qr = ghost_lo_M[bidx_m].reshape(K, -1, 4)
                    _dq = (_qh * _qr).sum(-1).abs().clamp(max=1.0)
                    _ang = 2.0 * torch.acos(_dq)                      # (K,n) geodesic rad
                    _los = _f("LO_SIG", 0.35)
                    lori = torch.exp(-(_ang ** 2).mean(1) / (_los * _los))
                    lori_acc += (lori * live).sum()
                    lori_per += (_ang * live.unsqueeze(1)).sum(0)
                    _ori_se = (_ang ** 2).mean(1)
                    v4_ori_acc += (torch.where(torch.isfinite(_ori_se), _ori_se, torch.zeros_like(_ori_se)) * live).sum()
            if ghost_att_t is not None:   # metric v2: attitude vs the GHOST's sway (not vs level)
                _bai = bidx_m if ghost_leg_M is not None else bidx
                _dr2 = (roll - ghost_att_t[_bai, 0]) ** 2 + (pitch - ghost_att_t[_bai, 1]) ** 2
                att_v = torch.exp(-_dr2 / (0.15 * 0.15))
                v4_att_acc += (torch.where(torch.isfinite(_dr2), _dr2, torch.zeros_like(_dr2)) * live).sum()
            else:
                _dr2 = roll * roll + pitch * pitch
                att_v = torch.exp(-_dr2 / (0.15 * 0.15))
                v4_att_acc += (torch.where(torch.isfinite(_dr2), _dr2, torch.zeros_like(_dr2)) * live).sum()
            att_acc += (att_v * live).sum()
            spd_v = torch.exp(-((fwd - _gvx_ref) ** 2) / (0.15 * 0.15))
            spd_acc += (spd_v * live).sum()
            vx_sema.mul_(1.0 - _sal).add_(_sal * fwd)
            _jv9 = qvel_t[:, dadr_use_t]   # all actuated joints (leg_dadr_t2 is seq-scope only -- crashed 40min, 2026-07-06)
            if _prev_jvel is not None:
                jjerk_acc += ((_jv9 - _prev_jvel).abs().mean(1) * live).sum()
            _prev_jvel = _jv9.clone()
            if _prev_act is not None:
                act_rate_acc += ((a - _prev_act).abs().mean(1) * live).sum()
            _prev_act = a.clone()
            _zL9 = xpos_t[:, fL, 2]; _zR9 = xpos_t[:, fRi, 2]
            _fb_now = torch.stack([_zL9 < z_stance + 0.03, _zR9 < z_stance + 0.03], 1)
            if t >= 500:
                _on = _fb_now & ~_fb_prev & (live > 0).unsqueeze(1).bool()
                _has = _on & (_fb_last > 0)
                if bool(_has.any()):
                    _fb_ivals.append((float(t) - _fb_last[_has]).cpu())
                _fb_last[_on] = float(t)
            _fb_prev = _fb_now
            if t >= 500:   # stance/splay accumulators (steady, live-weighted)
                _st_roll += (qpos_t[:, _hrq].abs().mean(1) * live).sum()
                _st_yaw += (qpos_t[:, _hyq].abs().mean(1) * live).sum()
                _dgs = xpos_t[:, fL, :2] - xpos_t[:, fRi, :2]
                _st_gap += ((-torch.sin(yaw) * _dgs[:, 0] + torch.cos(yaw) * _dgs[:, 1]).abs() * live).sum()
                _st_n += live.sum()
            if t >= 500:   # STEADY-STATE only: the launch transient inflates the range
                _hp = qpos_t[:, _hipq]
                hip_lo = torch.where(live > 0, torch.minimum(hip_lo, _hp), hip_lo)
                hip_hi = torch.where(live > 0, torch.maximum(hip_hi, _hp), hip_hi)
            spd2_v = torch.exp(-((vx_sema - _gvx_ref) ** 2) / (0.15 * 0.15))
            spd2_acc += (spd2_v * live).sum()
            if ghost_lp_M is not None and lp_ids_ev is not None and elb_qadr_t2 is not None and use_armg:
                _nbp = NBM if ghost_leg_M is not None else NB
                _bk = ((bidx_m if ghost_leg_M is not None else bidx) * 16) // max(1, _nbp)
                _c5v = (gm + _am2_v + att_v + spd2_v + lnk) / 5.0
                prof_acc.index_add_(0, _bk, _c5v * live)
                prof_lnk.index_add_(0, _bk, lnk * live)
                prof_n.index_add_(0, _bk, live)
            if _evcyc:  # stopv: mean |vx| while commanded to STAND and alive -- did it actually stop?
                _sw = (cmd_t[:, 0].abs() < 0.02).float() * live
                stopv_acc += (fwd.abs() * _sw).sum(); stopn_acc += _sw.sum()
            wob_acc += ((qvel_t[:, 3:6] ** 2).sum(1).sqrt() * live).sum()   # base angular-velocity magnitude = wobble
            fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz) | ~torch.isfinite(bz)  # NaN = dead, not immortal
            newly = fell & (~done); alive = torch.where(newly, torch.full_like(alive, float(t)), alive)
            done = done | fell; la = a
            if bool(done.all()):
                break
        fwd_dist = float((qpos_t[~done, 0] - x0[~done]).mean()) if bool((~done).any()) else 0.0
        dist = float((xmax - x0).mean())        # avg furthest-forward distance walked (all worlds, fall or not)
        ydrift = float((qpos_t[~done, 1] - y0[~done]).abs().mean()) if bool((~done).any()) else 0.0
        if use_armg:
            _n = max(1.0, float(live_n))
            _legs, _arms = float(gm_acc) / _n, float(am_acc) / _n
            _att, _spd = float(att_acc) / _n, float(spd_acc) / _n
            world._wr_last_att = _att   # harness graduation gate reads posture quality
            R._log(world, "    eval armmatch=%.3f (arm-swing ghost)" % _arms)
            R._log(world, "    eval WBMATCH=%.3f  [legs=%.3f arms=%.3f attitude=%.3f speed=%.3f]  (whole-body, eye-aligned)"
                   % ((_legs + _arms + _att + _spd) / 4.0, _legs, _arms, _att, _spd))
            if len(elb_qadr) == 2 and ghost_lp_M is not None:
                # WBMATCH v2 (owner 2026-07-05): arms term includes ELBOWS; 5th component = skeleton
                # LINK POSITIONS; speed = STRIDE-AVERAGED (metric v3). Legacy line above unchanged.
                _arms2 = float(elb_acc) / _n; _lnk = float(lnk_acc) / _n
                _spd2 = float(spd2_acc) / _n
                R._log(world, "    eval WBMATCH2=%.3f  [legs=%.3f arms+elb=%.3f attitude=%.3f speed(savg)=%.3f links=%.3f]  (v2 + stride-avg speed)"
                       % ((_legs + _arms2 + _att + _spd2 + _lnk) / 5.0, _legs, _arms2, _att, _spd2, _lnk))
                if ghost_lo_M is not None and xquat_t is not None and float(lori_acc) > 0:
                    _lo6 = float(lori_acc) / _n
                    R._log(world, "    eval WBMATCH3=%.3f  [v2 terms + links-ori=%.3f]  (v3: + link ORIENTATIONS, owner 2026-07-05)"
                           % ((_legs + _arms2 + _att + _spd2 + _lnk + _lo6) / 6.0, _lo6))
                    _lop = np.degrees((lori_per / max(1.0, float(live_n))).cpu().numpy())
                    R._log(world, "    LINKS-ORI(deg) [ankL ankR kneL kneR elbL elbR hndL hndR]: %s"
                           % " ".join("%.1f" % v for v in _lop))
                    # WBMATCH v4 (owner: SHAPE ONLY, no speed; honest sigmas; raw-error based so
                    # NaN worlds cannot poison it). Corridor floor printed so tautology is visible.
                    import math as _m4
                    _s4l, _s4a = _f("V4_SIG_LEG", 0.15), _f("V4_SIG_ARM", 0.15)
                    _s4p, _s4t, _s4o = _f("V4_SIG_LP", 0.06), _f("V4_SIG_ATT", 0.10), _f("V4_SIG_ORI", 0.35)
                    _v4 = [
                        _m4.exp(-float(v4_leg_acc) / _n / (_s4l * _s4l)),
                        _m4.exp(-float(v4_arm_acc) / _n / (_s4a * _s4a)),
                        _m4.exp(-float(v4_att_acc) / _n / (_s4t * _s4t)),
                        _m4.exp(-float(v4_lp_acc) / _n / (_s4p * _s4p)),
                        _m4.exp(-float(v4_ori_acc) / _n / (_s4o * _s4o)),
                    ]
                    _floor4 = _m4.exp(-(gres * gres) / (_s4l * _s4l)) if gres > 0 else 0.0
                    R._log(world, "    eval WBMATCH4=%.3f  [legs=%.3f arms+elb=%.3f attitude=%.3f links=%.3f ori=%.3f]  (SHAPE-ONLY, honest sigmas; corridor floor=%.2f)"
                           % (sum(_v4) / 5.0, _v4[0], _v4[1], _v4[2], _v4[3], _v4[4], _floor4))
                _pn = prof_n.clamp(min=1.0)
                _pc = (prof_acc / _pn).cpu().numpy(); _pl = (prof_lnk / _pn).cpu().numpy()
                R._log(world, "    WBM2-PROFILE (16 phase buckets): %s   worst=b%d"
                       % (" ".join("%.2f" % v for v in _pc), int(_pc.argmin())))
                R._log(world, "    LINKS-PROFILE:                   %s   worst=b%d"
                       % (" ".join("%.2f" % v for v in _pl), int(_pl.argmin())))
                _lper = (lnk_per / max(1.0, float(live_n))).cpu().numpy()
                _lplbl = ["ankL", "ankR", "kneL", "kneR", "elbL", "elbR", "hndL", "hndR"][:len(_lper)]
                R._log(world, "    LINKS-ERR(m) [%s]: %s"
                       % (" ".join(_lplbl), " ".join("%.3f" % v for v in _lper)))
        if map_mode:
            # per-beat survival (seconds) in 16 buckets across the routine
            _bkt = ((torch.arange(K, device=tdev) * NB) // K) * 16 // NB
            _surv_s = alive * (dt if isinstance(dt, float) else float(dt))
            _rowvals = []
            for _b in range(16):
                _m2 = _bkt == _b
                _rowvals.append(float(_surv_s[_m2].mean()) if bool(_m2.any()) else -1.0)
            R._log(world, "    FEASIBILITY MAP (mean survival s per routine 16th): "
                   + " ".join("%.2f" % v for v in _rowvals))
        if _colp is not None and _col_q:
            _cq9 = torch.cat(_col_q, 0).numpy(); _cv9 = torch.cat(_col_v, 0).numpy()
            np.savez(_colp, qpos=_cq9, qvel=_cv9)
            R._log(world, "COLLECT-STATES: %d braking-entry states -> %s" % (int(_cq9.shape[0]), _colp))
        if _recS:
            try:
                # among FULL survivors pick the LARGEST-SWING world: argmax(alive) alone
                # selects the most conservative walker (small safe swings survive longest) --
                # recorded 17deg while the eval median walked 21-25deg (2026-07-05).
                _nbo = int(_json.loads(open(os.environ["GHOST_LUT_JSON"]).read())["nb"])
                _alv8 = alive[:_recS]; _full8 = _alv8 >= (_alv8.max() - 1.0)
                _sw8 = (hip_hi - hip_lo)[:_recS].clone(); _sw8[~_full8] = -1.0
                _k = int(_sw8.argmax())
                _T = int(alive[_k])
                _cyc = int(round((twopi / omega) / (dt if isinstance(dt, float) else float(dt))))
                if seq_mode:
                    _T = min(_T, eval_H, _cyc)
                    traj = _recQ[:_T, _k].cpu().numpy()
                elif _i("REC_FOLD", 1):
                    # PHASE-FOLD (owner 2026-07-05: "the ghost's legs are vibrating -- no
                    # consistent motion to replicate"): a single-cycle recording preserves one
                    # world's balance TREMOR as if it were the gait. Fold ALL steady ticks of
                    # ALL full survivors per phase bin -- the systematic gait survives, the
                    # stochastic tremor cancels ~1/sqrt(N).
                    _st = int(np.ceil(500.0 / _cyc)) * _cyc
                    _sids = [kk for kk in range(_recS) if float(alive[kk]) >= eval_H - 1] or [_k]
                    Q9 = _recQ[_st:eval_H, _sids].cpu().numpy()          # (T', S, nq)
                    Tp, S9 = Q9.shape[0], Q9.shape[1]
                    binf = ((np.arange(_st, _st + Tp) * (omega * (dt if isinstance(dt, float) else float(dt)))
                             / (2.0 * np.pi)) * _nbo) % _nbo
                    bins9 = binf.astype(int) % _nbo
                    # roll/pitch per world/tick BEFORE averaging (yaw differs per world; averaging
                    # quaternions across headings is garbage)
                    w9, x9, y9, z9 = Q9[:, :, 3], Q9[:, :, 4], Q9[:, :, 5], Q9[:, :, 6]
                    r9 = np.arctan2(2 * (w9 * x9 + y9 * z9), 1 - 2 * (x9 ** 2 + y9 ** 2))
                    p9 = np.arcsin(np.clip(2 * (w9 * y9 - z9 * x9), -1, 1))
                    acc = np.zeros((_nbo, Q9.shape[2])); accA = np.zeros((_nbo, 2)); cnt = np.zeros(_nbo)
                    np.add.at(acc, bins9, Q9.mean(1))
                    np.add.at(accA, bins9, np.stack([r9.mean(1), p9.mean(1)], 1))
                    np.add.at(cnt, bins9, 1.0)
                    cnt = np.maximum(cnt, 1.0)
                    _tr = acc / cnt[:, None]
                    _attF = accA / cnt[:, None]
                    # rebuild quat cols yaw-free from folded roll/pitch (downstream extracts att
                    # and yaw from them); root x/y meaningless after folding -> zeroed
                    cr9, sr9 = np.cos(_attF[:, 0] / 2), np.sin(_attF[:, 0] / 2)
                    cp9, sp9 = np.cos(_attF[:, 1] / 2), np.sin(_attF[:, 1] / 2)
                    _tr[:, 3] = cp9 * cr9; _tr[:, 4] = cp9 * sr9; _tr[:, 5] = sp9 * cr9; _tr[:, 6] = -sp9 * sr9
                    _tr[:, 0] = 0.0; _tr[:, 1] = 0.0
                    _T = _cyc
                    _vx9 = float((_recQ[eval_H - 1, _sids, 0] - _recQ[_st, _sids, 0]).cpu().numpy().mean()
                                 / max(1e-6, Tp * 0.016))
                    R._log(world, "EVAL_RECORD FOLD: %d survivors x %d ticks -> %.0f samples/bin; vx=%.3f"
                           % (S9, Tp, float(cnt.mean()), _vx9))
                else:
                    # CYCLIC single-cycle window (legacy; REC_FOLD=0): steady, phase-aligned.
                    _st = int(np.ceil(500.0 / _cyc)) * _cyc
                    while _st + _cyc > min(_T, eval_H) and _st > 0:
                        _st -= _cyc                      # best survivor died early: latest full cycle
                    _T = _cyc
                    traj = _recQ[_st:_st + _cyc, _k].cpu().numpy()
                    R._log(world, "EVAL_RECORD: cyclic steady window ticks %d..%d (cycle=%d)"
                           % (_st, _st + _cyc, _cyc))
                _src = _json.loads(open(os.environ["GHOST_LUT_JSON"]).read())
                if "wb_joints" not in _src:   # cyclic control luts (unitree/v3c) carry no wb list
                    _src["wb_joints"] = [
                        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                        "waist_yaw_joint",
                        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                        "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
                        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                        "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"]
                _mjm2 = world.solver.mj_model
                import mujoco as _mj2
                _adr = []
                for _jn2 in _src["wb_joints"]:
                    _jid2 = _mj2.mj_name2id(_mjm2, _mj2.mjtObj.mjOBJ_JOINT, _jn2)
                    _adr.append(int(_mjm2.jnt_qposadr[_jid2]) if _jid2 >= 0 else -1)
                if all(_a < 0 for _a in _adr):
                    # the live model's imported-URDF joint names differ from the canonical
                    # ones -> every lookup failed and the wb track recorded as ZEROS (the
                    # owner-caught 'statue gliding in space'). Fall back to POSITIONAL
                    # pairing: both models enumerate joints in URDF declaration order.
                    _hin = [j for j in range(int(_mjm2.njnt))
                            if int(_mjm2.jnt_type[j]) != int(_mj2.mjtJoint.mjJNT_FREE)]
                    if len(_hin) >= len(_src["wb_joints"]):
                        _adr = [int(_mjm2.jnt_qposadr[_hin[k]]) for k in range(len(_src["wb_joints"]))]
                        R._log(world, "EVAL_RECORD: name lookup failed; using URDF-order positional joint map")
                try:
                    _tr
                except NameError:
                    _idx = np.linspace(0, _T - 1, _nbo).astype(int)
                    _tr = traj[_idx]
                _wb = np.zeros((_nbo, len(_adr)), np.float32)
                for _c, _ad in enumerate(_adr):
                    if _ad >= 0:
                        _wb[:, _c] = _tr[:, _ad]
                _w2, _x2, _y2, _z2 = _tr[:, 3], _tr[:, 4], _tr[:, 5], _tr[:, 6]
                _yaw2 = np.arctan2(2 * (_w2 * _z2 + _x2 * _y2), 1 - 2 * (_y2 ** 2 + _z2 ** 2))
                # RECORD the attitude track too: dict(_src) used to carry the SOURCE lut's
                # (synthetic) att_lut through -- the one channel still ruled by a made-up
                # reference (v6 att scored 0.72 vs a blend the robot never performed).
                _roll2 = np.arctan2(2 * (_w2 * _x2 + _y2 * _z2), 1 - 2 * (_x2 ** 2 + _y2 ** 2))
                _pitch2 = np.arcsin(np.clip(2 * (_w2 * _y2 - _z2 * _x2), -1, 1))
                _out = dict(_src)
                _out["att_lut"] = [[float(_roll2[i]), float(_pitch2[i])] for i in range(_nbo)]
                # true pace, measured, 0.016 s per recorded row (hand-derived labels produced
                # a 2x pace bug on 2026-07-05 -- the recorder owns this number now)
                try:
                    _out["vx"] = round(_vx9, 3)          # fold path: survivor-mean steady pace
                except NameError:
                    _out["vx"] = round(float(np.hypot(_tr[-1, 0] - _tr[0, 0],
                                                      _tr[-1, 1] - _tr[0, 1])) / max(1e-6, _T * 0.016), 3)
                _out["wb_joints"] = list(_src["wb_joints"])
                _out["wb_lut"] = [[float(v) for v in r] for r in _wb]
                _out["leg_lut"] = [[float(v) for v in r] for r in
                                   _tr[:, [int(qq) for qq in leg_qadr]]]
                _out["root_lut"] = [[float(_tr[i, 0]), float(_tr[i, 1]), float(_tr[i, 2]),
                                     float(_yaw2[i])] for i in range(_nbo)]
                if "arm_lut" in _src:
                    _ja = [_adr[_src["wb_joints"].index(nm)] for nm in
                           ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint")]
                    _out["arm_lut"] = [[float(_tr[i, _ja[0]]), float(_tr[i, _ja[1]])] for i in range(_nbo)]
                # elbow channel too: without it the metric scores elbows vs the HANG CONSTANT
                # (mislabeled the whole straight-arm campaign's arms component, 2026-07-05)
                _je = [_adr[_src["wb_joints"].index(nm)] for nm in
                       ("left_elbow_joint", "right_elbow_joint")]
                _out["elbow_lut"] = [[float(_tr[i, _je[0]]), float(_tr[i, _je[1]])] for i in range(_nbo)]
                _out["source"] = "RE-RECORDED achieved motion (rule 4): policy=%s survivor %d/%d ticks -- from %s" % (
                    os.path.basename(os.environ.get("RES_POLICY", "?")), _T, _cyc,
                    _out.get("source", ""))[:400]
                _json.dump(_out, open(_rec_path, "w"))
                R._log(world, "EVAL_RECORD: achieved lut written -> %s (survivor %d/%d ticks of one routine)"
                       % (_rec_path, _T, _cyc))
            except Exception as _e:
                R._log(world, "EVAL_RECORD failed: %r" % (_e,))
        R._log(world, "    SMOOTHNESS: act-rate %.4f  joint-accel %.4f (per-tick means; lower = smoother)"
               % (float(act_rate_acc) / max(1.0, float(live_n)), float(jjerk_acc) / max(1.0, float(live_n))))
        if _fb_ivals:
            _iv = torch.cat(_fb_ivals)
            _iv = _iv[(_iv > 12) & (_iv < 200)]        # drop contact chatter + missed cycles
            if _iv.numel() > 20:
                _m9, _s9 = float(_iv.mean()), float(_iv.std())
                R._log(world, "    FOOTBEAT: stride %.0f ticks (%.2f s)  CV %.1f%%  n=%d  (human metronome ~2-4%%; dancing >15%%)"
                       % (_m9, _m9 * 0.016, 100.0 * _s9 / max(_m9, 1e-6), int(_iv.numel())))
                # shape of the arrhythmia: bimodal (catch-steps) vs diffuse (drift) picks the lever
                _q9 = torch.quantile(_iv, torch.tensor([0.10, 0.25, 0.50, 0.75, 0.90]))
                _sh9 = float(((_iv < 0.6 * _m9).float()).mean())
                _mb9 = _iv[_iv >= 0.6 * _m9]           # main band = strides that aren't catch-steps
                _mbcv = 100.0 * float(_mb9.std()) / max(float(_mb9.mean()), 1e-6) if _mb9.numel() > 20 else -1.0
                R._log(world, "    FOOTBEAT-DIST: p10/25/50/75/90 = %s ticks  short-strides(<0.6x mean) %.0f%%  main-band CV %.1f%%"
                       % ("/".join("%.0f" % float(v) for v in _q9), 100.0 * _sh9, _mbcv))
        _sw = (hip_hi - hip_lo)[hip_hi > hip_lo]
        if _sw.numel():
            R._log(world, "    ACHIEVED-SWING: L hip pitch %.1f deg (median over surviving worlds)"
                   % float(np.degrees(float(_sw.median()))))
        if float(_st_n) > 0:
            R._log(world, "    ACHIEVED-STANCE: |hip roll| %.1f deg  |hip yaw| %.1f deg  foot gap %.3f m"
                   % (float(np.degrees(float(_st_roll) / float(_st_n))),
                      float(np.degrees(float(_st_yaw) / float(_st_n))),
                      float(_st_gap) / float(_st_n)))
        return (float(alive.mean()) / eval_H, float(done.float().mean()),
                float(vt_acc) / max(1.0, float(live_n)), fwd_dist, float(gm_acc) / max(1.0, float(live_n)), dist, ydrift,
                float(wob_acc) / max(1.0, float(live_n)), float(stopv_acc) / max(1.0, float(stopn_acc)))

    ep_ret = torch.zeros(K, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
    last_a = torch.zeros(K, ACT_DIM, device=tdev)
    hstate = net.init_hidden(K, tdev)   # actor recurrent hidden (None for MLP), carried across rollout
    ring_tr = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(ctrl_lat)]
    O = torch.zeros(T, K, OBS_DIM, device=tdev); P = torch.zeros(T, K, PRIV_DIM, device=tdev)
    A = torch.zeros(T, K, ACT_DIM, device=tdev); LP = torch.zeros(T, K, device=tdev)
    V = torch.zeros(T, K, device=tdev); RW = torch.zeros(T, K, device=tdev); DN = torch.zeros(T, K, device=tdev)
    done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
    vx_acc = torch.zeros((), device=tdev); vx_n = 0; step_ctr = 0
    t0 = _time.time(); total_steps = 0

    if _i("EVAL_ONLY", 0):
        _ann = 1.0   # eval closure reads the anneal factor; EVAL_ONLY never enters the training loop
        # In-training evals inherit cmd_t sampled by rollout resets; a fresh process still has
        # cmd_t=0 -- OUT-OF-DISTRIBUTION for a velocity-conditioned policy (obs say "stand still"
        # while the corridor steps). This silently invalidated every fresh-process EVAL_ONLY under
        # corridors (champion scored surv .038 on its own config) until found 2026-07-03.
        reset_fields(torch.ones(K, dtype=torch.bool, device=tdev), vx_max)
        with torch.no_grad():
            surv, frate, vt, fwd, gm, dist, ydrift, wob, stopv = deploy_eval()
        R._log(world, "DEPLOY-EVAL-ONLY ckpt surv=%.3f fall=%.3f vtrack=%.3f fwd=%.3f gmatch=%.3f dist=%.2f ydrift=%.2f wob=%.3f stopv=%.3f H=%d"
               % (surv, frate, vt, fwd, gm, dist, ydrift, wob, stopv, eval_H))
        _status_write("DONE")
        return

    _ann_final = _f("GHOST_RES_ANNEAL", 1.0)
    _ann = 1.0
    _bc = _i("DISTILL_BC", 300) if distill_free else 0
    if _bc > 0:
        # ── BC phase: teacher drives (with corridors), student regresses the EXACT free-action
        # equivalents of the teacher's final joint targets. Perfect labels; no reward involved.
        mse_fn = torch.nn.MSELoss()
        loss = torch.zeros((), device=tdev)
        off_prev = torch.zeros(K, 12, device=tdev) if teacher_off is not None else None
        if teacher_off is not None:
            # motion.pt is an LSTM with persistent (1,1,H) hidden buffers -- rebind to batch K
            # so one scripted teacher serves all envs (its forward writes back in-place [:]=).
            for _bn in ("hidden_state", "cell_state"):
                if hasattr(teacher_off, _bn):
                    _hb = getattr(teacher_off, _bn)
                    setattr(teacher_off, _bn, torch.zeros(_hb.shape[0], K, _hb.shape[-1], device=tdev))
                    R._log(world, "DISTILL-OFFICIAL: %s rebound to (%d,%d,%d)" % (_bn, _hb.shape[0], K, _hb.shape[-1]))
        for bit in range(_bc):
            OB = torch.zeros(T, K, OBS_DIM, device=tdev); LB = torch.zeros(T, K, ACT_DIM, device=tdev)
            with torch.no_grad():
                for t in range(T):
                    o, roll, pitch, yaw, bz = obs_of(last_a)
                    _gb = ((phase_t + _f("PHASE_LEAD", 0.0)) / twopi * NB).long() % NB  # CONTROL lead: cancels PD lag; metric stays on true phase
                    if teacher_off is not None:
                        # OFFICIAL teacher: unitree 47-dim obs (layout verified against the deploy
                        # controller AND the re-host trainer; mjw free-joint ang vel = body frame).
                        gx9, gy9, gz9, _r9, _p9, _y9 = kin(qpos_t, qvel_t)
                        _obs47 = torch.cat([
                            qvel_t[:, 3:6] * 0.25,
                            torch.stack([gx9, gy9, gz9], 1),
                            cmd_t * off_cmd_scale_t,
                            qpos_t[:, leg_qadr_t] - off_default_t,
                            qvel_t[:, leg_dadr_t] * 0.05,
                            off_prev,
                            torch.stack([torch.sin(phase_t), torch.cos(phase_t)], 1),
                        ], 1).clamp(-100, 100)
                        a_off = teacher_off(_obs47).clamp(-100, 100)
                        off_prev = a_off
                        tgt_leg = off_default_t + 0.25 * a_off        # unitree convention; slot order matches
                        ctrl_t[:, act_use_t] = nom_use; ctrl_t[:, act_use_t + 1] = 0.0
                        ctrl_t[:, leg_act_t2] = tgt_leg
                        lbl = torch.zeros(K, ACT_DIM, device=tdev)
                        lbl[:, liu_j] = torch.clamp((tgt_leg - nom_use[liu_j]) / act_scale, -1, 1)
                        if use_armcorr:
                            ctrl_t[:, arm_act_t2] = ghost_arm_t[_gb]
                            lbl[:, arm_liu] = torch.clamp((ghost_arm_t[_gb] - nom_use[arm_liu]) / act_scale, -1, 1)
                        if use_elbcorr:
                            _er8 = elb_lut_t[_gb] if elb_lut_t is not None else elb_cur
                            ctrl_t[:, elb_act_t2] = _er8
                            _er8b = _er8 if torch.is_tensor(_er8) else torch.full((K, 2), float(_er8), device=tdev)
                            if _er8b.dim() == 1:
                                _er8b = _er8b.unsqueeze(0).expand(K, -1)
                            lbl[:, elb_liu] = torch.clamp((_er8b - nom_use[elb_liu]) / act_scale, -1, 1)
                        ac_t = lbl
                    else:
                        mean_t, _s, _h = teacher.act(o, None)
                        ac_t = torch.clamp(mean_t, -1, 1)
                        ctrl_t[:, act_use_t] = nom_use + ac_t * act_scale; ctrl_t[:, act_use_t + 1] = 0.0
                        tgt_leg = ghost_leg_t[_gb] + ac_t[:, liu_j] * gres_w
                        ctrl_t[:, leg_act_t2] = tgt_leg
                        lbl = ac_t.clone()
                        lbl[:, liu_j] = torch.clamp((tgt_leg - nom_use[liu_j]) / act_scale, -1, 1)
                        if use_armcorr:
                            tgt_arm = ghost_arm_t[_gb] + ac_t[:, arm_liu] * ares
                            ctrl_t[:, arm_act_t2] = tgt_arm
                            lbl[:, arm_liu] = torch.clamp((tgt_arm - nom_use[arm_liu]) / act_scale, -1, 1)
                    _harness_apply(roll, pitch)   # BC on the puppet: the harness carries balance
                    OB[t] = o; LB[t] = lbl
                    physics(); phase_t.add_(omega * dt)
                    _o2, roll, pitch, yaw, bz = obs_of(ac_t)
                    fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz) | ~torch.isfinite(bz)  # NaN = dead, not immortal
                    last_a = torch.where(fell.unsqueeze(1), torch.zeros_like(ac_t), ac_t)
                    reset_fields(fell, vx_max)
            bo = OB.reshape(-1, OBS_DIM); bl = LB.reshape(-1, ACT_DIM)
            for _e in range(2):
                perm = torch.randperm(bo.shape[0], device=tdev)
                for mb in perm.chunk(nmb):
                    m_s, _sd, _hh = net.act(bo[mb], None)
                    loss = mse_fn(m_s, bl[mb])
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step()
            if bit % 20 == 0:
                R._log(world, "DISTILL-BC it=%d/%d mse=%.5f" % (bit, _bc, float(loss)))
                try:
                    torch.save(net.state_dict(), path)
                except Exception:
                    pass
        with torch.no_grad():
            _sv, _fr2, _vt2, _fw2, _gm2, _di2, _yd2, _wo2, _spv2 = deploy_eval()
        R._log(world, "DISTILL-BC DONE: FREE student eval surv=%.3f dist=%.2f gmatch=%.3f (PPO fine-tune next)"
               % (_sv, _di2, _gm2))
        last_a = torch.zeros(K, ACT_DIM, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
        reset_fields(torch.ones(K, dtype=torch.bool, device=tdev), vx_max); phase_t.zero_()
        hstate = net.init_hidden(K, tdev)

    for it in range(iters):
        if w_link > 0 and ghost_lp_M is not None and lp_ids_rt is None:   # one-time: FK-match link ids (v2 link reward)
            _sq = qpos_t.clone(); _svv = qvel_t.clone()
            qpos_t[:, leg_qadr_t] = (ghost_leg_M[0] if ghost_leg_M is not None else ghost_leg_t[0])
            if use_armg:
                qpos_t[:, arm_qadr_t] = (ghost_arm_M[0] if ghost_arm_M is not None else ghost_arm_t[0])
            if elb_qadr_t2 is not None:
                qpos_t[:, elb_qadr_t2] = (ghost_elb_M[0] if ghost_elb_M is not None else _f("ELB_HANG_REF", 0.30))
            ctrl_t[:, act_use_t] = qpos_t[:, qadr_use_t]; ctrl_t[:, act_use_t + 1] = 0.0
            physics()
            _rot0 = _yawrot_inv if _lp_heading else _qrot_inv
            _bf0 = _rot0(qpos_t[:1, 3:7], xpos_t[:1] - qpos_t[:1, 0:3].unsqueeze(1))[0]
            _r0 = ghost_lp_M[0].reshape(-1, 3)
            _idsL = [int(((_bf0 - _r0[k]) ** 2).sum(1).argmin()) for k in range(_r0.shape[0])]
            lp_ids_rt = torch.tensor(_idsL, dtype=torch.long, device=tdev)
            qpos_t.copy_(_sq); qvel_t.copy_(_svv)
            R._log(world, "W_LINK body ids (FK-matched) = %s" % (_idsL,))
        if mtrack and ref_kp_t is not None and kp_ids_t is None:   # one-time: FK-match keypoint body ids at ref frame 0
            _sq = qpos_t.clone(); _svv = qvel_t.clone()
            qpos_t[:, qadr_use_t] = ghost_wb_t[0]; qvel_t.zero_()
            ctrl_t[:, act_use_t] = ghost_wb_t[0]; ctrl_t[:, act_use_t + 1] = 0.0
            physics()                                                # updates xpos_t at (approximately) the ref pose
            _bf0 = _qrot_inv(qpos_t[:1, 3:7], xpos_t[:1] - qpos_t[:1, 0:3].unsqueeze(1))[0]   # (nbody,3) body frame
            _r0 = ref_kp_t[0].reshape(-1, 3)
            _ids = [int(((_bf0 - _r0[k]) ** 2).sum(1).argmin()) for k in range(_r0.shape[0])]
            kp_ids_t = torch.tensor(_ids, dtype=torch.long, device=tdev)
            qpos_t.copy_(_sq); qvel_t.copy_(_svv)
            _errs = [float(((_bf0[_ids[k]] - _r0[k]) ** 2).sum() ** 0.5) for k in range(len(_ids))]
            R._log(world, "MTRACK kp body ids (FK-matched) = %s  match-err(m) = %s"
                   % (_ids, [round(e, 3) for e in _errs]))
        _ann = 1.0 + (_ann_final - 1.0) * (it / max(1, iters - 1))
        # LAT-SHRINK ANNEAL (round 9: snap 0.28->0.20 cost the beat; widths shrink like centers
        # morph -- gradually). GHOST_LAT_FINAL interpolates ONLY the roll-slot widths from
        # GHOST_RESIDUAL_LAT to the target over the run; pitch corridor untouched.
        _latf = _f("GHOST_LAT_FINAL", 0.0)
        if _latf > 0 and gres > 0:
            _lat0 = _f("GHOST_RESIDUAL_LAT", gres)
            gres_w[torch.tensor([1, 5, 7, 11], device=tdev)] = \
                _lat0 + (_latf - _lat0) * (it / max(1, iters - 1))
        if morph is not None:   # GHOST-MORPH: slide the control references toward the target
            _alpha = min(1.0, it / max(1, morph_iters))
            if abs(morph["omega_tgt"] - morph["omega_src"]) > 1e-9:
                omega = (1.0 - _alpha) * morph["omega_src"] + _alpha * morph["omega_tgt"]
            ghost_leg_t = torch.tensor(morph["src_leg"] * (1 - _alpha) + morph["tgt_leg"] * _alpha,
                                       dtype=torch.float32, device=tdev)
            if "src_arm" in morph:
                ghost_arm_t = morph["src_arm"] * (1 - _alpha) + morph["tgt_arm"] * _alpha
            if use_elbcorr:
                elb_cur = elb_start + (elb_tgt_f - elb_start) * _alpha
        vx_cap = min(vx_max, vx_start + (vx_max - vx_start) * it / max(1, vx_curr))
        lat_this = ctrl_lat if lat_max <= 0 else int(torch.randint(0, lat_max + 1, (1,)).item())
        ring_tr = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(lat_this)]
        h0_stored = _detach_h(hstate)   # actor hidden at the start of this T-window (for BPTT replay)
        with torch.no_grad():
            for t in range(T):
                if seq_mode:               # time-varying command from the root trajectory
                    _sb = (phase_t / twopi * NB).long() % NB
                    cmd_t.copy_(seq_cmd_t[_sb])
                    ytgt_t.copy_(seq_yaw_t[_sb])       # harness/velocity frame FOLLOWS the reference turn
                o, roll, pitch, yaw, bz = obs_of(last_a)
                oin = o + torch.randn_like(o) * obs_noise if obs_noise > 0 else o
                mean, std, hstate = net.act(oin, hstate); val = net.value(oin, priv_of())
                mean = torch.nan_to_num(mean, nan=0.0, posinf=3.0, neginf=-3.0).clamp(-5, 5)   # NaN/inf guard
                d = torch.distributions.Normal(mean, std)
                a = d.sample(); lp = d.log_prob(a).sum(-1)
                ac = torch.clamp(a, -1, 1); env_a = ac * act_scale * motor_t
                if lat_this > 0:
                    ring_tr.append(env_a); env_a = ring_tr.pop(0)
                ctrl_t[:, act_use_t] = nom_use + env_a; ctrl_t[:, act_use_t + 1] = 0.0
                if vc_rest and _i("CMD_RESAMPLE_TICKS", 0) > 0 and                         ((it * T + t) % _i("CMD_RESAMPLE_TICKS", 0)) == 0 and (it * T + t) > 0:
                    sample_cmd(torch.ones(K, dtype=torch.bool, device=tdev), vx_cap, snap=False)
                if _yr > 0 and _i("YAW_RESAMPLE_TICKS", 0) > 0 and                         ((it * T + t) % _i("YAW_RESAMPLE_TICKS", 0)) == 0 and (it * T + t) > 0:
                    # heading conditioning: mid-episode target changes DEMAND turning within a
                    # lifetime (nav1 only saw targets at spawn -> learned to hold, not to turn).
                    # Safe to snap: the obs error is capped +-0.5 and no corridor table moves.
                    ytgt_t.copy_((torch.rand(K, device=tdev) - 0.5) * 2.0 * _yr)
                if vc_rest:
                    _sl = _f("CMD_SLEW", 0.008)
                    cmd_t.add_((cmd_tgt_t - cmd_t).clamp(-_sl, _sl))
                _corr_any2 = (gres > 0 and not distill_free) or (teacher_off is not None)
                if gres > 0 and not distill_free:   # legs: ghost feedforward + bounded residual
                    _gb = ((phase_t + _f("PHASE_LEAD", 0.0)) / twopi * NB).long() % NB  # CONTROL lead: cancels PD lag; metric stays on true phase
                    # CORRIDOR ANNEALING (GHOST_RES_ANNEAL>1): widths grow linearly to xANNEAL over
                    # the run -- the policy gains freedom gradually, never leaving its stable state
                    # distribution (the BC-cliff distillation failed at surv 0.06; this is the ramp).
                    if vc_rest:
                        _g6 = _vc_gate_step().unsqueeze(1)
                        _ff6 = _g6 * ghost_leg_t[_gb] + (1.0 - _g6) * vc_stand_t
                        ctrl_t[:, leg_act_t2] = _ff6 + ac[:, liu_j] * (_gres_gated(_gb) * _ann) * motor_t
                    else:
                        ctrl_t[:, leg_act_t2] = ghost_leg_t[_gb] + ac[:, liu_j] * (_gres_gated(_gb) * _ann) * motor_t
                if _corr_any2:   # arm/elbow/shoulder corridors are DESIGN -- they survive distillation (legs go free)
                    _gb = ((phase_t + _f("PHASE_LEAD", 0.0)) / twopi * NB).long() % NB
                    if use_armcorr:   # shoulders: arm-ghost feedforward + bounded residual
                        _gbA2 = ((phase_t + _f("PHASE_LEAD_ARM", _f("PHASE_LEAD", 0.0))) / twopi * NB).long() % NB
                        ctrl_t[:, arm_act_t2] = ghost_arm_t[_gbA2] + ac[:, arm_liu] * (ares * _ann) * motor_t
                    if use_elbcorr:   # elbows: fixed-angle corridor (human hang; elb_cur morphs to target)
                        _er1 = elb_lut_t[_gb] if elb_lut_t is not None else elb_cur
                        ctrl_t[:, elb_act_t2] = _er1 + ac[:, elb_liu] * (eres * _ann) * motor_t
                    if use_shrycorr:  # shoulder roll/yaw: pin to target (side-mirrored), bounded residual
                        ctrl_t[:, shry_act_t2] = shry_tgtv_t + ac[:, shry_liu] * (shry_res * _ann) * motor_t
                if not whole:
                    ctrl_t[:, waist_act_t] = waist_nom_t; ctrl_t[:, waist_act_t + 1] = 0.0
                # external push (velocity kick) every push_int steps -> push-recovery robustness
                step_ctr += 1
                if push_int > 0 and step_ctr % push_int == 0:
                    qvel_t[:, 0:2] += torch.randn(K, 2, device=tdev) * push_vel
                _harness_apply(roll, pitch)   # previous-tick attitude; one tick of lag is fine for a harness
                physics(); phase_t.add_(omega * dt)
                if _leashlead > 0.0:          # PROGRESS-LOCK: keep the reference within _leashlead bins of the robot's actual climb x
                    torch.minimum(phase_t, (torch.searchsorted(seq_rootx_t, qpos_t[:, 0].clamp(_rootx_lo, _rootx_hi)).float() + _leashlead) / NB * twopi, out=phase_t)
                if seq_mode and seq_hold_end:
                    phase_t.clamp_(max=_tp5_seq)   # play the routine once, then HOLD (no wrap-seam snap)
                if _wzr > 0:                  # arc curriculum: the heading target rotates at wz
                    ytgt_t.add_(wz_t * dt)
                    ytgt_t.copy_(torch.atan2(torch.sin(ytgt_t), torch.cos(ytgt_t)))
                _o, roll, pitch, yaw, bz = obs_of(ac)
                _yc, _ys = torch.cos(ytgt_t), torch.sin(ytgt_t)
                fwd = _yc * qvel_t[:, 0] + _ys * qvel_t[:, 1]        # speed ALONG the target heading
                _lat = -_ys * qvel_t[:, 0] + _yc * qvel_t[:, 1]      # lateral drift in the target frame
                # TENT (V-shaped) velocity reward: peaks at the commanded speed, gradient on BOTH sides
                # -> pulls forward from standstill (exp is flat far from target) AND penalizes over-speed
                # LUNGING (capped-linear rewards it, which re-destabilized). Converges to fwd ~= cmd.
                r_lin = torch.clamp(1.0 - torch.abs(fwd - cmd_t[:, 0]) / torch.clamp(cmd_t[:, 0], min=0.1), min=-1.0)
                # HEADING: face the TARGET heading (ytgt; 0 = the legacy straight-walk) and no
                # target-lateral drift. TENT on the heading error (2026-07-06: the exp form is
                # SATURATED at the training errors +-0.5 -- exp(-4) ~ 0.02, zero gradient; the
                # SAME flat-far-from-target trap the TENT velocity reward fixed. Four "turning"
                # campaigns failed on a gradientless reward). Lateral drift keeps the exp form.
                _ye9 = yaw - ytgt_t
                _ye9 = torch.atan2(torch.sin(_ye9), torch.cos(_ye9))
                r_ang = (torch.clamp(1.0 - _ye9.abs() / 0.5, min=-1.0)
                         + torch.exp(-(_lat ** 2) / (vtsig * vtsig))) * 0.5
                r_feet = foot_track_reward()
                bidx = (phase_t / twopi * NB).long() % NB
                # SEQ_TERRAIN: the base-height reference RISES with the climb (ghost root z) so the
                # ascent isn't penalized as "off nominal height"; flat runs keep the fixed Z_TGT.
                _zref = seq_rootz_t[bidx] if seq_terrain else z_tgt
                # VC-REST: the imitation REFERENCE must blend to the stand exactly like the
                # corridor does -- vc1/vc2 collapsed (surv 0.007 in 100 iters) because at
                # cmd=0 r_ghost still rewarded tracking the MARCHING gait while the corridor
                # held the stand: the policy was punished for obeying its own feedforward.
                _rrM = rr_metric and ghost_leg_M is not None
                if _rrM:
                    _bidxR = (phase_t / twopi * NBM).long() % NBM
                    _legref_t = ghost_leg_M[_bidxR]
                else:
                    _legref_t = ghost_leg_t[bidx]
                if vc_rest:
                    _g9 = vc_glat.unsqueeze(1)
                    _lref9 = _g9 * _legref_t + (1.0 - _g9) * vc_stand_t
                else:
                    _lref9 = _legref_t
                r_ghost = torch.exp(-((qpos_t[:, leg_qadr_t] - _lref9) ** 2).mean(1)
                                    / (ghost_sig * ghost_sig))     # soft leg-gait imitation (the anti-dive anchor)
                r_armg = 0.0
                if use_armg:
                    _aref9 = (ghost_arm_M[_bidxR] if (_rrM and ghost_arm_M is not None)
                              else ghost_arm_t[bidx])
                    if vc_rest:      # arms: swing with the gait, hang (~0.30) at rest
                        _aref9 = _g9[:, :1] * _aref9 + (1.0 - _g9[:, :1]) * 0.30
                    r_armg = torch.exp(-((qpos_t[:, arm_qadr_t] - _aref9) ** 2).mean(1)
                                       / (armg_sig * armg_sig))
                r_wb = 0.0
                if w_wb > 0 and ghost_wb_t is not None and whole:   # whole-body imitation: arms/waist/elbows track wb_lut
                    r_wb = torch.exp(-((qpos_t[:, qadr_use_t] - ghost_wb_t[bidx]) ** 2).mean(1)
                                     / (wb_sig * wb_sig))
                r_kp = 0.0
                if mtrack and ref_kp_t is not None:   # KEYPOINT tracking (feet/hands positions): forgiving, lets the policy deviate to balance
                    r_kp = torch.exp(-((sim_kp() - ref_kp_t[bidx]) ** 2).mean(1) / (kp_sig * kp_sig))
                arate = ((ac - prev_a) ** 2).sum(1)
                r_attg = 0.0
                if w_attg > 0 and ghost_att_t is not None:   # calm-sway tracking (metric ghost's attitude)
                    _nbam = NBM if ghost_leg_M is not None else NB
                    _bam = (phase_t / twopi * _nbam).long() % _nbam
                    _ar9 = ghost_att_t[_bam, 0]; _ap9 = ghost_att_t[_bam, 1]
                    if vc_rest:
                        _ga9 = vc_glat
                        _ar9 = _ga9 * _ar9; _ap9 = _ga9 * _ap9      # level attitude at rest
                    r_attg = torch.exp(-((roll - _ar9) ** 2
                                         + (pitch - _ap9) ** 2) / (attg_sig * attg_sig))
                r_elb = 0.0; r_link = 0.0
                if (w_elb > 0 or w_link > 0):
                    _nbam2 = NBM if ghost_leg_M is not None else NB
                    _bam2 = (phase_t / twopi * _nbam2).long() % max(1, _nbam2)
                if w_elb > 0 and elb_qadr_t2 is not None:
                    # elbows -> the v2 reference (recorded channel else the hang constant)
                    _eref2 = (ghost_elb_M[_bam2] if ghost_elb_M is not None else _f("ELB_HANG_REF", 0.30))
                    r_elb = torch.exp(-((qpos_t[:, elb_qadr_t2] - _eref2) ** 2).mean(1)
                                      / (armg_sig * armg_sig))
                if w_link > 0 and ghost_lp_M is not None and lp_ids_rt is not None:
                    # skeleton link positions (pelvis frame) -> the ruler's FK'd lp_lut: train ON
                    # the WBMATCH v2 links component (same sigma as the metric).
                    _lps2 = _lp_sim_of(lp_ids_rt)
                    _lsg2 = _f("LP_SIG", 0.10)
                    r_link = torch.exp(-((_lps2 - ghost_lp_M[_bam2]) ** 2).mean(1) / (_lsg2 * _lsg2))
                r_gsched = 0.0
                _wgs = _f("W_GAITSCHED", 0.0)
                if _wgs > 0 and lut_swing is not None:
                    # CONTACT-SCHEDULE reward (round 8): the binary contact state must match the
                    # phase gates -- on the ground in the stance window, airborne in the swing
                    # window. W_FEET (height-profile) was falsified for RHYTHM: clean CV went
                    # 41.5% -> 53.7% (feet traced z precisely while contact timing worsened).
                    _gl9 = lut_swing[0][bidx]; _gr9 = lut_swing[1][bidx]
                    _cl9 = (xpos_t[:, fL, 2] < z_stance + 0.03).float()
                    _cr9 = (xpos_t[:, fRi, 2] < z_stance + 0.03).float()
                    r_gsched = 0.5 * ((1 - _gl9) * _cl9 + _gl9 * (1 - _cl9)
                                      + (1 - _gr9) * _cr9 + _gr9 * (1 - _cr9))
                r_gap = 0.0
                _wgap = _f("W_GAP", 0.0)
                if _wgap > 0:
                    # STANCE-WIDTH reward (round 10, owner: "gap ~0.3 not 0.5"): every settle
                    # SLIDES narrow once arm momentum unblocks the corridor magnet; nothing else
                    # pins the width. Directly reward the foot lateral gap at GAP_TARGET.
                    _dg9 = xpos_t[:, fL, :2] - xpos_t[:, fRi, :2]                 # body-frame LATERAL gap:
                    _gp9 = (-torch.sin(yaw) * _dg9[:, 0] + torch.cos(yaw) * _dg9[:, 1]).abs()  # yaw-proof, no stride mix
                    _gs9 = _f("GAP_SIG", 0.06)
                    r_gap = torch.exp(-((_gp9 - _f("GAP_TARGET", 0.30)) ** 2) / (_gs9 * _gs9))
                r_stop = 0.0
                _wstop = _f("W_STOP", 0.0)
                if _wstop > 0:
                    # STOP reward: the stop is an undertrained skill (stopv ~0.2 m/s creep across
                    # every gate variant) -- survival metrics never punished creeping. Rewards
                    # actually standing still (planar base speed) ONLY in cmd=0 windows.
                    _sm9 = (cmd_t[:, 0].abs() < 0.02).float()
                    _ssig = _f("STOP_SIG", 0.10)
                    r_stop = _sm9 * torch.exp(-(qvel_t[:, 0] ** 2 + qvel_t[:, 1] ** 2) / (_ssig * _ssig))
                if mtrack:   # MOTION-TRACKING recipe (DeepMimic/GMT/BeyondMimic shape): keypoint + light joint match,
                    # NO walking terms (velocity/heading/foot-swing), NO attitude/corridor -- balance is EMERGENT
                    # (track the reference AND don't fall). Full action authority comes from GHOST_RESIDUAL=0.
                    rew = (w_kp * r_kp + w_wb * r_wb + walive
                           - wup * (roll * roll + pitch * pitch) - wheight * (bz - _zref) ** 2
                           - warate * arate - apen * (env_a * env_a).sum(1)
                           - w_wobble * (qvel_t[:, 3:6] ** 2).sum(1) - w_bounce * qvel_t[:, 2] ** 2)
                else:
                    rew = (wtl * r_lin + wta * r_ang + wghost * r_ghost + w_armg * r_armg + w_attg * r_attg
                           + w_elb * r_elb + w_link * r_link + _wgs * r_gsched + _wgap * r_gap
                           + w_wb * r_wb + _wstop * r_stop + wfeet * r_feet + walive
                           - wvshort * torch.clamp(cmd_t[:, 0] - fwd, min=0.0) / torch.clamp(cmd_t[:, 0], min=0.1)
                           - wup * (roll * roll + pitch * pitch) - wheight * (bz - _zref) ** 2
                           - warate * arate - apen * (env_a * env_a).sum(1)
                           - wypos * torch.clamp(qpos_t[:, 1].abs() - ypos_dead, min=0.0, max=3.0)   # bounded anti-drift
                           - w_wobble * (qvel_t[:, 3:6] ** 2).sum(1) - w_bounce * qvel_t[:, 2] ** 2)   # anti-wobble
                fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz) | ~torch.isfinite(bz)  # NaN = dead, not immortal
                if _tube_dz > 0.0:   # GHOST-TUBE ET: sinking below the phase-advanced ghost climb height = death (forces the step-up)
                    fell = fell | (bz < (seq_rootz_t[bidx] - _tube_dz))
                rew = rew - fpen * fell.float()
                # SEQ window: completing the segment is SUCCESS -- episode ends (bootstrap cut,
                # no fall penalty) and the world respawns inside the window.
                if seq_win:
                    winx = ((phase_t / twopi * NB) >= float(seq_hi)) & (~fell)
                    ends = fell | winx
                else:
                    ends = fell
                O[t] = oin; P[t] = priv_of(); A[t] = a; LP[t] = lp; V[t] = val; RW[t] = rew; DN[t] = ends.float()
                prev_a = torch.where(ends.unsqueeze(1), torch.zeros_like(ac), ac)
                vx_acc += fwd.detach().mean(); vx_n += 1
                ep_ret = ep_ret + rew; ff = ends.float()
                done_sum += (ep_ret * ff).sum(); done_cnt += ff.sum(); ep_ret = ep_ret * (1.0 - ff)
                last_a = torch.where(ends.unsqueeze(1), torch.zeros_like(ac), ac)
                reset_fields(ends, vx_cap)
            o, roll, pitch, yaw, bz = obs_of(last_a)
            lastv = net.value(o, priv_of())
            adv = torch.zeros(T, K, device=tdev); gae = torch.zeros(K, device=tdev)
            for t in reversed(range(T)):
                nv_ = lastv if t == T - 1 else V[t + 1]
                nonterm = 1.0 - DN[t]
                delta = RW[t] + gamma * nv_ * nonterm - V[t]
                gae = delta + gamma * lam * nonterm * gae; adv[t] = gae
            ret = adv + V
        total_steps += T * K
        if net.arch not in ("lstm", "gru"):
            bO = O.reshape(-1, OBS_DIM); bP = P.reshape(-1, PRIV_DIM); bA = A.reshape(-1, ACT_DIM)
            bLP = LP.reshape(-1); bRET = ret.reshape(-1)
            bADV = adv.reshape(-1); bADV = (bADV - bADV.mean()) / (bADV.std() + 1e-8)
            N = bO.shape[0]
            for _e in range(epochs):
                perm = torch.randperm(N, device=tdev)
                for mb in perm.chunk(nmb):
                    mean, std, _ = net.act(bO[mb]); val = net.value(bO[mb], bP[mb])
                    mean = torch.nan_to_num(mean, nan=0.0, posinf=3.0, neginf=-3.0).clamp(-5, 5)   # NaN/inf guard
                    d = torch.distributions.Normal(mean, std)
                    lp = d.log_prob(bA[mb]).sum(-1); ent = d.entropy().sum(-1).mean()
                    ratio = torch.exp(lp - bLP[mb]); a_ = bADV[mb]
                    pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                    vl = ((val - bRET[mb]) ** 2).mean()
                    loss = pl + vf_c * vl - ent_c * ent
                    if not torch.isfinite(loss):
                        continue
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step()
        else:
            # RECURRENT PPO: minibatch over WORLDS; replay each T-window in ONE cuDNN call from the
            # stored window-start hidden (truncated BPTT). Critic stays a stateless MLP over (obs,priv).
            adv_n = (adv - adv.mean()) / (adv.std() + 1e-8)   # (T,K)
            for _e in range(epochs):
                perm = torch.randperm(K, device=tdev)
                for mb in perm.chunk(nmb):
                    if isinstance(h0_stored, tuple):
                        h0 = (h0_stored[0][:, mb].contiguous(), h0_stored[1][:, mb].contiguous())
                    else:
                        h0 = h0_stored[:, mb].contiguous()
                    mean, std = net.act_seq(O[:, mb], h0)                     # (T,|mb|,act)
                    mean = torch.nan_to_num(mean, nan=0.0, posinf=3.0, neginf=-3.0).clamp(-5, 5)
                    nmw = mb.numel()
                    val = net.value(O[:, mb].reshape(-1, OBS_DIM),
                                    P[:, mb].reshape(-1, PRIV_DIM)).reshape(T, nmw)
                    d = torch.distributions.Normal(mean, std)
                    lp = d.log_prob(A[:, mb]).sum(-1); ent = d.entropy().sum(-1).mean()
                    ratio = torch.exp(lp - LP[:, mb]); a_ = adv_n[:, mb]
                    pl = -torch.min(ratio * a_, torch.clamp(ratio, 1 - clip, 1 + clip) * a_).mean()
                    vl = ((val - ret[:, mb]) ** 2).mean()
                    loss = pl + vf_c * vl - ent_c * ent
                    if not torch.isfinite(loss):
                        continue
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step()
        if it % 10 == 0:
            cnt = float(done_cnt); mret = float(done_sum / done_cnt) if cnt > 0 else 0.0
            fps = total_steps / max(1e-6, _time.time() - t0)
            R._log(world, "WALK-GPU it=%d epret~%.1f mfwd=%.3f(cap%.2f) std=%.2f neps=%d steps/s=%.0f"
                   % (it, mret, float(vx_acc) / max(1, vx_n), vx_cap, float(net.log_std.exp().mean()), int(cnt), fps))
            _status_write("TRAINING", it, iters)
            try:
                torch.save(net.state_dict(), path)
                ce = _i("CKPT_EVERY", 0)
                if ce > 0 and it % ce == 0 and it > 0 and path:
                    torch.save(net.state_dict(), path.replace(".pt", "_it%d.pt" % it))
            except Exception:
                pass
            t0 = _time.time(); total_steps = 0
            done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
            vx_acc = torch.zeros((), device=tdev); vx_n = 0
        if eval_every > 0 and it > 0 and it % eval_every == 0:
            with torch.no_grad():
                surv, frate, vt, fwd, gm, dist, ydrift, wob, stopv = deploy_eval()
            R._log(world, "  DEPLOY-EVAL it=%d surv=%.3f fall=%.3f vtrack=%.3f fwd=%.3f gmatch=%.3f dist=%.2f ydrift=%.2f wob=%.3f stopv=%.3f"
                   % (it, surv, frate, vt, fwd, gm, dist, ydrift, wob, stopv))
            if xfrc_t is not None:
                _att_ok = getattr(world, "_wr_last_att", 1.0) >= _f("HARNESS_GRAD_ATT", 0.0)
                _grad = surv >= _f("HARNESS_GRAD_SURV", 0.9) and _att_ok and harness_lam > 0
                if _grad:
                    harness_lam = max(0.0, harness_lam - _f("HARNESS_STEP", 0.1))
                R._log(world, "  HARNESS lam=%.2f %s (att %.2f%s)" % (harness_lam,
                       "(GRADUATED a level)" if _grad else "(holding this level)",
                       getattr(world, "_wr_last_att", -1.0), "" if _att_ok else " BELOW GATE: no lean rides the ladder"))
            reset_fields(torch.ones(K, dtype=torch.bool, device=tdev), vx_cap)   # no forward (OOM); step recomputes
            phase_t.zero_(); ep_ret = torch.zeros(K, device=tdev)
            last_a = torch.zeros(K, ACT_DIM, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
            hstate = net.init_hidden(K, tdev)   # all worlds freshly reset -> clear stale recurrent state
    try:
        torch.save(net.state_dict(), path)
    except Exception:
        pass
    _status_write("DONE", iters, iters)
    R._log(world, "TRAIN COMPLETE: %d iters -- final ckpt saved, status DONE (the launcher watchdog stops the sim)" % iters)


def _status_write_final_done():
    """Jobserver shutdown: emit the real DONE so the launcher watchdog reclaims the sim."""
    os.environ.pop("JOBSERVER", None)
    _status_write("DONE")


def _status_write(state, it=0, total=0):
    # under the JOBSERVER, per-job DONE must not kill the session (the launcher watchdog
    # greps the status file); report TRAINING until the server itself shuts down.
    if state == "DONE" and os.environ.get("JOBSERVER") == "1":
        state = "TRAINING"
    """Machine-readable heartbeat: <RES_LOG>.status -- {"state","it","iters","ts"}.
    STRUCTURAL FIX (owner, 2026-07-04): the in-engine trainer cannot stop the simulator
    itself, so a finished run used to IDLE silently until the wall budget expired, and a
    stalled one looked identical to a running one. The trainer now REPORTS its state here
    every log interval and writes state=DONE at every terminal point; run_walk_rl.sh's
    watchdog ENFORCES (stops the sim tree on DONE, kills + exit 3 on heartbeat stall)."""
    p = os.environ.get("RES_LOG")
    if not p:
        return
    try:
        import json as _j
        import time as _t
        with open(p + ".status", "w") as f:
            f.write(_j.dumps({"state": state, "it": int(it), "iters": int(total), "ts": _t.time()}))
    except Exception:
        pass


def g1_walk_recipe_step(world):
    if getattr(world, "_wr_started", False):
        return
    if not hasattr(world, "_wr_ctr"):
        world._wr_ctr = 0
    world._wr_ctr += 1
    if world._wr_ctr < _i("WALK_WARM_TICKS", 360):
        return
    world._wr_started = True
    try:
        _walk_recipe_train(world)
    except Exception as e:
        import traceback
        R._log(world, "walk-recipe err %r\n%s" % (e, traceback.format_exc()))


def _deploy_setup(world):
    torch = _torch()
    if not WM._build(world):
        return False
    slots = world._gwm_slots
    leg_qadr, leg_dadr = _build_legmap(world)
    whole = _i("WHOLE_BODY", 0)
    global ACT_DIM, OBS_DIM
    sp = world.solver.mjw_data.qpos.numpy().reshape(-1).copy()
    if _i("STAND_SEED", 0):   # match training: legs seeded to the crouch stand
        sp[leg_qadr] = _seed_legs()
    if whole:
        qadr_a, dadr_a, aidx_a = _build_all_pos_act(world)
        m2nd = world.solver.mjc_jnt_to_newton_dof.numpy()
        if m2nd.ndim == 2:
            m2nd = m2nd[0]
        mjm = world.solver.mj_model
        trnid = np.asarray(mjm.actuator_trnid).reshape(int(mjm.nu), -1)
        world._wr_ndof = np.array([int(m2nd[int(trnid[a][0])]) for a in aidx_a], np.int32)  # newton dof per joint
        # DIAG: the leg subset of the whole-body ndof must equal the proven leg-only gait dof map
        try:
            leg_gwm = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
            leg_wb = [int(world._wr_ndof[list(aidx_a).index(a)]) for a in aidx_a]  # all, in aidx order
            R._log(world, "DIAG wb_ndof(all)=%s leg_gwm=%s wb_qadr=%s leg_qadr=%s" %
                   (list(world._wr_ndof), leg_gwm, list(qadr_a), list(leg_qadr)))
        except Exception as e:
            R._log(world, "DIAG map err %r" % (e,))
        world._wr_qadr = qadr_a; world._wr_dadr = dadr_a
        world._wr_nom = sp[qadr_a].copy(); world._wr_whole = True
        ACT_DIM = len(aidx_a); OBS_DIM = 13 + 3 * ACT_DIM
    else:
        world._wr_legpos = [p for p, sl in enumerate(slots) if sl < 12]
        world._wr_waistpos = [p for p, sl in enumerate(slots) if sl >= 12]
        world._wr_gdof = [int(world._gwm_dof[p]) for p in range(len(slots))]
        world._wr_qadr = leg_qadr; world._wr_dadr = leg_dadr
        world._wr_nom = sp[leg_qadr].copy(); world._wr_whole = False
        legs, _a, _s = world._gwm_stg.targets_np(world._gwm_phase0, world._gwm_gp, t_since_start=10.0)
        world._wr_waistnom = {p: float(np.asarray(legs)[slots[p]]) for p in world._wr_waistpos}
        ACT_DIM = 12; OBS_DIM = 13 + 3 * 12
    # REF_OBS deploy support: append the reference block to the obs so a reference-in-obs policy
    # (community-tracker) runs live. Must expand OBS_DIM BEFORE the net is sized.
    world._wr_ref_on, world._wr_ref_K, world._wr_ref_stride = _refobs.ref_params()
    world._wr_reftgt = None; world._wr_refatt = None
    if world._wr_ref_on and os.environ.get("GHOST_LUT_JSON"):
        import json as _rjson
        _rgd = _rjson.loads(open(os.environ["GHOST_LUT_JSON"]).read())
        _ref_wb_on = bool(_i("REF_OBS_WB", 0) and "wb_lut" in _rgd and whole)
        world._wr_reftgt = np.asarray(_rgd["wb_lut" if _ref_wb_on else "leg_lut"], np.float32)
        # ref deltas index the TRACKED joints (12 legs unless wb): whole-body _wr_qadr is 23-wide
        world._wr_ref_qadr = np.asarray(world._wr_qadr if _ref_wb_on else leg_qadr)
        world._wr_refatt = np.asarray(_rgd["att_lut"], np.float32) if "att_lut" in _rgd else None
        world._wr_refnb = int(_rgd["nb"]); world._wr_ref_useatt = world._wr_refatt is not None
        OBS_DIM += _refobs.ref_block_dim(world._wr_reftgt.shape[1], world._wr_ref_K, world._wr_ref_useatt)
        R._log(world, "DEPLOY REF_OBS: tgt_w=%d wb=%s att=%s -> OBS_DIM=%d"
               % (world._wr_reftgt.shape[1], _ref_wb_on, world._wr_ref_useatt, OBS_DIM))
    net = _make_ac(_i("PPO_HID", 256))
    net.load_state_dict(torch.load(os.environ.get("RES_POLICY", ""), map_location="cpu"))
    net.eval(); world._wr_net = net
    world._wr_h = net.init_hidden(1, "cpu")   # actor recurrent hidden (None for MLP), carried live
    # SECOND POLICY (owner architecture, walk-stop-walk): STAND_POLICY=<ckpt> loads an ACTIVE
    # stand network (same trainer, same obs/act layout, e.g. runs/wr_standwb.pt). The
    # scheduler's stand segments run THIS net instead of a pose hold -- five pose-hold catch
    # variants all fell within 1-3.5 s (rigid holds rebound; snapped/morphed holds drag feet).
    world._wr_net2 = None
    _sp2 = os.environ.get("STAND_POLICY")
    if _sp2 and os.path.exists(_sp2):
        try:
            net2 = _make_ac(_i("PPO_HID", 256))
            net2.load_state_dict(torch.load(_sp2, map_location="cpu"))
            net2.eval(); world._wr_net2 = net2
            world._wr_h2 = net2.init_hidden(1, "cpu")
            R._log(world, "STAND_POLICY loaded: %s (active stand for schedule stand segments)"
                   % os.path.basename(_sp2))
        except Exception as _e2:
            R._log(world, "STAND_POLICY load FAILED: %r -- stand segments fall back to pose hold" % (_e2,))
    if os.environ.get("GHOST_LUT_JSON"):      # recorded-gait clock: deploy phase must match training
        try:
            import json as _json
            world._gwm_omega = 2.0 * math.pi * float(_json.loads(open(os.environ["GHOST_LUT_JSON"]).read()).get("freq", 1.25))
            R._log(world, "deploy clock from GHOST_LUT_JSON: omega=%.3f" % world._gwm_omega)
        except Exception as _e:
            R._log(world, "GHOST_LUT_JSON clock err %r" % (_e,))
    world._wr_gres = _f("GHOST_RESIDUAL", 0.0)
    world._wr_greslat = _f("GHOST_RESIDUAL_LAT", world._wr_gres)
    world._wr_gresyaw = _f("GHOST_RESIDUAL_YAW", world._wr_gres)   # hip-yaw slack (turning)
    if world._wr_gres > 0 and os.environ.get("GHOST_LUT_JSON"):
        import json as _json
        _gd = _json.loads(open(os.environ["GHOST_LUT_JSON"]).read())
        world._wr_glut = np.asarray(_gd["leg_lut"], np.float32)      # (nb,12) slot order
        world._wr_gnb = int(_gd["nb"])
        # leg position within the whole-body action vector (slot order), for the residual
        _laq, _lad = _build_legmap(world)
        world._wr_liu = [list(world._wr_qadr).index(int(q)) for q in _laq]
        world._wr_legnd = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
        # phase-gated stance corridor (train/deploy parity): swing gates from the recorded knees
        world._wr_stight = _f("GHOST_STANCE_TIGHT", 0.0)
        if 0.0 < world._wr_stight < 1.0:
            _kn = world._wr_glut[:, [3, 9]]
            world._wr_swgate = (_kn - _kn.min(0)) / np.maximum(1e-6, _kn.max(0) - _kn.min(0))
            R._log(world, "DEPLOY STANCE-GATED CORRIDOR: pitch width x%.2f in stance" % world._wr_stight)
        R._log(world, "DEPLOY GHOST-RESIDUAL: legs = ghost(phase) +/- %.3f rad" % world._wr_gres)
        world._wr_hlam = _f("HARNESS_LAM0", 0.0)
        if world._wr_hlam > 0:   # deploy-side harness -- via the backend's SANCTIONED force channel.
            # Direct mjw xfrc writes are DEAD here: clear_forces() zeroes Newton body_f every substep
            # and the solver re-derives xfrc from it (lift-probe proven: 20kN skyhook, zero lift).
            # add_body_force() is the W3.1 external-wrench API: applied after clear_forces each
            # substep, world frame [F,T], NEWTON body index (pelvis = 5), re-apply per tick.
            world._wr_hpb = int(_f("HARNESS_NEWTON_BODY", 5))
            world._wr_hx = None
            R._log(world, "DEPLOY HARNESS (add_body_force): lam=%.2f newton_body=%d" % (world._wr_hlam, world._wr_hpb))
        world._wr_ares = _f("ARM_RESIDUAL", 0.0)
        # run the arm/elbow detection when EITHER corridor is requested (the free-shoulder +
        # straight-elbow policy has ares=0 but still needs the elbow corridor live)
        if world._wr_ares > 0 or os.environ.get("ELBOW_TARGET") is not None:
            # geometric shoulder detection (names don't survive to mjm) -> ndof + action position
            import mujoco as _mjn
            _mjm2 = world.solver.mj_model
            _trn3 = np.asarray(_mjm2.actuator_trnid).reshape(int(_mjm2.nu), -1)
            _dcl2 = _mjn.MjData(_mjm2); _mjn.mj_forward(_mjm2, _dcl2)
            _pz = 0.7
            for j in range(int(_mjm2.njnt)):
                if int(_mjm2.jnt_type[j]) == int(_mjn.mjtJoint.mjJNT_FREE):
                    _pz = float(_dcl2.xpos[int(_mjm2.jnt_bodyid[j])][2]); break
            _qadr_a, _dadr_a, _aidx_a = _build_all_pos_act(world)
            _cand2 = {1.0: [], -1.0: []}   # side -> [(z, action-idx, newton-dof, qadr)]; +0.05 so the ELBOW (~11cm above pelvis) qualifies
            for i, a in enumerate(_aidx_a):
                j = int(_trn3[int(a)][0])
                ax = np.abs(np.array(_mjm2.jnt_axis[j], float))
                pos = np.array(_dcl2.xpos[int(_mjm2.jnt_bodyid[j])], float)
                if int(np.argmax(ax)) == 1 and pos[2] > _pz + 0.05 and abs(pos[1]) > 0.05:
                    sd = 1.0 if pos[1] > 0 else -1.0
                    _cand2[sd].append((float(pos[2]), i, int(m2nd[j]), int(_mjm2.jnt_qposadr[j])))
            _sh = {sd: (max(_cand2[sd]) if _cand2[sd] else None) for sd in (1.0, -1.0)}      # highest = shoulder
            _el = {sd: (sorted(_cand2[sd])[-2] if len(_cand2[sd]) > 1 else None) for sd in (1.0, -1.0)}  # next down = elbow
            _elbt2 = os.environ.get("ELBOW_TARGET")
            if _elbt2 in ("", "off"):
                _elbt2 = None
            if _elbt2 is not None and _el[1.0] and _el[-1.0]:
                world._wr_elbliu = [_el[1.0][1], _el[-1.0][1]]
                world._wr_elbnd = [_el[1.0][2], _el[-1.0][2]]
                world._wr_elbtgt = float(_elbt2); world._wr_eres = _f("ELBOW_RESIDUAL", 0.15)
                world._wr_elblut = np.asarray(_gd["elbow_lut"], np.float32) if "elbow_lut" in _gd else None
                R._log(world, "DEPLOY ELBOW-CORRIDOR: %s +/- %.2f rad"
                       % ("elbow_lut cycle" if world._wr_elblut is not None else ("%.2f" % world._wr_elbtgt),
                          world._wr_eres))
            _shryt2 = os.environ.get("SHRY_TARGET")
            if _shryt2 in ("", "off"):
                _shryt2 = None
            if _shryt2 is not None and _sh[1.0] and _sh[-1.0]:
                # shoulder ROLL/YAW corridor live (positional: qadr+1/+2 of each shoulder pitch,
                # same layout the trainer verified with the elbow==pitch+3 sanity check)
                _q2i5 = {}
                for i5, q in enumerate(_qadr_a):   # newton dof via m2nd (tp[] is newton-indexed;
                    _j5 = int(_trn3[int(_aidx_a[i5])][0])   # _dadr_a is the MUJOCO dofadr -- wrong table)
                    _q2i5[int(q)] = (i5, int(m2nd[_j5]))
                world._wr_shryliu = []; world._wr_shrynd = []; world._wr_shrytgts = []
                _yawt5 = _f("SHRY_YAW_TARGET", 0.0)
                for sd in (1.0, -1.0):
                    for off in (1, 2):
                        _hit5 = _q2i5.get(int(_sh[sd][3]) + off)
                        if _hit5 is not None:
                            world._wr_shryliu.append(_hit5[0]); world._wr_shrynd.append(_hit5[1])
                            # side-mirrored per-channel target (roll: + = out on the LEFT only --
                            # the unsigned scalar pulled the RIGHT arm INTO the body live)
                            world._wr_shrytgts.append(sd * (float(_shryt2) if off == 1 else _yawt5))
                if world._wr_shryliu:
                    world._wr_shrytgt = float(_shryt2); world._wr_shryres = _f("SHRY_RESIDUAL", 0.15)
                    R._log(world, "DEPLOY SHRY-CORRIDOR: %d ch roll=%.2f*side yaw=%.2f*side +/- %.2f rad"
                           % (len(world._wr_shryliu), world._wr_shrytgt, _yawt5, world._wr_shryres))
            if _sh[1.0] and _sh[-1.0]:
                world._wr_armliu = [_sh[1.0][1], _sh[-1.0][1]]     # [left, right] position in action vec
                world._wr_armnd = [_sh[1.0][2], _sh[-1.0][2]]
                _aq = [_sh[1.0][3], _sh[-1.0][3]]
                if "arm_lut" in _gd:      # GHOST v3: recorded shoulders, order [left,right] = _wr_armliu order
                    world._wr_armlut = np.asarray(_gd["arm_lut"], np.float32)                   # (nb,2)
                else:                     # legacy: arm ghost derived from the leg LUT hips (opposite-hip lock)
                    _hl = world._wr_glut[:, 0]; _hr = world._wr_glut[:, 6]
                    _dl = np.clip((_hl - _hl.mean()) / max(1e-6, (_hl.max() - _hl.min()) / 2), -1, 1)
                    _dr = np.clip((_hr - _hr.mean()) / max(1e-6, (_hr.max() - _hr.min()) / 2), -1, 1)
                    _A = _f("ARM_SWING_A", 0.3)
                    world._wr_armlut = np.stack([sp[_aq[0]] + _A * _dr, sp[_aq[1]] + _A * _dl], 1)  # (nb,2)
                R._log(world, "DEPLOY ARM-CORRIDOR: shoulders = arm_ghost(phase) +/- %.3f rad" % world._wr_ares)
            else:
                world._wr_ares = 0.0
                R._log(world, "DEPLOY ARM-CORRIDOR: shoulders not resolved -> disabled")
    world._wr_phase = 0.0; world._wr_omega = world._gwm_omega
    # LIVE SEQUENCE deploy (owner: "show me the actual robot moving"): a GHOST_SEQ policy
    # was trained with a TIME-VARYING command from the routine's root profile -- feed the
    # live loop the same, and pin at the final bin when the lut says hold_end.
    world._wr_seqcmd = None; world._wr_seq_hold = False
    # LIVE VC (velocity-conditioned walker with rest): latched corridor gate + command cycle
    world._wr_vc = bool(_i("VC_REST", 0))
    world._wr_vc_glat = 1.0
    world._wr_vc_cyc = bool(_i("WALK_CMD_CYCLE", 0))
    # OWNER PROFILE: finite command schedule "dur:cmd,dur:cmd,..." in seconds; the LAST
    # segment holds forever (walk 5 / stop 5 / walk 5 / stand permanently = "5:0.45,5:0,5:0.45,1:0").
    world._wr_vc_prof = None
    _pf5 = os.environ.get("WALK_CMD_PROFILE", "")
    if _pf5:
        _segs5 = [(float(a), float(b)) for a, b in (t.split(":") for t in _pf5.split(","))]
        world._wr_vc_prof = _segs5
        R._log(world, "CMD-PROFILE: %s (last segment holds forever)" % _pf5)
    try:
        import json as _json5
        _gd5 = _json5.loads(open(os.environ["GHOST_LUT_JSON"]).read())
        if _gd5.get("seq") and "root_lut" in _gd5:
            _rl5 = np.asarray(_gd5["root_lut"], np.float32)
            _nb5 = int(_gd5["nb"])
            _dtb5 = (1.0 / float(_gd5.get("freq", 1.25))) / _nb5
            _vw5 = (np.roll(_rl5, -1, axis=0) - _rl5) / _dtb5
            _vw5[-1] = _vw5[-2]
            _cy5, _sy5 = np.cos(_rl5[:, 3]), np.sin(_rl5[:, 3])
            _vbx5 = _cy5 * _vw5[:, 0] + _sy5 * _vw5[:, 1]
            world._wr_seqcmd = np.clip(_vbx5, -1.5, 1.5).astype(np.float32)
            world._wr_seqcmd[-4:] = world._wr_seqcmd[-5]
            world._wr_seq_hold = bool(_gd5.get("hold_end", False))
            world._wr_seqnb = _nb5
            world._wr_seq_yaw = _rl5[:, 3].astype(np.float32)   # root yaw/bin: the DEPLOY heading
            #   frame must FOLLOW the turn exactly as training did (ytgt=seq_yaw), else the tracker's
            #   heading obs is off-distribution and it under-executes the turn (measured: 24 of 90deg).
            R._log(world, "LIVE-SEQ: time-varying cmd from root profile (|vx|<=%.2f), hold_end=%s, yaw-span %.0fdeg"
                   % (float(np.abs(world._wr_seqcmd).max()), world._wr_seq_hold,
                      float(np.ptp(world._wr_seq_yaw)) * 57.3))
    except Exception as _e5:
        R._log(world, "LIVE-SEQ setup skipped: %r" % (_e5,))
    world._wr_dt = world._gwm_dt; world._wr_as = _f("RES_ACT_SCALE", 0.5)
    world._wr_last = np.zeros(ACT_DIM, np.float32); world._wr_cmd = _f("VX_MAX", 0.7)
    # fixed heading target for the steering test / simple courses (BATON walkto sets it per tick)
    world._wr_heading_tgt = float(os.environ["HEADING_TARGET"]) if os.environ.get("HEADING_TARGET") else None
    # ── BATON (policy switching, 2026-07-06 owner campaign; docs/developer/policy-switching.md) ──
    # Two SPECIALIST policies share the deploy infrastructure: per tick the arbiter blends the
    # reference tables (corridors + REF_OBS + harness-att all read the blended luts -- morph, never
    # snap) and crossfades the two nets' actions. Both nets run EVERY tick (warm recurrent handover).
    world._wr_baton = None
    _bp = os.environ.get("BATON_POLICY_B")
    _bl = os.environ.get("BATON_LUT_B")
    if (_bp and _bl and os.path.exists(_bp) and os.path.exists(_bl)) or os.environ.get("BATON_SPECIALISTS"):
        try:
            import json as _bjson

            def _tables_of(_gd6, _vx6):
                return {
                    "glut": np.asarray(_gd6["leg_lut"], np.float32),
                    "arm": np.asarray(_gd6["arm_lut"], np.float32) if "arm_lut" in _gd6 else None,
                    "elb": np.asarray(_gd6["elbow_lut"], np.float32) if "elbow_lut" in _gd6 else None,
                    "att": np.asarray(_gd6["att_lut"], np.float32) if "att_lut" in _gd6 else None,
                    "ref": np.asarray(_gd6["leg_lut"], np.float32),
                    "vx": _vx6,
                }
            # specialist registry: "walk" = the primary policy + its live tables; extras come from
            # BATON_SPECIALISTS="name|ckpt|lut[|vx];..." (pipe/semicolon: Windows paths carry colons;
            # BATON_POLICY_B/LUT_B = the legacy two-specialist "stand" form)
            _reg = {"walk": {
                "net": world._wr_net, "h": None,   # h=None -> use world._wr_h (the primary hidden)
                "tables": {
                    "glut": world._wr_glut.copy(),
                    "arm": world._wr_armlut.copy() if getattr(world, "_wr_armlut", None) is not None else None,
                    "elb": world._wr_elblut.copy() if getattr(world, "_wr_elblut", None) is not None else None,
                    "att": world._wr_refatt.copy() if getattr(world, "_wr_refatt", None) is not None else None,
                    "ref": world._wr_reftgt.copy() if getattr(world, "_wr_reftgt", None) is not None else None,
                    "vx": _f("VX_MAX", 0.45),
                }}}
            _specs = os.environ.get("BATON_SPECIALISTS", "")
            if not _specs and _bp and _bl:
                _specs = "stand|%s|%s|0" % (_bp, _bl)
            for _entry in _specs.split(";"):
                _pp = _entry.split("|")
                _nm6 = _pp[0].strip()
                _ck6, _lu6 = _pp[1], _pp[2]
                _vx6 = float(_pp[3]) if len(_pp) > 3 else 0.0
                _nB = _make_ac(_i("PPO_HID", 256))
                _nB.load_state_dict(torch.load(_ck6, map_location="cpu"))
                _nB.eval()
                _reg[_nm6] = {"net": _nB, "h": _nB.init_hidden(1, "cpu"),
                              "tables": _tables_of(_bjson.loads(open(_lu6).read()), _vx6)}
            _sched = []
            for seg in os.environ.get("BATON_SCHEDULE", "walk:8,stand:6").split(","):
                nm5, s5 = seg.split(":")
                _sched.append((nm5.strip(), float(s5)))
            world._wr_baton = {
                "reg": _reg, "active": _sched[0][0], "target": _sched[0][0],
                "src": dict(_reg[_sched[0][0]]["tables"]),   # morph source snapshot (updated per switch)
                "sched": _sched, "period": sum(s for _, s in _sched),
                "morph": max(1, _i("BATON_MORPH_TICKS", 30)),
                "u": 1.0, "switches": 0,
            }
            R._log(world, "BATON armed: specialists=%s sched=%s morph=%d ticks (warm all-LSTM handover)"
                   % (sorted(_reg), os.environ.get("BATON_SCHEDULE", "walk:8,stand:6"), world._wr_baton["morph"]))
        except Exception as _eB:
            R._log(world, "BATON setup FAILED: %r" % (_eB,))
    if _i("STAND_SEED", 0) and _i("STAND_TELEPORT", 1):
        # teleport the live robot to the seeded crouch so it STARTS at the pose the policy was
        # trained/eval'd on (the deterministic stand collapses on the real foot, so we can't rely
        # on the warmup controller to hold it up).
        _wr_teleport_crouch(world, _f("STAND_Z", 0.72))
    _wr_patch_foot_torsion(world)   # spin friction so a planted foot doesn't freewheel (turning)
    # ── BATON SOLO-TURN (cross-cadence turn specialist, 2026-07-08) ──────────────────────────────
    # The 90deg step-turn specialist is a SLOW WHOLE-BODY SEQUENCE (nb~225 seq, REF_OBS_WB=23 cols,
    # omega~0.3) -- it CANNOT element-wise blend with the fast cyclic walk/carry family (nb=64,
    # REF_OBS=12, omega~7.85: the BATON blend does (1-u)*a+u*b on the reference tables, which needs a
    # shared nb AND a shared cadence). So a "turn" segment runs the turner SOLO via a full context
    # swap: on turn entry EVERY world._wr_* deploy field (net, hidden, ref tables, ghost lut, phase
    # clock, cmd) switches to the turn specialist's, so the whole deploy machinery transparently runs
    # it (obs-build reads _wr_reftgt/_wr_ref_qadr/_wr_refnb; act reads _wr_net/_wr_h; ghost-ff reads
    # _wr_glut/_wr_gnb; phase advances at _wr_omega). On exit the walk/blend context is restored
    # (cold-hidden both ways -- the warm-handover stand-attractor lock law). Gated on BATON_TURN_CKPT;
    # off = the shipped blend path is byte-identical. Needs OMNISIM_FOOT_TORSION on (real footwork).
    world._wr_turnctx = None; world._wr_in_turn = False; world._wr_turn_active = False
    _tck = os.environ.get("BATON_TURN_CKPT"); _tlu = os.environ.get("BATON_TURN_LUT")
    if _tck and _tlu and os.path.exists(_tck) and os.path.exists(_tlu):
        try:
            import json as _tj   # OBS_DIM is already `global` for this fn (declared at the top)
            _tgd = _tj.loads(open(_tlu).read())
            _t_wb = bool(_i("BATON_TURN_REF_WB", 1) and "wb_lut" in _tgd and whole)
            _t_reftgt = np.asarray(_tgd["wb_lut" if _t_wb else "leg_lut"], np.float32)
            _t_useatt = ("att_lut" in _tgd)
            # the turner has its OWN obs width: the walk family is 12-leg REF_OBS (OBS_DIM 120) but the
            # 90deg turner is REF_OBS_WB (23-col ref -> 153). Re-size OBS_DIM (a module global _make_ac
            # reads) to the turner's ref block for the load, then restore the primary's. PRIV_DIM is
            # model-derived -> unchanged.
            _prim_rb = (_refobs.ref_block_dim(int(world._wr_reftgt.shape[1]), world._wr_ref_K, world._wr_ref_useatt)
                        if getattr(world, "_wr_reftgt", None) is not None else 0)
            _turn_rb = _refobs.ref_block_dim(int(_t_reftgt.shape[1]), world._wr_ref_K, _t_useatt)
            _sav_obs = OBS_DIM
            OBS_DIM = _sav_obs - _prim_rb + _turn_rb
            try:
                _tnet = _make_ac(_i("PPO_HID", 256))
                _tnet.load_state_dict(torch.load(_tck, map_location="cpu")); _tnet.eval()
            finally:
                OBS_DIM = _sav_obs                     # restore the primary obs width no matter what
            # the turner is a SEQUENCE: its time-varying command (fwd vel) + heading come from the
            # ghost root trajectory (same derivation as LIVE-SEQ above). Without these the turn net
            # gets a static cmd and never executes the turn. Swapped in with the rest of the context.
            _t_seqcmd = _t_seqyaw = None; _t_seqnb = int(_tgd["nb"]); _t_seqhold = bool(_tgd.get("hold_end", False))
            if _tgd.get("seq") and "root_lut" in _tgd:
                _trl = np.asarray(_tgd["root_lut"], np.float32)
                _tdtb = (1.0 / float(_tgd.get("freq", 0.05))) / _t_seqnb
                _tvw = (np.roll(_trl, -1, axis=0) - _trl) / _tdtb; _tvw[-1] = _tvw[-2]
                _tcy, _tsy = np.cos(_trl[:, 3]), np.sin(_trl[:, 3])
                _t_seqcmd = np.clip(_tcy * _tvw[:, 0] + _tsy * _tvw[:, 1], -1.5, 1.5).astype(np.float32)
                _t_seqcmd[-4:] = _t_seqcmd[-5]
                _t_seqyaw = _trl[:, 3].astype(np.float32)
            world._wr_turnctx = {
                "net": _tnet, "reftgt": _t_reftgt,
                "ref_qadr": np.asarray(world._wr_qadr if _t_wb else leg_qadr),
                "refnb": int(_tgd["nb"]), "ref_useatt": _t_useatt,
                "refatt": np.asarray(_tgd["att_lut"], np.float32) if _t_useatt else None,
                "glut": np.asarray(_tgd["leg_lut"], np.float32), "gnb": int(_tgd["nb"]),
                "omega": 2.0 * math.pi * float(_tgd.get("freq", 0.05)), "cmd": 0.0,
                "armlut": np.asarray(_tgd["arm_lut"], np.float32) if "arm_lut" in _tgd else None,
                "elblut": np.asarray(_tgd["elbow_lut"], np.float32) if "elbow_lut" in _tgd else None,
                "seqcmd": _t_seqcmd, "seqnb": _t_seqnb, "seq_yaw": _t_seqyaw, "seq_hold": _t_seqhold,
            }
            R._log(world, "BATON SOLO-TURN armed: %s (nb=%d omega=%.3f wb_ref=%s obs=%d seq=%s yaw-span=%.0f) -- swapped in on turn segments"
                   % (os.path.basename(_tck), int(_tgd["nb"]), world._wr_turnctx["omega"], _t_wb,
                      _sav_obs - _prim_rb + _turn_rb, _t_seqcmd is not None,
                      float(np.ptp(_t_seqyaw)) * 57.3 if _t_seqyaw is not None else 0.0))
        except Exception as _et:
            R._log(world, "BATON SOLO-TURN setup FAILED: %r" % (_et,))
    R._log(world, "WALK-RECIPE DEPLOY ready whole=%d act=%d cmd_vx=%.2f" % (int(whole), ACT_DIM, world._wr_cmd))
    return True


def _wr_turn_swap(world, is_turn):
    """Swap the deploy context to/from the SOLO turn specialist on turn-segment boundaries (see the
    BATON SOLO-TURN note in the deploy setup). No-op unless a turn context was armed. Cold-hidden on
    every boundary (the warm-handover stand-attractor lock: never hand a translating specialist a
    stand-settled recurrent state)."""
    ctx = getattr(world, "_wr_turnctx", None)
    if ctx is None:
        world._wr_turn_active = bool(is_turn)
        return
    _fields = ("net", "h", "reftgt", "ref_qadr", "refnb", "ref_useatt", "refatt", "glut", "gnb",
               "omega", "cmd", "armlut", "elblut", "seqcmd", "seqnb", "seq_yaw", "seq_hold")
    if is_turn and not world._wr_in_turn:
        world._wr_blendctx = {k: getattr(world, "_wr_" + k, None) for k in _fields}
        world._wr_net = ctx["net"]; world._wr_h = ctx["net"].init_hidden(1, "cpu")   # cold in
        world._wr_reftgt = ctx["reftgt"]; world._wr_ref_qadr = ctx["ref_qadr"]
        world._wr_refnb = ctx["refnb"]; world._wr_ref_useatt = ctx["ref_useatt"]
        world._wr_refatt = ctx["refatt"]; world._wr_glut = ctx["glut"]; world._wr_gnb = ctx["gnb"]
        world._wr_omega = ctx["omega"]; world._wr_cmd = ctx["cmd"]
        world._wr_armlut = ctx["armlut"]; world._wr_elblut = ctx["elblut"]
        world._wr_seqcmd = ctx["seqcmd"]; world._wr_seqnb = ctx["seqnb"]
        world._wr_seq_yaw = ctx["seq_yaw"]; world._wr_seq_hold = ctx["seq_hold"]
        world._wr_phase = 0.0; world._wr_turn_acc = 0.0; world._wr_turn_done = False
        if hasattr(world, "_wr_yaw_prev"):
            del world._wr_yaw_prev    # TURN_TO_DEG seeds _yaw_prev from getattr default on tick 1; a
        if hasattr(world, "_wr_turn_done_t"):
            del world._wr_turn_done_t  # fresh settle window for this turn
        world._wr_in_turn = True      # leftover None here would TypeError (_ynow - None) every tick
        R._log(world, "BATON SOLO-TURN: swapped IN at t=%d (90deg footwork turner active)" % world._wr_tt)
    elif (not is_turn) and world._wr_in_turn:
        b = world._wr_blendctx
        world._wr_net = b["net"]; world._wr_h = b["net"].init_hidden(1, "cpu")        # cold out
        world._wr_reftgt = b["reftgt"]; world._wr_ref_qadr = b["ref_qadr"]
        world._wr_refnb = b["refnb"]; world._wr_ref_useatt = b["ref_useatt"]
        world._wr_refatt = b["refatt"]; world._wr_glut = b["glut"]; world._wr_gnb = b["gnb"]
        world._wr_omega = b["omega"]; world._wr_cmd = b["cmd"]
        world._wr_armlut = b["armlut"]; world._wr_elblut = b["elblut"]
        world._wr_seqcmd = b["seqcmd"]; world._wr_seqnb = b["seqnb"]
        world._wr_seq_yaw = b["seq_yaw"]; world._wr_seq_hold = b["seq_hold"]
        world._wr_phase = 0.0
        world._wr_walk_entered = False   # force a FRESH walk/carry re-entry (settle-and-go) from the
        world._wr_catching = False       # turn's settled pose -- else the next bout inherits stale state
        world._wr_last = np.zeros(len(getattr(world, "_wr_qadr", [0] * ACT_DIM)), np.float32)
        # HEADING RETENTION (owner 2026-07-08, "walk in the NEW direction after the turn"): pin the
        # walk's heading target to the ACHIEVED standing yaw at turn-exit. The walker only HOLDS a
        # heading when the heading obs (yaw - htgt) sits in its trained band; at the settled post-turn
        # pose pelvis_yaw == travel yaw, so htgt = that yaw makes the next straight walk see the SAME
        # in-distribution gait offset it trained with -- and walk straight in the NEW direction,
        # instead of steering back to the old one (the drift measured in the box demo).
        if _i("BATON_TURN_KEEP_HEADING", 1):
            try:
                _qs = world.solver.mjw_data.qpos.numpy().reshape(-1)
                _w2, _x2, _y2, _z2 = float(_qs[3]), float(_qs[4]), float(_qs[5]), float(_qs[6])
                world._wr_heading_tgt = math.atan2(2 * (_w2 * _z2 + _x2 * _y2), 1 - 2 * (_y2 * _y2 + _z2 * _z2))
                R._log(world, "SOLO-TURN keep-heading: post-turn walk target pinned to %.0f deg"
                       % (world._wr_heading_tgt * 57.2958))
            except Exception as _ekh:
                R._log(world, "SOLO-TURN keep-heading failed: %r" % (_ekh,))
        world._wr_in_turn = False
        R._log(world, "BATON SOLO-TURN: swapped OUT at t=%d (walk/blend context restored)" % world._wr_tt)
    world._wr_turn_active = bool(is_turn)


def _wr_patch_foot_torsion(world):
    """FOOT SPIN FRICTION (2026-07-07, the turning wall): MuJoCo foot contacts default to condim=3
    -- slide friction only, ZERO torsional (spin) resistance. A planted flat foot then freewheels
    about its vertical axis, so hip-yaw rotation does NOT transfer to base rotation and a step-turn
    loses ~2/3 of its yaw (diagnosed: legs track the ghost, base rotates 30 of 90 deg). Real soles
    resist twist. OMNISIM_FOOT_TORSION=<coef> raises the foot geoms to condim=4 with a real
    torsional friction coefficient (default off -> exact prior physics). Mirrors the engine's
    OMNISIM_NEWTON_ROLL_MU patch (WbNewtonBackend.cpp) but for TORSION, and only on the feet."""
    _ft = _f("OMNISIM_FOOT_TORSION", 0.0)
    if _ft <= 0:
        return
    try:
        import numpy as _np
        _footb = set(int(b) for b in getattr(world, "_gwm_foot", {}).values())
        _slv = world.solver
        # foot geom indices from the CPU model (its geom_bodyid is clean); the geom order matches
        # the warp model, so patch the SAME indices there (the warp geom_bodyid isn't per-geom here).
        _fg = []
        _mjm = getattr(_slv, "mj_model", None)
        if _mjm is not None:
            _bid = np.asarray(_mjm.geom_bodyid)
            for g in range(int(_mjm.ngeom)):
                if int(_bid[g]) in _footb:
                    _fg.append(g); _mjm.geom_condim[g] = 4; _mjm.geom_friction[g, 1] = _ft
        _mjw = getattr(_slv, "mjw_model", None)   # the STEPPED model -- this is the one that matters
        if _mjw is not None:
            # the warp geom arrays are shaped differently between the deploy solver and the batched
            # trainer (per-geom vs a shared/broadcast axis), so per-geom feet-only indexing is not
            # portable. Set condim=4 + torsion on ALL geoms via whole-array ops (like the engine's
            # ROLL_MU) -- torsion only bites contacts that actually SPIN, i.e. the feet on the floor.
            try:
                _fr = _mjw.geom_friction.numpy(); _cd = _mjw.geom_condim.numpy()
                _fr[..., 1] = np.maximum(_fr[..., 1], _ft); _cd[...] = 4
                _mjw.geom_friction.assign(_fr); _mjw.geom_condim.assign(_cd)
                R._log(world, "FOOT-TORSION mjw(stepped): condim=4 torsion=%.2f on all geoms (bites the feet)" % _ft)
            except Exception as _ew:
                R._log(world, "FOOT-TORSION mjw patch FAILED: %r" % (_ew,))
        R._log(world, "FOOT-TORSION: %d foot geoms -> condim=4 torsion=%.2f (a planted foot now resists spin)" % (len(_fg), _ft))
    except Exception as _e:
        R._log(world, "FOOT-TORSION patch failed: %r" % (_e,))


def _wr_teleport_crouch(world, stand_z):
    """Teleport the live newton state to the seeded crouch (legs bent, base upright at stand_z)
    so the policy starts from its trained pose. Newton joint_q + eval_fk + push into the solver's
    mjw_data (== _try_reset mode='update' in g1_residual_inengine)."""
    import sys
    nt = sys.modules.get("newton")
    m = world.model
    jq = m.joint_q.numpy().reshape(-1).copy()
    crouch = _seed_legs()
    leg_nd = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
    # DEPLOY_IC_NOISE (2026-07-06 THE STRIDE-GAP FIX): the EXACT symmetric crouch start drops the
    # closed loop into a symmetric SHUFFLE attractor (~0.07 m/s) -- measured: trainer world-0 with
    # EVAL_IC=0 shuffles identically to live, and the live action-replay tracks it within 5 cm.
    # The trainer's fast 0.36 m/s walks all start from EVAL_IC=0.02-perturbed crouches: the noise
    # breaks the symmetry into the true walking limit cycle. Match it live.
    _icn = _f("DEPLOY_IC_NOISE", 0.02)
    _rng9 = np.random.default_rng(_i("DEPLOY_IC_SEED", 7))
    for i, nd in enumerate(leg_nd):
        qi = nd + 1                       # single free base: joint_q coord index = dof index + 1
        if 0 <= qi < len(jq):
            jq[qi] = float(crouch[i]) + float(_rng9.uniform(-_icn, _icn))
    if len(jq) > 2:
        jq[2] = float(stand_z)            # base height (jq[0:3] is position regardless of quat layout)
    zqd = m.joint_qd.numpy() * 0.0
    m.joint_q.assign(jq); m.joint_qd.assign(zqd)
    for st in (getattr(world, "state_a", None), getattr(world, "state_b", None)):
        if st is None:
            continue
        try:
            st.joint_q.assign(jq); st.joint_qd.assign(zqd)
        except Exception:
            pass
        if nt is not None:
            try:
                nt.eval_fk(m, m.joint_q, m.joint_qd, st)
            except Exception as e:
                R._log(world, "wr teleport eval_fk err %r" % (e,))
    try:
        world.solver._update_mjc_data()   # push newton state -> solver mjw_data (no rebuild)
    except Exception as e:
        R._log(world, "wr teleport _update_mjc_data err %r" % (e,))
    world._mjc_dirty = True
    try:
        lq = world.solver.mjw_data.qpos.numpy().reshape(-1)
        lk = _build_legmap(world)[0]
        R._log(world, "wr teleport crouch applied base_z=%.3f legL=%s" %
               (float(lq[2]), [round(float(lq[q]), 3) for q in lk[:6]]))
    except Exception:
        pass


def g1_latency_probe(world):
    """One-shot OPEN-LOOP parity probe: drive the LIVE world (C++ solver.step + joint_target_pos)
    AND a batched K=1 buffer (_mjw.step + rd.ctrl) with IDENTICAL leg targets from the SAME seeded
    crouch, logging both left-knee trajectories tick-aligned. Divergence localizes the
    train(batched) vs deploy(live) actuation gap. Env: LP_T0, LP_STEP, LP_LOGN."""
    st = getattr(world, "_lp", None)
    if st is None:
        if not WM._build(world):
            return
        leg_qadr, leg_dadr = _build_legmap(world)
        leg_nd = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
        leg_act = [int(world._gwm_act[p]) for p, s in enumerate(world._gwm_slots) if s < 12]
        # LP_FREEFALL=1: spawn high (feet clear of the floor) -> both sides free-fall. Gravity is a
        # CLOCK: z(t)=z0-g/2*t_phys^2, so the per-tick fall directly measures how much physics time
        # each side advances per control step. A 2x parabola = a 2x dt mismatch (the suspected gap).
        _ffz = 2.0 if _i("LP_FREEFALL", 0) else _f("STAND_Z", 0.73)
        _wr_teleport_crouch(world, _ffz)
        crouch = np.array([-0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
                           -0.30, 0.0, 0.0, 0.52, -0.23, 0.0], np.float32)
        buf = world._mpc_rollout_buffers(1)
        _mjw, rm, rd, (nq, nv, nu) = buf
        try:                                    # match the live substep dt (LP_TS overrides)
            _sd = _f("LP_TS", float(world._gwm_dt) / max(1, int(getattr(world, "_n_substeps", 4))))
            _cur = rm.opt.timestep
            if hasattr(_cur, "fill_"):
                _cur.fill_(_sd)
            elif hasattr(_cur, "shape") and getattr(_cur, "shape", ()) != ():
                _cur[...] = _sd
            else:
                rm.opt.timestep = _sd
            R._log(world, "LATPROBE batched ts=%.5f x sub=%d = %.5f s/tick" % (_sd, _i("LP_SUB", 4), _sd * _i("LP_SUB", 4)))
        except Exception:
            pass
        # OPTIONAL: make the BATCHED actuators replicate the LIVE single-PD (kp, kd) -> if this makes
        # the batched trajectory match live, the training/deploy gap IS the actuator damping structure.
        if _i("LP_MATCH_LIVE", 0):
            try:
                mjm = world.solver.mj_model
                nu_ = int(mjm.nu)
                bp = np.asarray(mjm.actuator_biasprm).reshape(nu_, -1).copy()
                gp = np.asarray(mjm.actuator_gainprm).reshape(nu_, -1).copy()
                kdl = float(world.model.joint_target_kd.numpy().reshape(-1)[int(leg_nd[3])])
                for a in range(nu_):
                    if abs(float(bp[a][1])) > 1e-9:      # position actuator -> add velocity damping (biasprm2=-kd)
                        bp[a][2] = -kdl
                    else:                                 # velocity actuator -> disable (batched used it for damping)
                        gp[a][0] = 0.0
                # write back to the BATCHED model rm (put_model copies; edit rm's arrays)
                if hasattr(rm.actuator_biasprm, "assign"):
                    rm.actuator_biasprm.assign(bp.reshape(rm.actuator_biasprm.numpy().shape))
                    rm.actuator_gainprm.assign(gp.reshape(rm.actuator_gainprm.numpy().shape))
                R._log(world, "LATPROBE MATCH_LIVE: batched pos-actuators kd->%.1f, vel-actuators disabled" % kdl)
            except Exception as _e:
                R._log(world, "LATPROBE MATCH_LIVE err %r" % (_e,))
        try:
            mjm = world.solver.mj_model
            gpv = np.asarray(mjm.actuator_gainprm).reshape(int(mjm.nu), -1)
            R._log(world, "LATPROBE vel-actuator[knee+1] gainprm0(kv)=%.1f" % float(gpv[int(leg_act[3]) + 1][0]))
        except Exception:
            pass
        # SYSTEM-ID: LP_GAIN_SCALE scales the BATCHED actuator gains (kp of the pos-actuators,
        # kv of the vel-actuators) by a factor. Sweep it until the batched sag matches the LIVE
        # sag under identical held targets -> that factor IS the live plant's effective softness
        # (the number DR must be centered on for sim-to-real transfer to the live engine).
        _gsc = _f("LP_GAIN_SCALE", 1.0)
        if abs(_gsc - 1.0) > 1e-9:
            try:
                mjm = world.solver.mj_model
                nu_ = int(mjm.nu)
                bp = np.asarray(mjm.actuator_biasprm).reshape(nu_, -1).copy()
                gp = np.asarray(mjm.actuator_gainprm).reshape(nu_, -1).copy()
                gp[:, 0] *= _gsc            # actuator gain (kp for pos, kv for vel)
                bp[:, 1] *= _gsc            # pos-actuator bias kp term (biasprm1=-kp)
                rm.actuator_gainprm.assign(gp.reshape(rm.actuator_gainprm.numpy().shape))
                rm.actuator_biasprm.assign(bp.reshape(rm.actuator_biasprm.numpy().shape))
                R._log(world, "LATPROBE GAIN_SCALE=%.2f applied to batched actuators" % _gsc)
            except Exception as _e:
                R._log(world, "LATPROBE GAIN_SCALE err %r" % (_e,))
        # seeding happens LAZILY at the first per-tick call (t==0) so the live mjw state already
        # reflects the teleport (at setup it is still pre-teleport -> the old confound).
        st = world._lp = dict(t=0, seeded=False, leg_qadr=leg_qadr, leg_nd=leg_nd, leg_act=leg_act, crouch=crouch,
                              buf=buf, sub=_i("LP_SUB", int(getattr(world, "_n_substeps", 4))), knee=3,
                              T0=_i("LP_T0", 60), step=_f("LP_STEP", 0.3), logn=_i("LP_LOGN", 45), done=False)
        R._log(world, "LATPROBE start knee_qadr=%d knee_nd=%d knee_act=%d sub=%d nu=%d"
               % (int(leg_qadr[3]), int(leg_nd[3]), int(leg_act[3]), st["sub"], nu))
        # CONFIG DIFF: the live SolverMuJoCo's own mjwarp model/data vs our batched put_model/put_data.
        # A mismatch here (contact buffer sizes, solver iterations, integrator...) is the cheapest
        # possible explanation for the live sag (e.g. dropped contacts -> the robot sinks).
        try:
            sol = world.solver
            lm = None; ld = getattr(sol, "mjw_data", None)
            for _nm in ("mjw_model", "m", "wm", "model_mjw"):
                if getattr(sol, _nm, None) is not None:
                    lm = getattr(sol, _nm); break
            if lm is None:
                cand = [n for n in dir(sol) if "model" in n.lower()]
                R._log(world, "LATPROBE cfg: no mjw_model attr; candidates=%s" % cand[:12])

            def _sc(x):
                try:
                    return float(np.asarray(x if not hasattr(x, "numpy") else x.numpy()).reshape(-1)[0])
                except Exception:
                    return float("nan")

            def _optline(tag, om):
                if om is None:
                    return "%s: <none>" % tag
                o = getattr(om, "opt", None)
                if o is None:
                    return "%s: <no opt>" % tag
                fields = ("timestep", "iterations", "ls_iterations", "solver", "integrator",
                          "cone", "impratio", "tolerance")
                return "%s: " % tag + " ".join("%s=%s" % (f, ("%g" % _sc(getattr(o, f))) if hasattr(o, f) else "-")
                                               for f in fields)
            R._log(world, "LATPROBE cfg " + _optline("LIVE", lm))
            R._log(world, "LATPROBE cfg " + _optline("BATCH", rm))
            for tag, dd in (("LIVE", ld), ("BATCH", rd)):
                if dd is None:
                    continue
                nj = getattr(dd, "njmax", None); nc = getattr(dd, "nconmax", None)
                ncon = getattr(dd, "ncon", None)
                try:
                    ncon_v = int(np.asarray(ncon.numpy() if hasattr(ncon, "numpy") else ncon).reshape(-1)[0])
                except Exception:
                    ncon_v = -1
                R._log(world, "LATPROBE cfg %s data: njmax=%s nconmax=%s ncon_now=%d" % (tag, nj, nc, ncon_v))
            # per-geom CONTACT params: friction + solref + solimp for floor(0) + a foot geom, both models
            def _garr(om, name):
                a = getattr(om, name, None)
                if a is None:
                    return None
                try:
                    return np.asarray(a.numpy() if hasattr(a, "numpy") else a)
                except Exception:
                    return None
            mjm2 = world.solver.mj_model
            gfoot = -1
            try:   # find a foot geom: smallest-z body geoms -> use body of foot map
                import mujoco as _mj
                fb = list(world._gwm_foot.values())[0]
                for g in range(int(mjm2.ngeom)):
                    if int(mjm2.geom_bodyid[g]) == int(fb):
                        gfoot = g; break
            except Exception:
                pass
            for tag, om in (("LIVE", lm), ("BATCH", rm)):
                if om is None:
                    continue
                fr = _garr(om, "geom_friction"); sr = _garr(om, "geom_solref"); si = _garr(om, "geom_solimp")
                for gname, g in (("floor0", 0), ("foot", gfoot)):
                    if g < 0:
                        continue
                    line = "LATPROBE contact %s %s(g%d):" % (tag, gname, g)
                    if fr is not None:
                        line += " fric=%s" % [round(float(v), 3) for v in np.asarray(fr).reshape(-1, fr.shape[-1])[g][:3]]
                    if sr is not None:
                        line += " solref=%s" % [round(float(v), 4) for v in np.asarray(sr).reshape(-1, sr.shape[-1])[g][:2]]
                    if si is not None:
                        line += " solimp=%s" % [round(float(v), 3) for v in np.asarray(si).reshape(-1, si.shape[-1])[g][:3]]
                    R._log(world, line)
        except Exception as _e:
            R._log(world, "LATPROBE cfg err %r" % (_e,))
        try:   # compare BATCHED mjw-actuator gains vs LIVE joint_target PD gains at the knee
            mjm = world.solver.mj_model
            gp = np.asarray(mjm.actuator_gainprm).reshape(int(mjm.nu), -1)
            bp = np.asarray(mjm.actuator_biasprm).reshape(int(mjm.nu), -1)
            ka = int(leg_act[3])
            ke = world.model.joint_target_ke.numpy().reshape(-1)
            kd = world.model.joint_target_kd.numpy().reshape(-1)
            knd = int(leg_nd[3])
            arm = np.asarray(getattr(mjm, "dof_armature", [])).reshape(-1)
            dmp = np.asarray(getattr(mjm, "dof_damping", [])).reshape(-1)
            kdof = int(leg_dadr[3])
            R._log(world, "LATPROBE gains knee: mjw gainprm0=%.1f biasprm1=%.1f biasprm2=%.2f | live joint_target_ke=%.1f kd=%.2f | armature=%.4f damping=%.4f"
                   % (float(gp[ka][0]), float(bp[ka][1]), float(bp[ka][2]),
                      float(ke[knd]) if knd < len(ke) else -1, float(kd[knd]) if knd < len(kd) else -1,
                      float(arm[kdof]) if kdof < len(arm) else -1, float(dmp[kdof]) if kdof < len(dmp) else -1))
        except Exception as _e:
            R._log(world, "LATPROBE gains err %r" % (_e,))
    if st["done"]:
        return
    _mjw, rm, rd, (nq, nv, nu) = st["buf"]
    if not st["seeded"] and st["t"] >= 1:
        # seed at t=1: the live mjw only reflects the teleport after the FIRST C++ tick's
        # dirty copy-in (which runs after hook t=0 returns). Copying qpos+qvel+ctrl here
        # tick-aligns both sides exactly.
        world._mpc_seed_from_live(1)
        st["seeded"] = True
    kq = int(st["leg_qadr"][st["knee"]])
    lqp = world.solver.mjw_data.qpos.numpy().reshape(-1)
    bqp = rd.qpos.numpy().reshape(1, nq)[0]
    t = st["t"]
    tgt_knee = float(st["crouch"][st["knee"]]) + (st["step"] if t >= st["T0"] else 0.0)
    if t <= st["T0"] + st["logn"]:
        ka = int(st["leg_act"][st["knee"]])
        try:   # what actually reaches mjw ctrl on the LIVE side (pos + vel actuator of the knee)
            lct = world.solver.mjw_data.ctrl.numpy().reshape(-1)
            lcp, lcv = float(lct[ka]), float(lct[ka + 1]) if ka + 1 < len(lct) else 0.0
        except Exception:
            lcp, lcv = float("nan"), float("nan")
        R._log(world, "LATPROBE t=%d tgt=%.3f live_knee=%.4f bat_knee=%.4f live_z=%.3f bat_z=%.3f live_ctrl=(%.3f,%.3f)"
               % (t, tgt_knee, float(lqp[kq]), float(bqp[kq]), float(lqp[2]), float(bqp[2]), lcp, lcv))
    # LP_WB_ALL=1: WHOLE-BODY mapping audit — at T0 offset ALL non-leg joints by +0.15 via each
    # side's own routing (live: joint_target_pos[ndof], batched: ctrl[act]); afterwards dump every
    # whole-body joint's qpos on both sides. Any joint where live != batched = a routing mismatch
    # (the arm/waist ndof map was never behaviourally verified — legs were).
    if _i("LP_WB_ALL", 0) and t == st["T0"] + st["logn"]:
        try:
            qadr_a, dadr_a, aidx_a = _build_all_pos_act(world)
            leg_set = set(int(q) for q in st["leg_qadr"])
            lines = []
            for i in range(len(qadr_a)):
                q = int(qadr_a[i])
                tagj = "LEG" if q in leg_set else "WB"
                lines.append("%s[%d] q=%d live=%.4f bat=%.4f d=%.4f" %
                             (tagj, i, q, float(lqp[q]), float(bqp[q]), float(lqp[q] - bqp[q])))
            R._log(world, "LP_WB_ALL audit:\n" + "\n".join(lines))
        except Exception as _e:
            R._log(world, "LP_WB_ALL err %r" % (_e,))
    if t > st["T0"] + st["logn"]:
        st["done"] = True; R._log(world, "LATPROBE done")
        return
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for i in range(12):
        nd = int(st["leg_nd"][i])
        if 0 <= nd < len(tp):
            tp[nd] = float(st["crouch"][i]) + (st["step"] if (i == st["knee"] and t >= st["T0"]) else 0.0)
    if _i("LP_WB_ALL", 0) and t >= st["T0"]:
        # offset every NON-leg joint (+0.15) through the LIVE routing (whole-body ndof map)
        qadr_a, dadr_a, aidx_a = _build_all_pos_act(world)
        m2nd_a = world.solver.mjc_jnt_to_newton_dof.numpy()
        if m2nd_a.ndim == 2:
            m2nd_a = m2nd_a[0]
        trn_a = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
        leg_set = set(int(q) for q in st["leg_qadr"])
        spn = world.solver.mjw_data.qpos.numpy().reshape(-1)
        for i in range(len(qadr_a)):
            if int(qadr_a[i]) in leg_set:
                continue
            nd = int(m2nd_a[int(trn_a[int(aidx_a[i])][0])])
            if 0 <= nd < len(tp):
                tp[nd] = 0.15    # absolute small offset target for all non-leg joints
    world.control.joint_target_pos.assign(tp)
    # LP_NO_DIRTY=1: do NOT force the per-tick newton->mjw state re-import. Control writes don't
    # need it (apply_mjc_control reads the control arrays each substep); only STATE mutations do.
    # If the live sag disappears with this off, the sag = the newton<->mjw state ROUND-TRIP decay
    # from dirtying every tick -- and the deploy-hook fix is simply to stop setting _mjc_dirty.
    if not _i("LP_NO_DIRTY", 0):
        world._mjc_dirty = True
    if st["seeded"]:                      # batched only steps once tick-aligned (seed at t=1)
        ctrl = rd.ctrl.numpy().reshape(1, nu)
        for i in range(12):
            a = int(st["leg_act"][i])
            if 0 <= a < nu:
                ctrl[0, a] = float(st["crouch"][i]) + (st["step"] if (i == st["knee"] and t >= st["T0"]) else 0.0)
                if a + 1 < nu:
                    ctrl[0, a + 1] = 0.0
        if _i("LP_WB_ALL", 0) and t >= st["T0"]:
            # same non-leg offsets through the BATCHED routing (actuator indices)
            qadr_a, dadr_a, aidx_a = _build_all_pos_act(world)
            leg_set = set(int(q) for q in st["leg_qadr"])
            for i in range(len(qadr_a)):
                if int(qadr_a[i]) in leg_set:
                    continue
                a = int(aidx_a[i])
                if 0 <= a < nu:
                    ctrl[0, a] = 0.15
                    if a + 1 < nu:
                        ctrl[0, a + 1] = 0.0
        rd.ctrl.assign(ctrl)
        for _ in range(st["sub"]):
            _mjw.step(rm, rd)
    st["t"] += 1


def g1_ghost_dump(world):
    """One-shot: export the EXACT ghost the trainer rewards against (leg LUT from the deterministic
    gait + the arm-swing reference + clock) to GHOST_JSON, for the visual ghost-replay controller."""
    if getattr(world, "_gd_done", False):
        return
    world._gd_done = True
    try:
        import json
        if not WM._build(world):
            R._log(world, "ghost-dump: WM build failed"); return
        slots = world._gwm_slots
        leg_pos = [p for p, s in enumerate(slots) if s < 12]
        NB = 32
        glut = np.zeros((NB, len(slots)))
        for bb in range(NB):
            legs, _a, _s = world._gwm_stg.targets_np(world._gwm_phase0 + 2 * math.pi * bb / NB,
                                                     world._gwm_gp, t_since_start=10.0)
            glut[bb] = np.asarray(legs)[[slots[p] for p in range(len(slots))]]
        out = dict(nb=NB, omega=float(world._gwm_omega), dt=float(world._gwm_dt),
                   slots=[int(slots[p]) for p in leg_pos],
                   leg_lut=[[float(v) for v in glut[bb, leg_pos]] for bb in range(NB)],
                   arm_A=_f("ARM_SWING_A", 0.3))
        path = os.environ.get("GHOST_JSON", "_scratch/foot_redesign/ghost_lut.json")
        with open(path, "w") as f:
            json.dump(out, f)
        R._log(world, "ghost-dump: wrote %s (omega=%.3f dt=%.4f)" % (path, out["omega"], out["dt"]))
    except Exception as e:
        import traceback
        R._log(world, "ghost-dump err %r\n%s" % (e, traceback.format_exc()))


def g1_closedloop_probe(world):
    """CLOSED-LOOP parity probe: run the SAME policy net on the LIVE world (deploy pipeline:
    obs from live mjw_data -> net -> joint_target_pos) and a batched K=1 buffer (eval pipeline:
    obs from rd -> net -> rd.ctrl) simultaneously, seeded from the same state at t=1. Logs both
    trajectories tick-aligned. Physics is bit-exact (proven open-loop), so an EARLY split
    (< ~20 ticks) = an obs/action PIPELINE bug; a slow late drift = chaos. Env: LP_POLICY."""
    torch = _torch()
    st = getattr(world, "_clp", None)
    if st is None:
        if getattr(world, "_wr_dep", None) is None:
            world._wr_dep = False
            if _deploy_setup(world):
                world._wr_dep = True
        if not world._wr_dep:
            return
        buf = world._mpc_rollout_buffers(1)
        _mjw, rm, rd, (nq, nv, nu) = buf
        try:
            _sd = 0.016 / 4.0
            _cur = rm.opt.timestep
            if hasattr(_cur, "fill_"):
                _cur.fill_(_sd)
            else:
                _cur[...] = _sd
        except Exception:
            pass
        st = world._clp = dict(t=0, seeded=False, buf=buf, la_live=np.zeros(ACT_DIM, np.float32),
                               la_bat=np.zeros(ACT_DIM, np.float32), ph=0.0, done=False)
        R._log(world, "CLP start act=%d obs=%d" % (ACT_DIM, OBS_DIM))
        _wr_teleport_crouch(world, _f("STAND_Z", 0.73))
        return

    def _hold_live_nominal():
        # never let the live world tick on default targets (straight legs from a crouch =
        # spring-load eject) — hold the nominal crouch during all pre-policy ticks
        if world.control is None:
            world.control = world.model.control()
        tp0 = world.control.joint_target_pos.numpy()
        for i in range(len(world._wr_ndof)):
            nd = int(world._wr_ndof[i])
            if 0 <= nd < len(tp0):
                tp0[nd] = float(world._wr_nom[i])
        world.control.joint_target_pos.assign(tp0)
    if st["done"]:
        return
    _mjw, rm, rd, (nq, nv, nu) = st["buf"]
    t = st["t"]; st["t"] += 1
    settle = _i("LP_SETTLE", 30)     # let live settle to PD equilibrium under held nominal first
    if not st["seeded"]:
        if t >= settle:
            world._mpc_seed_from_live(1)
            st["seeded"] = True
            R._log(world, "CLP seeded batched from live at t=%d (post-settle)" % t)
        _hold_live_nominal()
        return
    if t > _i("LP_CLN", 220):
        if not st["done"]:
            st["done"] = True; R._log(world, "CLP done")
        return

    def obs_from(qp, qv, ph, la):
        w, x, y, z = qp[3], qp[4], qp[5], qp[6]
        gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        jpos = qp[world._wr_qadr] - world._wr_nom; jvel = qv[world._wr_dadr]
        return np.concatenate([0.25 * qv[3:6], [gx, gy, gz], [_f("VX_MAX", 0.7), 0.0, 0.0],
                               jpos, 0.05 * jvel, la, [math.sin(ph), math.cos(ph)],
                               [math.sin(yaw), math.cos(yaw)]]).astype(np.float32)

    def act_of(o, la_key):
        with torch.no_grad():
            mean, _s, _h = world._wr_net.act(torch.from_numpy(o[None, :]), None)
        ac = np.clip(mean.numpy()[0], -1, 1)
        st[la_key] = ac.astype(np.float32)
        return ac * _f("RES_ACT_SCALE", 0.5)

    # LIVE side: deploy pipeline
    lqp = world.solver.mjw_data.qpos.numpy().reshape(-1); lqv = world.solver.mjw_data.qvel.numpy().reshape(-1)
    o_live = obs_from(lqp, lqv, st["ph"], st["la_live"])
    env_live = act_of(o_live, "la_live")
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for i in range(len(world._wr_ndof)):
        nd = int(world._wr_ndof[i])
        if 0 <= nd < len(tp):
            tp[nd] = float(world._wr_nom[i]) + float(env_live[i])
    world.control.joint_target_pos.assign(tp)
    # BATCHED side: eval pipeline
    bqp = rd.qpos.numpy().reshape(-1)[:nq]; bqv = rd.qvel.numpy().reshape(-1)[:nv]
    o_bat = obs_from(bqp, bqv, st["ph"], st["la_bat"])
    env_bat = act_of(o_bat, "la_bat")
    ctrl = rd.ctrl.numpy().reshape(1, nu)
    aidx = _build_all_pos_act(world)[2]
    for i, a in enumerate(aidx):
        ctrl[0, int(a)] = float(world._wr_nom[i]) + float(env_bat[i])
        if int(a) + 1 < nu:
            ctrl[0, int(a) + 1] = 0.0
    rd.ctrl.assign(ctrl)
    for _ in range(4):
        _mjw.step(rm, rd)
    st["ph"] += world._wr_omega * world._wr_dt
    odiff = float(np.abs(o_live - o_bat).max())
    adiff = float(np.abs(st["la_live"] - st["la_bat"]).max())
    if t <= 12 or t % 10 == 2:
        R._log(world, "CLP t=%d live(x=%.3f z=%.3f) bat(x=%.3f z=%.3f) odiff=%.5f adiff=%.5f"
               % (t, lqp[0], lqp[2], bqp[0], bqp[2], odiff, adiff))


def g1_walk_recipe_deploy(world):
    torch = _torch()
    if getattr(world, "_wr_dep", None) is None:
        world._wr_dep = False
        if _deploy_setup(world):
            world._wr_dep = True
    if not world._wr_dep:
        return
    if not hasattr(world, "_wr_dt0"):
        world._wr_dt0 = 0
    world._wr_dt0 += 1
    if world._wr_dt0 == _i("WALK_WARM_TICKS", 360) and _i("WALK_MJW_RESET", 1):
        # HANDOFF RESET on the FIRST POLICY TICK, bit-identical to a training/eval reset: write
        # the solver's INTERNAL mjw state directly (base at STAND_Z, quat identity, all joints at
        # nominal, zero vel). Must happen AFTER the last warm tick: the warm hold sets _mjc_dirty
        # each tick, and the resulting newton->mjw copy-in CLOBBERED this reset when it ran a tick
        # earlier (live always started at the settled z=0.767 -> off the training IC distribution
        # -> fine-tuned policies fell in ~2-4 s at eval surv 0.90). From here on NO code sets
        # _mjc_dirty (control writes don't need it), so mjw steps continuously from this exact
        # state -- the same regime as the batched trainer.
        try:
            lv = world.solver.mjw_data
            qp2 = lv.qpos.numpy().reshape(1, -1)
            qp2[0, 0:3] = [0.0, 0.0, _f("STAND_Z", 0.72)]
            qp2[0, 3:7] = [1.0, 0.0, 0.0, 0.0]
            qp2[0, world._wr_qadr] = world._wr_nom
            # HANDOFF JITTER: training resets ALWAYS add joint noise (IC_RAND/EVAL_IC), so the
            # policy has (almost) never seen the perfectly SYMMETRIC nominal crouch — which is
            # also the knife-edge start (symmetry must be broken before the first step can be
            # taken). Deploying from exact-nominal was measurably HARDER than the eval's noisy
            # starts (arm2: fall 0.23 @IC=0 vs 0.09 @IC=0.05). Seeded -> reproducible runs.
            jit = _f("WALK_HANDOFF_JITTER", 0.02)
            if jit > 0:
                rng = np.random.RandomState(_i("WALK_JITTER_SEED", 1))
                qp2[0, world._wr_qadr] += rng.randn(len(world._wr_qadr)).astype(qp2.dtype) * jit
            lv.qpos.assign(qp2.reshape(lv.qpos.numpy().shape))
            lv.qvel.assign(lv.qvel.numpy() * 0.0)
            R._log(world, "walk-recipe HANDOFF mjw reset: z=%.3f joints=nominal jit=%.3f vel=0"
                   % (_f("STAND_Z", 0.72), jit))
        except Exception as e:
            R._log(world, "HANDOFF reset err %r" % (e,))
    if world._wr_dt0 < _i("WALK_WARM_TICKS", 360):
        # hold the seeded crouch (zero-action nominal) so the robot settles stable
        # before the policy engages — the deterministic stand collapses on the real foot
        if world.control is None:
            world.control = world.model.control()
        tp = world.control.joint_target_pos.numpy()
        if world._wr_whole:
            for i in range(len(world._wr_ndof)):
                nd = int(world._wr_ndof[i])
                if 0 <= nd < len(tp):
                    tp[nd] = float(world._wr_nom[i])
        else:
            for i, p in enumerate(world._wr_legpos):
                nd = world._wr_gdof[p]
                if 0 <= nd < len(tp):
                    tp[nd] = float(world._wr_nom[i])
            for p in world._wr_waistpos:
                nd = world._wr_gdof[p]
                if 0 <= nd < len(tp):
                    tp[nd] = float(world._wr_waistnom[p])
        world.control.joint_target_pos.assign(tp); world._mjc_dirty = True
        return
    tt = world._wr_dt0 - _i("WALK_WARM_TICKS", 360)
    world._wr_tt = tt   # for _wr_turn_swap logging
    # ── TWO-POLICY SCHEDULER (owner architecture, 2026-07-04): WALK_SCHEDULE="walk:5,stand:5,
    # walk:5,stand:0" (seconds; 0 = forever). Combines the two PROVEN policies instead of one
    # monolith: WALK = the champion policy tick below; STAND = the deterministic nominal-crouch
    # hold (the same stand the warm phase and the shipped stand controller use). Handoffs:
    # stand->walk re-enters exactly like the proven settle-and-go (fresh gait phase, fresh
    # recurrent state, zero last-action); walk->stand ramps the policy's authority out over
    # WALK_RAMP_TICKS while the stand stiffness catches the residual momentum.
    _sched_env = os.environ.get("WALK_SCHEDULE")
    _course_env = os.environ.get("BATON_COURSE")
    _blend_u = 1.0
    if _course_env and world._wr_baton is not None:
        # ── BATON COURSE (goal-directed, owner 2026-07-06): segments are TASKS, not timers.
        #   "walkto,X,Y;stand,S;carryto,X,Y;..."  walkto/carryto steer the heading-conditioned
        # walker toward (X,Y) each tick and advance ON ARRIVAL (<0.4 m); stand,S advances after
        # S seconds (heading target HELD, so the robot keeps facing its work). Last segment holds.
        if not hasattr(world, "_wr_course"):
            _segs9 = []
            for _sg9 in _course_env.split(";"):
                _ff9 = _sg9.split(",")
                if _ff9[0] in ("walkto", "carryto"):
                    _segs9.append((_ff9[0], float(_ff9[1]), float(_ff9[2])))
                else:
                    _segs9.append((_ff9[0], float(_ff9[1]), 0.0))
            world._wr_course = _segs9; world._wr_course_i = 0; world._wr_course_t0 = tt
            R._log(world, "BATON COURSE: %s" % (_segs9,))
        _qpc = world.solver.mjw_data.qpos.numpy().reshape(-1)
        _cpx, _cpy = float(_qpc[0]), float(_qpc[1])
        _ci = world._wr_course_i
        _seg = world._wr_course[_ci]
        _bearing9 = None   # the course bearing the carrot window ratchets toward (turn-scoped)
        if _seg[0] in ("walkto", "carryto"):
            _dx9, _dy9 = _seg[1] - _cpx, _seg[2] - _cpy
            _btgt = math.atan2(_dy9, _dx9)
            _bearing9 = _btgt
            world._wr_last_walk_bearing = _btgt   # remembered so the next STAND can pivot by the exterior angle
            world._wr_pivot_active = False
            world._wr_turn_active = False
            # SLEW the heading target (missed-waypoint bearing flips thrashed the crane torque
            # into NaN); the steering stays gentle and continuous.
            _prev9 = getattr(world, "_wr_heading_tgt", None)
            if _prev9 is None:
                world._wr_heading_tgt = _btgt
            else:
                _d9 = math.atan2(math.sin(_btgt - _prev9), math.cos(_btgt - _prev9))
                _sl9 = _f("BATON_SLEW", 0.02)
                world._wr_heading_tgt = _prev9 + max(-_sl9, min(_sl9, _d9))
            _mode = "walk" if _seg[0] == "walkto" else "carry"
            if math.hypot(_dx9, _dy9) < _f("BATON_ARRIVE_R", 0.55) and _ci + 1 < len(world._wr_course):
                world._wr_course_i += 1; world._wr_course_t0 = tt
                R._log(world, "BATON COURSE arrived at (%.2f, %.2f) -> segment %d %s"
                       % (_seg[1], _seg[2], world._wr_course_i, world._wr_course[world._wr_course_i]))
        else:
            _mode = _seg[0]
            # TURN segment (2026-07-07): switch to the trained turn specialist for a timed live
            # turn (~38deg per trigger; BATON_COLD_HIDDEN gives a fresh turn each corner). The
            # timed advance is the shared dwell logic below (pivot inactive -> time-based).
            _wr_turn_swap(world, _seg[0] == "turn")   # SOLO-TURN swap (no-op unless BATON_TURN_CKPT armed)
            # STAND-PIVOT (square5 lesson): the walking crane's gentle budget (~30 N*m) cannot
            # corner a 3.5 m square -- it traces a ~5 m arc and spirals out. So don't corner
            # WHILE walking: stop at the corner and pivot IN PLACE during the stand, where the
            # stationary balance policy accepts far more yaw torque (no forward momentum to
            # tumble). Only a RELATIVE rotation is needed -- rotate the pelvis by the path's
            # EXTERIOR ANGLE -- so the messy, gait-dependent pelvis<->travel offset (which is
            # ~0 standing but ~-1.44 mid-gait) never has to be measured. Computed once on
            # stand entry, then held as the crane target for the whole pivot.
            _yawnow9 = math.atan2(2 * (float(_qpc[3]) * float(_qpc[6]) + float(_qpc[4]) * float(_qpc[5])),
                                  1 - 2 * (float(_qpc[5]) ** 2 + float(_qpc[6]) ** 2))
            if _seg[0] == "stand" and not _i("BATON_STAND_PIVOT", 1):
                # BATON_STAND_PIVOT=0 (2026-07-08): DISABLE the crane stand-pivot entirely. When the
                # corner is a real FOOTWORK turn segment (SOLO-TURN), the stand must just HOLD -- the
                # old crane pivot would fight the turn and spin the robot (NaN). Default 1 = unchanged.
                world._wr_pivot_active = False; world._wr_pivot_seg = _ci
            elif _seg[0] == "stand" and getattr(world, "_wr_pivot_seg", -1) != _ci:
                # compute the pivot GOAL once on entry (relative exterior-angle rotation)
                _out9 = None
                for _nx9 in world._wr_course[_ci + 1:]:
                    if _nx9[0] in ("walkto", "carryto"):
                        _out9 = math.atan2(_nx9[2] - _cpy, _nx9[1] - _cpx); break
                _in9 = getattr(world, "_wr_last_walk_bearing", None)
                if _out9 is not None and _in9 is not None:
                    _ext9 = math.atan2(math.sin(_out9 - _in9), math.cos(_out9 - _in9))
                    world._wr_pivot_final = math.atan2(math.sin(_yawnow9 + _ext9), math.cos(_yawnow9 + _ext9))
                    world._wr_heading_tgt = _yawnow9        # setpoint starts AT the robot, then slews
                    world._wr_pivot_active = True
                    R._log(world, "STAND-PIVOT seg %d: psi0=%.2f ext=%.2f -> final=%.2f"
                           % (_ci, _yawnow9, _ext9, world._wr_pivot_final))
                else:
                    world._wr_pivot_active = False
                world._wr_pivot_seg = _ci
            # SLEW the setpoint toward the pivot goal each tick: the crane PD always tracks a
            # SMALL error, so it never builds the momentum that made square6 overshoot/ring or
            # needed the square7 over-damping that stalled it. The robot arrives when the
            # setpoint does, with ~zero rate.
            if _seg[0] == "stand" and getattr(world, "_wr_pivot_active", False):
                _d9 = math.atan2(math.sin(world._wr_pivot_final - world._wr_heading_tgt),
                                 math.cos(world._wr_pivot_final - world._wr_heading_tgt))
                _ps9 = _f("BATON_PIVOT_SLEW", 0.006)
                world._wr_heading_tgt = math.atan2(math.sin(world._wr_heading_tgt + max(-_ps9, min(_ps9, _d9))),
                                                   math.cos(world._wr_heading_tgt + max(-_ps9, min(_ps9, _d9))))
            # advance stand->next only when the dwell elapsed AND (no pivot, or the pivot has
            # SETTLED at its goal) -- never hand the walk a robot mid-turn (square6/7 tumble).
            _pv_done = (not getattr(world, "_wr_pivot_active", False)) or (
                abs(math.atan2(math.sin(_yawnow9 - getattr(world, "_wr_pivot_final", _yawnow9)),
                               math.cos(_yawnow9 - getattr(world, "_wr_pivot_final", _yawnow9)))) < 0.22)
            _dwell_ok = (tt - world._wr_course_t0) * 0.016 >= _seg[1]
            _dwell_max = (tt - world._wr_course_t0) * 0.016 >= _seg[1] + 8.0   # hard timeout: never deadlock
            if ((_dwell_ok and _pv_done) or _dwell_max) and _ci + 1 < len(world._wr_course):
                world._wr_course_i += 1; world._wr_course_t0 = tt
                world._wr_pivot_active = False
                R._log(world, "BATON COURSE -> segment %d %s" % (world._wr_course_i, world._wr_course[world._wr_course_i]))
        # TURN-SCOPED CARROT WINDOW (square3 lesson): capping the target's LEAD over actual
        # yaw is fundamentally incompatible with a strong straight-leg anchor -- a wandering
        # gait yaw drags the lead-capped target with it (square3: leg-1 yaw walked to -1.26,
        # target followed to -0.96; nav4's drift cure IS the fixed-bearing anchor with FULL
        # restoring force). Resolution: the window is TURN-SCOPED. Right after a walk
        # re-entry the lead cap is tight (BATON_TGT_WIN) so the fresh gait isn't handed the
        # whole 90-degree error at once; it then RAMPS open to a full bearing anchor over
        # BATON_TGT_WIN_RAMP_TICKS, restoring the strong hold that keeps the straight leg on
        # course. Time-since-entry -- not current alignment -- is what separates "mid-turn,
        # protect" from "wandered off the straight, correct hard" (same error, opposite fix).
        _pm9 = getattr(world, "_wr_prev_cmode", None)
        if _mode == "walk" and _pm9 != "walk":
            world._wr_walk_t0 = tt
        world._wr_prev_cmode = _mode
        _win9 = _f("BATON_TGT_WIN", 0.0)
        if _win9 > 0 and _bearing9 is not None and getattr(world, "_wr_heading_tgt", None) is not None:
            _wt0 = getattr(world, "_wr_walk_t0", tt)
            _rmp = _i("BATON_TGT_WIN_RAMP_TICKS", 500)
            _fr9 = min(1.0, max(0.0, (tt - _wt0) / float(_rmp))) if _rmp > 0 else 1.0
            _win_eff = _win9 + _fr9 * math.pi   # ramps to >= pi == full fixed-bearing anchor
            _yawc = math.atan2(2 * (float(_qpc[3]) * float(_qpc[6]) + float(_qpc[4]) * float(_qpc[5])),
                               1 - 2 * (float(_qpc[5]) ** 2 + float(_qpc[6]) ** 2)) - _f("HARNESS_YAW_OFFSET", 0.0)
            _eyb = math.atan2(math.sin(_bearing9 - _yawc), math.cos(_bearing9 - _yawc))
            world._wr_heading_tgt = _yawc + max(-_win_eff, min(_win_eff, _eyb))
        _sched_env = None   # the course replaces the time schedule; fall through to the BATON arbiter
        B = world._wr_baton
        _des = _mode if _mode in B["reg"] else "walk"
        if _des != B["target"]:
            _gate_ok = True
            if B["target"] == "walk" and _i("BATON_DS_GATE", 1) and B["u"] >= 1.0:
                _DSb = 0.65 * 2.0 * math.pi
                _phb5 = (world._wr_phase - _DSb) % math.pi
                _gate_ok = min(_phb5, math.pi - _phb5) <= 0.15
            if _gate_ok:
                T = B["reg"]
                def _blc(k, _u):
                    a = B["src"].get(k); b = T[B["target"]]["tables"].get(k)
                    if a is None or b is None:
                        return a if b is None else b
                    return (1.0 - _u) * a + _u * b
                B["src"] = {k: (_blc(k, B["u"]) if k != "vx" else
                                (1.0 - B["u"]) * B["src"]["vx"] + B["u"] * T[B["target"]]["tables"]["vx"])
                            for k in ("glut", "arm", "elb", "att", "ref", "vx")}
                B["out"] = B["target"] if B["u"] >= 1.0 else B.get("out", B["target"])
                B["target"] = _des; B["u"] = 0.0
                B["switches"] += 1
                R._log(world, "BATON switch #%d (-> %s) at t=%d phase=%.2f" %
                       (B["switches"], _des, tt, world._wr_phase % (2 * math.pi)))
                _mf = os.environ.get("BATON_MODE_FILE")
                if _mf:
                    try:
                        open(_mf, "w").write(_des)
                    except OSError:
                        pass
        B["u"] = min(1.0, B["u"] + 1.0 / float(B["morph"]))
        if B["u"] >= 1.0:
            B["active"] = B["target"]
        _Tt = B["reg"][B["target"]]["tables"]; _ub = B["u"]
        def _effc(k):
            a = B["src"].get(k); b = _Tt.get(k)
            if a is None or b is None:
                return a if b is None else b
            return (1.0 - _ub) * a + _ub * b
        if not getattr(world, "_wr_in_turn", False):
            # don't clobber the SOLO-TURN's swapped context: writing the nb=64 walk ref here while
            # _wr_refnb stays 225 (turn) makes the obs ref-block index out of range -> garbage -> NaN.
            world._wr_glut = _effc("glut")
            _a6 = _effc("arm");  world._wr_armlut = _a6 if _a6 is not None else getattr(world, "_wr_armlut", None)
            _e6 = _effc("elb");  world._wr_elblut = _e6 if _e6 is not None else getattr(world, "_wr_elblut", None)
            _t6 = _effc("att");  world._wr_refatt = _t6 if _t6 is not None else getattr(world, "_wr_refatt", None)
            _r6 = _effc("ref");  world._wr_reftgt = _r6 if _r6 is not None else getattr(world, "_wr_reftgt", None)
            world._wr_cmd = (1.0 - _ub) * B["src"]["vx"] + _ub * _Tt["vx"]
    if _sched_env:
        if not hasattr(world, "_wr_sched"):
            segs = []
            for part in _sched_env.split(","):
                md, sec = part.split(":")
                segs.append((md.strip(), int(round(float(sec) / 0.016))))
            world._wr_sched = segs
            world._wr_ramp = _i("WALK_RAMP_TICKS", 25)
            world._wr_walk_entered = True
            R._log(world, "WALK-SCHEDULE: %s (ramp %d ticks)" % (segs, world._wr_ramp))
        _t = tt + getattr(world, "_wr_sched_skip", 0); _mode = "walk"; _into = 0; _rem_seg = 0
        for md, ntk in world._wr_sched:
            if ntk <= 0 or _t < ntk:
                _mode, _into = md, _t; _rem_seg = (ntk - _t) if ntk > 0 else 0
                break
            _t -= ntk
        else:
            _mode, _into = world._wr_sched[-1][0], 0
        # SOLO-TURN early exit (owner 2026-07-08, "walk->turn->walk clean"): a turn segment holds a
        # FIXED duration. Once TURN_TO_DEG reaches 90deg the phase FREEZES mid-turn; walking away from
        # that pose IMMEDIATELY topples (the feet are mid-stride), but HOLDING the frozen pose a short
        # SETTLE window lets the robot stabilize on two feet -- THEN the walk re-enters cleanly (the
        # tw14 lesson). So exit BATON_TURN_SETTLE_TICKS after the turn completes, not the instant it does.
        if _mode == "turn" and getattr(world, "_wr_turn_done", False):
            if not hasattr(world, "_wr_turn_done_t"):
                world._wr_turn_done_t = tt
            if (tt - world._wr_turn_done_t) >= _i("BATON_TURN_SETTLE_TICKS", 90) and _rem_seg > 3:
                world._wr_sched_skip = getattr(world, "_wr_sched_skip", 0) + _rem_seg
                _t = tt + world._wr_sched_skip; _mode = "walk"; _into = 0
                for md, ntk in world._wr_sched:
                    if ntk <= 0 or _t < ntk:
                        _mode, _into = md, _t; break
                    _t -= ntk
                else:
                    _mode, _into = world._wr_sched[-1][0], 0
        _wr_turn_swap(world, _mode == "turn")   # SOLO-TURN swap (no-op unless BATON_TURN_CKPT armed)
        # skip the element-wise blend ONLY while a solo-turn context is actively swapped in
        # (_wr_in_turn); with no solo ctx armed this is always False -> the existing turner-in-blend
        # path (turn as a normal BATON specialist) is unchanged.
        if world._wr_baton is not None and not getattr(world, "_wr_in_turn", False):
            # ── BATON (docs/developer/policy-switching.md): N-specialist handover on the NORMAL
            # tick path. On a schedule change the CURRENT effective tables are snapshotted as the
            # morph source and u ramps 0->1 toward the incoming specialist's tables over
            # BATON_MORPH_TICKS (morph, never snap); the act site crossfades outgoing->incoming
            # actions with the same u. ALL specialist LSTMs run every tick = warm handover.
            B = world._wr_baton
            _des = _mode if _mode in B["reg"] else "walk"
            if _des != B["target"]:
                _gate_ok = True
                if B["target"] == "walk" and _i("BATON_DS_GATE", 1) and B["u"] >= 1.0:
                    _DSb = 0.65 * 2.0 * math.pi   # leave a WALK only at double support
                    _phb5 = (world._wr_phase - _DSb) % math.pi
                    _gate_ok = min(_phb5, math.pi - _phb5) <= 0.15
                if _gate_ok:
                    T = B["reg"]
                    def _bl6(k, _u):
                        a = B["src"].get(k); b = T[B["target"]]["tables"].get(k)
                        if a is None or b is None:
                            return a if b is None else b
                        return (1.0 - _u) * a + _u * b
                    # snapshot the CURRENT effective tables as the new morph source
                    B["src"] = {k: (_bl6(k, B["u"]) if k != "vx" else
                                    (1.0 - B["u"]) * B["src"]["vx"] + B["u"] * T[B["target"]]["tables"]["vx"])
                                for k in ("glut", "arm", "elb", "att", "ref", "vx")}
                    B["out"] = B["target"] if B["u"] >= 1.0 else B.get("out", B["target"])
                    B["target"] = _des; B["u"] = 0.0
                    B["switches"] += 1
                    if _i("BATON_COLD_HIDDEN", 0):
                        _inc = B["reg"][_des]
                        if _inc["h"] is not None:
                            _inc["h"] = _inc["net"].init_hidden(1, "cpu")
                        else:
                            world._wr_h = world._wr_net.init_hidden(1, "cpu")
                    R._log(world, "BATON switch #%d (-> %s) at t=%d phase=%.2f" %
                           (B["switches"], _des, tt, world._wr_phase % (2 * math.pi)))
                    _mf = os.environ.get("BATON_MODE_FILE")
                    if _mf:   # visual props (the carry box) follow the arbiter's mode
                        try:
                            open(_mf, "w").write(_des)
                        except OSError:
                            pass
            B["u"] = min(1.0, B["u"] + 1.0 / float(B["morph"]))
            if B["u"] >= 1.0:
                B["active"] = B["target"]
            _Tt = B["reg"][B["target"]]["tables"]; _ub = B["u"]
            def _eff6(k):
                a = B["src"].get(k); b = _Tt.get(k)
                if a is None or b is None:
                    return a if b is None else b
                return (1.0 - _ub) * a + _ub * b
            world._wr_glut = _eff6("glut")
            _a6 = _eff6("arm");  world._wr_armlut = _a6 if _a6 is not None else getattr(world, "_wr_armlut", None)
            _e6 = _eff6("elb");  world._wr_elblut = _e6 if _e6 is not None else getattr(world, "_wr_elblut", None)
            _t6 = _eff6("att");  world._wr_refatt = _t6 if _t6 is not None else getattr(world, "_wr_refatt", None)
            _r6 = _eff6("ref");  world._wr_reftgt = _r6 if _r6 is not None else getattr(world, "_wr_reftgt", None)
            world._wr_cmd = (1.0 - _ub) * B["src"]["vx"] + _ub * _Tt["vx"]
            _mode = "walk"; _blend_u = 1.0   # stay on the normal path; BATON modulates its inputs
        if _mode == "stand":
            # THE CATCH, three measured lessons in one recipe:
            #   run2: snap-to-nominal dragged staggered feet -> slow forward topple (3.5s);
            #   run3: freeze-anywhere froze a mid-swing one-footed pose -> fell backward (1s);
            #   v4: (1) extend the walk until DOUBLE SUPPORT so the caught pose is two-footed,
            #       (2) freeze THAT pose, (3) morph it to the symmetric nominal over ~2.5s --
            #       too slow to drag feet, symmetric in time for the next walk re-entry.
            if world._wr_walk_entered:
                world._wr_walk_entered = False
                world._wr_dswait = 0
                world._wr_standq = None
                world._wr_catch_t = 0
            if world._wr_standq is None:
                _DS = 0.65 * 2.0 * math.pi        # gait-family double-support phase
                _phm = (world._wr_phase - _DS) % math.pi
                if min(_phm, math.pi - _phm) > 0.15 and world._wr_dswait < 80:
                    world._wr_dswait += 1          # not at DS yet: keep walking at the floor cmd
                    world._wr_cmd = _f("WALK_DECEL_FLOOR", 0.30)
                    world._wr_catching = True      # continue the CURRENT gait -- no re-entry reset
                    _mode = "walk"; _blend_u = 1.0
                else:
                    world._wr_catching = False
                    _qpn = world.solver.mjw_data.qpos.numpy().reshape(-1)
                    world._wr_standq = _qpn[world._wr_qadr].copy()
                    if hasattr(world, "_wr_nd2nom"):
                        del world._wr_nd2nom       # ramp blends toward the CAUGHT pose
                    R._log(world, "walk-schedule CATCH at t=%d (DS wait %d ticks)" % (tt, world._wr_dswait))
        if _mode == "stand":
            u_out = min(1.0, _into / max(1, world._wr_ramp))
            if u_out >= 1.0:
                # full stand: deterministic hold, no inference; gait state parked for re-entry
                if world.control is None:
                    world.control = world.model.control()
                tp = world.control.joint_target_pos.numpy()
                if world._wr_whole and getattr(world, "_wr_net2", None) is not None:
                    # ACTIVE stand: the second policy drives (cmd=0, own hidden state).
                    _qp2 = world.solver.mjw_data.qpos.numpy().reshape(-1)
                    _qv2 = world.solver.mjw_data.qvel.numpy().reshape(-1)
                    _w2, _x2, _y2, _z2 = _qp2[3], _qp2[4], _qp2[5], _qp2[6]
                    _gx2 = -2 * (_x2 * _z2 - _w2 * _y2); _gy2 = -2 * (_y2 * _z2 + _w2 * _x2)
                    _gz2 = -(1 - 2 * (_x2 * _x2 + _y2 * _y2))
                    _jp2 = _qp2[world._wr_qadr] - world._wr_nom; _jv2 = _qv2[world._wr_dadr]
                    _yaw2 = math.atan2(2 * (_w2 * _z2 + _x2 * _y2), 1 - 2 * (_y2 * _y2 + _z2 * _z2))
                    _la2 = getattr(world, "_wr_last2", np.zeros(ACT_DIM, np.float32))
                    _o2 = np.concatenate([0.25 * _qv2[3:6], [_gx2, _gy2, _gz2], [0.0, 0.0, 0.0],
                                          _jp2, 0.05 * _jv2, _la2, [0.0, 1.0],
                                          [math.sin(_yaw2), math.cos(_yaw2)]]).astype(np.float32)
                    with torch.no_grad():
                        _m2o, _s2o, world._wr_h2 = world._wr_net2.act(torch.from_numpy(_o2[None, :]), world._wr_h2)
                    _ac2 = np.clip(_m2o.numpy()[0], -1, 1)
                    world._wr_last2 = _ac2.astype(np.float32)
                    _ea2 = _ac2 * world._wr_as
                    for i in range(len(world._wr_ndof)):
                        nd = int(world._wr_ndof[i])
                        if 0 <= nd < len(tp):
                            tp[nd] = float(world._wr_nom[i]) + float(_ea2[i])
                elif world._wr_whole:
                    # frozen catch pose. NO morph toward the symmetric nominal by default:
                    # v6 measured the freeze holding rock-steady and the morph re-dragging
                    # the staggered feet into the same forward topple, just slower. The
                    # walk policy is push-robust; it re-enters from the staggered stance.
                    world._wr_catch_t = getattr(world, "_wr_catch_t", 0) + 1
                    _um = min(_f("WALK_CATCH_MORPH", 0.0), world._wr_catch_t / 160.0)
                    _sq = world._wr_standq if world._wr_standq is not None else world._wr_nom
                    # SOFTEN the catch (v7 lesson: a rigid pose-hold is a spring -- it absorbs
                    # the residual 0.3 m/s, then REBOUNDS the robot backward past its support).
                    # Sink into the knees over the first ~0.3 s and stay: damping + lower COM.
                    _soft = _f("WALK_CATCH_SOFT", 0.12) * min(1.0, world._wr_catch_t / 20.0)
                    if not hasattr(world, "_wr_kneei"):
                        world._wr_kneei = [i for i in range(len(world._wr_ndof))
                                           if int(world._wr_ndof[i]) in
                                           (int(world._wr_legnd[3]), int(world._wr_legnd[9]))]                             if hasattr(world, "_wr_legnd") else []
                    for i in range(len(world._wr_ndof)):
                        nd = int(world._wr_ndof[i])
                        if 0 <= nd < len(tp):
                            tp[nd] = (1.0 - _um) * float(_sq[i]) + _um * float(world._wr_nom[i])
                            if i in world._wr_kneei:
                                tp[nd] += _soft
                else:
                    for i, p in enumerate(world._wr_legpos):
                        nd = world._wr_gdof[p]
                        if 0 <= nd < len(tp):
                            tp[nd] = float(world._wr_nom[i])
                    for p in world._wr_waistpos:
                        nd = world._wr_gdof[p]
                        if 0 <= nd < len(tp):
                            tp[nd] = float(world._wr_waistnom[p])
                world.control.joint_target_pos.assign(tp)
                world._wr_h = world._wr_net.init_hidden(1, "cpu")
                world._wr_last = np.zeros(ACT_DIM, np.float32)
                world._wr_phase = 0.0
                if tt % 25 == 1:
                    _qp0 = world.solver.mjw_data.qpos.numpy().reshape(-1)
                    R._log(world, "walk-schedule STAND t=%d x=%.2f z=%.3f" % (tt, _qp0[0], _qp0[2]))
                return
            _blend_u = 1.0 - u_out           # ramping out: policy authority fades into the stand
        elif not world._wr_walk_entered and not getattr(world, "_wr_catching", False):
            world._wr_walk_entered = True     # fresh walk bout: proven settle-and-go re-entry
            world._wr_catching = False
            world._wr_h = world._wr_net.init_hidden(1, "cpu")
            world._wr_last = np.zeros(ACT_DIM, np.float32)
            world._wr_phase = 0.0
            world._wr_cmd = _f("VX_MAX", 0.7)
            world._wr_reentry_t = tt          # POST-TURN RESET-STEPS anchor (WALK_RESET_STEPS)
            world._wr_reset_xy = None          # crane position-hold origin, captured on first reset tick
            R._log(world, "walk-schedule WALK re-entry at t=%d (fresh phase/hidden)" % tt)
        if _mode == "walk" and not getattr(world, "_wr_catching", False):
            # BRAKE-BY-COMMAND: stopping from full walking speed needs the weight caught
            # ~v/omega0 ahead of the feet -- beyond the ankle strategy. Decelerate the
            # velocity COMMAND over the last WALK_DECEL_TICKS of the bout so the stand
            # catches a nearly-stopped robot instead of a moving one.
            _seg_len = 0
            _t3 = tt
            for md, ntk in world._wr_sched:
                if ntk <= 0 or _t3 < ntk:
                    _seg_len = ntk if ntk > 0 else 10 ** 9
                    break
                _t3 -= ntk
            _remain = _seg_len - _into
            _dec = _i("WALK_DECEL_TICKS", 75)
            if 0 < _remain <= _dec:
                # brake only to the policy's in-distribution FLOOR: cmd below ~VX_START was
                # never trained (documented: zero-cmd eval scored 0.038) -- braking through
                # it made the walker backpedal and topple backward at every catch.
                _u5 = max(0.0, (_remain - 10) / float(_dec))
                world._wr_cmd = _f("WALK_DECEL_FLOOR", 0.30) + (_f("VX_MAX", 0.7) - _f("WALK_DECEL_FLOOR", 0.30)) * _u5
    live = world.solver.mjw_data; qp = live.qpos.numpy().reshape(-1); qv = live.qvel.numpy().reshape(-1)
    w, x, y, z = qp[3], qp[4], qp[5], qp[6]
    gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
    ph = world._wr_phase
    if getattr(world, "_wr_vc_prof", None):
        _ts5 = tt * 0.016; _c5 = world._wr_vc_prof[-1][1]
        for _d5, _v5 in world._wr_vc_prof:
            if _ts5 < _d5:
                _c5 = _v5; break
            _ts5 -= _d5
        _sl5 = _f("CMD_SLEW", 0.008)
        world._wr_cmd = world._wr_cmd + max(-_sl5, min(_sl5, _c5 - world._wr_cmd))
    elif getattr(world, "_wr_vc_cyc", False):
        _c5 = _f("VX_MAX", 0.45) if ((tt * 0.016) % 10.0) < 5.0 else 0.0
        _sl5 = _f("CMD_SLEW", 0.008)
        world._wr_cmd = world._wr_cmd + max(-_sl5, min(_sl5, _c5 - world._wr_cmd))
    if getattr(world, "_wr_seqcmd", None) is not None:
        _tp5 = 2.0 * math.pi * (1.0 - 0.5 / max(1, world._wr_seqnb))
        if world._wr_seq_hold and ph >= _tp5:
            ph = _tp5; world._wr_phase = _tp5          # routine over: pin on the final bin
        _b5 = int((ph % (2.0 * math.pi)) / (2.0 * math.pi) * world._wr_seqnb) % world._wr_seqnb
        world._wr_cmd = float(world._wr_seqcmd[_b5])
        if getattr(world, "_wr_seq_yaw", None) is not None:
            world._wr_heading_tgt = float(world._wr_seq_yaw[_b5])   # heading frame FOLLOWS the turn (as trained)
    jpos = qp[world._wr_qadr] - world._wr_nom; jvel = qv[world._wr_dadr]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    # STEERING (BATON walkto): the heading obs is what the policy regulates to zero ("steer
    # straight"); feeding yaw RELATIVE to a target heading re-aims that regulation -- the
    # policy turns toward HEADING_TARGET with zero retraining. Slewed to avoid step inputs.
    _ht = getattr(world, "_wr_heading_tgt", None)
    _tdwz = _f("TURN_DEPLOY_WZ", 0.0)
    if _tdwz != 0.0:
        # TURN specialist live: in training the heading target ROTATED at wz and the policy
        # turned to track it (the heading error was the drive -- feeding a constant 0 stalls
        # the turn after the initial gait momentum). Rotate the deploy target at the SAME rate:
        # the policy tracks it and turns continuously. This is the controllable turn BATON needs.
        _cur = getattr(world, "_wr_heading_tgt", None)
        if _cur is None:
            _cur = yaw
        _cur = math.atan2(math.sin(_cur + _tdwz * 0.016), math.cos(_cur + _tdwz * 0.016))
        world._wr_heading_tgt = _cur
        _ht = _cur
    if _i("TURN_MODE", 0) or getattr(world, "_wr_turn_active", False):
        # UNCONDITIONAL TURN specialist: in training the heading target ROTATED with the robot,
        # so the heading obs sat near 0 while it spun. Feed 0 live so the baked-in turn runs
        # in-distribution (there is no live command -- the spin is intrinsic, like the stander).
        # _wr_turn_active is set by the BATON course arbiter during a "turn" segment.
        yaw_obs = 0.0
    elif _ht is not None:
        _he = yaw - _ht
        _he = math.atan2(math.sin(_he), math.cos(_he))   # wrap to [-pi, pi]
        _hcap = _f("HEADING_ERR_CAP", 0.5)               # the obs never saw |yaw| >> 0 in training
        _he = max(-_hcap, min(_hcap, _he))
        yaw_obs = _he
    else:
        yaw_obs = yaw
    o = np.concatenate([0.25 * qv[3:6], [gx, gy, gz], [world._wr_cmd, 0.0, 0.0], jpos, 0.05 * jvel,
                        world._wr_last, [math.sin(ph), math.cos(ph)],
                        [math.sin(yaw_obs), math.cos(yaw_obs)]]).astype(np.float32)   # heading
    if getattr(world, "_wr_reftgt", None) is not None:   # append reference block (matches build_ref_block)
        _gbr = int((ph % (2 * math.pi)) / (2 * math.pi) * world._wr_refnb) % world._wr_refnb
        _cur = qp[world._wr_ref_qadr]
        _rc = [world._wr_reftgt[(_gbr + k * world._wr_ref_stride) % world._wr_refnb] - _cur
               for k in range(1, world._wr_ref_K + 1)]
        if world._wr_ref_useatt:
            _rc.append(world._wr_refatt[(_gbr + world._wr_ref_stride) % world._wr_refnb])
        o = np.concatenate([o] + _rc).astype(np.float32)
    with torch.no_grad():
        mean, _std, world._wr_h = world._wr_net.act(torch.from_numpy(o[None, :]), world._wr_h)
    ac = np.clip(mean.numpy()[0], -1, 1)
    if os.environ.get("WALK_REPLAY_ACTIONS"):
        # ACTION-REPLAY parity probe: feed the RECORDED trainer actions (EVAL_ACT_RECORD, world 0,
        # EVAL_IC=0 so both sides start from the identical seeded crouch) through the identical
        # corridor+harness pipeline. If the live trajectory tracks the recording, the plant AND
        # harness are equivalent in motion -> the gap is policy-side; divergence localizes it.
        if not hasattr(world, "_wr_rp"):
            _rpz = np.load(os.environ["WALK_REPLAY_ACTIONS"])
            world._wr_rp = {"acts": _rpz["acts"], "qpos": _rpz["qpos"], "i": 0}
            R._log(world, "REPLAY armed: %d recorded ticks" % len(world._wr_rp["acts"]))
        _rp = world._wr_rp
        if _rp["i"] < len(_rp["acts"]):
            ac = _rp["acts"][_rp["i"]].astype(np.float32)
            if _rp["i"] % 50 == 0:
                _rq = _rp["qpos"][_rp["i"]]
                R._log(world, "REPLAY i=%d live(x=%.3f z=%.3f) rec(x=%.3f z=%.3f) dx=%.3f" %
                       (_rp["i"], qp[0], qp[2], _rq[0], _rq[2], qp[0] - _rq[0]))
            _rp["i"] += 1
    if world._wr_baton is not None and not getattr(world, "_wr_in_turn", False):
        # warm N-specialist handover: EVERY specialist runs on the SAME obs each tick (all LSTM
        # states stay live); the applied action crossfades outgoing -> incoming with the morph u.
        # SKIP while the solo-turn is swapped in: the obs is the turner's 153-dim vector and the
        # blend specialists are 120-dim -> feeding it to them size-mismatches (the turn runs solo).
        B = world._wr_baton
        _acts = {"walk": ac}
        with torch.no_grad():
            for _nm7, _sp7 in B["reg"].items():
                if _sp7["h"] is None:
                    continue                       # "walk" = the primary net, already computed
                _m7, _s7, _sp7["h"] = _sp7["net"].act(torch.from_numpy(o[None, :]), _sp7["h"])
                _acts[_nm7] = np.clip(_m7.numpy()[0], -1, 1)
        _out7 = B.get("out", B["target"])
        ac = (1.0 - B["u"]) * _acts.get(_out7, ac) + B["u"] * _acts.get(B["target"], ac)
    world._wr_last = ac.astype(np.float32)
    if os.environ.get("WALK_GAIT_LOG"):
        # per-tick FULL-BODY gait recording (t, phase, roll, pitch, vx, leg q[12], shoulder q[2])
        # -- raw material for the ACHIEVED ghost v3: a COMPLETE reference (joints + base attitude
        # + speed) built from OUR OWN robot's walking cycle, provably doable by this body.
        if not hasattr(world, "_wr_glog"):
            world._wr_glog = open(os.environ["WALK_GAIT_LOG"], "w", buffering=1)
            world._wr_gleg = _build_legmap(world)[0]
            world._wr_gshq = []
            try:   # shoulder qpos addresses (geometric, same rule as the arm corridor)
                import mujoco as _mjn2
                _m2 = world.solver.mj_model
                _t2 = np.asarray(_m2.actuator_trnid).reshape(int(_m2.nu), -1)
                _d2 = _mjn2.MjData(_m2); _mjn2.mj_forward(_m2, _d2)
                _pz2 = 0.7
                for j in range(int(_m2.njnt)):
                    if int(_m2.jnt_type[j]) == int(_mjn2.mjtJoint.mjJNT_FREE):
                        _pz2 = float(_d2.xpos[int(_m2.jnt_bodyid[j])][2]); break
                _qa2, _da2, _ai2 = _build_all_pos_act(world)
                _cands2 = {1.0: None, -1.0: None}
                for a in _ai2:
                    j = int(_t2[int(a)][0])
                    ax = np.abs(np.array(_m2.jnt_axis[j], float))
                    pos = np.array(_d2.xpos[int(_m2.jnt_bodyid[j])], float)
                    if int(np.argmax(ax)) == 1 and pos[2] > _pz2 + 0.15 and abs(pos[1]) > 0.05:
                        sd = 1.0 if pos[1] > 0 else -1.0
                        if _cands2[sd] is None or pos[2] > _cands2[sd][0]:
                            _cands2[sd] = (float(pos[2]), int(_m2.jnt_qposadr[j]))
                if _cands2[1.0] and _cands2[-1.0]:
                    world._wr_gshq = [_cands2[1.0][1], _cands2[-1.0][1]]
            except Exception:
                pass
        _rr2 = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        _pp2 = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        world._wr_glog.write("%.4f %.4f %.4f %.4f %.4f %s %s\n" % (
            tt * 0.016, (ph % (2 * math.pi)) / (2 * math.pi), _rr2, _pp2, qv[0],
            " ".join("%.5f" % qp[q] for q in world._wr_gleg),
            " ".join("%.5f" % qp[q] for q in world._wr_gshq)))
    if tt <= 3:   # obs/action parity logging vs EVAL_LOG_OBS on the eval side
        R._log(world, "DEP-OBS tt=%d angv=%s projg=%s jpos6=%s jvel6=%s ph=%.3f act6=%s" % (
            tt, [round(float(v), 4) for v in o[0:3]], [round(float(v), 4) for v in o[3:6]],
            [round(float(v), 4) for v in o[9:15]], [round(float(v), 4) for v in o[9 + ACT_DIM:15 + ACT_DIM]],
            math.sin(ph), [round(float(v), 3) for v in ac[0:6]]))
    _oat = _i("OBS_AUDIT_T", 0)
    if _oat > 0 and _oat <= tt < _oat + 3:   # steady-state FULL-vector audit (live side)
        R._log(world, "OBS-AUDIT-LIVE tt=%d o=%s act=%s" % (
            tt, [round(float(v), 4) for v in o], [round(float(v), 4) for v in ac]))
    env_a = ac * world._wr_as
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    if world._wr_whole:                                       # legs + waist + arms, by newton dof
        for i in range(len(world._wr_ndof)):
            nd = int(world._wr_ndof[i])
            if 0 <= nd < len(tp):
                tp[nd] = float(world._wr_nom[i]) + float(env_a[i])
        if world._wr_gres > 0:                                # legs: ghost feedforward + bounded residual
            gb = int(((ph + _f("PHASE_LEAD", 0.0)) % (2 * math.pi)) / (2 * math.pi) * world._wr_gnb) % world._wr_gnb
            _g5 = 1.0
            if getattr(world, "_wr_vc", False):
                # live latched gate, mirroring training's _vc_gate_step: full gait while
                # moving, step to stand at double support; restarts pass through instantly.
                # Velocity is LOW-PASSED (stride pulsation deadlocked the raw-vx latch).
                world._wr_vc_vema = 0.95 * getattr(world, "_wr_vc_vema", abs(world._wr_cmd))                     + 0.05 * abs(float(qv[0]))
                _gt5 = (min(1.0, abs(world._wr_cmd) / _f("VC_GATE_V", 0.15)) if _i("VC_GATE_CMDONLY", 0)
                        else min(1.0, max(abs(world._wr_cmd), world._wr_vc_vema) / _f("VC_GATE_V", 0.15)))
                _ds5 = 0.65 * 2.0 * math.pi
                _pm5 = (ph - _ds5) % math.pi
                _dn5 = _f("VC_GATE_DOWN", 0.0)
                if _gt5 >= world._wr_vc_glat:
                    world._wr_vc_glat = _gt5
                elif _dn5 > 0:
                    if min(_pm5, math.pi - _pm5) < 0.35 or world._wr_vc_glat < 0.999:
                        world._wr_vc_glat = max(_gt5, world._wr_vc_glat - _dn5)  # morph release
                elif min(_pm5, math.pi - _pm5) < 0.35:
                    world._wr_vc_glat = _gt5
                _g5 = world._wr_vc_glat
            for col in range(12):
                nd = int(world._wr_legnd[col])
                if 0 <= nd < len(tp):
                    _w = (world._wr_greslat if col in (1, 5, 7, 11)
                          else world._wr_gresyaw if col in (2, 8) else world._wr_gres)
                    if getattr(world, "_wr_stight", 0.0) > 0 and col not in (1, 5, 7, 11):
                        _sg5 = float(world._wr_swgate[gb][0 if col < 6 else 1])
                        _w *= world._wr_stight + (1.0 - world._wr_stight) * _sg5
                    _lref5 = world._wr_glut[gb][col]
                    if _g5 < 1.0:
                        _nom5 = float(world._wr_nom[world._wr_liu[col]]) if world._wr_whole else float(world._wr_nom[col])
                        _lref5 = _g5 * float(_lref5) + (1.0 - _g5) * _nom5
                    tp[nd] = float(_lref5) + float(ac[world._wr_liu[col]]) * _w
            if getattr(world, "_wr_ares", 0.0) > 0:           # shoulders: arm-ghost corridor
                for k in range(2):
                    nd = int(world._wr_armnd[k])
                    if 0 <= nd < len(tp):
                        tp[nd] = float(world._wr_armlut[gb][k]) + float(ac[world._wr_armliu[k]]) * world._wr_ares
            if getattr(world, "_wr_eres", 0.0) > 0:           # elbows: fixed-angle corridor (human hang)
                for k in range(2):
                    nd = int(world._wr_elbnd[k])
                    if 0 <= nd < len(tp):
                        _et5 = (float(world._wr_elblut[gb][k]) if getattr(world, "_wr_elblut", None) is not None
                                else world._wr_elbtgt)
                        tp[nd] = _et5 + float(ac[world._wr_elbliu[k]]) * world._wr_eres
            if getattr(world, "_wr_shryliu", None):               # shoulder roll/yaw: pinned corridor
                for k in range(len(world._wr_shryliu)):
                    nd = int(world._wr_shrynd[k])
                    if 0 <= nd < len(tp):
                        tp[nd] = world._wr_shrytgts[k] + float(ac[world._wr_shryliu[k]]) * world._wr_shryres
    else:
        for i, p in enumerate(world._wr_legpos):
            nd = world._wr_gdof[p]
            if 0 <= nd < len(tp):
                tp[nd] = float(world._wr_nom[i]) + float(env_a[i])
        for p in world._wr_waistpos:
            nd = world._wr_gdof[p]
            if 0 <= nd < len(tp):
                tp[nd] = float(world._wr_waistnom[p])
    if _blend_u < 1.0:
        # walk->stand ramp: fade every controlled dof's target toward the stand nominal
        if not hasattr(world, "_wr_nd2nom"):
            m2n = {}
            if world._wr_whole:
                _hold = getattr(world, "_wr_standq", world._wr_nom)
                for i in range(len(world._wr_ndof)):
                    m2n[int(world._wr_ndof[i])] = float(_hold[i])
            else:
                for i, p in enumerate(world._wr_legpos):
                    m2n[int(world._wr_gdof[p])] = float(world._wr_nom[i])
                for p in world._wr_waistpos:
                    m2n[int(world._wr_gdof[p])] = float(world._wr_waistnom[p])
            world._wr_nd2nom = m2n
        for nd, nomv in world._wr_nd2nom.items():
            if 0 <= nd < len(tp):
                tp[nd] = (1.0 - _blend_u) * nomv + _blend_u * float(tp[nd])
    # NOTE: deliberately NOT setting _mjc_dirty. Control writes are applied by the solver's
    # apply_mjc_control each substep and need no state re-import; dirtying every tick forced a
    # newton->mjw copy-in that CLOBBERED the handoff reset and re-imported state per tick — a
    # regime the batched trainer never sees. Clean ticks == the trainer's regime.
    if getattr(world, "_wr_hlam", 0.0) > 0 and qp[2] > 0.35:
        # DEPLOY HARNESS via control.joint_f on the FREE JOINT (dofs 0-5). SolverMuJoCo's
        # _apply_mjc_control REBUILDS xfrc from control.joint_f each step (source-verified):
        # direct xfrc writes get overwritten, and state.body_f/add_body_force is never read
        # by this adapter. joint_f free-dof layout probed via HARNESS_JF_ANGFIRST.
        _lam5 = world._wr_hlam
        _r5 = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        _p5 = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        if abs(_r5) > 0.6 or abs(_p5) > 0.6:
            _lam5 = 0.0   # robot is toppling: harness wrenches on a tumbling body pump energy -> numeric explosion (measured)
        # HARNESS_ATT_GHOST (deploy mirror): spring toward the ghost's recorded sway, not level
        if _i("HARNESS_ATT_GHOST", 0) and getattr(world, "_wr_refatt", None) is not None:
            _nb6 = world._wr_refnb
            _gb6 = int(world._wr_phase / (2.0 * math.pi) * _nb6) % _nb6
            _r5 -= float(world._wr_refatt[_gb6][0]); _p5 -= float(world._wr_refatt[_gb6][1])
        if _i("HARNESS_FY_HEADING", 0) and getattr(world, "_wr_heading_tgt", None) is not None:
            # HEADING-AWARE lateral catch (owner 2026-07-08, walk->turn->walk): the world-frame
            # brake (-FY*qv[1]) only damps the LATERAL axis when the robot faces world +x (leg 1).
            # After a 90deg footwork turn the robot faces +y, so world-y IS its forward travel ->
            # the brake FIGHTS the walk and leaves the unbraked +x drift, so leg 2 crabs diagonally
            # (measured: faces 82deg, travels 49deg). Rotate the catch into the travel-heading frame
            # exactly as the TRAINER harness does (line ~1227): brake only the velocity PERPENDICULAR
            # to the heading target, never along it. A no-op at heading 0 (leg 1 / straight demos).
            _hh5 = float(world._wr_heading_tgt); _ch5 = math.cos(_hh5); _sh5 = math.sin(_hh5)
            _lat5v = -_sh5 * float(qv[0]) + _ch5 * float(qv[1])   # lateral (cross-heading) velocity
            _Fl5 = max(-700.0, min(700.0, _lam5 * (-_f("HARNESS_FY", 400.0) * _lat5v)))
            _fx5 = -_sh5 * _Fl5; _fy5 = _ch5 * _Fl5
        else:
            _fx5 = 0.0
            _fy5 = max(-700.0, min(700.0, _lam5 * (-_f("HARNESS_FY", 400.0) * float(qv[1]))))
        _rsteps = _i("WALK_RESET_STEPS", 0)   # POST-TURN RESET-STEPS treadmill (2026-07-08)
        if _rsteps > 0 and _mode == "walk" and 0 <= (tt - getattr(world, "_wr_reentry_t", -10 ** 9)) < _rsteps:
            # After a footwork turn the feet are left in a ROTATED stance a static hold can't reset (the
            # crab persisted, ~45deg). Let the walker STEP at full cmd (in-distribution -- low cmd
            # backpedals) while the crane HOLDS its position (a treadmill): it steps IN PLACE and
            # re-plants its feet square to the new heading, then release -> leg 2 walks clean from a
            # re-squared stance. Position-hold force added to the (heading-aware) crane fx/fy, then clamped.
            if getattr(world, "_wr_reset_xy", None) is None:
                world._wr_reset_xy = (float(qp[0]), float(qp[1]))
            _kh = _f("WALK_RESET_KHOLD", 600.0); _dh = _f("WALK_RESET_DHOLD", 120.0)
            _fx5 += _lam5 * (-_kh * (float(qp[0]) - world._wr_reset_xy[0]) - _dh * float(qv[0]))
            _fy5 += _lam5 * (-_kh * (float(qp[1]) - world._wr_reset_xy[1]) - _dh * float(qv[1]))
            _fx5 = max(-700.0, min(700.0, _fx5)); _fy5 = max(-700.0, min(700.0, _fy5))
        # TURN-HOLD POSITION LOCK (2026-07-08 bugfix): after the decel-stop FREEZES the turn phase, the
        # legs are static and can't step to arrest the residual FORWARD velocity (the heading-aware FY
        # brake only damps LATERAL) -> the robot COASTS/SLIDES away forever (measured ~10 m, "flying
        # without moving its legs"). While a COMPLETED turn is held (done + active), lock the pelvis to
        # its turn-completion xy so it holds the turn IN PLACE. Off before completion and once the turn
        # segment ends (walk re-enters) -> no effect on the walk-turn-walk leg 2.
        if not getattr(world, "_wr_turn_done", False):
            world._wr_turnhold_xy = None
        elif getattr(world, "_wr_turn_active", False):
            if getattr(world, "_wr_turnhold_xy", None) is None:
                _yh0 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                world._wr_turnhold_xy = (float(qp[0]), float(qp[1]), _yh0)   # x, y, ACHIEVED turn yaw
            _khT = _f("TURN_HOLD_KHOLD", 450.0); _dhT = _f("TURN_HOLD_DHOLD", 90.0)
            _fx5 = max(-700.0, min(700.0, _lam5 * (-_khT * (float(qp[0]) - world._wr_turnhold_xy[0]) - _dhT * float(qv[0]))))
            _fy5 = max(-700.0, min(700.0, _lam5 * (-_khT * (float(qp[1]) - world._wr_turnhold_xy[1]) - _dhT * float(qv[1]))))
        _fz5 = max(0.0, min(700.0, _lam5 * (_f("HARNESS_KZ", 2000.0) * (_f("HARNESS_Z0", 0.72) - float(qp[2]))
                                            - _f("HARNESS_DZ", 150.0) * float(qv[2]))))
        _tx5 = max(-350.0, min(350.0, _lam5 * (-_f("HARNESS_KP", 600.0) * _r5 - _f("HARNESS_KD", 60.0) * float(qv[3]))))
        _ty5 = max(-350.0, min(350.0, _lam5 * (-_f("HARNESS_KP", 600.0) * _p5 - _f("HARNESS_KD", 60.0) * float(qv[4]))))
        _tz5 = 0.0
        _hky = _f("HARNESS_KYAW", 0.0)
        if getattr(world, "_wr_turn_active", False):
            _hky = 0.0   # a TURN segment: the turn specialist rotates itself -- the crane
            _tz5 = 0.0   # steering toward the stale pre-turn heading would fight it
            # TURN-HOLD YAW LOCK (2026-07-08): once the footwork turn has DONE (decel-stop), the frozen
            # turn pose is NOT yaw-stable -- the robot un-rotates/oscillates back (measured 90->34), so
            # leg 2 starts from the wrong heading and crabs backward. Hold the ACHIEVED turn yaw with a
            # gentle crane torque. The TURN itself stayed wtz=0 (real footwork); this only STABILIZES the
            # completed result, exactly as tx/ty already hold the attitude and fx/fy hold the position.
            _thold = getattr(world, "_wr_turnhold_xy", None)
            if getattr(world, "_wr_turn_done", False) and _thold is not None and len(_thold) >= 3:
                _yhn = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                _yeH = math.atan2(math.sin(_yhn - _thold[2]), math.cos(_yhn - _thold[2]))
                _tz5 = max(-350.0, min(350.0, _lam5 * (-_f("TURN_HOLD_KYAW", 0.0) * _yeH
                                                       - _f("TURN_HOLD_KYD", 45.0) * float(qv[5]))))
        if _hky > 0 and getattr(world, "_wr_heading_tgt", None) is not None:
            # CRANE-YAW steering (2026-07-06): the policy's heading channel is live-dead (verified:
            # correct obs, turns in-trainer, zero live response -- hidden-state-gated). The harness
            # already proves tx/ty work (pulse probe); a gentle yaw torque toward the course bearing
            # steers the CHASSIS while the policy walks. Puppet-consistent: the crane orients its robot.
            # ⛔ PELVIS-FRAME OFFSET (square4 telemetry, measured): the G1 pelvis frame sits
            # ~-1.44 rad off the travel direction -- walking straight along world +x reads
            # pelvis-yaw -1.44. Both the (dead) policy heading obs AND this crane compared raw
            # pelvis yaw to a WORLD bearing, so the crane could not tell a real 90-degree turn
            # from the baked-in offset (the delivery only worked because it was near-straight).
            # Give the CRANE the honest TRAVEL heading; leave the policy obs raw (it saw -1.44
            # in training -- offsetting it would be out-of-distribution on a dead channel).
            _yaw5 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) - _f("HARNESS_YAW_OFFSET", 0.0)
            _yerr5 = _yaw5 - world._wr_heading_tgt
            _yerr5 = math.atan2(math.sin(_yerr5), math.cos(_yerr5))
            # STAND-PIVOT vs WALK-TRIM: a WALKING robot tumbles above ~55 N*m of yaw (the feet
            # can't follow), so walk steering stays a gentle +/-0.12 budget. But a STATIONARY
            # standing robot has no forward momentum to tumble, so the corner pivot cranks a
            # much higher torque -- enough to break the mu=1.5 static friction and rotate the
            # pelvis by a full 90 degrees in place.
            if getattr(world, "_wr_pivot_active", False):
                # square6: at 108 N*m the pivot rotated fine (roll/pit stayed <0.15) but was
                # UNDERDAMPED -- it overshot the 90-degree target and oscillated ~1.7 rad back,
                # and the walk re-entry caught it mid-swing -> tumble. So the in-place pivot runs
                # a heavily-damped PD (HARNESS_KYD_STAND >> the walk damping): rotate to the
                # target and STOP, no ring.
                _ycap = _f("HARNESS_YAW_CAP_STAND", 0.6); _hkye = _f("HARNESS_KYAW_STAND", 120.0)
                _ramp5 = 1.0; _tzc = _f("HARNESS_TZ_CAP_STAND", 140.0); _kyd5 = _f("HARNESS_KYD_STAND", 140.0)
            else:
                _ycap = 0.12; _hkye = _hky
                _ramp5 = min(1.0, max(0.0, (tt - 150) / 400.0)); _tzc = 350.0; _kyd5 = _f("HARNESS_KYD", 30.0)
            _yerr5 = max(-_ycap, min(_ycap, _yerr5))
            _tz5 = max(-_tzc, min(_tzc, _ramp5 * _lam5 * (-_hkye * _yerr5 - _kyd5 * float(qv[5]))))
        if _i("HARNESS_PROBE", 0):
            # PULSE-PROBE rev2 (2026-07-06 live-gap hunt): torque pulses ADDED ON TOP of the
            # live spring (rev1 with the spring off toppled the lam0.9 stander -- pendulum
            # chaos, unreadable; accidental datum: the slots are NOT dead). With the spring
            # holding, a +80 N*m pulse should deflect the PULSED axis by ~T/(lam*KP) ~ 0.15 rad.
            _tx5 = max(-350.0, min(350.0, _tx5 + (80.0 if 500 <= tt < 750 else 0.0)))
            _ty5 = max(-350.0, min(350.0, _ty5 + (80.0 if 1200 <= tt < 1450 else 0.0)))
        # DEBUG (2026-07-07, owner "prove the turn is footwork, not the rope"): stash the harness
        # wrench so the telemetry can log the YAW torque. Note the harness applies NO forward force
        # (jf = [0, fy, fz, tx, ty, tz]) -- forward travel is always footwork. tz is the only channel
        # that could rotate the robot externally; with HARNESS_KYAW=0 it stays 0 -> any turn is footwork.
        world._wr_dbg_wrench = (_fy5, _fz5, _tx5, _ty5, _tz5)
        try:
            if not hasattr(world, "_wr_jf0"):
                # the robot's FREE-joint dof offset in control.joint_f: with other free bodies in
                # the scene (the REAL carry box) the robot's base is NOT at dof 0 -- assuming 0-5
                # sent the harness wrench to the box and left the robot bare (2026-07-06).
                world._wr_jf0 = 0
                try:
                    _jqds = world.model.joint_qd_start.numpy()
                    _jc = world.model.joint_child.numpy()
                    for _j in range(len(_jc)):
                        if int(_jc[_j]) == int(getattr(world, "_wr_hpb", 5)):
                            world._wr_jf0 = int(_jqds[_j])
                            break
                    R._log(world, "DEPLOY HARNESS joint_f base offset resolved: %d" % world._wr_jf0)
                except Exception as _eo5:
                    R._log(world, "DEPLOY HARNESS jf-offset err %r (fallback 0)" % (_eo5,))
            _jf5 = world.control.joint_f
            if _jf5 is not None:
                _jfa = _jf5.numpy()
                _jfa[:] = 0.0
                _o5 = world._wr_jf0
                if _i("HARNESS_JF_ANGFIRST", 0):
                    _jfa[_o5:_o5 + 6] = [_tx5, _ty5, _tz5, _fx5, _fy5, _fz5]
                else:
                    _jfa[_o5:_o5 + 6] = [_fx5, _fy5, _fz5, _tx5, _ty5, _tz5]
                _jf5.assign(_jfa)
        except Exception as _he5:
            if not getattr(world, "_wr_herr", False):
                world._wr_herr = True
                R._log(world, "DEPLOY HARNESS joint_f error: %r" % (_he5,))
    # TURN-SETTLE NEUTRAL STANCE (2026-07-08 bugfix): during the post-turn settle window the FROZEN turn
    # pose (legs in a CCW-turning config) actively pushes the robot to UN-ROTATE (measured 90->34/52), so
    # leg 2 starts from the wrong heading and crabs. Put the legs in the neutral SQUARE stand pose instead
    # so they stop pushing -> the robot settles two-footed at the ACHIEVED yaw (crane holds it), and leg 2
    # exits from a clean heading. Only while a COMPLETED turn is held; off before completion and in leg 2.
    if _i("TURN_SETTLE_NEUTRAL", 0) and getattr(world, "_wr_turn_done", False) and getattr(world, "_wr_turn_active", False):
        _hq2 = getattr(world, "_wr_standq", None)
        _nomq = getattr(world, "_wr_nom", None)
        if _nomq is not None:
            for i in range(len(world._wr_ndof)):
                nd = int(world._wr_ndof[i])
                if 0 <= nd < len(tp):
                    tp[nd] = float(_hq2[i]) if _hq2 is not None else float(_nomq[i])
    world.control.joint_target_pos.assign(tp)
    # TURN-TO-TARGET (2026-07-07, owner: "loop the turn, stop at 90 -- don't settle a one-shot").
    # A continuous turn is a stable limit cycle; the fall came from over-spinning with no target.
    # But you can't STOP a steady turn by freezing -- the angular momentum carries on (measured:
    # freeze at 85 -> spun to 306 and fell). You DECELERATE: ramp the reference speed down over the
    # last TURN_DECEL_DEG so the turn slows to a stop and the policy has time to arrest the spin
    # (exactly what the finite ghost's decel-into-stand did). Off by default -> exact prior behavior.
    _t2d = _f("TURN_TO_DEG", 0.0)
    if _t2d > 0.0 and (getattr(world, "_wr_turn_active", False) or not getattr(world, "_wr_turnctx", None)):
        # scope: with a SOLO-TURN context armed, the decel-to-stop applies ONLY during a turn segment
        # (the accumulator is reset on turn entry); a plain single-policy turn run (no solo ctx) keeps
        # the original always-on behavior.
        # accumulate the ACTUAL (unwrapped) heading change -- integrating the yaw RATE (qv[5])
        # drifts and fired the stop at inconsistent real angles (95->103, 82->overshoot+fall).
        _ynow = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        _yprev = getattr(world, "_wr_yaw_prev", _ynow)
        world._wr_turn_acc = getattr(world, "_wr_turn_acc", 0.0) + math.atan2(math.sin(_ynow - _yprev), math.cos(_ynow - _yprev))
        world._wr_yaw_prev = _ynow
        _rem = _t2d - abs(world._wr_turn_acc) * 57.2958            # degrees still to turn
        _sc = max(0.0, min(1.0, _rem / max(1.0, _f("TURN_DECEL_DEG", 35.0))))   # 1 far -> 0 at target
        world._wr_phase += world._wr_omega * world._wr_dt * _sc
        if _rem <= 0 and not getattr(world, "_wr_turn_done", False):
            world._wr_turn_done = True
            R._log(world, "TURN-TO-TARGET %.0f deg reached at t=%d (decelerated to a stop)" % (_t2d, tt))
    else:
        world._wr_phase += world._wr_omega * world._wr_dt
    if _i("FOOT_LOG", 0) and tt % _i("FOOT_LOG_EVERY", 8) == 0:   # numerical foot-vs-tread verification (owner 2026-07-08)
        try:
            if not hasattr(world, "_wr_footids5"):
                world._wr_footids5 = _footmap(world)[1]
            _xpf = np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
            _fLp = _xpf[int(world._wr_footids5[0])]; _fRp = _xpf[int(world._wr_footids5[1])]
            R._log(world, "FOOTLOG t=%d bx=%.4f bz=%.4f fLx=%.4f fLz=%.4f fRx=%.4f fRz=%.4f"
                   % (tt, float(qp[0]), float(qp[2]), float(_fLp[0]), float(_fLp[2]), float(_fRp[0]), float(_fRp[2])))
        except Exception:
            pass
    if tt % 20 == 1:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pit = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        _yaw9 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) - _f("HARNESS_YAW_OFFSET", 0.0)
        _ht9 = getattr(world, "_wr_heading_tgt", None)
        _wr5 = getattr(world, "_wr_dbg_wrench", (0.0, 0.0, 0.0, 0.0, 0.0))
        # hip-yaw joints (the turn actuators): actual vs ghost target, to tell tracking-gap from
        # ground-reaction-gap. leg slots 2 (L hip yaw) / 8 (R hip yaw).
        _hyL = float(qp[world._wr_qadr[2]]); _hyR = float(qp[world._wr_qadr[8]])
        _ghy = ""
        if getattr(world, "_wr_reftgt", None) is not None:
            _gb9 = int((world._wr_phase % (2 * math.pi)) / (2 * math.pi) * world._wr_refnb) % world._wr_refnb
            _ghy = " ghyL=%.3f ghyR=%.3f" % (float(world._wr_reftgt[_gb9][2]), float(world._wr_reftgt[_gb9][8]))
        R._log(world, "walk-recipe deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f cmd=%.2f vx=%.2f glat=%.2f yaw=%.3f htgt=%s wtz=%.1f wfy=%.1f hyL=%.3f hyR=%.3f%s"
               % (tt, qp[0], qp[1], qp[2], roll, pit, float(world._wr_cmd), float(qv[0]),
                  float(getattr(world, "_wr_vc_glat", -1.0)), _yaw9,
                  "%.3f" % _ht9 if _ht9 is not None else "-", float(_wr5[4]), float(_wr5[0]),
                  _hyL, _hyR, _ghy))
