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
from projects.policies.training import baton as BATON       # the policy-switching LIBRARY (robot-agnostic)
from projects.policies.training import baton_hosts as BHOST  # ...and this robot's adapter for it
from projects.policies.common import robot_registry as RR   # what a robot IS (urdf, joint roles)
from projects.policies.common import shadow_env as SE       # ONE env contract for Shadowing


def _ctl_target_pos(world):
    """`Control` position-target array, across the newton 1.5 rename.

    newton 1.5 REMOVED `Control.joint_target_pos` / `joint_target_vel` in favour
    of `joint_target_q` / `joint_target_qd`; the old names are `RemovedAttribute`
    descriptors that RAISE on access. The in-engine hook catches pymod errors and
    logs them to the MPC log only, so before this shim the deploy lane threw on
    EVERY tick, applied no joint target at all, and still exited 0 with the engine
    log reading "0 errors" -- the robot simply toppled (2026-08-24).

    The rename is layout-preserving FOR US: `joint_target_q` is DOF-shaped exactly
    like the old field unless `newton.use_coord_layout_targets` is on, and it
    defaults to False. Every index in this file is a qd/DOF index, so nothing
    needs reindexing -- but if that global is ever flipped, targets would land on
    the wrong joint SILENTLY. Do not enable it without reindexing this file.

    Resolved per call via getattr so ONE source drives both runtimes; mirrors
    OmniSimNewtonWorld._ctl_target_pos in src/omnisim/physics/omnisim_newton_runtime.py.
    """
    c = world.control
    a = getattr(c, "joint_target_q", None)
    return a if a is not None else getattr(c, "joint_target_pos", None)


ACT_DIM = 12
# actor obs: angvel(3)+projg(3)+cmd(3)+jpos(12)+jvel(12)+preva(12)+phase(2)+heading(2) = 49
# heading = sin/cos of world yaw: projected gravity is YAW-BLIND, so without this the policy can't
# sense which way it faces -> unbounded heading drift (it curves sideways and falls). This is the fix.
OBS_DIM = 3 + 3 + 3 + 12 + 12 + 12 + 2 + 2
# privileged extras the CRITIC also gets: base linvel(3)+foot z(2)+foot contact(2) = 7
PRIV_DIM = 3 + 2 + 2

# ── STATE-ESTIMATOR HEAD (EST_HEAD=1) ────────────────────────────────────────────────────────
# WHY (the free-standing campaign, 2026-07-13): the balance harness is a FEEDBACK CONTROLLER ON
# BASE LINEAR VELOCITY -- HARNESS_FY damps lateral base velocity, HARNESS_KZ/DZ regulate base
# height and vertical velocity. The actor cannot see ANY of it: base linvel is PRIV (critic-only)
# and is thrown away every tick. So weaning the crane asks the policy to replace a velocity damper
# with its own foot placement, using a network that has never been given one gradient telling it to
# represent velocity. And free-standing walking at speed must be DYNAMIC (capture-point stepping),
# whose control law is literally COM velocity x sqrt(z/g) -- you cannot express it without velocity.
#
# Fix (Ji et al., RA-L 2022, concurrent state estimation; cf. RMA): hang a head off the actor's
# recurrent trunk that REGRESSES the privileged vector from the actor's own observation history,
# supervised by the priv the rollout already collects. The gradient flows back through the LSTM, so
# the hidden state is FORCED to encode base velocity + contact state; the estimate is then fed to
# the policy head explicitly (detached -- the policy consumes it, but only the supervised loss
# shapes it, so PPO cannot hijack the estimator into a free-form feature). Zero deploy cost: the
# head runs inside the actor, and needs nothing the actor doesn't already have.
#
# Target is HEADING-FRAME (not world-frame like the critic's priv): forward/lateral velocity is the
# proprioceptively-inferable, physically-meaningful quantity (leg odometry gives it; world-frame vx
# under an arbitrary yaw does not). Scaled to O(1) so no term dominates the MSE.
EST_DIM = 3 + 2 + 2          # [vx_heading, vy_heading, vz, footz_L, footz_R, contact_L, contact_R]
EST_SCALE = (4.0, 4.0, 4.0, 10.0, 10.0, 1.0, 1.0)


def _make_ac(hid):
    """Asymmetric actor-critic. ACTOR: POLICY_ARCH = mlp | lstm | gru. The recurrent variants give the
    actor MEMORY (hidden state) so it can compensate the deploy CONTROL LATENCY -- a reactive MLP can't
    on a small-margin foot (ablation: LSTM surv~1.0 vs MLP~0.34 under ~6-tick latency). CRITIC: ALWAYS
    an MLP over OBS_DIM+PRIV_DIM (privileged sim-only state -> low-variance value; nothing leaks to the
    deployed actor). act()/act_seq() carry the actor hidden; value() is stateless. [512,256,128] ELU.

    Two opt-in structural upgrades (both default OFF -> byte-identical net + checkpoint compat):
      POLICY_HEAD=mlp  -- post-RNN MLP head [256,128] ELU. The recurrent actor's head is otherwise a
                          BARE LINEAR projection: outside the LSTM gates its whole nonlinear capacity
                          is one tanh, while the critic gets three hidden layers. That asymmetry looks
                          accidental (rsl_rl's ActorCriticRecurrent puts the MLP AFTER the memory).
      EST_HEAD=1       -- state-estimator head (see the EST_DIM note above). Requires a recurrent arch.
    """
    torch = _torch()
    import torch.nn as nn
    arch = os.environ.get("POLICY_ARCH", "mlp"); nlayers = _i("POLICY_LAYERS", 1)
    head = os.environ.get("POLICY_HEAD", "linear")
    est_on = bool(_i("EST_HEAD", 0))
    if est_on and arch not in ("lstm", "gru"):
        # An estimator needs a memory to estimate FROM: a reactive MLP has no history to integrate.
        raise SystemExit("EST_HEAD=1 requires POLICY_ARCH=lstm|gru (got %r): a state estimator "
                         "regresses velocity from the observation HISTORY, which an MLP has none of." % arch)

    def mlp(din, dout, last_gain, widths=(512, 256, 128)):
        layers = []
        d = din
        for w in widths:
            layers += [nn.Linear(d, w), nn.ELU()]; d = w
        layers += [nn.Linear(d, dout)]
        m = nn.Sequential(*layers)
        for layer in m:
            if isinstance(layer, nn.Linear):
                nn.init.orthogonal_(layer.weight, 1.0); nn.init.zeros_(layer.bias)
        nn.init.orthogonal_(m[-1].weight, last_gain)
        return m

    class AC(nn.Module):
        def __init__(s):
            super().__init__()
            s.arch = arch; s.hid = hid; s.nlayers = nlayers; s.est_on = est_on
            if arch in ("lstm", "gru"):
                s.enc = nn.Linear(OBS_DIM, hid)
                s.rnn = (nn.LSTM if arch == "lstm" else nn.GRU)(hid, hid, num_layers=nlayers)
                nn.init.orthogonal_(s.enc.weight, 1.0); nn.init.zeros_(s.enc.bias)
                # the estimate is APPENDED to the policy head's input (Ji et al.): the policy reads
                # velocity explicitly instead of having to re-derive it from the hidden state.
                s.est = nn.Linear(hid, EST_DIM) if est_on else None
                if est_on:
                    nn.init.orthogonal_(s.est.weight, 1.0); nn.init.zeros_(s.est.bias)
                pin = hid + (EST_DIM if est_on else 0)
                if head == "mlp":
                    s.pi = mlp(pin, ACT_DIM, 0.01, widths=(256, 128))
                else:
                    s.pi = nn.Linear(pin, ACT_DIM)
                    nn.init.orthogonal_(s.pi.weight, 0.01); nn.init.zeros_(s.pi.bias)
            else:
                s.est = None
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

        def _pi_in(s, y):
            """Policy-head input: the recurrent feature, plus the DETACHED state estimate when on.
            Detached on purpose -- only the supervised estimator loss shapes s.est, so PPO cannot
            quietly repurpose it into an unconstrained extra feature and destroy the estimate."""
            if s.est is None:
                return y
            return torch.cat([y, s.est(y).detach()], -1)

        def act(s, o, ha=None):
            if s.arch in ("lstm", "gru"):
                y, ha2 = s.rnn(torch.tanh(s.enc(o)).unsqueeze(0), ha)   # (1,K,obs) -> (1,K,hid)
                y = y.squeeze(0)
                return s.pi(s._pi_in(y)), s.std(), ha2
            return s.pi(o), s.std(), None

        def act_seq(s, o_seq, h0, want_est=False):
            """Full T-window in ONE cuDNN call (fast BPTT). o_seq (T,K,obs) -> (mean(T,K,act), std).
            want_est also returns the (T,K,EST_DIM) state estimate for the supervised loss."""
            y, _ = s.rnn(torch.tanh(s.enc(o_seq)), h0)
            mean = s.pi(s._pi_in(y))
            if want_est:
                return mean, s.std(), (s.est(y) if s.est is not None else None)
            return mean, s.std()

        def value(s, o, priv):
            return s.vf(torch.cat([o, priv], -1)).squeeze(-1)
    return AC()


def _detach_h(h):
    if h is None:
        return None
    return (h[0].detach(), h[1].detach()) if isinstance(h, tuple) else h.detach()


def _sanitize_h(h):
    """Scrub NaN/inf out of the recurrent hidden state. THE HIDDEN IS THE NaN CARRIER.

    ⛔ Measured 2026-07-13 (run corrA, it≈400). `mean` is nan_to_num'd on every path, so the ACTIONS
    always look clean -- but the hidden state is carried tick-to-tick and is scrubbed NOWHERE, and it is
    NOT reset when an episode ends. So one world whose physics blows up (NaN qpos) produces a NaN obs,
    which makes THAT WORLD'S HIDDEN STATE NaN *permanently* -- the respawn does not clear it. From then
    on every obs/action it records is NaN, every PPO minibatch containing it yields a NaN loss, and the
    `if not torch.isfinite(loss): continue` guard SILENTLY SKIPS THE UPDATE. Training does not crash. It
    keeps printing iterations while quietly learning nothing, and the eval reports gmatch=nan.

    ⚠️ Related but NOT fixed here: the hidden is never reset at episode boundaries either, so memory
    leaks from a dead episode into its successor. Fixing that properly means masking dones in the BPTT
    replay too (act_seq's single cuDNN call replays a whole T-window from one h0), which changes PPO's
    rollout/replay consistency -- a real change that deserves its own controlled run, not a drive-by.
    """
    import torch as _t
    if h is None:
        return None
    if isinstance(h, tuple):
        return (_t.nan_to_num(h[0], nan=0.0, posinf=0.0, neginf=0.0),
                _t.nan_to_num(h[1], nan=0.0, posinf=0.0, neginf=0.0))
    return _t.nan_to_num(h, nan=0.0, posinf=0.0, neginf=0.0)


def _seed_legs(n_leg=12):
    """The seeded leg pose (spawn + nominal anchor). STAND_POSE=unitree -> the OFFICIAL model's
    DEFAULT (hip -0.1, knee 0.3, ankle -0.2): the posture the RECORDED ghost cycles around.
    Anchoring the policy's nominal to the ghost's nominal removes the ~0.2 rad persistent
    posture offset that floored gmatch (the mimicry metric) regardless of reward weight.
    Default = the classic deep crouch (hip -0.30, knee 0.52) that bootstrapped the stand.

    `n_leg` is the ROBOT's leg-DOF count (from the compiled model), not a constant: the
    built-in poses below are 12-slot G1 postures in the G1's slot order, and they are
    meaningless on a robot with a different leg (the H1 has 10 leg DOF, no ankle roll, and
    a different slot order). For a non-G1 robot, pass the pose explicitly via
    STAND_POSE_LEGS -- or get None back and let the model keep its own spawn pose, which
    is the honest default. Never silently reshape a G1 crouch onto another skeleton.
    """
    if os.environ.get("STAND_POSE_LEGS"):
        # explicit seed (2026-07-11, seq-ghost entry-state parity): the batched seq
        # eval judges "from the TOP" with robots IN the ghost's bin-0 pose; the live launch
        # must be able to start there too or the LSTM opens in a state the routine never has.
        v = np.array([float(x) for x in os.environ["STAND_POSE_LEGS"].split(",")], np.float32)
        if v.size != n_leg:
            raise RuntimeError(f"STAND_POSE_LEGS has {v.size} values but the robot has "
                               f"{n_leg} leg DOF")
        return v
    if n_leg != 12:
        print(f"[recipe] STAND_SEED: no built-in stand pose for a {n_leg}-DOF leg "
              f"(the built-ins are G1 12-slot postures) -- keeping the model's own spawn "
              f"pose. Set STAND_POSE_LEGS to seed one explicitly.")
        return None
    if os.environ.get("STAND_POSE", "") == "unitree":
        return np.array([-0.10, 0.0, 0.0, 0.30, -0.20, 0.0,
                         -0.10, 0.0, 0.0, 0.30, -0.20, 0.0], np.float32)
    return np.array([-0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
                     -0.30, 0.0, 0.0, 0.52, -0.23, 0.0], np.float32)


# ── THE ROBOT SEAM (2026-07-12) ─────────────────────────────────────────────
# The corridor used to be hardcoded to the G1: `np.full(12, ...)`, roll slots
# [1,5,7,11], hip-yaw slots [2,8], a stance mask `[1,0,1,1,1,0]*2`, and
# `.expand(-1, 6)` -- i.e. 12 leg DOF, 6 per leg, in the G1's slot order. The H1
# has 10 leg DOF, 5 per leg, and a DIFFERENT order, so none of those literals
# survive contact with it.
#
# They do not need to be literals. Every one of them is a statement about the
# ROLE of a joint, and the role is in its NAME:
#     roll joints  -> the lateral stabilizers   (widened by GHOST_RESIDUAL_LAT)
#     hip-yaw      -> the turning authority     (widened by GHOST_RESIDUAL_YAW)
#     everything else = the pitch plane         (the stance-gated, visually
#                                                defining part of the gait)
# So derive the roles from the ghost's own joint names and the recipe becomes
# robot-agnostic for free. On the G1 this reproduces the old literals EXACTLY
# (asserted at import, below) -- so this is a generalization with zero behaviour
# change, not a retune.
G1_LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]


# The robot this deploy is driving, as DECLARED BY ITS GHOST (set when the lut is read).
# A one-element list because the recipe's helpers are module-level and the lut is read deep
# inside the env builder.
_GHOST_ROBOT = ["g1"]


def _ghost_robot(world=None):
    return _GHOST_ROBOT[0]


def _leg_roles(names):
    """Corridor roles from the ghost's joint NAMES -> (n, roll_idx, yaw_idx, pitch_mask, per_leg).

    pitch_mask is 1.0 on the pitch-plane joints and 0.0 on the roll stabilizers:
    the stance gate tightens the pitch plane during stance (no room to yank a foot
    into a catch-step) but must NEVER gate the rolls, because lateral balance is
    applied by the PLANTED leg.

    The law itself lives in the shared robot registry (common/robot_registry.py) so that the
    trainer's corridor and the validator's symmetry gate cannot disagree about what a joint IS.
    The G1 literals it replaces are still asserted below.
    """
    return RR.leg_roles(names)


# The generalization must not move the G1. Prove it at import: the derived roles
# must equal the literals they replace, or this file is lying about its own history.
_g1n, _g1roll, _g1yaw, _g1mask, _g1per = _leg_roles(G1_LEG_JOINTS)
assert (_g1n, _g1roll, _g1yaw, _g1per) == (12, [1, 5, 7, 11], [2, 8], 6), \
    f"G1 corridor roles regressed: {(_g1n, _g1roll, _g1yaw, _g1per)}"
assert _g1mask.tolist() == [1., 0., 1., 1., 1., 0.] * 2, \
    f"G1 stance mask regressed: {_g1mask.tolist()}"


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
    est_c = _f("EST_COEF", 1.0)   # weight of the supervised state-estimator loss (EST_HEAD=1)
    fr = _f("FALL_ROLL", 0.8); fp = _f("FALL_PITCH", 0.8); fbz = _f("FALL_BZ", 0.45)
    # reward weights (recipe)
    wtl = _f("W_TRACK_LIN", 1.5); wta = _f("W_TRACK_ANG", 0.5); vtsig = _f("VTRACK_SIG", 0.25)
    # Sequence turns need a signed angular-velocity objective in addition to pose/heading match.
    # Foot/joint imitation can score highly while ground slip rotates the pelvis the wrong way;
    # matching the ghost's signed wz makes that failure directly expensive.
    w_seqwz = _f("W_SEQ_YAWRATE", 0.0); seqwz_sig = _f("SEQ_YAWRATE_SIG", 0.18)
    seqwz_qvel_sign = _f("SEQ_YAWRATE_QVEL_SIGN", 1.0)
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
    # W_KNEE_LOW (2026-07-11, motion-legitimacy doctrine): penalize knee-support postures.
    # The stair champion levered itself up on its knees (verified: knee-stair contact on
    # 13.6% of climb ticks) -- a cheap CUDA-graph-safe proxy is the knee dropping within
    # KNEE_CLEAR of its own-side foot height (a human climb keeps ~0.25-0.35 m). Shaping
    # only; verify_motion_legitimacy.py remains the gate.
    w_kneelow = _f("W_KNEE_LOW", 0.0)
    # W_CRANE: dense penalty on the harness wrench the policy consumes -- the GRADIENT that makes the
    # HARNESS_GRAD_SUS gate reachable instead of merely blocking. Default 0 = old behaviour exactly.
    w_crane = _f("W_CRANE", 0.0)
    kneeL_bid = kneeR_bid = -1
    if w_kneelow > 0:
        try:
            _mjmK = world.solver.mj_model
            for _jK in range(int(_mjmK.njnt)):
                if int(_mjmK.jnt_qposadr[_jK]) == int(leg_qadr[3]):
                    kneeL_bid = int(_mjmK.jnt_bodyid[_jK])
                if int(_mjmK.jnt_qposadr[_jK]) == int(leg_qadr[9]):
                    kneeR_bid = int(_mjmK.jnt_bodyid[_jK])
            R._log(world, "W_KNEE_LOW=%.2f knee bodies L=%d R=%d clear=%.2f"
                   % (w_kneelow, kneeL_bid, kneeR_bid, _f("KNEE_CLEAR", 0.15)))
        except Exception as _eK:
            w_kneelow = 0.0
            R._log(world, "W_KNEE_LOW disabled: knee-body resolve failed %r" % (_eK,))
    if kneeL_bid < 0 or kneeR_bid < 0:
        w_kneelow = 0.0

    spawn = world.solver.mjw_data.qpos.numpy().reshape(-1)[:nq].copy()
    spawn[0:3] = [0.0, 0.0, float(spawn[2])]; spawn[3:7] = [1, 0, 0, 0]
    # STAND_SEED: force a KNOWN-GOOD crouched-stand pose (G1 nominal: hip -0.30, knee 0.52,
    # ankle -0.23) as the reset pose, independent of the deterministic controller. On small feet the
    # det. stand collapses during warmup -> RL inherited a falling pose (surv=0, backward eject). A
    # forced upright seed gives RL a survivable start so it can LEARN active balance (real-robot style).
    if _i("STAND_SEED", 0):
        # length from the COMPILED MODEL, not a constant: 12 on the G1, 10 on the H1.
        stand_leg = _seed_legs(len(leg_qadr))
        if stand_leg is not None:
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
    ghost_ffdq_t = None
    leg_names = list(G1_LEG_JOINTS)     # robot default; a self-describing lut overrides it
    # ONE env contract for the method (common/shadow_env.py): SHADOW_GHOST is canonical,
    # GHOST_LUT_JSON (humanoid) and QUAD_GHOST (quadruped) are accepted aliases of the same knob.
    _glj = SE.ghost()
    if _glj:
        import json as _json
        _gd = _json.loads(open(_glj).read())
        _lut = np.asarray(_gd["leg_lut"], np.float32)          # (nb, n_leg) slot order L+R
        NB = int(_gd["nb"])
        # ── the robot seam ───────────────────────────────────────────────────
        # A self-describing lut (H1 and every future humanoid) carries its own
        # joint names AND its own count. The G1 luts predate the convention and
        # are the 12-slot default. Everything downstream that used to hardcode
        # "12", "6 per leg", and the G1's slot indices now derives from THIS list
        # -- see _leg_roles(). An H1 lut is 10-wide with a DIFFERENT slot order
        # (yaw,roll,pitch,knee,ankle vs the G1's pitch,roll,yaw,knee,ankP,ankR),
        # so hardcoded indices are wrong for it in more ways than just the count.
        if "joints" in _gd:
            leg_names = [str(j) for j in _gd["joints"]]
        # WHICH ROBOT is this ghost for? The lut says so (schema 2). It decides the BATON
        # support gate (biped: hand over only at double support) and it is what the validator
        # limit-checks against. G1 luts that predate the key default to g1.
        _GHOST_ROBOT[0] = str(_gd.get("robot") or "g1").strip().lower()
        if _lut.shape[1] != len(leg_names):
            raise RuntimeError(
                f"GHOST_LUT_JSON leg_lut is {_lut.shape[1]}-wide but declares "
                f"{len(leg_names)} joints ({os.path.basename(_glj)}). The quads assert this "
                f"(quad_walk_recipe:_ghost_joint_order); the humanoid path used to mis-map in "
                f"silence.")
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
        # GHOST_FF (2026-07-10, the corridor-vs-torque law): the corridor clamps the policy's
        # position-target offsets to +-GHOST_RESIDUAL around the reference, but the torque the
        # reference ITSELF requires costs dq = tau_ff/kp = up to 0.23 rad at the knee (walk AND
        # stairs) -- more than any corridor we ever trained with. The policy could never hold its
        # own stance torque inside the corridor, and the crane absorbed the deficit (the 7 cm climb
        # was impossible by construction at corridor 0.12; gate-4 funnel = 1.000 WITH feedforward).
        # Fix: the ghost declares its feedforward (ffdq_lut, ghost_ff.py) and the corridor CENTRE
        # shifts by it -- q_cmd = q_ref + ffdq + a*corridor. The REWARD stays on q_ref: gmatch
        # scores the pose, not the command. Opt-in: GHOST_FF=1 + a lut that carries ffdq_lut.
        ghost_ffdq_t = None
        if SE.ff() and "ffdq_lut" in _gd:
            _ffdq = np.asarray(_gd["ffdq_lut"], np.float32)
            ghost_ffdq_t = torch.tensor(_ffdq, device=tdev)
            R._log(world, "GHOST-FF: corridor centre += tau_ff/kp (ffdq peak %.3f rad, kp=%s)"
                   % (float(np.abs(_ffdq).max()), _gd.get("ffdq_kp")))
        elif SE.ff():
            R._log(world, "GHOST-FF requested but the lut has no ffdq_lut -- run ghost_ff.py first")
        if "kp_lut" in _gd:   # (nb, n_kp*3) body-frame keypoints [L_foot,R_foot,L_hand,R_hand]
            ref_kp_t = torch.tensor(np.asarray(_gd["kp_lut"], np.float32), device=tdev)
            # Newton renames bodies (body_5..) so name lookup fails; body ids are FK-MATCHED at it=0
            # (set the sim to the reference pose, find the sim body nearest each reference keypoint).
        omega_lut = 2.0 * math.pi * float(_gd.get("freq", 1.25))
        # GHOST_OMEGA_SCALE (2026-07-10, the riser-refusal fix): CONTACT_LOG measured the live
        # swing landing ~6 cm SHORT of the batched landing (live top-contacts pinned at the first
        # 1 cm of the tread; batched median lands 6 cm in) -- the live gait can't cover the
        # reference stride (sway-lag propulsion deficit). Scaling the clock DOWN shortens the
        # commanded stride-per-cycle so the live foot lands ON the tread. Default 1.0 = unchanged.
        omega_lut *= _f("GHOST_OMEGA_SCALE", 1.0)
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
    # W_PROG (2026-07-09, stairsynth6 post-mortem): under PROGRESS-LOCK the leash makes the phase a
    # PROGRESS METER -- and nothing pays for it. With the reference waiting, the frozen bin's cmd is
    # ~zero, the height target waits too, and "stand at the base" scores surv=1.0 vtrack=0.98 forever
    # (measured: dist=0.00 at it=25 and no climb in 600 iters). Pure clock races away (the shuffle);
    # the leash waits (no drive). This term closes the loop: reward = W_PROG per BIN of leashed phase
    # advance per tick, so the ONLY way to earn it is to physically move the base up the climb.
    _wprog = _f("W_PROG", 0.0) if (_leashlead > 0.0) else 0.0
    # ── CONTACT-EVENT CLOCK (2026-07-10, the x-proxy result matrix's conclusion) ──────────────────
    # Base-x is the wrong proxy for seq progress: a bidirectional x-cap REWINDS the reference when the
    # live base oscillates during a weight transfer (the yo-yo: STEPS L [0,1,0,1,...]); an x-RATCHET
    # cannot re-present a missed swing and freezes (heading 10 deg, L [0,1,2] clean, R stuck at 0).
    # The reference must advance on the robot's own COMPLETED FOOTFALLS. Gates are derived from the
    # ghost's OWN declared contact_schedule: every swing->bearing transition is a touchdown gate; the
    # phase runs at omega but HOLDS AT THE GATE POSE until the gated foot lands near the ghost's foot
    # height there. The hold pose (swing foot hovering on its foothold) IS the retry mechanism.
    # Holding is a min against a FIXED gate phase -> no rewind by construction. SEQ_EVENT_CLOCK=1.
    _evclk = None
    if seq_terrain and _i("SEQ_EVENT_CLOCK", 0) and all(k in _gd for k in ("contact_schedule", "kp_lut", "root_lut")):
        _sch5 = _gd["contact_schedule"]
        _kp5 = np.asarray(_gd["kp_lut"], np.float32)
        _rl5 = np.asarray(_gd["root_lut"], np.float32)
        _g5l = []
        for _fi5, _nm5 in ((0, "foot_L"), (1, "foot_R")):
            for _b5 in range(1, len(_sch5)):
                if (_nm5 in _sch5[_b5]) and (_nm5 not in _sch5[_b5 - 1]):
                    _g5l.append((_b5, _fi5, float(_rl5[_b5, 2] + _kp5[_b5, _fi5 * 3 + 2])))
        _g5l.sort()
        if _g5l:
            _evclk = dict(
                bins=torch.tensor([g[0] for g in _g5l], dtype=torch.long, device=tdev),
                foot=torch.tensor([g[1] for g in _g5l], dtype=torch.long, device=tdev),
                z=torch.tensor([g[2] for g in _g5l], dtype=torch.float32, device=tdev),
                ph=torch.tensor([g[0] / NB * 2.0 * math.pi for g in _g5l], dtype=torch.float32, device=tdev),
                G=len(_g5l))
            R._log(world, "SEQ-EVENT-CLOCK: %d touchdown gates from contact_schedule (first %s)"
                   % (len(_g5l), _g5l[:3]))
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
    payload_kg = _f("CARRY_PAYLOAD_KG", 0.0)
    # W_ATTGHOST: track the (metric) ghost's base ATTITUDE cycle -- the calm-sway fine-tune lever.
    # Complements W_UP (level pull): this rewards matching the ghost's phase-locked sway exactly,
    # which is what the WBMATCH attitude component measures. 0 = off (exact legacy reward).
    w_attg = _f("W_ATTGHOST", 0.0); attg_sig = _f("ATTGHOST_SIG", 0.15)
    arm_qadr = []; arm_sign = []; elb_qadr = []; elb_sign = []
    # Hand-link discovery is also a PHYSICS precondition for payload injection;
    # it must not disappear merely because the separate arm-ghost reward is off.
    if whole and (w_armg > 0 or payload_kg > 0):
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
            if len(arm_qadr) != 2 or len(elb_qadr) != 2:
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
    gres = SE.residual(0.0)          # SHADOW_RESIDUAL | GHOST_RESIDUAL | QUAD_RES_SCALE
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
        # ROLE-DERIVED, not slot-hardcoded (see _leg_roles): on the G1 this IS
        # np.full(12, gres) with [1,5,7,11] widened -- asserted identical at import.
        _nleg, _roll_i, _yaw_i, _pitch_mask, _per_leg = _leg_roles(leg_names)
        _wv = np.full(_nleg, gres, np.float32)
        _wv[_roll_i] = _glat
        # GHOST_RESIDUAL_YAW: hip-yaw slack. Turning to a commanded heading needs
        # yaw authority the straight-walk corridor doesn't grant (nav1 verdict: the policy
        # physically COULDN'T turn much inside +-0.10 rad of the straight ghost's hip yaw).
        _gyaw = _f("GHOST_RESIDUAL_YAW", gres)
        _wv[_yaw_i] = _gyaw
        gres_w = torch.tensor(_wv, dtype=torch.float32, device=tdev)
    # corridor CENTRE: reference + declared feedforward (GHOST_FF). Precomputed once -- zero tick cost.
    # The reward keeps scoring against ghost_leg_t (the POSE); only the command centre shifts.
    ghost_legc_t = ghost_leg_t if ghost_ffdq_t is None else (ghost_leg_t + ghost_ffdq_t)
    # PHASE-GATED corridor width (round 8: catch-steps are contacts at the WRONG PHASE, and
    # this channel resisted reward -- so corridor the contact timing). During a leg's STANCE
    # window the PITCH-plane corridor tightens to GHOST_STANCE_TIGHT x base (no room to yank
    # the foot into a catch-step); full width returns through swing. ROLL joints stay full
    # width always: lateral balance is applied by the PLANTED leg (gating rolls in stance
    # would re-run the falsified width-snap).
    stance_tight = _f("GHOST_STANCE_TIGHT", 0.0)
    # ROLE-DERIVED stance mask: 1.0 on the pitch plane, 0.0 on the roll stabilizers.
    # On the G1 this IS [1,0,1,1,1,0]*2 -- asserted identical at import (_leg_roles).
    _sgn, _, _, _sg_np, _sg_per = _leg_roles(leg_names)
    _sg_mask = torch.tensor(_sg_np, dtype=torch.float32, device=tdev)

    def _gres_gated(_gb):
        if not (0.0 < stance_tight < 1.0) or lut_swing is None:
            return gres_w
        _sL = (stance_tight + (1.0 - stance_tight) * lut_swing[0][_gb]).unsqueeze(1).expand(-1, _sg_per)
        _sR = (stance_tight + (1.0 - stance_tight) * lut_swing[1][_gb]).unsqueeze(1).expand(-1, _sg_per)
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
        _shry_act = []; _shry_sign = []; _shry_tgts = []
        _shry_yawt = _f("SHRY_YAW_TARGET", 0.0)
        _trn5 = np.asarray(world.solver.mj_model.actuator_trnid).reshape(int(world.solver.mj_model.nu), -1)
        import mujoco as _mjn5
        for a5 in act_use:
            jid5 = int(_trn5[int(a5)][0])
            nm5 = (_mjn5.mj_id2name(world.solver.mj_model, _mjn5.mjtObj.mjOBJ_JOINT, jid5) or "")
            if "shoulder_roll" in nm5 or "shoulder_yaw" in nm5:
                _shry_act.append(int(a5))
                _sd5 = 1.0 if "left" in nm5.lower() else -1.0
                _shry_sign.append(_sd5)
                _shry_tgts.append(_sd5 * (shry_tgt_f if "shoulder_roll" in nm5 else _shry_yawt))
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
    _pk = payload_kg
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
            raise RuntimeError("required CARRY_PAYLOAD_KG setup failed") from _eC
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
    evgi_t = torch.zeros(K, dtype=torch.long, device=tdev) if _evclk is not None else None  # next gate per env
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
    # SEQUENCE HEADING RANDOMIZATION: a solved turn ghost is authored in a local 0->theta
    # frame, but the same footwork must work at every world heading.  Earlier turners only
    # ever saw the ghost's absolute yaw and consequently failed once a BATON course entered
    # a second corner outside the narrow 0..90deg training band.  Keep a per-environment
    # rigid yaw offset and apply it consistently to RSI pose/velocity and to the live target
    # frame.  The joint/reference tables remain body-relative and need no rotation.
    _seq_yaw0_rand = _f("SEQ_YAW0_RAND", 0.0) if seq_mode else 0.0
    seq_yaw0_t = torch.zeros(K, device=tdev)
    if _seq_yaw0_rand > 0.0:
        R._log(world, "SEQ-YAW0-RAND: routine frame uniformly rotated +/-%.1f deg per episode"
               % (_seq_yaw0_rand * 57.2958))
    # ARC CURRICULUM (the TURN specialist): per-episode yaw RATE -- the heading target ROTATES
    # at wz each tick, so the policy learns CONTINUOUS turning (arc walking), not just settling
    # onto a fixed heading. Half the episodes draw wz=0: the straight walk stays in-distribution.
    wz_t = torch.zeros(K, device=tdev)
    _wzr = _f("YAW_RATE_RAND", 0.0)
    # YAW-DR (2026-07-10): per-episode CONSTANT pelvis yaw torque (N*m, uniform +/-YAW_DR_TORQUE).
    # The LIVE plant applies a deterministic yaw moment the batched plant lacks (pure-PD puppet
    # veers -0.76 rad by x~1.0, bit-repeatable run-to-run; policies overcorrect to +0.6). A policy
    # that nulls a random constant tau_z via the heading obs also nulls the live veer -- robustness
    # in place of the engine-level contact-parity fix. Trainer-only: deploy_eval zeroes xfrc.
    dr_tz_t = torch.zeros(K, device=tdev)
    _drtz = _f("YAW_DR_TORQUE", 0.0)
    # SWAY-PHASE DR (see _harness_apply): per-episode sway-spring index offset in [-RAND, 0] bins.
    attoff_t = torch.zeros(K, dtype=torch.long, device=tdev)
    _attpr = _f("HARNESS_ATT_PHASE_RAND", 0.0)

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
            if _seq_yaw0_rand > 0.0:
                _yo = (torch.rand(K, device=tdev) * 2.0 - 1.0) * _seq_yaw0_rand
                seq_yaw0_t.copy_(torch.where(mask, _yo, seq_yaw0_t))
            # SEQ_BIN_LO/HI (segment-isolation mode, owner ask 2026-07-04): confine RSI (and the
            # episode window, see the rollout) to bins [LO, HI) so ONE segment trains alone --
            # per-segment mastery separates "this segment is infeasible" from "the transitions
            # break" in a way the shared-policy feasibility map cannot.
            if seq_win:
                b0 = torch.randint(seq_lo, max(seq_lo + 1, seq_hi - 4), (K,), device=tdev)
            else:
                b0 = torch.randint(0, NB, (K,), device=tdev)
            rq[:, leg_qadr_t] = ghost_leg_t[b0] + torch.randn(K, 12, device=tdev) * icj
            # +2 cm RSI clearance -- the SAME teleport-bounce trap the initial spawn guards against
            # (see spawn[2] above: exact contact depth on the warm solver explodes in ~6 ticks). It
            # went unnoticed for every ghost before ghost_synth because their "planted" feet FLOATED
            # ~10 mm -- the gate-1 defect doubled as accidental reset clearance. A kinematically
            # CLOSED ghost has soles flush to <1 mm, so exact-depth RSI teleports blew envs up from
            # it~100 (epret spike -> NaN storm -> neps x10, eval metrics nan, fwd<0).
            rq[:, 2] = seq_rootz_t[b0] + 0.02
            if seq_terrain:
                rq[:, 0] = seq_rootx_t[b0]   # respawn ON the terrain (x with z), not floating at x=0
            _y0 = seq_yaw_t[b0] + seq_yaw0_t
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
        if _drtz > 0:   # YAW-DR: fresh constant disturbance torque per episode
            dr_tz_t.copy_(torch.where(mask, (torch.rand(K, device=tdev) * 2 - 1) * _drtz, dr_tz_t))
        if _attpr > 0:  # SWAY-PHASE DR: fresh sway-spring lag per episode, [-RAND, 0] bins
            attoff_t.copy_(torch.where(mask, -(torch.rand(K, device=tdev) * _attpr).long(), attoff_t))
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

    _est_scale_t = torch.tensor(EST_SCALE, device=tdev)

    def est_of(yaw):
        """Supervised target for the actor's ESTIMATOR HEAD (EST_HEAD=1) -- the same 7 quantities the
        critic gets privileged, but expressed in the HEADING frame and scaled to O(1). Heading frame
        because that is the proprioceptively-inferable quantity (leg odometry gives you forward/lateral
        velocity; world-frame vx under an arbitrary yaw it cannot). vy_heading is precisely what
        HARNESS_FY damps for the policy today -- the head is what lets the policy damp it itself."""
        qv = qvel_t
        c = torch.cos(yaw); s = torch.sin(yaw)
        vxh = c * qv[:, 0] + s * qv[:, 1]
        vyh = -s * qv[:, 0] + c * qv[:, 1]
        zL = xpos_t[:, fL, 2] - z_stance; zR = xpos_t[:, fRi, 2] - z_stance
        cL = (zL < 0.03).float(); cR = (zR < 0.03).float()
        return torch.stack([vxh, vyh, qv[:, 2], zL, zR, cL, cR], 1) * _est_scale_t

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
        if _f("HARNESS_GRAD_SURV", 0.9) >= 1.0:
            # MOTION-LEGITIMACY doctrine (docs/developer/motion-legitimacy.md, owner 2026-07-11):
            # an unreachable graduation bar means the crane NEVER weans -- PPO then learns to
            # LEAN ON THE SPRINGS (measured: the stair champion rode 77.5 N*m of sustained pitch
            # torque up the staircase while passing every kinematic gate). Legal for puppet-stage
            # experiments; NEVER for a finished skill.
            R._log(world, "!! HARNESS_GRAD_SURV=%.2f >= 1.0: the crane will NEVER graduate -- the "
                          "policy CAN and WILL learn to lean on it. Gate any champion on "
                          "verify_motion_legitimacy.py (docs/developer/motion-legitimacy.md)"
                   % _f("HARNESS_GRAD_SURV", 0.9))
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
            # ⛔ BOUND BY THE TABLE'S OWN LENGTH, not by NBM. NBM is the METRIC ghost's bin count and
            # it is initialised to 0 (line ~897), set only when GHOST_METRIC_JSON is passed. This line
            # used NBM unguarded -- so HARNESS_ATT_GHOST=1 WITHOUT a metric ghost did `% 0` inside a
            # CUDA kernel: integer modulo-by-zero -> device-side assert -> the trainer dies on the
            # first rollout with an async CUDA error whose traceback points at an innocent line.
            # (The sibling uses at 2008/2638 are correctly guarded by `ghost_leg_M is not None`; the
            # deploy mirror indexes world._wr_refnb, which is why the shipped demo works and only
            # TRAINING on a ghost with no metric ghost blew up. It went unnoticed because the
            # flagship's training always passed GHOST_METRIC_JSON.)
            # ghost_att_t is loaded from the CONTROL ghost and OVERRIDDEN by the metric ghost when one
            # is given, so its own row count is the only bound that is right in both cases.
            _nbH = int(ghost_att_t.shape[0])
            _gbH = (phase_t / twopi * _nbH).long() % _nbH
            if _attpr > 0:
                # SWAY-PHASE DR (2026-07-10, the measured live disturbance): live the gait LAGS the
                # ghost clock, so the sway spring runs 0..~8 LUT bins out of phase and precesses the
                # robot (the veer / lateral slip that makes every straight climber toe the tread
                # corner). Randomizing the sway-spring INDEX per episode over [-RAND, 0] makes the
                # batched robot train INSIDE the live regime -- it must keep stride and heading
                # under exactly this disturbance shape (the constant-torque YAW_DR was the wrong
                # shape; this is the right one, measured).
                _gbH = (_gbH + attoff_t) % NBM
            _rH = _r - ghost_att_t[_gbH, 0]; _pH = _p - ghost_att_t[_gbH, 1]
        else:
            _rH = _r; _pH = _p
        # ── THE ATTITUDE SPRING: SAME FRAME BUG AS THE DEPLOY, NOW THE SAME FIX ──────────────
        # (2026-07-12.) This wrote a BODY-frame PD straight into a WORLD torque channel, exactly
        # as the deploy spring did (see the long note at the deploy mirror). The three frames:
        #   _rH / _pH          -- BODY roll/pitch error (heading-invariant)
        #   qvel_t[:, 3:5]     -- the free joint's angular velocity, in the BODY frame
        #   xfrc_t[..., 3:5]   -- a CARTESIAN WORLD wrench (Newton routes a free joint's force to
        #                         MuJoCo's xfrc_applied, not qfrc_applied)
        # so the PD must be computed in BODY and its output ROTATED INTO WORLD.
        #
        # WHY IT WAS INVISIBLE HERE: at heading 0 the rotation is the identity (cos=1, sin=0), and
        # straight-walk training lives at yaw~0 -- so every walk champion trained under a spring
        # that was, for them, exactly right. The TURN specialist does not: it sweeps 0->90 deg, and
        # trained under a progressively cross-wired spring (at 90 deg the ROLL correction is applied
        # as a PITCH torque, from a 600 N*m/rad, 350 N*m-capped spring).
        #
        # Fixing this restores train==deploy parity, which is this repo's core rule and which the
        # deploy-side fix had just broken. It is a NO-OP for yaw~0 training (asserted below in the
        # commit's verification), so no walk champion's training distribution moves.
        # ANGVEL_WORLD_FRAME=1 restores the old (incorrect) world-rate model for A/B, as in deploy.
        _wqT, _xqT, _yqT, _zqT = qpos_t[:, 3], qpos_t[:, 4], qpos_t[:, 5], qpos_t[:, 6]
        _ybT = torch.atan2(2 * (_wqT * _zqT + _xqT * _yqT), 1 - 2 * (_yqT * _yqT + _zqT * _zqT))
        _cbT, _sbT = torch.cos(_ybT), torch.sin(_ybT)
        if _i("ANGVEL_WORLD_FRAME", 0):
            _wxbT = _cbT * qvel_t[:, 3] + _sbT * qvel_t[:, 4]   # the old, false world->body rotation
            _wybT = -_sbT * qvel_t[:, 3] + _cbT * qvel_t[:, 4]
        else:
            _wxbT = qvel_t[:, 3]                                 # already body-frame: use as-is
            _wybT = qvel_t[:, 4]
        _txbT = -_hkp * _rH - _hkd * _wxbT                       # body-frame PD
        _tybT = -_hkp * _pH - _hkd * _wybT
        xfrc_t[:, pelvis_bid, 3] = _alv9 * (harness_lam * (_cbT * _txbT - _sbT * _tybT)).clamp(-_TCAP, _TCAP)
        xfrc_t[:, pelvis_bid, 4] = _alv9 * (harness_lam * (_sbT * _txbT + _cbT * _tybT)).clamp(-_TCAP, _TCAP)
        if _drtz > 0:   # YAW-DR disturbance (not harness assistance -- an adversary the policy must null)
            xfrc_t[:, pelvis_bid, 5] = _alv9 * dr_tz_t
        _hkyT = _f("HARNESS_KYAW", 0.0)
        if _hkyT > 0:
            # TRAINER YAW-TRIM (2026-07-10): mirror of the deploy walk-trim crane (gentle heading
            # torque, error capped at HARNESS_YAW_CAP). The chest-forward stair campaign found the
            # champion's learned climb strategy IS the yaw twist -- deploying it with the trim held
            # stops it dead at the stair base. Training under the SAME trim (parity by config)
            # forces the policy to learn a straight step-up: the crane holds the chest, the legs
            # must find the climb.
            _wq, _xq, _yq, _zq = qpos_t[:, 3], qpos_t[:, 4], qpos_t[:, 5], qpos_t[:, 6]
            _yawT = torch.atan2(2 * (_wq * _zq + _xq * _yq), 1 - 2 * (_yq * _yq + _zq * _zq))
            _yeT = torch.atan2(torch.sin(_yawT - ytgt_t), torch.cos(_yawT - ytgt_t))
            _yeT = _yeT.clamp(-_f("HARNESS_YAW_CAP", 0.12), _f("HARNESS_YAW_CAP", 0.12))
            xfrc_t[:, pelvis_bid, 5] += _alv9 * (harness_lam * (-_hkyT * _yeT
                                                 - _f("HARNESS_KYD", 30.0) * qvel_t[:, 5])).clamp(-_TCAP, _TCAP)

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
        if evgi_t is not None:
            evgi_t.zero_()   # event clock: evals start at bin 0, first gate pending
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
        _ew0, _ex0, _ey0, _ez0 = (qpos_t[:, i].clone() for i in (3, 4, 5, 6))
        _yaw0_eval = torch.atan2(2 * (_ew0 * _ez0 + _ex0 * _ey0),
                                  1 - 2 * (_ey0 * _ey0 + _ez0 * _ez0))
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
        # ── CRANE LOAD (2026-07-13): how hard is the policy LEANING on the harness, in newtons? ──
        # The ladder's anti-lean gate used to read world._wr_last_att, a POSTURE proxy that is only
        # ever assigned inside `if use_armg:` -- so for any ghost without an arm-ghost eval it was
        # never set, and the gate read it as getattr(..., 1.0), i.e. it PASSED UNCONDITIONALLY. That
        # is exactly the hole run p2e fell through (leaned on the springs, rode the ladder to lam=0.3,
        # froze at surv 0.478). Measure the thing itself: the wrench the crane is actually applying,
        # by the same criterion verify_motion_legitimacy.py uses post-hoc (L1: sustained-torque
        # fraction above TAU_SUS, mean |fy|). A policy that has earned the next rung is barely
        # touching these.
        cr_tau = torch.zeros((), device=tdev); cr_sus = torch.zeros((), device=tdev)
        cr_fy = torch.zeros((), device=tdev); cr_fz = torch.zeros((), device=tdev)
        cr_n = torch.zeros((), device=tdev)
        _TAU_SUS = _f("HARNESS_TAU_SUS", 40.0)   # N*m -- matches verify_motion_legitimacy.TAU_SUS
        # ── ACTION AUTHORITY (2026-07-13): is the policy PINNED AGAINST THE CORRIDOR WALL? ──
        # The residual is clamped to +-1 and scaled by GHOST_RESIDUAL, so |a|->1 means the policy is
        # asking for more correction than the corridor will grant. That distinguishes the two stories
        # behind a rising crane load: EXPLOITING the crane (lean is cheap -> price it with W_CRANE) vs
        # STARVED of authority (it wants to catch itself and cannot -> widen the corridor). Without
        # this you are guessing.
        act_sat = torch.zeros((), device=tdev); act_mag = torch.zeros((), device=tdev)
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
        # CONTACT_LOG (2026-07-10, the riser-refusal hunt): dump world-0 FOOT contacts per tick
        # in the SAME format as the live deploy's CONTACTLOG so the two contact streams diff
        # directly (geoms, penetration, position, normal, base x, phase). Batched side of the
        # live-vs-batched closed-loop instrumentation; EVAL_ONLY=1 CONTACT_LOG=1 to record.
        _clog_ev = _i("CONTACT_LOG", 0)
        if _clog_ev:
            _mjmC = world.solver.mj_model
            _cgC = {g for g in range(int(_mjmC.ngeom))
                    if int(_mjmC.geom_bodyid[g]) in (int(fL), int(fRi))}
            _allC = _i("CONTACT_LOG_ALL", 0)
            try:   # geom->body map for the legitimacy verifier (classify knee/shin/hand contacts)
                import mujoco as _mjnC
                for gC in range(int(_mjmC.ngeom)):
                    _bnC = _mjnC.mj_id2name(_mjmC, _mjnC.mjtObj.mjOBJ_BODY, int(_mjmC.geom_bodyid[gC])) or "?"
                    R._log(world, "CONTACTMAP g=%d body=%s" % (gC, _bnC))
            except Exception:
                pass
        for t in range(eval_H):
            if _clog_ev and t % 20 == 0:
                # EVALYAW: world-0 base yaw/x/y -- the live pure-PD veer (-0.76 rad by x~1.0) was
                # only ever compared against ydrift (METERS); this makes batched YAW comparable.
                _wq0, _xq0, _yq0, _zq0 = (float(qpos_t[0, i]) for i in (3, 4, 5, 6))
                _yawE = math.atan2(2 * (_wq0 * _zq0 + _xq0 * _yq0), 1 - 2 * (_yq0 * _yq0 + _zq0 * _zq0))
                R._log(world, "EVALYAW t=%d x=%.3f y=%.3f yaw=%.3f" % (t, float(qpos_t[0, 0]), float(qpos_t[0, 1]), _yawE))
                if _i("CRANE_LOG", 0) and xfrc_t is not None:
                    _wf0 = xfrc_t[0, pelvis_bid]
                    R._log(world, "CRANELOG side=BATCH t=%d fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f bx=%.3f bz=%.3f"
                           % (t, float(_wf0[0]), float(_wf0[1]), float(_wf0[2]), float(_wf0[3]), float(_wf0[4]), float(_wf0[5]),
                              float(qpos_t[0, 0]), float(qpos_t[0, 2])))
            if _colp is not None and _i("COLLECT_ONSTAIR", 0) and t % 25 == 0:
                # ON-STAIR state harvest (2026-07-11, the chest-forward step-up curriculum): grab
                # alive worlds whose base is ON the staircase. SPAWN_STATES then trains policies
                # FROM mid-climb states -- discovery-by-initialization for the discrete step-up
                # (robots otherwise only ever practice the riser-1 approach from a floor spawn).
                _osm = (~done) & (qpos_t[:, 0] > _f("COLLECT_X0", 0.75)) & (qpos_t[:, 0] < _f("COLLECT_X1", 1.85)) \
                       & (qpos_t[:, 2] > 0.55)
                if bool(_osm.any()):
                    _col_q.append(qpos_t[_osm][:128].detach().cpu())
                    _col_v.append(qvel_t[_osm][:128].detach().cpu())
            if _clog_ev and t % _i("CONTACT_LOG_EVERY", 2) == 0 and t < _i("CONTACT_LOG_T1", 1200):
                try:
                    _ncC = int(rd.nacon.numpy()[0])
                    _wiC = rd.contact.worldid.numpy()[:_ncC]
                    _geC = rd.contact.geom.numpy()[:_ncC]
                    _diC = rd.contact.dist.numpy()[:_ncC]
                    _poC = rd.contact.pos.numpy()[:_ncC]
                    _frC = rd.contact.frame.numpy()[:_ncC]
                    _bxC = float(qpos_t[0, 0]); _phC = float(phase_t[0])
                    for iC in range(_ncC):
                        if int(_wiC[iC]) != 0:
                            continue
                        aC, bC = int(_geC[iC][0]), int(_geC[iC][1])
                        if _allC or aC in _cgC or bC in _cgC:
                            _nC = _frC[iC][0]
                            R._log(world, "CONTACTLOG side=BATCH t=%d g=(%d,%d) d=%.4f p=(%.3f,%.3f,%.3f) n=(%.2f,%.2f,%.2f) bx=%.3f ph=%.2f"
                                   % (t, aC, bC, float(_diC[iC]), float(_poC[iC][0]), float(_poC[iC][1]), float(_poC[iC][2]),
                                      float(_nC[0]), float(_nC[1]), float(_nC[2]), _bxC, _phC))
                except Exception as _eC:
                    if t == 0:
                        R._log(world, "CONTACTLOG BATCH err %r" % (_eC,))
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
                ytgt_t.copy_(seq_yaw_t[_sbE] + seq_yaw0_t)  # reference turn in each env's rotated routine frame
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
                    _ff6 = _g6 * ghost_legc_t[_gb] + (1.0 - _g6) * vc_stand_t
                    ctrl_t[:, leg_act_t2] = _ff6 + a[:, liu_j].clamp(-1, 1) * (_gres_gated(_gb) * _ann)
                else:
                    ctrl_t[:, leg_act_t2] = ghost_legc_t[_gb] + a[:, liu_j].clamp(-1, 1) * (_gres_gated(_gb) * _ann)
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
            if _evclk is not None:        # CONTACT-EVENT CLOCK (mirror the rollout; evgi_t zeroed at eval start)
                _gie = evgi_t.clamp(max=_evclk["G"] - 1)
                _gphe = _evclk["ph"][_gie]
                _fze = torch.where(_evclk["foot"][_gie] == 0, xpos_t[:, fL, 2], xpos_t[:, fRi, 2])
                _lande = _fze <= (_evclk["z"][_gie] + 0.025)
                _opene = _lande | (evgi_t >= _evclk["G"])
                torch.minimum(phase_t, torch.where(_opene, torch.full_like(_gphe, 1e9), _gphe), out=phase_t)
                evgi_t.add_((_lande & (phase_t >= _gphe - 1e-6) & (evgi_t < _evclk["G"])).long())
            elif _leashlead > 0.0:        # PROGRESS-LOCK (mirror the rollout so the eval reflects the trained pacing)
                torch.minimum(phase_t, (torch.searchsorted(seq_rootx_t, qpos_t[:, 0].clamp(_rootx_lo, _rootx_hi)).float() + _leashlead) / NB * twopi, out=phase_t)
            if seq_mode and seq_hold_end:
                phase_t.clamp_(max=_tp5_seq)   # eval: play once then HOLD (mirror the rollout)
            if _recS:
                _recQ[t] = qpos_t[:_recS]
            _o, roll, pitch, yaw, bz = obs_of(a)
            fwd = qvel_t[:, 0]                                       # WORLD +x speed (match training)
            xmax = torch.maximum(xmax, qpos_t[:, 0])                 # track furthest-forward reached
            live = (~done).float()
            _amag9 = mean.abs()          # PRE-clip action: >1 means the corridor is refusing the request
            act_sat += (((_amag9 > 0.95).float().mean(dim=1)) * live).sum()
            act_mag += ((_amag9.mean(dim=1)) * live).sum()
            if xfrc_t is not None and harness_lam > 0:
                # xfrc_applied still holds THIS tick's harness wrench (physics() consumes it, the next
                # _harness_apply zeroes it), so read it here where the live mask is known.
                _w9 = xfrc_t[:, pelvis_bid]
                _tmag9 = torch.maximum(_w9[:, 3].abs(), _w9[:, 4].abs())   # attitude springs: the "hand on the torso"
                cr_tau += (_tmag9 * live).sum(); cr_sus += ((_tmag9 > _TAU_SUS).float() * live).sum()
                cr_fy += (_w9[:, 1].abs() * live).sum(); cr_fz += (_w9[:, 2].abs() * live).sum()
                cr_n += live.sum()
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
            # ⛔ NaN = DEAD, and the check must cover the WHOLE state, not just bz. The rollout got this
            # guard in 2026-07-08; the EVAL never did, and it only tested `bz`. So a world whose joints or
            # velocities blew up to NaN while its base height stayed finite was counted ALIVE FOREVER --
            # it never terminated, `live` stayed 1, and its NaNs poisoned every accumulator. The eval then
            # reported the impossible: surv=1.000 fall=0.000 with gmatch/vtrack/dist/wob ALL nan
            # (measured, run corrA it=400). An eval that reports PERFECT SURVIVAL on a dead world is worse
            # than one that crashes: it looks like a result.
            _bad = (~torch.isfinite(qpos_t).all(dim=1)) | (~torch.isfinite(qvel_t).all(dim=1))
            fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz) | ~torch.isfinite(bz) | _bad
            newly = fell & (~done); alive = torch.where(newly, torch.full_like(alive, float(t)), alive)
            done = done | fell; la = a
            if bool(done.all()):
                break
        fwd_dist = float((qpos_t[~done, 0] - x0[~done]).mean()) if bool((~done).any()) else 0.0
        dist = float((xmax - x0).mean())        # avg furthest-forward distance walked (all worlds, fall or not)
        ydrift = float((qpos_t[~done, 1] - y0[~done]).abs().mean()) if bool((~done).any()) else 0.0
        if seq_mode and bool((~done).any()):
            _efw, _efx, _efy, _efz = (qpos_t[:, i] for i in (3, 4, 5, 6))
            _yfinal = torch.atan2(2 * (_efw * _efz + _efx * _efy),
                                  1 - 2 * (_efy * _efy + _efz * _efz))
            _yprog = torch.atan2(torch.sin(_yfinal - _yaw0_eval), torch.cos(_yfinal - _yaw0_eval))
            _ytarg = float(seq_yaw_t[-1] - seq_yaw_t[0])
            _ypmean = float(_yprog[~done].mean())
            _yerrmean = float((_yprog[~done] - _ytarg).abs().mean())
            _ysign = float((torch.sign(_yprog[~done]) == (1.0 if _ytarg >= 0 else -1.0)).float().mean())
            world._wr_eval_seqyaw = {"progress_deg": _ypmean * 57.2958,
                                     "error_deg": _yerrmean * 57.2958, "correct_sign": _ysign}
            R._log(world, "    SEQ-YAW-EVAL progress=%.1fdeg target=%.1fdeg err=%.1fdeg correct-sign=%.1f%%"
                   % (_ypmean * 57.2958, _ytarg * 57.2958, _yerrmean * 57.2958, 100.0 * _ysign))
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
        _n8 = max(1.0, float(live_n))
        R._log(world, "    ACT-AUTHORITY: mean|a|=%.3f  saturated(|a|>0.95)=%.1f%% of action-dims  "
                      "(corridor=%.3f rad; high saturation = STARVED, not cheating)"
               % (float(act_mag) / _n8, 100.0 * float(act_sat) / _n8, _f("GHOST_RESIDUAL", 0.0)))
        # CRANE-LOAD verdict for this eval: what the policy actually took from the harness. Stashed on
        # the world (not returned) so no caller's tuple-unpack changes. `None` when there is no harness
        # -- and the graduation gate treats "unknown" as FAIL, never as pass.
        if xfrc_t is not None and harness_lam > 0 and float(cr_n) > 0:
            _n9 = float(cr_n)
            world._wr_crane = {"tau_mean": float(cr_tau) / _n9, "sus_frac": float(cr_sus) / _n9,
                               "fy_mean": float(cr_fy) / _n9, "fz_mean": float(cr_fz) / _n9}
            R._log(world, "    CRANE-LOAD lam=%.3f: |tau|mean=%.1f N*m sustained(>%.0f)=%.1f%% |fy|mean=%.1f N "
                          "|fz|mean=%.1f N  (L1 wants sustained<=%.0f%% and |fy|<=40 N)"
                   % (harness_lam, world._wr_crane["tau_mean"], _TAU_SUS, 100.0 * world._wr_crane["sus_frac"],
                      world._wr_crane["fy_mean"], world._wr_crane["fz_mean"],
                      100.0 * _f("HARNESS_GRAD_SUS", 0.15)))
        else:
            world._wr_crane = None
        return (float(alive.mean()) / eval_H, float(done.float().mean()),
                float(vt_acc) / max(1.0, float(live_n)), fwd_dist, float(gm_acc) / max(1.0, float(live_n)), dist, ydrift,
                float(wob_acc) / max(1.0, float(live_n)), float(stopv_acc) / max(1.0, float(stopn_acc)))

    ep_ret = torch.zeros(K, device=tdev); prev_a = torch.zeros(K, ACT_DIM, device=tdev)
    last_a = torch.zeros(K, ACT_DIM, device=tdev)
    hstate = net.init_hidden(K, tdev)   # actor recurrent hidden (None for MLP), carried across rollout
    ring_tr = [torch.zeros(K, ACT_DIM, device=tdev) for _ in range(ctrl_lat)]
    O = torch.zeros(T, K, OBS_DIM, device=tdev); P = torch.zeros(T, K, PRIV_DIM, device=tdev)
    E = torch.zeros(T, K, EST_DIM, device=tdev)   # estimator-head supervision target (EST_HEAD=1)
    A = torch.zeros(T, K, ACT_DIM, device=tdev); LP = torch.zeros(T, K, device=tdev)
    V = torch.zeros(T, K, device=tdev); RW = torch.zeros(T, K, device=tdev); DN = torch.zeros(T, K, device=tdev)
    done_sum = torch.zeros((), device=tdev); done_cnt = torch.zeros((), device=tdev)
    vx_acc = torch.zeros((), device=tdev); vx_n = 0; step_ctr = 0
    _est_err = float("nan")   # last estimator-head MSE (EST_HEAD=1); scaled units, see EST_SCALE
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
                    ytgt_t.copy_(seq_yaw_t[_sb] + seq_yaw0_t)  # heading-randomized sequence frame
                o, roll, pitch, yaw, bz = obs_of(last_a)
                oin = o + torch.randn_like(o) * obs_noise if obs_noise > 0 else o
                # PRIV/EST are snapshotted HERE, pre-step, so they describe the same state o_t does.
                # (Until 2026-07-13 the stored P[t] was read back AFTER the physics step while V[t]
                # was computed before it -- priv_of() reads live tensor views -- so the critic behaved
                # on V(o_t, priv_t) but was retrained on V(o_t, priv_{t+1}), leaking post-step state
                # into the value input and making GAE's advantages come from a different function than
                # the one being optimized. One snapshot, used for both, closes it.)
                pv = priv_of(); ev = est_of(yaw)
                mean, std, hstate = net.act(oin, hstate); val = net.value(oin, pv)
                hstate = _sanitize_h(hstate)   # a NaN hidden is PERMANENT otherwise -- see _sanitize_h
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
                        _ff6 = _g6 * ghost_legc_t[_gb] + (1.0 - _g6) * vc_stand_t
                        ctrl_t[:, leg_act_t2] = _ff6 + ac[:, liu_j] * (_gres_gated(_gb) * _ann) * motor_t
                    else:
                        ctrl_t[:, leg_act_t2] = ghost_legc_t[_gb] + ac[:, liu_j] * (_gres_gated(_gb) * _ann) * motor_t
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
                _php = phase_t.clone() if _wprog > 0.0 else None   # pre-advance phase, for the W_PROG meter
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
                r_seqwz = 0.0
                if w_seqwz > 0.0 and seq_mode and seq_cmd_t is not None:
                    _wzref9 = seq_cmd_t[bidx, 2]
                    # A Gaussian is effectively flat at standstill for the natural turn
                    # (|wz_ref| reaches 0.54 rad/s while the old sigma was 0.18), so a
                    # non-turning policy received no useful gradient.  Keep a signed,
                    # V-shaped objective all the way from zero yaw rate to the reference.
                    # The engine-sourced G1 free-joint yaw-rate convention is opposite
                    # the authored root-yaw/evaluator convention in the current model.
                    # Keep the conversion explicit and campaign-fingerprinted.
                    _wzmeas9 = seqwz_qvel_sign * qvel_t[:, 5]
                    r_seqwz = torch.clamp(
                        1.0 - (_wzmeas9 - _wzref9).abs() / seqwz_sig,
                        min=-1.0,
                    )
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
                    rew = (wtl * r_lin + wta * r_ang + w_seqwz * r_seqwz
                           + wghost * r_ghost + w_armg * r_armg + w_attg * r_attg
                           + w_elb * r_elb + w_link * r_link + _wgs * r_gsched + _wgap * r_gap
                           + w_wb * r_wb + _wstop * r_stop + wfeet * r_feet + walive
                           - wvshort * torch.clamp(cmd_t[:, 0] - fwd, min=0.0) / torch.clamp(cmd_t[:, 0], min=0.1)
                           - wup * (roll * roll + pitch * pitch) - wheight * (bz - _zref) ** 2
                           - warate * arate - apen * (env_a * env_a).sum(1)
                           - wypos * torch.clamp(qpos_t[:, 1].abs() - ypos_dead, min=0.0, max=3.0)   # bounded anti-drift
                           - w_wobble * (qvel_t[:, 3:6] ** 2).sum(1) - w_bounce * qvel_t[:, 2] ** 2)   # anti-wobble
                if w_kneelow > 0:   # motion-legitimacy shaping: no knee-support postures
                    _kcl = _f("KNEE_CLEAR", 0.15)
                    rew = rew - w_kneelow * (
                        torch.clamp(_kcl - (xpos_t[:, kneeL_bid, 2] - xpos_t[:, fL, 2]), min=0.0)
                        + torch.clamp(_kcl - (xpos_t[:, kneeR_bid, 2] - xpos_t[:, fRi, 2]), min=0.0))
                if w_crane > 0.0 and xfrc_t is not None and harness_lam > 0:
                    # ── CRANE-RELIANCE PENALTY (W_CRANE) — A GATE IS NOT A GRADIENT. ────────────────
                    # HARNESS_GRAD_SUS refuses to lower the crane while the policy leans on it. But a
                    # refusal is not an incentive: nothing in the reward ever told the policy that
                    # leaning COSTS anything, so PPO was free to buy speed with lean. Measured, run fs1:
                    # parked at lam=0.4, survival a flat 1.000, and the lean GREW away from the gate --
                    # sustained 17.0 -> 19.7 -> 21.4 -> 27.0%, |tau| 25 -> 32 N*m, gmatch 0.93 -> 0.87,
                    # wobble 0.32 -> 0.56 -- while vtrack and distance IMPROVED. It was walking faster
                    # by hanging harder. The ladder could never have opened again.
                    # So PAY for what the gate asks: penalize the wrench the crane actually applies, in
                    # the same units the gate and verify_motion_legitimacy L1 use (TAU_SUS/FY_MAX = 40).
                    # The wrench IS the policy's own error (spring response to attitude/lateral drift),
                    # so this is a dense, differentiable "carry yourself" signal, and it vanishes to zero
                    # exactly when the robot no longer needs the harness.
                    _wc9 = xfrc_t[:, pelvis_bid]
                    rew = rew - w_crane * (torch.maximum(_wc9[:, 3].abs(), _wc9[:, 4].abs()) / 40.0
                                           + (_wc9[:, 1].abs() + _wc9[:, 2].abs()) / 40.0)
                fell = (roll.abs() > fr) | (pitch.abs() > fp) | (bz < fbz) | ~torch.isfinite(bz)  # NaN = dead, not immortal
                # NaN GUARD (2026-07-08): a single env whose PHYSICS blows up while climbing high (a bad
                # stair contact -> NaN qpos/qvel) poisoned the whole batched reward sum -> epret=nan ->
                # the PPO update died and the policy stopped learning past ~2-3 treads. Terminate the
                # blown-up env AND sanitize the reward so one NaN can never break the gradient again.
                fell = fell | (~torch.isfinite(qpos_t).all(dim=1)) | (~torch.isfinite(qvel_t).all(dim=1))
                if _tube_dz > 0.0:   # GHOST-TUBE ET: sinking below the phase-advanced ghost climb height = death (forces the step-up)
                    fell = fell | (bz < (seq_rootz_t[bidx] - _tube_dz))
                if _wprog > 0.0:
                    # leashed-phase advance in BINS this tick (0 = the reference is waiting for you;
                    # 1 = you are climbing at reference pace). Clamped: an RSI respawn is not progress.
                    rew = rew + _wprog * ((phase_t - _php) * (NB / twopi)).clamp(0.0, 2.0)
                rew = torch.nan_to_num(rew - fpen * fell.float(), nan=-1.0, posinf=0.0, neginf=-1.0)
                # SEQ window: completing the segment is SUCCESS -- episode ends (bootstrap cut,
                # no fall penalty) and the world respawns inside the window.
                if seq_win:
                    winx = ((phase_t / twopi * NB) >= float(seq_hi)) & (~fell)
                    ends = fell | winx
                else:
                    ends = fell
                O[t] = oin; P[t] = pv; E[t] = ev; A[t] = a; LP[t] = lp; V[t] = val; RW[t] = rew; DN[t] = ends.float()
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
                    # want_est=True ALWAYS: act_seq then returns a 3-tuple whose `est` is None when the
                    # net has no estimator head, and the `if est is not None` guard below handles it.
                    # (Passing want_est=net.est_on made the ARITY depend on the config -- with
                    # EST_HEAD=0 it returned a 2-tuple and this unpack raised. Every estimator run had
                    # EST_HEAD=1, so the first run WITHOUT the head was the first to hit it.)
                    mean, std, est = net.act_seq(O[:, mb], h0, want_est=True)   # (T,|mb|,act)
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
                    if est is not None:
                        # SUPERVISED state estimation: force the LSTM's hidden state to encode the
                        # base velocity + contact state the crane is currently supplying for free.
                        el = ((est - E[:, mb]) ** 2).mean()
                        loss = loss + est_c * el
                        _est_err = float(el.detach())
                    if not torch.isfinite(loss):
                        continue
                    opt.zero_grad(); loss.backward()
                    torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step()
        if it % 10 == 0:
            cnt = float(done_cnt); mret = float(done_sum / done_cnt) if cnt > 0 else 0.0
            fps = total_steps / max(1e-6, _time.time() - t0)
            R._log(world, "WALK-GPU it=%d epret~%.1f mfwd=%.3f(cap%.2f) std=%.2f neps=%d steps/s=%.0f%s"
                   % (it, mret, float(vx_acc) / max(1, vx_n), vx_cap, float(net.log_std.exp().mean()), int(cnt), fps,
                      ("" if not net.est_on else " est_mse=%.4f" % _est_err)))
            # FAIL LOUDLY, NEVER GRIND. If the WEIGHTS go non-finite, every future loss is NaN and the
            # `isfinite(loss): continue` guard skips every update -- the trainer keeps printing iterations
            # forever while learning nothing, and the eval reports nan. Silent no-op training is the most
            # expensive failure mode there is: you only find out hours later, from a nan in a metric.
            _nf = [_n for _n, _p in net.named_parameters() if not torch.isfinite(_p).all()]
            if _nf:
                R._log(world, "!!!! POLICY WENT NON-FINITE at it=%d (%s). Every subsequent PPO update is "
                              "skipped by the isfinite(loss) guard -- this run is DEAD and will learn "
                              "nothing. Stopping instead of grinding." % (it, ", ".join(_nf[:4])))
                _status_write("DONE", it, iters)
                return
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
                # ── ANTI-LEAN GATES. Survival alone does NOT earn a rung. ────────────────────────
                # A policy that hangs on the attitude springs survives beautifully at every level and
                # then falls off the bottom of the ladder: run p2e rode it to lam=0.30 by leaning, then
                # froze at surv 0.478 for 400+ iterations. Two gates, and BOTH fail closed:
                #  HARNESS_GRAD_SUS -- THE DIRECT MEASUREMENT: fraction of eval ticks where the crane
                #     carried sustained attitude torque (>HARNESS_TAU_SUS). Identical criterion to
                #     verify_motion_legitimacy L1, so the ladder and the final exam agree by
                #     construction. Prefer this one.
                #  HARNESS_GRAD_ATT -- the legacy posture proxy. It is ONLY computable when the
                #     arm-ghost eval runs (use_armg); for every other ghost world._wr_last_att is never
                #     assigned, and this gate used to read it as getattr(..., 1.0) -- i.e. it PASSED
                #     UNCONDITIONALLY, silently reopening the exact hole it was built to close.
                _gsus = _f("HARNESS_GRAD_SUS", 0.0)
                _cr = getattr(world, "_wr_crane", None)
                if _gsus > 0:
                    _lean_ok = (_cr is not None) and (_cr["sus_frac"] <= _gsus)
                    _lean_why = ("crane load UNMEASURED" if _cr is None else
                                 "leaning: sustained %.1f%% > %.1f%%" % (100 * _cr["sus_frac"], 100 * _gsus))
                else:
                    _lean_ok = True; _lean_why = ""
                _gatt = _f("HARNESS_GRAD_ATT", 0.0)
                _attv = getattr(world, "_wr_last_att", None)
                if _gatt > 0 and _attv is None:
                    _att_ok = False       # you ASKED for the posture gate and it cannot be computed: HOLD
                    if not getattr(world, "_wr_att_warned", False):
                        world._wr_att_warned = True
                        R._log(world, "!! HARNESS_GRAD_ATT=%.2f but the posture metric is NEVER computed for this "
                                      "ghost (only set when the arm-ghost eval runs). Holding the ladder rather "
                                      "than passing blind. Use HARNESS_GRAD_SUS: it measures the crane load itself."
                               % _gatt)
                else:
                    _att_ok = (_attv is None) or (_attv >= _gatt)
                _grad = (surv >= _f("HARNESS_GRAD_SURV", 0.9)) and _att_ok and _lean_ok and harness_lam > 0
                _step = _f("HARNESS_STEP", 0.1)
                # THE LAST RUNG IS A CLIFF. Measured 2026-07-05 (run p2g): the ladder walked 1.0 -> 0.10
                # holding surv >0.92, then the 0.10 -> 0.00 step collapsed it to surv 0.23 / fall 1.00,
                # and it took 2000+ iterations of bare-physics recovery to climb back. The last rung is
                # not one rung: it is the entire difference between "assisted" and "self-supporting".
                # HARNESS_FINE_BELOW splits it into HARNESS_STEP_FINE-sized rungs the policy can pay for.
                _fb = _f("HARNESS_FINE_BELOW", 0.0)
                if _fb > 0 and harness_lam <= _fb + 1e-6:
                    _step = _f("HARNESS_STEP_FINE", 0.02)
                if _grad:
                    harness_lam = max(0.0, harness_lam - _step)
                R._log(world, "  HARNESS lam=%.3f %s%s%s%s" % (harness_lam,
                       "(GRADUATED a level, -%.3f)" % _step if _grad else "(holding this level)",
                       "" if _att_ok else "  [HELD: posture gate]",
                       "" if _lean_ok else "  [HELD: %s -- no lean rides the ladder]" % _lean_why,
                       "  FINE ladder" if (_fb > 0 and harness_lam <= _fb) else ""))
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
        _status_write("ERROR", 0, _i("PPO_ITERS", 0))


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
        _seed = _seed_legs(len(leg_qadr))   # width from the model (trainer: _seed_legs(len(leg_qadr)))
        if _seed is not None:               # None = no built-in pose for this skeleton; keep the spawn pose
            sp[leg_qadr] = _seed
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
        ACT_DIM = len(leg_qadr); OBS_DIM = 13 + 3 * ACT_DIM   # sized by the ROBOT, not by the G1
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
    # ── BARE GHOST: run a NEW ROBOT's ghost live, before any policy exists ──────────────
    # Bringing a robot up, the order is: ghost -> validate -> SEE IT MOVE -> train. There was no
    # way to do the third step: the deploy unconditionally torch.load()ed RES_POLICY, so a robot
    # with no checkpoint could not be run at all -- which is a large part of why nothing but the
    # G1 ever went through this path.
    #
    # ⛔ It is OPT-IN AND LOUD, on purpose. A ghost replays well enough on its own to LOOK like a
    # policy result (it walks; it scores a near-ceiling gmatch because it IS the ghost) -- that
    # trap voided a whole Go2 head-to-head. So: no policy + no BARE_GHOST=1 is FATAL, exactly as
    # in the Go2 deploy controller. Never a silent zero-residual baseline.
    _rp = os.environ.get("RES_POLICY", "")
    world._wr_bare = False
    if not _rp or not os.path.isfile(_rp):
        if not _i("BARE_GHOST", 0):
            raise RuntimeError(
                f"RES_POLICY missing/not found ({_rp!r}). Refusing to run: a zero-residual ghost "
                f"replay LOOKS like a good policy result and is not one. To watch a new robot's "
                f"ghost before it has a champion, set BARE_GHOST=1 explicitly.")
        world._wr_bare = True
        R._log(world, "*** BARE GHOST: no policy. Actions are ZERO -- the robot TRACKS ITS GHOST. "
                      "This is NOT a policy result and must never be reported as one. ***")
    # Deploy inference is CPU torch: pin its numerics (threads=1, deterministic
    # kernels, TF32 off) so the SAME checkpoint computes the same actions run-to-run
    # and machine-to-machine. Training never calls this -- it wants the throughput.
    from projects.policies.common.numerics import pin_deploy_numerics
    pin_deploy_numerics(log=lambda m: R._log(world, m))
    net = _make_ac(_i("PPO_HID", 256))
    if not world._wr_bare:
        _sd = torch.load(_rp, map_location="cpu")
        net.load_state_dict(_sd)
        import hashlib as _hashlib
        _hh = _hashlib.sha256()
        with open(_rp, "rb") as _pf:
            for _chunk in iter(lambda: _pf.read(1024 * 1024), b""):
                _hh.update(_chunk)
        R._log(world, "POLICY LOADED: %s bytes=%d sha256=%s"
               % (os.path.abspath(_rp), os.path.getsize(_rp), _hh.hexdigest()))
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
            world._gwm_omega = (2.0 * math.pi * float(_json.loads(open(os.environ["GHOST_LUT_JSON"]).read()).get("freq", 1.25))
                                * _f("GHOST_OMEGA_SCALE", 1.0))   # riser-refusal fix: see the trainer-side loader note
            R._log(world, "deploy clock from GHOST_LUT_JSON: omega=%.3f" % world._gwm_omega)
        except Exception as _e:
            R._log(world, "GHOST_LUT_JSON clock err %r" % (_e,))
    world._wr_gres = _f("GHOST_RESIDUAL", 0.0)
    world._wr_greslat = _f("GHOST_RESIDUAL_LAT", world._wr_gres)
    world._wr_gresyaw = _f("GHOST_RESIDUAL_YAW", world._wr_gres)   # hip-yaw slack (turning)
    if world._wr_gres > 0 and os.environ.get("GHOST_LUT_JSON"):
        import json as _json
        _gd = _json.loads(open(os.environ["GHOST_LUT_JSON"]).read())
        world._wr_glut = np.asarray(_gd["leg_lut"], np.float32)      # (nb, n_leg) in the lut's own order
        world._wr_gnb = int(_gd["nb"])
        # ── THE ROBOT SEAM, DEPLOY SIDE (2026-07-13) ───────────────────────────────────
        # The TRAINER reads the lut's declared `joints` and derives every corridor role from
        # the NAMES (_leg_roles). The DEPLOY re-opened this same file and never read them --
        # so it kept the G1's COLUMN INDICES hardcoded: knees at [3, 9], hip-pitch at [0, 6],
        # roll at (1,5,7,11), yaw at (2,8), and `for col in range(12)`. Those are not merely
        # a count: on a 10-wide H1 lut, column 9 is an ANKLE, not a knee. The trainer was
        # converted and the deploy was not, which is exactly how a "robot-general" method
        # stays G1-only in the one place that actually drives the robot.
        world._wr_legnames = [str(j) for j in _gd.get("joints", G1_LEG_JOINTS)]
        _GHOST_ROBOT[0] = str(_gd.get("robot") or "g1").strip().lower()
        if world._wr_glut.shape[1] != len(world._wr_legnames):
            raise RuntimeError(
                f"GHOST_LUT_JSON leg_lut is {world._wr_glut.shape[1]}-wide but declares "
                f"{len(world._wr_legnames)} joints -- refusing the positional fallback")
        (world._wr_nleg, world._wr_rolli, world._wr_yawi,
         world._wr_pmask, world._wr_perleg) = _leg_roles(world._wr_legnames)
        world._wr_kneei = [i for i, j in enumerate(world._wr_legnames)
                           if RR.axis_of(j) == "knee"]
        # the FIRST pitch-plane joint of each leg == the hip pitch (the arm ghost's phase source)
        world._wr_hipi = []
        for _side in ("L", "R"):
            _c = [i for i, j in enumerate(world._wr_legnames)
                  if RR.side_row(j)[0] == _side and RR.axis_of(j) == "pitch"]
            world._wr_hipi.append(_c[0] if _c else 0)
        R._log(world, "DEPLOY robot=%s legs=%d %s | knees=%s hips=%s roll=%s yaw=%s"
               % (_GHOST_ROBOT[0], world._wr_nleg, world._wr_legnames, world._wr_kneei,
                  world._wr_hipi, world._wr_rolli, world._wr_yawi))
        # GHOST_FF deploy parity (2026-07-10): the TRAINER shifts the corridor centre by the ghost's
        # declared feedforward; a deploy that composes around the raw reference puts every joint
        # ~tau_ff/kp (up to 0.19 rad) away from what the policy trained against -- measured: eval
        # surv 0.99 / live falls-in-seconds, a pure [BUG]-shaped gap. Kept SEPARATE from _wr_glut
        # because the swing/phase heuristics read _wr_glut as the POSE.
        world._wr_ffdq = None
        if SE.ff() and "ffdq_lut" in _gd:
            world._wr_ffdq = np.asarray(_gd["ffdq_lut"], np.float32)
            R._log(world, "GHOST-FF(deploy): corridor centre += ffdq (peak %.3f rad)"
                   % float(np.abs(world._wr_ffdq).max()))
        # deploy progress-lock needs the ghost's monotone root-x (searchsorted requires sorted; the
        # solved root wiggles millimetres during weight shifts -- ratchet it monotone)
        world._wr_seq_rootx = None
        if bool(_gd.get("seq")) and "root_lut" in _gd and _f("SEQ_LEASH_LEAD", 0.0) > 0:
            world._wr_seq_rootx = np.maximum.accumulate(
                np.asarray(_gd["root_lut"], np.float32)[:, 0])
            world._wr_ph_ratchet = 0.0
            R._log(world, "SEQ-LEASH(deploy): RATCHETED progress-lock, lead=%.0f bins"
                   % _f("SEQ_LEASH_LEAD", 0.0))
        # CONTACT-EVENT CLOCK at deploy (same gates as training; takes precedence over the ratchet)
        world._wr_evgates = None
        if (bool(_gd.get("seq")) and _i("SEQ_EVENT_CLOCK", 0)
                and all(k in _gd for k in ("contact_schedule", "kp_lut", "root_lut"))):
            _sch6 = _gd["contact_schedule"]
            _kp6 = np.asarray(_gd["kp_lut"], np.float32)
            _rl6 = np.asarray(_gd["root_lut"], np.float32)
            _gl6 = []
            for _fi6, _nm6 in ((0, "foot_L"), (1, "foot_R")):
                for _b6 in range(1, len(_sch6)):
                    if (_nm6 in _sch6[_b6]) and (_nm6 not in _sch6[_b6 - 1]):
                        _gl6.append((_b6, _fi6, float(_rl6[_b6, 2] + _kp6[_b6, _fi6 * 3 + 2])))
            _gl6.sort()
            if _gl6:
                world._wr_evgates = _gl6
                world._wr_evgi = 0
                R._log(world, "SEQ-EVENT-CLOCK(deploy): %d touchdown gates" % len(_gl6))
        # leg position within the whole-body action vector (slot order), for the residual
        _laq, _lad = _build_legmap(world)
        world._wr_legqadr = np.asarray(_laq, np.int32)
        world._wr_liu = [list(world._wr_qadr).index(int(q)) for q in _laq]
        world._wr_legnd = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
        # phase-gated stance corridor (train/deploy parity): swing gates from the recorded knees
        world._wr_stight = _f("GHOST_STANCE_TIGHT", 0.0)
        if 0.0 < world._wr_stight < 1.0:
            _kn = world._wr_glut[:, world._wr_kneei]      # knees BY NAME (was the G1's [3, 9])
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
            # HAND-TRACK (2026-07-10, owner: "the robot should actually GRAB the box with its
            # hands"): resolve the two HAND bodies (leaf of each elbow's forearm chain -- the
            # CARRY-PAYLOAD trainer recipe, ported to the deploy model) and publish their live
            # FK centroid every 2 ticks. harness_rig binds the carried box to THIS point instead
            # of the old pelvis+0.20m heuristic, and gates the pick on the REAL hands reaching
            # the box. Default OFF (no env -> no writes; the rig falls back to the heuristic).
            world._wr_handbids = None
            if _i("HAND_TRACK", 0) and _el[1.0] and _el[-1.0]:
                try:
                    _hb2 = []
                    for _sd2 in (1.0, -1.0):
                        _jid2 = next(j for j in range(int(_mjm2.njnt))
                                     if int(_mjm2.jnt_qposadr[j]) == int(_el[_sd2][3]))
                        _b2 = int(_mjm2.jnt_bodyid[_jid2])
                        while True:                     # descend the (linear) forearm chain to the leaf
                            _kids2 = [c for c in range(int(_mjm2.nbody))
                                      if int(_mjm2.body_parentid[c]) == _b2 and c != _b2]
                            if not _kids2:
                                break
                            _b2 = _kids2[0]
                        _hb2.append(_b2)
                    world._wr_handbids = _hb2
                    world._wr_handfile = os.environ.get("HAND_TRACK_FILE",
                                                        "_scratch/foot_redesign/rig_hands.json")
                    R._log(world, "HAND-TRACK: publishing FK hand centroid (bodies %s) -> %s"
                           % (_hb2, world._wr_handfile))
                except Exception as _eh2:
                    R._log(world, "HAND-TRACK setup FAILED: %r" % (_eh2,))
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
                    _hl = world._wr_glut[:, world._wr_hipi[0]]   # hip pitch BY NAME (was [0] / [6],
                    _hr = world._wr_glut[:, world._wr_hipi[1]]   # i.e. the G1's 6-per-leg offset)
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
    # The recipe intentionally advances its policy/ghost clock at half the
    # 16-ms engine control interval (the historical TRAINER_CYCLE_SCALE=2
    # convention).  Keep both clocks explicit so deploy diagnostics never turn
    # a 1.6-s physical cycle into a fictitious 0.8-s speed result.
    world._wr_control_dt = _f("DEPLOY_CONTROL_DT", 2.0 * world._wr_dt)
    R._log(world, "DEPLOY CLOCK: policy_dt=%.4f engine_control_dt=%.4f omega_scale=%.3f"
           % (world._wr_dt, world._wr_control_dt, _f("GHOST_OMEGA_SCALE", 1.0)))
    world._wr_last = np.zeros(ACT_DIM, np.float32); world._wr_cmd = _f("VX_MAX", 0.7)
    # fixed heading target for the steering test / simple courses (BATON walkto sets it per tick)
    world._wr_heading_tgt = float(os.environ["HEADING_TARGET"]) if os.environ.get("HEADING_TARGET") else None
    # ── BATON (policy switching, 2026-07-06 owner campaign; docs/developer/policy-switching.md) ──
    # Two SPECIALIST policies share the deploy infrastructure: per tick the arbiter blends the
    # reference tables (corridors + REF_OBS + harness-att all read the blended luts -- morph, never
    # snap) and crossfades the two nets' actions. Both nets run EVERY tick (warm recurrent handover).
    # The runtime is projects/policies/training/baton.py -- ONE arbiter, shared by the course
    # and schedule paths (they used to be two near-identical copies, and they had drifted).
    world._wr_baton = None
    world._wr_baton_host = BHOST.InEngineHost(
        world, torch, lambda: _make_ac(_i("PPO_HID", 256)), R._log)
    try:
        world._wr_baton = BATON.setup(
            world._wr_baton_host,
            primary_tables=world._wr_baton_host.primary_tables(_f("VX_MAX", 0.45)),
            load_policy=world._wr_baton_host.load_policy, geti=_i)
    except Exception as _eB:
        R._log(world, "BATON setup FAILED: %r" % (_eB,))
    # The support gate is the ONE morphology-aware part of BATON (a biped may only be handed
    # over at double support). Selected by robot class, not hardcoded in the arbiter.
    world._wr_baton_gate = BATON.gate_for(
        RR.robot_class(_ghost_robot(world)) if RR.is_known(_ghost_robot(world)) else "humanoid")
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
    world._wr_turnctx = None; world._wr_carry_turnctx = None
    world._wr_in_turn = False; world._wr_turn_active = False
    for _tpfx, _tattr, _tname in (("BATON_TURN", "_wr_turnctx", "empty"),
                                  ("BATON_CARRY_TURN", "_wr_carry_turnctx", "payload")):
        _tck = os.environ.get(_tpfx + "_CKPT"); _tlu = os.environ.get(_tpfx + "_LUT")
        if _tck and _tlu and os.path.exists(_tck) and os.path.exists(_tlu):
            try:
                _tgres = _f(_tpfx + "_RESIDUAL", world._wr_gres)
                setattr(world, _tattr, _wr_load_turn_context(
                    world, torch, whole, leg_qadr, _tck, _tlu,
                    _i(_tpfx + "_REF_WB", _i("BATON_TURN_REF_WB", 1)), _tname,
                    _tgres,
                    _f(_tpfx + "_RESIDUAL_LAT", _tgres),
                    _f(_tpfx + "_RESIDUAL_YAW", _tgres)))
            except Exception as _et:
                R._log(world, "BATON SOLO-TURN %s setup FAILED: %r" % (_tname, _et))
    try:
        _lae = [int(world._gwm_act[i]) for i, _s in enumerate(world._gwm_slots) if _s < 12]
        _mpe = world.solver.mj_model; _nue = int(_mpe.nu)
        _bpe = np.asarray(_mpe.actuator_biasprm).reshape(_nue, -1)
        _gpe = np.asarray(_mpe.actuator_gainprm).reshape(_nue, -1)
        _kpe = [-float(_bpe[_a][1]) for _a in _lae]
        _kde = [-float(_bpe[_a][2]) for _a in _lae]
        _kve = [float(_gpe[_a + 1][0]) if _a + 1 < _nue else -1.0 for _a in _lae]
        R._log(world, "PLANT-ECHO(deploy) leg kp=%s kd=%s vel-kv=%s"
               % ([round(v, 2) for v in _kpe], [round(v, 2) for v in _kde],
                  [round(v, 2) for v in _kve]))
    except Exception as _epe:
        R._log(world, "PLANT-ECHO(deploy) err %r" % (_epe,))
    R._log(world, "WALK-RECIPE DEPLOY ready whole=%d act=%d cmd_vx=%.2f" % (int(whole), ACT_DIM, world._wr_cmd))
    return True


def _wr_load_turn_context(world, torch, whole, leg_qadr, ckpt, lut, ref_wb, label,
                          gres, greslat, gresyaw):
    """Load one SOLO-TURN specialist without disturbing the primary deploy context."""
    global OBS_DIM
    import json

    gd = json.loads(open(lut).read())
    use_wb = bool(ref_wb and "wb_lut" in gd and whole)
    reftgt = np.asarray(gd["wb_lut" if use_wb else "leg_lut"], np.float32)
    useatt = "att_lut" in gd
    prim_rb = (_refobs.ref_block_dim(int(world._wr_reftgt.shape[1]), world._wr_ref_K,
                                    world._wr_ref_useatt)
               if getattr(world, "_wr_reftgt", None) is not None else 0)
    turn_rb = _refobs.ref_block_dim(int(reftgt.shape[1]), world._wr_ref_K, useatt)
    saved_obs = OBS_DIM
    OBS_DIM = saved_obs - prim_rb + turn_rb
    try:
        net = _make_ac(_i("PPO_HID", 256))
        net.load_state_dict(torch.load(ckpt, map_location="cpu")); net.eval()
    finally:
        OBS_DIM = saved_obs

    seqcmd = seqyaw = yawflat = None
    seqnb = int(gd["nb"]); seqhold = bool(gd.get("hold_end", False))
    if gd.get("seq") and "root_lut" in gd:
        root = np.asarray(gd["root_lut"], np.float32)
        dtb = (1.0 / float(gd.get("freq", 0.05))) / seqnb
        vw = (np.roll(root, -1, axis=0) - root) / dtb; vw[-1] = vw[-2]
        cy, sy = np.cos(root[:, 3]), np.sin(root[:, 3])
        seqcmd = np.clip(cy * vw[:, 0] + sy * vw[:, 1], -1.5, 1.5).astype(np.float32)
        seqcmd[-4:] = seqcmd[-5]
        seqyaw = root[:, 3].astype(np.float32)
        # Plateau starts let legacy modular ghosts replay a partial pass. Compact
        # professional ghosts usually finish in one pass, but retaining this table
        # keeps the loader backward-compatible with the shipped 90-degree turner.
        grad = np.abs(np.gradient(seqyaw.astype(np.float64)))
        flat = grad < 1e-4
        for shift in (-2, -1, 1, 2):
            flat = flat & np.roll(grad < 1e-4, shift)
        yawflat = []
        inside = False
        for b in range(len(flat)):
            if flat[b] and not inside:
                yawflat.append((int(b), float(np.degrees(seqyaw[b])))); inside = True
            elif not flat[b]:
                inside = False

    ctx = {
        "net": net, "reftgt": reftgt,
        "ref_qadr": np.asarray(world._wr_qadr if use_wb else leg_qadr),
        "refnb": int(gd["nb"]), "ref_useatt": useatt,
        "refatt": np.asarray(gd["att_lut"], np.float32) if useatt else None,
        "glut": np.asarray(gd["leg_lut"], np.float32), "gnb": int(gd["nb"]),
        "omega": 2.0 * math.pi * float(gd.get("freq", 0.05)), "cmd": 0.0,
        "gres": float(gres), "greslat": float(greslat), "gresyaw": float(gresyaw),
        "armlut": np.asarray(gd["arm_lut"], np.float32) if "arm_lut" in gd else None,
        "elblut": np.asarray(gd["elbow_lut"], np.float32) if "elbow_lut" in gd else None,
        "seqcmd": seqcmd, "seqnb": seqnb, "seq_yaw": seqyaw, "seq_hold": seqhold,
        "seq_root": root[:, :2].astype(np.float32) if seqcmd is not None else None,
        "plateau_starts": yawflat, "label": label, "checkpoint": ckpt,
    }
    R._log(world, "BATON SOLO-TURN %s armed: %s (nb=%d omega=%.3f wb_ref=%s obs=%d seq=%s yaw-span=%.0f residual=%.3f)"
           % (label, os.path.basename(ckpt), int(gd["nb"]), ctx["omega"], use_wb,
               saved_obs - prim_rb + turn_rb, seqcmd is not None,
               float(np.ptp(seqyaw)) * 57.3 if seqyaw is not None else 0.0,
               ctx["gres"]))
    return ctx


def _wr_turn_swap(world, is_turn):
    """Swap the deploy context to/from the SOLO turn specialist on turn-segment boundaries (see the
    BATON SOLO-TURN note in the deploy setup). No-op unless a turn context was armed. Cold-hidden on
    every boundary (the warm-handover stand-attractor lock: never hand a translating specialist a
    stand-settled recurrent state)."""
    ctx = getattr(world, "_wr_turnctx", None)
    if is_turn and (getattr(world, "_wr_suck", None) or {}).get("on", False):
        ctx = getattr(world, "_wr_carry_turnctx", None) or ctx
    if ctx is None:
        world._wr_turn_active = bool(is_turn)
        return
    _fields = ("net", "h", "reftgt", "ref_qadr", "refnb", "ref_useatt", "refatt", "glut", "gnb",
               "omega", "cmd", "gres", "greslat", "gresyaw", "armlut", "elblut", "seqcmd",
               "seqnb", "seq_yaw", "seq_hold")
    if is_turn and not world._wr_in_turn:
        world._wr_active_turnctx = ctx
        world._wr_blendctx = {k: getattr(world, "_wr_" + k, None) for k in _fields}
        world._wr_net = ctx["net"]; world._wr_h = ctx["net"].init_hidden(1, "cpu")   # cold in
        world._wr_reftgt = ctx["reftgt"]; world._wr_ref_qadr = ctx["ref_qadr"]
        world._wr_refnb = ctx["refnb"]; world._wr_ref_useatt = ctx["ref_useatt"]
        world._wr_refatt = ctx["refatt"]; world._wr_glut = ctx["glut"]; world._wr_gnb = ctx["gnb"]; world._wr_ffdq = ctx.get("ffdq")
        world._wr_omega = ctx["omega"]; world._wr_cmd = ctx["cmd"]
        world._wr_gres = ctx["gres"]; world._wr_greslat = ctx["greslat"]
        world._wr_gresyaw = ctx["gresyaw"]
        world._wr_armlut = ctx["armlut"]; world._wr_elblut = ctx["elblut"]
        world._wr_seqcmd = ctx["seqcmd"]; world._wr_seqnb = ctx["seqnb"]
        world._wr_seq_yaw = ctx["seq_yaw"]; world._wr_seq_hold = ctx["seq_hold"]
        world._wr_phase = 0.0; world._wr_turn_acc = 0.0; world._wr_turn_done = False; world._wr_turn_pass = 1
        world._wr_turn_failed = False
        # Anchor the live pelvis reference at the physical entry pose.  The compact turn ghost
        # deliberately shifts its pelvis over each stance foot and returns to its origin; retaining
        # that relative path lets the balance harness arrest tracking drift without freezing the
        # weight shifts that make the footwork possible.  The harness branch consumes this only when
        # TURN_ROOT_TRACK_KP is explicitly enabled and never applies yaw torque.
        try:
            _qsxy = world.solver.mjw_data.qpos.numpy().reshape(-1)
            world._wr_turn_xy0 = (float(_qsxy[0]), float(_qsxy[1]))
        except Exception:
            world._wr_turn_xy0 = None
        # TURN DIRECTION (2026-07-12): the ghost is a one-way routine (this one sweeps 0 -> +90 deg).
        # TURN-LOOP scored progress as abs(_wr_turn_acc), which CREDITS ROTATION THE WRONG WAY as
        # progress -- so a robot spun backwards past -125 deg tripped the over-rotation abort and was
        # reported as "over-rotated to 131 deg", ~220 deg away from its target. Take the sign from
        # the ghost's own yaw table and score progress ALONG it.
        _sy9 = ctx.get("seq_yaw")
        world._wr_turn_sgn = (1.0 if float(_sy9[-1]) - float(_sy9[0]) >= 0.0 else -1.0) \
            if (_sy9 is not None and len(_sy9) > 1) else 0.0
        _pl9 = ctx.get("plateau_starts") or []
        world._wr_turn_gplayed = float(_pl9[-1][1]) - float(_pl9[0][1]) if len(_pl9) >= 2 else 90.0
        try:   # entry yaw: TURN_OBS_RELATIVE presents every corner to the net at heading 0 (as trained)
            _qs0 = world.solver.mjw_data.qpos.numpy().reshape(-1)
            world._wr_turn_yaw0 = math.atan2(2 * (float(_qs0[3]) * float(_qs0[6]) + float(_qs0[4]) * float(_qs0[5])),
                                             1 - 2 * (float(_qs0[5]) ** 2 + float(_qs0[6]) ** 2))
        except Exception:
            world._wr_turn_yaw0 = 0.0
        if hasattr(world, "_wr_yaw_prev"):
            del world._wr_yaw_prev    # TURN_TO_DEG seeds _yaw_prev from getattr default on tick 1; a
        if hasattr(world, "_wr_turn_done_t"):
            del world._wr_turn_done_t  # fresh settle window for this turn
        for _ta in ("_wr_turn_coast_to", "_wr_turn_coast0"):
            if hasattr(world, _ta):
                delattr(world, _ta)   # fresh complete-the-step coast for this turn
        world._wr_in_turn = True      # leftover None here would TypeError (_ynow - None) every tick
        R._log(world, "BATON SOLO-TURN: swapped IN at t=%d (%s specialist active)"
               % (world._wr_tt, ctx.get("label", "turn")))
    elif (not is_turn) and world._wr_in_turn:
        b = world._wr_blendctx
        world._wr_net = b["net"]; world._wr_h = b["net"].init_hidden(1, "cpu")        # cold out
        world._wr_reftgt = b["reftgt"]; world._wr_ref_qadr = b["ref_qadr"]
        world._wr_refnb = b["refnb"]; world._wr_ref_useatt = b["ref_useatt"]
        world._wr_refatt = b["refatt"]; world._wr_glut = b["glut"]; world._wr_gnb = b["gnb"]; world._wr_ffdq = b.get("ffdq")
        world._wr_omega = b["omega"]; world._wr_cmd = b["cmd"]
        world._wr_gres = b["gres"]; world._wr_greslat = b["greslat"]
        world._wr_gresyaw = b["gresyaw"]
        world._wr_armlut = b["armlut"]; world._wr_elblut = b["elblut"]
        world._wr_seqcmd = b["seqcmd"]; world._wr_seqnb = b["seqnb"]
        world._wr_seq_yaw = b["seq_yaw"]; world._wr_seq_hold = b["seq_hold"]
        world._wr_phase = 0.0
        world._wr_turn_xy0 = None
        world._wr_walk_entered = False   # force a FRESH walk/carry re-entry (settle-and-go) from the
        world._wr_catching = False       # turn's settled pose -- else the next bout inherits stale state
        world._wr_last = np.zeros(len(getattr(world, "_wr_qadr", [0] * ACT_DIM)), np.float32)
        # HEADING RETENTION (owner 2026-07-08, "walk in the NEW direction after the turn"): pin the
        # walk's heading target to the ACHIEVED standing yaw at turn-exit. The walker only HOLDS a
        # heading when the heading obs (yaw - htgt) sits in its trained band; at the settled post-turn
        # pose pelvis_yaw == travel yaw, so htgt = that yaw makes the next straight walk see the SAME
        # in-distribution gait offset it trained with -- and walk straight in the NEW direction,
        # instead of steering back to the old one (the drift measured in the box demo).
        # ⛔ but NOT after a FAILED turn (2026-07-12): the pin trusts the achieved yaw, so an aborted
        # turn hands the next walk a heading target that is tens of degrees (measured: ~180) off the
        # course. Leave the previous target standing -- on a BATON course the next walkto segment
        # slews the target back to the real bearing within a second, which is the honest recovery.
        if _i("BATON_TURN_KEEP_HEADING", 1) and not getattr(world, "_wr_turn_failed", False):
            try:
                _qs = world.solver.mjw_data.qpos.numpy().reshape(-1)
                _w2, _x2, _y2, _z2 = float(_qs[3]), float(_qs[4]), float(_qs[5]), float(_qs[6])
                world._wr_heading_tgt = math.atan2(2 * (_w2 * _z2 + _x2 * _y2), 1 - 2 * (_y2 * _y2 + _z2 * _z2))
                R._log(world, "SOLO-TURN keep-heading: post-turn walk target pinned to %.0f deg"
                       % (world._wr_heading_tgt * 57.2958))
            except Exception as _ekh:
                R._log(world, "SOLO-TURN keep-heading failed: %r" % (_ekh,))
        elif getattr(world, "_wr_turn_failed", False):
            R._log(world, "SOLO-TURN keep-heading SKIPPED: the turn aborted -- not pinning the walk "
                          "target to a yaw the turn never reached")
        world._wr_in_turn = False; world._wr_active_turnctx = None
        R._log(world, "BATON SOLO-TURN: swapped OUT at t=%d (walk/blend context restored)" % world._wr_tt)
    world._wr_turn_active = bool(is_turn)


def _wr_patch_foot_torsion(world):
    """FOOT SPIN FRICTION (2026-07-07, the turning wall): MuJoCo foot contacts default to condim=3
    -- slide friction only, ZERO torsional (spin) resistance. A planted flat foot then freewheels
    about its vertical axis, so hip-yaw rotation does NOT transfer to base rotation and a step-turn
    loses ~2/3 of its yaw (diagnosed: legs track the ghost, base rotates 30 of 90 deg). Real soles
    resist twist. OMNISIM_FOOT_TORSION=<coef> raises the foot geoms to condim=4 with a real
    torsional friction coefficient (default off -> exact prior physics). Mirrors the engine's
    OMNISIM_NEWTON_ROLL_MU patch (OmNewtonBackend.cpp) but for TORSION, and only on the feet."""
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
    leg_nd = [int(world._gwm_dof[i]) for i, s in enumerate(world._gwm_slots) if s < 12]
    crouch = _seed_legs(len(leg_nd))   # was a 12-wide G1 crouch silently truncated onto an H1
    if crouch is None:                 # no built-in pose for this skeleton -> keep the spawn pose
        R._log(world, "TELEPORT-CROUCH skipped: no seed pose for a %d-DOF leg" % len(leg_nd))
        return
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
        # Newton 1.4 changed this private API from an implicit no-arg sync to
        # _update_mjc_data(mj_data, model, state=None).  Passing the live objects
        # is required: without it the perturbed crouch remains only in Newton's
        # model buffers and deploy falls back into the measured slow shuffle.
        world.solver._update_mjc_data(world.solver.mjw_data, m)
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
    tp = _ctl_target_pos(world).numpy()
    for i in range(len(st["leg_nd"])):          # the robot's leg DOFs, not the G1's 12
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
    _ctl_target_pos(world).assign(tp)
    # LP_NO_DIRTY=1: do NOT force the per-tick newton->mjw state re-import. Control writes don't
    # need it (apply_mjc_control reads the control arrays each substep); only STATE mutations do.
    # If the live sag disappears with this off, the sag = the newton<->mjw state ROUND-TRIP decay
    # from dirtying every tick -- and the deploy-hook fix is simply to stop setting _mjc_dirty.
    if not _i("LP_NO_DIRTY", 0):
        world._mjc_dirty = True
    if st["seeded"]:                      # batched only steps once tick-aligned (seed at t=1)
        ctrl = rd.ctrl.numpy().reshape(1, nu)
        for i in range(len(st["leg_act"])):     # the robot's leg DOFs, not the G1's 12
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
        tp0 = _ctl_target_pos(world).numpy()
        for i in range(len(world._wr_ndof)):
            nd = int(world._wr_ndof[i])
            if 0 <= nd < len(tp0):
                tp0[nd] = float(world._wr_nom[i])
        _ctl_target_pos(world).assign(tp0)
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
    tp = _ctl_target_pos(world).numpy()
    for i in range(len(world._wr_ndof)):
        nd = int(world._wr_ndof[i])
        if 0 <= nd < len(tp):
            tp[nd] = float(world._wr_nom[i]) + float(env_live[i])
    _ctl_target_pos(world).assign(tp)
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
            # DEPLOY_YAW0 (2026-07-12, diagnostic): seed the handoff reset at a nonzero world yaw.
            # The robot otherwise ALWAYS starts facing +x, so the first turn of every course is a
            # turn from heading 0 and no experiment can separate "fails at heading ~90" from "fails
            # on the SECOND turn". With this, a course can be rotated bodily and the first turn taken
            # at any heading. Default 0 -> the reset is byte-identical.
            _y09 = _f("DEPLOY_YAW0", 0.0)
            qp2[0, 3:7] = [math.cos(0.5 * _y09), 0.0, 0.0, math.sin(0.5 * _y09)]
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
        tp = _ctl_target_pos(world).numpy()
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
        _ctl_target_pos(world).assign(tp); world._mjc_dirty = True
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
        #   "walkto,X,Y;backto,X,Y;stand,S;carryto,X,Y;carrybackto,X,Y;..."
        # Forward and reverse segments steer the heading-conditioned specialist toward (X,Y)
        # and advance ON ARRIVAL; a reverse segment faces AWAY from its travel bearing, so it
        # genuinely walks backward instead of turning around and walking forward.
        # S seconds (heading target HELD, so the robot keeps facing its work). Last segment holds.
        if not hasattr(world, "_wr_course"):
            _segs9 = []
            for _sg9 in _course_env.split(";"):
                _ff9 = _sg9.split(",")
                if _ff9[0] in ("walkto", "carryto", "backto", "carrybackto"):
                    _segs9.append((_ff9[0], float(_ff9[1]), float(_ff9[2])))
                else:
                    # optional 3rd field on a TURN segment = per-segment turn angle in degrees
                    # ("turn,60,180" = a 180-deg about-face; TURN-LOOP replays passes until the
                    # ACTUAL heading reaches it). 0 -> the global TURN_TO_DEG applies as before.
                    _segs9.append((_ff9[0], float(_ff9[1]), float(_ff9[2]) if len(_ff9) > 2 else 0.0))
            world._wr_course = _segs9; world._wr_course_i = 0; world._wr_course_t0 = tt
            # ── PER-CYCLE SUCCESS TELEMETRY (2026-07-12) ─────────────────────
            # Every BATON claim needs this and none of it existed: the deploy could
            # tell you a segment advanced, but never whether a CYCLE succeeded. So
            # the switching-beats-a-monolith experiment could not be scored, and an
            # ablation run on 2026-07-12 had to be thrown away because both arms
            # "ended upright" and nothing distinguished them.
            #
            # BATON_COURSE_LOOPS=N runs the course N times. A cycle SUCCEEDS iff it
            # completed every segment and the pelvis never dropped below BATON_FALL_Z.
            # One line per cycle, machine-readable, consumed by baton_metrics.py:
            #     BATON-CYCLE k=<i> ok=<0|1> segs=<done>/<total> t=<tick> dur=<s> minz=<m>
            world._wr_loops = max(1, _i("BATON_COURSE_LOOPS", 1))
            world._wr_cycle = 0
            world._wr_cyc_t0 = tt
            world._wr_cyc_minz = 9.9
            world._wr_cyc_fell = False
            world._wr_cyc_segs = 0
            R._log(world, "BATON COURSE: %s  (loops=%d)" % (_segs9, world._wr_loops))
        if _i("TURN_DBG", 0) and tt % _i("TURN_DBG_EVERY", 10) == 0:
            _dq = world.solver.mjw_data.qpos.numpy().reshape(-1)
            _dv = world.solver.mjw_data.qvel.numpy().reshape(-1)
            _dy = math.atan2(2 * (float(_dq[3]) * float(_dq[6]) + float(_dq[4]) * float(_dq[5])),
                             1 - 2 * (float(_dq[5]) ** 2 + float(_dq[6]) ** 2))
            R._log(world, "CRS-DBG t=%d seg=%d %s yaw=%.4f htgt=%s wz=%.3f wtz=%.1f z=%.3f"
                   % (tt, world._wr_course_i, world._wr_course[world._wr_course_i][0], _dy,
                      "%.4f" % world._wr_heading_tgt if getattr(world, "_wr_heading_tgt", None) is not None else "-",
                      float(_dv[5]), float(getattr(world, "_wr_dbg_wrench", (0.0,) * 5)[4]), float(_dq[2])))
        # fall watch, sampled every tick of the CURRENT cycle (not just at switches)
        _fz = _f("BATON_FALL_Z", 0.45)
        _pz9 = float(world.solver.mjw_data.qpos.numpy().reshape(-1)[2])
        _nf9 = not math.isfinite(_pz9)
        if _nf9:
            world._wr_cyc_minz = -1.0
        elif _pz9 < world._wr_cyc_minz:
            world._wr_cyc_minz = _pz9
        if _nf9 or _pz9 < _fz:
            world._wr_cyc_fell = True
            # TERMINAL FALL VERDICT (2026-07-12): a cycle verdict was only ever emitted when the
            # course ADVANCED past its last segment -- so a run in which the robot FELL mid-cycle
            # emitted NO line at all, and "fell during cycle 0" read exactly like "the run never
            # started". Worse, the fallen robot kept TIMING OUT through its remaining segments and
            # scored them as completed (measured: 8/8 segments "done" while the pelvis lay at
            # z=0.09). A fallen G1 does not get up: emit the verdict for the CURRENT cycle NOW,
            # once, and stop the course there.
            if not getattr(world, "_wr_cyc_verdict", False):
                world._wr_cyc_verdict = True
                world._wr_course_done = True
                R._log(world, "BATON-CYCLE k=%d ok=0 segs=%d/%d t=%d dur=%.1f minz=%.3f"
                       % (world._wr_cycle, world._wr_cyc_segs, len(world._wr_course), tt,
                          (tt - world._wr_cyc_t0) * 0.016, world._wr_cyc_minz))
                if _nf9:
                    R._log(world, "BATON-COURSE FELL: cycle %d aborted at t=%d (non-finite pelvis state)"
                           % (world._wr_cycle, tt))
                else:
                    R._log(world, "BATON-COURSE FELL: cycle %d aborted at t=%d (pelvis z=%.3f < %.2f)"
                           % (world._wr_cycle, tt, _pz9, _fz))

        def _wr_course_advance(_w, _t):
            """Advance one segment; at the end of the course, close the CYCLE and wrap.

            Returns True while the course should keep running, False once every
            requested loop is done (the caller then holds the final segment).
            """
            _w._wr_cyc_segs += 1
            if _w._wr_course_i + 1 < len(_w._wr_course):
                _w._wr_course_i += 1
                _w._wr_course_t0 = _t
                return True
            # end of the course = end of a cycle. Score it, then wrap or stop.
            _ok = int((not _w._wr_cyc_fell) and _w._wr_cyc_segs >= len(_w._wr_course))
            R._log(_w, "BATON-CYCLE k=%d ok=%d segs=%d/%d t=%d dur=%.1f minz=%.3f"
                   % (_w._wr_cycle, _ok, _w._wr_cyc_segs, len(_w._wr_course), _t,
                      (_t - _w._wr_cyc_t0) * 0.016, _w._wr_cyc_minz))
            _w._wr_cycle += 1
            if _w._wr_cycle >= _w._wr_loops:
                R._log(_w, "BATON-COURSE DONE: %d/%d cycles requested"
                       % (_w._wr_cycle, _w._wr_loops))
                return False
            _w._wr_course_i = 0
            _w._wr_course_t0 = _t
            _w._wr_cyc_t0 = _t
            _w._wr_cyc_minz = 9.9
            _w._wr_cyc_fell = False
            _w._wr_cyc_segs = 0
            # the turn specialist's completion state is PER-TURN: carrying it across a
            # wrap would make the next cycle's turn segment believe it had already
            # finished and exit instantly (a silently shortened cycle).
            for _a in ("_wr_turn_done", "_wr_turn_done_t", "_wr_turn_acc", "_wr_turn_pass"):
                if hasattr(_w, _a):
                    delattr(_w, _a)
            return True
        _qpc = world.solver.mjw_data.qpos.numpy().reshape(-1)
        _cpx, _cpy = float(_qpc[0]), float(_qpc[1])
        _ci = world._wr_course_i
        _seg = world._wr_course[_ci]
        _bearing9 = None   # the course bearing the carrot window ratchets toward (turn-scoped)
        if _seg[0] in ("walkto", "carryto", "backto", "carrybackto"):
            _dx9, _dy9 = _seg[1] - _cpx, _seg[2] - _cpy
            _travel9 = math.atan2(_dy9, _dx9)
            _reverse9 = _seg[0] in ("backto", "carrybackto")
            _btgt = math.atan2(math.sin(_travel9 + (math.pi if _reverse9 else 0.0)),
                               math.cos(_travel9 + (math.pi if _reverse9 else 0.0)))
            _bearing9 = _btgt
            world._wr_last_walk_bearing = _btgt   # remembered so the next STAND can pivot by the exterior angle
            world._wr_pivot_active = False
            # swap the SOLO-TURN context OUT when a walkto/carryto directly follows a turn segment
            # (2026-07-10): only the schedule path called the swap every tick; a course that went
            # turn -> walkto left the turn net + seq tables active for the whole next leg.
            _wr_turn_swap(world, False)
            # SLEW the heading target (missed-waypoint bearing flips thrashed the crane torque
            # into NaN); the steering stays gentle and continuous.
            _prev9 = getattr(world, "_wr_heading_tgt", None)
            if _prev9 is None:
                world._wr_heading_tgt = _btgt
            else:
                _d9 = math.atan2(math.sin(_btgt - _prev9), math.cos(_btgt - _prev9))
                _sl9 = _f("BATON_SLEW", 0.02)
                _ht9n = _prev9 + max(-_sl9, min(_sl9, _d9))
                # re-wrap the STORED target (2026-07-10): after a ~180-deg keep-heading pin the
                # slew crosses +/-pi; an unwrapped store spirals (measured: htgt walked to -6.28,
                # and every telemetry/bearing read went garbage while a fallen robot slid past
                # the goal). All consumers wrap, but the stored value must stay principal too.
                world._wr_heading_tgt = math.atan2(math.sin(_ht9n), math.cos(_ht9n))
            _mode = {"walkto": "walk", "carryto": "carry",
                     "backto": "back", "carrybackto": "carryback"}[_seg[0]]
            # Reverse retreats exist to create real furniture clearance; the normal 0.55 m
            # waypoint radius would stop a nominal 1 m retreat after only ~45 cm.  Give reverse
            # segments their own tighter gate while preserving every existing forward course.
            _arr_r9 = _f("BATON_BACK_ARRIVE_R", 0.20) if _reverse9 else _f("BATON_ARRIVE_R", 0.55)
            _arrived9 = math.hypot(_dx9, _dy9) < _arr_r9
            # SUCTION PRESS-PLACE arrival (2026-07-11): on the carry leg INTO the place segment,
            # the delivery stand physically blocks the robot from ever reaching BATON_ARRIVE_R
            # (measured grS9: pinned at 0.64 m for 400 s, box parked 8 cm over the pedestal).
            # The place choreography runs FROM the carry-press (the press is the stable hover;
            # a stand-settle drags the hanging box ~20 cm back west, grS10/grS12) -- the course
            # advances only after the box has been RELEASED (grasp state open/withdraw/done).
            if not _arrived9 and _seg[0] == "carryto" and _ci + 1 == _i("GR_PLACE_SEG", -1):
                _sk3 = getattr(world, "_wr_suck", None)
                _gst3 = getattr(world, "_wr_gr", None)
                if _sk3 is not None and _gst3 is not None and \
                        _gst3.get("state") in ("open", "withdraw", "done"):
                    _arrived9 = True
                    R._log(world, "SUCTION PRESS-PLACE released -> course advances")
            if _arrived9 and not getattr(world, "_wr_course_done", False):
                if _wr_course_advance(world, tt):
                    R._log(world, "BATON COURSE arrived at (%.2f, %.2f) -> segment %d %s"
                           % (_seg[1], _seg[2], world._wr_course_i,
                              world._wr_course[world._wr_course_i]))
                else:
                    world._wr_course_done = True
        else:
            _mode = _seg[0]
            # TURN segment (2026-07-07): switch to the trained turn specialist for a timed live
            # turn (~38deg per trigger; BATON_COLD_HIDDEN gives a fresh turn each corner). The
            # timed advance is the shared dwell logic below (pivot inactive -> time-based).
            _wr_turn_swap(world, _seg[0] == "turn")   # SOLO-TURN swap (no-op unless BATON_TURN_CKPT armed)
            # per-segment turn angle override (the course's "turn,SECONDS,DEG" 3rd field)
            world._wr_turn_deg_ovr = _seg[2] if (_seg[0] == "turn" and len(_seg) > 2 and _seg[2] > 0) else 0.0
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
                # And HOLD THE ROBOT'S OWN HEADING (2026-07-10, measured): holding the stale leg
                # BEARING instead stored a 13-deg elastic twist against stiction; at the next turn's
                # swap-in the crane yaw cuts off, the twist RELEASED as a -20 deg snap in 0.35 s and
                # scrambled the turner's cold start (pass-1 gain collapsed 0.7 -> 0.35, then fell).
                if getattr(world, "_wr_pivot_seg", -1) != _ci:
                    world._wr_heading_tgt = _yawnow9
                world._wr_pivot_active = False; world._wr_pivot_seg = _ci
            elif _seg[0] == "stand" and getattr(world, "_wr_pivot_seg", -1) != _ci:
                # compute the pivot GOAL once on entry (relative exterior-angle rotation)
                _out9 = None
                for _nx9 in world._wr_course[_ci + 1:]:
                    if _nx9[0] in ("walkto", "carryto", "backto", "carrybackto"):
                        _out9 = math.atan2(_nx9[2] - _cpy, _nx9[1] - _cpx)
                        if _nx9[0] in ("backto", "carrybackto"):
                            _out9 = math.atan2(math.sin(_out9 + math.pi), math.cos(_out9 + math.pi))
                        break
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
            if _seg[0] == "turn" and getattr(world, "_wr_turn_done", False):
                # TURN-LOOP course parity (2026-07-10): a COMPLETED turn advances after its settle
                # window instead of holding the frozen end-hold for the whole timed dwell (the
                # schedule path's early exit, ported). The dwell seconds remain the upper bound.
                if not hasattr(world, "_wr_turn_done_t"):
                    world._wr_turn_done_t = tt
                _dwell_ok = _dwell_ok or (tt - world._wr_turn_done_t) >= _i("BATON_TURN_EXIT_TICKS",
                                                                            _i("BATON_TURN_SETTLE_TICKS", 90))
            _dwell_max = (tt - world._wr_course_t0) * 0.016 >= _seg[1] + 8.0   # hard timeout: never deadlock
            if _seg[0] == "stand" and _ci == _i("GR_PICK_SEG", -1) and _i("GR_SUCTION", 0):
                # PICK-AWARE dwell (2026-07-12): the fast gait's stop scatter makes the touch
                # servo retry; a fixed dwell marched the course into the TURN mid-retry and the
                # robot walked away from the box it was reaching for. Advance EARLY once the
                # box is lifted; HOLD (generous ceiling) while an attempt is in progress.
                _gst8 = getattr(world, "_wr_gr", None)
                _gs8 = _gst8.get("state") if _gst8 else None
                if _gs8 == "hold":
                    _dwell_ok = (tt - world._wr_course_t0) * 0.016 >= 2.0
                elif _gs8 in ("idle", "reach", "engage", "lift"):
                    _dwell_ok = False
                    _dwell_max = (tt - world._wr_course_t0) * 0.016 >= _seg[1] + 75.0
            if ((_dwell_ok and _pv_done) or _dwell_max) and not getattr(world, "_wr_course_done", False):
                world._wr_pivot_active = False
                if _wr_course_advance(world, tt):
                    R._log(world, "BATON COURSE -> segment %d %s"
                           % (world._wr_course_i, world._wr_course[world._wr_course_i]))
                else:
                    world._wr_course_done = True
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
        # THE COURSE ARBITER -- same BATON runtime as the schedule path below (it used to be a
        # second, hand-copied implementation, and the copy silently dropped BATON_COLD_HIDDEN).
        # write_tables=False while a SOLO-TURN context is swapped in: the turner runs nb=225 and
        # writing the nb=64 walk ref under it puts the obs ref-block index out of range -> NaN.
        BATON.step(world._wr_baton_host, world._wr_baton, _mode, tt,
                   gate=world._wr_baton_gate, geti=_i,
                   write_tables=not getattr(world, "_wr_in_turn", False))
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
        # WALK_STAND_AT_X (2026-07-10, the stair-summit demo): a POSITION trigger for the schedule's
        # walk->stand handover. The stair walker's pace varies ~3x run-to-run, so a TIMER hands over
        # anywhere between mid-staircase and past the landing's far edge; a raw BATON_COURSE arrival
        # hands over at full stride and the cold stand drops it (0/3 measured). This skips the
        # remaining walk ticks the moment base-x crosses the line -- the handover then flows through
        # the schedule's PROVEN catch path (decel to WALK_DECEL_FLOOR, wait for double support, catch).
        _wsx = _f("WALK_STAND_AT_X", 0.0)
        if _wsx > 0.0 and _mode == "walk" and _rem_seg > 3:
            _qpw = world.solver.mjw_data.qpos.numpy().reshape(-1)
            if float(_qpw[0]) >= _wsx:
                world._wr_sched_skip = getattr(world, "_wr_sched_skip", 0) + _rem_seg
                _t = tt + world._wr_sched_skip; _mode = "walk"; _into = 0
                for md, ntk in world._wr_sched:
                    if ntk <= 0 or _t < ntk:
                        _mode, _into = md, _t; _rem_seg = (ntk - _t) if ntk > 0 else 0
                        break
                    _t -= ntk
                else:
                    _mode, _into = world._wr_sched[-1][0], 0
                if not getattr(world, "_wr_wsx_logged", False):
                    world._wr_wsx_logged = True
                    R._log(world, "WALK_STAND_AT_X: base x %.2f >= %.2f at t=%d -> stand segment (proven catch path)"
                           % (float(_qpw[0]), _wsx, tt))
        # SOLO-TURN early exit (owner 2026-07-08, "walk->turn->walk clean"): a turn segment holds a
        # FIXED duration. Once TURN_TO_DEG reaches 90deg the phase FREEZES mid-turn; walking away from
        # that pose IMMEDIATELY topples (the feet are mid-stride), but HOLDING the frozen pose a short
        # SETTLE window lets the robot stabilize on two feet -- THEN the walk re-enters cleanly (the
        # tw14 lesson). So exit BATON_TURN_SETTLE_TICKS after the turn completes, not the instant it does.
        if _mode == "turn" and getattr(world, "_wr_turn_done", False):
            if not hasattr(world, "_wr_turn_done_t"):
                world._wr_turn_done_t = tt
            # EXIT delay: hold the frozen turn pose this long AFTER completion before advancing to the next
            # segment. The old (walk-next) recipe needed a long window (BATON_TURN_SETTLE_TICKS) to let the
            # feet re-square on two static legs -- but during it the frozen pose UN-ROTATES (legs push the
            # yaw back). With a STAND specialist next, the stand ACTIVELY catches the mid-stride feet, so we
            # exit FAST (BATON_TURN_EXIT_TICKS, small) and the stand grabs the heading at ~90deg before it
            # un-rotates. Falls back to the settle window when unset (walk-next backward compat).
            _exit_after = _i("BATON_TURN_EXIT_TICKS", _i("BATON_TURN_SETTLE_TICKS", 90))
            if (tt - world._wr_turn_done_t) >= _exit_after and _rem_seg > 3:
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
            BATON.step(world._wr_baton_host, world._wr_baton, _mode, tt,
                       gate=world._wr_baton_gate, geti=_i)
            _mode = "walk"; _blend_u = 1.0   # stay on the normal path; BATON modulates its inputs
            # PLACE ARREST (physical box demo): once lowering starts, the achieved two-foot
            # stance owns the legs until the cups have released and withdrawn.  Keep BATON on
            # its normal path so the arm/suction choreography remains live, but make the command
            # truthful: this is a stationary manipulation phase, not a 0.45 m/s carry pressed
            # into the furniture.  The leg targets themselves are pinned below after the grasp
            # state machine has had a chance to capture them.
            if getattr(world, "_wr_place_arrest", False):
                world._wr_cmd = 0.0
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
                tp = _ctl_target_pos(world).numpy()
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
                                           tuple(int(world._wr_legnd[k]) for k in world._wr_kneei)]                             if hasattr(world, "_wr_legnd") and hasattr(world, "_wr_kneei") else []
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
                _ctl_target_pos(world).assign(tp)
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
    # ── GOLDEN-TRAJECTORY DUMP (the cross-machine determinism gate) ──────────────
    # OMNISIM_GOLDEN_DUMP=<out.npz> records (qpos, qvel) at the top of each tick for
    # the first OMNISIM_GOLDEN_TICKS (default 400) ticks, then writes ONE npz and
    # stops recording. Short horizon on purpose: before chaos amplifies, two runs of
    # the same stack must agree to float tolerance -- an early mismatch is a BUG
    # (env/stack/seed), never chaos. Diff two dumps with training/golden_compare.py.
    _gd5 = os.environ.get("OMNISIM_GOLDEN_DUMP")
    if _gd5:
        _g5 = getattr(world, "_wr_gold", None)
        if _g5 is None:
            _g5 = world._wr_gold = {"qp": [], "qv": [], "n": _i("OMNISIM_GOLDEN_TICKS", 400)}
        if len(_g5["qp"]) < _g5["n"]:
            _g5["qp"].append(qp.copy()); _g5["qv"].append(qv.copy())
            if len(_g5["qp"]) == _g5["n"]:
                try:
                    import json as _gj5
                    _meta5 = _gj5.dumps({
                        "policy": os.path.basename(os.environ.get("RES_POLICY", "")),
                        "world": os.path.basename(os.environ.get("WALK_WORLD", "")),
                        "ic_seed": os.environ.get("DEPLOY_IC_SEED", ""),
                        "ghost": os.path.basename(os.environ.get("GHOST_LUT_JSON", ""))})
                    np.savez_compressed(_gd5, qpos=np.stack(_g5["qp"]),
                                        qvel=np.stack(_g5["qv"]), dt=0.016, meta=_meta5)
                    R._log(world, "GOLDEN dump written: %s (%d ticks)" % (_gd5, _g5["n"]))
                except Exception as _ge5:
                    R._log(world, "GOLDEN dump FAILED %r" % (_ge5,))
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
            if getattr(world, "_wr_in_turn", False):
                # ⛔ FRAME BUG, measured 3/3 (2026-07-10): the turn lut's yaw table is ROUTINE-
                # RELATIVE (0->90 from ITS OWN start) -- world-absolute only when the turn begins
                # at heading 0. A corner entered at +85 deg fed the 400 N heading-frame lateral
                # brake an axis ~85 deg wrong (= braking the SUPPORT direction): corner 1 of the
                # shuttle passed every run, corner 2 (entry ~+85) toppled every run. During a
                # solo turn the honest, continuous frame is the robot's OWN heading -- correct at
                # any entry yaw, and TURN-LOOP pass restarts never jump it.
                world._wr_heading_tgt = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            else:
                world._wr_heading_tgt = float(world._wr_seq_yaw[_b5])   # LIVE-SEQ routines: absolute frame, as before
    # Command profiles and finite-sequence ghosts are allowed to rewrite `_wr_cmd` above.  A
    # physical placement arrest has final authority: the observation presented to every policy
    # and the telemetry must both say zero while the achieved double-support leg pose is pinned.
    if getattr(world, "_wr_place_arrest", False):
        world._wr_cmd = 0.0
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
    _av9 = 0.25 * qv[3:6]
    if _i("TURN_OBS_RELATIVE", 0) and getattr(world, "_wr_in_turn", False) and _i("ANGVEL_WORLD_FRAME", 0):
        # ⛔⛔ THE PREMISE OF THIS BLOCK IS FALSE -- measured 2026-07-12. The comment it replaces
        # asserted "the angular-rate obs are WORLD-frame". They are not. A MuJoCo free joint's
        # qvel[3:6] is the angular velocity in the BODY (local) frame -- verified directly against
        # the solver:
        #     yaw=+90 deg, qvel[3:6]=[1,0,0]  ->  the body rotates about WORLD +Y (= about BODY x)
        # so qv[3:6] is ALREADY heading-invariant, and is exactly what the turner trained on (the
        # trainer builds this obs from the same qvel). Rotating it by the entry yaw does not remove
        # a heading dependence -- it CREATES one. It is an identity at heading 0 (which is why every
        # single-turn demo and every straight walk passed, and why this shipped), but at a corner
        # entered near 90 deg, cos(-1.4)=0.17 / sin(-1.4)=-0.99 all but SWAP the roll-rate and
        # pitch-rate channels: the turner is fed pitch rate where it expects roll rate.
        # (The dominant term at that corner is the ATTITUDE-SPRING frame bug -- see the harness note
        # below -- but this one is wrong on its own and is removed with it.)
        # Kept only behind ANGVEL_WORLD_FRAME=1 (an explicit assertion of the false model) so the
        # old behavior is still reachable for A/B; TURN_OBS_RELATIVE is now inert on its own.
        _t09 = getattr(world, "_wr_turn_yaw0", 0.0)
        _c09, _s09 = math.cos(-_t09), math.sin(-_t09)
        _av9 = np.array([_c09 * 0.25 * qv[3] - _s09 * 0.25 * qv[4],
                         _s09 * 0.25 * qv[3] + _c09 * 0.25 * qv[4], 0.25 * qv[5]], np.float32)
    o = np.concatenate([_av9, [gx, gy, gz], [world._wr_cmd, 0.0, 0.0], jpos, 0.05 * jvel,
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
    if getattr(world, "_wr_bare", False):
        ac = np.zeros(ACT_DIM, np.float32)      # BARE GHOST: zero residual -> track the ghost
        if tt % 250 == 0:
            R._log(world, "*** BARE GHOST tick %d: action == 0 (ghost replay, NOT a policy) ***" % tt)
    else:
        with torch.no_grad():
            mean, _std, world._wr_h = world._wr_net.act(torch.from_numpy(o[None, :]), world._wr_h)
        ac = np.clip(mean.numpy()[0], -1, 1)
    if _i("ACT_AUTH", 0) and float(qp[2]) > _f("ACT_AUTH_MIN_Z", 0.5):
        # ⚠️ UPRIGHT-GATED, and that gate is load-bearing. A FALLEN robot still emits actions, and
        # averaging them produces a confident, meaningless number: the shipped flagship dropped at
        # lam=0.2 (base z 0.07 m, roll 1.7 rad) and then lay on the floor twitching for 4800 ticks,
        # which this probe cheerfully reported as "mean|a|=0.292, saturated 1.0%" -- i.e. "healthy
        # corridor". It was measuring a corpse. Same failure verify_motion_legitimacy's L0 gate exists
        # to prevent. Never average a diagnostic over a robot that is not standing.
        #
        # ACT-AUTHORITY (live). Is this SHIPPED skill pinned against its own corridor? The residual is
        # clamped to +-1 and scaled by GHOST_RESIDUAL, so |a|->1 means the policy is asking for more
        # correction than the corridor grants -- and whatever it is denied, the crane absorbs. Measured
        # 2026-07-13 on the free-standing campaign: at corridor 0.12 the policy sat at mean|a|=0.809
        # with 43.6% of action-dims SATURATED; at 0.24 the same policy used mean|a|=0.107 and 0.0%
        # saturated, took 4x LESS deviation, matched the ghost BETTER (gmatch 0.931->0.946), and cut
        # the crane load 4x. A clipped corrector cannot fix an error while it is small, so the error
        # grows until the correction it needs is one the corridor will not grant: the narrow corridor
        # MANUFACTURES the deviation it looks like it is preventing. Every shipped skill trains at
        # GHOST_RESIDUAL 0.10-0.12 -- this is how you find out which of them are starving.
        _am7 = np.abs(mean.numpy()[0])
        world._wr_aa_n = getattr(world, "_wr_aa_n", 0) + 1
        world._wr_aa_mag = getattr(world, "_wr_aa_mag", 0.0) + float(_am7.mean())
        world._wr_aa_sat = getattr(world, "_wr_aa_sat", 0.0) + float((_am7 > 0.95).mean())
        world._wr_aa_max = max(getattr(world, "_wr_aa_max", 0.0), float(_am7.max()))
        _ae7 = _i("ACT_AUTH_EVERY", 250)
        if world._wr_aa_n % _ae7 == 0:
            _n7 = float(world._wr_aa_n)
            R._log(world, "ACT-AUTHORITY(live) n=%d mean|a|=%.3f saturated(|a|>0.95)=%.1f%% max|a|=%.2f "
                          "corridor=%.3f rad"
                   % (world._wr_aa_n, world._wr_aa_mag / _n7, 100.0 * world._wr_aa_sat / _n7,
                      world._wr_aa_max, _f("GHOST_RESIDUAL", 0.0)))
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
        ac = BATON.act(world._wr_baton_host, world._wr_baton, o, ac)
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
    tp = _ctl_target_pos(world).numpy()
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
            # THE DEPLOY CORRIDOR, role-derived (2026-07-13). It used to read
            #     for col in range(12):  ... col in (1,5,7,11) ... col in (2,8) ... col < 6
            # i.e. the G1's leg count, the G1's roll columns, the G1's yaw columns, and
            # "the left leg is the first 6 columns". The TRAINER derives all four from the
            # ghost's joint NAMES (_leg_roles); the deploy did not, so a 10-wide H1 lut would
            # have been corridor-ed as if column 9 were a knee and column 6 a hip. Same law
            # both sides now -- and on the G1 it reproduces the literals exactly (asserted at
            # import), so the flagship does not move.
            _roll_c = world._wr_rolli          # lateral stabilizers: NEVER stance-gated
            _yaw_c = world._wr_yawi            # hip yaw: the turning slack
            _per = world._wr_perleg            # columns per leg -> which side a column is on
            for col in range(world._wr_nleg):
                nd = int(world._wr_legnd[col])
                if 0 <= nd < len(tp):
                    _w = (world._wr_greslat if col in _roll_c
                          else world._wr_gresyaw if col in _yaw_c else world._wr_gres)
                    if getattr(world, "_wr_stight", 0.0) > 0 and col not in _roll_c:
                        _sg5 = float(world._wr_swgate[gb][0 if col < _per else 1])
                        _w *= world._wr_stight + (1.0 - world._wr_stight) * _sg5
                    _lref5 = world._wr_glut[gb][col]
                    _ffd5 = getattr(world, "_wr_ffdq", None)   # GHOST_FF: same centre shift as training
                    if _ffd5 is not None and gb < len(_ffd5):
                        _lref5 = float(_lref5) + float(_ffd5[gb][col])
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
            # ── PHYS-GRASP choreography (2026-07-10, owner: "physically grab the box only with
            # physics") ─────────────────────────────────────────────────────────────────────────
            # The box is a REAL Newton body (g1_box_grasp.omniworld); nothing ever writes its state.
            # This block choreographs the ARM TARGETS (the same deterministic design layer the
            # corridors own): REACH the hands to the box, CLOSE the shoulder rolls past its faces
            # (the arm PD supplies the squeeze -- a friction grip), LIFT, HOLD the squeeze through
            # the whole carry (targets pinned, no residual authority), then LOWER + OPEN at the
            # place stand and let the box settle by contact. The pick/place happen at course
            # segments GR_PICK_SEG / GR_PLACE_SEG. Default OFF -> byte-identical.
            if _i("PHYS_GRASP", 0) and getattr(world, "_wr_armnd", None) and getattr(world, "_wr_elbnd", None) \
                    and getattr(world, "_wr_shryliu", None) and len(world._wr_shrynd) >= 4:
                if getattr(world, "_wr_gr", None) is None:
                    _bxb = None
                    try:   # the box = the LIGHT free-jointed body (subtree ~1 kg; the robot's is ~34 kg)
                        _mjg = world.solver.mj_model
                        import mujoco as _mjn2
                        for _j in range(int(_mjg.njnt)):
                            if int(_mjg.jnt_type[_j]) == int(_mjn2.mjtJoint.mjJNT_FREE):
                                _bb = int(_mjg.jnt_bodyid[_j])
                                if float(_mjg.body_subtreemass[_bb]) < 5.0:
                                    _bxb = _bb
                                    break
                    except Exception as _eg:
                        R._log(world, "PHYS-GRASP: box body resolution failed %r" % (_eg,))
                    world._wr_gr = {"state": "idle", "t0": tt, "box": _bxb,
                                    "cur": None, "goal": None}
                    if _i("GR_DISABLE_ISLAND", 1):
                        # ⭐ THE CROSS-TREE CONTACT FIX (2026-07-10, contact forensics): mjwarp's
                        # ISLAND solver path only activates when ntree > 1 (forward.py: `if
                        # m.ntree > 1 and not disableflags & ISLAND`) -- i.e. exactly when a free
                        # prop exists. Every robot-only world runs single-tree and never touches
                        # it; the first ROBOT<->FREE-BODY contact (palm geom 24 vs box geom 25,
                        # tick-exact in the NaN watch) merges two islands and NaNs the solve.
                        # Disable islands for grasp scenes (MuJoCo's own mjDSBL_ISLAND flag;
                        # monolithic solve, bit-standard). Scoped to PHYS_GRASP runs.
                        try:
                            import mujoco as _mj9
                            for _mm9 in (getattr(world.solver, "mjw_model", None),
                                         getattr(world.solver, "mj_model", None)):
                                if _mm9 is not None:
                                    _mm9.opt.disableflags = int(_mm9.opt.disableflags) | int(_mj9.mjtDisableBit.mjDSBL_ISLAND)
                            R._log(world, "PHYS-GRASP: ISLAND solver disabled (cross-tree contact fix)")
                        except Exception as _ei9:
                            R._log(world, "PHYS-GRASP: island disable failed %r" % (_ei9,))
                    if _i("GR_CONTACT_SCOPE", 1) and _bxb is not None and getattr(world, "_wr_handbids", None):
                        # CONTACT SCOPE (2026-07-10, forensics): with islands off, cross-tree palm-box
                        # contact ran LIVE (box rose + slid under the hands) until the box wedged
                        # 1.3 cm into a FOREARM MESH geom -> explosion. Scope the box to touch ONLY
                        # the palms + non-robot geoms. ⛔ BIT LESSON (first attempt leaked): robot
                        # geoms carry conaffinity 0x7FFFFFFB -- every bit EXCEPT 4 is already set,
                        # so giving the box any contype matches ALL robot geoms. The working surgery
                        # is the inverse: box.contype=8/affinity=0, then CLEAR bit 8 from every
                        # ROBOT geom except the palms (robot bodies = rooted at the heavy free base).
                        # Non-robot geoms (ground, carts) keep bit 8 -> the box still rests on them.
                        # Patched on BOTH the CPU and STEPPED warp models (the FOOT-TORSION lesson).
                        try:
                            _mjg2 = world.solver.mj_model
                            _bg9 = [int(_mjg2.body_geomadr[_bxb]) + i for i in range(int(_mjg2.body_geomnum[_bxb]))]
                            _pg9 = []
                            for _hb2 in world._wr_handbids:
                                _pg9 += [int(_mjg2.body_geomadr[_hb2]) + i for i in range(int(_mjg2.body_geomnum[_hb2]))]
                            # robot bodies: walk each body's ancestry to its root-under-world; the
                            # robot's root is the free-jointed body with the HEAVY subtree (~34 kg)
                            _rroot9 = None
                            for _j in range(int(_mjg2.njnt)):
                                if int(_mjg2.jnt_type[_j]) == 0 and float(_mjg2.body_subtreemass[int(_mjg2.jnt_bodyid[_j])]) > 5.0:
                                    _rroot9 = int(_mjg2.jnt_bodyid[_j]); break
                            def _root9(b):
                                while int(_mjg2.body_parentid[b]) not in (0, b):
                                    b = int(_mjg2.body_parentid[b])
                                return b
                            _rgeoms9 = [g for g in range(int(_mjg2.ngeom))
                                        if _rroot9 is not None and _root9(int(_mjg2.geom_bodyid[g])) == _rroot9
                                        and g not in _pg9]
                            for g in _bg9:
                                _mjg2.geom_contype[g] = 8; _mjg2.geom_conaffinity[g] = 0
                            for g in _rgeoms9:
                                _mjg2.geom_conaffinity[g] = int(_mjg2.geom_conaffinity[g]) & ~8
                            _mw9 = getattr(world.solver, "mjw_model", None)
                            if _mw9 is not None:
                                _ct9 = _mw9.geom_contype.numpy(); _ca9 = _mw9.geom_conaffinity.numpy()
                                for g in _bg9:
                                    _ct9[..., g] = 8; _ca9[..., g] = 0
                                for g in _rgeoms9:
                                    _ca9[..., g] = _ca9[..., g] & ~8
                                _mw9.geom_contype.assign(_ct9); _mw9.geom_conaffinity.assign(_ca9)
                            R._log(world, "PHYS-GRASP contact scope: box %s touches palms %s + world only (bit8 cleared on %d robot geoms)"
                                   % (_bg9, _pg9, len(_rgeoms9)))
                        except Exception as _ec9:
                            R._log(world, "PHYS-GRASP contact scope failed: %r" % (_ec9,))
                    try:   # collider probe: a dropped Box geom (zero extents) = no graspable faces
                        _gn9 = int(_mjg.body_geomnum[_bxb]); _ga9 = int(_mjg.body_geomadr[_bxb])
                        _gsz = [list(np.round(np.array(_mjg.geom_size[_ga9 + i], float), 3)) for i in range(_gn9)]
                        _gty = [int(_mjg.geom_type[_ga9 + i]) for i in range(_gn9)]
                        R._log(world, "PHYS-GRASP box collider: ngeom=%d types=%s sizes=%s" % (_gn9, _gty, _gsz))
                        for _hb9 in (getattr(world, "_wr_handbids", None) or []):
                            _hn9 = int(_mjg.body_geomnum[_hb9]); _ha9 = int(_mjg.body_geomadr[_hb9])
                            _hsz = [list(np.round(np.array(_mjg.geom_size[_ha9 + i], float), 3)) for i in range(_hn9)]
                            _hty = [int(_mjg.geom_type[_ha9 + i]) for i in range(_hn9)]
                            _hcon = [(int(_mjg.geom_contype[_ha9 + i]), int(_mjg.geom_conaffinity[_ha9 + i]))
                                     for i in range(_hn9)]
                            R._log(world, "PHYS-GRASP hand body %d: ngeom=%d types=%s sizes=%s con=%s"
                                   % (_hb9, _hn9, _hty, _hsz, _hcon))
                    except Exception as _eg2:
                        R._log(world, "PHYS-GRASP box collider probe failed: %r" % (_eg2,))
                    R._log(world, "PHYS-GRASP armed: box body=%s (physical grip, zero writes)" % (_bxb,))
                    # ── SUCTION GRIPPER: a finite force coupling. The visual rubber lips must
                    # reach the box-top seal distance before a stiff spring-damper
                    # wrench ties the box's top anchor to the cup (hand centroid), delivered via
                    # control.joint_f (the channel SolverMuJoCo actually reads): +W on the BOX's
                    # free-joint dofs, the REACTION on the robot's base with the proper moment arm
                    # -- so the box is a REAL Newton body that swings and loads the balance. The
                    # lips stay collider-free because cross-tree cup contact is unstable in the
                    # deploy solver; a sub-centimeter seal prevents any attraction from a distance.
                    world._wr_suck = {"on": False, "w": None, "jf0": -1, "qadr": -1, "dadr": -1,
                                      "bid": int(_bxb) if _bxb is not None else -1}
                    if _i("GR_SUCTION", 0) and _bxb is not None:
                        try:
                            for _j in range(int(_mjg.njnt)):
                                if int(_mjg.jnt_type[_j]) == 0 and int(_mjg.jnt_bodyid[_j]) == _bxb:
                                    world._wr_suck["qadr"] = int(_mjg.jnt_qposadr[_j])
                                    world._wr_suck["dadr"] = int(_mjg.jnt_dofadr[_j])
                                    break
                            _njc = world.model.joint_child.numpy().reshape(-1)
                            _nqs = world.model.joint_qd_start.numpy().reshape(-1)
                            _nbm = world.model.body_mass.numpy().reshape(-1)
                            _rjf0 = int(getattr(world, "_wr_jf0", 0))
                            for _j in range(len(_njc)):
                                _cb = int(_njc[_j])
                                if 0 <= _cb < len(_nbm) and abs(float(_nbm[_cb]) - 1.0) < 0.05 \
                                        and int(_nqs[_j]) != _rjf0:
                                    world._wr_suck["jf0"] = int(_nqs[_j])
                                    break
                            R._log(world, "SUCTION armed: box joint_f offset=%d mjw qadr=%d dadr=%d"
                                   % (world._wr_suck["jf0"], world._wr_suck["qadr"], world._wr_suck["dadr"]))
                            if _i("GR_PREPICK_FIXTURE", 0) and world._wr_suck["qadr"] >= 0:
                                # COLD-START CONTACT FIXTURE: the free box/cart contact is the only
                                # cross-tree constraint present before the controller's first useful
                                # tick.  MuJoCo-Warp can nondeterministically diverge on its second
                                # solve.  Hold the box at its table pose with a finite world fixture
                                # wrench and make it palm-only until ENGAGE.  No qpos/qvel writes:
                                # release restores normal dynamic box/cart contact before suction.
                                _qf9 = world._wr_suck["qadr"]
                                world._wr_suck["fixture_home"] = np.array(qp[_qf9:_qf9 + 3], float)
                                world._wr_suck["prefixture"] = True
                                _fg9 = [int(_mjg.body_geomadr[_bxb]) + i
                                        for i in range(int(_mjg.body_geomnum[_bxb]))]
                                world._wr_suck["box_geoms"] = _fg9
                                for _g9 in _fg9:
                                    _mjg.geom_contype[_g9] = 0
                                    _mjg.geom_conaffinity[_g9] = 4  # palm contype only; no cart/world
                                _mwf9 = getattr(world.solver, "mjw_model", None)
                                if _mwf9 is not None:
                                    _ctf9 = _mwf9.geom_contype.numpy(); _caf9 = _mwf9.geom_conaffinity.numpy()
                                    for _g9 in _fg9:
                                        _ctf9[..., _g9] = 0; _caf9[..., _g9] = 4
                                    _mwf9.geom_contype.assign(_ctf9); _mwf9.geom_conaffinity.assign(_caf9)
                                R._log(world, "PREPICK-FIXTURE armed: finite wrench, box/cart contact off until engage")
                        except Exception as _es9:
                            R._log(world, "SUCTION resolution failed: %r" % (_es9,))
                _G = world._wr_gr
                # channel order: [pitchL, pitchR, elbL, elbR, rollL, rollR]
                _gnds = [int(world._wr_armnd[0]), int(world._wr_armnd[1]),
                         int(world._wr_elbnd[0]), int(world._wr_elbnd[1]),
                         int(world._wr_shrynd[0]), int(world._wr_shrynd[2])]
                _ci9 = int(getattr(world, "_wr_course_i", -1))
                _sq = _f("GR_SQUEEZE", 0.12); _op = _f("GR_OPEN", 0.35)
                if _i("GR_SUCTION", 0):
                    # suction mode: NO squeeze -- [-_sq, +_sq] must resolve to the nominal
                    # shoulder-roll hang (+0.15 out left / -0.15 out right), so _sq = -0.15.
                    _sq = -0.15; _op = 0.15
                _POSES = {
                    "reach":    [_f("GR_REACH_PITCH", -0.45)] * 2 + [_f("GR_REACH_ELB", 1.15)] * 2 + [+_op, -_op],
                    # engage gets its OWN (deeper) keyframe: the cups keep DESCENDING through
                    # the engage window until the latch fires -- a true touch by construction,
                    # not a fixed-hover snap (owner: "touch the box, then lift; don't levitate")
                    "engage":   [_f("GR_ENG_PITCH", _f("GR_REACH_PITCH", -0.45))] * 2
                                + [_f("GR_ENG_ELB", _f("GR_REACH_ELB", 1.15))] * 2 + [+_op, -_op],
                    "close":    [_f("GR_REACH_PITCH", -0.45)] * 2 + [_f("GR_REACH_ELB", 1.15)] * 2 + [-_sq, +_sq],
                    "lift":     [_f("GR_HOLD_PITCH", -0.15)] * 2 + [_f("GR_HOLD_ELB", 0.55)] * 2 + [-_sq, +_sq],
                    "hold":     [_f("GR_HOLD_PITCH", -0.15)] * 2 + [_f("GR_HOLD_ELB", 0.55)] * 2 + [-_sq, +_sq],
                    # place_brake keeps the box at the proven hover pose while the gait reaches
                    # its next double-support instant.  Lowering from that instant lets us arrest
                    # the feet without freezing a swing leg.
                    "place_brake": [_f("GR_HOLD_PITCH", -0.15)] * 2 + [_f("GR_HOLD_ELB", 0.55)] * 2 + [-_sq, +_sq],
                    "lower":    [_f("GR_REL_PITCH", -0.50)] * 2 + [_f("GR_REL_ELB", 1.15)] * 2 + [-_sq, +_sq],
                    "open":     [_f("GR_REL_PITCH", -0.50)] * 2 + [_f("GR_REL_ELB", 1.15)] * 2 + [+_op, -_op],
                    # unstick: palms straight UP off the released box before any backward sweep
                    # (the withdraw path dips through the box-top zone -- measured: clipped the
                    # placed box off the stand). Same elbow, shoulders pitch up.
                    "unstick":  [_f("GR_UNSTICK_PITCH", -0.85)] * 2 + [_f("GR_REL_ELB", 1.15)] * 2 + [+_op, -_op],
                    # clearout: arms STAY raised while the robot walks away from the stand --
                    # the walking arm-swing at corridor height drags across the placed box's
                    # top (palm bottoms 0.735 vs box top 0.739, measured: shoved it 26 cm off)
                    "clearout": [_f("GR_UNSTICK_PITCH", -0.85)] * 2 + [_f("GR_REL_ELB", 1.15)] * 2 + [+_op, -_op],
                    "withdraw": [0.0, 0.0, _f("ELBOW_TARGET", 1.6), _f("ELBOW_TARGET", 1.6), 0.15, -0.15],
                }
                _DUR = {"reach": _i("GR_REACH_TICKS", 90), "close": _i("GR_CLOSE_TICKS", 60),
                        "lift": _i("GR_LIFT_TICKS", 90), "lower": _i("GR_LOWER_TICKS", 90),
                        "open": _i("GR_OPEN_TICKS", 50), "withdraw": _i("GR_WD_TICKS", 60)}
                _st = _G["state"]; _el9 = tt - _G["t0"]
                if _st == "idle" and _ci9 == _i("GR_PICK_SEG", 1) and _el9 >= 0 and \
                        (tt - _G["t0"]) >= 0 and getattr(world, "_wr_course_t0", 0) + _i("GR_SETTLE", 60) <= tt:
                    _G["state"] = "reach"; _G["t0"] = tt
                    _G["cur"] = [float(tp[n]) if 0 <= n < len(tp) else 0.0 for n in _gnds]
                elif _st == "reach" and _el9 >= _DUR["reach"]:
                    _G["state"] = "engage" if _i("GR_SUCTION", 0) else "close"; _G["t0"] = tt
                    _G["useng"] = None   # fresh touch-servo state each engage attempt
                    _SKr9 = getattr(world, "_wr_suck", None)
                    if _SKr9 is not None and _SKr9.get("prefixture"):
                        try:
                            _mjr9 = world.solver.mj_model
                            for _g9 in _SKr9.get("box_geoms", []):
                                _mjr9.geom_contype[_g9] = 8; _mjr9.geom_conaffinity[_g9] = 0
                            _mwr9 = getattr(world.solver, "mjw_model", None)
                            if _mwr9 is not None:
                                _ctr9 = _mwr9.geom_contype.numpy(); _car9 = _mwr9.geom_conaffinity.numpy()
                                for _g9 in _SKr9.get("box_geoms", []):
                                    _ctr9[..., _g9] = 8; _car9[..., _g9] = 0
                                _mwr9.geom_contype.assign(_ctr9); _mwr9.geom_conaffinity.assign(_car9)
                            _SKr9["prefixture"] = False; _SKr9["w"] = None
                            R._log(world, "PREPICK-FIXTURE released: normal box/cart + palm contact enabled")
                        except Exception as _er9:
                            R._log(world, "PREPICK-FIXTURE release failed: %r" % (_er9,))
                elif _st == "engage":
                    # The rubber cup lips must reach the box surface before vacuum seals.
                    _SKe = getattr(world, "_wr_suck", None)
                    _near9 = _SKe is not None and _SKe.get("gap", 9.9) < _f("GR_SUCTION_R", 0.08)
                    if _near9:
                        _SKe["on"] = True
                        _G["state"] = "lift"; _G["t0"] = tt
                        R._log(world, "SUCTION ON at t=%d (surface-seal gap %.3f m)" % (tt, _SKe["gap"]))
                    elif _el9 >= 6 * _DUR["close"]:
                        # retry the descent before giving up: the engage-bottom gap wobbles
                        # +/-1 cm run-to-run; re-running reach->engage usually crosses the latch
                        _G["retry"] = _G.get("retry", 0) + 1
                        if _G["retry"] <= _i("GR_ENG_RETRY", 2):
                            _G["state"] = "reach"; _G["t0"] = tt
                            R._log(world, "SUCTION engage timeout (gap %.3f) -- RETRY %d"
                                   % ((_SKe.get("gap", -1.0) if _SKe else -1.0), _G["retry"]))
                        else:
                            _G["state"] = "withdraw"; _G["t0"] = tt
                            R._log(world, "SUCTION engage TIMED OUT (gap %.3f) -- aborting the pick"
                                   % (_SKe.get("gap", -1.0) if _SKe else -1.0))
                elif _st == "close" and (_G.get("sep_ok") or _el9 >= 4 * _DUR["close"]):
                    # advance to LIFT only once BOTH palms are on the box (sep servo confirms);
                    # a one-sided shove on a free 1 kg box is a runaway (measured: 9 mm single-palm
                    # penetration -> explosion). Timeout advances anyway (4x) so it can't deadlock.
                    _G["state"] = "lift"; _G["t0"] = tt
                elif _st == "lift" and _el9 >= _DUR["lift"]:
                    _G["state"] = "hold"; _G["t0"] = tt
                elif _st == "hold" and (
                        (_ci9 == _i("GR_PLACE_SEG", 4) and
                         getattr(world, "_wr_course_t0", 0) + _i("GR_PLACE_SETTLE", 180) <= tt) or
                        (_i("GR_SUCTION", 0) and
                         (getattr(world, "_wr_suck", None) or {}).get("dwell", 0) >= _i("GR_DROP_DWELL", 45))):
                    # two triggers: (a) the course reached the place segment (classic), or
                    # (b) SUCTION PRESS-PLACE -- the held box has hovered GR_DROP_DWELL
                    # consecutive ticks over the drop target.  Do not lower while the carrier
                    # keeps marching: first coast to the next two-foot support instant, then pin
                    # that achieved stance.  Switching to the general stand specialist here is
                    # also wrong (grS10/grS12: it dragged the hanging box ~20 cm west).
                    _G["state"] = "place_brake"; _G["t0"] = tt
                    R._log(world, "PHYS-GRASP place trigger: seg=%d dwell=%d"
                           % (_ci9, (getattr(world, "_wr_suck", None) or {}).get("dwell", 0)))
                elif _st == "place_brake":
                    # The biped gate is the same support model BATON uses for policy handovers.
                    # The timeout is defensive only; at the normal gait rate the next gate is
                    # reached in substantially less than one stride.
                    _ds9 = BATON.biped_double_support(ph, _f("GR_PLACE_DS_TOL", 0.20))
                    if _ds9 or _el9 >= _i("GR_PLACE_DS_MAX", 80):
                        _G["place_legq"] = np.array([
                            float(qp[int(world._wr_qadr[world._wr_liu[_lc9]])])
                            for _lc9 in range(world._wr_nleg)], np.float32)
                        _G["state"] = "lower"; _G["t0"] = tt
                        world._wr_place_arrest = True
                        R._log(world, "PLACE-ARREST engaged at t=%d phase=%.3f ds=%d x=%.3f"
                               % (tt, ph % (2.0 * math.pi), int(_ds9), float(qp[0])))
                elif _st == "lower" and _el9 >= _DUR["lower"]:
                    # release only once the box is OVER the drop target and slow -- cutting
                    # suction mid-swing lands the box wherever the pendulum happens to be
                    # (grS11: timeout release at 0.165 m/s put it 8 cm south, crept off the
                    # edge). Hard timeout at 6x so a bobbing box can never deadlock the course.
                    _SKr = getattr(world, "_wr_suck", None)
                    _bvr = _SKr.get("bv", 0.0) if _SKr else 0.0
                    _gdxr = _f("GR_DROP_X", 9e9)
                    _onr = True
                    if _SKr is not None and _SKr.get("bxy") and _gdxr < 1e8:
                        # release target defaults to the drop/trigger target but can differ:
                        # the box PARKS ~9 cm SE of where it hovers mid-carry (pelvis settle +
                        # lower-pose shift, measured grS11) -- GR_REL_X/Y is the pedestal center
                        _onr = math.hypot(_SKr["bxy"][0] - _f("GR_REL_X", _gdxr),
                                          _SKr["bxy"][1] - _f("GR_REL_Y", _f("GR_DROP_Y", 0.0))) \
                            < _f("GR_REL_RTOL", 0.10)
                    # SEATED gate (2026-07-11): the box must be DOWN on the stand before the
                    # vacuum cuts -- an early gated release at box_z 0.696 (5 cm airborne)
                    # let the pin-down pulse hammer a falling box clean off the slab
                    _seatr = (_SKr.get("bz", 9.9) < _f("GR_REL_ZMAX", 9.9)) if _SKr else True
                    if (_onr and _seatr and _bvr < _f("GR_REL_VMAX", 0.15)) or _el9 >= 6 * _DUR["lower"]:
                        _G["state"] = "open"; _G["t0"] = tt
                        R._log(world, "SUCTION release: box hspeed %.3f m/s on_target=%s seated=%s at t=%d"
                               % (_bvr, _onr, _seatr, tt))
                elif _st == "open" and _el9 >= _DUR["open"]:
                    _G["state"] = "unstick" if _i("GR_SUCTION", 0) else "withdraw"; _G["t0"] = tt
                elif _st == "unstick" and _el9 >= _i("GR_UNSTICK_TICKS", 60):
                    _G["state"] = "withdraw"; _G["t0"] = tt
                elif _st == "withdraw" and _el9 >= _DUR["withdraw"]:
                    # NOTE: the suck dict must be fetched here -- the shared `_SK` local is
                    # bound BELOW this elif chain (a bare `_SK` here NameError'd the whole
                    # deploy tick at this transition and the robot walked off blind)
                    _SKw = getattr(world, "_wr_suck", None)
                    if _i("GR_SUCTION", 0) and _f("GR_DROP_X", 9e9) < 1e8 and \
                            _SKw is not None and not _SKw.get("on"):
                        _G["state"] = "clearout"; _G["t0"] = tt
                        world._wr_place_arrest = False
                        R._log(world, "PLACE-ARREST released at t=%d; clearance locomotion enabled" % tt)
                    else:
                        _G["state"] = "done"; _G["t0"] = tt
                        world._wr_place_arrest = False
                        R._log(world, "PHYS-GRASP: choreography complete (arms back to corridors)")
                elif _st == "clearout":
                    _cd9 = 9.9
                    try:
                        _qpc = np.array(world.solver.mjw_data.qpos.numpy()).reshape(-1)
                        _cd9 = math.hypot(float(_qpc[0]) - _f("GR_DROP_X", 0.0),
                                          float(_qpc[1]) - _f("GR_DROP_Y", 0.0))
                    except Exception:
                        pass
                    if _cd9 > _f("GR_CLEAR_R", 0.85) or _el9 >= _i("GR_CLEAR_TICKS", 2000):
                        _G["state"] = "done"; _G["t0"] = tt
                        R._log(world, "PHYS-GRASP clearout done at t=%d (%.2f m from the stand)" % (tt, _cd9))
                _SK = getattr(world, "_wr_suck", None)
                if _SK is not None and _G["state"] in ("open", "withdraw", "done") and _SK.get("on"):
                    _SK["on"] = False; _SK["w"] = None
                    R._log(world, "SUCTION OFF at t=%d (release) -- held-box SPIN worst: err=%.1f deg "
                                  "om=%.2f rad/s" % (tt, _SK.get("maxrv", -1.0), _SK.get("maxom", -1.0)))
                    # drop the latched relative pose: a re-latch (engage retry) must capture fresh
                    _SK["Rrel"] = None; _SK["Rb_prev"] = None; _SK["Rc_prev"] = None
                if _i("GR_SUCTION", 0) and _SK is not None and _SK["qadr"] >= 0 and \
                        _G["state"] in ("reach", "engage", "lift", "hold", "place_brake", "lower", "open", "unstick"):
                    # per-tick coupling: cup pose from live hand FK, box anchor from its free-joint
                    # qpos; while ON, a clamped spring-damper wrench ties anchor to cup (+W box,
                    # reaction on the base with the cup's moment arm). Applied in the harness
                    # joint_f block (the only channel SolverMuJoCo reads).
                    try:
                        _xp7 = np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
                        # the cup point is the LIVE FK MIDPOINT OF THE CUP LIPS (owner: the box
                        # was visibly floating 24 cm from the tips -- the old anchor hung a fixed
                        # 7 cm below the wrist centroid in WORLD coordinates, nowhere near the
                        # visual cups). The box top now rides exactly at the lips and follows
                        # them rigidly; GR_SUCTION_DROP survives as a small along-axis pad.
                        _xqc7 = np.array(world.solver.mjw_data.xquat.numpy()).reshape(-1, 4)
                        _tipoff7 = np.array([0.214 + _f("GR_SUCTION_DROP", 0.0), 0.0, 0.012])
                        _tips0 = []; _Rms0 = []
                        for _hbc7 in world._wr_handbids:
                            _wc7, _xc7, _yc7, _zc7 = (float(v) for v in _xqc7[_hbc7])
                            _Rmc7 = np.array([
                                [1 - 2*(_yc7*_yc7 + _zc7*_zc7), 2*(_xc7*_yc7 - _wc7*_zc7), 2*(_xc7*_zc7 + _wc7*_yc7)],
                                [2*(_xc7*_yc7 + _wc7*_zc7), 1 - 2*(_xc7*_xc7 + _zc7*_zc7), 2*(_yc7*_zc7 - _wc7*_xc7)],
                                [2*(_xc7*_zc7 - _wc7*_yc7), 2*(_yc7*_zc7 + _wc7*_xc7), 1 - 2*(_xc7*_xc7 + _yc7*_yc7)]])
                            _tips0.append(_xp7[_hbc7] + _Rmc7 @ _tipoff7)
                            _Rms0.append(_Rmc7)
                        _cup7 = 0.5 * (_tips0[0] + _tips0[1])
                        # THE CUP FRAME for the ANGULAR LOCK below. Default: the LEFT cup's frame,
                        # i.e. the box is rigidly attached to one cup and the other rests on it.
                        # The averaged frame (GR_SUCTION_CUPREF=1) is more symmetric but the mean
                        # of two rotations goes ill-conditioned when the hands rotate APART (the
                        # turn shuffle does exactly that): the re-orthonormalized frame then JUMPS,
                        # the lock chases a discontinuous target, saturates, and snaps the box back
                        # -- measured as a 19.9 deg / 12.9 rad/s transient mid-carry. One cup, one
                        # frame, no singularity. _hpang7 logs how far the cups disagree.
                        _hpd7 = _Rms0[0].T @ _Rms0[1]
                        _hpang7 = math.degrees(math.acos(min(1.0, max(-1.0,
                                                0.5 * (float(np.trace(_hpd7)) - 1.0)))))
                        if _i("GR_SUCTION_CUPREF", 0):
                            try:
                                _Us0, _ss0, _Vs0 = np.linalg.svd(0.5 * (_Rms0[0] + _Rms0[1]))
                                _Rcp7 = _Us0 @ _Vs0
                                if np.linalg.det(_Rcp7) < 0:
                                    _Us0[:, 2] *= -1.0; _Rcp7 = _Us0 @ _Vs0
                            except Exception:
                                _Rcp7 = _Rms0[0]
                        else:
                            _Rcp7 = _Rms0[0]
                        if _i("GR_CUP_DBG", 0) and tt % (30 if _i("GR_CUP_SWEEP", 0) else 150) == 0 and \
                                _G["state"] in ("reach", "engage", "lift", "hold"):
                            try:
                                _xq7 = np.array(world.solver.mjw_data.xquat.numpy()).reshape(-1, 4)
                                _tips7 = []
                                for _hb7 in world._wr_handbids:
                                    _w7, _x7, _y7, _z7 = (float(v) for v in _xq7[_hb7])
                                    _Rm7 = np.array([
                                        [1 - 2*(_y7*_y7 + _z7*_z7), 2*(_x7*_y7 - _w7*_z7), 2*(_x7*_z7 + _w7*_y7)],
                                        [2*(_x7*_y7 + _w7*_z7), 1 - 2*(_x7*_x7 + _z7*_z7), 2*(_y7*_z7 - _w7*_x7)],
                                        [2*(_x7*_z7 - _w7*_y7), 2*(_y7*_z7 + _w7*_x7), 1 - 2*(_x7*_x7 + _y7*_y7)]])
                                    _tips7.append(_xp7[_hb7] + _Rm7 @ np.array([0.214, 0.0, 0.012]))
                                _tm7 = 0.5 * (_tips7[0] + _tips7[1])
                                _bt7 = np.array([float(qp[_SK["qadr"]]), float(qp[_SK["qadr"] + 1]),
                                                 float(qp[_SK["qadr"] + 2]) + 0.09])
                                R._log(world, "CUPTIP t=%d st=%s tipmid=%s boxtop=%s tip2box=%.3f dz=%.3f dxy=%.3f" %
                                       (tt, _G["state"], np.round(_tm7, 3).tolist(), np.round(_bt7, 3).tolist(),
                                        float(np.linalg.norm(_tm7 - _bt7)), float(_tm7[2] - _bt7[2]),
                                        float(np.hypot(_tm7[0] - _bt7[0], _tm7[1] - _bt7[1]))))
                            except Exception as _ed7:
                                R._log(world, "CUPDBG err %r" % (_ed7,))
                        # the anchor is the TOUCH POINT: the nearest point on the box's TOP FACE
                        # to the cup lips (a real suction cup grabs wherever it touches, not at
                        # the geometric center -- the stance heading scatters the lips +/-10 cm
                        # in y and the z-servo cannot correct y). Off-center grabs get the true
                        # moment arm + a small righting torque below.
                        _bcx7 = float(qp[_SK["qadr"]]); _bcy7 = float(qp[_SK["qadr"] + 1])
                        _anc7 = np.array([
                            min(max(_cup7[0], _bcx7 - _f("GR_PAD_X", 0.07)), _bcx7 + _f("GR_PAD_X", 0.07)),
                            min(max(_cup7[1], _bcy7 - _f("GR_PAD_Y", 0.10)), _bcy7 + _f("GR_PAD_Y", 0.10)),
                            float(qp[_SK["qadr"] + 2]) + 0.09])
                        _SK["grab_r"] = (_anc7[0] - _bcx7, _anc7[1] - _bcy7)
                        _SK["gap"] = float(np.linalg.norm(_cup7 - _anc7))
                        _SK["bxy"] = (_bcx7, _bcy7)
                        _SK["bz"] = float(_anc7[2]) - 0.09
                        _SK["tipz"] = float(_cup7[2]); _SK["ancz"] = float(_anc7[2])
                        _SK["bv"] = math.hypot(float(qv[_SK["dadr"]]), float(qv[_SK["dadr"] + 1]))
                        # dwell counter for the hover-drop arrival: consecutive ticks the box has
                        # spent inside GR_DROP_R of the drop target (first-touch of the radius
                        # mid-swing must NOT trigger the place -- measured 0.4 m scatter, grS10)
                        _gdx8 = _f("GR_DROP_X", 9e9)
                        if _gdx8 < 1e8:
                            _dd8 = math.hypot(_bcx7 - _gdx8, _bcy7 - _f("GR_DROP_Y", 0.0))
                            _SK["dwell"] = _SK.get("dwell", 0) + 1 if _dd8 < _f("GR_DROP_R", 0.13) else 0
                        _cupv7 = np.zeros(3) if _SK.get("cup_prev") is None \
                            else (_cup7 - _SK["cup_prev"]) / 0.016
                        _SK["cup_prev"] = _cup7.copy()
                        if _SK["on"]:
                            # DYNAMICS USE THE BOX TOP-CENTER, not the touch point: the clamped
                            # touch-point anchor is for the LATCH GATE only. Driving the force
                            # at an offset grab point (moment arm and/or righting torque) spun
                            # the 1 kg box into solver divergence at the turn TWICE -- 5-10 N*m
                            # transients on ~0.009 kg m^2 at 62 Hz is a stiff discrete system.
                            # The centered pull is the proven-stable path (3 clean full courses):
                            # after an off-center latch the box just slides under the cups.
                            _ancd7 = np.array([_bcx7, _bcy7, float(qp[_SK["qadr"] + 2]) + 0.09])
                            _bv7 = np.array([float(qv[_SK["dadr"] + i]) for i in range(3)])
                            _F7 = _f("GR_SUCTION_KP", 900.0) * (_cup7 - _ancd7) \
                                + _f("GR_SUCTION_KD", 60.0) * (_cupv7 - _bv7)
                            if _G["state"] == "lower":
                                # SOFT FULL-VECTOR set-down (the tip anchor changed the rules):
                                # - the old vertical-only descent was for the CENTROID anchor's
                                #   5-9 cm lateral offset (tilt torque); the TIP anchor rides
                                #   ~1 cm off the box, so the xy tether must STAY -- cutting it
                                #   let residual press-drift slide the box off the slab corner.
                                # - the whole vector is capped near 2x box weight (a full-clamp
                                #   pull slammed the box into the slab / bounce-cycled it away).
                                # - once SEATED: pure velocity brake only (damping cannot drag,
                                #   it only stills the box; a spring would slide it on the top).
                                _lfm7 = _f("GR_LOW_FMAX", 18.0)
                                if (float(_anc7[2]) - 0.09) < _f("GR_REL_ZMAX", 9.9):
                                    _F7 = -_f("GR_SUCTION_KD", 60.0) * _bv7
                                _n7 = float(np.linalg.norm(_F7))
                                if _n7 > _lfm7:
                                    _F7 *= _lfm7 / _n7
                            _Fm7 = float(np.linalg.norm(_F7)); _fc7 = _f("GR_SUCTION_FMAX", 80.0)
                            if _Fm7 > _fc7:
                                _F7 *= _fc7 / _Fm7
                            # moment arm of the pull applied at the box TOP (the force acts on the
                            # COM, so the offset to the anchor is an explicit torque)
                            _T7 = np.cross(np.array([0.0, 0.0, 0.09]), _F7)
                            # ── ANGULAR LOCK (owner 2026-07-12: "it cannot be rotating around while
                            # the robot is walking"). The coupling used to be a POINT force: nothing
                            # constrained the box's ORIENTATION, and the moment-arm torque above has
                            # an identically-zero z-component (cross of a +z arm), so box YAW was a
                            # free, undamped dof -- it drifted with every stride. A real suction cup
                            # is a rigid patch: it fixes the box IN the tool frame. So we latch the
                            # box's relative rotation at SUCTION ON and hold it with a torque
                            # spring-damper about the cup frame:
                            #     R_target = R_cup @ R_rel      (R_rel captured at latch)
                            #     tau = KPA * log(R_target R_box^T) + KDA * (om_cup - om_box)
                            # Damping is on the RELATIVE rate (the box must FOLLOW the cups when the
                            # robot turns, not resist them) -- the same structure as the linear term.
                            # Gains are sized to the 1 kg box (I ~ 0.011 kg m^2): KPA=10/KDA=0.7 is
                            # ~critically damped, and KDA*dt/I ~ 1.0 < 2 keeps the 62.5 Hz ZOH stable.
                            # TMAX=3 N*m stays well under the 5-10 N*m transients that spun the box
                            # into solver divergence when an off-center moment arm was tried.
                            _kpa7 = _f("GR_SUCTION_KPA", 10.0)
                            _Rbx7 = None
                            if int(_SK.get("bid", -1)) >= 0:
                                _wb7, _xb7, _yb7, _zb7 = (float(v) for v in _xqc7[int(_SK["bid"])])
                                _Rbx7 = np.array([
                                    [1 - 2*(_yb7*_yb7 + _zb7*_zb7), 2*(_xb7*_yb7 - _wb7*_zb7), 2*(_xb7*_zb7 + _wb7*_yb7)],
                                    [2*(_xb7*_yb7 + _wb7*_zb7), 1 - 2*(_xb7*_xb7 + _zb7*_zb7), 2*(_yb7*_zb7 - _wb7*_xb7)],
                                    [2*(_xb7*_zb7 - _wb7*_yb7), 2*(_yb7*_zb7 + _wb7*_xb7), 1 - 2*(_xb7*_xb7 + _yb7*_yb7)]])
                                if _SK.get("Rrel") is None:          # latch the relative pose ONCE
                                    _SK["Rrel"] = _Rcp7.T @ _Rbx7; _SK["lat_t"] = tt
                                    R._log(world, "SUCTION angular lock latched (KPA=%.1f KDA=%.2f TMAX=%.1f)"
                                           % (_kpa7, _f("GR_SUCTION_KDA", 0.7), _f("GR_SUCTION_TMAX", 3.0)))
                                _Erot7 = (_Rcp7 @ _SK["Rrel"]) @ _Rbx7.T     # world-frame error rot
                                _cth7 = min(1.0, max(-1.0, 0.5 * (float(np.trace(_Erot7)) - 1.0)))
                                _ang7 = math.acos(_cth7)
                                _vee7 = np.array([_Erot7[2, 1] - _Erot7[1, 2],
                                                  _Erot7[0, 2] - _Erot7[2, 0],
                                                  _Erot7[1, 0] - _Erot7[0, 1]])
                                _sth7 = math.sin(_ang7)
                                _rv7 = 0.5 * _vee7 if _ang7 < 1e-6 else (_ang7 / (2.0 * _sth7)) * _vee7
                                # world angular rates by finite difference of the rotation matrices
                                # (convention-free: no assumption about the free-joint qd basis)
                                _omB7 = np.zeros(3); _omC7 = np.zeros(3)
                                if _SK.get("Rb_prev") is not None:
                                    _dB7 = _Rbx7 @ _SK["Rb_prev"].T
                                    _omB7 = np.array([_dB7[2, 1] - _dB7[1, 2], _dB7[0, 2] - _dB7[2, 0],
                                                      _dB7[1, 0] - _dB7[0, 1]]) / (2.0 * 0.016)
                                    _dC7 = _Rcp7 @ _SK["Rc_prev"].T
                                    _omC7 = np.array([_dC7[2, 1] - _dC7[1, 2], _dC7[0, 2] - _dC7[2, 0],
                                                      _dC7[1, 0] - _dC7[0, 1]]) / (2.0 * 0.016)
                                _SK["Rb_prev"] = _Rbx7.copy(); _SK["Rc_prev"] = _Rcp7.copy()
                                # SOFT ENGAGE: ramp the lock in over ~1 s. A step-on lock SNAPS the
                                # box into alignment (measured 19.9 deg / 12.9 rad/s transient at
                                # latch) -- a real cup pulls the part in as the vacuum builds, and
                                # violent transients on this 1 kg box are what diverged the solver
                                # in the off-center-moment experiments.
                                _kr7 = 1.0
                                if _i("GR_SUCTION_ARAMP", 60) > 0 and _SK.get("lat_t") is not None:
                                    _kr7 = min(1.0, (tt - int(_SK["lat_t"])) / float(_i("GR_SUCTION_ARAMP", 60)))
                                _Ta7 = _kr7 * (_kpa7 * _rv7 + _f("GR_SUCTION_KDA", 0.7) * (_omC7 - _omB7))
                                if _G["state"] == "lower" and (float(_anc7[2]) - 0.09) < _f("GR_REL_ZMAX", 9.9):
                                    # SEATED: pure angular brake -- the box must settle FLAT on the
                                    # slab, not be twisted into the cups' pose while it rests on it
                                    _Ta7 = -_f("GR_SUCTION_KDA", 0.7) * _omB7
                                _tm7a = float(np.linalg.norm(_Ta7)); _tc7 = _f("GR_SUCTION_TMAX", 3.0)
                                if _tm7a > _tc7:
                                    _Ta7 *= _tc7 / _tm7a
                                if _kpa7 <= 0.0:
                                    # KPA=0 -> torque OFF but the telemetry above still runs: this is
                                    # the honest BASELINE (what a point-force-only coupling does)
                                    _Ta7 = np.zeros(3)
                                _T7 = _T7 + _Ta7
                                # spin telemetry (owner-facing): worst orientation error and worst
                                # spin rate seen while the box is held. GR_SUCTION_KPA=0 measures
                                # the SAME numbers with the lock OFF -> honest before/after.
                                _SK["maxrv"] = max(_SK.get("maxrv", 0.0), math.degrees(_ang7))
                                _SK["maxom"] = max(_SK.get("maxom", 0.0), float(np.linalg.norm(_omB7)))
                                if _i("GR_SPIN_DBG", 0) and tt % 150 == 0 and _G["state"] in ("hold", "lower"):
                                    R._log(world, "SPIN t=%d st=%s err=%.1fdeg om=%.2frad/s tau=%.2f cups=%.1fdeg "
                                                  "(worst err=%.1f om=%.2f)"
                                           % (tt, _G["state"], math.degrees(_ang7), float(np.linalg.norm(_omB7)),
                                              float(np.linalg.norm(_Ta7)), _hpang7, _SK["maxrv"], _SK["maxom"]))
                            _pel7 = np.array([float(qp[0]), float(qp[1]), float(qp[2])])
                            # reaction on the base: the cups feel the equal-and-opposite wrench
                            _SK["w"] = (_F7, _T7, -_F7, np.cross(_cup7 - _pel7, -_F7) - (_T7 - np.cross(
                                np.array([0.0, 0.0, 0.09]), _F7)))
                        elif _G["state"] == "open" and \
                                (float(_anc7[2]) - 0.09) < _f("GR_PIN_ZMAX", _f("GR_REL_ZMAX", 9.9) + 0.01):
                            # PIN-DOWN pulse (2026-07-11): right after the vacuum releases, press
                            # the box gently onto the stand (down on the box, reaction up on the
                            # base). The placed box keeps a residual micro-velocity the solver
                            # never kills -- a slow ~0.5 mm/s creep that can walk it off the
                            # stand minutes later (measured twice). The extra normal load lets
                            # friction lock it while the box is still under the cups. Applied
                            # ONLY while the box is seated -- pinning an airborne box slams it.
                            _pf7 = _f("GR_PIN_F", 30.0)
                            _SK["w"] = (np.array([0.0, 0.0, -_pf7]), np.zeros(3),
                                        np.array([0.0, 0.0, _pf7]), np.zeros(3))
                        else:
                            _SK["w"] = None
                    except Exception:
                        _SK["w"] = None
                if _G["state"] != _st:
                    _hz9 = _sep9 = _bz9 = -1.0
                    try:
                        _xp9 = np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
                        if getattr(world, "_wr_handbids", None) is not None:
                            _sep9 = float(np.linalg.norm(_xp9[world._wr_handbids[0]] - _xp9[world._wr_handbids[1]]))
                            _hz9 = float(0.5 * (_xp9[world._wr_handbids[0]][2] + _xp9[world._wr_handbids[1]][2]))
                        if _G["box"] is not None:
                            _bz9 = float(_xp9[_G["box"]][2])
                    except Exception:
                        pass
                    _bxy9 = (-1.0, -1.0)
                    try:
                        if _G["box"] is not None:
                            _bxy9 = (float(_xp9[_G["box"]][0]), float(_xp9[_G["box"]][1]))
                    except Exception:
                        pass
                    R._log(world, "PHYS-GRASP -> %s at t=%d (hand_z=%.3f sep=%.3f box=%.2f,%.2f,%.3f)"
                           % (_G["state"], tt, _hz9, _sep9, _bxy9[0], _bxy9[1], _bz9))
                if _i("GR_NAN_WATCH", 0) and tt > 500 and not getattr(world, "_wr_nanreported", False):
                    # first-NaN contact forensics: keep last tick's contact table; on the first
                    # non-finite qvel, dump both tables (pair geoms, dist, ncon) to the rl log.
                    try:
                        _dw = world.solver.mjw_data
                        _finite9 = bool(np.isfinite(qv[:6]).all())
                        _nc9 = int(np.array(_dw.nacon.numpy()).reshape(-1)[0]) if hasattr(_dw, "nacon") else -1
                        _cg9 = None
                        if _nc9 > 0 and hasattr(_dw, "contact"):
                            _cgeom = np.array(_dw.contact.geom.numpy()).reshape(-1, 2)[:_nc9]
                            _cdist = np.array(_dw.contact.dist.numpy()).reshape(-1)[:_nc9]
                            _cg9 = [(int(a), int(b), round(float(d), 4)) for (a, b), d in zip(_cgeom, _cdist)]
                        if _finite9:
                            world._wr_lastcon = (tt, _nc9, _cg9)
                        else:
                            world._wr_nanreported = True
                            R._log(world, "GR-NAN-WATCH: FIRST NON-FINITE at t=%d state=%s | ncon=%d contacts=%s"
                                   % (tt, _G["state"], _nc9, _cg9))
                            _lc9 = getattr(world, "_wr_lastcon", None)
                            R._log(world, "GR-NAN-WATCH: last finite tick=%s ncon=%s contacts=%s"
                                   % (_lc9[0] if _lc9 else None, _lc9[1] if _lc9 else None,
                                      _lc9[2] if _lc9 else None))
                    except Exception as _ew9:
                        world._wr_nanreported = True
                        R._log(world, "GR-NAN-WATCH failed: %r" % (_ew9,))
                if _G["state"] in _POSES and _G["cur"] is not None:
                    _gl9 = list(_POSES[_G["state"]])
                    if _i("GR_CUP_SWEEP", 0) and _G["state"] == "reach":
                        # calibration sweep: ramp pitch/elbow through a grid while CUPTIP logs
                        # the FK tip positions -- one run maps (pose -> cup tips) empirically
                        _ph9s = min(1.0, (tt - _G["t0"]) / 900.0)
                        _gl9[0] = _gl9[1] = -0.55 - 0.75 * _ph9s
                        _gl9[2] = _gl9[3] = 1.30 - 0.95 * _ph9s
                    if _i("GR_ENG_SERVO", 1) and _i("GR_SUCTION", 0) and _G["state"] == "engage":
                        # TOUCH SERVO (2026-07-11): fixed engage keyframes miss by up to 10 cm
                        # run-to-run (stance varies +/-3 cm and the tip position is hyper-
                        # sensitive near this arm configuration -- 0.1 rad moved the tips
                        # 44 cm). Walk the arm along the CALIBRATED sweep curve u -> pose,
                        # driven by live tip-vs-box-top z error, until the cups meet the box.
                        _SKu = getattr(world, "_wr_suck", None)
                        _u9 = _G.get("useng")
                        if _u9 is None:
                            _u9 = _f("GR_ENG_U0", 0.28)
                        if _SKu is not None and _SKu.get("tipz") is not None:
                            _u9 += max(-0.002, min(0.002,
                                       0.08 * (_SKu["ancz"] + 0.003 - _SKu["tipz"])))
                        _u9 = max(0.05, min(0.75, _u9))
                        _G["useng"] = _u9
                        _gl9[0] = _gl9[1] = -0.55 - 0.75 * _u9
                        _gl9[2] = _gl9[3] = 1.30 - 0.95 * _u9
                    _rt9 = _f("GR_RATE", 0.010)
                    # SEP SERVO (2026-07-10): at forward arm pitch, shoulder roll sweeps the hands
                    # mostly VERTICALLY -- fixed roll keyframes cannot reliably close the lateral
                    # gap (measured: sep stuck at 0.44, one palm engaged, runaway shove). Drive the
                    # roll targets by feedback on the MEASURED hand separation instead; the grip
                    # force then comes from the servo pushing past contact, balanced on both palms.
                    if _G["state"] in ("close", "lift", "hold", "place_brake", "lower"):
                        try:
                            _xp8 = np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
                            _sep8 = float(np.linalg.norm(_xp8[world._wr_handbids[0]] - _xp8[world._wr_handbids[1]]))
                            _tsep8 = _f("GR_SEP_TGT", 0.235)
                            _e8 = _sep8 - _tsep8
                            _G["sep_ok"] = bool(_sep8 < _f("GR_SEP_GO", 0.28))
                            _adj8 = _G.get("roll_adj", 0.0)
                            _adj8 += max(-_f("GR_SEP_K", 0.004), min(_f("GR_SEP_K", 0.004), 0.05 * _e8))
                            _adj8 = max(0.0, min(_f("GR_SEP_ADJ_MAX", 0.45), _adj8))
                            _G["roll_adj"] = _adj8
                            _gl9[4] = max(-0.5, _gl9[4] - _adj8)   # left roll further inward
                            _gl9[5] = min(0.5, _gl9[5] + _adj8)    # right roll further inward
                        except Exception:
                            pass
                    for k in range(6):
                        _d9g = _gl9[k] - _G["cur"][k]
                        _G["cur"][k] += max(-_rt9, min(_rt9, _d9g))
                        if 0 <= _gnds[k] < len(tp):
                            tp[_gnds[k]] = _G["cur"][k]
                    # yaw channels pinned to 0 while the choreography owns the arms
                    for _kk in (1, 3):
                        _ndy = int(world._wr_shrynd[_kk])
                        if 0 <= _ndy < len(tp):
                            tp[_ndy] = 0.0
                # The place arrest owns only the legs.  Arms above keep following their physical
                # lower/open/unstick/withdraw keyframes, while the waist and harness remain live.
                # Holding the achieved q (rather than a nominal stand) removes the foot-drag that
                # made the old carry->stand experiment pull the suspended box off target.
                _plq9 = _G.get("place_legq")
                if getattr(world, "_wr_place_arrest", False) and _plq9 is not None:
                    for _lc9 in range(min(world._wr_nleg, len(_plq9))):
                        _nd9 = int(world._wr_legnd[_lc9])
                        if 0 <= _nd9 < len(tp):
                            tp[_nd9] = float(_plq9[_lc9])
                _SKf9 = getattr(world, "_wr_suck", None)
                if _SKf9 is not None and _SKf9.get("prefixture") and _SKf9.get("fixture_home") is not None:
                    _qaf9, _daf9 = int(_SKf9["qadr"]), int(_SKf9["dadr"])
                    _pf9 = np.array([float(qp[_qaf9 + i]) for i in range(3)])
                    _vf9 = np.array([float(qv[_daf9 + i]) for i in range(3)])
                    _Ff9 = (_f("GR_PREPICK_KP", 1200.0) * (_SKf9["fixture_home"] - _pf9)
                            - _f("GR_PREPICK_KD", 80.0) * _vf9 + np.array([0.0, 0.0, 9.81]))
                    _cff9 = _f("GR_PREPICK_FMAX", 40.0)
                    _nff9 = float(np.linalg.norm(_Ff9))
                    if _nff9 > _cff9:
                        _Ff9 *= _cff9 / _nff9
                    _z3f9 = np.zeros(3)
                    _SKf9["w"] = (_Ff9, _z3f9, _z3f9, _z3f9)  # fixture reacts on world, not robot
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
    if getattr(world, "_wr_hlam", 0.0) <= 0 and _i("CRANE_LOG", 0) and tt % _i("CRANE_LOG_EVERY", 4) == 0:
        # FREE-STANDING (lam=0): the harness block below is skipped entirely, so no CRANELOG is ever
        # emitted -- and verify_motion_legitimacy reads "no CRANELOG lines" as L1 FAIL. That means the
        # ruler scores a robot with NO CRANE AT ALL as a crane-cheat. Assert the zero wrench POSITIVELY
        # so "there was no crane" is a recorded fact and not an absence of evidence (2026-07-13).
        R._log(world, "CRANELOG t=%d fx=0.0 fy=0.0 fz=0.0 tx=0.0 ty=0.0 tz=0.0 bx=%.3f bz=%.3f"
               % (tt, float(qp[0]), float(qp[2])))
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
            # HARNESS_ATT_PHASE (2026-07-10, the deterministic live veer): the live gait LAGS the
            # ghost clock (contact timing), so the clock-indexed sway leads the actual steps and the
            # sway-vs-step misalignment precesses the robot (pure-PD veers -0.76 rad by x~1.0,
            # bit-repeatable; sway off -> veer gone but so is the weight-shift propulsion). This
            # offset (in LUT bins, may be negative) re-aligns the sway target with the live gait.
            _gb6 = int(world._wr_phase / (2.0 * math.pi) * _nb6 + _f("HARNESS_ATT_PHASE", 0.0)) % _nb6
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
        # TURN ROOT REFERENCE FOLLOWER.  A turn-in-place still needs small horizontal pelvis shifts
        # over the stance foot.  Follow the ghost's *relative* xy path instead of clamping the pelvis
        # to one point (the latter fights weight transfer and was measured to topple the robot).
        # This is balance-harness assistance only: it supplies bounded horizontal force and no yaw
        # torque; rotation still comes from the feet (the turn branch below keeps wtz=0).
        _trcarry5 = bool((getattr(world, "_wr_suck", None) or {}).get("on", False))
        _trk5 = (_f("TURN_CARRY_ROOT_TRACK_KP", _f("TURN_ROOT_TRACK_KP", 0.0)) if _trcarry5
                 else _f("TURN_ROOT_TRACK_KP", 0.0))
        _tctx5 = getattr(world, "_wr_active_turnctx", None)
        _txy05 = getattr(world, "_wr_turn_xy0", None)
        _troot5 = _tctx5.get("seq_root") if isinstance(_tctx5, dict) else None
        if _trk5 > 0.0 and getattr(world, "_wr_turn_active", False) and not getattr(world, "_wr_turn_done", False) \
                and _txy05 is not None and _troot5 is not None and len(_troot5) > 1:
            _tnb5 = len(_troot5)
            _tb5 = int((world._wr_phase % (2.0 * math.pi)) / (2.0 * math.pi) * _tnb5) % _tnb5
            _tdx5 = float(_troot5[_tb5][0] - _troot5[0][0])
            _tdy5 = float(_troot5[_tb5][1] - _troot5[0][1])
            _ty05 = float(getattr(world, "_wr_turn_yaw0", 0.0))
            _tc5, _ts5 = math.cos(_ty05), math.sin(_ty05)
            _txref5 = _txy05[0] + _tc5 * _tdx5 - _ts5 * _tdy5
            _tyref5 = _txy05[1] + _ts5 * _tdx5 + _tc5 * _tdy5
            _td5 = (_f("TURN_CARRY_ROOT_TRACK_KD", _f("TURN_ROOT_TRACK_KD", 25.0)) if _trcarry5
                    else _f("TURN_ROOT_TRACK_KD", 25.0))
            _tfc5 = (_f("TURN_CARRY_ROOT_TRACK_FCAP", _f("TURN_ROOT_TRACK_FCAP", 180.0)) if _trcarry5
                     else _f("TURN_ROOT_TRACK_FCAP", 180.0))
            _fx5 += _lam5 * (-_trk5 * (float(qp[0]) - _txref5) - _td5 * float(qv[0]))
            _fy5 += _lam5 * (-_trk5 * (float(qp[1]) - _tyref5) - _td5 * float(qv[1]))
            _fx5 = max(-_tfc5, min(_tfc5, _fx5)); _fy5 = max(-_tfc5, min(_tfc5, _fy5))
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
        # STATIONARY POSITION HOLD (2026-07-08): hold the pelvis at its entry xy through every STAND and
        # TURN segment (the robot is meant to be in place there). The crane has NO forward brake -- only
        # the FY *lateral* catch above -- so when a fast walk hands off to a stand, the forward momentum
        # COASTS several metres before the stand settles ("sliding rapidly while standing", owner report:
        # measured +4m of pre-turn slide at vx~0.8, plus the turn's residual push + post-decel coast).
        # ⛔ Do NOT gate on `_mode == "stand"`: by the time the crane block runs, _mode has been reassigned
        # and only ever reads "walk"/"turn" (probe-verified -- the stand segments never surface here). The
        # honest stand signal is the BATON's TARGET specialist, which is "stand" from the walk->stand
        # boundary until leg 2 re-enters. One xy is captured on the FIRST stationary tick and held across
        # the whole stand->turn->stand block; the achieved turn yaw is latched separately at done.
        # ⛔ Do NOT hold DURING the footwork turn (before it completes): the turn specialist was trained
        # under the free lateral-catch harness, so a hard pelvis lock is out-of-distribution -- it fights
        # the COM-over-stance-foot shifts and the turn over-spins and topples (measured: peak 180deg, fall).
        # The turn is already naturally in-place (x drift <0.05m), so it needs no help. Hold only: (a) a
        # STAND phase, and (b) a COMPLETED turn being held (the original post-decel coast fix).
        _bt5 = getattr(world, "_wr_baton", None)
        _bstand5 = (_bt5 is not None) and _bt5.target == "stand"
        _tact5 = getattr(world, "_wr_turn_active", False)
        # TURN_HOLD_LOCK=0 (2026-07-10): skip the completed-turn xy lock (and the yaw latch that rides
        # on it) for the feet-together seq turner -- it pivots in place (x drift <0.05 m, no post-decel
        # slide to fix) and the clamp's setpoint, captured on the first done tick, is the OOD hard lock
        # the 2026-07-08 note above warns about: measured, the settled end-hold stays perfect for ~1 s
        # then the clamp winds it up into a spin+fall. The STAND-segment hold (the pre-turn slide fix)
        # is unaffected. Default 1 = exact prior behavior (turn_solo's continuous turner needs the lock).
        if (_bstand5 and not _tact5) or (_tact5 and getattr(world, "_wr_turn_done", False) and _i("TURN_HOLD_LOCK", 1)):
            if getattr(world, "_wr_turnhold_xy", None) is None:
                world._wr_turnhold_xy = (float(qp[0]), float(qp[1])); world._wr_turnhold_yaw = None
            if getattr(world, "_wr_turn_done", False) and getattr(world, "_wr_turnhold_yaw", None) is None:
                world._wr_turnhold_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))   # achieved turn yaw
            _khT = _f("TURN_HOLD_KHOLD", 450.0); _dhT = _f("TURN_HOLD_DHOLD", 90.0)
            _fx5 = max(-700.0, min(700.0, _lam5 * (-_khT * (float(qp[0]) - world._wr_turnhold_xy[0]) - _dhT * float(qv[0]))))
            _fy5 = max(-700.0, min(700.0, _lam5 * (-_khT * (float(qp[1]) - world._wr_turnhold_xy[1]) - _dhT * float(qv[1]))))
        else:
            world._wr_turnhold_xy = None; world._wr_turnhold_yaw = None
        _fz5 = max(0.0, min(700.0, _lam5 * (_f("HARNESS_KZ", 2000.0) * (_f("HARNESS_Z0", 0.72) - float(qp[2]))
                                            - _f("HARNESS_DZ", 150.0) * float(qv[2]))))
        # ── THE ATTITUDE SPRING, AND THE FRAME BUG THAT MADE EVERY OFF-AXIS TURN FAIL ─────────
        # (2026-07-12; this is the second root cause of the multi-turn BATON failure, and it is
        # why a course with a corner at heading ~90 deg fell while every shipped one-turn demo,
        # which corners at heading ~0, passed.)
        #
        # THE THREE FRAMES, each verified rather than assumed:
        #  1. _r5 / _p5 are BODY roll/pitch (yaw-first Euler)               -- heading-invariant.
        #  2. qv[3:6] is the free joint's angular velocity in the BODY frame -- NOT world. Verified
        #     against the solver: set qvel[3:6]=[1,0,0] at yaw=+90 deg and the body rotates about
        #     WORLD +Y, i.e. about its OWN x axis. (MuJoCo free-joint convention.)
        #  3. The harness torque channel is WORLD. Newton routes a FREE joint's control.joint_f to
        #     MuJoCo's xfrc_applied, not qfrc_applied -- see the engine's own
        #     newton/_src/solvers/mujoco/kernels.py::apply_mjc_qfrc_kernel:
        #         "Free/DISTANCE joint forces are routed via xfrc_applied ... skip them here."
        #     and xfrc_applied is a CARTESIAN WORLD wrench at the body COM.
        #
        # So the PD must be computed in the BODY frame (1 + 2) and its output ROTATED INTO WORLD (3).
        # Neither of the two previous forms did that:
        #  * the original spring fed a body-frame correction straight into a world torque channel --
        #    exact at heading 0, and at heading 90 the roll correction comes out as a PITCH torque:
        #    a 600 N*m/rad safety net wired to the wrong axis, actively toppling the robot. This is
        #    what the 2026-07-10 note correctly identified.
        #  * HARNESS_ATT_HEADING (its attempted cure) rotated the torque correctly but ALSO rotated
        #    the RATES world->body -- and they were already body. Its D term (KD=60) is therefore
        #    cross-wired at heading 90 instead, so it did not cure the fall either (measured: the
        #    same corner fails with it on AND with it off).
        # The form below is the only one that is right in all three frames, and it is byte-identical
        # to the original at heading 0 (cos=1, sin=0) -- where every shipped demo lives.
        # ANGVEL_WORLD_FRAME=1 restores the old (incorrect) world-rate model for A/B.
        _yb5 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        _cb5, _sb5 = math.cos(_yb5), math.sin(_yb5)
        if _i("ANGVEL_WORLD_FRAME", 0):
            _wxb5 = _cb5 * float(qv[3]) + _sb5 * float(qv[4])     # the old, false world->body rotation
            _wyb5 = -_sb5 * float(qv[3]) + _cb5 * float(qv[4])
        else:
            _wxb5 = float(qv[3])                                   # already body-frame: use as-is
            _wyb5 = float(qv[4])
        _txb5 = -_f("HARNESS_KP", 600.0) * _r5 - _f("HARNESS_KD", 60.0) * _wxb5   # body-frame PD
        _tyb5 = -_f("HARNESS_KP", 600.0) * _p5 - _f("HARNESS_KD", 60.0) * _wyb5
        _tx5 = max(-350.0, min(350.0, _lam5 * (_cb5 * _txb5 - _sb5 * _tyb5)))     # -> WORLD axes
        _ty5 = max(-350.0, min(350.0, _lam5 * (_sb5 * _txb5 + _cb5 * _tyb5)))
        _tz5 = 0.0
        _hky = _f("HARNESS_KYAW", 0.0)
        # ⛔⛔ NYQUIST LIMIT CYCLE in the STAND yaw PD -- the root cause of the multi-turn BATON
        # failure (2026-07-12, measured). HARNESS_KYD_STAND defaults to 140 N*m*s/rad, applied
        # EXPLICITLY on the raw per-tick yaw rate at dt=0.016 s. The pelvis's own yaw inertia is
        # small (measured from the cycle itself: |dw| = 10.4 rad/s under 140 N*m for one tick =>
        # I_zz ~ 0.215 kg*m^2), so the discrete damping ratio KD*dt/I ~ 10.4 is FIVE TIMES the
        # explicit-integration stability limit of 2. The D term therefore does not damp the yaw --
        # it DRIVES it, at the Nyquist frequency, and saturates:
        #     CRS-DBG t=870 stand yaw=0.0795 wz=+5.195 wtz=+140.0
        #     CRS-DBG t=875 stand yaw=0.0344 wz=-5.222 wtz=-140.0
        #     CRS-DBG t=880 stand yaw=0.0774 wz=+5.197 wtz=+140.0
        #     CRS-DBG t=885 turn  yaw=0.0327 wz=-5.217 wtz=-140.0   <- turn swaps in HERE
        # (yaw buzzing +/-0.03 rad at ~31 Hz for the WHOLE stand; the 20-tick telemetry sampled it
        # on one parity and reported a "steady" wtz=140, which is how it hid for so long.)
        # This is why a course with TWO turns broke while every one-turn demo passed: the turn
        # segment cuts the crane yaw torque to zero in a single tick, so the turner is cold-started
        # on a pelvis whose yaw rate is a +/-5 rad/s coin flip decided by the PARITY of the swap
        # tick. Corner 1 happened to win that flip; corner 2 did not, and was thrown into a
        # backwards spin (wz peaked at -7.3 rad/s mid-ghost).
        # FIX: damp the ROBOT's yaw motion, not a 31 Hz numerical buzz -- low-pass the rate that
        # feeds the D term. At alpha=0.08 the Nyquist gain is a/(2-a)=0.042, so the effective
        # KD at the buzz frequency drops 140 -> 5.8 (comfortably inside the stability limit of
        # ~27) while the damping seen by the real, slow yaw mode is untouched. HARNESS_KYD_LP=0
        # restores the exact prior behavior. Filtered every tick (incl. turn segments, where the
        # yaw PD is off) so it is never stale on re-entry.
        _wzraw5 = float(qv[5])
        _lp5 = _f("HARNESS_KYD_LP", 0.08)
        if _lp5 > 0.0:
            world._wr_wzlp = (1.0 - _lp5) * getattr(world, "_wr_wzlp", _wzraw5) + _lp5 * _wzraw5
        if getattr(world, "_wr_turn_active", False):
            _hky = 0.0   # a TURN segment: the turn specialist rotates itself -- the crane
            _tz5 = 0.0   # steering toward the stale pre-turn heading would fight it
            # TURN-HOLD YAW LOCK (2026-07-08): once the footwork turn has DONE (decel-stop), the frozen
            # turn pose is NOT yaw-stable -- the robot un-rotates/oscillates back (measured 90->34), so
            # leg 2 starts from the wrong heading and crabs backward. Hold the ACHIEVED turn yaw with a
            # gentle crane torque. The TURN itself stayed wtz=0 (real footwork); this only STABILIZES the
            # completed result, exactly as tx/ty already hold the attitude and fx/fy hold the position.
            _tyaw = getattr(world, "_wr_turnhold_yaw", None)
            if getattr(world, "_wr_turn_done", False) and _tyaw is not None:
                _yhn = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                _yeH = math.atan2(math.sin(_yhn - _tyaw), math.cos(_yhn - _tyaw))
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
            # A BATON STAND is stationary too, so it gets the same strong, heavily-damped yaw PD as the
            # pivot. With the pelvis position now held, the stand's residual leg push no longer slides the
            # robot -- it SPINS it instead (measured: yaw 1->58deg during the pre-turn stand, which then
            # made the turn overshoot to 110deg and topple). The walk-trim cap (0.12 rad) is far too weak
            # to hold a heading; a standing robot has no forward momentum to tumble, so crank it.
            if getattr(world, "_wr_pivot_active", False) or _bstand5:
                # square6: at 108 N*m the pivot rotated fine (roll/pit stayed <0.15) but was
                # UNDERDAMPED -- it overshot the 90-degree target and oscillated ~1.7 rad back,
                # and the walk re-entry caught it mid-swing -> tumble. So the in-place pivot runs
                # a heavily-damped PD (HARNESS_KYD_STAND >> the walk damping): rotate to the
                # target and STOP, no ring.
                _ycap = _f("HARNESS_YAW_CAP_STAND", 0.6); _hkye = _f("HARNESS_KYAW_STAND", 120.0)
                _ramp5 = 1.0; _tzc = _f("HARNESS_TZ_CAP_STAND", 140.0); _kyd5 = _f("HARNESS_KYD_STAND", 140.0)
                # the D term of THIS branch is the one that self-oscillates (see the note above):
                # it is the only place a 140 gain meets the raw per-tick rate. Feed it the filtered
                # rate. The walk-trim branch (KYD=30, no measured buzz) keeps the raw rate so every
                # straight-walk demo stays byte-identical.
                _wzd5 = getattr(world, "_wr_wzlp", _wzraw5) if _lp5 > 0.0 else _wzraw5
            else:
                _ycap = 0.12; _hkye = _hky
                _ramp5 = min(1.0, max(0.0, (tt - 150) / 400.0)); _tzc = 350.0; _kyd5 = _f("HARNESS_KYD", 30.0)
                _wzd5 = _wzraw5
            _yerr5 = max(-_ycap, min(_ycap, _yerr5))
            _tz5 = max(-_tzc, min(_tzc, _ramp5 * _lam5 * (-_hkye * _yerr5 - _kyd5 * _wzd5)))
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
        if _i("CRANE_LOG", 0) and tt % _i("CRANE_LOG_EVERY", 4) == 0:
            # MOTION-LEGITIMACY: the crane wrench per tick. With KZ=0 the rope carries no weight,
            # but fy (lateral catch) and ESPECIALLY tx/ty (attitude springs, cap 350 N*m) can hold
            # a leaning robot up -- "hanging by the torso". The verifier integrates these.
            R._log(world, "CRANELOG t=%d fx=%.1f fy=%.1f fz=%.1f tx=%.1f ty=%.1f tz=%.1f bx=%.3f bz=%.3f"
                   % (tt, _fx5, _fy5, _fz5, _tx5, _ty5, _tz5, float(qp[0]), float(qp[2])))
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
                _SK5 = getattr(world, "_wr_suck", None)
                if _SK5 and _SK5.get("w") is not None and _SK5["jf0"] >= 0 \
                        and _SK5["jf0"] + 6 <= len(_jfa):
                    # applies while latched (spring wrench) AND during the post-release
                    # PIN-DOWN pulse (state==open, w set with on=False)
                    # SUCTION coupling delivery (linear-first joint_f layout, same as the crane's
                    # default branch): +wrench on the box's free-joint dofs, reaction on the base.
                    _w5s = _SK5["w"]
                    _b05 = _SK5["jf0"]
                    _jfa[_b05:_b05 + 3] = _w5s[0]; _jfa[_b05 + 3:_b05 + 6] = _w5s[1]
                    _jfa[_o5:_o5 + 3] += _w5s[2]; _jfa[_o5 + 3:_o5 + 6] += _w5s[3]
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
    if _i("FOOT_HEADING_LOCK", 0) and not getattr(world, "_wr_turn_active", False):
        # Deterministic, contact-generated heading trim.  The policy's heading
        # observation is live but this champion's response is effectively dead;
        # at the faster reference cadence a small per-step bias otherwise grows
        # into >1 rad of yaw and converts forward travel into a circle.  Offset
        # both hip-yaw targets inside the existing leg corridor so planted feet,
        # not an external crane torque, produce the correction.
        if not hasattr(world, "_wr_heading_lock_yaw"):
            world._wr_heading_lock_yaw = (_f("HEADING_TARGET", yaw)
                                          if os.environ.get("HEADING_TARGET") is not None else yaw)
        # Straight demos hold the launch heading.  A BATON course continuously
        # slews _wr_heading_tgt toward its next waypoint, so opt-in course runs
        # track that live target instead of fighting every legitimate corner.
        _hlock7 = world._wr_heading_lock_yaw
        if _i("FOOT_HEADING_TRACK_TARGET", 0) and getattr(world, "_wr_heading_tgt", None) is not None:
            _hlock7 = float(world._wr_heading_tgt)
        _ye7 = math.atan2(math.sin(yaw - _hlock7), math.cos(yaw - _hlock7))
        _trim7 = _f("FOOT_HEADING_TRIM_SIGN", -1.0) * max(
            -_f("FOOT_HEADING_TRIM_CAP", 0.08),
            min(_f("FOOT_HEADING_TRIM_CAP", 0.08),
                -_f("FOOT_HEADING_KP", 0.20) * _ye7 - _f("FOOT_HEADING_KD", 0.015) * float(qv[5])))
        for _yi7 in getattr(world, "_wr_yawi", []):
            _nd7 = int(world._wr_legnd[_yi7])
            if 0 <= _nd7 < len(tp):
                _yc7 = float(world._wr_glut[gb][_yi7])
                _ffd7 = getattr(world, "_wr_ffdq", None)
                if _ffd7 is not None and gb < len(_ffd7):
                    _yc7 += float(_ffd7[gb][_yi7])
                _yw7 = float(world._wr_gresyaw)
                tp[_nd7] = max(_yc7 - _yw7, min(_yc7 + _yw7, float(tp[_nd7]) + _trim7))
        world._wr_heading_trim = _trim7
    _ctl_target_pos(world).assign(tp)
    # TURN-TO-TARGET (2026-07-07, owner: "loop the turn, stop at 90 -- don't settle a one-shot").
    # A continuous turn is a stable limit cycle; the fall came from over-spinning with no target.
    # But you can't STOP a steady turn by freezing -- the angular momentum carries on (measured:
    # freeze at 85 -> spun to 306 and fell). You DECELERATE: ramp the reference speed down over the
    # last TURN_DECEL_DEG so the turn slows to a stop and the policy has time to arrest the spin
    # (exactly what the finite ghost's decel-into-stand did). Off by default -> exact prior behavior.
    _t2d = getattr(world, "_wr_turn_deg_ovr", 0.0) or _f("TURN_TO_DEG", 0.0)
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
        # SIGNED progress along the ghost's OWN turn direction (2026-07-12). `abs()` here was
        # direction-blind: a robot rotating BACKWARDS scored the same as one rotating toward the
        # target, so a failed corner read as an over-rotated one and the end-hold was pinned at a
        # heading ~2x wrong. `_wr_turn_sgn` is 0 when no solo-turn ghost is armed (the plain
        # TURN_MODE / turn_solo path), and there the original abs() semantics are kept verbatim.
        _tsg9 = float(getattr(world, "_wr_turn_sgn", 0.0))
        _prog9 = (_tsg9 * world._wr_turn_acc if _tsg9 else abs(world._wr_turn_acc)) * 57.2958
        _rem = _t2d - _prog9                                       # degrees still to turn
        if _i("TURN_DBG", 0) and tt % _i("TURN_DBG_EVERY", 10) == 0:
            _dnb = max(1, int(getattr(world, "_wr_seqnb", 1)))
            _dbin = int((world._wr_phase % (2.0 * math.pi)) / (2.0 * math.pi) * _dnb) % _dnb
            _dgy = float(world._wr_seq_yaw[_dbin]) * 57.2958 if getattr(world, "_wr_seq_yaw", None) is not None else 0.0
            R._log(world, "TURN-DBG t=%d ph=%.4f tp=%.4f bin=%d gyaw=%.1f yaw=%.4f yprev=%.4f y0=%.4f "
                          "acc=%.2f rem=%.1f pass=%d done=%d hold=%d wz=%.3f wtz=%.1f htgt=%.3f"
                   % (tt, world._wr_phase, 2.0 * math.pi * (1.0 - 0.5 / _dnb), _dbin, _dgy,
                      _ynow, _yprev, getattr(world, "_wr_turn_yaw0", 0.0),
                      world._wr_turn_acc * 57.2958, _rem, getattr(world, "_wr_turn_pass", 1),
                      int(getattr(world, "_wr_turn_done", False)), int(getattr(world, "_wr_seq_hold", False)),
                      float(qv[5]), float(getattr(world, "_wr_dbg_wrench", (0,) * 5)[4]),
                      float(getattr(world, "_wr_heading_tgt", 0.0) or 0.0)))
        # TURN-LOOP (2026-07-10, the walk-turn-walk finalization): the 90-deg seq ghost played ONCE
        # only banks ~60-65% of its reference yaw live (foot-slip: the legs track the ghost but the
        # base under-rotates -- Round-11's ground-reaction gap, bounded but nonzero even at torsion
        # 0.25). A slower clock cannot add rotation (slip is per-STEP, not per-second) -- more STEPS
        # can. The step-turn ghost is a MODULAR staircase (15-deg mini-pivots, each starting and
        # ending on the same feet-together stand), so: replay PARTIAL passes -- restart at the
        # plateau chosen so the remaining staircase ~= remaining angle / measured gain -- until the
        # ACTUAL accumulated yaw reaches the target. ⛔ Never stop mid-lut: the robot LAGS the ghost
        # by ~1-2 mini-cycles, so a ghost plateau says nothing about the ROBOT's stance; both
        # mid-lut arrests measured (freeze-in-place, jump-to-end) recoiled -19 deg or spun+fell.
        # Every pass therefore plays THROUGH the ghost's own final decel into its end-hold -- the
        # arrival the once-through demo proved leak-free for 35 s. Owner's "loop the turn, stop on
        # the actual heading" law, applied at the sequence level. Default OFF.
        if _i("TURN_LOOP", 0) and getattr(world, "_wr_in_turn", False) and getattr(world, "_wr_seq_hold", False):
            if not getattr(world, "_wr_turn_done", False):
                _nb9 = max(1, int(world._wr_seqnb))
                _tp9 = 2.0 * math.pi * (1.0 - 0.5 / _nb9)
                world._wr_phase += world._wr_omega * world._wr_dt     # full rate, as trained
                _ctx9 = getattr(world, "_wr_active_turnctx", None) or {}
                _plats9 = _ctx9.get("plateau_starts") or []           # [(bin, ghost yaw deg), ...]
                _carry9 = bool((getattr(world, "_wr_suck", None) or {}).get("on", False))
                _lead9 = (_f("TURN_CARRY_PLATEAU_STOP_LEAD_DEG",
                             _f("TURN_PLATEAU_STOP_LEAD_DEG", 0.0)) if _carry9
                          else _f("TURN_PLATEAU_STOP_LEAD_DEG", 0.0))
                _bin9 = int((world._wr_phase % (2.0 * math.pi)) / (2.0 * math.pi) * _nb9) % _nb9
                _onplat9 = any(abs(_bin9 - int(_pb9)) <= 2 for _pb9, _ in _plats9)
                if _lead9 > 0.0 and _rem <= _lead9 and _onplat9:
                    # Professional finite-sequence stop: retain the trained full-rate footwork, then
                    # hand to the learned end hold only at a measured yaw plateau (double support).
                    # Slowing phase continuously is OOD for the sequence policy and produced a 1 m
                    # spiral; waiting for the next plateau keeps the turn compact and avoids freezing
                    # a swing foot. The production course exits to its adjacent stand specialist on
                    # the next tick, making this a real skill handoff instead of asking an OOD
                    # mid-sequence LSTM state to become the hold controller. Separate empty/payload
                    # leads account for their different inertia.
                    world._wr_turn_done = True
                    world._wr_phase = _tp9
                    R._log(world, "TURN-LOOP: professional plateau stop at t=%d acc=%.1f of %.0f deg "
                                  "(bin %d, lead %.1f%s)"
                           % (tt, _prog9, _t2d, _bin9, _lead9, ", payload" if _carry9 else ""))
                elif _i("TURN_TARGET_STOP", 0) and _rem <= 0.0:
                    # Physical-heading stop for compact one-pass ghosts.  Reference plateaus are
                    # clocked to the ghost, but live contact lags that clock by about one support
                    # transfer; waiting for the nominal plateau therefore overshot 90 -> 111 deg.
                    # Pinning the sequence here is safe only with the adjacent stand handoff
                    # (BATON_TURN_EXIT_TICKS=1): the finite-sequence LSTM is never asked to hold this
                    # mid-reference state.  TURN_TARGET_STOP stays opt-in for legacy turn demos.
                    world._wr_turn_done = True
                    world._wr_phase = _tp9
                    R._log(world, "TURN-LOOP: physical target stop at t=%d acc=%.1f of %.0f deg%s"
                           % (tt, _prog9, _t2d, ", payload" if _carry9 else ""))
                elif _rem < -_f("TURN_LOOP_ABORT_DEG", 60.0):
                    # over-rotation ABORT (2026-07-10, measured: a wound-up entry cascaded to a 311-deg
                    # spin and the loop only noticed at the 10-s pass end). A spinning robot is worse
                    # than a mid-lut reference step: stop driving the turn NOW, pin the end-hold stand,
                    # and let the settle/exit window hand off to the stand catch.
                    # NOTE (2026-07-12): _rem is now SIGNED progress, so this fires only on a genuine
                    # over-rotation PAST the target -- never on a robot that rotated backwards.
                    world._wr_turn_done = True; world._wr_turn_failed = True
                    world._wr_phase = _tp9
                    R._log(world, "TURN-LOOP ABORT: over-rotated to %.1f deg (target %.0f) at t=%d -- end-hold pinned"
                           % (_prog9, _t2d, tt))
                elif _tsg9 and _prog9 < -_f("TURN_WRONGWAY_DEG", 35.0):
                    # WRONG-WAY ABORT (2026-07-12, new): the robot is rotating AGAINST the ghost. Before
                    # the signed-progress fix this was invisible -- abs() scored it as progress and the
                    # over-rotation abort above eventually fired, mislabelling a backwards spin as an
                    # overshoot and pinning the end-hold ~180 deg from the target. Driving the turn
                    # harder cannot recover it; stop, pin the end-hold, and let the stand catch. With
                    # the stand-yaw limit cycle fixed this should not trigger -- it is the honest
                    # failure report if it ever does, not a workaround.
                    world._wr_turn_done = True; world._wr_turn_failed = True
                    world._wr_phase = _tp9
                    R._log(world, "TURN-LOOP WRONG-WAY ABORT: rotated %.1f deg AGAINST the ghost "
                                  "(target %.0f) at t=%d -- turn failed, end-hold pinned"
                           % (-_prog9, _t2d, tt))
                elif world._wr_phase >= _tp9 - 1e-9:                  # a pass reached the end-hold
                    _acc9 = _prog9                                    # signed progress, not abs()
                    _tol9 = _f("TURN_LOOP_TOL_DEG", 8.0)
                    if _rem <= _tol9 or getattr(world, "_wr_turn_pass", 1) >= _i("TURN_LOOP_MAX", 4) or not _plats9:
                        world._wr_turn_done = True
                        world._wr_phase = _tp9                        # settle on the end-hold pin
                        R._log(world, "TURN-LOOP: done at t=%d acc=%.1f of %.0f deg (pass %d%s)"
                               % (tt, _acc9, _t2d, getattr(world, "_wr_turn_pass", 1),
                                  ", pass cap" if _rem > _tol9 else ""))
                    elif abs(float(qv[5])) >= _f("TURN_LOOP_STILL_WZ", 0.35):
                        # STILLNESS GATE (2026-07-10, measured fall): a pass restart launches the
                        # routine's first full stride -- on a still robot that is its trained start,
                        # on a robot still rotating from a scrambled pass it toppled. The end-hold
                        # pin is safe indefinitely; wait there until the yaw rate settles.
                        world._wr_phase = _tp9
                    else:
                        # plan the next PARTIAL pass from the measured per-ghost-degree gain
                        _gp9 = max(1.0, getattr(world, "_wr_turn_gplayed", 0.0))
                        _G9 = max(0.35, min(1.0, _acc9 / _gp9))
                        _need9 = _rem / _G9                           # ghost degrees still needed
                        _end9 = float(_plats9[-1][1])                 # ghost yaw at the end-hold
                        _cand9 = [p for p in _plats9 if _end9 - float(p[1]) >= 14.0]  # >=1 mini-cycle
                        _rb9, _ry9 = min(_cand9, key=lambda p: abs((_end9 - float(p[1])) - _need9))
                        world._wr_turn_pass = getattr(world, "_wr_turn_pass", 1) + 1
                        world._wr_turn_gplayed = _gp9 + (_end9 - float(_ry9))
                        world._wr_phase = 2.0 * math.pi * (float(_rb9) + 0.5) / _nb9
                        R._log(world, "TURN-LOOP: pass %d from plateau bin %d (ghost %.0f->%.0f deg): acc=%.1f, rem=%.1f, gain=%.2f"
                               % (world._wr_turn_pass, int(_rb9), float(_ry9), _end9, _acc9, _rem, _G9))
            # done -> the end-hold pin keeps the phase; the settle/exit window runs on _wr_turn_done
        elif _rem > 0:
            _sc = max(0.0, min(1.0, _rem / max(1.0, _f("TURN_DECEL_DEG", 35.0))))   # 1 far -> 0 at target
            world._wr_phase += world._wr_omega * world._wr_dt * _sc
        elif not getattr(world, "_wr_turn_done", False):
            # COMPLETE THE STEP (owner 2026-07-08, "add a STAND step after the turn"): freezing the phase
            # the instant the target angle is hit strands the SWING foot mid-air -> an unstable one-foot
            # pose the STAND cannot catch (short exit topples; long exit only settles by UN-ROTATING to a
            # wrong heading). Instead COAST the phase to the next DOUBLE-SUPPORT (both feet planted, at
            # phase == _DS mod pi) then freeze -- the turn ends on TWO feet near the target angle, exactly
            # the stable pose the stand grabs and holds (proven: stand->walk preserves heading cleanly).
            # TURN_COAST_SC=0 -> exact old freeze-instantly behavior (backward compat).
            _csc = _f("TURN_COAST_SC", 0.0)
            if _csc <= 0.0:
                world._wr_turn_done = True
                R._log(world, "TURN-TO-TARGET %.0f deg reached at t=%d (decelerated to a stop)" % (_t2d, tt))
            else:
                if not hasattr(world, "_wr_turn_coast_to"):
                    _DSp = 0.65 * 2.0 * math.pi                     # gait-family double-support phase
                    _k = math.ceil((world._wr_phase - _DSp) / math.pi)
                    _tgt = _DSp + _k * math.pi
                    if _tgt <= world._wr_phase + 0.05:              # already at/just past a DS -> take the next
                        _tgt += math.pi
                    world._wr_turn_coast_to = _tgt
                    world._wr_turn_coast0 = tt
                world._wr_phase += world._wr_omega * world._wr_dt * _csc
                if world._wr_phase >= world._wr_turn_coast_to or (tt - world._wr_turn_coast0) > _i("TURN_COAST_MAX", 160):
                    world._wr_phase = min(world._wr_phase, world._wr_turn_coast_to)
                    world._wr_turn_done = True
                    R._log(world, "TURN-TO-TARGET %.0f deg reached at t=%d (coasted %d ticks to feet-together, phase=%.2f)"
                           % (_t2d, tt, tt - world._wr_turn_coast0, world._wr_phase))
    else:
        # A placement arrest is a genuine stationary manipulation state.  Freezing the gait clock
        # together with the achieved leg targets prevents the reference from silently marching
        # underneath the hold and makes re-entry deterministic after the arms have cleared.
        if not getattr(world, "_wr_place_arrest", False):
            world._wr_phase += world._wr_omega * world._wr_dt
        # DEPLOY PROGRESS-LOCK, RATCHETED (2026-07-10). SEQ_LEASH_LEAD was TRAINER-ONLY; every "leash
        # deploy" silently ran the free clock, and the live tracker yo-yoed: the reference marches at
        # omega through a step-up the live contact delays, the corridor's targets skip to later poses
        # mid-step, and the robot aborts back down (STEPS L [0,1,0,1,...] on every live run). Here the
        # reference WAITS when the robot is behind (cap = bin(base_x)+lead) -- and unlike the trainer's
        # torch.minimum, it can NEVER REWIND: live base-x oscillates centimetres during a weight
        # transfer, and a bidirectional cap would drag the reference (and the robot) back down with it.
        if getattr(world, "_wr_evgates", None) is not None:
            # CONTACT-EVENT CLOCK (scalar mirror of training; precedence over the ratchet): hold at
            # the next touchdown gate until the gated foot is at the ghost's foot height there.
            # try/except like FOOT_LOG: _footmap/solver reads can fail on early ticks, and one
            # per-tick exception kills the whole hook silently (measured: EMPTY rl log, robot falls
            # at 2.55 s under the bare stand controller).
            try:
                _gl6 = world._wr_evgates
                if world._wr_evgi < len(_gl6):
                    _gb6, _gf6, _gz6 = _gl6[world._wr_evgi]
                    _gph6 = _gb6 / float(world._wr_gnb) * 2.0 * math.pi
                    if not hasattr(world, "_wr_footids5"):
                        world._wr_footids5 = _footmap(world)[1]
                    _xp6 = world.solver.mjw_data.xpos.numpy().reshape(-1, 3)
                    _fz6 = float(_xp6[world._wr_footids5[_gf6]][2])
                    if _fz6 <= _gz6 + 0.025:
                        if world._wr_phase >= _gph6 - 1e-6:
                            world._wr_evgi += 1
                    else:
                        world._wr_phase = min(world._wr_phase, _gph6)
            except Exception as _e6:
                if not getattr(world, "_wr_evwarned", False):
                    world._wr_evwarned = True
                    R._log(world, "SEQ-EVENT-CLOCK(deploy) tick error (clock idles this tick): %r" % (_e6,))
        elif getattr(world, "_wr_seq_rootx", None) is not None and _f("SEQ_LEASH_LEAD", 0.0) > 0:
            _rx5 = world._wr_seq_rootx
            _cap5 = (float(np.searchsorted(_rx5, min(max(float(qp[0]), float(_rx5[0])), float(_rx5[-1]))))
                     + _f("SEQ_LEASH_LEAD", 0.0)) / len(_rx5) * 2.0 * math.pi
            _phr5 = max(getattr(world, "_wr_ph_ratchet", 0.0), min(world._wr_phase, _cap5))
            world._wr_phase = _phr5
            world._wr_ph_ratchet = _phr5
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
    if _i("CONTACT_LOG", 0) and tt % _i("CONTACT_LOG_EVERY", 2) == 0:
        # LIVE side of the riser-refusal instrumentation: same line format as the eval's
        # BATCH stream (geoms, penetration, position, normal, base x, phase) -> diff directly.
        try:
            if not hasattr(world, "_wr_cgeom6"):
                _mjm6 = world.solver.mj_model
                _fb6 = {int(b) for b in _footmap(world)[1]}
                world._wr_cgeom6 = {g for g in range(int(_mjm6.ngeom))
                                    if int(_mjm6.geom_bodyid[g]) in _fb6}
                # CONTACT MAP (2026-07-11, motion-legitimacy verifier): dump geom->body names once
                # so the verifier can classify contacts (foot vs KNEE/shin/hand -- the owner caught
                # the champion knee-climbing; the foot-filtered logger was blind to it).
                try:
                    import mujoco as _mjn6
                    for g6 in range(int(_mjm6.ngeom)):
                        _bn6 = _mjn6.mj_id2name(_mjm6, _mjn6.mjtObj.mjOBJ_BODY, int(_mjm6.geom_bodyid[g6])) or "?"
                        R._log(world, "CONTACTMAP g=%d body=%s" % (g6, _bn6))
                except Exception:
                    pass
            _md6 = world.solver.mjw_data
            _nc6 = int(_md6.nacon.numpy()[0])
            _ge6 = _md6.contact.geom.numpy()[:_nc6]
            _di6 = _md6.contact.dist.numpy()[:_nc6]
            _po6 = _md6.contact.pos.numpy()[:_nc6]
            _fr6 = _md6.contact.frame.numpy()[:_nc6]
            _all6 = _i("CONTACT_LOG_ALL", 0)
            for i6 in range(_nc6):
                a6, b6 = int(_ge6[i6][0]), int(_ge6[i6][1])
                if _all6 or a6 in world._wr_cgeom6 or b6 in world._wr_cgeom6:
                    _n6 = _fr6[i6][0]
                    R._log(world, "CONTACTLOG side=LIVE t=%d g=(%d,%d) d=%.4f p=(%.3f,%.3f,%.3f) n=(%.2f,%.2f,%.2f) bx=%.3f ph=%.2f"
                           % (tt, a6, b6, float(_di6[i6]), float(_po6[i6][0]), float(_po6[i6][1]), float(_po6[i6][2]),
                              float(_n6[0]), float(_n6[1]), float(_n6[2]), float(qp[0]), float(getattr(world, "_wr_phase", 0.0))))
        except Exception as _e6:
            if not getattr(world, "_wr_clogerr", False):
                world._wr_clogerr = True
                R._log(world, "CONTACTLOG LIVE err %r" % (_e6,))
    # GHOST-FOLLOW (2026-07-10, owner: "the ghost must execute the SAME motion as the puppet"):
    # publish the ACTIVE reference every 2 ticks -- mode, lut path, phase, display heading, robot
    # xy -- so the hologram renders the ENTIRE routine (walk legs, the step-turn, the stands), not
    # only the cyclic walk. g1_ghost.py ("follow" controllerArg) lazy-loads whatever lut the deploy
    # is tracking and poses itself at the SAME phase; stands freeze it; the turn displays at the
    # robot's ACTUAL yaw (lock-step, the 1799997f yawlock semantics -- REFERENCE yaw would jump
    # backward at every TURN-LOOP pass restart). GHOST_FOLLOW=1 arms it; default OFF (zero writes).
    if getattr(world, "_wr_gfollow", None) is None:
        world._wr_gfollow = os.environ.get("GHOST_FOLLOW_FILE", "_scratch/foot_redesign/ghost_follow.json") \
            if _i("GHOST_FOLLOW", 0) else ""
        # specialist name -> ghost lut path (so a CARRY segment publishes the CARRY reference,
        # arms chest-high, instead of the walking arms)
        world._wr_gf_luts = {}
        for _sp9 in os.environ.get("BATON_SPECIALISTS", "").split(";"):
            _pf9 = _sp9.split("|")
            if len(_pf9) >= 3:
                world._wr_gf_luts[_pf9[0].strip()] = _pf9[2].strip()
    if world._wr_gfollow and tt % 2 == 0:
        try:
            _yg9 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            _bt9 = world._wr_baton.target if getattr(world, "_wr_baton", None) is not None else None
            if getattr(world, "_wr_in_turn", False):
                _md9 = "turn"; _lp9 = os.environ.get("BATON_TURN_LUT", ""); _yd9 = _yg9
            elif _bt9 == "stand":
                _md9 = "stand"; _lp9 = os.environ.get("GHOST_LUT_JSON", ""); _yd9 = _yg9
            elif _bt9 is not None and _bt9 in getattr(world, "_wr_gf_luts", {}):
                _md9 = str(_bt9); _lp9 = world._wr_gf_luts[_bt9]
                _hd9 = getattr(world, "_wr_heading_tgt", None)
                _yd9 = float(_hd9) if _hd9 is not None else _yg9
            else:
                _md9 = "walk"; _lp9 = os.environ.get("GHOST_LUT_JSON", "")
                _hd9 = getattr(world, "_wr_heading_tgt", None)
                _yd9 = float(_hd9) if _hd9 is not None else _yg9
            _st9 = ('{"t":%.3f,"mode":"%s","lut":"%s","phase":%.5f,"held":%d,"x":%.4f,"y":%.4f,"yaw":%.5f}'
                    % (tt * 0.016, _md9, _lp9.replace("\\", "/"), float(world._wr_phase % (2.0 * math.pi)),
                       1 if (getattr(world, "_wr_in_turn", False) and getattr(world, "_wr_turn_done", False)) else 0,
                       float(qp[0]), float(qp[1]), _yd9))
            _tf9 = world._wr_gfollow + ".tmp"
            with open(_tf9, "w") as _fh9:
                _fh9.write(_st9)
            os.replace(_tf9, world._wr_gfollow)
        except OSError:
            pass
    if getattr(world, "_wr_handbids", None) is not None and tt % 2 == 0:
        try:   # HAND-TRACK: live FK hand centroid for the box rig (see the deploy-setup note)
            _xph = np.array(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
            _hL9 = _xph[int(world._wr_handbids[0])]; _hR9 = _xph[int(world._wr_handbids[1])]
            _hc9 = 0.5 * (_hL9 + _hR9)
            _tfh = world._wr_handfile + ".tmp"
            with open(_tfh, "w") as _fhh:
                _fhh.write('{"t":%.3f,"x":%.4f,"y":%.4f,"z":%.4f,"sep":%.3f}'
                           % (tt * 0.016, float(_hc9[0]), float(_hc9[1]), float(_hc9[2]),
                              float(np.linalg.norm(_hL9 - _hR9))))
            os.replace(_tfh, world._wr_handfile)
        except Exception:
            pass
    if _i("WALK_DIAG", 0) and getattr(world, "_wr_glut", None) is not None:
        # One compact, objective line per gait cycle: net propulsion, reverse
        # fraction, heading drift, stance-foot slip, and achieved/reference joint
        # amplitude.  This makes a headless A/B self-scoring instead of relying on
        # visual gait impressions.
        try:
            _cy9 = int(world._wr_phase // (2.0 * math.pi))
            _y9 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            _qa9 = np.asarray(qp[world._wr_legqadr], np.float32)
            _gb9 = int((world._wr_phase % (2 * math.pi)) / (2 * math.pi) * world._wr_gnb) % world._wr_gnb
            _qt9 = np.asarray(world._wr_glut[_gb9], np.float32)
            if not hasattr(world, "_wr_cd"):
                world._wr_cd = dict(cy=_cy9, n=0, x=float(qp[0]), y=float(qp[1]), yaw=_y9,
                                    back=0, amin=_qa9.copy(), amax=_qa9.copy(),
                                    tmin=_qt9.copy(), tmax=_qt9.copy(), slip=[0.0, 0.0], fp=None)
            _cd9 = world._wr_cd
            if _cy9 != _cd9["cy"]:
                if _cd9["n"] > 4:
                    _aa9 = _cd9["amax"] - _cd9["amin"]
                    _ta9 = _cd9["tmax"] - _cd9["tmin"]
                    _vm9 = _ta9 > 0.02
                    _ar9 = float(np.mean(_aa9[_vm9] / _ta9[_vm9])) if np.any(_vm9) else 0.0
                    _hi9 = list(getattr(world, "_wr_hipi", [0, len(_aa9) // 2]))
                    _hta9 = _ta9[_hi9]; _haa9 = _aa9[_hi9]; _hm9 = _hta9 > 0.02
                    _hr9 = float(np.mean(_haa9[_hm9] / _hta9[_hm9])) if np.any(_hm9) else 0.0
                    _dx9 = float(qp[0]) - _cd9["x"]; _dy9 = float(qp[1]) - _cd9["y"]
                    _pdt9 = _cd9["n"] * world._wr_control_dt
                    R._log(world, "CYCLE-DIAG cycle=%d dt=%.3f dx=%.4f speed=%.3f dy=%.4f "
                                  "back=%.1f%% dyaw=%.3f amp_ratio=%.3f hip_amp_ratio=%.3f "
                                  "stance_slip_L=%.4f stance_slip_R=%.4f"
                           % (_cd9["cy"], _pdt9, _dx9,
                              math.hypot(_dx9, _dy9) / max(world._wr_control_dt, _pdt9), _dy9,
                              100.0 * _cd9["back"] / _cd9["n"],
                              math.atan2(math.sin(_y9 - _cd9["yaw"]), math.cos(_y9 - _cd9["yaw"])),
                              _ar9, _hr9, _cd9["slip"][0], _cd9["slip"][1]))
                world._wr_cd = dict(cy=_cy9, n=0, x=float(qp[0]), y=float(qp[1]), yaw=_y9,
                                    back=0, amin=_qa9.copy(), amax=_qa9.copy(),
                                    tmin=_qt9.copy(), tmax=_qt9.copy(), slip=[0.0, 0.0], fp=None)
                _cd9 = world._wr_cd
            _vf9 = math.cos(_y9) * float(qv[0]) + math.sin(_y9) * float(qv[1])
            _cd9["n"] += 1; _cd9["back"] += int(_vf9 < -0.01)
            _cd9["amin"] = np.minimum(_cd9["amin"], _qa9); _cd9["amax"] = np.maximum(_cd9["amax"], _qa9)
            _cd9["tmin"] = np.minimum(_cd9["tmin"], _qt9); _cd9["tmax"] = np.maximum(_cd9["tmax"], _qt9)
            if not hasattr(world, "_wr_footids5"):
                world._wr_footids5 = _footmap(world)[1]
            _xp9 = np.asarray(world.solver.mjw_data.xpos.numpy()).reshape(-1, 3)
            _fp9 = np.asarray([_xp9[int(i)] for i in world._wr_footids5], np.float32)
            if _cd9["fp"] is not None:
                # The G1's foot body origins stay below ~7 cm even in swing, so
                # an absolute-z threshold labels both feet as stance.  The lower
                # foot is the load-bearing candidate; require it to be near the
                # floor and measure only that foot's horizontal motion.
                _fi9 = int(np.argmin(_fp9[:, 2]))
                if _fp9[_fi9, 2] < _f("FOOT_STANCE_Z", 0.055):
                    _cd9["slip"][_fi9] += float(np.linalg.norm(_fp9[_fi9, :2] - _cd9["fp"][_fi9, :2]))
            _cd9["fp"] = _fp9.copy()
        except Exception as _ed9:
            if not getattr(world, "_wr_diag_warned", False):
                world._wr_diag_warned = True
                R._log(world, "CYCLE-DIAG disabled after error %r" % (_ed9,))
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
        R._log(world, "walk-recipe deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f cmd=%.2f vx=%.2f glat=%.2f yaw=%.3f htgt=%s wtz=%.1f wfy=%.1f htrim=%.3f hyL=%.3f hyR=%.3f%s"
               % (tt, qp[0], qp[1], qp[2], roll, pit, float(world._wr_cmd), float(qv[0]),
                   float(getattr(world, "_wr_vc_glat", -1.0)), _yaw9,
                   "%.3f" % _ht9 if _ht9 is not None else "-", float(_wr5[4]), float(_wr5[0]),
                   float(getattr(world, "_wr_heading_trim", 0.0)), _hyL, _hyR, _ghy))
