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

"""IN-ENGINE quad WALK trainer (go2 | spot | b2) -- the in-engine port of the
standalone gpu_mjwarp_<robot>_walk_trainer.py.

WHY (owner direction 2026-07-10): OmniSim trains IN-ENGINE (through omnisim-bin,
train == deploy bit-exact). The quads historically trained on the STANDALONE
mjwarp path, which re-parses an MJCF dump to build its physics model -- that
re-parse is the last train->deploy parity gap. This module closes it: the K
batched rollout worlds come from `world._mpc_rollout_buffers(K)`, i.e. the SAME
compiled MjModel the live deploy SolverMuJoCo steps. Like the G1 flagship.

HOW (the whole trick): we SUBCLASS the robot's standalone env and, instead of
re-implementing its __init__, we run the BASE __init__ with its model source
redirected at the engine's buffers (temporarily patching MjModel.from_xml_path /
mjw.put_model / mjw.put_data). So every robot-specific thing each trainer's
__init__ builds -- spot's `prog_lag_t`, each robot's gait/limits/foot bodies --
is inherited for free, and NOTHING is copied. Swapping QUAD_ROBOT swaps the base
module and therefore the robot.

ADDITIVE-ONLY / ZERO-REGRESSION: NEW file. It imports (never edits) the
standalone trainers, so go2/spot/b2 standalone paths stay byte-identical.
Per-run MODEL-SPACE DR (rescaling mass/friction/kp of the MJCF) is STRIPPED --
the live compiled model is SHARED with the deploy sim and must never be mutated.
Per-env DR that acts on the rollout buffers at runtime (push, obs-noise, init
bands, latency) is retained. Warm-start the flat champion; that is enough.

LAZY IMPORTS (critical, mirrors g1_walk_recipe.py): the engine imports this
module *inside a physics tick* via OMNISIM_INENGINE_PYMOD. Importing torch / the
heavy env at module top-level there fails silently. So the top-level is trivially
light; torch + the base env load lazily inside _train.

Run:
    bash projects/policies/training/run_quad_walk_rl.sh <dur> <tag> train [gui] \\
        QUAD_ROBOT=go2 QUAD_ITERS=400 QUAD_ENVS=4096 \\
        QUAD_WARMSTART=$ROOT/projects/policies/research/inference/policies/gpu_go2_walk_main/warmstart.pt
    # honest durability re-eval of a saved champion (no training):
    ... QUAD_EVAL_ONLY=1 QUAD_EVAL_CKPT=<path>/policy.pt QUAD_EVAL_STEPS=3000

Hook: OMNISIM_INENGINE_PYMOD=projects.policies.training.quad_walk_recipe:quad_walk_recipe_step
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

_REPO = os.environ.get("OMNISIM_HOME") or str(
    next(_p for _p in Path(__file__).resolve().parents
         if (_p / "AGENTS.md").exists() or (_p / ".git").exists()))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)


# ── watchdog / log plumbing (the run_walk_rl.sh contract; os/json only) ───
def _log(msg: str) -> None:
    line = f"[quad_walk_recipe] {msg}"
    try:
        sys.stderr.write(line + "\n")
        sys.stderr.flush()
    except Exception:
        pass
    p = os.environ.get("RES_LOG")
    if p:
        try:
            with open(p, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass


def _status_write(state: str, **kw) -> None:
    """Write <RES_LOG>.status so the launcher watchdog can stop the sim."""
    p = os.environ.get("RES_LOG")
    if not p:
        return
    try:
        with open(p + ".status", "w", encoding="utf-8") as f:
            json.dump({"state": state, **kw}, f)
    except Exception:
        pass


def _f(k: str, d: float) -> float:
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _i(k: str, d: int) -> int:
    return int(_f(k, d))


def _ensure_torch_path():
    """The embedded newton-runtime interpreter runs isolated and IGNORES
    PYTHONPATH, so add the system torch site-packages (OMNISIM_TORCH_SITE, set by
    the launcher) to sys.path directly before importing torch. Same 3.14 ABI;
    warp version pin-matched by tests/test_newton_pins_parity.py.
    ⛔ Never put it on process-wide PYTHONPATH: the engine would then import the
    SYSTEM warp/newton during Newton init -> FFI smoke fails -> REQUIRE_NEWTON FATAL."""
    sp = os.environ.get("OMNISIM_TORCH_SITE")
    if sp and sp not in sys.path:
        sys.path.insert(0, sp)


# ── robot selection: QUAD_ROBOT = go2 | spot | b2 ────────────────────────
# All three standalone trainer modules expose the SAME names (BatchedSpotWalkEnv,
# NJ, RES_SCALE, DT, SUBSTEPS, MAX_EP, ... and each imports its OWN
# <robot>_trot_gait as `stg`). Swapping the base module swaps the robot.
_ROBOT_GAIT = {   # defaults == the per-robot deploy launchers
    "go2":  dict(vx=0.4, freq=1.8, step_h=0.05, body_h=0.30),
    "spot": dict(vx=0.4, freq=1.4, step_h=0.06, body_h=0.55),
    "b2":   dict(vx=0.5, freq=1.3, step_h=0.08, body_h=0.50),
}
_BASE_MOD = None


def _robot() -> str:
    return (os.environ.get("QUAD_ROBOT") or "go2").strip().lower()


def _base_mod():
    """Import (once) the robot's standalone trainer module. Never edits it."""
    global _BASE_MOD
    if _BASE_MOD is None:
        _ensure_torch_path()          # the trainer module imports torch
        import importlib
        r = _robot()
        if r not in _ROBOT_GAIT:
            raise RuntimeError(f"QUAD_ROBOT must be one of {sorted(_ROBOT_GAIT)}, got {r!r}")
        _BASE_MOD = importlib.import_module(
            f"projects.policies.research.training.gpu_mjwarp_{r}_walk_trainer")
    return _BASE_MOD


# ── lazy env-class factory ────────────────────────────────────────────────
_ENV_CLS = None

# DR keys that rescale the MJCF MODEL itself. The in-engine model is the LIVE
# compiled model shared with the deploy sim -- rescaling it would corrupt the
# simulation, so these are stripped. Per-env DR (below) is untouched.
_MODEL_SPACE_DR = ("mass_scale", "friction_scale", "actuator_kp_scale",
                   "actuator_kv_scale", "gravity_scale", "damping_scale")


def _get_env_cls():
    """Build (once) the in-engine env class. Deferred so importing this module
    (which happens inside a physics tick) never pulls torch/the heavy env."""
    global _ENV_CLS
    if _ENV_CLS is not None:
        return _ENV_CLS
    _ensure_torch_path()
    import torch
    BatchedSpotWalkEnv = _base_mod().BatchedSpotWalkEnv

    class _InEngineTerrain:
        """ground_h(x) built from the COMPILED model's own static boxes.

        The base env's height reward + fall test are ground-relative via
        self.terrain.ground_h(x), but that only exists on the MJCF/RoughRef path.
        In-engine the model ALREADY contains the world's real terrain, so we read
        it straight out of the compiled model -- strictly better than the legacy
        injected-bar approximation, because it IS the deploy terrain (train==deploy
        ON the terrain, which is exactly what the legacy pipeline could not do).

        Terrain geoms = static BOX geoms (body_dofnum == 0, so never the robot)
        whose top is below TOP_MAX (excludes the arena walls). Bars span full y,
        so an x-profile is the right model (same as RoughProfile)."""

        TOP_MAX = 0.5      # m -- bumps <=0.18, rocks <=0.05; walls are ~1 m

        def __init__(self, mujoco, mjm, tdev, dx=0.01):
            mjd = mujoco.MjData(mjm)
            mujoco.mj_forward(mjm, mjd)
            boxes = []
            for g in range(int(mjm.ngeom)):
                if int(mjm.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_BOX):
                    continue
                if int(mjm.body_dofnum[int(mjm.geom_bodyid[g])]) != 0:
                    continue                      # has dofs -> part of the robot
                p = mjd.geom_xpos[g]
                s = mjm.geom_size[g]              # half-extents
                top = float(p[2] + s[2])
                if top <= 1e-4 or top > self.TOP_MAX:
                    continue
                boxes.append((float(p[0] - s[0]), float(p[0] + s[0]), top))
            self.boxes = boxes
            lo = min([b[0] for b in boxes], default=0.0) - 1.0
            hi = max([b[1] for b in boxes], default=1.0) + 1.0
            n = max(2, int((hi - lo) / dx) + 1)
            xs = torch.linspace(lo, hi, n, device=tdev)
            gh = torch.zeros(n, device=tdev)
            for x0, x1, top in boxes:
                m = (xs >= x0) & (xs <= x1)
                gh[m] = torch.maximum(gh[m], torch.full_like(gh[m], top))
            self.gx, self.gh = xs, gh

        def ground_h(self, xq):
            xs, ys = self.gx, self.gh
            xq = torch.clamp(xq, float(xs[0]), float(xs[-1]))
            idx = torch.searchsorted(xs, xq).clamp(1, xs.shape[0] - 1)
            x0 = xs[idx - 1]
            f = (xq - x0) / (xs[idx] - x0).clamp(min=1e-6)
            return ys[idx - 1] + f * (ys[idx] - ys[idx - 1])

        def summary(self):
            tops = sorted({round(b[2], 3) for b in self.boxes})
            return (f"{len(self.boxes)} static boxes from the compiled model, "
                    f"heights={tops[:8]}{'...' if len(tops) > 8 else ''}")

    class InEngineQuadWalkEnv(BatchedSpotWalkEnv):
        """The robot's own batched env, but its K rollout worlds are the ENGINE's
        (world._mpc_rollout_buffers) instead of an MJCF re-parse.

        We do NOT re-implement __init__ -- we run the BASE one with its model
        source redirected, so all robot-specific state it builds (spot's
        prog_lag_t, etc.) is inherited. Every behavioural method (reset / obs /
        baseline / step / reward) is the base's, unchanged."""

        def __init__(self, world, n, reward_cfg=None, dr_cfg=None):
            import mujoco
            import mujoco_warp as mjw

            mjm = getattr(world.solver, "mj_model", None)
            if mjm is None:
                raise RuntimeError("world.solver has no mj_model (Newton/MuJoCo solver required)")
            buf = world._mpc_rollout_buffers(n)
            if buf is None:
                raise RuntimeError("world._mpc_rollout_buffers(n) returned None")
            _h, rm, rd, (nq, nv, nu) = buf
            if int(nq) != int(mjm.nq) or int(nu) != int(mjm.nu):
                raise RuntimeError(
                    f"rollout dims {(nq, nv, nu)} != mjm {(mjm.nq, mjm.nv, mjm.nu)}")

            # NEVER let the base's per-run model-space DR rescale the LIVE model.
            dr = {k: v for k, v in (dr_cfg or {}).items() if k not in _MODEL_SPACE_DR}

            # Redirect the base __init__'s model source at the engine's, run it
            # verbatim, then restore. sim_dt is the model's own timestep, so the
            # base's `self.mjm.opt.timestep = sim_dt` is a no-op (never mutate it).
            _fx = mujoco.MjModel.from_xml_path
            _pm, _pd = mjw.put_model, mjw.put_data
            mujoco.MjModel.from_xml_path = staticmethod(lambda *a, **k: mjm)
            mjw.put_model = lambda *a, **k: rm
            mjw.put_data = lambda *a, **k: rd
            try:
                super().__init__(n, "<in-engine>", reward_cfg=reward_cfg,
                                 sim_dt=float(mjm.opt.timestep),
                                 dr_cfg=dr, rough_cfg=None)
            finally:
                mujoco.MjModel.from_xml_path = _fx
                mjw.put_model = _pm
                mjw.put_data = _pd

            # ── FOOT BODIES: the base class hardcodes LOWER_LEG_BODIES=(4,7,10,13)
            # from the standalone MJCF dump's layout (world=0, base=1, legs 2-13).
            # The ENGINE-compiled model orders STATICS (floor/walls/terrain) before
            # the robot, so those indices point at walls/floor there -- silently
            # corrupting every foot-clearance (rw_sched) and anti-slip (rw_slip)
            # reward. Derive the calf bodies layout-independently instead: the calf
            # IS the knee hinge's jnt_bodyid (verified == (4,7,10,13) in the dump),
            # in FL/FR/RL/RR order, and override _foot_pos_t to use them.
            calf = []
            for ci in range(4):                      # FL, FR, RL, RR
                for j in range(mjm.njnt):
                    if mjm.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                        continue
                    if int(mjm.jnt_qposadr[j]) == int(self.controller_to_qpos[ci * 3 + 2]):
                        calf.append(int(mjm.jnt_bodyid[j]))
                        break
            if len(calf) != 4:
                raise RuntimeError(f"calf-body derivation failed: {calf}")
            self._calf_bodies = torch.tensor(calf, dtype=torch.long, device=self.tdev)
            _log(f"calf bodies (engine layout): {calf} "
                 f"(base-class constant would be (4,7,10,13) -- wrong in-engine)")

            # ── TERRAIN: the compiled model already CONTAINS the world's terrain.
            # Give the base env a ground_h() so its height reward + fall test are
            # ground-relative (else climbing a bump reads as a height error / a fall).
            self._x_spread = _f("QUAD_INIT_X_SPREAD", 0.0)
            self._spawn_dz = float(self.spawn_z)
            if _i("QUAD_TERRAIN", 0):
                self.terrain = _InEngineTerrain(mujoco, mjm, self.tdev)
                _log(f"terrain ON: {self.terrain.summary()}")
                if self._x_spread > 0:
                    _log(f"spawn spread: x ~ U[0, {self._x_spread}] (practise every bump)")
                self._reset_all()      # re-seed now that terrain/spread are known

        def _foot_pos_t(self):
            """Base-class math, but on the ENGINE-derived calf body indices.
            Falls back to the base constant during super().__init__ (before the
            derivation runs); env.reset() re-seeds with the correct indices."""
            idx = getattr(self, "_calf_bodies", None)
            if idx is None:
                return super()._foot_pos_t()
            FOOT_OFFSET = _base_mod().FOOT_OFFSET
            p = self.xpos_t[:, idx, :]
            q = self.xquat_t[:, idx, :]
            w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
            oz = FOOT_OFFSET[2]
            rx = oz * (2 * (x * z + w * y))
            ry = oz * (2 * (y * z - w * x))
            rz = oz * (1 - 2 * (x * x + y * y))
            return p + torch.stack([rx, ry, rz], dim=-1)

        def _reset_envs(self, env_mask=None):
            """Base reset, then spread the spawn along the path and lift the robot
            onto the local ground height (so an env can start ON/BEFORE any bump,
            not only at x=0 -- otherwise a 12 s episode never even reaches x=6)."""
            super()._reset_envs(env_mask)
            if getattr(self, "_x_spread", 0.0) <= 0:
                return
            idx = (torch.arange(self.n, device=self.tdev) if env_mask is None
                   else torch.nonzero(env_mask, as_tuple=False).squeeze(-1))
            if idx.numel() == 0:
                return
            x = torch.rand(idx.shape[0], device=self.tdev) * self._x_spread
            self.qpos_t[idx, 0] = x
            if self.terrain is not None:
                self.qpos_t[idx, 2] = self.terrain.ground_h(x) + self._spawn_dz
            with self.wp.ScopedDevice(self.device):
                self.mjw.forward(self.mw_m, self.mw_d)
            self._prev_q[idx] = self.qpos_t[idx].index_select(1, self.qpos_idx_t)
            feet = self._foot_pos_t()
            self.prev_foot_xy_t[idx] = feet[idx, :, 0:2]

    _ENV_CLS = InEngineQuadWalkEnv
    return _ENV_CLS


# ── the PPO trainer (the standalone trainer's loop, adapted) ─────────────
def _train(world) -> None:
    _ensure_torch_path()
    import torch
    import torch.nn as nn
    _bm = _base_mod()
    NJ, RES_SCALE, MAX_EP = _bm.NJ, _bm.RES_SCALE, _bm.MAX_EP
    DT, SUBSTEPS = _bm.DT, _bm.SUBSTEPS
    robot = _robot()
    g = _ROBOT_GAIT[robot]

    N = _i("QUAD_ENVS", 4096)
    iters = _i("QUAD_ITERS", 400)
    rollout = _i("QUAD_ROLLOUT", 12)
    lr = _f("QUAD_LR", 3e-4)
    save = os.environ.get("RES_POLICY") or str(
        Path(_REPO) / f"projects/policies/training/runs/gpu_{robot}_inengine/policy.pt")
    warmstart = os.environ.get("QUAD_WARMSTART") or None
    ent_coef = _f("QUAD_ENT_COEF", 0.01)

    reward_cfg = dict(
        alive=_f("QUAD_ALIVE", 1.0), upright=_f("QUAD_UPRIGHT", 0.5),
        act=_f("QUAD_ACT", -0.005), act_rate=_f("QUAD_ACT_RATE", -0.01),
        term=_f("QUAD_TERM", -10.0), vel=_f("QUAD_VEL", 2.0),
        vel_sigma=_f("QUAD_VEL_SIGMA", 0.10), vel_l1=_f("QUAD_VEL_L1", -0.3),
        lat=_f("QUAD_LAT", -0.5), yaw=_f("QUAD_YAW", -0.5),
        height=_f("QUAD_HEIGHT", -10.0), max_ep=_i("QUAD_MAX_EP", MAX_EP),
        res_scale=_f("QUAD_RES_SCALE", RES_SCALE),
        seed_gait=bool(_i("QUAD_SEED_GAIT", 1)),
        rest_start_frac=_f("QUAD_REST_START_FRAC", 0.25),
        wz_range=_f("QUAD_WZ_RANGE", 0.0),
        rw_sched=_f("QUAD_RW_SCHED", -5.0), rw_slip=_f("QUAD_RW_SLIP", -0.5),
        foot_z_contact=_f("QUAD_FOOT_Z_CONTACT", 0.05),   # == the standalone default
        foot_z_swing=_f("QUAD_FOOT_Z_SWING", 0.07),       # RAISE on terrain (clearance target)
        vx_cmd_max=_f("QUAD_VX_CMD_MAX", 0.0),
        gait_params=dict(
            vx=_f("QUAD_VX_TARGET", g["vx"]), freq=_f("QUAD_GAIT_FREQ", g["freq"]),
            duty=_f("QUAD_GAIT_DUTY", 0.6),
            step_height=_f("QUAD_GAIT_STEP_H", g["step_h"]),
            body_height=_f("QUAD_GAIT_BODY_H", g["body_h"]),
            ramp_s=_f("QUAD_GAIT_RAMP_S", 1.0)))
    # per-ENV DR only (model-space DR is stripped in the env -- shared live model)
    dr_cfg = dict(
        push_prob=_f("QUAD_DR_PUSH_PROB", 0.01),
        push_vmax=_f("QUAD_DR_PUSH_VMAX", 0.5),
        obs_noise=_f("QUAD_DR_OBS_NOISE", 0.01),
        action_latency_max=_i("QUAD_DR_LATENCY_MAX", 1),
        init_q_band=_f("QUAD_DR_INIT_Q", 0.05),
        init_xy_band=_f("QUAD_DR_INIT_XY", 0.03),
        init_z_band=_f("QUAD_DR_INIT_Z", 0.01),
        init_tilt_band=_f("QUAD_DR_INIT_TILT", 0.0),
        init_vel_band=_f("QUAD_DR_INIT_VEL", 0.0),
        init_vx_bias=_f("QUAD_DR_INIT_VX_BIAS", 0.3))

    _log(f"WALK-QUAD start robot={robot} N={N} iters={iters} rollout={rollout} "
         f"save={save} warmstart={warmstart}")
    _status_write("TRAINING", it=0, iters=iters)

    env = _get_env_cls()(world, N, reward_cfg=reward_cfg, dr_cfg=dr_cfg)
    OBS_IN = env.obs_dim
    _log(f"env ready: robot={robot} OBS_IN={OBS_IN} nominal_FL=({env.nominal[0]:+.3f},"
         f"{env.nominal[1]:+.3f},{env.nominal[2]:+.3f}) vx={env.vx_target} "
         f"freq={env.gp.freq} spawn_z={env.spawn_z:.3f} substeps={SUBSTEPS}")

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_IN, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(OBS_IN, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    tdev = env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    if warmstart and Path(warmstart).exists():
        sd = torch.load(warmstart, map_location=tdev)
        cur = ac.state_dict()
        for k in ("pi.0.weight", "v.0.weight"):   # obs-width warm-start pad (48->49)
            if (k in sd and k in cur and sd[k].shape[0] == cur[k].shape[0]
                    and sd[k].shape[1] < cur[k].shape[1]):
                w = cur[k].clone()
                w[:, :sd[k].shape[1]] = sd[k]
                w[:, sd[k].shape[1]:] = 0.0
                sd[k] = w
        try:
            ac.load_state_dict(sd)
            _log(f"warm-started from {warmstart}")
        except Exception as e:
            _log(f"warm-start failed ({e}); training from scratch")
    opt = torch.optim.Adam(ac.parameters(), lr=lr)

    # EVAL-ONLY: load a checkpoint and run the honest durability eval, no training.
    if _i("QUAD_EVAL_ONLY", 0):
        ckpt = os.environ.get("QUAD_EVAL_CKPT") or save
        if Path(ckpt).exists():
            ac.load_state_dict(torch.load(ckpt, map_location=tdev))
            _log(f"EVAL-ONLY: loaded {ckpt}")
        else:
            _log(f"EVAL-ONLY: ckpt NOT FOUND {ckpt} -- evaluating the init/warm-start net")
        _eval(env, ac, tdev)
        _status_write("DONE", eval_only=True)
        return

    Path(save).parent.mkdir(parents=True, exist_ok=True)
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    for it in range(1, iters + 1):
        for k in range(rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                d = torch.distributions.Normal(mu, log_std.exp())
                a = d.sample()
                logp = d.log_prob(a).sum(-1)
            obs_buf[k] = obs
            act_buf[k] = a
            logp_buf[k] = logp
            val_buf[k] = v
            obs, r, done, _ = env.step(a)
            rew_buf[k] = r
            done_buf[k] = done.float()
            total_steps += N

        with torch.no_grad():
            _, last_v, _ = ac(obs)
        gamma, lam = 0.99, 0.95
        adv = torch.zeros_like(rew_buf)
        last_gae = torch.zeros(N, device=tdev)
        for k in reversed(range(rollout)):
            nonterm = 1.0 - done_buf[k]
            nextv = last_v if k == rollout - 1 else val_buf[k + 1]
            delta = rew_buf[k] + gamma * nextv * nonterm - val_buf[k]
            last_gae = delta + gamma * lam * nonterm * last_gae
            adv[k] = last_gae
        ret = adv + val_buf
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        obs_flat = obs_buf.reshape(-1, OBS_IN)
        act_flat = act_buf.reshape(-1, NJ)
        logp_flat = logp_buf.reshape(-1)
        adv_flat = adv.reshape(-1)
        ret_flat = ret.reshape(-1)
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat)
            d = torch.distributions.Normal(mu, log_std.exp())
            new_logp = d.log_prob(act_flat).sum(-1)
            ratio = (new_logp - logp_flat).exp()
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 0.8, 1.2) * adv_flat
            pi_loss = -torch.min(surr1, surr2).mean()
            v_loss = ((v - ret_flat) ** 2).mean()
            ent = d.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - ent_coef * ent
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()

        if it % 5 == 0 or it == 1:
            fps = total_steps / max(time.time() - t0, 1e-6)
            _log(f"WALK-QUAD it={it:4d} ep_rew/step~{rew_buf.mean().item():+.3f} "
                 f"meanV {val_buf.mean().item():+.2f} steps {total_steps:,} "
                 f"{fps:,.0f} env-steps/s")
            _status_write("TRAINING", it=it, iters=iters,
                          rew=float(rew_buf.mean().item()))

    torch.save(ac.state_dict(), save)
    _log(f"saved {save} ({total_steps:,} steps in {time.time() - t0:.1f}s)")

    _eval(env, ac, tdev)
    _export_onnx(ac, AC, OBS_IN, save)
    _status_write("DONE", saved=save)


def _eval(env, ac, tdev, steps=None) -> None:
    """HONEST durability eval: raise max_ep above the eval horizon first, so a
    `done` means a REAL FALL and not the routine episode-timeout reset
    (env.step sets done = fall | ep_step >= max_ep). Without this, first-fall
    pins to MAX_EP*DT (11.9 s) and never_fell reads 0% for a policy that never
    actually fell -- and a shorter-than-MAX_EP eval reads a bogus 100%."""
    import torch
    DT = _base_mod().DT
    steps = steps or _i("QUAD_EVAL_STEPS", 1500)
    _saved_max_ep = env.max_ep
    env.max_ep = steps + 10
    ac.eval()
    obs = env.reset()
    first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
    dist = torch.zeros(env.n, device=tdev)
    vx_sum = torch.zeros(env.n, device=tdev)
    alive_steps = torch.zeros(env.n, device=tdev)
    for step in range(steps):
        with torch.no_grad():
            mu, _, _ = ac(obs)
        alive = (first_fall == 0)
        obs, _, done, _ = env.step(mu)
        step_alive = (alive & (~done)).float()
        vx = env.qvel_t[:, 0]
        dist += vx * DT * step_alive
        vx_sum += vx * step_alive
        alive_steps += step_alive
        newly = done & (first_fall == 0)
        first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
    ac.train()
    env.max_ep = _saved_max_ep
    ff = first_fall.cpu().numpy()
    d = dist.cpu().numpy()
    mean_vx = (vx_sum / torch.clamp(alive_steps, min=1.0)).cpu().numpy()
    never = (ff == 0)
    ff_fell = ff[~never]
    _log(f"IN-ENGINE EVAL robot={_robot()} envs={env.n} steps={steps} "
         f"({steps * DT:.0f}s): first-fall mean="
         f"{(ff_fell.mean() * DT) if ff_fell.size else -1:.2f}s "
         f"never_fell={never.mean():.1%} | fwd dist mean={d.mean():.2f}m "
         f"max={d.max():.2f}m | speed alive={mean_vx.mean():.3f} m/s "
         f"(target {env.vx_target})")


def _export_onnx(ac, AC, OBS_IN, save) -> None:
    import torch
    onnx_path = str(Path(save).with_suffix(".onnx"))

    class DeployPolicy(torch.nn.Module):
        def __init__(self, pi):
            super().__init__()
            self.pi = pi

        def forward(self, obs):
            return torch.clamp(self.pi(obs), -1.0, 1.0)   # the exact training squash

    cpu_ac = AC()
    cpu_ac.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
    wrapped = DeployPolicy(cpu_ac.pi)
    wrapped.eval()
    dummy = torch.zeros(1, OBS_IN, dtype=torch.float32)
    try:
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            torch.onnx.export(
                wrapped, dummy, onnx_path,
                input_names=["obs"], output_names=["action"],
                dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                opset_version=17)
        _log(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        _log(f"ONNX export failed (non-fatal): {e}")


# ── the in-engine hook (OMNISIM_INENGINE_PYMOD entry point; stays LIGHT) ──
def quad_walk_recipe_step(world):
    """Called every engine tick. No-ops until the model is warm, then runs the
    ENTIRE PPO loop once (blocking) and signals DONE for the watchdog."""
    if getattr(world, "_qwr_started", False):
        return
    world._qwr_ticks = getattr(world, "_qwr_ticks", 0) + 1
    if world._qwr_ticks < _i("QUAD_WARM_TICKS", 30):
        return
    world._qwr_started = True
    try:
        _train(world)
    except Exception as e:
        import traceback
        _log(f"WALK-QUAD ERROR: {e!r}")
        _log(traceback.format_exc())
        _status_write("DONE", error=repr(e))
