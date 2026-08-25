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

"""IN-ENGINE quad WALK trainer (go2 | omniquad | b2) -- the in-engine port of the
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
__init__ builds -- omniquad's `prog_lag_t`, each robot's gait/limits/foot bodies --
is inherited for free, and NOTHING is copied. Swapping QUAD_ROBOT swaps the base
module and therefore the robot.

ADDITIVE-ONLY / ZERO-REGRESSION: NEW file. It imports (never edits) the
standalone trainers, so go2/omniquad/b2 standalone paths stay byte-identical.
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
import math
import os
import sys
import time
from pathlib import Path

_REPO = os.environ.get("OMNISIM_HOME") or str(
    next(_p for _p in Path(__file__).resolve().parents
         if (_p / "AGENTS.md").exists() or (_p / ".git").exists()))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from projects.policies.common import shadow_env as SE   # ONE env contract for Shadowing


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


# ── robot selection: QUAD_ROBOT = go2 | omniquad | b2 ────────────────────────
# All three standalone trainer modules expose the SAME names (BatchedSpotWalkEnv,
# NJ, RES_SCALE, DT, SUBSTEPS, MAX_EP, ... and each imports its OWN
# <robot>_trot_gait as `stg`). Swapping the base module swaps the robot.
_ROBOT_GAIT = {   # defaults == the per-robot deploy launchers
    "go2":  dict(vx=0.4, freq=1.8, step_h=0.05, body_h=0.30),
    "omniquad": dict(vx=0.4, freq=1.4, step_h=0.06, body_h=0.55),
    "b2":   dict(vx=0.5, freq=1.3, step_h=0.08, body_h=0.50),
}

# The CONTROLLER joint order per robot -- what a ghost lut must declare in `joints`, and the one
# thing the ghost path used to hardcode to the Go2 (which is why no other quadruped could be
# Shadowing-trained). Unitree quads: FL/FR/RL/RR x (hip, thigh, calf) with a `_joint` suffix.
# OmniLink: front_left/... x (hip_x, hip_y, knee), NO suffix.
_QUAD_JOINTS = {
    "go2": [f"{leg}_{p}_joint" for leg in ("FL", "FR", "RL", "RR")
            for p in ("hip", "thigh", "calf")],
    "b2":  [f"{leg}_{p}_joint" for leg in ("FL", "FR", "RL", "RR")
            for p in ("hip", "thigh", "calf")],
    "omniquad": [f"{leg}_{p}" for leg in ("front_left", "front_right", "rear_left", "rear_right")
                 for p in ("hip_x", "hip_y", "knee")],
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
        source redirected, so all robot-specific state it builds (omniquad's
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

            # ── QUAD_FAST_RESET=1 (default OFF): the base _reset_envs calls
            # mjw.forward (FULL pipeline incl. collision) just to refresh
            # xpos/xquat for _foot_pos_t/_prev_q. The convex narrowphase inside
            # it wp.empty()s an EPA scratch scaled by nconmax*nworld EVERY call
            # (1.44 GB @ K=16384 -- the measured OOM), and the full pipeline
            # runs on ALL K worlds on every rollout step with >=1 fall.
            # mjw.kinematics computes the SAME xpos/xquat (forward's own first
            # phase) with no collision and no big allocs. Physics diff: the
            # first post-reset substep reuses a stale solver warmstart
            # (microscopic, solver-convergence-level) -- hence the flag.
            if _i("QUAD_FAST_RESET", 0):
                _kin = getattr(mjw, "kinematics", None)
                if _kin is None:
                    try:
                        from mujoco_warp._src import smooth as _mjw_smooth
                        _kin = _mjw_smooth.kinematics
                    except Exception:
                        _kin = None
                if _kin is not None:
                    class _ResetKinShim:
                        """mjw passthrough; .forward -> kinematics (reset path
                        is the ONLY base-env caller of self.mjw.forward)."""
                        def __init__(self, mod, kin):
                            self._mod, self._kin = mod, kin

                        def __getattr__(self, k):
                            return getattr(self._mod, k)

                        def forward(self, m, d):
                            return self._kin(m, d)
                    self.mjw = _ResetKinShim(self.mjw, _kin)
                    _log("FAST_RESET: reset-path mjw.forward -> mjw.kinematics "
                         "(no collision, no EPA scratch alloc)")
                else:
                    _log("FAST_RESET requested but mjw.kinematics not found; "
                         "keeping mjw.forward")

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

    # ── SHADOWING mode (QUAD_GHOST=<lut.json>, default OFF -> legacy byte-identical) ──
    # The quad port of the G1 flagship's ghost machinery, minimal-first (owner
    # campaign 2026-07-12): the corridor centre becomes a CERTIFIED, ACHIEVED
    # ghost lut (recorded from the real dynamics, gates 1-3 + validator passed
    # BEFORE training -- build_go2_shadow_ghost.py) instead of the analytic
    # kinematic trot. The action architecture is untouched: targets =
    # ghost(phase) + a * res_scale, so res_scale IS the corridor half-width
    # (QUAD_RES_SCALE; corridor-vs-torque law: it must exceed the ghost's own
    # peak |ctrl - q| unless QUAD_GHOST_FF=1 shifts the centre by the ghost's
    # declared feedforward). Obs layout is UNCHANGED (48-dim) -- the ghost
    # enters implicitly via clock + corridor + reward, exactly the pre-REF_OBS
    # G1 recipe. Deploy through go2_shadow_deploy (same contract, lut baseline).
    if not SE.ghost():      # SHADOW_GHOST | QUAD_GHOST | GHOST_LUT_JSON
        _ENV_CLS = InEngineQuadWalkEnv
        return _ENV_CLS

    class GhostQuadWalkEnv(InEngineQuadWalkEnv):
        """Corridor centre = the ghost lut; adds RSI-on-reference, a gmatch
        tracking reward (QUAD_W_GMATCH) and the gmatch readout for eval."""

        _TROT_OFF = (0.0, 0.5, 0.5, 0.0)      # FL, FR, RL, RR (trot diagonal pairs)

        def __init__(self, world, n, reward_cfg=None, dr_cfg=None):
            super().__init__(world, n, reward_cfg=reward_cfg, dr_cfg=dr_cfg)
            gp_path = SE.ghost()
            gd = json.loads(open(gp_path).read())
            # ⛔ the WBMATCH name-trap rule: verify the joint mapping EXPLICITLY -- never accept
            # the positional fallback. But verify it against THIS ROBOT, not against the Go2:
            # the expected order used to be hardcoded as FL/FR/RL/RR x (hip,thigh,calf), which is
            # the Go2's. A OmniQuad lut (front_left_hip_x, ...) hard-failed here, and that ONE list
            # was the entire reason Shadowing could not be trained on any quadruped but the Go2.
            # The order now comes from the robot's own trainer module (the same JOINT_NAMES the
            # deploy and the MJCF use), so a new quadruped needs no edit here at all.
            _bm = _base_mod()
            exp = [str(j) for j in getattr(_bm, "JOINT_NAMES", [])] or _QUAD_JOINTS[_robot()]
            # `joints` carries the REAL model joint names. (The Go2's lut briefly carried FAKE
            # humanoid aliases -- "FL_hip_roll_joint" -- so that a humanoid-name-keyed validator
            # gate would engage, with the real names hidden in `joints_go2`. The validator pairs
            # mirrors by name now, so the alias is gone; the fallback reads a stale lut.)
            got = list(gd.get("joints") or gd.get("joints_go2") or [])
            if got != exp:
                raise RuntimeError(
                    f"QUAD_GHOST joint-order mismatch for robot={_robot()}: lut joints={got} != "
                    f"controller order {exp} -- refusing the positional fallback")
            _lrob = str(gd.get("robot") or "").strip().lower()
            if _lrob and _lrob != _robot():
                raise RuntimeError(
                    f"QUAD_GHOST is a ghost for {_lrob!r} but QUAD_ROBOT={_robot()!r}")
            self._ghost_nb = int(gd["nb"])
            self._ghost_leg = torch.tensor(gd["leg_lut"], dtype=torch.float32,
                                           device=self.tdev)
            gfreq = float(gd.get("freq", self.gp.freq))
            DTL = _base_mod().DT
            self.phase_dt = 2.0 * math.pi * gfreq * DTL     # lut clock == the clock
            self._dtl = DTL
            self._ghost_ff = None
            if SE.ff():
                if "ffdq_lut" in gd:
                    self._ghost_ff = torch.tensor(gd["ffdq_lut"], dtype=torch.float32,
                                                  device=self.tdev)
                else:
                    _log("QUAD_GHOST_FF=1 but the lut carries no ffdq_lut -- ignored")
            self._gsig = _f("QUAD_GHOST_SIG", 0.35)
            self._wgm = _f("QUAD_W_GMATCH", 0.0)
            # QUAD_W_TAU: the servo-effort (foldability) penalty -- see step(). 0 = the old
            # behaviour exactly, so every previously-trained config is bit-unchanged.
            self._wtau = _f("QUAD_W_TAU", 0.0)
            # ⭐ QUAD_W_SATHINGE (2026-07-17): hinge penalty on the torque-saturation PEAK.
            # W_TAU prices RMS servo effort and was PROVEN (9dbe57c7) not to restore
            # ghost-foldability: serr rms barely moved (0.0429 -> 0.0420 across W_TAU
            # 0..50) because un-foldability comes from PEAK saturation -- the fraction of
            # steps the commanded torque rides the actuator limit (OmniQuad's legacy champion:
            # 210/750 = 28% vs the Go2's 3%; its fold FAILS closure p95 81.1 mm vs the
            # < 20 mm gate). This term prices ONLY the part of |tau_proxy| ABOVE
            # QUAD_SATHINGE_FRAC (default 0.85) of each joint's torque limit -- zero below
            # the threshold (the mean is untouched), quadratic in the violation above it.
            # tau_proxy = kp * (cmd - q), the SAME (cmd - q) tensor W_TAU prices. kp and
            # tau_limit are READ FROM THE LIVE COMPILED MODEL (position-actuator gainprm/
            # biasprm + jnt_actfrcrange -- the identical source build_quad_shadow_ghost.py's
            # _model_maps and its saturation gate use), never re-declared here: the
            # single-source-of-truth rule (docs/developer/g1-single-source-of-truth.md).
            self._wsat = _f("QUAD_W_SATHINGE", 0.0)
            self._sat_hinge_frac = _f("QUAD_SATHINGE_FRAC", 0.85)
            import mujoco as _mj
            _kp, _lim = [], []
            for i in range(len(exp)):
                a = int(self.controller_to_ctrl_pos[i])
                # position actuator: gainprm[0] = kp, biasprm[1] = -kp (the DR code keys
                # on biasprm[1] too); take whichever is populated.
                _kp.append(max(float(self.mjm.actuator_gainprm[a][0]),
                               abs(float(self.mjm.actuator_biasprm[a][1]))))
                qa = int(self.controller_to_qpos[i])
                lim = 1e9   # no declared limit -> ratio ~0, hinge inert (builder's dof_lim convention)
                for j in range(int(self.mjm.njnt)):
                    if (self.mjm.jnt_type[j] == _mj.mjtJoint.mjJNT_HINGE
                            and int(self.mjm.jnt_qposadr[j]) == qa):
                        if self.mjm.jnt_actfrclimited[j]:
                            r_ = self.mjm.jnt_actfrcrange[j]
                            lim = max(1e-3, max(abs(float(r_[0])), abs(float(r_[1]))))
                        elif self.mjm.actuator_forcelimited[a]:
                            fr = self.mjm.actuator_forcerange[a]
                            lim = max(1e-3, max(abs(float(fr[0])), abs(float(fr[1]))))
                        break
                _lim.append(lim)
            self._sat_kp_t = torch.tensor(_kp, dtype=torch.float32, device=self.tdev)
            self._sat_lim_t = torch.tensor(_lim, dtype=torch.float32, device=self.tdev)
            # sat_frac READOUT accumulators (0-dim GPU tensors: no per-step host sync).
            # Accumulated ALWAYS, penalised only when W_SATHINGE != 0 -- the same
            # always-measure rule as serr: a metric you only collect when you are
            # optimising it is useless as a baseline.
            self._sat_above_t = torch.zeros((), device=self.tdev)
            self._sat_samples_t = torch.zeros((), device=self.tdev)
            self._satpen_sum_t = torch.zeros((), device=self.tdev)
            self._satpen_n_t = torch.zeros((), device=self.tdev)
            self.last_satpen_t = None
            if max(_lim) >= 1e9:
                _log("SATHINGE WARNING: >=1 joint has NO actuator force limit in the "
                     "compiled model (lim=1e9) -- it can never register saturation")
            _log(f"SATHINGE: W_SATHINGE={self._wsat} frac={self._sat_hinge_frac} "
                 f"kp={[round(v, 1) for v in _kp]} tau_lim={[round(v, 1) for v in _lim]}")
            # ⭐ QUAD_W_FOOTSLIP (2026-07-17): THE STANCE-FOOT-SLIP penalty -- the designed
            # intervention for the Go2 Shadowing ladder's CLOSURE wall. Measured (d6846147):
            # projecting the round-2 champion through the ghost FAILS the round-3 fold on
            # CLOSURE (planted-foot world drift p95 28.3 mm vs the < 20 mm gate), and
            # QUAD_W_SATHINGE PROVED this is NOT torque saturation (the hinge cut saturation
            # ~4x -- sat_frac 0.065->0.0145 -- with the closure p95 UNMOVED, 28.0->28.3 mm).
            # Closure = a stance foot SLIDING instead of staying planted; the r2 gait is
            # faster (0.415 m/s) than the r1 gait that folded cleanly (16.5 mm @ 0.318 m/s),
            # and faster gaits slip more. So price the slip directly:
            #     penalty = mean over the 4 feet of (in_contact * |v_xy|^2)
            # i.e. squared horizontal foot speed WHILE PLANTED. Training then prefers gaits
            # whose stance feet stay put -> lower FOLDED closure -> foldable -> the ladder
            # climbs again. This is the measured mechanism turned into a reward term, exactly
            # like W_TAU/W_SATHINGE (docs/developer/shadow-iteration.md).
            #   * FEET come from self._foot_pos_t(), which uses the ENGINE-derived calf bodies
            #     (self._calf_bodies) + the base module's FOOT_OFFSET -- already robot-correct
            #     for go2|omniquad|b2, so there are NO hardcoded per-robot foot indices here.
            #   * STANCE = foot world-z below QUAD_FOOTSLIP_VZ (ground-relative on terrain),
            #     the SAME z-threshold notion the closure gate itself uses
            #     (build_quad_shadow_ghost.py: `planted = z < z.min()+0.015`), so the penalty
            #     and the gate agree on what "planted" means -- the point of the intervention.
            #   * v_xy is the DT finite-difference of the foot xy (self._dtl == the base env's
            #     DT, the same difference the base's rw_slip uses). A just-reset env teleported
            #     this step (not slip), so it is masked out of BOTH penalty and readout by done.
            # Default 0 == the reward is BIT-IDENTICAL; the mean stance-slip SPEED is
            # accumulated and logged ALWAYS (even at W=0) as the closure-precursor readout --
            # the same always-measure rule as sat_frac/serr (a metric you only collect while
            # optimising it is useless as a baseline).
            self._wfootslip = _f("QUAD_W_FOOTSLIP", 0.0)
            self._footslip_vz = _f("QUAD_FOOTSLIP_VZ", 0.02)
            self._fs_prev_xy = self._foot_pos_t()[:, :, 0:2].detach().clone()
            self._fs_speed_sum_t = torch.zeros((), device=self.tdev)   # Σ in_contact·|v_xy|
            self._fs_contact_n_t = torch.zeros((), device=self.tdev)   # Σ in_contact samples
            self._fs_pen_sum_t = torch.zeros((), device=self.tdev)     # Σ per-step mean penalty
            self._fs_pen_n_t = torch.zeros((), device=self.tdev)
            self.last_footslip_t = None
            # ⭐ QUAD_WZ_FIXED (2026-07-17, TRAIN_PLAN.md §4 -- the go2_turn upgrade hook): pin
            # the wz command (obs slot 48 / index 47, AND the r_yaw target) to a constant so a
            # turn policy sees a steady yaw target and r_yaw = QUAD_YAW*|wz - wz_cmd| becomes a
            # true yaw-TRACKING reward. Unset (None) == BIT-IDENTICAL (the base keeps wz_cmd
            # from QUAD_WZ_RANGE, default 0). Applied on every reset in _reset_envs below.
            _wzf = os.environ.get("QUAD_WZ_FIXED")
            self._wz_fixed = float(_wzf) if _wzf not in (None, "") else None
            self.last_serr_t = None
            self.last_gmatch_t = torch.ones(n, device=self.tdev)
            _log(f"GHOST=RECORDED {os.path.basename(gp_path)}: nb={self._ghost_nb} "
                 f"freq={gfreq}Hz corridor(res_scale)={self.res_scale} "
                 f"ff={'ON' if self._ghost_ff is not None else 'off'} "
                 f"W_GMATCH={self._wgm} sig={self._gsig} vx_ref={gd.get('vx')}")

        def _lut(self, table, ph):
            """Circular linear interpolation of a (nb, 12) table at phase ph (n,)."""
            x = torch.remainder(ph, 2.0 * math.pi) / (2.0 * math.pi) * self._ghost_nb
            b0 = torch.floor(x)
            f = (x - b0).unsqueeze(1)
            i0 = b0.long() % self._ghost_nb
            i1 = (i0 + 1) % self._ghost_nb
            return table[i0] * (1.0 - f) + table[i1] * f

        def _ramp_frac(self):
            t_since = self._ramp_t0 + self.ep_step_t.to(torch.float32) * self._dtl
            if self.gp.ramp_s <= 0:
                return torch.ones_like(t_since)
            return torch.clamp(t_since / self.gp.ramp_s, 0.0, 1.0)

        def _ghost_center(self):
            """Ramped reference POSE (rewards/metric score this, never the command)."""
            legs = self._lut(self._ghost_leg, self.phase_t)
            r = self._ramp_frac().unsqueeze(1)
            nom = self.nominal_t.unsqueeze(0)
            return nom + r * (legs - nom)

        def _baseline_targets_t(self):
            """Ghost corridor centre + analytic swing gates at the same clock.
            (The lut freq == the gait freq by construction; swings are the same
            phase gates the legacy trot uses, no IK needed.)"""
            legs = self._ghost_center()
            r = self._ramp_frac()
            if self._ghost_ff is not None:      # GHOST-FF: centre += tau_ff/kp
                legs = legs + r.unsqueeze(1) * self._lut(self._ghost_ff, self.phase_t)
            # analytic swing weights (stg's swing math, sans IK), scaled by the ramp
            phi = torch.remainder(self.phase_t / (2.0 * math.pi), 1.0)
            duty = float(self.gp.duty)
            swings = torch.zeros(self.n, 4, device=self.tdev)
            for i, off in enumerate(self._TROT_OFF):
                lp = torch.remainder(phi + off, 1.0)
                s_sw = torch.clamp((lp - duty) / (1.0 - duty), 0.0, 1.0)
                sw = torch.sin(math.pi * s_sw)
                swings[:, i] = torch.where(lp < duty, torch.zeros_like(sw), sw)
            swings = swings * r.unsqueeze(1)
            if self.vx_cond:                    # same VC blend as the base env
                s = torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3),
                                0.0, 1.25).unsqueeze(1)
                nom = self.nominal_t.unsqueeze(0)
                legs = nom + s * (legs - nom)
                swings = swings * s
            return legs, swings

        def _reset_envs(self, env_mask=None):
            """Base reset, then RSI-ON-REFERENCE: mid-stride worlds spawn on the
            GHOST pose (q AND qd from the lut) instead of the analytic trot."""
            super()._reset_envs(env_mask)
            # ── QUAD_WZ_FIXED (TRAIN_PLAN.md §4): pin the yaw command on EVERY reset, BEFORE
            # the RSI early-returns below (those skip rest-start / seed_gait-off resets), so
            # obs slot 48 always carries the steady turn target. getattr-guarded for the
            # construction-time _reset_all call (_wz_fixed not set yet -> None -> skip).
            # None == no override (default, bit-identical).
            if getattr(self, "_wz_fixed", None) is not None:
                widx = (torch.arange(self.n, device=self.tdev) if env_mask is None
                        else torch.nonzero(env_mask, as_tuple=False).squeeze(-1))
                self.wz_cmd_t[widx] = self._wz_fixed
            # During super().__init__'s _reset_all the lut is not loaded yet
            # (the base class's _calf_bodies init-ordering trap, same shape);
            # env.reset() re-seeds on-reference once construction completes.
            if getattr(self, "_ghost_leg", None) is None or not self.seed_gait:
                return
            idx = (torch.arange(self.n, device=self.tdev) if env_mask is None
                   else torch.nonzero(env_mask, as_tuple=False).squeeze(-1))
            if idx.numel() == 0:
                return
            mid = idx[self._ramp_t0[idx] > 1.0]     # mid-stride only (rest-starts keep nominal)
            if mid.numel() == 0:
                return
            ph = self.phase_t[mid]
            legs0 = self._lut(self._ghost_leg, ph)
            legs1 = self._lut(self._ghost_leg, ph + self.phase_dt)
            qd_ref = (legs1 - legs0) / self._dtl
            bi = mid.unsqueeze(1).expand(-1, legs0.shape[1])
            self.qpos_t[bi, self.qpos_idx_t.unsqueeze(0).expand(mid.shape[0], -1)] = legs0
            self.qvel_t[bi, self.qvel_idx_t.unsqueeze(0).expand(mid.shape[0], -1)] = qd_ref
            with self.wp.ScopedDevice(self.device):
                self.mjw.forward(self.mw_m, self.mw_d)   # FAST_RESET shim applies
            self._prev_q[mid] = legs0
            feet = self._foot_pos_t()
            self.prev_foot_xy_t[mid] = feet[mid, :, 0:2]

        def step(self, action_t):
            # ⭐ FOLDABLE-BY-CONSTRUCTION (QUAD_W_TAU, 2026-07-13). THE SERVO-EFFORT PENALTY.
            #
            # Shadowing folds a champion's ACHIEVED gait into a kinematic, phase-indexed ghost.
            # Measured (docs/developer/shadow-iteration.md): the round-2 ghost FAILED its closure
            # and support gates because the champion had learned to buy speed by RIDING ITS TORQUE
            # LIMIT (41/750 steps at 100% peak). A gait that only exists at the actuator limit
            # cannot be expressed as a position reference -- fold it and a bare PD servo can no
            # longer execute it: the planted feet slip. The very thing that made the policy better
            # made its gait UN-GHOSTABLE, and that is what stops the iteration compounding.
            #
            # So price the thing that makes a gait un-foldable. In a PD position servo
            #     tau ~= kp * (cmd - q)
            # and |cmd - q| is EXACTLY the quantity the corridor law and the closure gate measure
            # (build_go2_shadow_ghost.py prints it as "the corridor floor"). Penalising it pushes
            # the policy toward a gait the servo can actually track -- i.e. one that FOLDS.
            #
            # This is not a regulariser bolted on for neatness: it is the measured mechanism,
            # turned into a reward term. If the hypothesis is right, a champion trained with
            # W_TAU > 0 yields a ghost that CLOSES, and the Shadowing iteration compounds.
            # computed ALWAYS (not only when penalised): at W_TAU=0 it is the foldability
            # READOUT of the unmodified recipe, which is the baseline the hypothesis is tested
            # against. A metric you only collect when you are optimising it is useless.
            _cmd = self._ghost_center() + self.res_scale * torch.clamp(action_t, -1.0, 1.0)
            obs, rew, done, info = super().step(action_t)
            # gmatch: achieved pose vs the (ramped) reference POSE at the current
            # clock -- the ghost-similarity readout, and optionally a reward term.
            q = self.qpos_t.index_select(1, self.qpos_idx_t)
            err2 = ((q - self._ghost_center()) ** 2).mean(dim=1)
            gm = torch.exp(-err2 / (self._gsig * self._gsig))
            self.last_gmatch_t = gm
            if self._wgm != 0.0:
                rew = rew + self._wgm * gm
            # the servo error the actuator actually had to close, this tick == (tau / kp)
            serr = ((_cmd - q) ** 2).mean(dim=1)
            self.last_serr_t = serr
            if self._wtau != 0.0:
                rew = rew - self._wtau * serr
            # ⭐ SATURATION HINGE (QUAD_W_SATHINGE, 2026-07-17). W_TAU's RMS penalty barely
            # moves the PEAK (measured, 9dbe57c7) -- this term prices ONLY the peak.
            # tau_ratio = |kp*(cmd-q)| / tau_limit per (env, joint), pre-clamp on purpose:
            # a command the actuator cannot deliver is exactly the un-foldable demand.
            # relu(ratio - FRAC)^2: identically zero for every sample below FRAC*tau_limit
            # (mean effort untouched), quadratic in the violation above it. sat_frac is
            # accumulated even at W_SATHINGE=0 -- the campaign's foldability readout.
            tau_ratio = (self._sat_kp_t * (_cmd - q)).abs() / self._sat_lim_t
            over = torch.relu(tau_ratio - self._sat_hinge_frac)
            satpen = (over * over).mean(dim=1)
            self.last_satpen_t = satpen
            self._sat_above_t += (tau_ratio > self._sat_hinge_frac).float().sum()
            self._sat_samples_t += float(tau_ratio.numel())
            self._satpen_sum_t += satpen.mean()
            self._satpen_n_t += 1.0
            if self._wsat != 0.0:
                rew = rew - self._wsat * satpen
            # ⭐ STANCE-FOOT-SLIP (QUAD_W_FOOTSLIP): the closure-wall term. Feet from the
            # ENGINE-derived calf bodies via _foot_pos_t (robot-correct for go2|omniquad|b2);
            # stance = foot world-z below QUAD_FOOTSLIP_VZ (ground-relative on terrain, == the
            # closure gate's `planted` notion); v_xy = the DT finite-diff of the foot xy. A
            # just-reset env (done) teleported this step, which is not slip -> mask it from
            # BOTH the penalty and the readout via (~done).
            feet_fs = self._foot_pos_t()                       # (n,4,3)
            fs_z = feet_fs[:, :, 2]
            fs_xy = feet_fs[:, :, 0:2]
            if self.terrain is not None:                       # ground-relative, same as rw_slip
                fs_z = fs_z - self.terrain.ground_h(
                    fs_xy[:, :, 0].reshape(-1)).reshape(self.n, 4)
            v_xy = (fs_xy - self._fs_prev_xy) / self._dtl
            self._fs_prev_xy = fs_xy.detach().clone()
            in_contact = (fs_z < self._footslip_vz).float() * (~done).float().unsqueeze(1)
            fs_speed = torch.linalg.norm(v_xy, dim=2)          # horizontal foot speed (m/s)
            footslip = (in_contact * (v_xy * v_xy).sum(dim=2)).mean(dim=1)   # mean over 4 feet
            self.last_footslip_t = footslip
            self._fs_speed_sum_t += (in_contact * fs_speed).sum()
            self._fs_contact_n_t += in_contact.sum()
            self._fs_pen_sum_t += footslip.mean()
            self._fs_pen_n_t += 1.0
            if self._wfootslip != 0.0:
                rew = rew - self._wfootslip * footslip
            return obs, rew, done, info

    _ENV_CLS = GhostQuadWalkEnv
    return _ENV_CLS


# ── profiling (QUAD_PROFILE=1): per-phase wall-clock split of an iteration ──
# Sections use torch.cuda.synchronize() fences, so the ABSOLUTE fps of a
# profiled run is lower than an unprofiled one -- the value is the BREAKDOWN.
def _mk_prof(env, torch):
    import collections
    import contextlib

    class _Prof:
        def __init__(self):
            self.t = collections.defaultdict(float)
            self.c = collections.defaultdict(int)
            self.reset_calls = 0
            self.resets = 0

        @contextlib.contextmanager
        def sec(self, name):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            try:
                yield
            finally:
                torch.cuda.synchronize()
                self.t[name] += time.perf_counter() - t0
                self.c[name] += 1

        def dump(self, label):
            tot = sum(v for k, v in self.t.items()
                      if k in ("policy", "env_step", "buf", "gae", "update"))
            parts = " ".join(
                f"{k}={v * 1e3:.0f}ms/{self.c[k]}x"
                for k, v in sorted(self.t.items(), key=lambda kv: -kv[1]))
            try:
                mem = f" torch_mem={torch.cuda.memory_allocated() / 2**20:.0f}MB" \
                      f" torch_resv={torch.cuda.memory_reserved() / 2**20:.0f}MB"
                free, total = torch.cuda.mem_get_info()
                mem += f" gpu_free={free / 2**20:.0f}MB/{total / 2**20:.0f}MB"
            except Exception:
                mem = ""
            try:   # contact-capacity probe (are slim MPC_NCONMAX buffers ample?)
                import numpy as _np
                nc = getattr(env, "mw_d", None)
                if nc is not None and hasattr(nc, "ncon"):
                    mem += f" ncon={_np.asarray(nc.ncon.numpy()).sum()}"
            except Exception:
                pass
            _log(f"PROFILE {label} top_total={tot * 1e3:.0f}ms | {parts} | "
                 f"reset_calls={self.reset_calls} envs_reset={self.resets}{mem}")
            self.t.clear()
            self.c.clear()
            self.reset_calls = 0
            self.resets = 0

    p = _Prof()

    # sub-timers INSIDE env.step (nested in the env_step section; they overlap
    # env_step, never each other except _foot_pos_t inside reset).
    for nm in ("_baseline_targets_t", "_build_obs_t", "_foot_pos_t"):
        orig = getattr(env, nm)

        def _mk(orig, nm):
            def w(*a, **k):
                with p.sec(nm.lstrip("_")):
                    return orig(*a, **k)
            return w
        setattr(env, nm, _mk(orig, nm))

    orig_reset = env._reset_envs

    def _reset_w(env_mask=None):
        p.reset_calls += 1
        if env_mask is not None:
            p.resets += int(env_mask.sum().item())
        else:
            p.resets += env.n
        with p.sec("reset"):
            return orig_reset(env_mask)
    env._reset_envs = _reset_w

    class _ModProxy:
        """Delegates to a module, timing the named callables."""
        def __init__(self, mod, names):
            self._mod = mod
            self._names = names

        def __getattr__(self, k):
            v = getattr(self._mod, k)
            if k in self._names and callable(v):
                key = self._names[k]

                def w(*a, **kw):
                    with p.sec(key):
                        return v(*a, **kw)
                return w
            return v

    env.wp = _ModProxy(env.wp, {"capture_launch": "physics"})
    env.mjw = _ModProxy(env.mjw, {"step": "physics", "forward": "reset_fk"})
    return p


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
        res_scale=SE.residual(RES_SCALE),
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

    # Numerics-identical, unconditional: torch.distributions validates args by
    # default (torch.all(...) constraint checks -> a HOST SYNC on every Normal()
    # construction: 12+/iter in the rollout, 4/iter in the update). Validation
    # only raises on invalid inputs; the sampled values / log-probs / entropy
    # are bit-identical with it off.
    torch.distributions.Distribution.set_default_validate_args(False)

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
    # QUAD_SEED: the PPO seed was HARDCODED to 0, so two runs of the same config were the same run
    # and you could not measure the training noise. Any A/B between two trained policies is
    # worthless without it: a difference that is really seed noise looks exactly like a result.
    torch.manual_seed(_i("QUAD_SEED", 0))
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

    # ── QUAD_FAST_UPDATE=1 (default OFF): faster PPO-update path.
    #    - fused Adam (same algorithm; fp reduction order may differ)
    #    - zero_grad(set_to_none=True)
    #    - manual gaussian log-prob/entropy (the exact Normal formulas,
    #      fewer kernel launches)
    #    QUAD_MINIBATCH=<n> (default 0 = full batch, requires FAST_UPDATE):
    #      per-epoch shuffled minibatches -> more optimizer steps/iter; this
    #      CHANGES the training math (that is why it is opt-in).
    #    QUAD_BF16=1 (default OFF, requires FAST_UPDATE): bf16 autocast for
    #      the update fwd/bwd only (rollout + physics stay fp32).
    fast_update = bool(_i("QUAD_FAST_UPDATE", 0))
    minibatch = _i("QUAD_MINIBATCH", 0) if fast_update else 0
    use_bf16 = bool(_i("QUAD_BF16", 0)) and fast_update
    if fast_update:
        try:
            opt = torch.optim.Adam(ac.parameters(), lr=lr, fused=True)
            _log("FAST_UPDATE: fused Adam ON")
        except (RuntimeError, TypeError) as e:
            opt = torch.optim.Adam(ac.parameters(), lr=lr)
            _log(f"FAST_UPDATE: fused Adam unavailable ({e}); plain Adam")
        _log(f"FAST_UPDATE: minibatch={minibatch or 'full'} bf16={use_bf16}")
    else:
        opt = torch.optim.Adam(ac.parameters(), lr=lr)
    import math as _math
    _LOG_SQRT_2PI = _math.log(_math.sqrt(2.0 * _math.pi))  # == Normal.log_prob's const

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
    import contextlib
    prof = _mk_prof(env, torch) if _i("QUAD_PROFILE", 0) else None
    sec = (prof.sec if prof is not None
           else (lambda name: contextlib.nullcontext()))
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    _win_t, _win_steps = t0, 0
    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    for it in range(1, iters + 1):
        for k in range(rollout):
            with sec("policy"):
                with torch.no_grad():
                    mu, v, log_std = ac(obs)
                    d = torch.distributions.Normal(mu, log_std.exp())
                    a = d.sample()
                    logp = d.log_prob(a).sum(-1)
            with sec("buf"):
                obs_buf[k] = obs
                act_buf[k] = a
                logp_buf[k] = logp
                val_buf[k] = v
            with sec("env_step"):
                obs, r, done, _ = env.step(a)
            with sec("buf"):
                rew_buf[k] = r
                done_buf[k] = done.float()
            total_steps += N

        with sec("gae"):
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

        with sec("update"):
            obs_flat = obs_buf.reshape(-1, OBS_IN)
            act_flat = act_buf.reshape(-1, NJ)
            logp_flat = logp_buf.reshape(-1)
            adv_flat = adv.reshape(-1)
            ret_flat = ret.reshape(-1)
            B = obs_flat.shape[0]

            def _ppo_losses(o, a_t, lp_old, ad, rt):
                mu, v, log_std = ac(o)
                if fast_update:
                    # exact Normal.log_prob / .entropy math, fewer launches
                    inv_std2 = torch.exp(-2.0 * log_std)
                    new_logp = (-0.5 * ((a_t - mu) ** 2) * inv_std2
                                - log_std - _LOG_SQRT_2PI).sum(-1)
                    ent = (0.5 + _LOG_SQRT_2PI + log_std).sum()
                else:
                    d = torch.distributions.Normal(mu, log_std.exp())
                    new_logp = d.log_prob(a_t).sum(-1)
                    ent = d.entropy().sum(-1).mean()
                ratio = (new_logp - lp_old).exp()
                surr1 = ratio * ad
                surr2 = torch.clamp(ratio, 0.8, 1.2) * ad
                pi_loss = -torch.min(surr1, surr2).mean()
                v_loss = ((v - rt) ** 2).mean()
                return pi_loss + 0.5 * v_loss - ent_coef * ent

            for _epoch in range(4):
                if minibatch > 0:
                    perm = torch.randperm(B, device=tdev)
                    mb_slices = [perm[i:i + minibatch]
                                 for i in range(0, B, minibatch)]
                else:
                    mb_slices = [None]
                for sl in mb_slices:
                    if sl is None:
                        o, a_t, lp, ad, rt = (obs_flat, act_flat, logp_flat,
                                              adv_flat, ret_flat)
                    else:
                        o, a_t, lp, ad, rt = (obs_flat[sl], act_flat[sl],
                                              logp_flat[sl], adv_flat[sl],
                                              ret_flat[sl])
                    if use_bf16:
                        with torch.autocast("cuda", dtype=torch.bfloat16):
                            loss = _ppo_losses(o, a_t, lp, ad, rt)
                    else:
                        loss = _ppo_losses(o, a_t, lp, ad, rt)
                    opt.zero_grad(set_to_none=True)
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
                    opt.step()

        if it % 5 == 0 or it == 1:
            now = time.time()
            fps = total_steps / max(now - t0, 1e-6)
            win_fps = (total_steps - _win_steps) / max(now - _win_t, 1e-6)
            _win_t, _win_steps = now, total_steps
            # SATHINGE readout (SHADOWING mode only): sat_frac = fraction of
            # (step,joint) samples with |kp*(cmd-q)|/tau_lim above the hinge
            # threshold since the last log; logged even at W_SATHINGE=0 so the
            # campaign can measure foldability progress. Window counters reset.
            sat_note = ""
            if getattr(env, "_sat_samples_t", None) is not None:
                _ns = float(env._sat_samples_t.item())
                if _ns > 0:
                    _sf = float(env._sat_above_t.item()) / _ns
                    _sp = (float(env._satpen_sum_t.item())
                           / max(float(env._satpen_n_t.item()), 1.0))
                    sat_note = (f" sat_frac={_sf:.4f} sat_pen={_sp:.5f} "
                                f"(>{env._sat_hinge_frac:g}*tau_lim, "
                                f"W_SATHINGE={env._wsat:g})")
                    env._sat_above_t.zero_()
                    env._sat_samples_t.zero_()
                    env._satpen_sum_t.zero_()
                    env._satpen_n_t.zero_()
            # FOOTSLIP readout (SHADOWING mode only): mean stance-foot-slip SPEED over the
            # contact-samples since the last log -- the closure-precursor number, logged even
            # at W_FOOTSLIP=0 so the campaign can track fold-closure progress without a fold.
            fs_note = ""
            if getattr(env, "_fs_contact_n_t", None) is not None:
                _fn = float(env._fs_contact_n_t.item())
                if _fn > 0:
                    _fspeed = float(env._fs_speed_sum_t.item()) / _fn
                    _fpen = (float(env._fs_pen_sum_t.item())
                             / max(float(env._fs_pen_n_t.item()), 1.0))
                    fs_note = (f" footslip={_fspeed * 1000:.1f}mm/s pen={_fpen:.5f} "
                               f"(stance z<{env._footslip_vz:g}m, W_FOOTSLIP={env._wfootslip:g})")
                    env._fs_speed_sum_t.zero_()
                    env._fs_contact_n_t.zero_()
                    env._fs_pen_sum_t.zero_()
                    env._fs_pen_n_t.zero_()
            _log(f"WALK-QUAD it={it:4d} ep_rew/step~{rew_buf.mean().item():+.3f} "
                 f"meanV {val_buf.mean().item():+.2f} steps {total_steps:,} "
                 f"{fps:,.0f} env-steps/s (window {win_fps:,.0f}){sat_note}{fs_note}")
            _status_write("TRAINING", it=it, iters=iters,
                          rew=float(rew_buf.mean().item()))
            if prof is not None:
                prof.dump(f"it={it}")

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
    import numpy as np
    import torch
    DT = _base_mod().DT
    steps = steps or _i("QUAD_EVAL_STEPS", 1500)
    _saved_max_ep = env.max_ep
    env.max_ep = steps + 10
    ac.eval()
    if getattr(env, "_sat_samples_t", None) is not None:   # sat window = THIS eval only
        env._sat_above_t.zero_()
        env._sat_samples_t.zero_()
        env._satpen_sum_t.zero_()
        env._satpen_n_t.zero_()
    if getattr(env, "_fs_contact_n_t", None) is not None:  # footslip window = THIS eval only
        env._fs_speed_sum_t.zero_()
        env._fs_contact_n_t.zero_()
        env._fs_pen_sum_t.zero_()
        env._fs_pen_n_t.zero_()
    obs = env.reset()
    first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
    dist = torch.zeros(env.n, device=tdev)
    vx_sum = torch.zeros(env.n, device=tdev)
    alive_steps = torch.zeros(env.n, device=tdev)
    ydrift = torch.zeros(env.n, device=tdev)
    gm_sum = torch.zeros(env.n, device=tdev)   # ghost-similarity (SHADOWING mode)
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
        ydrift = torch.where(step_alive > 0, env.qpos_t[:, 1].abs(), ydrift)
        if hasattr(env, "last_gmatch_t"):
            gm_sum += env.last_gmatch_t * step_alive
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
    if hasattr(env, "last_gmatch_t"):
        gm = (gm_sum / torch.clamp(alive_steps, min=1.0)).cpu().numpy()
        _log(f"IN-ENGINE EVAL gmatch (ghost similarity, sig={env._gsig}): "
             f"mean={gm.mean():.3f} p5={np.percentile(gm, 5):.3f} | "
             f"|ydrift| final mean={ydrift.cpu().numpy().mean():.2f} m")
    # FOLDABILITY READOUT: the servo error the actuators had to close == tau/kp. This is the
    # quantity that decides whether this champion's achieved gait can become a ghost at all
    # (docs/developer/shadow-iteration.md). Report it whether or not it is being PENALISED, so a
    # W_TAU=0 baseline and a W_TAU>0 run are directly comparable on the thing that matters.
    if getattr(env, "last_serr_t", None) is not None:
        _se = env.last_serr_t.detach().cpu().numpy()
        # sat_frac: fraction of (step,joint) samples over the eval window whose torque
        # proxy exceeded the hinge threshold -- THE foldability-progress number
        # (peak saturation, not mean effort), reported whether or not it is penalised.
        _sat_note = ""
        if (getattr(env, "_sat_samples_t", None) is not None
                and float(env._sat_samples_t.item()) > 0):
            _sf = float(env._sat_above_t.item()) / float(env._sat_samples_t.item())
            _sat_note = (f" | sat_frac={_sf:.4f} of (step,joint) samples with "
                         f"|kp*(cmd-q)|/tau_lim > {env._sat_hinge_frac:g} "
                         f"(W_SATHINGE={env._wsat:g})")
        # stance-foot-slip: the mean horizontal speed of a planted foot over the eval window
        # -- THE closure precursor (a stance foot that slides is exactly what fails the < 20mm
        # fold-closure gate). Reported whether or not it is penalised.
        _fs_note = ""
        if (getattr(env, "_fs_contact_n_t", None) is not None
                and float(env._fs_contact_n_t.item()) > 0):
            _fspeed = float(env._fs_speed_sum_t.item()) / float(env._fs_contact_n_t.item())
            _fs_note = (f" | stance-foot-slip mean={_fspeed * 1000:.2f} mm/s "
                        f"(z<{env._footslip_vz:g}m stance; W_FOOTSLIP={env._wfootslip:g})")
        _log(f"IN-ENGINE EVAL foldability: servo |cmd-q| rms={float(np.sqrt(_se.mean())):.4f} rad "
             f"(== tau/kp; W_TAU={env._wtau}){_sat_note}{_fs_note}")


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
