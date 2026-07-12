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

"""IN-ENGINE residual RL for the G1 deterministic WALK -- train WHERE you deploy.

Walk sibling of g1_residual_inengine.py (the stand version). The deterministic stand was
already complete, so a residual on it was a net-negative saboteur. The deterministic WALK is
INCOMPLETE: the open-loop gait propels forward but cannot do single-support LATERAL balance,
so it falls in a few seconds. That missing lateral balance is exactly what a residual has to
learn -- the case where residual RL pays off.

This OMNISIM_INENGINE_PYMOD hook:
  * commands the deterministic walk = g1_human_gait (advancing clock) + the fore/aft lean,
    ABSOLUTE on the 12 leg servo targets (it OWNS the legs; the controller handles arms),
  * adds a small learned residual ON TOP (the lateral-balance corrector),
  * (train) runs episodic OpenAI-ES ENTIRELY in-process: reset -> blend into the gait -> walk
    -> fitness = forward progress + uprightness - lateral drift - effort; evolve the policy.

Train == deploy (same hook, same engine, model live from Newton) -> a policy that walks here
walks in deploy, no sim-to-deploy gap. Reuses the stand module's leg map + live obs + the
solved in-hook reset (state-write + eval_fk, no solver rebuild, no cold-load relaunch).

Modes (RES_MODE): probe | train | deploy.
Knobs: RES_POLICY(npz) RES_MODE RES_ACT_SCALE(0.20) RES_POP(16) RES_SIGMA(0.05) RES_LR(0.02)
  RES_SEED RES_LOG  WALK_VX(0.12) WALK_FREQ(0.80) WALK_STEP_WIDTH(0.18) WALK_BLEND_S(1.0)
  WALK_WARM_TICKS(360) WALK_EP_S(5.0)  W_VX(6) W_ALIVE(0.6) W_LAT(8) W_UP(4) W_YAW(3)
  RES_ACT_PEN(0.01) FALL_PITCH(0.7) FALL_ROLL(0.6) FALL_BZ(0.5).
"""
import importlib
import math
import os
import sys

import numpy as np

# ensure the repo root is importable inside the embedded engine interpreter
_RT = os.environ.get("OMNISIM_HOME", ".")
if _RT not in sys.path:
    sys.path.insert(0, _RT)

# reuse the engine-API helpers from the stand module (leg map, live obs, solved reset, lean)
from projects.policies.training import g1_residual_inengine as R

_GAIT = "projects.policies.control.gait.g1_human_gait"

OBS_DIM = 11      # proj_g(3) + ang_vel(3) + lin_vel(3) + gait_phase(sin,cos)(2)
N_LEG = 12        # 6 left + 6 right (leg-map order, == gait slots 0..11)
N_PARAM = N_LEG * OBS_DIM + N_LEG


def _policy_apply(theta, obs):
    W = theta[: N_LEG * OBS_DIM].reshape(N_LEG, OBS_DIM)
    b = theta[N_LEG * OBS_DIM:]
    return np.tanh(W @ obs + b)


def _load_theta(path):
    if path and os.path.exists(path):
        try:
            return np.load(path)["theta"].astype(np.float64)
        except Exception:
            pass
    return np.zeros(N_PARAM, dtype=np.float64)


def _save_theta(path, theta, meta=None):
    if not path:
        return
    try:
        d = {"theta": theta.astype(np.float32)}
        if meta is not None:
            d["meta"] = np.array(meta, dtype=np.float32)
        np.savez(path, **d)
    except Exception:
        pass


def _wbuild(world):
    """Build the stand-module leg map/obs/reset, then add the gait base."""
    if getattr(world, "_wres_ready", None) is not None:
        return world._wres_ready
    world._wres_ready = False
    if not R._build(world):                 # sets _res_leg_nd, _res_base_idx, _res_dt, lean ...
        R._log(world, "walk: base _build failed")
        return False
    stg = importlib.import_module(_GAIT)
    try:
        gp = stg.GaitParams(vx=R._f("WALK_VX", 0.12), freq=R._f("WALK_FREQ", 0.80),
                            style="ik", lateral="lipm", ramp_s=2.0)
    except TypeError:
        gp = stg.GaitParams(vx=R._f("WALK_VX", 0.12), freq=R._f("WALK_FREQ", 0.80))
    sw = R._f("WALK_STEP_WIDTH", 0.18)
    if sw > 0 and hasattr(gp, "step_width"):
        gp.step_width = sw
    world._wres_stg = stg
    world._wres_gp = gp
    world._wres_omega = 2.0 * math.pi * gp.freq
    world._wres_phase0 = float(stg.DS_PHASE)
    world._wres_phase = float(stg.DS_PHASE)
    world._wres_dt = world._res_dt
    world._wres_blend_s = R._f("WALK_BLEND_S", 1.0)
    world._wres_ready = True
    R._log(world, "walk ready: leg_nd=%s dt=%.4f vx=%.2f freq=%.2f sw=%.2f mode=%s"
           % (world._res_leg_nd, world._wres_dt, gp.vx, gp.freq,
              getattr(gp, "step_width", 0.0), os.environ.get("RES_MODE", "deploy")))
    return True


def _gait_legs(world, phase, tss):
    legs, _arms, _sw = world._wres_stg.targets_np(phase, world._wres_gp, t_since_start=tss)
    return np.asarray(legs, np.float64)[:12]   # leg-map order == gait slots 0..11


def _walk_obs(world):
    """9-dim base obs from the live engine + gait phase (sin,cos) -> 11-dim."""
    obs9, roll, pitch, bz, qpos, qvel = R._read_obs(world)
    ph = world._wres_phase
    obs = np.concatenate([obs9, [math.sin(ph), math.cos(ph)]])
    return obs, roll, pitch, bz, float(qpos[0]), float(qpos[1]), qvel


def _apply_walk(world, gait_legs, lean_map, res, blend):
    """SET the 12 leg servo targets = gait + lean + residual (absolute; owns the legs).
    blend in [0,1] eases from the current (stand) target into the gait."""
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for i, nd in enumerate(world._res_leg_nd):
        if 0 <= nd < len(tp):
            tgt = float(gait_legs[i]) + float(lean_map.get(nd, 0.0)) + float(res[i])
            tp[nd] = (1.0 - blend) * float(tp[nd]) + blend * tgt
    world.control.joint_target_pos.assign(tp)
    world._mjc_dirty = True


# ───────────────────────── walk ES ─────────────────────────
class _WES:
    def __init__(self, world):
        self.path = os.environ.get("RES_POLICY", "")
        self.pop = R._i("RES_POP", 16)
        self.sigma = R._f("RES_SIGMA", 0.05)
        self.lr = R._f("RES_LR", 0.02)
        self.rng = np.random.default_rng(R._i("RES_SEED", 0))
        self.theta = _load_theta(self.path)
        if self.theta.shape[0] != N_PARAM:
            self.theta = np.zeros(N_PARAM)
        self.act_scale = R._f("RES_ACT_SCALE", 0.20)
        self.act_pen = R._f("RES_ACT_PEN", 0.01)
        dt = world._wres_dt
        self.eplen = max(4, int(R._f("WALK_EP_S", 5.0) / dt))
        self.blend = max(1, int(world._wres_blend_s / dt))
        self.fp = R._f("FALL_PITCH", 0.7); self.fr = R._f("FALL_ROLL", 0.6); self.fbz = R._f("FALL_BZ", 0.5)
        self.wvx = R._f("W_VX", 6.0); self.walive = R._f("W_ALIVE", 0.6)
        self.wlat = R._f("W_LAT", 8.0); self.wup = R._f("W_UP", 4.0); self.wyaw = R._f("W_YAW", 3.0)
        self.vxt = world._wres_gp.vx
        self.gen = 0; self.idx = 0; self.ep_tick = 0; self.fit = 0.0
        self.best_fit = -1e18; self.x0 = 0.0
        self.fits = np.zeros(2 * self.pop)
        self._new_generation()

    def _new_generation(self):
        self.eps = self.rng.standard_normal((self.pop, N_PARAM))
        self.members = []
        for i in range(self.pop):
            self.members.append(self.theta + self.sigma * self.eps[i])
            self.members.append(self.theta - self.sigma * self.eps[i])

    def member_theta(self):
        return self.members[self.idx]

    def _update(self, world):
        F = self.fits.copy()
        order = np.argsort(F); ranks = np.empty_like(order, dtype=np.float64)
        ranks[order] = np.arange(len(F)); u = ranks / (len(F) - 1) - 0.5
        g = np.zeros(N_PARAM)
        for i in range(self.pop):
            g += (u[2 * i] - u[2 * i + 1]) * self.eps[i]
        g /= (2 * self.pop * self.sigma)
        self.theta = self.theta + self.lr * g
        meanF = float(F.mean()); maxF = float(F.max())
        self.best_fit = max(self.best_fit, maxF)
        _save_theta(self.path, self.theta, meta=[self.gen, meanF, maxF])
        R._log(world, "GEN %d meanF=%.2f maxF=%.2f bestF=%.2f |theta|=%.3f"
               % (self.gen, meanF, maxF, self.best_fit, float(np.linalg.norm(self.theta))))

    def start_episode(self, world, x0):
        self.ep_tick = 0; self.fit = 0.0; self.x0 = x0
        world._wres_phase = world._wres_phase0

    def record_and_advance(self, world, dx):
        self.fit += self.wvx * 0.0 + dx * 0.0   # (dx already folded per-tick)
        self.fits[self.idx] = self.fit
        if os.environ.get("RES_DEBUG", "0") == "1":
            R._log(world, "  ep idx=%d gen=%d fit=%.1f ticks=%d dx=%.2f"
                   % (self.idx, self.gen, self.fit, self.ep_tick, dx))
        self.idx += 1
        if self.idx >= 2 * self.pop:
            self._update(world); self.idx = 0; self.gen += 1; self._new_generation()
        if getattr(world, "_res_spawn_jq", None) is not None:
            R._try_reset(world, "state", world._res_spawn_jq)


def _walk_train_step(world, es):
    # capture the standing spawn pose once (for the in-hook reset)
    obs, roll, pitch, bz, bx, by, qvel = _walk_obs(world)
    if world._res_spawn_jq is None and world._wres_ctr > 60 and bz > 0.6:
        world._res_spawn_jq = world.model.joint_q.numpy().copy()
        world._res_spawn_z = float(bz)
        es.start_episode(world, bx)
        R._log(world, "captured spawn jq + z=%.3f; first episode start x=%.2f" % (bz, bx))
        return
    if world._res_spawn_jq is None:
        return
    es.ep_tick += 1; t = es.ep_tick
    tss = max(0.0, (t - es.blend) * world._wres_dt)
    blend = min(1.0, t / max(1, es.blend))
    # base = gait + lean; freeze the gait clock during the blend-in (ease into the gait squat)
    gait_legs = _gait_legs(world, world._wres_phase, tss)
    lean_map = R._base_lean(world, roll, pitch, qvel[3:6], qvel[0:3])
    res = es.act_scale * _policy_apply(es.member_theta(), obs)
    _apply_walk(world, gait_legs, lean_map, res, blend)
    if blend >= 1.0:
        world._wres_phase += world._wres_omega * world._wres_dt
    # per-tick reward: forward speed + alive - lateral - tilt - yaw - effort
    w_, x_, y_, z_ = obs[0], obs[1], obs[2], 0.0   # (obs[:3] = proj_g, not quat) -> use roll/pitch
    vx = float(qvel[0]); vy = float(qvel[1])
    yaw = math.atan2(2 * 0, 1)  # yaw from quat below
    # recompute yaw from live quat for the heading penalty
    live = world.solver.mjw_data; q = live.qpos.numpy().reshape(-1)
    qw, qx, qy, qz = float(q[3]), float(q[4]), float(q[5]), float(q[6])
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    upr = max(0.0, 1.0 - roll * roll - pitch * pitch)
    if blend >= 1.0:
        es.fit += (es.wvx * max(-0.1, min(es.vxt + 0.1, vx))
                   + es.walive
                   + es.wup * (upr - 1.0)
                   - es.wlat * vy * vy
                   - es.wyaw * yaw * yaw
                   - es.act_pen * float(np.sum(res * res)))
    fell = (abs(roll) > es.fr) or (abs(pitch) > es.fp) or (bz < es.fbz)
    if fell:
        es.fit -= 30.0
        es.record_and_advance(world, bx - es.x0)
        es.start_episode(world, _read_spawn_x(world))
        return
    if t >= es.eplen:
        es.record_and_advance(world, bx - es.x0)
        es.start_episode(world, _read_spawn_x(world))
        return


def _read_spawn_x(world):
    try:
        return float(world.solver.mjw_data.qpos.numpy().reshape(-1)[0])
    except Exception:
        return 0.0


def _walk_deploy_step(world, theta):
    obs, roll, pitch, bz, bx, by, qvel = _walk_obs(world)
    if not hasattr(world, "_wres_dep_t"):
        world._wres_dep_t = 0
    world._wres_dep_t += 1; t = world._wres_dep_t
    blend = min(1.0, t / max(1, int(world._wres_blend_s / world._wres_dt)))
    tss = max(0.0, t * world._wres_dt)
    gait_legs = _gait_legs(world, world._wres_phase, tss)
    lean_map = R._base_lean(world, roll, pitch, qvel[3:6], qvel[0:3])
    res = R._f("RES_ACT_SCALE", 0.20) * _policy_apply(theta, obs)
    _apply_walk(world, gait_legs, lean_map, res, blend)
    if blend >= 1.0:
        world._wres_phase += world._wres_omega * world._wres_dt
    if t % 125 == 1:
        R._log(world, "deploy t=%d x=%.2f y=%.2f z=%.3f roll=%.3f pit=%.3f |res|=%.2f"
               % (t, bx, by, bz, roll, pitch, float(np.abs(res).max())))


def g1_walk_res_step(world):
    if not _wbuild(world):
        return
    if not hasattr(world, "_wres_ctr"):
        world._wres_ctr = 0
        world._wres_es = None
        world._wres_theta = None
    world._wres_ctr += 1
    warm = R._i("WALK_WARM_TICKS", 360)
    if world._wres_ctr < warm:
        return                                  # let the stand settle first
    mode = os.environ.get("RES_MODE", "deploy")
    try:
        if mode == "train":
            if world._wres_es is None:
                world._wres_es = _WES(world)
                R._log(world, "TRAIN start pop=%d sigma=%.3f lr=%.3f eplen=%d N_PARAM=%d"
                       % (world._wres_es.pop, world._wres_es.sigma, world._wres_es.lr,
                          world._wres_es.eplen, N_PARAM))
            _walk_train_step(world, world._wres_es)
        elif mode == "deploy":
            if world._wres_theta is None:
                world._wres_theta = _load_theta(os.environ.get("RES_POLICY", ""))
                R._log(world, "DEPLOY policy |theta|=%.3f" % float(np.linalg.norm(world._wres_theta)))
            _walk_deploy_step(world, world._wres_theta)
        else:  # probe
            obs, roll, pitch, bz, bx, by, qvel = _walk_obs(world)
            if world._wres_ctr % 125 == 1:
                R._log(world, "probe x=%.2f z=%.3f roll=%.3f pit=%.3f" % (bx, bz, roll, pitch))
    except Exception as e:
        import traceback
        R._log(world, "step err %r\n%s" % (e, traceback.format_exc()))
