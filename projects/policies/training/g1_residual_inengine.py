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

"""IN-ENGINE residual RL for the G1 deterministic stand -- train WHERE you deploy.

The whole point: the obs gap and the trainer<->deploy PHYSICS gap both vanish when the
policy is trained inside the ACTUAL OmniSim Newton/mujoco_warp engine, on the ACTUAL
deterministic-stand base. This module is a single OMNISIM_INENGINE_PYMOD hook that:

  * reads the observation from the LIVE engine state (mjw_data) -- the exact same state the
    deployed policy reads, because deploy uses this SAME hook (zero obs gap),
  * runs a small residual policy and ADDS it to the position-servo targets ON TOP of the
    deterministic stand the controller commands (nominal + ank_bias + lean), so the residual
    is a bounded correction the genuinely-stable base carries (not the load-bearing
    stabiliser -- the failure mode of the walk-pipeline residual), and
  * (train mode) runs an episodic OpenAI-ES loop ENTIRELY in-process: it applies scripted
    pushes (qvel impulses, the deploy's own physics), scores survival+uprightness, soft-resets
    the world (qpos/qvel teleport, no relaunch -> no cold-load flake), and evolves the policy.

Because train == deploy (same hook, same engine, same model extracted live from Newton), a
policy that survives a push here survives it in deploy -- no sim-to-deploy gap to cross.

Modes (RES_MODE):
  probe   -- policy=0; just read obs + log base z/tilt. Verify the hook is inert (base
             unchanged) and the obs is sane.
  deploy  -- load policy from RES_POLICY (npz) and apply it. The deploy path.
  train   -- episodic ES; writes the evolving policy to RES_POLICY each generation.

Env knobs: RES_MODE, RES_POLICY (npz path), RES_ACT_SCALE (0.25), RES_LOG,
  RES_POP (24), RES_SIGMA (0.05), RES_LR (0.02), RES_SEED (0),
  RES_SETTLE_S (0.6), RES_EP_S (3.0), RES_PUSH_VMAX (1.4), RES_PUSH_T (0.8),
  RES_ACT_PEN (0.02), RES_FALL_PITCH (0.8), RES_FALL_ROLL (0.8), RES_FALL_BZ (0.45).
"""
import math
import os

import numpy as np

# ── obs layout: proj_g(3) + ang_vel_body(3) + lin_vel_world(3) = 9 (base-state feedback).
#    A compact push-recovery feedback obs; the deterministic base holds posture, the residual
#    reacts to tilt + the push velocity. Linear policy 9 -> 12 leg joints (tanh-bounded).
OBS_DIM = 9
N_LEG = 12        # 6 left + 6 right (waist excluded -- not a balance lever)


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
        world._mpc_log("res: " + msg)
    except Exception:
        import sys
        sys.stderr.write("[g1_res] " + msg + "\n")
    lp = os.environ.get("RES_LOG")
    if lp:
        try:
            with open(lp, "a", buffering=1) as fh:
                fh.write(msg + "\n")
        except Exception:
            pass


# ───────────────────────── policy (linear, tanh-bounded) ─────────────────────────
def _policy_apply(theta, obs):
    """theta packs W(N_LEG x OBS_DIM) then b(N_LEG); returns tanh(W obs + b) in [-1,1]."""
    W = theta[: N_LEG * OBS_DIM].reshape(N_LEG, OBS_DIM)
    b = theta[N_LEG * OBS_DIM:]
    return np.tanh(W @ obs + b)


N_PARAM = N_LEG * OBS_DIM + N_LEG


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


# ───────────────────────── build (leg map + obs indices) ─────────────────────────
def _build(world):
    if getattr(world, "_res_ready", None) is not None:
        return world._res_ready
    world._res_ready = False
    try:
        import mujoco as mj
    except Exception as e:
        _log(world, "mujoco import failed %r" % (e,)); return False
    sol = world.solver
    mjm = getattr(sol, "mj_model", None)
    m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or m2nd is None:
        _log(world, "solver lacks mj_model/jntmap"); return False
    m2nd = m2nd.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    world._res_mj = mj
    world._res_nq = int(mjm.nq); world._res_nv = int(mjm.nv)
    # geometric leg map (breadth-first newton layout) -- same approach as the centroidal driver.
    dcl = mj.MjData(mjm); mj.mj_forward(mjm, dcl)
    pelvis_z = 0.7
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            pelvis_z = float(dcl.xpos[int(mjm.jnt_bodyid[j])][2]); break
    cand = []
    for j in range(int(mjm.njnt)):
        nd = int(m2nd[j])
        if not (6 <= nd <= 28):
            continue
        bid = int(mjm.jnt_bodyid[j]); pos = np.array(dcl.xpos[bid], float)
        if pos[2] >= pelvis_z - 0.02:
            continue
        ax = np.abs(np.array(mjm.jnt_axis[j], float))
        cand.append(dict(nd=nd, pos=pos, axis=int(np.argmax(ax)),
                         qadr=int(mjm.jnt_qposadr[j]), dofadr=int(mjm.jnt_dofadr[j])))
    order6 = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
    legs = {}
    for side, ys in (("left", 1.0), ("right", -1.0)):
        sj = [c for c in cand if c["pos"][1] * ys > 0]
        pit = sorted([c for c in sj if c["axis"] == 1], key=lambda c: -c["pos"][2])
        rol = sorted([c for c in sj if c["axis"] == 0], key=lambda c: -c["pos"][2])
        yaw = [c for c in sj if c["axis"] == 2]
        if len(pit) >= 3 and len(rol) >= 2 and len(yaw) >= 1:
            ch = {"hip_pitch": pit[0], "hip_roll": rol[0], "hip_yaw": yaw[0],
                  "knee": pit[1], "ankle_pitch": pit[2], "ankle_roll": rol[-1]}
            legs[side] = [ch[k] for k in order6]
    if len(legs) != 2:
        _log(world, "leg map incomplete %s" % list(legs)); return False
    seq = legs["left"] + legs["right"]      # 12 joints, fixed order
    world._res_leg_nd = [c["nd"] for c in seq]        # newton dof (for joint_target_pos)
    world._res_leg_qadr = [c["qadr"] for c in seq]    # mjc qpos adr (for q)
    world._res_leg_dofadr = [c["dofadr"] for c in seq]  # mjc dof adr (for qd)
    # nominal leg pose (for q - nom); SPEC order matches the L6+R6 leg order.
    try:
        import sys
        rt = os.environ.get("OMNISIM_HOME", ".")
        if rt not in sys.path:
            sys.path.insert(0, rt)
        from projects.policies.research.backends import g1_physics_spec as SPEC
        world._res_nom = np.array(SPEC.NOMINAL_LEGS[:12], float)
    except Exception:
        world._res_nom = np.zeros(12)
    # pelvis Newton body index for set_body_vel/reset_body_pose. The URDF importer does
    # NOT put the pelvis at index 0 (feet come first, ~z=0.025); auto-detect the free base
    # as the first body near standing height. Override with RES_BASE_IDX.
    base_idx = _i("RES_BASE_IDX", -1)
    if base_idx < 0:
        base_idx = 5
        try:
            for i in range(0, 25):
                z = float(world.body_xform(i)[2])
                if 0.68 <= z <= 0.84:
                    base_idx = i; break
        except Exception:
            pass
    world._res_base_idx = base_idx
    world._res_spawn_z = None      # captured standing base height (for the reset pose)
    world._res_spawn_jq = None     # captured standing model.joint_q (for the in-hook reset)
    # ── In-hook STATELESS deterministic lean (the base balance) ──
    # So the base balance lives in the SAME hook as the residual + the reset -> a reset is
    # fully self-consistent (no separate stateful controller lean to desync). Run the
    # humanoid_stand_deploy controller with HSTAND_LEAN=0 (pure pose hold + ank_bias); this
    # hook adds the lean + residual. Gains = the strong lean + roll-PD that stood the trainer.
    nd = world._res_leg_nd     # [L: hp,hr,hy,kn,ap,ar][R: hp,hr,hy,kn,ap,ar]
    world._res_lean_dof = dict(ankp=[nd[4], nd[10]], hipp=[nd[0], nd[6]])
    # Faithful port of the humanoid_stand_deploy LEAN (g1.json gains): pitch-only reactive
    # ankle/hip lean with the slow pitch low-pass + the fast vx term + soft deadband. STATE
    # (the low-pass accumulators) lives on the world so a reset can CLEAR it -> reset-safe.
    world._res_lean_g = dict(
        on=_i("RES_BASE_LEAN", 1),
        kv=_f("RES_LEAN_KV", 0.14), kp=_f("RES_LEAN_KP", 1.6), kd=_f("RES_LEAN_KD", 0.25),
        hip=_f("RES_LEAN_HIP", 0.35), clamp=_f("RES_LEAN_CLAMP", 0.30),
        db=_f("RES_LEAN_DB", 0.008), smooth=_f("RES_LEAN_SMOOTH", 1.0))
    world._res_pitch_lp = 0.0
    world._res_vx_lp = 0.0
    world._res_pr_lp = 0.0
    # control timestep
    world._res_dt = float(getattr(world, "_n_substeps", 4)) * float(mjm.opt.timestep)
    world._res_ready = True
    _log(world, "ready nq=%d nv=%d leg_nd=%s dt=%.4f mode=%s"
         % (world._res_nq, world._res_nv, world._res_leg_nd,
            world._res_dt, os.environ.get("RES_MODE", "deploy")))
    if os.environ.get("RES_DIAG", "0") == "1":
        def _attrs(o):
            try:
                return [a for a in dir(o) if not a.startswith("__")]
            except Exception:
                return []
        _log(world, "DIAG world: %s" % [a for a in _attrs(world)
                                        if any(k in a.lower() for k in
                                        ("state", "reset", "fk", "joint", "model", "newton", "seed", "body", "control"))])
        _log(world, "DIAG model: %s" % [a for a in _attrs(getattr(world, "model", None))
                                        if any(k in a.lower() for k in
                                        ("joint_q", "joint_qd", "body_q", "joint_", "reset"))])
        for sn in ("state_a", "state_b"):
            st = getattr(world, sn, None)
            _log(world, "DIAG world.%s? %r attrs=%s" % (sn, st is not None,
                 [a for a in _attrs(st) if any(k in a.lower() for k in ("body_q", "joint", "clear"))]))
        sol = getattr(world, "solver", None)
        _log(world, "DIAG solver type=%s methods=%s" % (
            type(sol).__name__,
            [a for a in _attrs(sol) if any(k in a.lower() for k in
             ("notify", "update", "reset", "seed", "convert", "data", "model", "kinematic", "forward", "step"))]))
        import sys as _s
        if "newton" in _s.modules:
            nt = _s.modules["newton"]
            _log(world, "DIAG newton.eval_fk? %r" % hasattr(nt, "eval_fk"))
    return True


def _read_obs(world):
    """gap-free obs from the LIVE engine state (mjw_data). Returns (obs[9], roll, pitch, bz,
    qpos_copy, qvel_copy)."""
    live = world.solver.mjw_data
    qpos = live.qpos.numpy().reshape(-1)[: world._res_nq].copy()
    qvel = live.qvel.numpy().reshape(-1)[: world._res_nv].copy()
    w, x, y, z = float(qpos[3]), float(qpos[4]), float(qpos[5]), float(qpos[6])
    # projected gravity (body frame) = R^T (0,0,-1); same closed form as g1_env_core.
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    ang = qvel[3:6]                  # body-frame angular velocity (mujoco free-joint qvel)
    lin = qvel[0:3]                  # world-frame linear velocity
    obs = np.array([gx, gy, gz, ang[0], ang[1], ang[2], lin[0], lin[1], lin[2]], float)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    bz = float(qpos[2])
    return obs, roll, pitch, bz, qpos, qvel


def _base_lean(world, roll, pitch, ang, lin):
    """Faithful port of the humanoid_stand_deploy LEAN (the deterministic base balance).
    Pitch-only reactive ankle/hip lean: slow pitch low-pass baseline + fast vx term + soft
    deadband + clamp (g1.json gains). Roll is left to the binary's passive lateral stability
    (matching the deploy stand). The low-pass state lives on the world so a reset clears it.
    Returns {newton_dof: target_delta} to ADD on the nominal pose."""
    g = world._res_lean_g
    if not g["on"]:
        return {}
    vx = float(lin[0]); pr = float(ang[1])
    world._res_pitch_lp += 0.02 * (pitch - world._res_pitch_lp)
    world._res_vx_lp += g["smooth"] * (vx - world._res_vx_lp)
    world._res_pr_lp += g["smooth"] * (pr - world._res_pr_lp)
    fwd = (g["kv"] * world._res_vx_lp + g["kd"] * (-world._res_pr_lp)
           + g["kp"] * (-(pitch - world._res_pitch_lp)))
    if abs(fwd) <= g["db"]:
        fwd = 0.0
    else:
        fwd = math.copysign(abs(fwd) - g["db"], fwd)
    fwd = max(-g["clamp"], min(g["clamp"], fwd))
    d = {}
    for dof in world._res_lean_dof["ankp"]:
        d[dof] = d.get(dof, 0.0) - fwd            # negative ankle = lean back
    for dof in world._res_lean_dof["hipp"]:
        d[dof] = d.get(dof, 0.0) + fwd * g["hip"]
    return d


def _reset_lean_state(world):
    """Clear the lean low-pass accumulators (called on reset so the base lean has no stale
    wound-up state after a teleport -- the bug that toppled the stand post-reset)."""
    world._res_pitch_lp = 0.0
    world._res_vx_lp = 0.0
    world._res_pr_lp = 0.0


def _apply_targets(world, deltas):
    """ADD a {newton_dof: delta} map to the position-servo targets."""
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for dof, v in deltas.items():
        if 0 <= dof < len(tp):
            tp[dof] = float(tp[dof]) + float(v)
    world.control.joint_target_pos.assign(tp)
    world._mjc_dirty = True


def _apply_residual(world, res):
    """ADD the residual (12-vec, rad) to the position-servo targets (newton dof indexed)."""
    if world.control is None:
        world.control = world.model.control()
    tp = world.control.joint_target_pos.numpy()
    for i, nd in enumerate(world._res_leg_nd):
        if 0 <= nd < len(tp):
            tp[nd] = float(tp[nd]) + float(res[i])
    world.control.joint_target_pos.assign(tp)
    world._mjc_dirty = True


def _soft_reset(world):
    """Reset the robot to the standing spawn -- via the NEWTON state API (writing mjw_data
    is silently overwritten by the Newton->mujoco bridge each step). reset_joints_to_defaults
    restores model.joint_q to BOTH state buffers + eval_fk + REBUILDS the mujoco solver
    (mujoco caches qpos at solver construction and ignores a runtime joint_q assign -> without
    the rebuild every post-fall episode collapses). reset_body_pose re-centres + uprights the
    free base first. In-process, no relaunch -> no cold-load flake."""
    bi = world._res_base_idx
    z0 = getattr(world, "_res_spawn_z", None) or 0.78
    try:
        world.reset_body_pose(bi, 0.0, 0.0, float(z0), 0.0, 0.0, 0.0, 1.0)
    except Exception as e:
        _log(world, "reset_body_pose err %r" % (e,))
    try:
        world.reset_joints_to_defaults()
    except Exception as e:
        _log(world, "reset_joints err %r" % (e,))
    world._mjc_dirty = True


def _try_reset(world, mode, spawn_jq):
    """Reset the robot to the standing spawn via several candidate paths. The goal is a
    NON-DESYNCING reset: write the Newton state + push it into the solver's mjw_data
    (solver._update_mjc_data) WITHOUT rebuilding the solver (the rebuild desyncs the hook
    obs from the C++ step loop). spawn_jq = world.model.joint_q at a standing tick."""
    import sys
    nt = sys.modules.get("newton")
    m = world.model
    if mode in ("state", "update"):
        zqd = m.joint_qd.numpy() * 0.0
        m.joint_q.assign(spawn_jq)
        m.joint_qd.assign(zqd)
        for st in (world.state_a, world.state_b):
            if st is None:
                continue
            try:
                st.joint_q.assign(spawn_jq)
                st.joint_qd.assign(zqd)
            except Exception:
                pass
            if nt is not None:
                try:
                    nt.eval_fk(m, m.joint_q, m.joint_qd, st)
                except Exception as e:
                    _log(world, "eval_fk err %r" % (e,))
        if mode == "update":
            try:
                world.solver._update_mjc_data()      # push newton state -> solver mjw_data
            except Exception as e:
                _log(world, "_update_mjc_data err %r" % (e,))
    elif mode == "solverreset":
        try:
            world.solver.reset()
        except Exception as e:
            _log(world, "solver.reset err %r" % (e,))
    elif mode == "defaults":
        try:
            world.reset_body_pose(world._res_base_idx, 0.0, 0.0, 0.78, 0.0, 0.0, 0.0, 1.0)
            world.reset_joints_to_defaults()
        except Exception as e:
            _log(world, "defaults err %r" % (e,))
    _reset_lean_state(world)       # clear the lean low-pass so no stale wind-up post-reset
    world._mjc_dirty = True


def _force(world, fx, fy):
    """Apply a SUSTAINED world-frame horizontal force at the pelvis (add_body_force is
    re-applied each tick, cleared after). A horizontal force at the pelvis tips the
    inverted pendulum about the planted feet -- a genuine push the stiff servo must fight,
    unlike a velocity impulse which is arrested in one step."""
    try:
        world.add_body_force(world._res_base_idx, float(fx), float(fy), 0.0, 0.0, 0.0, 0.0)
    except Exception as e:
        _log(world, "force err %r" % (e,))
    world._mjc_dirty = True


def _push(world, vx, vy, wx=0.0, wy=0.0):
    """Apply a push to the base via the NEWTON body-velocity API (set_body_vel writes the
    free-joint qd; a raw mjw_data.qvel write is overwritten). Linear (vx,vy) shove +
    angular (wx,wy) TIP. angular=0 selects the linear half, =1 the angular half."""
    bi = world._res_base_idx
    try:
        world.set_body_vel(bi, float(vx), float(vy), 0.0, 0)        # linear half
        if wx or wy:
            world.set_body_vel(bi, float(wx), float(wy), 0.0, 1)    # angular half (tip)
    except Exception as e:
        _log(world, "push err %r" % (e,))
    world._mjc_dirty = True


# ───────────────────────── ES state machine ─────────────────────────
class _ES:
    def __init__(self, world):
        self.path = os.environ.get("RES_POLICY", "")
        self.pop = _i("RES_POP", 24)              # antithetic pairs => 2*pop evals/gen
        self.sigma = _f("RES_SIGMA", 0.05)
        self.lr = _f("RES_LR", 0.02)
        self.rng = np.random.default_rng(_i("RES_SEED", 0))
        self.theta = _load_theta(self.path)
        if self.theta.shape[0] != N_PARAM:
            self.theta = np.zeros(N_PARAM)
        self.gen = 0
        # build the antithetic perturbation set for gen 0
        self._new_generation()
        self.act_scale = _f("RES_ACT_SCALE", 0.25)
        # episode schedule (ticks)
        dt = world._res_dt
        self.settle = max(1, int(_f("RES_SETTLE_S", 0.6) / dt))
        self.eplen = max(2, int(_f("RES_EP_S", 3.0) / dt))
        self.push_t = max(1, int(_f("RES_PUSH_T", 0.8) / dt))
        self.push_period = max(1, int(_f("RES_PUSH_PERIOD", 0.0) / dt)) if _f("RES_PUSH_PERIOD", 0.0) > 0 else 0
        self.push_vmax = _f("RES_PUSH_VMAX", 0.0)        # optional linear vel shove (m/s)
        self.push_wmax = _f("RES_PUSH_WMAX", 0.0)        # optional angular tip (rad/s)
        self.push_fmax = _f("RES_PUSH_FMAX", 0.0)        # SUSTAINED force shove (N) -- the real challenge
        self.push_dur = max(1, int(_f("RES_PUSH_DUR", 0.15) / dt))   # force-window length (ticks)
        self.cur_force = (0.0, 0.0)
        self.force_left = 0
        self.n_push = 0
        self.act_pen = _f("RES_ACT_PEN", 0.02)
        self.fp = _f("RES_FALL_PITCH", 0.8); self.fr = _f("RES_FALL_ROLL", 0.8)
        self.fbz = _f("RES_FALL_BZ", 0.45)
        self.idx = 0            # which population member is being evaluated
        self.ep_tick = 0
        self.fit = 0.0
        self.pushed = False
        self.cur_push = (0.0, 0.0)
        self.best_fit = -1e18
        self.fits = np.zeros(2 * self.pop)
        self.fell_flag = False

    def _new_generation(self):
        # antithetic noise: eps and -eps
        self.eps = self.rng.standard_normal((self.pop, N_PARAM))
        self.members = []
        for i in range(self.pop):
            self.members.append(self.theta + self.sigma * self.eps[i])
            self.members.append(self.theta - self.sigma * self.eps[i])

    def member_theta(self):
        return self.members[self.idx]

    def episode_push(self):
        # a random-direction linear shove + a random-direction angular TIP. The tip
        # (set base angular velocity) is the real balance challenge -- pure translation
        # is absorbed by the stiff servo, but a tip drives pitch/roll like a cube hit.
        a1 = self.rng.uniform(0, 2 * math.pi)
        a2 = self.rng.uniform(0, 2 * math.pi)
        return (self.push_vmax * math.cos(a1), self.push_vmax * math.sin(a1),
                self.push_wmax * math.cos(a2), self.push_wmax * math.sin(a2))

    def record_and_advance(self, world):
        self.fits[self.idx] = self.fit
        if os.environ.get("RES_DEBUG", "0") == "1":
            try:
                _, _, _, _bz, _, _ = _read_obs(world)
            except Exception:
                _bz = -1.0
            _log(world, "  ep idx=%d gen=%d fit=%.1f ticks=%d force=(%.0f,%.0f) bz_now=%.3f"
                 % (self.idx, self.gen, self.fit, self.ep_tick,
                    self.cur_force[0], self.cur_force[1], _bz))
        self.idx += 1
        self.fit = 0.0
        self.ep_tick = 0
        self.pushed = False
        self.force_left = 0
        if self.idx >= 2 * self.pop:
            self._update(world)
            self.idx = 0
            self.gen += 1
            self._new_generation()
        # RESET the robot to a clean standing start for the next member (in-hook state reset
        # + lean-state clear -> no desync, no stale wind-up). This is what makes episodic
        # in-engine ES work.
        if getattr(world, "_res_spawn_jq", None) is not None:
            _try_reset(world, "state", world._res_spawn_jq)

    def _update(self, world):
        F = self.fits.copy()
        # rank-normalize to [-0.5, 0.5] (OpenAI-ES utility) for robustness to fitness scale
        order = np.argsort(F)
        ranks = np.empty_like(order, dtype=np.float64)
        ranks[order] = np.arange(len(F))
        u = ranks / (len(F) - 1) - 0.5
        # gradient estimate from antithetic pairs
        g = np.zeros(N_PARAM)
        for i in range(self.pop):
            g += (u[2 * i] - u[2 * i + 1]) * self.eps[i]
        g /= (2 * self.pop * self.sigma)
        self.theta = self.theta + self.lr * g
        meanF = float(F.mean()); maxF = float(F.max())
        if maxF > self.best_fit:
            self.best_fit = maxF
        _save_theta(self.path, self.theta, meta=[self.gen, meanF, maxF])
        _log(world, "GEN %d meanF=%.2f maxF=%.2f bestF=%.2f |theta|=%.3f"
             % (self.gen, meanF, maxF, self.best_fit, float(np.linalg.norm(self.theta))))


def _train_step(world, es, obs, roll, pitch, bz):
    # capture the standing pose (model.joint_q) + height once for the in-hook reset
    if world._res_spawn_jq is None and world._res_ctr > es.settle and bz > 0.6:
        world._res_spawn_jq = world.model.joint_q.numpy().copy()
        world._res_spawn_z = float(bz)
        _log(world, "captured spawn jq (|q|=%.2f) + z=%.3f for reset"
             % (float(np.linalg.norm(world._res_spawn_jq)), bz))
    theta = es.member_theta()
    res = es.act_scale * _policy_apply(theta, obs)
    _apply_residual(world, res)
    es.ep_tick += 1
    t = es.ep_tick
    # perturbation: a SUSTAINED FORCE shove at the pelvis (add_body_force every tick for
    # push_dur), which tips the inverted pendulum about the feet -- the regime where the
    # base actually fails (a velocity impulse is instantly arrested by the stiff servo).
    # Optional velocity/tip push too. Triggered once at push_t or as a barrage (push_period).
    trig = (t == es.push_t) or (es.push_period and t > es.push_t
                                and (t - es.push_t) % es.push_period == 0)
    if trig:
        if es.push_vmax or es.push_wmax:
            es.cur_push = es.episode_push()
            _push(world, *es.cur_push)
        if es.push_fmax:
            ang = es.rng.uniform(0, 2 * math.pi)
            es.cur_force = (es.push_fmax * math.cos(ang), es.push_fmax * math.sin(ang))
            es.force_left = es.push_dur
        es.pushed = True
        if os.environ.get("RES_DEBUG", "0") == "1" and es.idx == 0:
            _log(world, "  TRIG ep_tick=%d fmax=%.0f cur_force=(%.0f,%.0f) force_left=%d bz=%.3f"
                 % (t, es.push_fmax, es.cur_force[0], es.cur_force[1], es.force_left, bz))
    if es.force_left > 0:
        _force(world, es.cur_force[0], es.cur_force[1])
        es.force_left -= 1
        if os.environ.get("RES_DEBUG", "0") == "1" and es.idx == 0 and t % 6 == 0:
            _log(world, "  FORCING ep_tick=%d bz=%.3f pitch=%.3f roll=%.3f" % (t, bz, pitch, roll))
    # Fitness = SURVIVAL + uprightness − effort, scored over the recovery window after the
    # shove. The in-hook reset (state mode + lean-state clear) restores a clean standing
    # start each episode WITHOUT the desyncing solver rebuild, so episodic ES works: a bad
    # member falls, gets a low fitness, and the next member starts fresh. The residual learns
    # to SURVIVE shoves (esp. lateral) the base alone fails -> a real surpass, zero gap.
    fell = (abs(roll) > es.fr) or (abs(pitch) > es.fp) or (bz < es.fbz)
    if t >= es.push_t - 4:                         # score from just before the shove
        upr = max(0.0, 1.0 - roll * roll - pitch * pitch)
        es.fit += 1.0 + 0.5 * upr - es.act_pen * float(np.sum(res * res))
    if fell:
        es.fit -= 50.0
        es.record_and_advance(world)
        return
    if t >= es.eplen:
        es.record_and_advance(world)
        return


def _eval_step(world, roll, pitch, bz):
    """Deterministic force-LADDER eval (uses the working in-hook reset): for each force level
    and each of N evenly-spaced directions, reset -> settle -> shove -> recover -> record
    survived/fell. Logs a survival-by-force table. Run with the residual ON (RES_POLICY set)
    vs OFF (RES_ACT_SCALE=0) to measure the surpass. The residual + base lean apply normally
    (this only schedules shoves + resets + scoring)."""
    e = getattr(world, "_res_eval", None)
    if world._res_spawn_jq is None:
        world._res_spawn_jq = world.model.joint_q.numpy().copy()
    if e is None:
        levels = [float(x) for x in os.environ.get(
            "RES_EVAL_LEVELS", "120,160,200,240,280,320,360,400").split(",")]
        e = world._res_eval = dict(
            levels=levels, ntrial=_i("RES_EVAL_NTRIAL", 6), li=0, ti=0, phase="settle",
            t=0, survived={}, settle=_i("RES_EVAL_SETTLE", 90),
            dur=max(1, int(_f("RES_PUSH_DUR", 0.25) / world._res_dt)),
            recover=_i("RES_EVAL_RECOVER", 220), done=False, fell=False)
        _log(world, "EVAL start levels=%s ntrial=%d residual=%s scale=%.2f"
             % (levels, e["ntrial"], os.environ.get("RES_POLICY", "")[-20:],
                _f("RES_ACT_SCALE", 0.25)))
    if e["done"]:
        return
    fell = (abs(roll) > 0.8) or (abs(pitch) > 0.8) or (bz < 0.45)
    e["t"] += 1
    lvl = e["levels"][e["li"]]; ang = 2 * math.pi * e["ti"] / e["ntrial"]
    if e["phase"] == "settle":
        if e["t"] >= e["settle"]:
            e["phase"] = "shove"; e["t"] = 0; e["fell"] = False
    elif e["phase"] == "shove":
        _force(world, lvl * math.cos(ang), lvl * math.sin(ang))
        if fell:
            e["fell"] = True
        if e["t"] >= e["dur"]:
            e["phase"] = "recover"; e["t"] = 0
    elif e["phase"] == "recover":
        if fell:
            e["fell"] = True
        if e["t"] >= e["recover"]:
            k = int(lvl)
            e["survived"].setdefault(k, [0, 0])
            e["survived"][k][1] += 1
            if not e["fell"]:
                e["survived"][k][0] += 1
            _log(world, "EVAL %dN dir=%3.0f %s" % (k, math.degrees(ang),
                 "SURVIVED" if not e["fell"] else "FELL"))
            e["ti"] += 1
            if e["ti"] >= e["ntrial"]:
                e["ti"] = 0; e["li"] += 1
                if e["li"] >= len(e["levels"]):
                    e["done"] = True
                    tbl = ", ".join("%dN:%d/%d" % (k, v[0], v[1])
                                    for k, v in sorted(e["survived"].items()))
                    _log(world, "EVAL DONE survival-by-force: %s" % tbl)
                    return
            e["phase"] = "settle"; e["t"] = 0
            _try_reset(world, "state", world._res_spawn_jq)


def res_step(world):
    if not _build(world):
        return
    world._res_ctr = getattr(world, "_res_ctr", 0) + 1
    mode = os.environ.get("RES_MODE", "deploy")
    warm = _i("RES_WARM_TICKS", 120)
    try:
        obs, roll, pitch, bz, _q, _v = _read_obs(world)
    except Exception as e:
        _log(world, "obs read err %r" % (e,)); return
    # In-hook STATELESS base lean (the deterministic balance) -- applied in ALL modes so the
    # base is owned by THIS hook (reset-consistent). Run the controller with HSTAND_LEAN=0.
    if world._res_ctr > _i("RES_LEAN_WARM", 60):
        try:
            _apply_targets(world, _base_lean(world, roll, pitch, obs[3:6], obs[6:9]))
        except Exception as e:
            _log(world, "lean err %r" % (e,))
    if mode == "probe":
        if world._res_ctr % 125 == 1:
            _log(world, "probe t=%d bz=%.3f roll=%.3f pitch=%.3f obs=%s"
                 % (world._res_ctr, bz, roll, pitch, np.round(obs, 3).tolist()))
        # one-time: find the pelvis Newton body (the one near pelvis height) + confirm
        # body_xform/body_vel work, so set_body_vel/reset_body_pose target the right body.
        if world._res_ctr == 300 and os.environ.get("RES_FINDBASE", "0") == "1":
            try:
                zs = []
                for i in range(0, 45):
                    try:
                        bx = world.body_xform(i)
                        zs.append((i, round(float(bx[2]), 3)))
                    except Exception:
                        break
                _log(world, "BODY z: %s pelvis=%d" % (zs, world._res_base_idx))
            except Exception as e:
                _log(world, "findbase err %r" % (e,))
        # sustained-force test: apply RES_TEST_F (N, +x) to the pelvis every tick during
        # 300-345 and watch pitch/bz -- if a big force barely moves it, the force path is
        # ineffective and we use cubes instead.
        _tf0 = _i("RES_TEST_T", 300); _tdur = _i("RES_TEST_DUR", 31)
        if _tf0 <= world._res_ctr <= _tf0 + _tdur and os.environ.get("RES_FINDBASE", "0") == "1":
            _force(world, _f("RES_TEST_F", 1500.0), _f("RES_TEST_FY", 0.0))
        if _tf0 <= world._res_ctr <= _tf0 + 260 and world._res_ctr % 15 == 0 \
                and os.environ.get("RES_FINDBASE", "0") == "1":
            _log(world, "post-force t=%d bz=%.3f roll=%.3f pitch=%.3f vx=%.2f"
                 % (world._res_ctr, bz, roll, pitch, obs[6]))
        # RESET TEST: capture standing -> knock forward -> RESET -> knock lateral (liveness).
        # A good reset restores bz~0.76/pitch~0 AND the obs RESPONDS to the 2nd knock (live).
        if os.environ.get("RES_RESETTEST", "0") == "1":
            c = world._res_ctr
            if c == 150:
                world._res_spawn_jq = world.model.joint_q.numpy().copy()
                _log(world, "RT capture spawn_jq |q|=%.2f bz=%.3f"
                     % (float(np.linalg.norm(world._res_spawn_jq)), bz))
            if 200 <= c <= 250:
                _force(world, 450.0, 0.0)                  # knock forward (should topple)
            if c == 300 and getattr(world, "_res_spawn_jq", None) is not None:
                _try_reset(world, os.environ.get("RES_RESET_MODE", "update"), world._res_spawn_jq)
                _log(world, "RT RESET mode=%s applied" % os.environ.get("RES_RESET_MODE", "update"))
            if 400 <= c <= 430:
                _force(world, 0.0, 250.0)                  # 2nd knock (lateral) = liveness probe
            if c in (190, 250, 270, 299, 303, 308, 320, 345, 395, 415, 425, 445, 465):
                tag = ("PRE" if c < 200 else "KNOCK1" if c < 300
                       else "POSTRESET" if c < 400 else "KNOCK2")
                _log(world, "RT %-9s t=%d bz=%.3f roll=%.3f pitch=%.3f" % (tag, c, bz, roll, pitch))
        return
    if world._res_ctr < warm:
        return
    if mode == "train":
        if getattr(world, "_res_es", None) is None:
            world._res_es = _ES(world)
            _log(world, "TRAIN start pop=%d sigma=%.3f lr=%.3f params=%d ep=%d settle=%d push@%d vmax=%.2f"
                 % (world._res_es.pop, world._res_es.sigma, world._res_es.lr, N_PARAM,
                    world._res_es.eplen, world._res_es.settle, world._res_es.push_t,
                    world._res_es.push_vmax))
        try:
            _train_step(world, world._res_es, obs, roll, pitch, bz)
        except Exception as e:
            _log(world, "train step err %r" % (e,))
        return
    # deploy mode: load policy once, apply each tick
    if getattr(world, "_res_theta", None) is None:
        world._res_theta = _load_theta(os.environ.get("RES_POLICY", ""))
        world._res_scale = _f("RES_ACT_SCALE", 0.25)
        _log(world, "DEPLOY policy loaded |theta|=%.3f scale=%.2f"
             % (float(np.linalg.norm(world._res_theta)), world._res_scale))
    try:
        res = world._res_scale * _policy_apply(world._res_theta, obs)
        _apply_residual(world, res)
    except Exception as e:
        _log(world, "deploy apply err %r" % (e,))
    if os.environ.get("RES_EVAL", "0") == "1":
        try:
            _eval_step(world, roll, pitch, bz)
        except Exception as e:
            _log(world, "eval err %r" % (e,))
