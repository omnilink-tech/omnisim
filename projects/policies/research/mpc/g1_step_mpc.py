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

"""In-engine PUSH-RECOVERY MPC for the Unitree G1, on the POSITION SERVO (no torque,
no RL). Loaded via the engine hook
    OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_step_mpc:step_mpc"
    OMNISIM_INENGINE_PYMOD_SS=1            (per-substep, 250 Hz)

WHY THIS SIDESTEPS THE TORQUE-WBC WALLS
  The deterministic torque-WBC (g1_walk_wbc.py) died on torque-mode problems:
  contact-consistency (assumed rigid f vs the solver's realized soft f), the squat
  seed transient, the 62.5 Hz rate, the joint_f sign. This controller has NONE of
  those because it (a) drives the POSITION SERVO (control.joint_target_q), which
  the engine integrates robustly per-substep, and (b) plans by MPPI rolling out in
  the deploy's OWN mujoco_warp solver -- so the contact it optimizes against IS the
  realized contact (zero model gap, exactly like the validated quad MPC).

HOW IT WORKS  (mirrors the proven quad_mpc_engine.py)
  1. Let the deterministic stand controller settle the robot, then CAPTURE the
     settled servo targets as the static nominal pose q_nom.
  2. Each plan: sample K residual vectors, roll each out H steps in mujoco_warp with
     ctrl = q_nom + residual, score the terminal state (uprightness + height + low
     CoM velocity), importance-weight -> updated nominal residual (MPPI).
  3. Apply q_nom + residual to the live servo targets every substep. The servo holds
     the robot; the MPC is the balance/recovery feedback, optimized in real physics.

  Phase A (now): constant residual over the horizon = optimized ankle/hip/arm balance
  -- the question is whether MPPI-on-servo matches/beats the deterministic+WBC base.
  Phase B (next): capture-point-triggered time-varying leg residual = recovery STEPS.

Run it with scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Mpc  (servo mode).
Plain python+numpy+mujoco; edit + relaunch to iterate (no C++ rebuild).
"""
import math
import os

import numpy as np


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
        world._mpc_log("smpc: " + msg)
    except Exception:
        import sys
        sys.stderr.write("[g1_step_mpc] " + msg + "\n")


# ---------------------------------------------------------------------------
# One-time setup: map controlled joints to (newton DOF, mjc actuator, qpos adr).
# ---------------------------------------------------------------------------
def _build(world):
    if getattr(world, "_smpc_ready", None) is not None:
        return world._smpc_ready
    world._smpc_ready = False
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

    # joint -> its position actuator (affine position servo: biasprm[1] != 0)
    act_of = {}
    for a in range(int(mjm.nu)):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            j = int(mjm.actuator_trnid[a, 0])
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and j not in act_of:
                act_of[j] = a

    # Newton DOF layout (G1, URDF order): base free 0-5; 6-11 left leg
    # (hip_pitch/roll/yaw, knee, ankle_pitch, ankle_roll); 12-17 right leg;
    # 18 waist_yaw; 19-23 left arm; 24-28 right arm.
    dof = []; act = []; qadr = []; feet = {}
    for j in range(int(mjm.njnt)):
        nd = int(m2nd[j])
        if 6 <= nd <= 28 and j in act_of:
            dof.append(nd); act.append(act_of[j]); qadr.append(int(mjm.jnt_qposadr[j]))
        if nd == 11:
            feet["left"] = int(mjm.jnt_bodyid[j])
        if nd == 17:
            feet["right"] = int(mjm.jnt_bodyid[j])
    if not dof or "left" not in feet or "right" not in feet:
        _log(world, "bad maps n=%d feet=%s" % (len(dof), feet)); return False
    order = np.argsort(dof)
    world._smpc_dof = np.array(dof, np.int32)[order]
    world._smpc_act = np.array(act, np.int32)[order]
    world._smpc_qadr = np.array(qadr, np.int32)[order]
    world._smpc_feet = feet
    n = len(world._smpc_dof)
    world._smpc_n = n

    # leg DOFs (newton 6-17) vs arm/waist (18-28): different residual scale (legs do
    # the heavy balance/stepping work; arms add momentum; small for now).
    # Per-leg 6-dof map by JOINT GEOMETRY (the joint names are generic "joint_K" and
    # the newton dof order is breadth-first, not URDF order -- so classify by axis +
    # body height + y-sign). leg_ik order = [hip_pitch, hip_roll, hip_yaw, knee,
    # ankle_pitch, ankle_roll]. A leg joint = its body is below the pelvis; within a
    # leg, axis x -> {hip_roll(higher), ankle_roll(lower)}, axis y -> {hip_pitch(top),
    # knee(mid), ankle_pitch(bottom)}, axis z -> hip_yaw. L/R by body y-sign.
    nd2i = {int(world._smpc_dof[i]): i for i in range(n)}
    dcl = mj.MjData(mjm); mj.mj_forward(mjm, dcl)
    # base/pelvis body = the free joint's child body
    pelvis_bid = None
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            pelvis_bid = int(mjm.jnt_bodyid[j]); break
    pelvis_z = float(dcl.xpos[pelvis_bid][2]) if pelvis_bid is not None else 0.7
    cand = []   # (nd, qadr, act, bodypos(xyz), axisdom)  for leg joints (body below pelvis)
    for j in range(int(mjm.njnt)):
        nd = int(m2nd[j])
        if not (6 <= nd <= 28) or j not in act_of:
            continue
        bid = int(mjm.jnt_bodyid[j]); pos = np.array(dcl.xpos[bid], float)
        if pos[2] >= pelvis_z - 0.02:       # at/above pelvis -> arm/waist, not a leg
            continue
        ax = np.abs(np.array(mjm.jnt_axis[j], float)); axdom = int(np.argmax(ax))  # 0=x,1=y,2=z
        cand.append((nd, int(mjm.jnt_qposadr[j]), int(act_of[j]), pos, axdom))
    world._smpc_leg = {}
    for side, ysign in (("left", 1.0), ("right", -1.0)):
        side_j = [c for c in cand if (c[3][1] * ysign) > 0]
        pit = sorted([c for c in side_j if c[4] == 1], key=lambda c: -c[3][2])  # y-axis, top->bottom
        rol = sorted([c for c in side_j if c[4] == 0], key=lambda c: -c[3][2])  # x-axis, top->bottom
        yaw = [c for c in side_j if c[4] == 2]
        if len(pit) >= 3 and len(rol) >= 2 and len(yaw) >= 1:
            chosen = {"hip_pitch": pit[0], "hip_roll": rol[0], "hip_yaw": yaw[0],
                      "knee": pit[1], "ankle_pitch": pit[2], "ankle_roll": rol[-1]}
            order6 = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
            # foot body = the ankle_roll joint's child body
            ar_nd = chosen["ankle_roll"][0]
            foot_bid = None
            for j in range(int(mjm.njnt)):
                if int(m2nd[j]) == ar_nd:
                    foot_bid = int(mjm.jnt_bodyid[j]); break
            world._smpc_leg[side] = {
                "dof": [chosen[k][0] for k in order6],
                "qadr": [chosen[k][1] for k in order6],
                "act": [chosen[k][2] for k in order6],
                "ri": [nd2i.get(chosen[k][0], -1) for k in order6],
                "foot_bid": foot_bid,
            }
    world._smpc_step_ok = len(world._smpc_leg) == 2
    # persistent CPU MjData for per-tick CoM/foot reads (CP / step logic)
    world._smpc_d = mj.MjData(mjm)
    world._smpc_mj = mj
    world._smpc_nq = int(mjm.nq); world._smpc_nv = int(mjm.nv)
    if world._smpc_step_ok:
        _log(world, "leg map OK L.dof=%s R.dof=%s (pelvis_z=%.2f)" % (
            world._smpc_leg["left"]["dof"], world._smpc_leg["right"]["dof"], pelvis_z))
    else:
        _log(world, "leg map INCOMPLETE (%d leg cand) -> stepping disabled" % len(cand))

    # leg dofs from the geometric leg map (the newton layout is breadth-first, so a
    # dof<=17 heuristic mis-classifies knee_R/ankles). Legs get the big residual scale.
    _legset = set()
    for _s in world._smpc_leg.values():
        _legset.update(int(d) for d in _s["dof"])
    is_leg = np.array([int(world._smpc_dof[i]) in _legset for i in range(n)]) if _legset \
        else (world._smpc_dof <= 17)
    sig = np.where(is_leg, _f("SMPC_SIGMA_LEG", 0.06), _f("SMPC_SIGMA_ARM", 0.10))
    rmx = np.where(is_leg, _f("SMPC_RESMAX_LEG", 0.30), _f("SMPC_RESMAX_ARM", 0.40))
    world._smpc_sigma = sig.astype(np.float64)
    world._smpc_resmax = rmx.astype(np.float64)
    world._smpc_nom = np.zeros(n)            # MPPI nominal residual
    world._smpc_qnom = None                  # captured static stand pose (set at warm)
    world._smpc_rng = np.random.default_rng(0)
    world._smpc_dt = float(getattr(world, "_n_substeps", 4)) * float(mjm.opt.timestep)
    world._smpc_ctr = 0
    world._smpc_ready = True
    _log(world, "ready n=%d legs=%d dof=%s feet=%s dt=%.4f"
         % (n, int(is_leg.sum()), list(world._smpc_dof), feet, world._smpc_dt))
    return True


def _capture_nominal(world):
    """Snapshot the (settled) live servo targets as the static nominal pose.
    The nominal is read from the ACTUAL joint angles (qpos[qadr]) -- the robot is
    standing in a good squat by now, and qpos is unambiguously indexed (vs the
    joint_target_pos buffer whose dof indexing we cross-check below)."""
    tp = world.control.joint_target_q.numpy()
    qpos = world.solver.mjw_data.qpos.numpy().reshape(-1)
    qn = np.zeros(world._smpc_n)
    for i in range(world._smpc_n):
        qa = int(world._smpc_qadr[i])
        qn[i] = float(qpos[qa]) if 0 <= qa < len(qpos) else 0.0
    world._smpc_qnom = qn
    # anchor position: where the base is now (so the MPC holds station, not drifts).
    q = world.solver.mjw_data.qpos.numpy().reshape(-1)
    world._smpc_base0 = np.array([float(q[0]), float(q[1])])
    # one-time indexing cross-check: target-by-dof vs actual-by-qadr, first 6 (L leg)
    dbg = []
    for i in range(6):
        di = int(world._smpc_dof[i]); qa = int(world._smpc_qadr[i])
        tv = float(tp[di]) if 0 <= di < len(tp) else float("nan")
        dbg.append("nd%d:tp=%.2f q=%.2f" % (di, tv, qn[i]))
    _log(world, "captured nominal (from qpos) L=%s | xcheck %s"
         % (np.round(qn[:6], 3).tolist(), " ".join(dbg)))
    _log(world, "ALL nd->q: %s" % " ".join(
        "%d=%.2f" % (int(world._smpc_dof[i]), qn[i]) for i in range(world._smpc_n)))


def _cs(world):
    """Import the validated capture_step module (leg FK/IK + capture_point)."""
    cs = getattr(world, "_smpc_cs", None)
    if cs is not None:
        return cs
    import sys
    d = os.path.join(os.environ.get("OMNISIM_HOME", "."), "projects", "policies",
                     "controllers", "humanoid_stand_deploy")
    if d not in sys.path:
        sys.path.insert(0, d)
    import capture_step as cs
    world._smpc_cs = cs
    return cs


def _read(world):
    """Live CoM, CoM velocity, foot positions (world), pelvis xy + yaw, from a CPU MjData."""
    mj = world._smpc_mj; mjm = world.solver.mj_model; live = world.solver.mjw_data
    d = world._smpc_d
    d.qpos[:] = live.qpos.numpy().reshape(-1)[:world._smpc_nq]
    d.qvel[:] = live.qvel.numpy().reshape(-1)[:world._smpc_nv]
    mj.mj_forward(mjm, d)
    try:
        mj.mj_subtreeVel(mjm, d)
    except Exception:
        pass
    com = np.array(d.subtree_com[1], float)
    com_v = np.array(d.subtree_linvel[1], float)
    feet = {s: np.array(d.xpos[world._smpc_leg[s]["foot_bid"]], float)
            for s in world._smpc_leg}
    qw, qx, qy, qz = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return com, com_v, feet, np.array([d.qpos[0], d.qpos[1]]), yaw


def _step_fsm(world):
    """Capture-point step state machine. Sets world._smpc_swing = {dof: target} for the
    swing leg during a step (None when standing). The MPPI residual balances stance+arms;
    the swing leg follows leg_ik to a foothold at the predicted capture point."""
    cs = _cs(world)
    com, com_v, feet, pelvis, yaw = _read(world)
    cyaw, syaw = math.cos(yaw), math.sin(yaw)
    # world -> pelvis-yaw frame (fwd=x, lateral=y)
    def to_body(v2):
        return np.array([cyaw * v2[0] + syaw * v2[1], -syaw * v2[0] + cyaw * v2[1]])
    sup = 0.5 * (feet["left"][:2] + feet["right"][:2])
    cp = cs.capture_point(com[:2], com_v[:2], com[2])
    d_body = to_body(cp - sup)
    margin = _f("SMPC_CP_MARGIN", 0.11)
    lift_h = _f("SMPC_STEP_LIFT", 0.06)
    shift_ticks = max(0, _i("SMPC_SHIFT_TICKS", 12))
    # Khadiv-style step-TIMING adaptation (T-RO 2020): with tau=e^{omega*T} the DCM step
    # map is linear, and the lever for a hard push is to step SOONER (smaller T) so the
    # predicted DCM at touchdown stays within leg reach. dt=tick; reach_max ~ leg step.
    dt = float(world._smpc_dt)
    omega = math.sqrt(9.81 / max(0.3, com[2]))
    reach_max = _f("SMPC_REACH_MAX", 0.34)
    T_min = _f("SMPC_T_MIN", 0.10); T_nom = _f("SMPC_T_NOM", 0.30)

    def _plan_step(stance_xy):
        # adaptive step time: T = (1/omega) ln(reach_max / d), clamped. Gentle push (small
        # d) -> T~T_nom (normal rhythm); hard push (d->reach) -> T->T_min (step ASAP); d>=
        # reach -> step now at the reach boundary (partial capture, chain next step).
        drel = cp - stance_xy
        d = float(np.linalg.norm(drel))
        if d > 1e-3:
            T = (1.0 / omega) * math.log(max(1e-3, reach_max / d))
        else:
            T = T_nom
        T = float(min(max(T, T_min), T_nom))
        ticks = max(2, int(round(T / dt)))
        foot_world = stance_xy + drel * math.exp(omega * T)   # predicted DCM at touchdown
        # adaptive weight-shift: full when gentle (margin to spare), skip when urgent.
        frac = (T - T_min) / max(1e-6, T_nom - T_min)
        sh = int(round(shift_ticks * frac))
        return T, ticks, foot_world, d, sh

    st = getattr(world, "_smpc_state", "STAND")
    c = world._smpc_ctr
    if st == "STAND":
        world._smpc_swing = None
        world._smpc_anchor = None
        esc = max(abs(d_body[0]), abs(d_body[1]))
        if esc > margin:
            # step the leg on the side the CP escaped toward (lateral dominant) else the
            # trailing leg (fore/aft): swing foot lands UNDER the capture point.
            if abs(d_body[1]) >= abs(d_body[0]):
                swing = "left" if d_body[1] > 0 else "right"
            else:
                swing = "left" if (to_body(feet["left"][:2] - sup)[0] < 0) else "right"
            stance = "right" if swing == "left" else "left"
            T, ticks, _fw, d, sh = _plan_step(feet[stance][:2])
            world._smpc_step = {"swing": swing, "stance": stance, "shift_start": c,
                                "shift_ticks": sh, "T": T, "ticks": ticks}
            world._smpc_state = "SHIFT" if sh > 0 else "SWINGINIT"
            _log(world, "TRIG esc=%.3f(%.2f,%.2f) sw=%s d=%.2f T=%.2f(%dt) shift=%d -> %s" % (
                esc, d_body[0], d_body[1], swing, d, T, ticks, sh, world._smpc_state))
    if getattr(world, "_smpc_state", "STAND") == "SHIFT":
        # WEIGHT SHIFT in DOUBLE support: drive the CoM/pelvis over the stance foot (MPPI
        # position anchor) BEFORE lifting -- so the swing foot leaves the ground without the
        # CoM falling outside the single support laterally. Duration is push-adaptive.
        sp = world._smpc_step
        world._smpc_anchor = feet[sp["stance"]][:2].copy()
        world._smpc_swing = None
        if c - sp["shift_start"] >= sp["shift_ticks"]:
            world._smpc_state = "SWINGINIT"
    if getattr(world, "_smpc_state", "STAND") == "SWINGINIT":
        # weight is on the stance foot -> re-plan timing+foothold at the CURRENT DCM (it
        # moved during the shift) and start the swing. Adaptive T keeps the foot reachable.
        sp = world._smpc_step; swing = sp["swing"]; stance = sp["stance"]
        lg = world._smpc_leg[swing]
        q0 = np.array([float(world._smpc_d.qpos[lg["qadr"][k]]) for k in range(6)])
        f0 = cs.leg_fk(q0, swing)
        stance_xy = feet[stance][:2]
        T, ticks, foot_world, d, _sh = _plan_step(stance_xy)
        sp.update({"f0": f0, "q0": q0, "foot_world": foot_world.copy(),
                   "footz": float(f0[2]), "start": c, "ticks": ticks, "T": T})
        world._smpc_state = "STEP"
        world._smpc_anchor = stance_xy.copy()
        _log(world, "SWING sw=%s d=%.2f T=%.2f(%dt) e^wT=%.2f cp=%s foot=%s" % (
            swing, d, T, ticks, math.exp(omega * T), np.round(cp, 3).tolist(),
            np.round(foot_world, 3).tolist()))
    if getattr(world, "_smpc_state", "STAND") == "STEP":
        sp = world._smpc_step
        world._smpc_anchor = feet[sp["stance"]][:2].copy()   # keep weight on the stance foot
        ph = (c - sp["start"]) / float(sp["ticks"])
        if ph >= 1.0:
            world._smpc_state = "STAND"; world._smpc_swing = None; world._smpc_anchor = None
            _log(world, "STEP done -> STAND (cp esc now %.3f)" % max(abs(d_body[0]), abs(d_body[1])))
            return
        # world foothold -> CURRENT pelvis frame (the pelvis moves during the swing), so
        # the foot tracks the planted world target. Reach box is generous (a human takes
        # a BIG recovery step); leg_ik lands as far as the leg physically reaches.
        fxy = to_body(sp["foot_world"] - pelvis)
        rmax_f = _f("SMPC_REACH_FWD", 0.42); rmax_l = _f("SMPC_REACH_LAT", 0.34)
        ftgt = np.array([float(np.clip(fxy[0], -rmax_f, rmax_f)),
                         float(np.clip(fxy[1], -rmax_l, rmax_l)), sp["footz"]])
        fnow = sp["f0"] + (ftgt - sp["f0"]) * ph
        fnow = fnow.copy(); fnow[2] += lift_h * math.sin(math.pi * ph)
        q = cs.leg_ik(fnow, sp["swing"], q0=sp["q0"])
        lg = world._smpc_leg[sp["swing"]]
        world._smpc_swing = {"leg": sp["swing"],
                             "dof": {int(lg["dof"][k]): float(q[k]) for k in range(6)},
                             "act": {int(lg["act"][k]): float(q[k]) for k in range(6)},
                             "ri": [lg["ri"][k] for k in range(6)]}


def _plan(world, K, H):
    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        return
    _mjw, rm, rd, (nq, nv, nu) = buf
    import warp as wp
    sub = int(getattr(world, "_n_substeps", 4))
    lam = _f("SMPC_LAM", 0.15)
    act = world._smpc_act
    qnom = world._smpc_qnom
    zref = _f("SMPC_ZREF", 0.0)               # 0 => use captured base z as the ref
    # cost weights (terminal uprightness dominates; light running penalties so the
    # planner is free to use ankle/hip authority; FALL for toppling rollouts).
    WUP = _f("SMPC_W_UP", 30.0); WH = _f("SMPC_W_H", 120.0)
    WVXY = _f("SMPC_W_VXY", 6.0); WRATE = _f("SMPC_W_RATE", 2.0)
    WRES = _f("SMPC_W_RES", 0.4); FALL = _f("SMPC_W_FALL", 400.0)
    WPOS = _f("SMPC_W_POS", 40.0)            # hold station (anchor to capture xy)
    # during a SHIFT/STEP the FSM moves the anchor over the stance foot (weight shift);
    # otherwise hold the captured standing xy.
    anc = getattr(world, "_smpc_anchor", None)
    b0 = anc if anc is not None else getattr(world, "_smpc_base0", np.zeros(2))

    base = world._mpc_seed_qv(K)              # seeds rd.qpos/qvel from live; nominal ctrl
    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype
    q0 = rd.qpos.numpy().reshape(K, nq)
    if zref <= 0.0:
        zref = float(q0[0, 2])               # hold the live base height
    noise = world._smpc_rng.normal(0, 1, (K, world._smpc_n)) * world._smpc_sigma[None, :]
    delta = np.clip(world._smpc_nom[None, :] + noise, -world._smpc_resmax[None, :],
                    world._smpc_resmax[None, :])
    delta[0] = world._smpc_nom               # keep the current plan as a candidate
    # Architecture B: residual ON TOP of the LIVE deterministic-stand command (base =
    # the controller's current servo targets, from _mpc_seed_qv). The controller's solid
    # balance stays; the MPC adds delta. Undisturbed -> best delta ~0 -> stand unchanged.
    c = np.broadcast_to(base, (K, nu)).copy()
    c[:, act] = c[:, act] + delta
    # during a STEP, the swing leg follows leg_ik (override command+residual) so the
    # rollout evaluates the actual step; the MPPI still balances stance+arms via delta.
    sw = getattr(world, "_smpc_swing", None)
    if sw is not None:
        for a, tv in sw["act"].items():
            c[:, a] = tv
    rd.ctrl.assign(c.reshape(cshape).astype(cdt))
    # CUDA-graph the rollout: ctrl/qpos/qvel are set above (outside the graph), the
    # graph only RECORDS the H*sub step kernels reading those buffers, so each plan
    # = (assign seed+ctrl) + capture_launch -> no per-step Python/launch overhead.
    # ~10-50x over the Python step loop. Falls back to the loop if capture fails.
    def _roll_loop():
        for _h in range(H):
            for _ in range(sub):
                _mjw.step(rm, rd)
    g = getattr(world, "_smpc_graph", None)
    if g is None and not getattr(world, "_smpc_graph_failed", False):
        try:
            dev = world.model.device
            if "cuda" not in str(dev).lower():
                raise RuntimeError("rollout not on cuda")
            with wp.ScopedDevice(dev):
                wp.synchronize()
                wp.capture_begin(force_module_load=False)
                try:
                    _roll_loop()
                finally:
                    world._smpc_graph = wp.capture_end()
            g = world._smpc_graph
            _log(world, "rollout CUDA graph captured (K=%d H=%d sub=%d)" % (K, H, sub))
        except Exception as e:
            world._smpc_graph_failed = True; world._smpc_graph = None; g = None
            _log(world, "rollout graph capture failed (%r) -> python loop" % (e,))
    try:
        if g is not None:
            wp.capture_launch(g)
        else:
            _roll_loop()
        wp.synchronize()
    except Exception:
        pass
    q = rd.qpos.numpy().reshape(K, nq); v = rd.qvel.numpy().reshape(K, nv)
    w_, x_, y_, z_ = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    pit = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    bz = q[:, 2]
    vx, vy = v[:, 0], v[:, 1]; wx, wy = v[:, 3], v[:, 4]
    J = (WUP * (roll * roll + pit * pit)
         + WH * np.maximum(0.0, zref - bz) ** 2
         + WVXY * (vx * vx + vy * vy)
         + WRATE * (wx * wx + wy * wy)
         + WPOS * ((q[:, 0] - b0[0]) ** 2 + (q[:, 1] - b0[1]) ** 2)
         + WRES * (delta * delta).sum(1))
    J = J + ((bz < 0.55 * zref) | (np.abs(roll) > 0.7) | (np.abs(pit) > 0.7)) * FALL
    wts = np.exp(-(J - J.min()) / lam); wts /= wts.sum() + 1e-9
    world._smpc_nom = np.clip((wts[:, None] * delta).sum(0),
                              -world._smpc_resmax, world._smpc_resmax)


def _plan_seg(world, K, NSEG, HSEG):
    """Segmented (piecewise-constant spline) MPPI -- the MuJoCo-MPC / Predictive-Sampling
    recipe: each sample is NSEG residual segments held HSEG ticks each, rolled out over a
    LONG horizon (NSEG*HSEG*sub steps) in the engine, so a recovery STEP (lift->swing->
    plant, a time-varying motion) can fit in a sample and EMERGE from the balance cost.
    Graphed via a per-segment ctrl buffer (NSEG copies + steps) -> stays fast.
    world._smpc_nom is (NSEG, n); _apply uses segment 0; step_mpc warm-shifts the rest."""
    buf = world._mpc_rollout_buffers(K)
    if buf is None:
        return
    _mjw, rm, rd, (nq, nv, nu) = buf
    import warp as wp
    sub = int(getattr(world, "_n_substeps", 4))
    lam = _f("SMPC_LAM", 0.15)
    act = world._smpc_act
    n = world._smpc_n
    zref = _f("SMPC_ZREF", 0.0)
    WUP = _f("SMPC_W_UP", 30.0); WH = _f("SMPC_W_H", 120.0)
    WVXY = _f("SMPC_W_VXY", 6.0); WRATE = _f("SMPC_W_RATE", 2.0)
    WRES = _f("SMPC_W_RES", 0.4); FALL = _f("SMPC_W_FALL", 400.0)
    WPOS = _f("SMPC_W_POS", 40.0)
    WSUP = _f("SMPC_W_SUP", 0.0)              # CoM-over-feet-center (rewards STEPPING)
    # GATED MPC-OWNS: own only while ENGAGED (the recovery window). Then the rollout base is
    # the FIXED stand pose (qnom), matching what _apply deploys (qnom + spline) -> the rollout
    # is self-consistent (no reactive-base mismatch) so a recovering plan actually TRANSFERS.
    own = _i("SMPC_OWN", 0) or int(getattr(world, "_smpc_engaged", 0))
    b0 = getattr(world, "_smpc_base0", np.zeros(2))

    base = world._mpc_seed_qv(K)
    if own and getattr(world, "_smpc_qnom", None) is not None:
        base = np.array(base).reshape(-1).copy()
        for i in range(n):
            ai = int(world._smpc_act[i])
            if 0 <= ai < base.shape[0]:
                base[ai] = float(world._smpc_qnom[i])   # rollout commands the stand pose
    cshape = rd.ctrl.numpy().shape; cdt = rd.ctrl.numpy().dtype
    q0 = rd.qpos.numpy().reshape(K, nq)
    if zref <= 0.0:
        zref = float(q0[0, 2])
    # nominal (NSEG, n); sample noise per segment; sample 0 keeps the current plan.
    nomS = world._smpc_nom                                   # (NSEG, n)
    sig = world._smpc_sigma[None, None, :]; rmx = world._smpc_resmax[None, None, :]
    noise = world._smpc_rng.normal(0, 1, (K, NSEG, n)) * sig
    delta = np.clip(nomS[None, :, :] + noise, -rmx, rmx)     # (K, NSEG, n)
    delta[0] = nomS
    # build the per-segment ctrl buffer (NSEG, K, nu): base + that segment's residual.
    seg = np.broadcast_to(base, (NSEG, K, nu)).copy()
    for s in range(NSEG):
        seg[s][:, act] = seg[s][:, act] + delta[:, s, :]
    if getattr(world, "_smpc_segbuf", None) is None:
        world._smpc_segbuf = wp.zeros((NSEG,) + cshape, dtype=rd.ctrl.dtype, device=rd.ctrl.device)
    segbuf = world._smpc_segbuf
    segbuf.assign(seg.reshape((NSEG,) + cshape).astype(cdt))
    steps_seg = HSEG * sub

    def _roll_seg():
        for s in range(NSEG):
            wp.copy(rd.ctrl, segbuf[s])
            for _ in range(steps_seg):
                _mjw.step(rm, rd)
    g = getattr(world, "_smpc_graph", None)
    if g is None and not getattr(world, "_smpc_graph_failed", False):
        try:
            dev = world.model.device
            if "cuda" not in str(dev).lower():
                raise RuntimeError("rollout not on cuda")
            with wp.ScopedDevice(dev):
                wp.synchronize()
                wp.capture_begin(force_module_load=False)
                try:
                    _roll_seg()
                finally:
                    world._smpc_graph = wp.capture_end()
            g = world._smpc_graph
            _log(world, "SEG rollout graph captured (K=%d NSEG=%d HSEG=%d sub=%d horizon=%.2fs)"
                 % (K, NSEG, HSEG, sub, NSEG * steps_seg * float(rm.opt.timestep.numpy()[0])
                    if hasattr(rm.opt.timestep, "numpy") else NSEG * steps_seg * 0.002))
        except Exception as e:
            world._smpc_graph_failed = True; world._smpc_graph = None; g = None
            _log(world, "SEG graph capture failed (%r) -> loop" % (e,))
    try:
        if g is not None:
            wp.capture_launch(g)
        else:
            _roll_seg()
        wp.synchronize()
    except Exception:
        pass
    q = rd.qpos.numpy().reshape(K, nq); v = rd.qvel.numpy().reshape(K, nv)
    w_, x_, y_, z_ = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    pit = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    bz = q[:, 2]; vx, vy = v[:, 0], v[:, 1]; wx, wy = v[:, 3], v[:, 4]
    # CoM-over-feet-center (the balance term that REWARDS stepping: the support center
    # follows the feet, so moving a foot under the CoM is cheap -- vs absolute WPOS which
    # penalises any base motion incl. a recovery step). Read foot body xpos from the rollout.
    sup_pen = np.zeros(K)
    if WSUP > 0.0:
        try:
            xp = rd.xpos.numpy().reshape(K, -1, 3)
            Lb = world._smpc_leg["left"]["foot_bid"]; Rb = world._smpc_leg["right"]["foot_bid"]
            fcx = 0.5 * (xp[:, Lb, 0] + xp[:, Rb, 0]); fcy = 0.5 * (xp[:, Lb, 1] + xp[:, Rb, 1])
            sup_pen = (q[:, 0] - fcx) ** 2 + (q[:, 1] - fcy) ** 2
        except Exception:
            sup_pen = (q[:, 0] - b0[0]) ** 2 + (q[:, 1] - b0[1]) ** 2
    J = (WUP * (roll * roll + pit * pit)
         + WH * np.maximum(0.0, zref - bz) ** 2
         + WVXY * (vx * vx + vy * vy)
         + WRATE * (wx * wx + wy * wy)
         + WPOS * ((q[:, 0] - b0[0]) ** 2 + (q[:, 1] - b0[1]) ** 2)
         + WSUP * sup_pen
         + WRES * (delta * delta).sum((1, 2)))
    J = J + ((bz < 0.55 * zref) | (np.abs(roll) > 0.7) | (np.abs(pit) > 0.7)) * FALL
    wts = np.exp(-(J - J.min()) / lam); wts /= wts.sum() + 1e-9
    world._smpc_nom = np.clip((wts[:, None, None] * delta).sum(0), -rmx[0], rmx[0])
    world._smpc_lastJ = float(J.min())


def _apply(world):
    # Architecture B: ADD the MPPI residual to the controller's live per-tick command
    # (the controller setPositions every joint every tick, so tp is fresh -> no
    # accumulation; the hook runs before the substeps so the residual takes effect).
    # Segmented mode: nom is (NSEG, n) -> apply segment 0 (receding horizon).
    tp = world.control.joint_target_q.numpy()
    dof = world._smpc_dof; nom = world._smpc_nom
    seg0 = nom[0] if nom.ndim == 2 else nom
    own = _i("SMPC_OWN", 0) or int(getattr(world, "_smpc_engaged", 0))
    qn = getattr(world, "_smpc_qnom", None)
    for i in range(world._smpc_n):
        di = int(dof[i])
        if 0 <= di < len(tp):
            if own and qn is not None:
                tp[di] = float(qn[i]) + float(seg0[i])   # MPC OWNS (engaged): SET = stand pose + plan
            else:
                tp[di] = float(tp[di]) + float(seg0[i])  # idle: residual on the reactive base
    # during a STEP, override the swing leg with its leg_ik target (the controller
    # command + MPPI residual stays on the stance leg + arms).
    sw = getattr(world, "_smpc_swing", None)
    if sw is not None:
        for di, tv in sw["dof"].items():
            if 0 <= di < len(tp):
                tp[di] = tv
    world.control.joint_target_q.assign(tp)
    world._mjc_dirty = True


def _det_balance(world):
    """DETERMINISTIC ankle+hip+ARM-WINDMILL balance (Pratt hip/flywheel strategy + ankle),
    NO sampling -> no thrashing. The arm windmill is centroidal angular momentum: swinging
    the arms creates a reaction torque that arrests the torso rotation -- the body-rotation
    arrest every MPPI variant lacked. Sets world._smpc_nom (residual added on the base).
    Joint dofs are contiguous 6..28 so index = dof-6: hip_pitch 6,7 (idx0,1); shoulder_pitch
    11,12 (5,6); shoulder_roll 15,16 (9,10); ankle_pitch 21,22 (15,16); ankle_roll 25,26 (19,20)."""
    n = world._smpc_n
    live = world.solver.mjw_data
    qp = live.qpos.numpy().reshape(-1); qv = live.qvel.numpy().reshape(-1)
    qw, qx, qy, qz = qp[3], qp[4], qp[5], qp[6]
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    wx, wy = float(qv[3]), float(qv[4])
    sgn = _f("SMPC_DET_SIGN", 1.0)
    Karm = _f("SMPC_DET_ARM_KP", 5.0); Kad = _f("SMPC_DET_ARM_KD", 0.8)
    Krarm = _f("SMPC_DET_RARM_KP", 5.0)
    Kank = _f("SMPC_DET_ANK_KP", 1.5); Khip = _f("SMPC_DET_HIP_KP", 0.0)
    Kelb = _f("SMPC_DET_ELB_KP", 0.0)
    res = np.zeros(n)
    fwd = sgn * (Karm * pitch + Kad * wy)        # forward pitch -> arm windmill
    res[5] += fwd; res[6] += fwd                  # shoulder_pitch L,R
    if Kelb != 0.0:
        res[17] += sgn * Kelb * pitch; res[18] += sgn * Kelb * pitch   # elbow 23,24 (idx 17,18)
    rr = sgn * (Krarm * roll + Kad * wx)          # roll -> lateral arm windmill (opposite L/R)
    res[9] += rr; res[10] -= rr                   # shoulder_roll L,R
    res[15] += sgn * Kank * pitch; res[16] += sgn * Kank * pitch       # ankle_pitch lean
    res[0] += sgn * Khip * pitch; res[1] += sgn * Khip * pitch         # hip_pitch
    world._smpc_nom = np.clip(res, -world._smpc_resmax, world._smpc_resmax)
    world._smpc_det_dbg = (roll, pitch, wy, fwd)


def step_mpc(world):
    if not _build(world):
        return
    warm = _i("SMPC_WARM_TICKS", 380)         # ~1.5 s of substeps: let the stand settle
    K = _i("SMPC_K", 64); H = _i("SMPC_H", 15)
    every = max(1, _i("SMPC_EVERY", 2))
    NSEG = _i("SMPC_NSEG", 1)                  # >1 = segmented spline-MPPI (emergent stepping)
    HSEG = _i("SMPC_HSEG", 12)                 # ticks per segment
    world._smpc_ctr += 1
    c = world._smpc_ctr
    if c < warm:
        return                                # deterministic stand controller runs
    if world._smpc_qnom is None:
        _capture_nominal(world)
    # ----- DETERMINISTIC mode (SMPC_DET=1): ankle+hip+arm-windmill balance + capture-step,
    # NO sampling (no thrashing). The arm windmill is the body-rotation arrest. -----
    if _i("SMPC_DET", 0):
        if getattr(world, "_smpc_nom", None) is None or np.ndim(world._smpc_nom) != 1:
            world._smpc_nom = np.zeros(world._smpc_n)
        try:
            _det_balance(world)
        except Exception as e:
            _log(world, "det error: %r" % (e,)); world._smpc_nom = np.zeros(world._smpc_n)
        if world._smpc_step_ok and _f("SMPC_STEP_EN", 1.0) > 0:
            try:
                _step_fsm(world)
            except Exception as e:
                world._smpc_swing = None; _log(world, "fsm err %r" % (e,))
        _apply(world)
        if c % 60 == 1:
            try:
                r, p, wy, fw = getattr(world, "_smpc_det_dbg", (0, 0, 0, 0))
                q = world.solver.mjw_data.qpos.numpy().reshape(-1)
                _log(world, "DET t=%d base=(%.3f,%.3f,%.3f) roll=%.3f pit=%.3f arm=%.2f" %
                     (c, q[0], q[1], q[2], r, p, fw))
            except Exception:
                pass
        return
    # ----- SEGMENTED spline-MPPI mode (NSEG>1): reactive base stands + recovers small pushes
    # in-place; the MPPI engages ONLY when the capture point exceeds the base's in-place limit
    # (GATED) -> never over-acts on recoverable pushes, only plans a STEP when one is needed. --
    if NSEG > 1:
        if getattr(world, "_smpc_nom", None) is None or np.ndim(world._smpc_nom) != 2 \
                or world._smpc_nom.shape[0] != NSEG:
            world._smpc_nom = np.zeros((NSEG, world._smpc_n))
        world._smpc_swing = None
        trig = _f("SMPC_TRIG_MARGIN", 0.0)    # 0 = always engage; >0 = gate on CP escape
        engage = True; esc = -1.0
        if trig > 0.0 and world._smpc_step_ok:
            try:
                cs = _cs(world)
                com, com_v, feet, pelvis, yaw = _read(world)
                cp = cs.capture_point(com[:2], com_v[:2], com[2])
                sup = 0.5 * (feet["left"][:2] + feet["right"][:2])
                esc = float(np.linalg.norm(cp - sup))
                # latch: once engaged, stay engaged for SMPC_ENGAGE_TICKS to finish the recovery.
                lat = getattr(world, "_smpc_engage_until", 0)
                if esc > trig:
                    lat = c + _i("SMPC_ENGAGE_TICKS", 120)
                    world._smpc_engage_until = lat
                engage = (esc > trig) or (c < lat)
            except Exception:
                engage = False           # gate read failed -> let the reactive base stand
        world._smpc_engaged = 1 if engage else 0   # drives gated MPC-owns in _plan_seg/_apply
        if engage:
            if (c - warm) % every == 0:
                try:
                    _plan_seg(world, K, NSEG, HSEG)
                except Exception as e:
                    _log(world, "plan_seg error: %r" % (e,))
                world._smpc_nom = np.vstack([world._smpc_nom[1:], world._smpc_nom[-1:]])
        else:
            world._smpc_nom *= 0.5             # decay residual -> hand back to the reactive base
        _apply(world)
        if c % 250 == 1 or (engage and c % 25 == 1):
            try:
                q = world.solver.mjw_data.qpos.numpy().reshape(-1)
                _log(world, "SEG t=%d eng=%d esc=%.2f base=(%.3f,%.3f,%.3f) |res0|=%.3f max=%.3f J=%.1f"
                     % (c, int(engage), esc, q[0], q[1], q[2],
                        float(np.linalg.norm(world._smpc_nom[0])),
                        float(np.abs(world._smpc_nom).max()), getattr(world, "_smpc_lastJ", -1.0)))
            except Exception:
                pass
        return
    # ----- CONSTANT-residual mode (NSEG=1): capture-point FSM + balance residual -----
    if getattr(world, "_smpc_nom", None) is None or np.ndim(world._smpc_nom) != 1:
        world._smpc_nom = np.zeros(world._smpc_n)
    if world._smpc_step_ok and _f("SMPC_STEP_EN", 1.0) > 0:
        try:
            _step_fsm(world)
        except Exception as e:
            world._smpc_swing = None
            _log(world, "step fsm error: %r" % (e,))
    if (c - warm) % every == 0:
        try:
            _plan(world, K, H)
        except Exception as e:
            _log(world, "plan error: %r" % (e,))
    _apply(world)
    if c % 250 == 1:
        try:
            tp = world.control.joint_target_q.numpy()
            q = world.solver.mjw_data.qpos.numpy().reshape(-1)
            _log(world, "t=%d base=(%.3f,%.3f,%.3f) |res|=%.3f maxres=%.3f"
                 % (c, q[0], q[1], q[2], float(np.linalg.norm(world._smpc_nom)),
                    float(np.abs(world._smpc_nom).max())))
        except Exception:
            pass
