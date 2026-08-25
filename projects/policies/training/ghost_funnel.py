#!/usr/bin/env python3
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

"""ghost_funnel.py -- GATE 4: is the ghost PD-REALIZABLE? (the trainability gate)

WHY GATES 1-3 ARE NOT ENOUGH -- measured, 2026-07-09/10
The 7 cm stair-climb synth ghost passes all three static gates (closure 0.0 mm, COM in hull, FWP
feasible at 0.55 of torque) and yet SEVEN training configurations produced zero climbing, while the
3 cm ghost -- same method, same gates -- climbed on the first try. The static gates ask "does a
contact force distribution EXIST"; they never ask "does the CLOSED-LOOP system, tracked by the same
PD the deploy uses, stay NEAR the reference under small perturbations". That is a funnel question,
and it is answerable by pure simulation, with no RL in the loop:

    roll the real model forward with the deploy-grade joint PD tracking the ghost's clock,
    from the ghost's own state at phase b + a small perturbation, and measure how long the
    base stays inside a tube around the reference. Repeat over phases and samples.

The result is a per-phase SURVIVAL PROFILE -- the funnel. A phase where the funnel collapses is a
place the reference demands closed-loop behaviour that plain tracking cannot supply (a discrete
step-up, an aggressive transfer); that is exactly where RL training stalls, because early training
IS approximately PD tracking plus noise.

WHAT THIS IS AND IS NOT
- The PD here reproduces the deploy plant: tau = KP*(q_ref - q) - KD*qd, clipped to URDF effort
  limits, at 4 x 0.004 s substeps per 16 ms tick (the trainer's DT-parity numbers; PLANT-ECHO
  kp=200 kv=30). It is OPEN-LOOP in the base: no policy, no crane by default (--crane adds the
  training harness's lateral catch + attitude PD to measure the funnel the TRAINER sees).
- Survival is judged in the same coordinates the trainer kills in: base z within --tube of the
  phase-advanced reference height, base xy within --leash of the reference, and not fallen.
- This is a NECESSARY condition for trainability, not sufficient: a wide funnel can still hide a
  reward-hacking optimum. But a collapsed funnel is a hard prediction of a stalled tracker, and it
  says WHICH BINS to fix (retime with ghost_topp, replan the transfer, or curriculum the terrain).

Usage:
  python ghost_funnel.py <lut.json> [--samples 8] [--bin-stride 25] [--horizon 400]
                         [--kp 200 --kd 30] [--mu 2.0] [--tube 0.15] [--leash 0.5]
                         [--sig-pos 0.01 --sig-vel 0.05] [--crane] [--json out.json] [--png out.png]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco as mj

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ghost_contacts as GC

RT = os.environ.get("OMNISIM_HOME") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
DEF_MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")
DEF_URDF = os.path.join(RT, "projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf")
SUB_DT, N_SUB = 0.004, 4                       # the trainer's DT-parity plant: 4 x 0.004 = 0.016 s/tick


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def build_model(mjcf, terr, mu):
    """The robot model + the ghost's own terrain, via MjSpec. Friction matches the trainer
    (OMNISIM_NEWTON_GROUND_MU=2.0); stair boxes copy the ground plane's collision mask so the feet
    (contype=2) hit them exactly like the floor."""
    # MjSpec gates on the .xml extension; our file is .mjcf (pure primitives, no mesh assets)
    spec = mj.MjSpec.from_string(open(mjcf).read())
    if terr and terr.get("type") == "stairs":
        r, g = float(terr["riser"]), float(terr["going"])
        x0, n = float(terr.get("x0", 0.0)), int(terr["n"])
        wb = spec.worldbody
        for k in range(1, n + 1):
            gm = wb.add_geom()
            gm.name = "tread_%d" % k
            gm.type = mj.mjtGeom.mjGEOM_BOX
            gm.size = [g / 2, 2.0, k * r / 2]
            gm.pos = [x0 + (k - 0.5) * g, 0.0, k * r / 2]
            gm.conaffinity = 2; gm.contype = 0
        gm = wb.add_geom()
        gm.name = "landing"
        gm.type = mj.mjtGeom.mjGEOM_BOX
        gm.size = [0.75, 2.0, n * r / 2]
        gm.pos = [x0 + n * g + 0.75, 0.0, n * r / 2]
        gm.conaffinity = 2; gm.contype = 0
    m = spec.compile()
    m.opt.timestep = SUB_DT
    # ⛔ IMPLICITFAST, not the default Euler. The deploy plant (mujoco_warp) integrates actuator
    # velocity feedback implicitly; under plain Euler the kd=30 velocity servo on the low-inertia
    # knee/ankle chain rings a +/-139 N*m bang-bang limit cycle at dt=0.004 -- measured: the robot
    # shook itself airborne from a plain stand in 6 ticks, on every ghost equally.
    m.opt.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST
    m.geom_friction[:, 0] = mu
    # Mirror the deploy plant (g1_physics.json, the single source of truth):
    #   self_collision = false -- raw MuJoCo defaults collide non-adjacent robot bodies with each
    #   other; a crouch spawn interpenetrates the thighs and the repulsion impulse rails the knee at
    #   its clamp within 2 ticks and launches the robot (measured: every ghost "fell" in ~20 ticks).
    #   Robot geoms get contype=2/conaffinity=0 (hit the ground's conaffinity=2, never each other).
    #   NOTE: do NOT mirror the engine's contact_ke=2500 via direct-form solref -- raw-MuJoCo units
    #   differ (2500 N/m sinks the feet ~3 cm into mush and the soles pivot in it); MuJoCo's default
    #   impedance contact is the faithful stand-in here.
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0:
            m.geom_contype[g] = 2
            m.geom_conaffinity[g] = 0
    return m


def _urdf_eff(urdf):
    import xml.etree.ElementTree as ET
    eff = {}
    for j in ET.parse(urdf).getroot().iter("joint"):
        l = j.find("limit")
        if l is not None and l.get("effort"):
            eff[j.get("name")] = float(l.get("effort"))
    return eff


def run(lut_path, mjcf=None, urdf=None, samples=8, bin_stride=25, horizon=400,
        kp=200.0, kd=30.0, mu=2.0, tube=0.15, leash=0.5,
        sig_pos=0.01, sig_vel=0.05, crane=False, seed=0, ff=True, ff_stride=10):
    gd = json.load(open(lut_path))
    NB = int(gd["nb"]); cyc = float(gd.get("cycle_s", 1.0 / gd["freq"])); dt_bin = cyc / NB
    if abs(dt_bin - 0.016) > 1e-6:
        print("NOTE: lut bin %.4f s != 0.016 tick; clock scaled accordingly" % dt_bin)
    wb = np.asarray(gd["wb_lut"], float); WBJ = gd["wb_joints"]
    seq = bool(gd.get("seq", False)); hold_end = bool(gd.get("hold_end", False))
    if "root_lut" in gd:
        root = np.asarray(gd["root_lut"], float)
    else:                                       # cyclic lut without a root: glide at vx (walk)
        vx = float(gd.get("vx", 0.0)); z0 = float(gd.get("ghost_z", 0.72))
        tt = np.arange(NB) * dt_bin
        root = np.stack([vx * tt, np.zeros(NB), np.full(NB, z0), np.zeros(NB)], axis=1)
    att = np.asarray(gd["att_lut"], float) if "att_lut" in gd else np.zeros((NB, 2))
    terr = GC.make_terrain(gd.get("terrain"))

    m = build_model(mjcf or DEF_MJCF, gd.get("terrain"), mu)
    d = mj.MjData(m)
    qadr = np.array([m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ])
    dadr = np.array([m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WBJ])
    eff = _urdf_eff(urdf or DEF_URDF)
    tlim = np.array([eff.get(n, 100.0) for n in WBJ])

    # ⛔ THE PLANT IS THE MODEL'S OWN SERVO PAIR, retuned to the trainer's gains -- do NOT layer a
    # qfrc PD on top: the built-in position servos keep acting on ctrl=0, dragging every joint to
    # q=0 with kp=100 against the applied torques (measured: everything "fell" in 4 ticks).
    act_pos = np.full(len(WBJ), -1, int); act_vel = np.full(len(WBJ), -1, int)
    for a_ in range(m.nu):
        nm = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, a_) or ""
        for k, jn in enumerate(WBJ):
            if nm == jn + "_pos":
                act_pos[k] = a_
                m.actuator_gainprm[a_, 0] = kp; m.actuator_biasprm[a_, 1] = -kp
            elif nm == jn + "_vel":
                act_vel[k] = a_
                m.actuator_gainprm[a_, 0] = kd; m.actuator_biasprm[a_, 2] = -kd
        if nm.endswith(("_pos", "_vel")):
            jname = nm[:-4]
            m.actuator_forcelimited[a_] = 1
            m.actuator_forcerange[a_] = [-eff.get(jname, 100.0), eff.get(jname, 100.0)]
    if (act_pos < 0).any():
        raise SystemExit("funnel: no _pos actuator for %s" % [WBJ[i] for i in np.where(act_pos < 0)[0]])

    # ghost joint velocities: finite difference on the lut (wrapped when cyclic)
    wbv = np.zeros_like(wb)
    for i in range(NB):
        ip = (i + 1) % NB if not seq else min(i + 1, NB - 1)
        im = (i - 1) % NB if not seq else max(i - 1, 0)
        h = ((ip - im) % NB or 1) * dt_bin if not seq else max(ip - im, 1) * dt_bin
        wbv[i] = (wb[ip] - wb[im]) / h

    def ref(b):
        b = min(b, NB - 1) if (seq and hold_end) else (b % NB)
        return b

    def set_state(b, rng):
        d.qpos[:] = 0; d.qvel[:] = 0
        d.qpos[0:3] = root[b, 0:3]; d.qpos[2] += 0.02          # the spawn-clearance lesson
        d.qpos[3:7] = _quat(float(att[b, 0]), float(att[b, 1]), float(root[b, 3]))
        d.qpos[qadr] = wb[b]
        d.qvel[dadr] = wbv[b]
        if b + 1 < NB:                                          # base velocity from the root path
            d.qvel[0:3] = (root[min(b + 1, NB - 1), 0:3] - root[b, 0:3]) / dt_bin
        if rng is not None:
            d.qpos[0:2] += rng.normal(0, sig_pos, 2)
            d.qpos[qadr] += rng.normal(0, sig_pos, len(qadr))
            d.qvel[0:3] += rng.normal(0, sig_vel, 3)
        mj.mj_forward(m, d)

    def crane_wrench(b):
        """The trainer's lateral-only catch + attitude PD (HARNESS_FY=400, KP=600, KD=60, KZ=0),
        applied at the pelvis. lambda=0.9 to mirror the puppet."""
        lam = 0.9
        fy = -400.0 * (d.qpos[1] - root[ref(b), 1]) - 40.0 * d.qvel[1]
        R = np.zeros(9); mj.mju_quat2Mat(R, d.qpos[3:7]); R = R.reshape(3, 3)
        rollp = math.atan2(R[2, 1], R[2, 2]); pitchp = math.asin(max(-1, min(1, -R[2, 0])))
        tx = -600.0 * (rollp - float(att[ref(b), 0])) - 60.0 * d.qvel[3]
        ty = -600.0 * (pitchp - float(att[ref(b), 1])) - 60.0 * d.qvel[4]
        d.qfrc_applied[1] += lam * fy
        d.qfrc_applied[3] += lam * tx; d.qfrc_applied[4] += lam * ty

    # ── REFERENCE FEEDFORWARD (the definition of "realizable") ─────────────────────────────────
    # Pure PD on the raw reference measures GRAVITY SAG, not trackability: joints sag by tau/kp
    # (~0.1 rad at kp=200 in a crouch), the pelvis drifts off the support and even a plain SQUAT
    # "fails" in ~1.2 s. A trained policy's first lesson is precisely the missing feedforward. So
    # the probe tracks with tau_ff + PD, where tau_ff is the quasi-static actuator torque under the
    # FWP's own best contact-force split -- the torque gate 3 already proved exists. Funnel width
    # then measures what is left for FEEDBACK to do, which is the trainability question.
    tau_ff = np.zeros((NB, len(WBJ)))
    if ff:
        spec_c = GC.load_spec("feet")
        for c in spec_c["contacts"]:
            c["bid"] = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, c["body"])
        jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
        m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT)
        ffbins = list(range(0, NB, max(1, ff_stride)))
        for b in ffbins:
            set_state(b, None)
            d.qacc[:] = 0.0
            mj.mj_inverse(m, d)
            qfi = d.qfrc_inverse.copy()
            active = []
            for c in spec_c["contacts"]:
                R = d.xmat[c["bid"]].reshape(3, 3)
                cs = GC.corners(d.xpos[c["bid"]], R, c["half"], c["patch"])
                surf = [GC.surface_z(terr, float(q[0]), float(q[1])) for q in cs]
                on = [cs[j] for j in range(len(cs)) if float(cs[j][2]) - surf[j] <= 0.03]
                if on:
                    active.append(dict(name=c["name"], bid=c["bid"], corners=on,
                                       normals=[(0, 0, 1)] * len(on)))
            if active:
                cols, pts, owner, cp, cbid = GC.build_columns(active, 1.0)
                Jc = []
                for q_, bid_ in zip(cp, cbid):
                    mj.mj_jac(m, d, jacp, jacr, q_, bid_)
                    Jc.append(jacp.copy())
                Gf = np.stack([Jc[owner[j]].T @ cols[j] for j in range(len(cols))], axis=1)
                ok, _t, f = GC.fwp_check_G(Gf[0:6, :], qfi[0:6], Gf[dadr, :], qfi[dadr],
                                           list(tlim), owner, cols, len(cp))
                if ok and f is not None:
                    tc = np.zeros(m.nv)
                    for ci, q_ in enumerate(cp):
                        mj.mj_jac(m, d, jacp, jacr, q_, cbid[ci])
                        tc += jacp.T @ f[ci]
                    tau_ff[b] = np.clip((qfi - tc)[dadr], -tlim, tlim)
                else:
                    tau_ff[b] = np.clip(qfi[dadr], -tlim, tlim)
            else:
                tau_ff[b] = np.clip(qfi[dadr], -tlim, tlim)
        m.opt.disableflags &= ~int(mj.mjtDisableBit.mjDSBL_CONTACT)
        for k in range(len(WBJ)):                       # linear interp between strided bins
            tau_ff[:, k] = np.interp(np.arange(NB), ffbins, tau_ff[ffbins, k])

    rng0 = np.random.default_rng(seed)
    bins = list(range(0, NB, bin_stride))
    profile = []
    for b0 in bins:
        surv = 0; ticks_sum = 0
        for s in range(samples):
            set_state(b0, rng0 if s else None)                 # sample 0 = unperturbed
            alive_ticks = 0
            for t in range(horizon):
                b = ref(b0 + t)
                d.ctrl[act_pos] = wb[b]
                d.ctrl[act_vel] = 0.0
                for _ in range(N_SUB):
                    d.qfrc_applied[:] = 0.0
                    d.qfrc_applied[dadr] = tau_ff[b]
                    if crane:
                        crane_wrench(b)
                    mj.mj_step(m, d)
                if not np.all(np.isfinite(d.qpos)):
                    break
                bz_ref = root[ref(b0 + t), 2]
                if (abs(d.qpos[2] - bz_ref) > tube or
                        np.hypot(d.qpos[0] - root[ref(b0 + t), 0], d.qpos[1] - root[ref(b0 + t), 1]) > leash):
                    break
                alive_ticks += 1
            ticks_sum += alive_ticks
            if alive_ticks >= min(horizon, (NB - b0) if (seq and not hold_end) else horizon) - 1:
                surv += 1
        profile.append(dict(bin=b0, t=b0 * dt_bin, surv=surv / samples,
                            mean_ticks=ticks_sum / samples))
    score = float(np.mean([p["surv"] for p in profile]))
    worst = sorted(profile, key=lambda p: (p["surv"], p["mean_ticks"]))[:5]
    return dict(lut=os.path.basename(lut_path), nb=NB, crane=crane, ff=ff, kp=kp, kd=kd, mu=mu,
                tube=tube, leash=leash, samples=samples, horizon=horizon,
                funnel_score=score, profile=profile, worst=worst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut")
    ap.add_argument("--mjcf", default=None); ap.add_argument("--urdf", default=None)
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--bin-stride", type=int, default=25)
    ap.add_argument("--horizon", type=int, default=400)
    ap.add_argument("--kp", type=float, default=200.0); ap.add_argument("--kd", type=float, default=30.0)
    ap.add_argument("--mu", type=float, default=2.0)
    ap.add_argument("--tube", type=float, default=0.15); ap.add_argument("--leash", type=float, default=0.5)
    ap.add_argument("--sig-pos", type=float, default=0.01); ap.add_argument("--sig-vel", type=float, default=0.05)
    ap.add_argument("--crane", action="store_true")
    ap.add_argument("--no-ff", action="store_true", help="disable the reference feedforward (raw PD)")
    ap.add_argument("--ff-stride", type=int, default=10)
    ap.add_argument("--json", default=None); ap.add_argument("--png", default=None)
    a = ap.parse_args()
    r = run(a.lut, a.mjcf, a.urdf, a.samples, a.bin_stride, a.horizon, a.kp, a.kd, a.mu,
            a.tube, a.leash, a.sig_pos, a.sig_vel, a.crane, ff=not a.no_ff, ff_stride=a.ff_stride)
    print("GHOST FUNNEL (gate 4): %s   crane=%s ff=%s kp=%.0f kd=%.0f mu=%.1f tube=%.2f" %
          (r["lut"], r["crane"], r["ff"], r["kp"], r["kd"], r["mu"], r["tube"]))
    print("  FUNNEL SCORE %.3f  (mean survival over %d start phases x %d samples, horizon %d ticks)"
          % (r["funnel_score"], len(r["profile"]), r["samples"], r["horizon"]))
    print("  worst phases:")
    for p in r["worst"]:
        print("    bin %5d  t=%6.2fs  surv %.2f  mean alive %5.0f ticks" %
              (p["bin"], p["t"], p["surv"], p["mean_ticks"]))
    if a.json:
        json.dump(r, open(a.json, "w"), indent=1)
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [p["t"] for p in r["profile"]]; s = [p["surv"] for p in r["profile"]]
        plt.figure(figsize=(10, 3))
        plt.fill_between(t, s, alpha=0.4); plt.plot(t, s)
        plt.xlabel("start phase (s)"); plt.ylabel("survival"); plt.ylim(-0.02, 1.02)
        plt.title("PD-realizability funnel: %s (score %.2f)" % (r["lut"], r["funnel_score"]))
        plt.tight_layout(); plt.savefig(a.png, dpi=110)
        print("  wrote %s" % a.png)


if __name__ == "__main__":
    main()
