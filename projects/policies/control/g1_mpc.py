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

"""g1_mpc.py -- a DETERMINISTIC model-predictive "brain" for the G1 walker.

This is the controller that finally walks the physics G1 FORWARD and far. It is
deterministic (seeded sampling -> fully reproducible, no learned weights) and it
fixes the fatal flaw of the reflex/gain brain (projects/policies/control/g1_brain.py):
that one only REACTS proportionally to the current tilt, so it tops out at ~2.3 s
before an inverted-pendulum divergence it can't see coming. This one PREDICTS.

APPROACH -- sampling-based MPC (MPPI) with MuJoCo as the predictor
-----------------------------------------------------------------
Every control tick (62.5 Hz):
  1. snapshot the true state;
  2. sample K balance-residual vectors around a warm-started nominal;
  3. roll each forward H control-steps in a CLONE of the real plant (the ghost
     gait feedforward + that residual) -- all K rollouts in ONE batched
     mujoco.rollout C call (threaded);
  4. score each trajectory: stay upright + track the ghost's forward speed +
     no lateral/heading DRIFT; a fall is heavily penalised by how EARLY it is;
  5. MPPI-average the samples by exp(-cost/lambda) -> new nominal; apply its
     first action, advance, re-plan (receding horizon).

The residual acts on 8 balance joints (hip pitch/roll + ankle pitch/roll, both
legs) ON TOP of the g1_human_gait reference (the same "ghost" the reflex brain
and the RL policy use). Because every candidate is judged by a TRUE physics
rollout, MPC only commits to corrections that actually keep the robot up -- the
thing analytic capture-point placement could NOT do on the compliant kp=100 legs
(it kept over-braking / rolling). Prediction + re-planning is the whole trick.

WHY THE KEY KNOBS ARE WHAT THEY ARE (measured in the offline harness)
--------------------------------------------------------------------
- H (horizon) MUST exceed the gait step period (~0.77 s, i.e. H>=45): a one-step
  lookahead balances in place but can't plan a stable FORWARD gait (H=45 walks
  forward ~5 s, H=55 sustains). The divergence timescale sqrt(z/g)~0.28 s is why
  a too-short horizon is blind to the multi-step runaway.
- iters>=2 (iterative MPPI refinement) cuts the sampling variance that otherwise
  makes survival seed-dependent.
- w_vy / w_yaw (penalise lateral + heading drift) were THE fix for the "falls at
  ~18 s regardless of speed" wall: with only forward-speed in the cost, sideways/
  heading drift is unpenalised, accumulates, and tips the robot. Penalising it
  turned an 18 s topple into a self-correcting limit cycle (recovers from a 0.38
  rad roll wobble and keeps walking).

RESULT (offline harness, deploy-matched plant, default config below):
  reflex/gain brain : -0.53..+0.72 m, falls ~2.3 s   (the old ceiling)
  THIS (MPC)        : sustained FORWARD walking at ~0.18 m/s, recovers from
                      disturbances; +6.2 m in 35 s and still upright.

Runtime: needs `mujoco` (the planner rolls the model out). It is an offline /
research-harness controller; an OmniSim-deploy port would carry a MuJoCo model
alongside Newton purely for the planning rollouts.

  python projects/policies/control/g1_mpc.py 40         # 40 s demo, trace on
  python projects/policies/control/g1_mpc.py 40 --dist 50   # walk until +50 m
"""
from __future__ import annotations
import sys, os, math

if __name__ == "__main__" or __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))))  # repo root

import numpy as np
import mujoco
from mujoco import rollout

import projects.policies.control.g1_brain_sim as H
from projects.policies.control.gait import g1_human_gait as ghg
from projects.policies.control.g1_brain import LIM_LO, LIM_HI

# 8 balance joints the residual acts on (indices into LEGS_JOINTS).
RES_J = np.array([0, 6, 1, 7, 4, 10, 5, 11])   # Lhp Rhp Lhr Rhr Lap Rap Lar Rar
NR = len(RES_J)
FULL = mujoco.mjtState.mjSTATE_FULLPHYSICS

# Proven default config (offline harness, sustained forward walking).
CFG = dict(
    vx=0.4, freq=1.3, x0=-0.03,          # ghost gait feedforward
    K=100, H=55, iters=2, lam=0.3, nthread=8, tvary=False,
    sigma=np.array([0.10, 0.10, 0.08, 0.08, 0.10, 0.10, 0.06, 0.06]),
    res_max=0.5,
    # cost weights
    w_tilt=4.0, w_roll=5.0,              # uprightness (roll = the weak axis)
    w_vx=0.2, vx_target=0.35, w_excess=8.0, vx_cap=0.55,   # speed track + cap
    w_prog=1.5,                          # forward-progress reward (capped)
    w_prate=0.6, w_vy=4.0, w_yaw=4.0,    # damp divergence + lateral/heading drift
    w_bz=8.0, z_ref=0.74, w_res=0.3, w_term=6.0,
)


def _quat_rpy(q):                        # q: (..., 4) wxyz
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


class G1MPC:
    """Sampling-MPC controller. Build once, call plan() each control tick."""

    def __init__(self, cfg=None):
        self.cfg = dict(CFG, **(cfg or {}))
        c = self.cfg
        self.plant = H.Plant()
        self.m = self.plant.m
        self.gp = ghg.GaitParams(style="ik", vx=c["vx"], freq=c["freq"], x0=c["x0"])
        self.om = 2.0 * math.pi * self.gp.freq
        self.nq = self.m.nq
        self.nstate = mujoco.mj_stateSize(self.m, FULL)
        self.lpa = self.plant.leg_pos_act
        self.apa = self.plant.arm_pos_act
        self.res_act = self.plant.leg_pos_act[RES_J]
        self.SUB = H.SUBSTEPS
        self.nthread = int(c["nthread"])
        self.datas = [mujoco.MjData(self.m) for _ in range(self.nthread)]
        self.roll = rollout.Rollout(nthread=self.nthread)
        self.nominal = np.zeros((2, NR))     # (r0, r1) warm-started knots

    def close(self):
        self.roll.close()

    def standing_pose(self):
        return ghg.standing_pose(self.gp).astype(np.float64)

    def _base_ctrl(self, phase, t, Hh):
        """Gait control (Hh, nu) over the horizon -- same for every sample."""
        base = np.zeros((Hh, self.m.nu))
        ph, tt = phase, t
        for h in range(Hh):
            legs, _, _ = ghg.targets_np(ph, self.gp, t_since_start=max(0.0, tt))
            base[h, self.lpa] = np.clip(legs.astype(np.float64), LIM_LO, LIM_HI)
            base[h, self.apa] = H.ARM_NOMINAL
            ph = (ph + self.om * H.CTRL_DT) % (2 * math.pi); tt += H.CTRL_DT
        return base

    def plan(self, phase, t, rng):
        """Return the leg-residual r0 to apply this tick (8-vector on RES_J)."""
        c = self.cfg; K, Hh, nu = c["K"], c["H"], self.m.nu
        base = self._base_ctrl(phase, t, Hh)
        a = np.linspace(0.0, 1.0, Hh)[None, :, None]
        s0 = np.zeros(self.nstate); mujoco.mj_getState(self.m, self.plant.d, s0, FULL)
        s0b = np.broadcast_to(s0, (K, self.nstate)).copy()
        for _ in range(c["iters"]):
            noise = rng.normal(0, 1, size=(K, 2, NR)) * c["sigma"]
            knots = np.clip(self.nominal[None] + noise, -c["res_max"], c["res_max"])
            if not c["tvary"]:
                knots[:, 1] = knots[:, 0]
            knots[0] = self.nominal                         # incumbent
            resid = knots[:, 0:1, :] * (1 - a) + knots[:, 1:2, :] * a
            ctrl = np.broadcast_to(base, (K, Hh, nu)).copy()
            ctrl[:, :, self.res_act] += resid
            ctrl[:, :, self.lpa] = np.clip(ctrl[:, :, self.lpa], LIM_LO, LIM_HI)
            control = np.repeat(ctrl, self.SUB, axis=1)
            st, _ = self.roll.rollout(self.m, self.datas, s0b, control)
            st = st[:, self.SUB - 1::self.SUB, :]           # per control-step end
            roll, pitch, yaw = _quat_rpy(st[..., 4:8])
            self._mppi(knots, st[..., 3], roll, pitch, yaw,
                       st[..., 1 + self.nq], st[..., 2 + self.nq],
                       st[..., 5 + self.nq])
        return self.nominal[0]

    def _mppi(self, knots, bz, roll, pitch, yaw, vx, vy, prate):
        c = self.cfg; K, Hh = c["K"], c["H"]
        Js = (c["w_tilt"] * pitch * pitch + c["w_roll"] * roll * roll
              + c["w_vx"] * (vx - c["vx_target"]) ** 2
              + c["w_excess"] * np.maximum(0, np.abs(vx) - c["vx_cap"]) ** 2
              - c["w_prog"] * np.clip(vx, 0, c["vx_cap"])
              + c["w_prate"] * prate * prate
              + c["w_vy"] * vy * vy + c["w_yaw"] * yaw * yaw
              + c["w_bz"] * np.maximum(0, c["z_ref"] - bz) ** 2)
        fallen = (bz < 0.45) | (np.abs(roll) > 0.8) | (np.abs(pitch) > 0.8)
        firstfall = np.where(fallen.any(axis=1), fallen.argmax(axis=1), Hh)
        valid = np.arange(Hh)[None, :] < firstfall[:, None]
        J = (Js * valid).sum(axis=1) + 50.0 * (Hh - firstfall)
        ti = np.clip(firstfall - 1, 0, Hh - 1); ar = np.arange(K)
        J += c["w_term"] * (pitch[ar, ti] ** 2 + roll[ar, ti] ** 2
                            + 0.5 * (vx[ar, ti] - c["vx_target"]) ** 2)
        J += c["w_res"] * (knots[:, 0] ** 2 + knots[:, 1] ** 2).sum(axis=1)
        w = np.exp(-(J - J.min()) / c["lam"]); w /= w.sum() + 1e-9
        self.nominal = np.clip((w[:, None, None] * knots).sum(axis=0),
                               -c["res_max"], c["res_max"])


def run(seconds=40.0, target_dist=None, cfg=None, settle=0.4, seed=0, trace=True):
    import time
    mpc = G1MPC(cfg)
    nom = mpc.standing_pose()
    mpc.plant.reset(nom, H.ARM_NOMINAL)
    for _ in range(int(round(settle / H.CTRL_DT))):
        mpc.plant.set_ctrl(nom, H.ARM_NOMINAL); mpc.plant.substep()
    rng = np.random.default_rng(seed)
    phase = ghg.DS_PHASE; x0 = mpc.plant.base_x
    t0 = time.time(); nextlog = 2.0; k = 0; fell = None; t = 0.0
    while True:
        t = k * H.CTRL_DT
        r0 = mpc.plan(phase, t, rng)
        legs, arms, _ = ghg.targets_np(phase, mpc.gp, t_since_start=max(0.0, t))
        tgt = np.clip(legs.astype(np.float64), LIM_LO, LIM_HI); tgt[RES_J] += r0
        np.clip(tgt, LIM_LO, LIM_HI, out=tgt)
        mpc.plant.set_ctrl(tgt, arms.astype(np.float64)); mpc.plant.substep()
        phase = (phase + mpc.om * H.CTRL_DT) % (2 * math.pi)
        qw, qx, qy, qz = mpc.plant.d.qpos[3:7]
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1, min(1, 2 * (qw * qy - qz * qx))))
        bz = float(mpc.plant.d.qpos[2]); dist = mpc.plant.base_x - x0
        if trace and dist >= nextlog:
            print(f"  t={t:6.1f}s dist={dist:6.2f}m bz={bz:.3f} roll={roll:+.2f} "
                  f"pitch={pitch:+.2f} vx={mpc.plant.d.qvel[0]:+.2f} "
                  f"wall={time.time()-t0:.0f}s", flush=True)
            nextlog += 2.0
        if bz < 0.45 or abs(roll) > 0.8 or abs(pitch) > 0.8:
            fell = t; break
        if target_dist is not None and dist >= target_dist:
            print(f"*** REACHED {target_dist} m at t={t:.1f}s, wall {time.time()-t0:.0f}s ***", flush=True)
            break
        if target_dist is None and t >= seconds:
            break
        k += 1
    dist = mpc.plant.base_x - x0
    mpc.close()
    print(f"G1MPC: dist={dist:+.2f}m surv={t:.2f}s "
          f"{'FELL' if fell else 'OK'} avg_vx={dist/max(t,1e-6):.2f}", flush=True)
    return dist, t, fell


if __name__ == "__main__":
    sec = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    td = None
    if "--dist" in sys.argv:
        td = float(sys.argv[sys.argv.index("--dist") + 1])
    run(seconds=sec, target_dist=td)
