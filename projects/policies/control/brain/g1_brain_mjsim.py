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

"""Offline plain-MuJoCo harness for the deterministic G1 brain.

Runs the EXACT same control code (projects/policies/control/brain/g1_brain.G1Brain) against
the deploy-matched MJCF (g1_full_kp100.mjcf.xml, full body incl. arms) in plain
MuJoCo -- no OmniSim, no GPU, no world load. A 10 s rollout takes milliseconds,
so balance gains can be swept in seconds, then the winner validated in OmniSim
via run_g1_brain.ps1. The journey used plain MuJoCo for exactly these stand/
fall questions ("verified statically stable 15 s in plain mujoco").

The MJCF ships paired position(kp=100)/velocity(kd=5) actuators per joint; this
harness OVERRIDES their gains to (KE, KD) so the joint servo matches the bridge's
OMNISIM_NEWTON_TARGET_KE/KD (default 400/60 -- the stand-proven stiff PD).

Usage:
    python -m projects.policies.control.brain.g1_brain_mjsim                 # one run (env-configured)
    python -m projects.policies.control.brain.g1_brain_mjsim sweep           # built-in gain grid
    G1_BRAIN_STAND_ONLY=1 python -m projects.policies.control.brain.g1_brain_mjsim   # stand test

Gains read from G1_BRAIN_* env (BrainConfig.from_env) -- identical interface to
the deploy controller, so a config that stands here is launched unchanged.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import mujoco

from projects.policies.control.brain.g1_brain import G1Brain, BrainState, BrainConfig, NJ

_REPO = Path(__file__).resolve().parents[3]

MJCF = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_full_kp100.mjcf.xml")

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
ARM_NOMINAL = np.array([0.0, 0.20, 0.0, 0.0, 0.0,
                        0.0, -0.20, 0.0, 0.0, 0.0], dtype=np.float64)


def _id(m, typ, name):
    return mujoco.mj_name2id(m, typ, name)


def _set_pd_gains(m, ke, kd):
    """Override every *_pos actuator to kp=ke and *_vel to kd, replicating
    force = ke*(target - q) - kd*qd (the bridge's position servo)."""
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name is None:
            continue
        if name.endswith("_pos"):
            m.actuator_gainprm[i][0] = ke
            m.actuator_biasprm[i][1] = -ke
        elif name.endswith("_vel"):
            m.actuator_gainprm[i][0] = kd
            m.actuator_biasprm[i][2] = -kd


def _quat_to_rpy(q):
    """Match the deploy controller's rotation->rpy convention (so signs agree).
    q = (w, x, y, z); build row-major body->world R, then use the deploy formula."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    o = R  # row-major 3x3
    roll = math.atan2(o[7], o[8])
    pitch = math.asin(max(-1.0, min(1.0, -o[6])))
    yaw = math.atan2(o[3], o[0])
    return roll, pitch, yaw


class _Sim:
    def __init__(self, ke=400.0, kd=60.0):
        self.m = mujoco.MjModel.from_xml_path(MJCF)
        self.d = mujoco.MjData(self.m)
        _set_pd_gains(self.m, ke, kd)
        m = self.m
        # joint qpos/qvel addresses + paired actuator indices, by name
        self.jq, self.jv, self.act_pos, self.act_vel = {}, {}, {}, {}
        for jn in LEGS_JOINTS + ARM_JOINTS:
            jid = _id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self.jq[jn] = m.jnt_qposadr[jid]
            self.jv[jn] = m.jnt_dofadr[jid]
            self.act_pos[jn] = _id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + "_pos")
            self.act_vel[jn] = _id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + "_vel")
        self.root = _id(m, mujoco.mjtObj.mjOBJ_JOINT, "root_free")
        self.qadr0 = m.jnt_qposadr[self.root]   # base qpos start (0)
        self.vadr0 = m.jnt_dofadr[self.root]    # base qvel start (0)

    def seed(self, brain, base_z=0.90):
        """Seed the deep-squat stand, then lower the base until the feet just
        touch the ground (no free-fall, so no landing impact destabilises it)."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[self.qadr0 + 3:self.qadr0 + 7] = [1.0, 0.0, 0.0, 0.0]  # upright
        stand = brain.standing_pose()
        for k, jn in enumerate(LEGS_JOINTS):
            d.qpos[self.jq[jn]] = stand[k]
        for k, jn in enumerate(ARM_JOINTS):
            d.qpos[self.jq[jn]] = ARM_NOMINAL[k]
        # lower until first ground contact (mj_forward runs collision)
        z = base_z
        prev = z
        for _ in range(600):
            d.qpos[self.qadr0 + 2] = z
            mujoco.mj_forward(m, d)
            if d.ncon > 0:
                d.qpos[self.qadr0 + 2] = prev   # back off to ~zero penetration
                break
            prev = z
            z -= 0.002
        mujoco.mj_forward(m, d)

    def read_state(self, prev_roll, prev_pitch, ctrl_dt):
        m, d = self.m, self.d
        bx, by, bz = d.qpos[self.qadr0:self.qadr0 + 3]
        roll, pitch, yaw = _quat_to_rpy(d.qpos[self.qadr0 + 3:self.qadr0 + 7])
        lin = d.qvel[self.vadr0:self.vadr0 + 3].copy()           # world linear
        ang = d.qvel[self.vadr0 + 3:self.vadr0 + 6].copy()       # body angular
        roll_rate = (roll - prev_roll) / ctrl_dt
        pitch_rate = (pitch - prev_pitch) / ctrl_dt
        jq = np.array([d.qpos[self.jq[jn]] for jn in LEGS_JOINTS])
        jd = np.array([d.qvel[self.jv[jn]] for jn in LEGS_JOINTS])
        return dict(bx=bx, by=by, bz=bz, roll=roll, pitch=pitch, yaw=yaw,
                    lin=lin, ang=ang, roll_rate=roll_rate, pitch_rate=pitch_rate,
                    jq=jq, jd=jd)

    def apply(self, legs, arms):
        d = self.d
        for k, jn in enumerate(LEGS_JOINTS):
            d.ctrl[self.act_pos[jn]] = legs[k]
            d.ctrl[self.act_vel[jn]] = 0.0
        for k, jn in enumerate(ARM_JOINTS):
            d.ctrl[self.act_pos[jn]] = arms[k]
            d.ctrl[self.act_vel[jn]] = 0.0


def run(cfg: BrainConfig, duration=10.0, ctrl_dt=0.016, ke=400.0, kd=60.0,
        grace=0.8, trace=False, verbose=False):
    """One rollout. Returns dict(fall_t, dist, end_*). fall_t=None => survived."""
    sim = _Sim(ke, kd)
    brain = G1Brain(cfg)
    sim.seed(brain)
    n_sub = max(1, int(round(ctrl_dt / sim.m.opt.timestep)))
    prev_roll = prev_pitch = 0.0
    t = 0.0
    fall_t = None
    rows = []
    while t < duration:
        st = sim.read_state(prev_roll, prev_pitch, ctrl_dt)
        prev_roll, prev_pitch = st["roll"], st["pitch"]
        bs = BrainState(
            roll=st["roll"], pitch=st["pitch"], yaw=st["yaw"],
            roll_rate=st["roll_rate"], pitch_rate=st["pitch_rate"],
            yaw_rate=float(st["ang"][2]),
            vx=float(st["lin"][0]), vy=float(st["lin"][1]), vz=float(st["lin"][2]),
            bz=st["bz"], joint_q=st["jq"], joint_qd=st["jd"], t=t)
        legs = brain.step(bs, ctrl_dt)
        sim.apply(legs, brain.arms)
        for _ in range(n_sub):
            mujoco.mj_step(sim.m, sim.d)
        t += ctrl_dt
        upright = (st["bz"] > 0.45 and abs(st["roll"]) < 0.8 and abs(st["pitch"]) < 0.8)
        if (not upright) and t > grace and fall_t is None:
            fall_t = t
        if trace:
            rows.append((t, st["bx"], st["bz"], st["roll"], st["pitch"],
                         float(st["lin"][0]), float(st["lin"][1])))
        if verbose and abs(t / ctrl_dt % int(0.1 / ctrl_dt)) < 1e-9:
            print(f" t={t:5.2f} bx={st['bx']:+.2f} bz={st['bz']:.3f} "
                  f"roll={st['roll']:+.3f} pit={st['pitch']:+.3f} "
                  f"vx={st['lin'][0]:+.2f} vy={st['lin'][1]:+.2f}")
    fst = sim.read_state(prev_roll, prev_pitch, ctrl_dt)
    return dict(fall_t=fall_t, dist=fst["bx"], end_bz=fst["bz"],
                end_roll=fst["roll"], end_pitch=fst["pitch"], rows=rows)


def _summ(tag, r):
    f = "SURVIVED" if r["fall_t"] is None else f"fall@{r['fall_t']:.2f}s"
    print(f"{tag:28s} {f:14s} bx={r['dist']:+.2f} "
          f"end(bz={r['end_bz']:.3f} roll={r['end_roll']:+.3f} pit={r['end_pitch']:+.3f})")


def main():
    ke = float(os.environ.get("G1_BRAIN_KE", "400"))
    kd = float(os.environ.get("G1_BRAIN_KD", "60"))
    dur = float(os.environ.get("G1_BRAIN_DUR", "10"))
    if len(sys.argv) > 1 and sys.argv[1] == "sweep":
        # STAND sweep: the plain-mujoco failure is LATERAL (roll), so sweep the
        # stance width (passive lateral base) x lateral feedback gains.
        print(f"[sweep] stand-only, ke={ke} kd={kd}, dur={dur}s")
        base = BrainConfig.from_env()
        base.stand_only = True
        base.stance_width = 0.10
        base.kp_ap, base.kd_ap = -3.0, -0.5      # decent sagittal hold
        base.kp_post_p, base.kd_post_p = -1.0, -0.2
        # The lateral mode is fast (~0.27s); sweep the SIGN and DAMPING of the
        # roll feedback (ankle-roll CoP shift + hip-roll CoM shift).
        for kpr in (+2.0, -2.0):
            for kdr in (-0.4, -0.8, -1.5):
                for kar in (+4.0, -4.0):
                    c = BrainConfig(**{**base.__dict__})
                    c.kp_post_r, c.kd_post_r = kpr, kdr
                    c.kp_ar, c.kd_ar = kar, 0.2 * kar
                    r = run(c, duration=dur, ke=ke, kd=kd)
                    _summ(f"kp_post_r={kpr:+.0f} kd_post_r={kdr:+.1f} kp_ar={kar:+.0f}", r)
        return
    cfg = BrainConfig.from_env()
    r = run(cfg, duration=dur, ke=ke, kd=kd, verbose=True)
    print()
    _summ("RESULT", r)


if __name__ == "__main__":
    main()
