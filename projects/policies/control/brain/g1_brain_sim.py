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

"""Fast offline MuJoCo harness for the G1 deterministic brain.

The research instrument: runs projects/policies/control/brain/g1_brain.py against the
deploy-matched plant (g1_full_kp100.mjcf.xml, kp=100/kd=5, 16 ms control,
4x4 ms substeps -- identical to the trainer) in PLAIN MuJoCo. Seconds per run,
headless, scriptable -- so the brain's feedback gains can be tuned without ever
launching OmniSim. OmniSim is only for milestone validation.

Usage
-----
  python projects/policies/control/brain/g1_brain_sim.py                       # default brain, 20 s
  python projects/policies/control/brain/g1_brain_sim.py --controller openloop # zero-gain = ghost-in-physics baseline
  python projects/policies/control/brain/g1_brain_sim.py --seconds 30 --render # watch it (needs a display)
  python projects/policies/control/brain/g1_brain_sim.py --set kp_ar=-4 kd_ar=-0.5   # override BrainConfig gains
  python projects/policies/control/brain/g1_brain_sim.py --sweep kp_ar=-2,-3,-4,-5   # 1-D gain sweep
  python projects/policies/control/brain/g1_brain_sim.py --quiet --sweep kfp_y=0,0.05,0.1,0.15

Metrics (per run): distance walked (base x), survival time, fall time, mean |tilt|,
mean forward speed. FALL = base z < 0.45 or |roll| > 0.8 or |pitch| > 0.8.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, fields, asdict
from pathlib import Path

import numpy as np
import mujoco

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO))

from projects.policies.control.brain.g1_brain import (  # noqa: E402
    G1Brain, BrainConfig, BrainState, NJ,
)

MJCF = _REPO / "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml"

# Plant constants -- MUST match the trainer (gpu_mjwarp_g1_walk_trainer.py).
CTRL_DT = 0.016
PHYS_DT = 0.004
SUBSTEPS = int(round(CTRL_DT / PHYS_DT))   # 4
SPAWN_Z = 0.78

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


def _quat_to_rpy(qw, qx, qy, qz):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class Plant:
    """Wraps the MuJoCo model: name->index maps, init, ctrl write, state read."""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(MJCF))
        self.m.opt.timestep = PHYS_DT
        self.d = mujoco.MjData(self.m)
        m = self.m

        def aid(name):
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if i < 0:
                raise RuntimeError(f"actuator {name} not found")
            return i

        def jadr(name):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            return m.jnt_qposadr[jid], m.jnt_dofadr[jid]

        self.leg_pos_act = np.array([aid(j + "_pos") for j in LEGS_JOINTS])
        self.leg_vel_act = np.array([aid(j + "_vel") for j in LEGS_JOINTS])
        self.arm_pos_act = np.array([aid(j + "_pos") for j in ARM_JOINTS])
        self.arm_vel_act = np.array([aid(j + "_vel") for j in ARM_JOINTS])
        self.leg_qadr = np.array([jadr(j)[0] for j in LEGS_JOINTS])
        self.leg_dadr = np.array([jadr(j)[1] for j in LEGS_JOINTS])
        self.arm_qadr = np.array([jadr(j)[0] for j in ARM_JOINTS])

    def reset(self, leg_nominal, arm_nominal):
        d, m = self.d, self.m
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        d.qpos[self.leg_qadr] = leg_nominal
        d.qpos[self.arm_qadr] = arm_nominal
        mujoco.mj_forward(m, d)

    def set_ctrl(self, leg_targets, arm_targets):
        d = self.d
        d.ctrl[self.leg_pos_act] = leg_targets
        d.ctrl[self.leg_vel_act] = 0.0
        d.ctrl[self.arm_pos_act] = arm_targets
        d.ctrl[self.arm_vel_act] = 0.0

    def substep(self, n=SUBSTEPS):
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)

    def read_state(self, t) -> BrainState:
        d = self.d
        qw, qx, qy, qz = d.qpos[3], d.qpos[4], d.qpos[5], d.qpos[6]
        roll, pitch, yaw = _quat_to_rpy(qw, qx, qy, qz)
        return BrainState(
            roll=roll, pitch=pitch, yaw=yaw,
            roll_rate=float(d.qvel[3]), pitch_rate=float(d.qvel[4]),
            yaw_rate=float(d.qvel[5]),
            vx=float(d.qvel[0]), vy=float(d.qvel[1]), vz=float(d.qvel[2]),
            bz=float(d.qpos[2]),
            joint_q=d.qpos[self.leg_qadr].copy(),
            joint_qd=d.qvel[self.leg_dadr].copy(),
            t=t,
        )

    @property
    def base_x(self):
        return float(self.d.qpos[0])


@dataclass
class RunResult:
    distance: float
    survived_s: float
    fell_at: float | None
    mean_tilt: float
    mean_vx: float
    final_x: float

    def line(self):
        fa = f"FELL@{self.fell_at:.2f}s" if self.fell_at is not None else "SURVIVED"
        return (f"dist={self.distance:+.2f}m  surv={self.survived_s:.1f}s  {fa}  "
                f"mean|tilt|={self.mean_tilt:.3f}  mean_vx={self.mean_vx:+.2f}m/s")


def run(cfg: BrainConfig, seconds=20.0, settle_s=0.4, render=False,
        quiet=False, status_hz=1.0) -> RunResult:
    plant = Plant()
    brain = G1Brain(cfg)
    nominal = brain.standing_pose().astype(np.float64)
    plant.reset(nominal, ARM_NOMINAL)

    # Settle: hold the stand pose so the spawned robot lands its feet before walking.
    n_settle = int(round(settle_s / CTRL_DT))
    for _ in range(n_settle):
        plant.set_ctrl(nominal, ARM_NOMINAL)
        plant.substep()
    settle_end = n_settle * CTRL_DT
    brain.reset()

    viewer = None
    if render:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(plant.m, plant.d)

    n_steps = int(round(seconds / CTRL_DT))
    tilt_acc = 0.0
    vx_acc = 0.0
    n_acc = 0
    fell_at = None
    survived_s = 0.0
    x0 = plant.base_x
    last_status = -1.0
    status_every = 1.0 / max(status_hz, 1e-6)

    for k in range(n_steps):
        sim_t = settle_end + k * CTRL_DT
        walk_t = k * CTRL_DT
        s = plant.read_state(walk_t)
        legs = brain.step(s, CTRL_DT)
        plant.set_ctrl(np.asarray(legs, dtype=np.float64), brain.arms.astype(np.float64))
        plant.substep()
        if viewer is not None:
            viewer.sync()

        s2 = plant.read_state(walk_t)
        tilt = math.hypot(s2.roll, s2.pitch)
        fallen = (s2.bz < 0.45 or abs(s2.roll) > 0.8 or abs(s2.pitch) > 0.8)
        if not fallen:
            survived_s = sim_t - settle_end + CTRL_DT
            tilt_acc += tilt
            vx_acc += s2.vx
            n_acc += 1
        elif fell_at is None:
            fell_at = sim_t - settle_end
            if cfg_stop_on_fall:
                break

        if not quiet and (sim_t - last_status) >= status_every:
            last_status = sim_t
            tag = "OK " if not fallen else f"FALL@{fell_at:.2f}s"
            print(f"  t={sim_t - settle_end:5.1f}s  x={plant.base_x - x0:+.2f}  "
                  f"bz={s2.bz:.3f}  roll={s2.roll:+.3f}  pitch={s2.pitch:+.3f}  "
                  f"vx={s2.vx:+.2f}  ph={brain.phase:4.2f}  {tag}")

    if viewer is not None:
        viewer.close()

    return RunResult(
        distance=plant.base_x - x0,
        survived_s=survived_s,
        fell_at=fell_at,
        mean_tilt=tilt_acc / max(n_acc, 1),
        mean_vx=vx_acc / max(n_acc, 1),
        final_x=plant.base_x,
    )


# module-level flag toggled by main (keeps run() signature small)
cfg_stop_on_fall = True


def _apply_sets(cfg: BrainConfig, sets):
    names = {f.name: f for f in fields(cfg)}
    for kv in sets:
        k, _, v = kv.partition("=")
        if k not in names:
            raise SystemExit(f"unknown gain {k!r}; valid: {sorted(names)}")
        cur = getattr(cfg, k)
        setattr(cfg, k, v if isinstance(cur, str) else type(cur)(v))
    return cfg


def _zero_gain(cfg: BrainConfig) -> BrainConfig:
    """Open-loop baseline: every feedback gain off -> brain == ghost reference."""
    for f in fields(cfg):
        if f.name in ("vx", "freq", "style", "ramp_s", "hip_scale", "a_lat",
                      "a_arm", "z_ref", "phase_floor", "fp_clamp", "ankle_clamp",
                      "hip_clamp", "knee_clamp"):
            continue
        if f.type in ("float", float):
            setattr(cfg, f.name, 0.0)
    return cfg


def main():
    global cfg_stop_on_fall
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", choices=["brain", "openloop"], default="brain")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--settle", type=float, default=0.4)
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--no-stop", action="store_true",
                    help="keep simulating after a fall (for watching)")
    ap.add_argument("--set", nargs="*", default=[], metavar="k=v",
                    help="override BrainConfig gains")
    ap.add_argument("--sweep", default=None, metavar="field=v1,v2,...",
                    help="run one config per value of a single field")
    args = ap.parse_args()
    cfg_stop_on_fall = not (args.no_stop or args.render)

    def base_cfg():
        c = BrainConfig.from_env()
        if args.controller == "openloop":
            c = _zero_gain(c)
        _apply_sets(c, args.set)
        return c

    if args.sweep:
        field, _, vals = args.sweep.partition("=")
        print(f"# SWEEP {field} over {vals}  ({args.controller}, {args.seconds:.0f}s)\n")
        rows = []
        for v in vals.split(","):
            c = base_cfg()
            _apply_sets(c, [f"{field}={v}"])
            r = run(c, seconds=args.seconds, settle_s=args.settle, quiet=True)
            rows.append((v, r))
            print(f"  {field}={v:>7}  ->  {r.line()}")
        print()
        best = max(rows, key=lambda vr: (vr[1].survived_s, vr[1].distance))
        print(f"# BEST: {field}={best[0]}  ->  {best[1].line()}")
        return

    cfg = base_cfg()
    if not args.quiet:
        print(f"# controller={args.controller}  {args.seconds:.0f}s")
        print(f"# config: {asdict(cfg)}\n")
    r = run(cfg, seconds=args.seconds, settle_s=args.settle,
            render=args.render, quiet=args.quiet)
    print("\n" + r.line())


if __name__ == "__main__":
    main()
