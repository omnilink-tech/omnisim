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

"""Measure OmniQuad's settled body Z in MuJoCo and recommend gait `ground_z`.

The GPU mjwarp residual trainer (`gpu_mjwarp_residual_trainer.py`)
treadmills because the gait engine's foot targets are in body frame
with `ground_z = -0.56` (tuned against Webots ODE + kp=20), but the
MuJoCo body settles at a different height than Webots. If the body
sits higher than expected, body-frame foot targets at z=-0.56 float
above the ground; if lower, they push into the ground and the leg
locks at its IK limit.

This tool drops OmniQuad from STAND_Z=0.7 with the controller's nominal
leg joints, integrates a few seconds of physics, and reports:

  body_z_settled            — where the chassis equilibrates
  recommended_ground_z      — = -body_z_settled (foot in body frame
                                so that foot_world_z ≈ 0 at neutral)

Usage:

  # Use the URDF-derived MJCF (mujoco.MjModel built in-memory via
  # backends._urdf_to_mjcf.load_or_convert with the canonical OmniQuad
  # joint order / gains). This is what to do when the deploy-exported
  # MJCF isn't available.
  python projects/policies/research/tools/calibrate_ground_z.py

  # Use the deploy-exported MJCF that the trainer normally reads.
  python projects/policies/research/tools/calibrate_ground_z.py \
      --mjcf C:/tmp/omniquad_newton_fixed.xml

Once the recommended value is known, pass it to the trainer via the
new --ground-z arg (or edit GaitParams default in omniquad_gait.py).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from projects.policies.research.backends.omniquad_robot_spec import (  # noqa: E402
    JOINT_NAMES,
    JOINT_LIMITS_LO,
    JOINT_LIMITS_HI,
    NOMINAL_POSE,
    OMNIQUAD,
)


STAND_Z = 0.7
NJ = 12


def _load_model(mjcf_path: str | None) -> mujoco.MjModel:
    if mjcf_path:
        path = Path(mjcf_path)
        if not path.exists():
            raise SystemExit(f"MJCF not found: {path}")
        return mujoco.MjModel.from_xml_path(str(path))
    # Fall back to URDF-derived MJCF via the canonical converter.
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
    return load_or_convert(
        OMNIQUAD.urdf_path,
        actuator_joints=list(JOINT_NAMES),
        joint_limits=list(zip(JOINT_LIMITS_LO, JOINT_LIMITS_HI)),
        kp=20.0, kv=3.0,
    )


def calibrate(mjcf_path: str | None, settle_s: float = 2.0,
              dt: float = 0.016) -> dict:
    m = _load_model(mjcf_path)
    if dt > 0:
        m.opt.timestep = dt
    d = mujoco.MjData(m)

    # Seed root at STAND_Z, identity orientation, NOMINAL leg joints.
    if m.nq < 7 + NJ:
        raise SystemExit(
            f"MJCF nq={m.nq} too small; expected free root (7) + {NJ} leg "
            f"joints. The MJCF must include a free joint on the chassis.")
    d.qpos[0:3] = [0.0, 0.0, STAND_Z]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z (identity)
    d.qpos[7:7 + NJ] = NOMINAL_POSE
    d.qvel[:] = 0.0

    # Hold the nominal pose as the position-actuator setpoint while we
    # let physics settle. Actuator order matches JOINT_NAMES.
    if m.nu >= NJ:
        d.ctrl[:NJ] = NOMINAL_POSE

    mujoco.mj_forward(m, d)

    n_steps = int(settle_s / m.opt.timestep)
    bz_trace = np.zeros(n_steps, dtype=np.float32)
    vz_trace = np.zeros(n_steps, dtype=np.float32)
    for k in range(n_steps):
        mujoco.mj_step(m, d)
        bz_trace[k] = float(d.qpos[2])
        vz_trace[k] = float(d.qvel[2])

    # "Settled" = mean of the last 25% of the trace, where |vz| is small.
    tail = bz_trace[int(0.75 * n_steps):]
    tail_vz = vz_trace[int(0.75 * n_steps):]
    bz_settled = float(tail.mean())
    vz_rms = float(np.sqrt((tail_vz ** 2).mean()))

    # The gait emits foot targets in body frame at z = ground_z. For
    # the foot to touch the ground (foot_world_z ≈ 0) when the body
    # is settled at world z = bz_settled, we want:
    #   foot_world_z = bz_settled + ground_z  ≈ 0
    #   => ground_z ≈ -bz_settled
    recommended_ground_z = -bz_settled

    return dict(
        bz_initial=STAND_Z,
        bz_settled=bz_settled,
        vz_rms_tail=vz_rms,
        n_steps=n_steps,
        timestep=float(m.opt.timestep),
        recommended_ground_z=recommended_ground_z,
        nq=int(m.nq),
        nu=int(m.nu),
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mjcf", default=None,
                   help="path to MJCF; default = build from canonical URDF")
    p.add_argument("--settle-s", type=float, default=2.0,
                   help="seconds of simulated time to let the body settle")
    p.add_argument("--dt", type=float, default=0.016,
                   help="MuJoCo timestep (matches trainer's DT)")
    args = p.parse_args()

    result = calibrate(args.mjcf, settle_s=args.settle_s, dt=args.dt)

    src = args.mjcf if args.mjcf else f"URDF: {OMNIQUAD.urdf_path}"
    print("OmniQuad ground_z calibration")
    print("-" * 60)
    print(f"  model source            : {src}")
    print(f"  MJCF  nq={result['nq']}  nu={result['nu']}  "
          f"dt={result['timestep']:.4f}s  steps={result['n_steps']}")
    print(f"  body z initial          : {result['bz_initial']:+.4f} m")
    print(f"  body z settled (tail)   : {result['bz_settled']:+.4f} m")
    print(f"  vz RMS over tail        : {result['vz_rms_tail']:+.4f} m/s "
          f"({'settled' if result['vz_rms_tail'] < 0.05 else 'NOT settled'})")
    print()
    print(f"  RECOMMENDED gait ground_z = {result['recommended_ground_z']:+.4f} m")
    print()
    print("Pass to trainer with --ground-z, e.g.:")
    print(f"  python projects/policies/research/training/gpu_mjwarp_residual_trainer.py "
          f"--ground-z {result['recommended_ground_z']:+.3f} ...")


if __name__ == "__main__":
    main()
