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

"""Test if OmniQuad stands stably in our MuJoCo deploy environment when
the policy outputs ZERO action (i.e., motors hold nominal pose).

If the robot falls anyway, the env physics is misconfigured (probably
actuator gains or ground friction), not the policy.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO_ROOT))


def main():
    from projects.policies.research.backends.base import RobotSpec
    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
    import mujoco

    spec_path = Path("projects/policies/research/inference/policies/omniquad_walk_main/robot_spec.json")
    spec = RobotSpec.from_file(spec_path)
    nominal = np.asarray(spec.nominal_pose, dtype=np.float32)
    print(f"nominal pose: {nominal}")
    print(f"motor_pid (kp,ki,kd): {spec.motor_pid}")

    for kp_test in [20.0, 80.0, 200.0, 500.0, 1000.0, 2000.0]:
        m = load_or_convert(
            spec.urdf_path,
            actuator_joints=list(spec.joint_names),
            joint_limits=list(zip(spec.joint_limits_lo, spec.joint_limits_hi)),
            kp=kp_test, kv=2.0,
        )
        d = mujoco.MjData(m)
        d.qpos[0] = 0.0; d.qpos[1] = 0.0; d.qpos[2] = 0.70
        d.qpos[3] = 1.0; d.qpos[4:7] = 0.0
        for i, q in enumerate(spec.nominal_pose):
            d.qpos[7+i] = float(q)
        d.qvel[:] = 0.0
        d.ctrl[:len(nominal)] = nominal
        # Step 10 seconds and report final state.
        for _ in range(int(10.0 / 0.002)):
            mujoco.mj_step(m, d)
        bz = float(d.qpos[2])
        q_act = np.asarray(d.qpos[7:7+12], dtype=np.float32)
        v_lin = np.asarray(d.qvel[:3], dtype=np.float32)
        print(f"kp={kp_test:5.1f}: final bz={bz:.3f}  vx={v_lin[0]:+.3f}  vy={v_lin[1]:+.3f}  "
              f"|q_drift|max={np.abs(q_act - nominal).max():.3f}")


if __name__ == "__main__":
    main()
