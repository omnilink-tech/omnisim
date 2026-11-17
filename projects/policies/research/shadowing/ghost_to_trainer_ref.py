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

"""Convert a generated ghost .npz (full qpos per step) into the trainer reference format
that gpu_mjwarp_g1_sitstand_trainer consumes via SITSTAND_REF_NPZ:
  traj : (N, 26) = [23 joint angles (cols indexed by `joints`)] + [x, z, -pitch]
  dt, joints
So the RL tracker can SHADOW the dynamically-feasible ghost (vs the old hand-drawn ref).

  python projects/policies/research/shadowing/ghost_to_trainer_ref.py --npz <ghost.npz> --mjcf <model.xml> --out <ref.npz>
"""
import argparse

import numpy as np
import mujoco


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--npz", required=True)
    p.add_argument("--mjcf", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    g = np.load(a.npz, allow_pickle=True)
    q = g["q"]
    qvel = g["qvel"]
    joints = [str(s) for s in g["joints"]]
    dt = float(g["dt"])
    m = mujoco.MjModel.from_xml_path(a.mjcf)
    adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in joints]
    dadr = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in joints]

    nj = len(joints)
    n = q.shape[0]
    # DeepMimic-style reference: positions (cols 0:nj), base x/z/-pitch (nj:nj+3), AND
    # velocities -- joint vels (nj+3 : 2nj+3) + base linear vel (2nj+3 : 2nj+6). Velocity
    # tracking is the missing DeepMimic ingredient: a feasible reference is only learnable
    # if the policy matches its VELOCITIES too, not just poses.
    traj = np.zeros((n, 2 * nj + 6), dtype=np.float32)
    for k, ad in enumerate(adr):
        traj[:, k] = q[:, ad]                                # joint positions
    traj[:, nj + 0] = q[:, 0]                                # base x (achieved)
    traj[:, nj + 1] = q[:, 2]                                # base z (achieved)
    w, x, y, z = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    traj[:, nj + 2] = -pitch                                 # trainer reads REF_PITCH = -col
    for k, da in enumerate(dadr):
        traj[:, nj + 3 + k] = qvel[:, da]                   # joint velocities
    traj[:, 2 * nj + 3:2 * nj + 6] = qvel[:, 0:3]           # base linear velocity (world)

    np.savez(a.out, traj=traj, dt=np.float32(dt), joints=np.array(joints))
    print(f"wrote {a.out}  traj={traj.shape}  dt={dt} (pos+base+jvel+basevel)  "
          f"z[min,max]=[{traj[:, nj+1].min():.3f},{traj[:, nj+1].max():.3f}]  "
          f"jvel|max|={np.abs(traj[:, nj+3:2*nj+3]).max():.2f}")


if __name__ == "__main__":
    main()
