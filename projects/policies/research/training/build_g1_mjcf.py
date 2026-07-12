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

"""Build the static G1 MJCF used by gpu_mjwarp_g1_stand_trainer.

mujoco_warp needs an XML on disk (not a runtime-loaded URDF). This
one-off converter starts from `g1_legs_mjcf.urdf` (the MuJoCo-
compatible variant — `<mujoco>` block stripped) and adds:
  - a ground plane at z=0
  - a free joint on the pelvis so the chassis can translate + rotate
  - contype/conaffinity groups so robot geoms don't self-collide
  - position + velocity actuators per joint matching the controller
    layer's PD gains (kp=400, kv=20)

Run once per URDF change. Output is committed alongside the URDF.
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
URDF_IN = REPO / "projects/robots/unitree/g1/urdf/g1_legs_mjcf.urdf"
MJCF_OUT = REPO / "projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml"

JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
]


def main() -> int:
    if not URDF_IN.exists():
        print(f"ERROR: {URDF_IN} missing — generate g1_legs_mjcf.urdf first "
              f"(strip the <mujoco> block from g1_legs_omnisim.urdf).",
              file=sys.stderr)
        return 1

    spec = mujoco.MjSpec.from_file(str(URDF_IN))

    # Ground plane.
    ground = spec.worldbody.add_geom()
    ground.name = "ground"
    ground.type = mujoco.mjtGeom.mjGEOM_PLANE
    ground.size[:] = [50.0, 50.0, 0.1]
    ground.friction[:] = [1.2, 0.005, 0.0001]
    ground.condim = 3
    ground.contype = 1
    ground.conaffinity = 2

    # Free joint on root body.
    root_bodies = list(spec.worldbody.bodies)
    if root_bodies:
        root = root_bodies[0]
        fj = root.add_freejoint()
        fj.name = "root_free"

    # Disable self-collision: robot geoms on group 2, only collide with
    # ground (group 1). URDF mesh collisions on adjacent links overlap
    # slightly and would push the body around at sim start otherwise.
    def walk(body):
        for g in body.geoms:
            if g.type != mujoco.mjtGeom.mjGEOM_PLANE:
                g.contype = 2
                g.conaffinity = 1
        for child in body.bodies:
            walk(child)

    for top in spec.worldbody.bodies:
        walk(top)

    # Add position + velocity actuators per joint, matching OmniSim
    # Newton's exact gains (verified by dumping its internal MJCF via
    # OMNISIM_NEWTON_SAVE_MJCF). Without matching these the trained
    # policy expects 20x stiffer joints than deploy provides and the
    # robot collapses on transfer.
    #
    # Position actuator: gainprm=20, biasprm[1]=-20  → kp = 20
    # Velocity actuator: gainprm=3,  biasprm[2]=-3   → kv = 3
    for jn in JOINTS:
        a_pos = spec.add_actuator()
        a_pos.name = f"{jn}_pos"
        a_pos.target = jn
        a_pos.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a_pos.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a_pos.gainprm[0] = 20.0
        a_pos.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a_pos.biasprm[1] = -20.0
        a_vel = spec.add_actuator()
        a_vel.name = f"{jn}_vel"
        a_vel.target = jn
        a_vel.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a_vel.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a_vel.gainprm[0] = 3.0
        a_vel.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a_vel.biasprm[2] = -3.0

    model = spec.compile()
    print(f"compile OK: nq={model.nq} nv={model.nv} nu={model.nu} "
          f"njnt={model.njnt}")
    MJCF_OUT.write_text(spec.to_xml(), encoding="utf-8")
    print(f"saved {MJCF_OUT} ({MJCF_OUT.stat().st_size} bytes)")
    # Round-trip verification.
    m2 = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    print(f"reload OK: nq={m2.nq} nu={m2.nu}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
