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

"""Build a CLEAN Go2 MJCF for the Shadowing Component-1 planner / Component-2
verifier from the auto-dumped C:\\tmp\\go2_newton.xml (which has anonymous
joint_N and unnamed actuators that the planner can't introspect).

Transforms:
  * rename joints joint_N -> FL_hip/FL_thigh/FL_calf/... (controller order),
  * rename the four calf bodies -> <leg>_foot so the planner's balance cost
    finds foot bodies,
  * name the position actuators "<joint>_pos" (gainprm=KE) and the velocity
    actuators "<joint>_vel" (gainprm=KD) so GhostGenerator picks the position
    targets as its control space.

Output: projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml
Run:  python build_go2_planner_mjcf.py [src_mjcf]   (default C:\\tmp\\go2_newton.xml)
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(r"C:\tmp\go2_newton.xml")
# Go2 and B2 share the same joint_N numbering + base+4x3 nesting, so this
# builder serves both: pass [src_mjcf] [out_mjcf] to retarget (e.g. B2).
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "go2_planner.mjcf.xml"

# joint_N -> controller name (from the known go2_newton.xml body/joint layout)
JMAP = {
    "joint_2": "FL_hip", "joint_6": "FL_thigh", "joint_10": "FL_calf",
    "joint_3": "FR_hip", "joint_7": "FR_thigh", "joint_11": "FR_calf",
    "joint_4": "RL_hip", "joint_8": "RL_thigh", "joint_12": "RL_calf",
    "joint_5": "RR_hip", "joint_9": "RR_thigh", "joint_13": "RR_calf",
}
# calf bodies -> <leg>_foot (for the balance cost's foot detection)
BMAP = {"body_3": "FL_foot", "body_6": "FR_foot",
        "body_9": "RL_foot", "body_12": "RR_foot"}


def main():
    if not SRC.exists():
        raise SystemExit(f"source MJCF not found: {SRC} (dump it via "
                         f"OMNISIM_NEWTON_SAVE_MJCF on go2_stand_test.omniworld)")
    s = SRC.read_text(encoding="utf-8")

    # rename joints (joint="..." in actuators and name="..." on the <joint>)
    for old, new in JMAP.items():
        s = re.sub(rf'\bjoint="{old}"', f'joint="{new}"', s)
        s = re.sub(rf'<joint name="{old}"', f'<joint name="{new}"', s)
    # rename the calf bodies -> *_foot (declarations AND <exclude> contact refs)
    for old, new in BMAP.items():
        s = s.replace(f'"{old}"', f'"{new}"')

    # restore the TRUNK collider: the URDF->MJCF import collapses the base
    # collision box to a 1 mm sphere (shape_1_1 size="0.001"), so a fallen
    # robot rests on its legs, not its belly. Pass [trunk_hx,hy,hz] (half-
    # extents, m) to put the real trunk box back -- needed for get-up realism.
    if len(sys.argv) > 3:
        hx, hy, hz = (x.strip() for x in sys.argv[3].split(","))
        s = s.replace('<geom name="shape_1_1" size="0.001"',
                      f'<geom name="trunk_box" type="box" size="{hx} {hy} {hz}"')

    # name the actuators: position (gainprm large, biasprm "0 -KE") and velocity
    # (biasprm "0 0 -KD"). After the joint rename each <general joint="FL_hip" ...>.
    def name_act(m):
        joint = m.group(1)
        body = m.group(0)
        if 'biasprm="0 0 -' in body:          # velocity actuator (3-term biasprm)
            suffix = "_vel"
        else:                                  # position actuator
            suffix = "_pos"
        return body.replace(f'<general joint="{joint}"',
                            f'<general name="{joint}{suffix}" joint="{joint}"', 1)

    s = re.sub(r'<general joint="(FL_hip|FL_thigh|FL_calf|FR_hip|FR_thigh|FR_calf|'
               r'RL_hip|RL_thigh|RL_calf|RR_hip|RR_thigh|RR_calf)"[^/]*/>',
               name_act, s)

    OUT.write_text(s, encoding="utf-8")

    # validate
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT))
    pos = [a for a in range(m.nu)
           if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith("_pos")]
    feet = [b for b in range(m.nbody)
            if "foot" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "")]
    print(f"wrote {OUT.name}: {m.njnt} joints, {m.nu} actuators, "
          f"{len(pos)} position actuators (_pos), {len(feet)} foot bodies")
    assert len(pos) == 12 and len(feet) == 4, "expected 12 _pos actuators + 4 foot bodies"
    print("  pos actuators:", [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in pos])


if __name__ == "__main__":
    main()
