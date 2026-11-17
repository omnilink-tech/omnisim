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
"""Independent source-MJCF / converted-URDF forward-kinematics verification."""
import json
import numpy as np
import mujoco
from author_motion import Arm, ROOT


def main():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/act/assets/bimanual_viperx_transfer_cube.xml'))
    data=mujoco.MjData(model)
    episode=json.loads((ROOT/'episode.json').read_text())
    maximum=0.0
    for frame in (0,140,200,300,500,599):
        data.qpos[:16]=episode['reference'][frame]
        mujoco.mj_forward(model,data)
        for side,offset in (('left',0),('right',8)):
            T=Arm(side).fk(episode['reference'][frame][offset:offset+6])
            body=model.body(f'vx300s_{side}/gripper_link')
            position=data.xpos[body.id]-[0,.5,0]
            rotation=data.xmat[body.id].reshape(3,3)
            maximum=max(maximum,float(np.max(np.abs(T[:3,3]-position))),
                        float(np.max(np.abs(T[:3,:3]-rotation))))
    # Source root yaw is 3.1416; the world uses pi (7.35e-6 rad difference).
    result={'source_poses':12,'maximum_position_or_rotation_element_error':maximum,
            'tolerance':1e-5,'mujoco_version':mujoco.__version__,'passed':maximum<1e-5}
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(2)


if __name__=='__main__':main()
