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

"""Read-only assessment recorder. The reference controller alone actuates the robot."""
import json
import math
import os
from pathlib import Path
import traceback
from omnisim import Supervisor


def main():
    out = Path(os.environ['OMNISIM_ASSESSMENT_OUT'])
    spec = json.loads((out/'spec.json').read_text())
    sim = Supervisor()
    node = sim.getFromDef('ASSESSMENT_ROBOT')
    if node is None:
        raise RuntimeError('Assessed robot missing')
    capture = os.environ.get('OMNISIM_ASSESSMENT_CAPTURE') == '1'
    frames = out/'frames'
    if capture:
        frames.mkdir(exist_ok=True)
    seq = 0
    frame = 0
    view = sim.getRoot().getField('children')
    camera = next((view.getMFNode(i) for i in range(view.getCount()) if view.getMFNode(i).getTypeName()=='Viewpoint'),None)
    names = {}
    with (out/'trace.jsonl').open('w', encoding='utf-8') as handle:
        while sim.step(16) != -1:
            t = sim.getTime()
            p, v, o = node.getPosition(), node.getVelocity(), node.getOrientation()
            points=node.getContactPoints(True)
            for cp in points:
                if cp.node_id not in names:
                    contact_node=sim.getFromId(cp.node_id)
                    name_field=contact_node.getField('name') if contact_node else None
                    names[cp.node_id]=name_field.getSFString() if name_field else str(cp.node_id)
            row = dict(seq=seq, t_s=t, position_m=p, velocity=v,
                       roll_rad=math.atan2(o[7],o[8]),
                       pitch_rad=math.asin(max(-1.,min(1.,-o[6]))),
                       tilt_rad=math.acos(max(-1.,min(1.,o[8]))),
                       contacts=len(points),contact_nodes=[names[cp.node_id] for cp in points])
            handle.write(json.dumps(row, allow_nan=False)+'\n')
            if capture and t >= frame*.1:
                # Camera tracking only. Never mutate the assessed robot or its dynamics.
                camera.getField('position').setSFVec3f([p[0]+2.8,p[1]-3,p[2]+2.4])
                sim.exportImage(str(frames/f'{frame:05d}.jpg'), 90)
                frame += 1
            seq += 1
            if t >= spec['duration_s']:
                handle.flush()
                (out/'completion.json').write_text(json.dumps(dict(complete=True, samples=seq, t_s=t, frames=frame)))
                sim.simulationQuit(0)
                break


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        Path(os.environ['OMNISIM_ASSESSMENT_OUT'],'observer-error.txt').write_text(traceback.format_exc())
        raise
