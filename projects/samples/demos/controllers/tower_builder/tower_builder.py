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

"""Drive an independent physical Cartesian robot from central position targets."""
import json,math
from omnisim import Supervisor
r=Supervisor();dt=int(r.getBasicTimeStep());rid=r.getName()
motors={k:r.getDevice(k) for k in ['x','y','z','finger_l','finger_r']}
sensors={k:r.getDevice(k+'_sensor') for k in motors}
for s in sensors.values():s.enable(dt)
for k,m in motors.items():m.setVelocity(.8 if not k.startswith('finger') else .09)
director=r.getFromDef('DIRECTOR');field=director.getField('customData')
r.step(dt);r.step(dt)
q={k:s.getValue() for k,s in sensors.items()};v={k:0. for k in motors}
while r.step(dt)!=-1:
    c=json.loads(field.getSFString());cmd=c.get('commands',{}).get(rid)
    if cmd:
        for k,m in motors.items():
            if c['mode']=='jerk':m.setPosition(cmd[k]);continue
            speed,accel=(.025,.12) if k.startswith('finger') else ((.15,.35) if k=='z' else (.42,.8))
            error=cmd[k]-q[k];wanted=math.copysign(min(speed,math.sqrt(2*accel*abs(error))),error)
            v[k]+=max(-accel*dt/1000,min(accel*dt/1000,wanted-v[k]))
            dq=v[k]*dt/1000
            if abs(dq)>=abs(error):q[k]=cmd[k];v[k]=0
            else:q[k]+=dq
            m.setPosition(q[k])
    r.setCustomData(json.dumps({k:s.getValue() for k,s in sensors.items()}))
