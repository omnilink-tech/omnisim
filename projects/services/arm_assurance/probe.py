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

"""Development calibration. No source or result is silently overwritten."""
import argparse,json,os
from pathlib import Path
from session import Session,save
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--key-file',type=Path);p.add_argument('--capture',action='store_true');a=p.parse_args()
key=os.environ.get('OMNI_KEY','').strip() or a.key_file.read_text().strip()
with Session(a.out,key,capture=a.capture,reference=True) as s:
 initial=s.state();print(json.dumps({'initial':initial}),flush=True)
 # This pose is within the canonical robot's body limits but outside the new
 # proposed workcell's joint-1 limit. It is safe to probe in this empty fixture.
 q=[1.2,.1855,.7417,0.,2.2144,0.]
 text='Set joint positions to [1.2, 0.1855, 0.7417, 0, 2.2144, 0] radians.'
 result=s.request('/tool',{'tool':'set_joint_positions','q':q,'wait':True,'utterance':text})
 final=s.observe(1);save(a.out/'result.json',{'initial':initial,'proposal':q,'dispatch':result,'final':final})
 print(json.dumps({'dispatch':result,'final':final}),flush=True)
