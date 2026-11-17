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

import os
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[5]  # repo root (was the literal O:/omnisim in _scratch)
sys.path.insert(0,str(root))
from omnisim.dev.runner import omnisim_env
os.environ['PYTHON_HOME']=str(Path(sys.executable).parent)
env=omnisim_env()
env['PYTHON_HOME']=str(Path(sys.executable).parent)
env['WARP_CACHE_PATH']=str(root/'_scratch/foundry/warp_cache')
env['OMNISIM_NO_WINDOW']='1'
env['OMNISIM_BRINGUP_SKIP_CAP']='0'
out=root/'_scratch/foundry_realism/final_regressions'
out.mkdir(parents=True,exist_ok=True)
if '--unit' in sys.argv:
    files=subprocess.run([sys.executable,'tests/conftest.py','--list-engine-free','tests'],cwd=root,capture_output=True,text=True,check=True).stdout.split()
    command=[sys.executable,'-m','pytest','-m','not engine','-q','-p','no:cacheprovider','--basetemp',str(out/'unit_tmp'),
             'tests/harness','tests/python','tests/packaging','tests/dev']+files
    name='unit'
else:
    name='spin' if '--spin-only' in sys.argv else 'engine'
    target='tests/test_newton_rebuild_physics.py'+('::test_rebuild_preserves_surviving_joint_spin' if name=='spin' else '')
    command=[sys.executable,'-m','pytest',target,'-q','-p','no:cacheprovider',
             '--basetemp',str(out/(name+'_tmp'))]
guard=[sys.executable,str(root/'scripts/dev/thermal_guard.py'),'run','--ceiling','75','--interval','.5','--precool','65','--']
with (out/(name+'.log')).open('w') as log:
    r=subprocess.run(guard+command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
print((out/(name+'.log')).read_text(errors='replace')[-10000:])
raise SystemExit(r.returncode)
