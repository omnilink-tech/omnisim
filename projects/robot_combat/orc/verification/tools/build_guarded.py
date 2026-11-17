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
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[5]  # repo root (was the literal O:/omnisim in _scratch)
out=root/'_scratch/foundry_realism/compound_build'
out.mkdir(parents=True,exist_ok=True)
backup=out/'omnisim-bin.before.exe'
binary=root/'msys64/mingw64/bin/omnisim-bin.exe'
if not backup.exists():
    shutil.copy2(binary,backup)
env=os.environ.copy()
env['TMP']=env['TEMP']=env['TMPDIR']=str(out/'tmp')
Path(env['TMP']).mkdir(exist_ok=True)
env['CCACHE_DIR']=str(out/'ccache')
script='''export PATH=/mingw64/bin:/usr/bin:$PATH
make release -j2 OMNISIM_HOME=O:/omnisim WEBOTS_HOME=O:/omnisim PYTHON_HOME=C:/Users/<user>/.cache/codex-runtimes/codex-primary-runtime/dependencies/python PYTHON_LIB=-lpython312
'''
command=[sys.executable,str(root/'scripts/dev/thermal_guard.py'),'run','--ceiling','75','--interval','.5','--precool','65','--','C:/msys64/usr/bin/bash.exe','-c',script]
with (out/'build.log').open('w') as log:
    result=subprocess.run(command,cwd=root/'src/omnisim',env=env,stdout=log,stderr=subprocess.STDOUT)
print((out/'build.log').read_text(errors='replace')[-6000:])
if result.returncode:
    if not binary.exists(): shutil.copy2(backup,binary)
    raise SystemExit(result.returncode)
sys.path.insert(0,str(root/'scripts/packaging'))
from bundle_newton_runtime import stage_omnisim_runtime_module
print(stage_omnisim_runtime_module(root/'msys64/mingw64/bin/newton-runtime/site-packages'))
