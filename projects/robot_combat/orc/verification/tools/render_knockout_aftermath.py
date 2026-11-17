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

"""Render a frozen saved match state; this is not another physics test."""
from pathlib import Path
import os
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]  # repo root (was parents[2] in _scratch/foundry_realism/)
sys.path.insert(0, str(ROOT))
from omnisim.dev.runner import omnisim_env

output = ROOT / '_scratch/foundry_realism/knockout_snapshot'
worlds = output / 'worlds'
controller = output / 'controllers/aftermath_snapshot'
worlds.mkdir(parents=True, exist_ok=True)
controller.mkdir(parents=True, exist_ok=True)
source = ROOT / '_scratch/foundry_realism/20260913_042035_024789/match.aftermath.omniworld'
world = source.read_text()
world = world.replace('controller "orc_foundry_pilot"', 'controller "<none>"')
world = world.replace('controller "orc_foundry_probe"', 'controller "aftermath_snapshot"')
# Freeze the saved transforms for a still rendering. No new simulated outcome.
world, removed = re.subn(r'\bphysics Physics\s*\{[^}]*\}', '', world)
assert removed == 12, removed
(worlds / 'aftermath.omniworld').write_text(world)
(controller / 'aftermath_snapshot.py').write_text('''from omnisim import Supervisor
from pathlib import Path
import sys
sys.path.insert(0, "O:/omnisim/src/python")
from omniworld.viewpoint import look_at
s = Supervisor()
dt = int(s.getBasicTimeStep())
a = s.getFromDef('ANVIL').getPosition()
b = s.getFromDef('RAZOR').getPosition()
target = [(a[i]+b[i])/2 for i in range(3)]
eye = [target[0]+4.5,target[1]-5.5,target[2]+4.0]
camera = s.getFromDef('GAME_CAMERA')
camera.getField('position').setSFVec3f(eye)
camera.getField('orientation').setSFRotation(list(look_at(eye,target)))
for _ in range(8):
    s.step(dt)
s.exportImage("O:/omnisim/_scratch/foundry_realism/knockout_snapshot/verified_aftermath.jpg",95)
s.step(dt)
s.simulationQuit(0)
''')
os.environ.setdefault('PYTHON_HOME', str(Path(sys.executable).parent))
env = omnisim_env()
env['OMNISIM_LOG_PATH'] = str(output / 'engine.log')
env['WARP_CACHE_PATH'] = str(ROOT / '_scratch/foundry/warp_cache')
command = [sys.executable, str(ROOT / 'scripts/dev/thermal_guard.py'), 'run',
           '--ceiling', '75', '--interval', '.5', '--precool', '65', '--',
           sys.executable, str(ROOT / 'scripts/dev/headless_runner.py'), str(worlds / 'aftermath.omniworld'),
           '--duration', '12', '--gui', '--wait-for-step', '--realtime', '--race-attempts', '1']
with (output / 'run.log').open('w') as log:
    result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
print((output / 'run.log').read_text())
raise SystemExit(result.returncode)
