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
"""Launch Foundry with a 30 FPS preview cap and a 75 C GPU shutdown guard."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0,str(ROOT))
from omnisim.dev.runner import omnisim_env, omnisim_binary_or_die


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',action='store_true',help='start an AI-versus-AI encounter automatically')
    parser.add_argument('--headless',action='store_true',help='bounded physics check without a window')
    parser.add_argument('--seconds',type=int,default=45,help='headless observation duration in wall-clock seconds')
    parser.add_argument('--match-seconds',type=int,default=180,help='encounter time limit in simulation seconds')
    args = parser.parse_args()
    if args.seconds <= 0 or args.match_seconds <= 0:
        parser.error('durations must be positive')
    if not args.demo and not args.headless:
        from garage import serve
        serve()
        return 0
    os.environ.setdefault('PYTHON_HOME',str(Path(sys.executable).parent))
    env = omnisim_env()
    output = ROOT / '_scratch/foundry' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    env['OMNISIM_LOG_PATH'] = str(output/'engine.log')
    env['ORC_FOUNDRY_OUTPUT'] = str(output/'encounter.jsonl')
    env['ORC_FOUNDRY_DEMO'] = '1' if args.demo else '0'
    env['ORC_FOUNDRY_DURATION'] = str(args.match_seconds)
    env.setdefault('WARP_CACHE_PATH',str(ROOT/'_scratch/foundry/warp_cache'))
    world = HERE/'worlds/orc_foundry.omniworld'
    if args.headless:
        command = [sys.executable,str(ROOT/'scripts/dev/headless_runner.py'),str(world),
                   '--duration',str(args.seconds),'--wait-for-step','--no-window',
                   '--realtime','--fail-on-runaway','--race-attempts','1']
    else:
        command = [omnisim_binary_or_die(),str(world),'--mode=realtime','--stdout','--stderr']
    guard = [sys.executable,str(ROOT/'scripts/dev/thermal_guard.py'),'run',
             '--ceiling','75','--interval','0.5','--precool','65','--']
    print('ORC / FOUNDRY',flush=True)
    print('GPU protection: stop at 75 C; start at or below 65 C. CPU is not monitored.',flush=True)
    print('Click the 3D view, then ENTER to start. WASD drives; SPACE toggles the weapon.',flush=True)
    print('C camera | TAB autopilot | H HUD | R restart',flush=True)
    print(f'Encounter and temperature logs: {output}',flush=True)
    with (output/'run.log').open('w',encoding='utf-8') as log:
        result = subprocess.run(guard+command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
    lines = (output/'run.log').read_text(encoding='utf-8',errors='replace').splitlines()
    for line in lines:
        if line.startswith('thermal_guard:') or line.startswith('[headless] Results:') or line.startswith('[headless] FAIL'):
            print(line)
    if result.returncode:
        print(f'Encounter stopped (exit {result.returncode}); see {output / "run.log"}.')
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
