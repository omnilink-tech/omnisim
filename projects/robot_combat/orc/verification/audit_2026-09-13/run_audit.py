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

"""Bounded, isolated combat observations; production sources are left intact."""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path('O:/omnisim')
OUT = ROOT / '_scratch/orc_audit_20260913'
sys.path.insert(0, str(ROOT))
os.environ['PYTHON_HOME'] = str(Path(sys.executable).parent)
os.environ['PATH'] = str(Path(sys.executable).parent) + os.pathsep + os.environ['PATH']
from omnisim.dev.runner import omnisim_env

cases = [
    ('open_field', 'orc/worlds/orc_open_field.omniworld', 22, None),
    ('forest_war', 'orc/worlds/orc_forest_war.omniworld', 22, None),
    ('queen_defense', 'orc/worlds/orc_queen_defense.omniworld', 22, None),
    ('battlebox_duel', 'battlebots/worlds/battlebox_duel.omniworld', 22, None),
    ('forced_detach', 'orc/worlds/orc_open_field.omniworld', 8, 'blue_bravo:weapon'),
]
finishing = '--finish' in sys.argv
rebuild_probe = '--rebuild' in sys.argv
if rebuild_probe:
    finishing = True
if finishing:
    cases = [
        ('open_field_match', 'orc/worlds/orc_open_field.omniworld', 90, None),
        ('forced_detach_observed', 'orc/worlds/orc_open_field.omniworld', 30, 'blue_bravo:weapon'),
    ]
if rebuild_probe:
    cases = [('forced_detach_rebuild', 'orc/worlds/orc_open_field.omniworld', 30, 'blue_bravo:weapon')]
results = []
for name, relative, seconds, forced in cases:
    original = ROOT / 'projects/robot_combat' / relative
    temporary = original.with_name('.audit_20260913_' + name + '.omniworld')
    assert temporary.resolve().is_relative_to(ROOT.resolve())
    assert not temporary.exists(), temporary
    score = OUT / (name + '.scorecard.json')
    def replace_config(m):
        raw = json.loads('"' + m.group(1) + '"')
        try:
            cfg = json.loads(raw)
        except (ValueError, TypeError):
            return m.group(0)
        if isinstance(cfg, dict) and cfg.get('fighters'):
            cfg['output'] = score.as_posix()
            return 'customData ' + json.dumps(json.dumps(cfg))
        return m.group(0)
    world = re.sub(r'customData\s+"((?:[^"\\]|\\.)*)"', replace_config,
                   original.read_text(encoding='utf-8'))
    observer_path = None
    if finishing and forced:
        observer_dir = original.parents[1] / 'controllers/orc_audit_observer_20260913'
        assert not observer_dir.exists()
        observer_dir.mkdir()
        observer_path = observer_dir / 'orc_audit_observer_20260913.py'
        observer_text = '''import json
from omnisim import Supervisor
s = Supervisor()
dt = int(s.getBasicTimeStep())
f = open(s.getCustomData(), 'w', buffering=1)
tick = 0
while s.step(dt) != -1:
    tick += 1
    if tick % 25:
        continue
    row = {'t': s.getTime()}
    for label, key in [('original', 'BLUE_BRAVO_WEAPON'), ('detached', 'DETACHED_blue_bravo_weapon_1')]:
        n = s.getFromDef(key)
        row[label] = {'position': list(n.getPosition()), 'velocity': list(n.getVelocity())} if n else None
    f.write(json.dumps(row) + '\\n')
'''
        if rebuild_probe:
            observer_text = observer_text.replace('tick = 0', 'tick = 0\nrebuilt = False').replace("    row = {'t': s.getTime()}", "    if not rebuilt and s.getTime() >= 3.0:\n        s.simulationRebuildPhysics()\n        rebuilt = True\n    row = {'t': s.getTime()}")
        observer_path.write_text(observer_text, encoding='utf-8')
        observed_path = 'detachment_rebuild_observations.jsonl' if rebuild_probe else 'detachment_observations.jsonl'
        world += '\nRobot { supervisor TRUE name "audit_observer" controller "orc_audit_observer_20260913" customData ' + json.dumps((OUT / observed_path).as_posix()) + ' }\n'
    temporary.write_text(world, encoding='utf-8')
    env = omnisim_env()
    env.update({
        'WARP_CACHE_PATH': str(OUT / 'warp_cache'),
        'OMNISIM_LOG_PATH': str(OUT / (name + '.log')),
        'BATTLEBOT_LOG_DIR': str(OUT / (name + '_brains')),
        'OMNISIM_DAMAGE_TIMER_S': '15' if not forced else ('4' if finishing else '6'),
        'OMNISIM_DAMAGE_TRACE': str(OUT / (name + '.trace.jsonl')),
    })
    if forced:
        env['OMNISIM_DAMAGE_FORCE_DETACH'] = forced
        env['OMNISIM_DAMAGE_FORCE_DETACH_T'] = '2'
    cmd = [sys.executable, str(ROOT / 'scripts/dev/headless_runner.py'),
           str(temporary), '--duration', str(seconds), '--wait-for-step',
           '--startup-timeout', '60', '--no-window', '--realtime',
           '--fail-on-runaway', '--race-attempts', '1']
    if finishing:
        cmd.remove('--realtime')
        cmd.extend(['--completion-log', str(OUT / (name + '.log.stderr')),
                    '--completion-pattern', 'writing scorecard', '--completion-grace', '1'])
    print('START', name, flush=True)
    try:
        with (OUT / (name + '.runner.txt')).open('w', encoding='utf-8') as fh:
            p = subprocess.run(cmd, cwd=ROOT, env=env, stdout=fh, stderr=subprocess.STDOUT)
        log = (OUT / (name + '.log')).read_text(encoding='utf-8', errors='replace')
        log += (OUT / (name + '.log.stderr')).read_text(encoding='utf-8', errors='replace')
        runner = (OUT / (name + '.runner.txt')).read_text(encoding='utf-8', errors='replace')
        result = dict(name=name, source=str(original), exit_code=p.returncode,
                      hit_log_lines=len(re.findall(r'HIT impulse=', log)),
                      detach_log_lines=len(re.findall(r'-> DETACHED_', log)),
                      runtime_rebuilds=log.count('mid-run physics rebuild:'),
                      outcome=[x for x in runner.splitlines() if any(t in x for t in
                               ('Results:', 'PASS', 'FAIL', 'runaway]', 'RUNAWAY'))])
        if score.exists():
            result['scorecard'] = json.loads(score.read_text())
        sidecar = OUT / (name + '.log.newton.json')
        if sidecar.exists():
            result['physics'] = json.loads(sidecar.read_text())
        results.append(result)
        result_file = 'rebuild_results.json' if rebuild_probe else ('completion_results.json' if finishing else 'runtime_results.json')
        (OUT / result_file).write_text(json.dumps(results, indent=2), encoding='utf-8')
        print(json.dumps(result), flush=True)
    finally:
        temporary.unlink(missing_ok=True)
        if observer_path:
            assert observer_path.resolve().is_relative_to(ROOT.resolve())
            observer_path.unlink(missing_ok=True)
            observer_path.parent.rmdir()
        for suffix in ('.omniperspective', '.wbproj'):
            sibling = temporary.with_name('.' + temporary.stem + suffix)
            if sibling.resolve().is_relative_to(ROOT.resolve()):
                sibling.unlink(missing_ok=True)
