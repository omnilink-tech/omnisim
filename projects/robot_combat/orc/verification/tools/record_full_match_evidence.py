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

"""Collect this session's measured evidence without changing raw recordings."""
import hashlib
import json
import re
import sys
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]  # repo root (was parents[2] in _scratch/foundry_realism/)
ORC = ROOT / 'projects/robot_combat/orc'
SCRATCH = ROOT / '_scratch/foundry_realism'
sys.path.insert(0, str(ORC))
from verify_foundry import assess

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def relative(path):
    return path.relative_to(ROOT).as_posix()

def run_record(name, cases=None):
    directory = SCRATCH / name
    reports = json.loads((directory / 'summary.json').read_text())
    reports = [r for r in reports if cases is None or r['case'] in cases]
    logs = {}
    raw = {}
    reassessed = []
    for report in reports:
        case = report['case']
        readings = json.loads((directory / (case + '.json')).read_text())
        reassessed.append(assess(readings))
        log = (directory / (case + '.run.log')).read_text()
        peaks = re.findall(r'thermal_guard: peak=(\d+) C', log)
        logs[case] = {'gpu_peak_c': int(peaks[-1]) if peaks else None,
                      'runner_results': re.findall(r'\[headless\] Results:.*', log)}
        if case == 'match':
            logs[case]['outcomes'] = [
                {'t': e['t'], 'result': e['result']} for e in readings['events'] if e['type'] == 'result']
        movie = directory / (case + '.mp4')
        if movie.exists():
            probe = subprocess.run(['C:/ffmpeg/bin/ffprobe.exe', '-v', 'error',
                '-show_entries', 'stream=codec_name,width,height,r_frame_rate,nb_frames',
                '-show_entries', 'format=duration,size', '-of', 'json', str(movie)],
                check=True, capture_output=True, text=True)
            logs[case]['movie'] = json.loads(probe.stdout)
        for suffix in ('.json', '.events.jsonl', '.run.log', '.mp4', '.aftermath.jpg', '.aftermath.omniworld'):
            path = directory / (case + suffix)
            if path.exists():
                raw[relative(path)] = sha(path)
    return {'directory': relative(directory), 'reports': reports, 'run_logs': logs,
            'reassessed_with_current_verifier': reassessed,
            'current_verifier_sha256': sha(ORC / 'verify_foundry.py'),
            'source_sha256': json.loads((directory / 'sources.json').read_text()),
            'raw_artifact_sha256': raw}

native = [ROOT / p for p in (
    'src/omnisim/nodes/OmSolid.cpp', 'src/omnisim/physics/OmNewtonBackend.cpp',
    'src/omnisim/physics/OmNewtonBackend.hpp', 'src/omnisim/physics/omnisim_newton_runtime.py',
    'msys64/mingw64/bin/omnisim-bin.exe',
    'msys64/mingw64/bin/newton-runtime/site-packages/omnisim_newton_runtime.py',
    'tests/test_newton_compound_mesh_contacts.py', 'tests/test_orc_foundry.py')]
assert sha(native[3]) == sha(native[5]), 'Source and bundled runtime differ'
headless = '20260913_042035_024789'
evidence = {
    'date': '2026-09-13',
    'scope': 'Measured physical immobilization in a full Foundry match; internal numerical checks, not real-world damage validation.',
    'machine_fingerprint': (SCRATCH / headless / 'machine.txt').read_text().splitlines(),
    'build_note': 'Working-tree build on parent e85b672da; binary and native source hashes identify the tested build.',
    'native_sha256': {relative(p): sha(p) for p in native},
    'headless_match_and_final_geometry_probes': run_record(headless),
    'earlier_corrected_mesh_probes': run_record('20260913_040956_940810', ['spin', 'drum', 'drive', 'rebuild', 'detach']),
    'probe_geometry_note': 'The earlier probes used the corrected blade and wheel/drum meshes, before final top-armor colliders. Drive and release were repeated with final geometry.',
    'regressions': {
        'foundry_unit': {'passed': 33},
        'broad_unit': {'passed': 1329, 'skipped': 11, 'deselected': 40, 'warnings': 46, 'subtests_passed': 24,
                       'seconds': 59.33, 'gpu_peak_c': 64,
                       'log': '_scratch/foundry_realism/final_regressions/unit.log',
                       'log_sha256': sha(SCRATCH / 'final_regressions/unit.log')},
        'native_compound_mesh': {'passed': 2, 'seconds': 5.56, 'gpu_peak_c': 64,
            'raw_output': '_scratch/foundry_realism/compound_final/compound_mesh0/compound_mesh/probe_out.txt',
            'values': {'flat_z_m': .099933617, 'posed_z_m': .499986112,
                       'flat_rebuilt_z_m': .099933617, 'posed_rebuilt_z_m': .499980479,
                       'low_friction_speed_m_s': .607812941, 'high_friction_speed_m_s': .000329153}},
        'existing_compound_warning_tests': {'passed': 3}
    },
    'rendered_runs': [],
    'knockout_aftermath_still': {
        'description': 'Visualization of the successful run saved world with its existing poses frozen by removing Physics fields; only the camera is reframed. Not a new simulated outcome.',
        'source_world': '_scratch/foundry_realism/20260913_042035_024789/match.aftermath.omniworld',
        'image': '_scratch/foundry_realism/knockout_snapshot/verified_aftermath.jpg',
        'image_sha256': sha(SCRATCH / 'knockout_snapshot/verified_aftermath.jpg'),
        'render_world_sha256': sha(SCRATCH / 'knockout_snapshot/worlds/aftermath.omniworld'),
        'gpu_peak_c': 62
    },
    'limitations': [
        'Count-out permits small motion: ten seconds within 0.15 m translation and 0.25 rad rotation, with fresh drive requests and no paired opponent contact.',
        'Successful individual match; no guarantee every AI encounter ends in a knockout or repeats identically.',
        'Estimated normals and impact loads, not solver contact forces; uncalibrated mount strength, stiffness and plastic work.',
        'Rigid-body mount detachment; no deformable hull, finite-element fracture or damage from the floor, walls and loose debris.',
        'Authored mass/inertia and friction; no measured real-robot validation.',
        'GPU sampled every 0.5 seconds with a 75 C stop and 65 C precool; CPU temperature is not monitored.'
    ]
}
for directory in ('20260913_042937_917616', '20260913_044127_287511'):
    if (SCRATCH / directory / 'summary.json').exists():
        evidence['rendered_runs'].append(run_record(directory))
destination = ORC / 'verification/full_match_cpu.json'
destination.write_text(json.dumps(evidence, indent=2) + '\n')
print(destination)
