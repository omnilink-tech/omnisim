#!/usr/bin/env python3
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
"""Bounded, sequential cross-scene shadow A/B. Authored worlds remain unchanged.

Reports CPU submission cost and shadow draw counts, not GPU time or inferred FPS.
Run under the machine's workload guard where one is required.
"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from scripts.dev.render_ab import render, diff

SCENES = {
    'house': ('projects/samples/demos/worlds/rendering/beauty_bench_realism.omniworld', 'Authored local shadows'),
    'warehouse': ('projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld',
                  'Temporary variant: eight existing ceiling lights cast shadows; conveyor animation stopped'),
    'city': ('projects/samples/demos/worlds/showcase/city_traffic.omniworld',
             'Control without local shadows; traffic controller disabled for repeatability'),
}

@contextmanager
def benchmark_world(name, fps=5):
    source = REPO / SCENES[name][0]
    text = source.read_text(encoding='utf-8')
    # Limit drawing duty while measuring per-frame work, equally in both arms.
    text, count = re.subn(r'\bFPS\s+\d+', f'FPS {fps}', text, count=1)
    if not count:
        text, count = re.subn(r'\bWorldInfo\s*\{', f'WorldInfo {{\n  FPS {fps}', text, count=1)
        if count != 1:
            raise ValueError('WorldInfo was not found')
    if name == 'warehouse':
        text, count = re.subn(r'(PointLight\s*\{[^{}]*\bcastShadows\s+)FALSE', r'\g<1>TRUE', text)
        if count != 8:
            raise ValueError(f'Expected eight existing warehouse lights, found {count}')
        # The belt scrolls its texture using simulation time, which is not tied
        # to the capture frame counter. Preserve its geometry and freeze animation.
        text, count = re.subn(r'(ConveyorBelt\s*\{[^{}]*\bspeed\s+)[\d.]+', r'\g<1>0', text)
        if count != 1:
            raise ValueError('Warehouse conveyor speed was not found exactly once')
    if name == 'city':
        text, count = re.subn(r'controller\s+"city_traffic"', 'controller "<none>"', text)
        if count != 1:
            raise ValueError('City traffic controller was not found exactly once')
    fd, filename = tempfile.mkstemp(prefix='.shadow_bench_', suffix='.omniworld', dir=source.parent)
    os.close(fd)
    world = Path(filename)
    try:
        world.write_text(text, encoding='utf-8')
        yield world, hashlib.sha256(source.read_bytes()).hexdigest()
    finally:
        world.unlink(missing_ok=True)
        world.with_name(f'.{world.stem}.omniperspective').unlink(missing_ok=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--scene', choices=SCENES, action='append')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--frame', type=int, default=180)
    parser.add_argument('--fps', type=int, default=5, help='Drawing rate in both temporary worlds (1–60)')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 5 or not 90 <= args.frame <= 600:
        parser.error('Use 1–5 repeats and 90–600 frames')
    if not 1 <= args.fps <= 60:
        parser.error('Use 1–60 FPS')
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    # Apply identical controls to both arms and include them in each report.
    env = {'OMNISIM_BINARY': str(Path(args.binary).resolve()), 'OMNISIM_WGPU_TAA': '0',
           'OMNISIM_WGPU_CAMFX': '0', 'OMNISIM_WGPU_AUTOEXP': '0',
           'OMNISIM_WGPU_REPORT_EVERY': '1', 'OMNILIGHT': '0'}
    results = []
    for name in args.scene or SCENES:
        with benchmark_world(name, args.fps) as (world, source_hash):
            for repeat in range(args.repeats):
                record = {'scene': name, 'description': SCENES[name][1], 'source_world': SCENES[name][0],
                          'source_sha256': source_hash, 'repeat': repeat, 'draw_fps_cap': args.fps,
                          'capture_frame': args.frame}
                # Alternate order to reduce warm-cache/order bias.
                for arm in (('before', 'after') if repeat%2 == 0 else ('after', 'before')):
                    prefix = args.out / f'{name}-{repeat}-{arm}'
                    enabled = '0' if arm == 'before' else '1'
                    arm_env = dict(env, OMNISIM_WGPU_LOCAL_SHADOW_CACHE=enabled,
                                   OMNISIM_WGPU_LOCAL_SHADOW_CULL=enabled)
                    # render_ab budgets at 25 FPS. Allow the full capture interval
                    # at a lower requested cap as well; successful runs return early.
                    settle = 20 + max(0, args.frame / args.fps - args.frame / 25)
                    record[arm] = render(world, prefix.with_suffix('.png'), args.frame, arm_env,
                                         prefix.with_suffix('.log'), prefix.with_suffix('.report'), settle)
                    print(name, repeat, arm, json.dumps(record[arm]['profile']), flush=True)
                    if not record[arm]['dumped'] or record[arm]['errors']:
                        raise RuntimeError(f'{name} {arm} did not render successfully')
                    if record[arm]['profile'] is None or record[arm]['profile']['samples'] < 30:
                        raise RuntimeError(f'{name} {arm} produced insufficient warm-frame telemetry')
                record['diff'] = diff(args.out / f'{name}-{repeat}-before.png',
                                      args.out / f'{name}-{repeat}-after.png', 3)
                record['verdict'] = ('MATCH' if record['diff']['same_size'] and
                                     record['diff']['pixels_over_threshold'] == 0 else 'DIFFERS')
                results.append(record)
                (args.out / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    return int(any(record['verdict'] != 'MATCH' for record in results))

if __name__ == '__main__':
    sys.exit(main())
