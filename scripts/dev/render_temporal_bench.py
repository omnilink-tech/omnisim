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
"""Bounded cross-scene temporal/GPU comparison; run under the host's workload guard.

The scenes are stationary so image differences expose temporal stability/detail,
not animated world timing. Motion/disocclusion correctness has a separate GPU test.
"""
import argparse
import json
import os
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from scripts.dev.render_ab import render, diff
from scripts.dev.render_shadow_bench import benchmark_world, SCENES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--baseline', type=Path, help='Optional previous binary; verifies the TAA-off image')
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--scene', choices=SCENES, action='append')
    parser.add_argument('--frame', type=int, default=96)
    parser.add_argument('--fps', type=int, default=5)
    parser.add_argument('--repeats', type=int, default=1)
    args = parser.parse_args()
    if not 90 <= args.frame <= 600 or not 1 <= args.fps <= 30 or not 1 <= args.repeats <= 3:
        parser.error('Use 90–600 frames, 1–30 FPS, and 1–3 repeats')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = {'OMNISIM_BINARY': str(args.binary.resolve()), 'OMNILIGHT': '0',
           'OMNISIM_WGPU_CAMFX': '0', 'OMNISIM_WGPU_AUTOEXP': '0', 'OMNISIM_WGPU_REPORT_EVERY': '1'}
    if os.name == 'nt':
        runtime = REPO / 'msys64/mingw64/bin/newton-runtime'
        env['PATH'] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + os.environ.get('PATH', '')
    results = []
    for scene in args.scene or SCENES:
        with benchmark_world(scene, args.fps) as (world, source_hash):
            for repeat in range(args.repeats):
                record = {'scene': scene, 'source_sha256': source_hash, 'repeat': repeat,
                          'draw_fps_cap': args.fps, 'capture_frame': args.frame, 'arms': {}}
                arms = ['legacy', 'camera-only', 'validated', 'taa-off', 'timing-off']
                if args.baseline: arms.append('baseline')
                if repeat % 2: arms.reverse()
                for arm in arms:
                    prefix = out / f'{scene}-{repeat}-{arm}'
                    arm_env = dict(env, OMNISIM_WGPU_TAA='1' if arm in ('legacy', 'camera-only', 'validated') else '0',
                                   OMNISIM_WGPU_TAA_VALIDATION='0' if arm == 'legacy' else '1',
                                   OMNISIM_WGPU_MOTION_VECTORS='0' if arm == 'camera-only' else '1',
                                   OMNISIM_WGPU_GPU_TIMING='0' if arm in ('timing-off', 'baseline')
                                   else str(prefix.with_suffix('.gpu')))
                    if arm == 'baseline': arm_env['OMNISIM_BINARY'] = str(args.baseline.resolve())
                    row = render(world, prefix.with_suffix('.png'), args.frame, arm_env,
                                 prefix.with_suffix('.log'), prefix.with_suffix('.cpu'),
                                 20 + max(0, args.frame/args.fps - args.frame/25))
                    row['environment'].pop('PATH', None)
                    record['arms'][arm] = row
                    print(scene, repeat, arm, json.dumps(row['gpu_profile']), flush=True)
                    if not row['dumped'] or row['errors']:
                        raise RuntimeError(f'{scene} {arm} did not render successfully')
                def compare(a, b):
                    return diff(out / f'{scene}-{repeat}-{a}.png', out / f'{scene}-{repeat}-{b}.png', 0)
                record['timing_image_check'] = compare('taa-off', 'timing-off')
                if args.baseline: record['baseline_image_check'] = compare('baseline', 'taa-off')
                record['temporal_difference'] = compare('legacy', 'validated')
                record['object_motion_difference'] = compare('camera-only', 'validated')
                results.append(record)
                (out / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
                for check in ('timing_image_check', 'baseline_image_check'):
                    if check in record and (not record[check]['same_size'] or record[check]['pixels_over_threshold']):
                        raise RuntimeError(f'{scene} {check} changed pixels')
    return 0


if __name__ == '__main__':
    sys.exit(main())
