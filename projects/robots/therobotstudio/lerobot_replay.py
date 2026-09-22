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

"""Replay a LeRobot episode with explicit calibration and tick-synchronous motors.

Requires pandas and pyarrow. Normalised motor ranges are not URDF limits, and
numerical range cannot identify the recording's units. See README.md.
"""
import argparse
import json
from pathlib import Path
from replay_support import JOINTS, convert, execute, sha256


def prepare(parquet, calibration, episode=None, source='action', fps=30):
    import pandas as pd
    df = pd.read_parquet(parquet)
    if episode is not None:
        df = df[df.episode_index == episode]
    if len(df) == 0 or df.episode_index.nunique() != 1:
        raise ValueError('Select exactly one nonempty episode with --episode')
    df = df.sort_values('frame_index').reset_index(drop=True)
    indices = df.frame_index.tolist()
    if indices != list(range(indices[0], indices[0] + len(df))):
        raise ValueError('Frame indices must be contiguous and unique')
    if fps <= 0:
        raise ValueError('fps must be positive')
    if 'timestamp' in df and any(abs(float(b-a) - 1/fps) > .0001
                                 for a,b in zip(df.timestamp, df.timestamp.iloc[1:])):
        raise ValueError('Timestamps do not match the requested frame rate')
    return {'targets_rad': [convert(row, calibration) for row in df[source]],
            'reference_rad': [convert(row, calibration) for row in df['observation.state']],
            'fps': fps, 'mode': 'recorded-replay', 'calibration': calibration,
            'source': {'path': str(Path(parquet).resolve()), 'sha256': sha256(parquet),
                       'episode': int(df.episode_index.iloc[0]), 'column': source,
                       'first_frame': int(indices[0]), 'frames': len(df)}}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--parquet', required=True)
    ap.add_argument('--calibration', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--harness', required=True, help='URL of the harness session to load')
    ap.add_argument('--world')
    ap.add_argument('--source', choices=['action', 'observation.state'], default='action')
    ap.add_argument('--episode', type=int)
    ap.add_argument('--fps', type=float, default=30)
    ap.add_argument('--no-render', action='store_true')
    ap.add_argument('--no-grip', action='store_true', help='Drop control: hold the jaws open')
    args = ap.parse_args()
    calibration = json.loads(Path(args.calibration).read_text(encoding='utf-8'))
    config = prepare(args.parquet, calibration, args.episode, args.source, args.fps)
    config.update(capture=not args.no_render, no_grip=args.no_grip)
    if args.no_grip:
        import math
        for row in config['targets_rad']:
            row[JOINTS.index('gripper')] = math.radians(50)
    result = execute(config, args.out_dir, args.harness, args.world)
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
