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
# limitations under the License."""Rebuild motor targets from the saved, attributed LeRobot episode."""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REVISION = '06dc3da83c4fd3d1889b00f1dfd3780da8421f64'


def convert(values):
    if len(values) != 14 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Expected fourteen finite ALOHA values')
    result = []
    for offset in (0, 7):
        grip = .01844 + float(values[offset + 6]) * (.05800 - .01844)
        result.extend([*map(float, values[offset:offset + 6]), grip, -grip])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parquet', type=Path, default=ROOT.parents[3] / 'sim-to-real/aloha-battery/source/episode-000.parquet')
    args = parser.parse_args()
    import pyarrow.parquet as pq
    rows = pq.read_table(args.parquet).to_pylist()
    rows = sorted((r for r in rows if r['episode_index'] == 0), key=lambda r: r['frame_index'])
    if len(rows) != 600 or [r['frame_index'] for r in rows] != list(range(600)):
        raise ValueError('Expected the complete, ordered 600-frame episode zero')
    result = {'fps': 50, 'episode': 0, 'source_revision': REVISION,
              'targets': [convert(r['action']) for r in rows],
              'reference': [convert(r['observation.state']) for r in rows],
              'screenshot_frames': [0, 100, 150, 200, 250, 300, 350, 400, 500, 599]}
    (ROOT / 'episode.json').write_text(json.dumps(result), encoding='utf-8')
    print('Saved 600 frames; action gripper conversion is an estimated slider mapping.')


if __name__ == '__main__':
    main()
