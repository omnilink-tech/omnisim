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

"""Run the authored SO101 physical pick/place demo and its open-gripper control."""
import argparse
import json
import math
from replay_support import ROOT, execute, sha256


def trajectory():
    w = json.loads((ROOT / 'pickplace_motion.json').read_text())['waypoints']
    rows = []
    def hold(name, count):
        rows.extend([list(w[name]) for _ in range(count)])
    def move(a, b, count):
        for k in range(count):
            t = k / (count - 1)
            rows.append([x * (1-t) + y * t for x, y in zip(w[a], w[b])])
    hold('above', 30)
    move('above', 'grasp_open', 90)
    hold('grasp_open', 30)
    move('grasp_open', 'grasp_closed', 60)
    hold('grasp_closed', 90)
    move('grasp_closed', 'lift', 90)
    move('lift', 'place_closed', 90)
    hold('place_closed', 30)
    move('place_closed', 'place_open', 30)
    hold('place_open', 90)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--harness', required=True)
    parser.add_argument('--no-grip', action='store_true')
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()
    rows = trajectory()
    if args.no_grip:
        for row in rows:
            row[5] = math.radians(35)
    config = {'targets_rad': rows, 'fps': 30, 'mode': 'authored-simulation',
              'no_grip': args.no_grip, 'capture': not args.no_render,
              'source': {'path': 'pickplace_motion.json', 'sha256': sha256(ROOT / 'pickplace_motion.json')}}
    result = execute(config, args.out_dir, args.harness, ROOT / 'worlds/so101_pickplace.omniworld')
    print(json.dumps(result, indent=2))
    passed = (not result['lifted'] and not result['carried'] and not result['inside_box']
              and result['settled']) if args.no_grip else result['success']
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
