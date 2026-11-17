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

"""Encode a verified native capture, optionally beside its real recorded source."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--real', help='Source video, allowed only for recorded-replay mode')
    args = parser.parse_args()
    run = Path(args.run_dir)
    config = json.loads((run / 'trajectory.json').read_text())
    result = json.loads((run / 'result.json').read_text())
    count = len(config['targets_rad'])
    if not result['complete'] or result['frames'] != count:
        raise ValueError('Refusing to encode an incomplete run')
    ext = 'jpg'
    if any(not (run / 'frames' / f'sim_{i:05d}.{ext}').exists() for i in range(count)):
        raise ValueError('Capture is missing frames')
    mode = config['mode']
    if args.real and mode != 'recorded-replay':
        raise ValueError('An authored trajectory must not be presented as a replay comparison')
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    status = 'PASS - grasp, carry and placement verified' if result['success'] else 'FAIL - physical pick and place not reproduced'
    cmd = ['ffmpeg', '-nostdin', '-y', '-loglevel', 'error', '-framerate', str(config['fps']),
           '-i', str(run / 'frames' / f'sim_%05d.{ext}')]
    text = lambda label, y, size=20: f"drawtext=text='{label}':x=16:y={y}:fontsize={size}:fontcolor=white:box=1:boxcolor=black@0.7:boxborderw=5"
    if args.real:
        cmd += ['-i', args.real]
        scale = 'scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2,setsar=1'
        graph = (f"[0:v]{scale},{text('OmniSim - estimated calibration', 12, 17)}[sim];"
                 f"[1:v]{scale},{text('Real SO101 - recorded episode', 12, 17)}[real];"
                 f"[real][sim]hstack=2,pad=iw:ih+64:0:0,{text(status, 490, 18)},"
                 f"{text('Scene and calibration are estimates. No hardware dynamics validation.', 518, 16)}[out]")
        cmd += ['-filter_complex', graph, '-map', '[out]']
    else:
        label = 'Authored simulation trajectory - friction grasp - no attachment' if mode == 'authored-simulation' else 'Recorded commands - estimated calibration and scene'
        graph = f"scale=1280:-2,setsar=1,pad=iw:ih+64:0:0,{text(label, 12)}," + text(status, 'h-44')
        cmd += ['-vf', graph]
    cmd += ['-frames:v', str(count), '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18', str(output)]
    subprocess.run(cmd, check=True)
    print(output)


if __name__ == '__main__':
    main()
