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

"""Labelled visual comparisons; authored motion is explicitly not recorded replay."""
from pathlib import Path
import argparse
import json
import subprocess

ROOT = Path(__file__).resolve().parent
REAL = ROOT / 'source/videos/observation.images.right/chunk-000/file-000.mp4'
FONT = "fontfile='C\\:/Windows/Fonts/arial.ttf'" if Path("C:/Windows/Fonts/arial.ttf").exists() else "font=Arial"


def label(text, x, y, size=20, extra=''):
    return f"drawtext={FONT}:text='{text}':x={x}:y={y}:fontsize={size}:fontcolor=white{extra}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('kind', choices=['authored', 'replay'])
    parser.add_argument('--output', type=Path, required=True, help='New output file; existing files are never overwritten')
    args = parser.parse_args()
    authored = args.kind == 'authored'
    folder = ROOT / 'evidence' / ('authored' if authored else 'recorded-replay')
    config = json.loads((folder / 'trajectory.json').read_text())
    result = json.loads((folder / 'result.json').read_text())
    frames = len(config['targets_rad'])
    assert result['complete'] and result['frames'] == frames
    simulation = ROOT / 'videos' / ('simulation-authored.mp4' if authored else 'simulation-recorded-replay.mp4')
    duration = frames / config['fps']
    real_duration = float(json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'json', str(REAL)
    ]))['format']['duration'])
    hold = max(0, duration - real_duration)
    if hold < 1 / config['fps']:
        hold = 0
    real = '[0:v]setpts=PTS-STARTPTS,fps=30,scale=640:480,setsar=1'
    if hold:
        real += f',tpad=stop_mode=clone:stop_duration={hold:.6f}'
    real += ',pad=640:536:0:56:color=0x111820,'
    real += label('REAL SO101 | Episode 0', 16, 9, 22) + ','
    real += label('Physical recording | Original speed', 16, 35, 15)
    if hold:
        real += ',' + label('Real clip ended - final frame held', 12, 510, 18,
                            f":box=1:boxcolor=black@0.7:boxborderw=5:enable='gte(t,{real_duration})'")
    real += '[real];'
    # A fixed crop removes empty background; no animation or timing is changed.
    sim = '[1:v]setpts=PTS-STARTPTS,crop=1200:900:360:150,scale=640:480,setsar=1,pad=640:536:0:56:color=0x111820,'
    title = 'OMNISIM | Authored pick and place' if authored else 'OMNISIM | Recorded action replay'
    subtitle = 'Authored trajectory | Original speed' if authored else 'Estimated calibration | Original speed'
    sim += label(title, 16, 9, 22) + ',' + label(subtitle, 16, 35, 15) + '[sim];'
    footer1 = 'Same task, different trajectories. The simulation completes a physical pick and place.' if authored else 'Recorded actions replayed with estimated calibration. The simulated grasp fails.'
    footer2 = 'Real clip ends at 15.23 seconds; its final frame is held while the simulation finishes.' if authored else 'Both clips begin at episode start. Scene geometry and joint calibration are estimates.'
    graph = real + sim + '[real][sim]hstack=2,pad=1280:604:0:0:color=0x111820,'
    graph += label(footer1, 16, 546, 20) + ',' + label(footer2, 16, 577, 17) + '[out]'
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-nostdin', '-n', '-loglevel', 'error', '-i', str(REAL),
                    '-i', str(simulation),
                    '-filter_complex', graph, '-map', '[out]', '-frames:v', str(frames),
                    '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18',
                    '-movflags', '+faststart', str(output)], check=True)
    print(output)


if __name__ == '__main__':
    main()
