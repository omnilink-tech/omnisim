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
# limitations under the License."""Run the recorded or authored ALOHA task through a dedicated OmniSim harness."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import time
import urllib.request
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
JOINTS = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate', 'left_finger', 'right_finger']
# Framed through /scene/frame around the pickup/slot workspace (radius 0.14 m),
# then orbited toward the left side so the gripper exposes the battery slots.
INSERTION_VIEW = {
    'position': [-0.233096, -0.154371, 0.391032],
    'orientation': [-0.275885097, 0.753685835, 0.596527515, 1.100764343],
    'fieldOfView': 1.15,
    'exposure': .85,
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(config):
    expected=round(config.get('duration_s',12)*50) if config.get('spring_compartment') else 600
    if config['fps'] != 50 or len(config['targets']) != expected or len(config['reference']) != expected:
        raise ValueError(f'This run requires exactly {expected} frames at 50 Hz')
    import math
    limits = []
    for side in ('left', 'right'):
        model = ET.parse(ROOT / f'aloha_{side}.urdf').getroot()
        for name in JOINTS:
            limit = model.find(f'joint[@name="{side}_{name}"]/limit')
            limits.append((float(limit.get('lower')), float(limit.get('upper'))))
    for i, row in enumerate(config['targets']):
        if len(row) != 16 or any(not math.isfinite(q) or not lo <= q <= hi for q, (lo, hi) in zip(row, limits)):
            raise ValueError(f'Invalid motor target at frame {i}; refusing to clamp')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['recorded-actions', 'measured-arms', 'authored', 'spring-insertion'], required=True)
    parser.add_argument('--harness', required=True, help='Use a dedicated harness; this loads its world')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--view', choices=['overhead', 'insertion'], default='overhead',
                        help='The insertion view brings the battery slots and grippers closer')
    parser.add_argument('--no-grip', action='store_true')
    parser.add_argument('--fixed-targets',type=Path,help='Execute saved motor targets with object feedback disabled')
    parser.add_argument('--engine-log',type=Path,help='Copy the dedicated engine log and Newton sidecar on completion')
    parser.add_argument('--timeout', type=float, default=900)
    args = parser.parse_args()
    out = args.out_dir.resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError(f'Refusing to overwrite existing evidence: {out}')
    motion_file={'authored':'authored_motion.json','spring-insertion':'spring_motion.json'}.get(args.mode,'episode.json')
    cfg = json.loads((ROOT / motion_file).read_text())
    if args.fixed_targets:
        cfg['targets']=json.loads(args.fixed_targets.read_text())
        cfg['feedback_control']=False
    if args.mode == 'measured-arms':
        for target, measured in zip(cfg['targets'], cfg['reference']):
            target[:6] = measured[:6]
            target[8:14] = measured[8:14]
    cfg.update(motion_mode=args.mode, out_dir=str(out), capture=args.capture, no_grip=args.no_grip, view=args.view,
               table_height=.010, battery_position=cfg.get('battery_position',[.085, .016, .0168]), remote_position=[-.058, .043, .010])
    if args.view == 'insertion':
        cfg['viewpoint'] = INSERTION_VIEW
    if not args.capture:
        cfg['screenshot_frames'] = [175,280,420,490,580,650,730,850,950,1050,1199,len(cfg['targets'])-1] if cfg.get('spring_compartment') else []
    validate(cfg)
    out.mkdir(parents=True, exist_ok=True)
    config_path = out / 'config.json'
    config_path.write_text(json.dumps(cfg), encoding='utf-8')
    if args.mode == 'spring-insertion':
        from build_spring_scene import scene_text, write_spring_models
        write_spring_models()
        scene=scene_text()
        scene=scene.replace('controller "battery_compartment"', f'controller "battery_compartment" controllerArgs [ "--config" "{config_path.as_posix()}" ]')
    else:
        scene = (ROOT / 'worlds/aloha_battery.omniworld').read_text()
    if args.view == 'insertion':
        fields = ' '.join(f'{key} ' + (' '.join(map(str, value)) if isinstance(value, list) else str(value))
                          for key, value in INSERTION_VIEW.items())
        scene, count = re.subn(r'\bViewpoint\s*\{[^{}]*\}', 'Viewpoint { ' + fields + ' }', scene)
        if count != 1:
            raise ValueError('Expected exactly one simple Viewpoint block')
    # A distinct structural field makes consecutive runs unambiguously reload,
    # even when only controllerArgs would otherwise differ.
    scene = re.sub(r'title "ALOHA - [^"]*"', f'title "ALOHA - {args.mode} - {uuid.uuid4().hex}"', scene, count=1)
    scene = scene.replace('translation 0 0 -.025', 'translation 0 0 -.015')
    scene = scene.replace('translation .085 .013 .007', 'translation '+ ' '.join(map(str,cfg['battery_position'])))
    if scene.count('controller "aloha_replay"') != 2:
        raise ValueError('Expected exactly two ALOHA controllers')
    scene = scene.replace('controller "aloha_replay"', f'controller "aloha_replay" controllerArgs [ "--config" "{config_path.as_posix()}" ]')
    world = ROOT / 'worlds' / f'.aloha_{uuid.uuid4().hex}.omniworld'
    world.write_text(scene, encoding='utf-8')
    (out / 'world.omniworld.txt').write_text(scene, encoding='utf-8')
    manifest = {'mode': args.mode, 'view': args.view, 'world_sha256': sha256(world), 'config_sha256': sha256(config_path),
                'model_sha256': {s: sha256(ROOT / f'aloha_{s}{"_spring" if cfg.get("spring_compartment") else ""}.urdf') for s in ('left', 'right')},
                'source_revision': cfg['source_revision'], 'no_grip': args.no_grip,
                'actuation': 'Motor.setPosition', 'timed_episode_pose_writes': 0}
    manifest['fixed_targets_sha256']=sha256(args.fixed_targets) if args.fixed_targets else None
    manifest['controller_sha256']={str(p.relative_to(ROOT)):sha256(p) for p in
        [ROOT/'controllers/aloha_replay/aloha_replay.py',ROOT/'evaluate.py',
         *([ROOT/'insertion_control.py',ROOT/'kinematics.py',ROOT/'compartment.py',
            ROOT/'controllers/battery_compartment/battery_compartment.py'] if cfg.get('spring_compartment') else [])]}
    # Keep the actual sources beside each run, before controllers import them.
    sources=out/'sources'
    for relative in [*manifest['controller_sha256'],
                     *(f'aloha_{side}{"_spring" if cfg.get("spring_compartment") else ""}.urdf' for side in ('left','right'))]:
        dest=sources/relative;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/relative,dest)
    runtime=ROOT.parents[3]/'msys64/mingw64/bin/newton-runtime/site-packages/omnisim_newton_runtime.py'
    if runtime.exists():
        manifest['bundled_runtime_sha256']=sha256(runtime)
    (out / 'run_manifest.json').write_text(json.dumps(manifest, indent=2))
    request = urllib.request.Request(args.harness.rstrip('/') + '/world/sync',
              data=json.dumps({'path': str(world), 'light': True}).encode(), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=120) as response:
        reply = json.load(response)
    if not reply.get('ok'):
        raise RuntimeError(f'World did not load: {reply.get("error", reply.get("load_state"))}')
    deadline, report = time.monotonic() + args.timeout, -1
    while time.monotonic() < deadline:
        if (out / 'error.txt').exists():
            raise RuntimeError((out / 'error.txt').read_text())
        if (out / 'spring-error.txt').exists():
            raise RuntimeError((out / 'spring-error.txt').read_text())
        if (out / 'result.json').exists():
            result = json.loads((out / 'result.json').read_text())
            if not result.get('complete') or result.get('frames') != len(cfg['targets']):
                raise RuntimeError('Incomplete evidence cannot pass')
            if args.engine_log:
                for source,name in [(args.engine_log,'engine.log'),(Path(str(args.engine_log)+'.newton.json'),'engine.newton.json')]:
                    if not source.is_file():raise FileNotFoundError(source)
                    shutil.copy2(source,out/name)
            print(json.dumps(result, indent=2))
            passed = not result['carried'] and not result['inside_slot'] if args.no_grip else result['task_success']
            return 0 if passed else 2
        path = out / 'trace.jsonl'
        count = len(path.read_text().splitlines()) if path.exists() else 0
        if count // 100 > report:
            print(f'Frames {count}/{len(cfg["targets"])}', flush=True)
            report = count // 100
        time.sleep(.5)
    raise TimeoutError(f'Incomplete run; partial evidence remains in {out}')


if __name__ == '__main__':
    raise SystemExit(main())
