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

"""SO101 conversion, grasp verification and controller launch helpers."""
import hashlib
import json
import math
import re
from pathlib import Path
import time
import urllib.request
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def convert(values, calibration):
    """Explicit affine calibration; the gripper never inherits body units."""
    if calibration.get('units') not in ('normalised', 'degrees'):
        raise ValueError('Explicit normalised or degrees units required')
    if calibration.get('status') not in ('estimated', 'measured'):
        raise ValueError('Calibration must declare estimated or measured status')
    if len(values) != 6:
        raise ValueError('Expected six joint values')
    result = []
    for name, value in zip(JOINTS, values):
        entry = calibration['joints'][name]
        scale, offset = float(entry['scale_deg_per_unit']), float(entry['offset_deg'])
        if not all(math.isfinite(x) for x in (value, scale, offset)) or scale == 0:
            raise ValueError(f'Invalid calibration/value for {name}')
        if calibration['units'] == 'degrees' and name != 'gripper' and abs(scale) != 1:
            raise ValueError(f'Degree-valued body joint {name} requires scale +1 or -1')
        result.append(math.radians(value * scale + offset))
    return result


def validate_targets(targets):
    joints = ET.parse(ROOT / 'so101/so101.urdf').getroot().findall('joint')
    limits = {j.get('name'): j.find('limit') for j in joints}
    if not targets:
        raise ValueError('Trajectory is empty')
    for frame, row in enumerate(targets):
        if len(row) != 6:
            raise ValueError(f'Frame {frame}: expected six targets')
        for name, value in zip(JOINTS, row):
            limit = limits[name]
            if not math.isfinite(value) or not float(limit.get('lower')) <= value <= float(limit.get('upper')):
                raise ValueError(f'Frame {frame}: {name} target {value} exceeds URDF limits; refusing to silently clamp')


def grasp_evidence(trace, initial_z, fps):
    """A toss or one contact cannot pass as a sustained two-finger carry."""
    longest = current = 0
    run_start = None
    max_slip = 0.0
    for row in trace:
        contacts = set(row['cube_contacts'])
        airborne = row['cube_position'][2] > initial_z + .035
        bilateral = {'gripper_link', 'moving_jaw_so101_v1_link'} <= contacts
        if airborne and bilateral and not contacts.intersection(('table', 'pink_box')):
            current += 1
            if run_start is None:
                run_start = row['cube_in_gripper']
            max_slip = max(max_slip, math.dist(row['cube_in_gripper'], run_start))
            longest = max(longest, current)
        else:
            current, run_start = 0, None
    return {'bilateral_airborne_s': longest / fps, 'max_carry_slip_m': max_slip,
            'carried': longest / fps >= .5 and max_slip <= .015}


def contained(position, rotation, centre, cube_size=.04, inner_width=.067, base_thickness=.004, height=.065):
    # Axis-aligned box: check rotated cube extents, not just its centre.
    half = [cube_size / 2 * sum(abs(rotation[i * 3 + j]) for j in range(3)) for i in range(3)]
    delta = [a - b for a, b in zip(position, centre)]
    return (all(abs(delta[i]) + half[i] <= inner_width / 2 + .0005 for i in (0, 1))
            and delta[2] - half[2] >= base_thickness - .001
            and delta[2] + half[2] <= height + .001)


def rpc(base, route, payload=None):
    data = None if payload is None else json.dumps(payload).encode()
    request = urllib.request.Request(base.rstrip('/') + route, data=data,
                                     headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=120) as response:
        return json.load(response)


def execute(config, out_dir, harness, world=None, timeout=900):
    """Load a unique world; all timed control happens inside its controller."""
    validate_targets(config['targets_rad'])
    out = Path(out_dir).resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError(f'Output directory must be empty: {out}')
    out.mkdir(parents=True, exist_ok=True)
    config.update(out_dir=str(out), joints=JOINTS)
    trajectory = out / 'trajectory.json'
    trajectory.write_text(json.dumps(config, indent=2), encoding='utf-8')
    source = Path(world or ROOT / 'worlds/so101_pickplace_real.omniworld').resolve()
    scene = source.read_text(encoding='utf-8')
    pattern = r'controller "(?:<none>|so101_replay)"'
    if len(re.findall(pattern, scene)) != 1:
        raise ValueError('World must have exactly one SO101 replay robot')
    scene = re.sub(pattern, lambda _: f'controller "so101_replay"\n  controllerArgs ["--trajectory" "{trajectory.as_posix()}"]', scene)
    generated = ROOT / 'worlds' / f'.harness_so101_{uuid.uuid4().hex}.omniworld'
    generated.write_text(scene, encoding='utf-8')
    manifest = {'world': str(generated), 'world_sha256': sha256(generated),
                'urdf_sha256': sha256(ROOT / 'so101/so101.urdf'), 'trajectory_sha256': sha256(trajectory),
                'mode': config.get('mode'), 'calibration': config.get('calibration'),
                'source': config.get('source'), 'actuation': 'Motor.setPosition', 'timed_episode_pose_writes': 0}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    reply = rpc(harness, '/world/load', {'path': str(generated), 'light': True})
    if not reply.get('ok'):
        raise RuntimeError(f'World load failed: {reply}')
    deadline = time.monotonic() + timeout
    last_report = -1
    while time.monotonic() < deadline:
        error = out / 'error.txt'
        if error.exists():
            raise RuntimeError(error.read_text())
        result = out / 'result.json'
        if result.exists():
            value = json.loads(result.read_text())
            if value['frames'] != len(config['targets_rad']):
                raise RuntimeError('Controller result is incomplete')
            return value
        trace = out / 'trace.jsonl'
        count = len(trace.read_text().splitlines()) if trace.exists() else 0
        if count // 100 > last_report:
            print(f'Frames {count}/{len(config["targets_rad"])}', flush=True)
            last_report = count // 100
        time.sleep(.5)
    raise TimeoutError(f'Controller did not finish within {timeout}s; partial evidence remains in {out}')
