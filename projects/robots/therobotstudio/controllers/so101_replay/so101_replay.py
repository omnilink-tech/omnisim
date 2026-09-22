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

"""Tick-synchronous SO101 playback using motor commands, never joint pose writes."""
import argparse
import json
import math
from pathlib import Path
import traceback
import sys

from omnisim import Supervisor
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from replay_support import contained, grasp_evidence


def run(robot, config):
    out = Path(config['out_dir'])
    out.mkdir(parents=True, exist_ok=True)
    dt = int(robot.getBasicTimeStep())
    names = config['joints']
    motors = [robot.getDevice(name + '_motor') for name in names]
    sensors = [motor.getPositionSensor() for motor in motors]
    for sensor in sensors:
        sensor.enable(dt)
    cube = robot.getFromDef('GREEN_CUBE')
    box = robot.getFromDef('PINK_BOX')
    solids = {}
    def walk(node):
        if node.getTypeName() in ('Solid', 'Robot'):
            field = node.getField('name')
            if field:
                solids[field.getSFString()] = node
        children = node.getField('children')
        if children:
            for i in range(children.getCount()):
                walk(children.getMFNode(i))
        endpoint = node.getField('endPoint')
        if endpoint and endpoint.getSFNode():
            walk(endpoint.getSFNode())
    walk(robot.getFromDef('SO101'))
    grip = solids['gripper_link']
    contact_nodes = {name: solids[name] for name in ('gripper_link', 'moving_jaw_so101_v1_link')}
    contact_nodes.update(table=robot.getFromDef('TABLE'), pink_box=box)
    start_position = config.get('cube_position', list(cube.getField('translation').getSFVec3f()))
    start_rotation = config.get('cube_rotation', list(cube.getField('rotation').getSFRotation()))
    targets = config['targets_rad']

    def step():
        if robot.step(dt) == -1:
            raise RuntimeError('Simulation stopped before playback completed')

    def command(values):
        for motor, value in zip(motors, values):
            motor.setPosition(value)

    # Only initialisation may restore a prop. During the timed episode the
    # controller writes motor targets and reads physics, with no pose changes.
    cube.getField('translation').setSFVec3f([.8, .45, .02])
    cube.resetPhysics()
    command(targets[0])
    for _ in range(config.get('prologue_steps', round(1.6 * 1000 / dt))):
        step()
    cube.getField('translation').setSFVec3f(start_position)
    cube.getField('rotation').setSFRotation(start_rotation)
    cube.resetPhysics()
    for _ in range(20):
        step()
    start_time = robot.getTime()
    trace = []
    max_speed = 0.0
    max_height = -math.inf
    previous_position = cube.getPosition()
    pose_jumps = []
    screenshot_frames = set(config.get('screenshot_frames', []))
    if config.get('capture') or screenshot_frames:
        (out / 'frames').mkdir(exist_ok=True)
    try:
        with (out / 'trace.jsonl').open('w') as stream:
            for k, target in enumerate(targets):
                command(target)
                deadline = (k + 1) / config['fps']
                while robot.getTime() - start_time < deadline - dt / 2000:
                    step()
                    position = cube.getPosition()
                    speed = math.dist(position, previous_position) / (dt / 1000)
                    max_speed = max(max_speed, speed)
                    max_height = max(max_height, position[2])
                    if speed > 2:
                        pose_jumps.append({'frame': k, 'speed_m_s': speed})
                    previous_position = position
                # node_id names the reporting body, not its contact partner.
                # Pair matching world-space contact points, as the harness does.
                cube_points = {tuple(round(v, 4) for v in c.point) for c in cube.getContactPoints()}
                contact_names = [name for name, node in contact_nodes.items()
                                 if cube_points.intersection(tuple(round(v, 4) for v in c.point)
                                                             for c in node.getContactPoints())]
                gpos = grip.getPosition()
                grot = grip.getOrientation()
                delta = [a-b for a,b in zip(cube.getPosition(), gpos)]
                relative = [sum(grot[j*3+i]*delta[j] for j in range(3)) for i in range(3)]
                row = {'frame': k, 'time_s': robot.getTime() - start_time,
                       'command_rad': target, 'achieved_rad': [s.getValue() for s in sensors],
                       'cube_position': cube.getPosition(), 'cube_rotation': cube.getOrientation(),
                       'cube_velocity': cube.getVelocity(), 'cube_contacts': sorted(set(contact_names)),
                       'gripper_position': gpos, 'gripper_rotation': grot, 'cube_in_gripper': relative,
                       'contact_points': {name: [list(c.point) for c in node.getContactPoints()
                                                if tuple(round(v, 4) for v in c.point) in cube_points]
                                          for name, node in contact_nodes.items()}}
                if 'reference_rad' in config:
                    row['reference_rad'] = config['reference_rad'][k]
                trace.append(row)
                stream.write(json.dumps(row) + '\n')
                stream.flush()
                if config.get('capture') or k in screenshot_frames:
                    robot.exportImage(str(out / 'frames' / f'sim_{k:05d}.jpg'), 90)
                if k % 50 == 0:
                    print(f'[SO101] frame {k}/{len(targets)} cube={row["cube_position"]}', flush=True)
            for _ in range(round(1 / (dt / 1000))):
                step()
            final = cube.getPosition()
            centre = box.getPosition()
            evidence = grasp_evidence(trace, start_position[2], config['fps'])
            inside = contained(final, cube.getOrientation(), centre,
                               cube_size=config.get('cube_size_m', .04))
            final_points = {tuple(round(v, 4) for v in c.point) for c in cube.getContactPoints()}
            released = not any(final_points.intersection(tuple(round(v, 4) for v in c.point)
                                                         for c in node.getContactPoints())
                               for node in (grip, solids['moving_jaw_so101_v1_link']))
            settled = math.sqrt(sum(v*v for v in cube.getVelocity()[:3])) < .01
            result = {'complete': True, 'frames': len(trace), 'episode_duration_s': trace[-1]['time_s'],
                      'initial_cube_position': start_position, 'final_cube_position': final,
                      'box_position': centre, 'max_cube_height_m': max_height,
                      'max_cube_speed_m_s': max_speed, 'fast_cube_steps': pose_jumps,
                      'lifted': max_height > start_position[2] + .035,
                      **evidence, 'inside_box': inside, 'released': released, 'settled': settled,
                      'success': evidence['carried'] and inside and released and settled and max_speed < 2,
                      'actuation': 'Motor.setPosition', 'timed_episode_pose_writes': 0}
            temporary = out / 'result.tmp'
            temporary.write_text(json.dumps(result, indent=2))
            temporary.replace(out / 'result.json')
            print('[SO101] result ' + json.dumps(result), flush=True)
    except Exception:
        (out / 'error.txt').write_text(traceback.format_exc())
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory')
    args = parser.parse_args()
    if args.trajectory:
        config = json.loads(Path(args.trajectory).read_text())
    else:
        import tempfile
        from pickplace_demo import trajectory
        from replay_support import JOINTS
        config = {'targets_rad': trajectory(), 'joints': JOINTS, 'fps': 30,
                  'mode': 'authored-simulation', 'capture': False,
                  'out_dir': tempfile.mkdtemp(prefix='omnisim-so101-')}
        print('[SO101] Evidence: ' + config['out_dir'], flush=True)
    robot = Supervisor()
    for trial in config.get('trials', [config]):
        try:
            run(robot, trial)
        except Exception:
            out = Path(trial['out_dir'])
            out.mkdir(parents=True, exist_ok=True)
            (out / 'error.txt').write_text(traceback.format_exc())
            raise
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        pass


if __name__ == '__main__':
    main()
