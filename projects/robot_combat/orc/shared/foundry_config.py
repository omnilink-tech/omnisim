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
"""Validated local-match blueprints. Internal slot IDs stay stable across names."""
import copy
import re

SLOTS = ('ANVIL', 'RAZOR')
COLORS = {'ember':'#e94717', 'teal':'#178b94', 'cobalt':'#386ced',
          'violet':'#a467d7', 'lime':'#a6bc45', 'bone':'#dbd1b5'}
FRAMES = {'light':(36., .90), 'balanced':(48., 1.), 'reinforced':(60., 1.10)}
GEARING = {'torque':24., 'balanced':19.2, 'speed':14.}
STRATEGIES = {
    'pressure': {'label':'Pressure', 'description':'Close the distance, keep attacking, and back out of clinches.'},
    'flanker': {'label':'Flanker', 'description':'Approach exposed wheels from the side before committing.'},
    'counter': {'label':'Counter-striker', 'description':'Build weapon speed, strike, then make room for another run.'},
}
DEFAULT = {'version':1, 'duration_s':180, 'robots':{
    'ANVIL':{'name':'ANVIL','color':'ember','weapon':'bar','frame':'balanced',
             'gearing':'balanced','rotor_kg':8,'strategy':'pressure'},
    'RAZOR':{'name':'RAZOR','color':'teal','weapon':'drum','frame':'balanced',
             'gearing':'balanced','rotor_kg':8,'strategy':'pressure'},
}}


def normalize_match(value=None):
    """Reject malformed/custom executable inputs; no arbitrary paths or code."""
    if value is None:
        return copy.deepcopy(DEFAULT)
    if not isinstance(value, dict) or set(value)-{'version','duration_s','robots'}:
        raise ValueError('Use a Foundry robot blueprint.')
    if type(value.get('version',1)) is not int or value.get('version',1) != 1:
        raise ValueError('This blueprint version is not supported.')
    duration = value.get('duration_s',180)
    if type(duration) is not int or duration not in (30,60,180):
        raise ValueError('Choose a 30, 60 or 180 second match.')
    robots = value.get('robots')
    if not isinstance(robots, dict) or set(robots) != set(SLOTS):
        raise ValueError('Configure both robot slots.')
    result = {'version':1,'duration_s':duration,'robots':{}}
    for slot in SLOTS:
        robot = robots[slot]
        if not isinstance(robot, dict) or set(robot) != set(DEFAULT['robots'][slot]):
            raise ValueError('Each robot needs a complete build and strategy.')
        name = robot['name']
        if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9 -]{1,16}',name):
            raise ValueError('Names use 1–16 letters, numbers, spaces or hyphens.')
        name = name.strip().upper()
        if not name:
            raise ValueError('Give each robot a name.')
        for key,choices in (('color',COLORS),('weapon',('bar','drum')),('frame',FRAMES),
                            ('gearing',GEARING),('strategy',STRATEGIES)):
            if not isinstance(robot[key],str) or robot[key] not in choices:
                raise ValueError(f'Choose a listed {key} option.')
        if type(robot['rotor_kg']) is not int or robot['rotor_kg'] not in (6,8,10):
            raise ValueError('Choose a 6, 8 or 10 kg rotor.')
        result['robots'][slot] = {**robot,'name':name}
    if result['robots']['ANVIL']['name'] == result['robots']['RAZOR']['name']:
        raise ValueError('Use different names so the match is easy to follow.')
    return result


def catalog():
    return {'default':normalize_match(), 'colors':COLORS, 'strategies':STRATEGIES,
            'frames':list(FRAMES), 'gearing':list(GEARING), 'weight_limit_kg':80}


def build_stats(robot):
    """Design estimates derived from the same motor/part models as the engine."""
    from foundry_physics import drive_motor, weapon_motor, part_spec
    import math
    loadouts = {'ANVIL':robot}
    def free_speed(motor,duty):
        lo,hi = 0., 1000.
        for _ in range(45):
            mid = (lo+hi)/2
            if motor.evaluate(duty,mid)['torque_nm'] > 0:
                lo = mid
            else:
                hi = mid
        return (lo+hi)/2
    rotor = part_spec('ANVIL','weapon',loadouts)
    weapon_speed = free_speed(weapon_motor(robot),1)
    return {'mass_kg':round(FRAMES[robot['frame']][0]+4*2.4+rotor.mass,1),
            'drive_speed_m_s':round(.17*free_speed(drive_motor(robot),.75),2),
            'wheel_stall_torque_nm':round(drive_motor(robot).evaluate(.75,0)['torque_nm'],1),
            'weapon_rpm':round(weapon_speed*60/math.tau),
            'weapon_energy_j':round(.5*rotor.inertia[2]*weapon_speed**2)}
