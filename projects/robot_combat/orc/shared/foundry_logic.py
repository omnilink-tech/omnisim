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
"""Engine-independent rules for the Foundry playable encounter.

Damage accumulates estimated plastic work above component yield loads.
It is a reduced engineering model, not validated material failure prediction.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from foundry_physics import part_spec

PARTS = ("chassis", "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel", "weapon")
MAX_HP = {"chassis": 100.0, "wheel": 100.0, "weapon": 100.0}
WHEELS = ("fl", "fr", "rl", "rr")
YARD_LIMIT = 13.0


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_velocity(velocity, center, point):
    """v + omega cross r; angular velocity matters for weapon hits."""
    r = [point[i] - center[i] for i in range(3)]
    w = velocity[3:6]
    return [velocity[0] + w[1]*r[2] - w[2]*r[1],
            velocity[1] + w[2]*r[0] - w[0]*r[2],
            velocity[2] + w[0]*r[1] - w[1]*r[0]]


def impact_speed(a_velocity, a_center, b_velocity, b_center, point):
    a = point_velocity(a_velocity, a_center, point)
    b = point_velocity(b_velocity, b_center, point)
    return math.dist(a, b)


@dataclass
class Fighter:
    hp: dict = field(default_factory=lambda: {
        p: MAX_HP["wheel" if p.endswith("wheel") else p] for p in PARTS})
    broken: set = field(default_factory=set)
    immobilized: bool = False

    @property
    def defeated(self):
        # Integrity and missing parts do not tell us whether the robot moves.
        return self.immobilized

    def state(self):
        return {"hp": self.hp.copy(), "broken": sorted(self.broken), "defeated": self.defeated}


class Mobility:
    """Observe motion during powered recovery; never modify physical state.

    A count-out needs ten seconds with no 15 cm translation or 0.25 rad
    orientation change, fresh non-neutral drive commands, and no opponent
    contact. Rocking, spinning and being pushed all invalidate the proof.
    These are match adjudication tolerances, not material parameters.
    """
    def __init__(self, duration=10.):
        self.duration = duration
        self.anchor = None

    def update(self, now, position, rotation, attempted, obstructed=False):
        if not attempted or obstructed:
            self.anchor = None
            return {'stationary_s':0., 'immobilized':False}
        if self.anchor is None:
            self.anchor = (now, tuple(rotation))
            self.minimum = list(position)
            self.maximum = list(position)
            self.angle_peak = 0.
        self.minimum = [min(a,b) for a,b in zip(self.minimum,position)]
        self.maximum = [max(a,b) for a,b in zip(self.maximum,position)]
        distance = math.dist(self.minimum,self.maximum)
        angle = math.acos(clamp((sum(a*b for a,b in zip(rotation,self.anchor[1]))-1)/2,-1,1))
        self.angle_peak = max(self.angle_peak,angle)
        # The box diagonal bounds every pairwise displacement. Twice the
        # largest angular distance to the anchor bounds pairwise rotation.
        # Rocking around an anchor therefore cannot conceal actual motion.
        if distance >= .15 or self.angle_peak >= .125:
            self.anchor = None
            return self.update(now,position,rotation,attempted,obstructed)
        elapsed = now-self.anchor[0]
        return {'stationary_s':elapsed, 'immobilized':elapsed >= self.duration,
                'translation_m':distance, 'rotation_rad':2*self.angle_peak,
                'drive_attempted':bool(attempted), 'opponent_contact':bool(obstructed)}


class Combat:
    def __init__(self, loadouts=None):
        self.loadouts = loadouts
        self.fighters = {"ANVIL": Fighter(), "RAZOR": Fighter()}
        self.last_hit = {}
        self.events = 0

    def hit(self, now, victim, part, attacker, attacker_part, load):
        """Consume plastic-work capacity, with one energy budget per episode.

        Holding/sliding contact cannot repeatedly spend the same impact energy.
        Re-contact after 32 ms of separation starts a new episode. Yield and
        fracture parameters describe assumed mounts, not measured specimens.
        """
        fighter = self.fighters[victim]
        if part in fighter.broken or fighter.defeated:
            return None
        spec, other = part_spec(victim,part,self.loadouts), part_spec(attacker,attacker_part,self.loadouts)
        key = (victim,part,attacker,attacker_part)
        previous_time, previous_work = self.last_hit.get(key,(-100.,0.))
        if now-previous_time > .032+1e-9:
            previous_work = 0.
        # Compliance divides the pair's available energy between the mounts.
        # Below yield the deformation is recoverable and does not spend health.
        fraction = other.stiffness/(spec.stiffness+other.stiffness)
        elastic_limit = spec.yield_force**2/(2*spec.stiffness)
        work = max(0., load['absorbed_j']*fraction-elastic_limit)
        if load['estimated_peak_n'] <= spec.yield_force:
            work = 0.
        increment = max(0., work-previous_work)
        self.last_hit[key] = (now,max(previous_work,work))
        if increment <= 0:
            return None
        damage = 100*increment/spec.fracture_work
        fighter.hp[part] = max(0, fighter.hp[part] - damage)
        self.events += 1
        return {"type": "impact", "t": round(now, 3), "victim": victim,
                "part": part, "attacker": attacker, **load,
                "yield_n":spec.yield_force, "plastic_work_j":increment,
                "damage": round(damage, 2), "hp": round(fighter.hp[part], 2)}


def drive_to(position, yaw, target):
    """Differential drive pursuit, with a return-to-yard boundary policy."""
    x, y = position[:2]
    if max(abs(x), abs(y)) > YARD_LIMIT - 1.2:
        target = (0, 0, 0)
    angle = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
    # Braking into alignment matters for a high-torque skid-steer: constant
    # forward throttle while turning can orbit a nearby target indefinitely.
    forward = 21 * max(0,1-abs(angle)/.85) * min(1,math.dist(position[:2],target[:2])/.8)
    turn = clamp(angle * 10, -17, 17)
    return clamp(forward-turn, -25, 25), clamp(forward+turn, -25, 25)


def wheel_approach(position, center, targets):
    """Approach a surviving wheel around the chassis, using visible geometry.

    This is an AI waypoint, never a pose command. A ring waypoint avoids
    repeatedly hitting the empty side of a robot whose wheels already broke.
    """
    wheels = [p for name,p in targets.items() if name.endswith('wheel')]
    if not wheels:
        return center
    wheel = min(wheels,key=lambda p:math.dist(position,p))
    heading = math.atan2(position[1]-center[1],position[0]-center[0])
    destination = math.atan2(wheel[1]-center[1],wheel[0]-center[0])
    difference = wrap(destination-heading)
    if abs(difference) > .65:
        angle = heading+clamp(difference,-.65,.65)
        return [center[0]+2.0*math.cos(angle),center[1]+2.0*math.sin(angle),center[2]]
    return wheel


def orient_drive(left, right, up):
    """Keep forward/steering intent consistent if the chassis is inverted."""
    return (-right,-left) if up < 0 else (left,right)


def matrix_axis_angle(m):
    """Stable rotation conversion, including 180-degree detached-wheel poses."""
    cosine = clamp((m[0] + m[4] + m[8] - 1) / 2, -1, 1)
    angle = math.acos(cosine)
    if angle < 1e-8:
        return [0, 0, 1, 0]
    if math.pi - angle < 1e-6:
        diagonal = [m[0], m[4], m[8]]
        i = diagonal.index(max(diagonal))
        axis = [0.0, 0.0, 0.0]
        axis[i] = math.sqrt(max(0, (diagonal[i] + 1) / 2))
        for j in range(3):
            if i != j:
                axis[j] = (m[3*i+j] + m[3*j+i]) / (4*axis[i])
        return axis + [angle]
    denominator = 2*math.sin(angle)
    return [(m[7]-m[5])/denominator, (m[2]-m[6])/denominator,
            (m[3]-m[1])/denominator, angle]


def free_solid(exported, name, position, orientation, materials=None):
    """Rewrite only OUTER fields, never a nested visual's pose or name.

    Exported nodes are formatted one field per line by the engine. Track
    brace depth as well as indentation: absent outer pose fields must not
    cause a nested Shape/Pose's translation to be overwritten.
    """
    # Imported text has its own DEF/USE scope. Resolve appearance references
    # against cached source nodes so debris cannot silently lose its material.
    for material,appearance in (materials or {}).items():
        appearance = re.sub(r'^\s*DEF\s+\S+\s+', '',appearance,count=1).strip()
        exported = re.sub(r'\bUSE\s+'+re.escape(material)+r'\b',lambda m:appearance,exported)
    text = re.sub(r'^\s*DEF\s+\S+\s+', '', exported, count=1).strip()
    if not re.match(r'Solid\s*\{', text):
        raise ValueError("Detachable part must be an inline Solid")
    depth = 0
    kept = []
    for line in text.splitlines():
        if depth == 1 and re.match(r'\s*(translation|rotation|name)\s+', line):
            continue
        kept.append(line)
        # Quoted names/URLs can contain braces; ignore them while counting.
        structural = re.sub(r'"(?:[^"\\]|\\.)*"', '""', line)
        depth += structural.count('{') - structural.count('}')
    text = '\n'.join(kept)
    pose = ' '.join(f'{x:.9g}' for x in position)
    rotation = ' '.join(f'{x:.9g}' for x in orientation)
    return text.replace('{', f'{{\n translation {pose}\n rotation {rotation}\n name "{name}"', 1).replace('Solid', f'DEF {name} Solid', 1)
