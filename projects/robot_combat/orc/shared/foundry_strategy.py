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
"""Fighting styles produce drive requests; the physics determines movement."""
import math
from foundry_logic import drive_to, wheel_approach


def tactic(strategy, position, yaw, target, wheel_targets, weapon_fraction, missing_side=False):
    distance = math.dist(position,target)
    if strategy == 'counter' and weapon_fraction < .72 and distance < 4:
        away = [position[i]+(position[i]-target[i])*4/max(distance,.1) for i in range(3)]
        return drive_to(position,yaw,away)
    waypoint = wheel_approach(position,target,wheel_targets) if missing_side or strategy == 'flanker' else target
    left,right = drive_to(position,yaw,waypoint)
    return (left*.82,right*.82) if strategy == 'counter' else (left,right)


def clinch_timing(strategy):
    # Time to disengage, then time reversing to create a new approach.
    return {'pressure':(1.2,1.1),'flanker':(.85,1.3),'counter':(.45,1.8)}[strategy]
