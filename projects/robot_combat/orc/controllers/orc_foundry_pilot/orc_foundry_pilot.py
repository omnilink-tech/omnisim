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
"""Wheel/weapon control for Foundry. Physics determines achieved motion."""
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from foundry_logic import clamp, drive_to, orient_drive
from foundry_physics import dot, world, powertrain, drive_motor, weapon_motor
from foundry_config import DEFAULT
from foundry_strategy import tactic, clinch_timing
from omnisim import Supervisor, Keyboard


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    name = robot.getName().upper()
    settings = json.loads(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT['robots'][name]
    drive_model, weapon_model = drive_motor(settings), weapon_motor(settings)
    horizontal = settings['weapon'] == 'bar'
    strategy = settings['strategy']
    opponent_name = 'RAZOR' if name == 'ANVIL' else 'ANVIL'
    own = robot.getSelf()
    opponent = robot.getFromDef("RAZOR" if name == "ANVIL" else "ANVIL")
    director = robot.getFromDef("DIRECTOR").getField("customData")
    keyboard = robot.getKeyboard()
    keyboard.enable(dt)
    wheel_names = {"fl":"front_left", "fr":"front_right", "rl":"rear_left", "rr":"rear_right"}
    # Passive joints: apply an equal/opposite torque pair to the actual bodies.
    # This avoids a hidden velocity servo or an engine-global torque-mode flag.
    # No pose/velocity setters are used to propel a robot.
    parts = {p:robot.getFromDef(name+'_'+p.upper()+'_WHEEL') for p in wheel_names}
    parts['weapon'] = robot.getFromDef(name+'_WEAPON')
    battery_j = 24*12*3600.  # authored 12 Ah pack, not a measured hardware pack
    stalled_since = None
    reverse_until = 0
    engaged_since = None
    previous_command = [0, 0]
    telemetry = {}
    while robot.step(dt) != -1:
        keys = []
        key = keyboard.getKey()
        while key != -1:
            keys.append(key & Keyboard.KEY)
            key = keyboard.getKey()
        robot.setCustomData(json.dumps({'keys':keys,'t':robot.getTime(),'powertrain':telemetry}))
        try:
            state = json.loads(director.getSFString())
        except (ValueError, TypeError):
            continue
        now = robot.getTime()
        player = state.get("fighters", {}).get(name, {})
        broken = set(player.get("broken", []))
        live = state.get("phase") == "combat" and not player.get("defeated", False)
        left = right = weapon = 0.0
        if live:
            manual = name == "ANVIL" and state.get("mode") == "manual"
            if manual:
                keys = state.get("input", {})
                speed = keys.get("throttle", 0) * 25
                turn = keys.get("steer", 0) * 17
                left, right = clamp(speed-turn,-25,25), clamp(speed+turn,-25,25)
                weapon = 55 if keys.get("weapon", True) else 0
            else:
                position = own.getPosition()
                orientation = own.getOrientation()
                yaw = math.atan2(orientation[3], orientation[0])
                target = opponent.getPosition()
                opponent_broken = state.get('fighters',{}).get(opponent_name,{}).get('broken',[])
                missing_side = any(all(short+'_wheel' in opponent_broken for short in side)
                                   for side in (('fl','rl'),('fr','rr')))
                weapon_speed = abs(telemetry.get('motors',{}).get('weapon',{}).get('omega_rad_s',0))
                target_speed = 24/(weapon_model.k*weapon_model.reduction)
                left,right = tactic(strategy,position,yaw,target,state.get('targets',{}).get(opponent_name,{}),
                                    weapon_speed/target_speed,missing_side)
                velocity = own.getVelocity()
                speed = math.hypot(*velocity[:2]) + .2*abs(velocity[5])
                if now < reverse_until:
                    left, right = -15, -10
                elif speed < .15 and max(abs(left),abs(right)) > 6:
                    stalled_since = now if stalled_since is None else stalled_since
                    if now-stalled_since > .85:
                        reverse_until = now+.8
                        stalled_since = None
                else:
                    stalled_since = None
                # A driver makes space for another strike after a close clinch.
                # This changes only motor commands; damage still needs a load.
                if math.dist(position,target) < 1.65 and now >= reverse_until:
                    engaged_since = now if engaged_since is None else engaged_since
                    engage_s,retreat_s = clinch_timing(strategy)
                    if now-engaged_since > engage_s:
                        reverse_until = now+retreat_s
                        engaged_since = None
                else:
                    engaged_since = None
                stationary = state.get('mobility',{}).get(name,{}).get('stationary_s',0)
                if stationary >= 1:
                    # Keep attempting recovery throughout the count-out. No
                    # wheel-count or chassis-health branch turns off the drive.
                    left,right = ((25,25),(-25,-25),(-25,25),(25,-25))[int((stationary-1)/2)%4]
                elif state.get('mobility',{}).get(opponent_name,{}).get('stationary_s',0) >= 1:
                    # Give a struggling opponent space to demonstrate motion.
                    # Retreat with wheel torque; never relocate either body.
                    distance = math.dist(position,target)
                    if distance < 3.5:
                        away = [position[i]+(position[i]-target[i])*4/max(distance,.1) for i in range(3)]
                        left,right = drive_to(position,yaw,away)
                    else:
                        left = right = 0
                weapon = 55
        # Throttle ramp is seconds-based; it does not set achieved velocity.
        orientation = own.getOrientation()
        left,right = orient_drive(left,right,orientation[8])
        target_command = [left,right]
        command = [clamp(target_command[i], previous_command[i]-75*dt/1000, previous_command[i]+75*dt/1000)
                   for i in range(2)] if live else [0,0]
        previous_command = command
        base_omega = own.getVelocity()[3:]
        requests, axes = {}, {}
        for short in list(parts):
            part = 'weapon' if short == 'weapon' else short+'_wheel'
            if part in broken:
                parts.pop(short)
                continue
            axis = world(orientation,[0,0,1] if short == 'weapon' and horizontal else [0,1,0])
            omega = dot([x-y for x,y in zip(parts[short].getVelocity()[3:],base_omega)],axis)
            if short == 'weapon':
                duty = (1 if horizontal else -1) if weapon else 0
                model = weapon_model
                coast = live and not weapon
            else:
                duty = command[0 if short.endswith('l') else 1]/25*.75
                model, coast = drive_model, False
            requests[short] = (model,duty,omega,coast)
            axes[short] = axis
        values, battery = powertrain(requests,dt/1000,battery_j)
        battery_j = battery['remaining_j']
        reaction = [0.,0.,0.]
        for short,value in values.items():
            torque = [v*value['torque_nm'] for v in axes[short]]
            parts[short].addTorque(torque,False)
            reaction = [a-b for a,b in zip(reaction,torque)]
        own.addTorque(reaction,False)
        telemetry = {'battery':battery,'motors':values,'drive_command':command,
                     'drive_request':target_command,
                     'live':live, 'installed_wheels':sum(p != 'weapon' for p in parts),
                     'strategy':strategy,'weapon_type':settings['weapon']}


if __name__ == '__main__':
    main()
