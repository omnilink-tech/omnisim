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
"""Bounded physical measurements, only selected by verify_foundry.py."""
import json
import math
import os
from pathlib import Path
import sys

ORC = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ORC/'shared'))
sys.path.insert(0,str(ORC/'controllers/orc_foundry_director'))
from foundry_physics import DRIVE, WEAPON, dot, world, powertrain, part_spec
from orc_foundry_director import Foundry


class MatchProbe(Foundry):
    """Run the actual pilots and damage director, with observation only."""
    def __init__(self, output):
        self.recording = []
        self.finished_at = None
        self.output = output
        self.capture = os.environ.get('ORC_FOUNDRY_CAPTURE') == '1'
        self.movie_started = False
        self.recording_saved = False
        super().__init__()
        self.mode = 'demo'
        self.duration = 180
        if self.capture:
            self.camera_mode = 1
        self.initial_mass = self.assembly_audit()['mass_kg']

    def write_event(self, event):
        super().write_event(event)
        self.recording.append(event)
        if event['type'] in ('result','error'):
            self.finished_at = event['t']

    def after_tick(self, now):
        if self.capture and not self.movie_started:
            self.s.movieStartRecording(str(self.output.with_suffix('.mp4')),1280,720,0,85,1,False)
            self.movie_started = True
        if self.finished_at is not None and now >= self.finished_at+2 and not self.recording_saved:
            self.s.worldSave(str(self.output.with_suffix('.aftermath.omniworld')))
            if self.capture:
                self.s.exportImage(str(self.output.with_suffix('.aftermath.jpg')),95)
                self.s.movieStopRecording()
            self.output.write_text(json.dumps({'case':'match', 'events':self.recording,
                'initial_mass_kg':self.initial_mass, 'final_audit':self.assembly_audit()},indent=2))
            self.recording_saved = True
        if self.recording_saved and (not self.capture or self.s.movieIsReady()):
            self.s.simulationQuit(1 if self.capture and self.s.movieFailed() else 0)


def main():
    case = os.environ['ORC_FOUNDRY_PROBE']
    output = Path(os.environ['ORC_FOUNDRY_PROBE_OUTPUT'])
    if case == 'match':
        game = MatchProbe(output)
        try:
            game.run()
        finally:
            if game.log:
                game.log.close()
        return
    game = Foundry()
    dt = game.dt/1000
    name = 'RAZOR' if case == 'drum' else 'ANVIL'
    bot = game.bots[name]
    battery = 24*12*3600.
    records = []
    work = 0.
    last_torque = {}
    before = None
    detached = False
    started = None
    while game.s.step(game.dt) != -1:
        now = game.s.getTime()
        if started is None:
            started = now
        elapsed = now-started
        rotation = bot.getOrientation()
        base_omega = bot.getVelocity()[3:]
        keys = ['weapon'] if case != 'drive' else ['fl_wheel','fr_wheel','rl_wheel','rr_wheel']
        requests, axes = {}, {}
        for part in keys:
            if (name,part) not in game.parts:
                continue
            node = game.parts[(name,part)]
            axis = world(rotation,[0,0,1] if part == 'weapon' and name == 'ANVIL' else [0,1,0])
            omega = dot([x-y for x,y in zip(node.getVelocity()[3:],base_omega)],axis)
            if part in last_torque:
                tau,previous_omega = last_torque[part]
                work += tau*(omega+previous_omega)*.5*dt
            duty = (1. if part == 'weapon' else .75) if .5 <= elapsed < 4.5 else 0.
            if name == 'RAZOR':
                duty = -duty
            requests[part] = (WEAPON[name] if part == 'weapon' else DRIVE,duty,omega,elapsed>=4.5 and part == 'weapon')
            axes[part] = axis
        values, pack = powertrain(requests,dt,battery)
        battery = pack['remaining_j']
        if case in ('detach','rebuild') and elapsed >= 4.5 and not detached:
            before = game.assembly_audit()
            if case == 'detach':
                game.detach('ANVIL','weapon')
            game.s.simulationRebuildPhysics()
            detached = True
            values = {}
            last_torque = {}
        reaction = [0.,0.,0.]
        for part,value in values.items():
            tau = value['torque_nm']
            torque = [x*tau for x in axes[part]]
            game.parts[(name,part)].addTorque(torque,False)
            reaction = [x-y for x,y in zip(reaction,torque)]
            last_torque[part] = (tau,value['omega_rad_s'])
        bot.addTorque(reaction,False)
        records.append({'t':elapsed,'audit':game.assembly_audit(),'position':bot.getPosition(),
                        'velocity':bot.getVelocity(),'motors':values,'work_j':work,'pack':pack,
                        'detached':detached})
        if elapsed >= 6.:
            output.write_text(json.dumps({'case':case,'before_detach':before,'records':records},indent=2))
            print(f'[foundry probe] {case} complete: {output}',flush=True)
            game.s.simulationQuit(0)
            break


if __name__ == '__main__':
    main()
