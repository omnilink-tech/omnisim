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
"""Playable encounter, paired-contact damage, live debris, camera and HUD."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys

ORC = Path(__file__).resolve().parents[2]
ROOT = ORC.parents[2]
sys.path.insert(0, str(ORC / "shared"))
sys.path.insert(0, str(ROOT / "src/python"))
from foundry_logic import Combat, Mobility, PARTS, MAX_HP, clamp, free_solid, matrix_axis_angle
from foundry_logic import point_velocity
from foundry_physics import part_spec, surface_normal, contact_load, kinetic, body_record, dot
from foundry_config import normalize_match
from omniworld.viewpoint import look_at
from omnisim import Supervisor, Keyboard


class Foundry:
    def __init__(self):
        self.s = Supervisor()
        self.dt = int(self.s.getBasicTimeStep())
        config = json.loads(self.s.getCustomData() or '{}')
        self.match = normalize_match(config.get('match'))
        self.loadouts = self.match['robots'] if 'match' in config else None
        self.names = {slot:robot['name'] for slot,robot in self.match['robots'].items()}
        self.garage = bool(config.get('garage',False))
        self.last_preview = -100.
        self.finished_at = None
        self.mode = 'demo' if os.environ.get('ORC_FOUNDRY_DEMO') == '1' else config.get('mode','manual')
        self.duration = float(os.environ.get('ORC_FOUNDRY_DURATION',config.get('duration_s',180)))
        self.combat = Combat(self.loadouts)
        self.mobility = {name:Mobility() for name in self.combat.fighters}
        self.motion = {}
        self.last_opponent_contact = -100.
        self.bots = {name:self.s.getFromDef(name) for name in self.combat.fighters}
        self.pilot_inputs = [node.getField('customData') for node in self.bots.values()]
        self.parts = {}
        self.owner_ids = {}
        self.materials = {name:self.s.getFromDef(name).exportString() for name in
                          ('STEEL','DARK','RUBBER','ORANGE','BLUE','WHITE','AMBER','CYAN')}
        for name in ('TEAM_A','TEAM_B'):
            node = self.s.getFromDef(name)
            if node is not None:
                self.materials[name] = node.exportString()
        for name, node in self.bots.items():
            for part in PARTS:
                part_node = node if part == 'chassis' else self.s.getFromDef(name+'_'+part.upper())
                if part_node is None:
                    raise RuntimeError(f'Missing {name}.{part}')
                self.parts[(name,part)] = part_node
                self.owner_ids[part_node.getId()] = (name,part)
        self.keyboard = self.s.getKeyboard()
        self.keyboard.enable(self.dt)
        self.keys_before = set()
        self.phase = 'ready'
        self.start = None
        self.result = ''
        self.weapon_on = True
        self.camera_mode = 0
        self.hud = True
        self.camera = self.s.getFromDef('GAME_CAMERA')
        self.eye = list(self.camera.getField('position').getSFVec3f())
        self.message = 'SALVAGE YARD / SECTOR 07'
        self.message_until = 0
        self.debris = []
        self.rebuild_audit = None
        self.physics_generation = 0
        self.error = None
        self.events = []
        self.log = None
        output = os.environ.get('ORC_FOUNDRY_OUTPUT')
        if output:
            path = Path(output)
            path.parent.mkdir(parents=True,exist_ok=True)
            self.log = path.open('w',encoding='utf-8',buffering=1)
        self.ticks = 0
        self.write_event({'type':'init','mode':self.mode,'damage_model':'estimated load / plastic work',
                          'force_source':'estimated; public contacts omit force and normal',
                          'motor_model':'A28-150 electrical model / passive joints / reaction torque',
                          'match':self.match})

    def write_event(self,event):
        if self.log:
            self.log.write(json.dumps(event)+'\n')

    def detach(self,name,part):
        """Make a free rigid body, remove its joint, then rebuild once per batch.

        Do not mark the part broken on a failed import/removal. Both velocity
        halves are staged before finalization, so spinner momentum survives.
        """
        node = self.parts[(name,part)]
        position = list(node.getPosition())
        velocity = list(node.getVelocity())
        rotation = matrix_axis_angle(node.getOrientation())
        free_name = f'DEBRIS_{name}_{part.upper()}'
        exported = free_solid(node.exportString(),free_name,position,rotation,self.materials)
        joint = self.s.getFromDef(name+'_'+part.upper().replace('_WHEEL','')+'_JOINT')
        if joint is None:
            raise RuntimeError(f'Missing owning joint for {name}.{part}')
        children = self.s.getRoot().getField('children')
        children.importMFNodeFromString(-1,exported)
        free = self.s.getFromDef(free_name)
        if free is None:
            raise RuntimeError(f'Failed to import {free_name}')
        old_id = node.getId()
        joint.remove()
        if self.s.getFromDef(name+'_'+part.upper()) is not None:
            free.remove()
            raise RuntimeError(f'Original part still present: {name}.{part}')
        self.owner_ids.pop(old_id,None)
        self.parts.pop((name,part))
        self.combat.fighters[name].broken.add(part)
        # Queue on the unregistered body BEFORE the rebuild's first step. The
        # engine stages these velocities at finalization. Restoring one tick
        # later would overwrite a real collision impulse and corrupt momentum.
        free.setVelocity(velocity)
        self.debris.append({'def':free_name,'node':free,'origin':position,'robot':name,'part':part})
        self.write_event({'type':'detach','t':self.s.getTime(),'robot':name,'part':part,
                          'def':free_name,'position':position,'velocity':velocity})
        self.message = f'{self.names[name]} / {part.replace("_"," ").upper()} LOST'
        self.message_until = self.s.getTime()+3

    def sample_parts(self):
        return {key:(node.getPosition(),node.getVelocity(),node.getOrientation()) for key,node in self.parts.items()}

    def mechanical_energy(self,snapshot):
        return sum(kinetic(part_spec(*key,self.loadouts),velocity,rotation)+part_spec(*key,self.loadouts).mass*9.81*position[2]
                   for key,(position,velocity,rotation) in snapshot.items())

    def assembly_audit(self):
        records = [body_record(*key,node.getPosition(),node.getVelocity(),node.getOrientation(),self.loadouts)
                   for key,node in self.parts.items()]
        records += [body_record(d['robot'],d['part'],d['node'].getPosition(),d['node'].getVelocity(),
                                d['node'].getOrientation(),self.loadouts) for d in self.debris]
        return {'energy_j':sum(r['energy_j'] for r in records),
                'mass_kg':sum(r['mass_kg'] for r in records),
                'momentum':[sum(r['momentum'][i] for r in records) for i in range(3)],
                'angular_momentum':[sum(r['angular_momentum'][i] for r in records) for i in range(3)]}

    def contacts(self,now,snapshot):
        # ContactPoint.node_id identifies the owner on this API. Match the
        # opposite robot's contact position before applying damage; nearby
        # robots and ground/wall collisions alone cannot create a hit.
        points = {name:self.bots[name].getContactPoints(True) for name in self.bots}
        masses = {name:sum(part_spec(*key,self.loadouts).mass for key in snapshot if key[0] == name) for name in self.bots}
        motor_work = 0.
        for field in self.pilot_inputs:
            try:
                motors = json.loads(field.getSFString()).get('powertrain',{}).get('motors',{})
                motor_work += sum(max(0.,m['mechanical_w']) for m in motors.values())*self.dt/1000
            except (ValueError,TypeError,KeyError):
                pass
        # Whole-assembly loss is an upper bound; ground friction can contribute.
        # Never spend the full incident energy once for every contact manifold.
        previous = {key:value for key,value in self.previous_parts.items() if key in snapshot}
        current = {key:value for key,value in snapshot.items() if key in previous}
        budget = max(0.,self.mechanical_energy(previous)-self.mechanical_energy(current)+motor_work)
        matched = {}
        for a in points['ANVIL']:
            for b in points['RAZOR']:
                # Both entries of a native contact share the same world point.
                # A proximity allowance could mistake nearby floor contacts
                # for a collision between robots.
                if math.dist(a.point,b.point) > 1e-6:
                    continue
                ka = self.owner_ids.get(a.node_id)
                kb = self.owner_ids.get(b.node_id)
                if not ka or not kb or ka[0] == kb[0]:
                    continue
                if ka not in self.previous_parts or kb not in self.previous_parts:
                    continue
                # Use the non-weapon target surface where possible. Both frame
                # and speed are PRE-impact; post-solve motion may be separating.
                first,second = (kb,ka) if kb[1] == 'weapon' and ka[1] != 'weapon' else (ka,kb)
                target = self.previous_parts[second]
                normal = surface_normal(part_spec(*second,self.loadouts),target[0],target[2],a.point)
                load = contact_load((*first,*self.previous_parts[first]),(*second,*target),a.point,normal,masses,self.loadouts)
                va = point_velocity(snapshot[first][1],snapshot[first][0],a.point)
                vb = point_velocity(snapshot[second][1],snapshot[second][0],a.point)
                after_speed = dot([x-y for x,y in zip(va,vb)],normal)
                load['absorbed_j'] = max(0.,load['incident_j']-.5*load['effective_mass_kg']*after_speed**2)
                pair = (ka,kb)
                if load['absorbed_j'] >= matched.get(pair,{}).get('absorbed_j',0):
                    matched[pair] = load
        total = sum(load['absorbed_j'] for load in matched.values())
        scale = min(1.,budget/total) if total else 0.
        for (ka,kb),load in matched.items():
            load['absorbed_j'] *= scale
            load['assembly_loss_budget_j'] = budget
            for victim,attacker in ((ka,kb),(kb,ka)):
                event = self.combat.hit(now,*victim,*attacker,load)
                if event:
                    self.events.append(event)
                    self.write_event(event)
                    self.message = f'{self.names[victim[0]]} / {victim[1].replace("_"," ").upper()} HIT'
                    self.message_until = now+1.5
        detached = False
        breaking = any(hp <= 0 and part not in f.broken for f in self.combat.fighters.values()
                       for part,hp in f.hp.items())
        before_rebuild = self.assembly_audit() if breaking else None
        for name,fighter in self.combat.fighters.items():
            for part,hp in fighter.hp.items():
                if hp > 0 or part in fighter.broken:
                    continue
                if part == 'chassis':
                    fighter.broken.add(part)
                    continue
                self.detach(name,part)
                detached = True
        if detached:
            self.rebuild_audit = before_rebuild
            self.s.simulationRebuildPhysics()
            self.physics_generation += 1
            self.write_event({'type':'physics_rebuild_requested','t':now})
        return bool(matched)

    def observe_mobility(self, now, snapshot, opponent_contact):
        if opponent_contact:
            self.last_opponent_contact = now
        for name, bot in self.bots.items():
            pilot = json.loads(bot.getField('customData').getSFString() or '{}')
            command = pilot.get('powertrain',{}).get('drive_request',[])
            attempted = (now-pilot.get('t',-100) < .1 and
                         max((abs(v) for v in command), default=0.) >= 6)
            position, _, rotation = snapshot[(name,'chassis')]
            if rotation[8] < 0 <= self.previous_parts[(name,'chassis')][2][8]:
                self.write_event({'type':'overturned','t':now,'robot':name,
                                  'last_opponent_contact_t':self.last_opponent_contact})
            self.motion[name] = self.mobility[name].update(now,position,rotation,attempted,opponent_contact)
            if self.motion[name]['immobilized']:
                self.combat.fighters[name].immobilized = True
                self.write_event({'type':'immobilized','t':now,'robot':name,
                                  'cause':'overturned' if rotation[8] < -.5 else 'unable to recover',
                                  'mobility':self.motion[name], 'pilot':pilot,
                                  'broken':sorted(self.combat.fighters[name].broken)})

    def input(self):
        keys = set()
        key = self.keyboard.getKey()
        while key != -1:
            keys.add(key & Keyboard.KEY)
            key = self.keyboard.getKey()
        # A selected robot receives the viewport's keys exclusively. Pilots
        # relay their own keyboard state so clicking a fighter cannot disable
        # the game's director. With no selection, set union deduplicates it.
        for field in self.pilot_inputs:
            try:
                relay = json.loads(field.getSFString() or '{}')
                if self.s.getTime()-relay.get('t',-100) < .1:
                    keys.update(relay.get('keys',[]))
            except (ValueError,TypeError):
                pass
        pressed = keys-self.keys_before
        self.keys_before = keys
        if ord('R') in pressed and not getattr(self,'garage',False):
            self.s.worldReload()
            return None
        if ord('C') in pressed:
            self.camera_mode = (self.camera_mode+1)%3
        if ord('H') in pressed:
            self.hud = not self.hud
        if ord(' ') in pressed:
            self.weapon_on = not self.weapon_on
        # Qt special keys are masked to the lower 16 bits by OmniSim:
        # Tab=1, Return=4, keypad Enter=5. Also accept ASCII on other frontends.
        if pressed & {1,9} and not getattr(self,'garage',False):
            self.mode = 'demo' if self.mode == 'manual' else 'manual'
        if self.phase == 'ready' and (pressed & {4,5,13} or self.mode == 'demo'):
            self.start = self.s.getTime()+3
            self.phase = 'countdown'
        throttle = int(bool(keys & {ord('W'),Keyboard.UP}))-int(bool(keys & {ord('S'),Keyboard.DOWN}))
        steer = int(bool(keys & {ord('A'),Keyboard.LEFT}))-int(bool(keys & {ord('D'),Keyboard.RIGHT}))
        return {'throttle':throttle,'steer':steer,'weapon':self.weapon_on}

    def update_camera(self):
        a,b = (self.bots[name].getPosition() for name in ('ANVIL','RAZOR'))
        middle = [(a[i]+b[i])/2 for i in range(3)]
        if self.camera_mode == 1:
            target = middle
            distance = max(9,math.dist(a,b)*1.5)
            desired = [middle[0]-distance*.55,middle[1]-distance*.75,distance]
        elif self.camera_mode == 2 or self.phase in ('ready','countdown'):
            target = [middle[0],middle[1],.3]
            desired = [middle[0]-2.5,middle[1]-8,2.0]
        else:
            # Follow the vehicle's forward frame; smooth eye motion instead
            # of inheriting every chassis vibration into the camera.
            orient = self.bots['ANVIL'].getOrientation()
            yaw = math.atan2(orient[3],orient[0])
            c,s = math.cos(yaw),math.sin(yaw)
            target = [a[0]+c*1.2,a[1]+s*1.2,a[2]+.25]
            desired = [a[0]-c*3.6+s*.9,a[1]-s*3.6-c*.9,max(1.7,a[2]+1.7)]
        desired[0] = clamp(desired[0],-13.8,13.8)
        desired[1] = clamp(desired[1],-13.8,13.1)
        alpha = 1-math.exp(-self.dt/1000*3)
        self.eye = [self.eye[i]+(desired[i]-self.eye[i])*alpha for i in range(3)]
        self.camera.getField('position').setSFVec3f(self.eye)
        self.camera.getField('orientation').setSFRotation(list(look_at(self.eye,target)))

    def labels(self,now):
        if not self.hud:
            for i in range(10):
                self.s.setLabel(i,'',0,0,.03,0)
            return
        orange,white,gray,cyan = 0xF4AA64,0xF0EBDE,0xC5C7C2,0x71CED7
        self.s.setLabel(0,'O R C   /   F O U N D R Y',.035,.045,.048,white)
        self.s.setLabel(1,'SECTOR 07    /    SALVAGE CONTRACT',.035,.106,.025,gray)
        for i,(name,color,x) in enumerate([('ANVIL',orange,.035),('RAZOR',cyan,.72)]):
            f = self.combat.fighters[name]
            hp = f.hp['chassis']/MAX_HP['chassis']
            weapon = 'OFFLINE' if 'weapon' in f.broken else 'ONLINE'
            wheels = 4-sum(p.endswith('wheel') for p in f.broken)
            self.s.setLabel(2+i,f'{self.names[name]}   {math.ceil(hp*100):03d}%\n'+
                '|'*round(hp*20)+'.'*(20-round(hp*20))+f'\nDRIVE {wheels}/4   WEAPON {weapon}',
                x,.77,.033,color,font='LiberationMono-Regular')
        if self.phase == 'ready':
            title,sub = 'RECLAIM THE FOUNDRY','ENTER TO START    /    TAB TO WATCH THE AI DUEL'
        elif self.phase == 'countdown':
            title,sub = f'{max(1,math.ceil(self.start-now))}', 'WEAPONS ARMING'
        elif self.phase == 'aftermath':
            title = self.result
            sub = ('RESULT SAVED TO GARAGE' if self.garage else
                   'R TO RESTART    /    C TO INSPECT THE AFTERMATH')
        elif self.phase == 'error':
            title,sub = 'ENCOUNTER STOPPED','A PHYSICS CHANGE FAILED. SEE THE CONTROLLER LOG.'
        else:
            title = f'{max(0,math.ceil(self.duration-(now-self.start))):03d}  /  '+('AUTOPILOT' if self.mode=='demo' else 'PILOT CONTROL')
            sub = self.message if now < self.message_until else (
                'AUTONOMOUS MATCH    /    PHYSICAL COUNT-OUT' if self.garage else
                'DISABLE RAZOR TO SECURE THE YARD')
        self.s.setLabel(4,title,.32,.18,.048,white)
        self.s.setLabel(5,sub,.27,.245,.025,gray)
        controls = ('STRATEGY MATCH     C  CAMERA     H  HUD     RETURN TO GARAGE FOR BUILDS AND REMATCHES' if self.garage else
                    'WASD / ARROWS  DRIVE     SPACE  WEAPON     C  CAMERA     TAB  AUTOPILOT     H  HUD     R  RESTART')
        self.s.setLabel(6,controls,
                        .035,.94,.024,white)

    def run(self):
        self.previous_parts = self.sample_parts()
        while self.s.step(self.dt) != -1:
            now = self.s.getTime()
            self.ticks += 1
            # The request made last tick has finalized and advanced one step.
            if self.rebuild_audit is not None:
                self.write_event({'type':'rebuild_audit','t':now,'before':self.rebuild_audit,
                                  'after':self.assembly_audit(),
                                  'note':'one physics tick elapsed; contacts, gravity and motors may do work'})
                self.rebuild_audit = None
            control = self.input()
            if control is None:
                return
            if self.phase == 'countdown' and now >= self.start:
                self.phase = 'combat'
            snapshot = self.sample_parts()
            if self.phase == 'combat':
                try:
                    for name in self.bots:
                        position, _, rotation = snapshot[(name,'chassis')]
                        lower_bound = position[2]-sum(h*abs(r) for h,r in
                                                      zip((.56,.39,.15),rotation[6:9]))
                        if max(abs(position[0]),abs(position[1])) < 15.5 and lower_bound < -.05:
                            raise RuntimeError(f'{name} chassis penetrates the floor: lower bound {lower_bound:.3f} m')
                    opponent_contact = self.contacts(now,snapshot)
                    self.observe_mobility(now,snapshot,opponent_contact)
                except Exception as exc:
                    self.phase = 'error'
                    self.error = str(exc)
                    self.write_event({'type':'error','t':now,'message':str(exc)})
                    print(f'[foundry] {exc}',file=sys.stderr,flush=True)
                losers = [name for name,f in self.combat.fighters.items() if f.defeated]
                for name,bot in self.bots.items():
                    p = bot.getPosition()
                    if p[2] < -.5 or max(abs(p[0]),abs(p[1])) > 15.5:
                        if name not in losers:
                            losers.append(name)
                if self.phase == 'combat' and (losers or now-self.start >= self.duration):
                    self.phase = 'aftermath'
                    winner = next((name for name in self.bots if name not in losers),None) if len(losers)==1 else None
                    reason = ('immobilized' if any(self.combat.fighters[name].immobilized for name in losers)
                              else 'out_of_bounds' if losers else 'time_limit')
                    self.result = ('DOUBLE KNOCKOUT' if len(losers)==2 else
                                   'CONTRACT COMPLETE' if losers==['RAZOR'] else
                                   'ANVIL DISABLED' if losers==['ANVIL'] else 'TIME LIMIT / CONTRACT EXPIRED')
                    if self.garage:
                        self.result = self.names[winner]+' WINS' if winner else 'DOUBLE KNOCKOUT' if losers else 'TIME LIMIT / DRAW'
                    self.write_event({'type':'result','t':now,'result':self.result,
                                      'winner':winner,'reason':reason,'losers':losers,
                                      'mobility':self.motion,
                                      'fighters':{n:f.state() for n,f in self.combat.fighters.items()}})
            self.previous_parts = snapshot
            state = {'phase':self.phase,'mode':self.mode,'input':control,'result':self.result,
                     'fighters':{n:f.state() for n,f in self.combat.fighters.items()},
                     'impact_events':self.events[-8:], 'error':self.error,
                     'physics_generation':self.physics_generation}
            state['mobility'] = self.motion
            state['targets'] = {name:{part:pose[0] for (owner,part),pose in snapshot.items()
                                      if owner == name and part.endswith('wheel') and
                                      part not in self.combat.fighters[name].broken} for name in self.bots}
            self.s.setCustomData(json.dumps(state,separators=(',',':')))
            if self.ticks%2 == 0:
                self.update_camera()
            if self.ticks%6 == 0:
                self.labels(now)
            if self.ticks%30 == 0:
                self.write_event({'type':'sample','t':now,'phase':self.phase,
                                  'remaining_s':max(0,self.duration-(now-self.start)) if self.start else self.duration,
                                  'fighters':{n:f.state() for n,f in self.combat.fighters.items()},
                                  'mobility':self.motion,
                                  'bots':{n:{'position':b.getPosition(),'velocity':b.getVelocity(),
                                            'rotation':b.getOrientation(),
                                            'pilot':json.loads(b.getField('customData').getSFString() or '{}')} for n,b in self.bots.items()},
                                  'debris':{d['def']:{'position':d['node'].getPosition(),
                                            'displacement_m':math.dist(d['origin'],d['node'].getPosition())} for d in self.debris}})
            self.after_tick(now)

    def after_tick(self, now):
        """Garage snapshots and bounded exit; physics probes override this hook."""
        if not self.garage:
            return
        if now-self.last_preview >= 1:
            destination = Path(os.environ['ORC_FOUNDRY_OUTPUT']).parent
            temporary = destination/'preview.next.jpg'
            self.s.exportImage(str(temporary),90)
            if temporary.exists():
                try:
                    temporary.replace(destination/'preview.jpg')
                except PermissionError:
                    # A Windows browser response can briefly hold the previous
                    # frame open. A missed preview must not interrupt physics.
                    pass
            self.last_preview = now
        if self.phase in ('aftermath','error'):
            self.finished_at = now if self.finished_at is None else self.finished_at
            if now-self.finished_at >= 3:
                self.s.simulationQuit(1 if self.phase == 'error' else 0)


if __name__ == '__main__':
    game = Foundry()
    try:
        game.run()
    finally:
        if game.log:
            game.log.close()
