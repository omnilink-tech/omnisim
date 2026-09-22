# Copyright 2026 OmniLink. SPDX-License-Identifier: Apache-2.0
"""Engine-free checks for the Foundry contact rules and debris export boundary."""
import importlib.util
import copy
import math
import re
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
ORC = ROOT / 'projects/robot_combat/orc'
sys.path.insert(0,str(ORC/'shared'))
from foundry_logic import Combat, Fighter, Mobility, drive_to, wheel_approach, orient_drive, free_solid, impact_speed, matrix_axis_angle
from foundry_physics import Motor, DRIVE, WEAPON, powertrain, part_spec, contact_load

spec = importlib.util.spec_from_file_location('foundry_input_test',ORC/'controllers/orc_foundry_director/orc_foundry_director.py')
director_module = importlib.util.module_from_spec(spec)
keyboard_constants = types.SimpleNamespace(KEY=0xffff,UP=315,DOWN=317,LEFT=314,RIGHT=316)
original_omnisim = sys.modules.get('omnisim')
sys.modules['omnisim'] = types.SimpleNamespace(Supervisor=MagicMock(),Keyboard=keyboard_constants)
try:
    spec.loader.exec_module(director_module)
finally:
    if original_omnisim is None:
        sys.modules.pop('omnisim',None)
    else:
        sys.modules['omnisim'] = original_omnisim


class FoundryRulesTests(unittest.TestCase):
    def test_sub_yield_contact_does_not_accumulate_damage(self):
        combat = Combat()
        for t in range(100):
            self.assertIsNone(combat.hit(t,'ANVIL','chassis','RAZOR','weapon',
                                        {'absorbed_j':1,'estimated_peak_n':100}))
        self.assertEqual(combat.fighters['ANVIL'].hp['chassis'],100)

    def test_spinning_contact_uses_angular_velocity(self):
        speed = impact_speed([0,0,0,0,0,40],[0,0,0],[0]*6,[1,0,0],[.5,0,0])
        self.assertAlmostEqual(speed,20)

    def test_contact_episode_cannot_spend_same_energy_twice(self):
        combat = Combat()
        load = {'absorbed_j':1000,'estimated_peak_n':100000}
        self.assertIsNotNone(combat.hit(1,'ANVIL','weapon','RAZOR','weapon',load))
        first = combat.fighters['ANVIL'].hp['weapon']
        for tick in range(1,100):
            self.assertIsNone(combat.hit(1+tick*.008,'ANVIL','weapon','RAZOR','weapon',load))
        self.assertEqual(first,combat.fighters['ANVIL'].hp['weapon'])
        for t in range(3,15):
            combat.hit(t,'ANVIL','weapon','RAZOR','weapon',load)
        self.assertEqual(combat.fighters['ANVIL'].hp['weapon'],0)
        # Logical damage alone must not claim that graph removal succeeded.
        self.assertNotIn('weapon',combat.fighters['ANVIL'].broken)

    def test_damage_and_wheel_count_cannot_switch_off_a_moving_robot(self):
        f = Fighter()
        f.broken.update(('fl_wheel','fr_wheel','rl_wheel','weapon'))
        f.hp['chassis'] = 0
        self.assertFalse(f.defeated)
        f.immobilized = True
        self.assertTrue(f.defeated)

    def test_countout_requires_powered_attempts_and_no_opponent_pin(self):
        identity = [1,0,0,0,1,0,0,0,1]
        for attempted,contact in ((False,False),(True,True)):
            monitor = Mobility()
            for tick in range(151):
                reading = monitor.update(tick*.1,[0,0,0],identity,attempted,contact)
            self.assertFalse(reading['immobilized'])
        monitor = Mobility()
        for tick in range(101):
            reading = monitor.update(tick*.1,[0,0,0],identity,True)
        self.assertTrue(reading['immobilized'])

    def test_translation_or_rotation_resets_countout(self):
        identity = [1,0,0,0,1,0,0,0,1]
        for spinning in (False,True):
            monitor = Mobility()
            for tick in range(151):
                t = tick*.1
                c,s = math.cos(t),math.sin(t)
                rotation = [c,-s,0,s,c,0,0,0,1] if spinning else identity
                position = [0,0,0] if spinning else [.1*t,0,0]
                reading = monitor.update(t,position,rotation,True)
                self.assertFalse(reading['immobilized'])

    def test_steering_turns_towards_target_and_back_into_yard(self):
        left,right = drive_to((0,0,0),0,(0,5,0))
        self.assertLess(left,right)
        left,right = drive_to((12,0,0),0,(20,0,0))
        self.assertLess(left,right)
        self.assertAlmostEqual(left,-right)

    def test_drum_driver_routes_to_remaining_wheel_around_empty_side(self):
        wheel = [.39,-.455,.18]
        waypoint = wheel_approach([0,2,.18],[0,0,.18],{'fr_wheel':wheel})
        self.assertAlmostEqual(math.hypot(*waypoint[:2]),2.)
        self.assertGreater(waypoint[1],0.)
        self.assertEqual(wheel_approach([.6,-2,.18],[0,0,.18],{'fr_wheel':wheel}),wheel)

    def test_inverted_drive_preserves_forward_and_steering_intent(self):
        for up in (-1,1):
            left,right = orient_drive(4,9,up)
            self.assertEqual(up*(left+right),13)
            self.assertEqual(right-left,5)

    def test_free_solid_does_not_rewrite_child_transform(self):
        exported = '''DEF OLD Solid {
  name "part"
  children [
    Pose {
      translation 8 9 10
      rotation 1 0 0 1.2
      children [ Shape { geometry Box { size 1 1 1 } } ]
    }
  ]
  boundingObject Box { size 1 1 1 }
  physics Physics { mass 2 density -1 }
}'''
        result = free_solid(exported,'DEBRIS',[1,2,3],[0,0,1,.7])
        self.assertIn('DEF DEBRIS Solid',result)
        self.assertIn('translation 1 2 3',result)
        self.assertIn('translation 8 9 10',result)
        self.assertIn('rotation 1 0 0 1.2',result)
        self.assertNotIn('name "part"',result)
        self.assertIn('physics Physics',result)

    def test_export_rejects_robot(self):
        with self.assertRaises(ValueError):
            free_solid('Robot { }','DEBRIS',[0]*3,[0,0,1,0])

    def test_detached_material_is_self_contained(self):
        source = 'Solid {\n children [ Shape { appearance USE STEEL geometry Box { size 1 1 1 } } ]\n}'
        result = free_solid(source,'DEBRIS',[0]*3,[0,0,1,0],
                            {'STEEL':'DEF STEEL PBRAppearance { baseColor 0.6 0.7 0.8 metalness 1 }'})
        self.assertNotIn('USE STEEL',result)
        self.assertNotIn('DEF STEEL',result)
        self.assertIn('baseColor 0.6 0.7 0.8',result)

    def test_half_turn_preserves_axis(self):
        aa = matrix_axis_angle([1,0,0,0,-1,0,0,0,-1])
        self.assertEqual(aa[:3],[1,0,0])
        self.assertAlmostEqual(aa[3],math.pi)

    def test_world_matches_generator_and_has_real_part_colliders(self):
        spec = importlib.util.spec_from_file_location('foundry_build',ORC/'build_foundry.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        world = (ORC/'worlds/orc_foundry.omniworld').read_text(encoding='utf-8')
        self.assertEqual(world,module.build())
        self.assertEqual(world.count('RotationalMotor {'),0)
        self.assertEqual(world.count('PositionSensor {'),10)
        self.assertEqual(world.count('inertiaMatrix ['),12)
        self.assertEqual(world.count('physics Physics { density -1 mass'),12)
        self.assertIn('newtonCompoundColliders TRUE',world)

    def test_horizontal_weapon_clears_own_body_and_can_strike_low_armor(self):
        world = (ORC/'worlds/orc_foundry.omniworld').read_text(encoding='utf-8')
        anchor = re.search(r'DEF ANVIL_WEAPON_JOINT HingeJoint\s*\{.*?anchor\s+([\d. -]+)',world,re.S)
        x,y,z = map(float,anchor.group(1).split())
        tooth_radius = math.hypot(.45+.14/2,.22/2)
        self.assertGreater(x-tooth_radius,.56)
        self.assertLess(z+.09/2,.15)  # overlaps another chassis's vertical span
        self.assertGreater(.19+z-.09/2,.01)  # above the floor

    def test_cylinder_collision_mesh_is_closed_and_keeps_tire_dimensions(self):
        spec = importlib.util.spec_from_file_location('foundry_mesh_build',ORC/'build_foundry.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        mesh = module.cylinder_collider(.17,.122)
        points = [list(map(float,p.split())) for p in re.search(r'point \[([^]]+)\]',mesh).group(1).split(',')]
        self.assertAlmostEqual(min(p[2] for p in points),-.061)
        self.assertAlmostEqual(max(p[2] for p in points),.061)
        self.assertTrue(all(abs(math.hypot(*p[:2])-.17)<1e-6 for p in points))
        faces = [[int(i) for i in f.split()] for f in re.search(r'coordIndex \[([^]]+)\]',mesh).group(1).split('-1') if f.strip()]
        edges = [(a,b) for face in faces for a,b in zip(face,face[1:]+face[:1])]
        self.assertEqual(len(edges),len(set(edges)))
        self.assertTrue(all((b,a) in edges for a,b in edges))

    def test_both_victims_share_one_absorbed_energy_budget(self):
        combat = Combat()
        load = {'absorbed_j':1000,'estimated_peak_n':100000}
        events = [combat.hit(1,'ANVIL','fl_wheel','RAZOR','weapon',load),
                  combat.hit(1,'RAZOR','weapon','ANVIL','fl_wheel',load)]
        self.assertLessEqual(sum(e['plastic_work_j'] for e in events if e),1000)


class FoundryPhysicalModelTests(unittest.TestCase):
    def test_motor_matches_published_operating_point(self):
        motor = Motor(1,1000,efficiency=1)
        point = motor.evaluate(1,5500*math.tau/60)
        # AmpFlow A28-150: 5500 RPM, 40 A, 190 oz-in (1.342 Nm), rounded.
        self.assertAlmostEqual(point['current_a'],40,delta=1.5)
        self.assertAlmostEqual(point['torque_nm'],190*.0070615518,delta=.045)
        stopped = motor.evaluate(1,0)
        self.assertAlmostEqual(stopped['torque_nm'],1950*.0070615518,delta=.25)

    def test_current_limit_reverse_and_braking_are_passive(self):
        for duty in (-1,0,1):
            for omega in (-90,-10,0,10,90):
                result = DRIVE.evaluate(duty,omega)
                self.assertLessEqual(abs(result['current_a']),40)
                self.assertLessEqual(result['mechanical_w'],result['electrical_w']+1e-9)
                if duty == 0:
                    self.assertLessEqual(result['mechanical_w'],0)

    def test_no_load_speed_and_coast(self):
        motor = WEAPON['ANVIL']
        omega = (24-motor.no_load_current*motor.resistance)/(motor.k*motor.reduction)
        self.assertAlmostEqual(motor.evaluate(1,omega)['torque_nm'],0,places=8)
        coast = motor.evaluate(0,omega,coast=True)
        self.assertEqual(coast['current_a'],0)
        self.assertLess(coast['mechanical_w'],0)

    def test_shared_pack_sags_and_never_recharges_from_braking(self):
        requests = {str(i):(DRIVE,1,10,False) for i in range(4)}
        values,pack = powertrain(requests,.008,10000)
        self.assertLess(pack['voltage_v'],24)
        self.assertLess(pack['remaining_j'],10000)
        self.assertAlmostEqual(pack['voltage_v'],24-.012*pack['current_a'],places=4)
        _,brake = powertrain({'a':(DRIVE,0,10,False)},.008,pack['remaining_j'])
        self.assertLessEqual(brake['remaining_j'],pack['remaining_j'])

    def test_empty_battery_cannot_power_a_motor(self):
        values,pack = powertrain({'a':(DRIVE,1,10,False)},.008,0)
        self.assertEqual(pack['remaining_j'],0)
        self.assertLessEqual(values['a']['mechanical_w'],0)

    def test_grazing_contact_has_no_normal_impact_energy(self):
        rotation = [1,0,0,0,1,0,0,0,1]
        a = ('ANVIL','chassis',[0,0,0],[0,4,0,0,0,0],rotation)
        b = ('RAZOR','chassis',[1,0,0],[0]*6,rotation)
        load = contact_load(a,b,[.5,0,0],[-1,0,0],{'ANVIL':67.6,'RAZOR':67.6})
        self.assertEqual(load['incident_j'],0)

    def test_head_on_collision_has_reduced_mass_energy(self):
        rotation = [1,0,0,0,1,0,0,0,1]
        a = ('ANVIL','chassis',[0,0,0],[4,0,0,0,0,0],rotation)
        b = ('RAZOR','chassis',[1,0,0],[-4,0,0,0,0,0],rotation)
        load = contact_load(a,b,[.5,0,0],[-1,0,0],{'ANVIL':67.6,'RAZOR':67.6})
        self.assertAlmostEqual(load['effective_mass_kg'],33.8)
        self.assertAlmostEqual(load['incident_j'],.5*33.8*8**2)

    def test_authored_inertias_are_physically_valid(self):
        for name in ('ANVIL','RAZOR'):
            for part in ('chassis','fl_wheel','weapon'):
                inertia = part_spec(name,part).inertia
                self.assertGreater(min(inertia),0)
                self.assertLessEqual(max(inertia),sum(inertia)-max(inertia)+1e-10)


class FoundryMatchVerificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0,str(ORC))
        from verify_foundry import assess
        cls.assess = staticmethod(assess)

    def recording(self):
        identity = [1,0,0,0,1,0,0,0,1]
        samples = [{'type':'sample','t':2.04+i*.24,'phase':'combat',
                    'bots':{'ANVIL':{'position':[0,0,.18],'rotation':identity,
                                    'pilot':{'t':2.04+i*.24,
                                             'powertrain':{'drive_request':[25,25],'live':True}}},
                            'RAZOR':{'position':[3.5+i*.02,0,.18],'rotation':identity}},
                    'mobility':{'ANVIL':{'opponent_contact':False}}} for i in range(42)]
        return {'case':'match','initial_mass_kg':131.2,'final_audit':{'mass_kg':131.2},
                'events':[{'type':'impact','t':.8,'assembly_loss_budget_j':100,'plastic_work_j':20},
                          {'type':'detach','t':1,'robot':'ANVIL','part':'fl_wheel'}]+samples+
                         [{'type':'immobilized','t':12.008,'robot':'ANVIL','mobility':{'stationary_s':10}},
                          {'type':'result','t':12.008}]}

    def test_independent_recording_can_establish_a_powered_countout(self):
        self.assertTrue(self.assess(self.recording())['passed'])

    def test_time_limit_or_damage_score_is_not_a_physical_knockout(self):
        data = self.recording()
        data['events'] = [e for e in data['events'] if e['type'] != 'immobilized']
        self.assertFalse(self.assess(data)['passed'])

    def test_motor_shutdown_motion_and_repeated_energy_spending_fail(self):
        original = self.recording()
        for failure in ('shutdown','stale_command','motion','energy','floor_penetration'):
            data = copy.deepcopy(original)
            samples = [e for e in data['events'] if e['type'] == 'sample']
            if failure == 'shutdown':
                samples[20]['bots']['ANVIL']['pilot']['powertrain']['live'] = False
            elif failure == 'stale_command':
                samples[20]['bots']['ANVIL']['pilot']['t'] -= .5
            elif failure == 'motion':
                samples[20]['bots']['ANVIL']['position'][0] = .3
            elif failure == 'energy':
                data['events'][0]['plastic_work_j'] = 101
            else:
                samples[20]['bots']['ANVIL']['position'][2] = -.1
            self.assertFalse(self.assess(data)['passed'],failure)


class FoundryInputTests(unittest.TestCase):
    def game(self,keys=(),relay=None):
        game = director_module.Foundry.__new__(director_module.Foundry)
        game.s = MagicMock()
        game.s.getTime.return_value = 10
        game.keyboard = MagicMock()
        game.keyboard.getKey.side_effect = list(keys)+[-1]
        game.pilot_inputs = []
        if relay is not None:
            field = MagicMock()
            field.getSFString.return_value = relay
            game.pilot_inputs.append(field)
        game.keys_before = set()
        game.mode = 'manual'
        game.phase = 'ready'
        game.hud = True
        game.camera_mode = 0
        game.weapon_on = True
        return game

    def test_native_return_and_enter_start_countdown(self):
        for key in (4,5,13):
            with self.subTest(key=key):
                game = self.game([key])
                game.input()
                self.assertEqual(game.phase,'countdown')
                self.assertEqual(game.start,13)

    def test_native_tab_starts_autopilot(self):
        game = self.game([1])
        game.input()
        self.assertEqual(game.mode,'demo')
        self.assertEqual(game.phase,'countdown')

    def test_selected_robot_relays_keys_and_toggle_is_debounced(self):
        game = self.game(relay='{"keys":[87,65,32],"t":10}')
        self.assertEqual(game.input(),{'throttle':1,'steer':1,'weapon':False})
        game.keyboard.getKey.side_effect = [-1]
        self.assertFalse(game.input()['weapon'])

    def test_stale_input_does_not_latch_throttle(self):
        game = self.game(relay='{"keys":[87],"t":9}')
        self.assertEqual(game.input()['throttle'],0)

    def test_restart_reloads_complete_world(self):
        game = self.game([ord('R')])
        self.assertIsNone(game.input())
        game.s.worldReload.assert_called_once()


if __name__ == '__main__':
    unittest.main()
