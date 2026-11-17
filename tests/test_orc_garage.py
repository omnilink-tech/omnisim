# Copyright 2026 OmniLink. SPDX-License-Identifier: Apache-2.0
"""Blueprint/physics agreement and the local garage's launch boundary."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
ORC = ROOT/'projects/robot_combat/orc'
sys.path.insert(0,str(ORC))
sys.path.insert(0,str(ORC/'shared'))
from foundry_config import normalize_match, build_stats
from foundry_physics import part_spec, drive_motor, weapon_motor, body_record
from foundry_strategy import tactic, clinch_timing
from build_foundry import build
from garage import Garage, make_server


class BlueprintTests(unittest.TestCase):
    def test_same_weapon_on_either_side_uses_same_physics(self):
        for weapon in ('bar','drum'):
            match = normalize_match()
            for robot in match['robots'].values():
                robot.update(weapon=weapon,frame='reinforced',rotor_kg=10,gearing='torque')
            a,b = [part_spec(slot,'weapon',match['robots']) for slot in ('ANVIL','RAZOR')]
            self.assertEqual(a,b)
            self.assertEqual(weapon_motor(match['robots']['ANVIL']),weapon_motor(match['robots']['RAZOR']))
            world = build(match,garage=True)
            self.assertEqual(world.count('mass 60 '),2)
            self.assertEqual(world.count('mass 10 '),2)
            self.assertEqual(world.count('anchor .79 0 .07'),2 if weapon == 'drum' else 0)
            self.assertEqual(world.count('anchor 1.12 0 .03'),2 if weapon == 'bar' else 0)

    def test_stats_and_conservation_audit_use_authored_mass(self):
        match = normalize_match()
        match['robots']['ANVIL'].update(frame='light',rotor_kg=6,weapon='drum')
        identity = [1,0,0,0,1,0,0,0,1]
        total = sum(body_record('ANVIL',part,[0,0,0],[2,0,0,0,0,0],identity,match['robots'])['mass_kg']
                    for part in ('chassis','fl_wheel','fr_wheel','rl_wheel','rr_wheel','weapon'))
        self.assertAlmostEqual(total,build_stats(match['robots']['ANVIL'])['mass_kg'])
        self.assertAlmostEqual(total,51.6)

    def test_gearing_exchanges_torque_for_speed(self):
        robot = normalize_match()['robots']['ANVIL']
        robot['gearing'] = 'torque'
        torque = build_stats(robot)
        robot['gearing'] = 'speed'
        speed = build_stats(robot)
        self.assertGreater(torque['wheel_stall_torque_nm'],speed['wheel_stall_torque_nm'])
        self.assertLess(torque['drive_speed_m_s'],speed['drive_speed_m_s'])

    def test_heavy_rotor_increases_inertia_and_free_speed_energy(self):
        robot = normalize_match()['robots']['ANVIL']
        robot['rotor_kg'] = 6
        light = build_stats(robot)
        robot['rotor_kg'] = 10
        heavy = build_stats(robot)
        self.assertGreater(heavy['weapon_energy_j'],light['weapon_energy_j'])
        self.assertEqual(heavy['weapon_rpm'],light['weapon_rpm'])
        self.assertAlmostEqual(heavy['mass_kg']-light['mass_kg'],4)

    def test_frame_strength_has_a_mass_and_inertia_cost(self):
        match = normalize_match()
        match['robots']['ANVIL']['frame'] = 'light'
        light = part_spec('ANVIL','chassis',match['robots'])
        shaft_light = part_spec('ANVIL','fl_wheel',match['robots'])
        match['robots']['ANVIL']['frame'] = 'reinforced'
        heavy = part_spec('ANVIL','chassis',match['robots'])
        shaft_heavy = part_spec('ANVIL','fl_wheel',match['robots'])
        self.assertGreater(heavy.mass,light.mass)
        self.assertTrue(all(a>b for a,b in zip(heavy.inertia,light.inertia)))
        self.assertAlmostEqual(shaft_heavy.yield_force/shaft_light.yield_force,(1.1/.9)**3)

    def test_invalid_or_executable_blueprints_are_rejected(self):
        for key,value in (('name','bad" Robot {'),('weapon','laser'),('rotor_kg',float('nan')),
                          ('rotor_kg',True),('strategy','__import__("os")'),('frame','unlimited')):
            match = normalize_match()
            match['robots']['ANVIL'][key] = value
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                normalize_match(match)
        match = normalize_match()
        match['command'] = 'anything'
        with self.assertRaises(ValueError):
            normalize_match(match)
        match = normalize_match()
        match['robots']['RAZOR']['name'] = 'anvil'
        with self.assertRaises(ValueError):
            normalize_match(match)

    def test_styles_make_different_driving_decisions(self):
        args = ([0,0,.2],0,[2,0,.2],{'fl_wheel':[2,.5,.2]},.1)
        pressure = tactic('pressure',*args)
        counter = tactic('counter',*args)
        flanker = tactic('flanker',*args)
        self.assertGreater(sum(pressure),0)
        self.assertNotEqual(counter,pressure)
        self.assertNotEqual(flanker,pressure)
        self.assertGreater(clinch_timing('counter')[1],clinch_timing('pressure')[1])


class GarageTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.garage = Garage(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def test_blueprints_survive_reopening_garage(self):
        match = normalize_match()
        match['robots']['ANVIL']['name'] = 'forge 7'
        self.garage.save(match)
        self.assertEqual(Garage(self.directory.name).draft['robots']['ANVIL']['name'],'FORGE 7')

    def test_two_start_requests_cannot_launch_two_engines(self):
        with patch('garage.threading.Thread') as thread:
            self.garage.start(normalize_match())
            with self.assertRaises(RuntimeError):
                self.garage.start(normalize_match())
            self.assertEqual(thread.call_count,1)

    def test_stop_targets_only_the_owned_guard(self):
        self.garage.current = {'status':'running','cancelled':False}
        self.garage.process = MagicMock()
        self.garage.process.poll.return_value = None
        with patch('garage.os.name','nt'):
            self.garage.stop()
        self.garage.process.terminate.assert_called_once_with()
        self.assertEqual(self.garage.current['status'],'stopping')

    def test_local_http_validates_origin_token_and_blueprint(self):
        server = make_server(self.garage)
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        def post(payload,headers):
            client = http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=3)
            client.request('POST','/api/preview',json.dumps(payload),headers)
            response = client.getresponse()
            code,data = response.status,json.loads(response.read())
            client.close()
            return code,data
        try:
            headers = {'Content-Type':'application/json','X-ORC-Token':self.garage.token}
            code,data = post(normalize_match(),headers)
            self.assertEqual(code,200)
            self.assertAlmostEqual(data['stats']['ANVIL']['mass_kg'],65.6)
            self.assertEqual(post(normalize_match(),{'Content-Type':'application/json'})[0],403)
            self.assertEqual(post(normalize_match(),{**headers,'Origin':'https://untrusted.example'})[0],403)
            self.assertEqual(post({'robots':{}},headers)[0],400)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
