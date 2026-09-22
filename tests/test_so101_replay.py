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

"""Engine-free checks for SO101 replay's scientific validity gates."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('so101_replay_support', ROOT / 'projects/robots/therobotstudio/replay_support.py')
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)


class ReplayTests(unittest.TestCase):
    def calibration(self, units='degrees'):
        return {'units': units, 'status': 'measured',
                'joints': {j: {'scale_deg_per_unit': 1, 'offset_deg': 0} for j in support.JOINTS}}

    def test_gripper_is_normalised_even_when_body_is_degrees(self):
        c = self.calibration()
        c['joints']['gripper'] = {'scale_deg_per_unit': .8, 'offset_deg': -4}
        result = support.convert([10, 20, 30, 40, 50, 25], c)
        self.assertAlmostEqual(result[0], math.radians(10))
        self.assertAlmostEqual(result[5], math.radians(16))

    def test_missing_units_and_nan_are_rejected(self):
        c = self.calibration()
        c.pop('units')
        with self.assertRaises(ValueError):
            support.convert([0] * 6, c)
        with self.assertRaises(ValueError):
            support.convert([float('nan')] * 6, self.calibration())

    def test_limits_are_not_silently_clamped(self):
        with self.assertRaises(ValueError):
            support.validate_targets([[0, 0, 0, 2.2, 0, 0]])

    def row(self, contacts, relative=(0, 0, -.08), z=.1):
        return {'cube_position': [0, 0, z], 'cube_contacts': contacts, 'cube_in_gripper': relative}

    def test_toss_and_palm_rest_do_not_count_as_grasp(self):
        for contacts in ([], ['gripper_link'], ['gripper_link', 'moving_jaw_so101_v1_link', 'table']):
            self.assertFalse(support.grasp_evidence([self.row(contacts)] * 60, .02, 30)['carried'])

    def test_stable_bilateral_carry_passes_and_slip_fails(self):
        both = ['gripper_link', 'moving_jaw_so101_v1_link']
        trace = [self.row(both)] * 60
        self.assertTrue(support.grasp_evidence(trace, .02, 30)['carried'])
        trace[-1] = self.row(both, relative=(.03, 0, -.08))
        self.assertFalse(support.grasp_evidence(trace, .02, 30)['carried'])

    def test_box_contains_entire_rotated_cube(self):
        identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.assertTrue(support.contained([0, 0, .024], identity, [0, 0, 0]))
        self.assertFalse(support.contained([.025, 0, .024], identity, [0, 0, 0]))
        c = math.sqrt(.5)
        rotation = [c, -c, 0, c, c, 0, 0, 0, 1]
        self.assertFalse(support.contained([.01, 0, .024], rotation, [0, 0, 0]))


if __name__ == '__main__':
    unittest.main()
