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
# limitations under the License.import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate import cylinder_in_slot, evaluate
from prepare_episode import convert

IDENTITY = [1,0,0,0,1,0,0,0,1]
CYLINDER_X = [0,0,1,0,1,0,-1,0,0]


class EvidenceTests(unittest.TestCase):
    def test_cylinder_requires_full_containment(self):
        self.assertTrue(cylinder_in_slot([.0175,-.00925,.0128], CYLINDER_X, IDENTITY))
        self.assertFalse(cylinder_in_slot([.0175,-.003,.0128], CYLINDER_X, IDENTITY))
        self.assertFalse(cylinder_in_slot([.0175,-.00925,.0128], IDENTITY, IDENTITY))
        self.assertFalse(cylinder_in_slot([.021,-.00925,.0128], CYLINDER_X, IDENTITY))

    def test_moving_remote_frame(self):
        yaw90 = [0,-1,0,1,0,0,0,0,1]
        cylinder_y = [0,-1,0,0,0,1,-1,0,0]
        self.assertTrue(cylinder_in_slot([.0175,-.00925,.0128], cylinder_y, yaw90))

    def test_toss_and_unilateral_touch_cannot_pass(self):
        final = {'final_battery_in_remote':[.0175,-.00925,.0128],
                 'final_battery_rotation':CYLINDER_X,'final_remote_rotation':IDENTITY,
                 'final_battery_velocity':[0]*6}
        row = {'contacts':['right_left_finger_link'], 'battery_position':[0,0,.10],
               'battery_in_gripper':[.15,0,0]}
        rows = [copy.deepcopy(row) for _ in range(100)]
        for k,r in enumerate(rows):r['battery_position'][0]=k*.0012
        rows.extend({'contacts':[], 'battery_position':[0,0,.013]} for _ in range(25))
        self.assertFalse(evaluate(rows, final, 50, .0068)['task_success'])
        for r in rows[:100]:
            r['contacts'].append('right_right_finger_link')
        self.assertTrue(evaluate(rows, final, 50, .0068)['task_success'])
        rows[50]['battery_in_gripper'] = [.18,0,0]
        result = evaluate(rows, final, 50, .0068)
        self.assertTrue(result['task_success'])
        self.assertFalse(result['strict_grip_stability_passed'])

    def test_gripper_mapping_and_invalid_input(self):
        row=[0]*14
        self.assertAlmostEqual(convert(row)[6], .01844)
        row[13]=1
        self.assertAlmostEqual(convert(row)[14], .058)
        row[13]=float('nan')
        with self.assertRaises(ValueError):convert(row)


if __name__ == '__main__':unittest.main()
