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
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from compartment import spring_force
from evaluate import evaluate_insertion, FINGERS
from kinematics import Arm, rotation, rotvec, mm, transpose, norm, sub

I=[1,0,0,0,1,0,0,0,1]
BX=[0,0,1,0,1,0,-1,0,0]


class InsertionTests(unittest.TestCase):
    def successful_sequence(self):
        tilted=[v for row in rotation([0,math.radians(65),0]) for v in row]
        rows=[]
        for k in range(50):
            contacts=list(FINGERS) if k<25 else []
            contacts+=['negative_terminal','positive_terminal']
            rows.append(dict(frame=k,contacts=contacts,spring_compression_m=.006,
                             negative_contact_max_depth_m=.0003,
                             housing_contact_max_depth_m=.0002,
                             floor_contact_points=[[.02,-.0095,.006]],
                             remote_rotation=I,battery_rotation=tilted if k<10 else BX,
                             battery_in_remote=[.019,-.0095,.0128]))
        result=dict(task_success=True,carried=True,inside_slot=True,released=True,settled=True)
        return rows,result

    def test_sequence_requires_compress_then_seat_before_release(self):
        rows,result=self.successful_sequence()
        self.assertTrue(evaluate_insertion(rows,result,50)['task_success'])
        for row in rows:row['battery_rotation']=BX
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])

    def test_terminal_contacts_do_not_replace_full_containment(self):
        rows,result=self.successful_sequence()
        for row in rows[25:]:row['battery_in_remote'][2]=.017
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])

    def test_gravity_drop_and_regrasp_cannot_replace_seating(self):
        rows,result=self.successful_sequence()
        for row in rows[10:25]:row['contacts']=['negative_terminal','positive_terminal']
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])

    def test_release_then_fingertip_press_can_seat(self):
        rows,result=self.successful_sequence()
        for row in rows[10:25]:
            row['contacts']=['negative_terminal']
            row['battery_in_remote'][2]=.022
        for k in range(18,25):
            rows[k]['stage']='press' if k<24 else 'press_withdraw'
            rows[k]['contacts']+=list(FINGERS)
            rows[k]['battery_in_remote'][2]=.022-(k-18)*(.0092/6)
        report=evaluate_insertion(rows,result,50)
        self.assertTrue(report['pressed_to_seat'])
        self.assertTrue(report['task_success'])

    def test_missing_terminal_and_unfinished_runs_fail(self):
        rows,result=self.successful_sequence()
        rows[-1]['contacts'].remove('positive_terminal')
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])
        self.assertFalse(evaluate_insertion([],result,50)['task_success'])

    def test_penetrating_or_unmeasured_contacts_fail(self):
        rows,result=self.successful_sequence()
        rows[12]['negative_contact_max_depth_m']=.004
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])
        del rows[12]['negative_contact_max_depth_m']
        self.assertFalse(evaluate_insertion(rows,result,50)['task_success'])

    def test_spring_load_equilibrium_and_damping_sign(self):
        self.assertAlmostEqual(spring_force(-.006,0),.75)
        self.assertGreater(spring_force(-.006,-.01),.75)
        self.assertLess(spring_force(-.006,.01),.75)

    def test_cartesian_solver_moves_a_held_tool(self):
        arm=Arm()
        for q in ([0,.2,.2,0,1.2,0],[.1,.5,-.3,.05,1.6,-.1]):
            p,R,*_=arm.fk(q)
            target=[p[0]-.005,p[1]+.003,p[2]+.002]
            target_R=mm(rotation([0,.015,0]),R)
            actual=arm.solve(target,target_R,q)
            p2,R2,*_=arm.fk(actual)
            self.assertLess(norm(sub(target,p2)),.0001)
            self.assertLess(norm(rotvec(mm(target_R,transpose(R2)))),.001)


if __name__=='__main__':unittest.main()
