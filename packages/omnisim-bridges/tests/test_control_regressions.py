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
"""Product regressions found by the robotics-control development benchmark."""
import math

import pytest
from omnisim_bridges import gate, route


def frame(tool='drive_forward', **args):
    return {'tool':tool,'args':args}


@pytest.mark.parametrize('text',[
    'Back up a bit.', 'Move forward a little.', 'Drive a few metres.',
    'Nudge forward a touch.', 'Reverse a tad.', 'Go forward slightly.',
])
def test_vague_distance_asks_and_cannot_act_even_if_model_guesses(text):
    plan=route.parser_first_plan(text,'mobile')
    assert plan is not None and plan.ask and not plan.frames
    assert 'invented_magnitude' in {r.rule for r in gate.check(text,[frame(distance=-.5)])}


@pytest.mark.parametrize('text',[
    'Drive forward 0.4 metres. Never repeat a successful movement.',
    'Move the joints to [0.1, 0.2, 0.7, 0, 2.2, 0]. Do not repeat a successful motion.',
    'Drive 0.4 metres. If temporarily unavailable before any motion, retry once. Never repeat a successful movement.',
])
def test_retry_qualification_does_not_ban_first_attempt(text,monkeypatch):
    action=frame(distance=.4)
    if text.startswith('Move the joints'):
        monkeypatch.setitem(gate.SPECS,'set_joint_positions',{
            'physical':True,'tier':gate.GUARDED,'args':{'q':list},'required':('q',)})
        action=frame('set_joint_positions',q=[.1,.2,.7,0,2.2,0])
    assert not gate.check(text,[action])


@pytest.mark.parametrize('text',[
    'Never repeat a successful movement.',
    'I have 2 boxes. Never repeat a successful movement.',
    'Do not drive 0.4 metres. Never repeat a successful movement.',
    'Drive 0.4 metres. Never repeat a successful movement and do not move.',
    'Drive 0.4 metres. Never repeat a successful movement. Do not drive.',
    'Drive 0.4 metres. Do not repeat the movement.',
])
def test_other_prohibitions_remain_blocked(text):
    assert 'prohibition' in {r.rule for r in gate.check(text,[frame(distance=.4)])}


def test_no_turning_allows_straight_drive_but_blocks_rotation():
    text='Now drive forward 0.25 metres. Then reverse 0.25 metres without turning.'
    assert not gate.check(text,[frame(distance=.25),frame(distance=-.25)])
    assert 'self_negating' in {r.rule for r in gate.check(text,[frame('turn',angle_rad=math.pi/2)])}


@pytest.mark.parametrize('text',[
    'Drive 1 metre without turning or moving.',
    'Drive 1 metre without moving.',
    'Drive 1 metre without turning. Do not move.',
])
def test_other_no_motion_constraints_are_not_erased(text):
    assert gate.check(text,[frame(distance=1)])


def test_retry_cannot_be_compiled_into_adjacent_unconditional_duplicates():
    text='Drive 0.4 metres. Never repeat a successful movement.'
    assert 'duplicate_motion' in {r.rule for r in gate.check(text,[frame(distance=.4),frame(distance=.4)])}
    # Equal drives separated by a turn are distinct legs, not duplicate retries.
    text='Drive 0.4 metres, turn left 90 degrees, drive 0.4 metres. Never repeat a successful movement.'
    assert not gate.check(text,[frame(distance=.4),frame('turn',angle_rad=math.pi/2),frame(distance=.4)])


def test_typed_calls_and_explicit_distances_still_work():
    for text in ('','Back up 0.5 metres.','Reverse half a metre.'):
        assert not gate.check(text,[frame(distance=-.5)])


@pytest.mark.parametrize('text',[
    'Robot 2, back up a bit.',
    'Back up a bit two times.',
    'Drive 0.4 metres, then back up a bit.',
    'Drive forward a little after checking sensor 3.',
])
def test_unrelated_numbers_do_not_supply_a_vague_distance(text):
    assert 'invented_magnitude' in {r.rule for r in gate.check(text,[frame(distance=-.5)])}


def test_vague_word_with_an_explicit_local_distance_can_be_executed():
    assert not gate.check('Back up a little, specifically 0.5 metres.',[frame(distance=-.5)])


def test_unrelated_vague_aside_does_not_cancel_a_precise_command():
    assert not gate.check('Drive 0.4 metres. I have a few questions for later.',[frame(distance=.4)])
