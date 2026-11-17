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
"""Record-and-replay of model replies for the dev loop."""
import pytest

from omnisim_bridges import model_replay as mr


@pytest.fixture(autouse=True)
def _reset():
    mr._reset_for_tests()
    yield
    mr._reset_for_tests()


SI = {"mainTask": "Drive the cart.", "availableToolDetails": [{"name": "drive_forward"}]}


def test_off_unless_a_path_is_set():
    assert not mr.active()
    assert mr.lookup("anything") is None


def test_measured_numbers_and_live_context_do_not_break_the_match():
    a = mr.key([{"role": "tool", "content": "Drove 0.998 m"}], {**SI, "liveContext": "Odometer 12.4 m"}, "g4-engine", "m")
    b = mr.key([{"role": "tool", "content": "Drove 1.002 m"}], {**SI, "liveContext": "Odometer 13.9 m"}, "g4-engine", "m")
    c = mr.key([{"role": "tool", "content": "Drove 2.300 m"}], SI, "g4-engine", "m")
    assert a == b
    assert a != c
    assert mr.key([], SI, "g4-engine", "m") != mr.key([], SI, "g4-engine", "other-model")


def test_record_then_replay_serves_each_reply_once_in_order(tmp_path):
    path = tmp_path / "replies.jsonl"
    mr._reset_for_tests(record_path=str(path))
    k = mr.key([{"role": "user", "content": "hi"}], SI, "g4-engine", "m")
    mr.record(k, {"text": "first"})
    mr.record(k, {"text": "second"})
    mr._reset_for_tests(replay_path=str(path))
    assert mr.lookup(k) == {"text": "first"}
    assert mr.lookup(k) == {"text": "second"}
    assert mr.lookup(k) is None          # a miss goes to the live model
