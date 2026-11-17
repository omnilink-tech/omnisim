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
"""A round's trace belongs to the thread that ran the round.

The main worker and the aside workers run model rounds at the same time.
With one shared slot, a round could log a concurrent round's fresh record
(attempts 0, no usage), and the cost estimate charged it as unknown at the
most expensive request (measured 2026-10-20 in an opening-barrage probe).
"""
import threading

from omnisim_bridges.relay import OmniLinkRelay


class _Holder:
    pass


def test_concurrent_rounds_each_read_their_own_trace():
    relay = _Holder()
    set_trace = OmniLinkRelay._set_round_trace
    get_trace = OmniLinkRelay._get_round_trace
    a_set = threading.Event()
    b_set = threading.Event()
    seen = {}

    def round_a():
        set_trace(relay, {"round": "a", "prompt_tokens": 100})
        a_set.set()
        b_set.wait(5)  # B overwrites the shared slot meanwhile
        seen["a"] = get_trace(relay)

    def round_b():
        a_set.wait(5)
        set_trace(relay, {"round": "b", "attempts": 0})
        b_set.set()
        seen["b"] = get_trace(relay)

    threads = [threading.Thread(target=round_a), threading.Thread(target=round_b)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert seen["a"] == {"round": "a", "prompt_tokens": 100}
    assert seen["b"] == {"round": "b", "attempts": 0}
