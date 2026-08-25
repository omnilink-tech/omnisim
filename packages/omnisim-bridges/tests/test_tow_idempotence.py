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

"""An ordinary chat message must not be able to wedge the line.

Measured in an adversarial audit: a "stop" landed 3 s after the return tug
attached TROLLEY_C. On resume the idle loop correctly noticed `carrying` and
called _tow_to() to stage the cart -- but _tow_to() begins with _dock(),
_dock() asked the bridge to attach a cart it was ALREADY towing, the bridge
refused with `already_towing`, _dock() returned False, and _tow_to() bailed
BEFORE reaching the detach. So `carrying` stayed set and the next cycle did
exactly the same thing: 131 iterations over ~21 minutes, the fill station
starved, and the arm did zero picks for the entire unattended window.

It was invisible while it happened -- `fault: null`, `mode: "drive"`, and
`jobs_total` CLIMBING, because every iteration reached a task boundary. The
robot reported itself busy and healthy while achieving nothing.

This models the loop rather than importing the controller (which needs a live
simulator), and asserts the property that breaks the cycle.
"""

from __future__ import annotations

import threading


class FakeTug:
    """The two decisions that mattered, and nothing else."""

    def __init__(self, idempotent: bool, dock_short_circuits: bool):
        self.carrying = None
        self.lock = threading.Lock()
        self.idempotent = idempotent
        self.dock_short_circuits = dock_short_circuits

    def attach(self, target: str) -> dict:
        with self.lock:
            if self.carrying is not None:
                if self.idempotent and self.carrying == target:
                    return {"ok": True, "carrying": self.carrying,
                            "already_attached": True}
                return {"error": "already_towing", "carrying": self.carrying}
            self.carrying = target
            return {"ok": True}

    def dock(self, target: str) -> bool:
        if self.dock_short_circuits and self.carrying == target:
            return True
        return "error" not in self.attach(target)

    def cycle_return(self, limit: int = 200) -> int:
        """Iterations spent before the tug frees itself. `limit` == wedged."""
        n = 0
        while self.carrying and n < limit:
            n += 1
            if not self.dock(self.carrying):
                continue                    # _tow_to bails before the detach
            self.carrying = None            # act_detach_trolley
        return n


def test_the_livelock_reproduces_without_the_fix() -> None:
    """Guards the test itself: if this ever passes trivially, the model has
    drifted from the bug and the fix below proves nothing."""
    tug = FakeTug(idempotent=False, dock_short_circuits=False)
    tug.carrying = "TROLLEY_C"
    assert tug.cycle_return() == 200, "expected the wedge"
    assert tug.carrying == "TROLLEY_C", "expected it never to clear"


def test_idempotent_attach_breaks_the_livelock() -> None:
    tug = FakeTug(idempotent=True, dock_short_circuits=False)
    tug.carrying = "TROLLEY_C"
    assert tug.cycle_return() == 1
    assert tug.carrying is None


def test_dock_short_circuit_breaks_it_too() -> None:
    """Belt and braces: either fix alone is sufficient, so a regression in
    one does not resurrect the stall."""
    tug = FakeTug(idempotent=False, dock_short_circuits=True)
    tug.carrying = "TROLLEY_C"
    assert tug.cycle_return() == 1
    assert tug.carrying is None


def test_attaching_a_different_cart_while_towing_still_fails() -> None:
    """Idempotence must not become 'anything goes'. Asking for a cart you
    cannot take is a real error and must stay one."""
    tug = FakeTug(idempotent=True, dock_short_circuits=False)
    tug.carrying = "TROLLEY_C"
    res = tug.attach("TROLLEY_F")
    assert res.get("error") == "already_towing"
    assert res["carrying"] == "TROLLEY_C"


def test_normal_attach_is_unaffected() -> None:
    tug = FakeTug(idempotent=True, dock_short_circuits=True)
    assert tug.attach("TROLLEY_C").get("ok") is True
    assert tug.carrying == "TROLLEY_C"
