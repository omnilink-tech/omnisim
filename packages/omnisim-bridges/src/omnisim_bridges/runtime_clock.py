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
"""One clock for everything the OmniLink robot runtime schedules.

By default this IS the wall clock: `now()` is `time.time()`, `monotonic()`
is `time.monotonic()`, `sleep()` is `time.sleep()`. Nothing changes for a
real robot or a normal demo.

With OMNILINK_CLOCK=sim (benchmarks only), simulator time drives it, so an
order "in 11 minutes" waits 11 simulated minutes. The engine then runs in
FAST mode and the bridge paces it from `pace()`, called once per step:

  * while the runtime is BUSY -- an operator message being handled, a
    model call in flight (`with busy():`) -- the simulation runs at real
    time, because the agent's thinking takes real time and the world must
    not race ahead of it (a stop that takes 300 ms to parse must still find
    the robot 300 ms further on, not 3 s);
  * otherwise it runs at up to OMNILINK_CLOCK_MAX_SPEED x real time
    (default 6), capped so a test that polls the pose every few wall
    milliseconds still sees each metre the robot drives.

Measured 2026-10-20 on a 90-minute dev shift: the robot was moving, handling
a message or waiting on the model 36% of the time; the other 64% is what
this compresses.
"""
from __future__ import annotations

import contextlib
import os
import threading
import time
from typing import Iterator, Optional

_ENABLED = os.environ.get("OMNILINK_CLOCK", "").strip().lower() == "sim"
try:
    MAX_SPEED = max(1.0, float(os.environ.get("OMNILINK_CLOCK_MAX_SPEED", "6")))
except ValueError:
    MAX_SPEED = 6.0

_cond = threading.Condition()
_sim_s: Optional[float] = None      # last simulator time published by the sim thread
_epoch = time.time()                # wall time that sim t = 0 maps to, for now()
_busy = 0                           # how many busy() blocks are open
_anchor: Optional[tuple] = None     # (sim_s, wall_monotonic) the pacing is measured from
_anchor_speed = MAX_SPEED


def enabled() -> bool:
    return _ENABLED


def now() -> float:
    """Epoch seconds: the wall clock, or start-of-run + simulator time."""
    if not _ENABLED or _sim_s is None:
        return time.time()
    return _epoch + _sim_s


def monotonic() -> float:
    if not _ENABLED or _sim_s is None:
        return time.monotonic()
    return _sim_s


def sleep(seconds: float, wall_cap_s: Optional[float] = None) -> None:
    """Sleep on this clock. Under the sim clock, wait for the simulator to
    advance `seconds`; a wall-clock cap (default: the same number of seconds
    plus a minute) guarantees a stopped simulator cannot hang a thread."""
    seconds = max(0.0, float(seconds))
    if not _ENABLED or _sim_s is None:
        time.sleep(seconds)
        return
    target = _sim_s + seconds
    wall_end = time.monotonic() + (wall_cap_s if wall_cap_s is not None else seconds + 60.0)
    with _cond:
        while _sim_s is not None and _sim_s < target:
            remaining = wall_end - time.monotonic()
            if remaining <= 0:
                return
            _cond.wait(min(remaining, 0.25))


@contextlib.contextmanager
def busy() -> Iterator[None]:
    """Run the simulation at real time while this block is open."""
    global _busy
    with _cond:
        _busy += 1
    try:
        yield
    finally:
        with _cond:
            _busy -= 1


def is_busy() -> bool:
    return _busy > 0


def publish(sim_s: float) -> None:
    """SIM THREAD, once per step: record simulator time and wake sleepers."""
    global _sim_s
    with _cond:
        _sim_s = float(sim_s)
        _cond.notify_all()


def pace(sim_s: float) -> None:
    """SIM THREAD, once per step, after publish(). Under the sim clock, sleep
    just long enough that simulator time runs no faster than the current
    allowance: 1x while busy, MAX_SPEED x otherwise. A change of allowance
    re-anchors, so a burst of fast time is never 'paid back' later."""
    global _anchor, _anchor_speed
    if not _ENABLED:
        return
    speed = 1.0 if _busy > 0 else MAX_SPEED
    wall = time.monotonic()
    if _anchor is None or speed != _anchor_speed or sim_s < _anchor[0]:
        _anchor, _anchor_speed = (sim_s, wall), speed
        return
    ahead = (sim_s - _anchor[0]) / speed - (wall - _anchor[1])
    if ahead > 0:
        time.sleep(min(ahead, 1.0))


def _reset_for_tests(enabled: bool = False, max_speed: float = 6.0) -> None:
    global _ENABLED, MAX_SPEED, _sim_s, _busy, _anchor, _anchor_speed, _epoch
    _ENABLED, MAX_SPEED = enabled, max_speed
    _sim_s, _busy, _anchor, _anchor_speed, _epoch = None, 0, None, max_speed, time.time()
