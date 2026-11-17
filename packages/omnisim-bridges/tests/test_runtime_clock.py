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
"""The runtime's shared clock: the wall clock by default, simulator time
(paced) under OMNILINK_CLOCK=sim."""
import threading
import time

import pytest

from omnisim_bridges import runtime_clock as rc


@pytest.fixture(autouse=True)
def _reset():
    rc._reset_for_tests(enabled=False)
    yield
    rc._reset_for_tests(enabled=False)


def test_off_by_default_it_is_the_wall_clock():
    rc.publish(5.0)
    assert abs(rc.now() - time.time()) < 1.0
    rc.pace(5.0)                       # a no-op: never sleeps
    with rc.busy():
        assert rc.is_busy()
    assert not rc.is_busy()


def test_sim_clock_reads_simulator_time():
    rc._reset_for_tests(enabled=True)
    rc.publish(0.0)
    t0 = rc.now()
    rc.publish(660.0)                  # eleven simulated minutes later
    assert rc.now() - t0 == pytest.approx(660.0)
    assert rc.monotonic() == pytest.approx(660.0)


def test_sleep_waits_for_simulated_seconds():
    rc._reset_for_tests(enabled=True)
    rc.publish(10.0)
    woke = threading.Event()

    def sleeper():
        rc.sleep(30.0, wall_cap_s=5.0)
        woke.set()

    th = threading.Thread(target=sleeper)
    th.start()
    rc.publish(25.0)
    assert not woke.wait(0.2)          # only 15 of the 30 seconds have passed
    rc.publish(40.5)
    assert woke.wait(2.0)
    th.join(2.0)


def test_sleep_cannot_hang_on_a_stopped_simulator():
    rc._reset_for_tests(enabled=True)
    rc.publish(1.0)
    t0 = time.monotonic()
    rc.sleep(100.0, wall_cap_s=0.3)
    assert time.monotonic() - t0 < 2.0


def _sim_seconds_per_wall_second(busy: bool, max_speed: float) -> float:
    rc._reset_for_tests(enabled=True, max_speed=max_speed)
    sim, dt = 0.0, 0.032
    ctx = rc.busy() if busy else None
    if ctx:
        ctx.__enter__()
    try:
        rc.publish(sim)
        rc.pace(sim)                   # anchors
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            sim += dt                  # an engine that could run arbitrarily fast
            rc.publish(sim)
            rc.pace(sim)
        return sim / (time.monotonic() - t0)
    finally:
        if ctx:
            ctx.__exit__(None, None, None)


def test_busy_paces_the_world_to_real_time():
    assert _sim_seconds_per_wall_second(busy=True, max_speed=6.0) == pytest.approx(1.0, rel=0.25)


def test_idle_runs_faster_but_capped():
    assert _sim_seconds_per_wall_second(busy=False, max_speed=4.0) == pytest.approx(4.0, rel=0.25)
