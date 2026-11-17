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

"""A stop's rest measurement is differenced on the SIM clock (2026-10-18).

THE DEFECT. v9.2.6 (72febc7d2) moved the mobile bridge's stop settling window
to sim steps but left the tail, the span and the speed on the WALL stamp of
each pose sample. In headless `--mode=fast` the sim runs many times faster
than realtime: 0.30 s of sim passed in a few wall milliseconds, the 0.15 s
wall tail was empty, consecutive ticks shared a coarse `time.time()` stamp,
and every stop answered "pose samples arrived but spanned no measurable
interval" -- 3 of 3 runs of the MAB HB50W package on the installed v9.2.6.
The arm bridge's twin waited and differenced in wall seconds throughout, so
its rate was off by the realtime factor (10x too fast at 10x, and a creeping
joint read as still on a slow world).

⚠️ THIS DRIVES THE SHIPPED CODE. `_stop_measure_rest` is lifted out of each
controller with test_step_budget's AST loader and run against a fake world
whose `time` module is deterministic: every `time.sleep()` the waiter makes
advances a fake wall clock, and the fake sim thread ticks as many basic
steps as `RTF x wall` allows. Wall stamps are quantised to 15.6 ms, the
default Windows timer resolution the defect was found under.
"""

from __future__ import annotations

import ast
import math
import pathlib
import sys
import threading
import time as _real_time
from typing import Any, Dict, List, Optional, Tuple

import pytest

PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
TESTS = pathlib.Path(__file__).resolve().parent
for _p in (PKG_SRC, TESTS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import test_step_budget as _sb                        # noqa: E402
from omnisim_bridges.bridge_base import SimClock     # noqa: E402

DT = 0.032            # the basic timestep of the worlds the defect hit
WIN_TICK = 0.0156     # Windows' default timer resolution
# Sim time per wall second. 10x is the headless fast mode the defect was
# found in; 1x is a windowed run; 0.1x is a loaded laptop.
RTFS = [10.0, 1.0, 0.1]
CREEP = 0.01          # m per SIM second (mobile) / rad per SIM second (arm)


def _class_consts(path: pathlib.Path, names: List[str]) -> Dict[str, Any]:
    """The controller's own class constants, so the fake cannot drift."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out: Dict[str, Any] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            t = node.targets[0]
            if isinstance(t, ast.Name) and t.id in names and t.id not in out:
                try:
                    out[t.id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    missing = set(names) - set(out)
    assert not missing, f"{path.name} no longer defines {sorted(missing)}"
    return out


class _FakeWorld:
    """A deterministic sim thread plus the `time` module the waiter sees.

    The waiter's `time.sleep(d)` advances the fake wall clock by `d` and
    runs every basic step the sim would have produced in that interval at
    `rtf` x realtime. Nothing here really sleeps, so the result does not
    depend on the load of the machine running the test."""

    def __init__(self, bridge: Any, rtf: float, sample) -> None:
        self.bridge = bridge
        self.rtf = rtf
        self.sample = sample            # (wall_stamp, sim_t) -> history entry
        self.wall0 = _real_time.time()
        self.wall = self.wall0

    # -- the `time` module the lifted method is given --
    def time(self) -> float:
        return self.wall

    def sleep(self, d: float) -> None:
        self.wall += max(float(d), 1e-4)
        self.run()

    # -- the sim thread --
    def coarse(self, t: float) -> float:
        return self.wall0 + math.floor((t - self.wall0) / WIN_TICK) * WIN_TICK

    def run(self) -> None:
        b = self.bridge
        while b.clock.sim_time + DT <= self.rtf * (self.wall - self.wall0) + 1e-9:
            b.clock.tick()
            b.sim_time = b.clock.sim_time
            b.last_tick_at = self.wall
            with b.lock:
                b.hist().append(self.sample(self.coarse(self.wall), b.sim_time))


def _in_worker(fn, *args):
    """The stop's measurement refuses to run on the main thread (the sim
    thread cannot wait for itself), exactly as shipped."""
    box: Dict[str, Any] = {}

    def _go():
        try:
            box["out"] = fn(*args)
        except BaseException as exc:          # surfaced in the assertion
            box["exc"] = exc

    t = threading.Thread(target=_go, daemon=True)
    t.start()
    t.join(10.0)
    assert not t.is_alive(), "the rest measurement never returned"
    if "exc" in box:
        raise box["exc"]
    return box["out"]


# ── mobile bridge ───────────────────────────────────────────────────

MOBILE_CONSTS = ["STOP_SETTLE_S", "STOP_TAIL_FRAC", "STOP_MIN_SPAN_S",
                 "WAIT_POLL_S"]


class _FakeMobile:
    def __init__(self) -> None:
        self.__dict__.update(_class_consts(_sb.MOBILE, MOBILE_CONSTS))
        self.clock = SimClock(dt_s=DT)
        self.lock = threading.RLock()
        self._pose_hist: List[Tuple[float, float, float, float, float]] = []
        self.sim_time = 0.0
        self.last_tick_at = 0.0

    def hist(self):
        return self._pose_hist


def _mobile_stop(rtf: float, creep: float) -> dict:
    b = _FakeMobile()
    world = _FakeWorld(b, rtf, lambda w, s: (w, creep * s, 0.0, 0.0, s))
    world.sleep(1.0)                    # a second of driving history first
    ns = {"time": world, "threading": threading, "math": math,
          "Optional": Optional, "Tuple": Tuple}
    _sb.load_method(_sb.MOBILE, "wrap_pi", ns)
    fn = _sb.load_method(_sb.MOBILE, "_stop_measure_rest", ns)
    halt = (creep * b.sim_time, 0.0, 0.0)
    return _in_worker(fn, b, world.time(), halt, b.sim_time)


@pytest.mark.parametrize("rtf", RTFS)
def test_a_mobile_stop_reports_a_speed_at_any_realtime_factor(rtf) -> None:
    out = _mobile_stop(rtf, creep=0.0)
    assert out.get("speed_mps") is not None, (
        f"at {rtf}x realtime the stop could not confirm rest: "
        f"{out.get('reason')!r} -- the v9.2.6 fast-mode defect")
    assert out["speed_mps"] == 0.0
    assert out["yaw_rate_radps"] == 0.0
    # The span is SIM seconds: the tail of the 0.30 s window, never wall.
    assert out["over_s"] >= b_min_span(_sb.MOBILE)
    assert out["over_s"] <= 0.30 + 1e-9


@pytest.mark.parametrize("rtf", RTFS)
def test_a_mobile_creep_is_measured_per_sim_second(rtf) -> None:
    # A base creeping at 1 cm per SIM second reads 0.01 m/s whatever the
    # wall clock did. Differenced on wall time it read 0.1 at 10x and
    # 0.001 at 0.1x -- a still verdict on a robot that is moving.
    out = _mobile_stop(rtf, creep=CREEP)
    assert out["speed_mps"] == pytest.approx(CREEP, rel=1e-6)


def test_the_mobile_stop_hands_its_sim_halt_instant_to_the_measurement() -> None:
    src = _sb.MOBILE.read_text(encoding="utf-8")
    assert "self._stop_measure_rest(halted_at, halt_pose, halted_at_sim)" in src


# ── arm bridge ──────────────────────────────────────────────────────

ARM_CONSTS = ["STOP_SETTLE_S", "STOP_TAIL_FRAC", "STOP_MIN_SPAN_S",
              "WAIT_POLL_S", "WAIT_STALL_S"]


class _FakeArm:
    def __init__(self) -> None:
        self.__dict__.update(_class_consts(_sb.ARM, ARM_CONSTS))
        self.clock = SimClock(dt_s=DT)
        self.lock = threading.RLock()
        self._q_hist: List[Tuple[float, List[float], float]] = []
        self.joint_names = ["j1", "j2"]
        self.sim_time = 0.0
        self.last_tick_at = 0.0

    def hist(self):
        return self._q_hist


def _arm_stop(rtf: float, creep: float) -> dict:
    b = _FakeArm()
    world = _FakeWorld(b, rtf, lambda w, s: (w, [0.25, 0.5 + creep * s], s))
    world.sleep(1.0)
    ns = {"time": world, "threading": threading, "Dict": Dict, "Any": Any,
          "List": List, "Optional": Optional}
    fn = _sb.load_method(_sb.ARM, "_stop_measure_rest", ns)
    q_halt = [0.25, 0.5 + creep * b.sim_time]
    return _in_worker(fn, b, world.time(), q_halt, b.sim_time)


@pytest.mark.parametrize("rtf", RTFS)
def test_an_arm_stop_reports_a_rate_at_any_realtime_factor(rtf) -> None:
    out = _arm_stop(rtf, creep=0.0)
    assert out.get("max_joint_radps") is not None, (
        f"at {rtf}x realtime the arm stop measured nothing: "
        f"{out.get('reason')!r}")
    assert out["max_joint_radps"] == 0.0
    assert out["over_s"] >= b_min_span(_sb.ARM)


@pytest.mark.parametrize("rtf", RTFS)
def test_an_arm_creep_is_measured_per_sim_second(rtf) -> None:
    out = _arm_stop(rtf, creep=CREEP)
    assert out["max_joint_radps"] == pytest.approx(CREEP, rel=1e-6)
    assert out["joint"] == "j2"


def test_the_arm_stop_hands_its_sim_halt_instant_to_the_measurement() -> None:
    src = _sb.ARM.read_text(encoding="utf-8")
    assert "self._stop_measure_rest(halted_at, q, halted_at_sim)" in src


def b_min_span(path: pathlib.Path) -> float:
    return _class_consts(path, ["STOP_MIN_SPAN_S"])["STOP_MIN_SPAN_S"]
