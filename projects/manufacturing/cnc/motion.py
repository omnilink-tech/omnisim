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

"""Look-ahead motion planner and executor in GRBL's model.

Lines are queued with GRBL 1.1's junction-deviation corner speed, a backward
pass caps every entry speed so the queue can always stop at its end, and the
executor integrates a trapezoid in small fixed sub-steps. Sync items (dwell,
spindle/coolant change, M0, M2/M30) force a stop before them, as GRBL's buffer
synchronisation does.

This is GRBL's planning model, not its step timing: there is no step
generation, AMASS or segment buffer. Positions are exact floats along each
line. That is the right fidelity for limits, ordering and job time; it is not
a model of stepper resonance or missed steps under load.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, Sequence

from gcode import Block
from machine import AXES, Machine

SUB_STEP_S = 0.0005
PLANNER_CAPACITY = 15      # GRBL BLOCK_BUFFER_SIZE 16, one kept free


def limit_by_axis(max_values: Sequence[float], unit: Sequence[float]) -> float:
    limit = math.inf
    for m, u in zip(max_values, unit):
        if abs(u) > 1e-12:
            limit = min(limit, abs(m / u))
    return limit


@dataclass
class Item:
    block: Block
    start: List[float] = field(default_factory=list)
    unit: List[float] = field(default_factory=list)
    length: float = 0.0
    accel: float = 0.0           # mm/s^2 along the line
    rate_limit: float = 0.0      # mm/s along the line, from max rates
    max_entry: float = 0.0       # junction ceiling, mm/s
    entry: float = 0.0           # planned entry ceiling
    jog: bool = False

    @property
    def is_line(self) -> bool:
        return self.block.kind == "line"

    def nominal(self, feed_ov: float, rapid_ov: float) -> float:
        if self.block.rapid:
            return self.rate_limit * rapid_ov
        return min(self.block.feed / 60.0 * feed_ov, self.rate_limit)


class Planner:
    def __init__(self, machine: Machine, capacity: int = PLANNER_CAPACITY):
        self.machine = machine
        self.capacity = capacity
        self.queue: Deque[Item] = deque()
        self.position = [0.0, 0.0, 0.0]          # executed (commanded) machine position
        self.velocity = 0.0                       # mm/s along the current line
        self.progress = 0.0                       # mm along the current line
        self.dwell_left: Optional[float] = None
        self.feed_override = 1.0
        self.rapid_override = 1.0
        self.holding = False                      # decelerate to a stop and stay
        self.on_sync: Callable[[Block], None] = lambda b: None
        # Called after every executor sub-step with its length (s): a laser integrates its
        # output over the sub-steps, at the planner's own time resolution.
        self.on_substep: Optional[Callable[[float], None]] = None
        self._tail = [0.0, 0.0, 0.0]               # end of the last queued line

    # ── queue ─────────────────────────────────────────────────────────────
    def reset(self, position: Sequence[float]) -> None:
        self.queue.clear()
        self.position = list(position)
        self._tail = list(position)
        self.velocity = self.progress = 0.0
        self.dwell_left = None
        self.holding = False

    def has_room(self) -> bool:
        return len(self.queue) < self.capacity

    def idle(self) -> bool:
        return not self.queue

    def moving(self) -> bool:
        return self.velocity > 1e-9

    def push(self, block: Block, jog: bool = False, recalc: bool = True) -> None:
        item = Item(block, jog=jog)
        if block.kind == "line":
            start = list(self._tail)
            delta = [block.target[i] - start[i] for i in range(3)]
            length = math.sqrt(sum(d * d for d in delta))
            if length < 1e-6:
                return                        # GRBL drops zero-length lines
            item.start, item.length = start, length
            item.unit = [d / length for d in delta]
            axes = [self.machine.axes[a] for a in AXES]
            item.accel = limit_by_axis([a.accel for a in axes], item.unit)
            item.rate_limit = limit_by_axis([a.max_rate / 60.0 for a in axes], item.unit)
            prev = self.queue[-1] if self.queue else None
            item.max_entry = 0.0
            if prev is not None and prev.is_line:
                item.max_entry = self._junction_speed(prev, item)
            self._tail = list(block.target)
        self.queue.append(item)
        if recalc:
            self.recalculate()

    def _junction_speed(self, prev: Item, cur: Item) -> float:
        cos_theta = -sum(prev.unit[i] * cur.unit[i] for i in range(3))
        if cos_theta > 0.999999:
            return 0.0                       # full reversal
        nominal = min(prev.nominal(1, 1), cur.nominal(1, 1))
        if cos_theta < -0.999999:
            return nominal                   # straight continuation
        junction = [cur.unit[i] - prev.unit[i] for i in range(3)]
        norm = math.sqrt(sum(j * j for j in junction)) or 1.0
        junction = [j / norm for j in junction]
        accel = limit_by_axis([self.machine.axes[a].accel for a in AXES], junction)
        sin_half = math.sqrt(0.5 * (1.0 - cos_theta))
        deviation = float(self.machine.settings[11])
        v = math.sqrt(accel * deviation * sin_half / (1.0 - sin_half))
        return min(v, nominal)

    def recalculate(self) -> None:
        """Backward pass: every entry must allow a stop at the end of the queue."""
        exit_speed = 0.0
        items = list(self.queue)
        for idx in range(len(items) - 1, -1, -1):
            it = items[idx]
            if not it.is_line:
                exit_speed = 0.0
                continue
            it.entry = min(it.max_entry, math.sqrt(exit_speed ** 2 + 2.0 * it.accel * it.length))
            exit_speed = it.entry

    def exit_speed(self) -> float:
        if len(self.queue) > 1 and self.queue[1].is_line:
            return self.queue[1].entry
        return 0.0

    def flush(self) -> None:
        """Drop everything not yet executed (jog cancel, reset, alarm). Position stays."""
        self.queue.clear()
        self._tail = list(self.position)
        self.progress = 0.0
        self.dwell_left = None

    # ── execution ─────────────────────────────────────────────────────────
    def advance(self, dt: float) -> None:
        t = 0.0
        while t < dt - 1e-12:
            h = min(SUB_STEP_S, dt - t)
            self._step(h)
            if self.on_substep is not None:
                self.on_substep(h)
            t += h

    def _step(self, h: float) -> None:
        if not self.queue:
            self.velocity = 0.0
            return
        it = self.queue[0]
        if not it.is_line:
            if self.holding:
                return
            b = it.block
            if b.kind == "dwell":
                if self.dwell_left is None:
                    self.dwell_left = b.seconds
                self.dwell_left -= h
                if self.dwell_left > 0:
                    return
                self.dwell_left = None
            self.queue.popleft()
            self.on_sync(b)
            return
        remaining = it.length - self.progress
        a = it.accel
        if self.holding:
            v_new = max(0.0, self.velocity - a * h)
        else:
            nominal = it.nominal(self.feed_override, self.rapid_override)
            exit_v = self.exit_speed()
            if self.velocity < nominal:
                v_new = min(nominal, self.velocity + a * h)
            else:                                   # an override lowered the nominal speed
                v_new = max(nominal, self.velocity - a * h)
            v_brake = math.sqrt(max(0.0, exit_v ** 2 + 2.0 * a * max(0.0, remaining - self.velocity * h)))
            v_new = min(v_new, max(v_brake, exit_v))
            if v_new <= 0 and remaining > 0:
                v_new = min(a * h, nominal)
        ds = 0.5 * (self.velocity + v_new) * h
        self.velocity = v_new
        if ds >= remaining:
            self.position = list(it.block.target)
            self.queue.popleft()
            self.progress = 0.0
            if not self.queue or not self.queue[0].is_line:
                self.velocity = 0.0                 # a stop is planned before sync items
            return
        self.progress += ds
        self.position = [it.start[i] + it.unit[i] * self.progress for i in range(3)]

    def current_feed_mm_min(self) -> float:
        return self.velocity * 60.0

    def current_block(self) -> Optional[Block]:
        return self.queue[0].block if self.queue else None
