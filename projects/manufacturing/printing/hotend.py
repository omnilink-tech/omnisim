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

"""What leaves the nozzle, given what the extruder motor does.

The model is the one Klipper's pressure advance is built on (docs/Pressure_Advance.md):
the melt behind the nozzle is a spring, so flow out of the nozzle lags the filament the
extruder pushes by a time constant `lag_s`:

    d(V_out)/dt = (A_f * e_motor - V_out) / lag_s

With pressure advance set to that same constant, Klipper drives e_motor = e + pa * de/dt
and the flow out matches the slicer's request; with it off, corners and line ends blob
and starve. Two limits sit on top:

- **Melt limit.** The hotend melts at most `max_mm3_s`. When the spring would need more,
  the extruder gear skips: the excess filament is never pushed, and the line is thin.
- **Retraction** pulls the meniscus back; flow out is only counted once the meniscus is
  past the furthest point it has reached (a retract and its matching unretract deposit
  nothing). Ooze during travel is not modelled.

Units: mm, mm^3, seconds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

SUB_STEP_S = 0.0005


@dataclass
class Hotend:
    filament_area: float          # mm^2
    lag_s: float                  # melt pressure time constant
    max_mm3_s: float              # melt limit
    pushed: float = 0.0           # mm^3 of filament the motor has pushed, minus what skipped
    out: float = 0.0              # mm^3 past the nozzle tip (meniscus position)
    peak: float = 0.0             # furthest the meniscus has been
    skipped: float = 0.0          # mm^3 the gear failed to push (melt-limited)
    deposited: float = 0.0        # mm^3 laid down
    peak_flow: float = 0.0        # mm^3/s, the highest flow out seen
    _last_motor: float = field(default=None, repr=False)

    @classmethod
    def for_printer(cls, printer) -> "Hotend":
        h = printer.profile.get("hotend", {})
        return cls(printer.extruder.filament_area, float(h.get("pressure_lag_s", printer.extruder.pressure_advance)),
                   float(h.get("max_volumetric_mm3_s", math.inf)))

    def step(self, e_motor_mm: float, dt: float) -> float:
        """Advance by dt with the motor at e_motor_mm at the END of the step (linear in
        between). Returns the volume deposited during the step (mm^3)."""
        if self._last_motor is None:
            self._last_motor = e_motor_mm
            self.pushed = self.out = self.peak = e_motor_mm * self.filament_area
            return 0.0
        n = max(1, int(math.ceil(dt / SUB_STEP_S)))
        h = dt / n
        de = (e_motor_mm - self._last_motor) / n
        decay = 1.0 - math.exp(-h / self.lag_s) if self.lag_s > 0 else 1.0
        cap = self.lag_s * self.max_mm3_s               # the most pressure the melt holds
        before = self.deposited
        for _ in range(n):
            self.pushed += de * self.filament_area
            pressure = self.pushed - self.out
            if pressure > cap:                            # the gear skips the excess
                self.skipped += pressure - cap
                self.pushed -= pressure - cap
                pressure = cap
            flow = pressure * decay
            self.out += flow
            self.peak_flow = max(self.peak_flow, flow / h)
            if self.out > self.peak:
                self.deposited += self.out - self.peak
                self.peak = self.out
        self._last_motor = e_motor_mm
        return self.deposited - before
