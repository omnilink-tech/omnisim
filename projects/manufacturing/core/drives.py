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

"""Axis drives for machine twins: OmniSim position servos made to move like steppers.

Every machine in projects/manufacturing drives its axes the same way. A motorised
slider is an OmniSim position servo with stiffness ke = 10 x maxForce (N/m) and damping
kd = 0.5 x maxForce, so kd / ke is a fixed SERVO_TAU_S whatever the force. Such a servo
trails a moving command by SERVO_TAU_S x speed and lags under acceleration by
(effective inertia / ke) x acceleration. A stepper on a screw or belt does neither, so
the target each step is fed forward:

    target = midpoint of the command over the coming step
           + (SERVO_TAU_S - lead_trim) x its velocity over the step
           + accel_ff x its acceleration at the step's middle
           + an integral trim, integrated only while the command is at rest
             (a servo sags under a static load; a stepper holds)

The target holds for a whole physics step, so the planner runs two steps ahead and the
target is centred on the step it holds for.

**The effective inertia is not the moving mass.** Newton's servo-armature bound raises
any position-servo joint lighter than dt x kd to dt x kd (dt the substep; AGENTS.md
hard-won rules), so `accel_ff = max(moved mass, dt x kd) / ke` (`accel_feedforward`).
Measured both ways, 2026-11-01/02:

- the LongMill CNC (12.5 kg gantry, 2 ms substeps, so 20 kg effective): the coefficient
  tuned by hand on a finishing staircase was 1e-4 s^2, which is exactly 20 kg / 200 kN/m;
- the Sovol SV08 printer (a 0.65 kg toolhead, 1 ms substeps, so 10 kg effective): fed the
  bare masses its axes overshot every acceleration by up to 0.3 mm; fed 5e-5 they track
  at 9 um median.

`lead_trim` is per machine: the CNC's 2 ms substep leaves its axes 0.95 ms ahead without
it, the printer's 1 ms substep needs none (each measured, see the controllers).
"""
from __future__ import annotations

from typing import List, Sequence

SERVO_TAU_S = 0.05          # kd / ke of every OmniSim position servo (OmBasicJoint)
SERVO_KE_PER_N = 10.0       # ke = 10 x maxForce (N/m)
SERVO_KD_PER_N = 0.5        # kd = 0.5 x maxForce (N s/m)
# The integral trim that holds a static load: integrating during motion wound it up on
# the transient error and pushed past every reversal (1 mm bumps on the CNC's carved
# walls, 2026-11-01), so it integrates only at rest.
KI_PER_S = 4.0
TRIM_LIMIT_MM = 10.0
AT_REST_MM_S = 0.05


def accel_feedforward(moved_mass_kg: float, max_force_n: float, substep_s: float) -> float:
    """The acceleration feed-forward (s^2) for an axis moving `moved_mass_kg`."""
    effective = max(moved_mass_kg, substep_s * SERVO_KD_PER_N * max_force_n)
    return effective / (SERVO_KE_PER_N * max_force_n)


class FedForwardDrives:
    """The servo targets for N axes, in the machine's own units (mm)."""

    def __init__(self, axes: int, lead_trim_s: float, accel_ff: Sequence[float]):
        self.lead_trim_s = lead_trim_s
        self.accel_ff = list(accel_ff)
        self.trim = [0.0] * axes

    def targets(self, prev: Sequence[float], cur: Sequence[float], nxt: Sequence[float],
                after: Sequence[float], measured: Sequence[float], dt: float) -> List[float]:
        """`prev`, `cur`, `nxt`, `after`: the command at the step before, now, the end of the
        coming step and the step after that; `measured` where the axes are now."""
        out = []
        for i in range(len(self.trim)):
            velocity = (nxt[i] - cur[i]) / dt
            accel = ((after[i] - nxt[i]) - (cur[i] - prev[i])) / (2.0 * dt * dt)
            if abs(velocity) < AT_REST_MM_S and abs(cur[i] - prev[i]) < AT_REST_MM_S * dt:
                self.trim[i] = max(-TRIM_LIMIT_MM, min(TRIM_LIMIT_MM,
                                                       self.trim[i] + KI_PER_S * (cur[i] - measured[i]) * dt))
            out.append(0.5 * (cur[i] + nxt[i]) + (SERVO_TAU_S - self.lead_trim_s) * velocity
                       + self.accel_ff[i] * accel + self.trim[i])
        return out
