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

"""International Standard Atmosphere, and the wind the aircraft flies through.

Two jobs. The ISA half turns altitude into the density the aerodynamic model
needs and into the pressures a barometer and a pitot tube would actually report,
so the sensor stream handed to an autopilot is derived from the same state the
physics used rather than invented alongside it. The wind half exists because a
HIL rig whose air is always still cannot answer the question it was built for:
flight software is not interesting until something pushes back.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Tuple

from .vec3 import Vec3, ZERO, clamp

# ISA constants (ICAO Doc 7488).
T0_K = 288.15
P0_PA = 101325.0
RHO0 = 1.225
LAPSE_K_PER_M = 0.0065
R_SPECIFIC = 287.05287
G0 = 9.80665
GAMMA = 1.4

TROPOPAUSE_M = 11000.0
T_TROPOPAUSE_K = T0_K - LAPSE_K_PER_M * TROPOPAUSE_M       # 216.65 K
_PRESSURE_EXPONENT = G0 / (LAPSE_K_PER_M * R_SPECIFIC)     # ~5.2559
P_TROPOPAUSE_PA = P0_PA * (T_TROPOPAUSE_K / T0_K) ** _PRESSURE_EXPONENT


def temperature_k(altitude_m: float) -> float:
    if altitude_m < TROPOPAUSE_M:
        return T0_K - LAPSE_K_PER_M * altitude_m
    return T_TROPOPAUSE_K


def pressure_pa(altitude_m: float) -> float:
    if altitude_m < TROPOPAUSE_M:
        return P0_PA * (temperature_k(altitude_m) / T0_K) ** _PRESSURE_EXPONENT
    # Isothermal layer: the lapse-rate power law degenerates, so integrate the
    # hydrostatic equation at constant temperature instead.
    return P_TROPOPAUSE_PA * math.exp(
        -G0 * (altitude_m - TROPOPAUSE_M) / (R_SPECIFIC * T_TROPOPAUSE_K)
    )


def density(altitude_m: float) -> float:
    return pressure_pa(altitude_m) / (R_SPECIFIC * temperature_k(altitude_m))


def speed_of_sound(altitude_m: float) -> float:
    return math.sqrt(GAMMA * R_SPECIFIC * temperature_k(altitude_m))


def pressure_altitude_m(pressure: float) -> float:
    """Invert the troposphere pressure law: the altitude a barometer reports."""
    if pressure <= 0.0:
        return TROPOPAUSE_M
    if pressure >= P_TROPOPAUSE_PA:
        return (T0_K / LAPSE_K_PER_M) * (1.0 - (pressure / P0_PA) ** (1.0 / _PRESSURE_EXPONENT))
    return TROPOPAUSE_M - (R_SPECIFIC * T_TROPOPAUSE_K / G0) * math.log(pressure / P_TROPOPAUSE_PA)


def dynamic_pressure_pa(airspeed_m_s: float, altitude_m: float) -> float:
    """Pitot differential pressure, 0.5*rho*V^2 -- incompressible form.

    Valid to a few percent below about Mach 0.3, which covers every delivery
    aircraft this package is aimed at. It is deliberately not corrected for
    compressibility: a silent Mach correction would make the model disagree with
    the ``0.5*rho*V^2`` the aerodynamic surfaces use, and an autopilot tuned
    against one and flown against the other is exactly the class of bug this
    lane exists to catch.
    """
    return 0.5 * density(altitude_m) * airspeed_m_s * airspeed_m_s


@dataclass
class Atmosphere:
    """ISA, optionally offset -- a hot day is a real failure mode for a heavy lift.

    ``temperature_offset_k`` shifts the whole column (ISA+20 is the standard hot
    case) and thins the air accordingly, which is how density altitude eats a
    delivery aircraft's payload margin on a summer afternoon.
    """

    temperature_offset_k: float = 0.0
    sea_level_pressure_pa: float = P0_PA

    def temperature_k(self, altitude_m: float) -> float:
        return temperature_k(altitude_m) + self.temperature_offset_k

    def pressure_pa(self, altitude_m: float) -> float:
        return pressure_pa(altitude_m) * (self.sea_level_pressure_pa / P0_PA)

    def density(self, altitude_m: float) -> float:
        return self.pressure_pa(altitude_m) / (R_SPECIFIC * self.temperature_k(altitude_m))

    def density_altitude_m(self, altitude_m: float) -> float:
        """The ISA altitude at which the air is as thin as it is here.

        The number a pilot actually cares about on a hot day, and the one worth
        putting in a preflight report: it is what the wing and the propeller
        both respond to.
        """
        rho = self.density(altitude_m)
        return (T0_K / LAPSE_K_PER_M) * (1.0 - (rho / RHO0) ** (1.0 / (_PRESSURE_EXPONENT - 1.0)))


@dataclass
class Wind:
    """Steady wind plus band-limited turbulence, in WORLD frame (ENU, +Z up).

    The turbulence is a first-order Markov (Ornstein-Uhlenbeck) process per axis
    rather than a full Dryden spectrum: one pole, one time constant, the right
    variance and roughly the right correlation length. That is enough to make a
    controller work for its altitude hold, and it is honest about what it is --
    a Dryden implementation would claim a spectral shape this does not have.

    ``seed`` makes a run reproducible, which matters because a HIL regression
    that cannot be replayed cannot be bisected.
    """

    steady: Vec3 = ZERO
    turbulence_sigma_m_s: float = 0.0
    turbulence_length_m: float = 200.0
    seed: int = 0
    _state: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0), init=False, repr=False)
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def reset(self) -> None:
        self._state = (0.0, 0.0, 0.0)
        self._rng = random.Random(self.seed)

    def sample(self, dt: float, airspeed_m_s: float) -> Vec3:
        """Advance the gust state by ``dt`` and return the total wind vector.

        The correlation time is length/airspeed, so a faster aircraft passes
        through the same eddy sooner and sees a higher-frequency disturbance --
        which is why turbulence that was tuned at cruise can still upset an
        approach.
        """
        if self.turbulence_sigma_m_s <= 0.0:
            return self.steady
        v = max(airspeed_m_s, 1.0)
        tau = max(self.turbulence_length_m / v, 1e-3)
        beta = math.exp(-dt / tau)
        # Scale the driving noise so the stationary variance is sigma^2
        # regardless of dt; a bare beta*x + sigma*randn would make turbulence
        # intensity an artefact of the step size.
        drive = self.turbulence_sigma_m_s * math.sqrt(max(1.0 - beta * beta, 0.0))
        self._state = tuple(  # type: ignore[assignment]
            beta * s + drive * self._rng.gauss(0.0, 1.0) for s in self._state
        )
        return (
            self.steady[0] + self._state[0],
            self.steady[1] + self._state[1],
            self.steady[2] + self._state[2] * 0.5,  # vertical gusts are milder
        )


def wind_shear_factor(altitude_agl_m: float, reference_m: float = 10.0, exponent: float = 0.14) -> float:
    """Power-law boundary layer: wind falls off near the ground.

    Matters for the part of a delivery flight that actually goes wrong -- the
    approach, where the aircraft descends through a shear it did not see at
    cruise. Exponent 0.14 is the usual open-terrain value.
    """
    return (max(altitude_agl_m, 0.1) / reference_m) ** exponent


def clamp_unit(value: float) -> float:
    return clamp(value, -1.0, 1.0)
