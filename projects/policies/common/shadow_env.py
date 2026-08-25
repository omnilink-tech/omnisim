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

"""THE SHADOWING ENV CONTRACT -- one set of names for one method.

Shadowing has exactly four knobs, whatever the robot:

    the ghost              -- the achievable reference to track
    the corridor half-width-- how far the residual may move off it
    the feedforward        -- whether the ghost's declared torque/dq shifts the corridor centre
    the imitation weight   -- how hard the reward scores matching it

They were spelled TWICE, once per morphology, by two trainers that share the method:

    concept                humanoid (g1_walk_recipe)   quadruped (quad_walk_recipe)
    ---------------------  --------------------------  ---------------------------
    ghost lut              GHOST_LUT_JSON              QUAD_GHOST
    corridor half-width    GHOST_RESIDUAL              QUAD_RES_SCALE
    ghost feedforward      GHOST_FF                    QUAD_GHOST_FF
    imitation weight       W_GHOST                     QUAD_W_GMATCH
    imitation sigma        GHOST_SIG                   QUAD_GHOST_SIG
    robot selector         (none -- world-implied)     QUAD_ROBOT

Two dialects for one method is how a method stops generalizing: nothing reads both, so any tool
that wants to work across robots has to know which trainer it is talking to.

This module is the ONE reader. Canonical spelling is ``SHADOW_*``; the legacy humanoid and
quadruped names remain accepted ALIASES, so every shipped demo script, launcher and skill
manifest keeps working untouched. New code should write SHADOW_*.

Resolution order: SHADOW_<KNOB>  ->  the legacy aliases, in order  ->  the caller's default.
The DEFAULT stays with the caller on purpose: the humanoid and quadruped corridors are
genuinely different widths, and this module unifies the NAMES, not the tuning.
"""

from __future__ import annotations

import os

# canonical -> the legacy aliases it subsumes (checked in order)
ALIASES: dict[str, tuple[str, ...]] = {
    "SHADOW_GHOST":     ("GHOST_LUT_JSON", "QUAD_GHOST"),
    "SHADOW_RESIDUAL":  ("GHOST_RESIDUAL", "QUAD_RES_SCALE"),
    "SHADOW_FF":        ("GHOST_FF", "QUAD_GHOST_FF"),
    "SHADOW_W_GMATCH":  ("W_GHOST", "QUAD_W_GMATCH"),
    "SHADOW_SIG":       ("GHOST_SIG", "QUAD_GHOST_SIG"),
    "SHADOW_ROBOT":     ("QUAD_ROBOT",),
}


def raw(knob: str, default: str | None = None) -> str | None:
    """The value of a canonical knob, honouring the legacy aliases. None if unset."""
    v = os.environ.get(knob)
    if v not in (None, ""):
        return v
    for alias in ALIASES.get(knob, ()):
        v = os.environ.get(alias)
        if v not in (None, ""):
            return v
    return default


def ghost(default: str | None = None) -> str | None:
    """Path to the ghost lut (SHADOW_GHOST | GHOST_LUT_JSON | QUAD_GHOST)."""
    return raw("SHADOW_GHOST", default)


def residual(default: float) -> float:
    """Corridor half-width (SHADOW_RESIDUAL | GHOST_RESIDUAL | QUAD_RES_SCALE)."""
    v = raw("SHADOW_RESIDUAL")
    return float(v) if v is not None else float(default)


def ff(default: bool = False) -> bool:
    """Ghost feedforward on? (SHADOW_FF | GHOST_FF | QUAD_GHOST_FF)."""
    v = raw("SHADOW_FF")
    return (str(v).strip() == "1") if v is not None else bool(default)


def w_gmatch(default: float) -> float:
    """Imitation reward weight (SHADOW_W_GMATCH | W_GHOST | QUAD_W_GMATCH)."""
    v = raw("SHADOW_W_GMATCH")
    return float(v) if v is not None else float(default)


def sig(default: float) -> float:
    """Imitation reward sigma (SHADOW_SIG | GHOST_SIG | QUAD_GHOST_SIG)."""
    v = raw("SHADOW_SIG")
    return float(v) if v is not None else float(default)


def robot(default: str = "") -> str:
    """The robot to train/deploy (SHADOW_ROBOT | QUAD_ROBOT).

    The humanoid trainer had NO selector at all -- the robot was implied by the world file and
    the ghost lut. It is now declared by the ghost (`robot` key, schema 2); this is the override.
    """
    return str(raw("SHADOW_ROBOT", default) or default)
