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

"""Per-instance variation.

The plan's T1.10 calls out three perceptual lies in the Tier 1
pipeline. One of them — "every generated instance of a PROTO is
pixel-identical to its siblings" — is closed here. Every placed prop
gets a deterministic per-instance seed that drives:

- Scale jitter within a caller-supplied ``[lo, hi]`` range.
- Hue shift in HSL within a small ``±deg`` tolerance.
- Small rotation offset on the non-dominant axis.

Determinism: a ``Variation`` is a pure function of ``(parent_seed,
group_name, instance_index)``. Two runs agree byte-for-byte; inserting
a new instance does not perturb the variation of the earlier ones
because each draws from its own derived sub-seed.
"""

from __future__ import annotations

import colorsys
import math
from dataclasses import dataclass

from ..core.rng import derive_seed, rng_from_seed


@dataclass(frozen=True)
class Variation:
    """Per-instance jitter to apply when emitting a placement."""

    scale: float                      # multiplicative on the PROTO's size
    hue_shift_deg: float              # signed, in degrees
    rotation_offset: float            # additive to yaw, in radians


def variation_for_instance(
    parent_seed: int,
    group_name: str,
    instance_index: int,
    *,
    scale_min: float = 1.0,
    scale_max: float = 1.0,
    hue_shift_deg: float = 0.0,
    rotation_jitter_deg: float = 0.0,
    scale_distribution: str = "uniform",
) -> Variation:
    """Return the deterministic variation for one placement.

    - ``scale_min / scale_max`` bracket the scale multiplier. When equal,
      scale is exactly 1.0 and no jitter is introduced.
    - ``scale_distribution`` controls how the multiplier is sampled:
        ``"log_uniform"`` (default) draws ``scale = scale_min *
            (scale_max/scale_min)^u`` where ``u ∼ U(0, 1)``. This biases
            the draw toward the small end of the range — small rocks
            are more probable than large ones — which matches the
            heavy-tailed size distribution of natural rock fields
            (planetary surface rock counts follow a power law where
            small rocks vastly outnumber boulders). For a typical
            scale_max/scale_min ratio of 3, ~63 % of draws land in the
            lower half of the range vs 50 % under uniform.
        ``"uniform"`` draws ``scale = scale_min + (scale_max - scale_min)
            * u`` (the original linear-uniform behaviour).
    - ``hue_shift_deg`` is the maximum absolute hue shift; actual
      draw is uniform in ``[-hue_shift_deg, +hue_shift_deg]``.
    - ``rotation_jitter_deg`` is the maximum absolute yaw offset.
    """
    sub_seed = derive_seed(parent_seed, f"variation.{group_name}.{instance_index}")
    rng = rng_from_seed(sub_seed)

    if scale_min > scale_max:
        raise ValueError("variation_for_instance: scale_min > scale_max")
    if scale_min == scale_max:
        scale = float(scale_min)
    elif scale_distribution == "log_uniform":
        # Geometric / log-uniform: log(scale) is uniformly distributed
        # between log(min) and log(max). Always positive when both
        # bounds are positive (which is enforced by the assertion
        # below — a zero or negative scale would invert the mesh).
        if scale_min <= 0.0:
            raise ValueError(
                "variation_for_instance: log_uniform requires scale_min > 0"
            )
        u = rng.random()
        scale = float(scale_min) * ((float(scale_max) / float(scale_min)) ** u)
    elif scale_distribution == "uniform":
        scale = scale_min + (scale_max - scale_min) * rng.random()
    else:
        raise ValueError(
            f"variation_for_instance: unknown scale_distribution {scale_distribution!r}"
        )
    hue = (rng.random() * 2.0 - 1.0) * hue_shift_deg
    rot = (rng.random() * 2.0 - 1.0) * math.radians(rotation_jitter_deg)
    return Variation(scale=scale, hue_shift_deg=hue, rotation_offset=rot)


def jitter_scale(rng, lo: float, hi: float) -> float:
    """Uniform-in-range scale draw. Thin wrapper so recipes have a
    named helper rather than inline ``rng.random()`` arithmetic."""
    if lo > hi:
        raise ValueError("jitter_scale: lo > hi")
    return lo + (hi - lo) * rng.random()


def jitter_hue_rgb(
    base_rgb: tuple[float, float, float], hue_shift_deg: float
) -> tuple[float, float, float]:
    """Shift the hue of ``base_rgb`` by ``hue_shift_deg`` while
    preserving lightness and saturation.

    All channels are clamped into ``[0, 1]`` on return — RGB values
    slightly outside that range after round-tripping through HLS are
    a rounding artifact, not a bug.
    """
    r, g, b = base_rgb
    h, l, s = colorsys.rgb_to_hls(r, g, b)
    # colorsys H is in [0, 1]; 1.0 = 360 degrees.
    h = (h + hue_shift_deg / 360.0) % 1.0
    if h < 0.0:
        h += 1.0
    r2, g2, b2 = colorsys.hls_to_rgb(h, l, s)
    return (
        max(0.0, min(1.0, r2)),
        max(0.0, min(1.0, g2)),
        max(0.0, min(1.0, b2)),
    )


__all__ = [
    "Variation",
    "variation_for_instance",
    "jitter_scale",
    "jitter_hue_rgb",
]
