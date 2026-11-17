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

"""Sim-agnostic assertion primitives, in physical units only.

SPEC 6.2.6: "Every assertion is stated in physical units (metres, seconds,
contacts, exit codes). A grader that reads an OmniSim-shaped JSON response is
a bug." Everything in this module takes plain numbers and returns plain
numbers. The OmniSim-shaped reading lives in the adapter.

Two functions used to live here and no longer do: ``error_lines`` and
``controller_starts`` parsed a *specific simulator's log format*
(``^ERROR:``, ``^INFO: <name>: Starting controller:``), which is not a physical
unit -- and a competitor's log says neither of those things. They moved to the
adapter, and their results now arrive as
``evidence.ProcessFacts.error_lines`` / ``.behaviour_starts``.
"""

from __future__ import annotations

import math
import re

import numpy as np

# --- trajectories -----------------------------------------------------------


def net_displacement(xy):
    """Straight-line distance from the first to the last sample, metres.

    ``xy`` is an (N, 2) array of planar positions.
    """
    xy = np.asarray(xy, dtype=float)
    return float(np.hypot(*(xy[-1] - xy[0])))


def path_length(xy):
    """Integrated arc length over every sample, metres."""
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        return 0.0
    d = np.diff(xy, axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def bearing(xy):
    """Bearing of the net displacement, radians in (-pi, pi]."""
    xy = np.asarray(xy, dtype=float)
    dx, dy = xy[-1] - xy[0]
    return float(math.atan2(dy, dx))


def circular_resultant(thetas):
    """R = |sum e^{i theta}| / n, in [0, 1].

    1.0 == every bearing identical (a parade). For n uniform-random bearings
    E[R] ~ sqrt(pi)/(2 sqrt(n)); at n=10 that is ~0.28 and P(R >= 0.70) is
    under 1 % (SPEC A1.8).
    """
    thetas = np.asarray(list(thetas), dtype=float)
    if thetas.size == 0:
        return 0.0
    return float(abs(np.exp(1j * thetas).sum()) / thetas.size)


def z_band(z):
    """(z_first, z_last, z_min, z_max, abs_dz) for one robot's z track."""
    z = np.asarray(z, dtype=float)
    return (float(z[0]), float(z[-1]), float(z.min()), float(z.max()),
            float(abs(z[-1] - z[0])))


# --- geometry ---------------------------------------------------------------


def aabb_overlap_axes(a, b):
    """Per-axis overlap of two world AABBs, metres.

    ``a``/``b`` are ``(min_xyz, max_xyz)``. A NEGATIVE value on an axis is the
    clearance (separation) on that axis; a positive value is penetration.
    """
    amin, amax = np.asarray(a[0], float), np.asarray(a[1], float)
    bmin, bmax = np.asarray(b[0], float), np.asarray(b[1], float)
    return [float(min(amax[i], bmax[i]) - max(amin[i], bmin[i]))
            for i in range(3)]


def aabb_clearance(a, b):
    """Best separation across the three axes, metres.

    > 0 means the boxes are apart by that much on their best axis; <= 0 means
    they overlap on every axis (i.e. the boxes interpenetrate).
    """
    return float(-min(aabb_overlap_axes(a, b)))


def pairwise_min_clearance(boxes):
    """(min_clearance_m, (i, j)) over every unordered pair of AABBs."""
    worst = None
    pair = None
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            c = aabb_clearance(boxes[i], boxes[j])
            if worst is None or c < worst:
                worst, pair = c, (i, j)
    return (worst if worst is not None else float("inf")), pair


# --- answer extraction (inspect tier) ---------------------------------------

_NUM = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")


_DISTANCE = re.compile(
    r"([-+]?\d+(?:\.\d+)?)\s*(mm|cm|m|metres|meters|metre|meter)\b",
    re.IGNORECASE)

_UNIT_SCALE = {"mm": 0.001, "cm": 0.01}


def number_spans(text):
    """``[(value, start, end)]`` for every number, in order of appearance."""
    return [(float(m.group(0)), m.start(), m.end())
            for m in _NUM.finditer(text or "")]


def numbers_in(text):
    """Every number in a free-text answer, as floats."""
    return [v for v, _s, _e in number_spans(text)]


def distance_spans(text):
    """``[(metres, start, end)]`` for every unit-tagged distance, in order.

    The spans are what lets a grader ask *which* number an answer committed
    to, rather than scoring the best of all of them -- see b3_core: taking the
    minimum error over every number in an answer is how a list that blankets
    the plausible range scores a match without a measurement.
    """
    out = []
    for m in _DISTANCE.finditer(text or ""):
        v = float(m.group(1)) * _UNIT_SCALE.get(m.group(2).lower(), 1.0)
        out.append((v, m.start(), m.end()))
    return out


def distances_in(text):
    """Numbers a human would read as a distance in metres.

    Accepts ``3.4 m``, ``3.4m``, ``3.4 metres``, ``340 cm`` (converted). This
    is *extraction*, not judging -- the number still has to match the measured
    ground truth to within the task's tolerance.
    """
    return [v for v, _s, _e in distance_spans(text)]
