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

"""Color grading via ffmpeg filter chains.

We don't bundle .cube LUT files — every grade is expressed as a string
of native ffmpeg filters (eq, colorbalance, curves, hue, etc.). That
keeps the dependency footprint to "ffmpeg on PATH" and makes grades
easy to read in the source.

The `looks.Look.grade_filter` is the source of truth for each named
look. This module is mostly thin helpers to combine a grade with an
existing filter chain (e.g. when also adding letterbox bars).
"""

from __future__ import annotations


def combine_filters(*parts: str) -> str:
    """Concat non-empty filter strings with commas; returns "" if all empty."""
    return ",".join(p for p in parts if p)


def safe_chroma_pad(filter_chain: str) -> str:
    """Always end the filter chain with a chroma-safe pad. libx264 + yuv420p
    requires even width/height, and viewport frames sometimes come out odd."""
    pad = "scale=trunc(iw/2)*2:trunc(ih/2)*2"
    return combine_filters(filter_chain, pad)
