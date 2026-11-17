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

"""Turn a flight telemetry log into an SVG: ground track, altitude, airspeed.

Emits a self-contained, theme-aware SVG (no external fonts, no scripts, colours
from CSS custom properties with literal fallbacks) so the same file drops into a
report, an artifact page, or a browser unchanged.

    python tools/plot_flight.py flight.jsonl --out flight.svg [--mission m.json]

The telemetry format is the one `controllers/hil_aircraft` writes under
HIL_TELEMETRY: one JSON object per line with t, x, y, z, airspeed, and the
attitude and control channels.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import List, Sequence, Tuple

PALETTE = {
    "ink": "var(--hil-ink, #1c1c1e)",
    "muted": "var(--hil-muted, #8a8a8f)",
    "grid": "var(--hil-grid, #d8d8dc)",
    "track": "var(--hil-track, #d4a017)",
    "alt": "var(--hil-alt, #2f7fd4)",
    "speed": "var(--hil-speed, #c1503a)",
    "zone": "var(--hil-zone, #4c9a5a)",
}


def load(path: str) -> List[dict]:
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    # A run killed mid-write leaves a torn final line. Skipping
                    # it is right; refusing to plot the other 4000 samples is
                    # not.
                    continue
    if not rows:
        raise SystemExit("no telemetry samples in %s" % path)
    return rows


def _scale(values: Sequence[float], lo_px: float, hi_px: float, pad: float = 0.06):
    lo, hi = min(values), max(values)
    if hi - lo < 1e-9:
        hi = lo + 1.0
    span = hi - lo
    lo -= span * pad
    hi += span * pad
    def to_px(v: float) -> float:
        return lo_px + (v - lo) * (hi_px - lo_px) / (hi - lo)
    return to_px, lo, hi


def _polyline(points: Sequence[Tuple[float, float]], colour: str, width: float = 2.0) -> str:
    if not points:
        return ""
    d = " ".join("%.2f,%.2f" % p for p in points)
    return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f" '
            'stroke-linejoin="round" stroke-linecap="round"/>' % (d, colour, width))


def _text(x: float, y: float, s: str, colour: str, size: float = 11.0,
          anchor: str = "start", weight: str = "400") -> str:
    return ('<text x="%.1f" y="%.1f" fill="%s" font-size="%.0f" font-weight="%s" '
            'text-anchor="%s" font-family="ui-sans-serif,system-ui,sans-serif">%s</text>'
            % (x, y, colour, size, weight, anchor, _esc(s)))


def _esc(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def build_svg(rows: List[dict], mission: dict | None = None,
              width: int = 880, height: int = 620) -> str:
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    ts = [r["t"] for r in rows]
    zs = [r["z"] for r in rows]
    vs = [r.get("airspeed", 0.0) for r in rows]

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
        'width="100%%" height="auto" role="img" '
        'aria-label="Flight track, altitude and airspeed">' % (width, height)
    ]

    # --- ground track, top-down ------------------------------------------
    track_h = 360
    pad = 46
    # One scale for BOTH axes: a ground track drawn with independent x and y
    # scales is a lie about the shape of the turns.
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) * 1.12
    cx, cy = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    def gx(v: float) -> float:
        return pad + (v - (cx - span / 2)) * (width - 2 * pad) / span
    def gy(v: float) -> float:
        # SVG y grows downward; north (+y) must point up.
        return track_h - pad - (v - (cy - span / 2)) * (track_h - 2 * pad) / span

    parts.append('<rect x="0" y="0" width="%d" height="%d" fill="none"/>' % (width, height))
    parts.append(_text(pad, 22, "Ground track (metres, north up)", PALETTE["ink"], 13, weight="600"))

    for frac in (0.25, 0.5, 0.75):
        gxp = pad + frac * (width - 2 * pad)
        parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" '
                     'stroke-width="1" stroke-dasharray="2 4"/>'
                     % (gxp, 34, gxp, track_h - pad, PALETTE["grid"]))

    if mission:
        for wp in mission.get("waypoints", []):
            parts.append('<circle cx="%.1f" cy="%.1f" r="7" fill="none" stroke="%s" '
                         'stroke-width="1.6" stroke-dasharray="3 3"/>'
                         % (gx(wp["x"]), gy(wp["y"]), PALETTE["zone"]))
            label = wp.get("label", "")
            if label:
                parts.append(_text(gx(wp["x"]) + 11, gy(wp["y"]) + 4, label,
                                   PALETTE["muted"], 10))

    parts.append(_polyline([(gx(x), gy(y)) for x, y in zip(xs, ys)], PALETTE["track"], 2.0))
    parts.append('<circle cx="%.1f" cy="%.1f" r="4.5" fill="%s"/>'
                 % (gx(xs[0]), gy(ys[0]), PALETTE["ink"]))
    parts.append(_text(gx(xs[0]) + 9, gy(ys[0]) - 8, "launch", PALETTE["muted"], 10))
    parts.append('<circle cx="%.1f" cy="%.1f" r="4.5" fill="none" stroke="%s" stroke-width="2"/>'
                 % (gx(xs[-1]), gy(ys[-1]), PALETTE["ink"]))
    parts.append(_text(gx(xs[-1]) + 9, gy(ys[-1]) + 14, "end", PALETTE["muted"], 10))

    # --- altitude and airspeed against time ------------------------------
    strip_top = track_h + 14
    strip_h = (height - strip_top - 30) / 2.0

    for idx, (series, colour, title, unit) in enumerate(
            ((zs, PALETTE["alt"], "Altitude", "m"),
             (vs, PALETTE["speed"], "Airspeed", "m/s"))):
        top = strip_top + idx * (strip_h + 18)
        to_y, lo, hi = _scale(series, top + strip_h - 12, top + 16)
        tx, _, _ = _scale(ts, pad, width - pad, pad=0.0)
        parts.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1"/>'
                     % (pad, top + strip_h - 12, width - pad, top + strip_h - 12, PALETTE["grid"]))
        parts.append(_polyline([(tx(t), to_y(v)) for t, v in zip(ts, series)], colour, 1.8))
        parts.append(_text(pad, top + 8, "%s (%s)" % (title, unit), PALETTE["ink"], 12, weight="600"))
        parts.append(_text(width - pad, top + 8, "%.1f to %.1f" % (min(series), max(series)),
                           PALETTE["muted"], 10, anchor="end"))

    parts.append(_text(pad, height - 8,
                       "%.0f s of flight, %d samples" % (ts[-1], len(rows)),
                       PALETTE["muted"], 10))
    parts.append("</svg>")
    return "\n".join(parts)


def summarise(rows: List[dict]) -> dict:
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    path = sum(math.dist((xs[i], ys[i]), (xs[i + 1], ys[i + 1])) for i in range(len(rows) - 1))
    zs = [r["z"] for r in rows]
    vs = [r.get("airspeed", 0.0) for r in rows]
    return {
        "duration_s": rows[-1]["t"],
        "samples": len(rows),
        "ground_track_m": path,
        "altitude_min_m": min(zs),
        "altitude_max_m": max(zs),
        "airspeed_min_m_s": min(vs),
        "airspeed_max_m_s": max(vs),
        "max_alpha_rad": max((r.get("alpha", 0.0) for r in rows), default=0.0),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry")
    ap.add_argument("--out", default="")
    ap.add_argument("--mission", default="")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rows = load(args.telemetry)
    mission = None
    if args.mission and os.path.exists(args.mission):
        with open(args.mission, "r", encoding="utf-8") as handle:
            mission = json.load(handle)

    if args.json:
        print(json.dumps(summarise(rows), indent=2))

    svg = build_svg(rows, mission)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(svg)
        print("wrote %s (%d bytes)" % (args.out, len(svg)))
    else:
        sys.stdout.write(svg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
