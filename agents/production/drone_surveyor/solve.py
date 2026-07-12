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

"""Standalone drone-surveyor solver — drives the bridge directly without OmniLink.

This script exists for two reasons (mirrors `agents/production/husky_maze/solve.py`):

1. It's the fastest way to validate that the OmniSim-side bridge works end-to-end
   — motion + camera capture + perception classifier + world-coordinate projection
   + ground-truth verification.
2. It demonstrates the gap that makes the OmniLink agent worth its keep: this
   script commits to a strategy at compile time (4-corner perimeter loop, scan
   at each, dedup within 1.5 m), while the LLM agent (lands iter 2) picks the
   strategy at runtime by reading `capabilities.mission_brief`.

Strategy (deterministic):

* Take off to capabilities.default_takeoff_altitude_m.
* Point gimbal straight down (capabilities.camera.down_pitch_rad).
* Fly to each of 4 perimeter corners around the warehouse.
* At each corner, call /scan and collect every {color, world_x, world_y} hit.
* Dedup detections within 1.5 m (same marker seen from two vantages).
* Filter to red, print count + positions, ground-truth-check against
  /solid?def=MARKER_RED_*.
* Land.

Usage:
    launch.bat projects\\samples\\demos\\worlds\\chat\\omnilink_mavic.wbt
    python agents/production/drone_surveyor/solve.py

Environment overrides:
    MAVIC_BRIDGE_URL    default http://127.0.0.1:6090
    MAVIC_SOLVE_ALTITUDE  takeoff altitude in meters, default 12
    MAVIC_SOLVE_DEDUP_M   dedup radius in meters, default 1.5
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Dict, List, Optional, Tuple

BRIDGE_URL = os.environ.get("MAVIC_BRIDGE_URL", "http://127.0.0.1:6090").rstrip("/")
TAKEOFF_ALT = float(os.environ.get("MAVIC_SOLVE_ALTITUDE", "12.0"))
DEDUP_M = float(os.environ.get("MAVIC_SOLVE_DEDUP_M", "1.5"))


# --- HTTP plumbing ---------------------------------------------------------

def _http(method: str, endpoint: str, payload=None, timeout=120.0):
    url = f"{BRIDGE_URL}/{endpoint.lstrip('/')}"
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        raise RuntimeError(
            f"bridge unreachable at {url}: "
            f"{e.reason if hasattr(e, 'reason') else e}"
        )
    return json.loads(raw) if raw else {}


# --- Survey waypoints ------------------------------------------------------
# Camera at 12 m altitude with the Mavic's default FoV (0.785 rad horizontal,
# ~0.49 rad vertical) sees a ~10 m × 6 m ground patch. Markers are scattered
# across roughly ±13 m × ±8 m around origin, so a true "fly the perimeter at
# (±16, ±10)" pattern leaves blind spots — the corners look at ground patches
# that miss every marker.
#
# Instead: a 3-column × 2-row grid of overflight waypoints positioned so each
# camera footprint covers ≥1 marker (deduplication absorbs the overlap from
# markers visible in two adjacent waypoints). This is the "smart perimeter"
# the LLM agent will reason out from the brief in iter 2 — solve.py hard-codes
# the pattern that actually works so the validation run is comparable to the
# eventual agent run.

SURVEY_WAYPOINTS: List[Tuple[float, float]] = [
    (-10.0, +5.0),   # NW area: MARKER_RED_1 (-10,+6), MARKER_CYAN (-7,+2)
    (  0.0, +6.0),   # N area:  MARKER_GREEN (+6,+7), MARKER_YELLOW (+10,+5) edge
    (+10.0, +5.0),   # NE area: MARKER_YELLOW (+10,+5), MARKER_GREEN (+6,+7) edge
    (+10.0, -4.0),   # SE area: MARKER_RED_2 (+12,-4), MARKER_MAGENTA (+5,-7) edge
    (  0.0, -7.0),   # S area:  MARKER_RED_3 (-3,-8), MARKER_MAGENTA (+5,-7)
    (-10.0, -4.0),   # SW area: MARKER_BLUE (-12,-4)
]


# --- Detection aggregation ------------------------------------------------

def _dedup_detections(detections: List[dict], radius_m: float) -> List[dict]:
    """Merge detections within `radius_m` of each other into a single entry.
    Average the world position, take the max fraction (best vantage). Returns
    the deduped list sorted by world_x then world_y for stable output."""
    clusters: List[Dict] = []
    for d in detections:
        if not d.get("projection_valid", True):
            continue
        if d["world_x"] is None or d["world_y"] is None:
            continue
        if isinstance(d["world_x"], float) and math.isnan(d["world_x"]):
            continue
        merged = False
        for c in clusters:
            if math.hypot(c["world_x"] - d["world_x"], c["world_y"] - d["world_y"]) <= radius_m:
                # Weighted-average by pixel count so the higher-confidence
                # detection contributes more.
                w_old = c["pixels"]
                w_new = d["pixels"]
                tot = w_old + w_new
                c["world_x"] = (c["world_x"] * w_old + d["world_x"] * w_new) / tot
                c["world_y"] = (c["world_y"] * w_old + d["world_y"] * w_new) / tot
                c["pixels"] = tot
                c["fraction"] = max(c["fraction"], d["fraction"])
                c["sightings"] += 1
                merged = True
                break
        if not merged:
            clusters.append({
                "color": d["color"],
                "world_x": d["world_x"],
                "world_y": d["world_y"],
                "pixels": d["pixels"],
                "fraction": d["fraction"],
                "sightings": 1,
            })
    clusters.sort(key=lambda c: (c["color"], c["world_x"], c["world_y"]))
    return clusters


# --- Top level -------------------------------------------------------------

def main() -> int:
    print(f"[solve] bridge: {BRIDGE_URL}")
    try:
        caps = _http("GET", "capabilities")
    except RuntimeError as e:
        print(f"[solve] {e}")
        print("[solve] launch chat/omnilink_mavic.wbt first.")
        return 4

    print(f"[solve] world_title = {caps.get('world_title')!r}")
    print(f"[solve] camera     = {caps['camera']['width']}x{caps['camera']['height']} "
          f"fov_h={caps['camera']['fov_h_rad']:.2f} rad fov_v={caps['camera']['fov_v_rad']:.2f} rad")
    print(f"[solve] gt_defs    = {caps.get('ground_truth_def_names', [])}")
    print()
    print("[solve] mission brief:")
    for line in (caps.get("mission_brief") or "").splitlines():
        print(f"        {line}")
    print()

    # 1. Takeoff.
    print(f"[solve] takeoff to {TAKEOFF_ALT} m")
    res = _http("POST", "action", {"action": "takeoff", "altitude": TAKEOFF_ALT,
                                    "wait": True, "timeout_s": 60.0})
    if not res.get("done"):
        print(f"[solve] takeoff failed: {res}")
        _http("POST", "action", {"action": "stop"})
        return 5
    print(f"[solve] reached altitude {res.get('z'):.2f} m at sim_time {res.get('sim_time'):.1f}")

    # 2. Pitch gimbal straight down (already the default in the bridge,
    #    but explicit for clarity).
    down = caps["camera"]["down_pitch_rad"]
    _http("POST", "action", {"action": "set_gimbal_pitch", "pitch_rad": down})

    # 3. Fly the survey grid, scanning at each waypoint.
    all_detections: List[dict] = []
    for i, (wx, wy) in enumerate(SURVEY_WAYPOINTS):
        print(f"\n[solve] waypoint {i+1}/{len(SURVEY_WAYPOINTS)}: ({wx:+.1f}, {wy:+.1f})")
        res = _http("POST", "action", {
            "action": "goto_waypoint",
            "x": wx, "y": wy,
            "altitude": TAKEOFF_ALT,
            # 120s, not 60s — the first leg from cold-takeoff at south start
            # to the NW-most waypoint is ~20 m and the Mavic PID's conservative
            # velocity profile needs ~80-100 s of real wall time to traverse it.
            # Fast-mode OmniSim compresses this enough that 60 s suffices, but
            # the demo defaults to GUI real-time mode for first-time users.
            "wait": True, "timeout_s": 120.0,
            # Aim nose toward the warehouse centre so successive waypoints
            # don't add a 180-degree yaw spin in flight.
            "yaw_to": [0.0, 0.0],
        })
        print(f"[solve]   arrived: done={res.get('done')} fault={res.get('fault')} "
              f"pose=({res.get('x'):+.2f},{res.get('y'):+.2f},{res.get('z'):.2f})")
        if res.get("fault"):
            print(f"[solve]   waypoint fault: {res['fault']} — continuing to next")
        # Brief settle for the gimbal + frame to refresh.
        time.sleep(0.5)
        scan = _http("GET", "scan")
        markers = scan.get("markers", [])
        red = [m for m in markers if m["color"] == "red"]
        print(f"[solve]   scan: {len(markers)} blobs, {len(red)} red")
        for m in markers:
            print(f"           {m['color']:<7} world=({m['world_x']:+.2f},{m['world_y']:+.2f}) "
                  f"frac={m['fraction']:.4f} dist={m['distance_m']:.1f} m")
        all_detections.extend(markers)

    # 4. Dedup detections within DEDUP_M and report.
    deduped = _dedup_detections(all_detections, DEDUP_M)
    red_clusters = [c for c in deduped if c["color"] == "red"]

    print()
    print("=" * 64)
    print(f"[solve] survey complete — {len(deduped)} unique markers detected")
    print(f"[solve]                    {len(red_clusters)} RED markers (count target)")
    print("=" * 64)
    for c in deduped:
        print(f"  {c['color']:<7} ({c['world_x']:+6.2f}, {c['world_y']:+6.2f})  "
              f"sightings={c['sightings']} frac={c['fraction']:.4f}")

    # 5. Ground-truth check: every advertised MARKER_RED_* DEF should have
    #    a deduped red cluster within DEDUP_M.
    print()
    print("[solve] ground-truth check vs. /solid:")
    red_defs = [d for d in caps.get("ground_truth_def_names", []) if d.startswith("MARKER_RED_")]
    matched = 0
    for d in red_defs:
        gt = _http("GET", f"solid?def={d}")
        gt_pos = gt.get("world_position", [None, None, None])
        gx, gy = gt_pos[0], gt_pos[1]
        nearest = None
        nearest_d = float("inf")
        for c in red_clusters:
            dist = math.hypot(c["world_x"] - gx, c["world_y"] - gy)
            if dist < nearest_d:
                nearest_d = dist
                nearest = c
        ok = nearest is not None and nearest_d <= DEDUP_M * 2  # 2x tolerance for ground-truth check
        if ok:
            matched += 1
        verdict = "OK" if ok else "MISS"
        if nearest is not None:
            print(f"  {d}: ground-truth ({gx:+6.2f}, {gy:+6.2f})  "
                  f"nearest_red ({nearest['world_x']:+6.2f}, {nearest['world_y']:+6.2f})  "
                  f"err={nearest_d:.2f} m  {verdict}")
        else:
            print(f"  {d}: ground-truth ({gx:+6.2f}, {gy:+6.2f})  "
                  f"no red detected anywhere  {verdict}")
    print(f"[solve] matched {matched}/{len(red_defs)} red markers within {DEDUP_M*2:.1f} m")

    # 6. Land.
    print()
    print("[solve] landing")
    _http("POST", "action", {"action": "land", "wait": True, "timeout_s": 30.0})

    # 7. Log mission completion (mirrors what the LLM agent will do via
    #    complete_mission in iter 2).
    payload = {
        "red_count": len(red_clusters),
        "red_positions": [[round(c["world_x"], 2), round(c["world_y"], 2)] for c in red_clusters],
        "all_detections": [
            {"color": c["color"], "world_x": round(c["world_x"], 2),
             "world_y": round(c["world_y"], 2), "sightings": c["sightings"]}
            for c in deduped
        ],
        "ground_truth_matched": matched,
        "ground_truth_total_red": len(red_defs),
    }
    rationale = (
        f"Perimeter survey complete: {len(red_clusters)} red markers detected, "
        f"{matched}/{len(red_defs)} confirmed against /solid ground truth."
    )
    _http("POST", "action", {"action": "complete_mission",
                              "rationale": rationale, "payload": payload})
    print(f"[solve] complete_mission: {rationale}")
    return 0 if matched == len(red_defs) else 7


if __name__ == "__main__":
    sys.exit(main())
