#!/usr/bin/env python3
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

"""Self-test / numeric verifier for the OMNITUG500 warehouse-courier demo.

Assumes omnitug500_courier.omniworld is already running with its bridge on
127.0.0.1:<port> (the launcher script starts it). Drives a multi-stop route
through opposite-corner bays and verifies:

  * the run completes (packages loaded onto the deck, then delivered),
  * the rover never collides — checked with the ORIENTED footprint (not just
    the centre) against the static rack/wall geometry.

Exits 0 on PASS, 1 otherwise. Pure stdlib.

Usage:
    python scripts/dev/omnitug500_courier_selftest.py [--port 8765] [--timeout 200]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LAYOUT = os.path.join(REPO, "projects", "robots", "omnisim", "omnitug500", "worlds",
                      "omnitug500_courier_layout.json")


def _post(base, path, obj=None):
    req = urllib.request.Request(base + path, data=json.dumps(obj or {}).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=15) as r:
        return json.loads(r.read().decode())


def _get(base, path):
    with urllib.request.urlopen(base + path, timeout=15) as r:
        return json.loads(r.read().decode())


def _pt_rect(px, py, o):
    dx = max(o["x0"] - px, 0.0, px - o["x1"])
    dy = max(o["y0"] - py, 0.0, py - o["y1"])
    if dx == 0 and dy == 0:
        return -min(px - o["x0"], o["x1"] - px, py - o["y0"], o["y1"] - py)
    return math.hypot(dx, dy)


def _footprint_clearance(x, y, yaw, obst, hl, hw):
    fx, fy = -math.sin(yaw), math.cos(yaw)
    rx, ry = math.cos(yaw), math.sin(yaw)
    corners = [(x + a * hw * rx + b * hl * fx, y + a * hw * ry + b * hl * fy)
               for a in (-1, 1) for b in (-1, 1)]
    loop = [corners[0], corners[1], corners[3], corners[2]]
    pts = []
    for i in range(4):
        ax, ay = loop[i]
        bx, by = loop[(i + 1) % 4]
        for t in range(10):
            f = t / 10.0
            pts.append((ax + (bx - ax) * f, ay + (by - ay) * f))
    return min(min(_pt_rect(px, py, o) for o in obst) for px, py in pts)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--timeout", type=float, default=200.0)
    args = ap.parse_args()
    base = f"http://127.0.0.1:{args.port}"

    layout = json.load(open(LAYOUT, encoding="utf-8"))
    obst = layout["obstacles"]
    rv = layout.get("rover", {})
    hl, hw = rv.get("half_len", 0.63), rv.get("half_wid", 0.36)

    # wait for the bridge
    for _ in range(90):
        try:
            if _get(base, "/healthz").get("ok"):
                break
        except Exception:
            time.sleep(1)
    else:
        print("FAIL: bridge never came up on", base)
        return 1

    _post(base, "/reset"); time.sleep(2)
    route = {"steps": [
        {"action": "pick", "station": "bay-a"},     # NW corner
        {"action": "pick", "station": "bay-f"},     # SE corner
        {"action": "deliver", "station": "dock-2"},
    ]}
    print("route:", _post(base, "/run_route", route)["route"])

    t0 = time.time()
    min_clr = 1e9
    worst = None
    last = None
    done = False
    while time.time() - t0 < args.timeout:
        try:
            s = _get(base, "/get_robot_state")
        except Exception:
            time.sleep(0.4); continue
        yaw = math.radians(s["yaw_deg"])
        clr = _footprint_clearance(s["x"], s["y"], yaw, obst, hl, hw)
        if clr < min_clr:
            min_clr, worst = clr, (round(s["x"], 2), round(s["y"], 2), s["yaw_deg"])
        if s["last_event"] != last:
            last = s["last_event"]
            print(f"  t={time.time()-t0:6.1f} {s['mode']:6s} carry={s['carrying']} "
                  f"q={s['queue']} :: {last}")
        if s["mode"] == "idle" and s["queue"] == 0 and not s["carrying"] \
                and "delivered" in (last or ""):
            done = True
            break
        if s["fault"]:
            print("FAIL: fault", s["fault"])
            return 1
        time.sleep(0.5)

    print(f"\nmin oriented-footprint clearance = {min_clr:.3f} m at {worst}")
    if done and min_clr > 0.05:
        print("PASS: multi-stop route delivered, collision-free")
        return 0
    print("FAIL: incomplete" if not done else "FAIL: footprint graze")
    return 1


if __name__ == "__main__":
    sys.exit(main())
