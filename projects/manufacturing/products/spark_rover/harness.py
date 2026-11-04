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

"""SparkRover Mini: route every wire of electrical.wire_list() through the assembled robot.

The router is the desk robot's (desk_robot/wiring.py): the robot's solids (all four
subsystems, integrate.load) voxelised, grown by each wire's radius plus a clearance, A*
with a discount for running along surfaces, string-pulled; each wire then becomes solid
for the next. What differs here is the scale (128 wires through a 290 mm robot, on a
1.0 mm grid) and two kinds of cable:

- the DSI and CSI ribbons (FPC, ~16 mm wide, 0.2 mm thick) are routed along their centre
  line for their LENGTH, with a 2 mm round envelope: a round-wire model understates a
  ribbon's width, so their clearance is not claimed (the chassis keeps a 44 mm pass in
  the spine for them);
- the Pi cooler's fan lead comes wired to the cooler and is not routed.

    python harness.py            # routes, writes release/electrical/wires_routed.json and meshes/wires/
"""
from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.append(str(HERE.parent / "desk_robot"))     # after ours: our electrical.py wins

import electrical  # noqa: E402
import integrate  # noqa: E402
import wiring  # noqa: E402  (desk_robot's router)

GRID_MM = 1.0
INSULATED_OD_MM = {18: 2.0, 20: 1.8, 22: 1.6, 24: 1.4, 26: 1.25, 28: 1.1}   # UL 1007 PVC hook-up, typical
FPC_ENVELOPE_MM = 2.0
NOT_ROUTED = {"FAN": "the Pi Active Cooler's own lead, wired at the factory"}
CACHE = HERE.parents[2].parent / ".tmp" / "spark_rover_harness"
ROUTED = HERE / "release" / "electrical" / "wires_routed.json"
WIRE_MESHES = HERE / "meshes" / "wires"


def space() -> wiring.Space:
    """The whole robot's occupancy grid, cached by a hash of every solid."""
    mods = integrate.load()
    solids = {}
    for owner, mod in mods.items():
        for label, tris in mod.solids(None).items():
            solids[f"{owner}/{label}"] = tris
    h = hashlib.sha256()
    for k in sorted(solids):
        h.update(k.encode())
        h.update(np.ascontiguousarray(solids[k], dtype=np.float32).tobytes())
    CACHE.mkdir(parents=True, exist_ok=True)
    f = CACHE / f"space_{h.hexdigest()[:16]}_{GRID_MM}.npz"
    if f.exists():
        d = np.load(f)
        sp = wiring.Space.__new__(wiring.Space)
        sp.lo, sp.pitch, sp.occ, sp.dist = d["lo"], float(d["pitch"]), d["occ"], d["dist"]
        sp.shape = sp.occ.shape
        sp.dist0 = sp.dist.copy()
        return sp
    sp = wiring.Space(solids, lo=(-96.0, -122.0, 0.0), hi=(96.0, 122.0, 292.0), pitch=GRID_MM)
    sp.dist0 = sp.dist.copy()
    np.savez_compressed(f, lo=sp.lo, pitch=sp.pitch, occ=sp.occ, dist=sp.dist)
    return sp


def _lead(w: dict, end: str):
    """Which way the wire leaves its pin: the electrical design gives a path's first and
    last legs where it has one; otherwise up out of a board."""
    path = w.get("path") or []
    if len(path) >= 2:
        a, b = (np.asarray(path[0], float), np.asarray(path[1], float)) if end == "from" else \
               (np.asarray(path[-1], float), np.asarray(path[-2], float))
        d = b - a
        if np.linalg.norm(d) > 1e-6:
            return tuple(d / np.linalg.norm(d))
    return (0.0, 0.0, 1.0)


BREAKOUT_MAX_MM = 12.0     # a wire leaves a housing (connector, motor end cap, switch) this far at most
BREAKOUT_FLAG_MM = 6.0     # longer than this, summary() lists it for a person to look at
MAX_EXPAND = 30_000_000    # more than the grid: a search ends when the space is exhausted, not at a cap
GUARD_MM = 0.5             # half a 1.0 mm cell: the search keeps this much more room than the wire needs


_MAIN: Dict[float, np.ndarray] = {}


def _main_space(sp: wiring.Space, r: float) -> np.ndarray:
    """The cells of the robot's one big free region for a wire of radius r (before any
    wire is laid). A pin in a pocket the grid closes (the 3 mm annulus round an arm's post
    at a 1 mm pitch) is not connected to it."""
    if r not in _MAIN:
        from scipy import ndimage
        lab, _ = ndimage.label(sp.dist0 > r + wiring.CLEAR_MM, structure=np.ones((3, 3, 3)))
        _MAIN[r] = lab == np.argmax(np.bincount(lab.ravel())[1:]) + 1
    return _MAIN[r]


def _breakout(sp: wiring.Space, p, lead, r: float):
    """The wire's way out of the solid its pin sits in. A crimp contact is inside its
    housing and a motor tab under its end cap: the model's solids are closed, so the wire
    leaves by the shortest straight run to the robot's free space, along the pin's lead
    first, else along an axis. Returns (direction, length mm); the run is the lead, not
    routed, and its length is reported."""
    p = np.asarray(p, float)
    main = _main_space(sp, r)
    dirs = [np.asarray(lead, float)] + [np.eye(3)[i] * s for i in range(3) for s in (1, -1)]
    best = None
    for i, d in enumerate(dirs):
        d = d / np.linalg.norm(d)
        for s in np.arange(2.0, BREAKOUT_MAX_MM + 1e-9, 0.25):
            c = sp.cell(p + s * d)
            if not all(0 <= v < n for v, n in zip(c, sp.shape)):
                break
            if sp.dist[c] > r + wiring.CLEAR_MM and main[c]:
                if best is None or s < best[1] - 1e-9:
                    best = (tuple(d), float(s))
                break
        if best is not None and i == 0 and best[1] <= 2.0:
            break        # the lead is already clear: the plain 2 mm lead
    return best if best is not None else (tuple(np.asarray(lead, float)), 2.0)


def route_all() -> List[dict]:
    sp = space()
    pos = electrical.positions(None)
    wires = electrical.wire_list(None)
    order = lambda w: (w["awg"] == 0, -(w["awg"] and 40 - w["awg"]), w["id"])    # thick first, ribbons last
    out = []
    for w in sorted(wires, key=order):
        if w["net"] in NOT_ROUTED:
            out.append({**w, "skipped": NOT_ROUTED[w["net"]]})
            continue
        dia = FPC_ENVELOPE_MM if w["awg"] == 0 else INSULATED_OD_MM[w["awg"]]
        a, b = pos[tuple(w["from"])], pos[tuple(w["to"])]
        rg = dia / 2 + GUARD_MM
        (da, la), (db, lb) = _breakout(sp, a, _lead(w, "from"), rg), _breakout(sp, b, _lead(w, "to"), rg)
        try:
            res = wiring.route(sp, a, b, dia, da, db, (la, lb), max_expand=MAX_EXPAND, guard_mm=GUARD_MM)
            if w["awg"]:
                wiring.occupy(sp, res["points_mm"][2:-2] if len(res["points_mm"]) > 4 else [], dia)
        except RuntimeError as e:
            res = {"error": str(e)}
        out.append({**w, "dia_mm": dia, "ribbon": w["awg"] == 0, "breakout_mm": [round(la, 2), round(lb, 2)], **res})
    return sorted(out, key=lambda w: int(str(w["id"]).lstrip("W") or 0))


def write(wires: List[dict]) -> None:
    import design   # desk_robot's STL writer
    WIRE_MESHES.mkdir(parents=True, exist_ok=True)
    for w in wires:
        if "points_mm" in w:
            design.write_stl(WIRE_MESHES / f"{w['id']}.stl",
                             wiring.tube_triangles(w["points_mm"], w["dia_mm"]) * 0.001, f"sparkrover wire {w['id']}")
    ROUTED.parent.mkdir(parents=True, exist_ok=True)
    ROUTED.write_text(json.dumps(wires, indent=1, default=str), encoding="utf-8")


def summary(wires: List[dict]) -> dict:
    routed = [w for w in wires if "points_mm" in w]
    round_ = [w for w in routed if not w.get("ribbon")]
    return {"wires": len(wires), "routed": len(routed), "failed": [w["id"] for w in wires if "error" in w],
            "skipped": [w["id"] for w in wires if "skipped" in w],
            "total_cut_m": round(sum(w["cut_mm"] for w in routed) / 1000, 2),
            "estimated_m": round(sum(w.get("length_mm", 0) for w in wires) / 1000, 2),
            "min_clear_of_parts_mm": min((w["clear_of_parts_mm"] for w in round_), default=None),
            "min_clear_of_wires_mm": min((w["clear_of_wires_mm"] for w in round_), default=None),
            "worst_parts": sorted(((w["clear_of_parts_mm"], w["id"]) for w in round_))[:5],
            "long_breakouts": [(w["id"], w["breakout_mm"]) for w in wires
                               if max(w.get("breakout_mm") or [0]) > BREAKOUT_FLAG_MM]}


if __name__ == "__main__":
    t = time.time()
    wires = route_all()
    write(wires)
    print(json.dumps({**summary(wires), "wall_s": round(time.time() - t, 1)}, indent=1))
