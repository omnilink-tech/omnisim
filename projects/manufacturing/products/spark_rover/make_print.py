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

"""Print every 3D-printed part of the SparkRover Mini on the SV08 printer twin, and measure each.

    python projects/manufacturing/products/spark_rover/make_print.py --plan     # no engine
    python projects/manufacturing/products/spark_rover/make_print.py --run      # plan, print, measure
    python projects/manufacturing/products/spark_rover/make_print.py --remeasure

The parts are the subsystems' own: every `parts(None)` row with process "print" (its
material and mass) and its `jobs(None)` STL, in the print orientation the module chose
(shell.py's "Print orientations", arms.print_bands, electrical.print_orientation). What
this module adds:

- **Printability per part** (`printability`): the STL cut at every layer; what a layer
  holds beyond the layer under it (grown by Cura's support angle) is either a BRIDGE
  (carried on two opposite sides, or over a hole, no wider than BRIDGE_MAX_MM) or an
  OVERHANG (support needed). Features narrower than one line (0.4 mm) are measured too.
  A part with overhang gets build-plate support (`support_type=buildplate`); a part
  without it gets none, so bridges are not buried in support.
- **Material profiles**: the SV08 profile with the filament's own OrcaSlicer values
  (`MATERIALS`, each with its source): PETG for the PETG parts, TPU 95A for the bumpers.
- **Preflight** (`preflight.py`) for each part's real print time and grams, then
  **packing** into plates (`pack`): one material and one support setting per plate,
  the parts laid out on the 350 x 350 bed with room for their brims, sized so the plates
  balance across the engines (the longest single part is the floor).
- **The runs**: each plate in its own headless engine, at most `--engines` of ours and
  never more than `--max-engines` on the machine, none started while the GPU is at
  `--max-gpu-c` or above. Every engine has its own log and run folder; only our own
  process trees are stopped.
- **Measurement per part** (`measure_part`): `inspect_print.inspect` against the part's
  own STL where it sat on the plate (height, outer surface, footprint), plus the plastic
  laid in the part, the support under it, the round strands hanging inside it, and the
  plate's `unrecorded_fraction` and nozzle crashes.

G-code, profiles, worlds' traces and bead dumps go under `.tmp/spark_print/`; the plan,
the measurements and PRINT.md go to `release/make/print/`. make.py may call `run()` and
`measure()` later (`--process print`).
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import shapely
import trimesh
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PRINTING = ROOT / "projects/manufacturing/printing"
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"
BASE_PROFILE = ROOT / "projects/manufacturing/machines/sovol_sv08/sovol_sv08.json"
OUT = ROOT / ".tmp/spark_print"
RELEASE = HERE / "release/make/print"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
# printing/ shares module names with cnc/ (preflight, world): it goes on sys.path only while its
# own modules import (cura_slice and inspect_print put it first themselves), then to the END,
# so a later `import preflight` in this process still finds cnc/preflight.py.
sys.path.insert(0, str(PRINTING))
import cura_slice  # noqa: E402
import inspect_print  # noqa: E402
import printer as printer_mod  # noqa: E402
while str(PRINTING) in sys.path:
    sys.path.remove(str(PRINTING))
sys.path.append(str(PRINTING))


def _load_by_path(name: str, path: Path):
    """The printer twin's preflight.py shares its name with the CNC twin's: load it apart."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


preflight_mod = _load_by_path("printer_twin_preflight", PRINTING / "preflight.py")

# ---- materials -------------------------------------------------------------------------------

ORCA = "https://github.com/SoftFever/OrcaSlicer/blob/26a39377713c774d52ca7d7ff7242188df4de63e/resources/profiles"
MATERIALS = {
    # The PETG copy the 145 g shell section was printed with (README "Measured: a 145 g shell
    # section"), plus the profile's minimum layer time and speed.
    "PETG": {
        "preflight": "PETG",
        "max_volumetric_mm3_s": 17.0,
        "source": f"{ORCA}/Sovol/filament/Generic%20PETG%20%40Sovol%20SV08.json (filament_max_volumetric_speed 17, "
                  "nozzle 235 / first layer 255, hot plate 75, fan 30-70 %, slow_down_layer_time 5, "
                  "slow_down_min_speed 10, filament_flow_ratio 0.98, retraction 0.5)",
        "cura": {"material_print_temperature": 235, "material_print_temperature_layer_0": 255,
                 "material_bed_temperature": 75, "material_bed_temperature_layer_0": 75,
                 "cool_fan_speed_min": 30, "cool_fan_speed_max": 70, "cool_min_layer_time": 5, "cool_min_speed": 10,
                 "material_flow": 98, "retraction_amount": 0.5},
    },
    # TPU 95A: Orca's "Generic TPU @Sovol SV08" (inherits Generic TPU @System -> fdm_filament_tpu).
    # The 3.6 mm3/s melt limit caps every feature speed (cura_slice caps the feature speeds
    # from FLOW_LIMITED; the first layer and the brim are capped here, at 0.5 x 0.25 mm lines).
    "TPU 95A": {
        "preflight": "TPU",
        "max_volumetric_mm3_s": 3.6,
        "source": f"{ORCA}/Sovol/filament/Generic%20TPU%20%40Sovol%20SV08.json (filament_max_volumetric_speed 3.6, "
                  "nozzle 235 / first layer 240, hot plate 65, fan 80-100 %, full fan from layer 3, "
                  "slow_down_layer_time 5, slow_down_min_speed 10, filament_flow_ratio 0.98); retraction 0.4 mm from "
                  f"{ORCA}/OrcaFilamentLibrary/filament/base/fdm_filament_tpu.json (filament_retraction_length)",
        "cura": {"material_print_temperature": 235, "material_print_temperature_layer_0": 240,
                 "material_bed_temperature": 65, "material_bed_temperature_layer_0": 65,
                 "cool_fan_speed_min": 80, "cool_fan_speed_max": 100, "cool_fan_full_layer": 3,
                 "cool_min_layer_time": 5, "cool_min_speed": 10, "material_flow": 98, "retraction_amount": 0.4,
                 "speed_print_layer_0": 28, "skirt_brim_speed": 28},
    },
}


def material_profile(material: str, base: Path = BASE_PROFILE) -> dict:
    """The SV08 profile with the filament's own values (MATERIALS)."""
    m = MATERIALS[material]
    prof = json.loads(base.read_text(encoding="utf-8"))
    prof = copy.deepcopy(prof)
    prof["name"] = f"Sovol SV08 ({material})"
    prof["hotend"]["max_volumetric_mm3_s"] = m["max_volumetric_mm3_s"]
    prof["hotend"]["max_volumetric_source"] = m["source"]
    prof["slicer"]["cura"].update(m["cura"])
    prof["slicer"]["note"] = prof["slicer"].get("note", "") + f" {material}: {m['source']}. Written by spark_rover/make_print.py."
    return prof


def write_profiles(out: Path = OUT / "profiles") -> Dict[str, Path]:
    out.mkdir(parents=True, exist_ok=True)
    paths = {}
    for material in MATERIALS:
        p = out / f"sovol_sv08_{material.split()[0].lower()}.json"
        p.write_text(json.dumps(material_profile(material), indent=2), encoding="utf-8")
        paths[material] = p
    return paths


# ---- the parts --------------------------------------------------------------------------------

def _stl_triangles(job) -> np.ndarray:
    """Triangles (N, 3, 3) from a binary STL (bytes) or an ASCII one (str)."""
    if isinstance(job, (bytes, bytearray)) and not bytes(job[:5]).lower().startswith(b"solid"):
        (n,) = struct.unpack_from("<I", job, 80)
        rec = np.frombuffer(job, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
        return rec["v"].astype(float)
    text = job.decode() if isinstance(job, (bytes, bytearray)) else job
    v = [[float(x) for x in ln.split()[1:4]] for ln in text.splitlines() if ln.strip().startswith("vertex")]
    return np.asarray(v, float).reshape(-1, 3, 3)


def printed_parts(r=None) -> List[dict]:
    """Every printed part: {label, module, material, mass_g, tris (print orientation, mm)}."""
    import integrate
    out = []
    for name, mod in integrate.load().items():
        rows = [p for p in mod.parts(r) if p.get("process") == "print"]
        if not rows:
            continue
        jobs = mod.jobs(r)
        for p in rows:
            tris = _stl_triangles(jobs[p["label"]])
            v = tris.reshape(-1, 3)
            lo, hi = v.min(0), v.max(0)
            tris = tris - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
            out.append({"label": p["label"], "module": name, "material": p["material"], "mass_g": p["mass_g"],
                        "tris": tris})
    return out


def write_stl(tris: np.ndarray, path: Path, name: str = "part") -> Path:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(name.encode()[:80].ljust(80, b"\0") + struct.pack("<I", len(tris)) + rec.tobytes())
    return path


# ---- printability -----------------------------------------------------------------------------

LAYER_0_MM, LAYER_MM = 0.25, 0.2          # the SV08 profile's layer_height_0 / layer_height
SUPPORT_ANGLE_DEG = 50.0                  # Cura's support_angle default (fdmprinter.def.json)
BRIDGE_MAX_MM = 25.0                      # arms.print_check's bridge rule
MIN_REGION_MM2 = 1.0                      # free regions smaller than this are facets / hole-top slivers
NEEDS_SUPPORT_MM2 = 25.0                  # overhang (not bridge) area over all layers that gets support
LINE_MM = 0.4                             # the nozzle


def layer_tops(height: float) -> np.ndarray:
    n = max(1, int(math.ceil((height - LAYER_0_MM) / LAYER_MM - 1e-6)) + 1)
    return LAYER_0_MM + LAYER_MM * np.arange(n)


def _bodies(tris: np.ndarray) -> List[trimesh.Trimesh]:
    """The closed bodies a part is made of (arms and carriers are unions of prisms and boxes
    whose faces coincide; each piece on its own is closed)."""
    m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
    m.merge_vertices(digits_vertex=5)
    if m.is_watertight:
        return [m]
    parts = m.split(only_watertight=False)
    return list(parts) if len(parts) else [m]


def _polys(g) -> List[Polygon]:
    if g is None or g.is_empty:
        return []
    return [q for q in getattr(g, "geoms", [g]) if isinstance(q, Polygon) and not q.is_empty]


def part_section(bodies: Sequence[trimesh.Trimesh], z: float):
    out = [s for s in (inspect_print.section(b, z) for b in bodies) if s is not None and not s.is_empty]
    return unary_union(out).buffer(0) if out else None


def _is_bridge(free: Polygon, lower, span_mm: float) -> Tuple[bool, float]:
    """Carried along two opposite sides (or most of the way round, a lid over a pocket) and no
    wider than span_mm.
    `lower` is the layer under it as grown for the overhang test (free regions start there)."""
    mrr = free.minimum_rotated_rectangle
    if not isinstance(mrr, Polygon):
        return False, 0.0
    xs = np.asarray(mrr.exterior.coords)[:4]
    edges = [(xs[i], xs[(i + 1) % 4]) for i in range(4)]
    lens = [float(np.hypot(*(b - a))) for a, b in edges]
    carried = lower.buffer(0.05)
    ring = free.exterior
    if ring.intersection(carried).length >= 0.6 * ring.length and min(lens) <= span_mm:
        return True, min(lens)      # a lid over a pocket, carried on most of its edge
    for k in (0, 1):
        span = lens[(k + 1) % 4]
        if span > span_mm:
            continue
        held = []
        for a, b in (edges[k], edges[k + 2]):
            strip = LineString([tuple(a), tuple(b)]).buffer(0.6)
            held.append(free.boundary.intersection(strip).intersection(carried).length / max(lens[k], 1e-9))
        if min(held) >= 0.5:
            return True, span
    return False, min(lens)


def _area_over(f: Polygon, below: Sequence) -> float:
    """The area of f that stands over any of the sections below it."""
    if not below:
        return 0.0
    arr = np.asarray(below, dtype=object)
    hit = arr[shapely.intersects(arr, f)]
    if not len(hit):
        return 0.0
    return float(unary_union(list(shapely.intersection(hit, f))).area)


def printability(tris: np.ndarray) -> dict:
    """Layer by layer: overhang (needs support), bridges, features thinner than a line."""
    bodies = _bodies(tris)
    height = float(tris[:, :, 2].max())
    grow = LAYER_MM * math.tan(math.radians(SUPPORT_ANGLE_DEG))
    prev = None
    below: List[Polygon] = []
    overhang, bridges, thin = [], [], []
    thin_mm3 = 0.0
    tops = layer_tops(height)
    for i, top in enumerate(tops):
        h = LAYER_0_MM if i == 0 else LAYER_MM
        zc = min(top - h / 2.0, height - 1e-3)
        s = part_section(bodies, zc)
        if s is None:
            prev = None
            continue
        narrow = s.difference(s.buffer(-LINE_MM / 2.0, quad_segs=4).buffer(LINE_MM / 2.0, quad_segs=4))
        a_thin = sum(q.area for q in _polys(narrow) if q.area > 0.02)
        if a_thin > 0.0:
            thin_mm3 += a_thin * h
            thin.append((round(float(zc), 2), round(a_thin, 2)))
        if prev is not None:
            grown = prev.buffer(grow, quad_segs=8)
            free = s.difference(grown)
            for f in _polys(free):
                if f.area < MIN_REGION_MM2:
                    continue
                ok, w = _is_bridge(f, grown, BRIDGE_MAX_MM)
                row = {"z": round(float(zc), 2), "area_mm2": round(f.area, 2), "width_mm": round(w, 2),
                       "at": [round(c, 1) for c in f.representative_point().coords[0]]}
                if not ok:      # how much of it stands over the part (build-plate support cannot reach)
                    row["over_part_mm2"] = round(_area_over(f, below), 2)
                (bridges if ok else overhang).append(row)
        prev = s
        below.append(s)
    over_mm2 = sum(r["area_mm2"] for r in overhang)
    over_part = sum(r["over_part_mm2"] for r in overhang)
    return {"height_mm": round(height, 2), "layers": int(len(tops)),
            "overhang_mm2": round(over_mm2, 1), "overhang_over_part_mm2": round(over_part, 1),
            "overhang_regions": len(overhang),
            "overhang_worst": sorted(overhang, key=lambda r: -r["area_mm2"])[:5],
            "bridge_mm2": round(sum(r["area_mm2"] for r in bridges), 1), "bridges": len(bridges),
            "widest_bridge_mm": round(max([r["width_mm"] for r in bridges], default=0.0), 2),
            "thinner_than_line_mm3": round(thin_mm3, 2), "thin_layers": len(thin),
            "thin_worst": sorted(thin, key=lambda r: -r[1])[:5],
            "needs_support": over_mm2 > NEEDS_SUPPORT_MM2,
            # build plate support where every overhang stands over the bed; else everywhere
            "support": (False if over_mm2 <= NEEDS_SUPPORT_MM2 else
                        "buildplate" if over_part <= NEEDS_SUPPORT_MM2 else "everywhere")}


_TURNS = {"as designed": np.eye(3),
          "x +90": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),
          "x -90": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),
          "y +90": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),
          "y -90": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float),
          "x 180": np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)}


def turned(tris: np.ndarray, turn: str) -> np.ndarray:
    t = tris @ _TURNS[turn].T
    v = t.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return t - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])


def choose_orientation(part: dict, bed=(350.0, 350.0, 345.0)) -> dict:
    """The module's orientation unless it needs support and another face down does not
    (or needs much less). Returns {turn, printability, alternatives}."""
    base = printability(part["tris"])
    out = {"turn": "as designed", "printability": base, "alternatives": {}}
    if not base["needs_support"]:
        return out
    best = ("as designed", base)
    for turn in _TURNS:
        if turn == "as designed":
            continue
        t = turned(part["tris"], turn)
        size = t.reshape(-1, 3).max(0) - t.reshape(-1, 3).min(0)
        if np.any(size > np.asarray(bed) + 1e-6):
            continue
        pa = printability(t)
        out["alternatives"][turn] = {k: pa[k] for k in ("height_mm", "overhang_mm2", "overhang_over_part_mm2",
                                                         "bridge_mm2", "widest_bridge_mm", "support")}
        if pa["overhang_mm2"] < 0.25 * best[1]["overhang_mm2"] and pa["overhang_mm2"] + 50.0 < best[1]["overhang_mm2"]:
            best = (turn, pa)
    out["turn"], out["printability"] = best
    return out


# ---- slicing and preflight --------------------------------------------------------------------

SUPPORT_SETTINGS = {"support_enable": "true", "support_type": "buildplate"}


def support_overrides(support) -> Dict[str, str]:
    """False: none. 'buildplate' (the proven chain) or 'everywhere' (overhang over the part)."""
    return {**SUPPORT_SETTINGS, "support_type": support} if support else {}


def cura_estimate_s(text: str) -> Optional[float]:
    """CuraEngine's own estimate: the last ;TIME_ELAPSED: (the header's ;TIME:6666 is the
    placeholder the Cura GUI fills in)."""
    i = text.rfind(";TIME_ELAPSED:")
    return float(text[i + 14:].split(None, 1)[0]) if i >= 0 else None


def slice_and_preflight(stl: Path, gcode: Path, material: str, support, profile: Path,
                        at: Optional[Sequence[float]] = None) -> dict:
    p = printer_mod.load(profile)
    overrides = support_overrides(support)
    gcode.parent.mkdir(parents=True, exist_ok=True)
    gcode.unlink(missing_ok=True)
    sl = cura_slice.slice_stl(stl, p.profile, gcode, overrides=overrides, at=list(at) if at is not None else None)
    text = gcode.read_text(encoding="utf-8", errors="replace")
    pf = preflight_mod.preflight(p, text, material=MATERIALS[material]["preflight"])
    cura_s = cura_estimate_s(text)
    return {"gcode": _rel(gcode), "speed_caps": sl["speed_caps"], "engine_errors": sl["engine_errors"],
            "cura_time_s": cura_s, "time_s": pf["time_s"], "filament_g": pf["filament_g"], "layers": pf["layers"],
            "moves": pf["moves"], "stopped": pf["stopped"], "off_bed": pf["off_bed"],
            "over_limit_moves": pf["over_limit_moves"], "peak_flow_mm3_s": pf["peak_flow_mm3_s"],
            "skipped_mm3": pf["skipped_mm3"], "deposited_mm3": pf["deposited_mm3"], "extent_mm": pf["extent_mm"]}


def _rel(p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(p)


# ---- packing -------------------------------------------------------------------------------------

BED_MM = (350.0, 350.0)
# Drawings replaced since they were planned, by the design's triangle hash (`_sha`): recorded,
# and measured if they were already printing, but packed on no plate. The 2026-11-03 carriers
# were drawn as solid boxes with no screw or insert holes, and the bay carrier's rear corners
# blocked the upper shell halves as they slide on; electrical.py revised all three (2026-11-04).
_OLD_CARRIERS = "the 2026-11-03 drawing: solid boxes with no screw or insert holes (revised 2026-11-04)"
SUPERSEDED_SHA = {
    "11b6df3ff2b38555ff5d4f8780f6e3e6c2ad7858": ("el_bay_carrier", _OLD_CARRIERS + "; its rear corners blocked "
                                                 "the upper shell halves as they slide on"),
    "70d415f79540f47847143cb33256597868ed2eee": ("el_front_carrier", _OLD_CARRIERS),
    "3299a19553491efb67622012312bfedc025e7aa2": ("el_cooler_duct", _OLD_CARRIERS),
}


def superseded(row: dict) -> Optional[str]:
    hit = SUPERSEDED_SHA.get(row.get("design_tris_sha", ""))
    return hit[1] if hit and hit[0] == row["label"] else None
EDGE_MM = 8.0          # a brim (5 mm) and a little more from the bed's edge
GAP_MM = 12.0          # between parts: two brims and 2 mm


def footprint(tris: np.ndarray) -> Tuple[float, float]:
    v = tris.reshape(-1, 3)
    return float(v[:, 0].max() - v[:, 0].min()), float(v[:, 1].max() - v[:, 1].min())


def layout(sizes: Sequence[Tuple[float, float]], bed=BED_MM, edge=EDGE_MM, gap=GAP_MM) \
        -> Optional[List[Tuple[float, float, bool]]]:
    """Shelf packing, biggest first, each part turned 90 degrees about z if that fits better.
    Returns [(cx, cy, turned)] per input, in bed mm, or None if they do not fit."""
    W, H = bed[0] - 2 * edge, bed[1] - 2 * edge
    order = sorted(range(len(sizes)), key=lambda i: -max(sizes[i]) * min(sizes[i]))
    shelves: List[List[float]] = []        # [y0, height, x_used]
    place: Dict[int, Tuple[float, float, bool]] = {}
    for i in order:
        w0, h0 = sizes[i]
        done = False
        for (w, h, t) in sorted([(w0, h0, False), (h0, w0, True)], key=lambda c: c[1]):
            for sh in shelves:
                if h <= sh[1] + 1e-9 and sh[2] + (gap if sh[2] > 0 else 0) + w <= W + 1e-9:
                    x0 = sh[2] + (gap if sh[2] > 0 else 0)
                    place[i] = (edge + x0 + w / 2, edge + sh[0] + h / 2, t)
                    sh[2] = x0 + w
                    done = True
                    break
            if done:
                break
        if done:
            continue
        y0 = (shelves[-1][0] + shelves[-1][1] + gap) if shelves else 0.0
        for (w, h, t) in sorted([(w0, h0, False), (h0, w0, True)], key=lambda c: -c[1]):   # tall side as the shelf
            if y0 + h <= H + 1e-9 and w <= W + 1e-9:
                shelves.append([y0, h, w])
                place[i] = (edge + w / 2, edge + y0 + h / 2, t)
                done = True
                break
        if not done:
            return None
    return [place[i] for i in range(len(sizes))]


def centred(pos, sizes, bed=BED_MM):
    """The layout moved so the plate's footprint is centred on the bed: CuraEngine centres the
    plate's mesh there (center_object) and, called as cura_slice.py calls it, does not apply
    mesh_position_x/y (`--at`), so the drawing must be where the slicer puts it."""
    lo = np.array([np.inf, np.inf])
    hi = -lo
    for (w, h), (cx, cy, t) in zip(sizes, pos):
        w, h = (h, w) if t else (w, h)
        lo = np.minimum(lo, [cx - w / 2, cy - h / 2])
        hi = np.maximum(hi, [cx + w / 2, cy + h / 2])
    d = np.array(bed) / 2.0 - (lo + hi) / 2.0
    return [(cx + d[0], cy + d[1], t) for cx, cy, t in pos]


def pack(parts: Sequence[dict], engines: int, bed=BED_MM, prefix: str = "plate") -> List[dict]:
    """Plates (one material and one support setting each) balanced over `engines`.

    `parts` rows need label, material, support, time_s, footprint. The target per engine is
    max(longest part, total / engines); parts go longest first to the plate (of their group)
    with the least time that still fits on the bed and under the target, else a new plate.
    Plates are then given to engines longest first (LPT)."""
    total = sum(p["time_s"] for p in parts)
    target = max(max(p["time_s"] for p in parts), total / max(1, engines))
    plates: List[dict] = []
    for p in sorted(parts, key=lambda q: -q["time_s"]):
        key = (p["material"], p["support"])
        fits = []
        for pl in plates:
            if pl["key"] != key or pl["time_s"] + p["time_s"] > target * 1.02:
                continue
            if layout([q["footprint"] for q in pl["parts"]] + [p["footprint"]], bed) is not None:
                fits.append(pl)
        if fits:
            pl = min(fits, key=lambda q: q["time_s"])
        else:
            pl = {"key": key, "material": p["material"], "support": p["support"], "parts": [], "time_s": 0.0}
            plates.append(pl)
        pl["parts"].append(p)
        pl["time_s"] += p["time_s"]
    # Small plates of one group merge when that still fits under the target (fewer engines).
    loads = [0.0] * max(1, engines)
    for pl in sorted(plates, key=lambda q: -q["time_s"]):
        k = int(np.argmin(loads))
        pl["engine"] = k
        loads[k] += pl["time_s"]
    for i, pl in enumerate(sorted(plates, key=lambda q: (q["engine"], -q["time_s"]))):
        pl["name"] = f"{prefix}{i + 1:02d}"
        pos = centred(layout([q["footprint"] for q in pl["parts"]], bed), [q["footprint"] for q in pl["parts"]], bed)
        pl["layout"] = {q["label"]: {"centre_mm": [round(c[0], 2), round(c[1], 2)], "turned_90": c[2]}
                        for q, c in zip(pl["parts"], pos)}
        pl.pop("key")
    return sorted(plates, key=lambda q: q["name"])


def _turn_z(tris: np.ndarray) -> np.ndarray:
    t = tris @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float).T
    v = t.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return t - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0])


def placed(tris: np.ndarray, centre: Sequence[float], turned_90: bool) -> np.ndarray:
    """A part (centred, on z 0) at its plate position, in bed mm."""
    t = _turn_z(tris) if turned_90 else tris
    return t + np.array([centre[0], centre[1], 0.0])


# ---- the plan -------------------------------------------------------------------------------------

def _plan_part(args) -> Tuple[dict, np.ndarray]:
    """One part: orientation and printability, its STL, one slice and preflight."""
    p, out, profile = args
    t0 = time.time()
    ch = choose_orientation(p)
    tris = p["tris"] if ch["turn"] == "as designed" else turned(p["tris"], ch["turn"])
    stl = write_stl(tris, out / "stl" / f"{p['label']}.stl", f"SparkRover {p['label']}")
    support = ch["printability"]["support"]
    sp = slice_and_preflight(stl, out / "gcode" / "parts" / f"{p['label']}.gcode", p["material"], support, profile)
    row = {"label": p["label"], "module": p["module"], "material": p["material"], "design_mass_g": p["mass_g"],
           "design_tris_sha": _sha(p["tris"]), "turn": ch["turn"], "support": support, "printability": ch["printability"],
           "alternatives": ch["alternatives"], "footprint": [round(v, 2) for v in footprint(tris)],
           "stl": _rel(stl), "plan_s": round(time.time() - t0, 1),
           **{k: sp[k] for k in ("time_s", "cura_time_s", "filament_g", "layers", "over_limit_moves", "skipped_mm3",
                                 "speed_caps", "stopped")}}
    return row, tris


def _slice_job(args) -> dict:
    return slice_and_preflight(*args)


def _sha(tris: np.ndarray) -> str:
    import hashlib
    return hashlib.sha1(np.round(np.asarray(tris, float), 4).tobytes()).hexdigest()


def plan(engines: int = 4, out: Path = OUT, r=None, workers: int = 6, reuse: bool = True, log=print,
         only: Sequence[str] = (), tag: str = "") -> dict:
    """Parts -> orientation and printability -> one slice + preflight each -> plates -> one
    slice + preflight per plate. Writes out/plan.json, or with `tag` out/plan_<tag>.json whose
    plates (`<tag>NN`) hold only the parts in `only` (a later batch beside a running one)."""
    from concurrent.futures import ProcessPoolExecutor
    profiles = write_profiles(out / "profiles")
    parts = printed_parts(r)
    rows, tris_by = [], {}
    cache_path = out / "part_rows.json"
    cache = {}
    if reuse and cache_path.exists():
        cache = {q["label"]: q for q in json.loads(cache_path.read_text(encoding="utf-8"))}
    todo = []
    for p in parts:
        q = cache.get(p["label"])
        if q is not None and q.get("design_tris_sha") == _sha(p["tris"]) and (ROOT / q["stl"]).exists():
            rows.append(q)
            tris_by[q["label"]] = _stl_triangles((ROOT / q["stl"]).read_bytes()).astype(float)
        else:
            todo.append(p)
    order = sorted(todo, key=lambda p: -len(p["tris"]))          # the big ones first
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for row, tris in pool.map(_plan_part, [(p, out, profiles[p["material"]]) for p in order]):
            rows.append(row)
            tris_by[row["label"]] = tris
            pa = row["printability"]
            log(f"{row['label']:24s} {row['material']:8s} {row['turn']:11s} support={row['support']!s:10s} "
                f"{row['time_s']:8.0f} s {row['filament_g']:7.1f} g overhang {pa['overhang_mm2']:7.1f} mm2 "
                f"(on part {pa['overhang_over_part_mm2']}) bridge {pa['bridge_mm2']:7.1f} mm2 ({row['plan_s']:.0f} s)")
    rows.sort(key=lambda q: [p["label"] for p in parts].index(q["label"]))
    cache_path.write_text(json.dumps(rows, indent=1), encoding="utf-8")
    plates = pack([q for q in rows if not superseded(q) and (not only or q["label"] in only)], engines,
                  prefix=tag or "plate")
    jobs = []
    for pl in plates:
        tris = np.concatenate([placed(tris_by[q["label"]], pl["layout"][q["label"]]["centre_mm"],
                                      pl["layout"][q["label"]]["turned_90"]) for q in pl["parts"]])
        stl = write_stl(tris, out / "plates" / f"{pl['name']}.stl", f"SparkRover {pl['name']}")
        v = tris.reshape(-1, 3)
        assert np.allclose([(v[:, 0].min() + v[:, 0].max()) / 2, (v[:, 1].min() + v[:, 1].max()) / 2],
                           np.array(BED_MM) / 2, atol=0.05), "the plate is not centred where the slicer puts it"
        at = None
        for q in pl["parts"]:
            write_stl(placed(tris_by[q["label"]], pl["layout"][q["label"]]["centre_mm"], pl["layout"][q["label"]]["turned_90"]),
                      out / "plates" / pl["name"] / f"{q['label']}.stl", q["label"])
        pl["stl"] = _rel(stl)
        jobs.append((stl, out / "gcode" / f"{pl['name']}.gcode", pl["material"], pl["support"],
                     profiles[pl["material"]], at))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        sps = list(pool.map(_slice_job, jobs))
    for pl, sp in zip(plates, sps):
        pl.update({"profile": _rel(profiles[pl["material"]]), "parts_time_s": round(pl["time_s"], 1),
                   **{k: sp[k] for k in ("gcode", "time_s", "cura_time_s", "filament_g", "layers", "moves", "stopped",
                                         "off_bed", "over_limit_moves", "skipped_mm3", "peak_flow_mm3_s", "extent_mm")}})
        pl["parts"] = [q["label"] for q in pl["parts"]]
        log(f"{pl['name']}: engine {pl['engine']} {pl['material']} support={pl['support']} {len(pl['parts'])} parts "
            f"{pl['time_s']:.0f} s ({pl['parts_time_s']:.0f} s as parts) {pl['filament_g']} g stopped={pl['stopped']}")
    result = {"engines": engines, "parts": rows, "plates": plates,
              "total_time_s": round(sum(p["time_s"] for p in plates), 1),
              "materials": {m: {"source": v["source"], "max_volumetric_mm3_s": v["max_volumetric_mm3_s"]}
                            for m, v in MATERIALS.items()},
              "rules": {"support_angle_deg": SUPPORT_ANGLE_DEG, "bridge_max_mm": BRIDGE_MAX_MM,
                        "needs_support_mm2": NEEDS_SUPPORT_MM2, "support": SUPPORT_SETTINGS}}
    (out / plan_name(tag)).write_text(json.dumps(result, indent=1), encoding="utf-8")
    return result


def plan_name(tag: str = "") -> str:
    return f"plan_{tag}.json" if tag else "plan.json"


def all_plans(out: Path = OUT) -> List[dict]:
    """plan.json and every tagged batch after it, oldest first."""
    paths = sorted(out.glob("plan*.json"), key=lambda q: q.stat().st_mtime)
    return [json.loads(q.read_text(encoding="utf-8")) for q in paths]


# ---- the runs -------------------------------------------------------------------------------------

def engines_on_machine() -> int:
    import psutil
    n = 0
    for p in psutil.process_iter(["name"]):
        if (p.info["name"] or "").lower().startswith("omnisim-bin"):
            n += 1
    return n


def our_engines_on_machine() -> int:
    """Engines running one of this module's worlds, from any make_print process."""
    import psutil
    n = 0
    for p in psutil.process_iter(["name", "cmdline"]):
        try:
            if (p.info["name"] or "").lower().startswith("omnisim-bin") and \
                    any(".spark_rover_print_" in c for c in (p.info["cmdline"] or [])):
                n += 1
        except psutil.Error:
            pass
    return n


def gpu_c() -> Optional[int]:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=15).stdout
        return int(out.strip().splitlines()[0])
    except Exception:
        return None


def _stop_tree(proc: subprocess.Popen) -> None:
    """Our own run-headless process, the engine it started and the engine's controllers."""
    if proc.poll() is None:
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            proc.kill()


def _bead_count(path: Path) -> Optional[int]:
    try:
        with path.open("rb") as f:
            head = f.read(12)
        return struct.unpack_from("<i", head, 8)[0]
    except (OSError, struct.error):
        return None


@dataclass
class Gate:
    """Launch gate shared by the job threads."""
    engines: int = 4
    max_engines: int = 6
    max_gpu_c: int = 75
    lock: threading.Lock = field(default_factory=threading.Lock)
    ours: int = 0
    waiting: List[int] = field(default_factory=list)
    last_start: float = 0.0
    spacing_s: float = 45.0          # an engine takes this long to show up in the count

    def acquire(self, name: str, ticket: int = 0, log=print) -> dict:
        """Blocks until this job may start; jobs start in ticket order (longest first)."""
        waited = 0.0
        with self.lock:
            self.waiting.append(ticket)
        while True:
            with self.lock:
                n, g = engines_on_machine(), gpu_c()
                first = ticket == min(self.waiting) and time.time() - self.last_start >= self.spacing_s
                mine = max(self.ours, our_engines_on_machine())
                if first and mine < self.engines and n < self.max_engines and (g is None or g < self.max_gpu_c):
                    self.ours += 1
                    self.waiting.remove(ticket)
                    self.last_start = time.time()
                    return {"engines_on_machine_before": n, "gpu_c_at_start": g, "waited_s": round(waited)}
            if waited > 0 and waited % 600 < 15:
                log(f"[{name}] waiting: ours {self.ours}/{self.engines}, machine {n}/{self.max_engines}, GPU {g} C")
            time.sleep(15.0)
            waited += 15.0

    def release(self) -> None:
        with self.lock:
            self.ours -= 1


def run_plate(pl: dict, gate: Gate, out: Path = OUT, log=print, ticket: int = 0) -> dict:
    """One plate, headless, start to the controller's 'done' (or a timeout). Writes
    out/runs/<plate>/report.json."""
    import psutil
    run = out / "runs" / pl["name"]
    run.mkdir(parents=True, exist_ok=True)
    trace, beads = run / "trace.jsonl", run / "part.beads"
    for old in (trace, beads, run / "report.json"):
        old.unlink(missing_ok=True)
    world = WORLDS / f".spark_rover_print_{pl['name']}.omniworld"
    profile = ROOT / pl["profile"]
    subprocess.run([sys.executable, str(PRINTING / "world.py"), "--printer", str(profile), "--gcode", pl["gcode"],
                    "--trace", trace.resolve().as_posix(), "--out", str(world)], check=True, cwd=ROOT,
                   capture_output=True)
    timeout = pl["time_s"] / 0.7 + 1800.0
    start = gate.acquire(pl["name"], ticket, log)
    env = dict(os.environ, OMNISIM_PRINTED_PART_DUMP=str(beads.resolve()),
               OMNISIM_LOG_PATH=str((run / "omnisim_log.txt").resolve()))
    rep = {"plate": pl["name"], "parts": pl["parts"], "material": pl["material"], "support": pl["support"],
           "planned_s": pl["time_s"], "binary": os.environ.get("OMNISIM_BINARY") or "omnisim-bin (default)", **start}
    t0 = time.time()
    rep["started"] = time.strftime("%Y-%m-%d %H:%M:%S")
    log(f"[{pl['name']}] start: {len(pl['parts'])} parts, {pl['time_s']:.0f} s planned, machine engines "
        f"{start['engines_on_machine_before']} before, GPU {start['gpu_c_at_start']} C")
    with (run / "engine.out").open("w", encoding="utf-8") as eng_log:
        proc = subprocess.Popen([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration",
                                 str(int(timeout))], cwd=ROOT, env=env, stdout=eng_log, stderr=subprocess.STDOUT)
        done, seen, job_t = None, 0, 0.0
        progress, machine, ours, gpus = [], [], [], []
        last = 0.0
        try:
            while done is None:
                if proc.poll() is not None:
                    rep["error"] = "the engine exited before the print finished"
                    break
                if time.time() - t0 > timeout:
                    rep["error"] = "timeout"
                    break
                time.sleep(10.0)
                if trace.exists():
                    with trace.open("rb") as f:
                        f.seek(seen)
                        chunk = f.read()
                    chunk = chunk[:chunk.rfind(b"\n") + 1]
                    seen += len(chunk)
                    for ln in chunk.decode("utf-8", "replace").splitlines():
                        if '"kind": "done"' in ln:
                            done = json.loads(ln)
                        elif '"kind": "sample"' in ln:
                            try:
                                job_t = json.loads(ln)["t"]
                            except (ValueError, KeyError):
                                pass
                now = time.time() - t0
                if now - last >= 60.0:
                    last = now
                    g = gpu_c()
                    n = engines_on_machine()
                    machine.append(n)
                    ours.append(gate.ours)
                    if g is not None:
                        gpus.append(g)
                    progress.append({"wall_s": round(now), "job_s": job_t, "beads": _bead_count(beads),
                                     "machine_engines": n, "our_engines": gate.ours, "gpu_c": g,
                                     "cpu_pct": psutil.cpu_percent(None)})
                    (run / "progress.json").write_text(json.dumps(progress, indent=1), encoding="utf-8")
            rep["wall_s"] = round(time.time() - t0, 1)
            time.sleep(5.0)                                 # the engine's last bead dump (within 1 s)
        finally:
            _stop_tree(proc)
            gate.release()
    rep["done"] = done
    rep["speed_x_real_time"] = round(done["t"] / rep["wall_s"], 2) if done else None
    rep["machine_engines"] = {"min": min(machine), "max": max(machine), "mean": round(sum(machine) / len(machine), 2)} \
        if machine else None
    rep["our_engines"] = {"min": min(ours), "max": max(ours)} if ours else None
    rep["gpu_c_max"] = max(gpus) if gpus else None
    rep["beads"] = _bead_count(beads)
    logtxt = (run / "omnisim_log.txt").read_text(encoding="utf-8", errors="replace") \
        if (run / "omnisim_log.txt").exists() else ""
    rep["engine_warnings"] = [ln for ln in logtxt.splitlines() if "PrintedPart" in ln or "WARNING" in ln.upper()][:20]
    world.unlink(missing_ok=True)
    (run / "report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    log(f"[{pl['name']}] done in {rep.get('wall_s')} s wall ({rep['speed_x_real_time']}x real time): {done}")
    return rep


# ---- measurement ----------------------------------------------------------------------------------

class BodiesMesh:
    """A part as its closed bodies, for inspect_print: the arms and the carriers are unions
    of prisms and boxes whose faces coincide, which one trimesh section cannot cut cleanly.
    Picklable, so inspect_print's layer workers get it too (unpickling imports this module,
    which installs the section below)."""

    def __init__(self, tris: np.ndarray):
        self.bodies = _bodies(tris)
        v = tris.reshape(-1, 3)
        self.bounds = np.array([v.min(0), v.max(0)])


_SECTION = inspect_print.section


def _section(mesh, z):
    if isinstance(mesh, BodiesMesh):
        return part_section(mesh.bodies, z)
    return _SECTION(mesh, z)


inspect_print.section = _section

DENSITY_G_CM3 = {"PETG": 1.27, "TPU 95A": 1.21}
# PrintedPart's nozzle face (OmPrintedPart kFaceRatio, 2.5 nozzles): no line is laid wider since
# the 2026-11-04 bead fix. A capped bead is stored as float32 1.0000000475 mm, so "wide" means
# wider than the face by more than that rounding.
WIDE_BEAD_MM = 1.0
WIDE_BEAD_TOL_MM = 1e-4


def measure_part(dump: dict, tris_bed: np.ndarray, material: str, jobs: int = 4) -> dict:
    """One part of a plate against its drawing, where it sat (bed mm)."""
    mesh = BodiesMesh(tris_bed)
    beads = dump["beads"]
    # The first layer is fused to the brim (README, the 20 mm cube): it is left out of the
    # surface and footprint, as the cube's numbers leave it out.
    above = beads[:, 2] > LAYER_0_MM + 0.05
    rep = inspect_print.inspect({**dump, "beads": beads[above]}, mesh, 0.4, jobs)
    rep.pop("layers", None)
    full = inspect_print.bead_volume_mm3(beads)
    rep.update({"beads": int(len(beads)), "recorded_mm3": round(full, 1),
                "unrecorded_fraction": round(max(0.0, 1.0 - full / dump["deposited_mm3"]), 4)
                if dump["deposited_mm3"] > 0 else 0.0, "first_layer": "left out of the surface and footprint"})
    # Until the 2026-11-04 bead fix PrintedPart laid plastic held over "still" steps (a nozzle
    # under 20 mm/s) on the next 0.04-0.05 mm step as one bead up to 7 mm wide. Since then no
    # line is wider than the nozzle's face (WIDE_BEAD_MM) and these columns equal the main
    # ones; they stay so a dump laid by an older engine shows what its wide beads add.
    wide = (beads[:, 8] == 0.0) & (beads[:, 6] > WIDE_BEAD_MM + WIDE_BEAD_TOL_MM)
    alt = inspect_print.inspect({**dump, "beads": beads[above & ~wide]}, mesh, 0.4, jobs)
    rep["without_wide_beads"] = {"outer_surface_error_mm": alt["outer_surface_error_mm"],
                                 "size_error_mm": {k: alt["size_error_mm"][k] for k in ("x_mean", "y_mean")}}
    zmax = float(mesh.bounds[1][2])
    mid = np.c_[0.5 * (beads[:, 0] + beads[:, 3]), 0.5 * (beads[:, 1] + beads[:, 4])]
    line = beads[:, 8] == 0.0
    layer = np.rint((beads[:, 2] - LAYER_0_MM) / LAYER_MM).astype(int)
    in_part = np.zeros(len(beads), bool)
    near_outline = np.zeros(len(beads), bool)
    # A bead PrintedPart laid round (width == height) with a diameter above the layer height
    # had nothing within a layer under it: a strand hanging from the nozzle (a gap over 2
    # nozzle diameters) or a thin one over a smaller gap. Round beads thinner than the layer
    # are the starts and ends of lines (too little plastic to reach down), not strands.
    layer_h = np.where(beads[:, 2] < LAYER_0_MM + 0.05, LAYER_0_MM, LAYER_MM)
    strand = line & (np.abs(beads[:, 6] - beads[:, 7]) < 1e-6) & (beads[:, 6] > 1.05 * layer_h)
    shadow_parts = []
    for k in np.unique(layer[line]):
        top = LAYER_0_MM + LAYER_MM * k
        h = LAYER_0_MM if k == 0 else LAYER_MM
        zc = top - h / 2.0
        if zc <= 0.0 or zc >= zmax:
            continue
        s = part_section(mesh.bodies, zc)
        if s is None:
            continue
        shadow_parts.append(s)
        sel = np.flatnonzero(line & (layer == k))
        if len(sel):
            in_part[sel] = shapely.contains_xy(s.buffer(0.05), mid[sel, 0], mid[sel, 1])
            st = sel[in_part[sel] & strand[sel]]
            if len(st):         # within 1 mm of the drawn outline: a strand on a surface you see
                near_outline[st] = shapely.distance(s.boundary, shapely.points(mid[st])) < 1.0
    shadow = unary_union(shadow_parts).buffer(1.0) if shadow_parts else None
    under = np.zeros(len(beads), bool)
    if shadow is not None:
        under = shapely.contains_xy(shadow, mid[:, 0], mid[:, 1]) & ~in_part & (beads[:, 2] <= zmax + 0.5)
    blobs_in = (~line) & under
    rho = DENSITY_G_CM3[material]
    part_mm3 = inspect_print.bead_volume_mm3(beads[in_part]) + inspect_print.bead_volume_mm3(beads[blobs_in])
    support_mm3 = inspect_print.bead_volume_mm3(beads[under & line])
    lens = np.hypot(beads[:, 3] - beads[:, 0], beads[:, 4] - beads[:, 1])
    hs = in_part & strand
    so = hs & near_outline
    rep.update({
        "part_g": round(part_mm3 / 1000.0 * rho, 2),
        "support_or_brim_under_g": round(support_mm3 / 1000.0 * rho, 2),
        "hanging_strands": {"beads": int(hs.sum()), "length_mm": round(float(lens[hs].sum()), 1),
                            "share_of_part_plastic": round(inspect_print.bead_volume_mm3(beads[hs]) / max(part_mm3, 1e-9), 5),
                            "at_outline_beads": int(so.sum()), "at_outline_length_mm": round(float(lens[so].sum()), 1)},
        "blobs_in_part": int(blobs_in.sum()),
        "wide_beads": {"beads": int((in_part & wide).sum()),
                       "mm3": round(inspect_print.bead_volume_mm3(beads[in_part & wide]), 2),
                       "widest_mm": round(float(beads[in_part, 6].max()), 2) if in_part.any() else None},
    })
    return rep


MODES = {"twin": "runs", "ideal": "ideal"}     # print mode -> its run folder under out/


def bead_rule_of(measured: Optional[dict]) -> Optional[str]:
    """Which PrintedPart laying rule laid a measured dump, read from the dump itself: "fixed"
    (the 2026-11-04 bead fix: no line wider than the nozzle's face) or "pre-fix" (lines up to
    13 mm wide on these plates). From the plate's widest line, or its parts' widest."""
    if not measured:
        return None
    w = measured.get("widest_line_mm")
    if w is None:
        ws = [(q.get("wide_beads") or {}).get("widest_mm") for q in (measured.get("parts") or {}).values()]
        ws = [v for v in ws if v is not None]
        w = max(ws) if ws else None
    if w is None:
        return None
    return "fixed" if w <= WIDE_BEAD_MM + 1e-3 else "pre-fix"
MODE_NOTE = ("twin: the SV08 twin in the engine, full physics (servo-driven axes, PrintedPart). ideal: "
             "printing/ideal_print.py, no engine: the same Klipper plan, hotend and laying rule with the nozzle "
             "exactly where the steppers would put it, so the axis dynamics (following error, overshoot, jitter) "
             "are left out and a part reads about 0.02-0.03 mm smaller a side than on the twin "
             "(printing/README.md, \"Ideal print (no engine)\").")


def measure_plate(pl: dict, out: Path = OUT, jobs: int = 4, log=print, mode: str = "twin") -> dict:
    run = out / MODES[mode] / pl["name"]
    beads = run / "part.beads"
    report = json.loads((run / "report.json").read_text(encoding="utf-8")) if (run / "report.json").exists() else {}
    dump = inspect_print.load_beads(beads)
    allb = dump["beads"]
    recorded = inspect_print.bead_volume_mm3(allb)
    deposited = dump["deposited_mm3"]
    plate = {"beads": int(len(allb)), "deposited_mm3": round(deposited, 1), "recorded_mm3": round(recorded, 1),
             "unrecorded_fraction": round(max(0.0, 1.0 - recorded / deposited), 4) if deposited > 0 else None,
             "laid_g": round(deposited / 1000.0 * DENSITY_G_CM3[pl["material"]], 2),
             "nozzle_crashes": (report.get("done") or {}).get("nozzle_crashes")}
    lines = allb[allb[:, 8] == 0.0]
    plate["widest_line_mm"] = round(float(lines[:, 6].max()), 3) if len(lines) else None
    plate["bead_rule"] = bead_rule_of(plate)
    parts = {}
    for label in pl["parts"]:
        t0 = time.time()
        stl = out / "plates" / pl["name"] / f"{label}.stl"
        tris = _stl_triangles(stl.read_bytes())
        parts[label] = measure_part(dump, tris, pl["material"], jobs)
        parts[label]["inspect_s"] = round(time.time() - t0, 1)
        parts[label]["mode"] = mode
        e = parts[label]["outer_surface_error_mm"]
        log(f"  {label:24s} h {parts[label]['height_mm']} outer p05/p95 {e['p05']}/{e['p95']} footprint "
            f"{parts[label]['size_error_mm']['x_mean']}/{parts[label]['size_error_mm']['y_mean']} "
            f"{parts[label]['part_g']} g strands {parts[label]['hanging_strands']['length_mm']} mm "
            f"({parts[label]['hanging_strands']['at_outline_length_mm']} mm at the outline)")
    res = {"plate": pl["name"], "mode": mode, **plate, "parts": parts}
    (run / "measured.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    return res


# ---- ideal mode -------------------------------------------------------------------------------------

def ideal_plate(pl: dict, out: Path = OUT, jobs: int = 8, log=print) -> dict:
    """One plate through printing/ideal_print.py (no engine): the same G-code and profile as
    the twin's job. Writes out/ideal/<plate>/part.beads and report.json."""
    run = out / MODES["ideal"] / pl["name"]
    run.mkdir(parents=True, exist_ok=True)
    beads, rep_path = run / "part.beads", run / "ideal_report.json"
    for old in (beads, rep_path, run / "report.json", run / "measured.json"):
        old.unlink(missing_ok=True)
    t0 = time.time()
    log(f"[{pl['name']}] ideal: {len(pl['parts'])} parts, {pl['time_s']:.0f} s planned")
    proc = subprocess.run([sys.executable, str(PRINTING / "ideal_print.py"), str(ROOT / pl["gcode"]),
                           "--printer", str(ROOT / pl["profile"]), "--out", str(beads), "--report", str(rep_path),
                           "--jobs", str(jobs)], cwd=ROOT, capture_output=True, text=True)
    ideal = json.loads(rep_path.read_text(encoding="utf-8")) if rep_path.exists() else {}
    rep = {"plate": pl["name"], "mode": "ideal", "parts": pl["parts"], "material": pl["material"],
           "support": pl["support"], "planned_s": pl["time_s"], "started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)),
           "wall_s": round(time.time() - t0, 1), "done": ideal.get("done"), "timings": ideal.get("timings"),
           "returncode": proc.returncode, "stderr": proc.stderr[-1500:] if proc.returncode else ""}
    (run / "report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    log(f"[{pl['name']}] ideal done in {rep['wall_s']} s: {rep['done']}")
    return rep


# ---- the whole set ---------------------------------------------------------------------------------

def run(plan_: dict, engines: int = 4, max_engines: int = 6, max_gpu_c: int = 75, only: Sequence[str] = (),
        out: Path = OUT, jobs: int = 4, log=print) -> dict:
    """Every plate (or `only` those), each in its own engine as the gate allows, measured as
    each finishes. Plates start longest first."""
    gate = Gate(engines=engines, max_engines=max_engines, max_gpu_c=max_gpu_c)
    plates = [p for p in plan_["plates"] if not only or p["name"] in only]
    results: Dict[str, dict] = {}
    lock = threading.Lock()
    t0 = time.time()

    def one(pl, ticket):
        rep = run_plate(pl, gate, out, log, ticket)
        try:
            meas = measure_plate(pl, out, jobs, log) if (out / "runs" / pl["name"] / "part.beads").exists() else None
        except Exception as exc:                     # a measurement failure must not lose the run
            meas = {"error": f"{type(exc).__name__}: {exc}"}
        with lock:
            results[pl["name"]] = {"run": rep, "measured": meas}

    threads = []
    for ticket, pl in enumerate(sorted(plates, key=lambda q: -q["time_s"])):
        th = threading.Thread(target=one, args=(pl, ticket), daemon=False)
        th.start()
        threads.append(th)
        time.sleep(2.0)
    for th in threads:
        th.join()
    return {"wall_s": round(time.time() - t0, 1), "plates": results}


def collect(plans: Sequence[dict], out: Path = OUT) -> dict:
    """Every plan's plates, with each run and measurement, as one record.

    A part is its (label, drawing hash). Its CURRENT plate is the newest plan's plate that holds
    its current drawing (`current_plate` on the part's row); an older plate holding the same
    drawing is REPLANNED (`replanned_by`), and a drawing replaced since is SUPERSEDED. A plate
    lists the parts it is current for in `current_parts`.

    Per plate, `modes` holds each print mode's run and measurement (twin: the engine, full
    physics; ideal: ideal_print.py, no engine). `printed`, `run`, `measured` and `mode` give the
    one a reader should use: the twin's if it finished and was measured, else the ideal's."""
    if isinstance(plans, dict):
        plans = [plans]
    plan_ = plans[0]
    rows: Dict[Tuple[str, str], dict] = {}
    for pn in plans:
        for q in pn["parts"]:
            rows.setdefault((q["label"], q.get("design_tris_sha", "")), q)
    entries = [(pn, pl) for pn in plans for pl in pn["plates"]]
    current: Dict[Tuple[str, str], str] = {}
    for pn, pl in entries:                                   # oldest plan first: the newest wins
        own = {q["label"]: q for q in pn["parts"]}
        for lab in pl["parts"]:
            if not superseded(own[lab]):
                current[(lab, own[lab].get("design_tris_sha", ""))] = pl["name"]
    plates = []
    for pn, pl in entries:
        own = {q["label"]: q for q in pn["parts"]}
        modes = {}
        for mode, folder in MODES.items():
            d = out / folder / pl["name"]
            r = json.loads((d / "report.json").read_text()) if (d / "report.json").exists() else None
            m = json.loads((d / "measured.json").read_text()) if (d / "measured.json").exists() else None
            rule = bead_rule_of(m)
            # A run on the fixed engine still going: its folder has the trace and no finished report,
            # and whatever measured.json is left there is the pre-fix run's.
            running = (d / "trace.jsonl").exists() and not (r and r.get("done"))
            if rule != "fixed" or running:
                old = out / BEFORE_FIX / folder / pl["name"]
                if (old / "measured.json").exists():
                    r = json.loads((old / "report.json").read_text()) if (old / "report.json").exists() else None
                    m = json.loads((old / "measured.json").read_text())
                    rule = "pre-fix"
                elif running:
                    r, m, rule = None, None, None
            if r or m:
                rerun = None
                if rule == "pre-fix":
                    new = json.loads((d / "report.json").read_text()) if (d / "report.json").exists() else {}
                    rerun = "in progress" if running else (
                        "finished, being measured" if new.get("done") and (new.get("started") or "") >= BEAD_FIX_AT
                        else "not started")
                modes[mode] = {"run": r, "measured": m, "printed": bool(r and r.get("done") and m and m.get("parts")),
                               "bead_rule": rule, "rerun_on_fixed_engine": rerun}
        # The result a reader should use: on the fixed laying rule, the twin's if it finished and was
        # measured, else the ideal's; a pre-fix result only when nothing on the fixed rule exists.
        use = next((k for k in ("twin", "ideal") if (modes.get(k) or {}).get("printed")
                    and modes[k].get("bead_rule") == "fixed"), None) or \
            next((k for k in ("twin", "ideal") if (modes.get(k) or {}).get("printed")), None)
        keys = [(lab, own[lab].get("design_tris_sha", "")) for lab in pl["parts"]]
        cur = [lab for (lab, sha) in keys if current.get((lab, sha)) == pl["name"]]
        replanned = {lab: current[(lab, sha)] for (lab, sha) in keys
                     if not superseded(own[lab]) and current.get((lab, sha)) not in (None, pl["name"])}
        sup = [lab for lab in pl["parts"] if superseded(own[lab])]
        status = "current" if cur else ("superseded" if sup and len(sup) == len(pl["parts"]) else "replanned")
        plates.append({**{k: pl[k] for k in ("name", "material", "support", "parts", "layout", "time_s", "cura_time_s",
                                             "filament_g", "layers", "gcode", "profile", "engine")},
                       "part_sha": {lab: sha for lab, sha in keys},
                       "status": status, "current_parts": cur, "replanned_by": replanned,
                       "superseded": status == "superseded",
                       "printed": use is not None, "mode": use,
                       "bead_rule": (modes.get(use) or {}).get("bead_rule") if use else None,
                       "run": (modes.get(use) or {}).get("run") if use else (modes.get("twin") or {}).get("run"),
                       "measured": (modes.get(use) or {}).get("measured") if use else None,
                       "modes": modes})
    parts = []
    for (lab, sha), q in rows.items():
        name = current.get((lab, sha))
        pl = next((x for x in plates if x["name"] == name), None)
        parts.append({**q, "superseded": superseded(q), "current_plate": name,
                      "mode": pl["mode"] if pl else None, "bead_rule": pl.get("bead_rule") if pl else None,
                      "modes_printed": sorted(k for k, v in ((pl or {}).get("modes") or {}).items() if v["printed"])})
    return {"parts": parts, "plates": plates, "materials": plan_["materials"], "rules": plan_["rules"],
            "superseded": {v[0]: v[1] for v in SUPERSEDED_SHA.values()}, "mode_note": MODE_NOTE}


BEFORE_FIX = "before_bead_fix"     # out/<this>/{runs,ideal}/<plate>/measured.json: the dumps laid before 2026-11-04
PRE_FIX_LABEL = "before the bead fix (2026-11-04)"
BEAD_FIX_AT = "2026-11-04 16:54:00"   # the first engine with the fix (omnisim-bin-v10beads) was built then


def before_bead_fix(rec: dict, out: Path = OUT) -> Optional[dict]:
    """The plates and parts as measured from the dumps PrintedPart laid before the 2026-11-04
    bead fix (kept under out/before_bead_fix/, re-measured by the same inspect_print), against
    the current ones: crashes, grams, beads, outer p95, footprint and the widest line."""
    old = out / BEFORE_FIX
    if not old.is_dir():
        return None
    plates, parts = {}, {}
    now = {(pl["name"], mode): (mr.get("measured") or {}) for pl in rec["plates"] for mode, mr in (pl.get("modes") or {}).items()}
    for mode, folder in MODES.items():
        for d in sorted((old / folder).glob("*")):
            f = d / "measured.json"
            if not f.exists():
                continue
            m = json.loads(f.read_text(encoding="utf-8"))
            cur = now.get((d.name, mode)) or {}
            fresh = cur.get("beads") is not None and cur.get("beads") != m.get("beads")
            r_old = json.loads((d / "report.json").read_text(encoding="utf-8")) if (d / "report.json").exists() else {}
            plates[f"{d.name}/{mode}"] = {
                "finished_before": bool(r_old.get("done")),
                "crashes_before": m.get("nozzle_crashes"), "laid_g_before": m.get("laid_g"), "beads_before": m.get("beads"),
                **({"crashes_after": cur.get("nozzle_crashes"), "laid_g_after": cur.get("laid_g"),
                    "beads_after": cur.get("beads")} if fresh else {})}
            for label, q in (m.get("parts") or {}).items():
                c = ((cur.get("parts") or {}).get(label) or {}) if fresh else {}
                parts[f"{label}/{d.name}/{mode}"] = {
                    "plate": d.name, "p95_before": q["outer_surface_error_mm"]["p95"],
                    "fx_before": q["size_error_mm"]["x_mean"], "fy_before": q["size_error_mm"]["y_mean"],
                    "widest_before": (q.get("wide_beads") or {}).get("widest_mm"),
                    **({"p95_after": c["outer_surface_error_mm"]["p95"], "fx_after": c["size_error_mm"]["x_mean"],
                        "fy_after": c["size_error_mm"]["y_mean"],
                        "widest_after": (c.get("wide_beads") or {}).get("widest_mm")} if c else {})}
    note = ("The dumps PrintedPart laid before the 2026-11-04 bead fix are kept in .tmp/spark_print/before_bead_fix/. "
            "A plate's after column is empty until it has been printed again in that mode.")
    return {"note": note, "plates": plates, "parts": parts}


def _rule_note(mr: dict) -> str:
    """A plate's mode, as PRINT.md labels it: nothing on the fixed laying rule; a pre-fix result
    says so and what became of its re-run."""
    if mr.get("bead_rule") != "pre-fix":
        return ""
    rerun = mr.get("rerun_on_fixed_engine")
    tail = {"in progress": "; re-run on the fixed engine in progress",
            "finished, being measured": "; re-run on the fixed engine finished, being measured"}.get(rerun, "")
    return f", {PRE_FIX_LABEL}{tail}"


def _hms(s: Optional[float]) -> str:
    if s is None:
        return "-"
    s = int(round(s))
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}"


def _f(v, fmt="+.3f"):
    return "-" if v is None else format(v, fmt)


def write_release(rec: dict, release: Path = RELEASE) -> Path:
    release.mkdir(parents=True, exist_ok=True)
    (release / "printed.json").write_text(json.dumps(rec, indent=1), encoding="utf-8")
    for m in MATERIALS:
        p = release / f"sovol_sv08_{m.split()[0].lower()}.json"
        p.write_text(json.dumps(material_profile(m), indent=2), encoding="utf-8")
    parts = {(p["label"], p.get("design_tris_sha", "")): p for p in rec["parts"]}
    readme = "../../../../../printing/README.md#ideal-print-no-engine"
    L = ["# SparkRover Mini: the printed parts, made on the SV08 printer twin", "",
         "Generated by `spark_rover/make_print.py`; do not hand-edit. Simulation only: these are the twin's "
         "numbers (Klipper's plan, the hotend model, the beads PrintedPart laid), not a real SV08's.", "",
         "**Mode.** `twin` is a full-physics print on the SV08 twin in the engine (servo-driven axes, "
         "PrintedPart). `ideal` is `printing/ideal_print.py`, no engine: the same G-code, Klipper plan, hotend "
         "and laying rule, with the nozzle exactly where the steppers would put it. It leaves out the axis "
         "dynamics (following error, overshoot, servo jitter), so a part reads about 0.02-0.03 mm smaller a "
         f"side than on the twin. See [Ideal print (no engine)]({readme}). An ideal result is not a twin "
         "result.", "",
         "**Laying rule.** PrintedPart's bead rule was fixed on 2026-11-04 (see the notes). A row marked "
         f"\"{PRE_FIX_LABEL}\" was laid by the old rule: its numbers stay for comparison and are not a current "
         "result. Each part's current result (`current_plate`, `mode`, `bead_rule` in printed.json) is on the "
         "fixed rule: the twin's once its re-run on the fixed engine has finished and been measured, the ideal "
         "mode's until then.", "",
         "## Plates", "",
         "| Plate | Mode | Parts | Material | Support | Printer time | Wall time | Speed | Engines on the "
         "machine (ours) | Laid g | Unrecorded | Crashes |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for pl in rec["plates"]:
        status = "" if pl.get("status", "current") == "current" else \
            (" (SUPERSEDED)" if pl.get("superseded") else " (replanned on " + ", ".join(sorted(set(pl["replanned_by"].values()))) + ")")
        modes = pl.get("modes") or {}
        if not modes:
            L.append(f"| {pl['name']}{status} | not printed | {', '.join(pl['parts'])} | {pl['material']} | "
                     f"{pl['support'] or 'none'} | {_hms(pl['time_s'])} | | | | | | |")
        for mode, mr in modes.items():
            r, m = mr.get("run") or {}, mr.get("measured") or {}
            me, oe = r.get("machine_engines") or {}, r.get("our_engines") or {}
            eng = f"{me.get('min', '-')}-{me.get('max', '-')} ({oe.get('min', '-')}-{oe.get('max', '-')})" \
                if mode == "twin" else "none (no engine)"
            state = "" if r.get("done") else (" (stopped: " + r["stopped"] + ")" if r.get("stopped") else " (not finished)")
            state += _rule_note(mr)
            speed = f"{r['speed_x_real_time']}x" if r.get("speed_x_real_time") else \
                (f"{round(pl['time_s'] / r['wall_s'])}x" if mode == "ideal" and r.get("wall_s") else "-")
            L.append(f"| {pl['name']}{status} | {mode}{state} | {', '.join(pl['parts'])} | {pl['material']} | "
                     f"{pl['support'] or 'none'} | {_hms(pl['time_s'])} | {_hms(r.get('wall_s'))} | {speed} | {eng} | "
                     f"{m.get('laid_g', '-')} | {m.get('unrecorded_fraction', '-')} | {m.get('nozzle_crashes', '-')} |")
    L += ["", "## Parts", "",
          "Height: the top layer printed / drawn. Outer surface: printed outline minus the drawing, 5th to 95th "
          "percentile over all layers but the first (fused to the brim), + outside. Footprint: printed minus drawn "
          "size, mean over those layers, X / Y. Without wide beads: the same with the beads PrintedPart laid wider "
          f"than {WIDE_BEAD_MM:g} mm left out (none since the bead fix: the columns then equal the main ones; see the notes). Part g: plastic laid inside the drawing. "
          "Hanging strands: beads laid round with a diameter above the layer height (nothing within a layer "
          "under them), inside the part, and those within 1 mm of the outline (on a surface you see).", "",
          "| Part | Plate | Mode | Support | Height printed / drawn | Outer p05 / p95 mm | Footprint X / Y mm "
          "| Without wide beads: p95, footprint X / Y | Part g (design) | Support/brim under g "
          "| Hanging strands mm (at the outline) |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    both = []
    for pl in rec["plates"]:
        for label in pl["parts"]:
            pr = parts[(label, pl["part_sha"].get(label, ""))]
            name = f"{label} (SUPERSEDED)" if pr.get("superseded") else label
            got = {}
            for mode, mr in (pl.get("modes") or {}).items():
                q = ((mr.get("measured") or {}).get("parts") or {}).get(label)
                if not q:
                    continue
                got[mode] = q
                e, sz, hs = q["outer_surface_error_mm"], q["size_error_mm"], q["hanging_strands"]
                w = q.get("without_wide_beads") or {}
                we, ws = w.get("outer_surface_error_mm") or {}, w.get("size_error_mm") or {}
                L.append(f"| {name} | {pl['name']} | {mode}{_rule_note(mr)} | {pr['support'] or 'none'} | "
                         f"{q['height_mm']['printed']} / {q['height_mm']['drawing']} | {_f(e['p05'])} / {_f(e['p95'])} | "
                         f"{_f(sz['x_mean'])} / {_f(sz['y_mean'])} | {_f(we.get('p95'))}, {_f(ws.get('x_mean'))} / "
                         f"{_f(ws.get('y_mean'))} | {q['part_g']} ({pr['design_mass_g']}) | "
                         f"{q['support_or_brim_under_g']} | {hs['length_mm']} ({hs.get('at_outline_length_mm', '-')}) |")
            if not got:
                L.append(f"| {name} | {pl['name']} | not printed | {pr['support'] or 'none'} | | | | | | | |")
            rules = {k: (pl.get("modes") or {}).get(k, {}).get("bead_rule") for k in got}
            if "twin" in got and "ideal" in got and rules.get("twin") == rules.get("ideal"):
                both.append((label, got["ideal"], got["twin"]))
    if both:
        L += ["", "## Ideal against twin", "",
              "The parts printed both ways, ideal minus twin (mm; grams in g).", "",
              "| Part | Height | Outer p05 | Outer p95 | Outer mean | Footprint X | Footprint Y | Part g |",
              "|---|---|---|---|---|---|---|---|"]
        for label, i, t in both:
            d = lambda a, b: None if a is None or b is None else a - b  # noqa: E731
            L.append(f"| {label} | {_f(d(i['height_mm']['printed'], t['height_mm']['printed']), '+.2f')} | "
                     f"{_f(d(i['outer_surface_error_mm']['p05'], t['outer_surface_error_mm']['p05']))} | "
                     f"{_f(d(i['outer_surface_error_mm']['p95'], t['outer_surface_error_mm']['p95']))} | "
                     f"{_f(d(i['outer_surface_error_mm']['mean'], t['outer_surface_error_mm']['mean']))} | "
                     f"{_f(d(i['size_error_mm']['x_mean'], t['size_error_mm']['x_mean']))} | "
                     f"{_f(d(i['size_error_mm']['y_mean'], t['size_error_mm']['y_mean']))} | "
                     f"{_f(d(i['part_g'], t['part_g']), '+.2f')} |")
    L += ["", "## Printability (before printing)", "",
          f"Cut at every layer; what a layer holds beyond the one under it grown by {SUPPORT_ANGLE_DEG:g} degrees is a "
          f"bridge (carried on two sides, at most {BRIDGE_MAX_MM:g} mm) or overhang. Over {NEEDS_SUPPORT_MM2:g} mm2 of "
          "overhang gets support: build-plate support where all of it stands over the bed, else everywhere. Thin: "
          "section narrower than one 0.4 mm line.", "",
          "| Part | Module | Material | Orientation | Support | Preflight time | Cura's estimate | Preflight g "
          "| Overhang mm2 (over the part) | Bridges mm2 (widest mm) | Thin mm3 |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in rec["parts"]:
        pa = p["printability"]
        name = f"{p['label']} (SUPERSEDED)" if p.get("superseded") else p["label"]
        L.append(f"| {name} | {p['module']} | {p['material']} | {p['turn']} | {p['support'] or 'none'} | "
                 f"{_hms(p['time_s'])} | {_hms(p.get('cura_time_s'))} | {p['filament_g']} | {pa['overhang_mm2']} "
                 f"({pa.get('overhang_over_part_mm2', '-')}) | {pa['bridge_mm2']} ({pa['widest_bridge_mm']}) | "
                 f"{pa['thinner_than_line_mm3']} |")
    L += ["", "## Materials", ""] + [f"- **{m}**: {v['max_volumetric_mm3_s']} mm3/s; {v['source']}"
                                      for m, v in rec["materials"].items()]
    L += ["", "## Notes", "",
          "- **Beads and blobs (the bead fix, 2026-11-04).** No line is laid wider than the nozzle's flat "
          f"face ({WIDE_BEAD_MM:g} mm, 2.5 nozzles): what a step pushes out beyond that, and what leaves a nozzle "
          "standing still, is a blob where it left the nozzle, pressed under the face into a disc as tall as "
          "the gap, welling up around the nozzle only past what the face holds. Blobs count in the outline "
          "and footprint as discs of their width. Before the fix a nozzle slower than 20 mm/s counted as "
          "standing still; its plastic was laid on the next step's 0.04-0.05 mm chord as one bead up to 7 mm "
          "wide, or, past 0.05 mm3, as a dome above the tip that the next step counted as a nozzle crash "
          "(every crash on these plates was one). See \"Before the bead fix\" below.",
          "- **Hanging strands** inside a part are mostly the first skin layer laid over sparse (10 %) infill, "
          "which is what a real printer does too; those at the outline are on visible surfaces."]
    if rec.get("superseded"):
        L += ["", "## Superseded", ""] + [f"- **{k}**: {v}" for k, v in rec["superseded"].items()]
    before = rec.get("before_bead_fix")
    if before:
        L += ["", "## Before the bead fix", "",
              f"{before['note']} Before -> after, the same G-code and the same measurement (blobs in the outline).", "",
              "| Plate | Mode | Crashes | Laid g | Beads |", "|---|---|---|---|---|"]
        for key, b in sorted(before["plates"].items()):
            name, mode = key.split("/")
            stopped = "" if b.get("finished_before", True) else " (stopped before the end)"
            L.append(f"| {name} | {mode}{stopped} | {b['crashes_before']} -> {b.get('crashes_after', '-')} | "
                     f"{b['laid_g_before']} -> {b.get('laid_g_after', '-')} | {b['beads_before']} -> {b.get('beads_after', '-')} |")
        L += ["", "| Part | Plate | Mode | Outer p95 mm | Footprint X / Y mm | Widest line mm |", "|---|---|---|---|---|---|"]
        for key, b in sorted(before["parts"].items()):
            label, plate, mode = key.split("/")
            L.append(f"| {label} | {plate} | {mode} | {_f(b['p95_before'])} -> {_f(b.get('p95_after'))} | "
                     f"{_f(b['fx_before'])} / {_f(b['fy_before'])} -> {_f(b.get('fx_after'))} / {_f(b.get('fy_after'))} | "
                     f"{b['widest_before']} -> {b.get('widest_after', '-')} |")
    if rec.get("summary"):
        L += ["", "## Summary", ""] + [f"- {k}: {v}" for k, v in rec["summary"].items()]
    path = release / "PRINT.md"
    path.write_text("\n".join(L) + "\n", encoding="utf-8")
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plan", action="store_true", help="orient, check, slice, preflight and pack; no engine")
    ap.add_argument("--run", action="store_true", help="print the plan's plates on the twin and measure them")
    ap.add_argument("--remeasure", action="store_true", help="measure the finished runs again")
    ap.add_argument("--ideal", action="store_true", help="print the --only plates in ideal mode (no engine), one by one")
    ap.add_argument("--mode", choices=sorted(MODES), action="append", default=[],
                    help="with --remeasure: the modes to measure again (default both)")
    ap.add_argument("--only", action="append", default=[], help="plate name (repeatable)")
    ap.add_argument("--engines", type=int, default=4, help="our engines at once (default 4)")
    ap.add_argument("--max-engines", type=int, default=6, help="no launch while the machine has this many")
    ap.add_argument("--max-gpu-c", type=int, default=75, help="no launch while the GPU is this hot")
    ap.add_argument("--jobs", type=int, default=4, help="inspect_print layer processes")
    ap.add_argument("--part", action="append", default=[], help="plan only these parts (with --tag)")
    ap.add_argument("--tag", default="", help="a later batch: plan_<tag>.json, plates <tag>NN")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--release", default=str(RELEASE))
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    plan_path = out / plan_name(a.tag)
    if a.plan or not plan_path.exists():
        plan(a.engines, out, only=a.part, tag=a.tag)
    plan_ = json.loads(plan_path.read_text(encoding="utf-8"))
    summary = {}
    if a.run:
        res = run(plan_, a.engines, a.max_engines, a.max_gpu_c, a.only, out, a.jobs)
        summary["wall_s_this_run"] = res["wall_s"]
        (out / f"run_{time.strftime('%Y%m%d_%H%M%S')}.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    every = [pl for pn in all_plans(out) for pl in pn["plates"]]
    if a.ideal:
        for name in a.only:
            pl = next(q for q in every if q["name"] == name)
            ideal_plate(pl, out)
            measure_plate(pl, out, a.jobs, mode="ideal")
    if a.remeasure:
        for pl in every:
            for mode in a.mode or sorted(MODES):
                if (not a.only or pl["name"] in a.only) and (out / MODES[mode] / pl["name"] / "part.beads").exists():
                    measure_plate(pl, out, a.jobs, mode=mode)
    rec = collect(all_plans(out), out)
    rec["before_bead_fix"] = before_bead_fix(rec, out)
    if (out / "summary.json").exists():                # the run's own account, kept with the record
        summary = {**json.loads((out / "summary.json").read_text(encoding="utf-8")), **summary}
    if summary:
        rec["summary"] = summary
    print(write_release(rec, Path(a.release)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
