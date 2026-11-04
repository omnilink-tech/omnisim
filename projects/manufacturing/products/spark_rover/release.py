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

"""SparkRover Mini's release: everything someone needs to make it on real machines.

    python projects/manufacturing/products/spark_rover/release.py

writes `release/` (the desk robot's release.py is the template):

- drawings/   one SVG per made part, laser, CNC and printed: views, every hole with its
              position and size, a title block (mm)
- models/     every made part as STL (mm): the laser and CNC parts flat as they are cut,
              the printed parts as they lie on the bed
- cam/        the laser and CNC programs the twins ran (make.py's release/make/cam), and
              print/ the SV08 plates' G-code (gzip) with the material profiles
- BOM.md      made and bought parts, quantities, materials, masses and sources, the
              electrical parts and the wire cut list included; its total is integrate.mass
- ASSEMBLY.md the build order assemble.py ran on Newton (its stages, the step pictures,
              the carriers populated on the bench), with the screws of every step
- REPORT.md   every made part as the twins made it, against its drawing; the integration,
              the wiring, the drive test, the assembly, and the open items

Nothing is typed in by hand: every number is read from the subsystem modules and from the
result files their own scripts wrote (release/make/made.json, release/make/print/
printed.json, release/drive/*.json, release/assembly/assembly_log.json,
release/electrical/wires_routed.json). The printed parts are made by make_print.py,
which may still be running: a part whose plate has not printed (or whose drawing has
changed since) is PENDING in every document, never predicted. Run this again when
make_print.py has finished and the whole release regenerates.

The integration report (FCL across ~325 solids, ~3.5 min) is cached in
.tmp/spark_rover_release/, keyed by the design's sources (robot._key()); --fresh-integration
reruns it. Engine-free: no simulator is launched.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import pickle
import re
import shutil
import sys
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RELEASE = HERE / "release"
MAKE_RELEASE = RELEASE / "make"              # make.py's: made.json, MAKE.md, cam/
PRINT_RELEASE = MAKE_RELEASE / "print"       # make_print.py's: printed.json, PRINT.md, profiles
ASSEMBLY_LOG = RELEASE / "assembly" / "assembly_log.json"
DRIVE_DIR = RELEASE / "drive"
ROUTED = RELEASE / "electrical" / "wires_routed.json"
MAKE_RUNS = ROOT / ".tmp" / "spark_rover_make"    # make.py's --out: the height maps
TMP = ROOT / ".tmp" / "spark_rover_release"
README = HERE / "README.md"
NEWTON_RUNTIME = ROOT / "src/omnisim/physics/omnisim_newton_runtime.py"
README_BEGIN = "<!-- release.py: begin (generated; do not hand-edit) -->"
README_END = "<!-- release.py: end -->"
INK, CUT, HATCH, FILL = "#1b1f24", "#c0392b", "#9aa4ad", "#f4efe6"

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import skeleton as sk  # noqa: E402
import integrate  # noqa: E402


def _ours(name: str):
    """This product's module `name`. The desk robot has a robot.py, an electrical.py, an
    assemble.py and a make.py too, and the subsystem modules put its folder on sys.path, so
    a plain import may find the wrong one (assemble.py's rule)."""
    m = sys.modules.get(name)
    if m is not None and Path(getattr(m, "__file__", "") or "").resolve().parent == HERE:
        return m
    key = f"spark_rover_{name}"
    if key not in sys.modules:
        spec = importlib.util.spec_from_file_location(key, HERE / f"{name}.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
    return sys.modules[key]


def _rel(p) -> str:
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(p)


def _read_json(p: Path):
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


def _g(v, nd: int = 2) -> str:
    """A number for a table: '-' for None, else rounded and without a trailing '.0'."""
    if v is None:
        return "-"
    if isinstance(v, (bool, str)):
        return str(v)
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _hms(s: Optional[float]) -> str:
    if s is None:
        return "-"
    s = int(round(s))
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}"


# ---- what made which part -------------------------------------------------------------------------

# Design label -> (make.py job, the part's label in that job). make.py names the parts by their
# place on the sheet; the design names them by their place on the robot. Identical parts (the
# four spacers, the two brackets) may be any of the cut ones. test_spark_rover_release holds
# this table to both sides, so a part added to either one cannot be left out.
MADE_AS = {
    "deck_lower": ("deck_lower", "deck_lower"), "deck_upper": ("deck_upper", "deck_upper"),
    "spacer_fl": ("spacers", "spacer_1"), "spacer_fr": ("spacers", "spacer_2"),
    "spacer_rl": ("spacers", "spacer_3"), "spacer_rr": ("spacers", "spacer_4"),
    "spine_side_l": ("spine_side_l", "spine_side_l"), "spine_side_r": ("spine_side_r", "spine_side_r"),
    "spine_rear": ("spine_rear", "spine_rear"), "spine_front": ("spine_front", "spine_front"),
    "spine_rung_mid": ("spine_rungs", "mid_rung"), "spine_rung_top": ("spine_rungs", "top_rung"),
    "spine_foot": ("spine_foot", "spine_foot"),
    "shoulder_bracket_left": ("shoulder_brackets", "shoulder_bracket_1"),
    "shoulder_bracket_right": ("shoulder_brackets", "shoulder_bracket_2"),
    "face_cover": ("face_cover", "face_cover"),
}
# One drawing for parts that are the same part (same outline, same program).
SAME_DRAWING = {"spacer": ("spacer_fl", "spacer_fr", "spacer_rl", "spacer_rr"),
                "spine_side": ("spine_side_l", "spine_side_r"),
                "shoulder_bracket": ("shoulder_bracket_left", "shoulder_bracket_right")}


def drawing_of(label: str) -> str:
    for name, labels in SAME_DRAWING.items():
        if label in labels:
            return name
    return label


# ---- the context: everything read once ----------------------------------------------------------

class Ctx:
    """The design and every result file, loaded once."""

    def __init__(self, integration: str = "cached", height_maps: bool = True, log=print):
        self.log = log
        self.mods = integrate.load()
        wrong = {k: m.__file__ for k, m in self.mods.items() if Path(m.__file__).resolve().parent != HERE}
        if wrong:
            raise RuntimeError(f"integrate.load() returned modules from elsewhere: {wrong}")
        self.rb = _ours("robot")
        self.d = self.rb.design()
        self.parts = self.d["parts"]
        self.table = {p["label"]: p for p in self.parts}
        # Before make_print: it puts printing/ first on sys.path, and printing/preflight.py then
        # shadows cnc/preflight.py for shell.checks()' own `import preflight`.
        self.integration = integration_report(self, integration)
        self.make = _ours("make")
        self.mp = _ours("make_print")
        self.made = _read_json(MAKE_RELEASE / "made.json") or {"jobs": {}}
        self.printed = _read_json(PRINT_RELEASE / "printed.json")
        self.assembly = _read_json(ASSEMBLY_LOG)
        self.wires = _read_json(ROUTED)
        self.drives = OrderedDict((p.stem, json.loads(p.read_text(encoding="utf-8")))
                                  for p in sorted(DRIVE_DIR.glob("*.json")))
        self.jobs = {j.name: j for j in self.make.all_jobs(None)}
        self.print_parts = printed_now(self)
        self.status = print_status(self.printed, {k: v["sha"] for k, v in self.print_parts.items()})
        self.thickness = edge_thickness(self) if height_maps else {}

    def made_rec(self, label: str) -> Tuple[Optional[dict], Optional[dict], Optional[str]]:
        """(the job's record, the part's measurement, the job it was cut as) from made.json."""
        job, part = MADE_AS[label]
        rec = self.made["jobs"].get(job)
        if rec and rec.get("same_as"):
            src = self.made["jobs"].get(rec["same_as"]) or {}
            meas = (src.get("parts") or {}).get(part.replace(job, rec["same_as"]))
            return src, meas, rec["same_as"]
        return rec, (rec or {}).get("parts", {}).get(part) if rec else None, job

    def made_labels(self, process=None) -> List[str]:
        return [p["label"] for p in self.parts if p.get("kind") == "made" and (process is None or p["process"] in process)]


def printed_now(ctx: Ctx) -> Dict[str, dict]:
    """Every printed part as its module draws it NOW: make_print.printed_parts (the jobs' STLs in
    the module's print orientation), with make_print's design hash. Cached by robot._key()."""
    TMP.mkdir(parents=True, exist_ok=True)
    f = TMP / f"printed_parts_{ctx.rb._key()}.pkl"
    if f.exists():
        with f.open("rb") as fh:
            return pickle.load(fh)
    out = {}
    for p in ctx.mp.printed_parts(None):
        out[p["label"]] = {**p, "sha": ctx.mp._sha(p["tris"])}
    with f.open("wb") as fh:
        pickle.dump(out, fh)
    return out


# ---- the printed parts' state ----------------------------------------------------------------------

def print_status(printed: Optional[dict], current: Dict[str, str]) -> Dict[str, dict]:
    """For every printed part (label -> its drawing's hash now): PRINTED, with its plate and
    measurement, or PENDING, with why. Only a plate that holds the part's CURRENT drawing
    counts; a drawing make_print has recorded as superseded never does.

    state "printed": {plate, measured (make_print.measure_part's record), plate_measured, row}
    state "pending": {plates (planned, holding the current drawing), why}"""
    out = {}
    plates = (printed or {}).get("plates") or []
    rows = (printed or {}).get("parts") or []
    for label, sha in current.items():
        row = next((r for r in rows if r["label"] == label and r.get("design_tris_sha") == sha), None)
        holding = [pl for pl in plates if label in pl.get("parts", []) and (pl.get("part_sha") or {}).get(label) == sha]
        done = [pl for pl in holding if pl.get("printed") and ((pl.get("measured") or {}).get("parts") or {}).get(label)]
        if printed is None:
            out[label] = {"state": "pending", "plates": [], "row": None, "why": "make_print.py has not written printed.json"}
        elif row is None or (row.get("superseded")):
            older = [r for r in rows if r["label"] == label]
            why = ("drawn since make_print.py planned it: not on any plate yet" if older else
                   "not in make_print.py's plan yet")
            out[label] = {"state": "pending", "plates": [], "row": None, "why": why}
        elif done:
            pl = done[-1]
            out[label] = {"state": "printed", "plate": pl["name"], "plate_rec": pl, "row": row,
                          "measured": pl["measured"]["parts"][label]}
        else:
            runs = [pl for pl in holding if pl.get("run")]
            why = "printing or queued on " + ", ".join(pl["name"] for pl in holding) if holding else "planned on no plate"
            ended = [f"{pl['name']}'s run ended: {pl['run'].get('error')}" for pl in runs if pl["run"].get("error")]
            out[label] = {"state": "pending", "plates": [pl["name"] for pl in holding], "row": row,
                          "why": why + ("; " + "; ".join(ended) if ended else "")}
    return out


def plate_for(ctx: Ctx, label: str) -> Optional[dict]:
    """The plate whose G-code makes this part: the one it printed on, else the newest that holds
    its current drawing."""
    st = ctx.status.get(label) or {}
    plates = {pl["name"]: pl for pl in (ctx.printed or {}).get("plates", [])}
    if st.get("state") == "printed":
        return plates.get(st["plate"])
    names = st.get("plates") or []
    return plates.get(names[-1]) if names else None


# ---- the integration report (cached) ----------------------------------------------------------------

def integration_report(ctx: Ctx, mode: str = "cached") -> dict:
    """integrate.report(): 'cached' (by robot._key(); computed when missing), 'fresh', 'none'
    (no FCL interference run: mass, interfaces and the design rules) or 'mass' (mass and
    interfaces only, for a quick check)."""
    TMP.mkdir(parents=True, exist_ok=True)
    f = TMP / f"integrate_{ctx.rb._key()}.json"
    if mode in ("none", "mass"):
        rep = {"interference": None, "zones_left": None, "mounts": integrate.mounts(ctx.mods),
               "mass": integrate.mass(ctx.mods), "skeleton": sk.check_skeleton(),
               "checks": {o: m.checks(None) for o, m in ctx.mods.items() if hasattr(m, "checks")} if mode == "none" else {}}
        return json.loads(json.dumps(rep, default=str))
    if mode == "cached" and f.exists():
        return json.loads(f.read_text(encoding="utf-8"))
    ctx.log("integrate.report(): FCL across every solid (a few minutes)")
    rep = json.loads(json.dumps(integrate.report(), default=str))
    f.write_text(json.dumps(rep, indent=1), encoding="utf-8")
    return rep


# ---- the laser parts' thickness, from the saved height maps -----------------------------------------

EDGE_RING_MM = 0.25           # cells this close to a cut edge are the kerf's flank
SEAM_BAND_MM = 0.6            # cells this close to a window seam


def edge_thickness(ctx: Ctx) -> Dict[str, dict]:
    """For each laser part whose 1st-percentile thickness reads under its sheet: where the thin
    cells are. made.json's thickness_min_mm is the 1st percentile over EVERY cell of the part;
    here the cells within EDGE_RING_MM of a cut edge (the kerf's V-shaped flank) are split from
    the interior, and the interior cells along a window seam (make.py's stitching) are counted
    apart. Reads make.py's height maps (.tmp/spark_rover_make/runs) when they are there; cached
    by the maps' sizes and times."""
    from scipy import ndimage
    mk = ctx.make
    jobs = [j for j in ctx.jobs.values() if j.process == "laser" and not j.same_as]
    todo = []
    for j in jobs:
        rec = ctx.made["jobs"].get(j.name) or {}
        thin = [lab for lab, p in (rec.get("parts") or {}).items()
                if p.get("cut_free") and p["thickness_min_mm"] < p["drawn_thickness_mm"] - 0.05]
        wins = mk.windows(j)
        maps = [mk._run_dir(MAKE_RUNS, j, k if len(wins) > 1 else None) / "sheet.zmap" for k in range(len(wins))]
        if thin and all(m.exists() for m in maps):
            todo.append((j, thin, maps))
    if not todo:
        return {}
    key = hashlib.sha256(json.dumps([(j.name, [(_rel(m), m.stat().st_size, int(m.stat().st_mtime)) for m in maps])
                                     for j, _, maps in todo]).encode()).hexdigest()[:16]
    TMP.mkdir(parents=True, exist_ok=True)
    f = TMP / f"thickness_{key}.json"
    if f.exists():
        return json.loads(f.read_text(encoding="utf-8"))
    inspect_cut = mk.inspect_cut
    ctx.log(f"reading the height maps of {len(todo)} laser jobs")
    out = {}
    for j, thin, maps in todo:
        wins = mk.windows(j)
        z = mk.stitch(j, [inspect_cut.load(m) for m in maps]) if len(maps) > 1 else inspect_cut.load(maps[0])
        r = z["res"] * 1000.0
        h = z["h"] * 1000.0
        sx, sy = j.sheet_mm[:2]
        loose = [p for p in inspect_cut.pieces(z) if p["fell_out"]]
        seams_y = sorted({w[1] for w in wins} | {w[3] for w in wins})[1:-1]
        seams_x = sorted({w[0] for w in wins} | {w[2] for w in wins})[1:-1]
        yy, xx = np.mgrid[0:h.shape[0], 0:h.shape[1]]
        X, Y = -sx / 2 + xx * r, -sy / 2 + yy * r
        seam = np.zeros(h.shape, bool)
        for s in seams_y:
            seam |= np.abs(Y - s) < SEAM_BAND_MM
        for s in seams_x:
            seam |= np.abs(X - s) < SEAM_BAND_MM
        t0 = j.sheet_mm[2]
        for label in thin:
            poly = j.drawn[label]
            rp = poly.representative_point()
            jj, ii = mk._ij(z, rp.x, rp.y)
            part = next((p for p in loose if p["_mask"][jj, ii]), None)
            if part is None:
                continue
            mask = part["_mask"]
            dist = ndimage.distance_transform_edt(mask) * r
            ring = mask & (dist <= EDGE_RING_MM)
            inner = mask & ~ring
            low = inner & (h < t0 - 0.01)
            on_seam = low & seam
            out[f"{j.name}/{label}"] = {
                "job": j.name, "part": label, "drawn_mm": t0, "windows": len(wins),
                "p1_all_cells_mm": round(float(np.percentile(h[mask], 1)), 3),
                "edge_ring_share_pct": round(100.0 * ring.sum() / mask.sum(), 2),
                "edge_ring_p1_mm": round(float(np.percentile(h[ring], 1)), 3),
                "edge_ring_median_mm": round(float(np.median(h[ring])), 3),
                "interior_p1_mm": round(float(np.percentile(h[inner], 1)), 3),
                "interior_min_mm": round(float(h[inner].min()), 3),
                "interior_cells_below_drawn": int(low.sum()), "of_interior_cells": int(inner.sum()),
                "of_those_on_a_seam": int(on_seam.sum()),
                "seam_dip_mm": round(float(t0 - h[on_seam].min()), 3) if on_seam.any() else 0.0,
            }
    f.write_text(json.dumps(out, indent=1), encoding="utf-8")
    return out


# ---- drawings ----------------------------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["svg.hashsalt"] = "spark_rover"
    plt.rcParams["svg.fonttype"] = "none"
    return plt


def _polys(g) -> list:
    if g is None or g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    return [q for q in getattr(g, "geoms", []) if q.geom_type == "Polygon" and not q.is_empty]


def _patch(g, face=FILL, edge=INK, lw=0.8, hatch=None, ls="-"):
    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MPath
    verts, codes = [], []
    for p in _polys(g):
        for ring in [p.exterior, *p.interiors]:
            pts = list(ring.coords)
            verts += pts
            codes += [MPath.MOVETO] + [MPath.LINETO] * (len(pts) - 2) + [MPath.CLOSEPOLY]
    return PathPatch(MPath(verts, codes), facecolor=face, edgecolor=edge, lw=lw, hatch=hatch, ls=ls)


def hole_label(ring) -> str:
    """A hole by its shape: round (its diameter), a hex (across flats) or a slot / window (W x L)."""
    from shapely.geometry import Polygon
    h = Polygon(ring)
    x0, y0, x1, y1 = h.bounds
    w, l = x1 - x0, y1 - y0
    if abs(w - l) < 0.05 * max(w, l) and abs(h.area - math.pi * (w / 2) ** 2) < 0.03 * h.area:
        return f"⌀{(w + l) / 2:.2f}"
    simple = h.simplify(0.01)
    if len(simple.exterior.coords) == 7:                       # six corners, closed
        return f"hex {math.sqrt(3.0) * simple.length / 6:.2f} AF"          # a regular hexagon: AF = sqrt(3) x side
    return f"{w:.2f} × {l:.2f}"


def holes_of(g) -> List[Tuple[float, float, str]]:
    out = []
    from shapely.geometry import Polygon
    for p in _polys(g):
        for ring in p.interiors:
            c = Polygon(ring).centroid
            out.append((round(c.x, 3) + 0.0, round(c.y, 3) + 0.0, hole_label(ring)))
    return sorted(out, key=lambda t: (round(t[1], 1), t[0]))


def _dims(ax, g, pad):
    x0, y0, x1, y1 = g.bounds
    ax.annotate("", (x0, y1 + pad * 0.5), (x1, y1 + pad * 0.5), arrowprops=dict(arrowstyle="<->", lw=0.6, color=INK))
    ax.text((x0 + x1) / 2, y1 + pad * 0.5, f"{x1 - x0:.2f}", ha="center", va="bottom", fontsize=7, color=INK)
    ax.annotate("", (x0 - pad * 0.5, y0), (x0 - pad * 0.5, y1), arrowprops=dict(arrowstyle="<->", lw=0.6, color=INK))
    ax.text(x0 - pad * 0.5, (y0 + y1) / 2, f"{y1 - y0:.2f}", ha="right", va="center", rotation=90, fontsize=7, color=INK)


def _view(ax, g, title, holes=None, blind=None, centre_lines=True):
    x0, y0, x1, y1 = g.bounds
    pad = 0.14 * max(x1 - x0, y1 - y0, 1.0)
    ax.add_patch(_patch(g))
    for k, (hx, hy, _) in enumerate(holes or [], 1):
        ax.text(hx, hy, f"H{k}", fontsize=5, ha="center", va="center", color=CUT)
    for k, b in enumerate(blind or [], 1):
        from shapely.geometry import Point
        ax.add_patch(_patch(Point(b[0], b[1]).buffer(b[2] / 2, quad_segs=16), face="none", edge=CUT, lw=0.6, ls="--"))
        ax.text(b[0], b[1] + b[2], f"T{k}", fontsize=5, ha="center", va="bottom", color=CUT)
    if centre_lines:
        ax.plot([x0 - pad * 0.2, x1 + pad * 0.2], [0, 0], lw=0.3, ls="-.", color=HATCH)
        ax.plot([0, 0], [y0 - pad * 0.2, y1 + pad * 0.2], lw=0.3, ls="-.", color=HATCH)
    _dims(ax, g, pad)
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=8, loc="left")
    ax.tick_params(labelsize=5.5)


def _table_and_block(fig, rows: Sequence[str], block: Sequence[str], top: float = 0.25):
    tx = fig.add_axes([0.04, 0.02, 0.92, top])
    tx.axis("off")
    per = 11
    cols = [rows[i:i + per] for i in range(0, len(rows), per)] or [["(no holes)"]]
    shown = cols[:3]
    for c, col in enumerate(shown):
        tx.text(0.0 + 0.215 * c, 1.0, "\n".join(col), fontsize=5.8, family="monospace", va="top")
    if len(cols) > 3:
        tx.text(0.0, -0.02, f"(+{sum(len(c) for c in cols[3:])} more holes: see the model)", fontsize=5.8, va="top")
    tx.text(1.0, 1.0, "\n".join(block), fontsize=7, ha="right", va="top",
            bbox=dict(boxstyle="square,pad=0.6", fc="white", ec=INK, lw=0.8))


def _title_block(title: str, meta: Sequence[str], source: str) -> List[str]:
    return [title, *meta, "Units mm. Unless stated: holes through, ±0.1",
            f"OmniSim SparkRover Mini · OmniLink · {source}", "Simulation only: made on OmniSim's machine twins"]


def drawing_flat(path: Path, title: str, top, thickness: float, meta: Sequence[str], source: str,
                 blind: Sequence[Tuple[float, float, float, float]] = ()) -> None:
    """A flat part (laser or CNC): its top view (datum at the centre lines), every through hole
    and every blind feature tabled, a title block."""
    plt = _plt()
    fig = plt.figure(figsize=(11.69, 8.27))
    ax = fig.add_axes([0.05, 0.32, 0.9, 0.62])
    holes = holes_of(top)
    _view(ax, top, "TOP (datum X0 Y0 at the centre lines; Z up from the face on the bed)", holes, blind)
    fmt = lambda v: f"{(0.0 if abs(v) < 5e-4 else v):+8.2f}"   # noqa: E731
    rows = [f"H{k:<2d} X{fmt(x)} Y{fmt(y)}  {lab}" for k, (x, y, lab) in enumerate(holes, 1)]
    rows += [f"T{k:<2d} X{fmt(b[0])} Y{fmt(b[1])}  ⌀{b[2]:.2f} × {b[3]:.2f} deep" for k, b in enumerate(blind, 1)]
    _table_and_block(fig, rows, _title_block(title, [*meta, f"Thickness {thickness:g} mm"], source))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, metadata={"Date": None})
    plt.close(fig)


_VIEWS = {"front": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),     # X right, Z up (from -y)
          "side": np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)}       # Y right, Z up (from +x)


def projections(tris: np.ndarray) -> Dict[str, object]:
    """The part's silhouette along Z (top), Y (front) and X (side), and its section at mid-height."""
    import trimesh
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from trimesh.path.polygons import projected
    m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
    m.merge_vertices()
    out = {"top": projected(m, normal=[0, 0, 1])}
    for name, R in _VIEWS.items():
        mm = m.copy()
        T = np.eye(4)
        T[:3, :3] = R
        mm.apply_transform(T)
        out[name] = projected(mm, normal=[0, 0, 1])
    z0, z1 = m.bounds[0][2], m.bounds[1][2]
    zc = (z0 + z1) / 2
    sec = m.section(plane_origin=[0, 0, zc], plane_normal=[0, 0, 1])
    polys = []
    if sec is not None:
        p2, to3 = sec.to_2D(to_2D=np.eye(4))
        polys = [Polygon(p.exterior.coords, [i.coords for i in p.interiors]) for p in p2.polygons_full]
    out["section"] = unary_union(polys) if polys else None
    out["section_z"] = zc
    return out


def drawing_printed(path: Path, title: str, tris: np.ndarray, meta: Sequence[str], source: str) -> dict:
    """A printed part as it lies on the bed: top (with the holes through it along Z), front and
    side views, the section at mid-height, a title block."""
    plt = _plt()
    pr = projections(tris)
    fig = plt.figure(figsize=(11.69, 8.27))
    holes = holes_of(pr["top"])
    _view(fig.add_axes([0.04, 0.585, 0.42, 0.36]), pr["top"], "TOP (on the bed, Z up; H: holes through along Z)", holes)
    if pr["section"] is not None:
        ax = fig.add_axes([0.54, 0.585, 0.42, 0.36])
        _view(ax, pr["section"], f"SECTION at Z {pr['section_z']:.2f} (the layer at mid-height)")
        for p in _polys(pr["section"]):
            ax.add_patch(_patch(p, face=HATCH, lw=0.5, hatch="////"))
    _view(fig.add_axes([0.04, 0.27, 0.42, 0.24]), pr["front"], "FRONT (X right, Z up)", centre_lines=False)
    _view(fig.add_axes([0.54, 0.27, 0.42, 0.24]), pr["side"], "SIDE (Y right, Z up)", centre_lines=False)
    fmt = lambda v: f"{(0.0 if abs(v) < 5e-4 else v):+8.2f}"   # noqa: E731
    rows = [f"H{k:<2d} X{fmt(x)} Y{fmt(y)}  {lab}" for k, (x, y, lab) in enumerate(holes, 1)]
    v = tris.reshape(-1, 3)
    size = v.max(0) - v.min(0)
    _table_and_block(fig, rows, _title_block(title, [*meta, f"Size on the bed {size[0]:.2f} × {size[1]:.2f} × {size[2]:.2f}"],
                                             source), top=0.22)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, metadata={"Date": None})
    plt.close(fig)
    return {"holes": len(holes), "size_mm": [round(float(s), 2) for s in size]}


def _centred(poly):
    from shapely import affinity
    x0, y0, x1, y1 = poly.bounds
    return affinity.translate(poly, -(x0 + x1) / 2, -(y0 + y1) / 2), ((x0 + x1) / 2, (y0 + y1) / 2)


def flat_parts(ctx: Ctx) -> "OrderedDict[str, dict]":
    """Every laser and CNC drawing: its outline (centred), thickness, blind features, process notes."""
    chassis, arms = ctx.mods["chassis"], ctx.mods["arms"]
    out = OrderedDict()
    for label in ctx.made_labels(("laser", "cnc")):
        name = drawing_of(label)
        if name in out:
            out[name]["labels"].append(label)
            continue
        job_name, part = MADE_AS[label]
        job = ctx.jobs[job_name]
        drawn = job.drawn[part]
        job = ctx.jobs[job.same_as] if job.same_as else job       # the program that cuts it
        poly, (cx, cy) = _centred(drawn)
        blind = [(b.x - cx, b.y - cy, b.d, b.depth) for b in job.blind.get(part, [])]
        row = ctx.table[label]
        t = job.sheet_mm[2]
        prog = f"cam/{job.name}.{job.ext}"
        if job.process == "laser":
            s = chassis.LASER[t]
            meta = [f"Laser cut, {row['material']}", f"Sienci LaserBeam 7W: {s['passes']} passes, Z {s['z_step']:g} mm a pass, "
                    f"{s['mm_s']:g} mm/s (chassis.LASER)", f"Kerf offset {chassis.LASER_KERF_MM:g} mm in the program ({prog})"]
        elif label.startswith("shoulder_bracket"):
            meta = [f"CNC milled, {row['material']}", f"Both from one {' x '.join(_g(v) for v in arms.BRACKET_STOCK_MM)} mm blank ({prog})",
                    "T: tap pilots as milled; then drill ⌀2.5 through on a drill press and tap M3 by hand"]
        elif label == "face_cover":
            meta = [f"CNC routed, {row['material']}", f"1/8 in single-flute O-flute (cam/face_cover_tools.json), "
                    f"blank {' x '.join(_g(v) for v in ctx.make.COVER_BLANK_MM)} mm ({prog})",
                    "Bonded into the shell's rebate (shell step 7)"]
        else:
            meta = [f"CNC milled, {row['material']}", f"Blank {' x '.join(_g(v) for v in chassis.SPINE_FOOT_STOCK_MM)} mm, "
                    f"longmill_mk2_tools_aluminium_fine.json ({prog})"]
        out[name] = {"labels": [label], "poly": poly, "thickness": t, "blind": blind, "meta": meta, "job": job.name,
                     "program": prog, "process": job.process, "owner": row["owner"], "material": row["material"]}
    return out


def write_drawings(ctx: Ctx, out: Path) -> List[str]:
    files = []
    for name, f in flat_parts(ctx).items():
        qty = len(f["labels"])
        title = f"{name.replace('_', ' ').capitalize()} — qty {qty}" + (f" ({', '.join(f['labels'])})" if qty > 1 else "")
        p = out / "drawings" / f"{name}.svg"
        drawing_flat(p, title, f["poly"], f["thickness"], f["meta"], f"{f['owner']}.py", f["blind"])
        files.append(p.name)
    for label, pp in ctx.print_parts.items():
        tris, turn = bed_triangles(ctx, label)
        row = ctx.table[label]
        st = ctx.status.get(label, {})
        r = st.get("row") or {}
        meta = [f"3D printed, {row['material']}, qty 1, on the Sovol SV08 (CuraEngine, cam/print/)",
                f"Orientation: {turn}; support: {r.get('support') or 'none'}",
                f"Design mass {_g(row['mass_g'])} g; colour {row.get('colour') or '-'}"]
        p = out / "drawings" / f"{label}.svg"
        drawing_printed(p, label.replace("_", " ").capitalize(), tris, meta, f"{pp['module']}.py")
        files.append(p.name)
    return files


def bed_triangles(ctx: Ctx, label: str) -> Tuple[np.ndarray, str]:
    """A printed part as it lies on the bed: the module's orientation, turned as make_print chose."""
    pp = ctx.print_parts[label]
    row = (ctx.status.get(label) or {}).get("row") or {}
    turn = row.get("turn") or "as designed"
    return ctx.mp.turned(pp["tris"], turn), turn


# ---- models and machine files -----------------------------------------------------------------------

def write_models(ctx: Ctx, out: Path) -> List[str]:
    chassis = ctx.mods["chassis"]
    from shapely.geometry import Point
    files = []
    for name, f in flat_parts(ctx).items():
        t = f["thickness"]
        if f["blind"]:
            depth = max(b[3] for b in f["blind"])
            cut = f["poly"].difference(Point(0, 0).buffer(0))
            for b in f["blind"]:
                cut = cut.difference(Point(b[0], b[1]).buffer(b[2] / 2, quad_segs=16))
            tris = np.concatenate([chassis.p_xy(f["poly"], 0.0, t - depth), chassis.p_xy(cut, t - depth, t)])
        else:
            tris = chassis.p_xy(f["poly"], 0.0, t)
        p = out / "models" / f"{name}.stl"
        ctx.mp.write_stl(np.asarray(tris, float), p, f"SparkRover {name}")
        files.append(p.name)
    for label in ctx.print_parts:
        tris, _ = bed_triangles(ctx, label)
        p = out / "models" / f"{label}.stl"
        ctx.mp.write_stl(np.asarray(tris, float), p, f"SparkRover {label}")
        files.append(p.name)
    return files


def write_cam(ctx: Ctx, out: Path) -> Dict[str, List[str]]:
    """The laser / CNC programs make.py ran, and the G-code of each plate that makes a printed
    part (gzip: the plates run to 14 MB each)."""
    cam = out / "cam"
    cam.mkdir(parents=True, exist_ok=True)
    got = {"machine": [], "print": [], "missing": []}
    src = MAKE_RELEASE / "cam"
    for p in sorted(src.glob("*")) if src.exists() else []:
        shutil.copyfile(p, cam / p.name)
        got["machine"].append(p.name)
    pc = cam / "print"
    pc.mkdir(exist_ok=True)
    for p in sorted(PRINT_RELEASE.glob("sovol_sv08_*.json")):
        shutil.copyfile(p, pc / p.name)
    for name in sorted({pl["name"] for pl in (plate_for(ctx, lab) for lab in ctx.print_parts) if pl}):
        pl = next(x for x in ctx.printed["plates"] if x["name"] == name)
        g = ROOT / pl["gcode"]
        dst = pc / f"{name}.gcode.gz"
        if not g.exists():
            got["missing"].append(pl["gcode"])
            continue
        if not dst.exists() or dst.stat().st_mtime < g.stat().st_mtime:
            dst.write_bytes(gzip.compress(g.read_bytes(), 6, mtime=0))
        got["print"].append(dst.name)
    return got


# ---- BOM -----------------------------------------------------------------------------------------------

def _stem(label: str) -> str:
    """A part's label without its place: wheel corner, side, index (screw_spine_lb -> screw_spine)."""
    return re.sub(r"_(fl|fr|rl|rr|left|right|l|r|lb|rb|lf|rf|\d+)(?=_|$)", "", label)


def bom_rows(ctx: Ctx) -> Tuple[List[dict], List[dict]]:
    """(made rows, bought rows): every part of the design once, the bought ones grouped by
    (subsystem, spec, mass, source). Their masses add up to integrate.mass()."""
    made, bought = OrderedDict(), OrderedDict()
    for p in ctx.parts:
        m = float(p.get("mass_g") or 0.0)
        if p.get("kind") == "made":
            name = drawing_of(p["label"]) if p["process"] in ("laser", "cnc") else p["label"]
            r = made.setdefault(name, {"part": name, "labels": [], "owner": p["owner"], "process": p["process"],
                                       "material": p["material"], "mass_each_g": m, "qty": 0, "mass_g": 0.0})
            r["labels"].append(p["label"])
            r["qty"] += 1
            r["mass_g"] += m
        else:
            spec = p.get("spec") or ""
            if not spec and p["owner"] == "shell" and p["label"].startswith("insert"):
                spec = ctx.mods["shell"].INSERT["spec"] if p["mass_g"] >= 0.5 else "M2 brass heat-set insert"
            if not spec:
                spec = f"{_stem(p['label']).replace('_', ' ')} ({p.get('material') or '-'})"
            key = (p["owner"], spec, round(m, 3), p.get("source") or "")
            r = bought.setdefault(key, {"item": _stem(p["label"]), "owner": p["owner"], "spec": spec, "mass_each_g": m,
                                        "source": p.get("source") or "", "qty": 0, "mass_g": 0.0, "labels": [],
                                        "ref": p.get("ref"), "where": p.get("where"), "note": p.get("note")})
            r["qty"] += 1
            r["mass_g"] += m
            r["labels"].append(p["label"])
            r["material"] = p.get("material")
    rows = list(bought.values())
    for r in rows:
        stems = list(OrderedDict.fromkeys(_stem(x) for x in r["labels"]))
        r["item"] = " / ".join(stems) if len(stems) <= 3 else f"{stems[0]} (+{len(stems) - 1} more)"
        # two rows of one subsystem under one spec (a servo's case and its horn): say which is which
        dup = [q for q in rows if q["owner"] == r["owner"] and q["spec"] == r["spec"]]
        if len({q.get("material") for q in dup}) > 1:
            r["spec_shown"] = f"{r['spec']}: {r['material']}"
    return list(made.values()), rows


def bom_total_g(ctx: Ctx) -> float:
    made, bought = bom_rows(ctx)
    return sum(r["mass_g"] for r in made) + sum(r["mass_g"] for r in bought)


def _source_md(s: str) -> str:
    s = (s or "").strip()
    return s.replace("|", "/") if s else "-"


def bom_md(ctx: Ctx) -> str:
    made, bought = bom_rows(ctx)
    total = sum(r["mass_g"] for r in made) + sum(r["mass_g"] for r in bought)
    im = ctx.integration["mass"]
    lo, hi = sk.MASS_TARGET_G
    L = ["# SparkRover Mini: bill of materials", "",
         f"Generated by `release.py` from the subsystem modules' `parts()`; do not hand-edit. Total mass, as designed: "
         f"**{total:.1f} g** (integrate.mass: {im['total_g']} g; the skeleton's ceiling {hi:g} g, margin {hi - total:.1f} g), "
         f"centre of mass {im['com_mm']} mm (robot frame: origin on the floor under the wheels' centre).", "",
         "Masses are the design's: a printed part's is its module's estimate (the shell's from its solid volume, the "
         "electrical carriers' at their infill), not the twin's. What the printer twin laid is in [REPORT.md](REPORT.md).",
         "", "## Made", "",
         "| Part | Subsystem | Process | Material | Qty | Mass each (g) | Drawing | Model | Machine file |",
         "|---|---|---|---|---|---|---|---|---|"]
    flat = flat_parts(ctx)
    for r in made:
        if r["part"] in flat:
            target = flat[r["part"]]["program"]
            tail = " (run twice: both side plates)" if len(r["labels"]) > 1 and r["part"] == "spine_side" else ""
        else:
            pl = plate_for(ctx, r["part"])
            target = f"cam/print/{pl['name']}.gcode.gz" if pl else None
            tail = f" ({len(pl['parts'])} parts on the plate)" if pl else ""
        file_md = f"[{target}]({target}){tail}" if target else "not on a plate yet"
        L.append(f"| {r['part']} | {r['owner']} | {r['process']} | {r['material']} | {r['qty']} | {_g(r['mass_each_g'])} | "
                 f"[drawings/{r['part']}.svg](drawings/{r['part']}.svg) | [models/{r['part']}.stl](models/{r['part']}.stl) | "
                 f"{file_md} |")
    L.append(f"\nMade parts: {sum(r['qty'] for r in made)}, {sum(r['mass_g'] for r in made):.1f} g.")
    for owner in ("chassis", "shell", "arms"):
        rows = [r for r in bought if r["owner"] == owner]
        if not rows:
            continue
        L += ["", f"## Bought: {owner}", "", "| Item | Spec | Qty | Mass each (g) | Source |", "|---|---|---|---|---|"]
        for r in rows:
            note = f" ({r['note']})" if r.get("note") else ""
            L.append(f"| {r['item']} | {_source_md(r.get('spec_shown', r['spec']) + note)} | {r['qty']} | {_g(r['mass_each_g'], 3)} | "
                     f"{_source_md(r['source'])} |")
        L.append(f"\n{sum(r['qty'] for r in rows)} items, {sum(r['mass_g'] for r in rows):.1f} g.")
    L += electrical_bom_md(ctx, [r for r in bought if r["owner"] == "electrical"])
    L += stock_md(ctx)
    return "\n".join(L) + "\n"


def electrical_bom_md(ctx: Ctx, rows: List[dict]) -> List[str]:
    el = ctx.mods["electrical"]
    comps = el.components()
    L = ["", "## Bought: electrical", "",
         "From `electrical.py`; the schematic, wiring diagram, wire list, placement and power budget are its own output in "
         "[electrical/](electrical/).", "",
         "| Ref | Part | Where | Qty | Mass each (g) | Source |", "|---|---|---|---|---|---|"]
    for r in rows:
        ref = r.get("ref") if r.get("ref") not in (None, "-") else r["item"]
        part = comps.get(r.get("ref"), {}).get("part") if r.get("ref") in comps else r["spec"]
        L.append(f"| {ref} | {_source_md(part)} | {r.get('where') or '-'} | {r['qty']} | {_g(r['mass_each_g'], 3)} | "
                 f"{_source_md(r['source'])} |")
    L.append(f"\n{sum(r['qty'] for r in rows)} items, {sum(r['mass_g'] for r in rows):.1f} g (the harness by electrical.wire_mass_g, "
             "on its estimated lengths).")
    if ctx.wires:
        h = _ours("harness")
        s = h.summary(ctx.wires)
        L += ["", "## Wires: the cut list", "",
              f"Routed in 3D through the assembled robot by `harness.py` (the desk robot's router, a {h.GRID_MM:g} mm grid): "
              f"{s['routed']} of {s['wires']} wires routed"
              + (f", {len(s['skipped'])} not routed ({'; '.join(f'{w}: ' + h.NOT_ROUTED.get(_net(ctx, w), '') for w in s['skipped'])})"
                 if s["skipped"] else "") + ". Cut each to its length (the routed path plus the service ends). The DSI and CSI "
              "ribbons are routed along their centre line for their length only; their clearance is not claimed.", "",
              "| Wire | From | To | Net | AWG | Colour | End at From | End at To | Cut (mm) |", "|---|---|---|---|---|---|---|---|---|"]
        for w in ctx.wires:
            awg = "FPC" if w.get("ribbon") else ("cable" if w.get("awg") == 0 else w.get("awg"))
            cut = w.get("cut_mm", "not routed" if "skipped" in w else "NO ROUTE")
            L.append(f"| {w['id']} | {'.'.join(w['from'])} | {'.'.join(w['to'])} | {w['net']} | {awg} | {w.get('colour', '')} | "
                     f"{_source_md(w.get('connector_from', ''))} | {_source_md(w.get('connector_to', ''))} | {cut} |")
        L.append(f"\nTotal cut: {s['total_cut_m']} m.")
    return L


def _net(ctx: Ctx, wid: str) -> str:
    return next((w["net"] for w in ctx.wires or [] if w["id"] == wid), "")


def stock_md(ctx: Ctx) -> List[str]:
    L = ["", "## Stock", "", "| Sheet or blank | Material | Size (mm) | For |", "|---|---|---|---|"]
    for name, j in ctx.jobs.items():
        if j.same_as:
            continue
        mat = ctx.table[next(lab for lab, (jn, _) in MADE_AS.items() if jn == name)]["material"]
        times = " (twice: both side plates)" if name == "spine_side_l" else ""
        L.append(f"| {name}{times} | {mat} | {' x '.join(_g(v) for v in j.sheet_mm)} | {', '.join(j.drawn)} |")
    if ctx.printed:
        fil: Dict[str, float] = {}
        for pl in {p["name"]: p for p in (plate_for(ctx, lab) for lab in ctx.print_parts) if p}.values():
            fil[pl["material"]] = fil.get(pl["material"], 0.0) + float(pl.get("filament_g") or 0.0)
        for m, g in fil.items():
            L.append(f"| filament | {m}, 1.75 mm | {g:.0f} g | the plates in cam/print/ (the slicer's figure, brims and support included) |")
    return L


# ---- ASSEMBLY --------------------------------------------------------------------------------------------

def _is_fastener(label: str) -> bool:
    asm = _ours("assemble")
    return asm._is_fastener(label)


def _steps_text(ctx: Ctx, source: str, first_shell_1: bool) -> List[str]:
    """The design's own step texts a stage cites (shell.STEPS, electrical.STEPS)."""
    out = []
    for owner, mod in (("shell", ctx.mods["shell"]), ("electrical", ctx.mods["electrical"])):
        steps = getattr(mod, "STEPS", None)
        if not steps:
            continue
        for m in re.finditer(rf"{owner} steps? ([\d, \-]+)", source):
            nums = []
            for part in m.group(1).replace(" ", "").split(","):
                if "-" in part and part:
                    a, b = part.split("-")[:2]
                    nums += list(range(int(a), int(b) + 1))
                elif part.isdigit():
                    nums.append(int(part))
            for k in nums:
                if k in steps:
                    out.append(f"{owner} step {k}: {steps[k]}")
            if owner == "shell" and 1 in nums and first_shell_1 and 0 in getattr(ctx.mods["electrical"], "STEPS", {}):
                out.append(f"electrical step 0: {ctx.mods['electrical'].STEPS[0]}")
    return out


def _spec(ctx: Ctx, label: str) -> str:
    p = ctx.table.get(label) or {}
    if p.get("spec"):
        return p["spec"]
    if p.get("owner") == "shell" and label.startswith("insert"):
        return ctx.mods["shell"].INSERT["spec"] if (p.get("mass_g") or 0) >= 0.5 else "M2 brass heat-set insert"
    return _stem(label).replace("_", " ")


def assembly_md(ctx: Ctx) -> str:
    asm = _ours("assemble")
    log = ctx.assembly
    stages = asm.plan(ctx.d)
    L = ["# SparkRover Mini: assembly", "",
         "Generated by `release.py` from `assemble.py`'s build order and its run on Newton "
         "(`release/assembly/assembly_log.json`); do not hand-edit. The order is the design's: "
         f"\"{sk.ASSEMBLY_NOTE}\" (skeleton.ASSEMBLY_NOTE). Parts: [BOM.md](BOM.md); drawings: [drawings/](drawings/); "
         "the wiring: [electrical/](electrical/).", "",
         "Each step's picture is the twin's, at the end of the step. Under each unit, what the engine saw when it was let "
         "go at its seat: **rests** (it stays within the 0.2 mm screw clearance), **shifts** (within 2 mm) or **held by "
         "its screws** (it moves further without them: the screws are not simulated, they are placed at their seats). "
         "Bench steps are done off the robot: the carrier is populated on the bench and goes on as one unit.", ""]
    if log is None:
        L.append("assemble.py has not been run (no assembly_log.json).")
    logged = {s["step"]: s for s in (log or {}).get("steps", [])}
    counted: set = set()
    first_shell_1 = True
    for k, st in enumerate(stages, 1):
        s = logged.get(k)
        bench = " (bench)" if st.bench else ""
        L += [f"## {k}. {st.title}{bench}", ""]
        if s and s["title"] != st.title:
            L += [f"⚠️ The design's step is now \"{st.title}\"; the assembly run had \"{s['title']}\". Rerun assemble.py.", ""]
        texts = _steps_text(ctx, st.source, first_shell_1)
        if any(t.startswith("shell step 1:") for t in texts):
            first_shell_1 = False
        L.append(f"*{st.source}*")
        L += [""] + [f"> {t}" for t in texts] + ([""] if texts else [])
        if st.bench:
            L.append(f"On the bench, onto **{st.bench[0]}**" + (f" with its {len(st.bench) - 1} heat-set inserts" if len(st.bench) > 1 else "")
                     + (", in this order (found by taking the populated carrier apart): " + ", ".join(s.get("bench_order", []))
                        if s and s.get("bench_order") else "") + ".")
            L.append("")
        for u in st.units:
            ul = next((x for x in (s or {}).get("units", []) if x["unit"] == u.name), None)
            path = [(q["from"], q["mm"]) for q in ul["path"]] if ul and ul.get("path") else u.path
            frm = ", then ".join(f"from {_dirn(v)}, {_g(mm)} mm" for v, mm in path)
            if ul and (ul.get("exact_path") or {}).get("found_by") == "disassembly":
                frm += " (the way out found by taking the populated carrier apart)"
            nw = (ul or {}).get("newton") or {}
            res = {"rests": "rests", "shifts": f"shifts {nw.get('seat_error_mm')} mm",
                   "falls": "held by its screws"}.get(nw.get("released"), "not run")
            seat = (ul or {}).get("at_seat") or {}
            ok = "" if not seat.get("interference") else f"; ⚠️ interference: {seat['interference']}"
            blocked = "; ⚠️ path blocked" if (ul or {}).get("exact_path", {}).get("blocked") else ""
            what = ", ".join(u.parts) if len(u.parts) <= 6 else f"{len(u.parts)} parts"
            note = f" {u.note[0].upper()}{u.note[1:]}." if u.note else ""
            L.append(f"- **{u.name}** ({what}): in {frm}. On Newton: {res}{ok}{blocked}.{note}")
        labels = list(st.fasteners) + [x for u in st.units for x in u.parts] + list(st.bench or [])
        screws: "OrderedDict[str, int]" = OrderedDict()
        others = []
        for lab in labels:
            if lab in counted:
                continue
            counted.add(lab)
            if _is_fastener(lab):
                sp = _spec(ctx, lab)
                screws[sp] = screws.get(sp, 0) + 1
            elif lab in st.fasteners:
                others.append(lab)
        if screws:
            L += ["", "Fasteners in this step:", ""] + [f"- {n} set{'s' if n > 1 else ''} of {sp}" if re.match(r"\d+ x ", sp)
                                                         else f"- {n} x {sp}" for sp, n in screws.items()]
        if others:
            L += ["", "Also in this step: " + ", ".join(others) + "."]
        if s and s.get("fastener_check"):
            fc = s["fastener_check"]
            L.append(f"\nFasteners at their seats (FCL): {fc.get('threaded', 0)} threaded, {fc.get('touching', 0)} touching, "
                     f"{len(fc.get('interference') or [])} interfering.")
        pic = RELEASE / "assembly" / (s or {}).get("screenshot", f"physical_step_{k}.png")
        if pic.exists():
            L += ["", f"![Step {k}](assembly/{pic.name})"]
        L.append("")
    if log and log.get("standing_check"):
        sc = log["standing_check"]
        L += ["## Finished: on the bench", "",
              f"Stood on its four braked wheels as one body: contacts by wheel {sc.get('contacts_by_wheel')}, other contacts "
              f"{sc.get('other_contacts')}, tilt {sc.get('tilt_deg')} deg, drift {sc.get('drift_mm')} mm; centre of mass "
              f"{sc.get('com_height_mm')} mm up (designed {sc.get('designed_com_height_mm')}). "
              f"{'Pass' if sc.get('pass') else 'FAIL: ' + '; '.join(sc.get('reasons') or [])}.", ""]
        if (RELEASE / "assembly" / sc.get("screenshot", "")).is_file():
            L += [f"![Standing](assembly/{sc['screenshot']})", ""]
    return "\n".join(L)


def _dirn(v) -> str:
    """assemble.Unit.path's direction: the side the unit comes FROM (robot frame)."""
    names = {(1, 0, 0): "the front (+x)", (-1, 0, 0): "behind (-x)", (0, 1, 0): "the left (+y)",
             (0, -1, 0): "the right (-y)", (0, 0, 1): "above (+z)", (0, 0, -1): "below (-z)"}
    key = tuple(int(round(c)) for c in v)
    if key in names and np.allclose(v, key, atol=1e-6):
        return names[key]
    return "(" + ", ".join(_g(c, 3) for c in v) + ")"


# ---- REPORT ---------------------------------------------------------------------------------------------

def report_md(ctx: Ctx) -> str:
    L = ["# SparkRover Mini: as made on the twins", "",
         "Generated by `release.py`; do not hand-edit. Every number here is a **twin's**: the parts were made on OmniSim's "
         "machine twins (the LongMill MK2 with the Sienci LaserBeam 7W, its router, the Sovol SV08) and measured from the "
         "engine's own record of the material (the laser and CNC sheets' height maps, the printer's beads). No part has been "
         "made on a real machine, and no twin has been validated against its real machine.", ""]
    L += report_flat(ctx) + report_print(ctx) + report_integration(ctx) + report_wiring(ctx) + report_drive(ctx) \
        + report_assembly(ctx) + report_open(ctx)
    return "\n".join(L) + "\n"


def _hole_classes(p: dict) -> str:
    groups: Dict[float, List[float]] = {}
    for h in p.get("holes") or []:
        if "cut_d_mm" in h:
            groups.setdefault(round(h["drawn_d_mm"], 2), []).append(h["cut_d_mm"])
    return "; ".join(f"⌀{d:.2f}: {min(v):.2f}" + (f"-{max(v):.2f}" if max(v) - min(v) >= 0.005 else "") + f" (x{len(v)})"
                     for d, v in sorted(groups.items()))


def report_flat(ctx: Ctx) -> List[str]:
    made = ctx.made
    L = ["## Laser and CNC parts", "",
         f"From `make.py` ({made.get('generated', 'not run')}; its own table is [make/MAKE.md](make/MAKE.md)). Machine time for "
         f"the set {_g((made.get('machine_s_total') or 0) / 60, 1)} min. Outline: the size the part's height-map cells cover; "
         "edge offset: (cut area - drawn) / drawn perimeter, + is oversize. Holes: drawn ⌀ → cut ⌀ (each void's equivalent "
         "diameter; for a laser cut, at the BOTTOM face: the kerf is a V). Thickness: the median over the part's cells, and "
         "the 1st percentile (made.json's `thickness_min_mm`).", "",
         "| Part | Job | Process | Cut free | Outline (drawn), mm | Edge offset | Holes cut | Drawn ⌀ → cut ⌀ (count) | "
         "Thickness median / p1 (drawn) | Grid |", "|---|---|---|---|---|---|---|---|---|---|"]
    for label in ctx.made_labels(("laser", "cnc")):
        rec, p, job = ctx.made_rec(label)
        if not p:
            L.append(f"| {label} | {MADE_AS[label][0]} | {ctx.table[label]['process']} | **not made** | | | | | | |")
            continue
        same = f" (same program as {job})" if job != MADE_AS[label][0] else ""
        extra = ""
        if p.get("blind"):
            extra = f"; tap pilots {p['blind_cut']}, ⌀{p['blind_d_mm'][0]:.2f}-{p['blind_d_mm'][1]:.2f}, {p['blind_depth_mm'][0]:.2f} deep"
        free = "yes" if p["cut_free"] else f"**no**, a {p.get('free_at_skin_mm')} mm web"
        L.append(f"| {label} | {MADE_AS[label][0]}{same} | {rec['process']} | {free} | {p['size_mm'][0]:.2f} x {p['size_mm'][1]:.2f} "
                 f"({_g(p['drawn_mm'][0])} x {_g(p['drawn_mm'][1])}) | {p['outline_offset_mm']:+.3f} | {p['holes_cut']}{extra} | "
                 f"{_hole_classes(p) or '-'} | {p['thickness_mm']:.2f} / {p['thickness_min_mm']:.2f} ({_g(p['drawn_thickness_mm'])}) | "
                 f"{rec.get('resolution_mm', 0):.3f} |")
    if made.get("notes"):
        L += ["", "make.py's notes:", ""] + [f"- {n}" for n in made["notes"]]
    stale = [n for n, j in ctx.jobs.items() if not j.same_as and not _program_is_current(j)]
    laser = ctx.mods["chassis"].LASER
    L += ["", "The programs in [cam/](cam/) against the modules' `jobs()` now: "
          + ("every one is the same text." if not stale else f"**{', '.join(stale)} differ**: rerun make.py for them.")
          + " The laser settings now (chassis.LASER): " + "; ".join(f"{_g(t)} mm, {v['passes']} passes of {_g(v['z_step'])} mm"
                                                                   for t, v in laser.items()) + "."]
    L += ["", "### The thin readings", ""]
    brk = [lab for lab in ctx.made_labels(("cnc",)) if (ctx.made_rec(lab)[1] or {}).get("blind")]
    for lab in brk:
        p = ctx.made_rec(lab)[1]
        L.append(f"- **{lab}**: {p['thickness_min_mm']} mm = {_g(p['drawn_thickness_mm'])} - {p['blind_depth_mm'][1]} mm, the "
                 "floor of the blind tap pilots (by design: they are drilled through and tapped by hand).")
    if ctx.thickness:
        L += ["", "Laser parts whose 1st percentile reads under the sheet, from make.py's saved height maps "
              f"(.tmp/spark_rover_make/runs): the cells within {EDGE_RING_MM} mm of a cut edge (the kerf's V-shaped flank) "
              f"against the rest, and the rest's cells within {SEAM_BAND_MM} mm of a window seam.", "",
              "| Part | Windows | p1, all cells | Edge cells: share, p1, median | Interior: p1, min | Interior cells under the sheet "
              "(on a seam) | Deepest seam dip |", "|---|---|---|---|---|---|---|"]
        for k, t in ctx.thickness.items():
            L.append(f"| {k} | {t['windows']} | {t['p1_all_cells_mm']} | {t['edge_ring_share_pct']} %, {t['edge_ring_p1_mm']}, "
                     f"{t['edge_ring_median_mm']} | {t['interior_p1_mm']}, {t['interior_min_mm']} | {t['interior_cells_below_drawn']} "
                     f"of {t['of_interior_cells']} ({t['of_those_on_a_seam']}) | {t['seam_dip_mm']} mm |")
        L += ["", "Read: " + thickness_finding(ctx)]
    else:
        L += ["", "(make.py's height maps are not on this machine: the edge / interior split was not measured.)"]
    return L


def _program_is_current(job) -> bool:
    p = MAKE_RELEASE / "cam" / f"{job.name}.{job.ext}"
    norm = lambda t: t.replace("\r\n", "\n").strip()    # noqa: E731
    return p.exists() and norm(p.read_text(encoding="utf-8")) == norm(job.program)


def thickness_finding(ctx: Ctx) -> str:
    """One sentence from the numbers: is a part's low p1 its edges or its interior / seams?"""
    t = ctx.thickness
    if not t:
        return "not measured (no height maps)."
    edge = [k for k, v in t.items() if v["interior_p1_mm"] >= v["drawn_mm"] - 0.01]
    seam = [k for k, v in t.items() if v["of_those_on_a_seam"]]
    dips = [v["seam_dip_mm"] for v in t.values() if v["of_those_on_a_seam"]]
    s = (f"in {len(edge)} of {len(t)} parts the interior's 1st percentile is the full sheet; the low reading is the kerf's flank, "
         f"the cells within {EDGE_RING_MM} mm of an outline or hole, which make up "
         f"{min(v['edge_ring_share_pct'] for v in t.values())}-{max(v['edge_ring_share_pct'] for v in t.values())} % of a part's "
         "cells and so reach its 1st percentile on the narrow parts. It is the laser's V-shaped edge, not missing material.")
    if seam:
        s += (f" The window seams are a second, smaller artefact: on {len(seam)} parts some interior cells along a seam read up to "
              f"{max(dips)} mm under the sheet (make.py's stitching or the cropped programs' ends there; a real part has no seams).")
    return s


def _mode(st: dict) -> str:
    """How a printed part's result was made: 'twin' (full physics) or 'ideal' (no engine)."""
    return (st.get("plate_rec") or {}).get("mode") or (st.get("measured") or {}).get("mode") or "twin"


def report_print(ctx: Ctx) -> List[str]:
    L = ["", "## Printed parts", ""]
    if ctx.printed is None:
        L.append("make_print.py has not written release/make/print/printed.json: every printed part is **pending**.")
    else:
        L.append("From `make_print.py` (its own tables are [make/print/PRINT.md](make/print/PRINT.md)): each plate on the Sovol SV08 "
                 "twin, CuraEngine with the SV08 profile and the filament's OrcaSlicer values, Klipper's planner, the engine's "
                 "PrintedPart. Outer surface: printed outline minus the drawing, 5th / 95th percentile over the layers (+ is "
                 "outside; the first layer is left out). Footprint: printed minus drawn size, mean over the layers, X / Y. "
                 "Part g: the plastic laid in the part (the design's estimate in brackets). **Mode**: *twin* is the full twin "
                 "(the SV08's axes on Newton, the engine laying beads where the nozzle actually is); *ideal* is the same plan "
                 "and hotend with the nozzle exactly where it was commanded, no engine (printing/README.md, \"Ideal print (no "
                 "engine)\": within 0.03 mm of the twin's mean outer surface, a little smaller a side because it leaves out "
                 "the drives). A part printed both ways shows its twin result. **Pending**: not printed yet; nothing is "
                 "predicted for it.")
    n_done = sum(1 for s in ctx.status.values() if s["state"] == "printed")
    n_twin = sum(1 for s in ctx.status.values() if s["state"] == "printed" and _mode(s) == "twin")
    L += ["", f"{n_done} of {len(ctx.status)} printed parts printed: {n_twin} on the full twin, {n_done - n_twin} in ideal mode "
              f"(no engine), their full-twin runs on the fixed bead rule still to come; {len(ctx.status) - n_done} pending. "
              "Every result is on the fixed bead rule.", "",
          "| Part | Material | State | Mode | Plate | Height printed / drawn | Outer p05 / p95 mm | Bead rule "
          "| Footprint X / Y mm | Part g (design) | Hanging strands (beads, mm) |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    laid = design = 0.0
    for label, st in ctx.status.items():
        row = ctx.table[label]
        if st["state"] != "printed":
            L.append(f"| {label} | {row['material']} | **pending** | | {', '.join(st.get('plates') or []) or '-'} | {st['why']} | | | | "
                     f"({_g(row['mass_g'])}) | |")
            continue
        q = st["measured"]
        e, s, hs = q["outer_surface_error_mm"], q["size_error_mm"], q["hanging_strands"]
        rule = (st.get("row") or {}).get("bead_rule") or (st.get("plate_rec") or {}).get("bead_rule") or "-"
        laid += q["part_g"]
        design += row["mass_g"]
        L.append(f"| {label} | {row['material']} | printed | {_mode(st)} | {st['plate']} | {q['height_mm']['printed']} / {q['height_mm']['drawing']} | "
                 f"{e['p05']:+.3f} / {e['p95']:+.3f} | {rule} | {s['x_mean']:+.3f} / {s['y_mean']:+.3f} | "
                 f"{q['part_g']} ({_g(row['mass_g'])}) | {hs['beads']}, {hs['length_mm']} |")
    plates = {st["plate"]: st["plate_rec"] for st in ctx.status.values() if st["state"] == "printed"}
    for name, pl in plates.items():
        r, m = pl.get("run") or {}, pl.get("measured") or {}
        how = (f"{r.get('speed_x_real_time')}x real time on the twin" if pl.get("mode", "twin") == "twin"
               else "ideal mode, no engine")
        L.append(f"\n{name} ({pl.get('mode', 'twin')}): printer time {_hms(pl.get('time_s'))} planned, {_hms(r.get('wall_s'))} "
                 f"of wall time ({how}); {m.get('laid_g')} g laid, unrecorded fraction {m.get('unrecorded_fraction')}, "
                 f"{m.get('nozzle_crashes')} nozzle crashes (the nozzle touching beads already laid; none stops a print).")
    if design:
        L.append(f"\nThe printed parts so far weigh {laid:.1f} g as printed against {design:.1f} g designed ({laid - design:+.1f} g): "
                 "the design's mass of a printed part is not what the slicer's infill lays.")
    sup = (ctx.printed or {}).get("superseded") or {}
    if sup:
        L += ["", "Superseded drawings (make_print.py recorded them; not part of this robot): "
              + "; ".join(f"{k}: {v}" for k, v in sup.items()) + "."]
    return L


def _sgn(v) -> str:
    return "-" if v is None else f"{v:+.3f}"


def report_integration(ctx: Ctx) -> List[str]:
    rep = ctx.integration
    m = rep["mass"]
    lo, hi = m["target_g"] if "target_g" in m else sk.MASS_TARGET_G
    L = ["", "## Integration", "",
         "From `integrate.py`: the four subsystems' solids against each other (FCL), each subsystem inside its zones of "
         "skeleton.py, every interface provided by one side and used by the other, and the mass."]
    inter = rep.get("interference")
    if inter:
        L += ["", f"- Solids: {inter['solids']}. Pairs that meet: {len(inter['touching'])} touching (a 0.05 mm nudge frees "
              f"them: a face on its seat), {len(inter['threaded'])} threaded (a fastener in its thread), "
              f"**{len(inter['interference'])} interfering**."]
        for r in inter["interference"]:
            L.append(f"  - {r['a']} x {r['b']}: {r['depth_mm']} mm")
    else:
        L += ["", "- Interference: not run in this release (`--fresh-integration` runs it)."]
    zl = rep.get("zones_left")
    if zl is not None:
        L.append(f"- Solids outside their zones: {len(zl)}" + (": " + "; ".join(f"{z['owner']}/{z['part']}" for z in zl) if zl else "."))
    mounts = rep.get("mounts") or []
    miss = [x for x in mounts if x.get("missing")]
    L.append(f"- Interfaces: {len(mounts)}, {len(miss)} missing a side" + (": " + ", ".join(x["mount"] for x in miss) if miss else "."))
    L.append(f"- Mass: **{m['total_g']} g** ({', '.join(f'{k} {v}' for k, v in m['by_subsystem_g'].items())}), centre of mass "
             f"{m['com_mm']} mm; the skeleton holds it to {_g(lo)}-{_g(hi)} g: margin **{hi - m['total_g']:.1f} g**. The BOM adds "
             f"to {bom_total_g(ctx):.1f} g.")
    sk_ok = rep.get("skeleton") or {}
    L.append(f"- The skeleton's own checks: {sum(1 for k, v in sk_ok.items() if v is True)} of "
             f"{sum(1 for v in sk_ok.values() if isinstance(v, bool))} pass.")
    checks = rep.get("checks") or {}
    L += ["", "| Subsystem | Design rules | Pass | Not passing |", "|---|---|---|---|"]
    for owner, c in checks.items():
        bad = [(k, v) for k, v in c.items() if not (v[0] if isinstance(v, (list, tuple)) else v)]
        L.append(f"| {owner} | {len(c)} | {len(c) - len(bad)} | " + ("; ".join(f"{k}: {v[1] if isinstance(v, (list, tuple)) else v}"
                                                                         for k, v in bad) or "-") + " |")
    L += ["", "<details><summary>Every design rule</summary>", ""]
    for owner, c in checks.items():
        L += [f"**{owner}**", ""] + [f"- {'pass' if (v[0] if isinstance(v, (list, tuple)) else v) else 'NOT PASSING'} "
                                     f"`{k}`: {v[1] if isinstance(v, (list, tuple)) else ''}" for k, v in c.items()] + [""]
    L.append("</details>")
    return L


def report_wiring(ctx: Ctx) -> List[str]:
    L = ["", "## Wiring", ""]
    if not ctx.wires:
        return L + ["harness.py has not routed the wires (no release/electrical/wires_routed.json)."]
    h = _ours("harness")
    s = h.summary(ctx.wires)
    el = ctx.mods["electrical"]
    now = {w["id"]: (tuple(w["from"]), tuple(w["to"]), w["net"]) for w in el.wire_list(None)}
    was = {w["id"]: (tuple(w["from"]), tuple(w["to"]), w["net"]) for w in ctx.wires}
    drift = sorted(set(now) ^ set(was)) + sorted(k for k in set(now) & set(was) if now[k] != was[k])
    L += [f"From `harness.py` (release/electrical/wires_routed.json): {s['wires']} wires in electrical.wire_list(), "
          f"{s['routed']} routed through the assembled robot, {len(s['failed'])} failed"
          + (f" ({', '.join(s['failed'])})" if s["failed"] else "") + f", {len(s['skipped'])} not routed"
          + (f" ({', '.join(s['skipped'])}: {h.NOT_ROUTED.get(_net(ctx, s['skipped'][0]), '')})" if s["skipped"] else "") + ".",
          f"- Cut length {s['total_cut_m']} m (electrical's estimate before routing: {s['estimated_m']} m).",
          f"- Smallest gap of a round wire's insulation to a part {_g(s['min_clear_of_parts_mm'])} mm, to another wire "
          f"{_g(s['min_clear_of_wires_mm'])} mm (measured on the {h.GRID_MM:g} mm grid). The five closest to a part: "
          + ", ".join(f"{w} {_g(c)} mm" for c, w in s["worst_parts"]) + ".",
          f"- Long breakouts (a wire leaving its pin further than {h.BREAKOUT_FLAG_MM} mm before it is free): "
          + (", ".join(f"{w} {b}" for w, b in s["long_breakouts"]) or "none") + ".",
          "- The DSI and CSI ribbons are routed along their centre line for their length only (a 2 mm round envelope); "
          "their clearance is not claimed.",
          f"- The routed list against electrical.wire_list() now: " + ("the same wires, ends and nets." if not drift else
                                                                     f"**{len(drift)} differ** ({', '.join(drift[:10])}): rerun harness.py.")]
    return L


def armature_default() -> Optional[str]:
    """The engine's default OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO, read from the runtime's source."""
    if not NEWTON_RUNTIME.exists():
        return None
    m = re.search(r'_env_float\("OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO",\s*([\d.]+)\)', NEWTON_RUNTIME.read_text(encoding="utf-8"))
    return _g(float(m.group(1))) if m else None


def _drive_conditions(stem: str) -> Tuple[str, str]:
    mu = re.search(r"mu([\d.]+)", stem)
    ar = re.search(r"armature_ratio([\d.]+)", stem)
    return (mu.group(1) if mu else "?"), (ar.group(1) if ar else f"{armature_default() or '?'} (the engine's default)")


def report_drive(ctx: Ctx) -> List[str]:
    rb = ctx.rb
    pred = rb.predictions(ctx.d)
    mr = rb.mass_report(ctx.d)
    L = ["", "## The robot on Newton: the drive test", "",
         f"`robot.py` builds the robot from the design (mass {mr['total_g']} g, centre of mass {mr['com_mm']} mm) and writes "
         "`projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld`; its controller `spark_rover_drive` "
         "drives 1 m straight, stops for 1 s, spins in place for 4 s and waves both arms through every limit. The wheel motor "
         f"is the Pololu #4844's line at the pack's nominal {pred['pack_v_nom']} V (torque = stall x |1 - speed / no-load|, "
         "braking above no-load speed); the tyres are rigid cylinders (no rolling resistance). "
         "Each run: 2 ms step, MuJoCo CPU solver, 2 substeps; the floor's friction and the engine's wheel-armature ratio "
         "(OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO) as each result file in release/drive/ is named (a name with no ratio "
         "ran at the engine's default).", ""]
    if not ctx.drives:
        return L + ["No drive results (release/drive/ is empty)."]
    heads = list(ctx.drives)
    L += ["| | Predicted | " + " | ".join(f"mu {_drive_conditions(s)[0]}, armature {_drive_conditions(s)[1]}" for s in heads) + " |",
          "|---|---|" + "---|" * len(heads)]

    def row(name, p, f):
        L.append(f"| {name} | {p} | " + " | ".join(_g(f(d), 4) if not isinstance(f(d), str) else f(d) for d in ctx.drives.values()) + " |")
    row("Pack: where the motors' voltage came from", f"{pred['pack_v_nom']} V nominal",
        lambda d: "battery model, " + f"{_g(100 * d['battery']['start']['soc'])} % charge at the start"
        if (d.get("battery") or {}).get("start") else f"fixed {(d.get('battery') or {}).get('pack_v', pred['pack_v_nom'])} V")
    row("Bus voltage over the straight run, mean (V)", "", lambda d: d["straight"].get("pack_v_mean", pred["pack_v_nom"]))
    row("No-load speed at that voltage (m/s)", f"{pred['no_load_speed_m_s']} at {pred['pack_v_nom']} V",
        lambda d: rb.predictions(ctx.d, d["straight"].get("pack_v_mean"))["no_load_speed_m_s"])
    row("Straight speed (m/s)", f"{pred['no_load_speed_m_s']} (no load)", lambda d: d["straight"]["steady_speed_m_s"])
    row("Effective wheel radius (mm)", _g(pred["wheel_radius_mm"]), lambda d: d["straight"]["effective_radius_mm"])
    row("Heading drift, straight (deg)", "0", lambda d: d["straight"]["heading_drift_deg"])
    row("Pitch over the straight run (deg)", "", lambda d: f"{d['straight']['pitch_deg_range'][0]} to {d['straight']['pitch_deg_range'][1]}")
    row("Stopping distance (mm, in the 1 s stop)", "", lambda d: (d.get("phases") or {}).get("stop", {}).get("moved_mm"))
    no_slip = ", ".join(f"{_g(v['yaw_rate_rad_s_if_no_slip'], 3)} at mu {k.split('_')[-1]}" for k, v in pred.items()
                        if k.startswith("spin_mu_"))
    row("Spin yaw rate (rad/s)", f"{no_slip}, if the tyres did not slip", lambda d: d["spin"]["yaw_rate_rad_s"])
    row("Spin: mean wheel speed (rad/s)", f"{pred['spin_mu_1']['wheel_rad_s']} (mu 1)", lambda d: d["spin"]["wheel_rad_s_mean"])
    row("Spin: effective track (mm)", _g(pred["track_mm"]), lambda d: d["spin"]["effective_track_mm"])
    row("Spin: mean motor current (A)", f"{pred['spin_mu_1']['current_a']} (mu 1)", lambda d: d["spin"]["current_a_mean"])
    row("Arms: worst tracking error on the profile (deg)", "", lambda d: max(p["max_tracking_error_deg"] for p in d["arms"]["poses"]))
    row("Arms: worst settled error (deg)", "", lambda d: max(abs(p[j]["error_deg"]) for p in d["arms"]["poses"]
                                                           for j in p if isinstance(p[j], dict) and "error_deg" in p[j]))
    row("Arms: the robot moved, worst pose (mm)", "", lambda d: max(p["robot_moved_mm"] for p in d["arms"]["poses"]))
    row("Run complete", "", lambda d: str(d.get("complete")))
    L += ["", "The straight run does not depend on the armature; the spin and the stop do: the spin's yaw rate rises and the "
          "stop lengthens with the engine's wheel-armature ratio (a rotor inertia the engine adds to every velocity-servoed "
          "wheel), which no datasheet fixes. The skid-steer spin's effective track is wider than the wheels' because the "
          "tyres scrub sideways."]
    return L


def _stem_of(ctx: Ctx, d: dict) -> str:
    return next(k for k, v in ctx.drives.items() if v is d)


def report_assembly(ctx: Ctx) -> List[str]:
    log = ctx.assembly
    L = ["", "## Assembled on Newton", ""]
    if not log:
        return L + ["assemble.py has not been run."]
    s = log.get("summary") or {}
    sc = log.get("standing_check") or {}
    L += [f"From `assemble.py` (release/assembly/assembly_log.json; the steps are [ASSEMBLY.md](ASSEMBLY.md)): {log.get('what')}. "
          f"Solver {log.get('solver')}; tolerances {log.get('tolerances')}; {log.get('wall_s')} s of wall time on "
          f"{(log.get('machine') or {}).get('cpu')}, {(log.get('machine') or {}).get('gpu')}.",
          f"- {s.get('units')} units over {len(log.get('steps', []))} steps. Insertion paths blocked: {len(s.get('path_blocked') or [])}"
          + (f" ({'; '.join(s['path_blocked'])})" if s.get("path_blocked") else "") + ". Interference at a seat: "
          f"{len(s.get('interference_at_seat') or [])}" + (f" ({'; '.join(s['interference_at_seat'])})" if s.get("interference_at_seat") else "") + ".",
          f"- Let go at the seat: {s.get('rests')} rest, {len(s.get('shifts') or [])} shift ({', '.join(s.get('shifts') or []) or '-'}), "
          f"{len(s.get('falls') or [])} move further than 2 mm: these are the parts their screws hold (under the deck, on the "
          "spine's faces, the boards on their carriers); the screws are placed, not simulated.",
          f"- Standing on the bench: {'pass' if sc.get('pass') else 'FAIL'}; tilt {sc.get('tilt_deg')} deg, drift {sc.get('drift_mm')} mm, "
          f"centre of mass {sc.get('com_height_mm')} mm up (designed {sc.get('designed_com_height_mm')}).",
          f"- Run error: {log.get('error') or 'none'}."]
    return L


def report_open(ctx: Ctx) -> List[str]:
    """The open items: the design's own assumptions and the results' limits, each from its source."""
    arms, el, chassis = ctx.mods["arms"], ctx.mods["electrical"], ctx.mods["chassis"]
    m = ctx.integration["mass"]
    hi = sk.MASS_TARGET_G[1]
    pend = [k for k, s in ctx.status.items() if s["state"] != "printed"]
    holes = {ref: v["source"] for ref, v in el.BOARD_HOLES.items() if str(v.get("source", "")).startswith("ASSUMED")}
    at_mu1 = [(_drive_conditions(k)[1].split(" ")[0], d) for k, d in ctx.drives.items() if _drive_conditions(k)[0] == "1"]
    yaw = ", ".join(f"{_g(d['spin']['yaw_rate_rad_s'], 3)} (ratio {a})" for a, d in at_mu1)
    stop = ", ".join(f"{_g((d.get('phases') or {}).get('stop', {}).get('moved_mm'), 1)} (ratio {a})" for a, d in at_mu1)
    done = [s for s in ctx.status.values() if s["state"] == "printed"]
    lighter = sum(s["measured"]["part_g"] for s in done) - sum(ctx.table[s["row"]["label"]]["mass_g"] for s in done)
    hs = _ours("harness").summary(ctx.wires) if ctx.wires else None
    L = ["", "## Open items", ""]
    ideal_only = [k for k, v in ctx.status.items() if v["state"] == "printed" and _mode(v) == "ideal"]
    if pend:
        L.append(f"- **Printing is not finished**: {len(pend)} of {len(ctx.status)} printed parts are pending ({', '.join(pend)}). "
                 "Rerun `release.py` when make_print.py is done.")
    L += [f"- **The XC330 horn's dimensions are unverified**: horn ⌀{_g(arms.SERVO['horn_d_mm'])} x {_g(arms.SERVO['horn_t_mm'])}, "
          f"screw circle {_g(arms.SERVO['horn_screw_pcd_mm'])} mm, axis {_g(arms.SERVO['horn_axis_from_top_mm'])} mm from the "
          "case's top: scaled or estimated from ROBOTIS' e-manual figures (arms.SERVO), not measured on a servo or its STEP. "
          "The shoulder post's end plate and the elbow plate are drilled to them; check one horn before printing the arms.",
          "- **Board hole positions are assumed** on " + ", ".join(f"{k} ({v})" for k, v in holes.items())
          + " (electrical.BOARD_HOLES). Check each board before its carrier is printed.",
          f"- **The spin and the stop depend on the engine's wheel-armature default** (OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO, "
          f"{armature_default()}): at mu 1 the spin's yaw rate reads {yaw} rad/s and the stop {stop} mm (see the drive table). "
          "The straight run does not move. A real robot's figures are set by its motors' rotor inertia and its tyres, not by "
          "this engine constant.",
          "- **The acrylic face cover has no cutting-load model**: cutting.py's acrylic has no published cutting energy, so "
          "the twin removes the swept volume and models no load, melt or cracking (make.py, made.json's face_cover note).",
          "- **The bead model was corrected in v10** (OmPrintedPart.cpp and printing/ideal_print.py): a creeping nozzle's held "
          "plastic was laid as one bead up to ~7 mm wide or as a dome the nozzle then counted as a crash. Every result in the "
          "printed-parts table is on the fixed rule (its Bead rule column). The full-twin prints measured before the fix stay in "
          "[make/print/PRINT.md](make/print/PRINT.md), labelled so, with the before -> after per plate and part.",
          f"- **{len(ideal_only)} printed parts' current results are ideal-mode** (no engine, the fixed bead rule): "
          f"{', '.join(ideal_only) or 'none'}. Their full-twin runs on the fixed engine were still running at this release; "
          "a part's twin result becomes current only once its fixed run is measured.",
          "- **The front carrier needs about 64 g of support for a 75 g part** (an 81 mm unsupported span under the display "
          "ring's bars; the camera beam floats over the back plate), and the bay carrier's top bridge spans 34.5 mm: 45 degree "
          "ribs, or the head frame as its own part, would remove most of it (make_print.py's printability check).",
          f"- **The engine's MillingStock holds at most 1984 cells a side** (OmMillingStock.cpp kMaxCellsPerAxis): laser sheets "
          f"longer than {_g(ctx.make.WINDOW_MAX_MM)} mm were cut as {_g(ctx.make.RESOLUTION_MM['laser'])} mm windows and "
          "stitched (make.py windows / crop_program / stitch). The cap is worked around, not lifted.",
          f"- **Mass margin**: {m['total_g']} g against the skeleton's {_g(hi)} g ceiling, **{hi - m['total_g']:.1f} g** to spare "
          "on the design's masses" + (f" (the {len(done)} parts printed so far weigh {abs(lighter):.1f} g "
                                      f"{'less' if lighter < 0 else 'more'} as printed than their design masses: the "
                                      "slicer's walls and infill)." if done else ".")]
    if hs and (hs["min_clear_of_parts_mm"] or 0) <= 0:
        worst = [w for c, w in hs["worst_parts"] if c <= 0]
        L.append(f"- **A routed wire touches a part**: {', '.join(worst)} reads {_g(hs['min_clear_of_parts_mm'])} mm of clearance "
                 f"on harness.py's {_ours('harness').GRID_MM:g} mm grid. Check its route on the bench.")
    t = ctx.thickness
    thin = [lab for lab in ctx.made_labels(("laser",)) if (ctx.made_rec(lab)[1] or {}).get("thickness_min_mm", 99)
            < (ctx.made_rec(lab)[1] or {}).get("drawn_thickness_mm", 0) - 0.5]
    reads = ", ".join(f"{x} {ctx.made_rec(x)[1]['thickness_min_mm']} mm" for x in thin)
    L.append(f"- **The thin laser readings** ({reads}): "
             + (thickness_finding(ctx) if t else "the height maps were not read on this machine.")
             + " The 1st percentile over all cells is the wrong statistic for a laser part's thickness; make.py could report the "
             "interior's.")
    shell_issues = (ctx.integration.get("checks") or {}).get("shell", {}).get("skeleton_issues")
    if shell_issues and not shell_issues[0]:
        L.append(f"- **shell.skeleton_issues**: {shell_issues[1]} (the shell's own check, not passing: skeleton.py lists the "
                 "ones it has accepted).")
    L += ["- **Simulation only**: no part has been made on a real machine and no twin has been validated against its machine; "
          "every number above is the twin's, under the conditions given with it."]
    return L


# ---- README block -----------------------------------------------------------------------------------------

def readme_block(ctx: Ctx) -> str:
    """The README's measured-results block, from the same numbers as REPORT.md."""
    m = ctx.integration["mass"]
    inter = ctx.integration.get("interference")
    n_made = len(ctx.made_labels(("laser", "cnc")))
    free = sum(1 for lab in ctx.made_labels(("laser", "cnc")) if (ctx.made_rec(lab)[1] or {}).get("cut_free"))
    done = [k for k, s in ctx.status.items() if s["state"] == "printed"]
    d1 = next((d for s, d in ctx.drives.items() if _drive_conditions(s) == ("1", f"{armature_default() or '?'} (the engine's default)")), None)
    v_run = (d1 or {}).get("straight", {}).get("pack_v_mean")
    pred = ctx.rb.predictions(ctx.d, v_run)        # at the bus voltage the battery model gave during the run
    s = (ctx.assembly or {}).get("summary") or {}
    sc = (ctx.assembly or {}).get("standing_check") or {}
    hs = _ours("harness").summary(ctx.wires) if ctx.wires else None
    meas = [ctx.made_rec(lab)[1] or {} for lab in ctx.made_labels(("laser", "cnc"))]
    cut = [tuple(int(v) for v in m["holes_cut"].split("/")) for m in meas if m.get("holes_cut")]
    n_cut, n_holes = sum(c for c, _ in cut), sum(n for _, n in cut)
    holes_txt = "every hole cut" if n_cut == n_holes else f"{n_cut} of {n_holes} holes cut"
    res = [r.get("resolution_mm") for r in ctx.made["jobs"].values() if r.get("resolution_mm")]
    res_txt = f"{min(res):.2f}-{max(res):.2f}" if res else "-"
    L = [README_BEGIN, "",
         "| | Result (in simulation) | Conditions |", "|---|---|---|",
         f"| Laser and CNC parts | {free} of {n_made} cut free, {holes_txt} | LongMill MK2 twin with the Sienci LaserBeam 7W "
         f"and the router, height maps at {res_txt} mm ([MAKE.md](release/make/MAKE.md)) |",
         f"| Printed parts | {len(done)} of {len(ctx.status)} printed: "
         f"{sum(1 for k in done if _mode(ctx.status[k]) == 'twin')} on the full twin, "
         f"{sum(1 for k in done if _mode(ctx.status[k]) != 'twin')} in ideal mode (no engine; their fixed-rule twin runs to come); "
         f"{len(ctx.status) - len(done)} pending | Sovol SV08 twin, CuraEngine, "
         "Klipper's planner ([PRINT.md](release/make/print/PRINT.md)) |",
         f"| Integration | {inter['solids'] if inter else '-'} solids, {len(inter['interference']) if inter else '-'} interfering, "
         f"{len(ctx.integration.get('zones_left') or [])} outside their zones | FCL on the design's solids (`integrate.py`) |",
         f"| Mass | {m['total_g']} g (ceiling {_g(sk.MASS_TARGET_G[1])} g) | the design's masses, battery and harness included |",
         ]
    if hs:
        L.append(f"| Wiring | {hs['routed']} of {hs['wires']} wires routed, {hs['total_cut_m']} m, {_g(hs['min_clear_of_parts_mm'])} mm "
                 f"smallest gap to a part | `harness.py` on a 1 mm grid |")
    if d1:
        L += [f"| Straight speed | {d1['straight']['steady_speed_m_s']} m/s (no-load prediction {pred['no_load_speed_m_s']}) | "
              f"Newton, 2 ms, MuJoCo CPU, floor mu 1, {_g(pred['pack_v_nom'], 3)} V mean on the bus"
              + (" (the battery model, from full charge)" if (d1.get("battery") or {}).get("start") else "") + " |",
              f"| Spin in place | {d1['spin']['yaw_rate_rad_s']} rad/s ({pred['spin_mu_1']['yaw_rate_rad_s_if_no_slip']} if the tyres did "
              f"not slip) | the same, the engine's default wheel armature: it depends on it (REPORT.md) |",
              f"| Stop from full speed | {(d1.get('phases') or {}).get('stop', {}).get('moved_mm')} mm | the same |"]
    if s:
        L.append(f"| Assembly | {s.get('units')} units, {len(s.get('path_blocked') or [])} blocked, "
                 f"{len(s.get('interference_at_seat') or [])} interfering at a seat; stands on all four wheels: "
                 f"{'yes' if sc.get('pass') else 'no'} | `assemble.py` on Newton, the screws placed, not simulated |")
    L += ["", "The full tables are [release/REPORT.md](release/REPORT.md).", "", README_END]
    return "\n".join(L)


def update_readme(ctx: Ctx, path: Path = README) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8")
    a, b = text.find(README_BEGIN), text.find(README_END)
    if a < 0 or b < 0:
        return False
    new = text[:a] + readme_block(ctx) + text[b + len(README_END):]
    if new != text:
        path.write_text(new, encoding="utf-8", newline="\n")
    return True


# ---- main ---------------------------------------------------------------------------------------------------

def write(out: Path = RELEASE, integration: str = "cached", height_maps: bool = True, readme: bool = True,
          cam: bool = True, log=print, ctx: Optional[Ctx] = None) -> dict:
    ctx = ctx or Ctx(integration=integration, height_maps=height_maps, log=log)
    out.mkdir(parents=True, exist_ok=True)
    res = {"drawings": write_drawings(ctx, out), "models": write_models(ctx, out)}
    res["cam"] = write_cam(ctx, out) if cam else {}
    for name, fn in (("BOM.md", bom_md), ("ASSEMBLY.md", assembly_md), ("REPORT.md", report_md)):
        (out / name).write_text(fn(ctx), encoding="utf-8", newline="\n")
    res["readme"] = update_readme(ctx) if readme and out.resolve() == RELEASE.resolve() else False
    res["pending"] = [k for k, s in ctx.status.items() if s["state"] != "printed"]
    res["bom_total_g"] = round(bom_total_g(ctx), 1)
    res["integrate_total_g"] = ctx.integration["mass"]["total_g"]
    return res


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the SparkRover Mini's release folder.")
    p.add_argument("--out", default=str(RELEASE))
    p.add_argument("--fresh-integration", action="store_true", help="rerun integrate.report() (FCL, minutes)")
    p.add_argument("--no-interference", action="store_true", help="skip the FCL run (mass, checks, mounts only)")
    p.add_argument("--no-height-maps", action="store_true", help="do not read make.py's saved height maps")
    p.add_argument("--no-cam", action="store_true", help="do not copy the programs and G-code")
    a = p.parse_args(argv)
    mode = "none" if a.no_interference else "fresh" if a.fresh_integration else "cached"
    res = write(Path(a.out), mode, not a.no_height_maps, cam=not a.no_cam)
    print(json.dumps({k: (v if not isinstance(v, list) or len(v) < 12 else f"{len(v)} files") for k, v in res.items()}, indent=1))
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
