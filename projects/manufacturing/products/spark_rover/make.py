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

"""Make the SparkRover Mini's laser-cut and CNC-milled parts on the twins, and measure each.

    python projects/manufacturing/products/spark_rover/make.py --process laser --process cnc
    python projects/manufacturing/products/spark_rover/make.py --only spine_foot --only face_cover
    python projects/manufacturing/products/spark_rover/make.py --remeasure
    python projects/manufacturing/products/spark_rover/make.py --plan          # jobs, no engine

The desk robot's make.py is the precedent: the same twins, the same run_job.py, the same
measurements (did it cut free; its outline, every hole, its thickness, against the drawing).
What differs:

- The jobs are the subsystems' own `jobs()` text, run verbatim (chassis.jobs: eight laser
  sheets and the spine foot; arms.jobs: the shoulder brackets; shell.jobs: the face cover).
  Each CNC world powers Z up so the tool tip stands exactly where the program's
  `G10 L20 P1 ... Z` says it does over the blank.
- Jobs run in PARALLEL headless engines, at most `--parallel` (default 2) at a time. Before
  each launch the host's `omnisim-bin*` processes are counted (wait while there are
  `--max-engines`, default 6, or more) and the GPU temperature read (at `--max-gpu-c`,
  default 75 C, or above, only one of ours runs). Every engine has its own log, run folder
  and GRBL port.
- Two jobs whose programs make the same moves (comments aside: the two spine side plates'
  titles say left and right), on the same material and sheet,
  run once; the second part's record says so (`same_as`). `--no-dedupe` runs both.
- `--process print` is reserved for the printed parts (another lane owns printing; not
  implemented here yet).

Each world is written beside the other manufacturing worlds as a hidden, gitignored copy
(`.spark_rover_make_<job>.omniworld`: OmniSim finds a world's controllers from its
folder), its run into <out>/runs/<job>, its program into <out>/parts, and the measurements
into <out>/made.json. `--release` (default spark_rover/release/make) receives made.json,
the programs (cam/) and MAKE.md, the results table.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import shapely
from scipy import ndimage
from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
MANUFACTURING = HERE.parents[1]
ROOT = MANUFACTURING.parents[1]
for _p in (HERE, MANUFACTURING / "cnc", MANUFACTURING / "laser"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import chassis  # noqa: E402   (puts laser/ first on sys.path: inspect_cut, run_job are the laser's)
import arms  # noqa: E402
import inspect_cut  # noqa: E402
import run_job  # noqa: E402

LONGMILL = MANUFACTURING / "machines/longmill_mk2_30x30"
MACHINE = LONGMILL / "longmill_mk2_30x30.grbl"
LASER_MODULE = LONGMILL / "sienci_laserbeam_7w.json"
FINE_TOOLS = LONGMILL / "longmill_mk2_tools_aluminium_fine.json"
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"
DEFAULT_OUT = ROOT / ".tmp/spark_rover_make"
DEFAULT_RELEASE = HERE / "release/make"
PORT_BASE = 2351                # the twins' GRBL ports: not the demos' 2323 nor the desk robot's 2333
SHEET_MARGIN_MM = 4.0           # bare sheet round every laser layout (the desk robot's 98 x 88 for 90 x 80)
COVER_BLANK_MM = (152.0, 122.0)  # the face cover's acrylic blank: its 136 x 105 outline + 8 mm a side
                                 # (shell.cover_program names no blank: the cutter's channel is 3.2 mm wide)
RESOLUTION_MM = {"laser": 0.05, "cnc": 0.05}

# The face cover's program calls "T1 M6 (1/8 in single-flute O-flute, 3.175 mm)", a tool none
# of the LongMill's tool tables holds (their T1 is a 1/4 in end mill, which would cut the cover
# 1.6 mm a side small). This table is that tool as the program names it; the flute length is
# the common 1/2 in of a 1/8 in O-flute. It is a TOOL, not a material: the acrylic stock is
# cutting.py's own "acrylic", which has no published cutting energy, so the twin removes the
# swept volume and reports no cutting load (no melt, chip or crack model).
COVER_TOOLS = {
    "note": "SparkRover face cover (make.py): the 1/8 in single-flute O-flute shell.cover_program names as T1.",
    "tools": [{"number": 1, "name": "1/8 in single-flute O-flute (plastics)", "shape": "flat", "diameter_mm": 3.175,
               "flutes": 1, "stickout_mm": 0.0, "flute_mm": 12.7, "shank_mm": 3.175}],
    "spindle": json.loads(FINE_TOOLS.read_text(encoding="utf-8"))["spindle"],
}


def _load_by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _rel(p: Path) -> str:
    return Path(p).resolve().relative_to(ROOT).as_posix()


# ---- the jobs ---------------------------------------------------------------------------------

@dataclass
class Blind:
    """A feature cut part-way into a part (a tap pilot): centre, drawn diameter and depth, mm."""
    x: float
    y: float
    d: float
    depth: float


@dataclass
class Job:
    name: str                       # the subsystem's jobs() key
    process: str                    # laser | cnc (print: not here yet)
    subsystem: str
    program: str
    sheet_mm: Tuple[float, float, float]
    material: str                   # laser.MATERIALS / cutting.MATERIALS key
    drawn: Dict[str, Polygon]       # part label -> its drawing in WORK coordinates
    blind: Dict[str, List[Blind]] = field(default_factory=dict)
    tools: Optional[Path] = None
    note: str = ""
    same_as: Optional[str] = None   # an earlier job with the identical program, material and sheet

    @property
    def ext(self) -> str:
        return "gcode" if self.process == "laser" else "nc"

    def key(self):
        # the program without its comments: the two side plates' titles say left and right
        moves = "\n".join(re.sub(r"\(.*?\)", "", ln).strip() for ln in self.program.splitlines()
                          if not ln.lstrip().startswith(";"))
        return (self.process, moves, self.material, tuple(self.sheet_mm), str(self.tools))


def _gcode_xy_extent(program: str) -> Tuple[float, float, float, float]:
    xs = [float(v) for v in re.findall(r"X(-?[\d.]+)", program)]
    ys = [float(v) for v in re.findall(r"Y(-?[\d.]+)", program)]
    return min(xs), min(ys), max(xs), max(ys)


def _sheet_t(program: str) -> float:
    m = re.search(r"(\d+(?:\.\d+)?) mm birch plywood", program.splitlines()[0])
    if not m:
        raise ValueError("a laser job's title does not name its plywood thickness")
    return float(m.group(1))


def laser_layout(r=None) -> Dict[str, Dict[str, Polygon]]:
    """Each chassis laser job's parts where chassis.laser_jobs places them on the sheet
    (work coordinates). laser_jobs returns the program text only, so the layout is repeated
    here; check_layout() holds it to the programs."""
    c = chassis._r(r)
    xm = chassis.motor_xs()[0]
    sp = chassis._centred(chassis.spacer(c, xm, 1))
    side = chassis._centred(chassis.side_plate(c))
    return {
        "deck_lower": {"deck_lower": chassis.deck_layer(c, False)},
        "deck_upper": {"deck_upper": chassis.deck_layer(c, True)},
        "spacers": {f"spacer_{k + 1}": affinity.translate(sp, dx, 0.0) for k, dx in enumerate((-36.0, -12.0, 12.0, 36.0))},
        "spine_side_l": {"spine_side_l": side},
        "spine_side_r": {"spine_side_r": side},
        "spine_rear": {"spine_rear": chassis._centred(chassis.rear_plate(c))},
        "spine_front": {"spine_front": chassis._centred(chassis.front_plate(c))},
        "spine_rungs": {"mid_rung": affinity.translate(chassis._centred(chassis.mid_rung(c)), -20.0, 0.0),
                        "top_rung": affinity.translate(chassis._centred(chassis.top_rung(c)), 20.0, 0.0)},
    }


def bracket_layout(r=None) -> Tuple[Dict[str, Polygon], Dict[str, List[Blind]]]:
    """Both shoulder brackets where arms.bracket_program cuts them: through features in the
    drawing, the tap pilots (4.8 mm deep, finished by hand) as blind features."""
    a = arms._arm(r)
    outline = arms.bracket_outline(a)
    through = outline.difference(arms.C(0, 0, a.bore_r))
    pilot_depth = min(a.bracket_t + arms.THROUGH_MM, arms.PILOT_MM)

    def place(g, cx, cy):
        return affinity.translate(affinity.scale(g, 1.0, -1.0, origin=(0, 0)), cx, cy)
    drawn, blind = {}, {}
    pts = arms.shoulder_bolts_ts() + arms.FLANGE_SCREWS
    for k, (cx, cy) in enumerate(arms.BRACKET_CENTRES):
        label = f"shoulder_bracket_{k + 1}"
        drawn[label] = place(through, cx, cy)
        blind[label] = [Blind(cx + t, cy - s, arms.TAP["M3_alu"], pilot_depth) for t, s in pts]
    return drawn, blind


def cover_layout(c) -> Polygon:
    import shell
    o = shell.face_cover_outline(c)
    _, z0, _, z1 = o.bounds
    return affinity.translate(o, 0.0, -(z0 + z1) / 2.0)


def all_jobs(r=None, processes: Sequence[str] = ("laser", "cnc"), dedupe: bool = True) -> List[Job]:
    jobs: List[Job] = []
    cj = chassis.jobs(r)
    if "laser" in processes:
        layout = laser_layout(r)
        for name, program in cj.items():
            if name not in layout:
                continue
            drawn = layout[name]
            x0, y0, x1, y1 = unary_union(list(drawn.values())).bounds
            t = _sheet_t(program)
            # 4 mm of bare sheet round the layout, down to 2 mm where that keeps a side inside
            # one window (the spacers: 93.7 mm + 2 x 2.6)
            side = [max(2.0, min(SHEET_MARGIN_MM, (WINDOW_MAX_MM - e) / 2)) if e + 4.0 <= WINDOW_MAX_MM else SHEET_MARGIN_MM
                    for e in (x1 - x0, y1 - y0)]
            sheet = (round(x1 - x0 + 2 * side[0], 1), round(y1 - y0 + 2 * side[1], 1), t)
            jobs.append(Job(name, "laser", "chassis", program, sheet, "plywood", drawn))
        missing = [n for n in cj if n not in layout and n != "spine_foot"]
        if missing:
            raise ValueError(f"chassis.jobs() has laser jobs make.py does not lay out: {missing}")
    if "cnc" in processes:
        c = chassis._r(r)
        foot = chassis.spine_foot(c)
        jobs.append(Job("spine_foot", "cnc", "chassis", cj["spine_foot"], chassis.SPINE_FOOT_STOCK_MM,
                        "aluminium_alloy", {"spine_foot": foot}, tools=FINE_TOOLS))
        drawn, blind = bracket_layout(r)
        jobs.append(Job("shoulder_brackets", "cnc", "arms", arms.jobs(r)["shoulder_brackets"], arms.BRACKET_STOCK_MM,
                        "aluminium_alloy", drawn, blind, tools=FINE_TOOLS))
        import shell
        sc = shell.config(r)
        sj = shell.jobs(r)
        jobs.append(Job("face_cover", "cnc", "shell", sj["face_cover"], (*COVER_BLANK_MM, sc.cover_t), "acrylic",
                        {"face_cover": cover_layout(sc)}, tools=None,
                        note="acrylic: cutting.py has the material but no cutting energy, so no cutting load is "
                             "modelled; the tool is make.py's COVER_TOOLS (no LongMill table holds a 1/8 in O-flute)"))
    if dedupe:
        seen: Dict[tuple, str] = {}
        for j in jobs:
            if j.key() in seen:
                j.same_as = seen[j.key()]
            else:
                seen[j.key()] = j.name
    return jobs


def check_layout(job: Job, tol_mm: float = 0.15) -> dict:
    """The drawn layout against the program: the laser's path runs kerf/2 outside every outline,
    so the drawing's bounds grown by that must be the program's XY extent."""
    if job.process != "laser":
        return {"checked": False}
    x0, y0, x1, y1 = unary_union(list(job.drawn.values())).bounds
    k = chassis.LASER_KERF_MM / 2.0
    gx0, gy0, gx1, gy1 = _gcode_xy_extent(job.program)
    err = max(abs(gx0 - (x0 - k)), abs(gy0 - (y0 - k)), abs(gx1 - (x1 + k)), abs(gy1 - (y1 + k)))
    if err > tol_mm:
        raise ValueError(f"{job.name}: make.py's layout is {err:.3f} mm off the program's extent")
    return {"checked": True, "max_err_mm": round(err, 3)}


# ---- the worlds -------------------------------------------------------------------------------

def _world(name: str) -> Path:
    return WORLDS / f".spark_rover_make_{name}.omniworld"


def _pack() -> dict:
    return json.loads((LONGMILL / "cad/pack.json").read_text(encoding="utf-8"))


def s0_z_for(program: str, thickness: float) -> float:
    """Z's power-up carriage position that puts the tool tip where the program's
    `G10 L20 P1 ... Z<h>` says it is: h mm over the blank's top (cnc/world.py places the pack's
    tool tip at the CAD pose, moved by s0 - cad_pose_s)."""
    m = re.search(r"G10 L20 P1[^\n]*Z(-?[\d.]+)", program)
    if not m:
        raise ValueError("the program sets no work Z")
    pack = _pack()
    tip0 = pack["tool_tip"][2] * 1000.0
    return float(m.group(1)) + pack["bed_top_m"] * 1000.0 + thickness - tip0 + pack["cad_pose_s"][2]


_WORLD_LOCK = threading.Lock()


# The engine's MillingStock holds at most 31 tiles of 64 cells a side (OmMillingStock.cpp
# kMaxCellsPerAxis, 1984) and coarsens a longer sheet to fit: 0.09 mm on the 178 mm deck,
# 0.106 mm on the 210 mm side plate. The laser model was calibrated at 0.05 mm, and its kerf's
# bottom is about 0.05 mm wide (laser/README.md), so a coarser sheet keeps a 0.11 mm skin
# under every cut and nothing falls out (measured: the upper deck, 2026-11-03). A laser sheet
# longer than WINDOW_MAX_MM is therefore cut as WINDOWS, each its own sheet at 0.05 mm, each
# running the job with the burns that cannot reach it turned into rapids, and the windows'
# height maps are stitched back into the sheet before it is measured.
WINDOW_MAX_MM = 99.0
WINDOW_MARGIN_MM = 2.0          # a burn within this of a window is kept (spot + cone << 1 mm)


def windows(job: Job) -> List[Tuple[float, float, float, float]]:
    """The job's sheet split into windows (work coordinates x0, y0, x1, y1), edges on the
    0.05 mm grid; one window when the sheet already fits."""
    if job.process != "laser":
        return [(-job.sheet_mm[0] / 2, -job.sheet_mm[1] / 2, job.sheet_mm[0] / 2, job.sheet_mm[1] / 2)]
    res = RESOLUTION_MM["laser"]
    edges = []
    for s in job.sheet_mm[:2]:
        n = int(math.ceil(s / WINDOW_MAX_MM - 1e-9))
        cells = int(round(s / res))
        edges.append([-s / 2 + round(cells * k / n) * res for k in range(n + 1)])
    ex, ey = edges
    return [(ex[a], ey[b], ex[a + 1], ey[b + 1]) for b in range(len(ey) - 1) for a in range(len(ex) - 1)]


_XY = re.compile(r"^G([01]) X(-?[\d.]+) Y(-?[\d.]+)(.*)$")


def crop_program(program: str, win: Tuple[float, float, float, float], margin: float = WINDOW_MARGIN_MM) -> str:
    """The job as one window's sheet needs it: every burn (G1 in XY) whose segment cannot come
    within `margin` of the window becomes a rapid (the laser is off in a G0 under M4), runs of
    rapids collapse into one, and every burn kept states its S and F (they were modal on a line
    that may now be a rapid). Passes, focus moves and order are the job's."""
    x0, y0, x1, y1 = win[0] - margin, win[1] - margin, win[2] + margin, win[3] + margin
    out: List[str] = []
    x = y = 0.0
    s_word = f_word = ""
    pending = None
    for ln in program.splitlines():
        m = _XY.match(ln.strip())
        if not m:
            if pending and not ln.startswith(";"):
                out.append(pending)
                pending = None
            out.append(ln)
            continue
        g, nx, ny, rest = m.group(1), float(m.group(2)), float(m.group(3)), m.group(4)
        ms, mf = re.search(r"S[\d.]+", rest), re.search(r"F[\d.]+", rest)
        s_word = ms.group(0) if ms else s_word
        f_word = mf.group(0) if mf else f_word
        near = not (max(x, nx) < x0 or min(x, nx) > x1 or max(y, ny) < y0 or min(y, ny) > y1)
        if g == "1" and near:
            if pending:
                out.append(pending)
                pending = None
            out.append(f"G1 X{m.group(2)} Y{m.group(3)} {s_word} {f_word}".rstrip())
        else:
            pending = f"G0 X{m.group(2)} Y{m.group(3)}"
        x, y = nx, ny
    if pending:
        out.append(pending)
    return "\n".join(out) + "\n"


def build_world(job: Job, out: Path, port: int, win=None) -> Path:
    with _WORLD_LOCK:
        return _build_world(job, out, port, win)


def _build_world(job: Job, out: Path, port: int, win=None) -> Path:
    if job.process == "laser":
        world = _world(f"{job.name}_w{win[0]}" if win else job.name)
        laser_world = _load_by_path("laser_world", MANUFACTURING / "laser/world.py")
        sheet = job.sheet_mm
        if win:
            wx0, wy0, wx1, wy1 = win[1]
            sheet = (wx1 - wx0, wy1 - wy0, job.sheet_mm[2])
        text = laser_world.build(MACHINE, LONGMILL / "cad", LASER_MODULE, world, job.material, sheet,
                                 RESOLUTION_MM["laser"], port)
        if win:
            # build() centres the sheet under the focus (work 0, 0); move it to the window.
            m = re.search(r"DEF STOCK MillingStock \{ translation (\S+) (\S+) (\S+) ", text)
            tx = float(m.group(1)) + (sheet[0] / 2 + wx0) / 1000.0
            ty = float(m.group(2)) + (sheet[1] / 2 + wy0) / 1000.0
            text = text[:m.start()] + f"DEF STOCK MillingStock {{ translation {tx:.6f} {ty:.6f} {m.group(3)} " + text[m.end():]
        world.write_text(text, encoding="utf-8", newline="\n")
        return world
    world = _world(job.name)
    tools = job.tools
    if tools is None:
        tools = out / "tools" / f"{job.name}_tools.json"
        tools.parent.mkdir(parents=True, exist_ok=True)
        tools.write_text(json.dumps(COVER_TOOLS, indent=2), encoding="utf-8")
    sx, sy, t = job.sheet_mm
    s0 = _pack()["cad_pose_s"]
    subprocess.run([sys.executable, str(MANUFACTURING / "cnc/world.py"), "--machine", str(MACHINE),
                    "--pack", str(LONGMILL / "cad"), "--s0", f"{s0[0]:g},{s0[1]:g},{s0_z_for(job.program, t):.3f}",
                    "--stock", f"{sx:g},{sy:g},{t:g}", "--stock-resolution", f"{RESOLUTION_MM['cnc']:g}",
                    "--tools", str(tools), "--material", job.material, "--bridge-port", "0", "--port", str(port),
                    "--out", str(world)], check=True, cwd=ROOT, capture_output=True)
    return world


# ---- the measurements ------------------------------------------------------------------------

def _ij(z: dict, x: float, y: float) -> Tuple[int, int]:
    r = z["res"] * 1000.0
    return int(round(y / r + (z["ny"] - 1) / 2)), int(round(x / r + (z["nx"] - 1) / 2))


def _inside(z: dict, j: int, i: int) -> bool:
    return 0 <= j < z["ny"] and 0 <= i < z["nx"]


def _cells_mm(z: dict, ys: np.ndarray, xs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    r = z["res"] * 1000.0
    return (xs - (z["nx"] - 1) / 2) * r, (ys - (z["ny"] - 1) / 2) * r


def _blind(z: dict, mask: np.ndarray, b: Blind) -> dict:
    """A blind feature as cut: the cells of the part round its centre cut deeper than 0.2 mm."""
    r = z["res"] * 1000.0
    cell = r * r
    j, i = _ij(z, b.x, b.y)
    w = int(math.ceil((b.d / 2 + 1.0) / r))
    j0, j1, i0, i1 = max(0, j - w), min(z["ny"], j + w + 1), max(0, i - w), min(z["nx"], i + w + 1)
    depth = (z["top"] - z["h"][j0:j1, i0:i1]) * 1000.0
    deep = (depth > 0.2) & ndimage.binary_fill_holes(mask[j0:j1, i0:i1] | (depth > 0.2))
    labels, _ = ndimage.label(deep)
    k = labels[j - j0, i - i0] if _inside(z, j, i) else 0
    if not k:
        return {"at": [round(b.x, 2), round(b.y, 2)], "cut": False, "drawn_d_mm": b.d, "drawn_depth_mm": b.depth}
    reg = labels == k
    ys, xs = np.nonzero(reg)
    cx, cy = _cells_mm(z, ys + j0, xs + i0)
    core = np.hypot(cx - b.x, cy - b.y) <= max(r, b.d / 2 - 0.5)
    d_reg = depth[reg]
    return {"at": [round(b.x, 2), round(b.y, 2)], "cut": True, "drawn_d_mm": b.d, "cut_d_mm": round(2 * math.sqrt(reg.sum() * cell / math.pi), 3),
            "drawn_depth_mm": round(b.depth, 3), "depth_mm": round(float(np.median(d_reg[core])), 3) if core.any() else None,
            "depth_max_mm": round(float(d_reg.max()), 3),
            "through": bool((z["h"][j0:j1, i0:i1][reg] <= 1e-7).any())}


SKIN_MAX_MM = 1.0       # a part (or hole) still held at this residual height is reported not free at all
SKIN_TOL_MM = 0.002


def _bisect_skin(test, hi: float = SKIN_MAX_MM) -> Optional[float]:
    """The least residual height t (mm) at which test(t) holds, treating every cell no higher
    than t as cut through; None if it does not hold even at `hi`."""
    if not test(hi):
        return None
    lo = 0.0
    while hi - lo > SKIN_TOL_MM:
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if test(mid) else (mid, hi)
    return math.ceil(hi * 1000.0 - 1e-9) / 1000.0     # rounded UP: the test holds there


def part_skin(z: dict, poly: Polygon) -> Optional[float]:
    """The thickest web (mm) that still joins the part to the sheet: the part comes loose once
    every cell no higher than that is counted as cut through."""
    j, i = _ij(z, *poly.representative_point().coords[0])

    def free(t):
        lab, _ = ndimage.label(z["h"] > t / 1000.0)
        k = lab[j, i]
        return k == 0 or not (np.any(lab[0] == k) or np.any(lab[-1] == k) or np.any(lab[:, 0] == k) or np.any(lab[:, -1] == k))
    return _bisect_skin(free)


def hole_skin(z: dict, hole: Polygon) -> Tuple[Optional[float], Optional[float]]:
    """A hole whose slug did not come away: the web (mm) that holds the slug, and the hole's
    area (mm2) once that web is counted as cut. Measured in a window round the hole."""
    r = z["res"] * 1000.0
    x0, y0, x1, y1 = hole.bounds
    j0, i0 = _ij(z, x0 - 1.0, y0 - 1.0)
    j1, i1 = _ij(z, x1 + 1.0, y1 + 1.0)
    j0, i0, j1, i1 = max(0, j0), max(0, i0), min(z["ny"], j1 + 1), min(z["nx"], i1 + 1)
    h = z["h"][j0:j1, i0:i1]
    cj, ci = _ij(z, *(hole.centroid if hole.centroid.within(hole) else hole.representative_point()).coords[0])
    cj, ci = cj - j0, ci - i0

    ys, xs = np.mgrid[j0:j1, i0:i1]
    X, Y = _cells_mm(z, ys, xs)
    near = shapely.contains_xy(hole.buffer(0.5), X, Y)       # only this hole's opening

    def opening(t):
        """The cut ring (cells no higher than t, 8-connected) and what it encloses."""
        filled = ndimage.binary_fill_holes(h <= t / 1000.0)
        lab, _ = ndimage.label(filled & near)
        return lab == lab[cj, ci] if lab[cj, ci] else None

    def encloses(t):
        o = opening(t)
        return o is not None and not (o[0].any() or o[-1].any() or o[:, 0].any() or o[:, -1].any())             and (h[o] > t / 1000.0).any()                    # a slug inside, not a cell of the kerf
    t = _bisect_skin(encloses)
    if t is None:
        return None, None
    return t, float(opening(t).sum() * r * r)


def measure_job(z: dict, job: Job) -> dict:
    """measure_parts(), and for a part that did not come loose, the web that holds it
    (`free_at_skin_mm`) and its size and holes once that web is counted as cut."""
    rep = measure_parts(z, job, job.drawn)
    for label, p in list(rep["parts"].items()):
        if p["cut_free"]:
            continue
        t = part_skin(z, job.drawn[label])
        if t is None:
            p["free_at_skin_mm"] = None
            continue
        zt = dict(z, h=np.where(z["h"] <= t / 1000.0, 0.0, z["h"]))
        q = measure_parts(zt, job, {label: job.drawn[label]})["parts"][label]
        q.update(cut_free=False, free_at_skin_mm=t, measured_with_skin_cut=True)
        rep["parts"][label] = q
    return rep


def measure_parts(z: dict, job: Job, drawn: Dict[str, Polygon]) -> dict:
    """Every part of a job against its drawing.

    A part is the loose piece (material not joined to the sheet's edge) at its drawing's
    representative point. Outline: the size the part's cells cover (n cells x resolution)
    and its mean edge offset, (cut area with its holes filled - drawn outline area) / drawn
    perimeter (+ = oversize). Holes: the void each leaves in the filled part (a laser's slug
    falls out; an end mill's hole is empty), as the area's equivalent diameter and its mean
    edge offset (+ = the hole is small). Thickness: the part's cells' height (its top is never
    cut). Blind features: depth and diameter as cut."""
    r = z["res"] * 1000.0
    cell = r * r
    pieces = inspect_cut.pieces(z)
    loose = [p for p in pieces if p["fell_out"]]
    used = set()
    parts = {}
    for label, poly in drawn.items():
        rp = poly.representative_point()
        j, i = _ij(z, rp.x, rp.y)
        part = next((p for p in loose if p["_mask"][j, i]), None) if _inside(z, j, i) else None
        if part is None:
            attached = _inside(z, j, i) and z["h"][j, i] > 1e-7
            parts[label] = {"cut_free": False, "material_at_part": bool(attached)}
            continue
        used.add(part["label"])
        mask = part["_mask"]
        filled = ndimage.binary_fill_holes(mask)
        ys, xs = np.nonzero(filled)
        ext = Polygon(poly.exterior)
        x0, y0, x1, y1 = poly.bounds
        size = [round((xs.max() - xs.min() + 1) * r, 3), round((ys.max() - ys.min() + 1) * r, 3)]
        h_mm = z["h"][mask] * 1000.0
        rep = {"cut_free": True, "size_mm": size, "drawn_mm": [round(x1 - x0, 3), round(y1 - y0, 3)],
               "outline_area_mm2": round(float(filled.sum() * cell), 2), "drawn_outline_area_mm2": round(ext.area, 2),
               "outline_offset_mm": round(float((filled.sum() * cell - ext.area) / ext.length), 4),
               "thickness_mm": round(float(np.median(h_mm)), 3), "thickness_min_mm": round(float(np.percentile(h_mm, 1)), 3),
               "drawn_thickness_mm": job.sheet_mm[2], "holes": []}
        voids, _ = ndimage.label(filled & ~mask)
        for ring in poly.interiors:
            hole = Polygon(ring)
            c = hole.centroid if hole.centroid.within(hole) else hole.representative_point()
            hj, hi = _ij(z, c.x, c.y)
            k = voids[hj, hi]
            e = {"at": [round(c.x, 2) + 0.0, round(c.y, 2) + 0.0], "cut": bool(k),
                 "drawn_d_mm": round(2 * math.sqrt(hole.area / math.pi), 3)}
            if k:
                area = float((voids == k).sum() * cell)
            else:
                t, area = hole_skin(z, hole)
                e["free_at_skin_mm"] = t
            if area:
                e.update(cut_d_mm=round(2 * math.sqrt(area / math.pi), 3),
                         edge_offset_mm=round((hole.area - area) / hole.length, 4))
            rep["holes"].append(e)
        cut = [h for h in rep["holes"] if h["cut"]]
        rep["holes_cut"] = f"{len(cut)}/{len(rep['holes'])}"
        skins = [h["free_at_skin_mm"] for h in rep["holes"] if "free_at_skin_mm" in h]
        if skins:
            rep["holes_held_by_skin"] = len(skins)
            rep["hole_skin_max_mm"] = None if None in skins else max(skins)
        sized = [h for h in rep["holes"] if "cut_d_mm" in h]
        if sized:
            err = [h["cut_d_mm"] - h["drawn_d_mm"] for h in sized]
            rep["hole_d_error_mm"] = {"min": round(min(err), 3), "max": round(max(err), 3)}
        if label in job.blind:
            rep["blind"] = [_blind(z, mask, b) for b in job.blind[label]]
            bc = [b for b in rep["blind"] if b["cut"]]
            rep["blind_cut"] = f"{len(bc)}/{len(rep['blind'])}"
            if bc:
                rep["blind_d_mm"] = [min(b["cut_d_mm"] for b in bc), max(b["cut_d_mm"] for b in bc)]
                rep["blind_depth_mm"] = [min(b["depth_mm"] for b in bc if b["depth_mm"] is not None),
                                         max(b["depth_mm"] for b in bc if b["depth_mm"] is not None)]
        parts[label] = rep
    big = [p for p in loose if p["label"] not in used]
    return {"parts": parts, "loose_other": len(big),
            "loose_other_max_mm2": round(max((p["area_mm2"] for p in big), default=0.0), 2)}


def stitch(job: Job, maps: List[dict]) -> dict:
    """The windows' height maps (in windows() order) as one map of the whole sheet."""
    wins = windows(job)
    if len(maps) == 1:
        return maps[0]
    res = RESOLUTION_MM["laser"]
    sx, sy = job.sheet_mm[:2]
    nx, ny = int(round(sx / res)) + 1, int(round(sy / res)) + 1
    h = np.full((ny, nx), np.nan)
    for (wx0, wy0, wx1, wy1), z in zip(wins, maps):
        if abs(z["res"] * 1000.0 - res) > 1e-9:
            raise ValueError(f"{job.name}: a window was coarsened to {z['res'] * 1000:.4f} mm")
        i0, j0 = int(round((wx0 + sx / 2) / res)), int(round((wy0 + sy / 2) / res))
        if (z["nx"], z["ny"]) != (int(round((wx1 - wx0) / res)) + 1, int(round((wy1 - wy0) / res)) + 1):
            raise ValueError(f"{job.name}: a window's map is {z['nx']} x {z['ny']} cells")
        view = h[j0:j0 + z["ny"], i0:i0 + z["nx"]]
        np.copyto(view, np.where(np.isnan(view), z["h"], np.minimum(view, z["h"])))
    if np.isnan(h).any():
        raise ValueError(f"{job.name}: the windows leave cells uncovered")
    return {"nx": nx, "ny": ny, "res": res / 1000.0, "top": maps[0]["top"], "h": h, "fluence": None}


def trace_times(trace: Path) -> dict:
    """Machine time from the twin's trace: first to last sample the machine was not Idle."""
    first = last = None
    peak_w = None
    with trace.open("r", encoding="utf-8") as f:
        for line in f:
            if '"kind": "sample"' not in line:
                continue
            d = json.loads(line)
            if d.get("state") != "Idle":
                first = d["t"] if first is None else first
                last = d["t"]
            if d.get("power_w") is not None:
                peak_w = max(peak_w or 0.0, d["power_w"])
    return {"machine_s": round(last - first, 1) if first is not None else None, "peak_cutting_power_w": peak_w}


# ---- running ----------------------------------------------------------------------------------

def count_engines() -> int:
    try:
        out = subprocess.run(["tasklist"], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return 0
    return sum(1 for line in out.splitlines() if line.lower().startswith("omnisim-bin"))


def gpu_temp_c() -> Optional[float]:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=30).stdout
        return max(float(v) for v in out.split())
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def _port_free(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) != 0


class Gate:
    """Admits one launch at a time: waits while the host has max_engines omnisim-bin processes
    or more, and while the GPU is at max_gpu_c or above and one of ours is already running."""

    def __init__(self, max_engines: int, max_gpu_c: float, log):
        self.max_engines, self.max_gpu_c, self.log = max_engines, max_gpu_c, log
        self.lock = threading.Lock()
        self.running = 0
        self.last_launch = 0.0

    def enter(self, name: str) -> None:
        with self.lock:
            while True:
                n, t = count_engines(), gpu_temp_c()
                hot = t is not None and t >= self.max_gpu_c
                if n < self.max_engines and not (hot and self.running > 0):
                    break
                self.log(f"[gate] {name} waits: {n} engines, GPU {t} C, {self.running} of ours running")
                time.sleep(15.0)
            time.sleep(max(0.0, 8.0 - (time.time() - self.last_launch)))   # stagger engine start-ups
            self.running += 1
            self.last_launch = time.time()
            self.log(f"[gate] {name} launches: {n} engines on the host, GPU {t} C, {self.running} of ours")

    def leave(self) -> None:
        with self.lock:
            self.running -= 1


def _run_dir(out: Path, job: Job, k: Optional[int]) -> Path:
    return out / "runs" / job.name / (f"w{k}" if k is not None else "")


def run_task(job: Job, k: Optional[int], win, out: Path, port: int, gate: Gate, log) -> dict:
    """One engine run: the whole job, or (k, win) one window of a laser sheet."""
    parts = out / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    program = parts / f"{job.name}.{job.ext}"
    program.write_text(job.program, encoding="utf-8", newline="\n")
    run = _run_dir(out, job, k)
    if run.exists():
        shutil.rmtree(run)
    run.mkdir(parents=True)
    sent = program
    if k is not None:
        sent = run / f"window_{k}.{job.ext}"
        sent.write_text(crop_program(job.program, win), encoding="utf-8", newline="\n")
    while not _port_free(port):
        port += 10
    world = build_world(job, out, port, (k, win) if k is not None else None)
    tag = job.name + (f"/w{k}" if k is not None else "")
    gate.enter(tag)
    try:
        t0 = time.time()
        res = run_job.run(world, sent, run, port=port, timeout_s=4 * 3600)
        wall = round(time.time() - t0, 1)
    finally:
        gate.leave()
        world.unlink(missing_ok=True)
    twin = res.get("twin") or {}
    rep = {"window": list(win) if k is not None else None, "job": res.get("job"), "wall_s": wall,
           "connected_after_s": res.get("connected_after_s"), "alarms": twin.get("alarms"), "stalls": twin.get("stalls")}
    rep.update(trace_times(run / "trace.jsonl"))
    log(f"[run] {tag}: {(res.get('job') or {}).get('state')}, machine {rep['machine_s']} s, wall {wall} s")
    return rep


def finish_job(job: Job, out: Path, runs: List[dict], estimate_s: Optional[float]) -> dict:
    """Stitch (when windowed) and measure a job whose runs are all done."""
    wins = windows(job)
    windowed = len(wins) > 1
    if windowed:
        z = stitch(job, [inspect_cut.load(_run_dir(out, job, k) / "sheet.zmap") for k in range(len(wins))])
    else:
        z = inspect_cut.load(_run_dir(out, job, None) / "sheet.zmap")
    rep = {"process": job.process, "subsystem": job.subsystem, "material": job.material, "sheet_mm": list(job.sheet_mm),
           "resolution_mm": round(z["res"] * 1000.0, 5), "program": _rel(out / "parts" / f"{job.name}.{job.ext}"),
           "preflight_estimate_s": estimate_s}
    if job.note:
        rep["note"] = job.note
    states = {(r.get("job") or {}).get("state") for r in runs}
    rep["state"] = "done" if states == {"done"} else sorted(str(s) for s in states)
    rep["alarms"] = [a for r in runs for a in (r.get("alarms") or [])]
    rep["stalls"] = [a for r in runs for a in (r.get("stalls") or [])]
    if windowed:
        rep["windows"] = runs
        # The machine runs the whole program once: its time is preflight's estimate, which
        # matched the twin within 0.3 % on every job run whole (spine foot, brackets, upper deck, spacers).
        rep["machine_s"] = estimate_s
        rep["machine_s_source"] = f"preflight estimate of the whole program (the twin ran it as {len(wins)} windows)"
        rep["wall_s"] = round(sum(r["wall_s"] for r in runs), 1)
        rep["wall_s_note"] = "engine wall time summed over the windows"
    else:
        r = runs[0]
        rep.update(job=r["job"], wall_s=r["wall_s"], connected_after_s=r["connected_after_s"], machine_s=r["machine_s"],
                   machine_s_source="the twin's trace", peak_cutting_power_w=r.get("peak_cutting_power_w"))
    rep.update(measure_job(z, job))
    return rep


def remeasure(job: Job, out: Path, prev: dict) -> dict:
    runs = prev.get("windows") or prev.get("runs") or [{k: prev.get(k) for k in ("job", "wall_s", "connected_after_s", "machine_s",
                                                              "alarms", "stalls", "peak_cutting_power_w")}]
    return finish_job(job, out, runs, prev.get("preflight_estimate_s"))


# ---- the report -------------------------------------------------------------------------------

def _f(v, nd=2) -> str:
    return "n/a" if v is None else f"{v:.{nd}f}"


def report_md(made: dict) -> str:
    rows = ["| Job | Part | Machine time | Engine wall time | Cut free | Outline (drawn), mm | Edge offset | Holes | "
            "Hole d error | Thickness (drawn) | Grid |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, rep in made["jobs"].items():
        src = made["jobs"].get(rep.get("same_as")) if rep.get("same_as") else rep
        if "error" in src:
            rows.append(f"| {name} | | | | **error**: {src['error'][:80]} | | | | | | |")
            continue
        for label, p in src.get("parts", {}).items():
            label = name if rep.get("same_as") else label
            est = "est. " if src.get("windows") else ""
            mt = f"{est}{src['machine_s'] / 60:.1f} min" if src.get("machine_s") else "n/a"
            if rep.get("same_as"):
                wt = "same program as " + rep["same_as"]
            else:
                wt = f"{src['wall_s'] / 60:.1f} min" + (f" ({len(src['windows'])} windows)" if src.get("windows") else "")
            grid = f"{src.get('resolution_mm', 0):.3f}"
            if not p["cut_free"] and not p.get("measured_with_skin_cut"):
                rows.append(f"| {name} | {label} | {mt} | {wt} | **no** (held by more than {SKIN_MAX_MM:g} mm) | | | | | | {grid} |")
                continue
            free = "yes" if p["cut_free"] else f"**no**: a {p['free_at_skin_mm']:.2f} mm web holds it (measured with that web cut)"
            he = p.get("hole_d_error_mm")
            extra = ""
            if p.get("blind"):
                extra = (f"; tap pilots {p['blind_cut']}, d {p['blind_d_mm'][0]:.2f}-{p['blind_d_mm'][1]:.2f}, "
                         f"{p['blind_depth_mm'][0]:.2f}-{p['blind_depth_mm'][1]:.2f} deep (drawn 2.5, 4.8)")
            if p.get("holes_held_by_skin"):
                extra += f"; {p['holes_held_by_skin']} slugs held by a web of at most {_f(p.get('hole_skin_max_mm'))} mm"
            rows.append(f"| {name} | {label} | {mt} | {wt} | {free} | {p['size_mm'][0]:.2f} x {p['size_mm'][1]:.2f} "
                        f"({p['drawn_mm'][0]:g} x {p['drawn_mm'][1]:g}) | {p['outline_offset_mm']:+.3f} | {p['holes_cut']}{extra} | "
                        + (f"{he['min']:+.3f} / {he['max']:+.3f}" if he else "") +
                        f" | {p['thickness_mm']:.2f} ({p['drawn_thickness_mm']:g}) | {grid} |")
    sessions = made.get("sessions", [])
    head = ["# SparkRover Mini: parts made on the twins", "",
            f"Generated by `make.py` ({made.get('generated')}). LongMill MK2 30x30 twin with the Sienci LaserBeam 7W "
            "(laser) and the router (CNC), headless, at most two of these engines at a time beside the other lanes'. "
            f"Machine time for the set: {made.get('machine_s_total', 0) / 60:.1f} min. Wall time: "
            + "; ".join(f"{x['wall_s'] / 60:.1f} min for {', '.join(x['jobs'])}" for x in sessions) + ".", "",
            "Outline: the size the part's height-map cells cover. Edge offset: (cut outline area - drawn) / drawn "
            "perimeter, + is oversize. Hole d error: cut - drawn, each void's equivalent diameter (for a laser cut, "
            "the opening at the BOTTOM face: its kerf is a V). Grid: the height map's cell, mm. A laser sheet longer "
            f"than {WINDOW_MAX_MM:g} mm is cut as windows at 0.05 mm and stitched (the engine would coarsen it); its "
            "machine time is preflight's estimate of the whole program, its wall time the windows' sum.", ""]
    notes = [n for n in made.get("notes", [])]
    return "\n".join(head + rows + ([""] + [f"- {n}" for n in notes] if notes else []) + [""])


def write_release(made: dict, out: Path, release: Path) -> None:
    release.mkdir(parents=True, exist_ok=True)
    (release / "made.json").write_text(json.dumps(made, indent=1, default=float), encoding="utf-8")
    cam = release / "cam"
    cam.mkdir(exist_ok=True)
    for p in (out / "parts").glob("*"):
        shutil.copy2(p, cam / p.name)
    if (out / "tools").exists():
        for p in (out / "tools").glob("*.json"):
            shutil.copy2(p, cam / p.name)
    (release / "MAKE.md").write_text(report_md(made), encoding="utf-8")


# ---- main -------------------------------------------------------------------------------------

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Make the SparkRover Mini's parts on the twins and measure them.")
    p.add_argument("--process", choices=("laser", "cnc", "print"), action="append",
                   help="laser | cnc (default: both); print is not implemented here yet")
    p.add_argument("--only", action="append", help="one job by its jobs() key (repeatable)")
    p.add_argument("--out", default=str(DEFAULT_OUT))
    p.add_argument("--release", default=str(DEFAULT_RELEASE), help="'' to skip")
    p.add_argument("--parallel", type=int, default=2)
    p.add_argument("--max-engines", type=int, default=6, help="wait while the host runs this many omnisim-bin or more")
    p.add_argument("--max-gpu-c", type=float, default=75.0)
    p.add_argument("--no-dedupe", action="store_true")
    p.add_argument("--remeasure", action="store_true", help="measure saved runs again (no engine)")
    p.add_argument("--plan", action="store_true", help="list the jobs and check their layouts (no engine)")
    a = p.parse_args(argv)
    procs = a.process or ["laser", "cnc"]
    if "print" in procs:
        print("--process print: not implemented in spark_rover/make.py yet (the printed parts belong to the print lane)",
              file=sys.stderr)
        procs = [x for x in procs if x != "print"]
        if not procs:
            return 2
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    jobs = all_jobs(None, procs, dedupe=not a.no_dedupe)
    if a.only:
        unknown = set(a.only) - {j.name for j in jobs}
        if unknown:
            p.error(f"unknown job(s): {sorted(unknown)}")
        jobs = [j for j in jobs if j.name in a.only]
        for j in jobs:
            if j.same_as and j.same_as not in a.only:
                j.same_as = None
    for j in jobs:
        check_layout(j)
    if a.plan:
        for j in jobs:
            print(f"{j.name:20s} {j.process:5s} {j.material:16s} sheet {j.sheet_mm} parts {list(j.drawn)}"
                  + (f" same_as {j.same_as}" if j.same_as else ""))
        return 0
    path = out / "made.json"
    made = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {"jobs": {}}
    lock = threading.Lock()

    def log(msg: str) -> None:
        with lock:
            print(time.strftime("%H:%M:%S"), msg, flush=True)

    def save() -> None:
        with lock:
            path.write_text(json.dumps(made, indent=1, default=float), encoding="utf-8")

    started = time.time()
    if a.remeasure:
        for j in jobs:
            if j.same_as:
                made["jobs"][j.name] = {"same_as": j.same_as}
            elif j.name in made["jobs"]:
                made["jobs"][j.name] = remeasure(j, out, made["jobs"][j.name])
    else:
        gate = Gate(a.max_engines, a.max_gpu_c, log)
        todo = [j for j in jobs if not j.same_as]
        sys.path.insert(0, str(MANUFACTURING / "cnc"))
        import machine as machine_mod
        import preflight
        import tool_table
        m = machine_mod.load(MACHINE)

        def estimate(j: Job, program: str) -> float:
            return preflight.preflight(m, program, tools=tool_table.load(j.tools) if j.tools else None)["estimated_seconds"]
        # Tasks: one per job, or one per window of a laser sheet; longest first, so the pool's
        # tail is short (wall ~ machine / 3 for both twins with two engines, measured).
        tasks, full_est = [], {}
        for j in todo:
            full_est[j.name] = estimate(j, j.program)
            wins = windows(j)
            if len(wins) == 1:
                tasks.append((j, None, None, full_est[j.name]))
            else:
                for k, w in enumerate(wins):
                    tasks.append((j, k, w, estimate(j, crop_program(j.program, w))))
        tasks.sort(key=lambda t: -t[3])
        log("[plan] " + ", ".join(f"{t[0].name}{'' if t[1] is None else f'/w{t[1]}'} ~{t[3] / 60:.0f} min" for t in tasks))
        pending = {j.name: sum(1 for t in tasks if t[0] is j) for j in todo}
        runs: Dict[str, Dict[int, dict]] = {j.name: {} for j in todo}
        slots = list(range(a.parallel))
        slot_lock = threading.Lock()

        def work(t) -> None:
            j, k, w, _ = t
            with slot_lock:
                slot = slots.pop(0)
            try:
                r = run_task(j, k, w, out, PORT_BASE + slot, gate, log)
            except Exception as exc:          # one failed run must not lose the others
                log(f"[fail] {j.name}{'' if k is None else f'/w{k}'}: {exc!r}")
                r = {"error": repr(exc)}
            finally:
                with slot_lock:
                    slots.append(slot)
            with lock:
                runs[j.name][k if k is not None else 0] = r
                pending[j.name] -= 1
                last = pending[j.name] == 0
            if not last:
                return
            rs = [runs[j.name][i] for i in sorted(runs[j.name])]
            try:
                if any("error" in x for x in rs):
                    raise RuntimeError("; ".join(x["error"] for x in rs if "error" in x))
                rep = finish_job(j, out, rs, full_est[j.name])
                log(f"[done] {j.name}: machine {rep['machine_s']} s, wall {rep['wall_s']} s, " +
                    ", ".join(f"{lb} free={v['cut_free']}" for lb, v in rep["parts"].items()))
            except Exception as exc:
                log(f"[fail] {j.name}: {exc!r}")
                rep = {"process": j.process, "error": repr(exc), "runs": rs}
            with lock:
                made["jobs"][j.name] = rep
            save()
        with ThreadPoolExecutor(max_workers=a.parallel) as pool:
            list(pool.map(work, tasks))
        for j in jobs:
            if j.same_as:
                made["jobs"][j.name] = {"same_as": j.same_as, "program": f"move-for-move identical to {j.same_as}'s"}
        made.setdefault("sessions", []).append({"started": time.strftime("%Y-%m-%d %H:%M", time.localtime(started)),
                                                "jobs": [j.name for j in todo], "parallel": a.parallel,
                                                "wall_s": round(time.time() - started, 1)})
        made["wall_s_total"] = round(sum(x["wall_s"] for x in made["sessions"]), 1)
        made["parallel"] = a.parallel
    made["generated"] = time.strftime("%Y-%m-%d %H:%M")
    made["machine_s_total"] = round(sum(v.get("machine_s") or 0.0 for v in made["jobs"].values()), 1)
    save()
    if a.release:
        write_release(made, out, Path(a.release))
    return 0 if all("error" not in v for v in made["jobs"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
