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

"""Print a job without the engine: the twin's plan and hotend on an ideal machine.

The printer twin (fdm_twin + PrintedPart) runs Klipper's plan through servo-driven axes
on Newton at a 2 ms step and lays plastic where the nozzle really went. That costs about
1 ms of engine CPU per step, so a 3.3 h print takes about 1.9 h. This runs the same chain
with the axes left out:

- **Firmware:** `klipper.load_job` and `klipper.Trajectory`, the twin's planner, with its
  input shaper on X and Y and pressure advance on the extruder.
- **Sampling:** the trajectory at the twin's step times k * dt, exactly as
  `klipper.StepSampler` samples them (the same floats), in parallel chunks.
- **Hotend:** `hotend.Hotend`'s model, stepped once per step to the extruder command at
  the step's end, as fdm_twin steps it (a compiled copy, bitwise equal to hotend.py).
- **Laying:** PrintedPart's rule (src/omnisim/nodes/OmPrintedPart.cpp), step by step: the
  plastic that left the nozzle during a step is laid along the nozzle's path that step,
  with its height from the map of what is under the nozzle, its width from its volume,
  straight runs merged into one bead, blobs, bridges and dropped ooze, all with the
  engine's constants and its float32 bead storage.

The one difference from the twin is the nozzle: here it is exactly where Klipper's steppers
put it (the shaped command), there it is where the servo-driven axes carried it. Axis
error, servo jitter and anything else the engine adds are left out.

The output is a bead dump in PrintedPart's format (OMBEADS1), which inspect_print.py reads
unchanged, and the inspect report.

    python ideal_print.py part.gcode part.stl --printer profile.json --out part.beads
    python ideal_print.py jobs/cube20_sv08.gcode jobs/cube20.stl --json
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import struct
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
for _p in (HERE, HERE.parent / "core"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import hotend as hotend_mod  # noqa: E402
import inspect_print  # noqa: E402
import klipper  # noqa: E402
import printer as printer_mod  # noqa: E402

try:
    from numba import njit
    HAVE_NUMBA = True
except ImportError:  # pragma: no cover - the kernels then run as plain Python (slow)
    HAVE_NUMBA = False

    def njit(*args, **kwargs):
        if args and callable(args[0]):
            return args[0]
        return lambda f: f


def _world_module():
    """printing/world.py (the CNC twin has a world.py too: load this one by path)."""
    spec = importlib.util.spec_from_file_location("printer_twin_world_ideal", HERE / "world.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_WORLD = _world_module()

# ── PrintedPart's constants (src/omnisim/nodes/OmPrintedPart.cpp, anonymous namespace) ──
DEFAULT_RESOLUTION_M = 0.0002
MIN_RESOLUTION_M = 0.00005
MAX_CELLS_PER_AXIS = 8192
MAX_RECORDED_BEADS = 50_000_000
TELEPORT_DISTANCE_M = 0.05
BLOB_MIN_VOLUME_M3 = 5e-11
MIN_BEAD_HEIGHT_M = 0.00002
MERGE_COS = 0.9994
MERGE_RELATIVE = 0.12
SAME_LAYER_BAND_M = 0.00002
MIN_STRAND_DIAMETER_RATIO = 0.25
MAX_SUPPORT_SAMPLES = 4000
STILL_RATIO = 0.01
FACE_RATIO = 2.5

# fdm_twin: the hotend keeps stepping (and the engine keeps laying) until this long after
# the planned end, so the melt's last pressure is laid too.
FINISH_AFTER_S = 1.0


# ── the hotend: hotend.Hotend.step, compiled ──────────────────────────────────────────

@njit(cache=True)
def _hotend_run(e_motor, n_sub, h, decay, cap, area, deposited_out):
    """hotend.Hotend.step for every value of e_motor in turn, the first one initialising.
    Writes the cumulative deposited volume (mm^3) after each step into deposited_out and
    returns (skipped, peak_flow). The same operations in the same order as hotend.py, so
    the same doubles."""
    last = e_motor[0]
    pushed = last * area
    out = pushed
    peak = pushed
    skipped = 0.0
    deposited = 0.0
    peak_flow = 0.0
    deposited_out[0] = 0.0
    for s in range(1, e_motor.shape[0]):
        e = e_motor[s]
        de = (e - last) / n_sub
        for _ in range(n_sub):
            pushed += de * area
            pressure = pushed - out
            if pressure > cap:
                skipped += pressure - cap
                pushed -= pressure - cap
                pressure = cap
            flow = pressure * decay
            out += flow
            peak_flow = max(peak_flow, flow / h)
            if out > peak:
                deposited += out - peak
                peak = out
        last = e
        deposited_out[s] = deposited
    return skipped, peak_flow


def run_hotend(hot: hotend_mod.Hotend, e_motor: np.ndarray, dt: float):
    """Cumulative mm^3 deposited after each step of `e_motor` (the first value initialises
    the hotend, as fdm_twin's first hotend.step does); also returns skipped and peak flow."""
    n = max(1, int(math.ceil(dt / hotend_mod.SUB_STEP_S)))
    h = dt / n
    decay = 1.0 - math.exp(-h / hot.lag_s) if hot.lag_s > 0 else 1.0
    cap = hot.lag_s * hot.max_mm3_s
    out = np.zeros(len(e_motor))
    if not len(e_motor):
        return out, 0.0, 0.0
    skipped, peak_flow = _hotend_run(np.ascontiguousarray(e_motor, dtype=np.float64), n, h, decay, cap,
                                     hot.filament_area, out)
    return out, float(skipped), float(peak_flow)


# ── laying: OmPrintedPart::onPhysicsStepEnded, lay, blob, addBead, supportUnder, raise ──

@njit(cache=True)
def _lround(x):
    # C's lround: half away from zero
    return int(math.floor(x + 0.5)) if x >= 0.0 else -int(math.floor(-x + 0.5))


@njit(cache=True)
def _support_under(top, nx, ny, res, ax, ay, bx, by, below_z, blob_state, nozzle):
    """OmPrintedPart::supportUnder. blob_state as _blob's: plastic of the blob the nozzle is
    standing in (its footprint, this layer) is not a crash."""
    in_blob = blob_state[0] >= 0.0 and abs(blob_state[3] - below_z) <= 0.25 * nozzle
    length = math.hypot(bx - ax, by - ay)
    n = min(MAX_SUPPORT_SAMPLES, max(2, int(math.ceil(length / res)) + 1))
    best = -1.0
    crashed = False
    for k in range(n):
        t = k / (n - 1)
        px = ax + (bx - ax) * t
        py = ay + (by - ay) * t
        i = _lround(px / res)
        j = _lround(py / res)
        if i < 0 or j < 0 or i >= nx or j >= ny:
            continue
        hgt = np.float64(top[j * nx + i])
        if hgt > below_z + MIN_BEAD_HEIGHT_M * 2.5 and not (
                in_blob and math.hypot(px - blob_state[1], py - blob_state[2]) < blob_state[5] + res):
            crashed = True
        if hgt < below_z - SAME_LAYER_BAND_M:
            best = max(best, hgt)
    return best, crashed


@njit(cache=True)
def _raise(top, nx, ny, res, ax, ay, bx, by, half_width, z):
    x0 = min(ax, bx) - half_width
    x1 = max(ax, bx) + half_width
    y0 = min(ay, by) - half_width
    y1 = max(ay, by) + half_width
    i0 = max(0, int(math.floor(x0 / res)))
    i1 = min(nx - 1, int(math.ceil(x1 / res)))
    j0 = max(0, int(math.floor(y0 / res)))
    j1 = min(ny - 1, int(math.ceil(y1 / res)))
    dx = bx - ax
    dy = by - ay
    len2 = dx * dx + dy * dy
    t32 = np.float32(z)
    hw2 = half_width * half_width
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            px = i * res - ax
            py = j * res - ay
            t = min(1.0, max(0.0, (px * dx + py * dy) / len2)) if len2 > 0.0 else 0.0
            ex = px - t * dx
            ey = py - t * dy
            if ex * ex + ey * ey > hw2:
                continue
            idx = j * nx + i
            if top[idx] < t32:
                top[idx] = t32


@njit(cache=True)
def _grow(beads, count):
    if count < beads.shape[0]:
        return beads
    bigger = np.zeros((max(1024, 2 * beads.shape[0]), 9), dtype=np.float32)
    bigger[:count] = beads[:count]
    return bigger


@njit(cache=True)
def _add_bead(beads, count, is_open, ax, ay, az, bx, by, bz, width, height, may_merge):
    """OmPrintedPart::addBead. Returns (beads, count, open, merged_or_added)."""
    if may_merge and is_open and count > 0:
        la0 = np.float64(beads[count - 1, 0])
        la1 = np.float64(beads[count - 1, 1])
        la2 = np.float64(beads[count - 1, 2])
        lb0 = np.float64(beads[count - 1, 3])
        lb1 = np.float64(beads[count - 1, 4])
        lb2 = np.float64(beads[count - 1, 5])
        lw = np.float64(beads[count - 1, 6])
        lh = np.float64(beads[count - 1, 7])
        d00, d01, d02 = lb0 - la0, lb1 - la1, lb2 - la2
        d10, d11, d12 = bx - ax, by - ay, bz - az
        l0 = math.sqrt(d00 * d00 + d01 * d01 + d02 * d02)
        l1 = math.sqrt(d10 * d10 + d11 * d11 + d12 * d12)
        g0, g1, g2 = lb0 - ax, lb1 - ay, lb2 - az
        if (beads[count - 1, 8] == 0.0 and l0 > 0.0 and l1 > 0.0
                and math.sqrt(g0 * g0 + g1 * g1 + g2 * g2) < 1e-6 and abs(az - la2) < 1e-6
                and abs(bz - az) < 1e-6 and (d00 * d10 + d01 * d11 + d02 * d12) / (l0 * l1) > MERGE_COS
                and abs(width - lw) < MERGE_RELATIVE * lw and abs(height - lh) < MERGE_RELATIVE * lh):
            beads[count - 1, 6] = np.float32((lw * l0 + width * l1) / (l0 + l1))
            beads[count - 1, 7] = np.float32((lh * l0 + height * l1) / (l0 + l1))
            beads[count - 1, 3] = np.float32(bx)
            beads[count - 1, 4] = np.float32(by)
            beads[count - 1, 5] = np.float32(bz)
            return beads, count, is_open, True
    if count >= MAX_RECORDED_BEADS:
        return beads, count, False, False
    beads = _grow(beads, count)
    beads[count, 0] = np.float32(ax)
    beads[count, 1] = np.float32(ay)
    beads[count, 2] = np.float32(az)
    beads[count, 3] = np.float32(bx)
    beads[count, 4] = np.float32(by)
    beads[count, 5] = np.float32(bz)
    beads[count, 6] = np.float32(width)
    beads[count, 7] = np.float32(height)
    beads[count, 8] = np.float32(0.0)
    return beads, count + 1, True, True


@njit(cache=True)
def _near_blob(blob_state, x, y, z, nozzle):
    """OmPrintedPart::nearBlob: over the blob being grown (within its footprint, and never
    less than a nozzle of its centre), in the same layer."""
    return (math.hypot(blob_state[1] - x, blob_state[2] - y) < max(nozzle, blob_state[5])
            and abs(blob_state[3] - z) <= 0.25 * nozzle)


@njit(cache=True)
def _blob_shape(volume, gap, nozzle):
    """OmPrintedPart::blobShape: (width, height) of a blob of `volume` with `gap` between the
    nozzle tip and what is under it."""
    face = FACE_RATIO * nozzle
    if gap <= 2.0 * nozzle:
        if volume <= 0.25 * math.pi * face * face * gap:
            return math.sqrt(4.0 * volume / (math.pi * gap)), gap
        width = max(face, 2.0 * np.cbrt(3.0 * volume / (2.0 * math.pi)))
    else:
        width = 2.0 * np.cbrt(3.0 * volume / (2.0 * math.pi))
    return width, 4.0 * volume / (math.pi * width * width)


@njit(cache=True)
def _blob(beads, count, is_open, top, nx, ny, res, nozzle, last_support, blob_state, x, y, z, volume):
    """OmPrintedPart::blob: the plastic that left the nozzle at (x, y, z) and stays there.
    blob_state = [index of the blob being grown or -1, its centre x, y, nozzle z, its volume,
    its half width].
    Returns (beads, count, open)."""
    total = volume
    cx, cy, cz = x, y, z
    idx = int(blob_state[0])
    grow = idx >= 0 and _near_blob(blob_state, x, y, z, nozzle)
    if grow:
        total += blob_state[4]
        cx, cy, cz = blob_state[1], blob_state[2], blob_state[3]
    support, crashed = _support_under(top, nx, ny, res, cx, cy, cx + 1e-9, cy, z, blob_state, nozzle)
    if support < 0.0:
        close = z - last_support > 0.0 and z - last_support < 2.0 * nozzle
        support = last_support if close else z - 0.5 * nozzle
    gap = max(MIN_BEAD_HEIGHT_M, z - support)
    width, height = _blob_shape(total, gap, nozzle)
    btop = support + height
    ax, bx = cx - 0.25 * width, cx + 0.25 * width
    if grow:
        beads[idx, 0] = np.float32(ax)
        beads[idx, 1] = np.float32(cy)
        beads[idx, 2] = np.float32(btop)
        beads[idx, 3] = np.float32(bx)
        beads[idx, 4] = np.float32(cy)
        beads[idx, 5] = np.float32(btop)
        beads[idx, 6] = np.float32(width)
        beads[idx, 7] = np.float32(height)
        beads[idx, 8] = np.float32(total)
        blob_state[4] = total
        blob_state[5] = 0.5 * width
    else:
        before = count
        beads, count, is_open, _ = _add_bead(beads, count, False, ax, cy, btop, bx, cy, btop, width, height, False)
        is_open = False
        if count > before:
            beads[count - 1, 8] = np.float32(total)
            blob_state[0] = count - 1
            blob_state[1] = cx
            blob_state[2] = cy
            blob_state[3] = cz
            blob_state[4] = total
            blob_state[5] = 0.5 * width
        else:
            blob_state[0] = -1.0
    _raise(top, nx, ny, res, cx, cy, cx, cy, 0.5 * width, btop)
    return beads, count, is_open


@njit(cache=True)
def _lay_all(tips, volumes, top, nx, ny, res, nozzle_field, beads):
    """OmPrintedPart::onPhysicsStepEnded for every step: tips[j] is the nozzle tip at the
    end of step j (metres, the part's frame), volumes[j] the extrudedVolume field during
    it (m^3). Returns (beads, count, deposited m^3, crashes, dropped m^3)."""
    nozzle = max(1e-5, nozzle_field)
    face = FACE_RATIO * nozzle
    count = 0
    is_open = False
    have_last = False
    lx = ly = lz = 0.0
    last_volume = 0.0
    pending = 0.0
    last_support = 0.0
    deposited = 0.0
    dropped = 0.0
    crashes = 0
    blob_state = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for s in range(tips.shape[0]):
        tx, ty, tz = tips[s, 0], tips[s, 1], tips[s, 2]
        volume = volumes[s]
        added = volume - last_volume
        last_volume = volume
        if added > 0.0:
            pending += added
        if have_last:
            mx, my, mz = tx - lx, ty - ly, tz - lz
            horizontal = math.hypot(mx, my)
            if math.sqrt(mx * mx + my * my + mz * mz) > TELEPORT_DISTANCE_M:
                dropped += pending
                pending = 0.0
                is_open = False
                blob_state[0] = -1.0
            elif pending <= 0.0:
                is_open = False
            elif horizontal < STILL_RATIO * nozzle:
                if pending >= BLOB_MIN_VOLUME_M3:
                    beads, count, is_open = _blob(beads, count, is_open, top, nx, ny, res, nozzle, last_support,
                                                  blob_state, tx, ty, tz, pending)
                    deposited += pending
                    pending = 0.0
            elif added <= 0.0 and pending < BLOB_MIN_VOLUME_M3:
                dropped += pending
                pending = 0.0
                is_open = False
            else:
                # ---- lay(lastTip, tip, pending) ----
                ax, ay, az = lx, ly, lz
                dx, dy, dz = tx - ax, ty - ay, tz - az
                length = math.sqrt(dx * dx + dy * dy + dz * dz)
                ztop = 0.5 * (az + tz)
                support, crashed = _support_under(top, nx, ny, res, ax, ay, tx, ty, ztop, blob_state, nozzle)
                if support < 0.0:
                    close = ztop - last_support > 0.0 and ztop - last_support < 2.0 * nozzle
                    support = last_support if close else ztop - 0.5 * nozzle
                last_support = support
                if crashed:
                    crashes += 1
                area = pending / length
                min_strand = MIN_STRAND_DIAMETER_RATIO * nozzle
                if area < 0.25 * math.pi * min_strand * min_strand:
                    is_open = False
                    dropped += pending
                else:
                    height = max(MIN_BEAD_HEIGHT_M, ztop - support)
                    rnd = False
                    if height > 2.0 * nozzle:
                        width = math.sqrt(4.0 * area / math.pi)
                        height = width
                        rnd = True
                    else:
                        width = (area - 0.25 * math.pi * height * height) / height + height
                        if width < height:
                            width = math.sqrt(4.0 * area / math.pi)
                            height = width
                            rnd = True
                    laid = pending
                    if width > face:
                        if rnd:
                            width = height = face
                            laid = 0.25 * math.pi * face * face * length
                        else:
                            width = face
                            laid = ((face - height) * height + 0.25 * math.pi * height * height) * length
                    beads, count, is_open, _ = _add_bead(beads, count, is_open, ax, ay, az, tx, ty, tz,
                                                         width, height, True)
                    _raise(top, nx, ny, res, ax, ay, tx, ty, 0.5 * width, ztop)
                    deposited += laid
                    if laid < pending:
                        beads, count, is_open = _blob(beads, count, is_open, top, nx, ny, res, nozzle, last_support,
                                                      blob_state, ax, ay, az, pending - laid)
                        deposited += pending - laid
                pending = 0.0
        if blob_state[0] >= 0.0 and not _near_blob(blob_state, tx, ty, tz, nozzle):
            blob_state[0] = -1.0                       # the nozzle has left it: the next blob is a new one
        lx, ly, lz = tx, ty, tz
        have_last = True
    return beads, count, deposited, crashes, dropped


# ── the trajectory at the step times, in parallel ──────────────────────────────────────

def _sample_span(traj, k0: int, k1: int, dt: float):
    """Toolhead and extruder at steps [k0, k1): the floats StepSampler gives for them
    (every Trajectory sample is pointwise in t, and k * dt is the same float in any span)."""
    ts = np.arange(k0, k1) * dt
    return traj.toolhead(ts), traj.extruder(ts)


def sample_steps(traj: klipper.Trajectory, dt: float, n: int, jobs: int = 1, chunk: int = 65536):
    """toolhead (n + 1, 3) and extruder (n + 1,) at k * dt, k = 0 .. n.

    Spans run in threads: NumPy releases the GIL inside each array operation, and a thread
    pool starts in no time where Windows' spawned processes took ~14 s to import this module
    (MEASURED 2026-11-04, 16 hardware threads: 8 threads sample 3.1 M steps 4x faster than
    one, 16 no faster than 8)."""
    spans = [(k0, min(n + 1, k0 + chunk)) for k0 in range(0, n + 1, chunk)]
    toolhead = np.empty((n + 1, 3))
    extruder = np.empty(n + 1)

    def one(span):
        k0, k1 = span
        toolhead[k0:k1], extruder[k0:k1] = _sample_span(traj, k0, k1, dt)

    workers = max(1, min(jobs, 8, len(spans)))
    if workers > 1:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=workers) as pool:
            list(pool.map(one, spans))
    else:
        for span in spans:
            one(span)
    return toolhead, extruder


# ── the part's map (PrintedPart.rebuild) and the dump (PrintedPart.dumpBeads) ─────────

def part_frame(p: printer_mod.Printer, resolution_m: float = 0.0):
    """The PrintedPart world.py writes for this printer: (size m, resolution m, nozzle m,
    nx, ny), with the values parsed from the world's own text."""
    num = _WORLD._f
    size = [float(num(p.axes["x"].position_max / 1000.0)), float(num(p.axes["y"].position_max / 1000.0)),
            float(num((p.axes["z"].travel + p.axes["z"].position_min) / 1000.0))]
    nozzle = float(num(p.extruder.nozzle_diameter / 1000.0))
    res = max(resolution_m, MIN_RESOLUTION_M) if resolution_m > 0.0 else DEFAULT_RESOLUTION_M
    longest = max(size[0], size[1])
    if longest / res > MAX_CELLS_PER_AXIS:
        res = longest / MAX_CELLS_PER_AXIS
    nx = int(math.ceil(size[0] / res)) + 1
    ny = int(math.ceil(size[1] / res)) + 1
    return size, res, nozzle, nx, ny


def schematic_origin(p: printer_mod.Printer):
    """PrintedPart's translation in world.py's schematic world (machine X0 Y0 on the bed),
    written to the dump header as the engine writes it. inspect_print.py does not use it."""
    bed = p.profile.get("bed", {})
    bx, by = (v / 1000.0 for v in bed.get("size_mm", [p.axes["x"].travel, p.axes["y"].travel]))
    ox, oy = (v / 1000.0 for v in bed.get("origin_mm", [0.0, 0.0]))
    return [-bx / 2 - ox, -by / 2 - oy, _WORLD.BED_TOP_M]


def write_dump(path, beads: np.ndarray, origin: Sequence[float], deposited_m3: float, res_m: float) -> None:
    """PrintedPart's OMBEADS1 layout: magic, int32 count, 5 float64 (origin x/y/z, deposited
    m^3, resolution m), then 9 float32 per bead. Written to <path>.partial, then renamed."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + ".partial")
    body = np.ascontiguousarray(beads, dtype="<f4")
    with partial.open("wb") as f:
        f.write(b"OMBEADS1")
        f.write(struct.pack("<i", len(body)))
        f.write(struct.pack("<5d", origin[0], origin[1], origin[2], deposited_m3, res_m))
        f.write(body.tobytes())
    os.replace(partial, path)


# ── the whole print ────────────────────────────────────────────────────────────────────

def ideal_print(p: printer_mod.Printer, gcode_text: str, s0: Optional[Sequence[float]] = None,
                dt: Optional[float] = None, jobs: int = 1, resolution_m: float = 0.0) -> dict:
    """Plan, sample, extrude and lay a job. Returns the beads (float32, metres, the dump's
    columns), the totals fdm_twin's "done" line reports, and the time each stage took."""
    timings = {}
    t0 = time.perf_counter()
    dt = dt if dt is not None else _WORLD.BASIC_TIME_STEP_MS / 1000.0
    s0 = list(s0) if s0 is not None else [p.axes["x"].position_max / 2, p.axes["y"].position_max / 2, 100.0]
    job = klipper.load_job(p, gcode_text, start=s0)
    traj = klipper.Trajectory(job, p)
    timings["plan_s"] = time.perf_counter() - t0

    # fdm_twin: step k ends at k * dt; the sampler clamps past n; the hotend steps to the
    # extruder at k + 1 until the first k with k * dt > duration + 1 s.
    t = time.perf_counter()
    n = int(traj.duration / dt) + 3
    k_end = max(0, int((job.duration + FINISH_AFTER_S) / dt) - 2)
    while k_end * dt <= job.duration + FINISH_AFTER_S:      # the first k past it, as fdm_twin tests it
        k_end += 1
    steps = k_end + 2                                  # tips and volumes for j = 0 .. k_end + 1
    toolhead, extruder = sample_steps(traj, dt, n, jobs)
    idx = np.minimum(np.arange(steps), n)
    timings["sample_s"] = time.perf_counter() - t

    t = time.perf_counter()
    hot = hotend_mod.Hotend.for_printer(p)
    deposited_mm3, skipped, peak_flow = run_hotend(hot, extruder[idx], dt)
    timings["hotend_s"] = time.perf_counter() - t

    t = time.perf_counter()
    size, res, nozzle, nx, ny = part_frame(p, resolution_m)
    top = np.zeros(nx * ny, dtype=np.float32)
    tips = toolhead[idx] / 1000.0
    volumes = deposited_mm3 * 1e-9                     # fdm_twin: volume_field.setSFFloat(deposited * 1e-9)
    beads0 = np.zeros((max(1024, min(steps, 1 << 20)), 9), dtype=np.float32)
    beads, count, laid_m3, crashes, dropped_m3 = _lay_all(np.ascontiguousarray(tips), np.ascontiguousarray(volumes),
                                                          top, nx, ny, res, nozzle, beads0)
    timings["lay_s"] = time.perf_counter() - t
    timings["total_s"] = time.perf_counter() - t0
    return {
        "beads": beads[:count], "deposited_m3": float(laid_m3), "resolution_m": res, "steps": int(steps),
        "dt": dt, "s0": s0,
        "done": {"planned_s": round(job.duration, 3), "extruded_mm3": round(float(deposited_mm3[-1]), 2),
                 "skipped_mm3": round(skipped, 3), "laid_mm3": round(laid_m3 * 1e9, 2),
                 "dropped_mm3": round(dropped_m3 * 1e9, 3), "nozzle_crashes": int(crashes),
                 "peak_flow_mm3_s": round(peak_flow, 2), "beads": int(count)},
        "errors": [str(e) for e in job.errors], "notes": list(job.notes),
        "timings": {k: round(v, 2) for k, v in timings.items()},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("gcode")
    ap.add_argument("stl", nargs="?", help="the part the job was sliced from (omit to skip the inspection)")
    ap.add_argument("--printer", default=str(printer_mod.DEFAULT_PROFILE))
    ap.add_argument("--s0", help="x,y,z machine mm at power-up (world.py's default: centred, 100 mm up)")
    ap.add_argument("--at", help="x,y where the slicer centred the part (inspect_print.py --at)")
    ap.add_argument("--out", help="the bead dump (default <gcode>.ideal.beads)")
    ap.add_argument("--report", help="the JSON report (default <out>.json)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1, help="processes for sampling and inspection")
    ap.add_argument("--json", action="store_true", help="print the whole report as JSON")
    args = ap.parse_args(argv)

    p = printer_mod.load(args.printer)
    gcode = Path(args.gcode)
    s0 = [float(v) for v in args.s0.split(",")] if args.s0 else None
    wall = time.perf_counter()
    res = ideal_print(p, gcode.read_text(encoding="utf-8", errors="replace"), s0, jobs=args.jobs)
    out = Path(args.out) if args.out else gcode.with_suffix(".ideal.beads")
    write_dump(out, res["beads"], schematic_origin(p), res["deposited_m3"], res["resolution_m"])
    report = {"mode": "ideal", "gcode": str(gcode), "printer": p.name, "dump": str(out), "dt": res["dt"],
              "s0": res["s0"], "steps": res["steps"], "done": res["done"], "errors": res["errors"],
              "notes": res["notes"], "timings": res["timings"]}
    if args.stl:
        t = time.perf_counter()
        bed = p.profile.get("bed", {}).get("size_mm", [p.axes["x"].travel, p.axes["y"].travel])
        at = [float(v) for v in args.at.split(",")] if args.at else None
        report["inspect"] = inspect_print.inspect(inspect_print.load_beads(out), inspect_print.place_drawing(args.stl, bed, at),
                                                  p.extruder.nozzle_diameter, max(1, args.jobs))
        report["timings"]["inspect_s"] = round(time.perf_counter() - t, 2)
    report["timings"]["wall_s"] = round(time.perf_counter() - wall, 2)
    rpath = Path(args.report) if args.report else out.with_name(out.name + ".json")
    rpath.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"[ideal_print] {p.name}: {gcode.name}, {res['steps']} steps of {res['dt'] * 1000:g} ms, "
              f"{res['done']['planned_s']} s planned" + (f", STOPS: {res['errors'][0]}" if res["errors"] else ""))
        print(f"[ideal_print] done: {json.dumps(res['done'])}")
        print(f"[ideal_print] timings: {json.dumps(report['timings'])}")
        for k, v in report.get("inspect", {}).items():
            if k != "layers":
                print(f"{k:>24}: {v}")
        print(f"[ideal_print] wrote {out} and {rpath}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
