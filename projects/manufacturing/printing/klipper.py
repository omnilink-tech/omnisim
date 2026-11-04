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

"""A whole print job planned the way Klipper plans it, as a trajectory you can sample.

The G-code layer is Klipper's gcode_move (G0/G1/G2/G3, G4, G28, G90/G91, M82/M83, G92,
M204, M220/M221, M104/M109/M140/M190, M106/M107, M400, M84, SET_VELOCITY_LIMIT,
SET_PRESSURE_ADVANCE) plus the printer's own macros from its profile. The motion layer
follows klippy/toolhead.py and klippy/kinematics/extruder.py: square-corner-velocity
junctions, the extruder's instantaneous corner velocity, max_accel_to_decel (or
minimum_cruise_ratio) smoothing in the look-ahead, Z and extrude-only limits, and the
errors Klipper stops a print on ("Move out of range", "Extrude below minimum temp",
"Extrude only move too long", "Move exceeds maximum extrusion").

On top of the planned toolhead motion, `Trajectory` applies what Klipper does at step
generation: the input shaper on X and Y, and pressure advance with its smoothing on the
extruder. A print file is planned in one pass (Klipper's look-ahead flushes lazily, so
on a file it reaches the same junctions).

Not modelled: heater warm-up time (a wait for temperature takes no time), probing
(QUAD_GANTRY_LEVEL and BED_MESH_CALIBRATE are no-ops; the bed is flat and level), step
timing and stepper resonance. Each is reported in `Job.notes` when the job uses it.
"""
from __future__ import annotations

import bisect
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from printer import AXES, Printer

FILAMENT_DENSITY_G_CM3 = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "TPU": 1.21}
ARC_RESOLUTION_MM = 1.0   # Klipper [gcode_arcs] resolution default (the SV08 sets 1.0)


class KlipperError(Exception):
    def __init__(self, message: str, line: int):
        super().__init__(f"line {line}: {message}")
        self.message, self.line = message, line


@dataclass
class Move:
    start: List[float]
    end: List[float]
    move_d: float
    axes_r: List[float]
    accel: float
    junction_deviation: float
    max_cruise_v2: float
    delta_v2: float
    smooth_delta_v2: float
    is_kinematic: bool
    line: int
    feature: str
    max_start_v2: float = 0.0
    max_smoothed_v2: float = 0.0
    start_v: float = 0.0
    cruise_v: float = 0.0
    end_v: float = 0.0
    accel_t: float = 0.0
    cruise_t: float = 0.0
    decel_t: float = 0.0
    t0: float = 0.0
    pa: float = 0.0           # pressure advance in force for this move (0 unless extruding forward)

    def limit_speed(self, speed: float, accel: float) -> None:
        speed2 = speed * speed
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def set_junction(self, start_v2: float, cruise_v2: float, end_v2: float) -> None:
        half_inv_accel = 0.5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        self.start_v, self.cruise_v, self.end_v = math.sqrt(start_v2), math.sqrt(cruise_v2), math.sqrt(end_v2)
        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5) if accel_d > 0 else 0.0
        self.cruise_t = cruise_d / self.cruise_v if self.cruise_v > 0 else 0.0
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5) if decel_d > 0 else 0.0

    @property
    def duration(self) -> float:
        return self.accel_t + self.cruise_t + self.decel_t


@dataclass
class Event:
    t: float
    kind: str        # "temp", "fan", "layer", "feature", "dwell", "home", "note"
    value: object
    line: int


@dataclass
class Job:
    moves: List[Move]
    events: List[Event]
    duration: float
    errors: List[KlipperError]
    notes: List[str]
    unknown: Dict[str, int]
    end_position: List[float]


# ── the G-code layer ────────────────────────────────────────────────────────────────

_WORD = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def _strip(raw: str) -> Tuple[str, str]:
    """(command text, comment)."""
    code, _, comment = raw.partition(";")
    return code.strip(), comment.strip()


class Planner:
    """Klipper's toolhead + extruder + gcode_move state, consuming G-code lines."""

    def __init__(self, printer: Printer, start: Optional[Sequence[float]] = None,
                 heaters_hot: bool = False):
        self.p = printer
        self.position = list(start) + [0.0] if start is not None else \
            [printer.axes[a].position_min + 0.5 * printer.axes[a].travel for a in AXES] + [0.0]
        self.homed = set()
        self.max_velocity = printer.max_velocity
        self.max_accel = printer.max_accel
        self.requested_accel_to_decel = printer.max_accel_to_decel
        self.min_cruise_ratio = printer.minimum_cruise_ratio
        self.scv = printer.square_corner_velocity
        self._calc_limits()
        self.pressure_advance = printer.extruder.pressure_advance
        self.absolute_coord = True
        self.absolute_extrude = True
        self.base = [0.0, 0.0, 0.0, 0.0]          # G92 offsets
        self.last_gcode = [0.0, 0.0, 0.0, 0.0]    # in G-code coordinates
        self.speed = 25.0                          # mm/s (Klipper's default F1500)
        self.speed_factor = 1.0
        self.extrude_factor = 1.0
        self.temp_target = {"extruder": 0.0, "heater_bed": 0.0}
        self.temp_now = {"extruder": 220.0 if heaters_hot else 25.0, "heater_bed": 25.0}
        self.moves: List[Move] = []
        self.events: List[Event] = []
        self.errors: List[KlipperError] = []
        self.notes: List[str] = []
        self.unknown: Dict[str, int] = {}
        self.feature = ""
        self.pending_time = 0.0            # dwell time to insert before the next move
        self.layer = None
        self._sync_points: List[Tuple[int, float]] = []   # (index of next move, seconds)
        self.line = 0
        self._macro_depth = 0
        self._sync_last_gcode()

    # Klipper's toolhead limits (toolhead.py _calc_junction_deviation and the accel_to_decel rule).
    def _calc_limits(self) -> None:
        self.junction_deviation = self.scv * self.scv * (math.sqrt(2.0) - 1.0) / self.max_accel
        if self.min_cruise_ratio is not None:
            self.accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio)
        elif self.requested_accel_to_decel is not None:
            self.accel_to_decel = min(self.requested_accel_to_decel, self.max_accel)
        else:
            self.accel_to_decel = self.max_accel * 0.5      # Klipper's default minimum_cruise_ratio 0.5

    def _sync_last_gcode(self) -> None:
        self.last_gcode = [self.position[i] - self.base[i] for i in range(4)]

    # ── public ──
    def run(self, lines: Sequence[str]) -> Job:
        for n, raw in enumerate(lines, start=1):
            self.line = n
            try:
                self.execute(raw)
            except KlipperError as exc:
                self.errors.append(exc)
                self.notes.append(f"Klipper would stop the print at line {exc.line}: {exc.message}")
                break
        return self.finish()

    def finish(self) -> Job:
        plan(self.moves, self._sync_points)
        t = 0.0
        sync = dict()
        for idx, secs in self._sync_points:
            sync[idx] = sync.get(idx, 0.0) + secs
        for i, m in enumerate(self.moves):
            t += sync.get(i, 0.0)
            m.t0 = t
            t += m.duration
        end = t + sync.get(len(self.moves), 0.0)
        # An event carries the index of the move it precedes; it happens before that
        # move's dwell, if it has one.
        for ev in self.events:
            idx = ev.t
            ev.t = (self.moves[idx].t0 if idx < len(self.moves) else end) - sync.get(idx, 0.0)
        t = end
        return Job(self.moves, self.events, t, self.errors, sorted(set(self.notes)), self.unknown,
                   list(self.position))

    def execute(self, raw: str) -> None:
        code, comment = _strip(raw)
        if comment.startswith("LAYER:"):
            self.layer = int(comment[6:]) if comment[6:].lstrip("-").isdigit() else comment[6:]
            self._event("layer", self.layer)
        elif comment.startswith("TYPE:"):
            self.feature = comment[5:]
            self._event("feature", self.feature)
        if not code:
            return
        head = code.split()[0].upper()
        if re.fullmatch(r"[GM]\d+(\.\d+)?", head):
            words = {k.upper(): float(v) for k, v in _WORD.findall(code[len(head):])}
            handler = getattr(self, "_" + head.replace(".", "_"), None)
            if handler is None:
                self.unknown[head] = self.unknown.get(head, 0) + 1
                return
            handler(words, code)
        else:
            self._extended(head, code)

    # ── extended (Klipper-style) commands and macros ──
    def _extended(self, head: str, code: str) -> None:
        params = dict((k.upper(), v) for k, v in re.findall(r"(\w+)=(\S+)", code))
        if head == "SET_VELOCITY_LIMIT":
            if "VELOCITY" in params:
                self.max_velocity = float(params["VELOCITY"])
            if "ACCEL" in params:
                self.max_accel = float(params["ACCEL"])
            if "ACCEL_TO_DECEL" in params:
                self.requested_accel_to_decel = float(params["ACCEL_TO_DECEL"])
            if "MINIMUM_CRUISE_RATIO" in params:
                self.min_cruise_ratio = float(params["MINIMUM_CRUISE_RATIO"])
            if "SQUARE_CORNER_VELOCITY" in params:
                self.scv = float(params["SQUARE_CORNER_VELOCITY"])
            self._calc_limits()
            return
        if head == "SET_PRESSURE_ADVANCE":
            if "ADVANCE" in params:
                self.pressure_advance = float(params["ADVANCE"])
            return
        macro = self.p.profile.get("macros", {}).get(head)
        if macro is None:
            self.unknown[head] = self.unknown.get(head, 0) + 1
            return
        if macro.get("note"):
            self.notes.append(f"{head}: {macro['note']}")
        if self._macro_depth > 4:
            return
        self._macro_depth += 1
        try:
            for sub in macro.get("gcode", []):
                if sub.strip().upper() == "G28 IF_NOT_HOMED":
                    if self.homed != set(AXES):
                        self.execute("G28")
                    continue
                self.execute(sub)
        finally:
            self._macro_depth -= 1

    # ── G-codes ──
    def _G0(self, w, code):
        self._G1(w, code)

    def _G1(self, w, code):
        target = list(self.last_gcode)
        for i, a in enumerate("XYZ"):
            if a in w:
                target[i] = w[a] if self.absolute_coord else target[i] + w[a]
        if "E" in w:
            e = w["E"] * self.extrude_factor
            target[3] = e if self.absolute_extrude else target[3] + e
        if "F" in w:
            if w["F"] <= 0:
                raise KlipperError(f"Invalid speed in '{code}'", self.line)
            self.speed = w["F"] / 60.0
        self.last_gcode = target
        end = [target[i] + self.base[i] for i in range(4)]
        self._move(end, self.speed * self.speed_factor)

    def _G2(self, w, code, clockwise=True):
        """Arcs, as Klipper's gcode_arcs: chords of `resolution` mm (1.0 on most configs)."""
        start = list(self.last_gcode)
        end = list(start)
        for i, a in enumerate("XYZ"):
            if a in w:
                end[i] = w[a] if self.absolute_coord else end[i] + w[a]
        if "E" in w:
            e = w["E"] * self.extrude_factor
            end[3] = e if self.absolute_extrude else start[3] + e
        if "R" in w:
            raise KlipperError("G2/G3 does not support R moves", self.line)
        if "F" in w:
            self.speed = w["F"] / 60.0
        cx, cy = start[0] + w.get("I", 0.0), start[1] + w.get("J", 0.0)
        r = math.hypot(start[0] - cx, start[1] - cy)
        a0 = math.atan2(start[1] - cy, start[0] - cx)
        sweep = math.atan2(end[1] - cy, end[0] - cx) - a0
        if clockwise and sweep >= 0:
            sweep -= 2 * math.pi
        elif not clockwise and sweep <= 0:
            sweep += 2 * math.pi
        segments = max(1, int(math.floor(abs(sweep) * r / ARC_RESOLUTION_MM)))
        for k in range(1, segments + 1):
            f = k / segments
            ang = a0 + sweep * f
            point = [cx + r * math.cos(ang), cy + r * math.sin(ang),
                     start[2] + (end[2] - start[2]) * f, start[3] + (end[3] - start[3]) * f]
            if k == segments:
                point = list(end)
            self.last_gcode = point
            self._move([point[i] + self.base[i] for i in range(4)], self.speed * self.speed_factor)

    def _G3(self, w, code):
        self._G2(w, code, clockwise=False)

    def _G4(self, w, code):
        self._sync_points.append((len(self.moves), w.get("P", 0.0) / 1000.0 if "P" in w else w.get("S", 0.0)))
        self._event("dwell", w.get("P", 0.0) / 1000.0 if "P" in w else w.get("S", 0.0))

    def _G28(self, w, code):
        axes = [a for a in "XYZ" if a in code.upper()[3:]] or []
        homing = self.p.profile.get("homing", {})
        seq_key = "all" if not axes else None
        if self._macro_depth > 0 and axes:
            # Inside the homing sequence itself: the real homing move to the endstop.
            for a in axes:
                self._home_axis(a.lower())
            return
        # [homing_override] set_position_z: Klipper marks Z homed (at an assumed height) so
        # the override's first lift is allowed. The twin keeps the true height instead.
        if "set_position_z" in homing:
            self.homed.add("z")
        self._macro_depth += 1
        try:
            if seq_key:
                lines = homing.get("all")
                if lines is None:
                    for a in "xyz":
                        self._home_axis(a)
                else:
                    for sub in lines:
                        self.execute(sub)
            else:
                for a in axes:
                    for sub in homing.get(a.lower(), [f"G28 {a}"]):
                        self.execute(sub)
        finally:
            self._macro_depth -= 1
        self._event("home", "".join(axes) or "XYZ")

    def _home_axis(self, a: str) -> None:
        ax = self.p.axes[a]
        i = AXES.index(a)
        if a == "z" and ax.position_endstop == ax.position_min and "z_probe_speed" in self.p.profile.get("homing", {}):
            target, speed = 0.0, float(self.p.profile["homing"]["z_probe_speed"])   # probe: nozzle at the bed
        else:
            target, speed = (ax.position_max if ax.homing_positive_dir else ax.position_min), ax.homing_speed
            target = ax.position_endstop
        end = list(self.position)
        end[i] = target
        self.homed.discard(a)          # the move to the endstop is not range-checked
        self._move(end, speed, check_range=False)
        self.homed.add(a)

    def _G90(self, w, code):
        self.absolute_coord = True
        self.absolute_extrude = True

    def _G91(self, w, code):
        self.absolute_coord = False
        self.absolute_extrude = False

    def _G92(self, w, code):
        if not w:
            w = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
        for i, a in enumerate("XYZE"):
            if a in w:
                self.base[i] = self.position[i] - w[a]
        self._sync_last_gcode()

    def _M82(self, w, code):
        self.absolute_extrude = True

    def _M83(self, w, code):
        self.absolute_extrude = False

    def _M84(self, w, code):
        self.homed.clear()

    _M18 = _M84

    def _M204(self, w, code):
        if "S" in w:
            accel = w["S"]
        elif "P" in w and "T" in w:
            accel = min(w["P"], w["T"])
        else:
            return
        self.max_accel = accel
        self._calc_limits()

    def _M220(self, w, code):
        self.speed_factor = w.get("S", 100.0) / 100.0

    def _M221(self, w, code):
        self.extrude_factor = w.get("S", 100.0) / 100.0

    def _M104(self, w, code):
        self.temp_target["extruder"] = w.get("S", 0.0)
        self._event("temp", ("extruder", self.temp_target["extruder"]))

    def _M109(self, w, code):
        self._M104(w, code)
        self.temp_now["extruder"] = self.temp_target["extruder"]
        if self.temp_target["extruder"] > 0:
            self.notes.append("M109/M190 waits for temperature: heater warm-up is not modelled (no time)")

    def _M140(self, w, code):
        self.temp_target["heater_bed"] = w.get("S", 0.0)
        self._event("temp", ("heater_bed", self.temp_target["heater_bed"]))

    def _M190(self, w, code):
        self._M140(w, code)
        self.temp_now["heater_bed"] = self.temp_target["heater_bed"]

    def _M106(self, w, code):
        self._event("fan", min(1.0, w.get("S", 255.0) / 255.0))

    def _M107(self, w, code):
        self._event("fan", 0.0)

    def _M105(self, w, code):
        pass

    def _M400(self, w, code):
        pass

    def _M117(self, w, code):
        pass

    def _event(self, kind: str, value) -> None:
        self.events.append(Event(len(self.moves), kind, value, self.line))     # index; a time in finish()

    # ── toolhead.move ──
    def _move(self, end: List[float], speed: float, check_range: bool = True) -> None:
        start = self.position
        axes_d = [end[i] - start[i] for i in range(4)]
        move_d = math.sqrt(sum(d * d for d in axes_d[:3]))
        velocity = min(speed, self.max_velocity)
        accel = self.max_accel
        is_kin = True
        if move_d < 1e-9:
            end = start[:3] + [end[3]]
            axes_d[:3] = [0.0, 0.0, 0.0]
            move_d = abs(axes_d[3])
            if move_d == 0.0:
                return
            accel = 99999999.9
            velocity = speed
            is_kin = False
        inv = 1.0 / move_d
        m = Move(list(start), list(end), move_d, [d * inv for d in axes_d], accel, self.junction_deviation,
                 velocity * velocity, 2.0 * move_d * accel, 2.0 * move_d * self.accel_to_decel, is_kin,
                 self.line, self.feature)
        if is_kin:
            self._check_kinematics(m, check_range)
        if axes_d[3]:
            self._check_extruder(m)
        if m.is_kinematic and m.axes_r[3] > 0.0 and (axes_d[0] or axes_d[1]):
            m.pa = self.pressure_advance
        self.moves.append(m)
        self.position = list(end)

    def _check_kinematics(self, m: Move, check_range: bool) -> None:
        if check_range:
            for i, a in enumerate(AXES):
                if m.end[i] == m.start[i]:
                    continue
                ax = self.p.axes[a]
                if a not in self.homed:
                    raise KlipperError("Must home axis first: %.3f %.3f %.3f [%.3f]" % tuple(m.end), self.line)
                if not (ax.position_min - 1e-9 <= m.end[i] <= ax.position_max + 1e-9):
                    raise KlipperError("Move out of range: %.3f %.3f %.3f [%.3f]" % tuple(m.end), self.line)
        if m.axes_r[2] != 0.0:
            z_ratio = m.move_d / abs(m.end[2] - m.start[2])
            m.limit_speed(self.p.max_z_velocity * z_ratio, self.p.max_z_accel * z_ratio)

    def _check_extruder(self, m: Move) -> None:
        ex = self.p.extruder
        axis_r = m.axes_r[3]
        if self.temp_now["extruder"] < ex.min_extrude_temp:
            raise KlipperError("Extrude below minimum temp", self.line)
        e_d = m.end[3] - m.start[3]
        if (not m.is_kinematic or (m.end[0] == m.start[0] and m.end[1] == m.start[1])) or axis_r < 0.0:
            if abs(e_d) > ex.max_extrude_only_distance:
                raise KlipperError("Extrude only move too long (%.3fmm vs %.3fmm)"
                                   % (abs(e_d), ex.max_extrude_only_distance), self.line)
            inv_r = 1.0 / abs(axis_r)
            m.limit_speed(ex.max_e_velocity * inv_r, ex.max_e_accel * inv_r)
        elif axis_r > ex.max_cross_section / ex.filament_area:
            if e_d <= ex.nozzle_diameter * ex.max_cross_section / ex.filament_area:
                return     # Klipper permits a small over-extrusion (a "priming" move)
            area = axis_r * ex.filament_area
            raise KlipperError("Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)"
                               % (area, ex.max_cross_section), self.line)


# ── the look-ahead (toolhead.py LookAheadQueue.flush, lazy=False) ───────────────────

def _calc_junction(prev: Move, m: Move, instant_corner_v: float) -> None:
    if not m.is_kinematic or not prev.is_kinematic:
        return
    diff_r = m.axes_r[3] - prev.axes_r[3]
    extruder_v2 = (instant_corner_v / abs(diff_r)) ** 2 if diff_r else m.max_cruise_v2
    max_start_v2 = min(extruder_v2, m.max_cruise_v2, prev.max_cruise_v2, prev.max_start_v2 + prev.delta_v2)
    cos_theta = -(m.axes_r[0] * prev.axes_r[0] + m.axes_r[1] * prev.axes_r[1] + m.axes_r[2] * prev.axes_r[2])
    sin_d2 = math.sqrt(max(0.5 * (1.0 - cos_theta), 0.0))
    cos_d2 = math.sqrt(max(0.5 * (1.0 + cos_theta), 0.0))
    one_minus_sin_d2 = 1.0 - sin_d2
    if one_minus_sin_d2 > 0.0 and cos_d2 > 0.0:
        r_jd = sin_d2 / one_minus_sin_d2
        quarter_tan = 0.25 * sin_d2 / cos_d2
        max_start_v2 = min(max_start_v2, r_jd * m.junction_deviation * m.accel,
                           r_jd * prev.junction_deviation * prev.accel,
                           m.delta_v2 * quarter_tan, prev.delta_v2 * quarter_tan)
    m.max_start_v2 = max_start_v2
    m.max_smoothed_v2 = min(max_start_v2, prev.max_smoothed_v2 + prev.smooth_delta_v2)


def plan(moves: List[Move], sync_points: Sequence[Tuple[int, float]] = (), instant_corner_v: float = 1.0) -> None:
    """Junctions and trapezoids for every move. A sync point (dwell, M400 in a macro)
    is a full stop: the queue is flushed there, as Klipper's toolhead does on a dwell."""
    breaks = sorted({idx for idx, _ in sync_points} | {len(moves)})
    lo = 0
    for hi in breaks:
        chunk = moves[lo:hi]
        for i in range(1, len(chunk)):
            _calc_junction(chunk[i - 1], chunk[i], instant_corner_v)
        _flush(chunk)
        lo = hi


def _flush(queue: List[Move]) -> None:
    delayed: List[Tuple[Move, float, float]] = []
    next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.0
    for i in range(len(queue) - 1, -1, -1):
        move = queue[i]
        reachable_start_v2 = next_end_v2 + move.delta_v2
        start_v2 = min(move.max_start_v2, reachable_start_v2)
        reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
        smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
        if smoothed_v2 < reachable_smoothed_v2:
            if smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2 or delayed:
                peak_cruise_v2 = min(move.max_cruise_v2, (smoothed_v2 + reachable_smoothed_v2) * 0.5)
                if delayed:
                    mc_v2 = peak_cruise_v2
                    for m, ms_v2, me_v2 in reversed(delayed):
                        mc_v2 = min(mc_v2, ms_v2)
                        m.set_junction(min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2))
                    del delayed[:]
            cruise_v2 = min((start_v2 + reachable_start_v2) * 0.5, move.max_cruise_v2, peak_cruise_v2)
            move.set_junction(min(start_v2, cruise_v2), cruise_v2, min(next_end_v2, cruise_v2))
        else:
            delayed.append((move, start_v2, next_end_v2))
        next_end_v2 = start_v2
        next_smoothed_v2 = smoothed_v2


# ── step generation: shaper and pressure advance ────────────────────────────────────

def shaper_impulses(kind: str, freq: float, damping: float) -> Tuple[np.ndarray, np.ndarray]:
    """Klipper's shaper_defs.py: amplitudes (normalised) and times, centred on zero delay."""
    if not kind or freq <= 0:
        return np.array([1.0]), np.array([0.0])
    df = math.sqrt(1.0 - damping * damping)
    t_d = 1.0 / (freq * df)
    if kind == "zv":
        k = math.exp(-damping * math.pi / df)
        a, t = [1.0, k], [0.0, 0.5 * t_d]
    elif kind == "mzv":
        k = math.exp(-0.75 * damping * math.pi / df)
        a1 = 1.0 - 1.0 / math.sqrt(2.0)
        a, t = [a1, (math.sqrt(2.0) - 1.0) * k, a1 * k * k], [0.0, 0.375 * t_d, 0.75 * t_d]
    elif kind == "zvd":
        k = math.exp(-damping * math.pi / df)
        a, t = [1.0, 2.0 * k, k * k], [0.0, 0.5 * t_d, t_d]
    elif kind == "ei":
        v_tol = 0.05
        k = math.exp(-damping * math.pi / df)
        a1 = 0.25 * (1.0 + v_tol)
        a, t = [a1, 0.5 * (1.0 - v_tol) * k, a1 * k * k], [0.0, 0.5 * t_d, t_d]
    else:
        raise ValueError(f"input shaper '{kind}' is not modelled (zv, mzv, zvd, ei are)")
    a = np.array(a) / sum(a)
    t = np.array(t)
    return a, t - float(np.dot(a, t))


class Trajectory:
    """Sample the job at any times: toolhead (shaped), extruder command (with pressure advance)."""

    def __init__(self, job: Job, printer: Printer, shaper: bool = True):
        self.job = job
        ms = job.moves
        self.t0 = np.array([m.t0 for m in ms])
        self.start = np.array([m.start for m in ms]) if ms else np.zeros((0, 4))
        self.r = np.array([m.axes_r for m in ms]) if ms else np.zeros((0, 4))
        self.v0 = np.array([m.start_v for m in ms])
        self.vc = np.array([m.cruise_v for m in ms])
        self.acc = np.array([m.accel for m in ms])
        self.ta = np.array([m.accel_t for m in ms])
        self.tc = np.array([m.cruise_t for m in ms])
        self.td = np.array([m.decel_t for m in ms])
        self.move_d = np.array([m.move_d for m in ms])
        self.pa = np.array([m.pa for m in ms])
        self.end = np.array([m.end for m in ms]) if ms else np.zeros((0, 4))
        self.duration = job.duration
        self.smooth_t = printer.extruder.pressure_advance_smooth_time
        self.shapers = {a: shaper_impulses(printer.shapers[a].kind, printer.shapers[a].freq, printer.shapers[a].damping)
                        if shaper else (np.array([1.0]), np.array([0.0])) for a in ("x", "y")}

    def _index(self, t: np.ndarray) -> np.ndarray:
        return np.clip(np.searchsorted(self.t0, t, side="right") - 1, 0, max(0, len(self.t0) - 1))

    def _distance(self, i: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Distance along move i and speed at time t (clamped to the move)."""
        tau = np.clip(t - self.t0[i], 0.0, self.ta[i] + self.tc[i] + self.td[i])
        ta, tc, a, v0, vc = self.ta[i], self.tc[i], self.acc[i], self.v0[i], self.vc[i]
        t1 = np.minimum(tau, ta)
        d = v0 * t1 + 0.5 * a * t1 * t1
        v = v0 + a * t1
        t2 = np.clip(tau - ta, 0.0, tc)
        d += vc * t2
        t3 = np.clip(tau - ta - tc, 0.0, None)
        d += vc * t3 - 0.5 * a * t3 * t3
        v = np.where(tau > ta, vc - a * t3, v)
        return np.minimum(d, self.move_d[i]), v

    def raw(self, t) -> np.ndarray:
        """Planned toolhead and extruder position (mm), shape (n, 4)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        if not len(self.t0):
            return np.zeros((len(t), 4))
        i = self._index(t)
        d, _ = self._distance(i, t)
        return self.start[i] + self.r[i] * d[:, None]

    def velocity(self, t) -> np.ndarray:
        t = np.atleast_1d(np.asarray(t, dtype=float))
        i = self._index(t)
        _, v = self._distance(i, t)
        inside = (t >= self.t0[i]) & (t <= self.t0[i] + self.ta[i] + self.tc[i] + self.td[i])
        return self.r[i] * np.where(inside, v, 0.0)[:, None]

    def toolhead(self, t) -> np.ndarray:
        """Where the steppers put the nozzle: X and Y through the input shaper. Shape (n, 3)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        out = self.raw(t)[:, :3].copy()
        for k, a in ((0, "x"), (1, "y")):
            amps, times = self.shapers[a]
            if len(amps) > 1:
                out[:, k] = sum(amp * self.raw(t - dt)[:, k] for amp, dt in zip(amps, times))
        return out

    def extruder(self, t) -> np.ndarray:
        """The extruder stepper's position: e + pa * de/dt, averaged over the smooth time
        with Klipper's triangular weighting (kin_extruder.c)."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        hst = 0.5 * self.smooth_t
        if hst <= 0:
            return self._e_pa(t)
        offs = np.linspace(-hst, hst, 9)
        w = hst - np.abs(offs)
        w = w / w.sum()
        return sum(wi * self._e_pa(t + o) for wi, o in zip(w, offs))

    def _e_pa(self, t: np.ndarray) -> np.ndarray:
        i = self._index(t)
        d, v = self._distance(i, t)
        e = self.start[i, 3] + self.r[i, 3] * d
        inside = (t >= self.t0[i]) & (t <= self.t0[i] + self.ta[i] + self.tc[i] + self.td[i])
        return e + self.pa[i] * self.r[i, 3] * np.where(inside, v, 0.0)


class StepSampler:
    """The trajectory at the physics step times k * dt, k = 0 .. n, sampled a window at a time.

    The twin's controller used to sample the whole job up front: (n + 1) x 3 toolhead and
    n + 1 extruder values, plus the temporaries Trajectory builds for every sample. For a
    3.3 h print at 2 ms that is 5.9 M steps and 1.55 GB at the peak (MEASURED 2026-11-03,
    a 145 g PETG shell section); it grows with the print's length. Every Trajectory sample is
    pointwise in t, so a window gives exactly the values the full sampling gave, at a
    memory that does not depend on the print's length. Step indices past n read step n.
    """

    def __init__(self, traj: "Trajectory", dt: float, window: int = 8192):
        self.traj = traj
        self.dt = dt
        self.n = int(traj.duration / dt) + 3
        self.window = max(16, int(window))
        self._k0 = 0
        self._k1 = 0                       # [k0, k1) are cached
        self._toolhead = np.zeros((0, 3))
        self._extruder = np.zeros(0)

    def _fill(self, k: int) -> None:
        k0 = max(0, k - 4)                 # a little behind: the controller reads k - 1 .. k + 2
        k1 = min(self.n + 1, k0 + self.window)
        ts = np.arange(k0, k1) * self.dt   # the same floats as np.arange(n + 1) * dt
        self._toolhead = self.traj.toolhead(ts)
        self._extruder = self.traj.extruder(ts)
        self._k0, self._k1 = k0, k1

    def _at(self, k: int) -> int:
        k = min(max(int(k), 0), self.n)
        if not (self._k0 <= k < self._k1):
            self._fill(k)
        return k - self._k0

    def toolhead(self, k: int) -> List[float]:
        i = self._at(k)                    # first: it may refill the window
        return [float(v) for v in self._toolhead[i]]

    def extruder(self, k: int) -> float:
        i = self._at(k)
        return float(self._extruder[i])


def load_job(printer: Printer, gcode_text: str, start: Optional[Sequence[float]] = None,
             heaters_hot: bool = False) -> Job:
    return Planner(printer, start, heaters_hot).run(gcode_text.splitlines())
