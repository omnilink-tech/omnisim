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

"""GRBL 1.1 G-code interpreter: one line in, ordered motion/sync blocks out.

The interpreter keeps GRBL's modal state and its planned position (where the
machine will be once everything queued so far has run). It does not move
anything and does not judge soft limits; the controller (`grbl.py`) and the
offline preflight (`preflight.py`) both consume its blocks, so a job is judged
by exactly the interpretation the twin executes.

Errors carry GRBL 1.1's numeric codes (`error:N` on the wire).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Vec = Tuple[float, float, float]
MM_PER_INCH = 25.4


class GcodeError(Exception):
    def __init__(self, code: int, detail: str = ""):
        super().__init__(f"error:{code} {detail}".strip())
        self.code = code
        self.detail = detail


@dataclass
class Block:
    """One planner-ordered action. `kind` is line | dwell | pause | end | spindle | coolant."""
    kind: str
    target: Optional[Vec] = None        # machine coordinates, mm (line)
    rapid: bool = False
    feed: float = 0.0                   # mm/min (line; 0 for rapid)
    seconds: float = 0.0                # dwell
    spindle: Tuple[str, float] = ("off", 0.0)   # (off|cw|ccw, S) in force for this block
    coolant: str = "off"
    source_line: int = 0                # 1-based line number in the stream/program
    arc: bool = False


@dataclass
class Modal:
    motion: str = "G0"          # G0 G1 G2 G3 G80
    plane: str = "G17"
    distance: str = "G90"
    feed_mode: str = "G94"
    units: str = "G21"
    wcs: int = 0                # 0..5 -> G54..G59
    spindle: str = "off"        # off cw ccw
    coolant: str = "off"        # off flood mist (M7/M8 combine as "mist+flood")


AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}
PLANES = {"G17": (0, 1, 2), "G18": (2, 0, 1), "G19": (1, 2, 0)}
_WORD = re.compile(r"([A-Z])([-+]?(?:\d+\.?\d*|\.\d+))")
_MOTION = {0: "G0", 1: "G1", 2: "G2", 3: "G3", 80: "G80"}
_NONMODAL_G = {4, 10, 28, 30, 53, 92, 28.1, 30.1, 92.1}


def strip_line(raw: str) -> str:
    """Remove comments, whitespace and block delete; uppercase. GRBL ignores `%` lines."""
    out, depth = [], 0
    for ch in raw:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth = max(0, depth - 1)
        elif ch == ";" and not depth:
            break
        elif not depth and not ch.isspace():
            out.append(ch.upper())
    s = "".join(out)
    if s.startswith("/"):
        s = s[1:]
    return "" if s == "%" else s


@dataclass
class Interpreter:
    arc_tolerance: float = 0.002            # $12, mm
    spindle_max: float = 1000.0             # $30
    spindle_min: float = 0.0                # $31
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # planned, machine mm
    modal: Modal = field(default_factory=Modal)
    feed: float = 0.0                        # mm/min (G94) or 1/min (G93), in current units converted
    speed: float = 0.0
    tool: int = 0
    wcs_offsets: List[List[float]] = field(default_factory=lambda: [[0.0] * 3 for _ in range(6)])
    g92: List[float] = field(default_factory=lambda: [0.0] * 3)
    tlo: float = 0.0
    g28: List[float] = field(default_factory=lambda: [0.0] * 3)
    g30: List[float] = field(default_factory=lambda: [0.0] * 3)

    # ── coordinate helpers ────────────────────────────────────────────────
    def work_offset(self) -> List[float]:
        w = self.wcs_offsets[self.modal.wcs]
        return [w[0] + self.g92[0], w[1] + self.g92[1], w[2] + self.g92[2] + self.tlo]

    def work_position(self, mpos: Optional[List[float]] = None) -> List[float]:
        p = self.position if mpos is None else mpos
        o = self.work_offset()
        return [p[i] - o[i] for i in range(3)]

    def parser_state(self) -> str:
        """`$G` report body."""
        m = self.modal
        spindle = {"off": "M5", "cw": "M3", "ccw": "M4"}[m.spindle]
        coolant = {"off": "M9", "flood": "M8", "mist": "M7", "mist+flood": "M7 M8"}[m.coolant]
        feed = self.feed / MM_PER_INCH if m.units == "G20" and m.feed_mode == "G94" else self.feed
        return (f"[GC:{m.motion} G{54 + m.wcs} {m.plane} {m.units} {m.distance} {m.feed_mode} "
                f"{spindle} {coolant} T{self.tool} F{_fmt(feed)} S{_fmt(self.speed)}]")

    def program_end(self) -> None:
        """M2/M30 reset these modal groups (GRBL 1.1 gc_execute_line, program flow)."""
        m = self.modal
        m.motion, m.plane, m.distance, m.feed_mode, m.wcs = "G1", "G17", "G90", "G94", 0
        m.spindle, m.coolant = "off", "off"

    # ── execution ─────────────────────────────────────────────────────────
    def execute(self, raw: str, source_line: int = 0) -> List[Block]:
        """Interpret one line. Returns its blocks; raises GcodeError and leaves state untouched."""
        line = strip_line(raw)
        if not line:
            return []
        words = _parse_words(line)
        snapshot = _snapshot(self)
        try:
            return self._execute_words(words, source_line)
        except GcodeError:
            _restore(self, snapshot)
            raise

    def _execute_words(self, words: List[Tuple[str, float]], src: int) -> List[Block]:
        m = self.modal
        gcodes: List[float] = []
        mcodes: List[int] = []
        values: Dict[str, float] = {}
        for letter, value in words:
            if letter == "G":
                gcodes.append(round(value, 1))
            elif letter == "M":
                if value != int(value):
                    raise GcodeError(20, f"M{value}")
                mcodes.append(int(value))
            elif letter in "XYZIJKFSPRTLN":
                if letter in values:
                    raise GcodeError(25, f"repeated {letter}")
                values[letter] = value
            else:
                raise GcodeError(20, f"unsupported word {letter}")
        values.pop("N", None)

        motion = None
        nonmodal = None
        tlo_cmd = None
        new = Modal(**vars(m))
        groups_seen = set()

        def group(name: str) -> None:
            if name in groups_seen:
                raise GcodeError(21, f"modal group {name} twice")
            groups_seen.add(name)

        for g in gcodes:
            gi = int(g) if g == int(g) else g
            if gi in _MOTION:
                group("motion"); motion = _MOTION[gi]
            elif gi in (17, 18, 19):
                group("plane"); new.plane = f"G{gi}"
            elif gi in (90, 91):
                group("distance"); new.distance = f"G{gi}"
            elif gi == 91.1:
                group("arc_distance")      # IJK incremental is GRBL's only mode
            elif gi in (93, 94):
                group("feed_mode"); new.feed_mode = f"G{gi}"
            elif gi in (20, 21):
                group("units"); new.units = f"G{gi}"
            elif gi in (54, 55, 56, 57, 58, 59):
                group("wcs"); new.wcs = gi - 54
            elif gi in (40, 49, 61, 43.1):
                group({40: "cutter", 49: "tlo", 61: "control", 43.1: "tlo"}[gi])
                if gi in (49, 43.1):
                    tlo_cmd = "set" if gi == 43.1 else "clear"
            elif gi in _NONMODAL_G:
                group("nonmodal"); nonmodal = gi
            elif gi in (38.2, 38.3, 38.4, 38.5):
                raise GcodeError(20, "probing (G38.x) is not modelled by the twin")
            else:
                raise GcodeError(20, f"G{g}")

        for code in mcodes:
            if code in (0, 1, 2, 30):
                group("stopping")
            elif code in (3, 4, 5):
                group("spindle"); new.spindle = {3: "cw", 4: "ccw", 5: "off"}[code]
            elif code in (7, 8):
                kind = "mist" if code == 7 else "flood"
                new.coolant = kind if new.coolant in ("off", kind) else "mist+flood"
            elif code == 9:
                new.coolant = "off"
            elif code == 56:
                pass
            else:
                raise GcodeError(20, f"M{code}")

        scale = MM_PER_INCH if new.units == "G20" else 1.0
        axis_words = {k: values[k] * scale for k in "XYZ" if k in values}
        offsets = {k: values[k] * scale for k in "IJK" if k in values}

        if "F" in values:
            if values["F"] < 0:
                raise GcodeError(4, "negative feed")
            f_value = values["F"] * (scale if new.feed_mode == "G94" else 1.0)
        else:
            f_value = None
        if "S" in values and values["S"] < 0:
            raise GcodeError(4, "negative spindle speed")
        if "T" in values:
            if values["T"] != int(values["T"]) or values["T"] < 0:
                raise GcodeError(23, "T")
            if values["T"] > 255:
                raise GcodeError(38, "tool number")

        # Axis words belong to one consumer: G10/G28/G30/G92/G43.1 or the motion mode.
        axis_consumer = None
        if nonmodal in (10, 28, 30, 92):
            axis_consumer = nonmodal
        elif tlo_cmd == "set":
            axis_consumer = "tlo"
        motion_mode = motion or new.motion
        if motion and axis_consumer in (10, 92, "tlo") and axis_words:
            raise GcodeError(24, "axis words claimed by two commands")
        if nonmodal == 53 and motion_mode not in ("G0", "G1"):
            raise GcodeError(30, "G53 needs G0 or G1")
        if motion == "G80" and axis_words and axis_consumer is None:
            raise GcodeError(31, "axis words with G80")

        # Commit modal state (GRBL applies it before motion in the same block).
        self.modal = new
        if motion:
            self.modal.motion = motion
        if f_value is not None:
            self.feed = f_value
        if "S" in values:
            self.speed = values["S"]
        if "T" in values:
            self.tool = int(values["T"])
        blocks: List[Block] = []
        spindle_now = (self.modal.spindle, self._clamped_speed())

        if (new.spindle != m.spindle) or ("S" in values) or new.coolant != m.coolant:
            blocks.append(Block("spindle", spindle=spindle_now, coolant=new.coolant, source_line=src))

        if nonmodal == 4:
            if "P" not in values:
                raise GcodeError(28, "G4 needs P")
            blocks.append(Block("dwell", seconds=max(0.0, values["P"]), spindle=spindle_now, source_line=src))
        if tlo_cmd == "set":
            if set(axis_words) - {"Z"}:
                raise GcodeError(37, "G43.1 only on Z")
            self.tlo = axis_words.get("Z", 0.0)
        elif tlo_cmd == "clear":
            self.tlo = 0.0

        if nonmodal == 10:
            self._g10(values, axis_words)
        elif nonmodal in (28.1, 30.1):
            (self.g28 if nonmodal == 28.1 else self.g30)[:] = self.position
        elif nonmodal == 92:
            if not axis_words:
                raise GcodeError(26, "G92 needs axis words")
            o = self.wcs_offsets[self.modal.wcs]
            for k, v in axis_words.items():
                i = AXIS_INDEX[k]
                self.g92[i] = self.position[i] - o[i] - v - (self.tlo if i == 2 else 0.0)
        elif nonmodal == 92.1:
            self.g92 = [0.0, 0.0, 0.0]
        elif nonmodal in (28, 30):
            stored = self.g28 if nonmodal == 28 else self.g30
            targets = [i for i in range(3) if "XYZ"[i] in axis_words] or [0, 1, 2]
            if axis_words:
                mid = self._target(axis_words, machine=False)
                blocks.append(self._line(mid, True, src, spindle_now))
            dest = list(self.position)
            for i in targets:
                dest[i] = stored[i]
            blocks.append(self._line(dest, True, src, spindle_now))

        if axis_consumer is None and motion_mode != "G80" and axis_words:
            target = self._target(axis_words, machine=(nonmodal == 53))
            if motion_mode == "G0":
                blocks.append(self._line(target, True, src, spindle_now))
            elif motion_mode == "G1":
                blocks.append(self._line(target, False, src, spindle_now,
                                         self._feed_for(_dist(self.position, target))))
            else:
                blocks.extend(self._arc(target, offsets, values.get("R"), scale, src, spindle_now))
        elif motion_mode in ("G2", "G3") and motion and not axis_words and offsets:
            # A full circle: same target, centre from offsets.
            blocks.extend(self._arc(list(self.position), offsets, None, scale, src, spindle_now))

        for code in mcodes:
            if code in (0, 1):
                blocks.append(Block("pause", spindle=spindle_now, source_line=src))
            elif code in (2, 30):
                self.program_end()
                blocks.append(Block("end", spindle=("off", 0.0), source_line=src))
        return blocks

    # ── pieces ───────────────────────────────────────────────────────────
    def _clamped_speed(self) -> float:
        if self.modal.spindle == "off" or self.speed <= 0:
            return 0.0
        return min(max(self.speed, self.spindle_min), self.spindle_max)

    def _target(self, axis_words: Dict[str, float], machine: bool) -> List[float]:
        t = list(self.position)
        o = self.work_offset()
        for k, v in axis_words.items():
            i = AXIS_INDEX[k]
            if machine:                       # G53: absolute machine coordinates, this block only
                t[i] = v
            else:
                t[i] = self.position[i] + v if self.modal.distance == "G91" else v + o[i]
        return t

    def _feed_for(self, length: float) -> float:
        if self.modal.feed_mode == "G93":
            if not self.feed:
                raise GcodeError(22, "G93 needs F on every move")
            feed = length * self.feed
            self.feed = 0.0        # inverse time F is not modal
            return feed
        if not self.feed:
            raise GcodeError(22, "feed rate undefined")
        return self.feed

    def _line(self, target, rapid, src, spindle, feed=0.0) -> Block:
        if not rapid and feed <= 0:
            feed = self._feed_for(_dist(self.position, target))
        b = Block("line", tuple(target), rapid, 0.0 if rapid else feed, spindle=spindle,
                  coolant=self.modal.coolant, source_line=src)
        self.position = list(target)
        return b

    def _g10(self, values: Dict[str, float], axis_words: Dict[str, float]) -> None:
        if "L" not in values or "P" not in values:
            raise GcodeError(28, "G10 needs L and P")
        L, P = values["L"], values["P"]
        if L not in (2, 20) or P != int(P) or not 0 <= P <= 6:
            raise GcodeError(29 if P > 6 else 20, "G10 L/P")
        idx = self.modal.wcs if P == 0 else int(P) - 1
        if not axis_words:
            raise GcodeError(26, "G10 needs axis words")
        for k, v in axis_words.items():
            i = AXIS_INDEX[k]
            if L == 2:
                self.wcs_offsets[idx][i] = v
            else:
                self.wcs_offsets[idx][i] = self.position[i] - self.g92[i] - v - (self.tlo if i == 2 else 0.0)

    def _arc(self, target, offsets, radius, scale, src, spindle) -> List[Block]:
        a0, a1, lin = PLANES[self.modal.plane]
        names = "XYZ"
        in_plane = {names[a0], names[a1]}
        cw = self.modal.motion == "G2"
        pos = self.position
        if radius is not None:
            r = radius * scale
            x, y = target[a0] - pos[a0], target[a1] - pos[a1]
            if abs(x) < 1e-9 and abs(y) < 1e-9:
                raise GcodeError(33, "radius arc to its own start")
            h = 4.0 * r * r - x * x - y * y
            if h < 0:
                raise GcodeError(34, "arc radius too small for the target")
            h = -math.sqrt(h) / math.hypot(x, y)
            if cw:
                h = -h
            if r < 0:
                h, r = -h, -r
            ci, cj = 0.5 * (x - y * h), 0.5 * (y + x * h)
        else:
            off_names = {0: "I", 1: "J", 2: "K"}
            if not ({off_names[a0], off_names[a1]} & set(offsets)):
                raise GcodeError(35, "arc needs in-plane offsets")
            ci, cj = offsets.get(off_names[a0], 0.0), offsets.get(off_names[a1], 0.0)
            r = math.hypot(ci, cj)
            r_end = math.hypot(target[a0] - pos[a0] - ci, target[a1] - pos[a1] - cj)
            delta = abs(r_end - r)
            if delta > 0.005 and (delta > 0.5 or delta > 0.001 * r):
                raise GcodeError(33, "arc end not on the circle")
        center0, center1 = pos[a0] + ci, pos[a1] + cj
        r0, r1 = -ci, -cj
        rt0, rt1 = target[a0] - center0, target[a1] - center1
        travel = math.atan2(r0 * rt1 - r1 * rt0, r0 * rt0 + r1 * rt1)
        eps = 5e-7
        if cw and travel >= -eps:
            travel -= 2 * math.pi
        elif not cw and travel <= eps:
            travel += 2 * math.pi
        tol = self.arc_tolerance
        chord = math.sqrt(max(tol * (2 * r - tol), 1e-12))
        segments = max(1, int(abs(0.5 * travel * r) / chord))
        lin_travel = target[lin] - pos[lin]
        arc_length = math.hypot(travel * r, lin_travel)
        feed = self._feed_for(arc_length)
        blocks = []
        start = list(pos)
        for n in range(1, segments + 1):
            if n == segments:
                p = list(target)
            else:
                ang = travel * n / segments
                cos_a, sin_a = math.cos(ang), math.sin(ang)
                p = list(start)
                p[a0] = center0 + r0 * cos_a - r1 * sin_a
                p[a1] = center1 + r0 * sin_a + r1 * cos_a
                p[lin] = start[lin] + lin_travel * n / segments
            b = self._line(p, False, src, spindle, feed)
            b.arc = True
            blocks.append(b)
        return blocks


def _parse_words(line: str) -> List[Tuple[str, float]]:
    words, i = [], 0
    while i < len(line):
        match = _WORD.match(line, i)
        if not match:
            ch = line[i]
            raise GcodeError(1 if not ch.isalpha() else 2, f"at {line[i:i + 8]!r}")
        words.append((match.group(1), float(match.group(2))))
        i = match.end()
    return words


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _fmt(v: float) -> str:
    return f"{v:g}" if v == int(v) else f"{v:.3f}".rstrip("0").rstrip(".")


def _snapshot(it: Interpreter):
    return (list(it.position), Modal(**vars(it.modal)), it.feed, it.speed, it.tool,
            [list(w) for w in it.wcs_offsets], list(it.g92), it.tlo, list(it.g28), list(it.g30))


def _restore(it: Interpreter, s) -> None:
    (it.position, it.modal, it.feed, it.speed, it.tool, it.wcs_offsets, it.g92, it.tlo,
     it.g28, it.g30) = s
