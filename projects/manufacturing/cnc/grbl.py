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

"""A virtual GRBL 1.1 controller backed by the twin's rails.

Bytes go in through `feed()` (any thread); `tick(dt)` runs the controller on
the simulator's clock and writes GRBL's replies through `out`. A sender that
talks GRBL over a stream (UGS, CNCjs, bCNC, FluidNCSender, an MCP server, an
agent) can point at this instead of a real board.

Emulated: line protocol with `ok` / `error:N`, real-time characters (`?`, `!`,
`~`, Ctrl-X, jog cancel, feed/rapid/spindle overrides), status reports, `$$`,
`$x=val`, `$#`, `$G`, `$I`, `$N`, `$C`, `$X`, `$H`, `$J=`, `$SLP`, `$RST=`,
soft limits (ALARM:2, jog error:15), hard limits (ALARM:1), reset during
motion (ALARM:3), homing with a search limit (ALARM:9), the boot alarm when
homing is enabled, M0 pause and M2/M30 program end. Commands that sync the
planner (dwell, spindle, M0, M2) reply `ok` only once the buffer reaches them.

Not emulated: the 128-byte RX buffer limit (input is never dropped, so a
character-counting sender is safe but not stress-tested), probing, the safety
door, parking, coolant/spindle pins, FluidNC's own `$` commands. `$TWIN` is an
extension: it prints the twin's physical state as `[TWIN:{json}]`.
"""
from __future__ import annotations

import json
import re
import threading
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Sequence

from gcode import Block, GcodeError, Interpreter, MM_PER_INCH, strip_line
from machine import AXES, GRBL_DEFAULTS, INTEGER_SETTINGS, Machine
from motion import Planner
from tool_table import Tool, default_tools
from twin import Plant

VERSION = "1.1h"
BUILD = "20190825"
REALTIME = {ord("?"), ord("!"), ord("~"), 0x18, 0x84, 0x85, 0x9E, 0xA0, 0xA1} | set(range(0x90, 0x9E))

ALARM_TEXT = {
    1: "Hard limit triggered. Position lost.",
    2: "Soft limit: motion target exceeds machine travel.",
    3: "Reset while in motion. Position lost.",
    9: "Homing fail: limit switch not found within search distance.",
}


class GrblController:
    def __init__(self, machine: Machine, plant: Optional[Plant] = None,
                 out: Callable[[str], None] = lambda s: None, banner: Optional[str] = None):
        self.machine = machine.copy()
        self.defaults = machine.copy()
        self.plant = plant or Plant.build(self.machine)
        self.plant.machine = self.machine
        self.out = out
        self.banner = banner or f"Grbl {VERSION} ['$' for help]"
        self.planner = Planner(self.machine)
        self.planner.on_sync = self._on_sync
        self.interp = Interpreter()
        self._sync_interp_settings()
        self.time = 0.0
        self.state = "Idle"
        self.homed = False
        self.check_mode = False
        self.critical = None            # alarm code awaiting a reset
        self.pending_alarm = None       # soft-limit alarm raised once motion has stopped
        self.jog_cancel = False
        self.spindle = ("off", 0.0)
        self.spindle_override = 1.0
        self.startup_lines = ["", ""]
        self.events: List[dict] = []
        self.stock = None               # a stock.Stock the host carves, reported by $TWIN
        self.tools = default_tools()    # the tool table ($TWIN TOOL=n loads one)
        self.tool = self.tools[1]       # the tool physically in the spindle
        self.on_tool_change: Callable[[Tool], None] = lambda tool: None   # the host swaps the cutter
        self.cutting: Optional[dict] = None  # cutting load the host measures (cutting.LoadMeter)
        self._rx: List[tuple] = []                 # (client, bytes) in arrival order
        self._partials: Dict[object, bytearray] = {}
        # The client whose line is being answered: its `ok`/`error`, its reports and any
        # message it causes go to it alone (FluidNC does this per channel). None: everyone.
        self._reply_to = None
        self._wait_client = None                   # who is owed the deferred sync `ok`
        self._homing_client = None
        self._rx_lock = threading.Lock()
        self._lines: Deque[str] = deque()
        self._realtime: Deque[int] = deque()
        self._wait_block: Optional[Block] = None   # sync block whose `ok` is owed
        self._wait_after: Optional[Callable[[], None]] = None
        self._homing: Optional[dict] = None
        self._reports = 0
        self._last_wco = None
        self._last_ov = None
        self._boot(power_up=True)

    # ── input ─────────────────────────────────────────────────────────────
    def feed(self, data: bytes, client=None) -> None:
        """Bytes from one client (a socket, or None for a single host). Each client's lines
        are assembled apart, so two senders typing at once cannot splice a line."""
        with self._rx_lock:
            self._rx.append((client, bytes(data)))

    def _drain_input(self) -> None:
        with self._rx_lock:
            chunks, self._rx = self._rx, []
        for client, data in chunks:
            line = self._partials.get(client, bytearray())
            for b in data:
                if b in REALTIME:
                    self._realtime.append((b, client))
                elif b in (0x0A, 0x0D):
                    if line:
                        self._lines.append((line.decode("ascii", "replace"), client))
                        line = bytearray()
                elif 0x20 <= b < 0x7F:
                    line.append(b)
            self._partials[client] = line

    def _send(self, text: str, client=None) -> None:
        if client is None:
            self.out(text)
        else:
            self.out(text, client)

    def _say(self, text: str) -> None:
        self._send(text + "\r\n", self._reply_to)

    def _event(self, kind: str, **fields) -> None:
        self.events.append({"t": round(self.time, 4), "kind": kind, **fields})

    # ── lifecycle ─────────────────────────────────────────────────────────
    def _boot(self, power_up: bool = False) -> None:
        # GRBL locks at power-up when homing is enabled (HOMING_INIT_LOCK); after a
        # reset it stays in Alarm only if it was in Alarm (or Sleep) before.
        self.out("\r\n" + self.banner + "\r\n")
        if (power_up and self.machine.homing) or self.critical is not None or self.state in ("Alarm", "Sleep"):
            self.state = "Alarm"
            self._say("[MSG:'$H'|'$X' to unlock]")
        else:
            self.state = "Idle"
            self._run_startup_lines()
        self.critical = None

    def _run_startup_lines(self) -> None:
        for text in self.startup_lines:
            if text:
                status = self._gcode_line(text, startup=True)
                self.out(f">{text}:{'ok' if status == 0 else f'error:{status}'}\r\n")

    def soft_reset(self) -> None:
        if self.planner.moving() or self._homing is not None:
            self._raise_alarm(3, report=True, critical=False)
        elif self.critical is not None:
            self.state = "Alarm"
        self.planner.reset(self.planner.position)
        self._homing = None
        self._wait_block = self._wait_after = None
        self.pending_alarm = None
        self.jog_cancel = False
        self.check_mode = False
        self.spindle = ("off", 0.0)
        with self._rx_lock:
            self._rx.clear()
        self._partials.clear()
        self._lines.clear()
        self._realtime.clear()
        # GRBL re-initialises the parser: modal state and G92 reset, G54-G59/G28/G30 persist.
        keep = (self.interp.wcs_offsets, self.interp.g28, self.interp.g30)
        self.interp = Interpreter()
        self.interp.wcs_offsets, self.interp.g28, self.interp.g30 = keep
        self._sync_interp_settings()
        self.interp.position = list(self.planner.position)
        self.planner.feed_override = self.planner.rapid_override = self.spindle_override = 1.0
        self._event("reset")
        self._boot()

    def _sync_interp_settings(self) -> None:
        s = self.machine.settings
        self.interp.arc_tolerance = float(s[12])
        self.interp.spindle_max = float(s[30])
        self.interp.spindle_min = float(s[31])

    def _raise_alarm(self, code: int, report: bool = True, critical: bool = True) -> None:
        self.planner.flush()
        self.planner.velocity = 0.0
        self.planner.holding = False
        self.interp.position = list(self.planner.position)
        self._homing = None
        self._wait_block = self._wait_after = None
        self.state = "Alarm"
        if code in (1, 3, 9):
            self.homed = False
        self._event("alarm", code=code, text=ALARM_TEXT.get(code, ""),
                    mpos=[round(v, 4) for v in self.planner.position])
        if report:
            self._say(f"ALARM:{code}")
        if critical:
            self.critical = code
            self._say("[MSG:Reset to continue]")

    # ── the clock ─────────────────────────────────────────────────────────
    def tick(self, dt: float, measured_s: Optional[Sequence[float]] = None) -> List[float]:
        """Advance dt seconds. `measured_s` is the plant's physical reading (mm) if real.

        Returns the physical command per axis (mm along each rail).
        """
        self.time += dt
        if measured_s is not None:
            for a, s in zip(AXES, measured_s):
                self.plant.rails[a].measured = float(s)
        self._drain_input()
        while self._realtime:
            self._handle_realtime(*self._realtime.popleft())
        self._process_lines()
        if self._homing is not None:
            self._advance_homing(dt)
        elif not self.check_mode:
            self.planner.advance(dt)
        command = self.plant.command(self.planner.position, self.time)
        if measured_s is None:
            self.plant.settle_ideal()
        self._after_motion()
        return command

    def _after_motion(self) -> None:
        if self._homing is None and self.state not in ("Alarm",) and self.machine.hard_limits:
            tripped = self.plant.switches()
            if tripped:
                self._event("hard_limit", switches=tripped)
                self._raise_alarm(1)
                return
        if self.critical is not None:
            return
        stopped = not self.planner.moving()
        if self.planner.holding and stopped:
            if self.pending_alarm is not None:
                code, self.pending_alarm = self.pending_alarm, None
                self._raise_alarm(code)
                return
            if self.jog_cancel or self.state == "Jog":
                self.jog_cancel = False
                self.planner.holding = False
                self.planner.flush()
                self.interp.position = list(self.planner.position)
                self.state = "Idle"
                return
            if self.state == "Hold:1":
                self.state = "Hold:0"
        if self.state in ("Run", "Jog") and self.planner.idle():
            self.state = "Idle"
        elif self.state == "Idle" and not self.planner.idle() and not self.planner.holding:
            self.state = "Run"

    # ── real-time commands ───────────────────────────────────────────────
    def _handle_realtime(self, b: int, client=None) -> None:
        if b == ord("?"):
            self._send(self.status_report() + "\r\n", client)
        elif b == 0x18:
            self.soft_reset()
        elif self.critical is not None:
            return
        elif b == ord("!") or b == 0x84:
            if self.state in ("Run", "Idle"):
                self.planner.holding = True
                self.state = "Hold:1" if self.planner.moving() else "Hold:0"
                self._event("feed_hold")
            elif self.state == "Jog":
                self.jog_cancel = True
                self.planner.holding = True
        elif b == ord("~"):
            if self.state == "Hold:0":
                self.planner.holding = False
                self.state = "Run" if not self.planner.idle() else "Idle"
                self._event("cycle_start")
        elif b == 0x85:
            if self.state == "Jog":
                self.jog_cancel = True
                self.planner.holding = True
        elif 0x90 <= b <= 0x94:
            ov = self.planner.feed_override
            ov = {0x90: 1.0, 0x91: ov + 0.10, 0x92: ov - 0.10, 0x93: ov + 0.01, 0x94: ov - 0.01}[b]
            self.planner.feed_override = round(min(2.0, max(0.10, ov)), 2)
        elif 0x95 <= b <= 0x97:
            self.planner.rapid_override = {0x95: 1.0, 0x96: 0.5, 0x97: 0.25}[b]
        elif 0x99 <= b <= 0x9D:
            ov = self.spindle_override
            ov = {0x99: 1.0, 0x9A: ov + 0.10, 0x9B: ov - 0.10, 0x9C: ov + 0.01, 0x9D: ov - 0.01}[b]
            self.spindle_override = round(min(2.0, max(0.10, ov)), 2)

    # ── lines ────────────────────────────────────────────────────────────
    def _process_lines(self) -> None:
        while self._lines and self.critical is None:
            if self._wait_block is not None or self._homing is not None or self.state == "Sleep":
                return
            if not self.planner.has_room():
                return
            raw, self._reply_to = self._lines.popleft()
            try:
                text = strip_line(raw) if not raw.lstrip().startswith("$") else raw.strip()
                if not text:
                    self._say("ok")
                    continue
                if text.startswith("$"):
                    status = self._system_line(text)
                else:
                    status = self._gcode_line(text)
                if status is None:
                    # Reply deferred (sync, homing) or alarm raised: remember who is owed it.
                    self._wait_client = self._reply_to
                    if self._homing is not None:
                        self._homing_client = self._reply_to
                    continue
                self._say("ok" if status == 0 else f"error:{status}")
            finally:
                self._reply_to = None

    def _gcode_line(self, text: str, startup: bool = False) -> Optional[int]:
        if self.state == "Alarm" or self.state == "Jog":
            return 9
        laser = bool(self.machine.settings[32])
        try:
            blocks = self.interp.execute(text, source_line=self._line_no())
        except GcodeError as exc:
            return exc.code
        if laser and any(b.kind == "line" for b in blocks):
            blocks = [b for b in blocks if b.kind != "spindle"]
        if self.machine.soft_limits:
            for b in blocks:
                if b.kind == "line" and not self._inside(b.target):
                    self._event("soft_limit", line=text, target=[round(v, 4) for v in b.target])
                    self.interp.position = list(self.planner._tail)
                    if self.planner.moving():
                        self.pending_alarm = 2
                        self.planner.holding = True
                        self.state = "Hold:1"
                    else:
                        self._raise_alarm(2)
                    return None
        if self.check_mode:
            for b in blocks:
                if b.kind == "end":
                    self.interp.position = list(self.planner.position)
            return 0
        sync = None
        for b in blocks:
            self.planner.push(b)
            if b.kind != "line":
                sync = b
        if blocks and self.state == "Idle":
            self.state = "Run"
        if sync is not None and not startup:
            self._wait_block = sync
            return None
        return 0

    def _line_no(self) -> int:
        self._count = getattr(self, "_count", 0) + 1
        return self._count

    def _inside(self, target: Sequence[float]) -> bool:
        return all(self.machine.axes[a].inside(target[i]) for i, a in enumerate(AXES))

    def _on_sync(self, block: Block) -> None:
        if block.kind in ("spindle", "end"):
            self.spindle = block.spindle
        if block.kind == "pause":
            self.planner.holding = True
            self.state = "Hold:0"
            self._event("program_pause", line=block.source_line)
        elif block.kind == "end":
            self._say("[MSG:Pgm End]")
            self._event("program_end", line=block.source_line)
        if block is self._wait_block:
            self._wait_block = None
            self._send("ok\r\n", self._wait_client)

    def _load_tool(self, text: str) -> int:
        """`$TWIN TOOL=n`: put tool n in the spindle, as an operator would after M6.

        Only at rest (error:8 otherwise, like `$$`). The twin knows each tool's length,
        so `$TWIN` then reports the `G43.1 Z` a tool setter would have measured.
        """
        if self.state != "Idle":
            return 8
        try:
            number = int(text)
        except ValueError:
            return 3
        if number not in self.tools:
            self._say(f"[MSG:twin: no tool {number} in the table]")
            return 38
        self.tool = self.tools[number]
        self.on_tool_change(self.tool)
        self._event("tool_change", tool=self.tool.summary())
        return 0

    # ── $ commands ───────────────────────────────────────────────────────
    def _system_line(self, text: str) -> Optional[int]:
        body = text[1:].upper().replace(" ", "")
        idle_or_alarm = self.state in ("Idle", "Alarm")
        if body == "":
            self._say("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~ ! ? ctrl-x]")
            return 0
        if body.startswith("J="):
            return self._jog(text[3:])
        if body == "$":
            if self.state in ("Run", "Hold:0", "Hold:1"):
                return 8
            for key in sorted(self.machine.settings):
                v = self.machine.settings[key]
                self._say(f"${key}={int(v)}" if key in INTEGER_SETTINGS else f"${key}={v:.3f}")
            return 0
        if body == "G":
            self._say(self.interp.parser_state())
            return 0
        if body == "C":
            if self.check_mode:
                self._say("[MSG:Disabled]")
                self.soft_reset()
                return None
            if self.state != "Idle":
                return 8
            self.check_mode = True
            self.state = "Check"
            self._say("[MSG:Enabled]")
            return 0
        if body == "X":
            if self.state == "Alarm":
                self.state = "Idle"
                self._say("[MSG:Caution: Unlocked]")
                self._event("unlock", homed=self.homed)
            return 0
        if body == "TWIN":
            self._say("[TWIN:" + json.dumps(self.twin_report(), separators=(",", ":")) + "]")
            return 0
        if body.startswith("TWIN") and "TOOL=" in body:
            return self._load_tool(body.split("TOOL=", 1)[1])
        if not idle_or_alarm:
            return 8
        if body == "#":
            for i, w in enumerate(self.interp.wcs_offsets):
                self._say(f"[G{54 + i}:{self._vec(w)}]")
            self._say(f"[G28:{self._vec(self.interp.g28)}]")
            self._say(f"[G30:{self._vec(self.interp.g30)}]")
            self._say(f"[G92:{self._vec(self.interp.g92)}]")
            self._say(f"[TLO:{self._num(self.interp.tlo)}]")
            self._say(f"[PRB:{self._vec([0, 0, 0])}:0]")
            return 0
        if body == "I":
            self._say(f"[VER:{VERSION}.{BUILD}:OmniSim twin]")
            self._say("[OPT:V,15,128]")
            return 0
        if body == "H":
            if not self.machine.homing:
                return 5
            self._start_homing()
            return None
        if body == "SLP":
            self.planner.flush()
            self.state = "Sleep"
            self._say("[MSG:Sleeping]")
            return 0
        if body == "N":
            for i, line in enumerate(self.startup_lines):
                self._say(f"$N{i}={line}")
            return 0
        if body.startswith("N") and "=" in body:
            idx = body[1:body.index("=")]
            if idx not in ("0", "1"):
                return 3
            self.startup_lines[int(idx)] = text[text.index("=") + 1:].strip()
            return 0
        if body.startswith("RST="):
            what = body[4:]
            if what not in ("$", "#", "*"):
                return 3
            if what in ("$", "*"):
                self.machine.settings = dict(self.defaults.settings)
                for k, v in self.defaults.settings.items():
                    self.machine.apply_setting(k, v)
                self._sync_interp_settings()
            if what in ("#", "*"):
                self.interp.wcs_offsets = [[0.0] * 3 for _ in range(6)]
                self.interp.g28 = [0.0] * 3
                self.interp.g30 = [0.0] * 3
            return 0
        if "=" in body:
            key, _, value = body.partition("=")
            try:
                k, v = int(key), float(value)
            except ValueError:
                return 3
            if k not in GRBL_DEFAULTS:
                return 3
            if v < 0:
                return 4
            self.machine.apply_setting(k, v)
            self._sync_interp_settings()
            self._event("setting", key=k, value=v)
            return 0
        return 3

    def _jog(self, rest: str) -> Optional[int]:
        if self.state not in ("Idle", "Jog"):
            return 8
        line = strip_line(rest)
        if not line:
            return 16
        units, distance, machine_frame = self.interp.modal.units, self.interp.modal.distance, False
        values: Dict[str, float] = {}
        for letter, num in re.findall(r"([A-Z])([-+]?(?:\d+\.?\d*|\.\d+))", line):
            v = float(num)
            if letter == "G":
                if v in (20, 21):
                    units = f"G{int(v)}"
                elif v in (90, 91):
                    distance = f"G{int(v)}"
                elif v == 53:
                    machine_frame = True
                else:
                    return 16
            elif letter in "XYZF":
                values[letter] = v
            else:
                return 16
        if "".join(re.findall(r"[A-Z][-+]?(?:\d+\.?\d*|\.\d+)", line)) != line:
            return 2
        if "F" not in values:
            return 22
        if not any(k in values for k in "XYZ"):
            return 26
        scale = MM_PER_INCH if units == "G20" else 1.0
        start = list(self.interp.position)
        offset = [0.0, 0.0, 0.0] if machine_frame else self.interp.work_offset()
        target = list(start)
        for i, k in enumerate("XYZ"):
            if k in values:
                v = values[k] * scale
                target[i] = start[i] + v if distance == "G91" and not machine_frame else v + offset[i]
        if self.machine.soft_limits and not self._inside(target):
            return 15
        self.interp.position = target
        self.planner.push(Block("line", tuple(target), False, values["F"] * scale), jog=True)
        self.state = "Jog"
        return 0

    # ── homing ───────────────────────────────────────────────────────────
    def _start_homing(self) -> None:
        self.planner.flush()
        self.state = "Home"
        cycles = [["z"], ["x", "y"]]
        self._homing = {"cycles": cycles, "cycle": 0, "phase": "seek", "travelled": {}}
        self._event("homing_start")
        self._begin_phase()

    def _begin_phase(self) -> None:
        h = self._homing
        axes = h["cycles"][h["cycle"]]
        h["travelled"] = {a: 0.0 for a in axes}
        h["active"] = list(axes)

    def _advance_homing(self, dt: float) -> None:
        h = self._homing
        s = self.machine.settings
        phase = h["phase"]
        rate = (float(s[25]) if phase in ("seek", "final") else float(s[24])) / 60.0
        pulloff = self.machine.pulloff
        pos = self.planner.position
        for a in list(h["active"]):
            i = AXES.index(a)
            ax = self.machine.axes[a]
            toward = 1.0 if ax.home_positive else -1.0
            side = "max" if ax.home_positive else "min"
            direction = toward if phase in ("seek", "locate") else -toward
            step = rate * dt
            pos[i] += direction * step
            h["travelled"][a] += step
            physical = self.plant.to_physical(a, pos[i])
            switch_now = self.plant.rails[a].switch(physical)
            if phase in ("seek", "locate"):
                limit = 1.5 * ax.travel if phase == "seek" else 5.0 * pulloff
                if switch_now == side:
                    if phase == "locate":
                        # The rail end is the switch's trigger point; the overtravel
                        # past it is only what a moving carriage needs to re-trip it.
                        rail = self.plant.rails[a]
                        edge = rail.length if side == "max" else 0.0
                        pos[i] = edge + rail.env_min - rail.offset
                    h["active"].remove(a)
                elif h["travelled"][a] > limit:
                    self._raise_alarm(9)
                    return
            elif h["travelled"][a] >= pulloff:
                pos[i] -= direction * (h["travelled"][a] - pulloff)
                h["active"].remove(a)
        self.plant.command(pos, self.time)
        if h["active"]:
            return
        order = ["seek", "backoff", "locate", "final"]
        if phase != "final":
            h["phase"] = order[order.index(phase) + 1]
            self._begin_phase()
            return
        for a in h["cycles"][h["cycle"]]:
            home = self.machine.axes[a].home_mpos
            self.plant.rebase(a, home)
            pos[AXES.index(a)] = home
        h["cycle"] += 1
        if h["cycle"] < len(h["cycles"]):
            h["phase"] = "seek"
            self._begin_phase()
            return
        self._homing = None
        self.homed = True
        self.planner.reset(pos)
        self.interp.position = list(pos)
        self.state = "Idle"
        self._event("homing_done", mpos=[round(v, 4) for v in pos])
        self._send("ok\r\n", self._homing_client)
        self._run_startup_lines()

    # ── reports ──────────────────────────────────────────────────────────
    def _num(self, mm: float) -> str:
        if self.machine.settings[13]:
            return f"{mm / MM_PER_INCH:.4f}"
        return f"{mm:.3f}"

    def _vec(self, v: Sequence[float]) -> str:
        return ",".join(self._num(x) for x in v)

    def status_report(self) -> str:
        mpos = self.planner.position
        wco = self.interp.work_offset()
        if int(self.machine.settings[10]) & 1:
            parts = [self.state, f"MPos:{self._vec(mpos)}"]
        else:
            parts = [self.state, f"WPos:{self._vec([mpos[i] - wco[i] for i in range(3)])}"]
        if int(self.machine.settings[10]) & 2:
            parts.append(f"Bf:{self.planner.capacity - len(self.planner.queue)},128")
        spindle = self.spindle
        speed = 0 if spindle[0] == "off" else spindle[1] * self.spindle_override
        feed = self.planner.current_feed_mm_min()
        if self.machine.settings[13]:
            feed /= MM_PER_INCH
        parts.append(f"FS:{feed:.0f},{speed:.0f}")
        pins = self.plant.switches()
        if pins:
            parts.append(f"Pn:{pins}")
        self._reports += 1
        wco_key = tuple(round(v, 4) for v in wco)
        ov_key = (self.planner.feed_override, self.planner.rapid_override, self.spindle_override)
        if wco_key != self._last_wco or self._reports % 10 == 1:
            parts.append(f"WCO:{self._vec(wco)}")
            self._last_wco = wco_key
        elif ov_key != self._last_ov or self._reports % 10 == 2:
            parts.append("Ov:" + ",".join(f"{v * 100:.0f}" for v in ov_key))
            self._last_ov = ov_key
        return "<" + "|".join(parts) + ">"

    def twin_report(self) -> dict:
        """GRBL's belief beside the carriage's physical position (not part of GRBL)."""
        measured = self.plant.measured_mpos()
        belief = self.planner.position
        return {
            "t": round(self.time, 4), "state": self.state, "homed": self.homed,
            "belief_mpos": [round(v, 4) for v in belief],
            "measured_mpos": [round(v, 4) for v in measured],
            "error_mm": [round(measured[i] - belief[i], 4) for i in range(3)],
            "rails": self.plant.snapshot(), "stalls": list(self.plant.stalls[-5:]),
            "alarms": [e for e in self.events if e["kind"] == "alarm"][-5:],
            "spindle": list(self.spindle),
            "stock": self.stock.summary() if self.stock is not None else None,
            # `G43.1 Z<tlo_mm>` keeps work Z right for the tool in the spindle (see tool_table.py).
            "tool": {**self.tool.summary(), "tlo_mm": self.tool.stickout_mm},
            "cutting": self.cutting,
        }
