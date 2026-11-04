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

"""OmniLink bridge for a GRBL CNC machine: talk to it, and it machines.

It drives the machine through GRBL over a stream (grbl_link.GrblLink), so the
same bridge runs against the twin's virtual GRBL and, later, a real board.
OmniLink's pipeline is reused as is, in the order every bridge uses:

  POST /prompt  access check (no OmniKey -> 401; there is no keyless path)
                -> the package's deterministic parser (route.short_circuit):
                   "stop", "pause", "resume", "go home" and status questions
                   are answered here, without the model
                -> the model through the OmniLink relay, whose tool calls
                   pass the safety gate (intent rules: questions, negations,
                   quotations, retractions never move the machine)
  POST /tool    a typed call: the gate's schema check, then the tool
  POST /stop_robot, GET /get_robot_state, GET /chat (browser console)

The checks that need the MACHINE and the WORK live here, in each tool's
dispatch, and refuse with a named rule (the gate's own rails are metric robot
rails, deliberately not consulted: every CNC argument is named in mm):

  - preflight      every job runs through preflight.py first; a GRBL error,
                   an unsupported command or a feed move with the spindle off
                   is refused before anything moves
  - travel         every target is checked against where the carriages can
                   physically go (the twin's rails; on a real board, the soft
                   envelope once homed)
  - cut_depth      no job cuts deeper than the stock minus a 0.5 mm skin
                   (the spoilboard is not the part); Z0 is the stock top
  - busy           one job at a time; jogs and moves wait for it
  - plunge         a rapid or a move_to never goes below Z0: material is only
                   entered by a cutting job, at its plunge feed

Pause is a FEED HOLD (resumable, position kept). Stop is a feed hold, then a
reset once motion has stopped: the job is cancelled, the spindle is off, and
-- because the reset happens at rest -- the position is kept (a reset IN
motion would raise ALARM:3 and lose it).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for p in (HERE, ROOT / "packages/omnisim-bridges/src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from omnisim_bridges import access, route  # noqa: E402
from omnisim_bridges import gate as gate_mod  # noqa: E402
from omnisim_bridges.bridge_base import BridgeBase, serve_tool  # noqa: E402
from omnisim_bridges.tool import Tool  # noqa: E402

import machine as machine_mod  # noqa: E402
import preflight as preflight_mod  # noqa: E402
import toolpath  # noqa: E402
import tool_table as tools_mod  # noqa: E402
from grbl_link import GrblError, GrblLink  # noqa: E402

SURFACE = "cnc"
# The package parser has no "cnc" surface; its surface-independent rules (stop,
# pause, resume, go home, status questions) are exactly the CNC's parser-first set.
PARSER_SURFACE = "any"
SAFE_Z_MM = 5.0
MAX_JOG_MM = 300.0
DEPTH_SKIN_MM = 0.5
MAX_PROGRAM_LINES = 20000


def _refuse(rule: str, say: str, **extra) -> dict:
    return {"accepted": False, "rule": rule, "say": say, **extra}


class CncBridge(BridgeBase):
    surface = SURFACE
    GATED_PATHS = ("/prompt", "/tool")
    UNGATED_PATHS = ("/stop_robot",)

    def __init__(self, link: GrblLink, m: machine_mod.Machine, tool_d_mm: float = 6.35,
                 stock_thickness_mm: Optional[float] = None, name: str = "cnc",
                 tool_table: Optional[Dict[int, tools_mod.Tool]] = None):
        self.link = link
        self.machine = m
        self.tool_table = tool_table or tools_mod.default_tools(tool_d_mm)
        # Tool 1 at power-up. The twin reports the tool in the spindle; a real board cannot,
        # so there this is the last tool a job of ours loaded.
        self.loaded = self.tool_table[1]
        self.stock_thickness = stock_thickness_mm
        self.robot_id = name
        self.model = m.name
        # A dict, like every bridge's: profile_extras() merges it with the safety-gate block.
        self.capabilities = {"machine": "cnc", "controller": "grbl",
                             "actions": ["prompt", "jog", "move_to", "set_work_zero", "cut_pocket",
                                         "change_tool", "run_gcode", "pause", "resume", "stop"]}
        self.relay = None
        self.turn_text = ""
        self.last_job: Optional[dict] = None
        self._lock = threading.Lock()

    # ---- state ---------------------------------------------------------------
    def get_state(self) -> Dict[str, Any]:
        st = self.link.status()
        job = self.link.job.snapshot() if self.link.job is not None else None
        twin = self.link.twin() if (job is None or job["state"] != "running") else None
        self._note_loaded(twin)
        work = st.get("WPos")
        out = {
            "robot_id": self.robot_id, "model": self.model, "surface": SURFACE,
            "connected": self.link.connected,
            "mode": st.get("state"), "machine_mpos_mm": st.get("MPos"), "work_pos_mm": work,
            "work_offset_mm": st.get("WCO"), "feed_mm_min": st.get("feed"), "spindle_rpm": st.get("spindle"),
            "job": job, "last_job": self.last_job,
            "tool": self.loaded.summary(),
            "tool_table": [t.summary() for t in self.tool_table.values()],
            "stock_thickness_mm": self.stock_thickness,
            "travel_mm": {a: ax.travel for a, ax in self.machine.axes.items()},
            # The browser console's vitals panel reads `tcp`: the tool's work position.
            "tcp": work,
        }
        if twin is not None:
            out["twin"] = {"measured_vs_grbl_mm": twin.get("error_mm"), "stock": twin.get("stock"),
                           "cutting": twin.get("cutting")}
        return out

    def _note_loaded(self, twin: Optional[dict]) -> None:
        number = ((twin or {}).get("tool") or {}).get("number")
        if number in self.tool_table:
            self.loaded = self.tool_table[number]

    def _loaded_tool(self) -> tools_mod.Tool:
        job = self.link.job
        if job is None or job.state != "running":
            self._note_loaded(self.link.twin())
        return self.loaded

    def describe_state(self) -> str:
        s = self.get_state()
        pos = s.get("work_pos_mm") or [0, 0, 0]
        parts = [f"{s.get('mode') or 'unknown'}; tool at X{pos[0]:.2f} Y{pos[1]:.2f} Z{pos[2]:.2f} mm (work)"]
        if s.get("spindle_rpm"):
            parts.append(f"spindle {s['spindle_rpm']:.0f} rpm")
        job = s.get("job")
        if job:
            parts.append(f"job {job['state']}, line {job['lines_acked']}/{job['lines_total']}, "
                         f"{job['elapsed_s']:.0f} s")
        stock = (s.get("twin") or {}).get("stock") or {}
        eng = stock.get("engine") or {}
        if eng:
            parts.append(f"{eng.get('removed_mm3', 0) / 1000:.1f} cm3 removed, deepest {eng.get('deepest_mm')} mm")
        return "I'm " + ", ".join(parts) + "."

    # ---- the parser's verbs ----------------------------------------------------
    def act_stop(self) -> Dict[str, Any]:
        text = self.turn_text.lower()
        if re.search(r"\b(pause|hold)\b", text) and not re.search(r"\b(stop|abort|cancel|halt)\b", text):
            return self.pause()
        return self.stop()

    def act_resume_autonomy(self) -> Dict[str, Any]:
        return self.resume()

    def act_reset_to_home(self) -> Dict[str, Any]:
        """'Go home': raise to the safe height, then go to the work origin X0 Y0."""
        return self.move_to(x_mm=0.0, y_mm=0.0, z_mm=SAFE_Z_MM)

    # ---- real-time actions (never refused) ------------------------------------
    def _hold_until_still(self, timeout_s: float = 5.0) -> dict:
        """Feed hold, then MEASURE the machine coming to rest (the parser reports it as such)."""
        self.link.feed_hold()
        t0 = time.time()
        st = self.link.status()
        while time.time() - t0 < timeout_s:
            st = self.link.status()
            if st.get("state") in ("Hold:0", "Idle", "Alarm") and (st.get("feed") or 0) == 0:
                break
            time.sleep(0.05)
        feed = float(st.get("feed") or 0.0)
        return {"stationary": feed == 0.0,
                "measured": {"speed_mps": feed / 60000.0, "over_s": round(time.time() - t0, 2)},
                "state": st.get("state")}

    def pause(self) -> Dict[str, Any]:
        rest = self._hold_until_still()
        return {"accepted": True, **rest,
                "say": "Paused with a feed hold. The position is kept and the job is intact; say resume "
                       "to continue or stop to cancel it."}

    def resume(self) -> Dict[str, Any]:
        st = self.link.status()
        if not str(st.get("state", "")).startswith("Hold"):
            return {"accepted": False, "rule": "nothing_paused", "say": "Nothing is paused."}
        self.link.cycle_start()
        return {"accepted": True, "say": "Resumed."}

    def stop(self) -> Dict[str, Any]:
        rest = self._hold_until_still()
        self.link.cancel_job()          # a reset at rest: queue flushed, spindle off, position kept
        time.sleep(0.3)
        st = self.link.status()
        if st.get("state") == "Alarm":
            self.link.command("$X")
        rest["state"] = self.link.status().get("state")
        return {"accepted": True, **rest,
                "say": "Stopped. The job is cancelled and the spindle is off; the position is kept."}

    # ---- physical checks ---------------------------------------------------------
    def _physical_violation(self, mpos_lo: List[float], mpos_hi: List[float]) -> Optional[dict]:
        """Is the machine-coordinate box reachable by the carriages?"""
        twin = self.link.twin()
        if twin is not None:
            rails = twin.get("rails") or {}
            for i, a in enumerate("xyz"):
                r = rails.get(a)
                if not r:
                    continue
                c = r["measured_s"] - twin["measured_mpos"][i]     # belief -> rail position
                lo, hi = mpos_lo[i] + c, mpos_hi[i] + c
                if lo < -1e-6 or hi > r["rail_mm"] + 1e-6:
                    return _refuse("travel", f"That needs {a.upper()} from {lo:.1f} to {hi:.1f} mm along a "
                                             f"{r['rail_mm']:.0f} mm rail; the carriage can't go there.",
                                   axis=a)
            return None
        if self.machine.soft_limits:
            for i, (a, ax) in enumerate(self.machine.axes.items()):
                if not (ax.inside(mpos_lo[i]) and ax.inside(mpos_hi[i])):
                    return _refuse("travel", f"That leaves the {a.upper()} soft envelope.", axis=a)
            return None
        return _refuse("travel_unknown", "I can't check the travel on this machine (no twin and no soft "
                                         "limits), so I won't move it from here.")

    def _busy(self) -> Optional[dict]:
        job = self.link.job
        if job is not None and job.state == "running":
            return _refuse("busy", "A job is running. Pause or stop it first.")
        st = self.link.status()
        if st.get("state") == "Alarm":
            return _refuse("alarm", "The controller is in alarm. Say stop to reset it first.")
        return None

    # ---- tools -------------------------------------------------------------------
    def jog(self, x_mm=None, y_mm=None, z_mm=None, feed_mm_min=None) -> dict:
        moves = {a: float(v) for a, v in (("x", x_mm), ("y", y_mm), ("z", z_mm)) if v is not None}
        if not moves:
            return _refuse("missing_axis", "Which axis, and how far?")
        if any(abs(v) > MAX_JOG_MM for v in moves.values()):
            return _refuse("jog_limit", f"A jog is at most {MAX_JOG_MM:.0f} mm per axis.")
        busy = self._busy()
        if busy:
            return busy
        feed = min(float(feed_mm_min or 1000.0), min(ax.max_rate for ax in self.machine.axes.values()))
        mpos = self.link.status().get("MPos") or [0.0, 0.0, 0.0]
        target = [mpos[i] + moves.get(a, 0.0) for i, a in enumerate("xyz")]
        bad = self._physical_violation([min(mpos[i], target[i]) for i in range(3)],
                                       [max(mpos[i], target[i]) for i in range(3)])
        if bad:
            return bad
        words = " ".join(f"{a.upper()}{v:.3f}" for a, v in moves.items())
        answer = self.link.command(f"$J=G91 G21 {words} F{feed:.0f}")
        if answer != "ok":
            return _refuse("controller", f"The controller refused the jog ({answer}).")
        st = self.link.wait_idle(120)
        return {"accepted": True, "commanded_mm": moves, "work_pos_mm": st.get("WPos"), "settled": True}

    def move_to(self, x_mm=None, y_mm=None, z_mm=None) -> dict:
        if x_mm is None and y_mm is None and z_mm is None:
            return _refuse("missing_axis", "Move where?")
        if z_mm is not None and float(z_mm) < 0.0:
            return _refuse("plunge", "I only go below Z0 (the stock top) inside a cutting job, at the plunge "
                                     "feed. Use a job to cut.")
        busy = self._busy()
        if busy:
            return busy
        st = self.link.status()
        wpos, wco, mpos = st.get("WPos") or [0, 0, 0], st.get("WCO") or [0, 0, 0], st.get("MPos") or [0, 0, 0]
        target_w = [float(v) if v is not None else wpos[i] for i, v in enumerate((x_mm, y_mm, z_mm))]
        lines = []
        # Never travel sideways with the tool in or near the work: lift first.
        lift_z = max(SAFE_Z_MM, target_w[2]) if (x_mm is not None or y_mm is not None) else None
        if lift_z is not None and wpos[2] < lift_z:
            lines.append(f"G90 G21 G0 Z{lift_z:.3f}")
        if x_mm is not None or y_mm is not None:
            lines.append(f"G90 G21 G0 X{target_w[0]:.3f} Y{target_w[1]:.3f}")
        if z_mm is not None:
            lines.append(f"G90 G21 G0 Z{target_w[2]:.3f}")
        target_m = [target_w[i] + wco[i] for i in range(3)]
        top_m = [max(mpos[i], target_m[i]) for i in range(3)]
        if lift_z is not None:
            top_m[2] = max(top_m[2], lift_z + wco[2])
        bad = self._physical_violation([min(mpos[i], target_m[i]) for i in range(3)], top_m)
        if bad:
            return bad
        for line in lines:
            answer = self.link.command(line)
            if answer != "ok":
                return _refuse("controller", f"The controller refused the move ({answer}).")
        st = self.link.wait_idle(180)
        return {"accepted": True, "commanded_work_mm": target_w, "work_pos_mm": st.get("WPos"), "settled": True}

    def set_work_zero(self, axes=None) -> dict:
        axes = [str(a).lower() for a in (axes or [])]
        if not axes or any(a not in ("x", "y", "z") for a in axes):
            return _refuse("missing_axis", "Which axes should be zero here: x, y and/or z?")
        busy = self._busy()
        if busy:
            return busy
        words = " ".join(f"{a.upper()}0" for a in axes)
        answer = self.link.command(f"G10 L20 P1 {words}")
        if answer != "ok":
            return _refuse("controller", f"The controller refused ({answer}).")
        st = self.link.status()
        return {"accepted": True, "zeroed": axes, "work_offset_mm": st.get("WCO"), "work_pos_mm": st.get("WPos")}

    def cut_pocket(self, shape="circle", width_mm=None, height_mm=None, depth_mm=None,
                   center_x_mm=0.0, center_y_mm=0.0, stepdown_mm=None, feed_mm_min=None) -> dict:
        if width_mm is None or depth_mm is None:
            return _refuse("missing_size", "I need the pocket's size and depth in mm.")
        w, d = float(width_mm), float(depth_mm)
        h = float(height_mm) if height_mm is not None else w
        cx, cy = float(center_x_mm or 0.0), float(center_y_mm or 0.0)
        if shape == "circle":
            region = toolpath.circle(cx, cy, w / 2.0)
        elif shape == "rectangle":
            from shapely.geometry import box
            region = box(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
        elif shape == "star":
            region = toolpath.circle(cx, cy, w / 2.0).difference(
                toolpath.star(cx, cy, 0.3 * w, 0.12 * w))
        else:
            return _refuse("shape", "I can pocket a circle, a rectangle, or a star inside a circle.")
        tool = self._loaded_tool()
        if tool.shape != "flat":
            return _refuse("flat_tool_needed", f"Pockets need a flat end mill; the spindle holds T{tool.number}, "
                                               f"a {tool.shape} cutter. Change to a flat end mill first.")
        tool_d = tool.diameter_mm
        if w < tool_d * 1.05 or h < tool_d * 1.05:
            return _refuse("too_small", f"The cutter is {tool_d:g} mm; the pocket must be wider than that.")
        step = float(stepdown_mm or min(d, 3.0))
        feed = float(feed_mm_min or 2500.0)
        try:
            program = toolpath.pocket(region, tool_d, d, stepover=0.6 * tool_d, step_down=step,
                                      feed=feed, title=f"{shape} pocket {w:g}x{h:g} mm, {d:g} mm deep")
        except ValueError as exc:
            return _refuse("toolpath", str(exc))
        return self._start_job(program, f"{shape} pocket {w:g} x {h:g} mm, {d:g} mm deep at X{cx:g} Y{cy:g}")

    def change_tool(self, number=None) -> dict:
        if number is None or int(number) not in self.tool_table:
            listed = ", ".join(f"T{t.number} {t.name or t.shape}" for t in self.tool_table.values())
            return _refuse("unknown_tool", f"The tool table has {listed}.")
        tool = self.tool_table[int(number)]
        return self._start_job(f"T{tool.number} M6\n", f"tool change to T{tool.number} ({tool.name or tool.shape})")

    def run_gcode(self, program="") -> dict:
        lines = [l for l in str(program).splitlines() if l.strip()]
        if not lines:
            return _refuse("empty", "There is no G-code to run.")
        if len(lines) > MAX_PROGRAM_LINES:
            return _refuse("too_long", f"Programs are limited to {MAX_PROGRAM_LINES} lines here.")
        return self._start_job("\n".join(lines) + "\n", f"G-code program, {len(lines)} lines")

    def _start_job(self, program: str, title: str) -> dict:
        with self._lock:
            busy = self._busy()
            if busy:
                return busy
            st = self.link.status()
            mpos, wco = st.get("MPos") or [0, 0, 0], st.get("WCO") or [0, 0, 0]
            # Start at the safe height, as every sender's job does.
            if (st.get("WPos") or [0, 0, 0])[2] < SAFE_Z_MM:
                program = f"G90 G21 G0 Z{SAFE_Z_MM:.3f}\n" + program
            # The live work offset resolves the program's G54 coordinates; preflight's own envelope
            # verdict is not used (GRBL's soft envelope means nothing on an unhomed machine) --
            # _physical_violation judges the machine-coordinate bounds against the real rails.
            rep = preflight_mod.preflight(self.machine, program, wco=wco, start_mpos=mpos,
                                          tools=self.tool_table, tlo_mm=self._tlo())
            if rep["error_count"]:
                e = rep["errors"][0]
                return _refuse("preflight", f"Line {e['line']} would fail on the controller "
                                            f"(error:{e['code']} {e['detail']}).", preflight=rep)
            if rep["warning_counts"].get("feed_with_spindle_off"):
                return _refuse("spindle_off", "That program cuts with the spindle off.", preflight=rep)
            bounds = rep["bounds"]
            lo_m = [bounds[a]["machine"][0] if bounds[a]["machine"] else mpos[i] for i, a in enumerate("xyz")]
            hi_m = [bounds[a]["machine"][1] if bounds[a]["machine"] else mpos[i] for i, a in enumerate("xyz")]
            bad = self._physical_violation(lo_m, hi_m)
            if bad:
                bad["preflight"] = rep
                return bad
            deepest_work = lo_m[2] - wco[2]
            if self.stock_thickness is not None and -deepest_work > self.stock_thickness - DEPTH_SKIN_MM:
                return _refuse("cut_depth", f"That reaches {-deepest_work:.2f} mm below the stock top; the stock is "
                                            f"{self.stock_thickness:g} mm, so the limit is "
                                            f"{self.stock_thickness - DEPTH_SKIN_MM:g} mm.", preflight=rep)
            job = self.link.stream(program, on_done=self._job_done)
            self.last_job = {"title": title, "estimated_s": rep["estimated_seconds"], "lines": rep["lines"]}
            return {"accepted": True, "running": True, "job": title, "lines": rep["lines"],
                    "estimated_s": rep["estimated_seconds"],
                    "say": f"Started: {title}. About {rep['estimated_seconds']:.0f} s. "
                           f"Ask for status any time; say pause or stop to interrupt."}

    def _tlo(self) -> float:
        """The controller's tool length offset now (`$#` [TLO:], plain GRBL)."""
        try:
            lines = self.link.query("$#", "[TLO:")
        except GrblError:
            return 0.0
        return float(lines[-1][5:-1]) if lines else 0.0

    def _job_done(self, job) -> None:
        if self.last_job is not None:
            self.last_job.update(job.snapshot())

    # ---- registry ------------------------------------------------------------------
    def tools(self) -> List[Tool]:
        def obj(props, required=()):
            return {"type": "object", "properties": props, "required": list(required), "additionalProperties": False}

        num = {"type": "number"}
        return [
            Tool("get_robot_state", "Read the machine: controller state, tool position in work and machine mm, "
                 "spindle, the running job's progress, and (on the twin) material removed. Read it before "
                 "answering any status question or claiming a job finished.",
                 obj({}), lambda a: self.get_state(), physical=False, surface=SURFACE),
            Tool("jog", "Jog the tool RELATIVE to where it is, in mm (+X right, +Y away from the operator, "
                 "+Z up). Only axes the operator named. At most 300 mm per axis. Waits until it arrives.",
                 obj({"x_mm": num, "y_mm": num, "z_mm": num, "feed_mm_min": num}),
                 lambda a: self.jog(**a), physical=True, surface=SURFACE),
            Tool("move_to", "Rapid the tool to an ABSOLUTE work position in mm (X0 Y0 is the work origin, Z0 the "
                 "stock top). Lifts to the safe height before moving sideways. Never below Z0: cutting is "
                 "only done by jobs. Waits until it arrives.",
                 obj({"x_mm": num, "y_mm": num, "z_mm": num}),
                 lambda a: self.move_to(**a), physical=True, surface=SURFACE),
            Tool("set_work_zero", "Make the tool's current position the work origin on the named axes.",
                 obj({"axes": {"type": "array", "items": {"type": "string", "enum": ["x", "y", "z"]},
                               "minItems": 1}}, ["axes"]),
                 lambda a: self.set_work_zero(**a), physical=True, surface=SURFACE),
            Tool("cut_pocket", "Mill a pocket with the loaded end mill: a circle (width_mm = diameter), a rectangle "
                 "(width_mm x height_mm) or a star standing inside a circle, centred at center_x_mm/center_y_mm in "
                 "work coordinates, depth_mm deep below the stock top. Generates the toolpath, preflights it, and "
                 "starts it; returns at once (the job runs for a while). Never guess a size or depth.",
                 obj({"shape": {"type": "string", "enum": ["circle", "rectangle", "star"]},
                      "width_mm": num, "height_mm": num, "depth_mm": num,
                      "center_x_mm": num, "center_y_mm": num, "stepdown_mm": num, "feed_mm_min": num},
                     ["shape", "width_mm", "depth_mm"]),
                 lambda a: self.cut_pocket(**a), physical=True, surface=SURFACE),
            Tool("change_tool", "Put tool `number` from the machine's tool table in the spindle (an M6 tool "
                 "change: the spindle stops, the tool goes in, its length offset is set). get_robot_state lists "
                 "the table and the tool in the spindle. Only when the operator asked for that tool or a cut "
                 "they asked for needs it.",
                 obj({"number": {"type": "integer", "minimum": 1, "maximum": 255}}, ["number"]),
                 lambda a: self.change_tool(**a), physical=True, surface=SURFACE),
            Tool("run_gcode", "Run a G-code program the operator supplied. It is preflighted and refused if it "
                 "would error, cut with the spindle off, leave the travel or cut through the stock.",
                 obj({"program": {"type": "string"}}, ["program"]),
                 lambda a: self.run_gcode(**a), physical=True, surface=SURFACE),
            Tool("pause_job", "Feed hold: stop motion where it is, keeping the job and the position.",
                 obj({}), lambda a: self.pause(), physical=False, surface=SURFACE),
            Tool("resume_job", "Resume a paused job. Only on an explicit resume from the operator.",
                 obj({}), lambda a: self.resume(), physical=True, surface=SURFACE),
            Tool("stop_robot", "Stop now: halt motion, cancel the job, spindle off. Always allowed.",
                 obj({}), lambda a: self.stop(), physical=False, surface=SURFACE),
        ]


def main_task(bridge: CncBridge) -> str:
    t = bridge.machine.axes
    return (
        f"You operate a 3-axis CNC router ({bridge.machine.name}, travel X{t['x'].travel:g} x Y{t['y'].travel:g} x "
        f"Z{t['z'].travel:g} mm) through its GRBL controller. The tool table is "
        + "; ".join(f"T{tool.number} {tool.name or tool.shape}" for tool in bridge.tool_table.values())
        + ". get_robot_state reports which tool is in the spindle; pockets need a flat end mill. "
        "All lengths are millimetres and work coordinates: X0 Y0 is the work origin, Z0 is the top of the stock"
        + (f" ({bridge.stock_thickness:g} mm thick)" if bridge.stock_thickness else "") + ". "
        "Do exactly what the operator asked, and nothing they did not ask for. Never guess a size, depth or "
        "position: if one is missing, ask. Questions, negations, quoted text and hypotheticals must not move "
        "the machine. Use cut_pocket for pockets; run_gcode only for G-code the operator gave you. Jobs run "
        "asynchronously: report that a job started, and use get_robot_state for progress; never claim a job "
        "finished unless get_robot_state shows it done. Use stop_robot for stop or cancel, pause_job for pause, "
        "resume_job only on an explicit resume. Refusals from the machine checks (travel, cut_depth, preflight, "
        "busy, plunge) are final: explain them, do not work around them. This is a simulated machine; do not "
        "claim anything about a physical one."
    )


def setup_relay(bridge: CncBridge, http_port: int):
    """The OmniLink relay, or None without an OmniKey (then /prompt answers 401)."""
    from omnisim_bridges import relay as relay_mod
    from omnisim_bridges import profile_sync
    if not relay_mod.is_enabled():
        return None
    agent_name = profile_sync.agent_name_for(bridge.robot_id)
    rl = relay_mod.OmniLinkRelay(omni_key=relay_mod.get_omni_key(), agent_name=agent_name,
                                 main_task=main_task(bridge), tools=bridge.tools(), surface=SURFACE)
    if profile_sync.is_enabled():
        from omnisim_bridges.bridge_base import profile_extras
        profile_sync.ensure_profile(client=rl._client, agent_name=agent_name, main_task=main_task(bridge),
                                    tool_defs=rl.tool_defs, engine=rl.engine,
                                    tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
                                    **profile_extras(bridge, http_port=http_port, surface=SURFACE))
    return rl


def make_handler(bridge: CncBridge):
    registry = {t.name: t for t in bridge.tools()}

    class Handler(BaseHTTPRequestHandler):
        server_version = "OmniSimCncBridge/1"

        def log_message(self, fmt, *args):
            pass

        def _json(self, code: int, payload: Any) -> None:
            body = json.dumps(payload, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except json.JSONDecodeError:
                return {}

        def do_GET(self):
            if self.path in ("/get_robot_state", "/state"):
                return self._json(200, bridge.get_state())
            if self.path == "/healthz":
                return self._json(200, {"ok": True, "connected": bridge.link.connected})
            if self.path == "/chat_config":
                cfg = access.chat_config(bridge.relay)
                cfg.update({"robot": bridge.robot_id, "display_name": f"{bridge.model} (CNC)",
                            "tagline": "OmniLink CNC: tell it what to machine",
                            "suggestions": ["What's your status?", "Cut a 60 mm circle pocket 2 mm deep",
                                            "Pause", "Resume", "Stop"],
                            "voice": False, "has_ik": False, "has_gripper": False})
                return self._json(200, cfg)
            if self.path == "/chat":
                sys.path.insert(0, str(ROOT / "projects/samples/demos/controllers/omnilink_arm_bridge"))
                from _chat_page import CHAT_HTML
                body = CHAT_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            return self._json(404, {"error": "not found"})

        def do_POST(self):
            body = self._body()
            if self.path == "/stop_robot":
                return self._json(200, bridge.stop())
            if self.path == "/get_robot_state":
                return self._json(200, bridge.get_state())
            if self.path == "/tool":
                name = str(body.get("tool") or body.get("name") or "")
                tool = registry.get(name)
                code, env = serve_tool(name, body, tool.dispatch if tool else (lambda a: {}),
                                       surface=SURFACE, registered=tool is not None, bridge=bridge)
                return self._json(code, env)
            if self.path == "/prompt":
                text = str(body.get("text") or body.get("prompt") or "").strip()
                if not text:
                    return self._json(400, {"error": "empty prompt"})
                if bridge.relay is None:
                    err = access.connection_error()
                    return self._json(401 if err.get("error") == "omnikey_required" else 503, err)
                bridge.turn_text = text
                try:
                    fast = route.short_circuit(bridge, text, PARSER_SURFACE)
                    if fast is not None:
                        return self._json(200, route.reply_payload(fast.get("agent", ""), fast.get("tools"),
                                                                   via="parser"))
                    return self._json(200, bridge.relay.dispatch_sync(text, timeout_s=90.0))
                finally:
                    bridge.turn_text = ""
            return self._json(404, {"error": "not found"})

    return Handler


def serve(bridge: CncBridge, port: int = 8765, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer((host, port), make_handler(bridge))
    threading.Thread(target=httpd.serve_forever, name="cnc-bridge-http", daemon=True).start()
    return httpd


def start(grbl: str, machine_path: str, port: int = 8765, tool_d_mm: float = 6.35,
          stock_thickness_mm: Optional[float] = None, zero_above_stock_mm: Optional[float] = None,
          name: str = "cnc", tools_path: Optional[str] = None) -> CncBridge:
    """Connect to GRBL, register the tools with the gate, start the relay and HTTP."""
    host, _, gport = grbl.partition(":")
    link = GrblLink(host or "127.0.0.1", int(gport or 2323))
    table = tools_mod.load(Path(tools_path)) if tools_path else None
    bridge = CncBridge(link, machine_mod.load(machine_path), tool_d_mm, stock_thickness_mm, name, table)
    # Register even without a relay, so /tool calls are schema-checked by the gate too.
    gate_mod.register_tools(bridge.tools(), surface=SURFACE)
    if zero_above_stock_mm is not None:
        # The world places the tool over the stock's centre at power-up: make that the work
        # origin, with Z0 on the stock top, as an operator would by touching off.
        link.command(f"G10 L20 P1 X0 Y0 Z{zero_above_stock_mm:.3f}")
    try:
        bridge.relay = setup_relay(bridge, port)
    except Exception as exc:  # the relay failing must not take the machine's stop with it
        print(f"[cnc_bridge] OmniLink relay unavailable: {exc}", flush=True)
        bridge.relay = None
    serve(bridge, port)
    state = "ON" if bridge.relay is not None else "OFF (no OmniKey: /prompt answers 401; /tool and stop work)"
    print(f"[cnc_bridge] {bridge.model} on http://127.0.0.1:{port} (chat: /chat), GRBL {grbl}, relay {state}",
          flush=True)
    return bridge


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="OmniLink bridge for a GRBL CNC machine.")
    p.add_argument("--grbl", default="127.0.0.1:2323", help="GRBL stream host:port (the twin, or a board)")
    p.add_argument("--machine", required=True)
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--tool-d", type=float, default=6.35)
    p.add_argument("--stock-thickness", type=float)
    p.add_argument("--zero-above-stock", type=float)
    p.add_argument("--tools", help="tool table JSON (tool_table.py)")
    args = p.parse_args(argv)
    start(args.grbl, args.machine, args.port, args.tool_d, args.stock_thickness, args.zero_above_stock,
          tools_path=args.tools)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
