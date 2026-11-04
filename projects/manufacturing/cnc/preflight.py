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

"""Judge a whole G-code job against a machine before anything moves.

The job goes through the same interpreter the virtual controller executes, so
"preflight passed" and "the twin ran it" mean the same reading of the file.

Reported: GRBL errors per line, soft-envelope violations (needs the work
offset), bounds in machine and work coordinates, feeds the machine will
silently clamp, feed moves with the spindle off, rapids below Z0 (assumes the
common CAM convention Z0 = stock top), spindle speeds outside $30/$31, and an
estimated run time.

The time estimate plans the whole job at once. GRBL looks ahead only ~15
blocks, so on dense short segments a real machine can be slower than this.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from gcode import GcodeError, Interpreter  # noqa: E402
import machine as machine_mod                # noqa: E402
from machine import AXES, Machine            # noqa: E402
from motion import Item, Planner, limit_by_axis  # noqa: E402
import tool_table as tools_mod                    # noqa: E402
from tool_table import Tool, default_tools, tool_change_in  # noqa: E402

MAX_LISTED = 25


def _trapezoid_time(length: float, v0: float, v1: float, vn: float, a: float) -> float:
    if a <= 0 or vn <= 0:
        return 0.0
    vn = max(vn, v0, v1)
    d_acc = (vn * vn - v0 * v0) / (2 * a)
    d_dec = (vn * vn - v1 * v1) / (2 * a)
    if d_acc + d_dec <= length:
        return (vn - v0) / a + (vn - v1) / a + (length - d_acc - d_dec) / vn
    # No cruise: peak speed where the two ramps meet.
    vp = math.sqrt(max(0.0, (2 * a * length + v0 * v0 + v1 * v1) / 2))
    return max(0.0, (vp - v0) / a) + max(0.0, (vp - v1) / a)


def preflight(m: Machine, program: str, wco: Optional[Sequence[float]] = None,
              start_mpos: Optional[Sequence[float]] = None, tools: Optional[Dict[int, Tool]] = None,
              tlo_mm: float = 0.0) -> dict:
    it = Interpreter(arc_tolerance=float(m.settings[12]), spindle_max=float(m.settings[30]),
                     spindle_min=float(m.settings[31]))
    it.position = list(start_mpos) if start_mpos is not None else [m.axes[a].home_mpos for a in AXES]
    if wco is not None:
        it.wcs_offsets[0] = list(wco)
    it.tlo = float(tlo_mm)        # the controller's G43.1 in force (`$#` [TLO:])
    planner = Planner(m, capacity=10 ** 9)
    planner.reset(it.position)

    errors, violations, warnings = [], [], {"feed_clamped": [], "feed_with_spindle_off": [],
                                            "rapid_below_z0": [], "spindle_out_of_range": []}
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    wlo, whi = [math.inf] * 3, [-math.inf] * 3       # work coordinates, tool offset removed
    items: List[Item] = []
    lines = program.splitlines()
    ended = False
    max_rates = [m.axes[a].max_rate for a in AXES]
    tools = tools or default_tools()
    tool_changes = []
    for n, raw in enumerate(lines, 1):
        # M6 the way the sender runs it (grbl_link): the rest of the line goes to GRBL,
        # then the spindle stops, the tool goes in and G43.1 sets its length offset.
        send, change, _ = tool_change_in(raw)
        if len(send.strip()) + 1 > 127:
            # The sender (grbl_link) refuses it: GRBL's receive buffer holds 127 bytes and a newline.
            errors.append({"line": n, "code": "line_too_long", "detail": f"{len(send.strip()) + 1} bytes > 127",
                           "text": raw.strip()[:80]})
            continue
        try:
            blocks = it.execute(send, source_line=n)
        except GcodeError as exc:
            errors.append({"line": n, "code": exc.code, "detail": exc.detail, "text": raw.strip()[:80]})
            continue
        if change:
            if it.tool not in tools:
                errors.append({"line": n, "code": 38, "detail": f"T{it.tool} is not in the tool table",
                               "text": raw.strip()[:80]})
            else:
                it.modal.spindle = "off"
                it.tlo = tools[it.tool].stickout_mm
                tool_changes.append({"line": n, "tool": tools[it.tool].summary()})
        for b in blocks:
            if b.kind == "end":
                ended = True
            if b.kind == "spindle" and b.spindle[0] != "off" and it.speed:
                if not m.settings[31] <= it.speed <= m.settings[30]:
                    warnings["spindle_out_of_range"].append({"line": n, "S": it.speed})
            if b.kind != "line":
                planner.queue.append(Item(b))
                continue
            offset_now = it.work_offset()
            for i, a in enumerate(AXES):
                v = b.target[i]
                lo[i], hi[i] = min(lo[i], v), max(hi[i], v)
                wlo[i], whi[i] = min(wlo[i], v - offset_now[i]), max(whi[i], v - offset_now[i])
                if wco is not None and not m.axes[a].inside(v):
                    violations.append({"line": n, "axis": a, "mpos": round(v, 4),
                                       "envelope": [m.axes[a].env_min, m.axes[a].env_max],
                                       "text": raw.strip()[:80]})
            prev_len = len(planner.queue)
            planner.push(b, recalc=False)     # one backward pass at the end, not per line
            if len(planner.queue) > prev_len:
                item = planner.queue[-1]
                if not b.rapid:
                    achievable = limit_by_axis(max_rates, item.unit)
                    if b.feed > achievable * 1.0001:
                        warnings["feed_clamped"].append({"line": n, "requested": round(b.feed, 1),
                                                         "achievable": round(achievable, 1)})
                    if b.spindle[0] == "off" and not m.settings[32]:
                        warnings["feed_with_spindle_off"].append({"line": n})
                else:
                    work_z = b.target[2] - it.work_offset()[2]
                    if work_z < -1e-6 and item.unit[2] < 0:
                        warnings["rapid_below_z0"].append({"line": n, "work_z": round(work_z, 4)})
    planner.recalculate()
    seconds = _estimate(planner)
    # Without the work offset the job cannot be placed on the machine: work bounds only.
    bounds = {a: {"machine": [round(lo[i], 4), round(hi[i], 4)] if wco is not None and lo[i] <= hi[i] else None,
                  "work": [round(wlo[i], 4), round(whi[i], 4)] if wlo[i] <= whi[i] else None}
              for i, a in enumerate(AXES)}
    listed = {k: v[:MAX_LISTED] for k, v in warnings.items()}
    counts = {k: len(v) for k, v in warnings.items()}
    verdict = "fail" if errors or violations else ("warn" if any(counts.values()) or not ended else "pass")
    return {
        "machine": m.name, "lines": len(lines), "verdict": verdict,
        "envelope_checked": wco is not None,
        "errors": errors[:MAX_LISTED], "error_count": len(errors),
        "envelope_violations": violations[:MAX_LISTED], "violation_count": len(violations),
        "warnings": listed, "warning_counts": counts, "program_end": ended,
        "bounds": bounds, "estimated_seconds": round(seconds, 1), "tool_changes": tool_changes,
        "notes": [] if wco is not None else ["no work offset given: envelope not checked"],
    }


def _estimate(planner: Planner) -> float:
    items = list(planner.queue)
    total = 0.0
    entry = 0.0
    for idx, it in enumerate(items):
        if not it.is_line:
            if it.block.kind == "dwell":
                total += it.block.seconds
            entry = 0.0
            continue
        nxt = items[idx + 1] if idx + 1 < len(items) else None
        exit_v = nxt.entry if nxt is not None and nxt.is_line else 0.0
        # Forward pass: the entry actually reachable from the previous block.
        entry = min(it.entry, entry) if idx else 0.0
        reach = math.sqrt(entry * entry + 2 * it.accel * it.length)
        exit_v = min(exit_v, reach)
        total += _trapezoid_time(it.length, entry, exit_v, it.nominal(1, 1), it.accel)
        entry = exit_v
    return total


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Preflight a G-code job against a CNC machine config.")
    p.add_argument("program", help="G-code file")
    p.add_argument("--machine", required=True, help="GRBL $$ dump or FluidNC config.yaml")
    p.add_argument("--wco", help="G54 work offset in machine mm, e.g. -250,-150,-40")
    p.add_argument("--tools", help="tool table JSON (tool_table.py); default: one 6.35 mm flat end mill")
    args = p.parse_args(argv)
    m = machine_mod.load(args.machine)
    wco = [float(v) for v in args.wco.split(",")] if args.wco else None
    report = preflight(m, Path(args.program).read_text(encoding="utf-8", errors="replace"), wco,
                       tools=tools_mod.load(Path(args.tools)) if args.tools else None)
    print(json.dumps(report, indent=2))
    return 0 if report["verdict"] != "fail" else 1


if __name__ == "__main__":
    raise SystemExit(main())
