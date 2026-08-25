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

"""Counting closed loops in a session transcript.

A **cycle** is ``observe -> change -> observe``: the agent obtained a
measurement of the world, then modified its artifact, then obtained another
measurement. Two of those means it saw a result, acted on it, and saw the
consequence -- which is the whole of what L2 claims to test.

What this deliberately does NOT do
----------------------------------

It does not read the agent's prose. An agent saying *"the error is now 4 cm"*
is not evidence; the tool result that produced the number is. Every count here
comes from the recorded tool calls and their outputs, never from the narration
around them, for the same reason the physical clauses come from the recorder
and not from the agent's README.

It also does not credit a re-run of an unchanged artifact. Running the same
world twice is two observations and no cycle, and L2.5's causality clause is
the backstop for the case where the numbers never move.

Column-neutral by construction
------------------------------

An "observation" is any tool call whose *output* carries the shape of a
measurement. It does not matter whether the agent got there through an HTTP
service, a Python REPL, a log file or a print statement -- and it must not,
because the whole comparison is about what that observation COSTS on each
column, not about which mechanism we prefer.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

#: A number that looks like a measurement of this task: a distance in metres
#: or a time in seconds, written the way tools and prints actually write them.
_NUM = r"[-+]?\d+\.\d+|[-+]?\d+"

_MEASURE_PATTERNS = (
    re.compile(r"(?:final|error|err|dist(?:ance)?)[^\n]{0,24}?(%s)" % _NUM,
               re.I),
    re.compile(r"(?:settle|settled|elapsed|took|duration)[^\n]{0,24}?(%s)"
               % _NUM, re.I),
    re.compile(r"(%s)\s*(?:m|metres|meters)\b" % _NUM, re.I),
    re.compile(r"(%s)\s*(?:s|sec|seconds)\b" % _NUM, re.I),
)

#: Tools that CHANGE the artifact. A change is what separates two observations
#: into a cycle.
_MUTATORS = {"write", "edit", "notebookedit", "multiedit"}

#: Tools that can carry an observation in their OUTPUT. Deliberately broad --
#: a shell is how most columns will measure, and excluding it would build our
#: own surface into the definition of "observing".
_OBSERVERS = {"bash", "powershell", "read", "grep", "webfetch"}


def _blocks(rec):
    msg = rec.get("message") or {}
    content = msg.get("content")
    return content if isinstance(content, list) else []


def _text_of(result):
    if isinstance(result, str):
        return result
    if isinstance(result, list):
        return " ".join(_text_of(x) for x in result)
    if isinstance(result, dict):
        if "text" in result:
            return str(result.get("text") or "")
        return " ".join(str(v) for v in result.values()
                        if isinstance(v, (str, int, float)))
    return ""


def measurements_in(text, *, limit=6):
    """Numbers in a tool output that look like this task's measurements."""
    out = []
    for pat in _MEASURE_PATTERNS:
        for m in pat.finditer(text or ""):
            try:
                out.append(float(m.group(1)))
            except (TypeError, ValueError):
                continue
            if len(out) >= limit:
                return out
    return out


def read_events(transcript_path):
    """``[(kind, payload)]`` in order: ('observe', [numbers]) / ('change', path).

    Tolerant of a partial file: a killed session's transcript is truncated
    mid-line, and half a record is not a reason to score the whole cell zero.
    """
    events = []
    pending_tool = {}
    path = Path(transcript_path)
    if not path.is_file():
        return events
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        for blk in _blocks(rec):
            if not isinstance(blk, dict):
                continue
            if blk.get("type") == "tool_use":
                name = (blk.get("name") or "").lower()
                if name in _MUTATORS:
                    events.append(("change",
                                   (blk.get("input") or {}).get("file_path")))
                elif name in _OBSERVERS:
                    pending_tool[blk.get("id")] = name
            elif blk.get("type") == "tool_result":
                tid = blk.get("tool_use_id")
                if tid in pending_tool:
                    nums = measurements_in(_text_of(blk.get("content")))
                    if nums:
                        events.append(("observe", nums))
                    pending_tool.pop(tid, None)
    return events


def count_cycles(events):
    """``(cycles, measurements)`` -- observe -> change -> observe, in order."""
    cycles = 0
    measurements = []
    state = None            # None | 'observed' | 'changed'
    for kind, payload in events:
        if kind == "observe":
            measurements.extend(payload)
            if state == "changed":
                cycles += 1
                state = "observed"
            else:
                state = "observed"
        elif kind == "change":
            if state == "observed":
                state = "changed"
    return cycles, measurements


def analyse(transcript_path):
    """``{cycles, measurements, events}`` for one session."""
    events = read_events(transcript_path)
    cycles, measurements = count_cycles(events)
    return {"cycles": cycles, "measurements": measurements,
            "events": len(events),
            "rule": "a cycle is observe->change->observe, counted from "
                    "recorded tool calls and their outputs only; the agent's "
                    "prose is never evidence"}
