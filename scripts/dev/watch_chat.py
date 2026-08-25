#!/usr/bin/env python3
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

"""Live tail of the OmniLink chat transcript -- what the robots were asked,
and what they answered.

The bridges (`omnilink_arm_bridge`, `omnilink_mobile_bridge`) append one JSON
line per chat turn to `$OMNILINK_TRANSCRIPT` when that variable is set. This
pretty-prints those turns as they land.

    set OMNILINK_TRANSCRIPT=%TEMP%\\omnilink_chat.jsonl     # before launching
    python scripts/dev/watch_chat.py                        # in another shell

Why the tool calls are printed under each prompt: a reply cannot be judged on
its own. "I moved it a metre" is either true or invented depending on what the
tools MEASURED, so each turn shows the calls it made, their arguments, and the
summary each one returned -- which is what separates "said 1 m, the tool
measured 0.998" from "said 1 m, called nothing at all". A turn that answers a
question about the robot's own state with NO tool call under it is exactly the
answer worth auditing.

    --follow      keep watching (default; --no-follow dumps and exits)
    --robot ID    only this robot
    --last N      how many existing turns to show first (default 10, 0 = none)
    --json        print the raw JSON lines instead of formatting them

Handles the file not existing yet (it waits), a turn half-written as it reads
(it buffers until the newline), and the file being truncated or replaced
between runs (it reopens from the start).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional

POLL_S = 0.25


# ── terminal dressing ────────────────────────────────────────────────

def _supports(text: str) -> bool:
    enc = getattr(sys.stdout, "encoding", None) or "ascii"
    try:
        text.encode(enc)
        return True
    except Exception:
        return False


_FANCY = _supports("─│›")
RULE = "─" if _FANCY else "-"
ARROW = "›" if _FANCY else ">"
GEAR = "→" if _FANCY else "->"


class C:
    """ANSI colours, blanked when the output is not a terminal."""
    on = False
    DIM = "\033[2m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    CYAN = "\033[36m"
    OFF = "\033[0m"

    @classmethod
    def p(cls, colour: str, text: str) -> str:
        return (colour + text + cls.OFF) if cls.on else text


# ── formatting ───────────────────────────────────────────────────────

ELLIPSIS = "…" if _FANCY else "..."


def _short(value: Any, limit: int) -> str:
    text = value if isinstance(value, str) else json.dumps(value, default=str,
                                                           ensure_ascii=False)
    text = " ".join(str(text).split())
    if len(text) <= limit:
        return text
    return text[: max(1, limit - len(ELLIPSIS))] + ELLIPSIS


def _clock(ts: Optional[str]) -> str:
    # "2026-07-29T14:03:11.482Z" -> "14:03:11"
    if not ts or "T" not in ts:
        return "??:??:??"
    return ts.split("T", 1)[1][:8]


def _wrap(text: str, indent: str, width: int = 96) -> str:
    out: List[str] = []
    for para in str(text).splitlines() or [""]:
        line = ""
        for word in para.split():
            if line and len(line) + 1 + len(word) > width:
                out.append(indent + line)
                line = word
            else:
                line = (line + " " + word) if line else word
        out.append(indent + line)
    return "\n".join(out)


def _fmt_tool(call: Dict[str, Any]) -> str:
    name = call.get("name") or "?"
    ok = call.get("ok")
    mark = C.p(C.GREEN, "ok") if ok else C.p(C.RED, "ERR")
    args = call.get("args")
    shown = ""
    if isinstance(args, dict) and args:
        shown = "(" + _short(args, 64) + ")"
    elif args is None:
        shown = C.p(C.DIM, "(args not recorded)")
    head = "    %s %s%s %s %s" % (C.p(C.YELLOW, "tool"), name, shown, GEAR, mark)
    bits = []
    summary = call.get("summary")
    if summary:
        bits.append(_short(summary, 150))
    for key in ("accepted", "moved", "error"):
        if key in call:
            bits.append("%s=%s" % (key, _short(call[key], 60)))
    if bits:
        head += "\n" + _wrap(" | ".join(bits), "         ")
    return head


def _fmt_usage(usage: Any) -> Optional[str]:
    if not isinstance(usage, dict):
        return None
    text = usage.get("text")
    if text:
        return _short(text, 150)
    bits = [("%s=%s" % (k, v)) for k, v in usage.items()
            if isinstance(v, (int, float)) and v]
    return ", ".join(bits[:6]) if bits else None


def render(rec: Dict[str, Any]) -> str:
    robot = rec.get("robot") or "?"
    engine = rec.get("engine")
    mode = rec.get("mode") or "?"
    mode_s = "%s:%s" % (mode, engine) if engine else mode
    meta = [
        "%s [%s]" % (C.p(C.BOLD, str(robot)), rec.get("bridge") or "?"),
        "via %s" % (rec.get("source") or "?"),
        mode_s,
    ]
    if rec.get("latency_s") is not None:
        meta.append("%.2fs" % rec["latency_s"])
    if rec.get("sim_t") is not None:
        meta.append("sim t=%.1f" % rec["sim_t"])
    if rec.get("turn") is not None:
        meta.append("#%s" % rec["turn"])

    lines = [C.p(C.DIM, RULE * 78),
             "%s  %s" % (C.p(C.DIM, _clock(rec.get("ts"))),
                         C.p(C.DIM, " · ".join(meta) if _FANCY
                             else " | ".join(meta)))]
    lines.append(_wrap("%s %s" % (C.p(C.CYAN, "you " + ARROW),
                                  rec.get("prompt") or ""), "", 96))

    tools = rec.get("tools") or []
    for call in tools:
        if isinstance(call, dict):
            lines.append(_fmt_tool(call))
    if not tools:
        # The auditable case: an answer with nothing measured behind it.
        lines.append(C.p(C.DIM, "    (no tool calls this turn)"))

    reply = rec.get("reply") or ""
    lines.append(_wrap("%s %s" % (C.p(C.BLUE, "bot " + ARROW), reply), "", 96))

    if rec.get("error"):
        lines.append(C.p(C.RED, "    error: " + _short(rec["error"], 200)))
    usage = _fmt_usage(rec.get("usage"))
    if usage:
        lines.append(C.p(C.DIM, "    usage: " + usage))
    return "\n".join(lines)


# ── tail ─────────────────────────────────────────────────────────────

class Tail:
    """Byte-accurate follower that only ever yields COMPLETE lines.

    A turn is written as one line, but the reader can arrive mid-write, so
    anything after the last newline is held back until its newline shows up.
    """

    def __init__(self, path: str) -> None:
        self.path = path
        self._fh = None
        self._buf = b""
        self._pos = 0

    def _ensure_open(self) -> bool:
        if self._fh is not None:
            return True
        try:
            self._fh = open(self.path, "rb")
            self._fh.seek(self._pos)
            return True
        except OSError:
            return False

    def _reset(self) -> None:
        try:
            if self._fh is not None:
                self._fh.close()
        except OSError:
            pass
        self._fh = None
        self._buf = b""
        self._pos = 0

    def read_lines(self) -> List[str]:
        """Complete lines appended since the last call."""
        if not self._ensure_open():
            self._reset()
            return []
        try:
            size = os.path.getsize(self.path)
        except OSError:
            self._reset()
            return []
        if size < self._pos:
            # Truncated or replaced (a fresh run of the demo). Start over.
            self._reset()
            if not self._ensure_open():
                return []
        try:
            chunk = self._fh.read()
        except OSError:
            self._reset()
            return []
        if not chunk:
            return []
        self._pos += len(chunk)
        self._buf += chunk
        if b"\n" not in self._buf:
            return []                      # a turn is still being written
        whole, _, self._buf = self._buf.rpartition(b"\n")
        return [ln for ln in whole.decode("utf-8", "replace").splitlines() if ln.strip()]

def parse(line: str) -> Optional[Dict[str, Any]]:
    try:
        rec = json.loads(line)
        return rec if isinstance(rec, dict) else None
    except Exception:
        return None


def keep(rec: Dict[str, Any], robot: Optional[str]) -> bool:
    if not robot:
        return True
    return str(rec.get("robot") or "").lower() == robot.lower()


def emit(line: str, rec: Optional[Dict[str, Any]], as_json: bool) -> None:
    if as_json:
        print(line, flush=True)
        return
    if rec is None:
        print(C.p(C.RED, "  [unparseable line] " + _short(line, 160)), flush=True)
        return
    print(render(rec), flush=True)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Live tail of the OmniLink chat transcript.")
    ap.add_argument("path", nargs="?", default=None,
                    help="transcript file (default: $OMNILINK_TRANSCRIPT)")
    ap.add_argument("--follow", dest="follow", action="store_true", default=True,
                    help="keep watching for new turns (default)")
    ap.add_argument("--no-follow", dest="follow", action="store_false",
                    help="print what is there and exit")
    ap.add_argument("--robot", default=None, help="only show this robot id")
    ap.add_argument("--last", type=int, default=10,
                    help="existing turns to show first (default 10, 0 = none)")
    ap.add_argument("--json", action="store_true",
                    help="print raw JSON lines instead of formatting them")
    ap.add_argument("--no-color", action="store_true", help="disable colour")
    args = ap.parse_args(argv)

    path = args.path or os.environ.get("OMNILINK_TRANSCRIPT", "").strip()
    if not path:
        print("No transcript path. Pass one, or set OMNILINK_TRANSCRIPT "
              "(the same value the bridge was launched with).", file=sys.stderr)
        return 2

    C.on = (not args.no_color) and sys.stdout.isatty() \
        and os.environ.get("TERM") != "dumb"

    if not args.json:
        print(C.p(C.DIM, "watching %s%s" % (
            path, (" [robot=%s]" % args.robot) if args.robot else "")),
            flush=True)

    tail = Tail(path)

    # Where history ends and live begins. A file that did not exist when the
    # watcher started has no history: every turn in it happened while we were
    # watching, so `--last` must not filter it away. Getting this wrong loses
    # the first turn of a demo, which is the one most likely to be watched.
    existed_at_start = os.path.exists(path)

    # Wait for the file. The bridge only creates it on the FIRST chat turn,
    # so "not there yet" is the normal state at demo start, not an error.
    announced = False
    while not os.path.exists(path):
        if not args.follow:
            print(C.p(C.YELLOW, "no transcript at %s yet" % path), file=sys.stderr)
            return 1
        if not announced:
            print(C.p(C.DIM, "waiting for the first chat turn" + ELLIPSIS),
                  flush=True)
            announced = True
        try:
            time.sleep(POLL_S)
        except KeyboardInterrupt:
            return 0

    # Backlog.
    backlog = [(ln, parse(ln)) for ln in tail.read_lines()]
    backlog = [(ln, r) for ln, r in backlog if r is None or keep(r, args.robot)]
    shown = backlog if not existed_at_start else (
        backlog[-args.last:] if args.last > 0 else [])
    for ln, rec in shown:
        emit(ln, rec, args.json)
    if not args.follow:
        return 0

    try:
        while True:
            for ln in tail.read_lines():
                rec = parse(ln)
                if rec is None or keep(rec, args.robot):
                    emit(ln, rec, args.json)
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
