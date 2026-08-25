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

"""Cell EVIDENCE: the agent's workspace, the live session stream, and a
read-only status view over both.

WHY THIS MODULE EXISTS. The 2026-08-11 ``r1_3arm_20260810`` campaign produced
three cells and zero usable evidence:

* the webots cell was killed after 156.8 s of a 1800 s task budget, published
  ``FAIL / no_artifact`` and recorded ``session_id: null``;
* the omnisim and mujoco cells BLOCKED on "no JSON on stdout", and their
  workspaces -- the only place the agents' actual work existed -- were torn
  down by the ordinary teardown postscript moments later.

So the sole surviving trace of a mujoco run that claimed a complete working
solution was the agent's own PROSE SUMMARY. That is a self-report, not a
measurement, and there is now nothing on the machine that can confirm or
refute it. Three rules follow, and this module implements them:

1. **The workspace is the artifact of record.** It is copied into the cell
   directory on EVERY outcome -- pass, fail, blocked, timeout, crash -- and
   the copy is what a human opens. Exclusions are enumerated in the manifest;
   nothing is ever silently dropped.
2. **The session is METADATA.** Turns, tokens and cost come from the stream;
   the verdict comes from what is on disk.
3. **A running cell must be inspectable.** ``--output-format stream-json``
   writes one NDJSON event per turn/tool call as it happens, so "what is it
   doing right now" is answerable from disk while the session is still alive.
"""

from __future__ import annotations

import fnmatch
import calendar
import json
import os
import shutil
import sys
import threading
import time
from pathlib import Path

# --- workspace preservation ---------------------------------------------------

#: Directory names never copied into the preserved workspace. Build/VCS/venv
#: debris only -- nothing here can be the agent's deliverable, and each is
#: individually capable of dwarfing the real evidence.
EXCLUDE_DIR_NAMES = (
    ".git", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache",
    "node_modules", ".venv", "venv", "env", ".tox", ".idea", ".ipynb_checkpoints",
)

#: File glob patterns never copied. Same rule: derived, never authored.
EXCLUDE_FILE_GLOBS = ("*.pyc", "*.pyo", "*.o", "*.obj", "*.pdb")

#: A single file larger than this is recorded by name + size instead of
#: copied. Sized for the arms we run: an MJCF, a ``.wbt``, a controller and a
#: trajectory plot are all far below it; a recorded ``.mp4``, a downloaded
#: mesh pack or a core dump are above it. RECORDED, never silent.
DEFAULT_MAX_FILE_BYTES = 64 * 1024 * 1024

#: Total preserved bytes. Beyond this the copy STOPS and says so on the
#: manifest (``truncated: true`` + the first path it refused), so a runaway
#: workspace can never fill the results tree unnoticed. The measured arms sit
#: at 53 KB (webots), ~0 (mujoco) and 4.5 MB (omnisim template) + the agent's
#: own work, so this is ~100x headroom rather than a working limit.
DEFAULT_MAX_TOTAL_BYTES = 2 * 1024 * 1024 * 1024


def _is_link(p: Path) -> bool:
    """Junction or symlink. Never recursed into: the omnisim workspace
    junctions ``projects/``, ``lib/``, ``msys64/`` and ``resources/`` straight
    into the real repo, and the webots one symlinks the whole upstream
    R2025a install across a WSL share. Following those copies gigabytes of
    somebody else's tree and none of the agent's work."""
    try:
        return p.is_junction() or p.is_symlink()
    except OSError:
        return False


def _excluded_file(name: str) -> bool:
    return any(fnmatch.fnmatch(name, g) for g in EXCLUDE_FILE_GLOBS)


#: Junctioned dirs worth walking for session writes when ``newer_than`` is
#: given. ``projects/`` is where an OmniSim user's worlds and controllers live,
#: so it is where an agent writes -- and, being a junction, it is exactly what
#: the link-refusing walk cannot see. ``msys64``/``lib``/``resources`` are
#: binary runtime and are not in this list: nothing authored lands there and
#: walking 1.3 GB every 20 s is not free.
DEFAULT_LINK_WINDOW_DIRS = ("projects",)

#: Ceiling on the link-window pass, so a runaway build behind a junction can
#: never fill the results tree. Overflow is RECORDED, never silent.
DEFAULT_MAX_LINK_WINDOW_FILES = 2000


def preserve_workspace(ws, dest, *, max_file_bytes=DEFAULT_MAX_FILE_BYTES,
                       max_total_bytes=DEFAULT_MAX_TOTAL_BYTES,
                       manifest_name="workspace_manifest.json",
                       newer_than=None, link_dirs=(),
                       max_link_window_files=DEFAULT_MAX_LINK_WINDOW_FILES):
    """Copy ``ws`` into ``dest``, returning (and writing) a manifest.

    Idempotent and re-runnable: existing files are overwritten only when the
    source is newer or a different size, so this doubles as the live-mirror
    step (:class:`WorkspaceMirror`) and as the final authoritative copy.

    NEVER raises for an unreadable or locked file -- a cell that lost its
    evidence because one file was held open by a straggler engine is exactly
    the failure mode this module exists to prevent. Every skip is recorded.

    **``newer_than`` + ``link_dirs``: the deliverable is usually BEHIND a
    junction, and until 2026-08-12 the artifact of record did not contain it.**
    Measured on all three cells of the diagnostic round: the manifest read
    ``excluded_links: lib, msys64, projects, resources`` and every one of them
    had authored its world through the ``projects/`` junction, so the preserved
    copy held everything except the thing it existed to hold. Only the
    post-session ``repo_artifacts`` sweep recovered it -- and that never runs
    on a cell the operator kills, which is how an early-stopped cell lost its
    evidence entirely.

    Passing the session's start time and the junction names copies files under
    those links whose mtime is inside the session window -- the agent's writes
    and nothing else (instantiation preserves source mtimes, and 1.7 GB of
    shipped assets are all older). The link itself is STILL recorded as
    excluded: nothing is followed silently, and the copied files are listed
    individually under ``link_window_files``.
    """
    ws = Path(ws)
    dest = Path(dest)
    man = {
        "source": str(ws),
        "dest": str(dest),
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "files": 0,
        "bytes": 0,
        "excluded_links": [],
        "excluded_dirs": [],
        "excluded_files": [],
        "oversize_files": [],
        "unreadable": [],
        "link_window_files": [],
        "link_window_truncated": False,
        "truncated": False,
        "truncated_at": None,
        "source_present": ws.is_dir(),
        "policy": {
            "exclude_dir_names": list(EXCLUDE_DIR_NAMES),
            "exclude_file_globs": list(EXCLUDE_FILE_GLOBS),
            "max_file_bytes": max_file_bytes,
            "max_total_bytes": max_total_bytes,
            "links": ("recorded by target, never followed -- except files "
                      "inside link_window_dirs whose mtime is after "
                      "link_window_newer_than, which ARE copied and listed"),
            "link_window_dirs": list(link_dirs or ()),
            "link_window_newer_than": newer_than,
        },
    }
    if not ws.is_dir():
        man["note"] = ("the workspace directory does not exist; nothing to "
                       "preserve (it was torn down, or never staged)")
        _write_manifest(dest, manifest_name, man)
        return man

    dest.mkdir(parents=True, exist_ok=True)
    total = 0
    for dirpath, dirnames, filenames in os.walk(ws):
        here = Path(dirpath)
        keep = []
        for d in dirnames:
            p = here / d
            rel = str(p.relative_to(ws)).replace("\\", "/")
            if _is_link(p):
                try:
                    target = os.readlink(p)
                except OSError:
                    target = "<unreadable>"
                man["excluded_links"].append({"path": rel, "target": str(target),
                                              "reason": "junction/symlink"})
                continue
            if d in EXCLUDE_DIR_NAMES:
                man["excluded_dirs"].append({"path": rel, "reason": "junk dir"})
                continue
            keep.append(d)
        dirnames[:] = keep

        rel_dir = here.relative_to(ws)
        out_dir = dest / rel_dir
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            man["unreadable"].append({"path": str(rel_dir).replace("\\", "/"),
                                      "error": repr(exc)})
            continue

        for f in filenames:
            src = here / f
            rel = str(src.relative_to(ws)).replace("\\", "/")
            if _is_link(src):
                try:
                    target = os.readlink(src)
                except OSError:
                    target = "<unreadable>"
                man["excluded_links"].append({"path": rel, "target": str(target),
                                              "reason": "symlink"})
                continue
            if _excluded_file(f):
                man["excluded_files"].append({"path": rel, "reason": "junk file"})
                continue
            try:
                st = src.stat()
            except OSError as exc:
                man["unreadable"].append({"path": rel, "error": repr(exc)})
                continue
            if st.st_size > max_file_bytes:
                man["oversize_files"].append({"path": rel, "bytes": st.st_size,
                                              "reason": "> max_file_bytes"})
                continue
            if total + st.st_size > max_total_bytes:
                man["truncated"] = True
                man["truncated_at"] = rel
                _write_manifest(dest, manifest_name, man)
                return man
            out = out_dir / f
            try:
                ost = out.stat()
                if (ost.st_size == st.st_size
                        and ost.st_mtime >= st.st_mtime):
                    man["files"] += 1
                    man["bytes"] += st.st_size
                    total += st.st_size
                    continue                       # already mirrored, unchanged
            except OSError:
                pass
            try:
                shutil.copy2(src, out)
            except OSError as exc:
                man["unreadable"].append({"path": rel, "error": repr(exc)})
                continue
            man["files"] += 1
            man["bytes"] += st.st_size
            total += st.st_size

    if newer_than is not None and link_dirs:
        _preserve_link_window(ws, dest, man, newer_than=float(newer_than),
                              link_dirs=link_dirs,
                              max_files=max_link_window_files,
                              max_file_bytes=max_file_bytes)
    _write_manifest(dest, manifest_name, man)
    return man


def _preserve_link_window(ws, dest, man, *, newer_than, link_dirs, max_files,
                          max_file_bytes):
    """Copy session-window writes made THROUGH a junction. Never raises."""
    for name in link_dirs:
        src_root = Path(ws) / name
        if not src_root.is_dir():
            continue
        for dirpath, dirnames, filenames in os.walk(src_root):
            dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIR_NAMES]
            for f in filenames:
                if _excluded_file(f):
                    continue
                src = Path(dirpath) / f
                try:
                    st = src.stat()
                except OSError:
                    continue
                if st.st_mtime < newer_than or st.st_size > max_file_bytes:
                    continue
                if len(man["link_window_files"]) >= max_files:
                    man["link_window_truncated"] = True
                    return
                rel = str(src.relative_to(Path(ws))).replace("\\", "/")
                out = Path(dest) / rel
                try:
                    out.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, out)
                except OSError as exc:
                    man["unreadable"].append({"path": rel, "error": repr(exc)})
                    continue
                man["link_window_files"].append(
                    {"path": rel, "bytes": st.st_size, "mtime": st.st_mtime,
                     "reason": "written during the session, behind a junction"})
                man["files"] += 1
                man["bytes"] += st.st_size


def _write_manifest(dest, name, man):
    try:
        Path(dest).mkdir(parents=True, exist_ok=True)
        (Path(dest) / name).write_text(json.dumps(man, indent=2),
                                       encoding="utf-8")
    except OSError:
        pass


class WorkspaceMirror:
    """Refresh ``dest`` from ``ws`` every ``interval_s`` in a daemon thread.

    The point is LIVE visibility: watching files appear in the preserved copy
    is most of the answer to "what is the agent doing right now", and a copy
    taken only at the end tells you nothing while it matters. The mirror is
    best-effort and never fatal; the authoritative copy is the final
    :func:`preserve_workspace` call after the session exits.
    """

    def __init__(self, ws, dest, *, interval_s=20.0, newer_than=None,
                 link_dirs=()):
        self.ws = Path(ws)
        self.dest = Path(dest)
        self.interval_s = float(interval_s)
        # The mirror carries the SAME link window as the final copy. On a
        # killed cell the mirror is the only pass that ever runs, so a window
        # applied only at the end is a window that never applies when it
        # matters (measured: 2/2 killed cells lost their deliverable).
        self.newer_than = newer_than
        self.link_dirs = tuple(link_dirs or ())
        self._stop = threading.Event()
        self._thread = None
        self.passes = 0
        self.errors = []

    def start(self):
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="ws-mirror")
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.wait(self.interval_s):
            try:
                preserve_workspace(self.ws, self.dest,
                                   newer_than=self.newer_than,
                                   link_dirs=self.link_dirs)
                self.passes += 1
            except Exception as exc:                  # noqa: BLE001
                self.errors.append(repr(exc))

    def stop(self, timeout=30.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        return {"passes": self.passes, "errors": self.errors[-5:],
                "interval_s": self.interval_s}


# --- the session stream --------------------------------------------------------

#: Tool-input keys that name WHAT a tool acted on, most specific first. The
#: status view prints one of these per call, so "Write worlds/arena.wbt" is
#: readable at a glance instead of "Write {...}".
_TARGET_KEYS = ("file_path", "path", "notebook_path", "command", "pattern",
                "url", "prompt", "description", "query")


def _tool_target(inp):
    if not isinstance(inp, dict):
        return ""
    for k in _TARGET_KEYS:
        v = inp.get(k)
        if isinstance(v, str) and v.strip():
            return " ".join(v.split())[:140]
    return ""


def read_stream(path):
    """Parse a ``--output-format stream-json`` NDJSON file.

    Safe to call on a file a live session is still appending to: a trailing
    partial line is counted, not fatal. Returns everything the row needs even
    when the session never reached its ``result`` event -- which is exactly
    the case the old single-blob reader turned into "no JSON on stdout" and
    threw the whole cell away for.
    """
    out = {
        "path": str(path), "exists": False, "bytes": 0, "mtime": None,
        "events": 0, "unparsed_lines": 0, "result": None, "init": None,
        "session_id": None, "model": None, "permission_mode": None, "cwd": None,
        "assistant_turns": 0, "tool_calls": 0, "tool_call_log": [],
        "texts": [], "errors": [], "rate_limit_events": 0,
        "thinking_events": 0,
    }
    p = Path(path)
    try:
        st = p.stat()
    except OSError:
        return out
    out["exists"] = True
    out["bytes"] = st.st_size
    out["mtime"] = st.st_mtime
    try:
        text = p.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        out["errors"].append("unreadable: %r" % (exc,))
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except ValueError:
            out["unparsed_lines"] += 1
            continue
        if not isinstance(ev, dict):
            out["unparsed_lines"] += 1
            continue
        out["events"] += 1
        etype = ev.get("type")
        if ev.get("session_id") and not out["session_id"]:
            out["session_id"] = ev["session_id"]
        if etype == "system" and ev.get("subtype") == "init":
            out["init"] = ev
            out["model"] = ev.get("model") or out["model"]
            out["permission_mode"] = ev.get("permissionMode")
            # The session's cwd IS the agent's workspace. Reading it here is
            # what lets --status find the live workspace before the cell
            # report exists -- i.e. during the first minutes of a run, which
            # is exactly when someone asks what it is doing.
            out["cwd"] = ev.get("cwd") or out["cwd"]
        elif etype == "system" and ev.get("subtype") == "thinking_tokens":
            # Extended thinking ticks. Counting them is what distinguishes
            # "reasoning hard, no tool call for three minutes" from "stalled",
            # which from tool calls alone look identical.
            out["thinking_events"] += 1
        elif etype == "rate_limit_event":
            out["rate_limit_events"] += 1
        elif etype == "assistant":
            out["assistant_turns"] += 1
            msg = ev.get("message") or {}
            if isinstance(msg, dict):
                if msg.get("model"):
                    out["model"] = msg["model"]
                for b in (msg.get("content") or []):
                    if not isinstance(b, dict):
                        continue
                    if b.get("type") == "tool_use":
                        out["tool_calls"] += 1
                        out["tool_call_log"].append(
                            {"n": out["tool_calls"], "name": b.get("name"),
                             "target": _tool_target(b.get("input"))})
                    elif b.get("type") == "text" and b.get("text", "").strip():
                        out["texts"].append(b["text"].strip())
        elif etype == "result":
            out["result"] = ev
    if out["result"] and out["result"].get("session_id"):
        out["session_id"] = out["result"]["session_id"]
    return out


def stream_summary_for_report(stream):
    """The compact form recorded on the cell report / row (no full events)."""
    return {
        "path": stream.get("path"),
        "exists": stream.get("exists"),
        "bytes": stream.get("bytes"),
        "events": stream.get("events"),
        "unparsed_lines": stream.get("unparsed_lines"),
        "assistant_turns": stream.get("assistant_turns"),
        "tool_calls": stream.get("tool_calls"),
        "session_id": stream.get("session_id"),
        "model": stream.get("model"),
        "permission_mode": stream.get("permission_mode"),
        "reached_result_event": stream.get("result") is not None,
        "rate_limit_events": stream.get("rate_limit_events"),
        "thinking_events": stream.get("thinking_events"),
        "last_tool_calls": stream.get("tool_call_log", [])[-10:],
    }


# --- the status view (read-only) -----------------------------------------------


def _fmt_age(seconds):
    if seconds is None:
        return "?"
    seconds = max(0.0, float(seconds))
    if seconds < 90:
        return "%.0fs" % seconds
    if seconds < 5400:
        return "%.1fm" % (seconds / 60.0)
    return "%.1fh" % (seconds / 3600.0)


def _newest_files(root, *, limit=15, newer_than=None):
    """(path, mtime, size) for the most recently modified real files under
    ``root``. Links are never followed."""
    out = []
    root = Path(root)
    if not root.is_dir():
        return out
    for dirpath, dirnames, filenames in os.walk(root):
        here = Path(dirpath)
        dirnames[:] = [d for d in dirnames
                       if d not in EXCLUDE_DIR_NAMES and not _is_link(here / d)]
        for f in filenames:
            p = here / f
            if _is_link(p) or _excluded_file(f):
                continue
            try:
                st = p.stat()
            except OSError:
                continue
            if newer_than is not None and st.st_mtime < newer_than:
                continue
            out.append((str(p.relative_to(root)).replace("\\", "/"),
                        st.st_mtime, st.st_size))
    out.sort(key=lambda t: t[1], reverse=True)
    return out[:limit]


def cell_status(cell_dir):
    """Read-only snapshot of one cell -- live or finished. Opens no locks,
    sends no signals, writes nothing."""
    cell_dir = Path(cell_dir)
    st = {"cell_dir": str(cell_dir), "now": time.time()}
    rep = {}
    rp = cell_dir / "cell_report.json"
    if rp.is_file():
        try:
            rep = json.loads(rp.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            st["report_error"] = repr(exc)
    st["report_present"] = bool(rep)
    st["sim"] = rep.get("sim")
    st["task"] = rep.get("task")
    st["utc_start"] = rep.get("utc_start")
    st["utc_end"] = rep.get("utc_end")
    st["blocker"] = rep.get("blocker")
    st["row"] = rep.get("row")
    st["liveness"] = rep.get("liveness")
    budget = rep.get("budget") or {}
    st["budget"] = budget
    st["pinned_model"] = rep.get("pinned_model")

    stream = read_stream(cell_dir / "cc_stream.jsonl")
    st["stream"] = stream_summary_for_report(stream)
    st["last_tool_calls"] = stream.get("tool_call_log", [])[-8:]
    st["last_text"] = (stream.get("texts") or [None])[-1]
    st["stream_idle_s"] = (None if not stream.get("mtime")
                           else st["now"] - stream["mtime"])

    # Elapsed. Prefer the report's own start; fall back to when the session
    # stream was created, so a cell that has not written its report yet still
    # reports an elapsed time instead of a question mark.
    started = None
    if rep.get("utc_start"):
        try:
            # calendar.timegm, NOT mktime-minus-time.timezone. ``utc_start`` is
            # a UTC stamp; mktime reads its struct as LOCAL and applies the DST
            # rule for that date, while ``time.timezone`` is the STANDARD
            # offset and never includes DST -- so the correction is wrong by
            # exactly one hour for half the year. Measured on machine
            # 9722d23d12a3 (CEST, UTC+2) on 2026-08-11: a cell 12 s old
            # printed "elapsed 60.2m" against a "session budget 45.0m", which
            # reads as a cell that has already blown its budget and is the
            # kind of thing an operator kills. Display only -- the ENFORCED
            # deadline is armed on time.monotonic() (run_cc_cell:2143) and was
            # never affected -- but --status is the instrument an operator
            # actually reads, and an instrument that says an hour has passed
            # when twelve seconds have is worse than no instrument.
            started = calendar.timegm(time.strptime(rep["utc_start"],
                                                    "%Y-%m-%dT%H:%M:%SZ"))
        except (ValueError, OverflowError):
            started = None
    if started is None:
        try:
            started = (cell_dir / "cc_stream.jsonl").stat().st_ctime
            st["elapsed_source"] = "cc_stream.jsonl ctime (no cell report yet)"
        except OSError:
            started = None
    st["elapsed_s"] = (None if started is None else st["now"] - started)
    st["session_budget_s"] = ((budget.get("session_budgets_s") or [None])[-1])

    # SESSION ELAPSED, WHICH IS NOT CELL ELAPSED.
    #
    # `utc_start` is stamped before the same-task lock is acquired, and a lane
    # queued behind another lane's cell can sit there for tens of minutes. The
    # ENFORCED deadline already excludes that wait (`budget.queue_waits`, and
    # `run_cell` extends `cell_deadline_mono` by it) -- only this display
    # disagreed, and it is the display the operator reads. MEASURED
    # 2026-08-12: `--status` printed "27.0m / 36.0m" for a cell whose session
    # had used EIGHT minutes, because 19 of those minutes were queue. The
    # method the loop now depends on is "kill at plateau", so a 3x overstated
    # clock is not cosmetic -- it kills working cells.
    #
    # Preference order: the recorded session start (exact), else cell start
    # minus the recorded queue waits (good to a second), else unknown.
    queued = sum(float(q.get("waited_s") or 0.0)
                 for q in (budget.get("queue_waits") or []))
    st["queued_s"] = queued
    sess_started = None
    if rep.get("session_started_utc"):
        try:
            sess_started = calendar.timegm(
                time.strptime(rep["session_started_utc"], "%Y-%m-%dT%H:%M:%SZ"))
            st["session_elapsed_source"] = "session_started_utc (recorded)"
        except (ValueError, OverflowError):
            sess_started = None
    if sess_started is None and started is not None:
        sess_started = started + queued
        st["session_elapsed_source"] = ("cell start + recorded queue waits "
                                        "(%.0f s)" % queued)
    st["session_elapsed_s"] = (None if sess_started is None
                               else max(0.0, st["now"] - sess_started))

    # The workspace. LIVE if it still exists (the agent is working in it);
    # otherwise the preserved copy, and the status says which it read. The
    # stream's init event names the session's cwd, so the live workspace is
    # findable from the session's first millisecond -- before the cell report
    # exists, which is when someone actually asks.
    live_ws = rep.get("workspace") or stream.get("cwd")
    preserved = cell_dir / "workspace"
    finished = bool(rep.get("utc_end") or rep.get("blocker"))
    ws_src, ws_kind = None, "none"
    if not finished and live_ws and Path(live_ws).is_dir():
        ws_src, ws_kind = Path(live_ws), "live"
    elif preserved.is_dir():
        # ONCE THE CELL IS OVER THE PRESERVED COPY IS THE RECORD, even when
        # the temp directory still exists. Measured on the verification cell:
        # teardown hit a locked file, deleted most of the tree and marked the
        # rest `.pending_delete`, so a status read of the "live" path reported
        # `recently written files: (none)` for a cell whose 27 preserved files
        # were sitting in the cell dir.
        ws_src, ws_kind = preserved, "preserved copy"
    elif live_ws and Path(live_ws).is_dir():
        ws_src, ws_kind = Path(live_ws), "live (nothing preserved yet)"
    st["workspace_read"] = (str(ws_src) if ws_src else None)
    st["workspace_kind"] = ws_kind
    st["workspace_preserved"] = (str(preserved) if preserved.is_dir() else None)
    st["recent_files"] = ([{"path": p, "age_s": st["now"] - m, "bytes": b}
                           for p, m, b in _newest_files(ws_src)]
                          if ws_src else [])

    # Alive? Never asserted from a process handle (that would mean touching
    # the session); inferred from what the cell has written, and labelled as
    # an inference.
    st["deferred_until_utc"] = rep.get("deferred_until_utc")
    st["rate_limit_deferrals"] = rep.get("rate_limit_deferrals") or []
    if rep.get("utc_end") or rep.get("blocker"):
        st["state"] = "finished"
    elif rep.get("deferred_until_utc"):
        # NOT hung: asleep in a rate-limit backoff, still holding the task
        # lock, and about to retry. These looked identical from here.
        st["state"] = ("deferred (rate/usage limit): backing off until %s"
                       % rep["deferred_until_utc"])
    elif stream.get("exists") and st["stream_idle_s"] is not None \
            and st["stream_idle_s"] < 300:
        st["state"] = "running"
    elif stream.get("exists"):
        st["state"] = "started, but no stream activity in %s" % _fmt_age(
            st["stream_idle_s"])
    elif rep:
        st["state"] = "staging (report written, session not started)"
    else:
        st["state"] = "not started (no cell report and no session stream)"
    return st


def print_status(cell_dir, stream=None):
    """:func:`render_status`, printed so it CANNOT crash on its own content.

    MEASURED 2026-08-11, mid-run, against the very cell this machinery was
    written to make inspectable: the agent's last message contained a Unicode
    minus (U+2212), the Windows console is cp1252, and ``print()`` raised
    ``UnicodeEncodeError``. A status command that dies on the text it is
    quoting is not a status command -- and it fails exactly when the agent
    is doing something interesting enough to say so.
    """
    stream = stream or sys.stdout
    text = render_status(cell_dir)
    enc = getattr(stream, "encoding", None) or "utf-8"
    try:
        stream.write(text + "\n")
    except UnicodeEncodeError:
        stream.write(text.encode(enc, "replace").decode(enc, "replace") + "\n")


def render_status(cell_dir):
    """:func:`cell_status` as a short human block."""
    s = cell_status(cell_dir)
    L = []
    L.append("cell: %s" % s["cell_dir"])
    L.append("  %s / %s   state=%s" % (s.get("sim"), s.get("task"),
                                       s["state"]))
    if s.get("blocker"):
        L.append("  BLOCKER: %s" % str(s["blocker"])[:300])
    if s.get("row"):
        L.append("  row: outcome=%s failed=%s"
                 % (s["row"].get("outcome"), s["row"].get("failed_assertion")))
    budget = s.get("budget") or {}
    L.append("  session elapsed %s | session budget %s | task budget %s%s"
             % (_fmt_age(s.get("session_elapsed_s")),
                _fmt_age(s.get("session_budget_s")),
                _fmt_age(budget.get("task_timeout_s")),
                " (CURTAILED)" if budget.get("session_budget_curtailed")
                else ""))
    # Cell elapsed and queue time are shown SEPARATELY and never added into
    # the session clock the operator judges a plateau by.
    L.append("  cell elapsed %s (of which queued %s -- queue time is the "
             "scheduler's, not the agent's)"
             % (_fmt_age(s.get("elapsed_s")), _fmt_age(s.get("queued_s"))))
    if s.get("deferred_until_utc"):
        L.append("  DEFERRED (rate/usage limit): backing off until %s; the "
                 "cell is asleep, not hung, and still holds its task lock"
                 % s["deferred_until_utc"])
    st = s["stream"]
    L.append("  session: model=%s pinned=%s turns=%s tool_calls=%s events=%s "
             "thinking=%s result_event=%s"
             % (st.get("model"), s.get("pinned_model"),
                st.get("assistant_turns"), st.get("tool_calls"),
                st.get("events"), st.get("thinking_events"),
                st.get("reached_result_event")))
    L.append("  session_id=%s   last stream write %s ago"
             % (st.get("session_id"), _fmt_age(s.get("stream_idle_s"))))
    if s.get("liveness"):
        lv = s["liveness"]
        L.append("  liveness: %s (%s)" % ("OK" if lv.get("ok") else "FAILED",
                                          lv.get("verdict")))
    L.append("  last tool calls:")
    for c in s.get("last_tool_calls") or []:
        L.append("    %3s %-12s %s" % (c.get("n"), c.get("name") or "?",
                                       c.get("target") or ""))
    if not s.get("last_tool_calls"):
        L.append("    (none yet)")
    L.append("  workspace (%s): %s" % (s["workspace_kind"],
                                       s.get("workspace_read")))
    if s.get("workspace_preserved"):
        L.append("  preserved copy: %s" % s["workspace_preserved"])
    L.append("  recently written files:")
    for f in s.get("recent_files") or []:
        L.append("    %-52s %8d B  %s ago"
                 % (f["path"][:52], f["bytes"], _fmt_age(f["age_s"])))
    if not s.get("recent_files"):
        L.append("    (none)")
    if s.get("last_text"):
        L.append("  last assistant text: %s"
                 % " ".join(s["last_text"].split())[:220])
    return "\n".join(L)
