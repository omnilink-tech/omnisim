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

"""A GRBL 1.1 client: what a sender does, as a library an agent bridge can drive.

It talks to any GRBL stream -- the twin's virtual controller (grbl.py over TCP),
or later a real board through the same interface -- so the bridge above it never
knows which. Plain GRBL only, except `twin()`, which reads the twin's `$TWIN`
extension and returns None on a real board.

- A reader thread parses every line: `ok` / `error:N` answer the command in
  flight, `<...>` status reports update `status()`, `ALARM:N` and `[MSG:...]`
  land in `events`.
- A poller sends `?` every `poll_s`.
- `stream()` runs a whole program send-response in a background thread
  (one line in flight, as UGS does by default) and can be cancelled. It stops
  on the first `error:` or `ALARM:`.
"""
from __future__ import annotations

import json
import re
import socket
import threading
import time
from dataclasses import dataclass, field
from collections import deque
from typing import Callable, Deque, Dict, List, Optional

from tool_table import tool_change_in

_STATUS = re.compile(r"^<([^>]*)>$")
RX_BUFFER_BYTES = 127        # GRBL's 128-byte serial receive buffer, one byte kept free


class GrblError(RuntimeError):
    pass


@dataclass
class Job:
    total: int
    sent: int = 0
    acked: int = 0
    state: str = "running"            # running | done | failed | cancelled
    error: Optional[str] = None
    failed_line: Optional[str] = None
    tool_changes: int = 0
    started: float = field(default_factory=time.time)
    finished: Optional[float] = None
    _cancel: threading.Event = field(default_factory=threading.Event, repr=False)

    def snapshot(self) -> dict:
        return {"state": self.state, "lines_total": self.total, "lines_acked": self.acked,
                "error": self.error, "failed_line": self.failed_line, "tool_changes": self.tool_changes,
                "elapsed_s": round((self.finished or time.time()) - self.started, 1)}


def parse_status(text: str, last_wco: Optional[List[float]] = None) -> dict:
    """`<Run|MPos:1,2,3|FS:500,0|WCO:...>` -> dict (WPos derived when WCO is known)."""
    parts = text.split("|")
    out: Dict[str, object] = {"state": parts[0]}
    for p in parts[1:]:
        key, _, value = p.partition(":")
        nums = [float(v) for v in value.split(",") if v != ""] if key in (
            "MPos", "WPos", "WCO", "FS", "F", "Ov", "Bf") else value
        out[key] = nums
    wco = out.get("WCO") or last_wco
    if wco is not None:
        out["WCO"] = wco
        if "MPos" in out and "WPos" not in out:
            out["WPos"] = [round(m - w, 4) for m, w in zip(out["MPos"], wco)]
        elif "WPos" in out and "MPos" not in out:
            out["MPos"] = [round(p + w, 4) for p, w in zip(out["WPos"], wco)]
    fs = out.get("FS")
    if isinstance(fs, list) and len(fs) >= 2:
        out["feed"], out["spindle"] = fs[0], fs[1]
    return out


class GrblLink:
    def __init__(self, host: str = "127.0.0.1", port: int = 2323, poll_s: float = 0.2,
                 connect_timeout_s: float = 30.0):
        self.host, self.port, self.poll_s = host, port, poll_s
        self._sock = self._connect(connect_timeout_s)
        self._send_lock = threading.Lock()
        self._reply = threading.Condition()
        self._pending: List[str] = []           # ok / error / ALARM, in arrival order
        self._status: dict = {}
        self._status_at = 0.0
        self._wco: Optional[List[float]] = None
        self.events: List[dict] = []
        self._seq = 0                           # events ever recorded (events keeps the last 200)
        self.on_event: Callable[[dict], None] = lambda e: None
        self._job: Optional[Job] = None
        self._closed = threading.Event()
        self._buf = b""
        threading.Thread(target=self._read_loop, name="grbl-read", daemon=True).start()
        threading.Thread(target=self._poll_loop, name="grbl-poll", daemon=True).start()

    # ---- connection ---------------------------------------------------------
    def _connect(self, timeout_s: float) -> socket.socket:
        deadline = time.time() + timeout_s
        last = None
        while time.time() < deadline:
            try:
                s = socket.create_connection((self.host, self.port), timeout=2.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                return s
            except OSError as exc:
                last = exc
                time.sleep(0.3)
        raise GrblError(f"no GRBL at {self.host}:{self.port}: {last}")

    def close(self) -> None:
        self._closed.set()
        try:
            self._sock.close()
        except OSError:
            pass

    def _write(self, data: bytes) -> None:
        with self._send_lock:
            self._sock.sendall(data)

    def _read_loop(self) -> None:
        while not self._closed.is_set():
            try:
                chunk = self._sock.recv(65536)
            except OSError:
                break
            if not chunk:
                break
            self._buf += chunk
            *lines, self._buf = self._buf.split(b"\r\n")
            for raw in lines:
                self._on_line(raw.decode("ascii", "replace").strip())
        self._closed.set()
        with self._reply:
            self._reply.notify_all()

    def _on_line(self, line: str) -> None:
        if not line:
            return
        m = _STATUS.match(line)
        if m:
            st = parse_status(m.group(1), self._wco)
            if "WCO" in st:
                self._wco = st["WCO"]
            self._status, self._status_at = st, time.time()
            return
        if line == "ok" or line.startswith("error:") or line.startswith("ALARM:"):
            if line.startswith("ALARM:"):
                self._event("alarm", text=line)
            with self._reply:
                self._pending.append(line)
                self._reply.notify_all()
            return
        if line.startswith("[MSG:") or line.startswith("Grbl ") or line.startswith("[TWIN:"):
            self._event("message", text=line)
        elif line.startswith("["):
            self._event("report", text=line)

    def _event(self, kind: str, **fields) -> None:
        self._seq += 1
        e = {"t": round(time.time(), 3), "seq": self._seq, "kind": kind, **fields}
        self.events.append(e)
        del self.events[:-200]
        self.on_event(e)

    def _reports_since(self, seq: int, prefix: str) -> List[str]:
        # By sequence number, not list position: the list keeps only the last 200.
        return [e["text"] for e in list(self.events) if e["seq"] > seq and e.get("text", "").startswith(prefix)]

    def _poll_loop(self) -> None:
        while not self._closed.is_set():
            try:
                self._write(b"?")
            except OSError:
                break
            time.sleep(self.poll_s)

    # ---- state --------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return not self._closed.is_set()

    def status(self, max_age_s: float = 0.0) -> dict:
        """A status report taken after this call (or at most `max_age_s` old).

        Fresh by default: a report taken BEFORE a command that just changed the work
        offset would describe the old frame, and the caller would quote it as fact.
        """
        asked = time.time() - max_age_s
        if self._status_at < asked:
            try:
                self._write(b"?")
            except OSError:
                return dict(self._status)
            deadline = time.time() + 1.0
            while self._status_at < asked and time.time() < deadline:
                time.sleep(0.01)
        return dict(self._status)

    def wait_idle(self, timeout_s: float = 600.0) -> dict:
        deadline = time.time() + timeout_s
        time.sleep(self.poll_s * 2)
        while time.time() < deadline:
            st = self.status()
            if st.get("state") in ("Idle", "Alarm") and (self._job is None or self._job.state != "running"):
                return st
            time.sleep(self.poll_s)
        raise GrblError("timed out waiting for Idle")

    # ---- commands -----------------------------------------------------------
    def command(self, line: str, timeout_s: float = 30.0) -> str:
        """Send one line and return GRBL's answer (ok / error:N / ALARM:N)."""
        if self._job is not None and self._job.state == "running":
            raise GrblError("a job is streaming; pause or cancel it first")
        return self._send_and_wait(line, timeout_s)

    def _send_and_wait(self, line: str, timeout_s: float, cancel: Optional[threading.Event] = None) -> str:
        with self._reply:
            self._pending.clear()
        self._write(line.strip().encode("ascii") + b"\n")
        deadline = time.time() + timeout_s
        with self._reply:
            while not self._pending:
                # A reset discards the line in flight, so its `ok` never comes: a cancel
                # has to wake this wait, or a stopped job reads "running" forever.
                if cancel is not None and cancel.is_set():
                    raise GrblError("cancelled")
                left = deadline - time.time()
                if left <= 0 or self._closed.is_set():
                    raise GrblError(f"no answer to {line!r}")
                self._reply.wait(min(left, 0.1))
            return self._pending.pop(0)

    def query(self, line: str, prefix: str, timeout_s: float = 5.0) -> List[str]:
        """Send a `$` query and collect the report lines that start with `prefix`."""
        start = self._seq
        answer = self.command(line, timeout_s)
        if answer != "ok":
            raise GrblError(f"{line} -> {answer}")
        return self._reports_since(start, prefix)

    def twin(self) -> Optional[dict]:
        """The twin's `$TWIN` view (None on a real GRBL board, which answers error:3)."""
        try:
            lines = self.query("$TWIN", "[TWIN:")
        except GrblError:
            return None
        return json.loads(lines[-1][6:-1]) if lines else None

    def _change_tool(self, number: Optional[int], cancel: threading.Event) -> Optional[str]:
        """An M6 inside a job: what gSender does for GRBL 1.1, which has no M6.

        Wait for motion to stop, stop the spindle, put the tool in, and set its length
        offset. On the twin `$TWIN TOOL=n` is the operator's hands and `$TWIN` the tool
        setter. A real board answers error:3 to both: the job stops there, before a
        cut with the wrong bit, and says why. Returns None, or the reason it failed.
        """
        if number is None:
            return "M6 with no tool number (no T word on it or before it)"
        for line in ("G4 P0", "M5", f"$TWIN TOOL={number}"):
            answer = self._send_and_wait(line, timeout_s=3600.0, cancel=cancel)
            if answer != "ok":
                if line.startswith("$TWIN"):
                    return (f"tool change to T{number}: {answer}. This sender changes tools only on the "
                            "OmniSim twin; on a real machine change the bit, set Z, and resume after the M6")
                return f"tool change to T{number}: {line} -> {answer}"
        start = self._seq
        answer = self._send_and_wait("$TWIN", timeout_s=30.0, cancel=cancel)
        tool = None
        deadline = time.time() + 5.0
        while answer == "ok" and time.time() < deadline:
            # The report that shows the new tool, whenever it lands: never a stale one.
            for text in reversed(self._reports_since(start, "[TWIN:")):
                t = json.loads(text[6:-1]).get("tool") or {}
                if t.get("number") == number:
                    tool = t
                    break
            if tool is not None:
                break
            time.sleep(0.02)
        if tool is None:
            recent = [e.get("text", e["kind"])[:60] for e in list(self.events)[-6:]]
            return f"tool change to T{number}: no $TWIN report with T{number} ({answer}); last events {recent}"
        tlo = float(tool["tlo_mm"])
        answer = self._send_and_wait(f"G43.1 Z{tlo:.4f}", timeout_s=30.0, cancel=cancel)
        if answer != "ok":
            return f"tool change to T{number}: G43.1 -> {answer}"
        self._event("tool_change", tool=number, tlo_mm=tlo)
        return None

    # ---- real-time ----------------------------------------------------------
    def feed_hold(self) -> None:
        self._write(b"!")

    def cycle_start(self) -> None:
        self._write(b"~")

    def soft_reset(self) -> None:
        if self._job is not None and self._job.state == "running":
            self._job._cancel.set()
        self._write(b"\x18")

    def jog_cancel(self) -> None:
        self._write(b"\x85")

    # ---- jobs ---------------------------------------------------------------
    @property
    def job(self) -> Optional[Job]:
        return self._job

    def stream(self, program: str, on_done: Callable[[Job], None] = lambda j: None) -> Job:
        """Start streaming a program in the background (send-response). Returns at once."""
        if self._job is not None and self._job.state == "running":
            raise GrblError("a job is already streaming")
        lines = [l for l in (raw.strip() for raw in program.splitlines()) if l]
        job = Job(total=len(lines))
        self._job = job

        def fail(error: str, line: str) -> None:
            job.state, job.error, job.failed_line = "failed", error, line

        def run():
            # Character counting, as UGS and gSender stream: keep GRBL's 128-byte receive
            # buffer full instead of one line in flight. Send-response starves the planner on
            # 3D jobs (thousands of sub-millimetre moves): one round trip per line.
            selected = None
            in_flight: Deque[tuple] = deque()      # (bytes in GRBL's buffer, line)
            queued = 0
            pending = list(lines)
            pending.reverse()
            with self._reply:
                self._pending.clear()

            def collect() -> Optional[str]:
                """Wait for the oldest line's answer. Returns None, or why the job stops."""
                nonlocal queued
                with self._reply:
                    while not self._pending:
                        if job._cancel.is_set():
                            return "cancelled"
                        if self._closed.is_set():
                            return "connection lost"
                        self._reply.wait(0.1)
                    answer = self._pending.pop(0)
                size, line = in_flight.popleft()
                queued -= size
                if answer == "ok":
                    job.acked += 1
                    return None
                fail(answer, line)
                return answer

            while pending or in_flight:
                if job._cancel.is_set():
                    job.state = "cancelled"
                    break
                if pending and job.state == "running":
                    line = pending[-1]
                    send, change, number = tool_change_in(line)
                    if change:
                        # The tool change needs the machine at rest with nothing queued.
                        if in_flight:
                            if collect():
                                break
                            continue
                        pending.pop()
                        job.sent += 1
                        selected = number if number is not None else selected
                        try:
                            if send:
                                answer = self._send_and_wait(send, timeout_s=3600.0, cancel=job._cancel)
                                if answer != "ok":
                                    fail(answer, line)
                                    break
                            failed = self._change_tool(selected, job._cancel)
                        except GrblError as exc:
                            failed = "cancelled" if job._cancel.is_set() else str(exc)
                        if failed:
                            if failed == "cancelled":
                                job.state = "cancelled"
                            else:
                                fail(failed, line)
                            break
                        job.tool_changes += 1
                        job.acked += 1
                        continue
                    size = len(send) + 1
                    if size > RX_BUFFER_BYTES:
                        fail(f"line longer than GRBL's {RX_BUFFER_BYTES}-byte buffer", line)
                        break
                    if queued + size <= RX_BUFFER_BYTES:
                        pending.pop()
                        job.sent += 1
                        selected = number if number is not None else selected
                        try:
                            self._write(send.encode("ascii") + b"\n")
                        except OSError as exc:
                            fail(f"connection lost: {exc}", line)
                            break
                        in_flight.append((size, line))
                        queued += size
                        continue
                if not in_flight:
                    break
                stopped = collect()
                if stopped == "cancelled":
                    job.state = "cancelled"
                    break
                if stopped:
                    # Lines already in GRBL's buffer would still run: hold the machine, as a
                    # sender does on an error, and leave the operator to resume or stop.
                    self.feed_hold()
                    break
            if job.state == "running":
                job.state = "done"
            job.finished = time.time()
            self._event("job", state=job.state, error=job.error)
            on_done(job)

        threading.Thread(target=run, name="grbl-stream", daemon=True).start()
        return job

    def cancel_job(self) -> None:
        """Stop sending further lines and halt motion (soft reset, as a sender's Stop does)."""
        if self._job is not None and self._job.state == "running":
            self._job._cancel.set()
        self.soft_reset()
