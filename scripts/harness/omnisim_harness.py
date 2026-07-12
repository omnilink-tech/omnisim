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

"""OmniSim agent-facing validation harness.

A long-running HTTP service that wraps a headless OmniSim subprocess so coding
agents can load worlds, fetch structured load diagnostics, and probe the
running scene without launching the desktop GUI or paying full process-start
cost per check.

Endpoints:
    POST /world/load          {"path": "...", "wait_s": 30.0, "with_supervisor": true}
    GET  /world/diagnostics
    POST /world/screenshot    {"path"?: str, "quality"?: int}  -> image/png or {path}
    GET  /world/render_stats  -> {mean_brightness, mean_rgb, max_rgb, saturated_pct, ...}
    GET  /scene/tree
    GET  /scene/node/{def}
    POST /scene/look_at       {"position": [x,y,z], "target": [x,y,z], "push"?: bool}
    POST /sim/step            {"steps"?: int}
    POST /sim/reset
    GET  /sim/state
    GET  /sim/contacts         -> {contacts: [{a_def, b_def, point}]}
    GET  /sim/grips            -> {grips: [{gripper_def, held_def, since_t_ms}]}
    GET  /sim/events?since=<sup_seq>&log_since=<log_seq>&limit=<int>&types=<csv>
                                 -> unified event stream (sim + controller log + world log)
                                    {events:[{seq, source:"sup"|"log", t_sim_ms?, t_wall?,
                                              type, ...}], next_since, next_log_since,
                                     dropped_sup, dropped_log}
                                    Event types: contact.began/ended, joint.limit_hit,
                                    grip.acquired/released, damage.impact,
                                    damage.state_transition, controller.log,
                                    world.warning, world.error
    GET  /robots               -> {robots: [{def, name, model, controller, type,
                                              position, orientation, num_joints}]}
    GET  /robot/{def}/joints   -> {robot, joints: [{name, type, position, velocity,
                                                     lower, upper, hit_limit}]}
    GET  /robot/{def}/devices  -> {robot, devices: [{name, type}]}
    GET  /robot/{def}/sensor/{name}  501 — supervisor cannot read live sensor data
                                from devices it does not own; use /joints for joint
                                positions or a per-robot helper controller for cameras
                                and lidars.
    GET  /robot/damage         -> {robot, attached, parts, damage:{part:{state,hp,...}}, ...}
    GET  /robot/damage/events?since=<int>&limit=<int>
                                 -> {events: [...], last_step_id, events_total}
                                    (back-compat filtered view onto /sim/events damage.*)
    POST /robot/damage/reset    heal all parts back to pristine without resetting the sim
    POST /robot/damage/inject  {"part": "wheel_fl", "state"?: "broken", "hp_delta"?: -50}
                                 test/debug hook to set a part's state directly
    GET  /healthz

When `with_supervisor` is true (default), the harness writes a sibling
`.harness_<name>.wbt` next to the user's world that appends a generic
Supervisor robot, then launches OmniSim on the sibling. The supervisor
controller (projects/default/controllers/harness_supervisor) opens a TCP
socket on 127.0.0.1:6790 that the harness uses for screenshots, scene
queries, step, and reset. The original world file is never touched and the
sibling is removed on next load or shutdown.

See scripts/harness/README.md and AGENTS.md section 5.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

sys.path.insert(0, str(Path(__file__).resolve().parent))
from diagnostic_codes import classify_text  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 6789
# Supervised loads return as soon as the supervisor binds, so a generous
# default only costs time when something is genuinely slow or broken. A
# cold simulator start (CUDA + warp init) takes ~20s on a mid-range GPU
# laptop — the old 3s default missed it every time and bricked the
# session. Loads without a supervisor have no positive signal and always
# sleep the full window, so they keep a short default.
DEFAULT_LOAD_WAIT_S = 30.0
DEFAULT_LOAD_WAIT_BARE_S = 3.0
MAX_LOAD_WAIT_S = 60.0

SUPERVISOR_HOST = "127.0.0.1"
# Default supervisor IPC port. Each `HarnessState` carries its own
# `supervisor_port` attribute so that a second harness on a different
# `--port` can use a different supervisor port (otherwise two harnesses
# would race for the same TCP listener inside the OmniSim subprocess).
DEFAULT_SUPERVISOR_PORT = 6790
SUPERVISOR_PORT = DEFAULT_SUPERVISOR_PORT  # back-compat alias for callers/tests
SUPERVISOR_CONNECT_TIMEOUT_S = 30.0
SUPERVISOR_RPC_TIMEOUT_S = 30.0
SUPERVISOR_POLL_INTERVAL_S = 0.1
HOT_RELOAD_WAIT_S = 15.0

SUPERVISOR_INJECT_STANZA = """
Robot {
  name "harness_supervisor"
  controller "harness_supervisor"
  supervisor TRUE
  synchronization FALSE
}
"""


def infer_webots_home() -> Path:
    return Path(os.environ.get("OMNISIM_HOME", str(REPO_ROOT)))


def resolve_webots_binary(webots_home: Path) -> Path:
    if sys.platform == "win32":
        candidates = [
            webots_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            webots_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [webots_home / "Contents" / "MacOS" / "omnisim", webots_home / "Contents" / "MacOS" / "webots"]
    else:
        candidates = [webots_home / "bin" / "omnisim-bin", webots_home / "webots"]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise RuntimeError(
        f"Cannot find the OmniSim binary under {webots_home}. "
        "Set OMNISIM_HOME to a built tree or install location."
    )


def parse_log_lines(text: str) -> list[dict]:
    """Convert raw omnisim_log.txt content into a structured diagnostic list."""
    return classify_text(text)


def compute_look_at_orientation(position: list[float], target: list[float]) -> list[float]:
    """Return the axis-angle orientation that points the OmniSim Viewpoint
    default forward (+X) at `target` from `position`.

    The result is the four-tuple [ax, ay, az, angle] that the Viewpoint's
    `orientation` SFRotation field expects. Pure math, no IPC.
    """
    px, py, pz = position
    tx, ty, tz = target
    dx, dy, dz = tx - px, ty - py, tz - pz
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    if n < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    dx, dy, dz = dx / n, dy / n, dz / n
    # forward = (1, 0, 0); cos(angle) = forward · d = dx
    cos_angle = max(-1.0, min(1.0, dx))
    angle = math.acos(cos_angle)
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    # axis = forward × d = (0, -dz, dy)
    ax, ay, az = 0.0, -dz, dy
    am = math.sqrt(ax * ax + ay * ay + az * az)
    if am < 1e-9:
        # Antiparallel — pick any perpendicular axis.
        return [0.0, 0.0, 1.0, math.pi]
    return [ax / am, ay / am, az / am, angle]


def compute_render_stats(image_bytes: bytes) -> dict:
    """Quick brightness statistics over a PNG screenshot. Used by
    /world/render_stats so callers can check exposure without paying a
    full image round-trip + manual inspection.
    """
    if not _HAS_PIL:
        raise RuntimeError(
            "Pillow is not installed; render stats require it. "
            "Install with: pip install Pillow"
        )
    import io
    from PIL import ImageChops, ImageStat
    image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    width, height = image.size
    n = width * height
    if n == 0:
        return {"width": width, "height": height, "pixels": 0}
    # All-C-accelerated stats: no full-image Python materialization and no
    # per-pixel loop (the old list(getdata()) + for-loop cost ~64 MB and
    # seconds on a 4K frame). Equivalent semantics, verified by
    # tests/harness/test_helpers.py::test_render_stats_*.
    sum_r, sum_g, sum_b = ImageStat.Stat(image).sum
    (_, max_r), (_, max_g), (_, max_b) = image.getextrema()
    # Per-pixel max(r, g, b) as an L image, then a 256-bin histogram — the
    # saturated/black thresholds are on that per-pixel channel maximum.
    chan_r, chan_g, chan_b = image.split()
    chan_max = ImageChops.lighter(ImageChops.lighter(chan_r, chan_g), chan_b)
    hist = chan_max.histogram()
    saturated = sum(hist[250:256])
    black = sum(hist[0:6])
    mean = ((sum_r + sum_g + sum_b) / 3.0) / n
    sat_pct = 100.0 * saturated / n
    blk_pct = 100.0 * black / n
    warnings: list[str] = []
    if sat_pct > 30:
        warnings.append(
            f"blown out: {sat_pct:.1f}% of pixels are saturated; "
            "reduce DirectionalLight/PointLight intensities"
        )
    if blk_pct > 60:
        warnings.append(
            f"underexposed: {blk_pct:.1f}% of pixels are near-black; "
            "increase light intensities or check camera framing"
        )
    return {
        "width": width,
        "height": height,
        "pixels": n,
        "mean_brightness": round(mean, 2),
        "mean_rgb": [round(sum_r / n, 2), round(sum_g / n, 2), round(sum_b / n, 2)],
        "max_rgb": [max_r, max_g, max_b],
        "saturated_pct": round(sat_pct, 2),
        "black_pct": round(blk_pct, 2),
        "warnings": warnings,
    }


def sibling_path_for(world: Path) -> Path:
    """Return the temp sibling path used to host the supervisor injection.

    Lives in the same directory as the original so all relative asset paths
    resolve. Leading dot keeps it conventionally hidden.
    """
    return world.with_name(f".harness_{world.name}")


def write_sibling_world(original: Path) -> Path:
    """Read the user's world file and write a sibling copy with the generic
    `harness_supervisor` Robot stanza appended. Returns the sibling path.
    """
    sibling = sibling_path_for(original)
    content = original.read_text(encoding="utf-8", errors="replace")
    if not content.endswith("\n"):
        content += "\n"
    content += SUPERVISOR_INJECT_STANZA
    sibling.write_text(content, encoding="utf-8")
    return sibling


# ---------------------------------------------------------------------------
# Supervisor IPC
# ---------------------------------------------------------------------------


class SupervisorRPCError(Exception):
    pass


class SupervisorClient:
    """Length-prefixed JSON RPC over a single persistent TCP connection.

    All sends and receives are serialized through a single lock; the harness's
    HTTP server is multithreaded so concurrent requests must not interleave on
    the wire. Each RPC blocks until the supervisor responds (or
    SUPERVISOR_RPC_TIMEOUT_S elapses), which is the right shape for a
    request/response control plane.
    """

    def __init__(self):
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._next_id = 1

    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self, host: str, port: int, deadline_s: float) -> None:
        deadline = time.time() + deadline_s
        last_exc: Exception | None = None
        while time.time() < deadline:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((host, port))
                sock.settimeout(SUPERVISOR_RPC_TIMEOUT_S)
                with self._lock:
                    self._sock = sock
                return
            except OSError as exc:
                last_exc = exc
                time.sleep(0.1)
        raise SupervisorRPCError(
            f"could not connect to supervisor at {host}:{port} within {deadline_s:.1f}s "
            f"(last error: {last_exc})"
        )

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def call(self, cmd: str, args: dict | None = None) -> dict:
        with self._lock:
            if self._sock is None:
                raise SupervisorRPCError("supervisor is not connected")
            req_id = self._next_id
            self._next_id += 1
            request = {"id": req_id, "cmd": cmd, "args": args or {}}
            payload = json.dumps(request).encode("utf-8")
            try:
                self._sock.sendall(struct.pack(">I", len(payload)) + payload)
                header = self._recv_exact(4)
                (length,) = struct.unpack(">I", header)
                if length == 0 or length > 16 * 1024 * 1024:
                    raise SupervisorRPCError(f"bad response length: {length}")
                body = self._recv_exact(length)
            except (OSError, SupervisorRPCError) as exc:
                self._drop_locked()
                raise SupervisorRPCError(f"supervisor RPC failed: {exc}") from exc
            response = json.loads(body.decode("utf-8"))
            if not response.get("ok"):
                raise SupervisorRPCError(response.get("error", "supervisor rejected request"))
            return response.get("result") or {}

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        chunks: list[bytes] = []
        remaining = n
        while remaining > 0:
            chunk = self._sock.recv(remaining)
            if not chunk:
                raise SupervisorRPCError("supervisor closed the connection")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _drop_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


# ---------------------------------------------------------------------------
# Log capture (controller stdout/stderr + omnisim_log.txt deltas)
# ---------------------------------------------------------------------------


DEFAULT_LOG_BUFFER_SIZE = 4096


class LogRingBuffer:
    """Thread-safe ring buffer of log events.

    Holds two kinds of events: `controller.log` (one per line read off
    the OmniSim subprocess's stdout/stderr) and `world.warning` /
    `world.error` (parsed from omnisim_log.txt deltas). Both share a
    monotonic seq counter so an agent driving `/sim/events` can pull
    them with one cursor.

    Each event:
        {"seq": int, "type": str, "stream"?: str, "line"?: str,
         "code"?: str, "message"?: str, "t_wall": float}
    """

    def __init__(self, maxlen: int = DEFAULT_LOG_BUFFER_SIZE):
        self._lines: collections.deque = collections.deque(maxlen=maxlen)
        self._counter = 0
        self._dropped = 0
        self._lock = threading.Lock()

    def emit_controller_log(self, stream: str, line: str) -> None:
        with self._lock:
            if len(self._lines) == self._lines.maxlen:
                self._dropped += 1
            self._counter += 1
            self._lines.append({
                "seq": self._counter,
                "type": "controller.log",
                "stream": stream,
                "line": line,
                "t_wall": time.time(),
            })

    def emit_world_diagnostic(self, diag: dict) -> None:
        # Map severity to a type. fatal/error -> world.error;
        # warning -> world.warning; everything else gets dropped (info
        # and unknown lines aren't actionable for an agent).
        sev = diag.get("severity")
        if sev in ("fatal", "error"):
            type_ = "world.error"
        elif sev == "warning":
            type_ = "world.warning"
        else:
            return
        with self._lock:
            if len(self._lines) == self._lines.maxlen:
                self._dropped += 1
            self._counter += 1
            self._lines.append({
                "seq": self._counter,
                "type": type_,
                "code": diag.get("code"),
                "message": diag.get("message"),
                "raw": diag.get("raw"),
                "t_wall": time.time(),
            })

    def since(self, since_seq: int, limit: int = 256,
              types: list[str] | None = None) -> list[dict]:
        type_set = set(types) if types is not None else None
        with self._lock:
            out: list[dict] = []
            for evt in self._lines:
                if evt["seq"] <= since_seq:
                    continue
                if type_set is not None and evt["type"] not in type_set:
                    continue
                out.append(evt)
                if len(out) >= limit:
                    break
            return out

    @property
    def total(self) -> int:
        with self._lock:
            return self._counter

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped


def _pump_pipe(pipe, stream_name: str, log_buffer: LogRingBuffer,
               forward_to) -> None:
    """Daemon thread body: read lines off `pipe`, push each as a
    `controller.log` event into `log_buffer`, and forward the line to
    `forward_to` (typically sys.stdout/sys.stderr) so an operator
    running the harness in a terminal still sees the live output.

    Exits cleanly on EOF, which happens when the OmniSim subprocess is
    terminated. A new thread is started for each cold launch.
    """
    try:
        for raw_line in iter(pipe.readline, b""):
            try:
                line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:  # noqa: BLE001
                continue
            log_buffer.emit_controller_log(stream_name, line)
            try:
                forward_to.write(line + "\n")
                forward_to.flush()
            except Exception:  # noqa: BLE001
                pass
    finally:
        try:
            pipe.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Harness state
# ---------------------------------------------------------------------------


class HarnessState:
    """Owns the OmniSim subprocess, sibling file lifecycle, and supervisor RPC."""

    def __init__(
        self,
        webots_home: Path,
        supervisor_host: str = SUPERVISOR_HOST,
        supervisor_port: int = DEFAULT_SUPERVISOR_PORT,
    ):
        self.webots_home = webots_home
        self.binary = resolve_webots_binary(webots_home)
        self.log_path = webots_home / "omnisim_log.txt"
        self.supervisor_host = supervisor_host
        self.supervisor_port = supervisor_port
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.current_world: str | None = None
        self.current_sibling: Path | None = None
        self.last_load_started_at: float | None = None
        self.last_load_completed_at: float | None = None
        self.last_load_ok: bool | None = None
        self.last_load_ms: int | None = None
        self.last_diagnostics: list[dict] = []
        self.last_exit_code: int | None = None
        self.supervisor: SupervisorClient | None = None
        self.supervisor_connected_at: float | None = None
        self.started_at = time.time()
        # Phase 2: controller-log capture and world-log delta tail.
        # The buffer survives across cold launches; reader threads are
        # restarted per cold launch (the old ones drain to EOF when the
        # subprocess pipe closes). Hot reloads keep the same subprocess
        # so the existing threads stay attached.
        self.log_buffer = LogRingBuffer()
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._world_log_offset = 0
        self._world_log_lock = threading.Lock()

    def _kill_running(self) -> None:
        if self.supervisor is not None:
            self.supervisor.close()
            self.supervisor = None
            self.supervisor_connected_at = None
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=2)
            except Exception as exc:
                print(f"[harness] error terminating prior subprocess: {exc}", file=sys.stderr)
        self.proc = None

    def _cleanup_sibling(self) -> None:
        if self.current_sibling is not None:
            try:
                self.current_sibling.unlink()
            except OSError:
                pass
            self.current_sibling = None

    def _read_log(self) -> str:
        if not self.log_path.exists():
            return ""
        try:
            return self.log_path.read_text(errors="replace")
        except OSError as exc:
            return f"WARNING: harness could not read log: {exc}\n"

    def _drain_world_log_into_buffer(self) -> None:
        """Read any new bytes from omnisim_log.txt, parse them through
        the diagnostic classifier, and push warnings/errors into
        `log_buffer`. Called lazily on `/sim/events` so we don't pay
        the parse cost on every tick.

        File truncation (cold launch) is handled by `_world_log_offset`
        being reset there. If the file shrinks unexpectedly we reset to
        0 to avoid skipping past valid content.
        """
        if not self.log_path.exists():
            return
        with self._world_log_lock:
            try:
                size = self.log_path.stat().st_size
            except OSError:
                return
            if size < self._world_log_offset:
                self._world_log_offset = 0
            if size == self._world_log_offset:
                return
            try:
                with self.log_path.open("rb") as f:
                    f.seek(self._world_log_offset)
                    new_bytes = f.read()
                self._world_log_offset = size
            except OSError:
                return
        try:
            text = new_bytes.decode("utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            return
        for diag in parse_log_lines(text):
            self.log_buffer.emit_world_diagnostic(diag)

    def _try_connect_supervisor(self, deadline: float) -> SupervisorClient | None:
        """Poll the supervisor TCP port until it accepts a connection or the
        deadline expires. Returns a connected client, or None if the
        subprocess died or the deadline was missed.
        """
        while time.time() < deadline:
            if self.proc is None or self.proc.poll() is not None:
                return None
            client = SupervisorClient()
            try:
                client.connect(self.supervisor_host, self.supervisor_port, deadline_s=0.5)
                client.call("ping")
                return client
            except SupervisorRPCError:
                client.close()
                time.sleep(SUPERVISOR_POLL_INTERVAL_S)
        return None

    def _wait_for_load_signal(self, with_supervisor: bool, deadline: float) -> tuple[int | None, SupervisorClient | None]:
        """Block until we have a positive load signal or the deadline expires.

        With supervisor: poll the TCP port; the supervisor only binds after
        the world has loaded enough for its controller to spin up. This is
        much faster than waiting wait_s for a heuristic timeout.

        Without supervisor: there is no positive signal; sleep until deadline
        or subprocess exit, mirroring M0/M1 behavior.
        """
        if with_supervisor:
            client = self._try_connect_supervisor(deadline)
            exit_code = self.proc.poll() if self.proc else None
            return exit_code, client
        while time.time() < deadline:
            ret = self.proc.poll() if self.proc else None
            if ret is not None:
                return ret, None
            time.sleep(0.05)
        return None, None

    def _read_diagnostics(self, exit_code: int | None) -> list[dict]:
        diagnostics = parse_log_lines(self._read_log())
        if exit_code == 3221225781 and not diagnostics:
            diagnostics.append({
                "code": "LAUNCHER_DLL_NOT_FOUND",
                "severity": "fatal",
                "message": (
                    "Simulator binary exited with STATUS_DLL_NOT_FOUND (0xC0000135). "
                    "The bundled runtime at $OMNISIM_HOME/msys64/mingw64/bin is missing required DLLs. "
                    "Add a complete msys2 mingw64 bin directory to PATH before starting the harness."
                ),
                "raw": f"exit_code={exit_code}",
            })
        elif exit_code is not None and exit_code != 0 and not any(d["severity"] in ("fatal", "error") for d in diagnostics):
            diagnostics.append({
                "code": "SIMULATOR_EXITED_NONZERO",
                "severity": "error",
                "message": f"Simulator subprocess exited with code {exit_code} but no error was logged.",
                "raw": f"exit_code={exit_code}",
            })
        return diagnostics

    def _try_hot_reload(self, world: Path) -> dict | None:
        """Attempt to reload the simulator into the given world without
        restarting the OmniSim process. Requires an existing live supervisor.
        Returns a result dict on success, None if the path is not viable
        (caller should fall back to cold launch).
        """
        with self.lock:
            current_sup = self.supervisor
            current_proc = self.proc
            old_sibling = self.current_sibling
        if current_proc is None or current_proc.poll() is not None:
            return None
        if current_sup is None or not current_sup.is_connected():
            # The supervisor may have bound after the previous load's wait
            # window expired — adopt it now rather than falling back to a
            # cold relaunch of the whole simulator.
            current_sup = self._try_connect_supervisor(time.time() + 2.0)
            if current_sup is None:
                return None
            with self.lock:
                self.supervisor = current_sup
                self.supervisor_connected_at = time.time()

        started_at = time.time()
        try:
            new_sibling = write_sibling_world(world)
        except OSError:
            return None

        try:
            current_sup.call("world_load", {"path": str(new_sibling)})
        except SupervisorRPCError:
            # The old supervisor may have already started shutting down — this
            # is benign as long as the simulator is still alive. Continue to
            # the reconnect step.
            pass
        # The world swap fires on the next sim step, after which the old
        # supervisor controller terminates and a new one starts. Drop the
        # stale connection and wait for the new one to come up.
        current_sup.close()
        with self.lock:
            self.supervisor = None
            self.supervisor_connected_at = None

        deadline = time.time() + HOT_RELOAD_WAIT_S
        new_client = self._try_connect_supervisor(deadline)
        load_ms = int((time.time() - started_at) * 1000)

        if new_client is None:
            # Hot reload didn't come back. The subprocess might still be alive
            # but in an inconsistent state — the safe move is to fall back to
            # a cold launch from the caller.
            return None

        with self.lock:
            self.supervisor = new_client
            self.supervisor_connected_at = time.time()
            # The new world is the new sibling; the old sibling (if different
            # name) is now stale and can be removed.
            if old_sibling is not None and old_sibling != new_sibling:
                try:
                    old_sibling.unlink()
                except OSError:
                    pass
            self.current_sibling = new_sibling
            self.current_world = str(world)
            self.last_load_started_at = started_at
            self.last_load_completed_at = time.time()
            self.last_load_ok = True
            self.last_load_ms = load_ms
            self.last_diagnostics = []
            self.last_exit_code = None

        return {
            "ok": True,
            "world": str(world),
            "load_ms": load_ms,
            "exit_code": None,
            "diagnostics": [],
            "supervisor": "connected",
            "hot_reloaded": True,
        }

    def load_world(self, world_path: str, wait_s: float, with_supervisor: bool) -> dict:
        """Launch OmniSim on the given world. When with_supervisor is true (the
        default), inject a supervisor via a sibling file and connect to it.
        Reuses the running subprocess via a hot-reload RPC when possible.
        """
        world = Path(world_path)
        if not world.is_absolute():
            world = REPO_ROOT / world
        if not world.exists():
            return {
                "ok": False,
                "error": f"world not found: {world}",
                "diagnostics": [],
                "load_ms": 0,
            }

        wait_s = max(0.1, min(MAX_LOAD_WAIT_S, float(wait_s)))

        # Hot reload: only possible when a supervisor is already connected
        # AND the new request also wants supervisor. Otherwise fall through
        # to the cold-launch path.
        if with_supervisor:
            hot = self._try_hot_reload(world)
            if hot is not None:
                return hot

        with self.lock:
            self._kill_running()
            self._cleanup_sibling()
            self.current_world = str(world)
            self.last_load_started_at = time.time()
            self.last_load_completed_at = None
            self.last_load_ok = None
            self.last_diagnostics = []
            self.last_exit_code = None
            try:
                if self.log_path.exists():
                    self.log_path.unlink()
            except OSError:
                pass

            target_world = world
            if with_supervisor:
                try:
                    self.current_sibling = write_sibling_world(world)
                    target_world = self.current_sibling
                except OSError as exc:
                    return {
                        "ok": False,
                        "error": f"could not write supervisor sibling: {exc}",
                        "diagnostics": [],
                        "load_ms": 0,
                    }

            cmd = [
                str(self.binary),
                str(target_world),
                "--batch",
                "--mode=fast",
                "--minimize",
                "--stdout",
                "--stderr",
            ]
            # exportImage requires rendering to be enabled; only suppress
            # rendering when the supervisor is not in play.
            if not with_supervisor:
                cmd.insert(4, "--no-rendering")
            env = os.environ.copy()
            env["OMNISIM_HOME"] = str(self.webots_home)
            env["OMNISIM_HARNESS_SUPERVISOR_HOST"] = self.supervisor_host
            env["OMNISIM_HARNESS_SUPERVISOR_PORT"] = str(self.supervisor_port)
            if sys.platform == "win32":
                env["PATH"] = str(self.binary.parent) + ";" + env.get("PATH", "")

            try:
                self.proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=1,  # line-buffered on the parent side
                )
            except OSError as exc:
                self.last_load_completed_at = time.time()
                self.last_load_ok = False
                self.last_load_ms = 0
                self._cleanup_sibling()
                return {
                    "ok": False,
                    "error": f"failed to launch simulator: {exc}",
                    "diagnostics": [],
                    "load_ms": 0,
                }
            # Spin up the stdout/stderr pumps for this cold launch.
            # Daemon threads — they exit on pipe EOF when the
            # subprocess terminates, and the harness shuts down with
            # them on process exit even if EOF hasn't arrived.
            assert self.proc.stdout is not None and self.proc.stderr is not None
            self._stdout_thread = threading.Thread(
                target=_pump_pipe,
                args=(self.proc.stdout, "stdout", self.log_buffer, sys.stdout),
                daemon=True,
                name="harness-stdout-pump",
            )
            self._stderr_thread = threading.Thread(
                target=_pump_pipe,
                args=(self.proc.stderr, "stderr", self.log_buffer, sys.stderr),
                daemon=True,
                name="harness-stderr-pump",
            )
            self._stdout_thread.start()
            self._stderr_thread.start()
            # Reset the world-log tail offset for the new process —
            # the file just got truncated above.
            with self._world_log_lock:
                self._world_log_offset = 0
            launched_at = self.last_load_started_at

        deadline = launched_at + wait_s
        exit_code, supervisor_client = self._wait_for_load_signal(with_supervisor, deadline)

        diagnostics = self._read_diagnostics(exit_code)
        fatal_or_error = any(d["severity"] in ("fatal", "error") for d in diagnostics)
        ok = (exit_code is None or exit_code == 0) and not fatal_or_error
        load_ms = int((time.time() - launched_at) * 1000)

        supervisor_status: str | None = None
        if with_supervisor:
            if ok and supervisor_client is not None:
                with self.lock:
                    self.supervisor = supervisor_client
                    self.supervisor_connected_at = time.time()
                supervisor_status = "connected"
            elif ok:
                supervisor_status = f"unavailable: did not bind within {wait_s:.1f}s"
            else:
                supervisor_status = "not started (load failed)"
                if supervisor_client is not None:
                    supervisor_client.close()

        with self.lock:
            self.last_load_completed_at = time.time()
            self.last_load_ok = ok
            self.last_load_ms = load_ms
            self.last_diagnostics = diagnostics
            self.last_exit_code = exit_code

        result: dict = {
            "ok": ok,
            "world": str(world),
            "load_ms": load_ms,
            "exit_code": exit_code,
            "diagnostics": diagnostics,
        }
        if with_supervisor:
            result["supervisor"] = supervisor_status or "not requested"
            result["hot_reloaded"] = False
        return result

    def diagnostics(self) -> dict:
        live = parse_log_lines(self._read_log())
        with self.lock:
            return {
                "world": self.current_world,
                "load_ok": self.last_load_ok,
                "load_ms": self.last_load_ms,
                "diagnostics": live,
            }

    def sim_state(self) -> dict:
        with self.lock:
            running = self.proc is not None and self.proc.poll() is None
            exit_code = None if running else (self.proc.poll() if self.proc else None)
            supervisor_connected = self.supervisor is not None and self.supervisor.is_connected()
            return {
                "world": self.current_world,
                "running": running,
                "exit_code": exit_code,
                "load_ok": self.last_load_ok,
                "load_ms": self.last_load_ms,
                "load_started_at": self.last_load_started_at,
                "load_completed_at": self.last_load_completed_at,
                "supervisor_connected": supervisor_connected,
                "supervisor_connected_at": self.supervisor_connected_at,
                "binary": str(self.binary),
                "webots_home": str(self.webots_home),
            }

    def supervisor_call(self, cmd: str, args: dict | None = None) -> dict:
        with self.lock:
            client = self.supervisor
            proc_alive = self.proc is not None and self.proc.poll() is None
        if (client is None or not client.is_connected()) and proc_alive:
            # The supervisor may have bound after the load window expired
            # (cold CUDA/warp starts take ~20s) or a previous RPC hiccup
            # dropped the socket. One short reconnect attempt per call
            # un-bricks the session instead of 503ing forever.
            client = self._try_connect_supervisor(time.time() + 2.0)
            if client is not None:
                with self.lock:
                    self.supervisor = client
                    self.supervisor_connected_at = time.time()
        if client is None or not client.is_connected():
            raise SupervisorRPCError("supervisor not connected (load a world with with_supervisor=true)")
        return client.call(cmd, args)

    def shutdown(self) -> None:
        with self.lock:
            self._kill_running()
            self._cleanup_sibling()


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------


def make_handler(state: HarnessState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code: int, obj: dict) -> None:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _png(self, body: bytes) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            n = int(self.headers.get("Content-Length") or "0")
            if n == 0:
                return {}
            raw = self.rfile.read(n).decode("utf-8") or "{}"
            return json.loads(raw)

        def _drain_body(self) -> None:
            """Read and discard any client-sent request body. Used on
            POST handlers that don't otherwise parse a body. Windows
            clients see ConnectionReset if the body is still in flight
            when the server closes the socket after responding.
            """
            n = int(self.headers.get("Content-Length") or "0")
            if n > 0:
                try:
                    self.rfile.read(n)
                except OSError:
                    pass

        def _supervisor_call(self, cmd: str, args: dict | None = None) -> dict | None:
            try:
                return state.supervisor_call(cmd, args)
            except SupervisorRPCError as exc:
                self._json(503, {"error": str(exc)})
                return None

        def do_GET(self):  # noqa: N802
            path = self.path
            if path == "/healthz":
                self._json(200, {"ok": True, "uptime_s": time.time() - state.started_at})
                return
            if path == "/world/diagnostics":
                self._json(200, state.diagnostics())
                return
            if path == "/sim/state":
                self._json(200, state.sim_state())
                return
            if path == "/scene/tree":
                result = self._supervisor_call("scene_tree")
                if result is not None:
                    self._json(200, result)
                return
            if path == "/world/render_stats":
                if not _HAS_PIL:
                    self._json(503, {
                        "error": "Pillow is not installed; render stats require it. "
                                 "pip install Pillow"
                    })
                    return
                fd, name = tempfile.mkstemp(prefix="omnisim_harness_stats_", suffix=".png")
                os.close(fd)
                tmp_path = Path(name)
                try:
                    result = self._supervisor_call(
                        "screenshot", {"path": str(tmp_path), "quality": 100}
                    )
                    if result is None:
                        return
                    image_bytes = tmp_path.read_bytes()
                finally:
                    try:
                        tmp_path.unlink()
                    except OSError:
                        pass
                try:
                    stats = compute_render_stats(image_bytes)
                except RuntimeError as exc:
                    self._json(503, {"error": str(exc)})
                    return
                self._json(200, stats)
                return
            if path.startswith("/scene/node/"):
                def_name = path[len("/scene/node/"):]
                if not def_name:
                    self._json(400, {"error": "missing DEF in /scene/node/<def>"})
                    return
                result = self._supervisor_call("scene_node", {"def": def_name})
                if result is not None:
                    self._json(200, result)
                return

            # Strip query string for path-only routing on the damage
            # endpoints; query params are read separately below.
            parsed = urlparse(path)
            base = parsed.path

            if base == "/robot/damage":
                result = self._supervisor_call("damage_state")
                if result is not None:
                    self._json(200, result)
                return

            if base == "/robot/damage/events":
                qs = parse_qs(parsed.query)
                args: dict = {}
                if "since" in qs:
                    try:
                        args["since"] = int(qs["since"][0])
                    except ValueError:
                        self._json(400, {"error": "'since' must be an integer"})
                        return
                if "limit" in qs:
                    try:
                        args["limit"] = int(qs["limit"][0])
                    except ValueError:
                        self._json(400, {"error": "'limit' must be an integer"})
                        return
                result = self._supervisor_call("damage_events", args)
                if result is not None:
                    self._json(200, result)
                return

            if base == "/robots":
                result = self._supervisor_call("robots_list")
                if result is not None:
                    self._json(200, result)
                return

            if base == "/sim/contacts":
                result = self._supervisor_call("sim_contacts")
                if result is not None:
                    self._json(200, result)
                return

            if base == "/sim/grips":
                result = self._supervisor_call("sim_grips")
                if result is not None:
                    self._json(200, result)
                return

            if base == "/sim/events":
                # Composer: pulls supervisor-side events via the
                # events_drain RPC and merges in harness-side log
                # events. Two cursors so callers can keep them in sync
                # independently.
                qs = parse_qs(parsed.query)
                try:
                    sup_since = int(qs.get("since", ["0"])[0])
                    log_since = int(qs.get("log_since", ["0"])[0])
                    limit = int(qs.get("limit", ["256"])[0])
                except ValueError:
                    self._json(400, {
                        "error": "since/log_since/limit must be integers",
                    })
                    return
                if limit < 1:
                    limit = 1
                if limit > 1024:
                    limit = 1024
                types_csv = qs.get("types", [None])[0]
                types_list = [t for t in types_csv.split(",")
                              if t] if types_csv else None
                # Always tail the world log first so any error/warning
                # written since the last call shows up in this batch.
                state._drain_world_log_into_buffer()
                # Supervisor side (best-effort: if not connected, we
                # still return harness-side log events).
                sup_args: dict = {"since": sup_since, "limit": limit}
                if types_list is not None:
                    sup_args["types"] = types_list
                sup_result: dict = {
                    "events": [], "next_seq": sup_since,
                    "total": 0, "dropped": 0, "buffered": 0,
                }
                try:
                    sup_result = state.supervisor_call(
                        "events_drain", sup_args
                    )
                except SupervisorRPCError:
                    pass
                log_events = state.log_buffer.since(
                    log_since, limit, types=types_list
                )
                events: list[dict] = []
                for evt in sup_result.get("events", []):
                    e = dict(evt)
                    e["source"] = "sup"
                    events.append(e)
                for evt in log_events:
                    e = dict(evt)
                    e["source"] = "log"
                    events.append(e)
                next_log_since = (
                    log_events[-1]["seq"] if log_events else log_since
                )
                self._json(200, {
                    "events": events,
                    "next_since": sup_result.get("next_seq", sup_since),
                    "next_log_since": next_log_since,
                    "dropped_sup": sup_result.get("dropped", 0),
                    "dropped_log": state.log_buffer.dropped,
                })
                return

            # /robot/<def>/joints, /robot/<def>/devices,
            # /robot/<def>/sensor/<name> — split AFTER /robot/damage
            # routes have had their chance to match.
            if base.startswith("/robot/"):
                parts = base[1:].split("/")
                if len(parts) >= 3:
                    def_name = parts[1]
                    suffix = parts[2]
                    if suffix == "joints":
                        result = self._supervisor_call(
                            "robot_joints", {"def": def_name}
                        )
                        if result is not None:
                            self._json(200, result)
                        return
                    if suffix == "devices":
                        result = self._supervisor_call(
                            "robot_devices", {"def": def_name}
                        )
                        if result is not None:
                            self._json(200, result)
                        return
                    if suffix == "sensor" and len(parts) >= 4:
                        # The supervisor cannot read live sensor values
                        # from devices owned by other robots' controllers.
                        # Return 501 with the workaround so the agent
                        # can branch deterministically rather than
                        # eyeball-debugging an empty payload. /joints
                        # already covers the common case (joint
                        # positions == PositionSensor values).
                        sensor_name = "/".join(parts[3:])
                        self._json(501, {
                            "error": (
                                "live sensor reads not supported from the "
                                "supervisor (OmniSim restricts device APIs "
                                "to the controller that owns the device). "
                                "Use /robot/<def>/joints for joint positions, "
                                "or run a per-robot helper controller that "
                                "exports its sensor data over its own "
                                "endpoint."
                            ),
                            "robot": def_name,
                            "sensor": sensor_name,
                        })
                        return

            self._json(404, {"error": f"not found: {path}"})

        def do_POST(self):  # noqa: N802
            path = self.path
            if path == "/world/load":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                world_path = body.get("path")
                if not isinstance(world_path, str) or not world_path:
                    self._json(400, {"error": "path is required"})
                    return
                with_supervisor = bool(body.get("with_supervisor", True))
                default_wait = DEFAULT_LOAD_WAIT_S if with_supervisor else DEFAULT_LOAD_WAIT_BARE_S
                wait_s = body.get("wait_s", default_wait)
                try:
                    wait_s = float(wait_s)
                except (TypeError, ValueError):
                    self._json(400, {"error": "wait_s must be a number"})
                    return
                result = state.load_world(world_path, wait_s, with_supervisor)
                self._json(200 if result.get("ok") else 422, result)
                return

            if path == "/sim/step":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                steps = int(body.get("steps", 1))
                if steps < 1:
                    self._json(400, {"error": "steps must be >= 1"})
                    return
                result = self._supervisor_call("step", {"steps": steps})
                if result is not None:
                    self._json(200, result)
                return

            if path == "/sim/reset":
                result = self._supervisor_call("reset")
                if result is not None:
                    self._json(200, result)
                return

            if path == "/robot/damage/reset":
                # Drain any client-sent body so the socket can be closed
                # cleanly. Windows clients see a ConnectionReset otherwise
                # when their request body is still in flight as the
                # response arrives.
                self._drain_body()
                result = self._supervisor_call("damage_reset")
                if result is not None:
                    self._json(200, result)
                return

            if path == "/robot/damage/inject":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                part = body.get("part")
                if not isinstance(part, str) or not part:
                    self._json(400, {"error": "'part' is required"})
                    return
                args: dict = {"part": part}
                if "hp_delta" in body:
                    try:
                        args["hp_delta"] = float(body["hp_delta"])
                    except (TypeError, ValueError):
                        self._json(400, {"error": "'hp_delta' must be a number"})
                        return
                if "state" in body:
                    args["state"] = body["state"]
                result = self._supervisor_call("damage_inject", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/scene/look_at":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                position = body.get("position")
                target = body.get("target")
                if not isinstance(position, list) or len(position) != 3:
                    self._json(400, {"error": "position must be a list of 3 numbers"})
                    return
                if not isinstance(target, list) or len(target) != 3:
                    self._json(400, {"error": "target must be a list of 3 numbers"})
                    return
                try:
                    pos_f = [float(x) for x in position]
                    tgt_f = [float(x) for x in target]
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad number: {exc}"})
                    return
                orientation = compute_look_at_orientation(pos_f, tgt_f)
                push = bool(body.get("push", True))
                pushed: dict | None = None
                if push:
                    pushed = self._supervisor_call(
                        "set_viewpoint", {"position": pos_f, "orientation": orientation}
                    )
                    if pushed is None:
                        return
                self._json(200, {
                    "position": pos_f,
                    "target": tgt_f,
                    "orientation": orientation,
                    "pushed": push,
                })
                return

            if path == "/world/screenshot":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                quality = int(body.get("quality", 90))
                user_path = body.get("path")
                tmp_path: Path
                if isinstance(user_path, str) and user_path:
                    tmp_path = Path(user_path)
                else:
                    fd, name = tempfile.mkstemp(prefix="omnisim_harness_", suffix=".png")
                    os.close(fd)
                    tmp_path = Path(name)
                result = self._supervisor_call(
                    "screenshot", {"path": str(tmp_path), "quality": quality}
                )
                if result is None:
                    return
                if isinstance(user_path, str) and user_path:
                    self._json(200, {"path": str(tmp_path)})
                    return
                try:
                    image_bytes = tmp_path.read_bytes()
                finally:
                    try:
                        tmp_path.unlink()
                    except OSError:
                        pass
                self._png(image_bytes)
                return

            self._json(404, {"error": f"not found: {path}"})

    return Handler


def _tcp_port_in_use(host: str, port: int, timeout: float = 0.3) -> bool:
    """Return True if `host:port` accepts a TCP connection within `timeout`."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        return sock.connect_ex((host, port)) == 0
    finally:
        sock.close()


def _find_free_port_pair(
    host: str, start_port: int, max_pairs: int = 50
) -> tuple[int, int] | None:
    """Scan ``(p, p+1)`` pairs starting at ``start_port`` and return the
    first pair where both ports are free, or ``None`` if no free pair is
    found within ``max_pairs`` attempts.

    Pairs step by 2 so the HTTP and supervisor ports stay adjacent (the
    convention the rest of the system assumes when only one is named).
    """
    port = start_port
    for _ in range(max_pairs):
        if not _tcp_port_in_use(host, port) and not _tcp_port_in_use(host, port + 1):
            return port, port + 1
        port += 2
    return None


def probe_existing_harness(
    host: str, port: int, timeout: float = 0.6
) -> dict | None:
    """Identify what (if anything) is listening on `host:port`.

    Returns:
        None — nothing is listening; safe to bind.
        {"kind": "harness", "state": {...}} — an OmniSim harness is up; the
            payload is its `/sim/state` body so the caller can show the
            agent which world/binary the existing session is using.
        {"kind": "non_harness_http"} — port answers HTTP but not as us.
        {"kind": "non_http"} — port is bound but not HTTP-speaking.

    The shape lets `serve()` branch on the failure mode without crashing
    on `OSError: [Errno 10048]` (Windows) / EADDRINUSE — the agent gets a
    structured message it can act on instead of an opaque traceback.
    """
    if not _tcp_port_in_use(host, port, timeout=timeout):
        return None
    url = f"http://{host}:{port}/sim/state"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "omnisim-harness-preflight"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", errors="replace")
        try:
            state = json.loads(body)
        except json.JSONDecodeError:
            return {"kind": "non_harness_http"}
        # The harness's /sim/state always carries these keys; use them as
        # a fingerprint so we don't false-positive on an unrelated HTTP
        # service that happens to return JSON.
        if isinstance(state, dict) and "webots_home" in state and "binary" in state:
            return {"kind": "harness", "state": state}
        return {"kind": "non_harness_http"}
    except (urllib.error.URLError, OSError, ConnectionError, TimeoutError):
        return {"kind": "non_http"}


def _format_existing_harness_guidance(
    host: str,
    port: int,
    requested_supervisor_port: int,
    state: dict,
) -> str:
    """Build the structured 'a harness is already running' message.

    The format is shaped for an LLM agent reading a tool's stderr: short
    lines, fixed-width fields, two clearly numbered options, and copy-
    pasteable commands. The agent can either reuse the existing
    harness (cheap; hot reload swaps the world in ~600 ms) or start a
    parallel one on a different port pair. We pick `port + 100` for the
    suggestion to stay well clear of `:6790`/`:6791` (supervisor +
    capture) so the suggested numbers are always free in a fresh repo.
    """
    uptime = state.get("uptime_s")
    try:
        uptime_str = f"{float(uptime):.0f}s" if uptime is not None else "?"
    except (TypeError, ValueError):
        uptime_str = "?"
    parallel_http = port + 100
    parallel_sup = parallel_http + 1
    sup_status = "connected" if state.get("supervisor_connected") else "(not connected)"
    lines = [
        f"[harness] another OmniSim harness is already running on http://{host}:{port}",
        "[harness]",
        f"[harness]   uptime         {uptime_str}",
        f"[harness]   omnisim binary  {state.get('binary') or '?'}",
        f"[harness]   omnisim home    {state.get('webots_home') or '?'}",
        f"[harness]   current world  {state.get('world') or '(none loaded)'}",
        f"[harness]   load_ok        {state.get('load_ok')}",
        f"[harness]   supervisor     {sup_status}",
        "[harness]",
        "[harness] you have two options:",
        "[harness]",
        "[harness] (1) reuse the running harness  (recommended; cheapest path,",
        "[harness]     hot reload swaps the world in ~600 ms without restarting OmniSim):",
        "[harness]",
        f"[harness]       curl -X POST http://{host}:{port}/world/load \\",
        "[harness]            -H 'Content-Type: application/json' \\",
        "[harness]            -d '{\"path\": \"<your-world.wbt>\"}'",
        "[harness]",
        f"[harness]     all read-only endpoints (/sim/state, /scene/tree, /world/screenshot,",
        f"[harness]     /sim/events, /robots, /robot/damage) are already live on :{port};",
        "[harness]     if you only need to inspect state, just call them directly.",
        "[harness]",
        "[harness] (2) start a parallel harness on a separate port pair  (use this if the",
        "[harness]     other session is mid-task and you don't want to disturb its world):",
        "[harness]",
        f"[harness]       python -m omnisim harness --port {parallel_http} \\",
        f"[harness]                                  --supervisor-port {parallel_sup}",
        "[harness]",
        "[harness]     (--supervisor-port defaults to --port + 1; you can omit it if",
        "[harness]      that's free.)",
        "[harness]",
        f"[harness] refusing to bind on :{port}; pick one of the options above.",
    ]
    return "\n".join(lines)


def _print_to_stderr(text: str) -> None:
    sys.stderr.write(text + "\n")
    sys.stderr.flush()


def serve(
    host: str, port: int, supervisor_port: int, *, auto_port: bool = False
) -> int:
    # Preflight: refuse to start (with actionable guidance) if anything is
    # already on either port, so an agent in a parallel session sees a
    # clean explanation instead of `OSError: [WinError 10048]`.
    existing = probe_existing_harness(host, port)
    sup_busy = _tcp_port_in_use(host, supervisor_port)
    if auto_port and (existing is not None or sup_busy):
        # Skip the configured pair and scan upward for a free one. Start
        # two above the requested port so we don't immediately retry the
        # busy pair on the first iteration of the scan.
        pair = _find_free_port_pair(host, max(port, supervisor_port) + 2)
        if pair is None:
            _print_to_stderr(
                f"[harness] --auto-port: could not find a free port pair near "
                f":{port}; pass `--port <free>` manually."
            )
            return 2
        chosen_port, chosen_sup = pair
        _print_to_stderr(
            f"[harness] --auto-port: :{port}/:{supervisor_port} taken; "
            f"using :{chosen_port}/:{chosen_sup} instead."
        )
        port, supervisor_port = chosen_port, chosen_sup
        existing = None
        sup_busy = False
    if existing is not None:
        kind = existing.get("kind")
        if kind == "harness":
            _print_to_stderr(_format_existing_harness_guidance(
                host, port, supervisor_port, existing.get("state") or {}
            ))
        elif kind == "non_harness_http":
            _print_to_stderr(
                f"[harness] port {host}:{port} is already bound by a non-OmniSim HTTP "
                f"service; pass `--port <free>` and `--supervisor-port <free+1>`, "
                f"or re-run with `--auto-port` to pick a free pair automatically."
            )
        else:
            _print_to_stderr(
                f"[harness] port {host}:{port} is already bound (not HTTP); "
                f"pass `--port <free>` and `--supervisor-port <free+1>`, "
                f"or re-run with `--auto-port` to pick a free pair automatically."
            )
        return 2
    if sup_busy:
        _print_to_stderr(textwrap.dedent(f"""\
            [harness] supervisor port {host}:{supervisor_port} is already in use.
            [harness] this is the TCP port the injected supervisor controller binds inside the
            [harness] OmniSim subprocess; if it is taken, world loads will fail.
            [harness] either stop the conflicting service, pick a free pair, e.g.:
            [harness]
            [harness]     python -m omnisim harness --port {port + 100} --supervisor-port {port + 101}
            [harness]
            [harness] or re-run with `--auto-port` to pick a free pair automatically.""").rstrip())
        return 2

    state = HarnessState(
        infer_webots_home(),
        supervisor_host=SUPERVISOR_HOST,
        supervisor_port=supervisor_port,
    )
    try:
        server = ThreadingHTTPServer((host, port), make_handler(state))
    except OSError as exc:
        # TOCTOU: another process bound the port between probe and bind.
        # Re-run the probe so the agent still gets the structured guidance
        # rather than a raw OSError trace.
        late = probe_existing_harness(host, port)
        if late is not None and late.get("kind") == "harness":
            _print_to_stderr(_format_existing_harness_guidance(
                host, port, supervisor_port, late.get("state") or {}
            ))
        else:
            _print_to_stderr(
                f"[harness] could not bind {host}:{port}: {exc}. "
                f"pass `--port <free>` and `--supervisor-port <free+1>`."
            )
        return 2

    def handle_signal(signum, _frame):
        print(f"[harness] signal {signum} received; shutting down")
        state.shutdown()
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handle_signal)
        except (ValueError, OSError):
            pass

    print(f"[harness] listening on http://{host}:{port}")
    print(f"[harness] omnisim binary: {state.binary}")
    print(f"[harness] omnisim home:   {state.webots_home}")
    print(f"[harness] log file:      {state.log_path}")
    print(f"[harness] supervisor:    {state.supervisor_host}:{state.supervisor_port}")
    if not _HAS_PIL:
        print(
            "[harness] note: Pillow not installed; /world/render_stats will return 503. "
            "pip install Pillow",
            file=sys.stderr,
        )
    try:
        server.serve_forever()
    finally:
        state.shutdown()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="OmniSim agent-facing validation harness")
    p.add_argument("--host", default=DEFAULT_HOST, help=f"bind host (default: {DEFAULT_HOST}, loopback only)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"bind port (default: {DEFAULT_PORT})")
    p.add_argument(
        "--supervisor-port",
        type=int,
        default=None,
        help=(
            f"TCP port the injected supervisor controller binds inside the "
            f"OmniSim subprocess (default: --port + 1, i.e. {DEFAULT_PORT + 1} "
            f"when --port is the default). Pass an explicit value to run a "
            f"second harness alongside an existing one."
        ),
    )
    p.add_argument(
        "--auto-port",
        action="store_true",
        help=(
            "If the requested port pair is already in use, scan upward for "
            "a free (port, port+1) pair and bind there instead of failing. "
            "The chosen ports are printed to stderr so callers/agents can "
            "discover the actual listening address."
        ),
    )
    args = p.parse_args()
    supervisor_port = args.supervisor_port if args.supervisor_port is not None else args.port + 1
    return serve(args.host, args.port, supervisor_port, auto_port=args.auto_port)


if __name__ == "__main__":
    sys.exit(main())
