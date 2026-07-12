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

"""OmniSim agent-facing capture service.

A long-running HTTP service that wraps an OmniSim subprocess for cinematic
output: high-resolution stills, real-time movie recording, and
deterministic offline frame-sequence renders that get encoded to mp4 /
prores via ffmpeg.

Sister to scripts/harness/omnisim_harness.py — same shape (HTTP + sibling
supervisor injection over a length-prefixed JSON socket), different
defaults and endpoints. Runs on a different port (6791 vs 6789) so both
can coexist while you iterate on a world in the harness and capture in
this service.

Endpoints
---------
POST /world/load             {"path", "wait_s"?, "width"?, "height"?, "fov"?}
POST /capture/camera         {"position", "target"|"orientation", "sync_viewpoint"?}
POST /capture/screenshot     {"path"?, "quality"?, "source"?}    -> image/png or {path}
POST /capture/movie/start    {"path", "width"?, "height"?, "codec"?, "quality"?,
                               "acceleration"?, "caption"?, "fps"?}
POST /capture/movie/stop
GET  /capture/movie/status
POST /capture/sequence       {"path_keyframes": [...], "duration_s": float, "fps": int,
                               "output": "...", "codec"?, "crf"?, "ease"?,
                               "warmup_steps"?, "settle_steps_per_frame"?,
                               "keep_frames"?}
POST /sim/step               {"steps"?: int}
POST /sim/reset
GET  /sim/state
GET  /healthz

The /capture/sequence endpoint is the cinematic path: it
  1. samples the camera path at every output frame (fps * duration_s),
  2. for each frame: pushes the pose to the supervisor, optionally takes
     N "settle" steps, then dumps a lossless PNG frame from the Camera
     device,
  3. invokes ffmpeg on the resulting frame_%06d.png sequence.

Output resolution is fixed by the Camera device baked into the sibling
world stanza on /world/load (defaults 1920x1080). Pass `width`/`height`
on /world/load to change it; resolution changes require a cold reload
(the value is in the .wbt).
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

from camera_path import normalize_keyframes, sample_path_uniform  # noqa: E402
from encode import encode_sequence, find_ffmpeg  # noqa: E402

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 6791
DEFAULT_LOAD_WAIT_S = 8.0
MAX_LOAD_WAIT_S = 120.0

SUPERVISOR_HOST = "127.0.0.1"
SUPERVISOR_PORT = int(os.environ.get("OMNISIM_CAPTURE_SUPERVISOR_PORT_OVERRIDE", "6792"))
SUPERVISOR_CONNECT_TIMEOUT_S = 60.0
SUPERVISOR_RPC_TIMEOUT_S = 120.0
SUPERVISOR_POLL_INTERVAL_S = 0.1

DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080
DEFAULT_FOV = 0.785398  # 45deg
DEFAULT_OUTPUT_DIR = REPO_ROOT / "social" / "youtube_videos" / "captures"


def supervisor_stanza(width: int, height: int, fov: float) -> str:
    """Robot stanza appended to the user's world. Embeds a Camera device at
    the requested resolution; the controller enables it on startup.

    `synchronization TRUE` is critical here (and the difference from the
    validation harness, which uses FALSE). With sync=FALSE, the simulator
    runs at its own pace and our `step(N)` calls only advance the
    supervisor's local clock — the actual sim state per frame is whatever
    happens to be current when frame_dump runs, making
    settle_steps_per_frame and playback_speed effectively no-ops. With
    sync=TRUE, the supervisor gates the sim: each step(N) advances the
    world by exactly N basic timesteps before the next frame is dumped,
    which is the determinism the capture pipeline depends on.
    """
    # NOTE: Camera device in the supervisor stanza currently crashes Webots
    # in --batch render mode. Until that's fixed, screenshots use
    # Supervisor.exportImage (viewport-bound). Resolution baked here is
    # advisory: it sets viewpoint zoom hints, not the output size.
    _ = (int(width), int(height), float(fov))
    return """
Robot {
  name "capture_supervisor"
  controller "capture_supervisor"
  supervisor TRUE
  synchronization TRUE
  translation 0 0 5
  rotation 0 1 0 0
}
"""


def infer_webots_home() -> Path:
    return Path(os.environ.get("OMNISIM_HOME", str(REPO_ROOT)))


def resolve_webots_binary(webots_home: Path) -> Path:
    """Resolve the real simulator binary, NOT the launcher.

    On Windows, `webots.exe` is a thin launcher that spawns `omnisim-bin.exe`
    as its child. Spawning the launcher means our subprocess handle points
    at the launcher, not the actual sim — and `proc.terminate()` then kills
    only the launcher, leaving `omnisim-bin.exe` (a 400+ MB orphan) running.
    Always spawn `omnisim-bin.exe` directly so the subprocess handle controls
    the real sim. PATH must already include the bundled mingw64 `bin/` for
    DLL resolution; the capture service prepends it before spawning.
    """
    if sys.platform == "win32":
        candidates = [
            webots_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            webots_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [webots_home / "Contents" / "MacOS" / "webots"]
    else:
        candidates = [webots_home / "bin" / "omnisim-bin", webots_home / "webots"]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise RuntimeError(
        f"Cannot find the OmniSim binary under {webots_home}. "
        "Set OMNISIM_HOME to a built tree or install location."
    )


def sibling_path_for(world: Path) -> Path:
    return world.with_name(f".capture_{world.name}")


def _port_is_open(host: str, port: int, timeout_s: float = 0.3) -> bool:
    """Return True if anything is currently listening on host:port. Used to
    detect orphan capture supervisors from a previous render before we try
    to spawn a new one (and silently connect to the orphan instead).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_s)
    try:
        sock.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        try:
            sock.close()
        except OSError:
            pass


def write_sibling_world(original: Path, width: int, height: int, fov: float) -> Path:
    sibling = sibling_path_for(original)
    content = original.read_text(encoding="utf-8", errors="replace")
    if not content.endswith("\n"):
        content += "\n"
    content += supervisor_stanza(width, height, fov)
    sibling.write_text(content, encoding="utf-8")
    return sibling


# ---------------------------------------------------------------------------
# Supervisor IPC (same wire format as harness — share the shape, not the impl,
# since this service has different timeout defaults and no hot-reload path)
# ---------------------------------------------------------------------------


class SupervisorRPCError(Exception):
    pass


class SupervisorClient:
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
            f"could not connect to capture supervisor at {host}:{port} within {deadline_s:.1f}s "
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
                raise SupervisorRPCError("capture supervisor is not connected")
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
                raise SupervisorRPCError("capture supervisor closed the connection")
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
# Capture state
# ---------------------------------------------------------------------------


class CaptureServiceState:
    def __init__(self, webots_home: Path):
        self.webots_home = webots_home
        self.binary = resolve_webots_binary(webots_home)
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.current_world: str | None = None
        self.current_sibling: Path | None = None
        self.supervisor: SupervisorClient | None = None
        self.supervisor_connected_at: float | None = None
        self.last_camera_resolution: tuple[int, int] | None = None
        self.last_load_ms: int | None = None
        self.last_load_ok: bool | None = None
        self.started_at = time.time()
        self._sim_log_handle = None

    def _kill_running(self) -> None:
        if self.supervisor is not None:
            self.supervisor.close()
            self.supervisor = None
            self.supervisor_connected_at = None
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                # On Windows, terminating omnisim-bin leaves its python
                # controller children orphaned, and those children keep the
                # supervisor TCP port bound — blocking the next /world/load.
                # Kill the whole process tree.
                if sys.platform == "win32":
                    try:
                        subprocess.run(
                            ["taskkill", "/F", "/T", "/PID", str(self.proc.pid)],
                            timeout=10, capture_output=True, check=False,
                        )
                    except Exception:
                        self.proc.terminate()
                else:
                    self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=2)
            except Exception as exc:
                print(f"[capture] error terminating prior subprocess: {exc}", file=sys.stderr)
        self.proc = None
        if self._sim_log_handle is not None:
            try:
                self._sim_log_handle.close()
            except Exception:
                pass
            self._sim_log_handle = None

    def _cleanup_sibling(self) -> None:
        if self.current_sibling is not None:
            try:
                self.current_sibling.unlink()
            except OSError:
                pass
            self.current_sibling = None

    def _try_connect_supervisor(self, deadline: float) -> SupervisorClient | None:
        while time.time() < deadline:
            if self.proc is None or self.proc.poll() is not None:
                return None
            client = SupervisorClient()
            try:
                client.connect(SUPERVISOR_HOST, SUPERVISOR_PORT, deadline_s=0.5)
                client.call("ping")
                return client
            except SupervisorRPCError:
                client.close()
                time.sleep(SUPERVISOR_POLL_INTERVAL_S)
        return None

    def load_world(
        self,
        world_path: str,
        wait_s: float,
        width: int,
        height: int,
        fov: float,
    ) -> dict:
        world = Path(world_path)
        if not world.is_absolute():
            world = REPO_ROOT / world
        if not world.exists():
            return {"ok": False, "error": f"world not found: {world}", "load_ms": 0}

        wait_s = max(1.0, min(MAX_LOAD_WAIT_S, float(wait_s)))
        started_at = time.time()

        # Detect orphan supervisors on our port before launching. Otherwise
        # the new OmniSim subprocess we spawn will fail to bind, our connect
        # poll will hit the stale orphan instead, and the render will
        # silently use whatever world the orphan is in (typically the
        # previous run's, which leaves frames with stale physics state).
        # First kill anything we ourselves spawned before checking for orphans —
        # otherwise a successful prior load (which left omnisim-bin alive) trips
        # the orphan check and blocks every subsequent /world/load. We then
        # poll briefly because the controller's python child may outlive the
        # omnisim-bin parent on Windows and keep the supervisor port bound.
        with self.lock:
            self._kill_running()
        port_release_deadline = time.time() + 8.0
        while _port_is_open(SUPERVISOR_HOST, SUPERVISOR_PORT):
            if time.time() >= port_release_deadline:
                return {
                    "ok": False,
                    "error": (
                        f"capture supervisor port {SUPERVISOR_HOST}:{SUPERVISOR_PORT} is "
                        "still in use after killing our prior subprocess — possibly an "
                        "orphan from a previous run. On Windows: `taskkill /F /IM omnisim-bin.exe`."
                    ),
                    "load_ms": 0,
                }
            time.sleep(0.25)

        with self.lock:
            self._cleanup_sibling()
            self.current_world = str(world)
            self.last_camera_resolution = (width, height)
            try:
                self.current_sibling = write_sibling_world(world, width, height, fov)
            except OSError as exc:
                return {"ok": False, "error": f"could not write capture sibling: {exc}", "load_ms": 0}

            cmd = [
                str(self.binary),
                str(self.current_sibling),
                "--batch",
                "--mode=fast",
                "--stdout",
                "--stderr",
            ]
            env = os.environ.copy()
            env["OMNISIM_HOME"] = str(self.webots_home)
            env["OMNISIM_CAPTURE_SUPERVISOR_HOST"] = SUPERVISOR_HOST
            env["OMNISIM_CAPTURE_SUPERVISOR_PORT"] = str(SUPERVISOR_PORT)
            if sys.platform == "win32":
                env["PATH"] = str(self.binary.parent) + ";" + env.get("PATH", "")

            # Pipe OmniSim stdout/stderr to a log file so a crash mid-render
            # leaves something diagnosable. The previous DEVNULL redirect
                # threw away the death message and made "supervisor RPC closed"
            # errors a guessing game (4K + heavy fleet → renderer OOM was
            # the suspected culprit and we couldn't confirm without this).
            log_path = REPO_ROOT / "social" / "youtube_videos" / "captures" / ".capture_sim.log"
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self._sim_log_handle = open(log_path, "wb")
            except OSError:
                self._sim_log_handle = None
            try:
                self.proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=self._sim_log_handle or subprocess.DEVNULL,
                    stderr=subprocess.STDOUT if self._sim_log_handle else subprocess.DEVNULL,
                )
            except OSError as exc:
                self._cleanup_sibling()
                return {"ok": False, "error": f"failed to launch simulator: {exc}", "load_ms": 0}

        deadline = started_at + wait_s
        client = self._try_connect_supervisor(deadline)
        load_ms = int((time.time() - started_at) * 1000)

        if client is None:
            with self.lock:
                self.last_load_ok = False
                self.last_load_ms = load_ms
            return {
                "ok": False,
                "error": f"capture supervisor did not bind within {wait_s:.1f}s",
                "load_ms": load_ms,
            }

        with self.lock:
            self.supervisor = client
            self.supervisor_connected_at = time.time()
            self.last_load_ok = True
            self.last_load_ms = load_ms

        # Ask the supervisor what resolution it actually got, so we surface it.
        cam_state: dict = {}
        try:
            sim = client.call("sim_state")
            cam_state = sim.get("camera") or {}
        except SupervisorRPCError:
            pass

        return {
            "ok": True,
            "world": str(world),
            "load_ms": load_ms,
            "camera": cam_state,
        }

    def supervisor_call(self, cmd: str, args: dict | None = None) -> dict:
        with self.lock:
            client = self.supervisor
        if client is None or not client.is_connected():
            raise SupervisorRPCError("capture supervisor not connected (load a world first)")
        return client.call(cmd, args)

    def sim_state(self) -> dict:
        with self.lock:
            running = self.proc is not None and self.proc.poll() is None
            return {
                "world": self.current_world,
                "running": running,
                "load_ok": self.last_load_ok,
                "load_ms": self.last_load_ms,
                "camera_resolution": list(self.last_camera_resolution) if self.last_camera_resolution else None,
                "supervisor_connected": self.supervisor is not None and self.supervisor.is_connected(),
                "supervisor_connected_at": self.supervisor_connected_at,
                "binary": str(self.binary),
            }

    def shutdown(self) -> None:
        with self.lock:
            self._kill_running()
            self._cleanup_sibling()


# ---------------------------------------------------------------------------
# Sequence rendering (the cinematic offline path)
# ---------------------------------------------------------------------------


def resolve_settle_steps(
    *,
    explicit_settle: int | None,
    playback_speed: float | None,
    fps: int,
    basic_time_step_ms: int,
) -> tuple[int, float]:
    """Decide how many sim ticks each output frame consumes, and report the
    resulting playback speed so the caller can log it back.

    Priority: an explicit `settle_steps_per_frame` always wins. Otherwise
    `playback_speed` (1.0 = real-time, 0.5 = half-speed slow-mo, 2.0 =
    2x time-lapse) is converted to settle steps via:

        settle = round(playback_speed * (1000 / fps) / basic_time_step_ms)

    Clamped to >=1 so the renderer always advances at least one tick per
    frame. With basic_time_step_ms=16 and fps=30, playback_speed=0.5
    resolves to settle=1 (actual ratio 0.48); playback_speed=1.0 resolves
    to settle=2 (actual ratio 0.96).
    """
    if explicit_settle is not None:
        settle = max(1, int(explicit_settle))
        actual_ratio = (settle * basic_time_step_ms) * fps / 1000.0
        return settle, actual_ratio
    if playback_speed is not None:
        target_ms = float(playback_speed) * 1000.0 / max(1, fps)
        settle = max(1, round(target_ms / max(1, basic_time_step_ms)))
        actual_ratio = (settle * basic_time_step_ms) * fps / 1000.0
        return settle, actual_ratio
    # Default: 1 tick per frame.
    settle = 1
    actual_ratio = (settle * basic_time_step_ms) * fps / 1000.0
    return settle, actual_ratio


def _render_sequence(
    state: CaptureServiceState,
    keyframes: list[dict],
    duration_s: float,
    fps: int,
    output: Path,
    codec: str,
    crf: int,
    ease: str,
    warmup_steps: int,
    settle_steps_per_frame: int,
    keep_frames: bool,
    preset: str = "slow",
    lossless: bool = False,
    playback_ratio: float | None = None,
) -> dict:
    """Drive the supervisor frame-by-frame, dump PNGs, then ffmpeg-encode."""
    samples = sample_path_uniform(keyframes, duration_s, fps, ease=ease)

    # Use a deterministic temp dir alongside the output so failures leave a
    # debuggable artifact. Cleaned up on success unless keep_frames=true.
    output.parent.mkdir(parents=True, exist_ok=True)
    frames_dir = output.parent / f".{output.stem}_frames"
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True)

    started_at = time.time()
    try:
        if warmup_steps > 0:
            state.supervisor_call("step", {"steps": warmup_steps})

        for i, (pos, ori) in enumerate(samples):
            state.supervisor_call(
                "set_camera_pose",
                {"position": pos, "orientation": ori, "sync_viewpoint": True},
            )
            if settle_steps_per_frame > 0:
                state.supervisor_call("step", {"steps": settle_steps_per_frame})
            else:
                # Always at least one step so the renderer flushes the new pose.
                state.supervisor_call("step", {"steps": 1})
            frame_path = frames_dir / f"frame_{i:06d}.png"
            state.supervisor_call(
                "frame_dump",
                {"path": str(frame_path), "frame_index": i, "quality": 100},
            )

        encode_result = encode_sequence(
            frames_dir, output, fps=fps, codec=codec, crf=crf,
            preset=preset, lossless=lossless,
        )
    finally:
        if not keep_frames and frames_dir.exists():
            shutil.rmtree(frames_dir, ignore_errors=True)

    return {
        "output": str(output),
        "frames": len(samples),
        "fps": fps,
        "duration_s": duration_s,
        "codec": codec,
        "render_ms": int((time.time() - started_at) * 1000),
        "settle_steps_per_frame": settle_steps_per_frame,
        "playback_ratio": playback_ratio,
        "encode": {
            "stderr_tail": encode_result.get("stderr_tail"),
            "command": encode_result.get("command"),
        },
        "frames_dir": str(frames_dir) if keep_frames else None,
    }


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------


def make_handler(state: CaptureServiceState):
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

        def _supervisor_call(self, cmd: str, args: dict | None = None) -> dict | None:
            try:
                return state.supervisor_call(cmd, args)
            except SupervisorRPCError as exc:
                self._json(503, {"error": str(exc)})
                return None

        def do_GET(self):  # noqa: N802
            path = self.path
            if path == "/healthz":
                self._json(200, {"ok": True, "uptime_s": time.time() - state.started_at,
                                  "ffmpeg": find_ffmpeg()})
                return
            if path == "/sim/state":
                self._json(200, state.sim_state())
                return
            if path == "/capture/movie/status":
                result = self._supervisor_call("movie_status")
                if result is not None:
                    self._json(200, result)
                return
            if path == "/world/robots":
                result = self._supervisor_call("list_robots")
                if result is not None:
                    self._json(200, result)
                return
            self._json(404, {"error": f"not found: {path}"})

        def do_POST(self):  # noqa: N802
            path = self.path

            try:
                body = self._read_json() if self.headers.get("Content-Length") else {}
            except Exception as exc:  # noqa: BLE001
                self._json(400, {"error": f"bad json: {exc}"})
                return

            if path == "/world/load":
                world_path = body.get("path")
                if not isinstance(world_path, str) or not world_path:
                    self._json(400, {"error": "path is required"})
                    return
                wait_s = float(body.get("wait_s", DEFAULT_LOAD_WAIT_S))
                width = int(body.get("width", DEFAULT_WIDTH))
                height = int(body.get("height", DEFAULT_HEIGHT))
                fov = float(body.get("fov", DEFAULT_FOV))
                if width <= 0 or height <= 0:
                    self._json(400, {"error": "width and height must be > 0"})
                    return
                result = state.load_world(world_path, wait_s, width, height, fov)
                self._json(200 if result.get("ok") else 422, result)
                return

            if path == "/sim/step":
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

            if path == "/world/subject":
                # Subject-pose lookup for the cinema pipeline. Pass {"name": "spot"}
                # or {"def": "HUSKY"} — returns translation + rotation so camera
                # primitives can lock onto the subject.
                args: dict = {}
                if "name" in body:
                    args["name"] = body["name"]
                if "def" in body:
                    args["def"] = body["def"]
                result = self._supervisor_call("node_pose", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/world/robots":
                result = self._supervisor_call("list_robots")
                if result is not None:
                    self._json(200, result)
                return

            if path == "/capture/camera":
                args: dict = {}
                for k in ("position", "target", "orientation"):
                    if k in body:
                        args[k] = body[k]
                if "sync_viewpoint" in body:
                    args["sync_viewpoint"] = bool(body["sync_viewpoint"])
                result = self._supervisor_call("set_camera_pose", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/capture/screenshot":
                quality = int(body.get("quality", 95))
                source = body.get("source")
                user_path = body.get("path")
                if isinstance(user_path, str) and user_path:
                    tmp_path = Path(user_path)
                    tmp_path.parent.mkdir(parents=True, exist_ok=True)
                    return_inline = False
                else:
                    fd, name = tempfile.mkstemp(prefix="omnisim_capture_", suffix=".png")
                    os.close(fd)
                    tmp_path = Path(name)
                    return_inline = True
                args = {"path": str(tmp_path), "quality": quality}
                if source:
                    args["source"] = source
                result = self._supervisor_call("screenshot", args)
                if result is None:
                    if return_inline:
                        try:
                            tmp_path.unlink()
                        except OSError:
                            pass
                    return
                if not return_inline:
                    self._json(200, {"path": str(tmp_path), **{k: v for k, v in result.items() if k != "path"}})
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

            if path == "/capture/movie/start":
                args = {k: body[k] for k in
                        ("path", "width", "height", "codec", "quality", "acceleration", "caption", "fps")
                        if k in body}
                if "path" not in args:
                    self._json(400, {"error": "path is required"})
                    return
                user_path = Path(args["path"])
                if not user_path.is_absolute():
                    user_path = DEFAULT_OUTPUT_DIR / user_path
                user_path.parent.mkdir(parents=True, exist_ok=True)
                args["path"] = str(user_path)
                result = self._supervisor_call("movie_start", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/capture/movie/stop":
                result = self._supervisor_call("movie_stop")
                if result is not None:
                    self._json(200, result)
                return

            if path == "/capture/sequence":
                keyframes = body.get("path_keyframes")
                duration_s = body.get("duration_s")
                fps = body.get("fps")
                output = body.get("output")
                if keyframes is None or duration_s is None or fps is None or output is None:
                    self._json(400, {"error": "path_keyframes, duration_s, fps, output are required"})
                    return
                try:
                    norm = normalize_keyframes(keyframes)
                except ValueError as exc:
                    self._json(400, {"error": f"bad path_keyframes: {exc}"})
                    return
                output_path = Path(output)
                if not output_path.is_absolute():
                    output_path = DEFAULT_OUTPUT_DIR / output_path
                codec = str(body.get("codec", "h264"))
                crf = int(body.get("crf", 16))
                ease = str(body.get("ease", "smoothstep"))
                warmup_steps = int(body.get("warmup_steps", 0))
                explicit_settle = body.get("settle_steps_per_frame")
                playback_speed = body.get("playback_speed")
                keep_frames = bool(body.get("keep_frames", False))
                preset = str(body.get("preset", "slow"))
                lossless = bool(body.get("lossless", False))

                # Resolve settle_steps from playback_speed if needed. We need
                # the world's basic_time_step from the supervisor; ask now so
                # the response can report the actual playback ratio.
                try:
                    sim = state.supervisor_call("sim_state")
                except SupervisorRPCError as exc:
                    self._json(503, {"error": str(exc)})
                    return
                basic_step_ms = int(sim.get("basic_time_step_ms", 16))
                settle_steps_per_frame, actual_ratio = resolve_settle_steps(
                    explicit_settle=int(explicit_settle) if explicit_settle is not None else None,
                    playback_speed=float(playback_speed) if playback_speed is not None else None,
                    fps=int(fps),
                    basic_time_step_ms=basic_step_ms,
                )

                try:
                    result = _render_sequence(
                        state,
                        norm,
                        float(duration_s),
                        int(fps),
                        output_path,
                        codec=codec,
                        crf=crf,
                        ease=ease,
                        warmup_steps=warmup_steps,
                        settle_steps_per_frame=settle_steps_per_frame,
                        keep_frames=keep_frames,
                        preset=preset,
                        lossless=lossless,
                        playback_ratio=actual_ratio,
                    )
                except SupervisorRPCError as exc:
                    self._json(503, {"error": str(exc)})
                    return
                except (ValueError, RuntimeError) as exc:
                    self._json(500, {"error": str(exc)})
                    return
                self._json(200, result)
                return

            if path == "/shutdown":
                # Graceful exit: tear down the OmniSim subprocess in-process so
                # render.py's --ad-hoc mode can avoid leaving orphans on
                # Windows (where TerminateProcess on the service skips the
                # signal handler that would otherwise do this cleanup).
                state.shutdown()
                self._json(200, {"shutting_down": True})
                threading.Thread(
                    target=lambda: (time.sleep(0.5), os._exit(0)), daemon=True
                ).start()
                return

            self._json(404, {"error": f"not found: {path}"})

    return Handler


def serve(host: str, port: int) -> int:
    state = CaptureServiceState(infer_webots_home())
    server = ThreadingHTTPServer((host, port), make_handler(state))

    def handle_signal(signum, _frame):
        print(f"[capture] signal {signum} received; shutting down")
        state.shutdown()
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handle_signal)
        except (ValueError, OSError):
            pass

    print(f"[capture] listening on http://{host}:{port}")
    print(f"[capture] omnisim binary: {state.binary}")
    print(f"[capture] omnisim home:   {state.webots_home}")
    print(f"[capture] supervisor:    {SUPERVISOR_HOST}:{SUPERVISOR_PORT}")
    print(f"[capture] default output dir: {DEFAULT_OUTPUT_DIR}")
    ff = find_ffmpeg()
    if ff:
        print(f"[capture] ffmpeg:        {ff}")
    else:
        print("[capture] WARN: ffmpeg not found on PATH — /capture/sequence will fail at the encode step", file=sys.stderr)
    try:
        server.serve_forever()
    finally:
        state.shutdown()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="OmniSim agent-facing capture service")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = p.parse_args()
    return serve(args.host, args.port)


if __name__ == "__main__":
    sys.exit(main())
