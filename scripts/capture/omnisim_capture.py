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

Paths
-----
Every endpoint that takes an output `path` resolves a RELATIVE one against
DEFAULT_OUTPUT_DIR (social/youtube_videos/captures/) and answers with the
absolute path it actually wrote. Do not assume a relative path is relative
to your own process: the writer is a controller inside the OmniSim
subprocess and its cwd is its own controller directory.

Status codes
------------
200/204  done
400      the request is malformed or names a path this service cannot write
422      the supervisor received the request and REFUSED it (service is up)
502      the supervisor claimed success but produced no readable artefact
503      the supervisor is unreachable: no world loaded, or the transport died
"""

from __future__ import annotations

import argparse
import json
import math
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
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from camera_path import normalize_keyframes, sample_path_uniform  # noqa: E402
from encode import encode_sequence, find_ffmpeg  # noqa: E402
from omnisim.paths import linux_runtime_env  # noqa: E402

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 6791
# Demo-sized worlds cold-load in tens of seconds (warehouse_husky ~50s on a
# WSL2 box); the old 8s default made every such load fail. The caller's
# wait_s is honored as a *soft* deadline: while the engine subprocess is
# alive and its log is still growing (i.e. genuinely still loading) we keep
# waiting past it, up to the hard ceiling below.
DEFAULT_LOAD_WAIT_S = 60.0
MAX_LOAD_WAIT_S = 300.0
# Past the soft deadline, give up once the engine log has stopped growing
# for this long (the load has stalled rather than being merely slow).
LOAD_STALL_GRACE_S = 15.0

FFMPEG_MISSING_MSG = (
    "ffmpeg not found on PATH — /capture/sequence needs it to encode frames. "
    "Install it: `sudo apt install ffmpeg` (Debian/Ubuntu), `brew install ffmpeg` "
    "(macOS), or `winget install ffmpeg` / https://ffmpeg.org/download.html (Windows)."
)

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
    # HISTORY: the Camera child was removed in ce5228b26 (2026-05-15) because
    # enabling it segfaulted the then-current engine in --batch render mode,
    # which silently downgraded every "resolution-independent" screenshot to a
    # viewport-sized exportImage (316x316 under Xvfb for a 1920x1080 request).
    # Re-verified 2026-07-12 on the current engine (Windows native + Linux
    # under Xvfb/software-GL): the crash no longer reproduces, and the Camera
    # renders at the requested resolution on both. If a regression ever
    # resurfaces, set OMNISIM_CAPTURE_DISABLE_CAMERA=1 — the supervisor then
    # skips cam.enable() and the service *reports* the viewport fallback
    # honestly (camera_fallback in /world/load and screenshot responses)
    # instead of silently returning the wrong resolution.
    # ⚠ CAPTURE OUTPUT IS WREN-RENDERED, AND THAT IS CURRENTLY THE ONLY WORKING OPTION.
    # This service renders through a Camera DEVICE. Camera.renderBackend was flipped to "wgpu" on
    # 2026-08-22 and REVERTED the same day (it broke tests/api pen + camera_image_update), so the
    # default is "wren" again -- and this service was UNAFFECTED either way, because it writes the
    # field explicitly from _capture_render_backend(), which
    # still defaults to "wren". That is deliberate insulation, not an oversight: cinema output is
    # a deliverable, and it should move only on a measurement, not as a side effect of a default
    # flip elsewhere. So every still and every cinema frame it has produced came out of the LEGACY
    # renderer, WITHOUT the scattered sky, OmniLight GI, PCSS, SSR, TAA or the camera pass that
    # the main view has had since 2026-08-19. Measured on beauty_bench, the WREN capture and the
    # wgpu main view of the same world+camera differ on 98.6% of pixels (mean 169/765), so this
    # is a different image, not a slightly older look.
    #
    # ⚠️ RE-MEASURE BEFORE TRUSTING THE PARAGRAPH BELOW. The monochrome result it records was
    # measured on 2026-08-21, BEFORE the wgpu Camera gained its scene path (textures, world
    # lights, Background/sky and shadows, behind OMNISIM_WGPU_CAMERA_SCENE, which is default ON).
    # That path is very likely the exact fix for what is described here, and nobody has re-run the
    # reproducer since. Treat the next paragraph as dated evidence, not as current status.
    #
    # ⛔ The obvious fix -- flipping this Camera to wgpu -- DID NOT WORK as of 2026-08-21, measured
    # 2026-08-21 (machine 9722d23d12a3): OMNISIM_CAPTURE_BACKEND=wgpu on beauty_bench at
    # 1280x720 renders a MONOCHROME image (R=G=B=66.8, std 29.5) with no textures, no lighting
    # and blown-out geometry fragments -- identical before and after 120 settle steps, so it is
    # not a warm-up artefact. The wgpu Camera RTT path passes all 19 checks in
    # scripts/dev/wgpu_sensor_regression.py, but those run small synthetic probe worlds; this is
    # the first real, complex scene it has been pointed at, and it fails. That gap is exactly
    # phase W3 of docs/developer/wren-retirement-plan.md, and this is its reproducer:
    #   OMNISIM_CAPTURE_BACKEND=wgpu python scripts/capture/omnisim_capture.py --port 6791
    #   POST /world/load beauty_bench -> POST /capture/camera -> POST /capture/screenshot
    # Until W3 closes, cinematic output stays on WREN and the knob exists for A/B only.
    return f"""
Robot {{
  name "capture_supervisor"
  controller "capture_supervisor"
  supervisor TRUE
  synchronization TRUE
  translation 0 0 5
  rotation 0 1 0 0
  children [
    Camera {{
      name "capture_camera"
      width {int(width)}
      height {int(height)}
      fieldOfView {float(fov)}
      antiAliasing TRUE
      renderBackend "{_capture_render_backend()}"
    }}
  ]
}}
"""


def _capture_render_backend() -> str:
    """Which renderer the capture Camera uses.

    D1.5 (WREN deleted at D1.4, commit 976b9449d): "wgpu" by default -- it is the only
    renderer. A "wren" value still parses engine-side but is a warned no-op that renders
    wgpu anyway (F1). The monochrome-capture defect the old "wren" pin guarded against was
    closed by the P3/P5/P11 parity work (sensor texCache, post-FX ports, legacy textures).
    """
    v = (os.environ.get("OMNISIM_CAPTURE_BACKEND") or "").strip().lower()
    return v if v in ("wren", "wgpu", "vulkan") else "wgpu"


def infer_omnisim_home() -> Path:
    return Path(os.environ.get("OMNISIM_HOME", str(REPO_ROOT)))


def resolve_omnisim_binary(omnisim_home: Path) -> Path:
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
            omnisim_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            omnisim_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [omnisim_home / "Contents" / "MacOS" / "webots"]
    else:
        candidates = [omnisim_home / "bin" / "omnisim-bin", omnisim_home / "webots"]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    # Distinguish the two causes. A wrong OMNISIM_HOME and an unbuilt tree
    # look identical here, and telling someone to "set OMNISIM_HOME" when it
    # is already right sends them chasing the wrong thing.
    looks_like_a_checkout = (omnisim_home / "AGENTS.md").exists()
    if looks_like_a_checkout:
        raise RuntimeError(
            f"No OmniSim binary under {omnisim_home} -- this looks like a\n"
            f"checkout that has not been built yet (OMNISIM_HOME itself is fine).\n"
            f"  Build it:  build_omni.bat        (Windows)\n"
            f"             python -m omnisim build all   (other platforms)\n"
             "  then:      make -C src/omnisim bundle-newton-runtime\n"
             "  check:     python -m omnisim doctor"
        )
    raise RuntimeError(
        f"Cannot find the OmniSim binary under {omnisim_home}. "
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


def resolve_output_path(user_path: str) -> tuple[Path | None, str | None]:
    """Turn a caller's ``path`` into an ABSOLUTE path the supervisor can write.

    Returns ``(path, None)`` or ``(None, reason)``.

    WHY THIS EXISTS. The supervisor is a controller process inside the OmniSim
    subprocess and its working directory is its own controller directory
    (``projects/default/controllers/capture_supervisor/``), not this service's.
    Handing it a relative path therefore resolved somewhere the caller never
    named. Measured 2026-08-12 on a live service:

      * ``{"path": "shot.png"}``            -> HTTP 200, and the PNG landed in
        ``projects/default/controllers/capture_supervisor/shot.png`` while the
        response echoed back the bare relative name -- a success the caller
        could not act on.
      * ``{"path": "out/shot.png"}``        -> HTTP **503**
        ``camera.saveImage failed for path 'out\\shot.png'``. The service had
        created ``out/`` next to ITSELF; the supervisor looked for it next to
        the controller, did not find it, and ``Camera.saveImage`` returned -1.
      * ``{}`` on the same service, same scene, same second -> a real 32 KB PNG.

    A relative path is resolved against :data:`DEFAULT_OUTPUT_DIR`, matching
    what ``/capture/movie/start`` and ``/capture/sequence`` already do with
    theirs, so all three endpoints agree on what a bare filename means.
    """
    if not isinstance(user_path, str) or not user_path.strip():
        return None, "path must be a non-empty string"
    candidate = Path(user_path.strip())
    if not candidate.is_absolute():
        candidate = DEFAULT_OUTPUT_DIR / candidate
    candidate = Path(os.path.abspath(str(candidate)))
    if candidate.is_dir():
        return None, f"path is an existing directory: {candidate}"
    try:
        candidate.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        return None, f"cannot create the parent directory of {candidate}: {exc}"
    if not os.access(str(candidate.parent), os.W_OK):
        return None, f"parent directory is not writable: {candidate.parent}"
    return candidate, None


def png_dimensions(data: bytes) -> tuple[int, int] | None:
    """Read width/height straight out of a PNG's IHDR chunk (no PIL needed).
    Used to report the *actual* delivered resolution instead of trusting the
    supervisor's response — the defence against silent viewport downgrades.
    """
    if len(data) >= 24 and data[:8] == b"\x89PNG\r\n\x1a\n" and data[12:16] == b"IHDR":
        w, h = struct.unpack(">II", data[16:24])
        return int(w), int(h)
    return None


def sanitize_nonfinite(obj):
    """Recursively replace non-finite floats (NaN, +/-Infinity) with None.

    Python's json module happily emits bare ``NaN`` / ``Infinity`` tokens,
    which are NOT valid JSON (RFC 8259) -- non-Python clients choke on a
    response containing them. This service is exposed to them: the camera and
    viewpoint poses it echoes back come straight from the engine, and an
    uninitialised transform reads NaN. Dict keys are handled too: json.dumps
    stringifies float keys with ``repr``, which would still emit ``NaN`` as a
    key (and raises under allow_nan=False), so non-finite float keys become
    "null".

    Copied from scripts/harness/omnisim_harness.py (``sanitize_nonfinite``,
    landed 891225b5b, pinned by tests/harness/test_strict_json.py). Copied
    rather than imported for the same reason the IPC client below is a second
    implementation: capture imports nothing from scripts/harness, and pulling
    that module in for one pure function would drag its diagnostic_codes and
    spatial/omniworld dependencies into every capture process. Share the
    shape, not the impl.
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if isinstance(k, float) and not math.isfinite(k):
                k = "null"
            out[k] = sanitize_nonfinite(v)
        return out
    if isinstance(obj, (list, tuple)):
        return [sanitize_nonfinite(v) for v in obj]
    return obj


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
    """The supervisor could not be reached, or the transport broke."""


class SupervisorCommandError(SupervisorRPCError):
    """The supervisor was reached, answered, and REJECTED the request.

    Kept distinct from its parent on purpose. Both used to collapse into a
    single 503, which told the caller "this service is unavailable" for a
    request the service had in fact processed and refused -- measured on
    ``/capture/screenshot {"path": "sub/dir/shot.png"}``, which 503'd while
    ``{}`` on the same service, same scene, same second returned a real PNG.
    A rejection is a 4xx; an outage is a 503.
    """


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
                raise SupervisorCommandError(
                    response.get("error", "supervisor rejected request"))
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
    def __init__(self, omnisim_home: Path):
        self.omnisim_home = omnisim_home
        self.binary = resolve_omnisim_binary(omnisim_home)
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.current_world: str | None = None
        self.current_sibling: Path | None = None
        self.supervisor: SupervisorClient | None = None
        self.supervisor_connected_at: float | None = None
        self.last_camera_resolution: tuple[int, int] | None = None
        self.last_camera_state: dict = {}
        self.last_load_ms: int | None = None
        self.last_load_ok: bool | None = None
        self.started_at = time.time()
        self._sim_log_handle = None
        # The ENGINE's own log (OmLog), pinned to a capture-owned file.
        # Unset, OmLog::initFileLog (src/omnisim/core/OmLog.cpp:56-97) falls
        # back to <OMNISIM_HOME>/omnisim_log.txt -- the exact file a validation
        # harness on :6789 tails by byte offset -- opens it with
        # std::ios::trunc, and then QFile::remove()s the "<log>.newton.json"
        # backend-verdict sidecar next to it. One /world/load here would
        # therefore wipe the harness's diagnostics AND its proof that Newton
        # drove the run. The two services are built to run side by side
        # (6791 vs 6789, see the module docstring), so they must never share a
        # log file. OMNISIM_CAPTURE_LOG_PATH overrides it (same convention as
        # OMNISIM_CAPTURE_SUPERVISOR_PORT_OVERRIDE); a bare OMNISIM_LOG_PATH is
        # deliberately NOT read here, because the one shell that starts both
        # services would then aim them back at a single file -- the bug.
        _log_override = os.environ.get("OMNISIM_CAPTURE_LOG_PATH")
        self.engine_log_path = (
            Path(_log_override) if _log_override
            else DEFAULT_OUTPUT_DIR / ".capture_engine_log.txt"
        )

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

    def _wait_for_supervisor(
        self,
        soft_deadline: float,
        hard_deadline: float,
        log_path: Path | None,
    ) -> tuple[SupervisorClient | None, str | None]:
        """Poll until the injected capture supervisor binds.

        The caller's wait_s (soft_deadline) is not a cliff: while the engine
        subprocess is alive and its log file is still growing — i.e. the
        world is demonstrably still loading — we keep waiting past it, up to
        the hard ceiling. Past the soft deadline we only give up early when
        the log has stopped growing for LOAD_STALL_GRACE_S (a stalled load,
        not a slow one). Returns (client, None) on success or (None, reason).
        """
        last_size = -1
        last_growth_at = time.time()
        while True:
            now = time.time()
            if now >= hard_deadline:
                return None, "hard_ceiling"
            if self.proc is None or self.proc.poll() is not None:
                return None, "engine_exited"
            client = SupervisorClient()
            try:
                client.connect(SUPERVISOR_HOST, SUPERVISOR_PORT, deadline_s=0.5)
                client.call("ping")
                return client, None
            except SupervisorRPCError:
                client.close()
            if log_path is not None:
                try:
                    size = log_path.stat().st_size
                except OSError:
                    size = -1
                if size != last_size:
                    last_size = size
                    last_growth_at = now
            else:
                # No log to watch: treat a live subprocess as progress.
                last_growth_at = now
            if now >= soft_deadline and (now - last_growth_at) >= LOAD_STALL_GRACE_S:
                return None, "stalled"
            time.sleep(SUPERVISOR_POLL_INTERVAL_S)

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
                        "orphan from a previous run. Find it with scripts/capture/render.py's "
                        "detect_orphan_sim() (parent process gone) and kill THAT pid only -- never "
                        "`taskkill /F /IM omnisim-bin.exe`, which ends every other session's engine."
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

            cmd = [str(self.binary), str(self.current_sibling)]
            # The deterministic Camera-device render does not depend on a
            # visible main window.  Keep batch mode as the service default, but
            # allow a visible-window recovery path for hosts whose Qt platform
            # plugin cannot establish a sane hidden viewport geometry (for
            # example a QWIDGETSIZE_MAX-sized DIB on Windows).  This is an
            # explicit opt-in because opening a window is observable to the
            # desktop user.
            if not os.environ.get("OMNISIM_CAPTURE_SHOW_GUI"):
                cmd.append("--batch")
            cmd.extend(["--mode=fast", "--stdout", "--stderr"])
            env = os.environ.copy()
            env["OMNISIM_HOME"] = str(self.omnisim_home)
            # Pin the engine's log to OUR file. This assignment also overrides
            # any OMNISIM_LOG_PATH inherited from the launching shell via the
            # copy above -- see CaptureServiceState.__init__ for why sharing
            # that file with a running harness is destructive rather than
            # merely untidy.
            try:
                self.engine_log_path.parent.mkdir(parents=True, exist_ok=True)
            except OSError:
                pass
            env["OMNISIM_LOG_PATH"] = str(self.engine_log_path)
            env["OMNISIM_CAPTURE_SUPERVISOR_HOST"] = SUPERVISOR_HOST
            env["OMNISIM_CAPTURE_SUPERVISOR_PORT"] = str(SUPERVISOR_PORT)
            if sys.platform == "win32":
                # Directly spawning omnisim-bin.exe bypasses the normal Windows
                # launcher, which is also responsible for making the bundled
                # controller interpreter discoverable.  The release bundle
                # keeps that interpreter one level down in `newton-runtime/`,
                # not beside the engine binary.  Without this entry every
                # Python controller (including capture_supervisor itself) fails
                # with `"python.exe" was not found`, even though Newton's
                # embedded interpreter can initialise successfully.
                runtime = self.binary.parent / "newton-runtime"
                path_entries = [str(self.binary.parent)]
                if (runtime / "python.exe").is_file():
                    path_entries.append(str(runtime))
                    # The embeddable bundle stores third-party modules beside
                    # Lib/ rather than under Lib/site-packages.  A controller
                    # launched through runtime/python.exe therefore needs this
                    # explicit path; preserve any caller-provided paths after
                    # the bundled one so project-specific dependencies still
                    # work.
                    site_packages = runtime / "site-packages"
                    if site_packages.is_dir():
                        env["PYTHONPATH"] = str(site_packages) + ";" + env.get(
                            "PYTHONPATH", ""
                        )
                env["PATH"] = ";".join(path_entries) + ";" + env.get("PATH", "")
            # Linux: spawning omnisim-bin directly bypasses the `webots`
            # launcher shell — supply the runtime env it would otherwise miss
            # (bundled-Qt LD_LIBRARY_PATH, QT_QPA_PLATFORM, WEBOTS_TMPDIR,
            # LIBGL_ALWAYS_SOFTWARE). No-op on other platforms.
            env = linux_runtime_env(self.omnisim_home, env)

            # Pipe OmniSim stdout/stderr to a log file so a crash mid-render
            # leaves something diagnosable. The previous DEVNULL redirect
            # threw away the death message and made "supervisor RPC closed"
            # errors a guessing game (4K + heavy fleet → renderer OOM was
            # the suspected culprit and we couldn't confirm without this).
            # The same log doubles as the load-progress signal for
            # _wait_for_supervisor (growing log == still loading).
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

        client, fail_reason = self._wait_for_supervisor(
            soft_deadline=started_at + wait_s,
            hard_deadline=started_at + MAX_LOAD_WAIT_S,
            log_path=log_path if self._sim_log_handle is not None else None,
        )
        load_ms = int((time.time() - started_at) * 1000)

        if client is None:
            exit_code = self.proc.poll() if self.proc is not None else None
            # Expiry must leave a clean, retriable state: tear down the
            # half-loaded engine and the sibling world so the next
            # /world/load starts fresh instead of tripping the orphan check.
            with self.lock:
                self._kill_running()
                self._cleanup_sibling()
                self.last_load_ok = False
                self.last_load_ms = load_ms
            waited = load_ms / 1000.0
            if fail_reason == "engine_exited":
                error = (
                    f"the OmniSim subprocess exited (code {exit_code}) after {waited:.1f}s, "
                    f"before the capture supervisor bound — see {log_path} for the engine's "
                    "own error output."
                )
            elif fail_reason == "hard_ceiling":
                error = (
                    f"capture supervisor did not bind within the {MAX_LOAD_WAIT_S:.0f}s hard "
                    f"ceiling (caller wait_s={wait_s:.1f}). The engine subprocess was cleaned "
                    f"up; state is retriable. See {log_path}."
                )
            else:  # stalled
                error = (
                    f"capture supervisor did not bind within wait_s={wait_s:.1f}s and the "
                    f"engine log stopped growing for {LOAD_STALL_GRACE_S:.0f}s (load stalled "
                    f"after {waited:.1f}s). The engine subprocess was cleaned up; state is "
                    f"retriable — pass a larger wait_s if this world genuinely loads slower. "
                    f"See {log_path}."
                )
            return {"ok": False, "error": error, "load_ms": load_ms}

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
        with self.lock:
            self.last_camera_state = cam_state

        result = {
            "ok": True,
            "world": str(world),
            "load_ms": load_ms,
            "camera": cam_state,
        }
        if not cam_state.get("available"):
            # Never a silent quality downgrade: if the injected Camera did not
            # come up, say so up front — screenshots will be viewport-sized.
            reason = cam_state.get("reason") or "unknown (supervisor reported no camera)"
            result["camera_fallback"] = (
                f"screenshots will fall back to the window-sized viewport — the requested "
                f"{width}x{height} Camera device is unavailable: {reason}"
            )
        return result

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
    first_frame_dims: tuple[int, int] | None = None
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
            if i == 0:
                # Verify the real frame resolution from the file itself, so a
                # camera fallback is disclosed rather than discovered in the
                # encoded video.
                try:
                    with open(frame_path, "rb") as fh:
                        first_frame_dims = png_dimensions(fh.read(64))
                except OSError:
                    first_frame_dims = None

        encode_result = encode_sequence(
            frames_dir, output, fps=fps, codec=codec, crf=crf,
            preset=preset, lossless=lossless,
        )
    finally:
        if not keep_frames and frames_dir.exists():
            shutil.rmtree(frames_dir, ignore_errors=True)

    result = {
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
    if first_frame_dims is not None:
        result["frame_size"] = list(first_frame_dims)
        with state.lock:
            requested = state.last_camera_resolution
            cam_state = dict(state.last_camera_state)
        if requested is not None and first_frame_dims != (int(requested[0]), int(requested[1])):
            reason = cam_state.get("reason") or "the supervisor reported no usable camera"
            result["camera_fallback"] = (
                f"frames are viewport {first_frame_dims[0]}x{first_frame_dims[1]} -- requested "
                f"{int(requested[0])}x{int(requested[1])} unavailable because {reason}"
            )
    return result


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------


def make_handler(state: CaptureServiceState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code: int, obj: dict) -> None:
            # Strict JSON at the boundary: the supervisor forwards whatever
            # the engine reports, and an uninitialised transform can carry
            # NaN / Infinity (Python's json parses AND emits them, but they
            # are invalid JSON and break non-Python clients). Sanitize to
            # null first; allow_nan=False guarantees nothing slips through.
            body = json.dumps(sanitize_nonfinite(obj), allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _png(self, body: bytes, extra_headers: dict[str, str] | None = None) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra_headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _camera_fallback_note(
            self, actual: tuple[int, int] | None, source: str | None = None
        ) -> str | None:
            """If the delivered image is not at the requested Camera resolution,
            return an honest explanation (never silently downgrade)."""
            with state.lock:
                requested = state.last_camera_resolution
                cam_state = dict(state.last_camera_state)
            if actual is None or requested is None:
                return None
            if (actual[0], actual[1]) == (int(requested[0]), int(requested[1])):
                return None
            if cam_state.get("available") and source == "viewport":
                reason = "the caller explicitly requested source='viewport'"
            else:
                reason = cam_state.get("reason") or (
                    "the supervisor reported no usable camera" if not cam_state.get("available")
                    else "the camera resolution does not match the request"
                )
            return (
                f"viewport {actual[0]}x{actual[1]} -- requested "
                f"{int(requested[0])}x{int(requested[1])} unavailable because {reason}"
            )

        def _read_json(self) -> dict:
            n = int(self.headers.get("Content-Length") or "0")
            if n == 0:
                return {}
            raw = self.rfile.read(n).decode("utf-8") or "{}"
            return json.loads(raw)

        def _supervisor_call(self, cmd: str, args: dict | None = None) -> dict | None:
            try:
                return state.supervisor_call(cmd, args)
            except SupervisorCommandError as exc:
                # The supervisor answered and refused. The service is up, so
                # this is the caller's problem to fix, not a reason to retry.
                self._json(422, {"error": str(exc), "rejected_by": "supervisor",
                                 "command": cmd})
                return None
            except SupervisorRPCError as exc:
                # Genuinely unavailable: no world loaded, or the transport died.
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
                # Subject-pose lookup for the cinema pipeline. Pass {"name": "omniquad"}
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
                if user_path is not None and (not isinstance(user_path, str)
                                              or not user_path.strip()):
                    self._json(400, {
                        "error": "path must be a non-empty string; omit it "
                                 "entirely to get the PNG back inline",
                        "path": user_path})
                    return
                if isinstance(user_path, str) and user_path.strip():
                    tmp_path, why = resolve_output_path(user_path)
                    if tmp_path is None:
                        # A path we cannot write is the CALLER's problem and is
                        # knowable before the supervisor is involved. Say so as
                        # a 400 naming the resolved path, rather than letting
                        # Camera.saveImage fail and reporting the service down.
                        self._json(400, {
                            "error": f"cannot write screenshot to {user_path!r}: {why}",
                            "path": user_path,
                            "hint": "a relative path is resolved against "
                                    f"{DEFAULT_OUTPUT_DIR}",
                        })
                        return
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
                    # Decode the delivered PNG's real dimensions instead of
                    # trusting the supervisor's response.
                    actual: tuple[int, int] | None = None
                    try:
                        with open(tmp_path, "rb") as fh:
                            actual = png_dimensions(fh.read(64))
                    except OSError:
                        pass
                    # A 200 must never mean "there is no file there". The old
                    # relative-path behaviour wrote to the supervisor's own cwd
                    # and still answered 200 with the caller's relative string.
                    try:
                        size = tmp_path.stat().st_size
                    except OSError:
                        size = -1
                    if size <= 0:
                        self._json(502, {
                            "error": f"the supervisor reported success but no "
                                     f"readable PNG is at {tmp_path}",
                            "path": str(tmp_path),
                            "supervisor_result": result,
                        })
                        return
                    payload = {"path": str(tmp_path), **{k: v for k, v in result.items() if k != "path"}}
                    if actual is not None:
                        payload["width"], payload["height"] = actual
                    note = self._camera_fallback_note(actual, result.get("source"))
                    if note is not None:
                        payload["camera_fallback"] = note
                    self._json(200, payload)
                    return
                try:
                    image_bytes = tmp_path.read_bytes()
                finally:
                    try:
                        tmp_path.unlink()
                    except OSError:
                        pass
                actual = png_dimensions(image_bytes)
                headers: dict[str, str] = {}
                if actual is not None:
                    headers["X-OmniSim-Image-Width"] = str(actual[0])
                    headers["X-OmniSim-Image-Height"] = str(actual[1])
                note = self._camera_fallback_note(actual, result.get("source"))
                if note is not None:
                    # Header values must be latin-1; keep the note ASCII-safe.
                    headers["X-OmniSim-Camera-Fallback"] = (
                        note.encode("ascii", "replace").decode("ascii")
                    )
                self._png(image_bytes, headers)
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
                # Fail fast: a missing ffmpeg should be reported before we
                # spend minutes rendering frames, not at the encode step.
                if find_ffmpeg() is None:
                    self._json(503, {"error": FFMPEG_MISSING_MSG})
                    return
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
    state = CaptureServiceState(infer_omnisim_home())
    try:
        server = ThreadingHTTPServer((host, port), make_handler(state))
    except OSError as exc:
        # Almost always "a capture service is already on this port"
        # (WinError 10048 / EADDRINUSE). A raw traceback here reads as a crash
        # IN the service, which is the opposite of the truth. Say what is bound
        # and what to do about it. Deliberately NOT the harness's
        # probe-and-guidance system: capture has one port flag and a
        # module-constant supervisor port, so there are exactly two options.
        sys.stderr.write(
            f"[capture] could not bind {host}:{port}: {exc}\n"
            f"[capture] a capture service is probably already running there.\n"
            f"[capture]\n"
            f"[capture]   (1) reuse it -- POST http://{host}:{port}/world/load swaps the\n"
            f"[capture]       world without restarting OmniSim; every other endpoint is\n"
            f"[capture]       already live on :{port}.\n"
            f"[capture]\n"
            f"[capture]   (2) run a parallel one -- pick a free HTTP port AND move the\n"
            f"[capture]       supervisor port, which --port does not touch:\n"
            f"[capture]         OMNISIM_CAPTURE_SUPERVISOR_PORT_OVERRIDE={SUPERVISOR_PORT + 10}\n"
            f"[capture]         python -m omnisim capture --port {port + 10}\n"
            f"[capture]\n"
            f"[capture] refusing to bind on :{port}.\n"
        )
        sys.stderr.flush()
        return 2

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
    print(f"[capture] omnisim home:   {state.omnisim_home}")
    print(f"[capture] supervisor:    {SUPERVISOR_HOST}:{SUPERVISOR_PORT}")
    print(f"[capture] default output dir: {DEFAULT_OUTPUT_DIR}")
    ff = find_ffmpeg()
    if ff:
        print(f"[capture] ffmpeg:        {ff}")
    else:
        print(f"[capture] WARN: {FFMPEG_MISSING_MSG}", file=sys.stderr)
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
    try:
        sys.exit(main())
    except RuntimeError as exc:
        # An unbuilt clone is an ordinary, expected state -- report it as
        # advice, not as a stack trace an agent has to parse.
        print(f"\n{exc}\n", file=sys.stderr)
        sys.exit(1)
