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

"""/capture/screenshot path handling and error classification.

REGRESSION CONTEXT (measured 2026-08-12 against a live capture service on
spot / omnilink_husky, camera 960x540):

    POST /capture/screenshot {}                          -> 200 image/png, 32 KB
    POST /capture/screenshot {"path": "shot.png"}        -> 200, and the PNG
        landed in projects/default/controllers/capture_supervisor/shot.png
        while the response echoed the bare relative name back
    POST /capture/screenshot {"path": "out/shot.png"}    -> 503
        "camera.saveImage failed for path 'out\\shot.png'"
    POST /capture/screenshot {"path": "<absolute>"}      -> 200, correct file

Same service, same scene, same second -- only the argument differed. Two
distinct bugs met there:

1. The service mkdir'd the parent relative to ITS OWN cwd and then handed the
   still-relative path to the supervisor, which is a controller process with a
   different cwd. /capture/movie/start and /capture/sequence both resolve a
   relative path against DEFAULT_OUTPUT_DIR; /capture/screenshot did not.
2. Every supervisor-side failure -- including a request the supervisor received
   and refused -- was reported as 503, which tells the caller the service is
   down when it is serving fine.

These tests run the real HTTP handler against a fake supervisor that reproduces
Camera.saveImage's semantics (absolute paths work; a relative path resolves
against the CONTROLLER's cwd and fails if the directory is not there). No
simulator, no GPU.

Run with:
    pytest tests/capture/test_capture_screenshot_path.py
"""

from __future__ import annotations

import json
import struct
import sys
import threading
import urllib.error
import urllib.request
import zlib
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))

import omnisim_capture as cap  # noqa: E402


def _png_bytes(width: int = 8, height: int = 4) -> bytes:
    """A real, decodable PNG so png_dimensions() sees a valid IHDR."""
    raw = b"".join(b"\x00" + b"\x40\x80\xc0" * width for _ in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw))
            + chunk(b"IEND", b""))


class FakeSupervisor:
    """Stands in for the injected capture_supervisor controller.

    The one behaviour that matters: it resolves paths against ITS OWN cwd
    (``controller_cwd``), exactly as the real controller process does, and
    fails the way ``Camera.saveImage`` fails -- a -1 turned into a
    ``CommandError`` -- when the directory is not there.
    """

    def __init__(self, controller_cwd: Path):
        self.controller_cwd = controller_cwd
        self.connected = True
        self.camera_available = True
        self.calls: list[tuple[str, dict]] = []
        # Indirection so a test can swap the behaviour: patching __call__ on an
        # INSTANCE does nothing, because Python resolves dunders on the type.
        self.handler = self.default_handler

    def __call__(self, cmd: str, args: dict | None = None) -> dict:
        return self.handler(cmd, args)

    def default_handler(self, cmd: str, args: dict | None = None) -> dict:
        args = args or {}
        self.calls.append((cmd, args))
        if not self.connected:
            raise cap.SupervisorRPCError(
                "capture supervisor not connected (load a world first)")
        if cmd != "screenshot":
            return {}
        source = args.get("source") or ("camera" if self.camera_available
                                        else "viewport")
        if source == "camera" and not self.camera_available:
            raise cap.SupervisorCommandError(
                "camera device not available; pass source='viewport' to use "
                "exportImage")
        target = Path(args["path"])
        if not target.is_absolute():
            target = self.controller_cwd / target
        if not target.parent.is_dir():
            raise cap.SupervisorCommandError(
                f"camera.saveImage failed for path '{args['path']}'")
        target.write_bytes(_png_bytes())
        return {"path": args["path"], "width": 8, "height": 4, "source": source}


class FakeState:
    def __init__(self, supervisor: FakeSupervisor):
        self.lock = threading.Lock()
        self.last_camera_resolution = (8, 4)
        self.last_camera_state = {"available": True, "reason": None}
        self.started_at = 0.0
        self._sup = supervisor

    def supervisor_call(self, cmd: str, args: dict | None = None) -> dict:
        return self._sup(cmd, args)

    def sim_state(self) -> dict:
        return {"world": None, "running": False}


@pytest.fixture()
def service(tmp_path, monkeypatch):
    """A live capture HTTP service backed by the fake supervisor."""
    controller_cwd = tmp_path / "controllers" / "capture_supervisor"
    controller_cwd.mkdir(parents=True)
    out_dir = tmp_path / "captures"
    monkeypatch.setattr(cap, "DEFAULT_OUTPUT_DIR", out_dir)

    sup = FakeSupervisor(controller_cwd)
    state = FakeState(sup)
    server = ThreadingHTTPServer(("127.0.0.1", 0), cap.make_handler(state))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        yield type("Svc", (), {"base": base, "sup": sup, "out_dir": out_dir,
                               "controller_cwd": controller_cwd, "tmp": tmp_path})
    finally:
        server.shutdown()
        server.server_close()


def post(base: str, path: str, body: dict):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read(), dict(exc.headers)


def get(base: str, path: str):
    try:
        with urllib.request.urlopen(base + path, timeout=10) as response:
            return response.status, response.read(), dict(response.headers)
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read(), dict(exc.headers)


def as_json(payload: bytes) -> dict:
    return json.loads(payload.decode("utf-8") or "{}")


def test_capture_checkpoint_routes_forward_to_the_persistent_supervisor(service):
    code, _payload, _headers = post(service.base, "/sim/snapshot", {"name": "hero"})
    assert code == 200
    code, _payload, _headers = post(service.base, "/sim/restore", {"name": "hero"})
    assert code == 200
    code, _payload, _headers = get(service.base, "/sim/snapshots")
    assert code == 200
    assert ("sim_snapshot", {"name": "hero"}) in service.sup.calls
    assert ("sim_restore", {"name": "hero"}) in service.sup.calls
    assert ("sim_snapshots", {}) in service.sup.calls


def test_capture_checkpoint_name_is_validated_before_supervisor_rpc(service):
    code, payload, _headers = post(service.base, "/sim/snapshot", {"name": ""})
    assert code == 400
    assert "name is required" in as_json(payload)["error"]


# --------------------------------------------------------------------------
# The control: no path at all still returns a real PNG inline.
# --------------------------------------------------------------------------


def test_no_path_returns_inline_png(service):
    code, payload, hdrs = post(service.base, "/capture/screenshot", {})
    assert code == 200
    assert hdrs.get("Content-Type") == "image/png"
    assert payload[:8] == b"\x89PNG\r\n\x1a\n"


# --------------------------------------------------------------------------
# 1. A path argument must be HONOURED, not 503'd.
# --------------------------------------------------------------------------


def test_relative_path_with_subdir_is_honoured_not_503(service):
    """THE reported defect. `{}` returned a PNG; `{"path": "out/shot.png"}`
    on the same service returned 503 'camera.saveImage failed'."""
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"path": "out/shot.png"})
    assert code == 200, f"expected the path to be honoured, got {code}: {payload!r}"
    body = as_json(payload)
    written = Path(body["path"])
    assert written.is_absolute(), "the response must name a path the caller can open"
    assert written.is_file() and written.stat().st_size > 0
    assert written == service.out_dir / "out" / "shot.png"


def test_bare_relative_path_lands_where_the_response_says(service):
    """The other half: a bare relative name used to 'succeed' while writing
    into the supervisor's own cwd, and the response echoed the caller's
    relative string, so the file was unfindable."""
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"path": "shot.png"})
    assert code == 200
    body = as_json(payload)
    written = Path(body["path"])
    assert written.is_absolute()
    assert written.is_file()
    assert written == service.out_dir / "shot.png"
    stray = service.controller_cwd / "shot.png"
    assert not stray.exists(), (
        "the screenshot must not land in the supervisor's controller directory")


def test_absolute_path_still_works(service):
    target = service.tmp / "elsewhere" / "abs.png"
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"path": str(target)})
    assert code == 200
    body = as_json(payload)
    assert Path(body["path"]) == target
    assert target.is_file()


def test_relative_path_matches_what_movie_start_does(service):
    """/capture/movie/start and /capture/sequence already resolve a relative
    path against DEFAULT_OUTPUT_DIR. Screenshot must agree, or a bare filename
    means two different things on one service."""
    post(service.base, "/capture/screenshot", {"path": "same.png"})
    post(service.base, "/capture/movie/start", {"path": "same.mp4"})
    movie_call = next(a for c, a in service.sup.calls if c == "movie_start")
    shot_call = next(a for c, a in service.sup.calls if c == "screenshot")
    assert Path(shot_call["path"]).parent == Path(movie_call["path"]).parent


# --------------------------------------------------------------------------
# 2. A rejection is a 4xx. An outage is a 503. They must not collapse.
# --------------------------------------------------------------------------


def test_unwritable_path_is_a_400_not_a_503(service):
    """An unusable path is knowable before the supervisor is involved, and it
    is the caller's problem -- 400, naming the reason."""
    a_directory = service.tmp / "iam_a_dir"
    a_directory.mkdir()
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"path": str(a_directory)})
    assert code == 400, f"got {code}: {payload!r}"
    body = as_json(payload)
    assert "director" in body["error"].lower()


def test_non_string_path_is_a_400(service):
    code, payload, _ = post(service.base, "/capture/screenshot", {"path": 17})
    assert code == 400
    assert "non-empty string" in as_json(payload)["error"]


def test_supervisor_rejection_is_422_not_503(service):
    """The supervisor answered and refused. The service is up; 503 is a lie."""
    service.sup.camera_available = False
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"source": "camera"})
    assert code == 422, f"got {code}: {payload!r}"
    body = as_json(payload)
    assert body.get("rejected_by") == "supervisor"
    assert body.get("command") == "screenshot"
    assert "camera device not available" in body["error"]


def test_supervisor_outage_is_still_503(service):
    """Do not weaken the real unavailable case."""
    service.sup.connected = False
    code, payload, _ = post(service.base, "/capture/screenshot", {})
    assert code == 503, f"got {code}: {payload!r}"
    assert "not connected" in as_json(payload)["error"]


def test_step_rejection_is_also_422(service):
    """The classification lives in _supervisor_call, so it must hold for every
    endpoint, not just screenshot."""
    original = service.sup.default_handler

    def rejecting(cmd, args=None):
        if cmd == "step":
            raise cap.SupervisorCommandError("nope")
        return original(cmd, args)

    service.sup.handler = rejecting
    code, payload, _ = post(service.base, "/sim/step", {"steps": 1})
    assert code == 422
    assert as_json(payload)["command"] == "step"


# --------------------------------------------------------------------------
# 3. A 200 must never mean "there is no file there".
# --------------------------------------------------------------------------


def test_success_without_a_file_is_reported_as_502(service, monkeypatch):
    def lying(cmd, args=None):
        service.sup.calls.append((cmd, args or {}))
        return {"path": args["path"], "width": 8, "height": 4, "source": "camera"}

    monkeypatch.setattr(service.sup, "handler", lying)
    code, payload, _ = post(service.base, "/capture/screenshot",
                            {"path": "ghost.png"})
    assert code == 502, f"got {code}: {payload!r}"
    assert "no readable PNG" in as_json(payload)["error"]
