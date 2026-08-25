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

"""capture_supervisor — supervisor for the OmniSim capture service.

Sister to harness_supervisor. Injected into a user world by the capture
service via a sibling .wbt file. Carries a Camera device sized for the
desired render resolution so screenshots and frame-sequence renders are
independent of the host window. The supervisor robot's pose is the camera
pose; setting `set_camera_pose` moves both the robot (which moves the
attached Camera) and, optionally, the live Viewpoint to mirror it.

Wire protocol matches harness_supervisor: 4-byte big-endian length prefix
followed by a UTF-8 JSON payload.

Commands
--------
ping              -> {}
sim_state         -> {sim_time_ms, basic_time_step_ms, camera: {width, height, fov}}
step              -> {sim_time_ms}                  args: {steps?: int}
set_camera_pose   -> {position, orientation}        args: {position: [x,y,z],
                                                            orientation: [ax,ay,az,angle],
                                                            sync_viewpoint?: bool}
get_camera_pose   -> {position, orientation, viewpoint_position, viewpoint_orientation}
screenshot        -> {path, width, height}          args: {path: str, source?: "camera"|"viewport",
                                                            quality?: int}
frame_dump        -> {path, frame_index}            args: {path: str, frame_index?: int}
movie_start       -> {path, width, height, codec, fps}
                                                    args: {path: str, width?: int, height?: int,
                                                            codec?: int, quality?: int,
                                                            acceleration?: int, caption?: bool}
movie_stop        -> {path, ready, failed}
movie_status      -> {ready, failed}

The Camera device's resolution is fixed at world-load time by the sibling
stanza the capture service writes; it is read here for `sim_state` so the
caller can confirm.
"""

from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import time
import traceback

from omnisim import Supervisor

HOST = os.environ.get("OMNISIM_CAPTURE_SUPERVISOR_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_CAPTURE_SUPERVISOR_PORT", "6792"))

CAMERA_DEVICE_NAME = "capture_camera"


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = n
    while remaining > 0:
        try:
            chunk = sock.recv(remaining)
        except OSError:
            # Aborted/reset connections (the capture service timing out an
            # RPC at SUPERVISOR_RPC_TIMEOUT_S=120 and closing its socket --
            # /capture/sequence is the long path that reaches it) are a
            # disconnect, not a controller crash. Unguarded, the
            # ConnectionResetError propagates read_frame -> main() and takes
            # the supervisor down, so one timed-out request ends the render
            # instead of one request. The caller drops this client and keeps
            # serving. Matches harness_supervisor.recv_exact.
            return None
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(sock: socket.socket) -> dict | None:
    header = recv_exact(sock, 4)
    if header is None:
        return None
    (length,) = struct.unpack(">I", header)
    if length == 0 or length > 16 * 1024 * 1024:
        return None
    payload = recv_exact(sock, length)
    if payload is None:
        return None
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError:
        return None


def write_frame(sock: socket.socket, obj: dict) -> bool:
    body = json.dumps(obj).encode("utf-8")
    try:
        sock.sendall(struct.pack(">I", len(body)) + body)
        return True
    except OSError:
        return False


def png_file_dimensions(path: str) -> tuple[int, int] | None:
    """Width/height from a PNG's IHDR chunk — lets viewport-fallback
    screenshots report their REAL size instead of 0x0."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(24)
    except OSError:
        return None
    if len(head) >= 24 and head[:8] == b"\x89PNG\r\n\x1a\n" and head[12:16] == b"IHDR":
        w, h = struct.unpack(">II", head[16:24])
        return int(w), int(h)
    return None


def find_viewpoint(supervisor: Supervisor):
    root = supervisor.getRoot()
    if root is None:
        return None
    children = root.getField("children")
    if children is None:
        return None
    try:
        count = children.getCount()
    except Exception:
        return None
    for i in range(count):
        node = children.getMFNode(i)
        if node is not None and node.getTypeName() == "Viewpoint":
            return node
    return None


class CommandError(Exception):
    pass


class CaptureState:
    """Tracks the camera device, the supervisor's own node, and movie-recording
    progress. Everything the dispatcher needs in one place so individual
    handlers stay small.
    """

    def __init__(self, supervisor: Supervisor, basic_step_ms: int):
        self.supervisor = supervisor
        self.basic_step_ms = basic_step_ms
        self.self_node = supervisor.getSelf()
        self.viewpoint = find_viewpoint(supervisor)
        self.camera = None
        self.camera_width = 0
        self.camera_height = 0
        self.camera_fov = 0.0
        # camera_reason explains WHY self.camera is None, so the service can
        # disclose the viewport fallback instead of silently downgrading.
        self.camera_reason: str | None = None
        # HISTORY: cam.enable() used to segfault the May-2026 engine in
        # --batch render paths, so it was gated off (ce5228b26) — which made
        # every capture a viewport-sized exportImage. Re-verified 2026-07-12
        # on the current engine (Windows + Linux/Xvfb software-GL): no crash,
        # camera renders at the declared resolution. The camera is therefore
        # enabled BY DEFAULT now; OMNISIM_CAPTURE_DISABLE_CAMERA=1 is the
        # escape hatch if a regression resurfaces.
        try:
            cam = supervisor.getDevice(CAMERA_DEVICE_NAME)
        except Exception:
            cam = None
        if cam is not None:
            try:
                self.camera_width = int(cam.getWidth())
                self.camera_height = int(cam.getHeight())
                self.camera_fov = float(cam.getFov())
            except Exception:
                pass
        if cam is None:
            self.camera_reason = (
                f"no '{CAMERA_DEVICE_NAME}' device in the injected supervisor robot "
                "(stale sibling world, or a service older than the camera restore)"
            )
        elif os.environ.get("OMNISIM_CAPTURE_DISABLE_CAMERA") == "1":
            self.camera_reason = "disabled via OMNISIM_CAPTURE_DISABLE_CAMERA=1"
        else:
            try:
                cam.enable(basic_step_ms)
                self.camera = cam
            except Exception as exc:
                sys.stderr.write(f"[capture_supervisor] camera enable failed: {exc}\n")
                self.camera = None
                self.camera_reason = f"camera enable failed: {exc}"
        self.movie_active = False
        self.movie_path: str | None = None
        self.movie_width = 0
        self.movie_height = 0
        self.movie_codec = 0
        self.movie_fps = 0
        self.frame_counter = 0


def axis_angle_to_target(position: list[float], target: list[float],
                         up: tuple = (0.0, 0.0, 1.0)) -> list[float]:
    """Same axis-angle math as the harness's compute_look_at_orientation;
    duplicated here so this controller has no dependency on the harness
    package. The camera frame is +X forward, +Y left, +Z up.

    Builds the full basis from an explicit `up` so the horizon stays level.
    The old shortest-arc form aimed correctly but left roll uncontrolled (up to
    92 deg), which tilted every repositioned capture camera.
    """
    import math

    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    if n < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    fx, fy, fz = fx / n, fy / n, fz / n

    ux, uy, uz = up
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        ux, uy, uz, un = 0.0, 0.0, 1.0, 1.0
    ux, uy, uz = ux / un, uy / un, uz / un
    # Degenerate when up is (anti)parallel to the view direction (top-down).
    if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
        ux, uy, uz = 0.0, 1.0, 0.0
        if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
            ux, uy, uz = 1.0, 0.0, 0.0

    d = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - d * fx, uy - d * fy, uz - d * fz
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    ux, uy, uz = ux / un, uy / un, uz / un
    yx, yy, yz = uy * fz - uz * fy, uz * fx - ux * fz, ux * fy - uy * fx

    r = [[fx, yx, ux], [fy, yy, uy], [fz, yz, uz]]
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    s = 2.0 * math.sin(angle)
    if abs(s) < 1e-9:
        diag = (r[0][0], r[1][1], r[2][2])
        i = diag.index(max(diag))
        a = [0.0, 0.0, 0.0]
        a[i] = math.sqrt(max(0.0, (r[i][i] + 1.0) / 2.0))
        for j in range(3):
            if j != i:
                a[j] = (r[i][j] + r[j][i]) / (4.0 * a[i]) if a[i] > 1e-9 else 0.0
        an = math.sqrt(sum(c * c for c in a)) or 1.0
        return [a[0] / an, a[1] / an, a[2] / an, angle]
    return [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, angle]


def cmd_set_camera_pose(state: CaptureState, args: dict) -> dict:
    position = args.get("position")
    orientation = args.get("orientation")
    target = args.get("target")
    sync_viewpoint = bool(args.get("sync_viewpoint", True))

    if position is not None and (not isinstance(position, list) or len(position) != 3):
        raise CommandError("position must be a list of 3 numbers")
    if orientation is not None and (not isinstance(orientation, list) or len(orientation) != 4):
        raise CommandError("orientation must be a list of 4 numbers")
    if target is not None and (not isinstance(target, list) or len(target) != 3):
        raise CommandError("target must be a list of 3 numbers")
    if orientation is None and target is not None and position is not None:
        orientation = axis_angle_to_target(
            [float(x) for x in position], [float(x) for x in target]
        )

    if state.self_node is None:
        raise CommandError("supervisor self node unavailable")

    pos_f = [float(x) for x in position] if position is not None else None
    ori_f = [float(x) for x in orientation] if orientation is not None else None

    if pos_f is not None:
        f = state.self_node.getField("translation")
        if f is None:
            raise CommandError("supervisor robot has no translation field")
        f.setSFVec3f(pos_f)
    if ori_f is not None:
        f = state.self_node.getField("rotation")
        if f is None:
            raise CommandError("supervisor robot has no rotation field")
        f.setSFRotation(ori_f)

    if sync_viewpoint and state.viewpoint is not None:
        if pos_f is not None:
            vf = state.viewpoint.getField("position")
            if vf is not None:
                vf.setSFVec3f(pos_f)
        if ori_f is not None:
            vf = state.viewpoint.getField("orientation")
            if vf is not None:
                vf.setSFRotation(ori_f)

    return {"position": pos_f, "orientation": ori_f}


def _find_robot_node(root, name: str):
    """Walk the scene tree and find a Robot/URDFRobot whose `name` field
    matches the requested name. Used by the cinema pipeline to lock the
    camera onto a specific subject without hand-coding world coordinates."""
    children_field = root.getField("children")
    if children_field is None:
        return None
    for i in range(children_field.getCount()):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            tn = node.getTypeName()
        except Exception:
            tn = ""
        if tn in ("Robot", "URDFRobot"):
            try:
                nf = node.getField("name")
                if nf is not None and nf.getSFString() == name:
                    return node
            except Exception:
                pass
        # Recurse into Group/Transform-like containers via their children field.
        try:
            cf = node.getField("children")
            if cf is not None:
                found = _find_robot_node(node, name)
                if found is not None:
                    return found
        except Exception:
            pass
    return None


def _list_robot_nodes(root) -> list[dict]:
    out: list[dict] = []
    children_field = root.getField("children")
    if children_field is None:
        return out
    for i in range(children_field.getCount()):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            tn = node.getTypeName()
        except Exception:
            tn = ""
        if tn in ("Robot", "URDFRobot"):
            entry = {"type_name": tn, "name": "", "def": "", "translation": None}
            try:
                nf = node.getField("name")
                if nf is not None:
                    entry["name"] = nf.getSFString()
            except Exception:
                pass
            try:
                entry["def"] = node.getDef() or ""
            except Exception:
                pass
            try:
                tf = node.getField("translation")
                if tf is not None:
                    entry["translation"] = list(tf.getSFVec3f())
            except Exception:
                pass
            out.append(entry)
        try:
            cf = node.getField("children")
            if cf is not None:
                out.extend(_list_robot_nodes(node))
        except Exception:
            pass
    return out


def cmd_node_pose(state: CaptureState, args: dict) -> dict:
    """Return current world pose for a named robot (or DEF). The cinema
    pipeline uses this to make camera moves subject-relative instead of
    world-coordinate-relative — so `tracking_side(spot)` works regardless
    of where Spot was spawned."""
    name = args.get("name")
    def_name = args.get("def")
    if not name and not def_name:
        raise CommandError("node_pose requires 'name' or 'def'")
    node = None
    if def_name:
        try:
            node = state.supervisor.getFromDef(str(def_name))
        except Exception:
            node = None
    if node is None and name:
        root = state.supervisor.getRoot()
        node = _find_robot_node(root, str(name))
    if node is None:
        raise CommandError(f"no robot found with name={name!r} def={def_name!r}")
    out: dict = {"name": name or "", "def": def_name or ""}
    try:
        tf = node.getField("translation")
        if tf is not None:
            out["translation"] = list(tf.getSFVec3f())
    except Exception:
        pass
    try:
        rf = node.getField("rotation")
        if rf is not None:
            out["rotation"] = list(rf.getSFRotation())
    except Exception:
        pass
    return out


def cmd_list_robots(state: CaptureState, _args: dict) -> dict:
    root = state.supervisor.getRoot()
    return {"robots": _list_robot_nodes(root)}


def cmd_get_camera_pose(state: CaptureState, _args: dict) -> dict:
    out: dict = {}
    if state.self_node is not None:
        try:
            out["position"] = list(state.self_node.getPosition())
        except Exception:
            pass
        f = state.self_node.getField("rotation")
        if f is not None:
            try:
                out["orientation"] = list(f.getSFRotation())
            except Exception:
                pass
    if state.viewpoint is not None:
        vp_pos = state.viewpoint.getField("position")
        if vp_pos is not None:
            try:
                out["viewpoint_position"] = list(vp_pos.getSFVec3f())
            except Exception:
                pass
        vp_ori = state.viewpoint.getField("orientation")
        if vp_ori is not None:
            try:
                out["viewpoint_orientation"] = list(vp_ori.getSFRotation())
            except Exception:
                pass
    return out


def cmd_screenshot(state: CaptureState, args: dict) -> dict:
    path = args.get("path")
    if not isinstance(path, str) or not path:
        raise CommandError("screenshot requires a 'path' string")
    quality = int(args.get("quality", 95))
    if not (1 <= quality <= 100):
        quality = 95
    source = args.get("source") or ("camera" if state.camera is not None else "viewport")
    if source == "camera":
        if state.camera is None:
            raise CommandError(
                "camera device not available"
                + (f" ({state.camera_reason})" if state.camera_reason else "")
                + "; pass source='viewport' to use exportImage"
            )
        ok = state.camera.saveImage(path, quality)
        # Camera.saveImage returns 0 on success, -1 on failure. A -1 right
        # after enable can mean the first camera frame has not rendered yet —
        # advance one step and retry once before declaring failure.
        if ok == -1:
            state.supervisor.step(state.basic_step_ms)
            ok = state.camera.saveImage(path, quality)
        if ok == -1:
            raise CommandError(f"camera.saveImage failed for path '{path}'")
        return {"path": path, "width": state.camera_width, "height": state.camera_height,
                "source": "camera"}
    if source == "viewport":
        ok = state.supervisor.exportImage(path, quality)
        if ok is False:
            raise CommandError(f"exportImage failed for path '{path}'")
        dims = png_file_dimensions(path)
        out = {"path": path, "source": "viewport"}
        if dims is not None:
            out["width"], out["height"] = dims
        return out
    raise CommandError(f"unknown screenshot source: {source!r}")


def cmd_frame_dump(state: CaptureState, args: dict) -> dict:
    path = args.get("path")
    if not isinstance(path, str) or not path:
        raise CommandError("frame_dump requires a 'path' string")
    quality = int(args.get("quality", 100))
    if not (1 <= quality <= 100):
        quality = 100
    if state.camera is not None:
        ok = state.camera.saveImage(path, quality)
        if ok == -1:
            # First-frame race: step once and retry before failing.
            state.supervisor.step(state.basic_step_ms)
            ok = state.camera.saveImage(path, quality)
        if ok == -1:
            raise CommandError(f"camera.saveImage failed for path '{path}'")
        width = state.camera_width
        height = state.camera_height
        source = "camera"
    else:
        ok = state.supervisor.exportImage(path, quality)
        if ok is False:
            raise CommandError(f"exportImage failed for path '{path}'")
        dims = png_file_dimensions(path)
        width, height = dims if dims is not None else (0, 0)
        source = "viewport"
    frame_index = int(args.get("frame_index", state.frame_counter))
    state.frame_counter = frame_index + 1
    return {"path": path, "frame_index": frame_index,
            "width": width, "height": height, "source": source}


def cmd_movie_start(state: CaptureState, args: dict) -> dict:
    if state.movie_active:
        raise CommandError("a movie recording is already active; call movie_stop first")
    path = args.get("path")
    if not isinstance(path, str) or not path:
        raise CommandError("movie_start requires a 'path' string")
    width = int(args.get("width", 1920))
    height = int(args.get("height", 1080))
    codec = int(args.get("codec", 0))
    quality = int(args.get("quality", 90))
    if not (1 <= quality <= 100):
        quality = 90
    acceleration = int(args.get("acceleration", 1))
    caption = bool(args.get("caption", False))
    fps = int(args.get("fps", max(1, round(1000 / max(state.basic_step_ms, 1)))))
    state.supervisor.movieStartRecording(path, width, height, codec, quality, acceleration, caption)
    state.movie_active = True
    state.movie_path = path
    state.movie_width = width
    state.movie_height = height
    state.movie_codec = codec
    state.movie_fps = fps
    return {"path": path, "width": width, "height": height, "codec": codec, "fps": fps}


def cmd_movie_stop(state: CaptureState, _args: dict) -> dict:
    if not state.movie_active:
        raise CommandError("no active movie recording")
    state.supervisor.movieStopRecording()
    state.movie_active = False
    # The encoder runs asynchronously after stop; movie_status will reflect
    # ready/failed once it completes.
    return {
        "path": state.movie_path,
        "ready": bool(state.supervisor.movieIsReady()),
        "failed": bool(state.supervisor.movieFailed()),
    }


def cmd_movie_status(state: CaptureState, _args: dict) -> dict:
    return {
        "active": state.movie_active,
        "path": state.movie_path,
        "ready": bool(state.supervisor.movieIsReady()),
        "failed": bool(state.supervisor.movieFailed()),
    }


def dispatch(state: CaptureState, sim_time_ms: float, cmd: str, args: dict) -> dict:
    if cmd == "ping":
        return {}
    if cmd == "sim_state":
        return {
            "sim_time_ms": sim_time_ms,
            "basic_time_step_ms": state.basic_step_ms,
            "camera": {
                "available": state.camera is not None,
                "width": state.camera_width,
                "height": state.camera_height,
                "fov": state.camera_fov,
                "reason": state.camera_reason,
            },
            "movie_active": state.movie_active,
            "frame_counter": state.frame_counter,
        }
    if cmd == "step":
        steps = int(args.get("steps", 1))
        if steps < 1:
            steps = 1
        for _ in range(steps):
            if state.supervisor.step(state.basic_step_ms) == -1:
                raise CommandError("simulator step returned -1 (terminating)")
        return {"sim_time_ms": (sim_time_ms + steps * state.basic_step_ms)}
    if cmd == "set_camera_pose":
        return cmd_set_camera_pose(state, args)
    if cmd == "get_camera_pose":
        return cmd_get_camera_pose(state, args)
    if cmd == "node_pose":
        return cmd_node_pose(state, args)
    if cmd == "list_robots":
        return cmd_list_robots(state, args)
    if cmd == "screenshot":
        return cmd_screenshot(state, args)
    if cmd == "frame_dump":
        return cmd_frame_dump(state, args)
    if cmd == "movie_start":
        return cmd_movie_start(state, args)
    if cmd == "movie_stop":
        return cmd_movie_stop(state, args)
    if cmd == "movie_status":
        return cmd_movie_status(state, args)
    if cmd == "reset":
        state.supervisor.simulationReset()
        state.frame_counter = 0
        return {"sim_time_ms": 0}
    raise CommandError(f"unknown cmd: {cmd}")


def main() -> int:
    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())
    state = CaptureState(supervisor, basic_step_ms)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        sys.stderr.write(f"[capture_supervisor] bind {HOST}:{PORT} failed: {exc}\n")
        return 1
    server.listen(4)
    server.setblocking(False)
    sys.stderr.write(
        f"[capture_supervisor] listening on {HOST}:{PORT} "
        f"(camera {state.camera_width}x{state.camera_height}, "
        f"available={state.camera is not None})\n"
    )
    sys.stderr.flush()

    clients: list[socket.socket] = []
    sim_time_ms = 0.0

    while supervisor.step(basic_step_ms) != -1:
        sim_time_ms += basic_step_ms

        try:
            while True:
                client, _ = server.accept()
                client.setblocking(True)
                clients.append(client)
        except BlockingIOError:
            pass

        if clients:
            ready, _, _ = select.select(clients, [], [], 0)
            for client in ready:
                request = read_frame(client)
                if request is None:
                    try:
                        client.close()
                    except OSError:
                        pass
                    clients.remove(client)
                    continue
                req_id = request.get("id", 0)
                cmd = request.get("cmd", "")
                args = request.get("args") or {}
                try:
                    result = dispatch(state, sim_time_ms, cmd, args)
                    write_frame(client, {"id": req_id, "ok": True, "result": result})
                except CommandError as exc:
                    write_frame(client, {"id": req_id, "ok": False, "error": str(exc)})
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[capture_supervisor] {cmd} crashed: {exc}\n{traceback.format_exc()}"
                    )
                    write_frame(client, {"id": req_id, "ok": False, "error": f"internal: {exc}"})

    for client in clients:
        try:
            client.close()
        except OSError:
            pass
    server.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
