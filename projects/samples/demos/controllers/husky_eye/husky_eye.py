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

"""husky_eye — vision sidecar for husky_omnilink_bridge.

The main bridge controller (`husky_omnilink_bridge`) lives on the husky's
URDFRobot. It cannot itself host a Webots Camera device — three failure
modes have been verified:

    1. Camera in URDFRobot.children replaces the URDF expansion (the
       wheels disappear and the husky has no physics).
    2. <sensor type="camera"> in the URDF is not parsed by Webots' URDF
       importer.
    3. importMFNodeFromString into a child Solid at runtime adds the node
       but Webots does not re-register devices after world load, so
       getDevice() never finds it.

So vision lives in this sidecar Robot. It is a tiny Solid with a Camera
attached, supervisor=TRUE so it can teleport itself to track the husky's
pose every tick, and a small HTTP server (port 6071) that returns the
latest camera frame as base64 PNG. The main bridge proxies its /camera
endpoint to this server.

World setup: a Robot named "husky_eye" with controller "husky_eye" sits
near the husky at startup. The Camera is mounted ~0.5 m above and just
ahead of the husky's base. Each tick this controller finds the husky
via getFromDef("HUSKY_BODY") and sets its own translation/rotation to
match.
"""

from __future__ import annotations

import base64
import io
import json
import math
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from omnisim import Supervisor

# numpy is used by the /scan endpoint's color histograms + wall-close
# heuristic. Optional — if not available, /scan returns 503 and the
# agent falls back to /image.
try:
    import numpy as _np  # type: ignore
    _HAS_NUMPY = True
except Exception:
    _np = None  # type: ignore
    _HAS_NUMPY = False


# Mirror stdout/stderr to a tempfile so we can debug from outside Webots.
class _Tee:
    def __init__(self, *streams):
        self._streams = streams
    def write(self, s):
        for st in self._streams:
            try: st.write(s); st.flush()
            except Exception: pass
    def flush(self):
        for st in self._streams:
            try: st.flush()
            except Exception: pass

import os as _o
import tempfile as _t
import sys as _s
try:
    _path = _o.path.join(_t.gettempdir(), f"husky_eye_{_o.getpid()}.log")
    _fp = open(_path, "w", encoding="utf-8", buffering=1)
    _s.stdout = _Tee(_s.__stdout__, _fp)
    _s.stderr = _Tee(_s.__stderr__, _fp)
    print(f"[husky_eye] log file: {_path}")
except Exception:
    pass

# Camera mounting offset in the husky body frame (forward, left, up in m).
MOUNT_FORWARD_M = 0.30
MOUNT_LATERAL_M = 0.0
MOUNT_HEIGHT_M = 0.55

EYE_HOST = "127.0.0.1"

# Default 6071 + tracking DEF "HUSKY" preserve the single-husky world
# contract that all pre-multiplexing demos rely on (warehouse_logistics,
# husky_maze_*). Multi-husky worlds (warehouse_patrol) override per-
# instance via Webots controllerArgs:
#   controller "husky_eye"
#   controllerArgs ["--port" "6081" "--track-def" "HUSKY_SOUTH"]
import argparse as _argparse


def _parse_eye_args() -> tuple:
    parser = _argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", type=int, default=6071)
    parser.add_argument("--track-def", dest="track_def", default="HUSKY")
    args, _unknown = parser.parse_known_args()
    return args.port, args.track_def


EYE_PORT, EYE_TRACK_DEF = _parse_eye_args()


# --- PNG encoder (no PIL dependency) ---------------------------------------

def encode_png(width: int, height: int, bgra_bytes: bytes) -> bytes:
    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        b = bgra_bytes[i * 4 + 0]
        g = bgra_bytes[i * 4 + 1]
        r = bgra_bytes[i * 4 + 2]
        rgb[i * 3 + 0] = r
        rgb[i * 3 + 1] = g
        rgb[i * 3 + 2] = b
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * stride:(y + 1) * stride])
    idat = _png_chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    iend = _png_chunk(b"IEND", b"")
    return sig + ihdr + idat + iend


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


# --- Shared state ----------------------------------------------------------

class EyeState:
    def __init__(self):
        self.lock = threading.Lock()
        # cameras: dict of {name: camera_device}, populated at startup.
        # Names are "front", "right", "back", "left" — exactly the four
        # cardinal Robot-local directions the agent narrates.
        self.cameras = {}
        self.width = 0
        self.height = 0
        self.fov = 0.0
        self.husky_x = 0.0
        self.husky_y = 0.0
        self.husky_yaw = 0.0
        self.last_tick_at = time.time()
        self.sim_time = 0.0
        # Latest captured image bytes per camera.
        self.image_request = 0
        self.image_served = 0
        self.image_png = {}     # {name: png_bytes}    served by /image
        self.image_raw = {}     # {name: bgra_bytes}  used by /scan


def _analyze_bgra(bgra_bytes: bytes, w: int, h: int) -> dict:
    """Pure-Python analysis of one camera's raw BGRA frame. Returns a small
    structured dict the agent can reason about without ever seeing pixels.
    No numpy dependency — Webots's bundled mingw64 Python doesn't ship
    numpy and a system upgrade to add it broke the toolchain. 160×120 = 19 200
    pixels iterated in Python takes ~30 ms per frame, fine for /scan.

    The fields:
        wall_close       : bool — bottom-centre patch is uniform low-variance
                                  texture, suggesting a wall directly ahead
                                  within ~1 m.
        wall_close_score : 0..1 — confidence in wall_close (1 = very confident
                                  the agent is staring at a wall, 0 = clearly
                                  an open corridor).
        marker           : null | "red" | "green" | "blue" — strongly-saturated
                                  cylinder colour visible in the frame.
        marker_pixels    : int  — pixel count of the dominant marker colour.
        marker_fraction  : 0..1 — marker_pixels / total_pixels.
        marker_centroid  : null | {"x_norm": 0..1, "y_norm": 0..1} — where in
                                  the frame the marker's centre of mass is.
                                  x_norm 0=left, 1=right; y_norm 0=top, 1=bottom.
        mean_brightness  : 0..255 — overall frame brightness.
        floor_visible    : bool  — bottom strip mean darker than middle strip
                                   mean by >8 (corridor extends ahead).
    """
    total = w * h
    if len(bgra_bytes) < total * 4:
        return {"error": "frame buffer too small"}

    # Wall-close patch bounds (bottom-centre 60% × 40%).
    y0, y1 = int(h * 0.55), int(h * 0.95)
    x0, x1 = int(w * 0.20), int(w * 0.80)
    # Bottom / middle strip bounds for floor_visible.
    bot_y0 = int(h * 0.85)
    mid_y0, mid_y1 = int(h * 0.40), int(h * 0.60)

    # Six-color marker counters. Maze worlds only ever expose
    # red/green/blue, but warehouse_logistics.wbt uses all six
    # (red/green/blue on the north row, yellow/magenta/cyan on the
    # south). The classifier needs to distinguish all of them so the
    # picker's scan_for_tag tool can return a structured marker
    # response without sending pixels to the LLM.
    counts = {"red": 0, "green": 0, "blue": 0,
              "yellow": 0, "magenta": 0, "cyan": 0}
    sum_x = {k: 0 for k in counts}
    sum_y = {k: 0 for k in counts}
    bright_sum = 0
    # patch sum + sum of squares for std-dev.
    patch_sum = patch_sum_sq = patch_n = 0
    bot_sum = bot_n = 0
    mid_sum = mid_n = 0

    raw = bgra_bytes  # local alias for hot loop
    for y in range(h):
        in_patch = (y0 <= y < y1)
        in_bot   = (y >= bot_y0)
        in_mid   = (mid_y0 <= y < mid_y1)
        row_off = y * w * 4
        for x in range(w):
            i = row_off + x * 4
            B = raw[i]
            G = raw[i + 1]
            R = raw[i + 2]
            bright_sum += R + G + B
            # Six-way colour classification. Each branch matches a
            # reasonable region of (R, G, B) space:
            #   red     = R high, G+B low
            #   green   = G high, R+B low
            #   blue    = B high, R+G low
            #   yellow  = R+G high, B low      (additive R+G)
            #   magenta = R+B high, G low      (additive R+B)
            #   cyan    = G+B high, R low      (additive G+B)
            # The else-if chain is order-sensitive: secondary colours
            # (yellow/magenta/cyan) get tested AFTER primaries so a
            # noisy near-primary pixel gets binned correctly. Thresholds
            # tuned for the warehouse's emissive 0.30 m tag cubes — the
            # tag's emissiveColor pushes channels well past these gates
            # under any lighting.
            tag = None
            if R > 140 and G < 90 and B < 90:
                tag = "red"
            elif G > 140 and R < 90 and B < 90:
                tag = "green"
            elif B > 140 and R < 90 and G < 90:
                tag = "blue"
            elif R > 140 and G > 130 and B < 90:
                tag = "yellow"
            elif R > 140 and B > 130 and G < 90:
                tag = "magenta"
            elif G > 140 and B > 140 and R < 90:
                tag = "cyan"
            if tag is not None:
                counts[tag] += 1
                sum_x[tag] += x
                sum_y[tag] += y
            if in_patch and (x0 <= x < x1):
                v = R + G + B
                patch_sum += v
                patch_sum_sq += v * v
                patch_n += 1
            if in_bot:
                bot_sum += R + G + B; bot_n += 1
            if in_mid:
                mid_sum += R + G + B; mid_n += 1

    # Marker selection: the colour with the most pixels above the 0.2%
    # noise floor wins. Ties broken by enumeration order, which doesn't
    # matter at 0.002 threshold. Threshold is sized for the warehouse's
    # 0.30 m emissive cubes viewed from a 4 m vantage — those occupy
    # ~290 pixels of a 76,800-pixel frame (~0.38%). The maze's larger
    # 0.50 m cylinder markers at ~1 m typically fill 5-10%, so the
    # lower threshold is conservative for both worlds.
    marker = None
    n_marker = 0
    centroid = None
    threshold = total * 0.002
    best_tag = max(counts, key=lambda k: counts[k])
    if counts[best_tag] > threshold:
        marker = best_tag
        n_marker = counts[best_tag]
        sx = sum_x[best_tag]
        sy = sum_y[best_tag]
        centroid = {
            "x_norm": (sx / n_marker) / max(w - 1, 1),
            "y_norm": (sy / n_marker) / max(h - 1, 1),
        }

    # Patch std-dev of (R+G+B) summed-brightness; same shape as numpy std
    # over the 3-channel patch up to a normalising constant. Threshold is
    # tuned empirically from this maze world's roughcast walls.
    if patch_n > 0:
        mean_v = patch_sum / patch_n
        var_v = max(0.0, (patch_sum_sq / patch_n) - mean_v * mean_v)
        patch_std = var_v ** 0.5
    else:
        patch_std = 0.0
    # patch_std is std of (R+G+B) per pixel. Empirical thresholds tuned
    # against husky_maze_blind.wbt: a roughcast wall ~1 m away gives
    # std ~10-25; a wall touching the camera ~5-15; an open corridor
    # 70-200+; a partial wall + floor view in the 30-60 range.
    wall_close = patch_std < 50.0
    wall_close_score = max(0.0, min(1.0, (80.0 - patch_std) / 80.0))

    bot_mean = bot_sum / bot_n if bot_n else 0
    mid_mean = mid_sum / mid_n if mid_n else 0
    floor_visible = bot_mean < mid_mean - 24  # >8 per channel × 3 channels

    mean_brightness = bright_sum / (total * 3)

    # Per-colour fractions for any caller that wants to disambiguate
    # mixed frames (e.g., two coloured tags both partially in view).
    # Fractions tiny enough to be sensor noise are dropped on the wire
    # to keep the JSON response small.
    color_fractions = {
        c: round(n / total, 4)
        for c, n in counts.items()
        if n / total >= 0.001
    }
    return {
        "wall_close": bool(wall_close),
        "wall_close_score": round(wall_close_score, 2),
        "patch_std": round(patch_std, 1),
        "marker": marker,
        "marker_pixels": n_marker,
        "marker_fraction": round(n_marker / total, 4),
        "marker_centroid": centroid,
        "color_fractions": color_fractions,
        "mean_brightness": round(mean_brightness, 1),
        "floor_visible": bool(floor_visible),
    }


def make_handler(state: EyeState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/image":
                with state.lock:
                    if not state.cameras:
                        self._json(503, {"error": "cameras not yet ready"})
                        return
                    state.image_request += 1
                    req_id = state.image_request
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    with state.lock:
                        served = state.image_served
                        if served >= req_id:
                            pngs = dict(state.image_png)
                            break
                    time.sleep(0.02)
                else:
                    self._json(504, {"error": "timed out waiting for capture"})
                    return
                # Backwards-compat: image_base64 returns the front camera
                # so existing callers (the bridge's /camera proxy) keep
                # working. cameras: {name: base64} carries all four
                # cardinal frames so multi-camera-aware callers can use
                # them.
                front_b64 = base64.b64encode(pngs.get("front", b"")).decode("ascii")
                cams_b64 = {
                    name: base64.b64encode(p).decode("ascii")
                    for name, p in pngs.items()
                }
                self._json(200, {
                    "width": state.width,
                    "height": state.height,
                    "fov_rad": state.fov,
                    "encoding": "image/png; base64",
                    "image_base64": front_b64,
                    "cameras": cams_b64,
                    "camera_names": list(cams_b64.keys()),
                    "tracking_pose": {
                        "x": state.husky_x,
                        "y": state.husky_y,
                        "yaw": state.husky_yaw,
                    },
                    "sim_time": state.sim_time,
                })
                return
            if self.path == "/scan":
                # Local-perception path: pure-Python analyse all 4 cardinal
                # frames and return a small structured dict of high-level tags.
                # The agent never sees pixels; this drops the per-cell chat
                # input from ~120 K image tokens to ~1 K of JSON.
                with state.lock:
                    if not state.cameras:
                        self._json(503, {"error": "cameras not yet ready"})
                        return
                    state.image_request += 1
                    req_id = state.image_request
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    with state.lock:
                        served = state.image_served
                        if served >= req_id:
                            pngs = dict(state.image_png)
                            raws = dict(state.image_raw)
                            break
                    time.sleep(0.02)
                else:
                    self._json(504, {"error": "timed out waiting for capture"})
                    return
                cams = {}
                for name, raw in raws.items():
                    cams[name] = _analyze_bgra(raw, state.width, state.height)
                self._json(200, {
                    "width": state.width,
                    "height": state.height,
                    "fov_rad": state.fov,
                    "cameras": cams,
                    "tracking_pose": {
                        "x": state.husky_x,
                        "y": state.husky_y,
                        "yaw": state.husky_yaw,
                    },
                    "sim_time": state.sim_time,
                })
                return
            if self.path == "/status":
                with state.lock:
                    self._json(200, {
                        "ready": bool(state.cameras),
                        "width": state.width,
                        "height": state.height,
                        "fov_rad": state.fov,
                        "camera_names": list(state.cameras.keys()),
                        "tracking_pose": {
                            "x": state.husky_x,
                            "y": state.husky_y,
                            "yaw": state.husky_yaw,
                        },
                        "sim_time": state.sim_time,
                        "last_tick_at": state.last_tick_at,
                        "served": state.image_served,
                    })
                    return
            self._json(404, {"error": "not found"})

    return Handler


def _yaw_from_orientation(orient) -> float:
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def _find_husky_body_pose(supervisor):
    """Find the husky's base_link Solid via getFromDef(EYE_TRACK_DEF).
    The DEF name defaults to 'HUSKY' for single-husky worlds; multi-
    husky worlds override per-eye via the --track-def controllerArg."""
    husky = supervisor.getFromDef(EYE_TRACK_DEF)
    if husky is None:
        print(f"[husky_eye] getFromDef({EYE_TRACK_DEF!r}) returned None — world must have DEF {EYE_TRACK_DEF} URDFRobot (or pass --track-def)")
        return None
    body = _find_link(husky, "base_link") or _find_link(husky, "base_footprint")
    if body is None:
        print(f"[husky_eye] HUSKY found but base_link / base_footprint not in subtree")
    return body


def _find_link(node, target):
    if node is None:
        return None
    name_f = node.getField("name")
    if name_f is not None:
        try:
            if name_f.getSFString() == target:
                return node
        except Exception:
            pass
    children = node.getField("children")
    if children is not None:
        try:
            count = children.getCount()
        except Exception:
            count = 0
        for i in range(count):
            found = _find_link(children.getMFNode(i), target)
            if found is not None:
                return found
    endpoint = node.getField("endPoint")
    if endpoint is not None:
        found = _find_link(endpoint.getSFNode(), target)
        if found is not None:
            return found
    return None


def main():
    # UTF-8 stdout on Windows.
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(encoding="utf-8", errors="backslashreplace")
        except Exception:
            pass

    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    self_node = supervisor.getSelf()
    if self_node is None:
        print("[husky_eye] ERROR: getSelf() returned None — supervisor TRUE missing?")
        return

    # Enable every cardinal camera that exists on this Robot. front_camera
    # is required (legacy single-camera worlds still ship only that one);
    # right/back/left are optional — the multi-cam blind world has all four.
    cameras = {}
    for short, dev_name in [
        ("front", "front_camera"),
        ("right", "right_camera"),
        ("back",  "back_camera"),
        ("left",  "left_camera"),
    ]:
        d = supervisor.getDevice(dev_name)
        if d is not None:
            d.enable(time_step)
            cameras[short] = d
    if "front" not in cameras:
        print("[husky_eye] ERROR: no 'front_camera' device — must be a Camera child of this Robot")
        return
    front = cameras["front"]
    cam_w = front.getWidth()
    cam_h = front.getHeight()
    cam_fov = front.getFov()
    print(f"[husky_eye] cameras: {list(cameras.keys())} ({cam_w}x{cam_h} fov={cam_fov:.2f} rad)")

    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    # Prime URDF expansion so we can find husky's base_link.
    supervisor.step(time_step)
    supervisor.step(time_step)

    husky_body = _find_husky_body_pose(supervisor)
    if husky_body is None:
        print("[husky_eye] ERROR: could not find husky base_link in scene")
        return
    try:
        body_name = husky_body.getField("name").getSFString()
    except Exception:
        body_name = "?"
    print(f"[husky_eye] tracking husky body: {body_name!r}")

    state = EyeState()
    state.cameras = cameras
    state.width = cam_w
    state.height = cam_h
    state.fov = cam_fov

    # Start HTTP server.
    server = ThreadingHTTPServer((EYE_HOST, EYE_PORT), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[husky_eye] HTTP listening on http://{EYE_HOST}:{EYE_PORT}")

    while supervisor.step(time_step) != -1:
        # Track the husky's pose: position camera ~0.55 m above + 0.3 m
        # forward of the husky's base_link, oriented to match its yaw.
        try:
            pos = husky_body.getPosition()
            orient = husky_body.getOrientation()
        except Exception:
            continue
        if pos is None:
            continue
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if math.isnan(x) or math.isnan(y):
            continue
        yaw = _yaw_from_orientation(orient)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        # World-frame mounting offset.
        ox = MOUNT_FORWARD_M * cos_y - MOUNT_LATERAL_M * sin_y
        oy = MOUNT_FORWARD_M * sin_y + MOUNT_LATERAL_M * cos_y
        new_x = x + ox
        new_y = y + oy
        new_z = z + MOUNT_HEIGHT_M
        if translation_field is not None:
            translation_field.setSFVec3f([new_x, new_y, new_z])
        if rotation_field is not None:
            # Match yaw — the camera's local -Z (forward) should align
            # with the husky's +X heading. axis-angle (0, 0, 1, yaw).
            rotation_field.setSFRotation([0.0, 0.0, 1.0, yaw])

        with state.lock:
            state.husky_x = x
            state.husky_y = y
            state.husky_yaw = yaw
            state.last_tick_at = time.time()
            state.sim_time += time_step / 1000.0
            req = state.image_request
            served = state.image_served

        if req > served:
            try:
                pngs_now = {}
                raws_now = {}
                for name, dev in cameras.items():
                    raw = dev.getImage()
                    if raw:
                        raws_now[name] = raw
                        pngs_now[name] = encode_png(cam_w, cam_h, raw)
                with state.lock:
                    state.image_png = pngs_now
                    state.image_raw = raws_now
                    state.image_served = req
            except Exception as exc:
                print(f"[husky_eye] capture failed: {exc}")
                with state.lock:
                    state.image_served = req


if __name__ == "__main__":
    main()
