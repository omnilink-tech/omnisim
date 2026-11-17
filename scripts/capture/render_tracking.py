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

"""Subject-tracking renderer for OmniSim.

The standard `render.py` plays a pre-baked camera keyframe path. This one
*follows* one or more subjects each frame: it polls their world poses via
the capture service's `/world/subject` endpoint and aims the camera at
their midpoint with an adaptive height that zooms out when they separate.

Why: combat / action shots need the camera *close enough to see detail*
AND *framed around the subjects* even as they move. A static camera can't
do both. The tracking loop solves that framing-vs-detail tradeoff: it
holds the subjects in frame while pulling the camera only as far back as
their separation requires.

CLI:

    python scripts/capture/render_tracking.py \
        --world projects/robot_combat/worlds/demos/newton_husky_combat_2.omniworld \
        --subjects husky husky_b \
        --duration 45 --output combat_tracking.mp4 \
        --z-min 8.5 --z-max 13 --z-base 6.5 --z-sep-gain 1.4 \
        --request-timeout-s 1200

Also importable as `render_tracking(...)`.
"""
from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

# Sibling helpers: reuse render.py's service lifecycle + orphan cleanup.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from render import (  # noqa: E402
    detect_orphan_sim, kill_sim_pids,
    start_ad_hoc_service, wait_for_service, http_post, http_get,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "social" / "youtube_videos" / "captures"


def _ffmpeg() -> str:
    p = shutil.which("ffmpeg")
    if p:
        return p
    fb = "C:/ffmpeg/bin/ffmpeg.exe"
    if Path(fb).exists():
        return fb
    raise SystemExit("ffmpeg not found")


def _adaptive_z(sep: float, z_min: float, z_max: float, z_base: float, gain: float) -> float:
    """Adaptive height: zoom in when subjects are close, out when they separate."""
    return max(z_min, min(z_max, sep * gain + z_base))


def render_tracking(
    world: str,
    subjects: list[str],
    duration_s: float,
    output: str,
    fps: int = 30,
    width: int = 1920,
    height: int = 1080,
    fov: float = 0.785398,
    warmup_steps: int = 8,
    settle_steps: int = 2,
    target_z: float = 0.4,
    z_min: float = 8.5,
    z_max: float = 13.0,
    z_base: float = 6.5,
    z_sep_gain: float = 1.4,
    smoothing: float = 0.18,
    crf: int = 17,
    preset: str = "medium",
    svc_host: str = "127.0.0.1",
    svc_port: int = 6791,
    service_startup_s: float = 15.0,
    load_wait_s: float = 60.0,
    request_timeout_s: float = 1200.0,
    offset_x: float = 0.0,
    offset_y: float = 0.0,
) -> Path:
    """Render a tracking shot. Returns path to the output mp4.

    `subjects` is a list of robot `name` strings; the camera aims at their
    midpoint each frame. With one subject the camera locks onto it.
    """
    if not subjects:
        raise ValueError("at least one subject required")

    output_path = Path(output)
    if not output_path.is_absolute():
        output_path = DEFAULT_OUTPUT_DIR / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)

    orphans = detect_orphan_sim()
    if orphans:
        listing = ", ".join(f"PID {p}" for p, _ in orphans)
        raise SystemExit(
            f"{len(orphans)} omnisim-bin process(es) already running ({listing}); "
            f"refusing to start — kill the specific PID(s) and retry."
        )
    pre_existing = {p for p, _ in detect_orphan_sim()}

    print(f"[track] starting capture service on {svc_host}:{svc_port}")
    svc = start_ad_hoc_service(svc_host, svc_port)
    try:
        if not wait_for_service(svc_host, svc_port, service_startup_s):
            raise SystemExit("capture service did not bind in time")
        http_get(f"http://{svc_host}:{svc_port}/healthz", timeout=5.0)

        base = f"http://{svc_host}:{svc_port}"
        print(f"[track] loading {world}")
        load = http_post(f"{base}/world/load",
                         {"path": world, "wait_s": load_wait_s,
                          "width": width, "height": height, "fov": fov},
                         timeout=load_wait_s + 30.0)
        if not load.get("ok"):
            raise SystemExit(f"world load failed: {load}")
        print(f"[track] loaded in {load.get('load_ms')}ms")

        if warmup_steps > 0:
            http_post(f"{base}/sim/step", {"steps": warmup_steps}, timeout=30.0)

        frames_dir = output_path.parent / f".{output_path.stem}_frames"
        if frames_dir.exists():
            shutil.rmtree(frames_dir)
        frames_dir.mkdir(parents=True)

        n_frames = max(1, int(round(duration_s * fps)))
        print(f"[track] rendering {n_frames} frames @ {fps}fps "
              f"({duration_s:.1f}s, ~{settle_steps*16}ms sim/frame)")

        smx = smy = sz = None  # smoothed camera state
        started = time.time()
        for i in range(n_frames):
            poses = []
            for name in subjects:
                r = http_post(f"{base}/world/subject", {"name": name}, timeout=10.0)
                tr = r.get("translation")
                if not tr:
                    raise SystemExit(f"subject {name!r} has no translation: {r}")
                poses.append(tr)
            mx = sum(p[0] for p in poses) / len(poses)
            my = sum(p[1] for p in poses) / len(poses)
            if len(poses) >= 2:
                sep = max(
                    ((poses[a][0] - poses[b][0])**2 + (poses[a][1] - poses[b][1])**2) ** 0.5
                    for a in range(len(poses)) for b in range(a + 1, len(poses))
                )
            else:
                sep = 0.0
            z = _adaptive_z(sep, z_min, z_max, z_base, z_sep_gain)

            # EMA smoothing so subject jitter doesn't shake the camera.
            if smx is None:
                smx, smy, sz = mx, my, z
            else:
                a = smoothing
                smx = a * mx + (1 - a) * smx
                smy = a * my + (1 - a) * smy
                sz = a * z + (1 - a) * sz

            http_post(f"{base}/capture/camera",
                      {"position": [smx + offset_x, smy + offset_y, sz],
                       "target": [smx, smy, target_z],
                       "sync_viewpoint": True},
                      timeout=request_timeout_s)
            frame = frames_dir / f"frame_{i:06d}.png"
            http_post(f"{base}/capture/screenshot",
                      {"path": str(frame), "quality": 100},
                      timeout=request_timeout_s)
            http_post(f"{base}/sim/step", {"steps": settle_steps},
                      timeout=request_timeout_s)

            if (i + 1) % max(1, n_frames // 10) == 0:
                elapsed = time.time() - started
                eta = elapsed / (i + 1) * (n_frames - i - 1)
                print(f"[track]   {i+1}/{n_frames}  elapsed {elapsed:.0f}s  eta {eta:.0f}s")

        print(f"[track] encoding {n_frames} frames -> {output_path.name}")
        # `Supervisor.exportImage` viewport dumps can be odd-height (e.g.
        # 1896x1113); libx264 + yuv420p needs even dims, so truncate to even.
        r = subprocess.run(
            [_ffmpeg(), "-y", "-framerate", str(fps),
             "-i", str(frames_dir / "frame_%06d.png"),
             "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
             "-c:v", "libx264", "-preset", preset, "-crf", str(crf),
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_path)],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            raise SystemExit("ffmpeg encode failed:\n" + r.stderr[-1800:])
        shutil.rmtree(frames_dir, ignore_errors=True)
        elapsed = time.time() - started
        print(f"[track] done in {elapsed:.0f}s -> {output_path}")
        return output_path

    finally:
        if svc.poll() is None:
            try:
                http_post(f"http://{svc_host}:{svc_port}/shutdown", {}, timeout=5.0)
            except SystemExit:
                pass
            try:
                svc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                svc.terminate()
                try:
                    svc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    svc.kill()
        survivors = [pid for pid, _ in detect_orphan_sim() if pid not in pre_existing]
        if survivors:
            print(f"[track] cleaning {len(survivors)} surviving omnisim-bin(s): {survivors}")
            kill_sim_pids(survivors)


def main() -> int:
    p = argparse.ArgumentParser(description="Tracking-camera renderer for OmniSim.")
    p.add_argument("--world", required=True)
    p.add_argument("--subjects", nargs="+", required=True,
                   help="One or more robot name strings to follow.")
    p.add_argument("--duration", type=float, required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--warmup-steps", type=int, default=8)
    p.add_argument("--settle-steps", type=int, default=2)
    p.add_argument("--z-min", type=float, default=8.5)
    p.add_argument("--z-max", type=float, default=13.0)
    p.add_argument("--z-base", type=float, default=6.5)
    p.add_argument("--z-sep-gain", type=float, default=1.4)
    p.add_argument("--smoothing", type=float, default=0.18)
    p.add_argument("--crf", type=int, default=17)
    p.add_argument("--request-timeout-s", type=float, default=1200.0)
    p.add_argument("--offset-x", type=float, default=0.0)
    p.add_argument("--offset-y", type=float, default=0.0,
                   help="camera world Y offset (use negative for side-view shots of subjects walking along +X)")
    p.add_argument("--target-z", type=float, default=0.4,
                   help="camera target world Z (raise to ~0.5 for body-height side tracking)")
    args = p.parse_args()

    render_tracking(
        world=args.world, subjects=args.subjects,
        duration_s=args.duration, output=args.output,
        fps=args.fps, width=args.width, height=args.height,
        warmup_steps=args.warmup_steps, settle_steps=args.settle_steps,
        z_min=args.z_min, z_max=args.z_max, z_base=args.z_base, z_sep_gain=args.z_sep_gain,
        smoothing=args.smoothing, crf=args.crf,
        request_timeout_s=args.request_timeout_s,
        offset_x=args.offset_x, offset_y=args.offset_y, target_z=args.target_z,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
