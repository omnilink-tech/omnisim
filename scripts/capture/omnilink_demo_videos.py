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

"""Record short MP4 clips of every OmniLink chat demo, one per robot.

Produces files like:

    docs/media/videos/omnilink/husky.mp4
    docs/media/videos/omnilink/spot.mp4
    ...

The output drops into a folder the agent gallery (docs/showcase/agents.html)
references. Run once after wiring up OmniLink so the gallery's
"[recording pending]" placeholders fill in.

How it works
------------
For each demo, the script:

  1. Starts the OmniSim capture service (scripts/capture/omnisim_capture.py).
  2. Tells it to load the world.
  3. Posts a representative natural-language prompt to the bridge's
     /prompt endpoint so the robot does something during the recording.
  4. Triggers a short orbit-camera sequence (~6-8 s) via the capture
     service's /capture/sequence endpoint.
  5. ffmpeg-encodes the PNG sequence to h264 at 1080p60.

The capture service handles all the heavy lifting. This script is glue:
it knows what prompt to send per world and where to drop the output.

Run
---
    python scripts/capture/omnilink_demo_videos.py              # all demos
    python scripts/capture/omnilink_demo_videos.py --only husky # one demo
    python scripts/capture/omnilink_demo_videos.py --list       # show plan

The capture service has its own port (default 6791) and starts an OmniSim
subprocess. Don't run this alongside another OmniSim instance on the same
port; pass --capture-port 6892 to use a different one.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]
WORLDS_DIR = REPO_ROOT / "projects" / "samples" / "demos" / "worlds"
OUT_DIR = REPO_ROOT / "docs" / "media" / "videos" / "omnilink"
DURATION_S = float(os.environ.get("OMNILINK_VIDEO_DURATION", "6.0"))
RESOLUTION = (int(os.environ.get("OMNILINK_VIDEO_W", "1920")),
              int(os.environ.get("OMNILINK_VIDEO_H", "1080")))
FPS = int(os.environ.get("OMNILINK_VIDEO_FPS", "60"))


# (world_basename, output_basename, bridge_port, prompt, label)
DEMOS: List[Dict[str, Any]] = [
    {"world": "omnilink_tb3_burger.wbt",   "out": "tb3_burger.mp4",   "port": 8765, "prompt": "forward 1 meter, then turn left 90 degrees"},
    {"world": "omnilink_tb3_waffle.wbt",   "out": "tb3_waffle.mp4",   "port": 8765, "prompt": "forward 1 meter"},
    {"world": "omnilink_tb3_waffle_pi.wbt","out": "tb3_waffle_pi.mp4","port": 8765, "prompt": "spin"},
    {"world": "omnilink_husky.wbt",        "out": "husky.mp4",        "port": 8765, "prompt": "circle"},
    {"world": "omnilink_jackal.wbt",       "out": "jackal.mp4",       "port": 8765, "prompt": "turn around"},
    {"world": "omnilink_rosbot.wbt",       "out": "rosbot.mp4",       "port": 8765, "prompt": "forward 1 meter"},
    {"world": "omnilink_rosbot_xl.wbt",    "out": "rosbot_xl.mp4",    "port": 8765, "prompt": "forward 1 meter"},
    {"world": "omnilink_spot.wbt",         "out": "spot.mp4",         "port": 8765, "prompt": "wave hello"},
]


def _capture_url(port: int) -> str:
    return f"http://127.0.0.1:{port}"


def _http_post(url: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 30.0) -> Dict[str, Any]:
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8") or "{}")


def wait_for_service(port: int, timeout_s: float = 60.0) -> bool:
    """Poll the capture service's /health until it answers or we give up."""
    url = f"{_capture_url(port)}/health"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=1.5) as r:
                if r.status == 200:
                    return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def wait_for_bridge(port: int, timeout_s: float = 30.0) -> bool:
    """Poll the bridge's /list_robots until it responds."""
    url = f"http://127.0.0.1:{port}/list_robots"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            _http_post(url, {}, timeout=1.5)
            return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def render_demo(demo: Dict[str, Any], capture_port: int) -> bool:
    world = WORLDS_DIR / demo["world"]
    out_path = OUT_DIR / demo["out"]
    if not world.exists():
        print(f"  [SKIP] {demo['world']} not found")
        return False
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"\n--- {demo['world']} -> {out_path.relative_to(REPO_ROOT)} ---")
    print(f"  prompt: {demo['prompt']!r}")
    base = _capture_url(capture_port)

    # 1. Load the world. The capture service spawns its own OmniSim
    #    subprocess so this is independent of any user-facing OmniSim.
    _http_post(f"{base}/world/load", {
        "world": str(world),
        "width": RESOLUTION[0], "height": RESOLUTION[1],
    }, timeout=120.0)

    # 2. Wait for the world's bridge to come up.
    if not wait_for_bridge(demo["port"], timeout_s=30):
        print(f"  [FAIL] bridge {demo['port']} never came up")
        return False

    # 3. Send the prompt.
    try:
        _http_post(f"http://127.0.0.1:{demo['port']}/prompt", {"text": demo["prompt"]}, timeout=10)
    except Exception as e:
        print(f"  [WARN] /prompt failed ({e}); recording anyway")

    # 4. Trigger the orbit sequence.
    seq_payload = {
        "duration_s": DURATION_S,
        "fps": FPS,
        "shot": "orbit",  # capture_supervisor knows this preset
        "output": str(out_path),
        "codec": "h264", "crf": 16, "preset": "slow",
    }
    try:
        result = _http_post(f"{base}/capture/sequence", seq_payload, timeout=300.0)
        if not result.get("ok", True):
            print(f"  [FAIL] capture: {result}")
            return False
    except Exception as e:
        print(f"  [FAIL] /capture/sequence: {e}")
        return False

    if out_path.exists():
        print(f"  [OK] {out_path.stat().st_size / 1024 / 1024:.1f} MB")
        return True
    print(f"  [FAIL] no output file at {out_path}")
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", action="append", default=[],
                        help="Render only these demos (basename without .wbt; can repeat).")
    parser.add_argument("--list", action="store_true", help="List the demo plan and exit.")
    parser.add_argument("--capture-port", type=int, default=6791,
                        help="Port the OmniSim capture service is/will be on (default 6791).")
    parser.add_argument("--no-spawn-capture", action="store_true",
                        help="Don't start the capture service; assume it's already running.")
    args = parser.parse_args()

    demos = list(DEMOS)
    if args.only:
        wanted = set(args.only)
        demos = [d for d in demos if d["world"].replace("omnilink_", "").replace(".wbt", "") in wanted]
        if not demos:
            print(f"  no demos matched {args.only}")
            return 1

    if args.list:
        for d in demos:
            print(f"  {d['world']:32s} -> {d['out']}  prompt: {d['prompt']}")
        return 0

    capture_proc = None
    if not args.no_spawn_capture:
        capture_script = REPO_ROOT / "scripts" / "capture" / "omnisim_capture.py"
        if not capture_script.exists():
            print(f"ERROR: capture service script not found at {capture_script}")
            return 1
        print(f"  starting capture service on {args.capture_port}...")
        capture_proc = subprocess.Popen(
            [sys.executable, str(capture_script), "--port", str(args.capture_port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        if not wait_for_service(args.capture_port):
            print(f"ERROR: capture service never came up on {args.capture_port}")
            capture_proc.kill()
            return 1

    try:
        pass_count = 0
        for d in demos:
            if render_demo(d, args.capture_port):
                pass_count += 1
        print(f"\n{pass_count}/{len(demos)} demos rendered to {OUT_DIR.relative_to(REPO_ROOT)}/")
        return 0 if pass_count == len(demos) else 1
    finally:
        if capture_proc is not None:
            print("  shutting down capture service...")
            capture_proc.terminate()
            try:
                capture_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                capture_proc.kill()


if __name__ == "__main__":
    sys.exit(main())
