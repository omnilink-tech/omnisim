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



"""Shot-list-driven cinematic render CLI for OmniSim.



Reads a JSON or YAML shot list, talks to a running capture service (or

starts one ad-hoc), and produces video files.



Shot list shape (JSON shown; YAML works the same):



    {

      "world": "projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld",

      "resolution": [3840, 2160],

      "fov": 0.785398,

      "shots": [

        {

          "name": "orbit_warehouse",

          "output": "warehouse_orbit.mp4",

          "duration_s": 8.0,

          "fps": 60,

          "codec": "h264",

          "crf": 16,

          "ease": "smoothstep",

          "warmup_steps": 60,

          "settle_steps_per_frame": 1,

          "path_keyframes": [

            {"t": 0.0, "position": [-12, -12, 6], "target": [0, 0, 1]},

            {"t": 4.0, "position": [12, -12, 6],  "target": [0, 0, 1]},

            {"t": 8.0, "position": [12,  12, 6],  "target": [0, 0, 1]}

          ]

        }

      ]

    }



Usage:

    # Talk to an already-running capture service on the default port

    python scripts/capture/render.py shotlist.yaml



    # Start a dedicated capture service for this run, then exit

    python scripts/capture/render.py shotlist.yaml --ad-hoc



    # Override target service

    python scripts/capture/render.py shotlist.yaml --host 127.0.0.1 --port 6791

"""



from __future__ import annotations



import argparse

import json

import os

import socket

import subprocess

import sys

import time

import urllib.error

import urllib.request

from pathlib import Path



REPO_ROOT = Path(__file__).resolve().parents[2]





def load_shot_list(path: Path) -> dict:

    text = path.read_text(encoding="utf-8")

    if path.suffix.lower() in {".yaml", ".yml"}:

        try:

            import yaml  # type: ignore

        except ImportError as exc:

            raise SystemExit(

                f"YAML shot list requires PyYAML: pip install pyyaml ({exc})"

            )

        return yaml.safe_load(text) or {}

    return json.loads(text)





def http_post(url: str, payload: dict, timeout: float) -> dict:

    body = json.dumps(payload).encode("utf-8")

    req = urllib.request.Request(

        url, data=body, headers={"Content-Type": "application/json"}, method="POST"

    )

    try:

        with urllib.request.urlopen(req, timeout=timeout) as resp:

            data = resp.read()

            ctype = resp.headers.get("Content-Type", "")

            if "application/json" in ctype:

                return json.loads(data.decode("utf-8") or "{}")

            return {"raw_bytes": len(data), "content_type": ctype}

    except urllib.error.HTTPError as exc:

        try:

            err_body = json.loads(exc.read().decode("utf-8") or "{}")

        except Exception:

            err_body = {"error": str(exc)}

        raise SystemExit(f"POST {url} failed ({exc.code}): {err_body.get('error', err_body)}")

    except urllib.error.URLError as exc:

        raise SystemExit(f"POST {url} failed: {exc}")





def http_get(url: str, timeout: float) -> dict:

    try:

        with urllib.request.urlopen(url, timeout=timeout) as resp:

            return json.loads(resp.read().decode("utf-8") or "{}")

    except urllib.error.URLError as exc:

        raise SystemExit(f"GET {url} failed: {exc}")





def detect_orphan_sim() -> list[tuple[int, str]]:

    """Return `(pid, image_or_cmd)` tuples for any omnisim-bin processes

    (or `webots` on non-Windows) currently running. The capture pipeline

    can't recover from an orphan OmniSim from a prior failed render: even

    if the supervisor controller's TCP port has been freed, the surviving

    sim process holds GPU/memory state that destabilizes a freshly-

    launched second instance, and the failure mode is a mid-render

    supervisor RPC crash with no diagnostic in the captured OmniSim log.

    Better to refuse-and-tell than retry-and-silently-corrupt.



    PIDs are required so callers can target specific processes

    (`taskkill /F /PID <pid>`) instead of killing every omnisim-bin on the

    host (`taskkill /F /IM omnisim-bin.exe`), which would take down any

    unrelated harness or render session running in parallel.

    """

    try:

        if sys.platform == "win32":

            out = subprocess.check_output(

                ["tasklist", "/NH", "/FO", "CSV"], text=True, errors="replace"

            )

            results: list[tuple[int, str]] = []

            for line in out.splitlines():

                if not line.lower().startswith('"omnisim-bin'):

                    continue

                parts = [p.strip('"') for p in line.split(",")]

                if len(parts) < 2:

                    continue

                try:

                    pid = int(parts[1])

                except ValueError:

                    continue

                results.append((pid, parts[0]))

            return results

        else:

            out = subprocess.check_output(

                ["pgrep", "-a", "webots"], text=True, errors="replace"

            )

            results = []

            for line in out.splitlines():

                line = line.strip()

                if not line:

                    continue

                pid_str, _, cmd = line.partition(" ")

                try:

                    pid = int(pid_str)

                except ValueError:

                    continue

                results.append((pid, cmd or "webots"))

            return results

    except (subprocess.CalledProcessError, FileNotFoundError):

        return []





def kill_sim_pids(pids: list[int]) -> None:

    """Force-kill the given PIDs. Per-PID so we don't take down unrelated

    sessions; uses `taskkill /F /PID` on Windows and `kill -9` on POSIX.

    """

    if not pids:

        return

    if sys.platform == "win32":

        cmd = ["taskkill", "/F"]

        for pid in pids:

            cmd.extend(["/PID", str(pid)])

        try:

            subprocess.run(cmd, capture_output=True, check=False)

        except Exception:

            pass

    else:

        for pid in pids:

            try:

                subprocess.run(

                    ["kill", "-9", str(pid)], capture_output=True, check=False

                )

            except Exception:

                pass





def wait_for_service(host: str, port: int, deadline_s: float) -> bool:

    deadline = time.time() + deadline_s

    while time.time() < deadline:

        try:

            with socket.create_connection((host, port), timeout=0.5):

                return True

        except OSError:

            time.sleep(0.2)

    return False





def start_ad_hoc_service(host: str, port: int) -> subprocess.Popen:

    """Spawn an omnisim_capture.py subprocess. Returns the Popen handle so

    the caller can SIGTERM it on exit. Inherits PATH/env from the parent so

    the user's existing Webots/PATH wiring carries over.

    """

    script = Path(__file__).resolve().parent / "omnisim_capture.py"

    cmd = [sys.executable, str(script), "--host", host, "--port", str(port)]

    proc = subprocess.Popen(

        cmd,

        env=os.environ.copy(),

        stdout=sys.stdout,

        stderr=sys.stderr,

    )

    return proc





def render_shot_list(

    shot_list: dict,

    host: str,

    port: int,

    request_timeout_s: float,

    dry_run: bool,

) -> int:

    base = f"http://{host}:{port}"



    world = shot_list.get("world")

    if not isinstance(world, str) or not world:

        raise SystemExit("shot list missing 'world'")

    resolution = shot_list.get("resolution") or [1920, 1080]

    if not isinstance(resolution, list) or len(resolution) != 2:

        raise SystemExit("'resolution' must be a 2-list [width, height]")

    width, height = int(resolution[0]), int(resolution[1])

    fov = float(shot_list.get("fov", 0.785398))

    load_wait_s = float(shot_list.get("load_wait_s", 12.0))

    shots = shot_list.get("shots") or []

    if not isinstance(shots, list) or not shots:

        raise SystemExit("shot list 'shots' must be a non-empty list")



    if dry_run:

        print(json.dumps({

            "world": world,

            "resolution": [width, height],

            "fov": fov,

            "shots": shots,

        }, indent=2))

        return 0



    print(f"[render] loading world {world} ({width}x{height})")

    load_result = http_post(

        f"{base}/world/load",

        {"path": world, "wait_s": load_wait_s,

         "width": width, "height": height, "fov": fov},

        timeout=load_wait_s + 30.0,

    )

    if not load_result.get("ok"):

        print(f"[render] load failed: {load_result}", file=sys.stderr)

        return 1

    print(f"[render] loaded in {load_result.get('load_ms')}ms; camera {load_result.get('camera')}")



    failures = 0

    for shot in shots:

        name = shot.get("name") or "shot"

        output = shot.get("output")

        if not isinstance(output, str) or not output:

            print(f"[render] shot {name!r} skipped: missing 'output'", file=sys.stderr)

            failures += 1

            continue

        payload = {

            "path_keyframes": shot.get("path_keyframes"),

            "duration_s": shot.get("duration_s"),

            "fps": shot.get("fps", 60),

            "output": output,

            "codec": shot.get("codec", "h264"),

            "crf": shot.get("crf", 16),

            "ease": shot.get("ease", "smoothstep"),

            "warmup_steps": shot.get("warmup_steps", 0),

            "keep_frames": shot.get("keep_frames", False),

            "preset": shot.get("preset", "slow"),

            "lossless": shot.get("lossless", False),

        }

        # settle_steps_per_frame and playback_speed are mutually exclusive at

        # the API layer (explicit settle wins). Only forward whichever the user

        # set, so we don't accidentally pin a default that overrides

        # playback_speed.

        if "settle_steps_per_frame" in shot:

            payload["settle_steps_per_frame"] = shot["settle_steps_per_frame"]

        if "playback_speed" in shot:

            payload["playback_speed"] = shot["playback_speed"]

        print(f"[render] shot {name!r}: {payload['fps']}fps × {payload['duration_s']}s -> {output}")

        started = time.time()

        result = http_post(f"{base}/capture/sequence", payload, timeout=request_timeout_s)

        elapsed_ms = int((time.time() - started) * 1000)

        if "output" in result:

            ratio = result.get("playback_ratio")

            ratio_str = f", playback {ratio:.2f}x" if isinstance(ratio, (int, float)) else ""

            print(f"[render] shot {name!r} done in {elapsed_ms}ms -> {result['output']}{ratio_str}")

        else:

            print(f"[render] shot {name!r} failed: {result}", file=sys.stderr)

            failures += 1



    return 0 if failures == 0 else 2





def main() -> int:

    p = argparse.ArgumentParser(

        description="Render a shot list against the OmniSim capture service"

    )

    p.add_argument("shot_list", help="path to a JSON or YAML shot list")

    p.add_argument("--host", default="127.0.0.1")

    p.add_argument("--port", type=int, default=6791)

    p.add_argument(

        "--ad-hoc",

        action="store_true",

        help="start a dedicated capture service for this run and shut it down on exit",

    )

    p.add_argument(

        "--service-startup-s",

        type=float,

        default=15.0,

        help="seconds to wait for an --ad-hoc service to bind",

    )

    p.add_argument(

        "--request-timeout-s",

        type=float,

        default=600.0,

        help="seconds to wait for /capture/sequence (long jobs need a high cap)",

    )

    p.add_argument("--dry-run", action="store_true", help="parse and validate the shot list, then exit")

    args = p.parse_args()



    shot_list_path = Path(args.shot_list)

    if not shot_list_path.exists():

        raise SystemExit(f"shot list not found: {shot_list_path}")

    shot_list = load_shot_list(shot_list_path)



    proc: subprocess.Popen | None = None

    # PIDs of omnisim-bin processes that existed BEFORE we launched our ad-hoc

    # service. Used at cleanup to scope the survivor-kill to "our" instance:

    # any omnisim-bin PID we see at shutdown that wasn't already there at

    # startup is something our run spawned. This is what stops the cleanup

    # path from killing omnisim-bin instances owned by other sessions.

    pre_existing_sim: set[int] = set()

    try:

        if args.ad_hoc:

            orphans = detect_orphan_sim()

            if orphans:

                listing = "\n".join(f"    PID {pid}  ({name})" for pid, name in orphans)

                kill_cmd = (

                    "taskkill /F /PID <pid>" if sys.platform == "win32"

                    else "kill -9 <pid>"

                )

                print(

                    f"[render] refusing to start: {len(orphans)} OmniSim process(es) "

                    f"still running. The capture pipeline can't reliably co-exist "

                    f"with another OmniSim instance (GPU/memory pressure causes "

                    f"mid-render crashes). Kill the specific PID(s) below; do NOT "

                    f"use `/IM omnisim-bin.exe` if other sessions are running.\n"

                    f"{listing}\n"

                    f"  {kill_cmd}",

                    file=sys.stderr,

                )

                return 1

            pre_existing_sim = {pid for pid, _ in detect_orphan_sim()}

            print(f"[render] starting ad-hoc capture service on {args.host}:{args.port}")

            proc = start_ad_hoc_service(args.host, args.port)

            if not wait_for_service(args.host, args.port, args.service_startup_s):

                print("[render] ad-hoc service failed to bind in time", file=sys.stderr)

                return 1

            # Confirm the service is actually serving HTTP, not just bound.

            try:

                http_get(f"http://{args.host}:{args.port}/healthz", timeout=5.0)

            except SystemExit:

                print("[render] ad-hoc service bound but /healthz did not respond", file=sys.stderr)

                return 1

        return render_shot_list(

            shot_list,

            host=args.host,

            port=args.port,

            request_timeout_s=args.request_timeout_s,

            dry_run=args.dry_run,

        )

    finally:

        if proc is not None and proc.poll() is None:

            # Try the graceful path first: POST /shutdown, which lets the

            # service tear down its OmniSim subprocess in-process. On Windows,

            # proc.terminate() (TerminateProcess) bypasses the service's

            # signal handler and would orphan OmniSim otherwise.

            try:

                http_post(f"http://{args.host}:{args.port}/shutdown", {}, timeout=5.0)

            except SystemExit:

                # Service may already be exiting / unreachable; fall through

                # to the forceful path.

                pass

            try:

                proc.wait(timeout=15)

            except subprocess.TimeoutExpired:

                try:

                    proc.terminate()

                except Exception:

                    pass

                try:

                    proc.wait(timeout=5)

                except subprocess.TimeoutExpired:

                    proc.kill()

                except Exception:

                    pass

            except Exception:

                pass



        # Belt-and-suspenders: scan for any omnisim-bin processes that survived

        # the graceful shutdown. If the supervisor controller crashed mid-

        # render (the "WinError 10054" scenario), omnisim-bin.exe can be left

        # in a zombie state — main thread dead, sockets closed, but the

        # process still alive holding GPU/memory state. proc.terminate() on

        # the parent doesn't reach it.

        #

        # Scope the kill to PIDs that *weren't* there when we launched: any

        # omnisim-bin still running that wasn't in `pre_existing_sim` is

        # something our run spawned. Killing by /IM (or `pkill -f`) would

        # take down every omnisim-bin on the host, including unrelated

        # harness/render sessions — which is exactly the failure mode that

        # made parallel agents step on each other.

        if args.ad_hoc:

            survivors = detect_orphan_sim()

            our_survivors = [

                (pid, name) for pid, name in survivors

                if pid not in pre_existing_sim

            ]

            foreign = [

                (pid, name) for pid, name in survivors

                if pid in pre_existing_sim

            ]

            if our_survivors:

                listing = ", ".join(f"PID {pid}" for pid, _ in our_survivors)

                print(

                    f"[render] cleaning up {len(our_survivors)} surviving OmniSim "

                    f"process(es) from this run ({listing})",

                    file=sys.stderr,

                )

                kill_sim_pids([pid for pid, _ in our_survivors])

            if foreign:

                listing = ", ".join(f"PID {pid}" for pid, _ in foreign)

                print(

                    f"[render] note: {len(foreign)} other OmniSim process(es) are "

                    f"still running but were not spawned by this render "

                    f"({listing}); leaving them alone",

                    file=sys.stderr,

                )





if __name__ == "__main__":

    sys.exit(main())

