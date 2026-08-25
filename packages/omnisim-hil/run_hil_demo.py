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

"""One command that flies the whole hardware-in-the-loop rig.

Starts the autopilot process, then the simulator, waits for the flight, and
prints a report built from what was MEASURED on the link rather than from what
either side says about itself.

Order matters and is not arbitrary. The autopilot binds the UDP port and the
aircraft sends to it, so the autopilot must be listening first: start the engine
first and the opening seconds of the sensor stream go to a closed port, which
looks exactly like an autopilot that was slow to arm.

Process hygiene is the other thing this script owes you. An orphaned
``omnisim-bin`` holds a TCP port, a GPU context and about a gigabyte, and this
repo has a history of leaking them. Every exit path here -- success, failure,
exception, Ctrl-C, timeout -- goes through one teardown that kills the engine's
whole process TREE, because the engine spawns the controller as a child and
killing only the parent leaves that child running and still sending MAVLink.
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
import platform
import signal
import socket
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
WORLD = os.path.join(HERE, "worlds", "hil_delivery_aircraft.omniworld")
AUTOPILOT = os.path.join(HERE, "autopilot", "omnisim_autopilot.py")
DEFAULT_MISSION = os.path.join(HERE, "autopilot", "missions", "delivery_route.json")

IS_WINDOWS = platform.system() == "Windows"

_LIVE: List[subprocess.Popen] = []


# --------------------------------------------------------------------------
# process lifetime
# --------------------------------------------------------------------------


def _spawn(cmd: List[str], env: Dict[str, str], stdout=None, stderr=None) -> subprocess.Popen:
    kwargs: Dict[str, Any] = {"env": env, "cwd": REPO, "stdout": stdout, "stderr": stderr}
    if IS_WINDOWS:
        # A new process group means our own Ctrl-C does not race the children:
        # we tear them down deliberately in _teardown instead.
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True
    proc = subprocess.Popen(cmd, **kwargs)
    _LIVE.append(proc)
    return proc


def _kill_tree(proc: Optional[subprocess.Popen], grace: float = 3.0) -> None:
    """Kill a process AND everything it spawned. Idempotent."""
    if proc is None or proc.poll() is not None:
        return
    try:
        if IS_WINDOWS:
            # taskkill /T is the only reliable way to reach the controller the
            # engine spawned: TerminateProcess on the engine alone orphans it.
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=20, check=False)
        else:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass
    try:
        proc.wait(timeout=grace)
    except Exception:
        try:
            if not IS_WINDOWS:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            else:
                proc.kill()
        except Exception:
            pass


def _teardown() -> None:
    for proc in reversed(_LIVE):
        _kill_tree(proc)


atexit.register(_teardown)


# --------------------------------------------------------------------------
# environment
# --------------------------------------------------------------------------


def engine_binary() -> str:
    candidates = [
        os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe"),
        os.path.join(REPO, "bin", "omnisim-bin"),
        os.path.join(REPO, "Contents", "MacOS", "omnisim"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise SystemExit(
        "no OmniSim binary found. Looked for:\n  " + "\n  ".join(candidates) +
        "\nBuild it first (build_omni.bat on Windows).")


def engine_env(log_path: str, extra: Dict[str, str]) -> Dict[str, str]:
    env = dict(os.environ)
    env["OMNISIM_HOME"] = REPO
    env["WEBOTS_HOME"] = REPO          # still consumed by qt_utils and the build
    env["OMNISIM_LOG_PATH"] = log_path
    if IS_WINDOWS:
        # Exactly launch.bat's ordering, and for its reasons: mingw64\bin leads
        # so Qt6 and libstdc++ resolve, and newton-runtime goes on the TAIL so it
        # cannot become the interpreter that runs the controller.
        mingw = os.path.join(REPO, "msys64", "mingw64", "bin")
        newton = os.path.join(mingw, "newton-runtime")
        path = mingw + os.pathsep + env.get("PATH", "")
        if os.path.isdir(newton):
            path = path + os.pathsep + newton
        env["PATH"] = path
    env.update(extra)
    return env


def find_free_udp_port(preferred: int, span: int = 40) -> int:
    """Return `preferred` if it is free, else the next free port above it.

    This machine runs more than one agent and more than one engine, so a hard
    failure on a busy 14560 would be a false negative about the rig.
    """
    for port in range(preferred, preferred + span):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", port))
            return port
        except OSError:
            continue
        finally:
            probe.close()
    raise SystemExit("no free UDP port in [%d, %d)" % (preferred, preferred + span))


def wait_until_bound(port: int, timeout: float = 20.0) -> bool:
    """Block until something else holds the UDP port -- i.e. the autopilot is up."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return True
        finally:
            probe.close()
        time.sleep(0.1)
    return False


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------


def read_newton_verdict(log_path: str) -> Optional[Dict[str, Any]]:
    """Read the engine's own backend-verdict sidecar.

    Its presence is the proof Newton actually drove the run: the engine deletes
    any stale copy when it truncates the log at startup, so a file here belongs
    to THIS run. Scraping the log for the same claim is the fallback, not this.
    """
    sidecar = log_path + ".newton.json"
    try:
        with open(sidecar, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def count_log_lines(log_path: str) -> Dict[str, int]:
    counts = {"errors": 0, "warnings": 0}
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                low = line.lower()
                if "error" in low:
                    counts["errors"] += 1
                elif "warning" in low:
                    counts["warnings"] += 1
    except OSError:
        pass
    return counts


def print_report(report: Dict[str, Any]) -> None:
    flight = report.get("flight") or {}
    line = "=" * 74
    print("")
    print(line)
    print("OmniSim HIL demo -- %s" % report.get("condition", "nominal"))
    print(line)
    print("  world              %s" % os.path.relpath(WORLD, REPO))
    print("  mission            %s" % flight.get("mission", "?"))
    print("  link               udp 127.0.0.1:%d" % report.get("port", 0))
    print("  engine             %s" % report.get("engine_mode", "?"))
    physics = report.get("physics") or {}
    if physics:
        print("  physics            %s / %s (finalised=%s, degraded=%s)"
              % (physics.get("backend"), physics.get("solver"),
                 physics.get("finalised"), physics.get("degraded")))
    else:
        print("  physics            NO backend-verdict sidecar -- Newton did not "
              "finalise this world")
    print("  " + "-" * 70)
    if not flight:
        print("  NO FLIGHT SUMMARY -- the autopilot produced no report.")
        print(line)
        return

    reached = flight.get("waypoints_reached_labels") or []
    print("  waypoints          %d of %d: %s"
          % (flight.get("waypoints_reached", 0), flight.get("waypoints_total", 0),
             ", ".join(reached) or "none"))
    if flight.get("waypoint_reach_times_s"):
        print("  reached at         %s s"
              % ", ".join("%.1f" % t for t in flight["waypoint_reach_times_s"]))
    print("  flight time        %.1f s simulated in %.1f s wall (%.1fx real time)"
          % (flight.get("sim_seconds", 0.0), report.get("wall_seconds", 0.0),
             report.get("realtime_factor", 0.0)))
    print("  distance flown     %.1f m" % flight.get("distance_m", 0.0))
    if flight.get("altitude_error_mean_m") is not None:
        print("  altitude tracking  mean %.2f m, worst %.2f m against the commanded "
              "profile" % (flight["altitude_error_mean_m"], flight["altitude_error_max_m"]))
    if flight.get("altitude_settled_error_mean_m") is not None:
        print("  ... settled        mean %.2f m, worst %.2f m (excluding the %.0f s "
              "after each commanded step)"
              % (flight["altitude_settled_error_mean_m"],
                 flight["altitude_settled_error_max_m"], flight.get("settle_delay_s", 5.0)))
    print("  altitude range     %.1f to %.1f m"
          % (flight.get("altitude_min_m") or 0.0, flight.get("altitude_max_m") or 0.0))
    print("  airspeed           %.1f to %.1f m/s, mean %.1f"
          % (flight.get("airspeed_min_m_s") or 0.0, flight.get("airspeed_max_m_s") or 0.0,
             flight.get("airspeed_mean_m_s") or 0.0))
    print("  peak bank flown    %.1f deg (command limit 35)"
          % (flight.get("bank_max_deg") or 0.0))
    rx = flight.get("mavlink_rx_from_sim") or {}
    print("  mavlink sim->ap    %d packets (%s)"
          % (flight.get("mavlink_rx_total", 0),
             ", ".join("%s %d" % kv for kv in sorted(rx.items())) or "none"))
    print("  mavlink ap->sim    %d packets (HIL_ACTUATOR_CONTROLS + HEARTBEAT)"
          % flight.get("mavlink_tx_to_sim", 0))
    print("  codec              %s" % (flight.get("codec_stats"),))
    print("  control rate       %d updates, %.1f ms of sim time apart"
          % (flight.get("control_ticks", 0), flight.get("mean_control_dt_ms", 0.0)))
    print("  gps vs truth       %.2f m worst disagreement"
          % (flight.get("gps_vs_truth_max_m") or 0.0))
    print("  airborne at end    %s (%.1f m, %.1f m/s)"
          % (flight.get("airborne_at_end"), flight.get("final_altitude_m", 0.0),
             flight.get("final_airspeed_m_s", 0.0)))
    print("  stopped because    %s" % flight.get("stopped_because", "?"))
    print("  " + "-" * 70)
    print("  VERDICT            %s" % report.get("verdict", "?"))
    print("  artifacts          %s" % os.path.relpath(report.get("out_dir", ""), REPO))
    print(line)


# --------------------------------------------------------------------------


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Fly the OmniSim HIL delivery aircraft under the external autopilot.")
    p.add_argument("--gui", action="store_true",
                   help="windowed engine instead of headless (slower: this world's "
                        "8 ms step cannot be paced in real time on Windows)")
    p.add_argument("--wind", type=float, default=0.0, help="steady wind, m/s from the east")
    p.add_argument("--turbulence", type=float, default=0.0, help="gust sigma, m/s")
    p.add_argument("--payload", type=float, default=0.0,
                   help="payload kg. NOTE: this reaches the aero model's mass, which "
                        "only feeds its trim-speed report -- the simulated inertia "
                        "comes from the world's Physics node, so this does not "
                        "currently change the dynamics")
    p.add_argument("--duration", type=float, default=90.0,
                   help="simulated seconds before the aircraft stops flying")
    p.add_argument("--mission", default=DEFAULT_MISSION)
    p.add_argument("--port", type=int, default=14560)
    p.add_argument("--cruise-airspeed", type=float, default=16.0)
    p.add_argument("--json", action="store_true", help="print the report as JSON as well")
    p.add_argument("--out-dir", default="",
                   help="where to write logs (default .local-runs/hil/<stamp>)")
    p.add_argument("--keep-engine", action="store_true",
                   help="leave the engine running at exit (debugging only)")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    # The autopilot child inherits this stdout. Ours is block-buffered when the
    # demo is piped or redirected, so without this the parent's lines all land
    # after the child's and the report reads as though it ran first.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass
    args = parse_args(argv)
    binary = engine_binary()
    if not os.path.exists(WORLD):
        raise SystemExit("world not found: %s" % WORLD)
    if not os.path.exists(args.mission):
        raise SystemExit("mission not found: %s" % args.mission)

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = args.out_dir or os.path.join(REPO, ".local-runs", "hil", stamp)
    os.makedirs(out_dir, exist_ok=True)

    log_path = os.path.join(out_dir, "omnisim_log.txt")
    summary_path = os.path.join(out_dir, "flight_summary.json")
    tick_log = os.path.join(out_dir, "autopilot_ticks.jsonl")
    engine_out = os.path.join(out_dir, "engine_stdout.txt")

    port = find_free_udp_port(args.port)
    if port != args.port:
        print("[demo] udp %d was busy; using %d" % (args.port, port))

    condition = "wind %.1f m/s, turbulence sigma %.1f m/s" % (args.wind, args.turbulence) \
        if (args.wind or args.turbulence) else "calm air"

    print("[demo] %s" % condition)
    print("[demo] artifacts -> %s" % os.path.relpath(out_dir, REPO))

    autopilot_proc: Optional[subprocess.Popen] = None
    engine_proc: Optional[subprocess.Popen] = None
    engine_handle = None
    started = time.time()
    verdict = "not run"
    interrupted = False

    try:
        # 1. Autopilot first: it owns the port the aircraft sends to.
        autopilot_cmd = [
            sys.executable, AUTOPILOT,
            "--port", str(port),
            "--mission", args.mission,
            "--cruise-airspeed", str(args.cruise_airspeed),
            "--summary", summary_path,
            "--log", tick_log,
            "--max-sim-seconds", str(args.duration + 5.0),
        ]
        autopilot_proc = _spawn(autopilot_cmd, dict(os.environ))
        if not wait_until_bound(port):
            raise SystemExit("the autopilot never bound udp %d" % port)
        print("[demo] autopilot listening on udp 127.0.0.1:%d" % port)

        # 2. Engine. HIL_INTERNAL_AUTOPILOT is pinned OFF so a stale shell value
        #    cannot quietly turn this into a flight-model demo with the external
        #    autopilot talking to nothing.
        controller_env = {
            "HIL_INTERNAL_AUTOPILOT": "0",
            "HIL_MAVLINK_PORT": str(port),
            "HIL_WIND": str(args.wind),
            "HIL_TURBULENCE": str(args.turbulence),
            "HIL_PAYLOAD_KG": str(args.payload),
            "HIL_MAX_SECONDS": str(args.duration),
            "HIL_TELEMETRY": os.path.join(out_dir, "aircraft_telemetry.jsonl"),
        }
        cmd = [binary, WORLD]
        if args.gui:
            # Be EXPLICIT about the mode. With no --mode flag the engine falls
            # back to the General/startupMode preference, which is PAUSE on a
            # fresh profile -- the window opens, nothing flies, and the
            # autopilot times out waiting for a sensor stream that never starts.
            cmd += ["--mode=realtime", "--stdout", "--stderr"]
            engine_mode = "windowed --mode=realtime"
        else:
            cmd += ["--batch", "--mode=fast", "--no-rendering", "--minimize",
                    "--stdout", "--stderr"]
            engine_mode = "headless --mode=fast"
        # Expect this to be EMPTY on Windows, and do not read a diagnosis into
        # that: omnisim-bin.exe is a GUI-subsystem binary, and the controller
        # print() path (WbLog::appendStdout) writes to std::cout without ever
        # reaching the file log. So the aircraft controller's own view of the
        # link is not recoverable here -- every packet count in the report is
        # the AUTOPILOT's count, which covers both directions anyway.
        engine_handle = open(engine_out, "w", encoding="utf-8", errors="replace")
        engine_proc = _spawn(cmd, engine_env(log_path, controller_env),
                             stdout=engine_handle, stderr=subprocess.STDOUT)
        print("[demo] engine started (%s), pid %d" % (engine_mode, engine_proc.pid))

        # 3. Wait for the FLIGHT, not the engine: the engine keeps stepping after
        #    the aircraft controller has finished, so the autopilot exiting is
        #    what marks the end of the run.
        budget = (args.duration * 4.0 + 180.0) if args.gui else (args.duration * 2.0 + 240.0)
        try:
            autopilot_proc.wait(timeout=budget)
        except subprocess.TimeoutExpired:
            print("[demo] autopilot did not finish within %.0f s wall; stopping" % budget)
            _kill_tree(autopilot_proc)
            verdict = "TIMED OUT"

    except KeyboardInterrupt:
        interrupted = True
        verdict = "INTERRUPTED"
        print("\n[demo] interrupted; shutting both processes down")
    finally:
        _kill_tree(autopilot_proc)
        if not args.keep_engine:
            _kill_tree(engine_proc)
        elif engine_proc is not None:
            print("[demo] --keep-engine: leaving pid %d running" % engine_proc.pid)
        if engine_handle is not None:
            engine_handle.close()

    wall = time.time() - started

    flight: Dict[str, Any] = {}
    try:
        with open(summary_path, "r", encoding="utf-8") as handle:
            flight = json.load(handle)
    except (OSError, ValueError):
        pass

    if verdict == "not run":
        if not flight:
            verdict = "FAILED -- no flight summary"
        elif flight.get("mission_complete") and flight.get("airborne_at_end"):
            verdict = "PASS -- every waypoint reached and still flying"
        elif flight.get("airborne_at_end"):
            verdict = "PARTIAL -- still airborne, %d of %d waypoints" % (
                flight.get("waypoints_reached", 0), flight.get("waypoints_total", 0))
        else:
            verdict = "FAILED -- the aircraft did not stay airborne"

    report: Dict[str, Any] = {
        "condition": condition,
        "wind_m_s": args.wind,
        "turbulence_sigma_m_s": args.turbulence,
        "payload_kg": args.payload,
        "duration_s": args.duration,
        "port": port,
        "engine_mode": "windowed" if args.gui else "headless --mode=fast",
        "wall_seconds": round(wall, 1),
        "realtime_factor": round((flight.get("sim_seconds") or 0.0) / wall, 2) if wall else 0.0,
        "physics": read_newton_verdict(log_path),
        "engine_log": count_log_lines(log_path),
        "out_dir": out_dir,
        "flight": flight,
        "verdict": verdict,
    }

    print_report(report)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))

    with open(os.path.join(out_dir, "demo_report.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)

    if interrupted:
        return 130
    return 0 if verdict.startswith("PASS") else 1


if __name__ == "__main__":
    sys.exit(main())
