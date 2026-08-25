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
"""thermal_guard.py -- run a load under a hard GPU temperature ceiling.

WHY THIS EXISTS. The owner's standing constraint on this laptop is a temperature ceiling, and
"I will keep an eye on it" does not survive contact with a background agent running a
--mode=fast throughput benchmark. Measured 2026-08-22: an unattended verification sweep peaked
at 87 C against a 75 C ceiling (mean 74.8 C, 68 of 135 samples at or above 73) while the only
control in place was an intention. This turns the intention into a process that can actually
stop something.

WHAT IT CAN AND CANNOT SEE -- read this before trusting a green run.
  * GPU temperature comes from nvidia-smi. That works here.
  * CPU package temperature is NOT READABLE on this box: MSAcpi_ThermalZoneTemperature is not
    exposed by the firmware, the "Thermal Zone Information" perf counter has no instance, and
    no LibreHardwareMonitor / OpenHardwareMonitor service is installed. So a clean run here
    means "the GPU stayed under the ceiling", NEVER "the laptop stayed cool".
  * nvidia-smi -pl (power cap) is refused on this laptop GPU -- "not supported in current
    scope" -- so the hardware cannot be throttled directly. The only lever is starting less
    work, and stopping work already running.

THE CEILING IS ENFORCED AT SAMPLING RESOLUTION, NOT INSTANTLY. `run` polls every --interval
seconds (default 2) and kills on the first sample at or above the ceiling, so the TRUE peak can
overshoot. Measured 2026-08-22 with a 5 s interval: a run breached a 75 C ceiling and was killed,
but the sampled peak was 78 C. Treat the ceiling as "the load is stopped shortly after here",
not "this temperature is never reached", and report the peak this tool prints rather than the
ceiling you asked for.

FAIL-CLOSED, DELIBERATELY. If the temperature cannot be read, `run` REFUSES to start unless
--unguarded is passed. A guard that silently degrades into no guard is worse than no guard,
because it reads as protection.

USAGE
  # wrap any load; kills the whole process tree if the ceiling is breached
  python scripts/dev/thermal_guard.py run --ceiling 75 -- <cmd> [args...]

  # block until the machine has cooled before starting heavy work
  python scripts/dev/thermal_guard.py wait --below 68

  # one-shot reading, plus what is currently loading the machine
  python scripts/dev/thermal_guard.py status

Every `run` prints the PEAK it observed, so a measurement taken under it is self-describing:
a benchmark that ran hot is visible in its own output, instead of being discovered afterwards
in a log nobody opened.
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

WIN_SMI = "C:\\Windows\\System32\\nvidia-smi.exe"


def smi_path():
    if os.path.exists(WIN_SMI):
        return WIN_SMI
    return shutil.which("nvidia-smi")


def read_temp():
    """GPU temperature in C, or None if it cannot be read."""
    smi = smi_path()
    if not smi:
        return None
    try:
        out = subprocess.run([smi, "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10)
        if out.returncode != 0:
            return None
        return int(out.stdout.strip().splitlines()[0].strip())
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def engines_running():
    """Live omnisim-bin processes -- the thing that actually heats this machine."""
    try:
        if os.name == "nt":
            out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe", "/NH"],
                                 capture_output=True, text=True, timeout=10)
            return sum(1 for ln in out.stdout.splitlines() if "omnisim-bin" in ln)
        out = subprocess.run(["pgrep", "-c", "omnisim-bin"], capture_output=True, text=True,
                             timeout=10)
        return int(out.stdout.strip() or 0)
    except (OSError, ValueError, subprocess.SubprocessError):
        return -1


def kill_tree(proc):
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)],
                       capture_output=True, text=True)
    else:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except OSError:
            proc.kill()


def cmd_status(_args):
    t = read_temp()
    print("gpu_temp=%s C  engines=%d" % (t if t is not None else "UNREADABLE", engines_running()))
    print("cpu_temp=UNREADABLE (no firmware thermal zone, no perf-counter instance, "
          "no LibreHardwareMonitor/OpenHardwareMonitor)")
    return 0 if t is not None else 2


def cmd_wait(args):
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        t = read_temp()
        if t is None:
            print("thermal_guard: temperature UNREADABLE -- not waiting", file=sys.stderr)
            return 2
        if t <= args.below:
            print("thermal_guard: %d C <= %d C, proceeding" % (t, args.below))
            return 0
        print("thermal_guard: %d C > %d C, cooling..." % (t, args.below), flush=True)
        time.sleep(args.interval)
    print("thermal_guard: still above %d C after %ss" % (args.below, args.timeout),
          file=sys.stderr)
    return 1


def cmd_run(args):
    if not args.command:
        print("thermal_guard: nothing to run (put the command after `--`)", file=sys.stderr)
        return 2
    t0 = read_temp()
    if t0 is None and not args.unguarded:
        print("thermal_guard: REFUSING to start -- GPU temperature is unreadable, so the ceiling "
              "cannot be enforced. Pass --unguarded to run anyway, and say so in any result you "
              "report from it.", file=sys.stderr)
        return 2

    if t0 is not None and args.precool is not None:
        rc = cmd_wait(argparse.Namespace(below=args.precool, timeout=args.cool_timeout,
                                         interval=args.interval))
        if rc != 0:
            return rc

    popen_kw = {}
    if os.name != "nt":
        popen_kw["preexec_fn"] = os.setsid
    proc = subprocess.Popen(args.command, **popen_kw)

    peak = t0 if t0 is not None else -1
    breached = False
    try:
        while proc.poll() is None:
            time.sleep(args.interval)
            t = read_temp()
            if t is None:
                continue
            peak = max(peak, t)
            if t >= args.ceiling:
                breached = True
                print("\nthermal_guard: %d C >= ceiling %d C -- KILLING the load"
                      % (t, args.ceiling), file=sys.stderr, flush=True)
                kill_tree(proc)
                break
    except KeyboardInterrupt:
        kill_tree(proc)
        raise
    rc = proc.wait()
    print("thermal_guard: peak=%d C ceiling=%d C end=%s C %s"
          % (peak, args.ceiling, read_temp(),
             "BREACHED (load killed)" if breached else "ok"))
    return 3 if breached else rc


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("status", help="one-shot reading + what is loading the machine")
    s.set_defaults(func=cmd_status)

    w = sub.add_parser("wait", help="block until the GPU has cooled below a threshold")
    w.add_argument("--below", type=int, default=68)
    w.add_argument("--timeout", type=float, default=900)
    w.add_argument("--interval", type=float, default=10)
    w.set_defaults(func=cmd_wait)

    r = sub.add_parser("run", help="run a command under a hard ceiling")
    r.add_argument("--ceiling", type=int, default=75,
                   help="kill the load at or above this temperature (C)")
    r.add_argument("--precool", type=int, default=None,
                   help="wait until at/below this before starting (C)")
    r.add_argument("--cool-timeout", type=float, default=900)
    r.add_argument("--interval", type=float, default=2,
                   help="seconds between samples; the ceiling is only enforced at "
                        "this resolution, so the true peak can overshoot it")
    r.add_argument("--unguarded", action="store_true",
                   help="proceed even if temperature is unreadable (say so in your results)")
    r.add_argument("command", nargs=argparse.REMAINDER)
    r.set_defaults(func=cmd_run)

    args = ap.parse_args()
    if getattr(args, "command", None) and args.command and args.command[0] == "--":
        args.command = args.command[1:]
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
