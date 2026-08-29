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

"""Measure the engine STARTUP-RACE rate: N back-to-back headless launches, counted.

The race (public issue #3; `looks_like_startup_race` in headless_runner.py) is a
launch that dies during engine startup with the Qt teardown signature
(`QWaitCondition: Destroyed while threads are still waiting` /
`QThreadStorage: entry`), a non-zero exit, and NOT ONE ERROR/FATAL line. Its
rate was recorded as "roughly one launch in three" (AgentBench adapter,
2026-07) and 1-of-19 / 3-of-10 (batch_validate.py, 2026-08-15) -- all on
machine 9722d23d12a3, all on older binaries -- and re-measured as 0 of 80 on
the 2026-08-29 binary (25 raw + 25 raw-with-overlapping-teardown + 15 runner
+ 15 runner-heavier-worlds). A rate is only meaningful for a NAMED binary on a
NAMED machine, which is why this script prints both.

    python scripts/dev/launch_race_stress.py                     # 25 raw launches
    python scripts/dev/launch_race_stress.py -n 50 --overlap     # next engine starts while the previous tears down
    python scripts/dev/launch_race_stress.py --via-runner        # through run-headless --until-finalized (retries OFF)
    python scripts/dev/launch_race_stress.py --world a.omniworld --world b.omniworld   # rotate worlds
    python scripts/dev/launch_race_stress.py --concurrent 2 -n 20   # 10 rounds of 2 engines STARTING together
    python scripts/dev/launch_race_stress.py --concurrent 2 --stagger 12 --stdio file -n 8   # the shape that reproduced

--concurrent is the mode the 0-of-80 sweep did not have, and --stagger is the
one that reproduced. Every earlier variant starts engines one AFTER another
(back-to-back, or while the previous one tears down). What sent us here was a
parallel agent lane whose long headless runs died five times with exit 1 and
no diagnostic whenever another lane started a capture -- and THAT turned out
to be a kill, not a race: the capture service's orphan detector listed every
omnisim-bin on the host and told the second lane to `taskkill /F` them
(fixed in 85bb046d0; a live engine of another session is not an orphan). The
race below is what the search found on the way, and it is real on its own:
measured the same evening on machine 9722d23d12a3 (binary 785786e4621110d4,
pre-fix), the shape that fails is an engine STARTING AGAINST ONE ALREADY
RUNNING, 12 s apart:

    --concurrent 2 --stagger 12, stdout=devnull   3 of 7 second starters died
    --concurrent 2 --stagger 12, stdout=pipe      0 of 8
    --concurrent 2 (simultaneous), stdout=devnull 0 of 10 -- and the two never
                                                  shared a port (the connect()
                                                  probe + listen() window is
                                                  narrower than Popen's ~10 ms)

The death was never Qt's: the second engine's embedded interpreter found fd 1
dead ("[Errno 9] Bad file descriptor" out of warp's greeting print inside
newton.ModelBuilder()), the FFI smoke read that as a broken runtime, FATAL,
exit 1. The cause is main.cpp's RedirectIOToConsole(): a stdout that was not a
PIPE made the engine AttachConsole() to its launcher's console -- a console it
did not own -- and the engine handed a FILE (run-headless, the capture service)
or the null device threw that handle away for it. Both fixed 2026-08-29: the
engine keeps any stdout it was given, and the interpreter's stdio probe is a
real os.fstat() instead of a zero-length write. --stdio selects the launcher
shape so all three can be re-measured; the row records each engine's listening
port so a shared port would be visible; and a non-zero exit with zero
ERROR/FATAL lines is classified `signature: "silent_exit"` next to the Qt
`qt_teardown` marks -- both count as raced.

Exit code is 0 when no launch raced, 75 (EX_TEMPFAIL, the runner's own race
code) when at least one did, 1 when a launch failed for any OTHER reason -- a
world that genuinely fails to load must never be counted as "the flake".
Every launch gets its own OMNISIM_LOG_PATH under --out so the logs survive.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))
from headless_runner import EXIT_STARTUP_RACE, find_binary, looks_like_startup_race  # noqa: E402

DEFAULT_WORLD = "projects/samples/demos/worlds/physics/newton_smoke_test.omniworld"
HEADLESS_FLAGS = ["--minimize", "--batch", "--no-rendering", "--mode=fast", "--stdout", "--stderr"]


def _sha256_prefix(path: Path, n: int = 16) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:n]


STDIO_MODE = "devnull"  # set from --stdio; how the engine's stdout/stderr are handed to it


def _stdio(log_path: Path):
    """(stdout, stderr) kwargs for Popen, per --stdio.

    The engine's own stdio handling branches on WHAT KIND of handle fd 1 is
    (main.cpp RedirectIOToConsole: a pipe is left alone, anything else makes
    it attach to the parent console), so the launcher's choice is part of the
    measurement: run-headless hands it a FILE, the harness a PIPE, and a bare
    Popen(stdout=DEVNULL) the null device.
    """
    if STDIO_MODE == "pipe":
        # The harness reads its pipes continuously; so must we, or the engine
        # blocks on a full pipe buffer and the row reads as a hang.
        return dict(stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if STDIO_MODE == "file":
        fh = open(str(log_path) + ".stdout", "wb")
        return dict(stdout=fh, stderr=subprocess.STDOUT)
    return dict(stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _drain(proc: subprocess.Popen) -> None:
    """Start daemon readers on any pipe the engine was given (see _stdio)."""
    import threading

    for stream in (proc.stdout, proc.stderr):
        if stream is not None:
            threading.Thread(target=lambda s=stream: [None for _ in iter(lambda: s.read(1 << 16), b"")],
                             daemon=True).start()


def _env(log_path: Path) -> dict:
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    env["PATH"] = str(REPO_ROOT / "msys64" / "mingw64" / "bin") + os.pathsep + env.get("PATH", "")
    return env


def launch_raw(binary: Path, world: Path, log_path: Path, timeout_s: float, overlap: bool) -> dict:
    sidecar = Path(str(log_path) + ".newton.json")
    for p in (log_path, sidecar):
        if p.exists():
            p.unlink()
    t0 = time.time()
    proc = subprocess.Popen([str(binary), str(world)] + HEADLESS_FLAGS, env=_env(log_path), cwd=str(REPO_ROOT),
                            **_stdio(log_path))
    _drain(proc)
    rc = None
    finalised = False
    while time.time() - t0 < timeout_s:
        rc = proc.poll()
        if rc is not None:
            break
        if sidecar.exists():
            finalised = True
            break
        time.sleep(0.2)
    if rc is None:
        proc.terminate()
        # --overlap: let the NEXT engine start while this one is still tearing
        # down (the shape a batch runner that kills on timeout produces).
        try:
            proc.wait(0.05 if overlap else 15)
        except subprocess.TimeoutExpired:
            if not overlap:
                proc.kill()
    return dict(rc=rc, finalised=finalised, secs=round(time.time() - t0, 2))


def _listening_ports_by_pid(pids: set) -> dict:
    """{pid: [port, ...]} for every TCP LISTEN socket owned by one of `pids`.

    netstat on Windows, ss on Linux; anything unparseable yields {} so a port
    read can only ever be MISSING from a row, never wrong.
    """
    found: dict = {}
    try:
        if os.name == "nt":
            txt = subprocess.run(["netstat", "-ano", "-p", "tcp"], capture_output=True, text=True, timeout=20).stdout
            for ln in txt.splitlines():
                parts = ln.split()
                if len(parts) >= 5 and parts[0] == "TCP" and parts[3] == "LISTENING":
                    pid = int(parts[4])
                    if pid in pids:
                        found.setdefault(pid, []).append(int(parts[1].rsplit(":", 1)[1]))
        else:
            txt = subprocess.run(["ss", "-ltnp"], capture_output=True, text=True, timeout=20).stdout
            for ln in txt.splitlines():
                for pid in pids:
                    if f"pid={pid}," in ln:
                        found.setdefault(pid, []).append(int(ln.split()[3].rsplit(":", 1)[1]))
    except Exception:  # noqa: BLE001 -- a missing tool or an odd line must not abort the measurement
        return {}
    return {pid: sorted(set(ports)) for pid, ports in found.items()}


def launch_concurrent(binary: Path, worlds: list, log_paths: list, timeout_s: float, stagger_s: float = 0.0,
                      grace_s: float = 8.0) -> list:
    """Start len(log_paths) engines as close to simultaneously as Popen allows.

    Each engine runs until it finalises (sidecar) or exits on its own, then
    the survivors are terminated. Every row records the port(s) the engine was
    listening on while alive, so two rows sharing a port is directly visible.
    """
    procs = []
    sidecars = []
    for world, log_path in zip(worlds, log_paths):
        sidecar = Path(str(log_path) + ".newton.json")
        for p in (log_path, sidecar):
            if p.exists():
                p.unlink()
        sidecars.append(sidecar)
    t0 = time.time()
    for j, (world, log_path) in enumerate(zip(worlds, log_paths)):
        if j and stagger_s:
            # --stagger: the k-th engine starts stagger_s AFTER the previous one,
            # i.e. against an engine that is already RUNNING (finalised, with
            # controllers up), which is the shape of the 2026-08-29 losses.
            time.sleep(stagger_s)
        procs.append(subprocess.Popen([str(binary), str(world)] + HEADLESS_FLAGS, env=_env(log_path),
                                      cwd=str(REPO_ROOT), **_stdio(log_path)))
        _drain(procs[-1])
    spread_ms = round((time.time() - t0) * 1000, 1)
    n = len(procs)
    rc = [None] * n
    finalised = [False] * n
    finalised_at = [None] * n
    exited_at = [None] * n
    ports: dict = {}
    port_sampled = False
    while time.time() - t0 < timeout_s:
        now = time.time() - t0
        for i, proc in enumerate(procs):
            if exited_at[i] is not None:
                continue
            r = proc.poll()
            if r is not None:
                rc[i] = r
                exited_at[i] = round(now, 2)
            # A finalised engine is NOT done being watched: the 2026-08-29 losers
            # had a healthy .newton.json sidecar and died AFTER it, at step ~6.
            if not finalised[i] and sidecars[i].exists():
                finalised[i] = True
                finalised_at[i] = round(now, 2)
        # Sample the listening ports once every engine has had ~2 s to bind (the
        # scan is a connect() probe + listen(), well under a second), and again
        # right before the end so a late binder is not missed.
        if not port_sampled and now > 2.0:
            ports = _listening_ports_by_pid({p.pid for p in procs})
            port_sampled = True
        if all(e is not None for e in exited_at):
            break
        # Every survivor has finalised: keep watching for a post-finalize death
        # for a grace window, then end them ourselves.
        if all(exited_at[i] is not None or finalised[i] for i in range(n)):
            last = max(f for f in finalised_at if f is not None)
            if now - last > grace_s:
                break
        time.sleep(0.1)
    ports_late = _listening_ports_by_pid({p.pid for p in procs if p.poll() is None})
    for pid, plist in ports_late.items():
        ports.setdefault(pid, plist)
    for i, proc in enumerate(procs):
        if proc.poll() is None:
            proc.terminate()
    for i, proc in enumerate(procs):
        try:
            proc.wait(15)
        except subprocess.TimeoutExpired:
            proc.kill()
    rows = []
    for i, proc in enumerate(procs):
        # rc stays None for an engine WE ended (finalised, or hit the ceiling);
        # it is the engine's own code only when it exited by itself.
        rows.append(dict(rc=rc[i], finalised=finalised[i], finalised_at=finalised_at[i], exited_at=exited_at[i],
                         secs=exited_at[i] if exited_at[i] is not None else round(time.time() - t0, 2),
                         pid=proc.pid, ports=ports.get(proc.pid), spawn_spread_ms=spread_ms))
    return rows


def launch_via_runner(world: Path, log_path: Path, timeout_s: float) -> dict:
    t0 = time.time()
    cmd = [sys.executable, str(REPO_ROOT / "scripts" / "dev" / "headless_runner.py"), str(world),
           "--until-finalized", "--race-attempts", "1"]
    try:
        res = subprocess.run(cmd, env=_env(log_path), cwd=str(REPO_ROOT), capture_output=True, text=True,
                             timeout=timeout_s)
        rc, out = res.returncode, res.stdout + res.stderr
    except subprocess.TimeoutExpired:
        rc, out = None, ""
    tail = out.strip().splitlines()[-1] if out.strip() else ""
    return dict(rc=rc, finalised=(rc == 0 and tail.endswith("PASS")), secs=round(time.time() - t0, 2),
                runner_tail=tail)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--launches", type=int, default=25)
    ap.add_argument("--world", action="append", help="world(s) to rotate through (default: newton_smoke_test)")
    ap.add_argument("--overlap", action="store_true",
                    help="start the next engine while the previous is still tearing down")
    ap.add_argument("--via-runner", action="store_true",
                    help="launch through headless_runner.py --until-finalized (its retry OFF)")
    ap.add_argument("--concurrent", type=int, default=0, metavar="K",
                    help="start K engines AT THE SAME MOMENT per round (-n launches total, K per round); "
                         "the mode that reproduces the race")
    ap.add_argument("--stagger", type=float, default=0.0, metavar="S",
                    help="with --concurrent: start each engine S seconds after the previous one in the round, "
                         "so the later ones start against an engine that is already RUNNING (0 = simultaneous)")
    ap.add_argument("--grace", type=float, default=8.0,
                    help="with --concurrent: seconds to keep watching after every engine has finalised, "
                         "to catch a post-finalize death")
    ap.add_argument("--stdio", choices=("devnull", "pipe", "file"), default="devnull",
                    help="what the engine's stdout/stderr are: the null device (default), a pipe (the harness's "
                         "shape), or a per-launch file (run-headless's shape)")
    ap.add_argument("--gap", type=float, default=0.0, help="seconds to sleep between launches (0 = back-to-back)")
    ap.add_argument("--timeout", type=float, default=90.0, help="per-launch ceiling in seconds")
    ap.add_argument("--out", default=".local-runs/launch_race_stress",
                    help="directory for per-launch logs + summary.json")
    args = ap.parse_args()

    global STDIO_MODE
    STDIO_MODE = args.stdio
    binary = find_binary(REPO_ROOT)
    worlds = [REPO_ROOT / w for w in (args.world or [DEFAULT_WORLD])]
    for w in worlds:
        if not w.exists():
            print(f"world not found: {w}", file=sys.stderr)
            return 2
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    provenance = dict(binary=str(binary), binary_sha256_16=_sha256_prefix(binary),
                      binary_mtime=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(binary.stat().st_mtime)),
                      machine=platform.node(), platform=platform.platform(),
                      mode=(f"concurrent{args.concurrent}" + (f"+stagger{args.stagger:g}s" if args.stagger else "")
                            if args.concurrent else
                            "runner" if args.via_runner else ("raw+overlap" if args.overlap else "raw")),
                      gap_s=args.gap, stdio=args.stdio, worlds=[str(w.relative_to(REPO_ROOT)) for w in worlds])
    print("launch_race_stress:", json.dumps(provenance), flush=True)

    def classify(row: dict, i: int, world: Path, log_path: Path) -> dict:
        text = log_path.read_text(errors="replace") if log_path.exists() else ""
        errors = sum(1 for ln in text.splitlines() if ln.startswith(("ERROR:", "FATAL:")))
        qt = looks_like_startup_race(text) and not row["finalised"]
        # The second signature: the engine exited non-zero on its own, logged
        # nothing at ERROR/FATAL, and never finalised. (rc None = we ended it.)
        silent = row["rc"] not in (None, 0) and errors == 0 and not qt
        row.update(i=i, world=world.name, raced=(qt or silent),
                   signature=("qt_teardown" if qt else "silent_exit" if silent else None),
                   error_lines=errors, log_lines=len(text.splitlines()))
        return row

    rows = []
    if args.concurrent:
        k = args.concurrent
        for r in range(max(1, args.launches // k)):
            idx = list(range(r * k, r * k + k))
            ws = [worlds[i % len(worlds)] for i in idx]
            lps = [out / f"launch{i:03d}.log" for i in idx]
            launched = launch_concurrent(binary, ws, lps, args.timeout, args.stagger, args.grace)
            for row, i, w, lp in zip(launched, idx, ws, lps):
                row["round"] = r
                rows.append(classify(row, i, w, lp))
                print(json.dumps(rows[-1]), flush=True)
            if args.gap:
                time.sleep(args.gap)
    else:
        for i in range(args.launches):
            world = worlds[i % len(worlds)]
            log_path = out / f"launch{i:03d}.log"
            if args.via_runner:
                row = launch_via_runner(world, log_path, args.timeout)
            else:
                row = launch_raw(binary, world, log_path, args.timeout, args.overlap)
            rows.append(classify(row, i, world, log_path))
            print(json.dumps(rows[-1]), flush=True)
            if args.gap:
                time.sleep(args.gap)

    raced = sum(1 for r in rows if r["raced"])
    finalised = sum(1 for r in rows if r["finalised"])
    other_fail = sum(1 for r in rows if not r["finalised"] and not r["raced"])
    shared_port_rounds = 0
    if args.concurrent:
        by_round: dict = {}
        for r in rows:
            for p in (r.get("ports") or []):
                by_round.setdefault(r["round"], []).append(p)
        shared_port_rounds = sum(1 for ps in by_round.values() if len(ps) != len(set(ps)))
    summary = dict(provenance=provenance, launches=len(rows), finalised=finalised, raced=raced,
                   raced_qt_teardown=sum(1 for r in rows if r.get("signature") == "qt_teardown"),
                   raced_silent_exit=sum(1 for r in rows if r.get("signature") == "silent_exit"),
                   shared_port_rounds=shared_port_rounds,
                   failed_other=other_fail, race_rate=(raced / len(rows)) if rows else None)
    (out / "summary.json").write_text(json.dumps(dict(summary=summary, rows=rows), indent=1))
    print(f"SUMMARY launches={len(rows)} finalised={finalised} raced={raced} "
          f"(qt_teardown={summary['raced_qt_teardown']} silent_exit={summary['raced_silent_exit']}) "
          f"shared_port_rounds={shared_port_rounds} failed_other={other_fail} "
          f"binary={provenance['binary_sha256_16']} machine={provenance['machine']}")
    if other_fail:
        return 1
    return EXIT_STARTUP_RACE if raced else 0


if __name__ == "__main__":
    sys.exit(main())
