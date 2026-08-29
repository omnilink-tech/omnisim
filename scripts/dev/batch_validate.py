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

"""Validate MANY worlds through ONE engine process instead of N.

WHY THIS EXISTS
---------------
Every batch lane in this tree -- the smoke set, the 140-world newton readiness
sweep, the 45-world device sweep, conformance -- spawns a fresh `omnisim-bin`
per world. That pays the engine's fixed per-process cost every single time:
process start, the embedded CPython, and the warp / newton / mujoco_warp import
chain. MEASURED in a bare interpreter (machine 9722d23d12a3, warm warp cache):

    import warp + newton            1251 ms
    import mujoco_warp               859 ms
    SolverMuJoCo(1 body)              58 ms   <- the only per-WORLD part
    SolverMuJoCo (2nd, same process)  22 ms

i.e. the expensive part is per-PROCESS, and a second world in the same process
skips essentially all of it. The harness already knows how to hot-reload a
world into a live engine; this just drives it over a work list.

MEASURED on machine 9722d23d12a3 (16-core, RTX 3060 laptop), 19 loadable
worlds -- the newton smoke set, the cloth world, the 8-Husky swarm, heightfield
terrain, the starter/showcase demos and empty.wbt:

    fresh process per world  174.0 s   (9158 ms/world)   18/19 ok
    one engine   (-j1)        70.7 s   (3719 ms/world)   19/19    2.46x
    two engines  (-j2)        49.4 s   (2602 ms/world)   19/19    3.52x
    four engines (-j4)        34.6 s   (1822 ms/world)   19/19    5.03x
    six engines  (-j6)        36.9 s   (1940 ms/world)   19/19    4.71x

-j4 is the sweet spot on this box and -j6 is already SLOWER: each engine is a
whole process with its own embedded CPython (~1 GB resident), so RAM, disk and
GPU contention bound this well before the 16 cores do. Reuse and parallelism
compose -- reuse removes the per-process import cost within a worker,
parallelism hides what is left behind other workers.

The reuse half grows with how many SMALL worlds a lane checks: a repeat load of
a simple world settles at ~1.25 s against ~7.4 s cold, while an asset-heavy
world (warehouse_husky) still costs ~8 s hot because its cost is parse, not
startup.

⚠ AND IT IS NOT ONLY SPEED -- look at the ok counts above. Spawning fresh
engines back-to-back loses worlds to the documented engine startup race
("roughly one launch in three" on this machine -- attributed and fixed in the
engine on 2026-08-29, see the comment block above looks_like_startup_race in
headless_runner.py): 18/19 here, and 7/10 on an earlier 10-world corpus where
all three casualties PASS when run individually. Every reuse configuration
scored 19/19. Reusing the engine sidesteps the race entirely, which on a
140-world sweep may matter more than the 5x.

Verified it can go RED, on worlds broken two different ways: a bad EXTERNPROTO
target and an undeclared node type both FAIL here with named diagnostics
(PARSE_ERROR, missing-declaration), and `run-headless` independently FAILs the
same two -- so this is equally strict, not merely faster.

WHAT THIS DOES NOT DO
---------------------
This is a LOAD check: it answers "does this world parse, build and finalise",
which is the same question `run-headless --until-finalized` answers, with the
same limits. It does NOT observe behaviour, and it is NOT a physics verdict --
a body free-falling through a missing collision surface loads perfectly well.
For that, keep using `run-headless --duration N --fail-on-runaway`.

⚠ The harness injects a supervisor into a sibling copy of each world. That is
how it can hot-reload at all, and it is why this reports a load verdict rather
than a pristine-world verdict. A world that ONLY fails with a supervisor
present would pass here and fail under run-headless; nothing observed so far
does that, but it is the honest caveat.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PORT = 6789
# Generous: a cold FIRST load on an asset-heavy world legitimately takes tens of
# seconds on a slow disk. This is a ceiling, never a sleep.
LOAD_TIMEOUT_S = 300.0
HARNESS_READY_TIMEOUT_S = 90.0


def _post(port: int, path: str, payload: dict, timeout: float) -> dict:
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode(errors="replace"))


def _get(port: int, path: str, timeout: float = 10.0) -> dict:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as resp:
        return json.loads(resp.read().decode(errors="replace"))


def _harness_alive(port: int) -> bool:
    try:
        return bool(_get(port, "/healthz", timeout=1.0).get("ok"))
    except Exception:  # noqa: BLE001
        return False


def _port_free(port: int) -> bool:
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) != 0


def pick_ports(preferred: int, count: int) -> list[int]:
    """`count` DISTINCT ports, each with its (port, port+1) pair free.

    Parallel workers each own a whole engine, so they each need their own pair.
    Spaced 100 apart so a worker's supervisor port can never collide with the
    next worker's HTTP port.
    """
    ports: list[int] = []
    candidate = preferred
    while len(ports) < count and candidate < preferred + 100 * (count + 40):
        if _port_free(candidate) and _port_free(candidate + 1):
            ports.append(candidate)
        candidate += 100
    if len(ports) < count:
        raise SystemExit(f"[batch] could not find {count} free (port, port+1) pairs "
                         f"from :{preferred}")
    return ports


def pick_port(preferred: int) -> int:
    """First port whose (port, port+1) PAIR is free.

    The harness needs both: `port` for HTTP and `port+1` for the supervisor
    controller that binds inside the engine subprocess. Batch lanes hit this
    constantly, because the thing most likely to be holding port+1 is the
    PREVIOUS batch run's engine -- terminating the harness does not always take
    its child with it, and the symptom is a refusal at startup rather than
    anything the caller did wrong. Walk to the next free pair instead of dying;
    an explicitly-requested --port is still honoured, and only the default
    scans.
    """
    # Scan WIDE. Two candidates was not enough in practice: a previous run's
    # leaked harness holds its pair, so consecutive runs walk up the range, and
    # the scan has to outlast a session's worth of residue rather than give up
    # after one hop.
    for candidate in range(preferred, preferred + 4000, 100):
        if _harness_alive(candidate):
            return candidate  # reuse a live one
        if _port_free(candidate) and _port_free(candidate + 1):
            return candidate
    raise SystemExit(
        f"[batch] no free (port, port+1) pair in :{preferred}-{preferred + 4000}. "
        f"Almost certainly leaked harnesses from earlier runs still holding them.")


def start_harness(port: int, light: bool) -> subprocess.Popen | None:
    """Start a harness we own. Returns None if one is already listening."""
    if _harness_alive(port):
        print(f"[batch] reusing the harness already on :{port}")
        return None
    script = REPO_ROOT / "scripts" / "harness" / "omnisim_harness.py"
    env = dict(**__import__("os").environ)
    env.setdefault("OMNISIM_HOME", str(REPO_ROOT))
    if light:
        env["OMNISIM_HARNESS_LIGHT"] = "1"
    # ⚠ CAPTURE the harness's output, never DEVNULL it. The harness refuses to
    # bind when the port is already taken and explains exactly why -- including
    # which world the existing harness is on. Discarding that turned a
    # one-line, self-explaining failure into a bare "exited early (code 2)"
    # with nothing to go on. A port can be held by a listener that no longer
    # answers /healthz (a killed harness's zombie child), so "not alive" and
    # "free to bind" are NOT the same question and this path must report which
    # one failed.
    log_path = REPO_ROOT / ".build_tmp" / f"batch_harness_{port}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, str(script), "--port", str(port)],
        stdout=log_handle, stderr=subprocess.STDOUT, env=env,
    )
    deadline = time.time() + HARNESS_READY_TIMEOUT_S
    while time.time() < deadline:
        if proc.poll() is not None:
            log_handle.close()
            detail = log_path.read_text(encoding="utf-8", errors="replace").strip()
            tail = "\n".join(detail.splitlines()[-12:]) or "(no output)"
            raise SystemExit(
                f"[batch] the harness exited immediately (code {proc.returncode}).\n"
                f"[batch] its own explanation follows -- the usual cause is that "
                f"port {port} is already held:\n{tail}")
        if _harness_alive(port):
            return proc
        time.sleep(0.25)
    proc.terminate()
    log_handle.close()
    raise SystemExit(f"[batch] harness did not answer /healthz on :{port} within "
                     f"{HARNESS_READY_TIMEOUT_S:.0f}s (see {log_path})")


def _kill_tree(proc: subprocess.Popen) -> None:
    """Kill a process AND its descendants (the harness's engine child).

    Only useful while the parent is still ALIVE -- once it exits, the engine is
    re-parented and there is no tree left to walk, which is why the port-holder
    sweep below exists as the real backstop.
    """
    if sys.platform == "win32":
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False)
    else:
        import os
        import signal
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            proc.kill()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def _kill_port_holder(port: int) -> bool:
    """Kill whatever is LISTENING on `port`, whoever's child it is.

    This is the backstop that actually works. The engine is spawned by the
    harness, so killing the harness first ORPHANS it -- and a tree kill aimed
    at the dead parent then finds nothing to walk. MEASURED: a -j3 run left 3
    orphaned engines holding their supervisor ports even with terminate(),
    kill() and taskkill /T all in the path. Identify the holder by the socket
    instead of by parentage.
    """
    if sys.platform == "win32":
        probe = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             f"(Get-NetTCPConnection -LocalPort {port} -State Listen "
             f"-ErrorAction SilentlyContinue | Select-Object -First 1)"
             f".OwningProcess"],
            capture_output=True, text=True, check=False)
        pid = (probe.stdout or "").strip()
        if not pid.isdigit():
            return False
        subprocess.run(["taskkill", "/F", "/T", "/PID", pid],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False)
        return True
    probe = subprocess.run(["lsof", "-ti", f"tcp:{port}", "-sTCP:LISTEN"],
                           capture_output=True, text=True, check=False)
    killed = False
    for line in (probe.stdout or "").split():
        if line.strip().isdigit():
            import os
            import signal
            try:
                os.kill(int(line), signal.SIGKILL)
                killed = True
            except (ProcessLookupError, PermissionError):
                pass
    return killed


def shutdown_harness(proc: subprocess.Popen | None, port: int) -> None:
    """Stop a harness we started, and PROVE its ports are released.

    terminate() alone was not enough: MEASURED, two harnesses survived it and
    kept holding their supervisor ports (:6790, :6890) into the next run, which
    then failed to find a free pair. A batch tool that leaks a port per
    invocation poisons every later run on the machine, so this escalates to
    kill() and then waits for the socket to actually go away rather than
    assuming it did.

    BE QUICK ABOUT IT. The first version waited 15 s for terminate() before
    escalating -- and terminate() is exactly the thing that does not work here,
    so every shutdown paid the full 15 s. With -j4 that was ~64 s of pure
    teardown: MEASURED, it made a 41.5 s parallel run take 105 s of wall clock,
    i.e. the cleanup cost more than the work and made -j4 look SLOWER than -j1
    while the in-band number said otherwise. Give terminate() a short courtesy
    window, then kill.

    KILL THE TREE, NOT THE PROCESS. The harness spawns `omnisim-bin` as a
    CHILD, and that child is what holds the supervisor port. Killing only the
    harness python leaves the engine alive holding :port+1 for ever --
    MEASURED after a J-sweep: 11 ports held and 7 orphaned engines, which then
    pushed later runs further and further up the port range. Neither
    terminate() nor kill() on the parent touches it, so this uses taskkill /T
    on Windows and a process group elsewhere.
    """
    if proc is None:
        return
    # TREE FIRST, while the tree still exists. The tempting order -- terminate()
    # the harness, then clean up -- cannot work on Windows: terminate() is
    # TerminateProcess, i.e. an immediate kill with NO chance for the harness to
    # reap its engine, so the engine is orphaned the instant the parent dies and
    # nothing downstream can find it by parentage. MEASURED with terminate-first:
    # 3 orphaned engines per -j3 run even with kill() and taskkill /T also in the
    # path. Killing the tree while the parent is alive takes the engine with it.
    _kill_tree(proc)
    deadline = time.time() + 5
    while time.time() < deadline:
        if _port_free(port) and _port_free(port + 1):
            return
        time.sleep(0.2)
    # Still held: the engine outlived its parent and is now an orphan, so go by
    # socket rather than by parentage.
    for p in (port + 1, port):
        if not _port_free(p):
            _kill_port_holder(p)
    deadline = time.time() + 8
    while time.time() < deadline:
        if _port_free(port) and _port_free(port + 1):
            return
        time.sleep(0.2)
    print(f"[batch] warning: :{port}/:{port + 1} still held after shutdown; a later "
          f"run will skip to the next pair", file=sys.stderr)


def validate(port: int, world: Path, light: bool) -> dict:
    started = time.time()
    try:
        body = _post(port, "/world/load",
                     {"path": str(world), "light": light},
                     timeout=LOAD_TIMEOUT_S)
    except urllib.error.HTTPError as exc:
        # A rejected world comes back as 4xx with the harness's STRUCTURED
        # diagnostics in the body (WORLD_PARSE_SYNTAX_ERROR, PROTO_NAME_MISMATCH,
        # ...). Reporting just "HTTP Error 422" throws away the only part that
        # tells the caller what is wrong with their world -- which is the whole
        # reason to drive the harness rather than scrape a log.
        ms = int((time.time() - started) * 1000)
        try:
            body = json.loads(exc.read().decode(errors="replace"))
        except Exception:  # noqa: BLE001
            return {"world": str(world), "ok": False, "ms": ms,
                    "errors": [f"harness rejected the world: HTTP {exc.code}"]}
        diags = body.get("diagnostics") or []
        errs = [f"{d.get('code')}: {d.get('message')}" for d in diags
                if str(d.get("severity", "")).lower() == "error"]
        return {"world": str(world), "ok": False, "ms": ms,
                "errors": errs or [f"{body.get('code') or 'LOAD_REJECTED'}: "
                                   f"{body.get('error') or f'HTTP {exc.code}'}"],
                "warnings": sum(1 for d in diags
                                if str(d.get("severity", "")).lower() == "warning")}
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        return {"world": str(world), "ok": False, "ms": int((time.time() - started) * 1000),
                "errors": [f"harness call failed: {exc}"]}
    ms = int((time.time() - started) * 1000)
    diags = body.get("diagnostics") or []
    errors = [d for d in diags if str(d.get("severity", "")).lower() == "error"]
    return {
        "world": str(world),
        "ok": bool(body.get("ok")) and not errors,
        "ms": ms,
        "errors": [f"{d.get('code')}: {d.get('message')}" for d in errors],
        "warnings": sum(1 for d in diags
                        if str(d.get("severity", "")).lower() == "warning"),
    }


def _run_parallel(worlds: list[Path], jobs: int, args, light: bool) -> int:
    """K engines, each hot-reloading its own slice of the work list.

    This composes with the single-engine win rather than replacing it: reuse
    removes the per-PROCESS import cost within a worker, and parallelism hides
    what is left behind other workers. Each worker owns a whole engine (~1 GB
    resident, its own embedded CPython), so the useful K is bounded by RAM and
    thermals long before it is bounded by cores.

    Slices are CONTIGUOUS, not round-robin, so a worker's first (cold) load is
    amortised over its own slice and the per-world cost a reader sees in the
    log stays interpretable.
    """
    import threading

    ports = pick_ports(args.port, jobs)
    chunks: list[list[Path]] = [worlds[i::jobs] for i in range(jobs)]
    print(f"[batch] {len(worlds)} world(s), {jobs} engines in parallel "
          f"(ports {', '.join(str(p) for p in ports)}), "
          f"{'light' if light else 'heavy'} supervisor")

    results: list[dict] = []
    lock = threading.Lock()
    owned: list[subprocess.Popen | None] = [None] * jobs
    done = [0]

    def worker(idx: int) -> None:
        port, mine = ports[idx], chunks[idx]
        if not mine:
            return
        try:
            owned[idx] = start_harness(port, light)
        except SystemExit as exc:
            with lock:
                for w in mine:
                    results.append({"world": str(w), "ok": False, "ms": 0,
                                    "errors": [f"worker {idx} could not start: {exc}"]})
            return
        for w in mine:
            r = validate(port, w, light)
            with lock:
                results.append(r)
                done[0] += 1
                mark = "ok  " if r["ok"] else "FAIL"
                rel = w.relative_to(REPO_ROOT) if w.is_relative_to(REPO_ROOT) else w
                print(f"  [{done[0]:>3}/{len(worlds)}] {mark} {r['ms']:>6} ms  "
                      f"(w{idx}) {rel}")
                for e in r["errors"]:
                    print(f"           {e}")

    t0 = time.time()
    threads = [threading.Thread(target=worker, args=(i,), daemon=True)
               for i in range(jobs)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    total_ms = int((time.time() - t0) * 1000)

    if not args.keep_harness:
        # Concurrently: serial teardown of K engines is itself O(K) seconds,
        # and it lands entirely outside the measured window where it silently
        # inflates wall clock.
        closers = [threading.Thread(target=shutdown_harness, args=(p, ports[i]),
                                    daemon=True)
                   for i, p in enumerate(owned) if p is not None]
        for c in closers:
            c.start()
        for c in closers:
            c.join(timeout=20)

    failed = [r for r in results if not r["ok"]]
    print(f"[batch] {len(results) - len(failed)}/{len(results)} ok in "
          f"{total_ms / 1000:.1f}s ({total_ms // max(len(results), 1)} ms/world, "
          f"{jobs} engines)")
    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps({"total_ms": total_ms, "jobs": jobs, "results": results},
                       indent=2), encoding="utf-8")
        print(f"[batch] wrote {args.json_out}")
    if failed:
        print(f"[batch] FAILED: {len(failed)}")
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Validate many worlds through ONE engine process (load check).")
    ap.add_argument("worlds", nargs="*", help="World paths (repo-relative or absolute).")
    ap.add_argument("--from-file", help="Read world paths from a file, one per line.")
    ap.add_argument("--glob",
                    help="Glob for worlds, e.g. 'projects/**/worlds/*.omniworld'. "
                         "Both .omniworld and the legacy .wbt are accepted; pass the "
                         "flag twice to sweep both.")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--jobs", "-j", type=int, default=1,
                    help="Engines to run in PARALLEL, each on its own port pair "
                         "(default 1). Composes with the single-engine reuse. "
                         "Each engine is a whole process (~1 GB resident), so RAM "
                         "and thermals bound the useful value well before cores do.")
    ap.add_argument("--heavy", action="store_true",
                    help="Keep the per-step contact/joint/grip trackers. Default is "
                         "light: this is a LOAD check, so the trackers are pure cost "
                         "(and on a cloth world they are ~50x the step).")
    ap.add_argument("--json", dest="json_out", help="Write per-world results as JSON.")
    ap.add_argument("--keep-harness", action="store_true",
                    help="Leave the harness running afterwards (for a follow-up session).")
    args = ap.parse_args()

    worlds: list[Path] = []
    for w in args.worlds:
        worlds.append(Path(w) if Path(w).is_absolute() else REPO_ROOT / w)
    if args.from_file:
        for line in Path(args.from_file).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                worlds.append(Path(line) if Path(line).is_absolute() else REPO_ROOT / line)
    if args.glob:
        worlds.extend(sorted(REPO_ROOT.glob(args.glob)))
    # Preserve order, drop duplicates.
    seen: set[str] = set()
    worlds = [w for w in worlds if not (str(w) in seen or seen.add(str(w)))]
    if not worlds:
        ap.error("no worlds given (positional, --from-file or --glob)")

    missing = [w for w in worlds if not w.exists()]
    if missing:
        for w in missing:
            print(f"[batch] MISSING: {w}", file=sys.stderr)
        return 2

    light = not args.heavy
    jobs = max(1, min(args.jobs, len(worlds)))
    if jobs > 1:
        return _run_parallel(worlds, jobs, args, light)

    port = args.port if args.port != DEFAULT_PORT else pick_port(DEFAULT_PORT)
    if port != args.port:
        print(f"[batch] :{args.port} pair busy -- using :{port}")
    print(f"[batch] {len(worlds)} world(s), one engine process, "
          f"{'light' if light else 'heavy'} supervisor")
    owned = start_harness(port, light)
    results: list[dict] = []
    t0 = time.time()
    try:
        for i, w in enumerate(worlds, 1):
            r = validate(port, w, light)
            results.append(r)
            mark = "ok  " if r["ok"] else "FAIL"
            print(f"  [{i:>3}/{len(worlds)}] {mark} {r['ms']:>6} ms  "
                  f"{w.relative_to(REPO_ROOT) if w.is_relative_to(REPO_ROOT) else w}")
            for e in r["errors"]:
                print(f"           {e}")
    finally:
        if owned is not None and not args.keep_harness:
            shutdown_harness(owned, port)

    total_ms = int((time.time() - t0) * 1000)
    failed = [r for r in results if not r["ok"]]
    print(f"[batch] {len(results) - len(failed)}/{len(results)} ok in "
          f"{total_ms / 1000:.1f}s ({total_ms // max(len(results), 1)} ms/world)")
    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps({"total_ms": total_ms, "results": results}, indent=2),
            encoding="utf-8")
        print(f"[batch] wrote {args.json_out}")
    if failed:
        print(f"[batch] FAILED: {len(failed)}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
