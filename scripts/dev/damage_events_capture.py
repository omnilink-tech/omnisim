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

"""damage_events_capture — launch an OmniSim world that has a
harness_supervisor and stream every damage_event the supervisor emits
into a JSONL file.

This is the harness-side half of the P6 damage-suite quantitative
parity tooling (engine-migration-plan.md §13.3 P6 row). It captures
ONE world's event stream. The companion diff tool
(damage_events_diff.py) compares two such captures — typically an
ODE run vs a Newton run of the same head-on scenario.

Usage:
    python scripts/dev/damage_events_capture.py \\
        projects/robot_combat/worlds/tests/husky_head_on.omniworld \\
        --duration 20 \\
        --output _scratch/p6_ode.jsonl

Output is JSONL — one JSON object per line, ordered by step_id:
    {"step_id": ..., "ts_ms": ..., "kind": ..., "part": ..., ...}

The harness wire protocol is documented in
projects/default/controllers/harness_supervisor/harness_supervisor.py.
Both sides agree on a 4-byte big-endian length prefix + JSON body
frame, listening on localhost:6790 by default.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path


HOST = os.environ.get("OMNISIM_HARNESS_SUPERVISOR_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_HARNESS_SUPERVISOR_PORT", "6790"))


def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
    chunks = []
    remaining = n
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _read_frame(sock: socket.socket) -> dict | None:
    header = _recv_exact(sock, 4)
    if header is None:
        return None
    (length,) = struct.unpack(">I", header)
    if length == 0 or length > 16 * 1024 * 1024:
        return None
    payload = _recv_exact(sock, length)
    if payload is None:
        return None
    return json.loads(payload.decode("utf-8"))


def _write_frame(sock: socket.socket, obj: dict) -> bool:
    body = json.dumps(obj).encode("utf-8")
    try:
        sock.sendall(struct.pack(">I", len(body)) + body)
        return True
    except OSError:
        return False


def _request(sock: socket.socket, cmd: str, args: dict | None = None) -> dict | None:
    if not _write_frame(sock, {"cmd": cmd, "args": args or {}}):
        return None
    return _read_frame(sock)


def _connect_with_retry(host: str, port: int, timeout_s: float) -> socket.socket | None:
    """The harness opens its listening socket from inside the
    simulator process, which only happens AFTER the world load + first
    step. So we have to retry while the simulator is starting."""
    deadline = time.time() + timeout_s
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            sock = socket.create_connection((host, port), timeout=1.0)
            sock.settimeout(5.0)
            # Verify the listener is the harness (not a stale socket).
            reply = _request(sock, "ping")
            if reply is not None:
                return sock
            sock.close()
        except (ConnectionRefusedError, OSError, socket.timeout) as exc:
            last_error = exc
        time.sleep(0.5)
    if last_error:
        print(f"[capture] failed to connect to {host}:{port}: {last_error!r}",
              file=sys.stderr)
    return None


def _resolve_binary() -> Path:
    """Cross-platform resolution via omnisim.paths.resolve_omnisim_binary().

    Note: no webots-bin.exe fallback. No Makefile produces that name any
    more, so a leftover copy is a stale artefact; falling back to it would
    silently run a weeks-old binary instead of failing loudly.
    """
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from omnisim.paths import resolve_omnisim_binary
    cand = resolve_omnisim_binary()
    if cand:
        return Path(cand)
    raise FileNotFoundError(
        "omnisim-bin not found (Windows: msys64/mingw64/bin/omnisim-bin.exe; "
        "Linux: bin/omnisim-bin) — build the simulator first")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Capture damage_events from a harness-supervised OmniSim world.")
    ap.add_argument("world", type=Path,
                    help="Path to the .wbt world file (must declare a harness_supervisor controller).")
    ap.add_argument("--duration", type=int, default=20,
                    help="Seconds of wall-clock to capture before stopping the simulator (default: 20).")
    ap.add_argument("--output", type=Path, required=True,
                    help="JSONL output path; one event per line.")
    ap.add_argument("--poll-ms", type=int, default=200,
                    help="Polling interval for damage_events (default: 200 ms).")
    ap.add_argument("--connect-timeout", type=int, default=60,
                    help="Wait this many seconds for the harness TCP listener (default: 60).")
    ap.add_argument("--log-path", type=Path, default=None,
                    help="OMNISIM_LOG_PATH for the spawned simulator. Default: per-run unique tmp.")
    args = ap.parse_args()

    if not args.world.is_file():
        print(f"[capture] world not found: {args.world}", file=sys.stderr)
        return 2

    binary = _resolve_binary()
    repo_root = Path(__file__).resolve().parents[2]
    log_path = (args.log_path if args.log_path is not None
                else repo_root / "_scratch" / f"p6_capture_{int(time.time())}.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(repo_root)
    env["OMNISIM_LOG_PATH"] = str(log_path)
    if sys.platform == "win32":
        mingw_bin = repo_root / "msys64" / "mingw64" / "bin"
        env["PATH"] = ";".join([str(binary.parent), str(mingw_bin)]) + ";" + env.get("PATH", "")

    cmd = [str(binary), str(args.world), "--minimize", "--batch",
           "--no-rendering", "--mode=fast", "--stdout", "--stderr"]
    print(f"[capture] launching {args.world.name} (log: {log_path})")
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)

    try:
        sock = _connect_with_retry(HOST, PORT, args.connect_timeout)
        if sock is None:
            return 1
        print(f"[capture] connected to harness at {HOST}:{PORT}")

        since = 0
        all_events: list[dict] = []
        start = time.time()
        with args.output.open("w", encoding="utf-8") as outf:
            while time.time() - start < args.duration:
                if proc.poll() is not None:
                    print("[capture] simulator exited early", file=sys.stderr)
                    break
                reply = _request(sock, "damage_events",
                                 {"since": since, "limit": 1024})
                if reply is None:
                    print("[capture] socket lost", file=sys.stderr)
                    break
                if not reply.get("ok", False):
                    print(f"[capture] harness error: {reply.get('error', reply)}",
                          file=sys.stderr)
                    break
                # Wire schema: {"ok": True, "result": {"events": [...],
                # "last_step_id": int, "events_total": int}}
                result = reply.get("result") or {}
                evts = result.get("events", [])
                for evt in evts:
                    outf.write(json.dumps(evt, sort_keys=True) + "\n")
                    all_events.append(evt)
                if evts:
                    since = result.get("last_step_id", since)
                time.sleep(args.poll_ms / 1000.0)
            outf.flush()

        # Summary — events use "type" field per damage_tracker.py::_emit
        kinds: dict[str, int] = {}
        parts: dict[str, int] = {}
        for evt in all_events:
            k = evt.get("type", "?")
            kinds[k] = kinds.get(k, 0) + 1
            p = evt.get("part", "?")
            parts[p] = parts.get(p, 0) + 1
        print(f"[capture] captured {len(all_events)} events in "
              f"{time.time()-start:.1f}s wall")
        print(f"[capture] types: {sorted(kinds.items())}")
        print(f"[capture] parts: {sorted(parts.items())}")
        print(f"[capture] wrote: {args.output}")
        sock.close()
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    return 0


if __name__ == "__main__":
    sys.exit(main())
