"""Damage-system headless dev harness.

Spawns OmniSim in --no-rendering --batch --mode=fast against a damage
arena world, connects to the harness_supervisor's :6790 wire protocol,
runs a scenario closure, reports stats, tears down.

Iteration loop is "edit code -> rerun -> read numbers" in seconds;
~5-10x faster than GUI re-launch for the same scenario.

Used by:
    python -m omnisim damage [--scenario NAME] [--world PATH] [--list]

Also imported by `omnisim.damage.regression_suite` for the numerical
regression suite.
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

from ..paths import REPO_ROOT, resolve_webots_binary

DEFAULT_WORLD = (
    REPO_ROOT / "projects" / "robot_combat" / "worlds"
    / "husky_damage_arena_drive.wbt"
)

SUPERVISOR_HOST = "127.0.0.1"
SUPERVISOR_PORT = 6790


def _pick_webots_binary() -> Path:
    binary = resolve_webots_binary()
    if not binary:
        raise SystemExit(
            "omnisim binary not found. Set OMNISIM_HOME to a built tree "
            "or build first (`python -m omnisim build all`)."
        )
    return Path(binary)


def _ensure_path() -> dict[str, str]:
    """Return an env dict with OMNISIM_HOME set and PATH augmented so
    OmniSim's Qt/mingw runtime is reachable. Required when launching
    from a context that doesn't already have the toolchain on PATH.
    """
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    extra = [
        r"C:\msys64\mingw64\bin",
        str(REPO_ROOT / "msys64" / "mingw64" / "bin"),
    ]
    if sys.platform == "win32":
        env["PATH"] = ";".join(extra + [env.get("PATH", "")])
    return env


# ---------------------------------------------------------------------------
# Wire-protocol client
# ---------------------------------------------------------------------------


class SupervisorClient:
    """Minimal client for the harness_supervisor wire protocol. Reuses
    the same length-prefixed JSON framing as scripts/harness — just
    inlined here to keep this script self-contained.
    """

    def __init__(self, host: str = SUPERVISOR_HOST, port: int = SUPERVISOR_PORT,
                 connect_timeout_s: float = 30.0):
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self._sock: socket.socket | None = None
        self._next_id = 1

    def connect(self) -> None:
        deadline = time.monotonic() + self.connect_timeout_s
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection((self.host, self.port), timeout=2)
                # create_connection's timeout becomes the socket's default
                # for subsequent recv/send. With --mode=fast the supervisor
                # can spend many wall-clock seconds on a single sim step
                # while it processes a torrent of contact events; bump the
                # post-connect timeout to 60s so calls don't spuriously
                # time out waiting for a busy supervisor.
                s.settimeout(60.0)
                self._sock = s
                return
            except OSError as exc:
                last_err = exc
                time.sleep(0.5)
        raise SystemExit(
            f"could not connect to supervisor on {self.host}:{self.port} "
            f"within {self.connect_timeout_s}s: {last_err}"
        )

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def call(self, cmd: str, args: dict | None = None) -> dict:
        if self._sock is None:
            raise RuntimeError("not connected")
        req_id = self._next_id
        self._next_id += 1
        payload = json.dumps({
            "id": req_id, "cmd": cmd, "args": args or {}
        }).encode("utf-8")
        self._sock.sendall(struct.pack(">I", len(payload)) + payload)
        header = b""
        while len(header) < 4:
            chunk = self._sock.recv(4 - len(header))
            if not chunk:
                raise RuntimeError(f"supervisor closed mid-header on {cmd}")
            header += chunk
        (length,) = struct.unpack(">I", header)
        body = b""
        while len(body) < length:
            chunk = self._sock.recv(length - len(body))
            if not chunk:
                raise RuntimeError(f"supervisor closed mid-body on {cmd}")
            body += chunk
        resp = json.loads(body.decode("utf-8"))
        if not resp.get("ok"):
            raise RuntimeError(f"{cmd} failed: {resp.get('error')}")
        return resp.get("result") or {}


# ---------------------------------------------------------------------------
# OmniSim subprocess management
# ---------------------------------------------------------------------------


class HeadlessWebots:
    """Owns a headless omnisim-bin subprocess pointed at a given world.
    Use as a context manager so cleanup is guaranteed even on errors.
    """

    def __init__(self, world: Path, log_path: Path | None = None):
        self.world = world
        self.log_path = log_path or (REPO_ROOT / "tmp_headless.log")
        self.proc: subprocess.Popen | None = None
        self._log_file = None

    def __enter__(self):
        self._log_file = open(self.log_path, "w", encoding="utf-8", errors="replace")
        binary = _pick_webots_binary()
        cmd = [
            str(binary),
            "--no-rendering",
            "--batch",
            "--mode=fast",
            "--minimize",
            str(self.world),
        ]
        env = _ensure_path()
        self.proc = subprocess.Popen(
            cmd, stdout=self._log_file, stderr=subprocess.STDOUT,
            env=env, cwd=str(REPO_ROOT),
        )
        return self

    def __exit__(self, *_exc):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=3)
            except Exception:
                pass
            self.proc = None
        if self._log_file is not None:
            self._log_file.close()


# ---------------------------------------------------------------------------
# Scenario runners — one per built-in scenario name
# ---------------------------------------------------------------------------


def _format_geom_stats(stats: dict) -> str:
    parts = []
    for part, s in sorted(stats.items()):
        if not isinstance(s, dict):
            continue
        parts.append(
            f"  {part:<14} verts={s['vertex_count']} "
            f"displaced={s['displaced_count']} "
            f"max={100*s['max_displacement_m']:.2f}cm "
            f"rms={100*s['rms_displacement_m']:.2f}cm "
            f"crumple={s['current_crumple']:.3f}"
        )
    return "\n".join(parts) if parts else "  (no parts with vertex buffer)"


def scenario_idle_warmup(c: SupervisorClient) -> dict:
    """Sanity check: connect, ping, read state, query geom stats. No
    impacts. Verifies headless OmniSim + supervisor are alive.
    """
    c.call("ping")
    sim = c.call("sim_state")
    state = c.call("damage_state")
    stats = c.call("damage_geometry_stats")
    print(f"sim_time = {sim['sim_time_ms']:.0f}ms")
    print(f"chassis: {state['damage']['chassis']['state']} "
          f"hp={state['damage']['chassis']['hp']:.1f}")
    print("geometry:")
    print(_format_geom_stats(stats))
    return {"ok": True, "stats": stats, "state": state}


def scenario_broken_inject(c: SupervisorClient) -> dict:
    """Inject chassis: broken via the test/debug hook, then read geom
    stats. Validates the Phase 14b additive-crumple path triggers
    correctly.
    """
    c.call("damage_reset")
    c.call("damage_inject", {"part": "chassis", "state": "broken"})
    # One sim step lets the supervisor process the transition
    c.call("step", {"steps": 1})
    state = c.call("damage_state")
    stats = c.call("damage_geometry_stats")
    print(f"chassis: {state['damage']['chassis']['state']}")
    print("geometry after broken inject:")
    print(_format_geom_stats(stats))
    return {"ok": True, "stats": stats, "state": state}


def scenario_box_drop_run(c: SupervisorClient, sim_seconds: float = 30.0) -> dict:
    """Run the standard box-drop schedule for `sim_seconds`, reading
    stats throughout. Most realistic scenario for Phase 15 deformation.
    """
    c.call("damage_reset")
    sim_step_ms = c.call("sim_state")["basic_time_step_ms"]
    target_steps = int((sim_seconds * 1000) / sim_step_ms)
    print(f"running {sim_seconds:.0f}s ({target_steps} steps)...")
    # Step in chunks so we can poll metrics mid-run
    chunk = max(1, target_steps // 6)
    for i in range(0, target_steps, chunk):
        c.call("step", {"steps": chunk})
        sim = c.call("sim_state")
        state = c.call("damage_state")
        chassis = state["damage"]["chassis"]
        print(f"  t={sim['sim_time_ms']:>6.0f}ms  chassis={chassis['state']:<10} "
              f"hp={chassis['hp']:>5.1f}/{chassis['hp_max']:.0f}  "
              f"events={state['events_total']}")
    final_state = c.call("damage_state")
    final_stats = c.call("damage_geometry_stats")
    print()
    print("final geometry:")
    print(_format_geom_stats(final_stats))
    return {"ok": True, "stats": final_stats, "state": final_state}


def scenario_reset_round_trip(c: SupervisorClient) -> dict:
    """Damage the chassis, reset, verify vertex buffer is back to
    baseline (zero displacement)."""
    c.call("damage_reset")
    c.call("damage_inject", {"part": "chassis", "state": "broken"})
    c.call("step", {"steps": 1})
    pre = c.call("damage_geometry_stats")
    c.call("damage_reset")
    c.call("step", {"steps": 1})
    post = c.call("damage_geometry_stats")
    print("pre-reset:")
    print(_format_geom_stats(pre))
    print("post-reset:")
    print(_format_geom_stats(post))
    return {"ok": True, "pre": pre, "post": post}


SCENARIOS = {
    "idle_warmup":      scenario_idle_warmup,
    "broken_inject":    scenario_broken_inject,
    "box_drop_run":     scenario_box_drop_run,
    "reset_round_trip": scenario_reset_round_trip,
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def run_scenario(name: str, world: Path = DEFAULT_WORLD,
                 connect_timeout_s: float = 30.0) -> dict:
    """Spawn headless OmniSim, connect, run scenario by name, return
    result dict. Handles cleanup. Raises if scenario name unknown.
    """
    if name not in SCENARIOS:
        raise SystemExit(f"unknown scenario {name!r}; known: {sorted(SCENARIOS)}")
    fn = SCENARIOS[name]
    with HeadlessWebots(world):
        client = SupervisorClient(connect_timeout_s=connect_timeout_s)
        client.connect()
        try:
            return fn(client)
        finally:
            client.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim damage",
        description="Headless damage-system test harness",
    )
    parser.add_argument("--scenario", default="box_drop_run",
                        help=f"one of: {', '.join(sorted(SCENARIOS))}")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD,
                        help="world file to load")
    parser.add_argument("--list", action="store_true",
                        help="list available scenarios and exit")
    args = parser.parse_args(argv)

    if args.list:
        for name in sorted(SCENARIOS):
            print(name)
        return 0

    print(f"=== scenario: {args.scenario} ===")
    print(f"=== world:    {args.world}")
    t0 = time.monotonic()
    result = run_scenario(args.scenario, world=args.world)
    elapsed = time.monotonic() - t0
    print()
    print(f"=== done in {elapsed:.1f}s; ok={result.get('ok')} ===")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
