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

"""omnisim-runner — one-command launcher for an OmniSim ↔ OmniLink agent.

Boots the right OmniSim world + waits for its supervisor bridge + starts
the OmniLink agent runner, in the right order. The contract is the
"runCommand" the OmniLink platform's project page can list:

    python scripts/dev/omnisim_run_agent.py --agent husky_maze

What it does
------------

1. Look up the agent in the embedded registry to find:
   * the ``.wbt`` world file
   * the bridge HTTP port the world's supervisor controller exposes
   * the agent runner script and its tool-callback port
2. Launch OmniSim with that world (windowed by default; ``--headless``
   for ``--mode=fast --no-rendering``).
3. Poll the bridge port until it returns ``200`` from ``/state`` or
   ``/healthz``, then move on.
4. Launch the agent runner in a child process with ``OMNI_KEY``
   forwarded from the parent env.
5. Wait for either child to exit, forward Ctrl-C / SIGINT to both,
   and clean up on exit.

Why this script
---------------

Before this, running an OmniSim agent took two terminals and a
~6-step sequence ("launch OmniSim, wait, set OMNI_KEY, run the agent
script in another shell"). The OmniLink platform's project showcase
needs a one-line ``runCommand`` it can put in front of users — that
turned out to be hard to write because nobody had codified the boot
order. This script is that codification.

It is a launcher, not a manager: if you need to restart the bridge
without rebooting OmniSim, use the bridge's ``/admin/reload`` endpoint
(see ``husky_omnilink_bridge.py`` for how that's wired). If you need
to drive the agent without OmniLink, run the agent's ``solve.py``
script directly.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[2]
AGENTS_DIR = REPO_ROOT / "agents" / "production"


# ---------------------------------------------------------------------------
# Agent registry — discovered from agents/production/<name>/omnilink.json
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AgentSpec:
    """Static metadata for one OmniSim ↔ OmniLink agent."""

    name: str
    world: Optional[str]  # path relative to repo root; None = orchestrator
    bridge_port: Optional[int]  # supervisor HTTP bridge port
    agent_script: Optional[str]  # path to <agent>_agent.py
    agent_port: Optional[int]  # tool-callback server port
    description: str
    agent_port_env: Optional[str] = None
    bridge_url_env: Optional[str] = None
    omnilink_agent_name: Optional[str] = None
    tags: tuple = ()


def _coerce_int_or_none(value) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _load_manifest(path: Path) -> Optional[AgentSpec]:
    """Read one omnilink.json. Returns None on invalid shape."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"  [warn] skipping {path}: {exc}")
        return None
    if not isinstance(data, dict):
        return None
    name = data.get("name") or path.parent.name
    return AgentSpec(
        name=name,
        world=data.get("world"),
        bridge_port=_coerce_int_or_none(data.get("bridge_port")),
        agent_script=data.get("agent_script"),
        agent_port=_coerce_int_or_none(data.get("agent_port")),
        description=data.get("description", ""),
        agent_port_env=data.get("agent_port_env"),
        bridge_url_env=data.get("bridge_url_env"),
        omnilink_agent_name=data.get("omnilink_agent_name") or data.get("display_name"),
        tags=tuple(data.get("tags") or ()),
    )


def discover_agents(agents_dir: Path = AGENTS_DIR) -> Dict[str, AgentSpec]:
    """Walk agents/production/<name>/omnilink.json into the registry."""
    out: Dict[str, AgentSpec] = {}
    if not agents_dir.exists():
        return out
    for manifest in sorted(agents_dir.glob("*/omnilink.json")):
        spec = _load_manifest(manifest)
        if spec is None:
            continue
        if spec.name in out:
            print(f"  [warn] duplicate agent name {spec.name!r}; keeping first.")
            continue
        out[spec.name] = spec
    return out


AGENTS: Dict[str, AgentSpec] = discover_agents()


# ---------------------------------------------------------------------------
# OmniSim binary discovery
# ---------------------------------------------------------------------------


def find_omnisim_binary() -> Path:
    """Locate ``omnisim-bin`` for this platform, preferring the bundled msys2."""
    sys_name = platform.system()
    if sys_name == "Windows":
        candidate = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
        if candidate.exists():
            return candidate
        raise FileNotFoundError(
            f"omnisim-bin.exe not found at {candidate}. Build with build_omni.bat first."
        )
    if sys_name == "Darwin":
        for c in (
            REPO_ROOT / "Contents" / "MacOS" / "omnisim",
            REPO_ROOT / "bin" / "omnisim-bin",
            REPO_ROOT / "Contents" / "MacOS" / "webots",
        ):
            if c.exists():
                return c
    # Linux + fallback
    for c in (REPO_ROOT / "bin" / "omnisim-bin", REPO_ROOT / "webots"):
        if c.exists():
            return c
    raise FileNotFoundError(
        f"omnisim-bin not found under {REPO_ROOT}. Run `python -m omnisim build all`."
    )


def omnisim_env() -> dict:
    """Build the env dict OmniSim expects, with msys2 toolchain on Windows."""
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO_ROOT)
    if platform.system() == "Windows":
        msys_bin = REPO_ROOT / "msys64" / "mingw64" / "bin"
        if msys_bin.exists():
            env["PATH"] = f"{msys_bin}{os.pathsep}{env.get('PATH', '')}"
        bundled_python = msys_bin / "newton-runtime" / "python.exe"
        if bundled_python.exists():
            env["PATH"] = f"{bundled_python.parent}{os.pathsep}{env['PATH']}"
    return env


# ---------------------------------------------------------------------------
# Bridge readiness probe
# ---------------------------------------------------------------------------


def probe_bridge(port: int, timeout_s: float, label: str = "bridge") -> bool:
    """Block until the bridge HTTP port answers on ``/state`` or ``/healthz``.

    Tries ``/healthz`` first (the SDK default) and falls back to
    ``/state`` (which every existing bridge already supports).
    Returns ``True`` on first 200; ``False`` on timeout.
    """
    deadline = time.time() + timeout_s
    last_err: Optional[str] = None
    paths = ("/healthz", "/state")
    while time.time() < deadline:
        for path in paths:
            url = f"http://127.0.0.1:{port}{path}"
            try:
                with urllib.request.urlopen(url, timeout=1.5) as r:
                    if 200 <= r.status < 300:
                        return True
            except urllib.error.HTTPError as exc:
                # 404 on /healthz is fine (older bridges); keep probing.
                if exc.code == 404 and path == "/healthz":
                    continue
                last_err = f"{exc.code} {exc.reason}"
            except Exception as exc:  # pragma: no cover - connection refused, etc.
                last_err = f"{exc.__class__.__name__}: {exc}"
        time.sleep(0.5)
    print(f"  [WARN] {label} on :{port} did not become ready within {timeout_s:.0f}s")
    if last_err:
        print(f"         last error: {last_err}")
    return False


# ---------------------------------------------------------------------------
# Process group / signal handling
# ---------------------------------------------------------------------------


def _is_windows() -> bool:
    return platform.system() == "Windows"


def _spawn(args: List[str], *, env: Optional[dict] = None) -> subprocess.Popen:
    """Spawn a child process in its own group so SIGINT can target the tree.

    On Windows we use ``CREATE_NEW_PROCESS_GROUP`` so we can later send
    ``CTRL_BREAK_EVENT``. On POSIX we ``setsid`` so a single
    ``killpg`` reaches every descendant.
    """
    kwargs: dict = {"env": env or os.environ.copy()}
    if _is_windows():
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
    else:
        kwargs["start_new_session"] = True
    return subprocess.Popen(args, **kwargs)


def _terminate(proc: Optional[subprocess.Popen], label: str, timeout: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"  [stop] {label} (pid={proc.pid})")
    try:
        if _is_windows():
            proc.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        else:
            os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"  [stop] {label} did not exit; killing")
        proc.kill()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass


# ---------------------------------------------------------------------------
# Main launch flow
# ---------------------------------------------------------------------------


def world_command(spec: AgentSpec, *, headless: bool) -> List[str]:
    """The exact ``omnisim-bin`` argv for this agent's world.

    Shared by :func:`launch_world` and the ``--keep-agent`` relaunch hint
    (and mirrored by agent-side relaunch endpoints, e.g. the HuskySwarm
    coordinator's ``POST /relaunch-sim``) so every path launches the sim
    the same way.
    """
    if spec.world is None:
        raise ValueError(f"{spec.name} has no world (orchestrator-only)")
    binary = find_omnisim_binary()
    world_path = REPO_ROOT / spec.world
    if not world_path.exists():
        raise FileNotFoundError(f"world file not found: {world_path}")
    args = [str(binary)]
    if headless:
        args += ["--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr"]
    args.append(str(world_path))
    return args


def launch_world(spec: AgentSpec, *, headless: bool, duration: Optional[int]) -> Optional[subprocess.Popen]:
    """Start OmniSim with the agent's world. Returns the child process."""
    if spec.world is None:
        print(f"  [skip] {spec.name} has no world (orchestrator-only).")
        return None
    args = world_command(spec, headless=headless)
    print(f"  [start] sim: {' '.join(args)}")
    proc = _spawn(args, env=omnisim_env())
    if duration:
        # Caller will time-out on its own. OmniSim itself doesn't have a
        # built-in --duration flag we want to rely on across modes.
        pass
    return proc


def launch_agent(spec: AgentSpec, *, port_override: Optional[int]) -> Optional[subprocess.Popen]:
    """Start the agent runner with OMNI_KEY forwarded."""
    if spec.agent_script is None:
        print(f"  [skip] {spec.name} has no agent runner script.")
        return None
    script = REPO_ROOT / spec.agent_script
    if not script.exists():
        raise FileNotFoundError(f"agent script not found: {script}")
    if not os.environ.get("OMNI_KEY", "").strip():
        print()
        print("  [error] OMNI_KEY is not set. The agent runner needs an Omni Key.")
        print('  Set it and re-run:')
        print('    Windows: set OMNI_KEY=olink_YOUR_KEY_HERE')
        print('    bash:    export OMNI_KEY="olink_YOUR_KEY_HERE"')
        print('  Get one at https://www.omnilink-agents.com/key')
        print('  or run:   python -m omnisim key')
        return None

    env = os.environ.copy()
    if port_override is not None and spec.agent_port is not None:
        # Map override to the agent's own per-agent env var so the
        # script's existing parsing keeps working unchanged.
        if spec.agent_port_env:
            env[spec.agent_port_env] = str(port_override)

    args = [sys.executable, str(script)]
    print(f"  [start] agent:  {' '.join(args)}")
    return _spawn(args, env=env)


# ---------------------------------------------------------------------------
# Per-husky unit agents (husky_swarm team)
# ---------------------------------------------------------------------------

UNIT_TEAM_BASE_PORT = 51521
UNIT_AGENT_SCRIPT = AGENTS_DIR / "husky_swarm" / "husky_unit_agent.py"


def _husky_ids_from_env() -> List[str]:
    """Husky ids the swarm agent will resolve — same env contract as
    swarm_agent._parse_bridges_env (HUSKY_SWARM_BRIDGES), defaults to the
    four quadrant huskies of omnilink_husky_swarm.omniworld."""
    spec = os.environ.get("HUSKY_SWARM_BRIDGES", "").strip()
    if not spec:
        return ["ne", "nw", "se", "sw"]
    ids: List[str] = []
    for piece in spec.split(","):
        piece = piece.strip()
        if not piece or "=" not in piece:
            continue
        key = piece.split("=", 1)[0].strip()
        ids.append(key[len("husky_"):] if key.startswith("husky_") else key)
    return ids


def launch_unit_agents() -> List[tuple]:
    """Spawn one husky_unit_agent.py per husky (ports 51521..), with the
    parent env passed through (OMNI_KEY, OMNISIM_ALLOWED_ORIGINS, bridge
    overrides). Returns [(label, Popen), ...]. Each unit agent prefixes its
    own log lines with [HuskyNE] etc., so interleaved output stays legible."""
    if not UNIT_AGENT_SCRIPT.exists():
        print(f"  [warn] unit agent script missing: {UNIT_AGENT_SCRIPT}; skipping team.")
        return []
    procs: List[tuple] = []
    for i, hid in enumerate(_husky_ids_from_env()):
        port = UNIT_TEAM_BASE_PORT + i
        label = "Husky" + hid.upper()
        args = [sys.executable, str(UNIT_AGENT_SCRIPT),
                "--husky", hid, "--port", str(port)]
        print(f"  [start] unit:   {label} on :{port}")
        procs.append((label, _spawn(args, env=os.environ.copy())))
    return procs


def _load_usage_record(agent_name: str, headless: bool, duration_capped):
    """Best-effort UsageRecord. Falls back to None if `_lib` isn't importable."""
    try:
        sys.path.insert(0, str(REPO_ROOT / "agents" / "production"))
        from _lib import UsageRecord  # type: ignore[import-not-found]
    except Exception:
        return None
    return UsageRecord(
        agent=agent_name,
        headless=headless,
        duration_capped=duration_capped,
    )


def run(spec: AgentSpec, args: argparse.Namespace) -> int:
    """Drive the full launch sequence and wait for child exits."""
    print()
    print("=" * 64)
    print(f"  omnisim-runner — {spec.name}")
    print("=" * 64)
    print(f"  Description:  {spec.description}")
    if spec.world:
        print(f"  World:        {spec.world}")
    if spec.bridge_port:
        print(f"  Bridge port:  {spec.bridge_port}")
    if spec.agent_script:
        print(f"  Agent:        {spec.agent_script}")
    if spec.agent_port:
        print(f"  Agent port:   {args.port or spec.agent_port}")
    print()

    agent_requested = not args.no_agent and spec.agent_script is not None
    omni_key_present = bool(os.environ.get("OMNI_KEY", "").strip())
    if agent_requested and not omni_key_present and not args.allow_missing_omni_key:
        print("  [error] OMNI_KEY is not set, so the requested OmniLink agent cannot start.")
        print("          Set OMNI_KEY, use --no-agent, or explicitly use --allow-missing-omni-key.")
        return 4

    world_proc: Optional[subprocess.Popen] = None
    agent_proc: Optional[subprocess.Popen] = None
    team_procs: List[tuple] = []
    exit_code = 0

    usage_record = None
    if agent_requested and omni_key_present:
        usage_record = _load_usage_record(
            spec.name, headless=args.headless, duration_capped=args.duration
        )
    if usage_record is not None:
        usage_record.start()

    try:
        if not args.no_world:
            world_proc = launch_world(
                spec, headless=args.headless, duration=args.duration
            )
            if world_proc is None and spec.world is not None:
                exit_code = 2
                return exit_code

        if spec.bridge_port and not args.no_world:
            print(f"  [wait]  bridge ready on :{spec.bridge_port} (timeout {args.bridge_timeout}s)")
            ok = probe_bridge(spec.bridge_port, args.bridge_timeout, label=f"{spec.name} bridge")
            if not ok and not args.continue_on_bridge_timeout:
                print("  [error] bridge never came up; aborting.")
                exit_code = 3
                return exit_code

        if not args.no_agent and (omni_key_present or spec.agent_script is None):
            agent_proc = launch_agent(spec, port_override=args.port)
            # husky_swarm is a Pro agent commanding a team: also spawn the
            # four per-husky unit agents unless --no-team. They share the
            # swarm agent's lifecycle (kept alive under keep-agent when the
            # sim dies; terminated with the agent on shutdown).
            if (spec.name == "husky_swarm" and agent_proc is not None
                    and not args.no_team):
                team_procs = launch_unit_agents()
        elif agent_requested and not omni_key_present:
            print("  [info]  OMNI_KEY is missing; running OmniSim without the agent by explicit request.")

        if world_proc is None and agent_proc is None:
            return exit_code

        # keep-agent: when the sim dies the agent stays up (it truthfully
        # reports its bridges unreachable and keeps its memory, presence
        # heartbeat, and tool server). Defaults ON for --headless runs;
        # --no-keep-agent restores the old both-die-together behaviour.
        keep_agent = args.keep_agent if args.keep_agent is not None else args.headless

        # Block waiting for any child to exit, or for a duration cap.
        deadline = time.time() + args.duration if args.duration else None
        while True:
            if deadline and time.time() > deadline:
                print(f"  [timer] {args.duration}s elapsed; shutting down.")
                break
            if world_proc is not None and world_proc.poll() is not None:
                rc = world_proc.returncode
                if keep_agent and agent_proc is not None and agent_proc.poll() is None:
                    agent_port = args.port or spec.agent_port or "?"
                    try:
                        relaunch = " ".join(world_command(spec, headless=args.headless))
                    except (ValueError, FileNotFoundError):
                        relaunch = "(world command unavailable)"
                    print(f"  [keep-agent] sim exited (rc={rc}); agent still serving "
                          f"on :{agent_port} — relaunch the world with: {relaunch}")
                    world_proc = None  # supervise the agent only from here on
                    continue
                print(f"  [exit]  sim exited (rc={rc}); shutting down.")
                if rc:
                    exit_code = rc
                break
            if agent_proc is not None and agent_proc.poll() is not None:
                rc = agent_proc.returncode
                print(f"  [exit]  agent exited (rc={rc}); shutting down.")
                if rc:
                    exit_code = rc
                break
            # A dead unit agent degrades the team but does not end the run:
            # log it once and keep supervising the rest.
            for label, proc in list(team_procs):
                if proc.poll() is not None:
                    print(f"  [warn]  unit agent {label} exited "
                          f"(rc={proc.returncode}); continuing without it.")
                    team_procs.remove((label, proc))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  [stop]  Ctrl+C")
    finally:
        for label, proc in team_procs:
            _terminate(proc, label=f"unit {label}")
        _terminate(agent_proc, label="agent")
        _terminate(world_proc, label="sim")
        if usage_record is not None:
            out_path = usage_record.finish(exit_code=exit_code)
            if out_path is not None:
                try:
                    rel = out_path.relative_to(REPO_ROOT)
                    print(f"  [usage] snapshot written to {rel}")
                except ValueError:
                    print(f"  [usage] snapshot written to {out_path}")

    return exit_code


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omnisim-runner",
        description="Launch an OmniSim world + its OmniLink agent runner together.",
    )
    parser.add_argument(
        "--agent",
        choices=sorted(AGENTS.keys()) or None,
        help="Which agent to launch (see --list).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List known agents and exit.",
    )
    parser.add_argument(
        "--list-json",
        action="store_true",
        help="Print the agent registry as JSON and exit (used by tooling).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run OmniSim without a window (--batch --mode=fast --no-rendering).",
    )
    parser.add_argument(
        "--no-world",
        action="store_true",
        help="Skip launching OmniSim - assume the world is already running.",
    )
    parser.add_argument(
        "--no-agent",
        action="store_true",
        help="Skip launching the agent runner - just open the world + bridge.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Override the agent's tool-callback port.",
    )
    parser.add_argument(
        "--no-team",
        action="store_true",
        help="For husky_swarm: skip the four per-husky unit agents "
        "(HuskyNE..HuskySW on ports 51521-51524) and run the coordinator alone.",
    )
    parser.add_argument(
        "--keep-agent",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Keep the agent alive when the sim exits (it reports its bridges "
        "unreachable and keeps its memory + heartbeat; relaunch the world to "
        "reconnect). Default: ON for --headless runs, OFF for windowed runs. "
        "--no-keep-agent restores shut-down-together.",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=None,
        help="Stop both children after N seconds (smoke-test mode).",
    )
    parser.add_argument(
        "--bridge-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the bridge HTTP port to come up (default 60).",
    )
    parser.add_argument(
        "--continue-on-bridge-timeout",
        action="store_true",
        help="Don't abort if the bridge doesn't respond - start the agent anyway.",
    )
    parser.add_argument(
        "--allow-missing-omni-key",
        action="store_true",
        help="Don't return non-zero if OMNI_KEY is unset (--no-agent runs).",
    )
    return parser


def cmd_list() -> int:
    print("Known agents:")
    print()
    if not AGENTS:
        print(f"  (none found — expected agents/production/<name>/omnilink.json under {AGENTS_DIR.parent})")
        return 1
    name_w = max(len(n) for n in AGENTS)
    for name in sorted(AGENTS):
        spec = AGENTS[name]
        print(f"  {name.ljust(name_w)}  {spec.description}")
    print()
    print("Run with: python scripts/dev/omnisim_run_agent.py --agent <name>")
    return 0


def cmd_list_json() -> int:
    payload = {
        name: {
            "world": spec.world,
            "bridge_port": spec.bridge_port,
            "agent_script": spec.agent_script,
            "agent_port": spec.agent_port,
            "agent_port_env": spec.agent_port_env,
            "bridge_url_env": spec.bridge_url_env,
            "omnilink_agent_name": spec.omnilink_agent_name,
            "tags": list(spec.tags),
            "description": spec.description,
        }
        for name, spec in AGENTS.items()
    }
    print(json.dumps(payload, indent=2))
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        return cmd_list()
    if args.list_json:
        return cmd_list_json()
    if not args.agent:
        build_parser().print_help()
        return 1
    spec = AGENTS[args.agent]
    return run(spec, args)


if __name__ == "__main__":
    sys.exit(main())
