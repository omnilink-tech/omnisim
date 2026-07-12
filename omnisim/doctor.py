"""omnisim doctor — print ground-truth repo / runtime state for agents.

A new agent session (Claude Code, Codex, Cursor, Aider, …) that lands in
an OmniSim clone runs this on its first turn to know what is actually
true about this clone right now, instead of guessing from documentation.

AGENTS.md §3 (cold-start bootstrap) directs agents here.
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
from pathlib import Path

from . import __version__
from .paths import DEMOS_WORLDS, REPO_ROOT, resolve_webots_binary


HARNESS_PORT = 6789
SUPERVISOR_PORT = 6790
CAPTURE_PORT = 6791


def _git(*args: str) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", *args], cwd=REPO_ROOT, stderr=subprocess.DEVNULL
        )
        return out.decode("utf-8", errors="replace").strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def _port_status(port: int, host: str = "127.0.0.1") -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.2)
    try:
        return "in-use" if sock.connect_ex((host, port)) == 0 else "free"
    finally:
        sock.close()


def _worlds() -> list[str]:
    if not DEMOS_WORLDS.is_dir():
        return []
    # Recursive: most demos live in subdirs (showcase/, flagship/, chat/,
    # ...). A top-level-only glob reported 2 of 73 worlds and made fresh
    # clones look empty.
    return sorted(
        p.relative_to(DEMOS_WORLDS).as_posix() for p in DEMOS_WORLDS.rglob("*.wbt")
    )


def run(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim doctor",
        description="Report ground-truth repo and runtime state.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of text.")
    parser.add_argument(
        "--fingerprint",
        action="store_true",
        help="Also capture the install config fingerprint (resolved physics "
        "backend, Newton-runtime presence, GPU, fingerprint_id).",
    )
    args = parser.parse_args(argv)

    fingerprint = None
    if args.fingerprint:
        try:
            from .conformance import collect_fingerprint
            fingerprint = collect_fingerprint(scrub_paths=True)
        except Exception as exc:  # never let the fingerprint break doctor
            fingerprint = {"error": f"fingerprint failed: {exc}"}

    branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "?"
    commit = _git("rev-parse", "--short", "HEAD") or "?"
    recent = _git("log", "-3", "--format=%h %s") or ""
    webots = resolve_webots_binary()
    worlds = _worlds()

    ports = {
        "harness": _port_status(HARNESS_PORT),
        "supervisor": _port_status(SUPERVISOR_PORT),
        "capture": _port_status(CAPTURE_PORT),
    }

    if args.json:
        print(
            json.dumps(
                {
                    "omnisim_version": __version__,
                    "git_branch": branch,
                    "git_commit": commit,
                    "omnisim_binary": webots,
                    "webots_binary": webots,  # legacy alias for tools that still read the old key
                    "ports": ports,
                    "worlds_dir": str(DEMOS_WORLDS),
                    "worlds": worlds,
                    "recent_commits": recent.splitlines(),
                    "fingerprint": fingerprint,
                },
                indent=2,
            )
        )
        return 0

    print(f"omnisim     {__version__}")
    print(f"git         {branch} @ {commit}")
    print(f"binary      {webots or 'NOT FOUND  (set OMNISIM_HOME or build the simulator)'}")
    print("ports")
    print(f"  6789 harness     {ports['harness']}")
    print(f"  6790 supervisor  {ports['supervisor']}")
    print(f"  6791 capture     {ports['capture']}")
    rel_worlds = DEMOS_WORLDS.relative_to(REPO_ROOT) if DEMOS_WORLDS.exists() else Path("(missing)")
    print(f"worlds      {len(worlds)} in {rel_worlds}")
    if worlds:
        head = ", ".join(worlds[:5])
        more = f" (+{len(worlds) - 5} more)" if len(worlds) > 5 else ""
        print(f"            {head}{more}")
    if recent:
        print("recent")
        for line in recent.splitlines():
            print(f"  {line}")
    if fingerprint is not None:
        from .conformance.report import format_fingerprint
        print(format_fingerprint(fingerprint))
    return 0
