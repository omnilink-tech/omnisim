"""Drive ``POST /world/load`` to hot-reload a running OmniSim session.

The harness (started via ``python -m omnisim harness``) already exposes
``/world/load``. This subcommand is a thin convenience wrapper plus an
optional file-watcher: when PROTOs (or their Python sources) change, the
running world is re-loaded so the new defaults take effect without
killing ``omnisim-bin``.

This is the pragmatic, reuses-what's-already-there flavor of hot reload
called out in the design doc — no C++ changes, no in-place PROTO swap.

Run with ``python -m omnisim proto reload --world <path>`` for a one-shot
reload, or add ``--watch`` to keep watching.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Iterable
from urllib import error as urlerror
from urllib import request as urlrequest

from ..paths import REPO_ROOT


_DEFAULT_TIMEOUT = 30.0


def _post_world_load(host: str, port: int, world: Path) -> tuple[int, str]:
    payload = json.dumps({"path": str(world), "wait_s": 3.0}).encode("utf-8")
    req = urlrequest.Request(
        f"http://{host}:{port}/world/load",
        data=payload,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urlrequest.urlopen(req, timeout=_DEFAULT_TIMEOUT) as resp:
            return resp.status, resp.read().decode("utf-8", "replace")
    except urlerror.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")
    except urlerror.URLError as e:
        return 0, f"harness unreachable: {e.reason}"
    except TimeoutError:
        return 0, "harness timeout"


def _iter_watched(paths: Iterable[Path]) -> dict[Path, float]:
    out: dict[Path, float] = {}
    for p in paths:
        if p.is_dir():
            for f in p.rglob("*.proto"):
                out[f] = f.stat().st_mtime
            for f in p.rglob("*.proto.py"):
                out[f] = f.stat().st_mtime
        elif p.exists():
            out[p] = p.stat().st_mtime
    return out


def run(args: argparse.Namespace) -> int:
    if not args.world:
        print("--world is required (path to the .wbt to (re)load)", file=sys.stderr)
        return 2
    world = Path(args.world)
    if not world.is_absolute():
        world = REPO_ROOT / world
    if not world.exists():
        print(f"world not found: {world}", file=sys.stderr)
        return 1

    def fire(reason: str) -> int:
        status, body = _post_world_load(args.host, args.port, world)
        if status == 200:
            print(f"reload OK ({reason}): {world}")
            return 0
        print(f"reload FAILED ({reason}) status={status}: {body}", file=sys.stderr)
        return 1

    rc = fire("manual")
    if not args.watch:
        return rc

    # Watch mode: snapshot mtimes, poll, fire on any delta.
    watch_roots = [REPO_ROOT / "projects"]
    snapshot = _iter_watched(watch_roots)
    print(f"watching {len(snapshot)} files under projects/... (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(args.interval)
            current = _iter_watched(watch_roots)
            changed = [p for p, mt in current.items() if snapshot.get(p) != mt]
            if changed:
                snapshot = current
                names = ", ".join(str(p.relative_to(REPO_ROOT)) for p in changed[:3])
                if len(changed) > 3:
                    names += f", +{len(changed) - 3} more"
                fire(f"change: {names}")
    except KeyboardInterrupt:
        print()
        return 0
