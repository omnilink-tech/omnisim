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

"""Migrate world/PROTO headers from the legacy `#VRML_SIM` to `#OMNISIM`.

The engine accepts both forms forever (OmTokenizer dual-accept); this
tool flips the first line of git-tracked `.wbt` / `.proto` files so the
corpus carries the canonical OmniSim identity. Only the header WORD
changes -- the version token and everything else in the file stay
byte-identical.

Deliberately excluded (upstream-Webots control arms and fixtures whose
whole point is to be upstream-shaped):

- tests/benchmarks/agentbench/**
- tests/benchmarks/ladder/**

Dry-run by default; pass --apply to write. Requires a dual-accept
engine binary (2026-08-08 or later) to be installed before applying,
since an older engine refuses `#OMNISIM` headers.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

LEGACY = "#VRML_SIM "
CANONICAL = "#OMNISIM "

EXCLUDED_PREFIXES = (
    "tests/benchmarks/agentbench/",
    "tests/benchmarks/ladder/",
)


def tracked_world_files(repo_root: Path) -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.wbt", "*.proto"],
        cwd=repo_root, capture_output=True, text=True, check=True,
    ).stdout.splitlines()
    return [
        repo_root / p for p in out
        if not p.startswith(EXCLUDED_PREFIXES)
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--apply", action="store_true",
                    help="write the flipped headers (default: dry-run report)")
    ap.add_argument("--root", default=None,
                    help="repo root (default: derived from this script's location)")
    args = ap.parse_args()

    repo_root = Path(args.root) if args.root else Path(__file__).resolve().parents[2]
    flipped, already, headerless = [], [], []

    for path in tracked_world_files(repo_root):
        try:
            raw = path.read_bytes()
        except OSError as e:
            print(f"SKIP (unreadable): {path} -- {e}", file=sys.stderr)
            continue
        # Only the first line is considered; preserve the file's own EOLs.
        nl = raw.find(b"\n")
        first = raw[: nl if nl >= 0 else len(raw)]
        text_first = first.decode("utf-8", errors="replace").lstrip("﻿")
        if text_first.startswith(CANONICAL.rstrip()):
            already.append(path)
        elif text_first.startswith(LEGACY.rstrip()):
            new_first = first.replace(b"#VRML_SIM ", b"#OMNISIM ", 1)
            if args.apply:
                path.write_bytes(new_first + raw[len(first):])
            flipped.append(path)
        else:
            headerless.append(path)

    mode = "APPLIED" if args.apply else "DRY-RUN (pass --apply to write)"
    print(f"{mode}: {len(flipped)} flipped, {len(already)} already canonical, "
          f"{len(headerless)} without a recognizable header")
    for p in headerless:
        print(f"  no header: {p.relative_to(repo_root)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
