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

"""Re-sync `g1_deploy_runtime.py` with OmniSim's canonical Newton runtime.

WHY THIS EXISTS. `projects/policies/research/backends/g1_deploy_runtime.py` is a
byte-verbatim mirror of
`src/omnisim/physics/omnisim_newton_runtime.py`. The G1 golden-parity test builds the
LITERAL deploy model from the mirror, so a stale mirror means parity is measured
against physics the engine no longer runs.
`tests/test_g1_deploy_runtime_sync.py` guards it.

Every edit to the C++ block re-drifts the mirror. It drifted twice in two days,
and the only recorded remedy was "regenerate the extract" with no command that
does it -- so the guard was on its way to becoming background noise, which is how
a single-source-of-truth check dies.

WHY IT IS A SEPARATE SCRIPT. The obvious home is a `--sync` flag on the mirror
itself, and that cannot work: the mirror copies the C++ block *including* the
seam between its two adjacent raw literals, so the file contains a bare `)PY"`
and is not importable or runnable Python at all. Only the sync-guard tail below
the delimiter is real code, which the test reaches by slicing and `exec`-ing it.
So the marker logic is replicated here rather than imported.

USAGE
    python scripts/dev/sync_g1_deploy_runtime.py            # rewrite if drifted
    python scripts/dev/sync_g1_deploy_runtime.py --check    # report only, exit 1 on drift
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
RUNTIME = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
MIRROR = REPO / "projects" / "policies" / "research" / "backends" / "g1_deploy_runtime.py"

GUARD_MARK = "# " + "=" * 75 + "\n# Sync guard"


def runtime_block() -> str:
    """The canonical Python module imported by OmNewtonBackend."""
    return RUNTIME.read_text(encoding="utf-8")


def _slice(src: str) -> tuple[int, int]:
    """(start, end) of the mirrored body: after the docstring, before the guard."""
    first = src.index('"""')
    return src.index('"""', first + 3) + 3, src.index(GUARD_MARK)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="report drift without writing; exit 1 if drifted")
    args = ap.parse_args(argv)

    for p in (RUNTIME, MIRROR):
        if not p.is_file():
            raise SystemExit(f"missing: {p}")

    src = MIRROR.read_text(encoding="utf-8")          # universal newlines
    body_start, body_end = _slice(src)
    current = src[body_start:body_end]
    wanted = "\n" + runtime_block().rstrip("\n") + "\n\n\n"

    if current == wanted:
        print("g1_deploy_runtime.py IS IN SYNC with omnisim_newton_runtime.py.")
        return 0

    cl, ml = wanted.split("\n"), current.split("\n")
    first = next((k for k in range(min(len(cl), len(ml))) if cl[k] != ml[k]),
                 min(len(cl), len(ml)))
    print(f"DRIFTED: runtime body {len(cl)} lines, mirror {len(ml)} lines, "
          f"first difference at index {first}")

    if args.check:
        print("run without --check to rewrite the mirror.")
        return 1

    # The mirror is CRLF in-tree; preserve that.
    with open(MIRROR, "w", newline="\r\n", encoding="utf-8") as fh:
        fh.write(src[:body_start] + wanted + src[body_end:])
    print(f"RE-SYNCED {MIRROR.relative_to(REPO).as_posix()} "
          f"({len(ml)} -> {len(cl)} lines).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
