#!/usr/bin/env python
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

"""Phase G of the Webots->OmniSim rebrand.

Rewrites every occurrence of the local URL scheme ``webots://`` to
``omnisim://`` inside ``.wbt`` and ``.proto`` files across the repo.
The engine accepts both schemes (see WbUrl.cpp ::normalizeLocalScheme),
so a half-migrated repo still loads; this script just normalises the
on-disk form so a future Phase&nbsp;I can finally drop the alias.

Usage:
    py -3 scripts/dev/migrate_url_scheme.py             # dry-run, prints what would change
    py -3 scripts/dev/migrate_url_scheme.py --apply     # actually rewrite files

The substitution is a byte-level ``webots://`` -> ``omnisim://`` and is
idempotent. Historical archives under ``docs/blog/upstream-webots-history``
and ``docs/reference/upstream-webots-history`` are skipped.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

EXCLUDE_DIR_PATHS = {
    ".git",
    "msys64",
    "Contents",
    "lib",
    "dependencies",
    "docs/blog/upstream-webots-history",
    "docs/reference/upstream-webots-history",
}

EXCLUDE_DIR_BASENAMES = {"build", "build_release", "build_debug", "__pycache__"}

# Only rewrite formats where webots:// is a URL-scheme reference. Python
# helpers and docs are left alone -- they refer to the scheme by name and
# may need to generate both forms during the transition. .proto.yaml is
# the auto-generated PROTO schema sidecar that mirrors EXTERNPROTO lines
# from the .proto file, so it must migrate in lock-step.
SUFFIX_MATCHES = (".wbt", ".proto", ".proto.yaml")

OLD = b"webots://"
NEW = b"omnisim://"


def _is_excluded_dir(p: Path) -> bool:
    if p.name in EXCLUDE_DIR_BASENAMES:
        return True
    rel = p.relative_to(REPO_ROOT).as_posix()
    for ex in EXCLUDE_DIR_PATHS:
        if rel == ex or rel.startswith(ex + "/"):
            return True
    return False


def _walk():
    for root, dirs, files in os.walk(REPO_ROOT):
        root_path = Path(root)
        dirs[:] = [d for d in dirs if not _is_excluded_dir(root_path / d) and not d.startswith(".")]
        for f in files:
            yield root_path / f


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--apply", action="store_true", help="rewrite files in place (default: dry-run)")
    p.add_argument("--quiet", action="store_true", help="suppress per-file lines")
    args = p.parse_args()

    total_files = 0
    total_replacements = 0
    changed_files: list[tuple[Path, int]] = []

    for path in _walk():
        name_lower = path.name.lower()
        if not any(name_lower.endswith(s) for s in SUFFIX_MATCHES):
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        if OLD not in data:
            continue
        total_files += 1
        count = data.count(OLD)
        total_replacements += count
        new_data = data.replace(OLD, NEW)
        changed_files.append((path, count))
        if args.apply:
            path.write_bytes(new_data)

    if not args.quiet:
        for path, count in changed_files:
            rel = path.relative_to(REPO_ROOT).as_posix()
            print(f"  {count:>4}  {rel}")

    verb = "rewrote" if args.apply else "would rewrite"
    print(f"{verb} {total_replacements} occurrence(s) across {total_files} file(s).")
    if not args.apply and total_files:
        print("(dry-run -- pass --apply to actually rewrite)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
