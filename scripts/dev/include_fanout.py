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
"""Report C/C++ header rebuild blast radius from generated dependency files.

This deliberately reads the build's existing ``.d`` files rather than trying
to infer dependencies from source text.  It therefore reports the exact
translation units that make will invalidate on this configured build.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEPENDENCY_ROOT = REPO_ROOT / "src" / "omnisim" / "build" / "release" / ".deps"


def normalized_dependencies(path: Path) -> set[str]:
    content = path.read_text(encoding="utf-8", errors="replace").replace("\\\n", " ")
    _, _, prerequisites = content.partition(":")
    # Make escapes spaces in paths. Preserve them while splitting the prerequisite
    # list, then normalize any remaining Windows-style path separators.
    escaped_space = "\0"
    prerequisites = prerequisites.replace("\\ ", escaped_space)
    return {
        item.replace(escaped_space, " ").replace("\\", "/")
        for item in prerequisites.split()
    }


def collect_fanout(dependency_root: Path) -> tuple[int, dict[str, int]]:
    dependency_files = sorted(dependency_root.glob("*.d"))
    consumers: dict[str, set[Path]] = defaultdict(set)
    for dependency_file in dependency_files:
        for dependency in normalized_dependencies(dependency_file):
            if dependency.endswith((".h", ".hh", ".hpp", ".hxx")):
                consumers[Path(dependency).name].add(dependency_file)
    return len(dependency_files), {name: len(files) for name, files in consumers.items()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dependency-root", type=Path, default=DEFAULT_DEPENDENCY_ROOT)
    parser.add_argument("--limit", type=int, default=25)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("headers", nargs="*", help="Optional header basenames to report")
    args = parser.parse_args()

    count, fanout = collect_fanout(args.dependency_root)
    if count == 0:
        parser.error(f"no .d files found under {args.dependency_root}; build OmniSim first")

    if args.headers:
        rows = sorted(((name, fanout.get(Path(name).name, 0)) for name in args.headers), key=lambda row: (-row[1], row[0]))
    else:
        rows = sorted(fanout.items(), key=lambda row: (-row[1], row[0]))[: args.limit]

    if args.json:
        print(json.dumps({"translation_units": count, "headers": dict(rows)}, indent=2))
    else:
        print(f"translation units with dependency files: {count}")
        for name, consumers in rows:
            print(f"{consumers:4d}  {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
