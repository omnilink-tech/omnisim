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
"""Migrate OmniSim's world files from `.wbt` to `.omniworld`.

POLICY: DUAL-READ, SINGLE-WRITE.

  * Everything that READS a world accepts BOTH `.wbt` and `.omniworld`, forever.
    A `.wbt` world must keep working: there are external users and forks.
  * Everything that WRITES or GENERATES a world emits `.omniworld`.
  * The tree's own worlds are renamed to `.omniworld`.

This script owns the mechanical half: the `git mv` of the worlds themselves and
the rewrite of every reference to them. The dual-read half (globs, extension
tests, file dialogs) is ordinary code and lives in the sources it belongs to.

Usage:
    python scripts/dev/migrate_world_extension.py --plan     # print, change nothing
    python scripts/dev/migrate_world_extension.py --apply
    python scripts/dev/migrate_world_extension.py --verify   # post-migration audit
"""

from __future__ import annotations

import argparse
import collections
import os
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

OLD_EXT = ".wbt"
NEW_EXT = ".omniworld"

# --------------------------------------------------------------------------
# What we deliberately do NOT rename.
# --------------------------------------------------------------------------

# Whole subtrees held back, each for a stated reason. These are not oversights:
# every one of them is a place where the FILENAME is load-bearing evidence, a
# frozen artifact, or a deliberate upstream-Webots control arm.
EXCLUDED_PREFIXES = {
    # Frozen distribution artifacts + their `<world>.wbt.seed.json` sidecars,
    # whose determinism claim is "same recipe+seed -> byte-identical file".
    "distribution/": "frozen generated-world artifacts (+ .wbt.seed.json sidecars)",
    # The measurement corpus. Path stability IS the evidence here:
    #  - agentbench/preregister/freeze_manifest.json pins SHA256 BY PATH and a
    #    red freeze test is a release gate;
    #  - ladder0/ and adapters/webots/ are upstream-Webots control arms that
    #    must keep loading in real Webots, which cannot read `.omniworld`;
    #  - omnibench results/*.jsonl carry world paths as measurement provenance.
    "tests/benchmarks/": "benchmark corpus: SHA256 freeze gate, Webots control arms, results provenance",
}

# Individual worlds kept on `.wbt` ON PURPOSE, as live proof that dual-read
# still works. If these ever stop loading, the compatibility promise is broken.
DUAL_READ_KEEPERS = {
    "tests/cache/worlds/backwards_compatibility.wbt":
        "named for this: the standing backward-compatibility world",
    "projects/samples/demos/worlds/physics/newton_dual_read_legacy.wbt":
        "deliberate legacy-extension demo world (created by this migration)",
}

# Owned by other agents this session -- report, never edit.
DO_NOT_EDIT = {
    "src/omnisim/physics/omnisim_newton_runtime.py",
    "src/omnisim/physics/OmNewtonBackend.cpp",
    "src/omnisim/physics/OmNewtonBackend.hpp",
    "resources/nodes/WorldInfo.wrl",
    "resources/nodes/Cloth.wrl",
    "resources/nodes/Solid.wrl",
    "docs/developer/omniworld-format.md",
}

# Reference-bearing text files we rewrite.
TEXT_SUFFIXES = {
    ".py", ".md", ".json", ".txt", ".sh", ".bat", ".yaml", ".yml", ".js",
    ".cpp", ".hpp", ".h", ".c", ".proto", ".wrl", ".ini", ".cfg", ".html",
    ".wbt", ".omniworld", ".mime", ".desktop", ".applications", ".keys",
    ".xml", ".plist", ".csv", ".jsonl", ".wbproj", ".gitignore",
}
TEXT_NAMES = {"Makefile", "Makefile.include", "pre-push", ".gitignore", "publish_deny.txt"}

# Never scan these for references.
SCAN_SKIP_DIRS = {
    ".git", "msys64", "node_modules", "_scratch", ".claude", "build",
    "dependencies", "__pycache__", ".build_tmp", ".pytest_cache",
}
# Machine-generated measurement output. A benchmark row records what was
# measured, on the path it was measured at -- rewriting it would falsify the
# evidence, not update it. `results*` catches results/ and results_published/.
SCAN_SKIP_RE = re.compile(
    r"(^|/)(results[\w-]*|evidence|preregister|goldens|optim-baseline"
    r"|\.agentbench[\w.-]*|captures)(/|$)"
)


def run(args, **kw):
    return subprocess.run(args, cwd=REPO, capture_output=True, text=True, **kw)


def tracked_worlds() -> list[str]:
    out = run(["git", "ls-files", "*" + OLD_EXT]).stdout
    return sorted(p for p in out.splitlines() if p.strip())


def excluded_reason(rel: str) -> str | None:
    for pref, why in EXCLUDED_PREFIXES.items():
        if rel.startswith(pref):
            return why
    if rel in DUAL_READ_KEEPERS:
        return "DUAL-READ KEEPER: " + DUAL_READ_KEEPERS[rel]
    return None


def build_rename_map() -> tuple[dict[str, str], dict[str, list[str]]]:
    """Returns (rename_map rel->rel, skipped {reason: [rel, ...]})."""
    renames, skipped = {}, collections.defaultdict(list)
    for rel in tracked_worlds():
        why = excluded_reason(rel)
        if why:
            skipped[why].append(rel)
            continue
        renames[rel] = rel[: -len(OLD_EXT)] + NEW_EXT
    return renames, skipped


def scannable_files() -> list[Path]:
    files = []
    for dirpath, dirnames, filenames in os.walk(REPO):
        rel_dir = Path(dirpath).relative_to(REPO).as_posix()
        dirnames[:] = [
            d for d in dirnames
            if d not in SCAN_SKIP_DIRS
            and not SCAN_SKIP_RE.search((rel_dir + "/" + d).lstrip("./"))
        ]
        for fn in filenames:
            p = Path(dirpath) / fn
            if fn in TEXT_NAMES or p.suffix in TEXT_SUFFIXES:
                files.append(p)
    return files


def basename_index(renames: dict[str, str], all_worlds: list[str]):
    """Basenames safe to rewrite bare (e.g. `husky_maze.omniworld` with no directory).

    A bare basename is only rewritten when it is unique among the RENAMED
    worlds AND is not also the basename of a world we deliberately did NOT
    rename -- otherwise `empty.wbt` in a doc could be retargeted at the wrong
    file, or at a Webots-arm world that must stay `.wbt`.
    """
    renamed_names = collections.Counter(Path(r).name for r in renames)
    kept_names = {Path(r).name for r in all_worlds if r not in renames}
    safe = {}
    ambiguous = []
    for name, n in renamed_names.items():
        if n > 1:
            ambiguous.append((name, n, "basename not unique among renamed worlds"))
        elif name in kept_names:
            ambiguous.append((name, n, "basename also used by a world we do NOT rename"))
        else:
            safe[name] = name[: -len(OLD_EXT)] + NEW_EXT
    return safe, ambiguous


# Any path-ish token ending in the legacy extension. One pass over the file,
# then a dict lookup per hit -- doing it the other way round (loop the ~660
# renames across every file) is minutes per run instead of seconds.
TOKEN_RE = re.compile(r"[\w.\\/-]*" + re.escape(OLD_EXT) + r"(?![\w])")


def build_tail_index(renames: dict[str, str]) -> dict[str, list[str]]:
    """basename -> [repo-relative paths that carry it], for disambiguation."""
    idx = collections.defaultdict(list)
    for old in renames:
        idx[Path(old).name].append(old)
    return idx


def rewrite_text(text, renames, safe_names, tail_idx):
    """Rewrite references. Returns (new_text, n_path_hits, n_name_hits)."""
    counts = [0, 0]  # [path hits, bare-name hits]

    def sub(m):
        tok = m.group(0)
        norm = tok.replace("\\", "/").lstrip("./")
        base = norm.rsplit("/", 1)[-1]
        renamed = tok[: -len(OLD_EXT)] + NEW_EXT

        # (a) Unambiguous basename: safe to rewrite wherever it appears, at any
        #     depth of path prefix, because exactly one renamed world owns it.
        if base in safe_names:
            counts[1 if "/" not in norm else 0] += 1
            return renamed

        # (b) Ambiguous basename: only rewrite when the token carries enough
        #     path to identify exactly one renamed world.
        cands = tail_idx.get(base, [])
        if cands and "/" in norm:
            hit = [c for c in cands if c == norm or c.endswith("/" + norm)]
            if len(hit) == 1:
                counts[0] += 1
                return renamed
        return tok

    new = TOKEN_RE.sub(sub, text)
    return new, counts[0], counts[1]


def cmd_plan(args):
    all_worlds = tracked_worlds()
    renames, skipped = build_rename_map()
    safe_names, ambiguous = basename_index(renames, all_worlds)
    tail_idx = build_tail_index(renames)

    print(f"tracked {OLD_EXT} worlds : {len(all_worlds)}")
    print(f"  -> rename            : {len(renames)}")
    print(f"  -> hold back          : {sum(len(v) for v in skipped.values())}\n")

    print("HELD BACK, by reason:")
    for why, rels in sorted(skipped.items()):
        print(f"  [{len(rels):>4}] {why}")
        if len(rels) <= 3:
            for r in rels:
                print(f"         {r}")
    print()

    by_area = collections.Counter(r.split("/")[0] + "/" + r.split("/")[1] for r in renames)
    print("RENAMES by area:")
    for area, n in sorted(by_area.items(), key=lambda kv: -kv[1])[:14]:
        print(f"  {n:>4}  {area}/")
    print()

    if ambiguous:
        print(f"BARE-BASENAME rewrites SKIPPED as unsafe ({len(ambiguous)}):")
        for name, n, why in sorted(ambiguous)[:20]:
            print(f"  {name}  ({why})")
        print("  -> these are only rewritten where a full path is present.\n")

    # Dry-run the reference rewrite to size it.
    touched, blocked, total_hits = [], [], 0
    for p in scannable_files():
        rel = p.relative_to(REPO).as_posix()
        try:
            text = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if OLD_EXT not in text:
            continue
        new, np_, nn = rewrite_text(text, renames, safe_names, tail_idx)
        if new == text:
            continue
        total_hits += np_ + nn
        (blocked if rel in DO_NOT_EDIT else touched).append((rel, np_ + nn))

    print(f"REFERENCE REWRITES: {total_hits} occurrence(s) across {len(touched)} file(s)")
    for rel, n in sorted(touched, key=lambda kv: -kv[1])[:20]:
        print(f"  {n:>5}  {rel}")
    if len(touched) > 20:
        print(f"  ... and {len(touched)-20} more files")
    if blocked:
        print(f"\n!! OWNED BY ANOTHER AGENT -- NOT EDITED, reported instead ({len(blocked)}):")
        for rel, n in blocked:
            print(f"  {n:>5}  {rel}")
    return 0


def cmd_apply(args):
    all_worlds = tracked_worlds()
    renames, _ = build_rename_map()
    safe_names, _ = basename_index(renames, all_worlds)
    tail_idx = build_tail_index(renames)

    # 1. Rename on disk, then stage old+new together so git records a RENAME
    #    (content is identical, so rename detection is exact). Done as a
    #    handful of batched `git add` calls rather than 661 `git mv`
    #    subprocesses, which is minutes of process spawn on Windows.
    moved = []
    for old, new in sorted(renames.items()):
        src, dst = REPO / old, REPO / new
        if not src.exists():
            continue
        if dst.exists():
            print(f"  SKIP (target exists): {new}", file=sys.stderr)
            continue
        os.replace(src, dst)
        moved += [old, new]
    for i in range(0, len(moved), 200):
        chunk = moved[i:i + 200]
        r = run(["git", "add", "-A", "--"] + chunk)
        if r.returncode != 0:
            print(f"  git add FAILED: {r.stderr.strip()}", file=sys.stderr)
    print(f"renamed {len(moved)//2} world(s)")

    # 2. rewrite references
    n_files = 0
    blocked = []
    for p in scannable_files():
        rel = p.relative_to(REPO).as_posix()
        try:
            text = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if OLD_EXT not in text:
            continue
        new, np_, nn = rewrite_text(text, renames, safe_names, tail_idx)
        if new == text:
            continue
        if rel in DO_NOT_EDIT:
            blocked.append((rel, np_ + nn))
            continue
        p.write_text(new, encoding="utf-8", newline="")
        n_files += 1
    print(f"rewrote references in {n_files} file(s)")
    if blocked:
        print("\nNOT EDITED (owned by another agent) -- apply by hand:")
        for rel, n in blocked:
            print(f"  {n:>5} hit(s)  {rel}")
    return 0


def cmd_verify(args):
    """Audit: every world path referenced in the tree should resolve."""
    bad = collections.defaultdict(list)
    world_re = re.compile(r"[\w./\\-]+\.(?:wbt|omniworld)")
    checked = 0
    for p in scannable_files():
        rel = p.relative_to(REPO).as_posix()
        try:
            text = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in set(world_re.findall(text)):
            cand = m.replace("\\", "/").lstrip("./")
            if "/" not in cand:
                continue  # bare filename, cannot resolve without context
            if not cand.startswith(("projects/", "tests/", "distribution/",
                                    "resources/", "scripts/")):
                continue
            checked += 1
            if not (REPO / cand).exists():
                bad[rel].append(cand)
    total = sum(len(v) for v in bad.values())
    print(f"checked {checked} repo-relative world reference(s)")
    if not total:
        print("OK: every resolvable world reference points at a file that exists")
        return 0
    print(f"\nDANGLING: {total} reference(s) in {len(bad)} file(s)")
    for rel, misses in sorted(bad.items(), key=lambda kv: -len(kv[1]))[:30]:
        print(f"  {rel}")
        for m in sorted(set(misses))[:6]:
            print(f"      {m}")
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--plan", action="store_true", help="print the plan, change nothing")
    g.add_argument("--apply", action="store_true", help="perform the migration")
    g.add_argument("--verify", action="store_true", help="audit world references resolve")
    args = ap.parse_args()
    if args.plan:
        return cmd_plan(args)
    if args.apply:
        return cmd_apply(args)
    return cmd_verify(args)


if __name__ == "__main__":
    sys.exit(main())
