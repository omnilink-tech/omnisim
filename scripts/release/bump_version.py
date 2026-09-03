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

"""Bump every OmniSim version site at once, in ONE commit.

The release label lives in five files that nothing links (the four
tests/test_version_consistency.py pins, plus the packaging tag that names the
installer). publish_snapshot.sh rewrites them one file-group at a time and
commits each group separately, so a release is FOUR `version: bump ...`
commits (5 releases in 4 days produced 20 of them, 2026-08-29..09-01), and
before it rewrote all of them the strings drifted: f29cdae88, "version strings
two majors stale". This script rewrites all of them together, proves it with
the consistency test, and makes one commit.

    python scripts/release/bump_version.py 8.2.1 --dry-run   # print the diff, touch nothing
    python scripts/release/bump_version.py 8.2.1             # rewrite + run the test, no commit
    python scripts/release/bump_version.py 8.2.1 --commit    # ... and ONE commit:
                                                             #   version: bump to v8.2.1 (five sites)

A leading `v` on the version is accepted and stripped. The sites are:

    omnisim/__init__.py                       __version__ = "X.Y.Z"
    src/omnisim/core/OmApplicationInfo.cpp    omniSimVersionString = "X.Y.Z"
    docker/Dockerfile.train                   ARG OMNISIM_TAG=vX.Y.Z   (+ the docker-run example)
    .github/workflows/train-image.yml         default: "vX.Y.Z"  and  || 'vX.Y.Z'  (+ the gh example)
    scripts/packaging/omnisim_version.txt     vX.Y.Z   (read by scripts/packaging/generic_distro.py)

Rules the script enforces, because each one was a real failure:

  * The current values must AGREE before anything is rewritten. If they do
    not, it prints every site with its value and refuses -- a bump that
    "fixes" a drift by overwriting it hides which release the drift shipped
    in. Reconcile by hand (or run the test) first.
  * Only the version characters are replaced, in place, by span. The rest of
    each file -- line endings, BOM, quoting, trailing newline -- is written
    back byte-for-byte. publish_snapshot.sh's blanket `sed s/vX.Y.Z/.../g`
    on the two training-image files also rewrote a HISTORICAL sentence
    ("which is what broke the v8.1.17 train-image job" became "v8.2.0" in
    beb1698e0); the patterns here are anchored to the sites and leave prose
    alone.
  * `--commit` refuses when any of the five files already has uncommitted
    changes, so the one commit carries the bump and nothing else. It stages
    those five paths only (never `git add -A`).
  * The consistency test runs after the rewrite (not on --dry-run). A red
    test leaves the files rewritten for inspection and does NOT commit.

Exit codes: 0 done (or nothing to do), 1 refused (disagreeing sites, missing
pattern, dirty files, test failure, commit failure), 2 usage.
"""

from __future__ import annotations

import argparse
import difflib
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CONSISTENCY_TEST = Path("tests") / "test_version_consistency.py"

_SEMVER = r"\d+\.\d+\.\d+"
_COUNT_WORDS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six", 7: "seven", 8: "eight"}


@dataclass(frozen=True)
class Pattern:
    """One regex whose group(1) is the bare version (no leading v)."""
    label: str
    regex: str
    required: bool = True   # a required pattern with no match is a refusal, not a warning
    flags: int = re.M


@dataclass(frozen=True)
class Site:
    path: str                # repo-relative, forward slashes
    patterns: tuple[Pattern, ...]


# No end-of-line anchors on purpose: `$` under re.M sits BEFORE a `\n`, so a
# CRLF file would leave a `\r` between the closing quote and the anchor and the
# pattern would silently match nothing. Anchoring only the START keeps every
# pattern line-ending-agnostic, and replacing group(1) by span keeps the
# ending itself untouched.
SITES: tuple[Site, ...] = (
    Site("omnisim/__init__.py", (
        Pattern("__version__", rf'^__version__\s*=\s*"({_SEMVER})"'),
    )),
    Site("src/omnisim/core/OmApplicationInfo.cpp", (
        Pattern("omniSimVersionString", rf'omniSimVersionString\s*=\s*"({_SEMVER})"', flags=0),
    )),
    Site("docker/Dockerfile.train", (
        Pattern("ARG OMNISIM_TAG", rf'^ARG OMNISIM_TAG=v({_SEMVER})'),
        # The `docker run ... omnisim-train:vX.Y.Z bash` example at the top of
        # the file: what a user copies, so it tracks the release. Optional --
        # the example is documentation, the ARG is the contract.
        Pattern("docker run example", rf'omnisim-train:v({_SEMVER}) bash', required=False, flags=0),
    )),
    Site(".github/workflows/train-image.yml", (
        Pattern("workflow_dispatch default", rf'default:\s*"v({_SEMVER})"', flags=0),
        Pattern("env fallback", rf"\|\|\s*'v({_SEMVER})'", flags=0),
        Pattern("gh workflow run example", rf'-f omnisim_tag=v({_SEMVER})', required=False, flags=0),
    )),
    Site("scripts/packaging/omnisim_version.txt", (
        Pattern("package version", rf'\Av({_SEMVER})', flags=0),
    )),
)


@dataclass
class Hit:
    site: Site
    pattern: Pattern
    value: str
    span: tuple[int, int]     # span of group(1) in the file's text


def _read(site: Site) -> str:
    # newline="" -> no universal-newline translation: `\r\n` stays `\r\n`, so
    # the write-back is byte-exact. utf-8 (not utf-8-sig) keeps a BOM in-band.
    with open(REPO_ROOT / site.path, "r", encoding="utf-8", newline="") as fh:
        return fh.read()


def _write(site: Site, text: str) -> None:
    with open(REPO_ROOT / site.path, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)


def scan(site: Site, text: str) -> list[Hit]:
    """Every version hit in `text`, or raise with the site + pattern that is missing."""
    hits: list[Hit] = []
    for pat in site.patterns:
        matches = list(re.finditer(pat.regex, text, pat.flags))
        if not matches and pat.required:
            raise SystemExit(f"refusing: {site.path}: no match for {pat.label!r} ({pat.regex!r}); "
                             "the site moved -- update SITES in this script AND the consistency test")
        for m in matches:
            hits.append(Hit(site, pat, m.group(1), m.span(1)))
    return hits


def current_values() -> tuple[dict[str, str], list[tuple[Site, str, list[Hit]]]]:
    """({site.path: version}, [(site, text, hits)]) -- refuses on any disagreement.

    A site's value is the value of its REQUIRED hits; an optional hit that
    disagrees is reported the same way, because a stale example is the exact
    drift this script exists to end.
    """
    scanned = []
    values: dict[str, str] = {}
    rows: list[tuple[str, str]] = []
    for site in SITES:
        if not (REPO_ROOT / site.path).is_file():
            raise SystemExit(f"refusing: {site.path} not found (the site table is stale)")
        text = _read(site)
        hits = scan(site, text)
        scanned.append((site, text, hits))
        for h in hits:
            rows.append((f"{site.path} [{h.pattern.label}]", h.value))
        values[site.path] = hits[0].value
    distinct = sorted({v for _, v in rows})
    if len(distinct) != 1:
        width = max(len(k) for k, _ in rows)
        print("refusing: the version sites DISAGREE -- reconcile by hand before bumping:", file=sys.stderr)
        for k, v in rows:
            print(f"  {k.ljust(width)}  {v}", file=sys.stderr)
        print(f"  distinct values: {', '.join(distinct)}", file=sys.stderr)
        raise SystemExit(1)
    return values, scanned


def rewrite(text: str, hits: list[Hit], new: str) -> str:
    """Replace each hit's group(1) span with `new`, right-to-left so spans stay valid."""
    out = text
    for h in sorted(hits, key=lambda h: h.span[0], reverse=True):
        a, b = h.span
        out = out[:a] + new + out[b:]
    return out


def unified_diff(site: Site, old: str, new: str) -> str:
    return "".join(difflib.unified_diff(
        old.splitlines(keepends=True), new.splitlines(keepends=True),
        fromfile=f"a/{site.path}", tofile=f"b/{site.path}"))


def _git(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=check)


def dirty_version_files() -> list[str]:
    res = _git("status", "--porcelain", "--", *[s.path for s in SITES])
    return [ln[3:] for ln in res.stdout.splitlines() if ln.strip()]


def run_consistency_test() -> bool:
    cmd = [sys.executable, "-m", "pytest", str(CONSISTENCY_TEST), "-q", "-p", "no:cacheprovider"]
    print("running:", " ".join(cmd), flush=True)
    res = subprocess.run(cmd, cwd=REPO_ROOT)
    return res.returncode == 0


def commit_bump(old: str, new: str) -> None:
    n = len(SITES)
    subject = f"version: bump to v{new} ({_COUNT_WORDS.get(n, str(n))} sites)"
    body_lines = [f"{old} -> {new}, one commit for every version site (scripts/release/bump_version.py):", ""]
    body_lines += [f"  {s.path}" for s in SITES]
    _git("add", "--", *[s.path for s in SITES])
    res = _git("commit", "-m", subject, "-m", "\n".join(body_lines), check=False)
    if res.returncode != 0:
        sys.stderr.write(res.stdout + res.stderr)
        raise SystemExit("refusing: git commit failed (index.lock? a hook?) -- the files are rewritten and "
                         "staged; commit them yourself once the cause is cleared")
    head = _git("log", "-1", "--format=%h %s").stdout.strip()
    print(f"committed: {head}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("version", help="the new version, X.Y.Z (a leading v is accepted)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="print the diff; write nothing, run nothing")
    mode.add_argument("--commit", action="store_true",
                      help="after rewriting and a green consistency test, make ONE commit of the five files")
    args = ap.parse_args()

    new = args.version[1:] if args.version.startswith("v") else args.version
    if not re.fullmatch(_SEMVER, new):
        print(f"usage: version must be X.Y.Z (got {args.version!r})", file=sys.stderr)
        return 2

    values, scanned = current_values()
    old = next(iter(values.values()))
    n_hits = sum(len(hits) for _, _, hits in scanned)
    if old == new:
        print(f"no change: all {len(SITES)} version sites ({n_hits} occurrences) already read {old}")
        return 0

    if args.commit:
        dirty = dirty_version_files()
        if dirty:
            print("refusing: --commit would sweep unrelated edits into the bump; these version files have "
                  "uncommitted changes:", file=sys.stderr)
            for p in dirty:
                print(f"  {p}", file=sys.stderr)
            return 1

    label = "[dry run]" if args.dry_run else ""
    print(f"bump_version: {old} -> {new} ({len(SITES)} sites, {n_hits} occurrences) {label}".rstrip())
    rewritten: list[tuple[Site, str]] = []
    for site, text, hits in scanned:
        new_text = rewrite(text, hits, new)
        sys.stdout.write(unified_diff(site, text, new_text))
        rewritten.append((site, new_text))
    sys.stdout.flush()
    if args.dry_run:
        return 0

    for site, new_text in rewritten:
        _write(site, new_text)
    # Re-read through the same scanner: the write-back must leave every site
    # agreeing on the NEW value, or something about a pattern is wrong.
    after, _ = current_values()
    if set(after.values()) != {new}:
        raise SystemExit(f"refusing: after rewrite the sites read {sorted(set(after.values()))}, not {new}")
    print(f"rewrote {len(SITES)} files")

    if not run_consistency_test():
        print(f"{CONSISTENCY_TEST.as_posix()} is RED -- files left rewritten for inspection, nothing committed",
              file=sys.stderr)
        return 1

    if args.commit:
        commit_bump(old, new)
    else:
        print("not committed (pass --commit for the single commit, or commit the five paths yourself)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
