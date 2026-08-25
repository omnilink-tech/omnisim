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

"""Backfill GitHub Release pages for tags that were pushed without one.

The first six versions (v1.0.0 — v1.0.6) of OmniSim were published before
publish_snapshot.sh learned to call the GitHub Releases API. The tags exist
on the public remote but the Releases sidebar on the repo home page is
empty and "Watch -> Releases only" subscribers got no notifications.

This script reads CHANGELOG.md, extracts the per-version section for each
named tag, and POSTs a Release page for any that doesn't already exist.

Usage:
    GITHUB_TOKEN=ghp_... python scripts/release/backfill_release_pages.py \\
        --owner-repo omnilink-tech/omnisim v1.0.0 v1.0.1 v1.0.2

Or to backfill every section in CHANGELOG.md that doesn't already have a
Release page:
    GITHUB_TOKEN=ghp_... python scripts/release/backfill_release_pages.py \\
        --owner-repo omnilink-tech/omnisim --all

Idempotent: existing Release pages are left alone. Tags that have no matching
section in CHANGELOG.md are skipped with a warning rather than published with
an empty body.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

VERSION_HEADING = re.compile(r"^## \[(v\d+\.\d+\.\d+(?:-[A-Za-z0-9.-]+)?)\]")


def parse_changelog(path: Path) -> dict[str, str]:
    """Return {version: body_markdown} for every "## [vX.Y.Z]" section in
    the file. The body is the markdown between this heading and the next
    "## [" heading, stripped of trailing horizontal-rule + blank lines."""
    sections: dict[str, str] = {}
    current_version: str | None = None
    current_lines: list[str] = []

    def commit():
        if current_version is None:
            return
        body = "\n".join(current_lines).rstrip()
        # Strip trailing "---" rule + blank lines (document-layout markers).
        body = re.sub(r"\n+---+\s*$", "", body).rstrip()
        # Strip leading blank lines.
        body = body.lstrip("\n")
        sections[current_version] = body

    with path.open(encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip("\n")
            m = VERSION_HEADING.match(line)
            if m:
                commit()
                current_version = m.group(1)
                current_lines = []
                continue
            if current_version is not None:
                current_lines.append(line)
    commit()
    return sections


def github_api(method: str, url: str, token: str, data: dict | None = None):
    body = None if data is None else json.dumps(data).encode("utf-8")
    req = urllib.request.Request(url, data=body, method=method)
    req.add_header("Authorization", f"Bearer {token}")
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("X-GitHub-Api-Version", "2022-11-28")
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8") or "null")
    except urllib.error.HTTPError as e:
        try:
            payload = json.loads(e.read().decode("utf-8") or "null")
        except Exception:
            payload = None
        return e.code, payload


def existing_release_tags(owner_repo: str, token: str) -> set[str]:
    """Return the set of tag names that already have a Release page."""
    tags: set[str] = set()
    page = 1
    while True:
        status, data = github_api(
            "GET",
            f"https://api.github.com/repos/{owner_repo}/releases?per_page=100&page={page}",
            token,
        )
        if status != 200 or not data:
            break
        for rel in data:
            tag = rel.get("tag_name")
            if tag:
                tags.add(tag)
        if len(data) < 100:
            break
        page += 1
    return tags


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--owner-repo",
        required=True,
        help="GitHub owner/repo, e.g. omnilink-tech/omnisim",
    )
    p.add_argument(
        "--changelog",
        default="CHANGELOG.md",
        help="Path to CHANGELOG.md (default: ./CHANGELOG.md)",
    )
    p.add_argument(
        "--all",
        action="store_true",
        help="Backfill every version found in CHANGELOG.md that doesn't already have a Release page.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would happen without calling the API.",
    )
    p.add_argument(
        "versions",
        nargs="*",
        help="Specific versions to backfill (e.g. v1.0.1 v1.0.2). Ignored when --all is set.",
    )
    args = p.parse_args()

    token = os.environ.get("GITHUB_TOKEN", "").strip()
    if not args.dry_run and not token:
        print("error: GITHUB_TOKEN env var is required (or use --dry-run)", file=sys.stderr)
        return 2

    changelog_path = Path(args.changelog)
    if not changelog_path.is_file():
        print(f"error: {changelog_path} not found", file=sys.stderr)
        return 2
    sections = parse_changelog(changelog_path)
    if not sections:
        print("error: no '## [vX.Y.Z]' sections found in CHANGELOG.md", file=sys.stderr)
        return 2

    if args.dry_run:
        existing: set[str] = set()
    else:
        print(f"==> fetching existing Release pages on {args.owner_repo}...")
        existing = existing_release_tags(args.owner_repo, token)
        print(f"    {len(existing)} existing Release(s): {sorted(existing) or '(none)'}")

    if args.all:
        targets = sorted(sections.keys())
    else:
        if not args.versions:
            print("error: provide versions or pass --all", file=sys.stderr)
            return 2
        targets = args.versions

    successes = 0
    skipped = 0
    failures = 0
    for version in targets:
        if version in existing:
            print(f"==> {version}: already has a Release page — skip")
            skipped += 1
            continue
        body = sections.get(version)
        if body is None:
            print(f"==> {version}: no [{version}] section in CHANGELOG.md — skip")
            skipped += 1
            continue
        payload = {
            "tag_name":   version,
            "name":       f"OmniSim {version}",
            "body":       body,
            "draft":      False,
            "prerelease": "-" in version,
        }
        if args.dry_run:
            print(f"==> {version}: would POST {len(body)} bytes of notes (dry run)")
            successes += 1
            continue
        status, resp = github_api(
            "POST",
            f"https://api.github.com/repos/{args.owner_repo}/releases",
            token,
            payload,
        )
        if status == 201 and isinstance(resp, dict):
            print(f"==> {version}: created -> {resp.get('html_url', '(no url returned)')}")
            successes += 1
        else:
            msg = ""
            if isinstance(resp, dict):
                msg = resp.get("message", "")
                errs = resp.get("errors") or []
                if errs:
                    msg += " | " + json.dumps(errs)
            print(f"==> {version}: FAILED (HTTP {status}): {msg}", file=sys.stderr)
            failures += 1

    print(f"\nsummary: created={successes}, skipped={skipped}, failed={failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
