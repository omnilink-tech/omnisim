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

"""Detect and repair duplicate .dist-info metadata in a --target site-packages.

`pip install --upgrade --target <dir>` overwrites package FILES but does not
remove the previous version's .dist-info, so a re-staged bundle accumulates
one dist-info per historical install. That makes `importlib.metadata.version()`
ambiguous -- it returns whichever dist-info wins the directory scan -- and any
provenance built on it (env_fingerprint, skill manifests) records a version
that may not be what is installed. Found live 2026-07-16: the shipped
newton-runtime carried TWO dist-infos each for newton, mujoco, mujoco_warp and
warp_lang.

The ground truth is each dist-info's RECORD: the dist whose recorded file
hashes match the installed tree is the real one; every other is stale metadata.

Usage:
    python audit_dist_info.py <site-packages>            # report only
    python audit_dist_info.py <site-packages> --repair   # delete stale dist-infos

Exit codes: 0 clean, 1 duplicates present (report mode) or unrepairable
(ambiguous/no matching RECORD -- never auto-deleted).
"""
import argparse
import base64
import csv
import hashlib
import re
import shutil
import sys
from collections import defaultdict
from pathlib import Path

_DIST_RE = re.compile(r"^(.+)-([0-9][^-]*)\.dist-info$")


def _record_matches(site: Path, dist: Path, sample_cap: int = 400):
    """(ok, bad, missing) for the dist's RECORD-listed code files."""
    rec = dist / "RECORD"
    if not rec.exists():
        return 0, 0, 1
    ok = bad = missing = 0
    with rec.open(newline="", encoding="utf-8") as fh:
        for row in csv.reader(fh):
            if len(row) < 2 or not row[1]:
                continue
            p, want = row[0], row[1]
            # code files only: metadata files self-reference and change freely
            if p.startswith(dist.name) or not p.endswith((".py", ".pyd", ".so", ".dll")):
                continue
            if ok + bad + missing >= sample_cap:
                break
            f = site / p
            if not f.exists():
                missing += 1
                continue
            digest = base64.urlsafe_b64encode(
                hashlib.sha256(f.read_bytes()).digest()).rstrip(b"=").decode()
            if want == "sha256=" + digest:
                ok += 1
            else:
                bad += 1
    return ok, bad, missing


def audit(site: Path, repair: bool = False):
    """Return (anomalies, repaired): duplicate groups found, and those fixed."""
    groups = defaultdict(list)
    for d in sorted(site.glob("*.dist-info")):
        m = _DIST_RE.match(d.name)
        if m:
            groups[m.group(1).lower()].append(d)
    anomalies, repaired = [], []
    for name, dists in sorted(groups.items()):
        if len(dists) < 2:
            continue
        scored = []
        for d in dists:
            ok, bad, missing = _record_matches(site, d)
            scored.append((d, ok, bad, missing))
            print(f"  {d.name}: match={ok} mismatch={bad} missing={missing}")
        real = [s for s in scored if s[2] == 0 and s[3] == 0 and s[1] > 0]
        stale = [s for s in scored if s not in real]
        if len(real) == 1 and repair:
            for d, *_ in stale:
                print(f"  REPAIR: deleting stale {d.name}")
                shutil.rmtree(d)
            repaired.append(name)
        else:
            anomalies.append((name, [d.name for d in dists],
                              "ambiguous" if len(real) != 1 else "report-only"))
    return anomalies, repaired


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("site_packages", type=Path)
    ap.add_argument("--repair", action="store_true",
                    help="delete stale dist-infos (only when exactly one dist's "
                         "RECORD matches the installed tree)")
    args = ap.parse_args(argv)
    site = args.site_packages
    if not site.is_dir():
        ap.error(f"not a directory: {site}")
    anomalies, repaired = audit(site, repair=args.repair)
    if repaired:
        print(f"repaired: {', '.join(repaired)}")
    if anomalies:
        for name, dists, why in anomalies:
            print(f"DUPLICATE dist-info for '{name}' ({why}): {', '.join(dists)}")
        return 1
    print("dist-info metadata clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
