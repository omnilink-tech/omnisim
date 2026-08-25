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

"""lane4 — top up a coverage sweep with re-measured probes, reproducibly.

    python .../merge_coverage.py --into results/coverage.jsonl /tmp/relidar.jsonl
    python .../merge_coverage.py --into results/coverage.jsonl --dry-run new.jsonl

Re-running all 45 probes to correct one of them wastes an engine launch per
probe and, worse, re-dates 44 rows that nobody re-measured. So the file is
topped up instead: a probe row is replaced, every other row is left BYTE
IDENTICAL, and the summary row is recomputed from what is actually in the file.

Until 2026-08-17 that was done by hand, which is how a stale row survives:
`device.lidar` was published as `no result` for two days after the instrument
failure behind it was gone, and it took a run on a different machine to notice.
Hand-editing is also how the summary and the rows drift apart, since the
percentage is a derived number that nobody re-derives with a text editor.

Three refusals, each one a mistake that would otherwise be silent:

1. **A row from a DIFFERENT MACHINE is refused.** `results/coverage.jsonl` is
   one machine's matrix and `report.py` keys probes by id, so merging another
   box's rows would not add a column -- it would silently overwrite verdicts and
   publish them under the first machine's name. Cross-machine comparison is
   `report.py --cross`, which keeps the files separate on purpose.
2. **A CONTROL RUN is refused.** Rows produced by `run_coverage.py --env` are
   deliberately not measurements of the shipped default; they exist to attribute
   a verdict to an engine change. They say so in `deviations`, and this refuses
   on that marker.
3. **An OLDER row is refused.** Top-ups move forward in time; a top-up whose
   `utc` predates the row it would replace is a mistake, not a correction.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OMNIBENCH = HERE.parent
REPO = HERE.parents[3]
for p in (str(HERE), str(OMNIBENCH), str(REPO)):
    if p not in sys.path:
        sys.path.insert(0, p)

import capabilities as caps                      # noqa: E402
import run_coverage                              # noqa: E402
from common import results as results_mod        # noqa: E402

CONTROL_MARK = "CONTROL RUN"
TOPPED_UP = ("TOPPED UP, not re-swept: only the re-measured probes carry this "
             "row's binary; every other row keeps the binary in its own "
             "`machine` block and report.py's provenance banner names the "
             "spread")


def _mid(row):
    return ((row.get("machine") or {}).get("machine") or {}).get("id")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--into", type=Path,
                    default=HERE / "results" / "coverage.jsonl")
    ap.add_argument("topup", type=Path, nargs="+",
                    help="jsonl files of re-measured probe rows")
    ap.add_argument("--out", type=Path, default=None,
                    help="write here instead of updating --into in place")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    base = results_mod.read_rows(a.into)
    base_probes = [r for r in base if r["test"].startswith("capability:")]
    if not base_probes:
        print("no probe rows in %s" % a.into)
        return 2
    base_mid = _mid(base_probes[0])

    new = {}
    for p in a.topup:
        for r in results_mod.read_rows(p):
            if not r["test"].startswith("capability:"):
                continue
            if any(CONTROL_MARK in str(d) for d in (r.get("deviations") or [])):
                print("REFUSED (control run): %s from %s" % (r["test"], p))
                return 2
            if _mid(r) != base_mid:
                print("REFUSED (machine %s != %s): %s from %s -- use "
                      "report.py --cross for another machine"
                      % (_mid(r), base_mid, r["test"], p))
                return 2
            prev = new.get(r["test"])
            if prev is None or (r.get("utc") or "") > (prev.get("utc") or ""):
                new[r["test"]] = r
    if not new:
        print("no probe rows to merge")
        return 2

    by_test = {r["test"]: r for r in base_probes}
    for t, r in sorted(new.items()):
        old = by_test.get(t)
        if old is None:
            print("ADD    %-50s %s" % (t, r["metrics"]["verdict"]))
            continue
        if (r.get("utc") or "") < (old.get("utc") or ""):
            print("REFUSED (older than the row it replaces): %s (%s < %s)"
                  % (t, r.get("utc"), old.get("utc")))
            return 2
        print("%s %-50s %s -> %s"
              % ("CHANGE" if old["metrics"]["verdict"] != r["metrics"]["verdict"]
                 else "resweep", t, old["metrics"]["verdict"],
                 r["metrics"]["verdict"]))

    merged = [new.get(r["test"], r) for r in base_probes]
    merged += [r for t, r in sorted(new.items()) if t not in by_test]

    # The summary is DERIVED. Recompute it rather than editing counts, or the
    # percentage and the rows go out of sync -- which is the thing this lane
    # exists to make impossible.
    srow = run_coverage.summary_row(
        merged, results_mod.machine_fingerprint(),
        float((next((r for r in base if r["test"] == "capability_summary"),
                    {}).get("metrics") or {}).get("sweep_wall_s") or 0.0))
    srow["deviations"] = list(srow["deviations"]) + [TOPPED_UP]
    m = srow["metrics"]
    print("\nsummary: works %d | degraded %d | broken %d | absent %d | "
          "inconclusive %d -> %.1f%% of what exists works"
          % (m["works"], m["degraded"], m["broken"], m["absent"],
             m["inconclusive"], 100.0 * m["implemented_working_frac"]))
    if m["doc_mismatch_count"]:
        print("DOC MISMATCHES (%d): %s"
              % (m["doc_mismatch_count"], ", ".join(m["doc_mismatches"])))

    if a.dry_run:
        print("\n--dry-run: nothing written")
        return 0
    out = a.out or a.into
    tmp = out.with_suffix(out.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        for r in merged + [srow]:
            f.write(json.dumps(r, sort_keys=False) + "\n")
    tmp.replace(out)
    print("\nwrote %s (%d rows)" % (out, len(merged) + 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
