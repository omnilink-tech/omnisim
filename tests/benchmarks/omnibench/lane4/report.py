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

"""lane4/report.py — render the MEASURED capability matrix from lane-4 rows.

    python tests/benchmarks/omnibench/lane4/report.py
    python .../report.py --out docs/benchmarks/lane4-capability-2026-08-10.md

The point of this script is that the capability matrix stops being prose.
docs/developer/simulator-comparison.md marks its own OmniSim column with the
weakest evidence tier it defines ("checked by us, by us, with no external
source and no independent audit"); everything this script emits is generated
from result rows that carry a machine id, an engine-binary sha256 and a
backend verdict sidecar, and every claim links to the probe that produced it.

It deliberately reports losses at the same size as wins, and keeps
`inconclusive` in its own section rather than folding it into a score:
an instrument failure is not evidence about the engine, in either direction.
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
from common import results as results_mod        # noqa: E402
from gen_worlds import _ascii, world_path        # noqa: E402

MARK = {
    caps.WORKS: "PASS",
    caps.DEGRADED: "PARTIAL",
    caps.BROKEN: "BROKEN",
    caps.ABSENT: "absent",
    caps.INCONCLUSIVE: "no result",
}
FAMILY_TITLE = {
    caps.FAM_OBJECT: "Objects and geometry -- what can carry physics",
    caps.FAM_JOINT: "Joints and actuation",
    caps.FAM_DEVICE: "Devices and sensors",
    caps.FAM_PHENOMENON: "Phenomena and world-level contracts",
}


def _rows(path):
    p = Path(path)
    return results_mod.read_rows(p) if p.exists() else []


def _num(v, digits=4):
    if isinstance(v, bool) or v is None:
        return str(v)
    if isinstance(v, float):
        if v != v:
            return "nan"
        s = "%.*f" % (digits, v)
        # Strip trailing zeros ONLY past a decimal point. An unguarded
        # rstrip("0") turns the integer 20 into "2", which is a silently wrong
        # number in a table nobody would re-derive.
        if "." in s:
            s = s.rstrip("0").rstrip(".")
        return s or "0"
    return str(v)


def _evidence_cell(ev, limit=3):
    """The two or three numbers that carry the verdict, with their units."""
    if not isinstance(ev, dict):
        return ""
    skip = {"scope", "settled_as"}
    items = [(k, v) for k, v in ev.items()
             if k not in skip and not isinstance(v, (list, dict))]
    return "; ".join("%s=%s" % (k, _num(v)) for k, v in items[:limit])


def _cap_cell(telemetry, sampled):
    """The constraint column. When the telemetry never sampled, the honest
    cell is not "None / None" -- that reads as a missing measurement when it
    actually means the solver has no fixed constraint buffer to overflow."""
    if not sampled:
        return "n/a"
    return "%s / %s" % (telemetry.get("peak_nefc"), telemetry.get("njmax"))


def machine_block(rows):
    for r in rows:
        m = r.get("machine") or {}
        mid = (m.get("machine") or {})
        if mid:
            binary = m.get("binary") or {}
            return {
                "id": mid.get("id"), "gpu": mid.get("gpu"),
                "cpu": mid.get("cpu"), "cores": mid.get("cores"),
                "os": m.get("os"),
                "binary_sha256": (binary.get("sha256") or "")[:16],
                "solver": m.get("solver"), "physics": m.get("physics"),
                "utc": r.get("utc"),
            }
    return {}


def binary_spread(rows):
    """Which engine binaries produced these rows, newest row per binary.

    The Machine block reports ONE binary -- the first row's. That is a lie the
    moment a campaign is topped up rather than re-run whole: a re-measured row
    carries a different `binary.sha256`, and the header silently attributes it
    to the old build. Since the whole point of this lane is that a stale record
    is as bad as a broken engine, the mixing has to be visible in the artefact
    rather than recoverable only from the raw jsonl.

    Returns [(sha16, n_rows, newest_utc)] sorted by row count, descending.
    """
    seen = {}
    for r in rows:
        if not r["test"].startswith("capability:"):
            continue
        sha = (((r.get("machine") or {}).get("binary") or {})
               .get("sha256") or "")[:16] or "unknown"
        n, utc = seen.get(sha, (0, ""))
        seen[sha] = (n + 1, max(utc, r.get("utc") or ""))
    return sorted(((s, n, u) for s, (n, u) in seen.items()),
                  key=lambda t: -t[1])


def _machine_label(rows):
    """`<id> <gpu>` for a result set, from the first row that carries one."""
    for r in rows:
        mid = ((r.get("machine") or {}).get("machine") or {})
        if mid.get("id"):
            return mid["id"], mid.get("gpu") or "?"
    return "unknown", "?"


def _verdict_map(rows):
    return {r["test"].split(":", 1)[1]: r["metrics"]["verdict"]
            for r in rows if r["test"].startswith("capability:")}


def cross_machine_section(A, cov_rows, cross_sets):
    """Per-probe verdicts on every machine that ran the lane.

    This exists because the coverage score was published for a year off ONE
    box, and a capability verdict is exactly the kind of claim that reads as a
    property of the simulator while being a property of a laptop. A row where
    the machines agree is worth one line; a row where they disagree is the
    finding, and it gets named rather than averaged away.

    ⚠️ A disagreement is NOT automatically a machine finding. Two machines
    almost never run the same engine binary (one is a Windows build, the other
    a Linux one, and they are rarely built from the same commit), so a
    difference has to be attributed to machine-vs-build before it is reported
    as either. Each machine's binary sha256 is in the table for that reason.
    """
    prim_id, prim_gpu = _machine_label(cov_rows)
    sets = [(prim_id, prim_gpu, cov_rows)] + [
        (mid, gpu, c["cov"]) for c in cross_sets if c.get("cov")
        for mid, gpu in [_machine_label(c["cov"])]]

    A("## Cross-machine — is this matrix a property of OmniSim or of one box?")
    A("")
    A("| machine | GPU | CPU | OS | engine binary | probes | PASS | PARTIAL | "
      "BROKEN | absent | no result |")
    A("|---|---|---|---|---|---|---|---|---|---|---|")
    for mid, gpu, rows in sets:
        m = machine_block(rows)
        probes = [r for r in rows if r["test"].startswith("capability:")]
        c = {v: 0 for v in caps.VERDICTS}
        for r in probes:
            c[r["metrics"]["verdict"]] = c.get(r["metrics"]["verdict"], 0) + 1
        # NEVER print just the first row's binary here: a topped-up campaign
        # mixes builds, and this table's whole job is to keep a build
        # difference from being read as a machine difference.
        sp = binary_spread(rows)
        binary = "`%s`" % (sp[0][0] if sp else "?")
        if len(sp) > 1:
            binary += " +%d more" % (len(sp) - 1)
        A("| `%s` | %s | %s | %s | %s | %d | %d | %d | %d | %d | %d |"
          % (mid, _ascii(gpu), _ascii(str(m.get("cpu"))), m.get("os"),
             binary, len(probes),
             c[caps.WORKS], c[caps.DEGRADED], c[caps.BROKEN],
             c[caps.ABSENT], c[caps.INCONCLUSIVE]))
    A("")

    maps = [(mid, _verdict_map(rows)) for mid, _gpu, rows in sets]
    common = set(maps[0][1])
    for _mid, mp in maps[1:]:
        common &= set(mp)
    disagree = sorted(p for p in common
                      if len({mp[p] for _mid, mp in maps}) > 1)
    A("**%d probes ran on all %d machines. %d agree, %d disagree.**"
      % (len(common), len(maps), len(common) - len(disagree), len(disagree)))
    A("")
    if disagree:
        A("| capability | " + " | ".join("`%s`" % mid for mid, _ in maps) + " |")
        A("|---|" + "---|" * len(maps))
        for p in disagree:
            A("| `%s` | %s |" % (p, " | ".join(
                "**%s**" % MARK.get(mp[p], mp[p]) for _mid, mp in maps)))
        A("")
        A("> Every row above is either a machine-dependent capability or a "
          "difference between the")
        A("> engine binaries in the table. **It is not publishable as either "
          "until it has been")
        A("> attributed.** The attribution lives in the campaign document for "
          "these rows, under")
        A("> [`docs/benchmarks/`](.) -- and the raw rows, one directory per "
          "machine per date, under")
        A("> [`tests/benchmarks/omnibench/results/`]"
          "(../../tests/benchmarks/omnibench/results/).")
        A("")
    only = [(mid, sorted(set(mp) - common)) for mid, mp in maps]
    extra = [(mid, ps) for mid, ps in only if ps]
    if extra:
        for mid, ps in extra:
            A("- `%s` additionally ran: %s" % (mid, ", ".join("`%s`" % p
                                                              for p in ps)))
        A("")


def render(cov_rows, env_rows, cpu_rows, cross_sets=()):
    probe_rows = [r for r in cov_rows if r["test"].startswith("capability:")]
    summary = next((r for r in cov_rows if r["test"] == "capability_summary"),
                   None)
    mach = machine_block(cov_rows or env_rows or cpu_rows)
    by_id = {r["test"].split(":", 1)[1]: r for r in probe_rows}

    L = []
    A = L.append
    A("# OmniSim capability matrix -- MEASURED")
    A("")
    A("> Generated by `tests/benchmarks/omnibench/lane4/report.py` from lane-4 "
      "result rows.")
    A("> **Do not hand-edit.** Re-run the lane and regenerate.")
    A("")
    A("Every row below is a probe that was executed against `omnisim-bin` and "
      "judged by an")
    A("assertion in physical units. This file exists because the capability "
      "table in")
    A("[simulator-comparison.md](../developer/simulator-comparison.md) marks "
      "its own OmniSim")
    A("column as self-attested -- the weakest evidence tier that document "
      "defines.")
    A("")
    A("## Machine")
    A("")
    A("| field | value |")
    A("|---|---|")
    for k in ("id", "gpu", "cpu", "cores", "os", "binary_sha256", "solver",
              "utc"):
        if mach.get(k) is not None:
            A("| %s | `%s` |" % (k, mach[k]))
    A("")
    if cross_sets:
        A("> ⚠️ **That is the PRIMARY machine only.** Other machines ran this "
          "lane too; the")
        A("> [Cross-machine](#cross-machine--is-this-matrix-a-property-of-"
          "omnisim-or-of-one-box) section")
        A("> below lists every one of them and every per-probe disagreement.")
        A("")
    spread = binary_spread(cov_rows)
    if len(spread) > 1:
        A("> ⚠️ **These rows were NOT all measured on one engine binary.** The "
          "block above")
        A("> names only the first row's. Rows re-measured after an engine "
          "change carry their")
        A("> own build, and each row's provenance is in its `machine` block in "
          "`results/coverage.jsonl`.")
        A("")
        A("| binary sha256[:16] | probe rows | newest row |")
        A("|---|---|---|")
        for sha, n, utc in spread:
            A("| `%s` | %d | %s |" % (sha, n, utc or "n/a"))
        A("")
    A("Reproduce: `python tests/benchmarks/omnibench/lane4/run_coverage.py`")
    A("")
    A("## What the verdicts mean")
    A("")
    A("| verdict | meaning |")
    A("|---|---|")
    A("| **PASS** | present, and the measurement lands where physics says it "
      "must |")
    A("| **PARTIAL** | present and does something, but misses the physical "
      "target (the note says by how much) |")
    A("| **BROKEN** | present in the schema -- the world loads, the device is "
      "accepted -- and measurably does nothing. **This is the verdict a "
      "load-only smoke test reports as PASS.** |")
    A("| absent | not in the schema; established by the engine REFUSING the "
      "declaration, never by \"we did not try\" |")
    A("| no result | the probe's own instrument failed. Not evidence about "
      "the engine, and excluded from every score. |")
    A("")
    if summary:
        m = summary["metrics"]
        A("## Summary")
        A("")
        A("| | count |")
        A("|---|---|")
        # via _num: the row schema stores every numeric metric as a float, so a
        # raw count prints as "23.0" and reads like a measurement rather than a
        # tally.
        for k in ("works", "degraded", "broken", "absent", "inconclusive"):
            A("| %s | %s |" % (MARK.get(k, k), _num(m.get(k), 0)))
        A("| **probes total** | **%s** |" % _num(m.get("probes_total"), 0))
        A("")
        A("Of the capabilities that **exist** (excluding `absent`, which is a "
          "scope statement")
        A("rather than a defect), **%.0f%%** work."
          % (100.0 * float(m.get("implemented_working_frac") or 0.0)))
        A("")
        A("> **Scope.** Rigid-body simulation only. Rendering quality is out "
          "of OmniBench's")
        A("> scope by design; the Camera and Lidar probes assert that an "
          "image EXISTS, nothing")
        A("> about how it looks.")
        A("")

    for fam in caps.FAMILIES:
        fam_probes = [p for p in caps.PROBES if p.family == fam
                      and p.id in by_id]
        if not fam_probes:
            continue
        A("## %s" % FAMILY_TITLE[fam])
        A("")
        A("| capability | verdict | measured | what the probe asserts |")
        A("|---|---|---|---|")
        for p in fam_probes:
            r = by_id[p.id]
            m = r["metrics"]
            note = _ascii(m.get("note") or "")
            claim = _ascii(p.claim)
            ev = _evidence_cell(m.get("evidence"))
            # For a PASS, the useful last column is WHAT WAS ASSERTED (the
            # assertion docstring's first sentence -- a claim a reader can
            # check). For anything else it is the failure mode, which is the
            # thing somebody is here to find.
            if m["verdict"] == caps.WORKS:
                phys = _ascii((m.get("physical_claim") or claim).strip())
                cell = " ".join(phys.split())
                cell = cell.split(". ")[0][:240]
            else:
                cell = note or claim
            A("| `%s`<br/>%s | **%s** | %s | %s |"
              % (p.id, claim, MARK.get(m["verdict"], m["verdict"]),
                 ev or "--", cell))
        A("")

    if cross_sets:
        cross_machine_section(A, cov_rows, cross_sets)

    mism = [r for r in probe_rows if r["metrics"].get("doc_agrees") is False]
    A("## Documentation audit")
    A("")
    if not mism:
        A("Every probe carrying a `documented_as` claim agreed with its "
          "measurement.")
    else:
        A("These capabilities are documented as one thing and measure as "
          "another. **Both**")
        A("directions matter: a feature listed broken that now works "
          "misleads as much as the")
        A("reverse, and costs the same debugging hours.")
        A("")
        A("| capability | docs say | measured | evidence |")
        A("|---|---|---|---|")
        for r in mism:
            m = r["metrics"]
            A("| `%s` | %s | **%s** | %s |"
              % (r["test"].split(":", 1)[1],
                 MARK.get(m.get("documented_as"), m.get("documented_as")),
                 MARK.get(m["verdict"], m["verdict"]),
                 _ascii(_evidence_cell(m.get("evidence")))))
    A("")

    incon = [r for r in probe_rows
             if r["metrics"]["verdict"] == caps.INCONCLUSIVE]
    if incon:
        A("## No result (instrument failures -- not engine findings)")
        A("")
        for r in incon:
            A("- `%s` -- %s" % (r["test"].split(":", 1)[1],
                                _ascii(r["metrics"].get("note") or "")))
        A("")

    # ---------------- envelope ----------------
    # Per MACHINE, never merged: the boxes sweep is a CPU claim, and the whole
    # reason this section exists is that "N bodies at real time" is a property
    # of the silicon. A single envelope table with no machine on it was read as
    # a property of OmniSim for as long as it was published.
    env_sets = [(env_rows, cov_rows)] + [(c.get("env") or [], c.get("cov") or [])
                                         for c in cross_sets]
    env_sets = [(e, c) for e, c in env_sets if e]
    if env_sets:
        A("## Resource envelope")
        A("")
        A("How large a scene each machine carries in real time, and where the "
          "scene begins")
        A("degrading **silently**. Wall-clock rows step a whole engine "
          "process (controller IPC")
        A("included) and are not comparable to a bare-library step time.")
        A("")
    for env_rows_m, cov_rows_m in env_sets:
        emid, egpu = _machine_label(env_rows_m or cov_rows_m)
        emb = machine_block(env_rows_m or cov_rows_m)
        for s in [r for r in env_rows_m
                  if r["test"].startswith("envelope_summary_")]:
            m = s["metrics"]
            A("### %s (`%s`) on `%s` -- %s, %s"
              % (m.get("scene"), m.get("solver"), emid,
                 _ascii(str(emb.get("cpu"))), _ascii(egpu)))
            A("")
            A("- realtime envelope: **%s**" % m.get("realtime_envelope"))
            A("- first N with a silent constraint overflow: **%s**"
              % m.get("first_n_with_constraint_overflow"))
            A("- constraint telemetry sampled: %s"
              % m.get("constraint_telemetry_sampled"))
            if (m.get("constraint_telemetry_sampled")
                    and not m.get("cliff_detector_validated")):
                # Never let an unfireable detector's silence read as a pass.
                A("")
                A("> **The overflow detector was never observed to fire on "
                  "this sweep, so its \"no overflow\" is UNVALIDATED, not a "
                  "pass.** See the row's `deviations` for what was tried.")
            A("")
            cells = [r for r in env_rows_m
                     if r["test"] == "envelope_%s" % m.get("scene")
                     and (r["metrics"].get("solver") == m.get("solver"))]
            sampled = bool(m.get("constraint_telemetry_sampled"))
            if cells:
                A("| N | ms/step | realtime | %s | overflow |"
                  % ("peak nefc / njmax" if sampled
                     else "constraints (no cap on this solver)"))
                A("|---|---|---|---|---|")
                for c in sorted(cells, key=lambda r: r["metrics"]["n"]):
                    cm = c["metrics"]
                    t = cm.get("constraint_telemetry") or {}
                    if not cm.get("measured"):
                        # A gap row still carries its constraint telemetry,
                        # which is the half of the rover sweep that matters.
                        A("| %s | (no measurement) | -- | %s | %s |"
                          % (_num(cm["n"], 0), _cap_cell(t, sampled),
                             "**YES**" if t.get("overflow") else "no"))
                        continue
                    A("| %s | %s | %sx | %s | %s |"
                      % (_num(cm["n"], 0), _num(cm.get("ms_per_step_total"), 3),
                         _num(cm.get("realtime_factor"), 2),
                         _cap_cell(t, sampled),
                         "**YES**" if t.get("overflow") else "no"))
                A("")

    # ---------------- cpu only ----------------
    cpu_sets = [cpu_rows] + [c.get("cpu") or [] for c in cross_sets]
    cpu_sets = [c for c in cpu_sets if c]
    if cpu_sets:
        A("## CPU-only (no CUDA device visible)")
        A("")
        A("| machine | verdict | Newton finalised | solver | rest z (m) | "
          "expected | identical to GPU-visible run |")
        A("|---|---|---|---|---|---|---|")
        for rows in cpu_sets:
            m = rows[0]["metrics"]
            cmid, _cgpu = _machine_label(rows)
            A("| `%s` | **%s** | %s | `%s` | %s | %s | %s |"
              % (cmid, MARK.get(m["verdict"], m["verdict"]),
                 m.get("newton_finalised_without_cuda"),
                 m.get("solver_without_cuda"), _num(m.get("rest_z_m")),
                 _num(m.get("expected_rest_z_m")),
                 m.get("trajectory_identical_to_gpu_run")))
        A("")
        A("> **%s**" % _ascii(cpu_sets[0][0]["metrics"].get("scope") or ""))
        A("")

    A("---")
    A("")
    A("Probe worlds are committed under "
      "[`tests/benchmarks/omnibench/lane4/worlds/`]"
      "(../../tests/benchmarks/omnibench/lane4/worlds/) -- one `.wbt` per row, "
      "loadable by anyone.")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--coverage", type=Path,
                    default=HERE / "results" / "coverage.jsonl")
    ap.add_argument("--envelope", type=Path,
                    default=HERE / "results" / "envelope.jsonl")
    ap.add_argument("--cpu-only", type=Path,
                    default=HERE / "results" / "cpu_only.jsonl")
    ap.add_argument("--out", type=Path, default=None,
                    help="write markdown here (default: stdout)")
    ap.add_argument("--cross", type=Path, action="append", default=None,
                    metavar="DIR|COVERAGE.JSONL",
                    help="another machine's lane-4 results, repeatable. Give a "
                         "results/<machine-id>/<date>/lane4/ directory and its "
                         "coverage/envelope/cpu_only rows are all picked up. "
                         "The primary --coverage file still owns the matrix, "
                         "so a second machine can never silently overwrite a "
                         "verdict.")
    a = ap.parse_args(argv)

    cross = []
    for p in (a.cross or []):
        if p.is_dir():
            c = {"cov": _rows(p / "coverage.jsonl"),
                 "env": _rows(p / "envelope.jsonl"),
                 "cpu": _rows(p / "cpu_only.jsonl")}
        else:
            c = {"cov": _rows(p), "env": [], "cpu": []}
        if not any(c.values()):
            sys.stderr.write("no lane-4 rows read from: %s\n" % p)
            return 2
        cross.append(c)
    text = render(_rows(a.coverage), _rows(a.envelope), _rows(a.cpu_only),
                  cross_sets=cross)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(text, encoding="utf-8", newline="\n")
        print("wrote %s (%d lines)" % (a.out, text.count("\n")))
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
