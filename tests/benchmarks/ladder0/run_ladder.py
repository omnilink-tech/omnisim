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

"""run_ladder.py -- the ladder's single entry point.

    python tests/benchmarks/ladder0/run_ladder.py                # every arm
    python tests/benchmarks/ladder0/run_ladder.py --sims omnisim
    python tests/benchmarks/ladder0/run_ladder.py --rungs 3,4
    python tests/benchmarks/ladder0/run_ladder.py --self-test
    python tests/benchmarks/ladder0/run_ladder.py --regen-worlds
    python tests/benchmarks/ladder0/run_ladder.py --list-binaries
    python tests/benchmarks/ladder0/run_ladder.py --sims omnisim \\
        --omnisim-bin msys64/mingw64/bin/omnisim-bin-round1.exe   # A/B

Exit code is 0 only when every requested (rung x arm) cell is green.  A cell
that could not run at all is a FAILURE, not a skip: an arm that is absent is
reported as absent, and an arm that is present but produced nothing is red.

Arms are discovered as ``ladder0/<name>/arm.py`` -- see CONTRACT.md for the
three-function interface an arm must implement.
"""

from __future__ import annotations

import argparse
import datetime
import importlib.util
import json
import os
import statistics
import sys
import types

HERE = os.path.abspath(os.path.dirname(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import analysis                                  # noqa: E402
import rungs                                     # noqa: E402
import selftest                                  # noqa: E402

ARM_NAMES = ("omnisim", "webots", "mujoco")
RESULTS = os.path.join(HERE, "results")

# Filled by ``load_arm``: arm-local modules registered in ``sys.modules`` under
# a name generic enough for another arm to collide with.  Not fatal on its own
# -- nothing has gone wrong until a SECOND arm picks one up -- but it is the
# precondition for the collision below, so it is printed rather than kept.
IMPORT_HAZARDS = []


class ArmImportCollision(RuntimeError):
    """An arm is holding a module that belongs to a different arm.

    ``sys.modules`` is process-global and all three arms ship a module called
    ``worldgen``, so a bare ``import worldgen`` in the second arm loaded
    resolves to the FIRST arm's generator.  That arm would then generate, and
    measure, another simulator's scenes while reporting them as its own -- a
    row that is wrong in a way no assertion in this ladder can see, because
    every number in it is internally consistent.

    This is fatal on purpose.  A measurement whose provenance is unknown is
    worth less than no measurement, and the ladder's whole claim is that a
    green row means something.
    """


def _module_file(mod):
    f = getattr(mod, "__file__", None)
    return os.path.realpath(f) if f else None


def check_arm_modules(name, mod, root=None):
    """Every module reachable from arm ``name`` must belong to it or be shared.

    Walks the module graph rooted at the arm module -- descending only into
    modules whose file lives under the ladder, so this never wanders into the
    stdlib -- and returns a list of human-readable violations.

    A module is legitimate when its file is
      * outside the ladder entirely (stdlib, site-packages, a controller lib),
      * directly in ``ladder0/`` (the shared contract: ``rungs``, ``analysis``,
        ``selftest`` -- CONTRACT.md section 2 tells arms to import these), or
      * anywhere under ``ladder0/<name>/`` (the arm's own tree).
    Anything under ``ladder0/<other arm>/`` is the collision.
    """
    root = os.path.realpath(root or HERE)
    mine = os.path.join(root, name) + os.sep
    violations = []
    seen = set()
    queue = [(name, mod)]
    while queue:
        attr_name, m = queue.pop()
        if id(m) in seen:
            continue
        seen.add(id(m))
        path = _module_file(m)
        if path is None or not path.startswith(root + os.sep):
            continue                             # outside the ladder: fine
        if os.path.dirname(path) == root:
            continue                             # the shared contract: fine
        if not path.startswith(mine):
            violations.append(
                "arm %r bound %r to %s, which belongs to arm %r"
                % (name, attr_name, os.path.relpath(path, root),
                   os.path.relpath(path, root).split(os.sep)[0]))
            continue
        for sub_name, value in list(vars(m).items()):
            if isinstance(value, types.ModuleType):
                queue.append((sub_name, value))
    return violations


def _collidable_names(name, root=None):
    """Arm-local modules registered in ``sys.modules`` under a generic name.

    These are what a later arm's bare import will find.  The Webots arm's
    ``_load_sibling`` shows the fix: load by path under an arm-qualified name
    (``ladder0_webots_worldgen``), which cannot collide with anything.
    """
    root = os.path.realpath(root or HERE)
    mine = os.path.join(root, name) + os.sep
    out = []
    for key, m in list(sys.modules.items()):
        if not isinstance(m, types.ModuleType) or key.startswith("ladder0_"):
            continue
        path = _module_file(m)
        if path and path.startswith(mine):
            out.append("%s -> %s" % (key, os.path.relpath(path, root)))
    return out


def load_arm(name):
    """Import ``ladder0/<name>/arm.py``, or ``None`` if it does not exist.

    Never raises for a BROKEN arm -- that is a measurement, and it comes back
    as an arm that reports itself unavailable.  It does raise
    :class:`ArmImportCollision` for an arm that has loaded another arm's
    modules, which is not a measurement but a corrupt instrument.
    """
    path = os.path.join(HERE, name, "arm.py")
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("ladder0_arm_%s" % name,
                                                  path)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:                     # noqa: BLE001
        # BIND THE TEXT HERE.  ``except ... as exc`` unbinds ``exc`` at the end
        # of the block, so a closure that reads it raises NameError when it is
        # eventually called -- which turned "this arm is broken" into "the
        # runner is broken", and hid a second defect behind the traceback.
        why = "arm failed to import: %r" % (exc,)

        class _Broken:
            NAME = name

            @staticmethod
            def available():
                return False, why

            @staticmethod
            def run(*_a, **_k):
                return ({"rung": -1, "sim": name, "t": [], "steps": 0},
                        {"sim": name, "error": why, "exit_code": None,
                         "proc_t0": None, "proc_t1": None})
        return _Broken
    if not hasattr(mod, "NAME"):
        mod.NAME = name
    violations = check_arm_modules(name, mod)
    if violations:
        raise ArmImportCollision(
            "%s\nAll three arms ship a module called 'worldgen'; a bare import "
            "resolves to whichever arm was loaded first.  Load siblings BY "
            "PATH under an arm-qualified name (see webots/arm.py "
            "_load_sibling)." % "\n".join(violations))
    for hazard in _collidable_names(name):
        entry = "%s: %s" % (name, hazard)
        if entry not in IMPORT_HAZARDS:
            IMPORT_HAZARDS.append(entry)
    return mod


# --------------------------------------------------------------------------
# one cell
# --------------------------------------------------------------------------

def cell_verdict(cell):
    """PASS / FAIL / PARTIAL / NOT_EXPRESSIBLE for one cell.

    An ``N/E`` check is not green and not red (CONTRACT.md amendment B): it
    does not set the exit code and it is never drawn in the same colour as a
    failure.  A cell with at least one is ``PARTIAL``; an all-``N/E`` cell is
    ``NOT_EXPRESSIBLE``.  Neither is a pass and neither is a failure -- an
    arm's missing capability must not flatter us and must not be scored
    against it.
    """
    checks = cell["checks"]
    ne = [c for c in checks if c.get("not_expressible")]
    live = [c for c in checks if not c.get("not_expressible")]
    bad = [c for c in live if not c["ok"]]
    if cell["meta"].get("error"):
        return "FAIL"
    if not live:
        return "NOT_EXPRESSIBLE"
    if bad:
        return "FAIL"
    return "PARTIAL" if ne else "PASS"


def run_cell(arm, rung, out_root, **run_kw):
    out_dir = os.path.join(out_root, arm.NAME, "rung%d" % rung)
    os.makedirs(out_dir, exist_ok=True)
    samples, meta = arm.run(rung, out_dir, **run_kw)
    m = analysis.reduce_samples(samples, exit_code=meta.get("exit_code"))
    checks = rungs.apply_not_expressible(rungs.check_rung(rung, m),
                                         meta.get("not_expressible"))
    checks = [c.as_dict() for c in checks]
    timing = analysis.wall_timings(samples, meta.get("proc_t0"),
                                   meta.get("proc_t1"))
    cell = {
        "sim": arm.NAME, "rung": rung, "title": rungs.RUNG_TITLE[rung],
        "checks": checks, "timing": timing,
        "meta": {k: v for k, v in meta.items() if k != "script_output"},
        "measurement": {k: v for k, v in m.items()
                        if not isinstance(v, (list, dict))},
    }
    cell["verdict"] = cell_verdict(cell)
    cell["green"] = cell["verdict"] in ("PASS", "PARTIAL")
    # The FULL reduction, kept out of the serialised cell (it carries per-toss
    # and per-fleet dicts) and used by the cross-cell pass below.
    cell["_m"] = m
    try:
        with open(os.path.join(out_dir, "cell.json"), "w",
                  encoding="utf-8") as f:
            json.dump({k: v for k, v in cell.items() if k != "_m"}, f, indent=1)
        with open(os.path.join(out_dir, "measurement.json"), "w",
                  encoding="utf-8") as f:
            json.dump(m, f, indent=1)
    except OSError:
        pass
    return cell


#: The reference arm for rung 18's cross-arm check.  Our solver IS MuJoCo, so
#: the bare MuJoCo arm is the only honest reference for what our translation
#: layer costs -- and the check is written so that arm can never win it.
EMBED_REFERENCE = "mujoco"


def cross_cell_checks(cells):
    """Checks that read TWO cells.  Today: rung 18's ``embed_gap``.

    It cannot live in ``check_rung``, which sees one arm's measurement.  It is
    still JUDGED IN rungs.py -- ``check_rung18_embed_gap`` -- so no verdict is
    computed here or in an arm; this function only pairs the cells.

    The reference arm does not get the check at all.  ``|its error - its own
    error|`` is zero by construction, and a free green on the one row that is
    supposed to be unwinnable would be the worst possible way to report it.
    """
    ref = next((c for c in cells
                if c["rung"] == 18 and c["sim"] == EMBED_REFERENCE), None)
    for cell in cells:
        if cell["rung"] != 18 or cell["sim"] == EMBED_REFERENCE:
            continue
        if ref is None:
            chk = rungs.Check(
                "embed_gap", None, 0.0, rungs.RUNG18_EMBED_GAP_TOL,
                "% of cube width", "cross-arm; needs the reference cell")
            chk.not_expressible = {
                "missing": "the bare %s arm's rung-18 cell was not run, and "
                           "this check is |our error - ITS error| on the same "
                           "tosses.  It is a CROSS-ARM measurement and a "
                           "single arm structurally cannot produce it"
                           % EMBED_REFERENCE,
                "citation": "tests/benchmarks/ladder0/CONTRACT.md section 3c; "
                            "rungs.check_rung18_embed_gap",
                "status": "CITED",
            }
        else:
            chk = rungs.check_rung18_embed_gap(cell["_m"], ref["_m"])
        cell["checks"].append(chk.as_dict())
        cell["verdict"] = cell_verdict(cell)
        cell["green"] = cell["verdict"] in ("PASS", "PARTIAL")
    return cells


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------

def _f(v, nd=6):
    if v is None:
        return "-"
    if isinstance(v, float):
        if v != v:
            return "nan"
        return ("%.*g" % (nd, v))
    return str(v)


def print_table(cells):
    print()
    print("RUNG x SIMULATOR -- every expected value is analytic "
          "(tests/benchmarks/ladder0/rungs.py)")
    print("=" * 108)
    hdr = ("%-5s %-9s %-21s %14s %14s %11s %11s  %s"
           % ("rung", "sim", "check", "measured", "expected", "tol",
              "margin", ""))
    print(hdr)
    print("-" * 108)
    for c in cells:
        for i, chk in enumerate(c["checks"]):
            # N/E is drawn as its own word, never as ok and never as FAIL.
            # Colouring a missing capability like a failure punishes an engine
            # for a category it never claimed; colouring it like a pass
            # flatters whoever happens to have the capability.
            state = ("N/E" if chk.get("not_expressible")
                     else "ok" if chk["ok"] else "FAIL")
            print("%-5s %-9s %-21s %14s %14s %11s %11s  %s"
                  % (c["rung"] if i == 0 else "", c["sim"] if i == 0 else "",
                     chk["name"], _f(chk["measured"]), _f(chk["expected"]),
                     _f(chk["tol"], 3), _f(chk["margin"], 3), state))
            if chk.get("not_expressible"):
                print("      %-9s   %s" % ("", "N/E: "
                      + str(chk["not_expressible"].get("missing", ""))[:150]))
            if chk.get("ne_invalid"):
                print("      %-9s   %s" % ("", "! " + chk["ne_invalid"]))
        note = c["meta"].get("error")
        if note:
            print("      %-9s %s" % ("", "! " + str(note).replace("\n",
                                                                  " ")[:180]))
        print("-" * 108)


def print_summary(cells):
    print()
    print("VERDICT   (N/E is neither a pass nor a failure and does not set "
          "the exit code)")
    sims = sorted({c["sim"] for c in cells})
    print("  %-6s %s" % ("rung", "  ".join("%-22s" % s for s in sims)))
    for rung in sorted({c["rung"] for c in cells}):
        row = []
        for s in sims:
            hit = [c for c in cells if c["rung"] == rung and c["sim"] == s]
            if not hit:
                row.append("%-22s" % "- (not run)")
                continue
            c = hit[0]
            live = [k for k in c["checks"] if not k.get("not_expressible")]
            bad = [k["name"] for k in live if not k["ok"]]
            ne = len(c["checks"]) - len(live)
            if c["verdict"] == "FAIL":
                text = "FAIL(%s)" % bad[0] if bad else "FAIL"
            elif c["verdict"] == "PARTIAL":
                text = "PARTIAL(%d/%d)" % (len(live), len(c["checks"]))
            elif c["verdict"] == "NOT_EXPRESSIBLE":
                text = "N/E(%d)" % ne
            else:
                text = "PASS"
            row.append("%-22s" % text)
        print("  %-6d %s" % (rung, "  ".join(row)))


def _collision_banner(name, exc):
    return ("\n" + "!" * 76
            + "\nARM IMPORT COLLISION while loading %r -- NOTHING WAS "
              "MEASURED.\n%s\n" % (name, exc) + "!" * 76)


def print_import_hazards():
    """Name every arm-local module that a later arm's bare import could grab.

    Nothing is wrong yet -- the arm holding it is holding its own module -- but
    this is the exact precondition for ``ArmImportCollision``, and it is worth
    seeing before a fourth arm arrives rather than after.
    """
    if not IMPORT_HAZARDS:
        return
    print()
    print("IMPORT HAZARDS (not a failure -- an arm-local module is registered "
          "under a name another arm could collide with):")
    for h in IMPORT_HAZARDS:
        print("  %s" % h)
    print("  fix: load siblings by path under an arm-qualified name "
          "(webots/arm.py _load_sibling)")


def print_timings(cells):
    print()
    print("WALL CLOCK (seconds) -- startup is process spawn -> first "
          "completed step")
    print("  %-9s %-5s %10s %10s %10s" % ("sim", "rung", "startup", "step",
                                          "total"))
    for c in cells:
        t = c["timing"]
        print("  %-9s %-5d %10s %10s %10s"
              % (c["sim"], c["rung"], _f(t.get("startup_s"), 4),
                 _f(t.get("step_s"), 4), _f(t.get("total_s"), 4)))
    for sim in sorted({c["sim"] for c in cells}):
        vals = [c["timing"].get("startup_s") for c in cells
                if c["sim"] == sim and c["timing"].get("startup_s")]
        if len(vals) >= 2:
            print("  %-9s median startup %.3f s over %d rungs"
                  % (sim, statistics.median(vals), len(vals)))


# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sims", default="all",
                    help="comma list of %s, or 'all'" % ",".join(ARM_NAMES))
    ap.add_argument("--rungs", default="all",
                    help="comma list, or 'all' (the every-commit set %s) or "
                         "'everything' (adds the on-demand rungs %s, which "
                         "cost minutes rather than seconds)"
                         % (",".join(map(str, rungs.RUNGS)),
                            ",".join(map(str, rungs.RUNGS_ON_DEMAND))))
    ap.add_argument("--out", default=None, help="results dir")
    ap.add_argument("--omnisim-bin", default=None,
                    help="engine binary for the omnisim arm (A/B work)")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--repeat", type=int, default=1,
                    help="repeat each cell N times; the LAST run is reported "
                         "and every run's timing is kept")
    ap.add_argument("--regen-worlds", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--no-live", action="store_true",
                    help="--self-test: skip the engine-backed red proofs")
    ap.add_argument("--list-binaries", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # ABSOLUTE, always.  An arm hands this path to a child process whose
    # working directory it does not control -- an engine runs its controllers
    # from the controller's own directory -- so a relative --out silently
    # writes the whole run somewhere else and reads back as "the controller
    # produced nothing".  Measured: it built a phantom
    # controllers/ladder0_probe/tests/benchmarks/... tree.
    out_root = os.path.abspath(args.out or os.path.join(RESULTS, stamp))
    os.makedirs(out_root, exist_ok=True)

    if args.list_binaries:
        arm = load_arm("omnisim")
        for b in (arm.which_binaries() if arm else []):
            print("%-34s %-20s %s  %d bytes"
                  % (b["name"], b["mtime"], b["sha256"], b["size"]))
        return 0

    if args.regen_worlds:
        for name in ARM_NAMES:
            # ``scenes.py`` as well as ``worldgen.py``: the MuJoCo arm calls
            # its emitter the former, and while this loop knew only the latter
            # its committed models sat at a floor size from an earlier
            # revision of the contract -- a scene judged against an
            # expectation it no longer matches.
            for filename in ("worldgen.py", "scenes.py"):
                path = os.path.join(HERE, name, filename)
                if not os.path.exists(path):
                    continue
                spec = importlib.util.spec_from_file_location(
                    "ladder0_worldgen_%s" % name, path)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                print("[%s] %s" % (name, filename))
                written = mod.write_all() or []
                print("  %d scene file(s) regenerated from rungs.py"
                      % len(written))
                break
        if not args.self_test and args.sims == "all" and not args.json:
            return 0

    want_sims = (list(ARM_NAMES) if args.sims == "all"
                 else [s.strip() for s in args.sims.split(",") if s.strip()])
    if args.rungs == "all":
        want_rungs = list(rungs.RUNGS)
    elif args.rungs in ("everything", "ondemand", "on-demand"):
        want_rungs = (list(rungs.ALL_RUNGS) if args.rungs == "everything"
                      else list(rungs.RUNGS_ON_DEMAND))
    else:
        want_rungs = [int(s) for s in args.rungs.split(",") if s.strip()]

    run_kw = {"timeout_s": args.timeout}
    if args.omnisim_bin:
        run_kw["binary"] = os.path.abspath(args.omnisim_bin)

    if args.self_test:
        arm = None
        if not args.no_live:
            for name in want_sims:
                try:
                    a = load_arm(name)
                except ArmImportCollision as exc:
                    print(_collision_banner(name, exc), file=sys.stderr)
                    return 2
                if a is not None and a.available()[0]:
                    arm = a
                    break
        kw = dict(run_kw) if (arm is not None and arm.NAME == "omnisim") \
            else {}
        cases, failed = selftest.run_self_test(out_root, arm=arm,
                                               only_rungs=set(want_rungs),
                                               **kw)
        pure = [c for c in cases if c["kind"] != "live"]
        live = [c for c in cases if c["kind"] == "live"]
        print("SELF-TEST -- can every assertion go red?")
        print("=" * 96)
        print("A. assertion mutation (no simulator): %d cases, %d failed"
              % (len(pure), len([c for c in pure if not c["ok"]])))
        for c in pure:
            if not c["ok"]:
                print("   FAIL rung %s %-20s %-14s %s"
                      % (c["rung"], c["target"], c["kind"], c["detail"]))
        by_kind = {}
        for c in pure:
            by_kind.setdefault(c["kind"], []).append(c)
        for k in sorted(by_kind):
            n = len(by_kind[k])
            bad = len([c for c in by_kind[k] if not c["ok"]])
            print("   %-16s %3d cases, %d failed" % (k, n, bad))
        print()
        if live:
            print("B. live fault injection (%s, end to end): %d cases, "
                  "%d failed" % (arm.NAME if arm else "-", len(live),
                                 len([c for c in live if not c["ok"]])))
            for c in live:
                print("   [%s] rung %s %-12s %s"
                      % ("red-as-required" if c["ok"] else "DID NOT GO RED",
                         c["rung"], c.get("fault", "-"), c["detail"]))
        else:
            print("B. live fault injection: SKIPPED -- a self-test that only "
                  "mutates its own arithmetic has not shown the ENGINE path "
                  "can fail")
        validation = selftest.detector_validation(cases)
        print()
        print("C. detector validation (CONTRACT.md amendment C) -- a green "
              "that cannot be made RED is not a pass")
        for rung in sorted(validation):
            v = validation[rung]
            print("   rung %-3s %-14s %s"
                  % (rung, v["status"],
                     v.get("why", "%d/%d declared faults went red"
                           % (v.get("ran", 0), v.get("declared", 0)))))
        with open(os.path.join(out_root, "selftest.json"), "w",
                  encoding="utf-8") as f:
            json.dump({"cases": cases, "detector_validation":
                       {str(k): v for k, v in validation.items()}}, f, indent=1)
        print()
        print("self-test: %s (%d/%d cases behaved as required)"
              % ("PASS" if not failed else "FAIL", len(cases) - len(failed),
                 len(cases)))
        return 0 if not failed else 1

    cells, absent = [], []
    for name in want_sims:
        try:
            arm = load_arm(name)
        except ArmImportCollision as exc:
            print(_collision_banner(name, exc), file=sys.stderr)
            return 2
        if arm is None:
            absent.append((name, "no %s/arm.py in the tree yet" % name))
            continue
        ok, why = arm.available()
        if not ok:
            absent.append((name, why))
            continue
        kw = dict(run_kw)
        if name != "omnisim":
            kw.pop("binary", None)
        # An arm may declare a rung UNIMPLEMENTED.  That is a third state and
        # it is not NOT_EXPRESSIBLE: N/E means the SIMULATOR structurally
        # cannot express the quantity, and is declared per check with a
        # citation; this means nobody has written the code yet.  Reporting the
        # second as the first would blame an engine for our backlog, and
        # reporting either as a failure would be worse.  It is an UNKNOWN, and
        # an unknown is not a pass.
        unimpl = dict(getattr(arm, "UNIMPLEMENTED", {}) or {})
        for rung in want_rungs:
            if rung in unimpl:
                absent.append(("%s rung %d" % (name, rung),
                               "not implemented on this arm: %s"
                               % unimpl[rung]))
                continue
            cell = None
            for _ in range(max(1, args.repeat)):
                cell = run_cell(arm, rung, out_root, **kw)
            cells.append(cell)

    cross_cell_checks(cells)
    print_table(cells)
    print_summary(cells)
    print_timings(cells)
    if absent:
        print()
        print("ARMS NOT RUN (not a pass -- an unrun arm is an unknown):")
        for name, why in absent:
            print("  %-9s %s" % (name, why))
    print_import_hazards()

    doc = {"stamp": stamp,
           "cells": [{k: v for k, v in c.items() if k != "_m"} for c in cells],
           "absent": [{"sim": n, "why": w} for n, w in absent],
           "import_hazards": list(IMPORT_HAZARDS),
           "contract": "tests/benchmarks/ladder0/rungs.py"}
    with open(os.path.join(out_root, "ladder.json"), "w",
              encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    if args.json:
        print(json.dumps(doc, indent=1))
    print()
    print("results: %s" % out_root)
    # PARTIAL does not set the exit code: an N/E check is a declared absence of
    # a capability, not a failed measurement, and treating it as red would make
    # the honest verdict the expensive one to report.
    return 0 if (cells and not any(c["verdict"] == "FAIL" for c in cells)) \
        else 1


if __name__ == "__main__":
    sys.exit(main())
