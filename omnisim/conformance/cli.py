"""`omnisim verify-install` — the non-gating install conformance reporter (Phase 1).

Wired from ``omnisim.cli``. Runs a small acceptance manifest of leading demos,
compares each metric to its band, and reports PASS / PASS-WITH-DRIFT / FAIL.
It does NOT block any launch (the first-run gate is Phase 3 — see
docs/developer/install-conformance.md §2).

Exit codes: 0 = PASS or PASS-WITH-DRIFT (drift is non-fatal by design),
1 = FAIL, 3 = DRIFT under --strict (for CI lanes that want drift to be loud).

stdout carries ONLY the report (human text or JSON); all run-time chatter (the
headless subprocesses, progress lines) is routed to stderr so
``verify-install --json | jq`` is a clean machine-readable contract.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import sys

from . import compare, fingerprint, report, runner


@contextlib.contextmanager
def _chatter_to_stderr():
    """Redirect fd 1 -> fd 2 for the duration, so child-process stdout (the
    sim / headless_runner banners) lands on stderr, not on our report stdout.
    Degrades to a no-op if the fds aren't real (e.g. captured by a harness)."""
    try:
        sys.stdout.flush()
        saved = os.dup(1)
    except (OSError, ValueError, AttributeError):
        yield
        return
    try:
        os.dup2(2, 1)
        yield
    finally:
        try:
            sys.stdout.flush()
            os.dup2(saved, 1)
            os.close(saved)
        except OSError:
            pass


def run(args: argparse.Namespace) -> int:
    if getattr(args, "never_ask", False):
        from .gate import write_skip_file
        ok = write_skip_file()
        print("conformance gate disabled for this clone (skip file written)."
              if ok else "could not write the skip file.")
        return 0

    if getattr(args, "gate", False) or getattr(args, "post_build", False):
        # --gate: run iff no escape condition (used by the interactive launch
        # path). --post-build: same, but advisory -- never blocks.
        from .gate import gate as run_gate
        rc = run_gate([], blocking=not getattr(args, "post_build", False))
        # Exit 42 on a DELIBERATE blocking FAIL so a launcher (launch.bat) can
        # block only on that, never on a python/import problem (which must fail
        # OPEN). Any other outcome -> 0 (proceed).
        return 42 if rc != 0 else 0

    if getattr(args, "calibrate", False):
        from . import calibrate as calib_mod
        n = max(2, int(getattr(args, "runs", 3) or 3))
        with _chatter_to_stderr():
            out = calib_mod.calibrate(n=n)
        print(f"calibration written: {out['path']}  (runs={n})\n")
        for demo, metric, old, new, st in out["summary"]:
            print(f"  {demo}.{metric}: mean={st['mean']} sd={st['sd']} "
                  f"min={st['min']} max={st['max']} n={st['n']}")
            print(f"     old: {old}")
            print(f"     new: {new}")
        for demo, metric, k in out.get("skipped", []):
            print(f"  SKIP {demo}.{metric}: only {k} sample(s) collected")
        return 0

    if getattr(args, "fingerprint_only", False):
        fp = fingerprint.collect(scrub_paths=True)
        print(report.fingerprint_json(fp) if args.json else report.format_fingerprint(fp))
        return 0

    if getattr(args, "deep", False):
        print("verify-install --deep (the opt-in deep lane: grasp, wgpu golden, "
              "physics oracles) is not implemented yet - it is Phase 4.\n"
              "See docs/developer/install-conformance.md §3.2. The deep lane never "
              "gates an install.", file=sys.stderr)
        return 0

    lane = "fast" if getattr(args, "fast", False) else "core"

    def _progress(check):
        print(f"[verify-install] running {check['id']} ({check.get('run_mode')}) ...",
              file=sys.stderr, flush=True)

    with _chatter_to_stderr():
        result = runner.run(lane=lane, progress=_progress)

    if getattr(args, "report", False):
        result["bundle"] = report.write_bundle(result)

    if args.json:
        print(report.to_json(result))
    else:
        print(report.format_human(result))
        if result.get("bundle"):
            print(f"\nreport bundle: {result['bundle']['json']}")

    status = result["status"]
    if status == compare.INSTALL_FAIL:
        return 1
    if status == compare.INSTALL_DRIFT and getattr(args, "strict", False):
        return 3
    return 0
