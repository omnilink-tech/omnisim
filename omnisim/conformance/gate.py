"""Phase 3 — the first-run gate: mandatory-but-bypassable on the interactive path.

`gate(argv)` runs the FAST conformance lane before an interactive launch, unless
an escape condition holds (CI/env, batch/headless flags, a valid prior-pass
stamp keyed on the build's fingerprint, or a per-clone skip file). PASS/DRIFT ->
proceed and record the stamp; FAIL -> block with a call-to-action.

Two hard rules from the design (docs/developer/install-conformance.md §2):
  * It is wired ONLY into the interactive human launch path (run-world /
    launch.bat), never into run-headless / harness / agents / CI.
  * It FAILS OPEN: any error inside the gate itself lets the launch proceed.
    A self-test must never become an outage.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

STAMP_DIR = Path.home() / ".omnisim"
STAMP = STAMP_DIR / "conformance.json"
SKIP_FILE = STAMP_DIR / "conformance.skip"
STAMP_SCHEMA = "omnisim.conformance.stamp/v1"

# Any of these in the launch argv means a headless/automation launch -> skip.
_BATCH_FLAGS = ("--batch", "--no-rendering", "--minimize", "--mode=fast")
_TRUTHY_OFF = (None, "", "0", "false", "False")


def _env_skip() -> str | None:
    if os.environ.get("OMNISIM_SKIP_CONFORMANCE") not in _TRUTHY_OFF:
        return "OMNISIM_SKIP_CONFORMANCE set"
    if os.environ.get("CI") not in _TRUTHY_OFF:
        return "CI environment"
    return None


def _argv_skip(argv) -> str | None:
    for arg in argv or []:
        for flag in _BATCH_FLAGS:
            if arg == flag or arg.startswith(flag):
                return f"headless/batch flag {flag}"
    return None


def current_fingerprint_id() -> str | None:
    try:
        from .fingerprint import collect
        return collect(scrub_paths=True).get("fingerprint_id")
    except Exception:
        return None


def read_stamp() -> dict | None:
    try:
        return json.loads(STAMP.read_text(encoding="utf-8"))
    except Exception:
        return None


def write_stamp(result: dict) -> None:
    try:
        STAMP_DIR.mkdir(parents=True, exist_ok=True)
        STAMP.write_text(json.dumps({
            "schema": STAMP_SCHEMA,
            "status": result.get("status"),
            "fingerprint_id": (result.get("fingerprint") or {}).get("fingerprint_id"),
            "lane": result.get("lane"),
        }, indent=2), encoding="utf-8")
    except Exception:
        pass


def _valid_stamp(fp_id: str | None) -> bool:
    stamp = read_stamp()
    return bool(stamp and fp_id
                and stamp.get("fingerprint_id") == fp_id
                and stamp.get("status") in ("PASS", "PASS-WITH-DRIFT"))


def should_skip(argv) -> str | None:
    """Return the reason the gate should clear silently, or None to run it."""
    reason = _env_skip() or _argv_skip(argv)
    if reason:
        return reason
    if SKIP_FILE.exists():
        return "conformance.skip present (--never-ask)"
    if _valid_stamp(current_fingerprint_id()):
        return "valid prior-pass stamp for this build"
    return None


def write_skip_file() -> bool:
    try:
        STAMP_DIR.mkdir(parents=True, exist_ok=True)
        SKIP_FILE.write_text("verify-install --never-ask\n", encoding="utf-8")
        return True
    except Exception:
        return False


def _fail_cta() -> str:
    return (
        "\n  INSTALL CONFORMANCE FAILED -- launch blocked.\n"
        "  WHAT TO DO\n"
        "   1. python -m omnisim verify-install          # see the failure + the fix\n"
        "   2. python -m omnisim doctor                  # ground truth\n"
        "   3. OMNISIM_SKIP_CONFORMANCE=1 <relaunch>     # bypass once\n"
        "      (or: python -m omnisim verify-install --never-ask  to stop asking)\n\n"
    )


def gate(argv=None, *, blocking: bool = True) -> int:
    """0 = proceed with launch, non-zero = block (only on a genuine FAIL when
    blocking). Fails OPEN on any internal error."""
    try:
        reason = should_skip(argv)
        if reason:
            return 0
        from . import report, runner
        result = runner.run(lane="fast")
        if result.get("status") in ("PASS", "PASS-WITH-DRIFT"):
            write_stamp(result)
            return 0
        sys.stderr.write(report.format_human(result) + "\n")
        sys.stderr.write(_fail_cta())
        return 1 if blocking else 0
    except Exception as exc:  # a self-test must never become an outage
        sys.stderr.write(f"[verify-install] gate skipped (internal error: {exc})\n")
        return 0
