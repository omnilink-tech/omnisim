"""Check executors for install conformance.

Each function runs ONE manifest demo and returns the raw measured metric
values (keyed by the manifest ``source`` strings); the band comparison happens
later in ``compare.py``. Two run modes are implemented in Phase 1:

* ``headless`` — launch the world via the supported ``headless_runner.py``
  contract (warm by default), then scrape the per-check engine log for the
  error/warning counts and the authoritative ``world finalised (solver=...)``
  line. Reuses ``omnisim.dev.run_headless`` so the cold/warm + DLL-PATH launch
  logic has a single source of truth.
* ``generate`` — the omniworld byte-determinism canary: generate the same
  ``(recipe, seed)`` twice in two subprocesses and compare sha256.

The ``harness`` run mode (live displacement / render_stats via the validation
harness) is Phase 1.5; its metrics are sourced ``harness:...`` and reported
PENDING by the comparator until that path lands.
"""

from __future__ import annotations

import hashlib
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from ..paths import REPO_ROOT
from ..dev import commands as dev
from . import fingerprint as fp_mod

_OMNIWORLD = REPO_ROOT / "scripts" / "dev" / "omniworld.py"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _scan_log(text: str):
    """(scan dict, error_count, warning_count) for an engine-log body."""
    mod = fp_mod._load_envfp()
    scan: dict = {}
    if mod is not None:
        try:
            scan = mod._scan_engine_log(text)
        except Exception:
            scan = {}
    errors = warnings = 0
    for line in text.splitlines():
        if line.startswith("ERROR:") or line.startswith("FATAL:"):
            errors += 1
        elif line.startswith("WARNING:"):
            warnings += 1
    return scan, errors, warnings


def _free_port() -> int:
    """Grab an OS-assigned free TCP port. The sim binds it a moment later
    (small TOCTOU window), but this avoids the default-1234 collision that
    tears down a second OmniSim instance (AGENTS.md §3e)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])
    finally:
        s.close()


def _one_headless_attempt(world: str, duration: int, log_path: Path) -> dict:
    """One headless launch on its own port + log. Returns measured raw fields."""
    # Per-check log so sequential checks don't clobber each other and we scrape
    # the right one (the parallel-runs convention in AGENTS.md §3e).
    prev = os.environ.get("OMNISIM_LOG_PATH")
    os.environ["OMNISIM_LOG_PATH"] = str(log_path)
    try:
        port = _free_port()
        exit_code = dev.run_headless(world, duration=duration, fail_on_warning=False,
                                     extra_args=[f"--port={port}"])
    except SystemExit as exc:  # require_world / binary resolution inside dev
        return {"launch_error": f"launch_failed: {exc}"}
    finally:
        if prev is None:
            os.environ.pop("OMNISIM_LOG_PATH", None)
        else:
            os.environ["OMNISIM_LOG_PATH"] = prev

    text = ""
    try:
        text = log_path.read_text(errors="replace")
    except OSError:
        pass
    scan, errors, warnings = _scan_log(text)
    return {"exit_code": exit_code, "text": text, "scan": scan,
            "errors": errors, "warnings": warnings}


def run_headless_check(check: dict, ctx: dict) -> dict:
    """Launch a world headlessly and measure liveness + solver metrics.

    Retries once on a fresh port if the first launch exits non-zero — a cold
    launch can lose a port race or tear down on transient contention, and a
    HARD liveness check must not false-FAIL on that (see the design's cold-load
    note). A note records what happened so a real failure stays actionable.
    """
    if not ctx.get("binary"):
        return {"status": "error", "error": "binary_missing", "measured": {},
                "resolved_backend": "unknown"}

    world = check["world"]
    duration = int(check.get("run_args", {}).get("duration_s", 12))
    base_log = Path(ctx["report_dir"]) / f"{check['id']}.log"

    note = None
    attempt = {}
    max_attempts = 2
    for i in range(max_attempts):
        log_path = base_log if i == 0 else base_log.with_name(f"{check['id']}.retry{i}.log")
        attempt = _one_headless_attempt(world, duration, log_path)
        if attempt.get("launch_error"):
            return {"status": "error", "error": attempt["launch_error"], "measured": {},
                    "resolved_backend": "unknown"}
        if attempt["exit_code"] == 0:
            break
        if i + 1 < max_attempts:
            kind = "no engine log" if not attempt["text"] else \
                "early teardown (likely a TCP port collision with another OmniSim)"
            note = f"first attempt exit_code={attempt['exit_code']} - {kind}; retried on a fresh port"

    exit_code = attempt["exit_code"]
    scan = attempt["scan"]
    load_ok = exit_code == 0
    backend = fp_mod.resolved_backend({"scan": scan})
    if not load_ok and note is None:
        note = f"exit_code={exit_code}" + (
            "" if attempt["text"] else "; no engine log produced - sim never started "
            "(check binary / PATH / DLLs)")

    measured = {
        "headless.load_ok": load_ok,
        "headless.exit_code": exit_code,
        "headless.error_count": attempt["errors"],
        "headless.warning_count": attempt["warnings"],
        "headless.finalised": bool(scan.get("finalised")),
        "headless.solver": scan.get("solver"),
    }
    res = {"status": "ran", "resolved_backend": backend,
           "measured": measured, "log_path": str(base_log)}
    if note:
        res["note"] = note
    return res


def run_generate_check(check: dict, ctx: dict) -> dict:
    """The omniworld byte-determinism canary (generate twice, compare sha256)."""
    spec = check["world"]
    recipe = str(spec["recipe"])
    seed = str(spec["seed"])
    out_dir = Path(ctx["report_dir"])
    a = out_dir / f"{check['id']}_a.wbt"
    b = out_dir / f"{check['id']}_b.wbt"

    def _generate(dest: Path):
        cmd = [sys.executable, str(_OMNIWORLD), "generate", recipe,
               "--seed", seed, "--out", str(dest)]
        return subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)

    ra = _generate(a)
    rb = _generate(b)
    gen_ok = ra.returncode == 0 and rb.returncode == 0 and a.exists() and b.exists()

    sha_match = _sha256(a) == _sha256(b) if gen_ok else False

    validate_ok = False
    if a.exists():
        vr = subprocess.run([sys.executable, str(_OMNIWORLD), "validate", str(a)],
                            cwd=REPO_ROOT, capture_output=True, text=True)
        validate_ok = vr.returncode == 0

    measured = {
        "omniworld.sha_match": sha_match,
        "omniworld.validate_ok": validate_ok,
        "omniworld.gen_ok": gen_ok,
    }
    res = {"status": "ran" if gen_ok else "error", "resolved_backend": "any",
           "measured": measured}
    if not gen_ok:
        res["error"] = (ra.stderr or rb.stderr or "omniworld generate failed").strip()[:300]
    return res


def _max_displacement(r0: dict, r1: dict) -> float:
    """Largest per-robot euclidean pose displacement between two /robots reads."""
    p0 = {x.get("def"): x.get("position") for x in r0.get("robots", [])}
    p1 = {x.get("def"): x.get("position") for x in r1.get("robots", [])}
    best = 0.0
    for d, a in p0.items():
        b = p1.get(d)
        if a and b and len(a) >= 3 and len(b) >= 3:
            best = max(best, sum((b[i] - a[i]) ** 2 for i in range(3)) ** 0.5)
    return round(best, 3)


def run_harness_check(check: dict, ctx: dict) -> dict:
    """Drive the validation harness: load, free-run on wall-clock, measure
    render exposure + robot displacement. All metrics are SOFT; any harness
    fragility returns status 'skipped' so it can never FAIL the install."""
    if not ctx.get("binary"):
        return {"status": "skipped", "reason": "binary_missing", "measured": {}}

    from .harness_client import HarnessSession, HarnessError

    world = check["world"]
    settle_s = float(check.get("run_args", {}).get("settle_s", 2.0))
    observe_s = float(check.get("run_args", {}).get("observe_s", 6.0))
    try:
        with HarnessSession() as h:
            code, res = h.load(world)
            if code != 200 or not res.get("ok"):
                return {"status": "skipped",
                        "reason": f"harness /world/load failed (code {code})",
                        "measured": {}}
            time.sleep(settle_s)
            _, r0 = h.robots()
            werr = h.world_error_count()
            time.sleep(observe_s)
            _, r1 = h.robots()
            disp = _max_displacement(r0, r1)
            stats = {}
            try:
                sc, body = h.render_stats()
                if sc == 200:
                    stats = body
            except Exception:
                stats = {}
            measured = {
                "harness.load_ok": True,
                "harness.world_errors": werr,
                "harness.displacement_m": disp,
                "harness.mean_brightness": stats.get("mean_brightness"),
                "harness.black_pct": stats.get("black_pct"),
                "harness.saturated_pct": stats.get("saturated_pct"),
            }
            return {"status": "ran", "resolved_backend": "any", "measured": measured}
    except HarnessError as exc:
        return {"status": "skipped", "reason": f"harness unavailable: {exc}", "measured": {}}
    except Exception as exc:  # defensive: a harness hiccup must never FAIL the install
        return {"status": "skipped", "reason": f"harness error: {exc}", "measured": {}}


def run_check(check: dict, ctx: dict) -> dict:
    """Dispatch on run_mode."""
    mode = check.get("run_mode")
    if mode == "headless":
        return run_headless_check(check, ctx)
    if mode == "generate":
        return run_generate_check(check, ctx)
    if mode == "harness":
        return run_harness_check(check, ctx)
    return {"status": "error", "error": f"unsupported run_mode: {mode}",
            "measured": {}, "resolved_backend": "unknown"}
