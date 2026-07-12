"""N-run band calibration for install conformance.

Runs each SOFT-metric demo N times on THIS host, computes per-metric stats, and
derives generous tolerance bands written to a committed overlay
(`calibration.json`) that the runner applies on top of the hand-authored
manifest defaults. This is **within-host** calibration (N repeated runs on one
machine) -- the right first step. Multi-host hardening is the CI follow-up: run
this on diverse machines and take the union of the bands.

Band derivation is deliberately generous (sigma multiplier _K, plus floors and
headroom) so normal cross-machine drift reports DRIFT, not FAIL.
"""

from __future__ import annotations

import json
import statistics
import tempfile
import time
from pathlib import Path

from ..paths import resolve_webots_binary
from . import checks as checks_mod
from . import fingerprint as fp_mod
from .runner import load_manifest

CALIB_PATH = Path(__file__).resolve().parent / "calibration.json"
CALIB_SCHEMA = "omnisim.install_check.calibration/v1"
_K = 3.0          # sigma multiplier for band half-width
_INTER_RUN_S = 6.0  # settle gap between runs so back-to-back harness spawns don't contend


def _derive_band(existing: dict, vals: list[float]) -> dict:
    """Derive a band of the SAME shape as `existing`, fit to the samples.

    WIDEN-ONLY relative to the hand-authored default: single-host calibration
    must never make a band *stricter* than the cross-machine-safe default (a
    zero-variance host would otherwise collapse the band and false-FAIL other
    machines). We only (a) anchor a two-sided `expected` to the measured mean,
    and (b) loosen where this host shows variance.
    """
    mean = statistics.fmean(vals)
    sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    lo, hi = min(vals), max(vals)
    band = dict(existing)
    if "expected" in band:  # two-sided (e.g. mean_brightness): anchor + widen
        tol = max(existing.get("abs_tol", 0.0), _K * sd, abs(mean) * 0.15, 5.0)
        band["expected"] = round(mean, 2)
        band["abs_tol"] = round(tol, 2)
        band["drift_abs_tol"] = round(max(existing.get("drift_abs_tol", 0.0), tol * 2), 2)
    elif "band_max" in band:  # one-sided upper: never lower than the default
        derived = max(hi + max(1.0, 0.25 * hi), mean + _K * sd)
        bm = max(existing.get("band_max", 0.0), derived)
        band["band_max"] = round(bm, 2)
        band["drift_max"] = round(max(existing.get("drift_max", 0.0), bm * 2, bm + 10), 2)
    elif "band_min" in band:  # one-sided lower: never higher than the default
        derived = max(0.0, min(lo, mean - _K * sd)) * 0.6
        band["band_min"] = round(min(existing.get("band_min", derived), derived), 2)
        band["drift_min"] = round(min(existing.get("drift_min", band["band_min"] * 0.2),
                                      band["band_min"] * 0.2), 2)
    return band


def calibrate(n: int = 3, report_dir=None) -> dict:
    """Run SOFT-metric demos N times, derive bands, write calibration.json."""
    manifest = load_manifest()
    ctx = {
        "report_dir": Path(report_dir) if report_dir
        else Path(tempfile.mkdtemp(prefix="omnisim-calib-")),
        "binary": resolve_webots_binary(),
    }
    ctx["report_dir"].mkdir(parents=True, exist_ok=True)

    bands_out: dict = {}
    stats_out: dict = {}
    summary: list = []
    skipped: list = []

    for demo in manifest.get("demos", []):
        soft = [m for m in demo.get("metrics", []) if m.get("severity") == "SOFT"]
        if not soft:
            continue
        series = {m["name"]: [] for m in soft}
        for i in range(n):
            measured = checks_mod.run_check(demo, ctx).get("measured", {})
            for m in soft:
                v = measured.get(m["source"])
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    series[m["name"]].append(float(v))
            if i + 1 < n:
                time.sleep(_INTER_RUN_S)  # let the prior sim/harness fully release
        for m in soft:
            vals = series[m["name"]]
            if len(vals) < 2:
                skipped.append((demo["id"], m["name"], len(vals)))
                continue
            existing = demo.get("tolerance_bands", {}).get(m["name"], {}) or {}
            ref = existing.get("any") or next(iter(existing.values()), {})
            new = _derive_band(ref, vals)
            bands_out.setdefault(demo["id"], {})[m["name"]] = {"any": new}
            st = {
                "mean": round(statistics.fmean(vals), 3),
                "sd": round(statistics.pstdev(vals), 3) if len(vals) > 1 else 0.0,
                "min": round(min(vals), 3), "max": round(max(vals), 3), "n": len(vals),
            }
            stats_out.setdefault(demo["id"], {})[m["name"]] = st
            summary.append((demo["id"], m["name"], ref, new, st))

    fp = fp_mod.collect(scrub_paths=True)
    calib = {
        "schema": CALIB_SCHEMA,
        "runs": n,
        "commit": fp.get("build"),
        "calibrated_on": {
            "fingerprint_id": fp.get("fingerprint_id"),
            "resolved_backend": fp.get("resolved_backend"),
            "gpu": fp.get("gpu"),
            "os": fp.get("os"),
            "render_backend": fp.get("render_backend"),
            "note": "within-host (single-machine) N-run calibration; harden multi-host in CI",
        },
        "bands": bands_out,
        "stats": stats_out,
    }
    CALIB_PATH.write_text(json.dumps(calib, indent=2), encoding="utf-8")
    return {"path": str(CALIB_PATH), "summary": summary, "skipped": skipped, "calib": calib}
