"""Band comparator + roll-up classifier for install conformance.

Turns measured metric values into PASS / PASS-WITH-DRIFT / FAIL, honoring the
design's core rule (docs/developer/install-conformance.md §5.2): only HARD
metrics (bit-exact canaries + load liveness) can FAIL a demo; SOFT metrics in
their drift band are DRIFT, never broken; harness-sourced metrics are PENDING
until Phase 1.5.
"""

from __future__ import annotations

# Severities (from the manifest)
HARD, SOFT, INFO = "HARD", "SOFT", "INFO"
# Per-metric / per-demo / per-install statuses
PASS, DRIFT, FAIL, SKIP, PENDING, ERROR = "PASS", "DRIFT", "FAIL", "SKIP", "PENDING", "ERROR"

INSTALL_PASS = "PASS"
INSTALL_DRIFT = "PASS-WITH-DRIFT"
INSTALL_FAIL = "FAIL"


def _select_band(bands: dict, metric_name: str, backend: str):
    by_backend = bands.get(metric_name)
    if not by_backend:
        return None
    if backend in by_backend:
        return by_backend[backend]
    return by_backend.get("any")


def _apply_band(band: dict, value):
    """Return (status, human detail) for a measured value against one band."""
    if "skip" in band and band["skip"]:
        return SKIP, "skipped for this backend"
    if "equals" in band:
        ok = value == band["equals"]
        return (PASS if ok else FAIL), f"expected == {band['equals']}, got {value}"
    if "max" in band:
        return (PASS if value <= band["max"] else FAIL), f"expected <= {band['max']}, got {value}"
    if "min" in band:
        return (PASS if value >= band["min"] else FAIL), f"expected >= {band['min']}, got {value}"
    if "band_max" in band:
        drift = band.get("drift_max", band["band_max"])
        if value <= band["band_max"]:
            return PASS, f"{value} <= {band['band_max']}"
        if value <= drift:
            return DRIFT, f"{value} in drift (<= {drift})"
        return FAIL, f"{value} beyond drift (> {drift})"
    if "band_min" in band:
        drift = band.get("drift_min", band["band_min"])
        if value >= band["band_min"]:
            return PASS, f"{value} >= {band['band_min']}"
        if value >= drift:
            return DRIFT, f"{value} in drift (>= {drift})"
        return FAIL, f"{value} beyond drift (< {drift})"
    if "expected" in band:
        delta = abs(value - band["expected"])
        tol = band.get("abs_tol", 0)
        drift_tol = band.get("drift_abs_tol", tol)
        if delta <= tol:
            return PASS, f"|{value} - {band['expected']}| <= {tol}"
        if delta <= drift_tol:
            return DRIFT, f"|{value} - {band['expected']}| in drift (<= {drift_tol})"
        return FAIL, f"|{value} - {band['expected']}| beyond drift (> {drift_tol})"
    return SKIP, "no recognized band rule"


def evaluate_metric(mdef: dict, measured: dict, backend: str, bands: dict) -> dict:
    name = mdef["name"]
    severity = mdef["severity"]
    source = mdef["source"]
    out = {"name": name, "severity": severity, "source": source, "value": None}

    if source.startswith("harness:"):
        out.update(status=PENDING, detail="harness metric - Phase 1.5")
        return out

    value = measured.get(source)
    out["value"] = value
    if severity == INFO:
        # INFO metrics are recorded for the report, never banded or gated.
        out.update(status=INFO, detail="recorded")
        return out
    band = _select_band(bands, name, backend)
    if band is None:
        out.update(status=SKIP, detail=f"no band for backend '{backend}'")
        return out
    if value is None:
        out.update(status=(ERROR if severity == HARD else SKIP), detail="no measured value")
        return out
    status, detail = _apply_band(band, value)
    out.update(status=status, detail=detail)
    return out


def classify_demo(check: dict, result: dict) -> dict:
    """Roll metric statuses up to a single demo verdict."""
    if result.get("status") == "error":
        return {
            "id": check["id"], "status": FAIL, "lane": check.get("lane"),
            "subsystem": check.get("subsystem"),
            "reason": result.get("error", "check error"), "metrics": [],
            "resolved_backend": result.get("resolved_backend", "unknown"),
        }
    if result.get("status") == "skipped":
        # A skipped demo (e.g. the harness was unavailable) never fails the
        # install — it just isn't counted.
        return {
            "id": check["id"], "status": SKIP, "lane": check.get("lane"),
            "subsystem": check.get("subsystem"),
            "reason": result.get("reason", "skipped"), "metrics": [],
            "resolved_backend": result.get("resolved_backend", "any"),
        }

    backend = result.get("resolved_backend", "unknown")
    measured = result.get("measured", {})
    bands = check.get("tolerance_bands", {})
    metrics = [evaluate_metric(m, measured, backend, bands) for m in check.get("metrics", [])]

    status = PASS
    if any(m["status"] == FAIL for m in metrics) or \
       any(m["status"] == ERROR and m["severity"] == HARD for m in metrics):
        status = FAIL
    elif any(m["status"] == DRIFT for m in metrics):
        status = DRIFT

    demo = {
        "id": check["id"], "status": status, "lane": check.get("lane"),
        "subsystem": check.get("subsystem"), "resolved_backend": backend,
        "metrics": metrics,
    }
    if result.get("note"):
        demo["note"] = result["note"]
    return demo


def classify_install(demo_results: list, advisories: list) -> str:
    if any(d["status"] == FAIL for d in demo_results):
        return INSTALL_FAIL
    drift = any(d["status"] == DRIFT for d in demo_results)
    drift = drift or any(a.get("severity") == "drift" for a in advisories)
    return INSTALL_DRIFT if drift else INSTALL_PASS
