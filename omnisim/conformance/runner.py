"""Acceptance-manifest runner: load manifest, run demos, classify, assemble result.

Phase 1 is a NON-GATING reporter — it never blocks a launch. It returns a
structured result (schema ``omnisim.install_check/v1``) that the CLI renders.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

from ..paths import resolve_webots_binary
from . import checks as checks_mod
from . import compare
from . import fingerprint as fp_mod
from .advisor import advise

_MANIFEST = Path(__file__).resolve().parent / "manifest.json"
_CALIB = Path(__file__).resolve().parent / "calibration.json"
RESULT_SCHEMA = "omnisim.install_check/v1"


def load_manifest(path=None) -> dict:
    with open(Path(path) if path else _MANIFEST, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _load_calibration():
    try:
        return json.loads(_CALIB.read_text(encoding="utf-8"))
    except Exception:
        return None


def _apply_calibration(manifest: dict, calib: dict | None) -> bool:
    """Overlay calibrated bands onto the manifest's hand-authored defaults."""
    if not calib or not calib.get("bands"):
        return False
    for demo in manifest.get("demos", []):
        overrides = calib["bands"].get(demo["id"])
        if not overrides:
            continue
        tb = demo.setdefault("tolerance_bands", {})
        for metric, band in overrides.items():
            tb[metric] = band
    return True


def select_demos(manifest: dict, lane: str = "core") -> list:
    demos = manifest.get("demos", [])
    if lane == "fast":
        return [d for d in demos if d.get("fast")]
    if lane == "deep":
        return [d for d in demos if d.get("lane") == "deep"]
    return [d for d in demos if d.get("lane", "core") == "core"]


def run(lane: str = "core", manifest_path=None, report_dir=None, progress=None) -> dict:
    manifest = load_manifest(manifest_path)
    calib = _load_calibration()
    calibrated = _apply_calibration(manifest, calib)
    demos = select_demos(manifest, lane)

    fp = fp_mod.collect(scrub_paths=True)
    binary = resolve_webots_binary()
    rdir = Path(report_dir) if report_dir else Path(tempfile.mkdtemp(prefix="omnisim-conformance-"))
    rdir.mkdir(parents=True, exist_ok=True)
    ctx = {"report_dir": rdir, "binary": binary}

    demo_results = []
    observed_backend = None
    for check in demos:
        if progress:
            progress(check)
        raw = checks_mod.run_check(check, ctx)
        backend = raw.get("resolved_backend")
        if observed_backend is None and backend not in (None, "unknown", "any"):
            observed_backend = backend
        demo_results.append(compare.classify_demo(check, raw))

    advisories = advise(fp, demo_results, observed_backend)
    status = compare.classify_install(demo_results, advisories)

    return {
        "schema": RESULT_SCHEMA,
        "status": status,
        "lane": lane,
        "manifest_schema": manifest.get("schema"),
        "manifest_owner": manifest.get("owner"),
        "calibrated": calibrated,
        "calibration": ({
            "commit": calib.get("commit"),
            "runs": calib.get("runs"),
            "fingerprint_id": (calib.get("calibrated_on") or {}).get("fingerprint_id"),
        } if calibrated else None),
        "observed_backend": observed_backend or fp.get("resolved_backend"),
        "fingerprint": fp,
        "demos": demo_results,
        "advisories": advisories,
        "report_dir": str(rdir),
    }
