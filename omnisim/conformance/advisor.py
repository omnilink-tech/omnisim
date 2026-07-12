"""Remediation advisor: FAIL/DRIFT cause -> exact fix.

A keyed lookup over the fingerprint (+ observed backend) that emits actionable
advisories. Phase 1 implements the highest-value subset of the full table in
docs/developer/install-conformance.md §6.2; more codes land with the gate.
"""

from __future__ import annotations


def advise(fp: dict, demo_results: list, observed_backend: str | None = None) -> list[dict]:
    out: list[dict] = []
    intent = fp.get("intent") or {}
    require_newton = str(intent.get("OMNISIM_REQUIRE_NEWTON", "0")) not in ("0", "", "false", "False")
    backend = observed_backend or fp.get("resolved_backend")

    if not fp.get("omnisim_binary"):
        out.append({
            "code": "BINARY_MISSING", "severity": "fail",
            "message": "OmniSim simulator binary not found.",
            "fix": "Build it: build_omni.bat  (Windows) / python -m omnisim build all  "
                   "(then: make -C src/omnisim bundle-newton-runtime).",
        })

    if backend == "ode" and not fp.get("newton_runtime_present"):
        out.append({
            "code": "NEWTON_FELL_BACK_TO_ODE",
            "severity": "fail" if require_newton else "drift",
            "message": "Physics resolved to ODE and the Newton runtime is not bundled "
                       "next to the binary. ODE is supported, but precise-physics demos "
                       "were tuned on the MuJoCo solver and can differ.",
            "fix": "make -C src/omnisim bundle-newton-runtime  (one-time ~600 MB); "
                   "set OMNISIM_REQUIRE_NEWTON=1 to fail loudly instead of falling back.",
        })

    if backend == "newton" and fp.get("newton_runtime_present") and \
            not (fp.get("scan") or {}).get("finalised"):
        out.append({
            "code": "NEWTON_PRESENT_BUT_NOT_DRIVING", "severity": "drift",
            "message": "Newton runtime is present but no 'world finalised' line was seen "
                       "('imports OK' is NOT proof Newton drove the world).",
            "fix": "Re-run with a longer duration (>=15 s) and confirm the "
                   "'[WbNewtonBackend] world finalised (solver=...)' line.",
        })

    if fp.get("pillow_present") is False:
        out.append({
            "code": "MISSING_PILLOW", "severity": "drift",
            "message": "Pillow is not importable in this interpreter; the harness "
                       "/world/render_stats endpoint will return 503.",
            "fix": "pip install Pillow  (use the Windows interpreter, not msys2 python).",
        })

    knob = (fp.get("knobs") or {}).get("OMNISIM_NEWTON_SUBSTEPS")
    if knob is not None:
        try:
            if int(knob) % 2 == 1:
                out.append({
                    "code": "ODD_SUBSTEPS", "severity": "drift",
                    "message": f"OMNISIM_NEWTON_SUBSTEPS={knob} is odd - odd substep counts "
                               "break CUDA-graph capture (~0.25x) and can destabilise bipeds.",
                    "fix": "Use an even count (e.g. OMNISIM_NEWTON_SUBSTEPS=4) or unset it.",
                })
        except (TypeError, ValueError):
            pass

    return out
