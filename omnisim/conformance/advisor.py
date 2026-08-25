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

"""Remediation advisor: FAIL/DRIFT cause -> exact fix.

A keyed lookup over the fingerprint (+ observed backend) that emits actionable
advisories. Phase 1 implements the highest-value subset of the full table in
docs/developer/install-conformance.md §6.2; more codes land with the gate.
"""

from __future__ import annotations

import sys


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

    # RENAMED FROM `NEWTON_FELL_BACK_TO_ODE` (2026-08-08). The old advisory said
    # "Physics resolved to ODE ... ODE is supported" -- both halves are now false:
    # src/ode was DELETED (commit bdc02139), so nothing fell back anywhere and
    # there is no supported second backend. It fired on the ACTUAL failure (the
    # Newton runtime is not there) and reported it as a benign configuration,
    # which is the exact tool-lies-to-the-agent failure AGENTS.md forbids
    # (docs/developer/v6-readiness.md section 6 lists this as one of three sites).
    if backend == "unverified" and not fp.get("newton_runtime_present"):
        out.append({
            "code": "NEWTON_RUNTIME_MISSING",
            "severity": "fail" if require_newton else "drift",
            "message": "No physics backend was verified for this run and the Newton "
                       "runtime is not bundled next to the binary. This is NOT an ODE "
                       "run -- ODE was deleted (src/ode, bdc02139) and Newton is the "
                       "only backend, so there is nothing to fall back to.",
            "fix": "make -C src/omnisim bundle-newton-runtime  (one-time ~600 MB); "
                   "set OMNISIM_REQUIRE_NEWTON=1 to fail loudly instead of running "
                   "with an unverified backend.",
        })

    if fp.get("newton_runtime_present") and backend in ("newton", "unverified") and \
            not (fp.get("scan") or {}).get("finalised"):
        out.append({
            "code": "NEWTON_PRESENT_BUT_NOT_DRIVING", "severity": "drift",
            "message": "Newton runtime is present but no 'world finalised' line was seen "
                       "('imports OK' is NOT proof Newton drove the world). The backend "
                       "for this run is UNVERIFIED, not ODE.",
            "fix": "Re-run with a longer duration (>=15 s locally, >=45 s on a slow or "
                   "virtualized disk) and confirm the "
                   "'[OmNewtonBackend] world finalised (solver=...)' line.",
        })

    if fp.get("pillow_present") is False:
        out.append({
            "code": "MISSING_PILLOW", "severity": "drift",
            "message": "Pillow is not importable in this interpreter; the harness "
                       "/world/render_stats endpoint will return 503.",
            "fix": "pip install Pillow  (use the Windows interpreter, not msys2 python)."
                   if sys.platform == "win32" else
                   "pip install Pillow  (into the interpreter running the harness).",
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
