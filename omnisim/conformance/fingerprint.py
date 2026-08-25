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

"""Phase 0 — the config fingerprint for install conformance.

Wraps ``projects/policies/common/env_fingerprint.collect()`` — the load-bearing
producer that reads the authoritative ``[OmNewtonBackend] world finalised
(solver=...)`` line out of the engine log — and augments it with the few
install-relevant fields that producer doesn't carry, then derives a stable
``fingerprint_id`` over only the physics-affecting subset.

Two consumers: ``omnisim doctor --fingerprint`` and ``omnisim verify-install``.

``fingerprint_id`` deliberately keys on the things that actually change install
behaviour (commit, resolved backend, solver, Newton-runtime presence, GPU
model, knobs, binary mtime) and excludes cosmetics (OS minor version, paths) —
so a rebuild or an ODE<->Newton swap invalidates a prior conformance pass, but
a harmless env tweak does not.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import platform
from pathlib import Path

from ..paths import REPO_ROOT, resolve_omnisim_binary

_ENVFP_PATH = REPO_ROOT / "projects" / "policies" / "common" / "env_fingerprint.py"
_envfp = None  # module | False (tried-and-failed) | None (not tried)


def _load_envfp():
    """Load env_fingerprint by file path (it lives outside the omnisim pkg)."""
    global _envfp
    if _envfp is not None:
        return _envfp or None
    try:
        spec = importlib.util.spec_from_file_location("omnisim_conf_envfp", _ENVFP_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)  # type: ignore[union-attr]
        _envfp = mod
    except Exception:
        _envfp = False
    return _envfp or None


def _pkg_ver(mod_name: str) -> str | None:
    try:
        mod = __import__(mod_name)
        return getattr(mod, "__version__", None)
    except Exception:
        return None


def _mtime(path: str | None) -> int | None:
    if not path:
        return None
    try:
        return int(os.path.getmtime(path))
    except OSError:
        return None


def newton_runtime_present(binary: str | None) -> bool:
    """True iff the bundled Newton runtime sits next to the binary.

    A stock release bundles it (``BUNDLE_NEWTON=1``); a from-source clone /
    ``make debug`` without the runtime silently runs ODE — this is the single
    field that explains a 'works on the dev box, ODE on mine' divergence.
    """
    if not binary:
        return False
    warp = Path(binary).parent / "newton-runtime" / "site-packages" / "warp"
    return warp.exists()


def resolved_backend(fp: dict) -> str:
    """Collapse the env_fingerprint log scan to ``newton`` | ``unverified`` |
    ``unknown``.

    A finalise line (any solver) means the Newton backend drove the world.
    Anything else -- a logged "falling back to ODE", or a readable log with no
    finalise line -- is ``unverified``. No readable log (fresh install, never
    run) is ``unknown``.

    ``"ode"`` IS NOT A RETURN VALUE ANY MORE (2026-08-08). Both of the branches
    above used to return it, which meant this function invented a backend: ODE
    was DELETED (src/ode, commit bdc02139) and Newton is the only physics
    backend, so no log can imply "ODE drove the run". It mattered concretely --
    the value keys `_select_band()` in compare.py, is hashed into
    `fingerprint_id()`, and drove the advisor's `NEWTON_FELL_BACK_TO_ODE`
    advisory, so a broken Newton install was reported as a benign supported
    configuration (see docs/developer/v6-readiness.md section 6).
    """
    scan = fp.get("scan") or {}
    if scan.get("finalised") and scan.get("solver"):
        return "newton"
    if scan.get("ode_fallback"):
        return "unverified"
    if scan.get("log_found") and not scan.get("finalised"):
        return "unverified"
    return "unknown"


def _gpu_model(gpu: str | None) -> str:
    """Drop the driver version so a driver bump doesn't churn fingerprint_id."""
    if not gpu:
        return ""
    return gpu.split("(driver")[0].strip()


def fingerprint_id(fp: dict) -> str:
    """12-hex digest over only the install-behaviour-affecting subset."""
    scan = fp.get("scan") or {}
    payload = {
        "build": fp.get("build"),
        "resolved_backend": fp.get("resolved_backend"),
        "solver": scan.get("solver"),
        "newton_runtime_present": fp.get("newton_runtime_present"),
        "warp": fp.get("warp_version"),
        "mujoco": fp.get("mujoco_version"),
        "gpu": _gpu_model(fp.get("gpu")),
        "os": fp.get("os"),
        "render_backend": fp.get("render_backend"),
        "knobs": fp.get("knobs"),
        "binary_mtime": fp.get("binary_mtime"),
    }
    blob = json.dumps(payload, sort_keys=True, default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:12]


def collect(*, engine_log_path=None, warm=None, scrub_paths: bool = True) -> dict:
    """Gather the full install fingerprint. Never raises.

    ``scrub_paths`` (default True) runs the result through the privacy scrub so
    it is safe to print / share. Internal callers that need the real engine-log
    path should pass ``scrub_paths=False`` (the conformance runner uses its own
    per-check log paths, so it doesn't).
    """
    binary = resolve_omnisim_binary()
    mod = _load_envfp()
    if mod is not None:
        try:
            base = mod.collect(engine_log_path=engine_log_path, warm=warm)
        except Exception as exc:  # defensive: never break the caller
            base = {"warnings": [f"env_fingerprint.collect failed: {exc}"], "scan": {}}
    else:
        base = {"warnings": ["env_fingerprint module could not be loaded"], "scan": {}}

    extra = {
        "resolved_backend": resolved_backend(base),
        "newton_runtime_present": newton_runtime_present(binary),
        "pillow_present": _pkg_ver("PIL") is not None,
        "pillow_version": _pkg_ver("PIL"),
        "warp_version": _pkg_ver("warp"),
        "mujoco_version": _pkg_ver("mujoco"),
        "cpu_core_count": os.cpu_count(),
        "render_backend": (
            os.environ.get("OMNISIM_RENDER_BACKEND")
            or os.environ.get("renderBackend")
            or "wren"
        ),
        "omnisim_binary": binary,
        "binary_mtime": _mtime(binary),
        "fingerprint_schema": "omnisim.install_fingerprint/v1",
    }
    fp = {**base, **extra}
    fp["fingerprint_id"] = fingerprint_id(fp)

    if scrub_paths:
        from .scrub import scrub as _scrub
        fp = _scrub(fp)
    return fp
