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

"""OmniBench results-row writer/reader (SPEC.md 'Results schema').

JSON Lines, one row per (test, engine, dt) run:

    {"suite": "omnibench/v0", "test": "bounce", "engine": "mujoco",
     "dt_ms": 4, "metrics": {...}, "wall_ms_per_step": 0.0, "steps": 0,
     "sim_seconds": 0.0, "deviations": [...], "machine": {...},
     "utc": "2026-07-24T00:00:00Z"}

The machine fingerprint comes from projects/policies/common/env_fingerprint.py
(imported and collect()ed in-process; subprocess stdout parse as fallback),
trimmed to the attribution-relevant fields, cached once per process.
"""

import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

SUITE = "omnibench/v0"
# Every `engine` value a row may CARRY (readable). Only a subset is writable:
#   omnisim-ode         HISTORY ONLY -- src/ode was DELETED (bdc02139). Recorded
#                       results*/ rows carry it; nothing may emit it again.
#   omnisim-unverified  produced through omnisim-bin, but the .newton.json
#                       backend-verdict sidecar did not prove which backend drove
#                       the run. Honest but not publishable -- the row must also
#                       carry the reason in `deviations`. It exists because the
#                       old fallback was `omnisim-ode`, i.e. an unprovable row was
#                       published under the name of a deleted engine.
ENGINES = ("mujoco", "pybullet", "omnisim-ode", "omnisim-newton",
           "omnisim-unverified", "mujoco-warp-raw")

# .../tests/benchmarks/omnibench/common/results.py -> repo root is 4 up
_REPO_ROOT = Path(__file__).resolve().parents[4]
_FINGERPRINT_CACHE = None


def _trim_fingerprint(fp, with_backend=False):
    """Keep the attribution fields SPEC asks for (machine id, GPU, CPU,
    build sha, binary sha, libController, stack versions) — drop log-scan
    noise. with_backend additionally keeps the physics/solver verdict, which
    is only meaningful for rows produced THROUGH omnisim-bin (the caller
    passed an engine_log_path)."""
    out = {}
    for k in ("machine", "gpu", "build", "os", "python", "numpy",
              "libcontroller"):
        if k in fp:
            out[k] = fp[k]
    binary = fp.get("binary")
    if isinstance(binary, dict):
        out["binary"] = {k: binary.get(k) for k in ("path", "sha256", "mtime")}
    stack = fp.get("stack")
    if isinstance(stack, dict):
        out["stack_versions"] = stack.get("versions", {})
    if with_backend:
        out["physics"] = fp.get("physics")
        out["solver"] = fp.get("solver")
    return out


def machine_fingerprint(engine_log_path=None):
    """Machine fingerprint dict, cached per process (the plain no-log-path
    variant only). Pass engine_log_path for rows produced through omnisim-bin
    so the Newton backend-verdict sidecar of THAT run decides `physics`."""
    global _FINGERPRINT_CACHE
    if _FINGERPRINT_CACHE is not None and engine_log_path is None:
        return _FINGERPRINT_CACHE
    fp = None
    try:
        sys.path.insert(0, str(_REPO_ROOT / "projects" / "policies" / "common"))
        import env_fingerprint
        fp = _trim_fingerprint(
            env_fingerprint.collect(engine_log_path=engine_log_path),
            with_backend=engine_log_path is not None)
    except Exception as exc:  # fall back to parsing the CLI's stdout
        try:
            proc = subprocess.run(
                [sys.executable,
                 str(_REPO_ROOT / "projects" / "policies" / "common"
                     / "env_fingerprint.py")],
                capture_output=True, text=True, timeout=120)
            oneline = ""
            for line in proc.stdout.splitlines():
                if line.startswith("oneline:"):
                    oneline = line.partition(":")[2].strip()
            fp = {"raw_oneline": oneline or proc.stdout.strip()[-500:],
                  "import_error": repr(exc)}
        except Exception as exc2:
            fp = {"error": "fingerprint unavailable: %r / %r" % (exc, exc2)}
    # force JSON-serializability (paths etc.)
    fp = json.loads(json.dumps(fp, default=str))
    if engine_log_path is not None:
        return fp
    _FINGERPRINT_CACHE = fp
    return _FINGERPRINT_CACHE


def _norm_metric(v):
    """Numbers -> float; everything else (None, bool, str, list, dict) is
    passed through — lane 2/3 rows carry structured metrics (tier labels,
    per-window lists, probe extras) that must not be coerced."""
    if v is None or isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return float(v)
    return v


def make_row(*, test, engine, dt_ms, metrics, wall_ms_per_step, steps,
             sim_seconds, deviations=(), machine=None):
    """Build one schema-conformant row dict (does not write it).

    machine: optional pre-collected fingerprint dict (e.g. from
    machine_fingerprint(engine_log_path=...) for omnisim-bin-backed rows);
    defaults to the cached plain fingerprint."""
    return {
        "suite": SUITE,
        "test": test,                       # SPEC test name, e.g. "bounce"
        "engine": engine,
        "dt_ms": dt_ms,
        "metrics": {k: _norm_metric(v) for k, v in metrics.items()},
        "wall_ms_per_step": float(wall_ms_per_step),
        "steps": int(steps),
        "sim_seconds": float(sim_seconds),
        "deviations": list(deviations),
        "machine": machine if machine is not None else machine_fingerprint(),
        "utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }


def append_row(path, row):
    """Append one pre-built row dict to a .jsonl file (creating parents)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(row) + "\n")
    return row


def write_row(path, **kwargs):
    """Append one row to a .jsonl file (creating parents). Returns the row."""
    return append_row(path, make_row(**kwargs))


def read_rows(path):
    """Read a .jsonl results file -> list of row dicts (skips blank lines)."""
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows
