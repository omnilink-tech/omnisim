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

"""Runtime ENVIRONMENT FINGERPRINT for deploy controllers.

Why this exists
---------------
A demo that "works on the dev box but regressed on another computer" (or that
people "couldn't see") needs to report, from inside the running controller,
*what actually drove the world on THIS machine*:

  * which physics solver really stepped the world (Newton/MuJoCo vs the silent
    ODE fallback) -- the #1 cause of a humanoid-stand regression across machines
    is the Newton runtime not being present, so the world runs ODE instead of
    the mujoco_warp solver the stand was tuned on. The stand's stability basin
    is razor-thin and behaves *differently* under ODE, so it tips on the other
    box even though the build "loaded fine".
  * which GPU + driver are present (the cross-machine variable for "couldn't
    see it" / rendering-gone-wrong and for whether the GPU mujoco_warp solver
    can run at all),
  * whether the world was loaded WARM or COLD (a cold first-load under-tracks
    the articulation -- see reference_cold_first_load_trap),
  * the build/OS/python the run used,
  * the physics env knobs that change the outcome.

The point: a future bug report arrives with a diagnostic fingerprint instead of
a guess, so "it regressed on Bob's PC" becomes actionable in one line.

Ground truth, not a guess
-------------------------
The authoritative signal that Newton actually *drove* the world is the engine
log line ``[WbNewtonBackend] world finalised (solver=...)`` -- NOT the earlier
``imports OK`` / ``FFI smoke OK`` (those only mean the runtime *loaded*; the
world can still finalise on ODE/XPBD after them). This module reads that line
out of the engine log and reports the solver verbatim, flagging the known
silent-degrade strings (``FAILED`` / ``XPBD fallback``).

Everything here is DEFENSIVE: no call may raise into the controller, every
external probe (engine log, ``nvidia-smi``, ``git``) is wrapped + time-bounded.
"""
from __future__ import annotations

import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

# Engine log markers (see src/omnisim/physics/WbNewtonBackend.cpp).
# Greedy to the LAST ')' on the line so nested-paren solver names survive, e.g.
# "world finalised (solver=MuJoCo (mujoco_warp))" -> "MuJoCo (mujoco_warp)".
_RE_FINALISED = re.compile(r"world finalised \(solver=(.*)\)")
_RE_IMPORTS_OK = re.compile(r"\[WbNewtonBackend\].*imports OK")
_RE_ODE_FALLBACK = re.compile(r"\[WbNewtonBackend\].*[Ff]alling back to ODE")
# Physics env knobs worth recording (set -> changes the stepped physics).
_PHYS_KNOBS = (
    "OMNISIM_NEWTON_SUBSTEPS", "OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_TARGET_KD",
    "OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_STATICS", "OMNISIM_NEWTON_FORCE_MUJOCO",
    "OMNISIM_NEWTON_MJWARP", "OMNISIM_NEWTON_USE_LINK_COM", "OMNISIM_NEWTON_SEED_POSE",
    "OMNISIM_NEWTON_BASE_GUARD", "OMNISIM_URDF_USE_INERTIA",
)
# Env that declares intent about the backend (so we can flag intent != reality).
_INTENT = ("OMNISIM_REQUIRE_NEWTON", "OMNISIM_LEGACY", "OMNISIM_FORCE_ODE",
           "renderBackend", "OMNISIM_RENDER_BACKEND")


def _repo_root(repo_root=None) -> Path:
    if repo_root:
        return Path(repo_root)
    env = os.environ.get("OMNISIM_HOME") or os.environ.get("WEBOTS_HOME")
    if env:
        return Path(env)
    return Path(__file__).resolve().parents[3]


def _resolve_engine_log(explicit, repo_root: Path):
    if explicit:
        return Path(explicit)
    p = os.environ.get("OMNISIM_LOG_PATH")
    if p:
        return Path(p)
    return repo_root / "omnisim_log.txt"


def _read_log_window(path: Path, window: int = 262144) -> str:
    """Read the engine log's HEAD **and** TAIL (the log can be large).

    The physics ground-truth markers -- ``imports OK``, ``world finalised``,
    ``[Ff]alling back to ODE`` -- are all emitted at LOAD time, i.e. at the very
    TOP of the log. A long or chatty run then pushes that ``world finalised``
    line far above any tail-only window, so a tail-only read misses it and the
    verdict flips to "ODE or unknown" even though Newton drove the world (the
    "viewport says ODE but Newton is being used" bug, and the same false-ODE for
    a post-hoc ``omnisim doctor`` read of a finished run's log).

    Reading the HEAD captures the load-time verdict; reading the TAIL still
    captures a late warmup-reload re-finalise or a mid-run degrade. The middle
    is pure per-tick chatter with no markers, so skipping it is lossless.
    ``findall(...)[-1]`` in the scanner keeps "last finalise wins" because the
    head is concatenated before the tail.
    """
    try:
        size = path.stat().st_size
        with open(path, "rb") as f:
            if size <= 2 * window:
                return f.read().decode("utf-8", "replace")
            head = f.read(window)
            f.seek(size - window)
            tail = f.read(window)
        # Decode the two halves independently so a multibyte char split at
        # either seam degrades to a single replacement char rather than
        # corrupting a marker line at the boundary.
        return head.decode("utf-8", "replace") + "\n" + tail.decode("utf-8", "replace")
    except Exception:
        return ""


def _scan_engine_log(text: str) -> dict:
    """Extract physics ground-truth from the engine log text."""
    out = {
        "log_found": bool(text),
        "newton_imported": bool(_RE_IMPORTS_OK.search(text)),
        "finalised": False,
        "solver": None,
        "ode_fallback": bool(_RE_ODE_FALLBACK.search(text)),
    }
    finals = _RE_FINALISED.findall(text)
    if finals:
        out["finalised"] = True
        out["solver"] = finals[-1].strip()  # last finalise wins (warmup reloads)
    return out


def _read_backend_sidecar(log_path: Path) -> dict | None:
    """Read the engine's race-free backend-verdict sidecar, or None.

    `WbNewtonBackend::finalizeWorld()` writes ``<engine-log>.newton.json`` iff
    Newton actually finalised the world THIS run, and `WbLog::initFileLog` deletes
    any stale prior-run copy when it truncates the log at process start. So the
    file's presence is *proof* Newton drove the world -- authoritative and
    race-free, independent of the log's size/position (which a scrape depends on).
    Absent (pure-ODE run, older engine build, or a mismatched log path) -> caller
    falls back to the log scan, which is itself reliable now (head+tail read).
    """
    try:
        sc = log_path.with_name(log_path.name + ".newton.json")
        if not sc.is_file():
            return None
        d = json.loads(sc.read_text(encoding="utf-8"))
        if d.get("backend") == "newton" and d.get("finalised"):
            return d
        return None
    except Exception:
        return None


def _startup_race(scan: dict) -> bool:
    """True when the scan looks like a PRE-FIRST-TICK read, not a real verdict.

    The engine finalizes the Newton world lazily on the FIRST simulation tick
    (WbSimulationWorld::step -> WbNewtonBackend::finalizeWorld) -- which is also
    the moment the ``world finalised`` log line and the ``.newton.json`` sidecar
    are written. A controller that fingerprints before its first robot.step()
    therefore reads a log that has ``imports OK`` but *cannot yet* contain the
    verdict, and reporting "UNCONFIRMED (SUSPECT)" there is a false alarm, not
    ground truth. Callers that run pre-step pass ``pre_step=True`` to report()
    and re-report after stepping (see humanoid_stand_deploy).
    """
    return bool(scan.get("newton_imported")) and not scan.get("finalised") \
        and not scan.get("ode_fallback")


def _verdict(scan: dict) -> tuple[str, list[str], bool]:
    """Collapse the log scan into (one-phrase verdict, warnings, healthy?).

    `healthy` is True ONLY when Newton cleanly finalised on a non-degraded
    MuJoCo solver (GPU mujoco_warp or CPU MuJoCo). Everything else -- ODE
    fallback, an XPBD degrade, an unconfirmed finalise, or an unreadable log --
    is a CHECK state for a stand tuned on the MuJoCo solver.
    """
    warns: list[str] = []
    solver = scan.get("solver")
    if scan.get("finalised") and solver:
        low = solver.lower()
        if "failed" in low or "xpbd fallback" in low:
            warns.append(f"MuJoCo solver DEGRADED to '{solver}' -- different physics "
                         "than the mujoco_warp tuning; stand may behave differently.")
            return f"newton/{solver} (DEGRADED)", warns, False
        if "mujoco_warp" in low or "mjwarp" in low:
            return "newton/mujoco_warp (GPU)", warns, True
        if "cpu" in low:
            return f"newton/{solver} (CPU)", warns, True
        return f"newton/{solver}", warns, True
    if scan.get("ode_fallback"):
        warns.append("Newton FELL BACK TO ODE -- the humanoid stand was tuned on "
                     "the MuJoCo solver; ODE has a different contact/solver model "
                     "and the stand can regress. Bundle the Newton runtime "
                     "(make -C src/omnisim bundle-newton-runtime) or set "
                     "OMNISIM_REQUIRE_NEWTON=1 to fail loudly.")
        return "ODE (Newton fell back)", warns, False
    if scan.get("newton_imported"):
        warns.append("Newton runtime LOADED but no 'world finalised' line seen -- "
                     "it may not have driven this world (stall/XPBD/short run). "
                     "'imports OK' is NOT proof Newton stepped the world.")
        return "newton loaded, finalise UNCONFIRMED (SUSPECT)", warns, False
    if not scan.get("log_found"):
        return "UNKNOWN (engine log not found)", [
            "Could not read the engine log, so the solver that drove the world is "
            "unknown. Set OMNISIM_LOG_PATH to the run's log."], False
    warns.append("No Newton finalise line in the engine log -- world likely ran ODE.")
    return "ODE or unknown (no Newton finalise)", warns, False


def _nvidia_gpu() -> str | None:
    """GPU name + driver via nvidia-smi (time-bounded, optional).

    Distinguishes "no NVIDIA tooling" (None) from "tooling present but the query
    failed/timed out" (a descriptive string) -- a laptop dGPU can take a couple
    seconds to wake on the first query, and a false "no GPU" would mislead the
    very cross-machine diagnosis this fingerprint is for.
    """
    exe = shutil.which("nvidia-smi")
    if not exe:
        return None
    try:
        r = subprocess.run(
            [exe, "--query-gpu=name,driver_version", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=6.0)
        line = (r.stdout or "").strip().splitlines()
        if line:
            parts = [p.strip() for p in line[0].split(",")]
            if len(parts) >= 2:
                return f"{parts[0]} (driver {parts[1]})"
            return parts[0]
        return "nvidia-smi present but returned no GPU (query empty)"
    except subprocess.TimeoutExpired:
        return "nvidia-smi present but query TIMED OUT (dGPU asleep?)"
    except Exception as e:
        return f"nvidia-smi present but query failed ({type(e).__name__})"


def _git_commit(repo_root: Path) -> str | None:
    exe = shutil.which("git")
    if not exe:
        return None
    try:
        r = subprocess.run([exe, "-C", str(repo_root), "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=2.0)
        h = (r.stdout or "").strip()
        return h or None
    except Exception:
        return None


def _pkg_ver(mod_name: str) -> str | None:
    try:
        mod = __import__(mod_name)
        return getattr(mod, "__version__", None)
    except Exception:
        return None


def _short_gpu(name: str | None) -> str:
    if not name:
        return "none"
    # "NVIDIA GeForce RTX 4090 (driver 555.42)" -> "RTX 4090"
    m = re.search(r"(RTX|GTX|A\d{2,4}|H\d{2,3}|L\d{1,2}|T\d{1,2})[\s-]?\w*", name)
    return m.group(0) if m else name.split("(")[0].strip()[:18]


def collect(robot=None, *, engine_log_path=None, repo_root=None, warm=None) -> dict:
    """Gather the full fingerprint. Never raises."""
    root = _repo_root(repo_root)
    log_path = _resolve_engine_log(engine_log_path, root)
    scan = _scan_engine_log(_read_log_window(log_path))
    # Prefer the engine's authoritative backend-verdict sidecar over the log
    # scrape: its presence is race-free proof Newton finalised THIS run, so it
    # overrides any log-scrape ambiguity (finalise line not yet flushed, or a
    # truncated/rotated log). Absent -> keep the (now reliable) log-scan verdict.
    sidecar = _read_backend_sidecar(log_path)
    if sidecar is not None:
        scan["finalised"] = True
        scan["newton_imported"] = True
        scan["solver"] = sidecar.get("solver") or scan.get("solver") or "unrecorded"
        scan["source"] = "sidecar"
    physics, warns, healthy = _verdict(scan)

    intent = {k: os.environ[k] for k in _INTENT if os.environ.get(k)}
    # Intent-vs-reality: required Newton but didn't get a clean finalise.
    if intent.get("OMNISIM_REQUIRE_NEWTON", "0") not in ("0", "", "false", "False"):
        if not healthy:
            warns.append("OMNISIM_REQUIRE_NEWTON is set but Newton did not cleanly "
                         "finalise -- the engine should have aborted; investigate.")

    fp = {
        "physics": physics,
        "solver": scan.get("solver"),
        "scan": scan,
        "engine_log": str(log_path),
        "load": ("warm" if warm else "cold") if warm is not None else "unknown",
        "warmup_token": bool(os.environ.get("OMNISIM_WARMUP_TOKEN")),
        "gpu": _nvidia_gpu(),
        "build": _git_commit(root),
        "os": platform.platform(terse=True),
        "python": platform.python_version(),
        "numpy": _pkg_ver("numpy"),
        "onnxruntime": _pkg_ver("onnxruntime"),
        "knobs": {k: os.environ[k] for k in _PHYS_KNOBS if os.environ.get(k)},
        "intent": intent,
        "warnings": warns,
    }
    # OK only when Newton cleanly finalised on a non-degraded MuJoCo solver AND
    # no intent-vs-reality conflict was raised.
    fp["ok"] = healthy and not any("REQUIRE_NEWTON" in w for w in warns)
    return fp


def format_block(fp: dict, prefix: str = "[envfp]") -> str:
    """Multi-line, greppable block for the deploy log / stderr."""
    knobs = " ".join(f"{k.split('_')[-1].lower()}={v}" for k, v in fp["knobs"].items())
    intent = " ".join(f"{k}={v}" for k, v in fp["intent"].items()) or "(none)"
    load = fp["load"].upper()
    if fp["load"] == "cold":
        load += " (first-load articulation under-tracks)"
    lines = [
        f"{prefix} ===== OmniSim environment fingerprint =====",
        f"{prefix} physics : {fp['physics']}"
        + (f"   solver='{fp['solver']}'" if fp["solver"] else ""),
        f"{prefix} load    : {load}   warmup_token={fp['warmup_token']}",
        f"{prefix} gpu     : {fp['gpu'] or 'no nvidia-smi on PATH (no NVIDIA driver?)'}",
        f"{prefix} build   : {fp['build'] or '?'}  os={fp['os']}  py={fp['python']}"
        f"  numpy={fp['numpy']}  ort={fp['onnxruntime']}",
        f"{prefix} knobs   : {knobs or '(defaults)'}",
        f"{prefix} intent  : {intent}",
        f"{prefix} log     : {fp['engine_log']}",
    ]
    for w in fp["warnings"]:
        lines.append(f"{prefix} WARN    : {w}")
    lines.append(f"{prefix} verdict : {'OK' if fp['ok'] else 'CHECK WARNINGS ABOVE'}")
    lines.append(f"{prefix} ===========================================")
    return "\n".join(lines) + "\n"


def format_oneline(fp: dict) -> str:
    """Compact single line for an on-screen overlay."""
    phys = fp["physics"].split(" (")[0]
    return (f"OmniSim  physics={phys}  gpu={_short_gpu(fp['gpu'])}  "
            f"load={fp['load'].upper()}  build={fp['build'] or '?'}")


def report(robot, say, *, warm=None, label=True,
           engine_log_path=None, repo_root=None, pre_step=False) -> dict:
    """Collect the fingerprint, write it to the controller log, and (if the
    controller is a Supervisor) draw a compact on-screen overlay.

    `say` is the controller's logging fn (writes stderr + deploy side-log).
    Returns the fingerprint dict so callers may assert on `fp["ok"]`.

    ``pre_step=True`` declares this call runs BEFORE the caller's first
    robot.step(). The engine only finalizes the physics backend (and writes the
    verdict sidecar + ``world finalised`` log line) on the FIRST simulation
    tick, so at that point no verdict can exist yet. When the scan looks like
    that pre-tick state, this draws a neutral grey "verifying" label instead of
    a false-alarm orange one and returns ``fp["pending"] = True`` -- the caller
    MUST then call report() again (without pre_step) after it has stepped the
    world at least once, which redraws the label with the real verdict.
    """
    try:
        fp = collect(robot, engine_log_path=engine_log_path,
                     repo_root=repo_root, warm=warm)
    except Exception as e:  # defensive: never break the controller
        try:
            say(f"[envfp] fingerprint collect failed ({e})\n")
        except Exception:
            pass
        return {"ok": False, "warnings": [f"collect failed: {e}"]}

    if pre_step and not fp["ok"] and _startup_race(fp.get("scan", {})):
        fp["pending"] = True
        try:
            say("[envfp] physics verdict PENDING: the engine finalizes the backend "
                "on the first simulation tick, which has not run yet -- the label "
                "will be redrawn with the real verdict after the first step\n")
        except Exception:
            pass
        if label and robot is not None and hasattr(robot, "setLabel"):
            try:
                robot.setLabel(90, f"OmniSim  physics=verifying (pre-step)...  "
                                   f"gpu={_short_gpu(fp['gpu'])}  build={fp['build'] or '?'}",
                               0.01, 0.01, 0.06, 0xBBBBBB, 0.0)
                robot.setLabel(91, "", 0.01, 0.06, 0.05, 0xBBBBBB, 0.0)
            except Exception:
                pass
        return fp

    try:
        say(format_block(fp))
    except Exception:
        pass

    if label and robot is not None and hasattr(robot, "setLabel"):
        try:
            color = 0x44FF66 if fp["ok"] else 0xFFAA00
            robot.setLabel(90, format_oneline(fp), 0.01, 0.01, 0.06, color, 0.0)
            # Always redraw label 91: a pre-step "verifying" pass may have drawn
            # it, and a clean verdict must clear it rather than leave it behind.
            warn_line = ("! " + fp["warnings"][0][:70]) \
                if (fp["warnings"] and not fp["ok"]) else ""
            robot.setLabel(91, warn_line, 0.01, 0.06, 0.05, 0xFFAA00, 0.0)
        except Exception:
            pass
    return fp


if __name__ == "__main__":
    # Inspect an existing engine log post-hoc:
    #   python projects/policies/common/env_fingerprint.py [path/to/omnisim_log.txt]
    _log = sys.argv[1] if len(sys.argv) > 1 else None
    _fp = collect(engine_log_path=_log, warm=None)
    sys.stdout.write(format_block(_fp))
    sys.stdout.write("oneline: " + format_oneline(_fp) + "\n")
    sys.exit(0 if _fp.get("ok") else 2)
