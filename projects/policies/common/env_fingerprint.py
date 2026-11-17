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

  * which physics solver really stepped the world, or whether that is UNVERIFIED
    -- the #1 cause of a humanoid-stand regression across machines is the Newton
    runtime not coming up on the other box, so the world is not stepped by the
    mujoco_warp solver the stand was tuned on. The stand's stability basin is
    razor-thin, so it tips even though the build "loaded fine".

    !! This used to be phrased "vs the silent ODE fallback", and the code
    reported "ODE or unknown" whenever it could not find a finalise line. That
    is now FALSE ATTRIBUTION and was fixed on 2026-08-08: **src/ode was DELETED
    (commit bdc02139) and Newton is the only physics backend**, so no evidence
    of Newton can no longer mean "ODE ran instead" -- there is nothing else to
    run. It means the run's backend is UNVERIFIED (usually: the run never
    reached world-finalize, where the verdict line and the `.newton.json`
    sidecar are written). This matters beyond wording because
    `tests/benchmarks/omnibench/common/results.py` embeds this dict in EVERY
    OmniBench result row: the old string stamped published measurements with the
    name of an engine that does not exist.
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
log line ``[OmNewtonBackend] world finalised (solver=...)`` -- NOT the earlier
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

# Engine log markers (see src/omnisim/physics/OmNewtonBackend.cpp).
# Greedy to the LAST ')' on the line so nested-paren solver names survive, e.g.
# "world finalised (solver=MuJoCo (mujoco_warp))" -> "MuJoCo (mujoco_warp)".
_RE_FINALISED = re.compile(r"world finalised \(solver=(.*)\)")
# The bracketed tag is named after the emitting C++ class, and those classes are
# being renamed Wb* -> Om*, so the same lines will read "[OmNewtonBackend] ..."
# once the rename lands. Accept BOTH prefixes permanently: a fingerprint must
# resolve on either side of the rename, and log files captured before it must
# keep parsing forever.
_NEWTON_TAG = r"\[(?:Wb|Om)NewtonBackend\]"
_RE_IMPORTS_OK = re.compile(_NEWTON_TAG + r".*imports OK")
_RE_ODE_FALLBACK = re.compile(_NEWTON_TAG + r".*[Ff]alling back to ODE")
# Physics env knobs worth recording (set -> changes the stepped physics).
_PHYS_KNOBS = (
    "OMNISIM_NEWTON_SUBSTEPS", "OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_TARGET_KD",
    "OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_STATICS", "OMNISIM_NEWTON_FORCE_MUJOCO",
    "OMNISIM_NEWTON_MJWARP", "OMNISIM_NEWTON_USE_LINK_COM", "OMNISIM_NEWTON_SEED_POSE",
    "OMNISIM_NEWTON_BASE_GUARD", "OMNISIM_URDF_USE_INERTIA",
)
# Env that declares intent about the backend (so we can flag intent != reality).
# OMNISIM_LEGACY / OMNISIM_FORCE_ODE stay on this list DELIBERATELY even though
# they are RETIRED (src/ode deleted, commit bdc02139): the whole job of this
# fingerprint is to record what a result was produced under, and "the operator
# had a dead backend knob exported" is exactly the kind of thing that must land
# in the record rather than be filtered out of it. _verdict()/the warning block
# below flags them as retired so a fingerprint never reads as a working ODE run.
_INTENT = ("OMNISIM_REQUIRE_NEWTON", "OMNISIM_LEGACY", "OMNISIM_FORCE_ODE",
           "renderBackend", "OMNISIM_RENDER_BACKEND")

#: Backend knobs that no longer select anything (src/ode deleted, bdc02139).
_RETIRED_ODE_KNOBS = ("OMNISIM_LEGACY", "OMNISIM_FORCE_ODE")


def _resolve_home(repo_root=None) -> tuple[Path, str, list[str]]:
    """Resolve the checkout root AND report how it was found.

    Returns ``(root, source, warnings)``. The old resolver simply trusted
    ``OMNISIM_HOME`` / ``WEBOTS_HOME`` -- so when the legacy ``WEBOTS_HOME`` is
    left set to a phantom install (on THIS machine it is a machine-scope
    ``C:\\Program Files\\Webots`` that does not exist, with ``OMNISIM_HOME``
    unset), every path below it -- engine binary, engine log, git commit --
    silently resolved into that phantom dir and the fingerprint degraded to
    "binary NOT FOUND" and a log path under a directory that cannot exist, with
    no hint why. That is the exact "silently degrades" failure this fingerprint
    exists to kill, so a set-but-nonexistent home var is now IGNORED (we fall
    back to the module-relative root, which is always correct because this file
    lives at ``<root>/projects/policies/common/env_fingerprint.py``) and it
    emits a loud warning naming the phantom var.
    """
    if repo_root:
        return Path(repo_root), "explicit", []
    file_root = Path(__file__).resolve().parents[3]
    warns: list[str] = []
    for var in ("OMNISIM_HOME", "WEBOTS_HOME"):
        val = os.environ.get(var)
        if not val:
            continue
        p = Path(val)
        if p.exists():
            return p, var, warns
        warns.append(
            f"{var}='{val}' points at a directory that DOES NOT EXIST -- "
            f"ignoring it and using the module-relative repo root instead "
            f"({file_root}). A stale/legacy {var} (this machine carries a "
            f"machine-scope WEBOTS_HOME pointing at a phantom install) is what "
            f"silently degraded this fingerprint to 'binary NOT FOUND'. Set "
            f"OMNISIM_HOME to this checkout, or clear the phantom {var}.")
    return file_root, "module-relative", warns


def _repo_root(repo_root=None) -> Path:
    """Backward-compatible thin wrapper: just the resolved root."""
    return _resolve_home(repo_root)[0]


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
    verdict flips to the not-verified one even though Newton drove the world (the
    "viewport says ODE but Newton is being used" bug -- back then that verdict
    read "ODE or unknown", which is why the symptom named ODE -- and the same
    false negative for a post-hoc ``omnisim doctor`` read of a finished run).

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

    `OmNewtonBackend::finalizeWorld()` writes ``<engine-log>.newton.json`` iff
    Newton actually finalised the world THIS run, and `OmLog::initFileLog` deletes
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
    (OmSimulationWorld::step -> OmNewtonBackend::finalizeWorld) -- which is also
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
    MuJoCo solver (GPU mujoco_warp or CPU MuJoCo). Everything else -- a solver
    degrade, an unconfirmed finalise, a runtime that never came up, or an
    unreadable log -- is a CHECK state for a stand tuned on the MuJoCo solver.

    NAMING RULE (2026-08-08, and the whole point of this function now): a
    not-healthy verdict must never name ANOTHER BACKEND as the thing that ran.
    Newton is the only physics backend (src/ode DELETED, commit bdc02139), so
    every failure mode here is "unverified", not "ODE". This string is copied
    verbatim into OmniBench result rows via common/results.py, so a wrong word
    here is a published false attribution, not a cosmetic label.
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
        warns.append("The engine logged 'falling back to ODE', which means the "
                     "NEWTON RUNTIME DID NOT COME UP (warp/newton import or FFI "
                     "smoke failed). It does NOT mean ODE drove the world: "
                     "src/ode was DELETED (commit bdc02139) and there is no "
                     "second backend to fall back to -- the message is stale "
                     "wording in OmNewtonBackend.cpp. Treat this run as having "
                     "NO VERIFIED PHYSICS. (Only a log captured before bdc02139 "
                     "could have meant a real ODE run.) Fix the runtime: "
                     "make -C src/omnisim bundle-newton-runtime, or set "
                     "OMNISIM_REQUIRE_NEWTON=1 to fail loudly instead.")
        return "UNVERIFIED (Newton runtime absent/broken)", warns, False
    if scan.get("newton_imported"):
        warns.append("Newton runtime LOADED but no 'world finalised' line seen -- "
                     "it may not have driven this world (stall, or a run that "
                     "ended before the first tick). 'imports OK' is NOT proof "
                     "Newton stepped the world.")
        return "newton loaded, finalise UNCONFIRMED (SUSPECT)", warns, False
    if not scan.get("log_found"):
        return "UNKNOWN (engine log not found)", [
            "Could not read the engine log, so the solver that drove the world is "
            "unknown. Set OMNISIM_LOG_PATH to the run's log."], False
    # This used to read "ODE or unknown (no Newton finalise)" and it was FALSE
    # ATTRIBUTION: this fingerprint is embedded verbatim in every OmniBench
    # result row, so the absence of a finalise line stamped the result with a
    # backend that NO LONGER EXISTS (src/ode deleted, bdc02139). Newton is the
    # only backend, so "no Newton evidence" cannot name a different engine that
    # ran instead -- the honest reading is that the run almost certainly never
    # reached world-finalize (too-short duration is the common cause; AGENTS.md
    # budgets >=15 s locally and >=45 s on slow/virtualized disks), or this is
    # not that run's log.
    warns.append("No Newton evidence in the engine log (neither 'imports OK' nor "
                 "'world finalised') -- the backend that drove this run is "
                 "UNVERIFIED. This is NOT an ODE run: ODE was deleted (src/ode, "
                 "commit bdc02139) and Newton is the only backend. Most likely "
                 "the run never reached world-finalize (too short: budget >=15 s, "
                 ">=45 s on a slow/virtualized disk) or OMNISIM_LOG_PATH does not "
                 "point at this run's log. Re-run longer before concluding "
                 "anything about the physics.")
    return "UNVERIFIED (no Newton evidence in the log)", warns, False


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


def _binary_info(repo_root: Path) -> dict | None:
    """Identity of the engine binary that (presumably) drove this run.

    The tree's git commit alone is not provenance: this repo has shipped runs
    where the binary trailed the tree by 8 days. Hash the artifact itself, and
    record whether it carries the ``OMNISIM_IPC_NONCE`` machinery (the env-var
    name is embedded verbatim in the binary) so a stale-libController IPC
    mismatch becomes detectable from the fingerprint -- see ``collect()``.
    """
    import hashlib
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        p = repo_root / rel
        if p.exists():
            try:
                data = p.read_bytes()
                st = p.stat()
                return {"path": rel, "sha256": hashlib.sha256(data).hexdigest()[:16],
                        "size": st.st_size, "mtime": int(st.st_mtime),
                        "ipc_nonce": (b"OMNISIM_IPC_NONCE" in data)}
            except Exception:
                return {"path": rel, "sha256": None, "size": None, "mtime": None,
                        "ipc_nonce": None}
    return None


def _libcontroller_info(repo_root: Path) -> dict | None:
    """Provenance of the C controller library (libController).

    db31686a hashed the ENGINE binary but not this -- and a libController that
    trailed the engine is exactly what silently broke every demo this session:
    the engine rebuilt with the nonce-salted intern pipe
    (``webots-<tmpId>-<nonce>-<robot>``) while the shipped ``Controller.dll``
    still spoke the legacy ``webots-<tmpId>-<robot>``; the two listened on
    different pipes and never met, so every controller blocked forever in
    ``Robot()`` and the sim never stepped (commit 6eea9d76). A fingerprint that
    records the engine but not its controller lib is blind to the #1 way a
    "loads fine" build produces zero ticks -- so we hash the lib and, like the
    engine, note whether it carries the ``OMNISIM_IPC_NONCE`` machinery.
    """
    import hashlib
    for rel in ("lib/controller/Controller.dll",         # Windows
                "lib/controller/libController.so",        # Linux
                "lib/controller/libController.dylib"):    # macOS
        p = repo_root / rel
        if p.exists():
            try:
                data = p.read_bytes()
                st = p.stat()
                return {"path": rel, "sha256": hashlib.sha256(data).hexdigest()[:16],
                        "size": st.st_size, "mtime": int(st.st_mtime),
                        "ipc_nonce": (b"OMNISIM_IPC_NONCE" in data)}
            except Exception:
                return {"path": rel, "sha256": None, "size": None, "mtime": None,
                        "ipc_nonce": None}
    return None


# The physics/inference packages whose exact versions decide cross-machine
# behaviour. Read from the BUNDLE's dist-info directory names, not from
# importlib in this interpreter: the fingerprint often runs in a different
# python than the engine's embedded one, and importlib.metadata is ambiguous
# anyway when a stale dist-info survives a pip --target upgrade (seen live
# 2026-07-16: newton/mujoco/mujoco_warp/warp_lang each carried two).
_STACK_PKGS = ("newton", "warp_lang", "mujoco", "mujoco_warp", "torch",
               "onnxruntime")


def _scan_dist_infos(site: Path) -> tuple[dict, list]:
    """{name: version} from dist-info dir names + duplicate-name list."""
    versions: dict[str, str] = {}
    dupes: list[str] = []
    for d in sorted(site.glob("*.dist-info")):
        m = re.match(r"^(.+)-([0-9][^-]*)\.dist-info$", d.name)
        if not m:
            continue
        name, ver = m.group(1).lower(), m.group(2)
        if name in versions and name not in dupes:
            dupes.append(name)
        versions[name] = ver
    return versions, dupes


def _runtime_stack(repo_root: Path) -> dict:
    """{pkg: (version, where)} resolved the way the ENGINE resolves imports.

    On Windows the embedded interpreter's path is the bundle's _pth file:
    the bundled site-packages first, then any extra site-packages lines the
    bundler appended (torch deliberately rides that system fallback -- it is
    not in DEPLOY_STACK). A package's version therefore depends on WHERE the
    engine finds it, and cross-machine drift hides in the system-resolved
    ones, so each entry is tagged bundle/system.
    """
    bin_dir = repo_root / "msys64/mingw64/bin"
    site = bin_dir / "newton-runtime/site-packages"
    if not site.is_dir():
        # Linux / unbundled: the embedded interpreter is the system python3,
        # so this interpreter's view is the best available answer.
        return {"source": "interpreter",
                "versions": {p: _pkg_ver(p.replace("warp_lang", "warp"))
                             for p in _STACK_PKGS},
                "origin": {p: "interpreter" for p in _STACK_PKGS},
                "anomalies": []}
    versions, dupes = _scan_dist_infos(site)
    origin = {n: "bundle" for n in versions}
    # walk the _pth fallback site-packages exactly as the engine would
    for pth in bin_dir.glob("python*._pth"):
        for line in pth.read_text(encoding="utf-8", errors="replace").splitlines():
            line = line.strip()
            if not line.lower().endswith("site-packages") or line.startswith("newton-runtime"):
                continue
            extra = Path(line)
            if not extra.is_dir():
                continue
            ev, _ = _scan_dist_infos(extra)
            for name, ver in ev.items():
                if name not in versions:      # bundle always wins, like the path order
                    versions[name] = ver
                    origin[name] = "system"
        break
    return {"source": "bundle+pth",
            "versions": {p: versions.get(p) for p in _STACK_PKGS},
            "origin": {p: origin.get(p, "absent") for p in _STACK_PKGS},
            "anomalies": dupes}


def _short_gpu(name: str | None) -> str:
    if not name:
        return "none"
    # "NVIDIA GeForce RTX 4090 (driver 555.42)" -> "RTX 4090"
    m = re.search(r"(RTX|GTX|A\d{2,4}|H\d{2,3}|L\d{1,2}|T\d{1,2})[\s-]?\w*", name)
    return m.group(0) if m else name.split("(")[0].strip()[:18]


def _machine_identity(gpu: str | None) -> dict:
    """A stable, low-churn, checkable identity for the MACHINE a result is from.

    The owner's problem: results get attributed to "local" / "this machine" with
    no way to tell WHICH box after the fact -- AGENTS.md quotes a laptop-5070-Ti
    throughput, the Go2 champion was trained on a RunPod 4090, and this box is an
    RTX 3060 laptop; a 4090-pod result reads identically to a laptop one, and
    "this machine" in a commit is unresolvable later. This records an identity a
    commit/result can NAME and a later run can re-derive and match.

    Signals are chosen to be STABLE (they don't churn run-to-run) and
    NON-PERSONAL: the GPU *model* (driver version stripped so a driver bump does
    not churn the id -- same convention as the conformance fingerprint), the CPU
    brand string, core count and OS family. The hostname is folded into the id
    hash (so two same-spec boxes still get distinct ids) and reported as a short
    HASH -- a raw hostname can carry a personal name and these dicts land
    verbatim in committed result files, so the box stays identifiable without
    being named.
    """
    import hashlib
    try:
        host = platform.node() or ""
    except Exception:
        host = ""
    try:
        cpu = platform.processor() or platform.machine() or ""
    except Exception:
        cpu = ""
    # On Linux `platform.processor()` returns the bare architecture ("x86_64"),
    # so a Linux row names no CPU at all. That bit during the multi-machine
    # campaign (2026-08-17): two RunPod hosts were compared and neither CPU
    # could be named afterwards from the committed rows -- precisely the "a
    # result that cannot name its box is not a result" failure this module
    # exists to prevent. /proc/cpuinfo has the model, so read it when the
    # generic value is all `platform` gave us.
    #
    # ⚠ This enriches the REPORTED cpu only. `id` keeps hashing the generic
    # value (see `cpu_for_id` below) on purpose: `id` is the join key across
    # campaigns, and changing what feeds it would silently renumber every
    # Linux machine, so rows taken before and after this commit would look
    # like different boxes. Host and GPU already separate distinct hosts in
    # the hash, so nothing was being collapsed -- only the display was blank.
    cpu_for_id = cpu
    if cpu in ("", "x86_64", "aarch64", "arm64") and sys.platform.startswith("linux"):
        try:
            with open("/proc/cpuinfo", encoding="utf-8", errors="replace") as fh:
                for line in fh:
                    if line.startswith(("model name", "Model")):
                        cpu = line.split(":", 1)[1].strip() or cpu
                        break
        except Exception:
            pass
    gpu_model = gpu.split("(driver")[0].strip() if gpu else ""
    osfam = platform.platform(terse=True)
    try:
        cores = os.cpu_count()
    except Exception:
        cores = None
    payload = "|".join([gpu_model, cpu_for_id, osfam, host])
    mid = hashlib.sha256(payload.encode("utf-8", "replace")).hexdigest()[:12]
    # host ships HASHED: a raw hostname is a personal identifier and committed
    # result files carry this dict verbatim (a laptop hostname reached the
    # committed bench results on 2026-07-24 and had to be scrubbed). The hash
    # keeps "same box?" answerable without naming the box.
    host_id = ("h" + hashlib.sha256(host.encode("utf-8", "replace")).hexdigest()[:10]) if host else "?"
    return {"id": mid, "host": host_id, "gpu": gpu_model or "none",
            "cpu": cpu or "?", "cores": cores}


def collect(robot=None, *, engine_log_path=None, repo_root=None, warm=None) -> dict:
    """Gather the full fingerprint. Never raises."""
    root, home_source, home_warns = _resolve_home(repo_root)
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
    warns.extend(home_warns)  # phantom OMNISIM_HOME/WEBOTS_HOME made visible

    intent = {k: os.environ[k] for k in _INTENT if os.environ.get(k)}
    # Intent-vs-reality: required Newton but didn't get a clean finalise.
    if intent.get("OMNISIM_REQUIRE_NEWTON", "0") not in ("0", "", "false", "False"):
        if not healthy:
            warns.append("OMNISIM_REQUIRE_NEWTON is set but Newton did not cleanly "
                         "finalise -- the engine should have aborted; investigate.")
    # Intent-vs-reality: a RETIRED backend knob is set. It is recorded (above) so
    # the provenance stays complete, and warned about here because it no longer
    # does what its name says: src/ode was DELETED (commit bdc02139). The engine
    # now IGNORES it (verified 2026-08-08 -- the run comes up on Newton and writes
    # a normal sidecar), so the recorded INTENT and the backend that actually ran
    # disagree, which is precisely the mismatch this fingerprint exists to
    # surface. On an earlier build the same day it WAS honoured, and then the run
    # had no physics at all (a body sat bit-identical to its authored pose for
    # 3000 ms) -- so a result recorded under this knob is ambiguous at best and
    # void at worst, depending on which build produced it.
    for knob in _RETIRED_ODE_KNOBS:
        if intent.get(knob):
            warns.append(
                f"{knob}={intent[knob]} is set and is RETIRED: ODE was deleted "
                "(src/ode, commit bdc02139). It does NOT give a legacy-physics "
                "run. Current engines IGNORE it, so your recorded intent does not "
                "match the backend that ran; older post-deletion builds honoured "
                "it and produced no physics at all. Either way the knob makes this "
                "result ambiguous -- unset it and re-measure.")

    stack = _runtime_stack(root)
    for name in stack["anomalies"]:
        warns.append(f"runtime bundle has DUPLICATE dist-info metadata for "
                     f"'{name}' -- its reported version is untrustworthy; run "
                     f"scripts/packaging/audit_dist_info.py --repair")

    gpu = _nvidia_gpu()
    binary = _binary_info(root)
    libctrl = _libcontroller_info(root)
    # Engine <-> libController IPC-nonce ABI check. The engine names the intern
    # pipe webots-<tmpId>-<nonce>-<robot>; a libController without the nonce
    # falls back to webots-<tmpId>-<robot>, so the two listen on different pipes
    # and never meet -- EVERY controller blocks forever in Robot(), the sim
    # never steps, and the headless runner still prints PASS. That is the fault
    # that silently broke every demo this session; `doctor` gates it definitively
    # (exact symbol probe, commit 6eea9d76). The fingerprint must at least SEE it
    # rather than lie by omission, so we mirror the check from the recorded
    # provenance (definitive nonce-mismatch first, then an mtime-skew heuristic).
    if binary and libctrl and binary.get("ipc_nonce") and libctrl.get("ipc_nonce") is False:
        warns.append("ENGINE/libController IPC-NONCE MISMATCH: the engine expects "
                     "the nonce-salted intern pipe but " + libctrl["path"] + " does "
                     "not speak it -- every controller will block forever in Robot() "
                     "and the sim will never step (a headless run still prints PASS). "
                     "Rebuild the controller lib; confirm with 'python -m omnisim doctor'.")
    elif (binary and libctrl and binary.get("mtime") and libctrl.get("mtime")
          and binary["mtime"] - libctrl["mtime"] > 86400):
        warns.append("libController (" + libctrl["path"] + ") is >24h OLDER than the "
                     "engine binary -- possible ABI/IPC drift. If controllers hang in "
                     "Robot(), rebuild the controller lib; run 'python -m omnisim doctor'.")

    fp = {
        "physics": physics,
        "solver": scan.get("solver"),
        "scan": scan,
        "engine_log": str(log_path),
        "home": {"root": str(root), "source": home_source},
        "load": ("warm" if warm else "cold") if warm is not None else "unknown",
        "warmup_token": bool(os.environ.get("OMNISIM_WARMUP_TOKEN")),
        "gpu": gpu,
        "machine": _machine_identity(gpu),
        "build": _git_commit(root),
        "binary": binary,
        "libcontroller": libctrl,
        "stack": stack,
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
    m = fp.get("machine") or {}
    b = fp.get("binary")
    lc = fp.get("libcontroller")
    home = fp.get("home") or {}
    _yn = lambda x: "yes" if x else ("?" if x is None else "NO")
    lines = [
        f"{prefix} ===== OmniSim environment fingerprint =====",
        f"{prefix} physics : {fp['physics']}"
        + (f"   solver='{fp['solver']}'" if fp["solver"] else ""),
        f"{prefix} machine : id={m.get('id', '?')}  host={m.get('host', '?')}"
        f"  cpu={m.get('cpu', '?')}  cores={m.get('cores')}",
        f"{prefix} load    : {load}   warmup_token={fp['warmup_token']}",
        f"{prefix} gpu     : {fp['gpu'] or 'no nvidia-smi on PATH (no NVIDIA driver?)'}",
        f"{prefix} build   : {fp['build'] or '?'}  os={fp['os']}  py={fp['python']}"
        f"  numpy={fp['numpy']}  ort={fp['onnxruntime']}",
        f"{prefix} binary  : "
        + (f"{b['path']}  sha256={b['sha256']}  mtime={b['mtime']}"
           f"  nonce={_yn(b.get('ipc_nonce'))}" if b else "NOT FOUND"),
        f"{prefix} libctrl : "
        + (f"{lc['path']}  sha256={lc['sha256']}  mtime={lc['mtime']}"
           f"  nonce={_yn(lc.get('ipc_nonce'))}" if lc else "NOT FOUND"),
        f"{prefix} stack   : ({fp['stack']['source']}) "
        + "  ".join(
            f"{k}={v or '?'}"
            + ("[sys]" if fp["stack"].get("origin", {}).get(k) == "system" else "")
            for k, v in fp["stack"]["versions"].items()),
        f"{prefix} knobs   : {knobs or '(defaults)'}",
        f"{prefix} intent  : {intent}",
        f"{prefix} home    : {home.get('root', '?')}  (source={home.get('source', '?')})",
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
    mid = (fp.get("machine") or {}).get("id", "?")
    return (f"OmniSim  physics={phys}  gpu={_short_gpu(fp['gpu'])}  "
            f"load={fp['load'].upper()}  build={fp['build'] or '?'}  machine={mid}")


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
