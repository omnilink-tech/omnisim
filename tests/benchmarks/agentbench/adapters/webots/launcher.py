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

"""Launch one **upstream Webots R2025a** run headless in WSL2, from Windows.

The execution half of the Phase W control arm
(``docs/developer/agent-edge-validation-plan.md`` Phase R item 4). The grading
half already exists (:mod:`agentbench.adapters.webots.recording` /
``evidence``); this module produces the artifact set that half reads:

1. locates the world's Webots project (``worlds/`` + ``controllers/``, same
   root-resolution walk the OmniSim adapter transcribes from
   ``WbProject::projectPathFromWorldFile``);
2. writes a **sibling copy** of the world with the grader-owned Supervisor
   recorder appended (``webots_lane/controllers/agentbench_webots_recorder``)
   -- upstream has no ``--duration`` and no auto-exit
   (``webots-control-baseline.md`` sec. 3), so the recorder owns termination
   via ``simulationQuit()``;
3. copies the project into a WSL-side workdir and runs the baseline sec. 3
   invocation::

       WEBOTS_HOME=/opt/upstream-webots/R2025a timeout N xvfb-run -a \\
         $WEBOTS_HOME/webots --batch --mode=fast --no-rendering --minimize \\
         --stdout --stderr --log-performance=<out>/perf.txt,<steps> \\
         --port=<port> <world>

   ``WEBOTS_HOME`` is **per-process only** -- it is prefixed onto that one
   command line and never written to any profile file (baseline sec. 2);
4. copies the artifact run-dir back onto the Windows side and writes
   ``process.json`` (exit code, timed_out, wall_s, attempts_used, perf_log,
   webots_version, command) -- the process facts an in-world controller cannot
   honestly attest.

Port discipline (baseline sec. 6): the default TCP port upstream uses is 1234,
the same one OmniSim auto-scans (1234-1244). Every port this module accepts
must sit in ``WEBOTS_PORT_RANGE`` (1500-1510); WSL2's network namespace is the
structural backstop, the range is the belt.

Everything WSL-independent (command construction, path translation, stanza
injection, the artifact-dir contract) is a pure function, tested in
``test_launcher.py`` with no WSL and no Webots.
"""

from __future__ import annotations

import json
import re
import subprocess
import time
import uuid
from pathlib import Path, PureWindowsPath

# --- constants: the verified install (BRINGUP.md records hashes) ------------

DISTRO = "Ubuntu-22.04"
WEBOTS_HOME = "/opt/upstream-webots/R2025a"

# Outside OmniSim's 1234-1244 auto-scan range (baseline sec. 6).
WEBOTS_PORT_RANGE = (1500, 1510)
DEFAULT_PORT = 1504

DEFAULT_TIMEOUT_S = 420.0
DEFAULT_DURATION_S = 10.0

# Same marker prefix the OmniSim adapter uses, so common.worldtext's
# pick_artifact skips injected copies on both arms.
INJECTED_PREFIX = "_agentbench_"

RECORDER_NAME = "agentbench_webots_recorder"
WEBOTS_LANE = Path(__file__).resolve().parent / "webots_lane"
RECORDER_DIR = WEBOTS_LANE / "controllers" / RECORDER_NAME

# ``timeout``'s exit code when it had to kill the process.
_TIMEOUT_RC = 124

RECORDER_STANZA = """
# --- appended by the AgentBench Webots launcher (Phase W) -------------------
Robot {
  name "agentbench_recorder"
  controller "%s"
  supervisor TRUE
  controllerArgs [
%s
  ]
}
"""

_BTS_RE = re.compile(r"basicTimeStep\s+([0-9.]+)")
# rotation <x> <y> <z> <angle> with |angle| within 1e-3 of pi -- the
# baseline sec. 5 trap 3 hazard (a yaw of exactly +/-pi sent a Moose
# non-finite within ten steps, reproducibly).
_ROTATION_RE = re.compile(
    r"rotation\s+[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s+([-0-9.eE]+)")
_PI = 3.14159265358979


# --- pure helpers (tested without WSL) ---------------------------------------


def win_to_wsl_path(p):
    """``O:\\omnisim\\x`` -> ``/mnt/o/omnisim/x``. POSIX paths pass through.

    Raises ``ValueError`` for UNC paths -- a ``\\\\wsl$`` or network path has
    no ``/mnt`` translation and silently mistranslating one would point the
    run at a directory that does not exist.
    """
    s = str(p)
    if s.startswith("/"):
        return s
    if s.startswith("\\\\") or s.startswith("//"):
        raise ValueError("UNC path has no /mnt translation: %r" % s)
    wp = PureWindowsPath(s)
    if not wp.drive or ":" not in wp.drive:
        raise ValueError("not an absolute Windows path: %r" % s)
    drive = wp.drive[0].lower()
    rest = "/".join(wp.parts[1:])
    return "/mnt/%s/%s" % (drive, rest) if rest else "/mnt/%s" % drive


def wsl_to_win_path(p):
    """``/mnt/o/x/y`` -> ``O:\\x\\y``. Only ``/mnt/<drive>/`` translates."""
    s = str(p).replace("\\", "/")
    m = re.match(r"^/mnt/([a-zA-Z])(/.*)?$", s)
    if not m:
        raise ValueError("not a /mnt/<drive> path: %r" % s)
    drive = m.group(1).upper()
    rest = (m.group(2) or "/").replace("/", "\\")
    return "%s:%s" % (drive, rest)


def parse_basic_time_step_ms(world_text, default=32.0):
    """``WorldInfo.basicTimeStep`` in ms, upstream's default when unstated."""
    m = _BTS_RE.search(world_text or "")
    if not m:
        return float(default)
    try:
        return float(m.group(1))
    except ValueError:
        return float(default)


def pi_rotation_lines(world_text, tol=1e-3):
    """1-based line numbers whose rotation angle is within ``tol`` of +/-pi.

    Baseline sec. 5 trap 3: an axis-angle of exactly +/-pi reproducibly sent a
    body non-finite within ten steps. The launcher does not rewrite the
    agent's world (that would be grading a world the agent did not write) --
    it reports the hazard so the row can say why a NaN happened.
    """
    hits = []
    for i, line in enumerate((world_text or "").splitlines(), 1):
        for m in _ROTATION_RE.finditer(line):
            try:
                ang = float(m.group(1))
            except ValueError:
                continue
            if abs(abs(ang) - _PI) < tol:
                hits.append(i)
                break
    return hits


def recorder_stanza(out_dir_wsl, *, duration, contact_steps=10,
                    controller=RECORDER_NAME, solids=(), links=0,
                    solid_tracks=None, scan_solids=None):
    """The Supervisor Robot stanza appended to the world copy.

    ``contact_steps`` is the contact-sampling WINDOW in basic timesteps:
    ``N`` samples the first N steps, ``0`` samples none (and the recorder then
    reports ``supported: false``, because a run that never looked must not read
    as "nothing was hit"), and **``-1`` samples the whole run** -- which is what
    a collision assertion phrased over a whole navigation run needs, and what
    the MuJoCo arm does unconditionally.

    ``solids`` / ``links`` / ``solid_tracks`` / ``scan_solids`` are the SAME
    four knobs the OmniSim arm's ``headless.inject_recorder`` emits, spelled
    the same way, so a task asking for an end-effector track gets one on both
    arms or on neither. Each is emitted only when it differs from the
    recorder's own default, so a task that asks for none produces a
    byte-identical stanza to every run recorded before these existed.

    ``scan_solids`` caps the name-free t=0 scene scan; leaving it ``None``
    takes the recorder's default (on) and ``0`` turns the scan off.
    """
    args = [
        '    "--out-dir=%s"' % out_dir_wsl,
        '    "--duration=%r"' % float(duration),
        '    "--contact-steps=%d"' % int(contact_steps),
    ]
    if solids:
        args.append('    "--solids=%s"' % ",".join(solids))
    if links:
        args.append('    "--links=%d"' % int(links))
    if solid_tracks:
        args.append('    "--solid-tracks=%s"' % solid_tracks)
    if scan_solids is not None:
        args.append('    "--scan-solids=%d"' % int(scan_solids))
    return RECORDER_STANZA % (controller, "\n".join(args))


def inject_recorder_text(world_text, out_dir_wsl, *, duration,
                         contact_steps=10, solids=(), links=0,
                         solid_tracks=None, scan_solids=None):
    """World text + the recorder stanza. Never touches the original file."""
    text = world_text if world_text.endswith("\n") else world_text + "\n"
    return text + recorder_stanza(out_dir_wsl, duration=duration,
                                  contact_steps=contact_steps, solids=solids,
                                  links=links, solid_tracks=solid_tracks,
                                  scan_solids=scan_solids)


def project_root_for_world(world):
    """The project root upstream will resolve: nearest ``worlds`` ancestor's
    parent, else the world's parent's parent (same walk as the OmniSim
    adapter transcribes from ``WbProject::projectPathFromWorldFile``)."""
    world_dir = Path(world).resolve().parent
    cur = world_dir
    while True:
        if cur.name == "worlds":
            return cur.parent
        if cur.parent == cur:
            break
        cur = cur.parent
    return world_dir.parent


def check_port(port):
    lo, hi = WEBOTS_PORT_RANGE
    if not (lo <= int(port) <= hi):
        raise ValueError(
            "port %r is outside the Webots lane's range %d-%d; OmniSim "
            "auto-scans 1234-1244 and the two lanes must not share ports "
            "(webots-control-baseline.md sec. 6)" % (port, lo, hi))
    return int(port)


def webots_invocation(world_wsl, out_dir_wsl, *, perf_steps, port=DEFAULT_PORT,
                      timeout_s=DEFAULT_TIMEOUT_S, webots_home=WEBOTS_HOME):
    """The baseline sec. 3 command line, as one bash-ready string.

    ``WEBOTS_HOME`` is a per-process prefix on this one line -- never
    persisted (baseline sec. 2).
    """
    check_port(port)
    return (
        'WEBOTS_HOME=%(home)s timeout %(t)d xvfb-run -a %(home)s/webots '
        '--batch --mode=fast --no-rendering --minimize --stdout --stderr '
        '--log-performance="%(out)s/perf.txt",%(steps)d --port=%(port)d '
        '"%(world)s"'
        % {"home": webots_home, "t": int(timeout_s), "out": out_dir_wsl,
           "steps": int(perf_steps), "port": int(port), "world": world_wsl})


def build_run_script(*, work_dir, project_src_wsl, injected_world_wsl,
                     recorder_src_wsl, run_dir_wsl, world_name,
                     perf_steps, port=DEFAULT_PORT,
                     timeout_s=DEFAULT_TIMEOUT_S, webots_home=WEBOTS_HOME,
                     extra_project_paths=()):
    """The whole WSL-side run as one bash script (a pure string).

    Layout it builds::

        <work_dir>/project/worlds/<world_name>       the injected world copy
        <work_dir>/project/controllers/...           the world's + the recorder
        <work_dir>/project/<extra_project_paths>     project files a controller
                                                     reads at run time
        <work_dir>/out/                              every artifact

    stdout/stderr are captured to files (upstream writes NO log file --
    recording.py's console candidates are exactly these names), the perf log
    goes to ``out/perf.txt``, and the artifact dir is copied back to the
    Windows-side run dir at the end. Markers ``RC=`` / ``WALL=`` carry the
    process facts back to the parent.

    ``extra_project_paths`` are project-root-relative POSIX paths to carry
    across the WSL boundary alongside ``worlds/`` and ``controllers/``. Empty
    by default, so every existing caller builds a byte-identical script. It
    exists because the project is the unit of delivery and only two of its
    directories used to travel: a controller reading a task asset by a
    relative path outside its own directory resolved it on the Windows side
    and found nothing here. MEASURED 2026-08-11 on R4's first cell -- the
    agent's controller opened ``../../benchmark_assets/scene.json`` at
    ``__init__``, died, and its robot recorded 0.00 m over 150 s.
    ``cc_lane/run_cc_cell.stage_task_assets`` is what puts those files in the
    project root, and it copies them from the frozen task tree, so nothing an
    agent wrote can arrive by this route.
    """
    inv = webots_invocation(
        "%s/project/worlds/%s" % (work_dir, world_name),
        "%s/out" % work_dir, perf_steps=perf_steps, port=port,
        timeout_s=timeout_s, webots_home=webots_home)
    extras = []
    for rel in extra_project_paths:
        rel = str(rel).replace("\\", "/").strip()
        # No traversal and no absolute escape: this list decides what crosses
        # into the run, so it is validated rather than trusted. Checked BEFORE
        # any normalisation, or stripping a leading slash would launder an
        # absolute path into an accepted one.
        if not rel or rel.startswith("/") or ".." in rel.split("/"):
            continue
        parent = rel.rsplit("/", 1)[0] if "/" in rel else ""
        extras.append(
            'if [ -e "%(src)s/%(rel)s" ]; then mkdir -p '
            '"$WORK/project/%(par)s" && cp -r "%(src)s/%(rel)s" '
            '"$WORK/project/%(rel)s"; fi'
            % {"src": project_src_wsl, "rel": rel,
               "par": parent or "."})
    return "\n".join([
        "set -u",
        'WORK="%s"' % work_dir,
        'rm -rf "$WORK"',
        'mkdir -p "$WORK/project/worlds" "$WORK/project/controllers" '
        '"$WORK/out"',
        # the world's own controllers, if it has any
        'if [ -d "%s/controllers" ]; then cp -r "%s/controllers/." '
        '"$WORK/project/controllers/"; fi' % (project_src_wsl,
                                              project_src_wsl),
        # ...and the project files a controller reads at run time
        *extras,
        # the grader's recorder ALWAYS wins: copied last, over anything the
        # project supplied under the same name
        'rm -rf "$WORK/project/controllers/%s"' % RECORDER_NAME,
        'cp -r "%s" "$WORK/project/controllers/%s"' % (recorder_src_wsl,
                                                       RECORDER_NAME),
        'cp "%s" "$WORK/project/worlds/%s"' % (injected_world_wsl,
                                               world_name),
        'cat "%s/resources/version.txt" > "$WORK/out/version.txt" '
        '2>/dev/null || true' % webots_home,
        'S=$(date +%s.%N)',
        "%s > \"$WORK/out/stdout.log\" 2> \"$WORK/out/stderr.log\"" % inv,
        "RC=$?",
        'E=$(date +%s.%N)',
        'echo "RC=$RC"',
        'awk "BEGIN{printf \\"WALL=%.3f\\n\\", $E-$S}"',
        'mkdir -p "%s"' % run_dir_wsl,
        'cp -r "$WORK/out/." "%s/"' % run_dir_wsl,
        'rm -rf "$WORK"',
        'echo "COPIED=1"',
    ]) + "\n"


def parse_markers(text):
    """``RC=`` / ``WALL=`` / ``COPIED=`` markers out of the script's stdout."""
    out = {"rc": None, "wall_s": None, "copied": False}
    for line in (text or "").splitlines():
        line = line.strip()
        if line.startswith("RC="):
            try:
                out["rc"] = int(line[3:])
            except ValueError:
                pass
        elif line.startswith("WALL="):
            try:
                out["wall_s"] = float(line[5:])
            except ValueError:
                pass
        elif line.startswith("COPIED="):
            out["copied"] = line[7:] == "1"
    return out


def process_facts(*, rc, timed_out, wall_s, attempts_used, perf_log, command,
                  webots_version, world):
    """The ``process.json`` document, exactly the shape recording.py reads."""
    return {
        "exit_code": rc,
        "timed_out": bool(timed_out),
        "wall_s": wall_s,
        "attempts_used": int(attempts_used),
        "perf_log": str(perf_log) if perf_log else None,
        "webots_version": webots_version,
        "command": command,
        "world": str(world) if world else None,
    }


# --- the launch --------------------------------------------------------------


class WebotsLaunch:
    """What one launch attempt produced. Facts only, no interpretation."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.world = None
        self.injected_world = None
        self.rc = None
        self.wall_s = None
        self.timed_out = False
        self.attempts_used = 1
        self.command = None
        self.webots_version = None
        self.script_output = ""
        self.pi_rotation_warnings = []
        self.error = None

    def as_dict(self):
        return {
            "run_dir": str(self.run_dir),
            "world": str(self.world) if self.world else None,
            "injected_world": (str(self.injected_world)
                               if self.injected_world else None),
            "rc": self.rc, "wall_s": self.wall_s,
            "timed_out": self.timed_out,
            "attempts_used": self.attempts_used,
            "command": self.command,
            "webots_version": self.webots_version,
            "pi_rotation_warnings": list(self.pi_rotation_warnings),
            "error": self.error,
        }


def _wsl(script_path_win, *, distro=DISTRO, timeout_s=600.0):
    """Run one bash script inside WSL. Returns (rc, stdout+stderr text)."""
    cmd = ["wsl", "-d", distro, "--", "bash",
           win_to_wsl_path(script_path_win)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout_s, encoding="utf-8",
                              errors="replace")
        return proc.returncode, (proc.stdout or "") + (proc.stderr or "")
    except subprocess.TimeoutExpired as exc:
        return None, "OUTER-TIMEOUT after %ss: %r" % (timeout_s, exc)
    except FileNotFoundError:
        return None, "wsl.exe not found on PATH"


def launch(world, run_dir, *, duration=DEFAULT_DURATION_S, contact_steps=10,
           port=DEFAULT_PORT, timeout_s=DEFAULT_TIMEOUT_S, attempts=2,
           distro=DISTRO, webots_home=WEBOTS_HOME, keep_injected=True,
           solids=(), links=0, solid_tracks=None, scan_solids=None,
           extra_project_paths=()):
    """Run ``world`` headless under upstream Webots in WSL2. **Never raises**
    on a failed run -- a broken simulator is a measurement.

    Returns ``(run_dir, facts)`` where ``facts`` is the ``process.json``
    content (also written into ``run_dir``), and ``run_dir`` holds the full
    artifact set ``recording.read_run`` parses: ``trajectory.json``,
    ``roster.json``, ``contacts.json``, ``completion.json``, ``process.json``,
    ``stdout.log`` / ``stderr.log``, ``perf.txt``.

    A retry is taken only when the run left NO recorder evidence at all
    (no ``roster.json``) -- a world the recorder could enumerate produced a
    verdict, and retrying a verdict would launder it.
    """
    res = WebotsLaunch(run_dir)
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    world = Path(world)
    res.world = world

    try:
        world_text = world.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        res.error = "world unreadable: %r" % (exc,)
        return _finish(res, run_dir)

    res.pi_rotation_warnings = [
        "line %d: rotation angle within 1e-3 of +/-pi "
        "(webots-control-baseline.md sec. 5 trap 3)" % ln
        for ln in pi_rotation_lines(world_text)]

    bts_ms = parse_basic_time_step_ms(world_text)
    perf_steps = max(1, int(round(duration * 1000.0 / bts_ms)))
    tag = "%s_%s" % (world.stem, uuid.uuid4().hex[:8])
    work_dir = "/tmp/agentbench_webots/%s" % tag
    out_dir_wsl = "%s/out" % work_dir

    # The injected sibling copy lives in the RUN dir (an artifact of the run),
    # and is copied into the WSL project under the world's own stem so any
    # relative references keep resolving.
    injected = run_dir / ("%s%s.wbt" % (INJECTED_PREFIX, world.stem))
    world_name = world.name
    try:
        injected.write_text(
            inject_recorder_text(world_text, out_dir_wsl, duration=duration,
                                 contact_steps=contact_steps, solids=solids,
                                 links=links, solid_tracks=solid_tracks,
                                 scan_solids=scan_solids),
            encoding="utf-8")
    except OSError as exc:
        res.error = "could not write the injected world: %r" % (exc,)
        return _finish(res, run_dir)
    res.injected_world = injected

    proj = project_root_for_world(world)
    try:
        script = build_run_script(
            work_dir=work_dir,
            project_src_wsl=win_to_wsl_path(proj),
            injected_world_wsl=win_to_wsl_path(injected),
            recorder_src_wsl=win_to_wsl_path(RECORDER_DIR),
            run_dir_wsl=win_to_wsl_path(run_dir.resolve()),
            world_name=world_name, perf_steps=perf_steps, port=port,
            timeout_s=timeout_s, webots_home=webots_home,
            extra_project_paths=extra_project_paths)
    except ValueError as exc:
        res.error = str(exc)
        return _finish(res, run_dir)
    res.command = webots_invocation(
        "%s/project/worlds/%s" % (work_dir, world_name), out_dir_wsl,
        perf_steps=perf_steps, port=port, timeout_s=timeout_s,
        webots_home=webots_home)

    script_path = run_dir / "_run_webots.sh"
    script_path.write_text(script, encoding="utf-8", newline="\n")

    for attempt in range(1, max(1, attempts) + 1):
        res.attempts_used = attempt
        t0 = time.monotonic()
        wsl_rc, out = _wsl(script_path, distro=distro,
                           timeout_s=timeout_s + 120.0)
        res.script_output = out
        markers = parse_markers(out)
        res.rc = markers["rc"]
        res.wall_s = (markers["wall_s"] if markers["wall_s"] is not None
                      else round(time.monotonic() - t0, 3))
        res.timed_out = (res.rc == _TIMEOUT_RC
                         or (wsl_rc is None and "OUTER-TIMEOUT" in out))
        if wsl_rc is None and not res.timed_out:
            res.error = out.strip()
        elif not markers["copied"]:
            res.error = "artifact copy-back did not run; script output: %s" \
                % out.strip()[-500:]
        # Evidence check: did the recorder leave anything? Retry only a run
        # with NO recorder evidence -- that is a launch failure, not a
        # verdict.
        if (run_dir / "roster.json").is_file() or attempt >= max(1, attempts):
            break
        time.sleep(2.0 * attempt)

    res.webots_version = _read_version(run_dir)
    return _finish(res, run_dir, keep_injected=keep_injected)


def _read_version(run_dir):
    p = Path(run_dir) / "version.txt"
    try:
        return p.read_text(encoding="utf-8").strip() or None
    except OSError:
        return None


def _finish(res, run_dir, *, keep_injected=True):
    facts = process_facts(
        rc=res.rc, timed_out=res.timed_out, wall_s=res.wall_s,
        attempts_used=res.attempts_used,
        perf_log=(run_dir / "perf.txt") if (run_dir / "perf.txt").is_file()
        else None,
        command=res.command, webots_version=res.webots_version,
        world=res.world)
    if res.error:
        facts["error"] = res.error
    if res.pi_rotation_warnings:
        facts["pi_rotation_warnings"] = list(res.pi_rotation_warnings)
    try:
        (run_dir / "process.json").write_text(json.dumps(facts, indent=1),
                                              encoding="utf-8")
    except OSError:
        pass
    if not keep_injected and res.injected_world is not None:
        try:
            Path(res.injected_world).unlink()
        except OSError:
            pass
    return run_dir, facts


__all__ = ["DEFAULT_PORT", "DISTRO", "INJECTED_PREFIX", "RECORDER_DIR",
           "RECORDER_NAME", "WEBOTS_HOME", "WEBOTS_PORT_RANGE",
           "WebotsLaunch", "build_run_script", "check_port",
           "inject_recorder_text", "launch", "parse_basic_time_step_ms",
           "parse_markers", "pi_rotation_lines", "process_facts",
           "project_root_for_world", "recorder_stanza", "webots_invocation",
           "win_to_wsl_path", "wsl_to_win_path"]
