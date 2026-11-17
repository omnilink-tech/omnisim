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

"""Run one **MuJoCo** cell: the parent-side half of this arm's execution path.

The counterpart of :mod:`agentbench.adapters.webots.launcher`, and much smaller
than it, for a reason worth stating plainly: MuJoCo has no simulator process to
install, no display to fake, no world file to inject a supervisor into and no
project layout to reproduce. A MuJoCo run is **one Python process** that
imports ``mujoco``, so this module's whole job is to start that process in a
directory it owns, with a timeout, and write down what happened to it.

That is a genuine and large advantage for this comparator on the *operational*
axis, and the reader of a cross-sim table should see it here rather than infer
it: the Webots arm needs WSL2, a 871 MB asset pre-seed, ``xvfb-run`` and a
copy-back; this arm needs an interpreter that can ``import mujoco``.

What the launcher does NOT do, and must not:

* it does not edit the agent's model or driver;
* it does not supply a driver. A MuJoCo scene is inert data; if the agent
  wrote no program to step it, that is the finding, and the run records "no
  driver" rather than the grader inventing one;
* it does not install anything. The interpreter is resolved
  (``$AGENTBENCH_MUJOCO_PYTHON``, else the parent's own), probed once for
  ``import mujoco``, and the probe result is published so a missing dependency
  reads as a missing dependency and not as a simulator that failed.

Everything except the actual ``subprocess.run`` is a pure function, tested in
``test_launcher.py`` with no MuJoCo and no child process.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNNER = HERE / "runner_main.py"
LANE = HERE / "mujoco_lane"

#: The default recording window when a task does not state one.
DEFAULT_DURATION_S = 10.0

#: Wall-clock guard handed to the CHILD, so it stops itself and still writes
#: its artifacts. The outer ``timeout`` below is the backstop for a child that
#: is wedged somewhere the recorder cannot see (a blocking viewer, a socket).
DEFAULT_WALL_LIMIT_S = 600.0
DEFAULT_TIMEOUT_S = 900.0

#: Environment variable naming the interpreter that has ``mujoco``.
PYTHON_ENV = "AGENTBENCH_MUJOCO_PYTHON"

#: Filenames the grader owns inside a run dir. The driver executes in the same
#: process as the recorder and could in principle write these itself -- the
#: recorder rewrites every one of them at ``finish()``, which runs last, so the
#: grader has the last word. Listed here because "you cannot fake it" is a
#: claim that should name its mechanism.
GRADER_ARTIFACTS = ("trajectory.csv", "trajectory.json", "roster.json",
                    "contacts.json", "completion.json", "model_info.json",
                    "model_load.json", "process.json")

#: Names never treated as the agent's driver when guessing one.
_NOT_A_DRIVER = {"__init__.py", "conftest.py", "setup.py"}


# --- pure helpers ------------------------------------------------------------


def resolve_python(explicit=None):
    """The interpreter to run the cell in: explicit, else env, else ours."""
    return str(explicit or os.environ.get(PYTHON_ENV) or sys.executable)


def probe_python(python=None, *, timeout_s=60.0):
    """``{"ok", "python", "mujoco_version", "error"}`` -- can it import mujoco?

    Run once per launch and published on every row. A cell that could not even
    import the library must not be reported as a simulator that failed to
    simulate; those are different facts and only one of them is about MuJoCo.
    """
    py = resolve_python(python)
    code = ("import json,mujoco;"
            "print(json.dumps({'v':mujoco.mj_versionString(),"
            "'p':getattr(mujoco,'__version__',None),'f':mujoco.__file__}))")
    try:
        proc = subprocess.run([py, "-c", code], capture_output=True, text=True,
                              timeout=timeout_s, encoding="utf-8",
                              errors="replace")
    except (OSError, subprocess.SubprocessError) as exc:
        return {"ok": False, "python": py, "mujoco_version": None,
                "error": repr(exc)}
    if proc.returncode != 0:
        return {"ok": False, "python": py, "mujoco_version": None,
                "error": (proc.stderr or proc.stdout or "").strip()[-400:]}
    try:
        doc = json.loads((proc.stdout or "").strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {"ok": False, "python": py, "mujoco_version": None,
                "error": "unparseable probe output: %r" % (proc.stdout,)}
    return {"ok": True, "python": py, "mujoco_version": doc.get("v"),
            "mujoco_python_version": doc.get("p"),
            "mujoco_file": doc.get("f"), "error": None}


def find_driver(model, explicit=None):
    """``(driver_path_or_None, rule)`` -- the program that steps the model.

    A MuJoCo deliverable is **two** files: an MJCF scene and a Python program
    that loads and steps it. The scene alone is inert -- there is no
    ``controller`` field to name a behaviour and no engine that would run one
    -- so the grader has to have a discovery rule, and it has to be one an
    author can satisfy without being told the benchmark's conventions.

    In order:

    1. an explicitly supplied path;
    2. ``<model stem>.py`` beside the model -- the convention this arm asks
       for, and the one a collector should use when it lifts the pair out of a
       workspace;
    3. the only ``*.py`` in the model's directory, when there is exactly one;
    4. otherwise **nothing, and the reason**. Several candidates are NOT
       guessed between: picking one would be the grader choosing which program
       the agent meant, and a wrong pick reads as an agent failure.
    """
    model = Path(model)
    if explicit:
        p = Path(explicit)
        return (p, "explicitly supplied") if p.is_file() else (
            None, "the supplied driver %s does not exist" % p)
    sib = model.with_suffix(".py")
    if sib.is_file():
        return sib, "<model stem>.py beside the model"
    try:
        cands = sorted(p for p in model.parent.glob("*.py")
                       if p.name not in _NOT_A_DRIVER)
    except OSError:
        cands = []
    if len(cands) == 1:
        return cands[0], "the only .py file in the model's directory"
    if not cands:
        return None, ("no driver: neither %s nor any .py file beside the "
                      "model. A MuJoCo scene does not move on its own."
                      % sib.name)
    return None, ("%d candidate .py files beside the model (%s) and none named "
                  "%s; refusing to guess which one the agent meant"
                  % (len(cands), ", ".join(p.name for p in cands[:4]),
                     sib.name))


def runner_command(python, model, driver, out_dir, *, duration=None,
                   wall_limit=None, contact_stride=1, body_cap=None,
                   pair_cap=None, driver_args=()):
    """The child's argv. A pure function so the shape is testable."""
    cmd = [str(python), str(RUNNER), "--model", str(model),
           "--out", str(out_dir)]
    if driver:
        cmd += ["--driver", str(driver)]
    if duration is not None:
        cmd += ["--duration", repr(float(duration))]
    if wall_limit is not None:
        cmd += ["--wall-limit", repr(float(wall_limit))]
    if contact_stride and int(contact_stride) != 1:
        cmd += ["--contact-stride", str(int(contact_stride))]
    if body_cap:
        cmd += ["--body-cap", str(int(body_cap))]
    if pair_cap:
        cmd += ["--pair-cap", str(int(pair_cap))]
    for a in driver_args:
        cmd += ["--driver-arg", str(a)]
    return cmd


def child_env(base=None, extra=None):
    """The child's environment.

    ``MUJOCO_GL`` is deliberately **not** forced. A driver that opens a viewer
    or a renderer will fail on a headless box, and that failure is the honest
    outcome of a benchmark that is headless for every arm -- pinning a software
    GL backend here would be the grader changing what the agent's program does.
    """
    env = dict(os.environ if base is None else base)
    env.setdefault("PYTHONUNBUFFERED", "1")
    if extra:
        env.update({k: str(v) for k, v in extra.items()})
    return env


def process_facts(*, rc, timed_out, wall_s, attempts_used, command, python,
                  model, driver, driver_rule, probe):
    """The ``process.json`` document, exactly the shape recording.py reads."""
    return {
        "exit_code": rc,
        "timed_out": bool(timed_out),
        "wall_s": wall_s,
        "attempts_used": int(attempts_used),
        "command": list(command) if command else None,
        "python": str(python) if python else None,
        "mujoco_version": (probe or {}).get("mujoco_version"),
        "mujoco_python_version": (probe or {}).get("mujoco_python_version"),
        "interpreter_probe": probe,
        "model": str(model) if model else None,
        "driver": str(driver) if driver else None,
        "driver_rule": driver_rule,
    }


# --- the launch ---------------------------------------------------------------


class MujocoLaunch:
    """What one launch produced. Facts only, no interpretation."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.model = None
        self.driver = None
        self.driver_rule = ""
        self.python = None
        self.probe = None
        self.rc = None
        self.wall_s = None
        self.timed_out = False
        self.attempts_used = 1
        self.command = None
        self.error = None

    def as_dict(self):
        return {"run_dir": str(self.run_dir),
                "model": str(self.model) if self.model else None,
                "driver": str(self.driver) if self.driver else None,
                "driver_rule": self.driver_rule,
                "python": self.python, "interpreter_probe": self.probe,
                "rc": self.rc, "wall_s": self.wall_s,
                "timed_out": self.timed_out,
                "attempts_used": self.attempts_used,
                "command": self.command, "error": self.error}


def launch(model, run_dir, *, driver=None, duration=DEFAULT_DURATION_S,
           wall_limit=None, timeout_s=DEFAULT_TIMEOUT_S, contact_stride=1,
           body_cap=None, pair_cap=None, python=None, attempts=1,
           driver_args=(), env_extra=None):
    """Run ``model`` + its driver under the grader's recorder. **Never raises.**

    Returns ``(run_dir, facts)`` where ``facts`` is the ``process.json``
    content (also written into ``run_dir``), and ``run_dir`` holds the artifact
    set :func:`agentbench.adapters.mujoco.recording.read_run` parses.

    Retries are OFF by default (``attempts=1``) and that is a deliberate
    difference from the Webots and OmniSim arms. Both of those retry a known
    *launch* flake -- a Qt startup race, a WSL copy-back -- which exists
    because there is a simulator process with a GUI stack in it. Here there is
    a Python interpreter and an import; there is no such flake to absorb, and a
    retry loop would only ever re-roll a genuine result.
    """
    res = MujocoLaunch(run_dir)
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    res.model = Path(model)
    res.python = resolve_python(python)

    if not res.model.is_file():
        res.error = "the model file does not exist: %s" % res.model
        return _finish(res, run_dir)

    drv, rule = find_driver(res.model, driver)
    res.driver, res.driver_rule = drv, rule

    res.probe = probe_python(res.python)
    if not res.probe.get("ok"):
        res.error = ("this cell's interpreter cannot import mujoco (%s): %s. "
                     "That is a MISSING DEPENDENCY, not a simulator failure."
                     % (res.python, res.probe.get("error")))
        return _finish(res, run_dir)

    if wall_limit is None:
        wall_limit = min(DEFAULT_WALL_LIMIT_S, max(30.0, timeout_s - 60.0))

    res.command = runner_command(
        res.python, res.model, drv, run_dir, duration=duration,
        wall_limit=wall_limit, contact_stride=contact_stride,
        body_cap=body_cap, pair_cap=pair_cap, driver_args=driver_args)

    stdout_p = run_dir / "stdout.log"
    stderr_p = run_dir / "stderr.log"
    for attempt in range(1, max(1, int(attempts)) + 1):
        res.attempts_used = attempt
        t0 = time.monotonic()
        try:
            proc = subprocess.run(res.command, capture_output=True, text=True,
                                  timeout=timeout_s,
                                  cwd=str(res.model.parent),
                                  env=child_env(extra=env_extra),
                                  encoding="utf-8", errors="replace")
            res.rc, out, err = proc.returncode, proc.stdout, proc.stderr
        except subprocess.TimeoutExpired as exc:
            res.timed_out = True
            res.rc = None
            out = exc.stdout if isinstance(exc.stdout, str) else ""
            err = exc.stderr if isinstance(exc.stderr, str) else ""
            err = (err or "") + ("\n[agentbench] the child exceeded the outer "
                                 "%.0f s timeout and was killed; its own "
                                 "wall-clock guard (%.0f s) did not fire "
                                 "first, so it was blocked somewhere the "
                                 "recorder cannot see.\n"
                                 % (timeout_s, wall_limit))
        except OSError as exc:
            res.error = "could not start the child: %r" % (exc,)
            out = err = ""
        res.wall_s = round(time.monotonic() - t0, 4)
        _write_text(stdout_p, out)
        _write_text(stderr_p, err)
        if (run_dir / "completion.json").is_file() \
                or attempt >= max(1, int(attempts)):
            break
        time.sleep(1.0 * attempt)
    return _finish(res, run_dir)


def _finish(res, run_dir):
    facts = process_facts(
        rc=res.rc, timed_out=res.timed_out, wall_s=res.wall_s,
        attempts_used=res.attempts_used, command=res.command,
        python=res.python, model=res.model, driver=res.driver,
        driver_rule=res.driver_rule, probe=res.probe)
    if res.error:
        facts["error"] = res.error
    try:
        (Path(run_dir) / "process.json").write_text(
            json.dumps(facts, indent=1, sort_keys=True), encoding="utf-8")
    except OSError:
        pass
    return Path(run_dir), facts


def _write_text(path, text):
    try:
        Path(path).write_text(text or "", encoding="utf-8")
    except OSError:
        pass


__all__ = ["DEFAULT_DURATION_S", "DEFAULT_TIMEOUT_S", "DEFAULT_WALL_LIMIT_S",
           "GRADER_ARTIFACTS", "LANE", "MujocoLaunch", "PYTHON_ENV", "RUNNER",
           "child_env", "find_driver", "launch", "probe_python",
           "process_facts", "resolve_python", "runner_command"]
