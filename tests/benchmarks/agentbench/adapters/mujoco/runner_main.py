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

"""The child process of one MuJoCo cell: observe, then run the agent's driver.

Invoked as a plain script by :mod:`agentbench.adapters.mujoco.launcher` so that
nothing about the parent's ``sys.path`` has to reach it::

    <python> runner_main.py --model M.xml --driver M.py --out <dir> \\
             --duration 60 --wall-limit 600

Order of operations, and why it is this order:

1. **Compile the agent's model on its own, first.** ``MjModel.from_xml_path``
   is the only thing in MuJoCo that corresponds to "did the world load", and
   doing it here -- before the driver gets a chance to do anything -- separates
   *"the MJCF is invalid"* (an agent failure, and the whole of C1) from *"the
   MJCF is fine and the driver is broken"*. The result goes in
   ``model_load.json`` whether it succeeded or not.
2. **Install the recorder**, so every integration step from here on is
   observed (:mod:`agentbench.adapters.mujoco.recorder`).
3. **Run the driver unchanged**, as ``__main__``, with ``sys.argv[1]`` set to
   the model path and the working directory set to the model's own directory
   -- so both of the two obvious ways an author refers to their model (a
   relative ``from_xml_path("scene.xml")`` and ``sys.argv[1]``) resolve.
4. **Always write the artifact set**, in a ``finally``. A driver that raises,
   a driver that never steps and a driver the clock cut off all leave the same
   files; what differs is what those files say.

Exit codes: ``0`` the driver completed or the grader's clock stopped it;
``1`` the driver raised; ``2`` the runner could not even get that far (no
mujoco, unreadable driver). The exit code is evidence, never the verdict --
the artifacts are the verdict.
"""

from __future__ import annotations

import argparse
import json
import os
import runpy
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--driver", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--wall-limit", type=float, default=None)
    ap.add_argument("--contact-stride", type=int, default=1)
    ap.add_argument("--body-cap", type=int, default=None)
    ap.add_argument("--pair-cap", type=int, default=None)
    ap.add_argument("--driver-arg", action="append", default=[])
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    model_path = Path(args.model).resolve()

    try:
        import mujoco                                    # noqa: F401
        from agentbench.adapters.mujoco import recorder as rec_mod
    except Exception as exc:                             # noqa: BLE001
        _write(out / "model_load.json",
               {"compiled": None, "error": "mujoco is not importable in this "
                                           "interpreter: %r" % (exc,),
                "python": sys.executable})
        return 2

    # -- 1. does the agent's MJCF compile at all? -------------------------
    load = {"model": str(model_path), "compiled": False, "error": None,
            "python": sys.executable,
            "mujoco_version": mujoco.mj_versionString()}
    try:
        probe = mujoco.MjModel.from_xml_path(str(model_path))
        load["compiled"] = True
        load["stats"] = {"nbody": int(probe.nbody), "ngeom": int(probe.ngeom),
                         "njnt": int(probe.njnt), "nu": int(probe.nu)}
        del probe
    except Exception as exc:                             # noqa: BLE001
        # A compile failure is a MEASUREMENT (the agent's file is broken), so
        # it is recorded and the driver is still run: a driver that builds its
        # model some other way is entitled to work.
        load["error"] = "%s: %s" % (type(exc).__name__, exc)
    _write(out / "model_load.json", load)

    # -- 2. observe ------------------------------------------------------
    kw = {}
    if args.body_cap:
        kw["body_cap"] = args.body_cap
    if args.pair_cap:
        kw["pair_cap"] = args.pair_cap
    rec = rec_mod.Recorder(out, duration_s=args.duration,
                           wall_limit_s=args.wall_limit,
                           contact_stride=args.contact_stride,
                           driver_label=(Path(args.driver).name
                                         if args.driver else ""),
                           model_path=str(model_path), **kw)
    rec.install()

    # -- 3. the agent's driver, unchanged --------------------------------
    complete, driver_error, rc = None, None, 0
    t0 = time.monotonic()
    if not args.driver:
        complete = False
        driver_error = ("no driver program was supplied, so nothing stepped "
                        "the model; a MuJoCo scene is inert without one")
        rc = 1
    else:
        driver = Path(args.driver).resolve()
        cwd0 = os.getcwd()
        argv0 = list(sys.argv)
        try:
            os.chdir(str(model_path.parent))
            sys.argv = [str(driver), str(model_path)] + list(args.driver_arg)
            runpy.run_path(str(driver), run_name="__main__")
            complete = None            # the recorder decides: it stepped or not
        except (rec_mod.DurationReached, rec_mod.WallClockExceeded) as stop:
            # The grader's own clock. This is a clean stop, not a failure --
            # it is what simulationQuit() is on the arms that have one.
            complete = True
            sys.stderr.write("[agentbench] %s\n" % stop)
        except SystemExit as exc:
            code = exc.code
            if code in (None, 0):
                complete = None
            else:
                complete, rc = False, 1
                driver_error = "the driver called sys.exit(%r)" % (code,)
        except BaseException as exc:                     # noqa: BLE001
            complete, rc = False, 1
            driver_error = "%s: %s" % (type(exc).__name__, exc)
            traceback.print_exc()
        finally:
            os.chdir(cwd0)
            sys.argv = argv0

    # -- 4. always write the artifacts -----------------------------------
    try:
        rec.finish(complete=complete, driver_error=driver_error,
                   wall_s=round(time.monotonic() - t0, 4))
    except Exception:                                    # noqa: BLE001
        traceback.print_exc()
        return 2
    return rc


def _write(path, doc):
    try:
        Path(path).write_text(json.dumps(doc, indent=1, sort_keys=True),
                              encoding="utf-8")
    except (OSError, TypeError, ValueError):
        pass


if __name__ == "__main__":
    raise SystemExit(main())
