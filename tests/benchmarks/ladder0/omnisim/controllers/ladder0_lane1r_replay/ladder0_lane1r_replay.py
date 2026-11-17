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

"""A shim, and deliberately nothing more: it EXECUTES lane1r's own replay
controller from its own source file.

Rung 18's honest scene is lane1r's world, run by lane1r's controller, verbatim.
Its two scene-level faults cannot be -- a fault is a deliberate corruption of
the scene, so the faulted world has to live in this arm's tree, and an engine
resolves ``controller "<name>"`` against the world's OWN project directory.
Copying lane1r's controller here would create a second copy of another lane's
code that can drift from it silently, and the drift would present as "OmniSim
disagrees with reality" rather than as "the two controllers diverged".

So this file loads that file.  There is no logic here to get wrong, and
``__name__`` is forced to ``"__main__"`` because the controller's own entry
point is guarded by it.
"""

import os
import runpy
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
# .../ladder0/omnisim/controllers/ladder0_lane1r_replay -> the repo root
REPO = os.path.abspath(os.path.join(HERE, *([os.pardir] * 6)))
TARGET = os.path.join(REPO, "tests", "benchmarks", "omnibench", "lane1r",
                      "controllers", "lane1r_replay", "lane1r_replay.py")


def _fail(msg):
    """A controller that dies mutely is this arm's worst failure mode: on
    Windows ``omnisim-bin.exe`` is a GUI-subsystem binary and controller stderr
    goes nowhere, so leave the evidence on disk beside the output."""
    out = os.environ.get("LANE1R_OUT")
    if out:
        try:
            with open(out + ".traceback.txt", "w", encoding="utf-8") as f:
                f.write(msg)
        except OSError:
            pass
    sys.stderr.write(msg)
    raise SystemExit(msg)


if not os.path.isfile(TARGET):
    _fail("[ladder0] lane1r replay controller not found at %s -- rung 18's "
          "fault scenes run lane1r's controller by path and may not carry a "
          "copy of it\n" % TARGET)

runpy.run_path(TARGET, run_name="__main__")
