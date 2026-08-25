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

"""ladder0_rover -- rung 7 only.  One of the five, driving its own command.

It exists because a Webots-family supervisor may NOT read another robot's
devices, and that restriction is honest rather than a gap: a supervisor cannot
truthfully claim to have read a sibling's sensor.  So each rover owns its own
motors and the probe measures the fleet from the scene graph instead, reading
each wheel's world ORIENTATION -- which observes the wheel rather than the
servo's own bookkeeping of it.

It RECORDS NOTHING and JUDGES NOTHING.  Its whole job is to hold four wheels at
the rate its ``--omega=`` argument names, which the world file wrote there from
``rungs.RUNG7_OMEGA``.  There is no default: a rover launched without the
argument would drive at some rate the contract never chose, and the row would
be judged against an expectation nothing in the scene produced.
"""

import os
import sys
import time

T_START = time.time()
HERE = os.path.abspath(os.path.dirname(__file__))

from controller import Robot  # noqa: E402

WHEELS = ("fl", "fr", "rl", "rr")


def arg(name):
    pref = "--%s=" % name
    for a in sys.argv[1:]:
        if a.startswith(pref):
            return a[len(pref):]
    return None


def _note(text):
    """Leave evidence on disk.  ``omnisim-bin.exe`` is a GUI-subsystem binary
    on Windows, so a controller's stdout goes nowhere and a dead rover is
    indistinguishable from one that was never launched."""
    out = os.environ.get("LADDER0_OUT")
    if not out:
        return
    try:
        d = os.path.dirname(out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(out + ".rover", "a", encoding="utf-8") as f:
            f.write("%.3f %s\n" % (time.time() - T_START, text))
    except OSError:
        pass


def main():
    raw = arg("omega")
    robot = Robot()
    dt_ms = int(robot.getBasicTimeStep())
    if raw is None:
        _note("%s: NO --omega= argument; refusing to drive"
              % robot.getName())
        while robot.step(dt_ms) != -1:
            pass
        return
    omega = float(raw)
    motors = [robot.getDevice("wheel_%s" % t) for t in WHEELS]
    missing = [t for t, m in zip(WHEELS, motors) if m is None]
    _note("%s omega=%g missing=%s" % (robot.getName(), omega, missing))
    for m in motors:
        if m is not None:
            m.setPosition(float("inf"))          # velocity control
            m.setVelocity(omega)
    while robot.step(dt_ms) != -1:
        pass


try:
    main()
except BaseException as _exc:                    # noqa: BLE001
    import traceback
    _note("CRASH %s" % "".join(traceback.format_exception(
        type(_exc), _exc, _exc.__traceback__)))
    raise
