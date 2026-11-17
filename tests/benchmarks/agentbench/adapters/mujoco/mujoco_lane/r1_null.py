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

"""R1 NULL driver -- the do-nothing half of the oracle/null gate.

It runs against **the same** ``r1_oracle.xml``. That is the experiment: the
scene, the robot, the obstacles, the recorder, the window and the grader are
held fixed and only the *program* changes, so a difference in the verdict can
only be the program. A null with its own world would be testing two things at
once.

What it does is the most generous reading of "nothing": it steps the model, so
the world is real, time passes, the robot exists, is drivable, and every
obstacle is where it should be. It simply never commands a wheel and never
casts a beam. ``ctrl`` is written once, to zero, so that "the actuators were
left alone" is a statement in the file rather than an omission from it.

SPEC 7.1's rule is that no task may be passable by doing nothing. The
assertions this must fail are the ones that require the robot to have *done*
the task -- R1.4 (arrive), R1.5 (which credits collision-free only to a robot
that travelled at least 0.5 m, so a parked robot earns nothing) and R1.6
(drive at least 11.5 m). R1.1 to R1.3 SHOULD pass: the run is clean, there is
one drivable robot, and the obstacles are intact. Those three are true of this
run, they are worth measuring, and a gate that demanded they fail would be
asking the null to be a broken world instead of an idle agent -- which is a
different negative control (this arm already has it: a model that does not
compile, a driver that raises, a scene with no driver at all, all pinned in
``test_mujoco_end_to_end.py``).

The measured split is recorded in ``BRINGUP.md`` sec. 5 and asserted in
``test_r1_discriminates_mujoco.py``.
"""

from __future__ import annotations

import sys

import mujoco


def main():
    model_path = sys.argv[1] if len(sys.argv) > 1 else "r1_oracle.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    data.ctrl[:] = 0.0          # explicitly: the wheels are commanded nothing

    # No end condition. The grader's recording window owns termination on this
    # arm, exactly as it does for the blind probe -- so this driver cannot end
    # the run early and make "it did not go anywhere" look like "it finished".
    while True:
        mujoco.mj_step(model, data)


if __name__ == "__main__":
    main()
