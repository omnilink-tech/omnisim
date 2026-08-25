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

"""Top-down recognition camera over the bin-picking box.

Enables the Camera + its Recognition node and, each step, publishes the cubes
it currently SEES to this robot's customData as a comma-separated list of
"<nodeId>:<red|blue>". Occlusion is on, so the camera only reports the cubes
on top of the pile -- exactly what the arm should pick next. The OMNIARM6
supervisor reads this customData (via the BIN_CAM node's customData field),
maps each nodeId back to the cube, and picks the topmost one the camera sees.
"""

from omnisim import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
cam = robot.getDevice("bin_camera")
cam.enable(dt)
cam.recognitionEnable(dt)


def colour_of(obj):
    """red/blue from the recognition object's first recognitionColor."""
    try:
        c = obj.getColors()              # flat [r,g,b, ...]
        return "red" if float(c[0]) >= float(c[2]) else "blue"
    except Exception:
        return "red"


while robot.step(dt) != -1:
    seen = []
    try:
        for obj in cam.getRecognitionObjects():
            seen.append("%d:%s" % (obj.getId(), colour_of(obj)))
    except Exception:
        pass
    robot.setCustomData(",".join(seen))
