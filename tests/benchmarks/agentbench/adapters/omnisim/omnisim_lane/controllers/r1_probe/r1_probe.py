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

"""R1 bring-up probe -- MEASURE the sensing conventions, do not assume them.

Paired with ``r1_probe.wbt``: the rover sits at the origin yawed +0.5 rad and
one post stands at world (0, 3), i.e. bearing +1.0708 rad in the robot frame
at a radial range of 3.0 m. That is a single unambiguous return whose angle
and distance are both known in closed form, so the range image can be read
back against arithmetic instead of against a hope.

It reports, and asserts nothing: the oracle then hard-codes what this
measured, with this file as the citation.

⚠ **It writes to a FILE, not to stdout, and that is not a style choice.** On
Windows ``omnisim-bin.exe`` is a GUI-subsystem binary, so a controller's
stdout goes nowhere a caller can read -- measured here on the first probe run,
whose every ``print`` vanished while the engine log happily recorded
``'r1_probe' controller exited successfully``. Every controller in this lane
therefore reports through a JSON sidecar, which is also what makes the
perception claim checkable from a test.

Run it the way the gate runs everything else -- ``omnisim-bin r1_probe.wbt
--batch --mode=fast --no-rendering --minimize``.
"""

import json
import math
import os

POST_XY = (0.0, 3.0)
STEPS = 12

OUT = os.environ.get(
    "AGENTBENCH_R1_PROBE_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe.json"))


def _emit(doc):
    try:
        with open(OUT, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=2)
    except OSError:
        pass


def main():
    from omnisim import Robot

    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    lidar = robot.getDevice("lidar")
    lidar.enable(dt)
    gps = robot.getDevice("gps")
    gps.enable(dt)
    imu = robot.getDevice("imu")
    imu.enable(dt)
    for _ in range(STEPS):
        if robot.step(dt) == -1:
            return

    res = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    rng = list(lidar.getRangeImage() or [])
    max_range = lidar.getMaxRange()
    x, y, _z = gps.getValues()
    roll, pitch, yaw = imu.getRollPitchYaw()

    doc = {"resolution": res, "fov_rad": fov, "fov_deg": math.degrees(fov),
           "max_range_m": max_range, "gps_xy": [x, y],
           "imu_rpy": [roll, pitch, yaw], "n_beams_returned": len(rng)}

    # The post's true bearing in the robot frame, from the world geometry.
    bearing = math.atan2(POST_XY[1] - y, POST_XY[0] - x) - yaw
    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
    true_range = math.hypot(POST_XY[0] - x, POST_XY[1] - y)
    doc["post_true_bearing_rad"] = bearing
    doc["post_true_range_m"] = true_range

    finite = [(i, r) for i, r in enumerate(rng)
              if r is not None and r == r and r < max_range - 1e-6]
    doc["n_short_beams"] = len(finite)
    if not finite:
        doc["verdict"] = "NO SHORT RETURN -- the lidar saw nothing"
        _emit(doc)
        return
    hit_i, hit_r = min(finite, key=lambda p: p[1])
    doc["nearest_index"] = hit_i
    doc["nearest_range_m"] = hit_r
    doc["range_error_m"] = hit_r - true_range

    # The two candidate conventions, each stated as the angle of beam i.
    dtheta = fov / (res - 1)
    cand = {
        "index_0_is_plus_half_fov": lambda i: fov / 2.0 - i * dtheta,
        "index_0_is_minus_half_fov": lambda i: -fov / 2.0 + i * dtheta,
    }
    doc["conventions"] = {}
    for label, f in cand.items():
        want = min(range(res), key=lambda i: abs(f(i) - bearing))
        doc["conventions"][label] = {
            "predicted_index": want, "measured_index": hit_i,
            "angle_of_measured_index_rad": f(hit_i),
            "match": abs(want - hit_i) <= 2}
    doc["verdict"] = next(
        (k for k, v in doc["conventions"].items() if v["match"]), "NEITHER")
    _emit(doc)


if __name__ == "__main__":
    main()
