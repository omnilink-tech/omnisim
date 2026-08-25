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

"""Bring-up probe: measure upstream's LiDAR conventions instead of assuming.

Facts the R1 oracle's map depends on, none of them safe to guess -- getting
any of them wrong mirrors or truncates the occupancy grid, which does not fail
loudly, it just drives the robot into the thing it thinks it avoided:

1. **which end of ``getRangeImage()`` is the LEFT of the fan**, and at what
   angle each bin sits -- the scene places ONE box front-LEFT of the robot;
2. **what frame ``getPointCloud()`` reports in**;
3. **how high the mount may sit** before the fan sails over a 0.5 m obstacle
   at the far end of a 10 m arena (a resting robot is not necessarily level);
4. **whether a Lidar works at all under ``--no-rendering`` in xvfb**, which is
   how this arm launches every run (``launcher.webots_invocation``).

Prints and quits. Not a benchmark controller.
"""

import math

from controller import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

LIDARS = [robot.getDevice(n) for n in ("lidar_low", "lidar_mid", "lidar_high")]
for _l in LIDARS:
    _l.enable(dt)
    _l.enablePointCloud()
gps = robot.getDevice("gps")
gps.enable(dt)
imu = robot.getDevice("inertial unit")
imu.enable(dt)

for _ in range(40):
    robot.step(dt)

print("[probe] rpy=%s gps=%s"
      % (["%.5f" % v for v in imu.getRollPitchYaw()],
         ["%.3f" % v for v in gps.getValues()]), flush=True)

for lidar in LIDARS:
    n = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    print("[probe] --- %s: resolution=%d fov=%.5f rad (%.2f deg) "
          "maxRange=%.1f layers=%d"
          % (lidar.getName(), n, fov, math.degrees(fov), lidar.getMaxRange(),
             lidar.getNumberOfLayers()), flush=True)
    ranges = list(lidar.getRangeImage() or [])
    finite = [(i, r) for i, r in enumerate(ranges)
              if r == r and abs(r) != float("inf")]
    print("[probe] %d values, %d finite" % (len(ranges), len(finite)),
          flush=True)
    if finite:
        i_min, r_min = min(finite, key=lambda t: t[1])
        print("[probe] MIN %.4f m at index %d; first=%.4f mid=%s last=%.4f"
              % (r_min, i_min, ranges[0],
                 ("inf" if ranges[len(ranges) // 2] == float("inf")
                  else "%.4f" % ranges[len(ranges) // 2]), ranges[-1]),
              flush=True)
        short = [i for i, r in finite if r < 4.0]
        if short:
            print("[probe] indices with range < 4 m: %d..%d"
                  % (min(short), max(short)), flush=True)
    print("[probe] profile: " + " ".join(
        "%d:%s" % (i, ("inf" if ranges[i] == float("inf")
                       else "%.2f" % ranges[i]))
        for i in range(0, len(ranges), 5)), flush=True)
    cloud = lidar.getPointCloud()
    if cloud:
        pts = [(p.x, p.y, p.z) for p in cloud]
        ok = [(i, q) for i, q in enumerate(pts)
              if all(v == v and abs(v) < 1e6 for v in q)]
        if ok:
            i_near, p_near = min(ok, key=lambda t: math.hypot(t[1][0],
                                                              t[1][1]))
            print("[probe] nearest cloud point idx=%d xyz=(%.4f, %.4f, %.4f)"
                  % (i_near, p_near[0], p_near[1], p_near[2]), flush=True)
            print("[probe] cloud[0]=(%.3f, %.3f, %.3f) "
                  "cloud[-1]=(%.3f, %.3f, %.3f)"
                  % (pts[0][0], pts[0][1], pts[0][2],
                     pts[-1][0], pts[-1][1], pts[-1][2]), flush=True)
    else:
        print("[probe] point cloud EMPTY", flush=True)

while robot.step(dt) != -1:
    pass
