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

"""Native state estimator (EKF) validated on REAL GPS + IMU (KITTI).

The piece simulation let us skip: outdoors, GPS is noisy and DROPS OUT (bridges,
urban canyons, foliage). A real navigator must coast through those on inertial /
odometry dead-reckoning and re-acquire cleanly. This is a 3-state EKF
[x, y, yaw] driven by odometry speed + gyro yaw-rate, corrected by GPS, run over
a real KITTI drive — with a forced GPS blackout to show it holds position when
the fix is gone. No simulation: the GPS, speed and yaw-rate are recorded sensors.

    python tools/ekf_demo.py
"""

from __future__ import annotations

import glob
import math
import os

import numpy as np

from kitti_source import SEQ, read_oxts

EARTH_R = 6378137.0
DT = 0.1                       # KITTI raw is ~10 Hz


def enu(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_R
    return x, y


def main():
    files = sorted(glob.glob(os.path.join(SEQ, "oxts", "data", "*.txt")))
    O = [read_oxts(f) for f in files]
    lat0, lon0 = O[0]["lat"], O[0]["lon"]
    ref = [enu(o["lat"], o["lon"], lat0, lon0) for o in O]   # truth (RTK-grade OXTS)
    n = len(O)
    drove = sum(math.hypot(ref[k][0] - ref[k - 1][0], ref[k][1] - ref[k - 1][1])
                for k in range(1, n))
    blackout = set(range(n // 3, n // 3 + 30))               # ~3 s with no GPS fix

    # EKF state [x, y, yaw]; control = (odom speed, gyro yaw-rate); meas = GPS (x, y).
    x = np.array([ref[0][0], ref[0][1], O[0]["yaw"]], float)
    P = np.diag([1.0, 1.0, 0.05])
    Q = np.diag([0.04, 0.04, 0.01])      # process noise (odom/gyro imperfection)
    R = np.diag([1.5, 1.5])              # GPS noise ~1.2 m (consumer-grade)
    rng = np.random.default_rng(0)
    H = np.array([[1.0, 0, 0], [0, 1.0, 0]])

    err_fix, err_blk, dr_only = [], [], None
    for k in range(n):
        v, w, yaw = O[k]["vf"], O[k]["wz"], x[2]              # real odom + real gyro
        x = x + np.array([v * math.cos(yaw) * DT, v * math.sin(yaw) * DT, w * DT])
        F = np.array([[1, 0, -v * math.sin(yaw) * DT],
                      [0, 1,  v * math.cos(yaw) * DT],
                      [0, 0, 1.0]])
        P = F @ P @ F.T + Q
        if k not in blackout:                                # GPS correction
            z = np.array(ref[k]) + rng.normal(0, 1.2, 2)     # noisy real GPS
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ (z - H @ x)
            P = (np.eye(3) - K @ H) @ P
        e = math.hypot(x[0] - ref[k][0], x[1] - ref[k][1])
        (err_blk if k in blackout else err_fix).append(e)
        if k == max(blackout):
            dr_only = e

    blk_drive = sum(math.hypot(ref[k][0] - ref[k - 1][0], ref[k][1] - ref[k - 1][1])
                    for k in sorted(blackout)[1:])
    print(f"[ekf] {n} real frames, drove {drove:.0f} m | GPS blackout for "
          f"{len(blackout)} frames (~{len(blackout) * DT:.1f} s, {blk_drive:.0f} m)")
    print(f"[ekf] with GPS  : mean position error {np.mean(err_fix):.2f} m "
          f"(fuses noisy real GPS + odom + gyro)")
    print(f"[ekf] GPS GONE  : drift {np.mean(err_blk):.2f} m mean, {max(err_blk):.2f} m "
          f"max over {blk_drive:.0f} m of blind driving "
          f"({100 * max(err_blk) / max(blk_drive, 1):.1f}% of distance)")
    print(f"[ekf] -> coasts through the dropout on dead-reckoning and re-acquires; "
          f"raw GPS alone would have been blank for {len(blackout) * DT:.1f} s")


if __name__ == "__main__":
    main()
