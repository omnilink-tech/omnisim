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

"""Record a Monte-Carlo-localization dataset from the OmniTug 500 lidar arena.

Drives the rover on a fixed elliptical loop, logging on every recorded tick:
  ground-truth pose, noisy wheel-odometry, and one 512-beam lidar scan.
A kidnapped-robot event teleports the rover at a known time WITHOUT touching
odometry, which is the standard recovery test.

Outputs (next to this file, in ./out):
  mcl_poses.csv   t_s, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, kidnapped
  mcl_scans.csv   t_s, then 512 range columns (metres; maxRange means no return)
  mcl_meta.json   lidar intrinsics, arena map, loop and noise parameters, seed

Everything is seeded, so two runs of the same build produce the same dataset.
"""

import json
import math
import os
import random

from omnisim import Supervisor

SEED = 20260903
DURATION_S = 60.0
RECORD_HZ = 5.0
KIDNAP_T = 30.0
KIDNAP_DXY = (1.60, -1.10)
KIDNAP_DYAW = 1.05

LOOP_A, LOOP_B = 2.6, 1.8
LOOP_PERIOD_S = 40.0
SCANNER = "scanner_front_right"

ODOM_TRANS_NOISE = 0.010
ODOM_YAW_NOISE = 0.004
ODOM_YAW_BIAS = 0.0025

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def pose_at(t):
    ph = 2.0 * math.pi * (t / LOOP_PERIOD_S)
    x, y = LOOP_A * math.cos(ph), LOOP_B * math.sin(ph)
    dx, dy = -LOOP_A * math.sin(ph), LOOP_B * math.cos(ph)
    return x, y, math.atan2(dy, dx)


def main():
    rng = random.Random(SEED)
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    lidar = robot.getDevice(SCANNER)
    if lidar is None:
        raise SystemExit("lidar device %r not found" % SCANNER)
    lidar.enable(ts)

    me = robot.getSelf()
    my_tf, my_rf = me.getField("translation"), me.getField("rotation")
    tug = robot.getFromDef("OMNITUG500")
    tug_tf = tug.getField("translation") if tug else None
    tug_rf = tug.getField("rotation") if tug else None

    os.makedirs(OUT, exist_ok=True)
    fp = open(os.path.join(OUT, "mcl_poses.csv"), "w", encoding="utf-8")
    fs = open(os.path.join(OUT, "mcl_scans.csv"), "w", encoding="utf-8")
    fp.write("t_s,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,kidnapped\n")

    ox, oy, oyaw = pose_at(0.0)
    px, py, pyaw = ox, oy, oyaw
    kidnap_done = False
    kx = ky = kyaw = 0.0
    every = max(1, int(round((1000.0 / RECORD_HZ) / ts)))
    tick = 0
    header_written = False
    n_rows = 0

    while robot.step(ts) != -1:
        t = tick * ts / 1000.0
        if t > DURATION_S:
            break

        gx, gy, gyaw = pose_at(t)
        if t >= KIDNAP_T and not kidnap_done:
            kx, ky, kyaw = KIDNAP_DXY[0], KIDNAP_DXY[1], KIDNAP_DYAW
            kidnap_done = True
        gx, gy, gyaw = gx + kx, gy + ky, gyaw + kyaw

        my_tf.setSFVec3f([gx, gy, 0.0])
        my_rf.setSFRotation([0.0, 0.0, 1.0, gyaw])
        if tug_tf:
            tug_tf.setSFVec3f([gx, gy, 0.0])
            tug_rf.setSFRotation([0.0, 0.0, 1.0, gyaw])

        if tick % every == 0:
            tx, ty, tyaw = pose_at(t)
            step = math.hypot(tx - px, ty - py)
            dyaw = math.atan2(math.sin(tyaw - pyaw), math.cos(tyaw - pyaw))
            px, py, pyaw = tx, ty, tyaw
            oyaw += dyaw + ODOM_YAW_BIAS * step + rng.gauss(0.0, ODOM_YAW_NOISE)
            noisy = step * (1.0 + rng.gauss(0.0, ODOM_TRANS_NOISE))
            ox += noisy * math.cos(oyaw)
            oy += noisy * math.sin(oyaw)
            fp.write("%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n"
                     % (t, gx, gy, gyaw, ox, oy, oyaw, 1 if kidnap_done else 0))

            rngs = lidar.getRangeImage()
            if rngs:
                if not header_written:
                    fs.write("t_s," + ",".join("r%d" % i for i in range(len(rngs))) + "\n")
                    header_written = True
                fs.write("%.3f," % t + ",".join("%.4f" % v for v in rngs) + "\n")
                n_rows += 1
        tick += 1

    fp.close()
    fs.close()
    meta = {
        "seed": SEED, "duration_s": DURATION_S, "record_hz": RECORD_HZ,
        "basic_time_step_ms": ts, "rows": n_rows,
        "lidar": {"device": SCANNER, "beams": 512, "fov_rad": 3.0,
                  "min_range_m": 0.06, "max_range_m": 12.0, "layers": 1,
                  "mount_xyz": [0.290, 0.537, 0.131], "mount_yaw_rad": 0.7854,
                  "frame": "sensor frame, +X forward, point i at azimuth "
                           "-fov/2 + i*fov/(beams-1)"},
        "kidnap": {"t_s": KIDNAP_T, "d_xy_m": list(KIDNAP_DXY),
                   "d_yaw_rad": KIDNAP_DYAW},
        "odometry_noise": {"translation_frac_sigma": ODOM_TRANS_NOISE,
                           "yaw_sigma_rad": ODOM_YAW_NOISE,
                           "yaw_bias_rad_per_m": ODOM_YAW_BIAS},
        "loop": {"semi_axis_x_m": LOOP_A, "semi_axis_y_m": LOOP_B,
                 "period_s": LOOP_PERIOD_S,
                 "note": "kinematic replay: the pose is scripted, not produced "
                         "by wheel dynamics"},
        "world": "projects/robots/omnisim/omnitug500/worlds/omnitug500_mcl_record.omniworld",
    }
    with open(os.path.join(OUT, "mcl_meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2)
    print("[mcl_record] wrote %d scans to %s" % (n_rows, OUT))


main()
