#!/usr/bin/env python3
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

"""Convert a whole-body cyclic walk ghost to the box-demo suction carry pose.

The leg, root and contact plans are deliberately left untouched.  Only the ten arm
channels are replaced, so an empty/carry pair has identical support timing and can be
handed over by BATON at double support.  The defaults are the measured physical hold
used by g1_box_grasp (cups level, elbows bent, box close to the torso).
"""
import argparse
import json


POSE = {
    "left_shoulder_pitch_joint": -0.90,
    "left_shoulder_roll_joint": 0.15,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.86,
    "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": -0.90,
    "right_shoulder_roll_joint": -0.15,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.86,
    "right_wrist_roll_joint": 0.0,
}
G1_LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    args = ap.parse_args()

    d = json.load(open(args.src))
    d.setdefault("robot", "g1")
    d.setdefault("joints", G1_LEG_JOINTS)
    names = d.get("wb_joints", [])
    if not names or "wb_lut" not in d:
        raise SystemExit("source must contain wb_joints + wb_lut")
    slots = {name: names.index(name) for name in POSE}
    for row in d["wb_lut"]:
        for name, value in POSE.items():
            row[slots[name]] = value
    d["arm_lut"] = [[POSE["left_shoulder_pitch_joint"],
                     POSE["right_shoulder_pitch_joint"]] for _ in range(d["nb"])]
    d["elbow_lut"] = [[POSE["left_elbow_joint"], POSE["right_elbow_joint"]]
                      for _ in range(d["nb"])]
    d["shroll"] = POSE["left_shoulder_roll_joint"]
    d["shyaw"] = POSE["left_shoulder_yaw_joint"]
    d["carry"] = True
    d["carry_pose"] = POSE
    d["source"] = d.get("source", "") + " | exact physical suction-carry upper-body pose"
    d.pop("validator", None)
    json.dump(d, open(args.dst, "w"))
    print("CARRY WALK GHOST: %s -> %s (%d frames)" % (args.src, args.dst, d["nb"]))


if __name__ == "__main__":
    main()
