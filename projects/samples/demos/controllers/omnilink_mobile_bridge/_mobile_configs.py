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

"""Per-robot configs for omnilink_mobile_bridge.

Adding a new wheeled base: drop a new entry into MOBILE_CONFIGS. The
bridge's generic differential / skid-steer driver picks it up.

Config fields:

    layout                    "4wheel_full" -- Husky / Jackal naming
                              "4wheel_fl"   -- Rosbot family
                              "2wheel"      -- TurtleBot3 family
                              "kinematic"   -- no wheel motors; the bridge
                                               integrates (v, w) and writes
                                               the pose via supervisor
    wheel_radius_m            metres
    half_track_m              wheel separation / 2, metres
    max_wheel_speed_radps     ceiling on rad/s for each wheel
    cruise_frac               fraction of max used for default forward speed
    spin_speed                rad/s for the "spin in place" intent preset

Motor names per layout (the URDF importer derives them):

    4wheel_full -> front_left_wheel_motor / front_right_wheel_motor / ...
    4wheel_fl   -> fl_wheel_joint_motor / fr_wheel_joint_motor / ...
    2wheel      -> wheel_left_joint_motor / wheel_right_joint_motor
    kinematic   -> none (supervisor-driven body)
"""

HUSKY = {
    "model": "Clearpath Husky",
    "layout": "4wheel_full",
    "wheel_radius_m": 0.1651,
    "half_track_m": 0.2854,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.55,
    "spin_speed": 0.6,
}

JACKAL = {
    "model": "Clearpath Jackal",
    "layout": "4wheel_full",
    "wheel_radius_m": 0.098,
    "half_track_m": 0.187,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.55,
    "spin_speed": 0.8,
}

ROSBOT = {
    "model": "Husarion Rosbot",
    "layout": "4wheel_fl",
    "wheel_radius_m": 0.0425,
    "half_track_m": 0.105,
    "max_wheel_speed_radps": 12.0,
    "cruise_frac": 0.45,
    "spin_speed": 0.9,
}

ROSBOT_XL = {
    "model": "Husarion Rosbot XL",
    "layout": "4wheel_fl",
    "wheel_radius_m": 0.05,
    "half_track_m": 0.124,
    "max_wheel_speed_radps": 12.0,
    "cruise_frac": 0.5,
    "spin_speed": 0.9,
}

TB3_BURGER = {
    "model": "TurtleBot3 Burger",
    "layout": "2wheel",
    "wheel_radius_m": 0.033,
    "half_track_m": 0.080,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.45,
    "spin_speed": 1.0,
}

TB3_WAFFLE = {
    "model": "TurtleBot3 Waffle",
    "layout": "2wheel",
    "wheel_radius_m": 0.033,
    "half_track_m": 0.144,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.45,
    "spin_speed": 0.9,
}

TB3_WAFFLE_PI = dict(TB3_WAFFLE, model="TurtleBot3 Waffle Pi")


# ── OmniTug 500 intralogistics AGV ──────────────────────────────────────
# The OmniTug 500 is a low four-wheeled ground tug. Its URDF is a single
# visual link with NO wheel joints and no collision geometry -- its own
# shipped controllers (projects/robots/omnisim/omnitug500/controllers/*) drive
# the body kinematically via supervisor setSFVec3f/setSFRotation at floor
# z=0, forward along local +Y. layout "kinematic" tells the bridge to do
# the same: integrate (v, w) each tick and write the pose (requires
# supervisor TRUE on the Robot node). The diff-drive fields below only
# parameterise the velocity envelope for the shared command surface.
OMNITUG500 = {
    "model": "OmniTug 500",
    "layout": "kinematic",
    "wheel_radius_m": 0.10,
    "half_track_m": 0.30,
    "max_wheel_speed_radps": 10.0,     # -> 1.0 m/s linear ceiling
    "cruise_frac": 0.6,                # 0.60 m/s cruise (AMR walking pace)
    "spin_speed": 0.9,
    "kinematic": True,
    # local +Y is forward: to face world heading h, node yaw = h - pi/2
    "forward_yaw_offset": -1.5707963,
    "floor_z": 0.0,
    # body is 1.259 m long -> rear coupling (trolley hitch dock) offset
    "rear_offset_m": 0.63,
}


MOBILE_CONFIGS = {
    "husky":         HUSKY,
    "jackal":        JACKAL,
    "rosbot":        ROSBOT,
    "rosbot_xl":     ROSBOT_XL,
    "tb3_burger":    TB3_BURGER,
    "tb3_waffle":    TB3_WAFFLE,
    "tb3_waffle_pi": TB3_WAFFLE_PI,
    "omnitug500":    OMNITUG500,
}

# The arm is a URDFRobot, so its pedestal is not in the scene tree as a Box
# the obstacle walk can find. It is a fixed, staticBase machine standing in
# the pick cell, and the tugs must treat it as one.
# DEF -> (half_x, half_y, top_z).
STATIC_BASE_OBSTACLES = {
    "OMNIARM6": (0.30, 0.30, 1.2),
}

WHEEL_MOTORS = {
    "4wheel_full": {
        "left":  ["front_left_wheel_motor",  "rear_left_wheel_motor"],
        "right": ["front_right_wheel_motor", "rear_right_wheel_motor"],
    },
    "4wheel_fl": {
        "left":  ["fl_wheel_joint_motor", "rl_wheel_joint_motor"],
        "right": ["fr_wheel_joint_motor", "rr_wheel_joint_motor"],
    },
    "2wheel": {
        "left":  ["wheel_left_joint_motor"],
        "right": ["wheel_right_joint_motor"],
    },
    # Supervisor-driven bodies with no wheel motors (kinematic layout).
    "kinematic": {
        "left":  [],
        "right": [],
    },
}


def get_config(robot_id: str) -> dict:
    if robot_id not in MOBILE_CONFIGS:
        raise ValueError(
            f"Unknown mobile robot '{robot_id}'. Known: {sorted(MOBILE_CONFIGS.keys())}"
        )
    return MOBILE_CONFIGS[robot_id]
