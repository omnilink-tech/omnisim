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

"""Per-robot configs for omnilink_quadruped_bridge.

One dict per quadruped the bridge can drive. `--robot <key>` in the world's
controllerArgs selects it. Every angle is in radians, in the robot's own joint
convention (the sign conventions differ between families -- that is what the
``*_dir`` entries encode, so the bridge's gestures read "flex the knee a bit"
instead of "add -0.3").

Keys
----
model             human-readable name reported on /list_robots and in the chat.
legs              leg id -> {"hip_x": motor, "hip_y": motor, "knee": motor}.
                  Motor names are what the URDF importer produced (joint name +
                  "_motor"); the bridge falls back to the bare joint name.
hip_x             leg id -> hip-roll angle held in every pose.
stand / sit       leg id -> (hip_y, knee): the standing crouch and the sit.
hip_sweep_dir     leg id -> +1/-1: the hip_y direction that moves the FOOT
                  BACKWARD relative to the body (stance phase of a walk).
knee_flex_dir     leg id -> +1/-1: the knee direction that FLEXES the leg.
settle_s          seconds to ramp from the spawn pose to `stand` after load.
body_lock         True = pin the floating base upright with the supervisor
                  every tick (OmniQuad: its stiff stance is not stable under
                  Newton). False = the robot stands on its own physics.
walk              "gait" = joint-space wave gait with supervisor-driven body
                  translation (OmniQuad's original); "wheels" = hold `stand`
                  and spin the wheel motors (the M20 family); "march" = the
                  wave gait in place, no supervisor translation.
walk_velocity_ms  forward speed for "gait" / "wheels".
wheels            leg id -> wheel motor name (only for walk == "wheels").
wheel_radius_m    wheel radius (only for walk == "wheels").

Adding a robot: copy the closest entry, fix the motor names and the two poses
(the URDF `<rest>` values are the stand), set the two ``*_dir`` maps from the
joint axes, and give the world `--robot <key>`.
"""

LEGS = ("front_left", "front_right", "rear_left", "rear_right")


def _per_leg(fl, fr, rl, rr):
    return {"front_left": fl, "front_right": fr, "rear_left": rl, "rear_right": rr}


def _same(v):
    return _per_leg(v, v, v, v)


# ---------------------------------------------------------------------------
# OmniQuad (OmniSim's own quadruped). Constants are the ones the bridge has
# always used (copied from omniquad_simple_pose); behaviour is unchanged.
# ---------------------------------------------------------------------------
OMNIQUAD = {
    "model": "OmniQuad",
    "legs": {leg: {"hip_x": f"{leg}_hip_x_motor", "hip_y": f"{leg}_hip_y_motor",
                   "knee": f"{leg}_knee_motor"} for leg in LEGS},
    "hip_x": _per_leg(0.30, -0.30, 0.30, -0.30),
    "stand": _same((0.30, -0.60)),
    "sit": _same((0.55, -1.20)),
    "extend": _same((0.15, -0.30)),      # the soft-drop pose held for 1.5 s at load
    "hip_sweep_dir": _same(+1),
    "knee_flex_dir": _same(-1),
    "settle_s": 3.0,
    "body_lock": True,
    "walk": "gait",
    "walk_velocity_ms": 0.30,
}

# ---------------------------------------------------------------------------
# Deep Robotics (projects/robots/deep_robotics/, BSD-3). Hip-pitch and knee
# axes are (0,-1,0): a NEGATIVE hip_y swings the foot forward, a POSITIVE
# knee flexes. The stand poses are the URDF <rest> values (PROVENANCE.md).
# These robots stand on their own physics under the world recipe
# newtonSubsteps 8 + newtonCompoundColliders TRUE + newtonGroundMu 2.
# ---------------------------------------------------------------------------
def _deep_legs(prefixes, names):
    hx, hy, kn = names
    return {leg: {"hip_x": f"{p}_{hx}_motor", "hip_y": f"{p}_{hy}_motor", "knee": f"{p}_{kn}_motor"}
            for leg, p in zip(LEGS, prefixes)}


LITE3 = {
    "model": "Deep Robotics Lite3",
    "legs": _deep_legs(("FL", "FR", "HL", "HR"), ("HipX_joint", "HipY_joint", "Knee_joint")),
    "hip_x": _same(0.0),
    "stand": _same((-1.0, 1.8)),
    "sit": _same((-1.6, 2.6)),           # knee range [0.524, 2.792]
    "hip_sweep_dir": _same(-1),
    "knee_flex_dir": _same(+1),
    "settle_s": 1.5,
    "body_lock": False,
    "walk": "march",
    "walk_velocity_ms": 0.0,
}

X30 = {
    "model": "Deep Robotics X30",
    "legs": _deep_legs(("FL", "FR", "HL", "HR"), ("HipX_joint", "HipY_joint", "Knee_joint")),
    "hip_x": _same(0.0),
    "stand": _same((-0.9, 1.8)),
    "sit": _same((-1.5, 2.4)),           # knee range [0.349, 2.531]
    "hip_sweep_dir": _same(-1),
    "knee_flex_dir": _same(+1),
    "settle_s": 1.5,
    "body_lock": False,
    "walk": "march",
    "walk_velocity_ms": 0.0,
}

# M20 family: wheeled-legged. The REAR legs fold the other way (their joint
# ranges are the mirror image of the front ones), so their directions flip.
_M20_LEGS = _deep_legs(("fl", "fr", "hl", "hr"), ("hipx_joint", "hipy_joint", "knee_joint"))
_M20_WHEELS = {leg: f"{p}_wheel_joint_motor" for leg, p in zip(LEGS, ("fl", "fr", "hl", "hr"))}


def _m20(model, wheel_radius=0.09):
    return {
        "model": model,
        "legs": _M20_LEGS,
        "hip_x": _same(0.0),
        "stand": _per_leg((-0.8, 1.6), (-0.8, 1.6), (0.8, -1.6), (0.8, -1.6)),
        "sit": _per_leg((-1.3, 2.4), (-1.3, 2.4), (1.3, -2.4), (1.3, -2.4)),
        "hip_sweep_dir": _per_leg(-1, -1, +1, +1),
        "knee_flex_dir": _per_leg(+1, +1, -1, -1),
        "settle_s": 1.5,
        "body_lock": False,
        "walk": "wheels",
        "walk_velocity_ms": 0.5,
        "wheels": _M20_WHEELS,
        "wheel_radius_m": wheel_radius,
    }


M20 = _m20("Deep Robotics M20")
M20S = _m20("Deep Robotics M20S")
M20_PIPER = _m20("Deep Robotics M20 + Piper arm")

QUADRUPED_CONFIGS = {
    "omniquad": OMNIQUAD,
    "lite3": LITE3,
    "x30": X30,
    "m20": M20,
    "m20s": M20S,
    "m20_piper": M20_PIPER,
}
