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
                              (and, via min(spin_speed, v_max_angular), the
                              `turn` pulse rate and the `circle` curvature --
                              see SPIN_SPEED below)
    yaw_rate_gain             the fraction of the IDEAL kinematic yaw
                              ceiling the base can actually hold, 0..1

`yaw_rate_gain` is the one field here that is not geometry. Everything else
describes the robot; this describes what the SOLVER does with it, and it is
the difference between a bridge that reports what it commanded and one that
reports what the robot did.

A skid-steer base cannot pivot as fast as its wheel geometry says, because
a pivot has to scrub every wheel sideways. How much it loses is measured,
per base, below.

CURRENT -- measured 2026-09-11 (this laptop, RTX 3060, Newton/MuJoCo
cpu/mj_step, basicTimeStep 16, the chat worlds' default floor) THROUGH THIS
BRIDGE: yaw servo OFF (OMNISIM_MOBILE_YAW_SERVO=0), gain forced to 1.0
(OMNISIM_MOBILE_YAW_GAIN=1.0), a pure `set_velocity` yaw command held and
the settled supervisor yaw differenced in SIM time. Every base measured
separately -- no twin's number is carried across any more.

    base           kinematic ceiling  real ceiling   gain    shape
    tb3_waffle     1.375 rad/s        1.314 rad/s    0.955   flat
    tb3_waffle_pi  1.375 rad/s        1.314 rad/s    0.955   flat
    tb3_burger     2.475 rad/s        2.333 rad/s    0.942   flat
    rosbot         4.857 rad/s        3.503 rad/s    0.720   droop
    rosbot_xl      4.645 rad/s        2.435 rad/s    0.520   droop
    husky          3.471 rad/s        1.808 rad/s    0.520   droop
    jackal         3.144 rad/s        1.552 rad/s    0.490   droop, dips

`gain` is the fraction of the kinematic ceiling the base HOLDS AT FULL
COMMAND, so `max_angular_rad_s` is a rate a caller can actually reach.

RE-MEASURED 2026-10-03, same recipe (servo OFF, gain 1.0, a held
set_velocity(0, kinematic ceiling), settled supervisor yaw differenced in SIM
time over the last 3 s of a 5 s hold), 3 repeats per base, current engine
defaults (composite inertia + fixed-child colliders ON, joint dynamics OFF),
same laptop, CPU mujoco, each base on its own chat world. Rig + raw results:
social/launch/recheck_2026-10-03/followup/rig (run_wheeled.py --gain).

    base           real ceiling (3 runs)        gain (mean)  config
    husky          1.6494 / 1.6522 / 1.6463     0.4752       0.520 -> 0.475
    tb3_burger     2.2585 / 2.2585 / 2.2581     0.9125       0.942 -> 0.912
    tb3_waffle     1.2834 / 1.2831 / 1.2831     0.9333       0.955 -> 0.933
    jackal         1.5823 / 1.5822 / 1.5823     0.5033       0.490  (kept)
    rosbot         3.4230 x3                    0.7048       0.720  (kept)
    rosbot_xl      2.5025 x3                    0.5388       0.520  (kept)

The three that moved moved because the URDF fixed children (Husky top plate
and bumpers, TB3 scan/caster/plate links) now contribute their inertia to the
wheel-carrying body (OMNISIM_NEWTON_COMPOSITE_INERTIA, default ON): a heavier
yaw inertia pivots slower on the same wheel torque. With that hatch off the
same binary gives the old figures (Husky 0.534, Burger 0.951), so the drop is
the more complete model, not a regression. The other three bases are
unchanged against the hatches-off engine to 0.3%, so their 2026-09-11 values
were left alone; note the ROSbot's is 2.1% generous against today's 0.705 and
the Jackal / XL are 2.7% / 3.5% conservative. tb3_waffle_pi shares the Waffle
dict and was not re-swept separately.

THE SHAPE IS NO LONGER A DEAD-BAND. That was the whole reason this number
could not do the job alone, and it is gone:

    flat   the two-wheel TB3s are linear to within 1.5% of a single
           constant across the entire range (burger 0.947..0.962, waffle
           0.972..0.975). Their wheels sit on one axle through the centre,
           so a pivot scrubs nothing. A feed-forward divide is exact.
    droop  the four-wheel skid-steers are linear above ~1.0-1.5 rad/s and
           lose some gain at the slowest commands, where scrub break-away
           is a larger fraction of the demand: husky 0.480 (at 0.3 rad/s)
           -> 0.532 plateau; rosbot_xl 0.410 -> 0.53; rosbot 0.635 -> 0.736.
           Worst-case error from using one constant is about 20%, against
           the 20x a single constant used to be wrong by.
    dips   the Jackal alone is non-monotonic at the top: it peaks at 0.554
           (command 3.0) and falls to 0.504 at its kinematic ceiling. This
           REPRODUCES exactly across runs, so it is the platform, not noise.
           The gain quoted is the ceiling value, which is the conservative
           one -- the Jackal will not be asked for a rate it drops below.

HISTORY -- measured 2026-09-10, before commit 69b4b024b:

    base        kinematic ceiling   real ceiling   gain
    tb3_burger  2.475 rad/s         0.328 rad/s    0.1324   LINEAR
    husky       3.471 rad/s         0.119 rad/s    0.0343   DEAD-BAND
    rosbot_xl   4.645 rad/s         0.0322 rad/s   0.0069   DEAD-BAND

WHAT CHANGED: commit 69b4b024b fixed a solver defect, not a bridge one.
`_clamp_velocity_servo_gains` bounded a wheel's velocity-servo gain at
kv <= M_ii/dt, and since a velocity servo's peak torque is kv*(cmd - w),
that tied a wheel's STALL TORQUE to its own rotational inertia and severed
it from the effort its URDF declares (Husky: 9.5 N.m available against the
~25 N.m a four-tyre scrub pivot needs). Rolling straight needs almost no
torque, so it always tracked at 1.000; a pivot is the first real LOAD a
wheeled base meets, so it stalled at 0.013. The fix holds the same solver
bound with real rotor armature instead of by starving the gain. Yaw is now
15.2x better on the Husky, 7.1x on the TB3 Burger and 76x on the ROSbot XL,
and the dead-band that made those old numbers un-fittable is gone with it.

The gain SEEDS a feed-forward and an integrator in the bridge finishes the
job (MobileBridge._yaw_mix). What the gain owns on its own is the CEILING:
`max_angular_rad_s` published to callers, the clamp on `set_velocity`, and
whether `turn` accepts a rotation or refuses it with the measured rate
attached.

Re-measure with OMNISIM_MOBILE_YAW_SERVO=0 and OMNISIM_MOBILE_YAW_GAIN=1.0
if the solver, the floor friction or the robot model changes -- a gain that
is too high makes the bridge accept turns the base cannot finish. A base
with no entry defaults to 1.0, i.e. "assume the ideal kinematics" -- the
behaviour every base had before this was measured.

SPIN_SPEED -- REVIEWED 2026-09-11, and every value LEFT AS IT IS.

These presets had never been reviewed, because until 69b4b024b they could not
bind: the bridge takes `min(spin_speed, v_max_angular)` and v_max_angular was
0.03-0.33 rad/s on every base, so the CEILING always won and the preset was
inert. Post-fix every ceiling is 1.3-3.5 rad/s, so every preset binds, on
every base, for the first time -- and it binds in THREE places: the `spin`
chat verb (`set_velocity(0, spin_speed)`), the `circle` chat verb
(`set_velocity(cruise*0.6, spin_speed*0.6)`), and the magnitude of each
`turn` pulse once the residual exceeds spin_speed * TURN_RATE_HORIZON_S
(the pulse rate is sized continuously from the remaining angle; the 0.25-rad
TURN_SLOW_RAD switch it replaced was removed with the short-turn fix).

Measured through the bridge on each base's own chat world, machine
9722d23d12a3, OmniSim 8.4.0, engine binary sha256 2b44bbbdf9587eb0 -- ALL
SEVEN on that one binary, verified before and after each run, because the
first pass straddled a relink by another lane and silently mixed two engines.
Newton 1.5.0 SolverMuJoCo cpu/mj_step (sidecar-confirmed), basicTimeStep 16,
each figure differenced in SIM time over a 12 s hold -- i.e. exactly what an
operator typing "spin" or "circle" gets:

    base           cmd    held   ratio   rev in 12 s   in-place drift
    husky          0.60   0.565  0.941      1.08         0.018 mm
    jackal         0.80   0.752  0.940      1.44        36.90  mm
    rosbot         0.90   0.894  0.994      1.71         0.001 mm
    rosbot_xl      0.90   0.856  0.951      1.63         0.006 mm
    tb3_burger     1.00   1.000  1.000      1.91         0.040 mm
    tb3_waffle     0.90   0.898  0.998      1.72         0.249 mm
    tb3_waffle_pi  0.90   0.905  1.006      1.73         0.248 mm

EVERY preset is held to within 6% (the yaw servo closes the feed-forward; the
shortfall is the spin-up inside the window, not a ceiling -- the two skid
steers that read lowest here measured 0.995 and 1.003 on the previous
binary), every one produces 1.1-1.9 revolutions inside the VELOCITY_MAX_S
window -- visible, and nowhere near dizzying -- and "spin in place" really is
in place: sub-millimetre translation on six of seven. The Jackal walks 37 mm
over 12 s of spinning, which is 2000x the others and still 3.7 cm; it is the
same base whose yaw curve is the only non-monotonic one here, and it is not
worth trading pivot rate for. Nothing in this table asks to be changed.

⚠ ONE THING THE REVIEW DID TURN UP, and it is NOT a fault of these numbers.
`circle` derives its turn radius as cruise_frac*v_max_linear / spin_speed,
and those two quantities are independent, so on a base that is slow but
nimble the radius collapses below the machine itself. Measured radii
(commanded -> traced):

    husky 0.908 -> 0.886 m    jackal    0.404 -> 0.405 m
    rosbot 0.255 -> 0.250 m   rosbot_xl 0.320 -> 0.313 m
    tb3_burger 0.089 -> 0.087 m        tb3_waffle(_pi) 0.099 -> 0.097 m

The four-wheelers trace 2.1-3.1 half-tracks and look like circles. The TB3
Waffle traces 0.67 of a half-track: the instantaneous centre of rotation sits
INSIDE its own wheelbase and the inner wheel runs backwards, so `circle` on a
Waffle is a pirouette, not a circle. Left alone deliberately -- `circle` is
only a documented prompt for the Husky (docs/guide/omnilink-chat-demos.md),
where it measures 0.89 m, and the obvious fixes all cost something real: a
radius floor slows the arc on the one base that cannot afford it (a Waffle
covers 0.64 m of path in 12 s, so a 0.43 m-radius circle is 85 degrees of
arc, not a circle), and lowering spin_speed to buy radius would slow every
`turn` pulse on that base too. Flagged here with the numbers rather than
changed on taste.

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
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 0.565 rad/s = 1.08 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 0.6,
    # Measured 2026-09-11: real ceiling 1.808 rad/s against a 3.471 rad/s
    # kinematic one -- 15.2x the 0.119 rad/s this base held before
    # 69b4b024b. A quarter turn is ~0.9 s of spinning, not ~13 s. The
    # response droops at the slowest commands (0.480 of command at
    # 0.3 rad/s) and is flat at 0.53 above ~1.5; the yaw servo closes that
    # remainder, this number sets the ceiling.
    # RE-MEASURED 2026-10-03: 1.649 rad/s held (3 runs, 1.6463-1.6522),
    # gain 0.4752 -- down from 0.534 on the same binary because the fixed
    # top plate / bumpers now add their inertia to the chassis (composite
    # inertia, default ON). Table in the module docstring.
    "yaw_rate_gain": 0.475,
    # Footprint for route planning: 0.99 m x 0.67 m (husky.urdf base_link),
    # so a 0.45 m clearance radius keeps the body off what it routes around.
    "body_radius_m": 0.45,
    "half_length_m": 0.50,
}

JACKAL = {
    "model": "Clearpath Jackal",
    "layout": "4wheel_full",
    "wheel_radius_m": 0.098,
    "half_track_m": 0.187,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.55,
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 0.752 rad/s = 1.44 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 0.8,
    # MEASURED 2026-09-11 on its own world -- it no longer inherits the
    # Husky's number. Real ceiling 1.552 rad/s against a 3.144 rad/s
    # kinematic one. Alone among these bases its curve is non-monotonic:
    # it peaks at 0.554 of command at 3.0 rad/s and falls back to 0.504 at
    # the kinematic ceiling, reproducibly. The ceiling value is quoted
    # because it is the conservative end of that curve.
    "yaw_rate_gain": 0.490,
}

ROSBOT = {
    "model": "Husarion Rosbot",
    "layout": "4wheel_fl",
    "wheel_radius_m": 0.0425,
    "half_track_m": 0.105,
    "max_wheel_speed_radps": 12.0,
    "cruise_frac": 0.45,
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 0.894 rad/s = 1.71 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 0.9,
    # MEASURED 2026-09-11 on its own world -- it no longer inherits the
    # XL's number, and it is materially BETTER: real ceiling 3.503 rad/s
    # against a 4.857 rad/s kinematic one, gain 0.720 where the XL holds
    # 0.520. Smaller wheels on a narrower track scrub less. Droops to 0.635
    # at 0.3 rad/s, flat at ~0.73 above 1.5.
    "yaw_rate_gain": 0.720,
}

ROSBOT_XL = {
    "model": "Husarion Rosbot XL",
    "layout": "4wheel_fl",
    # 0.048, not 0.05: the URDF's wheel collision cylinders are r=0.048
    # (rosbot_xl.urdf), and the collision radius is what actually rolls.
    "wheel_radius_m": 0.048,
    "half_track_m": 0.124,
    "max_wheel_speed_radps": 12.0,
    "cruise_frac": 0.5,
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 0.856 rad/s = 1.63 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 0.9,
    # Measured 2026-09-11: real ceiling 2.435 rad/s against a 4.645 rad/s
    # kinematic one -- 76x the 0.0322 rad/s it held before 69b4b024b, which
    # is the largest change of any base here. A 90 deg pivot now costs
    # ~0.6 s of spinning instead of 49 s, so `turn` NO LONGER REFUSES on
    # this base (can_rotate_in_place is derived from the ceiling and is now
    # true). Droops to 0.410 at 0.3 rad/s, flat at ~0.53 above 1.0.
    #
    # STILL TRUE, and unrelated to the above: the XL's mecanum wheels are
    # modelled as plain CYLINDERS (rosbot_xl.urdf has no rollers), so the
    # bridge drives it as four-wheel skid-steer -- correctly, for the model
    # it is given. IT STILL CANNOT STRAFE. `can_strafe` stays false; only
    # the rotate-in-place refusal was lifted.
    "yaw_rate_gain": 0.520,
}

TB3_BURGER = {
    "model": "TurtleBot3 Burger",
    "layout": "2wheel",
    "wheel_radius_m": 0.033,
    "half_track_m": 0.080,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.45,
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 1.000 rad/s = 1.91 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 1.0,
    # Measured 2026-09-11: 0.947..0.962 of command, FLAT from 0.3 rad/s
    # right up to the 2.475 rad/s kinematic ceiling -- a single constant is
    # exact to within 1.5%. Real ceiling 2.333 rad/s, 7.1x the 0.328 rad/s
    # it held before 69b4b024b. Its two wheels sit on one axle through the
    # centre, so a pivot scrubs nothing -- which is why it lost least to
    # the old defect and gains least from the fix.
    # RE-MEASURED 2026-10-03: 2.258 rad/s held (3 runs, 2.2581-2.2585),
    # gain 0.9125 -- down from 0.951 with composite inertia off on the same
    # binary. Table in the module docstring.
    "yaw_rate_gain": 0.912,
}

TB3_WAFFLE = {
    "model": "TurtleBot3 Waffle",
    "layout": "2wheel",
    "wheel_radius_m": 0.033,
    "half_track_m": 0.144,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.45,
    # Reviewed 2026-09-11: binds (it never did before 69b4b024b),
    # held at 0.898 rad/s = 1.72 revolutions inside VELOCITY_MAX_S,
    # in place. Full table in the module docstring under SPIN_SPEED.
    "spin_speed": 0.9,
    # MEASURED 2026-09-11 on its own world -- it no longer inherits the
    # Burger's number. 0.972..0.975 of command, FLAT; real ceiling
    # 1.314 rad/s against a 1.375 rad/s kinematic one. The wider track
    # costs it kinematic ceiling but buys back scrub: it is the best
    # tracker of the whole set. TB3_WAFFLE_PI measured BIT-IDENTICAL on
    # every sweep point, so sharing this dict is correct, not an
    # assumption.
    # RE-MEASURED 2026-10-03: 1.283 rad/s held (3 runs, 1.2831-1.2834),
    # gain 0.9333 -- down from 0.975 hatches-off on the same binary
    # (composite inertia). Table in the module docstring.
    "yaw_rate_gain": 0.933,
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
    # Reviewed 2026-09-11: a supervisor-integrated body turns at exactly
    # the rate asked for, so this preset is delivered verbatim.
    "spin_speed": 0.9,
    # A supervisor-integrated body has no wheels to scrub: it turns at
    # exactly the rate it is asked for.
    "yaw_rate_gain": 1.0,
    "kinematic": True,
    # local +Y is forward: to face world heading h, node yaw = h - pi/2
    "forward_yaw_offset": -1.5707963,
    "floor_z": 0.0,
    # body is 1.259 m long -> rear coupling (trolley hitch dock) offset
    "rear_offset_m": 0.63,
    # Linear acceleration limit for the integrated pose (m/s^2). drive_to's
    # approach tapers at 1.5 m/s per metre (~0.9 m/s^2 at cruise), so this
    # only shapes the start and the last 0.25 m/s brake (~2 cm).
    "kin_accel_mps2": 1.5,
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
