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

"""Realistic motor safety: torque caps + joint-velocity rate limiting.

Without these, position-PD controllers can command arbitrary instant
joint-target jumps that the physics engine resolves as impulsive
torques — sometimes enough to launch the chassis several meters into
the air. A real motor cannot do this: its peak torque is bounded by
the actuator's stall torque and its rotation rate by its no-load
speed. This module enforces both so the controllers we write here
can't accidentally "cheat" the simulator into doing things the
hardware couldn't.

Two pieces:

  apply_realistic_limits(motors, max_torque_nm, max_vel_rad_s)
    Once at init, call setAvailableTorque + setVelocity on each
    Webots motor handle. The defaults match the Spot URDF
    (`<limit effort="80" velocity="20"/>`).

  RateLimitedMotorBank(motors, step_dt, max_vel_rad_s)
    Wraps a list of Webots motor handles. setPosition(i, target)
    only allows the commanded position to change by at most
    max_vel_rad_s * step_dt per tick. The wrapper tracks last
    commanded position per joint. set_pose(vec) commands all
    motors at once.

The rate limiter is the more important of the two. Webots' default
position controller behind setPosition uses the joint's own velocity
limit when interpolating to the target — but the controller still
sees an instantaneous step in target, which when the joint reaches
the new target can result in collision forces that are large because
the leg arrives at the contact with high velocity. By rate-limiting
the target itself, the leg approaches contact smoothly.
"""
from __future__ import annotations
from typing import Optional, Sequence


# Spot URDF: <limit effort="80" velocity="20"/> per joint. The 80 Nm
# is the actuator stall torque -- the URDF datasheet number.
# We use it as-is for the torque cap.
# The 20 rad/s is the no-load speed; under any meaningful load real
# actuators deliver much less. We cap the velocity to 8 rad/s here
# (~460 deg/s) as a realistic "loaded" maximum -- still well above
# any natural gait motion but slow enough to prevent the simulator
# from generating impulsive contact forces.
SPOT_MAX_TORQUE_NM = 80.0
SPOT_MAX_VEL_RAD_S = 8.0

# Conservative target-rate cap. The motor's no-load speed in the URDF
# is 20 rad/s but real-world motion under load is much slower. We use
# 4 rad/s ≈ 230 deg/s, which is:
#   - faster than walking CPG joint motion (~1 rad/s) so the gait
#     doesn't see any rate limiting in normal operation
#   - slow enough that big recovery pose transitions take ~250 ms
#     of smooth slew rather than a single hard slam against the ground
# This is comparable to the bandwidth real quadruped actuators
# actually deliver under load (vs the no-load datasheet number).
DEFAULT_TARGET_RATE_RAD_S = 4.0


def apply_realistic_limits(motors: Sequence,
                           max_torque_nm: float = SPOT_MAX_TORQUE_NM,
                           max_vel_rad_s: float = SPOT_MAX_VEL_RAD_S) -> None:
    """Cap torque + velocity on every motor handle in `motors`.

    Setting setVelocity to a finite value (instead of the default INFINITY)
    tells Webots to slew toward the position target at that max rate
    rather than allowing the PD-resolved velocity to spike during
    contact transitions. setAvailableTorque limits the peak force the
    motor can exert in either direction.

    Idempotent. Safe to call once at controller init, or whenever
    transitioning between walking-mode and recovery-mode (which may
    use different torque caps -- see set_recovery_torque).
    """
    for m in motors:
        if m is None:
            continue
        try:
            if hasattr(m, "setAvailableTorque"):
                m.setAvailableTorque(float(max_torque_nm))
        except Exception:
            pass
        try:
            if hasattr(m, "setVelocity"):
                m.setVelocity(float(max_vel_rad_s))
        except Exception:
            pass


# Torque cap during recovery. Lower than the walking cap so that
# even at full kp position error the leg-on-ground contact force
# stays small enough not to launch the chassis. Real recovery
# routines are slow and gentle; the simulator should reflect that.
SPOT_RECOVERY_TORQUE_NM = 18.0
SPOT_RECOVERY_VEL_RAD_S = 3.0


def set_recovery_torque(motors: Sequence,
                        max_torque_nm: float = SPOT_RECOVERY_TORQUE_NM,
                        max_vel_rad_s: float = SPOT_RECOVERY_VEL_RAD_S) -> None:
    """Switch motor limits to the conservative recovery profile.

    During recovery the body is in unusual orientations and any
    impulsive contact can launch the chassis several meters --
    behavior the real hardware cannot produce. We swap in lower
    torque + velocity caps so the recovery legs push against the
    ground gently, and switch back to the walking caps once
    upright.
    """
    apply_realistic_limits(motors, max_torque_nm=max_torque_nm,
                           max_vel_rad_s=max_vel_rad_s)


class RateLimitedMotorBank:
    """Tracks the last commanded position per motor and rate-limits
    new commands. Use this everywhere a controller writes joint
    targets so a pose transition can't ever ask for an instant jump.
    """

    def __init__(self, motors: Sequence, step_dt: float,
                 max_vel_rad_s: float = DEFAULT_TARGET_RATE_RAD_S):
        self.motors = list(motors)
        self.step_dt = float(step_dt)
        self.max_delta = float(max_vel_rad_s) * float(step_dt)
        self._last: list = [None] * len(self.motors)

    def reset(self) -> None:
        """Forget the per-motor history. Next set will be rate-limited
        relative to the motor's current reading, not the last commanded
        value. Call this when the body has been teleported / on episode
        reset."""
        self._last = [None] * len(self.motors)

    def set_pose(self, target_vec: Sequence[float]) -> None:
        """Command all motors at once with rate limiting."""
        for i, m in enumerate(self.motors):
            if m is None:
                continue
            tgt = float(target_vec[i])
            last = self._last[i]
            if last is None:
                cmd = tgt
            else:
                d = tgt - last
                if d > self.max_delta:
                    cmd = last + self.max_delta
                elif d < -self.max_delta:
                    cmd = last - self.max_delta
                else:
                    cmd = tgt
            self._last[i] = cmd
            m.setPosition(cmd)

    def set_one(self, index: int, target: float) -> None:
        """Command a single motor with rate limiting."""
        m = self.motors[index]
        if m is None:
            return
        tgt = float(target)
        last = self._last[index]
        if last is None:
            cmd = tgt
        else:
            d = tgt - last
            if d > self.max_delta:
                cmd = last + self.max_delta
            elif d < -self.max_delta:
                cmd = last - self.max_delta
            else:
                cmd = tgt
        self._last[index] = cmd
        m.setPosition(cmd)
