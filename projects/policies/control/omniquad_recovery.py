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

"""Model-based self-righting for OmniQuad.

When the robot falls, classify which orientation the body is in (on
its left side, on its right side, on its back, face down, ...) and
apply the geometrically-correct leg motion to roll it toward upright.
Unlike a generic tuck/extend flail, this controller knows which legs
to push with and which to tuck for each fallen configuration:

  LEFT SIDE  — left legs push (extend toward ground), right legs tuck.
               Reaction force on body lifts left side up, rotating the
               chassis right (around the body X axis) toward upright.
  RIGHT SIDE — mirror.
  ON BACK    — body Z points down; pick one side and start a roll
               (commits to "roll right", same maneuver as LEFT SIDE
               since once past 90 deg the body will be on its right
               side and the RIGHT SIDE strategy takes over).
  FACE PLANT — body pitched nose-down. Rear legs extend (push body
               back over its center of mass), front legs tuck out of
               the way.
  UPRIGHT BUT LOW — body is level but bz too low to be standing.
                    All legs extend straight down to push body up.

The orientation is classified each tick from the body's Z axis
expressed in world frame, which is the third column of the
body-to-world rotation matrix that Webots returns. The classifier has
a "deadband" around the upright/fallen boundary so the controller
doesn't oscillate between strategies as the body rocks.

Once classified as UPRIGHT with bz > BZ_UPRIGHT and |vz| < SETTLED_VZ,
the FSM exits recovery and the gait resumes.

The mapping is the analytic model. A residual RL policy can sit on
top later (same recipe as the walker): observe orientation + body
state, output small per-leg joint offsets to refine the timing and
compensate for friction / inertia.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum


# ── Orientation thresholds ───────────────────────────────────────────

# Body Z axis in world frame is mostly aligned with one of the world
# axes. Classify by which component dominates.
ZAXIS_DOMINANT = 0.55   # |component| above this dominates the classification.

# Settled-upright thresholds — leave recovery only when the body is
# actually standing on its feet, not at the apex of a leg kick.
BZ_UPRIGHT = 0.55
BZ_UPRIGHT_MAX = 0.85
ROLL_UPRIGHT = 0.40
PITCH_UPRIGHT = 0.40
SETTLED_VZ = 0.5

# Fall trigger thresholds — slightly looser than the recovery exit so
# small perturbations during walking don't engage recovery.
BZ_FAIL = 0.30
ROLL_FAIL = 1.0
PITCH_FAIL = 1.0


# ── Orientation classes ──────────────────────────────────────────────

class Orientation(Enum):
    UPRIGHT = "upright"          # body Z mostly +world-Z
    ON_BACK = "on_back"          # body Z mostly -world-Z
    LEFT_SIDE = "left_side"      # body Z mostly +world-Y (left side down, top of body points right)
    RIGHT_SIDE = "right_side"    # body Z mostly -world-Y (right side down)
    FACE_PLANT = "face_plant"    # body Z mostly +world-X (chest down, head forward)
    REAR_PLANT = "rear_plant"    # body Z mostly -world-X (chest up, head backward)
    UNKNOWN = "unknown"          # in between -- treat conservatively


def classify_orientation(z_axis_world_x: float,
                         z_axis_world_y: float,
                         z_axis_world_z: float) -> Orientation:
    """Given the body's local Z axis expressed in world frame
    (ori[2], ori[5], ori[8] from Webots' getOrientation), return the
    Orientation that best describes the body's pose.

    Webots returns the rotation matrix row-major, so body's local Z
    axis in world coordinates is the third column of R:
    (R[0][2], R[1][2], R[2][2]) = (ori[2], ori[5], ori[8]).

    UPRIGHT is detected GENEROUSLY (z_world_z > 0.30) so the FSM
    transitions to the STAND pose well before the body fully levels.
    Continuing to push extend / tuck after the body crosses the
    equator overshoots and tumbles into ON_BACK. The is_upright
    check (used for FSM exit) is stricter; this loose classifier is
    used inside recovery to STOP the active push early enough.
    """
    if z_axis_world_z > 0.30:
        return Orientation.UPRIGHT
    if z_axis_world_z < -ZAXIS_DOMINANT:
        return Orientation.ON_BACK
    if z_axis_world_y > ZAXIS_DOMINANT:
        return Orientation.LEFT_SIDE
    if z_axis_world_y < -ZAXIS_DOMINANT:
        return Orientation.RIGHT_SIDE
    if z_axis_world_x > ZAXIS_DOMINANT:
        return Orientation.FACE_PLANT
    if z_axis_world_x < -ZAXIS_DOMINANT:
        return Orientation.REAR_PLANT
    return Orientation.UNKNOWN


# ── Per-leg joint pose primitives ────────────────────────────────────

# Joint signs per leg. hip_x is positive for the left side of the
# body, negative for the right (matches the agent's NOMINAL_POSE).
_LEG_SIGN = {"front_left": +1.0, "front_right": -1.0,
             "rear_left":  +1.0, "rear_right":  -1.0}

# TUCK: leg folded close to body. hip_x small (leg vertical, not
# splayed), hip_y small (thigh straight down), knee nearly fully bent
# (shank up against thigh, foot near the hip joint).
_TUCK_HIP_X_MAG = 0.05
_TUCK_HIP_Y = 0.05
_TUCK_KNEE = -1.15

# EXTEND_DOWN: leg pushed toward the ground, foot below the body.
# hip_x at nominal spread, hip_y at nominal forward sweep, knee
# moderately bent. Same as a slightly-deeper STAND pose so the leg
# can push the body off the ground without launching it on contact.
_EXTEND_HIP_X_MAG = 0.30
_EXTEND_HIP_Y = 0.35
_EXTEND_KNEE = -0.30

# STAND: the gait's nominal-stance joint angles.
_STAND_HIP_X_MAG = 0.30
_STAND_HIP_Y = 0.30
_STAND_KNEE = -0.60

# OVER: leg folded all the way OVER the body. Used for on-back
# recovery. When body is on its back, body's +Z direction points DOWN
# in the world frame, so a leg with hip_y near pi (= upper leg
# pointing in body +Z) reaches toward the ground. The foot presses
# against the floor and the reaction lifts the chassis off its back.
# Requires the widened URDF hip_y limit (>= 2.5 rad, see
# projects/robots/omnisim/omniquad/urdf/omniquad.urdf hip_y joints).
# Knee bent (-1.0) so the leg is compact through the swing -- reduces
# self-collision with the chassis during the fold-over.
_OVER_HIP_X_MAG = 0.05
_OVER_HIP_Y = 2.50
_OVER_KNEE = -1.00


def _per_leg_pose(leg: str, mode: str) -> tuple:
    """Return (hip_x, hip_y, knee) for one leg in a named mode."""
    sign = _LEG_SIGN[leg]
    if mode == "tuck":
        return (sign * _TUCK_HIP_X_MAG, _TUCK_HIP_Y, _TUCK_KNEE)
    if mode == "extend":
        return (sign * _EXTEND_HIP_X_MAG, _EXTEND_HIP_Y, _EXTEND_KNEE)
    if mode == "stand":
        return (sign * _STAND_HIP_X_MAG, _STAND_HIP_Y, _STAND_KNEE)
    if mode == "over":
        return (sign * _OVER_HIP_X_MAG, _OVER_HIP_Y, _OVER_KNEE)
    raise ValueError(f"unknown pose mode {mode!r}")


def righting_joint_targets(orientation: Orientation, joint_order) -> list:
    """Compute the per-leg joint targets for a given fallen orientation.

    `joint_order` is the controller's ordered list of (leg, joint)
    tuples (e.g. [('front_left','hip_x'), ('front_left','hip_y'),
    ('front_left','knee'), ...]). Returns a parallel list of float
    joint-target radians, 12 entries total.

    Strategy per orientation:
      LEFT_SIDE  : left legs EXTEND (push ground -> reaction lifts
                   left side of chassis), right legs TUCK
      RIGHT_SIDE : mirror
      ON_BACK    : pick a side (we pick LEFT EXTEND / RIGHT TUCK) to
                   commit to a roll; once past 90 deg the body becomes
                   RIGHT_SIDE and that strategy takes over naturally.
      FACE_PLANT : rear legs EXTEND (push the rear of body up,
                   pitching back toward upright), front legs TUCK
      REAR_PLANT : mirror
      UPRIGHT (called when bz too low): all legs STAND -> push up
      UNKNOWN    : all legs STAND, hope physics settles to a
                   classifiable orientation
    """
    out = [0.0] * (3 * 4)
    for i, (leg, joint) in enumerate(joint_order):
        if orientation == Orientation.LEFT_SIDE:
            mode = "extend" if "left" in leg else "tuck"
        elif orientation == Orientation.RIGHT_SIDE:
            mode = "extend" if "right" in leg else "tuck"
        elif orientation == Orientation.ON_BACK:
            # All four legs fold OVER the body in body frame. With body
            # on its back, body's +Z direction is world -Z (down), so
            # this puts the feet against the floor and the reaction
            # force lifts the chassis off its back. Requires the
            # widened URDF hip_y limit.
            mode = "over"
        elif orientation == Orientation.FACE_PLANT:
            mode = "extend" if "rear" in leg else "tuck"
        elif orientation == Orientation.REAR_PLANT:
            mode = "extend" if "front" in leg else "tuck"
        else:  # UPRIGHT (low) or UNKNOWN
            mode = "stand"
        hx, hy, kn = _per_leg_pose(leg, mode)
        if joint == "hip_x":
            out[i] = hx
        elif joint == "hip_y":
            out[i] = hy
        else:
            out[i] = kn
    return out


# ── FSM ──────────────────────────────────────────────────────────────

class RecoveryAction(Enum):
    NORMAL = "normal"
    RIGHTING = "righting"     # in recovery; controller commands the pose
                              # returned by righting_joint_targets


def is_upright(bz: float, roll: float, pitch: float,
               vz: float = 0.0, max_vz: float = SETTLED_VZ) -> bool:
    """Body is at a normal stand height, level, and not airborne."""
    return (BZ_UPRIGHT < bz < BZ_UPRIGHT_MAX
            and abs(roll) < ROLL_UPRIGHT
            and abs(pitch) < PITCH_UPRIGHT
            and abs(vz) < max_vz)


def is_fallen(bz: float, roll: float, pitch: float) -> bool:
    return (bz < BZ_FAIL or abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL)


@dataclass
class RecoveryFSM:
    """Per-controller recovery state.

    Picks a recovery strategy on entry and HOLDS it. Re-classifying
    orientation every tick made the controller flap legs back and forth
    as the body tumbled through orientations; the body never built up
    enough rotational momentum from any single push to actually roll
    upright. Instead the FSM picks a strategy at fall-time and only
    re-classifies once per "commit window" (default 1.5 s) so the body
    has time to respond.

    A min-time-between-flips constraint ensures committed strategies
    aren't abandoned mid-roll, and a timeout caps total recovery time
    so the FSM can't loop forever.
    """
    max_recovery_s: float = 8.0          # absolute recovery timeout
    commit_window_s: float = 1.5         # min seconds before reclassifying

    _in_recovery: bool = False
    _time_in_recovery: float = 0.0
    _committed_orient: Orientation = Orientation.UNKNOWN
    _time_in_committed: float = 0.0

    def step(self,
             bz: float, roll: float, pitch: float, vz: float,
             body_z_world_x: float, body_z_world_y: float,
             body_z_world_z: float,
             step_dt: float) -> tuple:
        """Decide what the controller should do this tick.

        Returns: (RecoveryAction, Orientation)
          action — NORMAL or RIGHTING.
          orient — the COMMITTED orientation strategy (held for
                   commit_window_s before reclassification). When
                   action == NORMAL this is just informational.
        """
        live_orient = classify_orientation(
            body_z_world_x, body_z_world_y, body_z_world_z)

        if not self._in_recovery:
            if is_fallen(bz, roll, pitch):
                self._enter(live_orient)
                return (RecoveryAction.RIGHTING, self._committed_orient)
            return (RecoveryAction.NORMAL, live_orient)

        # In recovery.
        self._time_in_recovery += step_dt
        self._time_in_committed += step_dt

        if is_upright(bz, roll, pitch, vz=vz):
            self._exit()
            return (RecoveryAction.NORMAL, live_orient)
        if self._time_in_recovery >= self.max_recovery_s:
            self._exit()
            return (RecoveryAction.NORMAL, live_orient)

        # Holding the committed strategy.
        # - Transition to UPRIGHT immediately (never wait, or we
        #   overshoot the upright window and tumble past it).
        # - Transitions between fallen orientations are gated by the
        #   commit window so we don't flap legs back and forth as the
        #   body rocks across class boundaries.
        if live_orient == Orientation.UPRIGHT:
            self._committed_orient = Orientation.UPRIGHT
            self._time_in_committed = 0.0
        elif (self._time_in_committed >= self.commit_window_s
                and live_orient != self._committed_orient
                and live_orient != Orientation.UNKNOWN):
            self._committed_orient = live_orient
            self._time_in_committed = 0.0

        return (RecoveryAction.RIGHTING, self._committed_orient)

    def _enter(self, orient: Orientation) -> None:
        self._in_recovery = True
        self._time_in_recovery = 0.0
        self._committed_orient = orient
        self._time_in_committed = 0.0

    def _exit(self) -> None:
        self._in_recovery = False
        self._time_in_recovery = 0.0
        self._committed_orient = Orientation.UNKNOWN
        self._time_in_committed = 0.0
