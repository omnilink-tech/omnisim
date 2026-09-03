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

"""Documented external-wrench helper over Supervisor addForce / addTorque.

This module adds no engine capability. It wraps the three primitives that are
already public -- ``Node.addForce``, ``Node.addForceWithOffset`` and
``Node.addTorque`` -- with the two properties of that contract that are easy to
get silently wrong:

**Frames.** ``relative`` selects the frame of the *force vector only*. The
``offset`` is ALWAYS expressed in the node's local frame, whatever ``relative``
is set to. That asymmetry is deliberate and callers depend on it, so this
helper does not change it -- it surfaces it as an explicit ``frame=`` argument
and states the offset rule at the call site.

**Persistence.** A supervisor wrench is consumed into the solver's body-force
accumulator and cleared after the tick, so one call is a single-step impulse,
not a sustained load. Anything lasting must be re-applied every step. Because a
controller owns its own step loop, ``apply_wrench`` cannot schedule that
itself: it applies once and hands back a :class:`Wrench` whose ``tick()``
re-applies and goes falsy when the span is spent::

    w = apply_wrench(node, force=[0, 0, 30], frame="world", duration_s=0.5)
    while supervisor.step(timestep) != -1 and w.tick():
        ...

Units are SI throughout: force in newtons, torque in newton-metres, offset in
metres, duration in seconds.
"""

import ctypes
import math
import sys
import typing

from .node import Node
from .wb import wb

# Idempotent, and set here too so this module is usable before robot.py has
# been imported -- without it ctypes would truncate the double to an int.
wb.wb_robot_get_time.restype = ctypes.c_double

# Warn -- but still apply -- once the force exceeds this multiple of the body's
# own weight. Disturbance testing legitimately explores the extreme range, so a
# silent clamp or a refusal would trade one plausible-looking wrong trajectory
# for another. Callers who do want a hard ceiling pass ``clamp=``.
DEFAULT_WARN_FACTOR = 10.0

_GRAVITY = 9.81

# Keyed by (node id, kind, threshold) so a duration run warns once rather than
# once per tick.
_warned: typing.Set[typing.Tuple[int, str, float]] = set()


def _warn_once(node_id: int, kind: str, threshold: float, message: str) -> None:
    key = (node_id, kind, round(threshold, 6))
    if key in _warned:
        return
    _warned.add(key)
    sys.stderr.write("WARNING: [wrench] %s\n" % message)
    sys.stderr.flush()


def _check_vector(value: typing.Sequence[float], name: str) -> typing.List[float]:
    try:
        out = [float(v) for v in value]
    except (TypeError, ValueError):
        raise TypeError("%s must be a sequence of three numbers, got %r"
                        % (name, value))
    if len(out) != 3:
        raise ValueError("%s must have exactly three components, got %d"
                         % (name, len(out)))
    for v in out:
        if not math.isfinite(v):
            raise ValueError("%s contains a non-finite component (%r); refusing "
                             "to hand NaN or infinity to the solver" % (name, v))
    return out


def _body_mass(node: Node) -> typing.Optional[float]:
    """The body's mass in kg, or None when it cannot be read.

    A Physics node with mass -1 derives its mass from density and bounding
    volume, which is not resolvable from here; in that case the magnitude
    warning is skipped rather than computed from a wrong number.
    """
    try:
        physics_field = node.getField("physics")
        if physics_field is None:
            return None
        physics = physics_field.getSFNode()
        if physics is None:
            return None
        mass_field = physics.getField("mass")
        if mass_field is None:
            return None
        mass = mass_field.getSFFloat()
    except Exception:
        return None
    return mass if mass and mass > 0.0 else None


def _validate_target(node: Node) -> None:
    if node is None:
        raise TypeError("node must be a Node, got None")
    if not hasattr(node, "addForce") or not hasattr(node, "addTorque"):
        raise TypeError("node must be a Solid-derived Node exposing addForce / "
                        "addTorque, got %r" % type(node).__name__)
    try:
        physics_field = node.getField("physics")
    except Exception:
        physics_field = None
    if physics_field is None:
        raise ValueError("node %r has no 'physics' field, so it is not a "
                         "Solid-derived node the solver can push"
                         % node.getBaseTypeName())
    if physics_field.getSFNode() is None:
        raise ValueError("node has 'physics' set to NULL: the solver does not "
                         "own this body, so an external wrench would be "
                         "silently discarded")


class Wrench:
    """A wrench that has been applied once and can be re-applied per step.

    Returned by :func:`apply_wrench`; not constructed directly. Truthiness and
    ``tick()`` both report whether the span still has time left to run.
    """

    def __init__(self, node: Node, force, torque, offset, relative,
                 duration_s, start_time):
        self._node = node
        self._force = force
        self._torque = torque
        self._offset = offset
        self._relative = relative
        self._duration_s = duration_s
        self._start_time = start_time
        self._applications = 0
        self._cancelled = False
        self._apply()

    def _apply(self) -> None:
        if self._force is not None:
            if self._offset is not None:
                self._node.addForceWithOffset(self._force, self._offset,
                                              self._relative)
            else:
                self._node.addForce(self._force, self._relative)
        if self._torque is not None:
            self._node.addTorque(self._torque, self._relative)
        self._applications += 1

    def _elapsed(self) -> float:
        return wb.wb_robot_get_time() - self._start_time

    def tick(self) -> bool:
        """Re-apply for one more step; falsy once the span is spent.

        Call after the step that the previous application acted on. A one-shot
        wrench (``duration_s=None``) has already been applied by the time this
        object exists, so the first ``tick()`` reports False without applying
        anything again.
        """
        if self._cancelled or self._duration_s is None:
            return False
        if self._elapsed() >= self._duration_s:
            return False
        self._apply()
        return True

    def cancel(self) -> None:
        """Stop re-applying. Already-applied steps are not undone."""
        self._cancelled = True

    @property
    def applications(self) -> int:
        """How many times the wrench has actually been handed to the solver."""
        return self._applications

    @property
    def active(self) -> bool:
        if self._cancelled or self._duration_s is None:
            return False
        return self._elapsed() < self._duration_s

    def __bool__(self) -> bool:
        return self.active

    def __repr__(self) -> str:
        return ("Wrench(force=%r, torque=%r, offset=%r, frame=%r, "
                "duration_s=%r, applications=%d)"
                % (self._force, self._torque, self._offset,
                   "body" if self._relative else "world",
                   self._duration_s, self._applications))


def apply_wrench(node: Node,
                 force: typing.Optional[typing.Sequence[float]] = None,
                 torque: typing.Optional[typing.Sequence[float]] = None,
                 offset: typing.Optional[typing.Sequence[float]] = None,
                 frame: str = "world",
                 duration_s: typing.Optional[float] = None,
                 clamp: typing.Optional[float] = None,
                 warn_factor: float = DEFAULT_WARN_FACTOR) -> Wrench:
    """Apply an external wrench to ``node`` and return a handle for re-applying.

    :param node: a Solid-derived Node with a non-NULL ``physics``.
    :param force: force in newtons, in the frame given by ``frame``.
    :param torque: torque in newton-metres, in the frame given by ``frame``.
    :param offset: point of application in metres. ALWAYS in the node's local
        frame, including when ``frame="world"`` -- this mirrors the underlying
        ``addForceWithOffset`` contract rather than hiding it.
    :param frame: ``"world"`` or ``"body"``. Selects the frame of ``force`` and
        ``torque`` only; see ``offset``.
    :param duration_s: ``None`` applies for exactly one step. A value sets the
        span that ``Wrench.tick()`` will keep re-applying over, measured in
        simulation time so it lands on step boundaries.
    :param clamp: optional hard ceiling on force magnitude in newtons. Off by
        default: an unexpected magnitude is warned about and still applied.
    :param warn_factor: warn once when the force exceeds this multiple of the
        body's weight. Skipped when the mass cannot be read.

    :raises TypeError: on a target that is not a usable Node.
    :raises ValueError: on non-finite components, a bad ``frame``, a body with
        no Physics, or a non-positive ``duration_s`` / ``clamp``.
    """
    _validate_target(node)

    if frame not in ("world", "body"):
        raise ValueError("frame must be 'world' or 'body', got %r. There is no "
                         "default: the caller has to say which one it means."
                         % (frame,))
    if force is None and torque is None:
        raise ValueError("apply_wrench needs at least one of force= or torque=")
    if offset is not None and force is None:
        raise ValueError("offset= is only meaningful with force=; a torque is a "
                         "free vector and has no point of application")

    force = _check_vector(force, "force") if force is not None else None
    torque = _check_vector(torque, "torque") if torque is not None else None
    offset = _check_vector(offset, "offset") if offset is not None else None

    if duration_s is not None:
        duration_s = float(duration_s)
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be a positive finite number of "
                             "seconds, or None for a single step; got %r"
                             % (duration_s,))
    if clamp is not None:
        clamp = float(clamp)
        if not math.isfinite(clamp) or clamp <= 0.0:
            raise ValueError("clamp must be a positive finite magnitude in "
                             "newtons, or None; got %r" % (clamp,))

    if force is not None:
        magnitude = math.sqrt(sum(v * v for v in force))
        if clamp is not None and magnitude > clamp:
            scale = clamp / magnitude
            force = [v * scale for v in force]
            _warn_once(node.getId(), "clamp", clamp,
                       "force magnitude %.3f N exceeded clamp=%.3f N and was "
                       "scaled down to it" % (magnitude, clamp))
        elif magnitude > 0.0:
            mass = _body_mass(node)
            if mass is not None:
                threshold = warn_factor * mass * _GRAVITY
                if magnitude > threshold:
                    _warn_once(node.getId(), "magnitude", threshold,
                               "force magnitude %.3f N is over %.1fx the body's "
                               "own weight (%.3f N); applying it anyway -- pass "
                               "clamp= if you want a hard ceiling"
                               % (magnitude, warn_factor, mass * _GRAVITY))

    return Wrench(node, force, torque, offset, frame == "body",
                  duration_s, wb.wb_robot_get_time())
