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

"""Drive an OmniSim robot from ROS 2 topics (Tier 2).

Subscribes the two standard command interfaces and forwards them to the robot's
own **bridge** (PROTOCOL.md §6), not to the harness:

* ``cmd_vel``       (``geometry_msgs/Twist``)      -> ``POST /set_velocity``
* ``joint_command`` (``sensor_msgs/JointState``)   -> ``POST /servo_joint_positions``
  when the bridge advertises it (``capabilities.servo``), else
  ``POST /set_joint_positions``

JOINT-COMMAND LANE SELECTION
----------------------------
``joint_command`` is a *stream* (a trajectory controller or teleop publishes a
new setpoint every cycle), and the two bridge verbs treat a stream very
differently (PROTOCOL.md §6.1):

* ``/servo_joint_positions`` is the streaming lane: non-blocking,
  last-write-wins, never 409s for servo-on-servo, and preempts an in-flight
  goal. This is the verb a stream belongs on.
* ``/set_joint_positions`` is a GOAL: a command arriving while the previous
  interpolation is still running answers **409 busy and is NOT applied**, so a
  stream pointed at it lands in pieces (measured: a second command 50 ms after
  the first was refused).

The bridge self-describes, so the node reads ``/capabilities`` once and picks
the servo verb when ``capabilities.servo`` is there, falling back to the goal
verb for older bridges and other robot classes. The chosen lane is logged at
startup; on the fallback lane a 409 is an *expected* property of the contract
and the warning says so, on the servo lane it would mean a non-conforming
bridge.

**Why the bridge and not the harness.** The harness's supervisor can teleport a
body, but it cannot drive a motor that belongs to another robot -- OmniSim
restricts device APIs to the owning controller. Teleporting a robot instead of
actuating it would look like it worked and would be physically meaningless, so
commands go to the controller that owns the motors.

TWIST CONVENTION
----------------
``linear.x`` (forward, m/s) and ``angular.z`` (yaw rate, rad/s) are used; that is
REP-103 body frame and matches the bridge's ``linear``/``angular`` fields exactly,
because OmniSim worlds are Z-up/ENU. **The other four components are ignored** --
a differential-drive base cannot execute them. Rather than silently discarding
them, the node warns once if a caller ever sends a non-zero ``linear.y``,
``linear.z``, ``angular.x`` or ``angular.y``, so "my robot ignores my command" is
a message instead of a mystery.

WATCHDOG
--------
``cmd_vel`` is a *continuous* interface: the convention is that a publisher keeps
sending, and the robot stops when it stops hearing. A simulator that keeps driving
after the planner dies is a hazard even in simulation, so if no ``cmd_vel``
arrives within ``cmd_vel_timeout_s`` (default 0.5 s) and the last command was
non-zero, the node sends one explicit stop. It sends exactly one -- not a stream
of zeros -- so it never fights a bridge-side motion started by another client.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from omnisim_ros2.bridge_client import BridgeClient, DEFAULT_BRIDGE_URL, bridge_servo_verb
from omnisim_ros2.harness_client import HarnessUnreachable
from omnisim_ros2.node_support import guard_timer

# Below this, a command is treated as "stop" for watchdog purposes.
ZERO_EPS = 1e-9


class CommandNode(Node):
    def __init__(self) -> None:
        super().__init__("omnisim_command")

        self.declare_parameter("bridge_url", DEFAULT_BRIDGE_URL)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("enable_joint_command", True)

        url = self.get_parameter("bridge_url").get_parameter_value().string_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value
        self.cmd_timeout = (
            self.get_parameter("cmd_vel_timeout_s").get_parameter_value().double_value
        )

        self.bridge = BridgeClient(url, timeout_s=timeout)
        # ⚠ DEPTH 1, DELIBERATELY. Forwarding one command is a blocking HTTP
        # round trip (tens to hundreds of ms through a tunnel), so a deeper queue
        # does not smooth anything -- it stores STALE velocities and executes them
        # late. A command topic wants the NEWEST command, never a queued one, and
        # a velocity executed a second after it was superseded is a wrong command,
        # not a late one.
        #
        # Observed 2026-08-17 at the default depth of 10: after the publisher
        # stopped, queued commands kept arriving and each one re-armed the stop
        # watchdog (18 consecutive fires). The robot's over-travel on that run is
        # NOT offered as evidence -- it had driven into the arena wall, so the
        # distance is unattributable either way.
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 1)
        if self.get_parameter("enable_joint_command").get_parameter_value().bool_value:
            self.create_subscription(JointState, "joint_command", self.on_joint_command, 1)

        self._last_cmd_at: float | None = None
        self._last_watchdog_log = -1e9
        self._moving = False
        self._warned_ignored = False
        self._warned_unreachable = False
        # Joint-command lane (PROTOCOL.md §6.1): the advertised servo verb, or
        # None for the goal verb. Unprobed until the bridge has answered
        # /capabilities once -- probed at startup when reachable, else lazily
        # on the first joint_command, so a bridge that comes up late is still
        # detected rather than permanently demoted to the goal lane.
        self._servo_verb: str | None = None
        self._servo_probed = False
        self.create_timer(max(self.cmd_timeout / 4.0, 0.05), self._watchdog)

        self.get_logger().info(
            f"forwarding cmd_vel -> {url}/set_velocity "
            f"(watchdog {self.cmd_timeout:g} s)"
        )
        self._log_bridge_banner()

    def _log_bridge_banner(self) -> None:
        try:
            resp = self.bridge.get_robot_state()
        except HarnessUnreachable as exc:
            self.get_logger().warn(
                f"{exc} -- commands will be dropped until the bridge is reachable. "
                f"The bridge is started by the robot's controller inside the world; "
                f"check the world's controllerArgs for its --port."
            )
            return
        body = resp.body
        self.get_logger().info(
            f"bridge reachable: id={body.get('id')!r} model={body.get('model')!r} "
            f"mode={body.get('mode')!r}"
        )
        if self.get_parameter("enable_joint_command").get_parameter_value().bool_value:
            self._probe_servo()

    def _probe_servo(self) -> None:
        """One ``/capabilities`` read decides the joint-command lane.

        Idempotent and cheap; called at startup and again from
        ``on_joint_command`` while unprobed (bridge not up yet). An unreachable
        bridge leaves the probe pending; a reachable bridge that does not
        advertise ``capabilities.servo`` settles the fallback for good.
        """
        if self._servo_probed:
            return
        try:
            caps = self.bridge.capabilities()
        except HarnessUnreachable:
            return  # stay unprobed; retried on the next joint_command
        self._servo_probed = True
        self._servo_verb = bridge_servo_verb(caps.body) if caps.ok else None
        if self._servo_verb:
            self.get_logger().info(
                f"joint_command -> POST /{self._servo_verb} (bridge advertises "
                f"capabilities.servo: non-blocking, last-write-wins, preempts goals; "
                f"a setpoint stream never answers 409)"
            )
        else:
            self.get_logger().warn(
                "joint_command -> POST /set_joint_positions (this bridge does not "
                "advertise capabilities.servo). That is a GOAL verb: a command "
                "arriving while the previous interpolation is still running answers "
                "409 busy and is NOT applied, so a setpoint stream will land in "
                "pieces. Arm bridges with the streaming verb are picked up "
                "automatically -- this affects older bridges and non-arm bridges only."
            )

    @staticmethod
    def _now() -> float:
        """Monotonic WALL time for the watchdog.

        ⚠ NOT ``self.get_clock()``. Under ``use_sim_time:=true`` that returns
        *simulation* time, and OmniSim runs headless worlds with ``--mode=fast``,
        where simulated seconds elapse far faster than real ones. Measured
        2026-08-17: with the node clock, the 0.5 s watchdog expired between
        consecutive 5 Hz commands and stopped the robot after every one -- a
        6-second drive covered **0.028 m** instead of ~3 m, and the watchdog
        logged continuously.

        The watchdog asks "is the ROS publisher still alive?", which is a
        wall-clock question about another process. It must keep working even when
        the simulation clock is stopped, which is exactly when a runaway command
        matters most.
        """
        return time.monotonic()

    @guard_timer
    def on_cmd_vel(self, msg: Twist) -> None:
        ignored = (
            abs(msg.linear.y) + abs(msg.linear.z)
            + abs(msg.angular.x) + abs(msg.angular.y)
        )
        if ignored > ZERO_EPS and not self._warned_ignored:
            self.get_logger().warn(
                "cmd_vel carries non-zero linear.y/linear.z/angular.x/angular.y; "
                "a differential-drive base cannot execute those and they are "
                "ignored. Only linear.x and angular.z are used."
            )
            self._warned_ignored = True

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        self._last_cmd_at = self._now()
        self._moving = abs(linear) > ZERO_EPS or abs(angular) > ZERO_EPS
        self._send_velocity(linear, angular)

    def _send_velocity(self, linear: float, angular: float) -> None:
        try:
            resp = self.bridge.set_velocity(linear, angular)
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"bridge unreachable, dropping command: {exc}")
                self._warned_unreachable = True
            return
        if self._warned_unreachable:
            self.get_logger().info("bridge is back")
            self._warned_unreachable = False
        if not resp.ok:
            self.get_logger().warn(f"set_velocity rejected: {resp.error}")

    @guard_timer
    def _watchdog(self) -> None:
        if not self._moving or self._last_cmd_at is None:
            return
        if self._now() - self._last_cmd_at < self.cmd_timeout:
            return
        # Rate-limit the message, not the stop. A late command re-arms the
        # watchdog legitimately, so the stop must still fire every time -- but
        # one line per stop episode is enough for a human to act on.
        now = self._now()
        if now - self._last_watchdog_log > 5.0:
            self.get_logger().warn(
                f"no cmd_vel for {self.cmd_timeout:g} s; stopping the robot. "
                f"cmd_vel is a continuous interface -- keep publishing to keep moving."
            )
            self._last_watchdog_log = now
        # Clear the flag FIRST so a failed send cannot loop the warning.
        self._moving = False
        self._send_velocity(0.0, 0.0)

    @guard_timer
    def on_joint_command(self, msg: JointState) -> None:
        if not msg.position:
            self.get_logger().warn(
                "joint_command carried no positions; OmniSim's arm bridge takes "
                "positions only (velocity and effort commands are not supported)"
            )
            return
        self._probe_servo()
        verb = self._servo_verb
        try:
            if verb:
                resp = self.bridge.servo_joint_positions(list(msg.position), verb=verb)
            else:
                resp = self.bridge.set_joint_positions(list(msg.position))
        except HarnessUnreachable as exc:
            if not self._warned_unreachable:
                self.get_logger().warn(f"bridge unreachable, dropping command: {exc}")
                self._warned_unreachable = True
            return
        if not resp.ok:
            if verb:
                # On the servo lane a 409 is NOT an expected condition: the
                # contract (PROTOCOL.md §6.1) is last-write-wins, servo-on-servo
                # retargets in place and a goal in flight is preempted. Seeing
                # one means the bridge is not honouring its own advertisement.
                extra = (
                    " -- a 409 on the servo lane violates the bridge's own "
                    "capabilities.servo contract (last-write-wins, never busy); "
                    "check the bridge version"
                    if resp.status == 409 else ""
                )
                self.get_logger().warn(f"{verb} rejected: {resp.error}{extra}")
            elif resp.status == 409:
                # Expected on the goal lane: the bridge refuses a command while
                # the previous interpolation runs, and the refused command is
                # NOT applied. This is the contract, not a transient error.
                self.get_logger().warn(
                    f"set_joint_positions answered 409 busy: {resp.error}. This "
                    f"bridge has no servo verb, so overlapping commands are "
                    f"refused by design -- publish below the motion duration, or "
                    f"use a bridge that advertises capabilities.servo."
                )
            else:
                self.get_logger().warn(
                    f"set_joint_positions rejected: {resp.error} "
                    f"(is this bridge an arm bridge? mobile bridges do not expose it)"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
