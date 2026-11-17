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

"""Shared robustness helpers for the publisher nodes.

WHY THIS EXISTS
---------------
An exception raised inside an rclpy **timer callback** propagates out of
``rclpy.spin()`` and **terminates the process**. For a bridge whose every tick is
an HTTP round trip to another process, that turns any transient network hiccup
into a dead node that stays dead.

Measured 2026-08-17: a single ``ConnectionResetError``, raised while reading the
body of a 502 during a burst of traffic, killed ``robot_state_node`` outright
(``process has died ... exit code 1``) while every other node recovered on its
own moments later.

So every periodic callback is wrapped: a failure is logged, rate-limited, and the
node keeps running so it can recover when the far side does.
"""

from __future__ import annotations

import functools
import time
from typing import Callable

# Seconds between repeats of the same failure text. A node polling at 20 Hz that
# loses its peer would otherwise emit 20 identical lines a second.
REPEAT_LOG_INTERVAL_S = 10.0


def guard_timer(fn: Callable) -> Callable:
    """Wrap a node timer callback so it cannot terminate the process.

    Attaches its own rate-limiter state to the node instance, keyed by callback
    name, so two guarded timers on one node do not suppress each other.
    """

    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        try:
            return fn(self, *args, **kwargs)
        except Exception as exc:  # noqa: BLE001 -- deliberate; see module docstring
            key = f"_guard_{fn.__name__}"
            text = f"{type(exc).__name__}: {exc}"
            last_text, last_at = getattr(self, key, (None, 0.0))
            now = time.monotonic()
            if text != last_text or now - last_at > REPEAT_LOG_INTERVAL_S:
                self.get_logger().warn(
                    f"{fn.__name__} failed and was suppressed so the node stays "
                    f"alive: {text}"
                )
                setattr(self, key, (text, now))
            return None

    return wrapper
