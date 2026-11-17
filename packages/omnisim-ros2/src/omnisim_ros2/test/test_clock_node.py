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

"""ClockNode contracts using ROS, but no simulator or HTTP server.

The Linux rollback test injects realtime only in a disposable subprocess.
Its default-clock control verifies that injection actually reproduces #24.
"""

import ctypes
import json
import os
import shutil
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('rosgraph_msgs.msg')

# Dependency guards must run before importing the ROS-dependent application.
from omnisim_ros2.clock_node import ClockNode, HarnessUnreachable, LIVE_SOURCE  # noqa: E402, I100

from rclpy.clock import ClockType, ROSClock  # noqa: E402
from rclpy.timer import Timer  # noqa: E402


@contextmanager
def node_under_test(rate=20.0):
    """Create a real ClockNode with immediate, isolated bridge/harness responses."""
    bridge = SimpleNamespace(get_robot_state=lambda: SimpleNamespace(body={'sim_time': 12.5}))
    harness = SimpleNamespace(sim_state=lambda: SimpleNamespace(
        body={'engine_time_ms': 25000.0, 'sim_time_source': LIVE_SOURCE}))
    # ClockNode must override an accidental use_sim_time=true even when bare.
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'use_bridge_clock:=true',
                     '-p', f'publish_rate_hz:={float(rate)}'])
    node = None
    try:
        with patch('omnisim_ros2.clock_node.BridgeClient', return_value=bridge), \
                patch('omnisim_ros2.clock_node.HarnessClient', return_value=harness):
            node = ClockNode()
        yield node, bridge, harness
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_poll_clock_rate_and_qos():
    """Preserve the configured period, retained steady clock, and publisher QoS."""
    with node_under_test(rate=8.0) as (node, _, __):
        timer, = node.timers
        assert timer.clock is node._poll_clock
        assert timer.clock.clock_type == ClockType.STEADY_TIME
        assert timer.timer_period_ns == 125_000_000
        assert not node.get_parameter('use_sim_time').value
        assert node.pub.qos_profile.depth == 10


def test_bridge_stamps_include_pauses_and_world_reload_rewinds():
    """Publish simulator stamps unchanged through pauses and legitimate rewinds."""
    with node_under_test() as (node, bridge, harness):
        messages = []
        node.pub = SimpleNamespace(publish=messages.append)
        harness.sim_state = lambda: pytest.fail('healthy bridge should not use harness')
        for value in (12.5, 12.5, 13.0, 1.25):
            bridge.get_robot_state = lambda value=value: SimpleNamespace(body={'sim_time': value})
            node.tick()
        assert [(m.clock.sec, m.clock.nanosec) for m in messages] == [
            (12, 500_000_000), (12, 500_000_000), (13, 0), (1, 250_000_000)]


@pytest.mark.parametrize('unreachable', [False, True])
def test_harness_fallback_and_absent_time(unreachable):
    """Use the harness fallback without fabricating time when both sources lack it."""
    with node_under_test() as (node, bridge, harness):
        messages = []
        node.pub = SimpleNamespace(publish=messages.append)

        def no_bridge_time():
            if unreachable:
                raise HarnessUnreachable('test bridge unavailable')
            return SimpleNamespace(body={})

        bridge.get_robot_state = no_bridge_time
        node.tick()
        assert [(m.clock.sec, m.clock.nanosec) for m in messages] == [(25, 0)]
        harness.sim_state = lambda: SimpleNamespace(body={})
        node.tick()
        assert len(messages) == 1  # No fabricated zero when neither source has time.


def rollback_probe(shim_path):
    """Measure node/control timer readiness under a process-local realtime rollback."""
    shim = ctypes.CDLL(shim_path)
    shim.set_realtime_offset.argtypes = [ctypes.c_long]
    ticks = {'node': [], 'default_control': []}
    with node_under_test() as (node, bridge, _):
        timer, = node.timers
        control = Timer(None, None, timer.timer_period_ns, clock=ROSClock())
        stamps = []
        node.pub = SimpleNamespace(publish=lambda m: stamps.append((m.clock.sec, m.clock.nanosec)))
        bridge.get_robot_state = lambda: SimpleNamespace(body={'sim_time': 12.5})
        start = time.monotonic()
        injected = False
        try:
            while time.monotonic() - start < 2.8:
                elapsed = time.monotonic() - start
                if elapsed >= 0.3 and not injected:
                    shim.set_realtime_offset(-1_900_000_000)
                    injected = True
                for name, current in (('node', timer), ('default_control', control)):
                    if current.is_ready():
                        with current.handle:
                            current.handle.call_timer()
                        if name == 'node':
                            node.tick()
                        ticks[name].append(time.monotonic() - start)
                time.sleep(0.001)
        finally:
            shim.set_realtime_offset(0)
            control.destroy()
        assert stamps and all(s == (12, 500_000_000) for s in stamps)
    return {name: {'ticks': len(values), 'max_gap_s': max(b - a for a, b in zip(values, values[1:]))}
            for name, values in ticks.items()}


def test_polling_survives_process_local_realtime_rollback(tmp_path):
    """Keep polling under rollback while the default-clock control stalls."""
    if sys.platform != 'linux' or not shutil.which('gcc'):
        pytest.skip('process-local LD_PRELOAD injection requires Linux and gcc')
    shim = tmp_path / 'clock_rollback.so'
    subprocess.run(['gcc', '-shared', '-fPIC', '-o', str(shim),
                    str(Path(__file__).with_name('clock_rollback_shim.c')), '-ldl'], check=True)
    env = dict(os.environ, LD_PRELOAD=str(shim), ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    result = subprocess.run([sys.executable, __file__, str(shim)], env=env,
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(result.stdout.splitlines()[-1])
    (tmp_path / 'rollback-result.json').write_text(json.dumps(data, indent=2) + '\n')
    assert data['default_control']['max_gap_s'] > 1.8, data
    # Five periods allow ordinary CI jitter while rejecting the ~1.9 s stall.
    assert data['node']['max_gap_s'] < 0.25, data


if __name__ == '__main__':
    print(json.dumps(rollback_probe(sys.argv[1])))
