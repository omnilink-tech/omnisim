# Copyright 2026 OmniLink. SPDX-License-Identifier: Apache-2.0
"""Safety checks: never start hot; stop owned workload on sensor loss."""
import argparse
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

spec = importlib.util.spec_from_file_location('thermal_guard',Path(__file__).resolve().parents[1]/'scripts/dev/thermal_guard.py')
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class ThermalGuardTests(unittest.TestCase):
    def args(self):
        return argparse.Namespace(command=['dummy'],unguarded=False,ceiling=75,
                                  precool=None,interval=.01)

    def test_refuses_hot_start(self):
        with patch.object(guard,'read_temp',return_value=75), patch.object(guard.subprocess,'Popen') as launch:
            self.assertEqual(guard.cmd_run(self.args()),3)
            launch.assert_not_called()

    def test_refuses_unreadable_start(self):
        with patch.object(guard,'read_temp',return_value=None), patch.object(guard.subprocess,'Popen') as launch:
            self.assertEqual(guard.cmd_run(self.args()),2)
            launch.assert_not_called()

    def test_kills_owned_process_when_sensor_disappears(self):
        process = MagicMock()
        process.poll.return_value = None
        process.wait.return_value = 1
        with patch.object(guard,'read_temp',side_effect=[60,None,61]), \
             patch.object(guard.subprocess,'Popen',return_value=process), \
             patch.object(guard,'kill_tree') as kill, patch.object(guard.time,'sleep'), \
             patch.object(guard,'WindowsJob'):
            self.assertEqual(guard.cmd_run(self.args()),2)
            kill.assert_called_once_with(process)

    def test_kills_owned_process_at_threshold(self):
        process = MagicMock()
        process.poll.return_value = None
        process.wait.return_value = 1
        with patch.object(guard,'read_temp',side_effect=[60,75,74]), \
             patch.object(guard.subprocess,'Popen',return_value=process), \
             patch.object(guard,'kill_tree') as kill, patch.object(guard.time,'sleep'), \
             patch.object(guard,'WindowsJob'):
            self.assertEqual(guard.cmd_run(self.args()),3)
            kill.assert_called_once_with(process)


if __name__ == '__main__':
    unittest.main()
