#!/usr/bin/env python3
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

"""Runs the shipped mavic_omnilink_bridge from this benchmark's project tree.

A controller is resolved against the project directory that owns the world, so a
world under tests/ cannot see projects/samples/demos/controllers/ and the engine
silently falls back to `generic` -- the bridge never binds 6090 and the mission
script waits forever on a port nothing is listening to.

This execs the shipped controller rather than copying it: a regression that runs
its own fork of the thing it is testing is not a regression.
"""

import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
BRIDGE = os.path.join(ROOT, "projects", "samples", "demos", "controllers",
                      "mavic_omnilink_bridge", "mavic_omnilink_bridge.py")

if not os.path.isfile(BRIDGE):
    sys.exit("mavic_omnilink_bridge not found at %s" % BRIDGE)

sys.path.insert(0, os.path.dirname(BRIDGE))
runpy.run_path(BRIDGE, run_name="__main__")
