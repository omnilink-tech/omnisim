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

"""Emit on every Emitter, count arrivals on every Receiver; write the counts.

Instrument for tests/test_newton_receiver_occlusion_parity.py. Both robots in
the probe world run this controller: the emitter robot has only Emitters (it
sends a packet per tick and writes nothing), the receiver robot has only
Receivers (it drains the queues and writes JSON). Output path comes from
OMNISIM_RECEIVER_PROBE_OUT (written once, after the packets settle).
"""
import json
import os
import sys

from omnisim import Emitter, Receiver, Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

emitters, receivers = [], []
for i in range(robot.getNumberOfDevices()):
    d = robot.getDeviceByIndex(i)
    if isinstance(d, Emitter):
        emitters.append(d)
    elif isinstance(d, Receiver):
        d.enable(dt)
        receivers.append(d)

counts = {r.getName(): 0 for r in receivers}
for _ in range(24):
    for e in emitters:
        e.send(b"ping")
    if robot.step(dt) == -1:
        break
    for r in receivers:
        while r.getQueueLength() > 0:
            counts[r.getName()] += 1
            r.nextPacket()

if receivers:
    out = {name: {"received": c > 0, "count": c} for name, c in counts.items()}
    path = os.environ.get("OMNISIM_RECEIVER_PROBE_OUT", "receiver_probe_out.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    sys.stdout.write("receiver_occlusion_probe: wrote %d receivers -> %s\n" % (len(out), path))
    sys.stdout.flush()
