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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Vacuum / suction effector over digital IO.

Suction grippers are usually driven by a single digital output (solenoid
valve on/off) with an optional vacuum/part-present feedback input. There
is no width control. open()/release() vent (output OFF); close()/grasp()
apply suction (output ON). `object_present` reads the feedback channel.

Channels are configurable via opts: `valve_channel` (output) and
`sense_channel` (input). With a DryTransport the output toggles a
simulated input so presence echoes the valve state.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .driver_base import RealGripperDriver


class VacuumIODriver(RealGripperDriver):
    kind = "vacuum"
    driver_id = "vacuum_io"

    def __init__(self, *, model: str = "Vacuum Suction",
                 valve_channel: int = 0, sense_channel: int = 0,
                 **opts: Any) -> None:
        # max_width stays 0 -> no width control advertised.
        super().__init__(model=model, max_width=0.0, **opts)
        self.valve_channel = int(valve_channel)
        self.sense_channel = int(sense_channel)

    def _set_valve(self, on: bool) -> None:
        self.transport.set_output(self.valve_channel, on)

    def open(self) -> Dict[str, Any]:
        self._set_valve(False)
        self._holding = False
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._set_valve(True)
        self._holding = True
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        return self.close()

    def release(self) -> Dict[str, Any]:
        return self.open()

    def object_present(self) -> Optional[bool]:
        try:
            return bool(self.transport.read_input(self.sense_channel))
        except Exception:
            return None
