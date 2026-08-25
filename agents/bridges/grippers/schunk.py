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

"""Schunk EGK/EGx electric parallel grippers.

Schunk's electric grippers (EGK, EGU, EGL) expose a cyclic control/status
data layout over their fieldbus (PROFINET / EtherCAT / Modbus gateway).
Motion is commanded by a target position (in 1/100 mm), velocity and grip
force, with a control word toggling the move/grip bits; the status word
reports "position reached" and "gripped workpiece".

Reference skeleton -- the control/status *words* follow Schunk's EGK
fieldbus profile; map them onto your gateway's register addresses and
confirm scaling before deployment.  # TODO confirm
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .driver_base import RealGripperDriver, clamp

CTRL_WORD = 0x0000        # bit0 move, bit1 grip
TARGET_POS = 0x0001       # 1/100 mm
TARGET_VEL = 0x0002       # 1/100 mm/s
TARGET_FORCE = 0x0003     # percent
STATUS_WORD = 0x0010      # bit0 reached, bit1 gripped
STATUS_POS = 0x0011       # 1/100 mm


class SchunkEGxDriver(RealGripperDriver):
    kind = "parallel"
    driver_id = "schunk_egx"

    def __init__(self, *, model: str = "Schunk EGK", max_width: float = 0.084,
                 force_pct: int = 60, vel_mm_s: float = 50.0, **opts: Any) -> None:
        super().__init__(model=model, max_width=max_width, **opts)
        self.force_pct = int(clamp(force_pct, 0, 100))
        self.vel_mm_s = float(vel_mm_s)

    def _send(self, width_m: float, grip: bool) -> None:
        pos_hundredths = int(round(clamp(width_m, 0.0, self.max_width) * 100000))
        vel = int(round(self.vel_mm_s * 100))
        ctrl = 0b1 | (0b10 if grip else 0)
        self.transport.write_registers(TARGET_POS, [pos_hundredths & 0xFFFF])
        self.transport.write_registers(TARGET_VEL, [vel & 0xFFFF])
        self.transport.write_registers(TARGET_FORCE, [self.force_pct & 0xFFFF])
        self.transport.write_registers(CTRL_WORD, [ctrl])

    def open(self) -> Dict[str, Any]:
        self._width = self.max_width
        self._send(self.max_width, grip=False)
        self._holding = False
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._width = 0.0
        self._send(0.0, grip=True)
        self._holding = True
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        self._width = clamp(float(width_m), 0.0, self.max_width)
        self._send(self._width, grip=False)
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        if force is not None:
            self.force_pct = int(clamp(round(float(force) * 100), 0, 100))
        self._width = clamp(width if width is not None else 0.0, 0.0, self.max_width)
        self._send(self._width, grip=True)
        self._holding = True
        return self.state()

    def object_present(self) -> Optional[bool]:
        try:
            sw = self.transport.read_registers(STATUS_WORD, 1)[0]
            return bool(sw & 0b10)
        except Exception:
            return None
