# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""OnRobot RG2 / RG6 parallel grippers.

OnRobot grippers are driven through the Compute Box over Modbus/TCP. The
RG family is commanded by a target width (in 1/10 mm) and a target force
(in 1/10 N) written to holding registers, with actual width / busy / grip
status read back.

This is a reference skeleton: the register *addresses* below follow the
OnRobot RG Modbus map (rTargetWidth / rTargetForce / rControl at the
gripper's command block; rActualWidth / rStatus at the status block). The
encoding is unit-testable through the injected transport; confirm the
exact register offsets against your Compute Box firmware before going
live -- they differ slightly across firmware revisions.  # TODO confirm
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .driver_base import RealGripperDriver, clamp

# Command block (write) and status block (read). Offsets per OnRobot RG
# Modbus/TCP documentation; verify against firmware.  # TODO confirm
CMD_TARGET_FORCE = 0x0000   # 1/10 N
CMD_TARGET_WIDTH = 0x0001   # 1/10 mm
CMD_CONTROL = 0x0002        # bit0 grip
STATUS_ACTUAL_WIDTH = 0x0100  # 1/10 mm
STATUS_FLAGS = 0x0101         # bit0 busy, bit2 grip-detected


class OnRobotRGDriver(RealGripperDriver):
    kind = "parallel"
    driver_id = "onrobot_rg"

    def __init__(self, *, model: str = "OnRobot RG", max_width: float = 0.110,
                 force_n: float = 40.0, **opts: Any) -> None:
        super().__init__(model=model, max_width=max_width, **opts)
        self.force_n = float(force_n)

    def _send(self, width_m: float, grip: bool) -> None:
        width_tenths_mm = int(round(clamp(width_m, 0.0, self.max_width) * 10000))
        force_tenths_n = int(round(self.force_n * 10))
        self.transport.write_registers(CMD_TARGET_FORCE, [force_tenths_n & 0xFFFF])
        self.transport.write_registers(CMD_TARGET_WIDTH, [width_tenths_mm & 0xFFFF])
        self.transport.write_registers(CMD_CONTROL, [1 if grip else 0])

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
            self.force_n = clamp(float(force), 0.0, 1.0) * 120.0  # 0-1 -> 0-120 N
        self._width = clamp(width if width is not None else 0.0, 0.0, self.max_width)
        self._send(self._width, grip=True)
        self._holding = True
        return self.state()

    def object_present(self) -> Optional[bool]:
        try:
            flags = self.transport.read_registers(STATUS_FLAGS, 1)[0]
            return bool(flags & 0b100)
        except Exception:
            return None
