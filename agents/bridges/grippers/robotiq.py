# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Robotiq grippers over the URCap Modbus register protocol.

Covers the 2F-85 / 2F-140 (and Hand-E) parallel grippers and the 3F
adaptive hand. Both speak the same style of Modbus register block:

  Robot OUTPUT (write), 3 x 16-bit registers @ 0x03E8 == 6 bytes:
    byte0  ACTION REQUEST  bit0 rACT  bit3 rGTO  bit4 rATR  bit5 rARD
    byte1  reserved
    byte2  reserved (2F)   / rMOD grip mode (3F: 0 basic 1 pinch 2 wide 3 scissor)
    byte3  rPR  position request  0 = open, 255 = closed
    byte4  rSP  speed             0..255
    byte5  rFR  force             0..255

  Robot INPUT (read), 3 x 16-bit registers @ 0x07D0 == 6 bytes:
    byte0  gACT gGTO gSTA(2) gOBJ(2)
    byte2  gFLT fault
    byte3  gPR  echoed position request
    byte4  gPO  actual position
    byte5  gCU  motor current

  gOBJ object-detection: 0 moving, 1 stopped-while-opening (object),
       2 stopped-while-closing (object), 3 at requested position.

The register *encoding* below is pure and unit-testable. Bytes reach the
device through an injected `Transport` (a pymodbus client in production,
a `DryTransport` otherwise), so this driver runs with no hardware.

Datasheet refs: Robotiq 2F-85/2F-140 Instruction Manual (register map
0x03E8 / 0x07D0); Robotiq 3-Finger Adaptive Gripper Instruction Manual.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .driver_base import RealGripperDriver, clamp

OUTPUT_BASE = 0x03E8   # 1000 — robot output / functional registers
INPUT_BASE = 0x07D0    # 2000 — robot input / status registers

# gOBJ codes
OBJ_MOVING = 0
OBJ_STOP_OPENING = 1
OBJ_STOP_CLOSING = 2
OBJ_AT_TARGET = 3


def pack_output(*, r_act: int, r_gto: int, position: int, speed: int,
                force: int, mode: int = 0, r_atr: int = 0) -> List[int]:
    """Pack a Robotiq 6-byte output frame into 3 Modbus registers.

    position/speed/force are 0..255. Returns [reg0, reg1, reg2], each
    16-bit big-endian (high byte first), matching FC 0x10 to 0x03E8.
    """
    b0 = (r_act & 1) | ((r_gto & 1) << 3) | ((r_atr & 1) << 4)
    b1 = 0
    b2 = mode & 0xFF
    b3 = int(clamp(position, 0, 255))
    b4 = int(clamp(speed, 0, 255))
    b5 = int(clamp(force, 0, 255))
    return [(b0 << 8) | b1, (b2 << 8) | b3, (b4 << 8) | b5]


def unpack_input(regs: List[int]) -> Dict[str, int]:
    """Decode the 3-register status block read from 0x07D0."""
    regs = (regs + [0, 0, 0])[:3]
    b0 = (regs[0] >> 8) & 0xFF
    b2 = (regs[1] >> 8) & 0xFF
    b3 = regs[1] & 0xFF
    b4 = (regs[2] >> 8) & 0xFF
    b5 = regs[2] & 0xFF
    return {
        "gACT": b0 & 1,
        "gGTO": (b0 >> 3) & 1,
        "gSTA": (b0 >> 4) & 0x3,
        "gOBJ": (b0 >> 6) & 0x3,
        "gFLT": b2 & 0xF,
        "gPR": b3,
        "gPO": b4,
        "gCU": b5,
    }


class Robotiq2FDriver(RealGripperDriver):
    """Robotiq 2F-85 / 2F-140 parallel gripper (also Hand-E)."""

    kind = "parallel"
    driver_id = "robotiq_2f"

    def __init__(self, *, model: str = "Robotiq 2F", max_width: float = 0.085,
                 speed: int = 255, force: int = 150, **opts: Any) -> None:
        super().__init__(model=model, max_width=max_width, **opts)
        self.speed = int(clamp(speed, 0, 255))
        self.force = int(clamp(force, 0, 255))
        self._pos = 0  # last commanded rPR (0 open .. 255 closed)

    # width (m) -> rPR (0 open, 255 closed)
    def _width_to_pos(self, width_m: float) -> int:
        if self.max_width <= 0:
            return 0
        frac = clamp(width_m / self.max_width, 0.0, 1.0)
        return int(round((1.0 - frac) * 255))

    def _pos_to_width(self, pos: int) -> float:
        return self.max_width * (1.0 - clamp(pos, 0, 255) / 255.0)

    def _send(self, position: int) -> None:
        self._pos = int(clamp(position, 0, 255))
        regs = pack_output(r_act=1, r_gto=1, position=self._pos,
                           speed=self.speed, force=self.force)
        self.transport.write_registers(OUTPUT_BASE, regs)

    def connect(self) -> Dict[str, Any]:
        # Activation handshake: clear, then set rACT=1.
        self.transport.write_registers(
            OUTPUT_BASE, pack_output(r_act=0, r_gto=0, position=0, speed=0, force=0))
        self.transport.write_registers(
            OUTPUT_BASE, pack_output(r_act=1, r_gto=0, position=0,
                                     speed=self.speed, force=self.force))
        self.connected = True
        return {"connected": True, "model": self.model, "activated": True}

    def open(self) -> Dict[str, Any]:
        self._send(0)
        self._holding = False
        self._width = self.max_width
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._send(255)
        self._holding = True
        self._width = 0.0
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        self._width = clamp(float(width_m), 0.0, self.max_width)
        self._send(self._width_to_pos(self._width))
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        if force is not None:
            self.force = int(clamp(round(float(force) * 255), 0, 255))
        target = width if width is not None else 0.0
        self.set_width(target)
        self._holding = True
        return self.state()

    def _status(self) -> Dict[str, int]:
        return unpack_input(self.transport.read_registers(INPUT_BASE, 3))

    def object_present(self) -> Optional[bool]:
        try:
            return self._status().get("gOBJ", 0) in (OBJ_STOP_OPENING, OBJ_STOP_CLOSING)
        except Exception:
            return None

    def state(self) -> Dict[str, Any]:
        st = super().state()
        st["rPR"] = self._pos
        return st


class Robotiq3FDriver(RealGripperDriver):
    """Robotiq 3-Finger Adaptive Gripper (synchronized control + grip modes)."""

    kind = "angular"
    driver_id = "robotiq_3f"

    _MODES = {"basic": 0, "pinch": 1, "wide": 2, "scissor": 3}

    def __init__(self, *, model: str = "Robotiq 3F", max_width: float = 0.155,
                 speed: int = 255, force: int = 150, mode: str = "basic",
                 **opts: Any) -> None:
        super().__init__(model=model, max_width=max_width, **opts)
        self.speed = int(clamp(speed, 0, 255))
        self.force = int(clamp(force, 0, 255))
        self.mode = mode if mode in self._MODES else "basic"
        self._pos = 0

    def _width_to_pos(self, width_m: float) -> int:
        if self.max_width <= 0:
            return 0
        frac = clamp(width_m / self.max_width, 0.0, 1.0)
        return int(round((1.0 - frac) * 255))

    def _send(self, position: int) -> None:
        self._pos = int(clamp(position, 0, 255))
        regs = pack_output(r_act=1, r_gto=1, position=self._pos,
                           speed=self.speed, force=self.force,
                           mode=self._MODES[self.mode])
        self.transport.write_registers(OUTPUT_BASE, regs)

    def connect(self) -> Dict[str, Any]:
        self.transport.write_registers(
            OUTPUT_BASE, pack_output(r_act=0, r_gto=0, position=0, speed=0, force=0))
        self.transport.write_registers(
            OUTPUT_BASE, pack_output(r_act=1, r_gto=0, position=0,
                                     speed=self.speed, force=self.force,
                                     mode=self._MODES[self.mode]))
        self.connected = True
        return {"connected": True, "model": self.model, "mode": self.mode}

    def set_mode(self, mode: str) -> Dict[str, Any]:
        if mode not in self._MODES:
            return {"error": f"unknown_mode:{mode}", **self.state()}
        self.mode = mode
        self._send(self._pos)
        return self.state()

    def open(self) -> Dict[str, Any]:
        self._send(0)
        self._holding = False
        self._width = self.max_width
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._send(255)
        self._holding = True
        self._width = 0.0
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        self._width = clamp(float(width_m), 0.0, self.max_width)
        self._send(self._width_to_pos(self._width))
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        if force is not None:
            self.force = int(clamp(round(float(force) * 255), 0, 255))
        self.set_width(width if width is not None else 0.0)
        self._holding = True
        return self.state()

    def object_present(self) -> Optional[bool]:
        try:
            st = unpack_input(self.transport.read_registers(INPUT_BASE, 3))
            return st.get("gOBJ", 0) in (OBJ_STOP_OPENING, OBJ_STOP_CLOSING)
        except Exception:
            return None

    def state(self) -> Dict[str, Any]:
        st = super().state()
        st["mode"] = self.mode
        st["rPR"] = self._pos
        return st

    def capabilities(self) -> Dict[str, Any]:
        caps = super().capabilities()
        caps["modes"] = sorted(self._MODES.keys())
        return caps
