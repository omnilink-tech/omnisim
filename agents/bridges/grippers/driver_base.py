# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""RealGripperDriver -- hardware-side counterpart of the sim GripperEffector.

The sim bridge wraps a gripper config in a `GripperEffector` and exposes
open/close/set_width/grasp/release/state. On real hardware the *same*
verbs are served by a `RealGripperDriver` whose body talks to the gripper
over its bus (Modbus/URCap, digital IO, Compute Box, …). Because both
sides return the same `state()` shape, OmniLink agents, chat, tools and
the HTTP surface are byte-identical between sim and real.

A driver separates *protocol* from *transport*:

  - protocol  = how a command maps to bytes/registers (pure, testable).
  - transport = how those bytes reach the device (serial, TCP, IO).

Transports default to a `DryTransport` that records what *would* have been
sent, so every driver runs and is unit-testable with no hardware attached.
Swap in a real transport (e.g. a pymodbus client) for deployment.
"""

from __future__ import annotations

import abc
import time
from typing import Any, Dict, List, Optional, Tuple


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Transport(abc.ABC):
    """Abstract byte/register sink for a driver. Real deployments inject a
    concrete transport (pymodbus client, pyserial port, GPIO lib)."""

    @abc.abstractmethod
    def write_registers(self, address: int, values: List[int]) -> None: ...

    def read_registers(self, address: int, count: int) -> List[int]:
        return [0] * count

    def set_output(self, channel: int, on: bool) -> None: ...

    def read_input(self, channel: int) -> bool:
        return False

    def close(self) -> None: ...


class DryTransport(Transport):
    """No-hardware transport. Records every operation so drivers run and
    can be asserted on in tests / demos without a device attached."""

    def __init__(self, label: str = "dry") -> None:
        self.label = label
        self.log: List[Tuple[str, Any]] = []
        # Simulated register file so read-after-write echoes sanely.
        self._regs: Dict[int, int] = {}
        self._outputs: Dict[int, bool] = {}

    def write_registers(self, address: int, values: List[int]) -> None:
        for i, v in enumerate(values):
            self._regs[address + i] = int(v) & 0xFFFF
        self.log.append(("write_registers", (address, list(values))))
        print(f"  [{self.label}] write_registers @0x{address:04X} = {values}")

    def read_registers(self, address: int, count: int) -> List[int]:
        out = [self._regs.get(address + i, 0) for i in range(count)]
        self.log.append(("read_registers", (address, count, out)))
        return out

    def set_output(self, channel: int, on: bool) -> None:
        self._outputs[channel] = bool(on)
        self.log.append(("set_output", (channel, bool(on))))
        print(f"  [{self.label}] set_output ch{channel} = {on}")

    def read_input(self, channel: int) -> bool:
        return self._outputs.get(channel, False)

    def close(self) -> None:
        self.log.append(("close", None))


class RealGripperDriver(abc.ABC):
    """Implement the verb methods for one gripper family. Subclasses set
    `kind` / `model` and translate verbs into transport operations.

    The default verb bodies only track width/holding so a minimal driver
    still reports coherent state; concrete drivers override them.
    """

    kind: str = "generic"
    driver_id: str = "generic"

    def __init__(self, *, model: str = "", max_width: float = 0.0,
                 transport: Optional[Transport] = None,
                 **opts: Any) -> None:
        self.model = model or self.driver_id
        self.max_width = float(max_width or 0.0)
        self.transport = transport or DryTransport(self.driver_id)
        self.opts = opts
        self._width = self.max_width
        self._holding = False
        self.fault: Optional[str] = None
        self.connected = False

    # ── lifecycle ───────────────────────────────────────────────────
    def connect(self) -> Dict[str, Any]:
        """Open the bus and activate the gripper. Override for real
        activation handshakes (e.g. Robotiq rACT)."""
        self.connected = True
        return {"connected": True, "model": self.model}

    def disconnect(self) -> None:
        try:
            self.transport.close()
        finally:
            self.connected = False

    # ── verbs (override per family) ─────────────────────────────────
    def open(self) -> Dict[str, Any]:
        return self.set_width(self.max_width) if self.max_width else self._mark(False, self.max_width)

    def close(self) -> Dict[str, Any]:
        return self.set_width(0.0) if self.max_width else self._mark(True, 0.0)

    def set_width(self, width_m: float) -> Dict[str, Any]:
        if self.max_width <= 0.0:
            return {"error": "width_control_unavailable", **self.state()}
        self._width = clamp(float(width_m), 0.0, self.max_width)
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        self.set_width(width if width is not None else 0.0)
        self._holding = True
        return self.state()

    def release(self) -> Dict[str, Any]:
        self.open()
        self._holding = False
        return self.state()

    # ── readback ────────────────────────────────────────────────────
    def object_present(self) -> Optional[bool]:
        return None

    def state(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "model": self.model,
            "width": round(self._width, 4) if self.max_width > 0 else None,
            "max_width": self.max_width or None,
            "holding": self._holding,
            "object_present": self.object_present(),
            "fault": self.fault,
            "connected": self.connected,
            "last_update": time.time(),
        }

    def capabilities(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "model": self.model,
            "has_width_control": self.max_width > 0.0,
            "max_width": self.max_width or None,
        }

    # ── helper ──────────────────────────────────────────────────────
    def _mark(self, holding: bool, width: float) -> Dict[str, Any]:
        self._holding = holding
        self._width = width
        return self.state()
