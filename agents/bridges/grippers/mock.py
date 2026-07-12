# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""MockGripperDriver -- prints what it would do, holds no hardware.

The gripper counterpart of MockArmDriver. Use it to bring a real-arm
bridge up end-to-end (chat -> tools -> HTTP -> driver) before the actual
gripper SDK is wired, and as the default when no `--gripper` is given.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .driver_base import RealGripperDriver, clamp


class MockGripperDriver(RealGripperDriver):
    kind = "mock"
    driver_id = "mock"

    def __init__(self, *, model: str = "MockGripper", max_width: float = 0.08,
                 **opts: Any) -> None:
        super().__init__(model=model, max_width=max_width, **opts)

    def open(self) -> Dict[str, Any]:
        print("  [MOCK_GRIPPER] open")
        self._width = self.max_width
        self._holding = False
        return self.state()

    def close(self) -> Dict[str, Any]:
        print("  [MOCK_GRIPPER] close")
        self._width = 0.0
        self._holding = True
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        self._width = clamp(float(width_m), 0.0, self.max_width)
        print(f"  [MOCK_GRIPPER] set_width -> {self._width * 1000:.1f} mm")
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        print(f"  [MOCK_GRIPPER] grasp force={force} width={width}")
        self.set_width(width if width is not None else 0.0)
        self._holding = True
        return self.state()

    def release(self) -> Dict[str, Any]:
        print("  [MOCK_GRIPPER] release")
        return self.open()

    def object_present(self) -> Optional[bool]:
        return self._holding
