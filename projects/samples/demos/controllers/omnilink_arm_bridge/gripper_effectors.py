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

"""End-effector abstraction for omnilink_arm_bridge.

A gripper is decoupled from the arm: the bridge resolves a gripper config
(see `_gripper_configs.py`) and wraps it in a `GripperEffector`. Every
effector exposes the same surface regardless of hardware family, so the
bridge -- and everything above it (HTTP, OmniLink tools, chat) -- never
branches on gripper type:

    open()              close()
    set_width(meters)   grasp(force=, width=)   release()
    state()  -> {kind, width, holding, object_present, fault}

Sim grasping is modelled as a kinematic attach (see Phase 3): the concrete
`grasp()` here drives the fingers / suction and flips the `holding` flag;
the Supervisor weld that makes a picked Solid follow the TCP is layered on
top by the bridge. `release()` reverses it.

Families
--------
    ParallelFingerGripper  2-finger position-driven (Robotiq 2F-85/2F-140,
                           Panda hand, OnRobot RG2/RG6, Schunk EGK).
    AngularGripper         adaptive 3-finger (Robotiq 3F).
    VacuumEffector         Webots VacuumGripper device (suction).
    MagneticEffector       Connector-node based magnetic coupling.

Missing devices are tolerated (warn, no crash) -- the same way the arm
tolerates a missing joint motor -- so a config can be registered before
its sim assets are wired into a world.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class GripperEffector:
    """Base class. Subclasses override the verb methods.

    The default verb implementations are no-ops that only track the
    `holding` flag, so a brand-new family at least reports state sanely.
    """

    kind = "generic"

    def __init__(self, robot: Any, cfg: Dict[str, Any]) -> None:
        self.robot = robot
        self.cfg = cfg
        self.kind = cfg.get("kind", self.kind)
        self.model = cfg.get("model", self.kind)
        self.max_width = float(cfg.get("max_width", 0.0) or 0.0)
        self.timestep = int(robot.getBasicTimeStep()) if robot is not None else 32
        self._holding = False
        self._width = self.max_width
        self.fault: Optional[str] = None
        # Names from cfg["motors"] that did NOT resolve to a device. Populated
        # by _device(); reported by state() so a fake grasp is visible.
        self.missing_motors: List[str] = []

    # ── device resolution (mirrors the arm's motor lookup) ──────────
    def _device(self, name: str):
        if self.robot is None or not name:
            return None
        dev = self.robot.getDevice(name + "_motor")
        if dev is None:
            dev = self.robot.getDevice(name)
        if dev is None:
            print(f"[gripper:{self.kind}] WARNING: missing device {name!r}")
            self.missing_motors.append(name)
        return dev

    # ── the honesty gate ────────────────────────────────────────────
    def _can_actuate(self) -> bool:
        """Did ANY commanded motor actually resolve to a device?

        ⚠ THIS EXISTS BECAUSE A GRIPPER THAT MOVED NOTHING STILL REPORTED
        `holding: True`. Tolerating a missing device is a deliberate and
        reasonable choice -- this module's own docstring explains it, so a
        config can be registered before its sim assets are wired into a world
        -- but the tolerance leaked into the RESULT: `_apply` skipped every
        `None` motor and `grasp()` then set `_holding = True` regardless. The
        measured consequence was a `robotiq_2f140` grasp that reported success
        while moving nothing at all, because its config named a motor
        (`finger_joint`) that exists in no URDF in this tree.

        A silent fake success is the one outcome this project's tool-design
        doctrine forbids outright, and it is far worse here than a crash: the
        whole point of the 2F-140 demo is that a grasp is proved rather than
        asserted. So: warn and continue at CONSTRUCTION (the tolerance is
        kept), but refuse to CLAIM a hold that no motor could have produced.
        """
        return any(m is not None for m in getattr(self, "motors", []))

    # ── verbs (overridden per family) ───────────────────────────────
    def open(self) -> Dict[str, Any]:
        self._holding = False
        self._width = self.max_width
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._holding = True
        self._width = 0.0
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        if self.max_width <= 0.0:
            return {"error": "width_control_unavailable", **self.state()}
        self._width = _clamp(float(width_m), 0.0, self.max_width)
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        """Close to grasp. The bridge layers the kinematic-attach weld on
        top of this (Phase 3); here we drive the effector + set holding."""
        if width is not None:
            self.set_width(width)
        else:
            self.close()
        self._holding = True
        return self.state()

    def release(self) -> Dict[str, Any]:
        self.open()
        self._holding = False
        return self.state()

    # ── readback ────────────────────────────────────────────────────
    def object_present(self) -> Optional[bool]:
        """True/False if the hardware can sense a held object, else None."""
        return None

    def state(self) -> Dict[str, Any]:
        st = {
            "kind": self.kind,
            "model": self.model,
            "width": round(self._width, 4) if self.max_width > 0 else None,
            "max_width": self.max_width or None,
            "holding": self._holding,
            "object_present": self.object_present(),
            "fault": self.fault,
        }
        # An effector that resolved no motors is inert. Say so on EVERY read,
        # not only when someone thinks to ask, because the caller most likely
        # to be misled is an LLM reading this dict after a grasp.
        if self.missing_motors:
            st["missing_motors"] = list(self.missing_motors)
        if hasattr(self, "motors") and not self._can_actuate():
            st["actuable"] = False
            st["holding"] = False
            st["fault"] = st["fault"] or "no_motors_resolved"
        return st

    def capabilities(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "model": self.model,
            "has_width_control": self.max_width > 0.0,
            "max_width": self.max_width or None,
        }


class ParallelFingerGripper(GripperEffector):
    """Two-finger parallel gripper driven by position-controlled motors.

    `open_q` / `close_q` are per-motor joint targets. Width is a linear
    interpolation between them: width=max_width at open_q, 0 at close_q.
    """

    kind = "parallel"

    def __init__(self, robot: Any, cfg: Dict[str, Any]) -> None:
        super().__init__(robot, cfg)
        self.motors = [self._device(n) for n in (cfg.get("motors") or [])]
        self.open_q: List[float] = list(cfg.get("open_q") or [])
        self.close_q: List[float] = list(cfg.get("close_q") or [])
        # Physics (contact) grasp: stiffen the finger position control so the
        # pinch develops REAL clamping force. A position servo on a part it can
        # never fully close around develops force = P x (target - actual); the
        # URDF-imported default P is small, so the clamp is only a couple of
        # newtons -- friction barely beats gravity and the part slowly creeps
        # out of the grip. A high P saturates the actuator at its effort cap for
        # a firm, non-creeping squeeze (still a real contact grasp, no weld).
        # The available force is the finger's OWN rated effort, read off the motor
        # (getMaxForce() == the URDF <limit effort=...>; the 2F-85 declares 50 N). We used to
        # ask for a flat 60 N, which is ABOVE that: the engine clamped it back to 50 N anyway
        # (OmMotor::setAvailableForceOrTorque) and printed
        #     "The requested available motor force 60 exceeds 'maxForce' = 50"
        # on every launch of every physics-grasp demo. So the 60 bought nothing and only
        # advertised that we were over-driving the gripper. Sourcing the cap from the motor
        # keeps the same real clamp force (50 N) and can never drift from the URDF.
        if cfg.get("physics_grasp"):
            for m in self.motors:
                if m is None:
                    continue
                try:
                    m.setControlPID(5000.0, 0.0, 0.0)
                except Exception:
                    pass
                try:
                    fmax = float(m.getMaxForce())
                except Exception:
                    fmax = 0.0
                try:
                    m.setAvailableForce(fmax if fmax > 0.0 else 50.0)
                except Exception:
                    pass

    def _apply(self, q: List[float]) -> None:
        for m, qi in zip(self.motors, q):
            if m is not None:
                try:
                    m.setPosition(qi)
                except Exception:
                    pass

    def open(self) -> Dict[str, Any]:
        self._apply(self.open_q)
        self._width = self.max_width
        self._holding = False
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._apply(self.close_q)
        self._width = 0.0
        return self.state()

    def set_width(self, width_m: float) -> Dict[str, Any]:
        if self.max_width <= 0.0 or not self.open_q or not self.close_q:
            return {"error": "width_control_unavailable", **self.state()}
        self._width = _clamp(float(width_m), 0.0, self.max_width)
        a = self._width / self.max_width  # 0 = closed, 1 = open
        q = [c + a * (o - c) for c, o in zip(self.close_q, self.open_q)]
        self._apply(q)
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        if not self._can_actuate():
            # Refuse the CLAIM, not the call: the config is allowed to exist
            # before its sim assets do (see _can_actuate), but it may not
            # report a hold it cannot possibly have produced.
            self._holding = False
            return {"error": "no_motors_resolved",
                    "detail": ("gripper %r resolved none of its motors %s -- "
                               "nothing was commanded, so this is NOT a grasp"
                               % (self.model, self.missing_motors)),
                    **self.state()}
        if width is not None:
            self.set_width(width)
        else:
            self.close()
        self._holding = True
        return self.state()


class AngularGripper(ParallelFingerGripper):
    """Adaptive 3-finger gripper (e.g. Robotiq 3F).

    Treated as a position-driven gripper with coupled finger groups. The
    extra fingers just mean `motors`/`open_q`/`close_q` carry more entries;
    named grip *modes* (basic/pinch/wide) are exposed via `set_mode`.
    """

    kind = "angular"

    def __init__(self, robot: Any, cfg: Dict[str, Any]) -> None:
        super().__init__(robot, cfg)
        self.modes: Dict[str, List[float]] = dict(cfg.get("modes") or {})
        self.mode = cfg.get("default_mode", "basic")

    def set_mode(self, mode: str) -> Dict[str, Any]:
        if mode not in self.modes:
            return {"error": f"unknown_mode:{mode}", **self.state()}
        self.mode = mode
        self._apply(self.modes[mode])
        return self.state()

    def state(self) -> Dict[str, Any]:
        st = super().state()
        st["mode"] = self.mode
        return st

    def capabilities(self) -> Dict[str, Any]:
        caps = super().capabilities()
        caps["modes"] = sorted(self.modes.keys()) or ["basic"]
        return caps


class VacuumEffector(GripperEffector):
    """Suction effector backed by a Webots VacuumGripper device.

    open()/release() turn suction OFF; close()/grasp() turn it ON. The
    device's own presence sensor reports whether a part is held.
    """

    kind = "vacuum"

    def __init__(self, robot: Any, cfg: Dict[str, Any]) -> None:
        super().__init__(robot, cfg)
        self.device = self._device(cfg.get("device", "vacuum gripper"))
        if self.device is not None:
            try:
                self.device.enablePresence(self.timestep)
            except Exception:
                pass

    def _set(self, on: bool) -> None:
        if self.device is None:
            return
        try:
            self.device.turnOn() if on else self.device.turnOff()
        except Exception:
            pass

    def open(self) -> Dict[str, Any]:
        self._set(False)
        self._holding = False
        return self.state()

    def close(self) -> Dict[str, Any]:
        self._set(True)
        self._holding = True
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        self._set(True)
        self._holding = True
        return self.state()

    def release(self) -> Dict[str, Any]:
        self._set(False)
        self._holding = False
        return self.state()

    def object_present(self) -> Optional[bool]:
        if self.device is None:
            return None
        try:
            return bool(self.device.getPresence())
        except Exception:
            return None


class MagneticEffector(GripperEffector):
    """Magnetic coupling backed by a Webots Connector device.

    grasp()/close() lock the connector; release()/open() unlock it. Used
    for the assembly-line style pick where a part welds to the tool.
    """

    kind = "magnetic"

    def __init__(self, robot: Any, cfg: Dict[str, Any]) -> None:
        super().__init__(robot, cfg)
        self.device = self._device(cfg.get("device", "connector"))
        if self.device is not None:
            try:
                self.device.enablePresence(self.timestep)
            except Exception:
                pass

    def close(self) -> Dict[str, Any]:
        if self.device is not None:
            try:
                self.device.lock()
            except Exception:
                pass
        self._holding = True
        return self.state()

    def open(self) -> Dict[str, Any]:
        if self.device is not None:
            try:
                self.device.unlock()
            except Exception:
                pass
        self._holding = False
        return self.state()

    def grasp(self, force: Optional[float] = None,
              width: Optional[float] = None) -> Dict[str, Any]:
        return self.close()

    def release(self) -> Dict[str, Any]:
        return self.open()

    def object_present(self) -> Optional[bool]:
        if self.device is None:
            return None
        try:
            return bool(self.device.getPresence())
        except Exception:
            return None


_FAMILIES = {
    "parallel": ParallelFingerGripper,
    "angular": AngularGripper,
    "vacuum": VacuumEffector,
    "magnetic": MagneticEffector,
}


def make_effector(robot: Any, cfg: Dict[str, Any]) -> GripperEffector:
    """Build the effector for a gripper config, dispatching on `kind`."""
    kind = (cfg or {}).get("kind", "parallel")
    klass = _FAMILIES.get(kind, GripperEffector)
    return klass(robot, cfg)
