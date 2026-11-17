"""Frozen, unit-explicit contract for the first braking assessment."""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math


@dataclass(frozen=True)
class BrakingSpec:
    schema_version: int = 1
    robot: str = "husky"
    target_speed_m_s: float = 0.6
    brake_delay_s: float = 0.0
    max_stop_distance_m: float = 0.25
    max_stop_time_s: float = 1.5
    settled_speed_m_s: float = 0.025
    settled_hold_s: float = 0.4
    settle_s: float = 1.0
    min_drive_distance_m: float = 0.3
    speed_hold_s: float = 0.3
    prepare_timeout_s: float = 6.0
    observe_after_request_s: float = 2.5
    timestep_ms: int = 16
    slope_deg: float = 0.0
    ground_mu: float = 0.8
    payload_kg: float = 0.0
    payload_height_m: float = 0.35
    payload_offset_x_m: float = 0.0

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version not in (1, 2):
            raise ValueError("Supported assessment schemas are 1 and 2")
        if not isinstance(self.robot, str) or not self.robot:
            raise ValueError("robot must be a registered robot id")
        if self.schema_version == 1 and (self.robot != "husky" or self.slope_deg != 0 or
                self.ground_mu != .8 or self.payload_kg != 0):
            raise ValueError("Schema 1 is the original flat unloaded Husky fixture; use schema 2")
        if type(self.timestep_ms) is not int or self.timestep_ms != 16:
            raise ValueError("This qualified world requires timestep_ms=16")
        for f in fields(self):
            if f.name in ("robot", "schema_version", "timestep_ms"):
                continue
            v = getattr(self, f.name)
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v):
                raise ValueError(f"{f.name} must be finite numeric")
            if f.name in ("slope_deg", "payload_offset_x_m"):
                continue
            if v < 0 or (v == 0 and f.name not in ("brake_delay_s", "payload_kg")):
                raise ValueError(f"{f.name} must be positive (delay may be zero)")
        if not 0.1 <= self.target_speed_m_s <= 0.8:
            raise ValueError("target_speed_m_s must be within the experimental range [0.1, 0.8]")
        if self.brake_delay_s > 1.5:
            raise ValueError("brake_delay_s exceeds the first slice's 1.5 s limit")
        if abs(self.slope_deg) > 35 or not .01 <= self.ground_mu <= 2:
            raise ValueError("Experimental range: slope +/-35 degrees and friction [0.01, 2]")
        if self.payload_kg > 100 or not .05 <= self.payload_height_m <= 1 or abs(self.payload_offset_x_m) > 1:
            raise ValueError("Payload outside experimental mass/position range")
        if self.max_stop_time_s + self.settled_hold_s > self.observe_after_request_s:
            raise ValueError("Observation must cover the stop deadline plus settled hold")
        if self.brake_delay_s + self.settled_hold_s >= self.observe_after_request_s:
            raise ValueError("Observation does not cover delayed braking and settled hold")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("Assessment spec must be an object")
        unknown = set(data) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown spec fields: {sorted(unknown)}")
        return cls(**data)
