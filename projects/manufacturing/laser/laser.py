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

"""Laser process: GRBL's laser-mode output, the laser module, materials and the kerf model.

The OUTPUT follows GRBL 1.1 ($32=1), from its laser-mode docs and source
(stepper.c, spindle_control.c, config.h):

- G0 never lights the laser; G1/G2/G3 do, at S.
- M3 (constant power) holds its power "regardless if the machine is moving,
  accelerating, or stopped": it stays lit when the buffer runs dry after a G1, and a
  bare `M3 S255` at rest lights it. That is why lasers burn holes in M3.
- M4 (dynamic power) scales S by current speed / programmed feed and is off at rest.
- S to PWM is spindle_compute_pwm_value: 0 is off, at or above $30 full, at or
  below $31 the minimum PWM, linear between, on an 8-bit PWM. The spindle override
  scales S first.
- A completed feed hold turns the laser off (config.h DISABLE_LASER_DURING_HOLD,
  enabled by default).

The KERF model is the engine's own (MillingStock toolShape "laser"), in one dimension
across a straight cut: every pass lays the line energy E = P / v (J/mm) as a Gaussian
across the cut, of 1/e^2 radius w at the surface under the beam's centre,

    F(y) = E sqrt(2/pi) / w exp(-2 y^2 / w^2)        J/mm^2
    depth removed = F / e_v                          e_v: the material's ablation energy
    w = sqrt(w0^2 + (tan(theta) dz)^2)               dz: that surface's distance from focus

so a kerf deepens until it falls out of focus. `e_v` is EFFECTIVE: it lumps absorption,
conduction, the plume and wall losses into one number fitted to the vendor's table, which
is why it is several times wood's bulk decomposition energy. It is a calibration, not a
property, and one test cut on the real machine replaces it.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

PWM_MAX = 255            # GRBL's 8-bit spindle PWM (SPINDLE_PWM_MAX_VALUE)
PWM_MIN = 1              # SPINDLE_PWM_MIN_VALUE: the lowest PWM a non-zero S gives


@dataclass(frozen=True)
class LaserModule:
    name: str
    optical_w: float
    lens_efficiency: float
    spot_mm: Tuple[float, float]          # 1/e^2 diameters at focus along machine X, Y
    beam_half_angle_rad: float
    grbl: Dict[int, float]
    source: str = ""

    @property
    def power_w(self) -> float:
        """Power delivered to the work at full PWM."""
        return self.optical_w * self.lens_efficiency

    @classmethod
    def load(cls, path: Path) -> "LaserModule":
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(d["name"], float(d["optical_w"]), float(d.get("lens_efficiency", 1.0)),
                   (float(d["spot_mm"][0]), float(d["spot_mm"][1])), float(d["beam_half_angle_rad"]),
                   {int(k): float(v) for k, v in d.get("grbl", {}).items()}, str(path))


@dataclass(frozen=True)
class Material:
    name: str
    ablation_j_mm3: float                 # effective e_v (see the module docstring)
    char_j_mm2: float                     # absorbed fluence that chars 63 % of the way
    color: Tuple[float, float, float]
    char_color: Tuple[float, float, float]
    calibration: str


SIENCI_TABLE = "https://resources.sienci.com/view/lb-material-guide/"

MATERIALS: Dict[str, Material] = {m.name: m for m in (
    # Fitted to ONE row of Sienci's LaserBeam table (3 mm plywood: 100 %, 2.2 mm/s, 3 passes,
    # Z step 1 mm): with the module's spot and cone, exactly 3 passes cut through on both
    # spot axes for e_v in [15, 26] J/mm^3; 20 is its middle. Checked, not fitted, with
    # `kerf_check`, on both spot axes: 6 mm (2.2 mm/s, Z 1.2 mm) needs 5 passes (Sienci: 5)
    # and 1/4" (5 mm/s, Z 0.4 mm) 14 (Sienci: 16).
    # char_j_mm2 is a LOOK, not a measurement: Sienci's engrave setting (100 %, 25.4 mm/s)
    # at a 0.1 mm line interval lays 2.6 J/mm^2, which it makes a dark brown (90 % char).
    Material("plywood", 20.0, 1.13, (0.84, 0.70, 0.50), (0.09, 0.06, 0.04),
             f"Sienci LaserBeam 7W material guide, 3 mm plywood cut row ({SIENCI_TABLE})"),
)}


def pwm_value(s: float, s_min: float, s_max: float, override_pct: float = 100.0) -> int:
    """GRBL 1.1 spindle_compute_pwm_value (non-piecewise): S -> 0..255."""
    rpm = s * 0.01 * override_pct
    if s_max <= s_min:
        return PWM_MAX if rpm > 0 else 0
    if rpm >= s_max:
        return PWM_MAX
    if rpm <= 0.0:
        return 0
    if rpm <= s_min:
        return PWM_MIN
    gradient = (PWM_MAX - PWM_MIN) / (s_max - s_min)
    return int(math.floor((rpm - s_min) * gradient)) + PWM_MIN


class LaserOutput:
    """What GRBL's laser pin does, sub-step by sub-step, for one controller.

    `substep(h)` is hooked to the planner (Planner.on_substep) and accumulates the energy
    the beam delivers; `take()` returns and clears it. `on_sync` must see every spindle /
    end block the planner executes (M3/M4/M5 and S changes at rest).
    """

    def __init__(self, ctl, module: LaserModule):
        self.ctl = ctl
        self.module = module
        self.pwm = 0
        self.energy_j = 0.0
        self.lit_s = 0.0
        self.total_j = 0.0

    def _settings(self) -> Tuple[float, float]:
        s = self.ctl.machine.settings
        return float(s[31]), float(s[30])

    def on_sync(self, block) -> None:
        state, speed = block.spindle
        if block.kind == "end" or state == "off":
            self.pwm = 0
        elif state == "cw":     # M3 lights at rest: spindle_set_state sets the PWM at once
            lo, hi = self._settings()
            self.pwm = pwm_value(speed, lo, hi, 100.0 * self.ctl.spindle_override)
        else:                   # M4 is off until the machine moves
            self.pwm = 0

    def _sample(self) -> None:
        ctl, planner = self.ctl, self.ctl.planner
        if not bool(ctl.machine.settings[32]) or ctl.state == "Alarm" or ctl.spindle[0] == "off":
            self.pwm = 0
            return
        if planner.holding and not planner.moving():
            self.pwm = 0          # DISABLE_LASER_DURING_HOLD: the hold completed
            return
        block = planner.current_block()
        if block is None or block.kind != "line":
            if ctl.spindle[0] == "ccw":
                self.pwm = 0      # M4 at rest: is_pwm_rate_adjusted blocks switch off
            return                # M3 keeps its last PWM
        state, speed = block.spindle
        if block.rapid or state == "off" or speed <= 0:
            self.pwm = 0          # G0 carries S0 in laser mode
            return
        lo, hi = self._settings()
        rpm = speed
        if state == "ccw" and block.feed > 0:
            rpm *= planner.velocity / (block.feed / 60.0)   # current_speed * inv_rate
        self.pwm = pwm_value(rpm, lo, hi, 100.0 * ctl.spindle_override)

    def substep(self, h: float) -> None:
        self._sample()
        if self.pwm > 0:
            e = self.module.power_w * self.pwm / PWM_MAX * h
            self.energy_j += e
            self.total_j += e
            self.lit_s += h

    def take(self) -> float:
        e, self.energy_j = self.energy_j, 0.0
        return e


# ── kerf model ────────────────────────────────────────────────────────────────

@dataclass
class Pass:
    power_w: float            # at the work
    speed_mm_s: float
    focus_mm: float           # focal point height above the sheet's bottom


@dataclass
class KerfResult:
    passes_run: int
    through_at: Optional[int]          # first pass that cut through, 1-based
    depth_mm: List[float] = field(default_factory=list)       # centre depth after each pass
    top_width_mm: float = 0.0          # width where the cut is deeper than 0.1 mm
    bottom_width_mm: float = 0.0       # width of the through gap (0 if not through)


def kerf(thickness_mm: float, passes: Sequence[Pass], material: Material, spot_mm: float,
         beam_half_angle_rad: float, res_mm: float = 0.002, half_width_mm: float = 1.5,
         slices: int = 24) -> KerfResult:
    """The cross-section of a straight cut, pass by pass. `spot_mm` is the spot's 1/e^2
    diameter ACROSS the cut.

    A point on the line is burned while the spot sweeps over it, so the surface under the
    beam sinks DURING a pass and the rest of the pass meets it further from focus. Each
    pass is laid in `slices` equal parts with the surface updated between them; the
    engine does the same thing by burning along the path step by step."""
    n = int(round(half_width_mm / res_mm))
    ys = [(k - n) * res_mm for k in range(2 * n + 1)]
    h = [thickness_mm] * len(ys)
    w0 = 0.5 * spot_mm
    tan = math.tan(beam_half_angle_rad)
    out = KerfResult(0, None)
    for idx, p in enumerate(passes):
        line_j_mm = p.power_w / p.speed_mm_s / slices
        for _ in range(slices):
            spread = tan * (h[n] - p.focus_mm)
            w = math.sqrt(w0 * w0 + spread * spread)
            peak = line_j_mm * math.sqrt(2.0 / math.pi) / w / material.ablation_j_mm3
            for k, y in enumerate(ys):
                if h[k] > 0.0:
                    h[k] = max(0.0, h[k] - peak * math.exp(-2.0 * y * y / (w * w)))
        out.passes_run = idx + 1
        out.depth_mm.append(thickness_mm - h[n])
        if out.through_at is None and h[n] <= 0.0:
            out.through_at = idx + 1
    out.top_width_mm = res_mm * sum(1 for v in h if thickness_mm - v > 0.1)
    out.bottom_width_mm = res_mm * sum(1 for v in h if v <= 0.0)
    return out


def table_passes(thickness_mm: float, power_w: float, speed_mm_s: float, z_step_mm: float,
                 count: int, focus_start_mm: Optional[float] = None) -> List[Pass]:
    """Passes as a vendor table states them: focus on the top, lowered z_step each pass."""
    top = thickness_mm if focus_start_mm is None else focus_start_mm
    return [Pass(power_w, speed_mm_s, top - k * z_step_mm) for k in range(count)]


# Sienci's table rows the plywood calibration is judged on: (name, thickness, mm/s, power
# fraction, passes Sienci gives, Z step).
SIENCI_PLYWOOD = (("3 mm plywood", 3.0, 2.2, 1.0, 3, 1.0),
                  ("6 mm plywood", 6.0, 2.2, 1.0, 5, 1.2),
                  ('1/4" plywood', 6.35, 5.0, 1.0, 16, 0.4))


def kerf_check(module: LaserModule, material: Material = MATERIALS["plywood"], max_passes: int = 40) -> List[dict]:
    """Each Sienci row: how many passes the model needs on each spot axis."""
    rows = []
    for name, t, v, frac, passes, z in SIENCI_PLYWOOD:
        row = {"row": name, "sienci_passes": passes}
        for axis, spot in (("wide", max(module.spot_mm)), ("narrow", min(module.spot_mm))):
            r = kerf(t, table_passes(t, module.power_w * frac, v, z, max_passes), material, spot,
                     module.beam_half_angle_rad)
            row[f"model_passes_{axis}"] = r.through_at
        rows.append(row)
    return rows


if __name__ == "__main__":
    import sys
    here = Path(__file__).resolve().parent
    mod = LaserModule.load(Path(sys.argv[1]) if len(sys.argv) > 1 else
                           here.parent / "machines/longmill_mk2_30x30/sienci_laserbeam_7w.json")
    print(json.dumps(kerf_check(mod), indent=1))
