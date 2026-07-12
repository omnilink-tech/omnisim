#!/usr/bin/env python3
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

"""Canonical schema + loader for GHOST look-up tables (the Shadowing reference format).

A *ghost* is the achievable reference a Shadowing policy tracks. On disk it is a
phase-indexed JSON look-up table under
``projects/policies/controllers/g1_ghost/ghost_*_lut.json``. Historically the
schema was a *de-facto convention*: every ``design_*``/``build_*`` script emits the
same core keys and both the ``g1_ghost`` hologram controller and the recipe trainer
read them with ``.get()``/None-guards -- but nothing was typed or enforced, so
channel widths drifted (``arm_lut`` shoulder-pitch-only vs elbow/waist split), old
cyclic luts dropped ``root_lut``/``wb_lut``, and ``kp_bodies`` sets differed per
motion. This module formalises that convention WITHOUT changing it:

    * ``GhostLut.load(path)``       -- parse + wrap, no mutation.
    * ``lut.validate()``           -- structural issues (errors + warnings), pure, <1ms.
    * ``lut.describe()``           -- one-line human summary (bins, cadence, channels).
    * ``lut.motion_class``         -- "sequence" | "cyclic" | "static" (the semantic axis).

It is deliberately PERMISSIVE: unknown keys are allowed (a warning only in --strict),
optional channels may be absent, and only the load-bearing widths (leg=12, att=2,
root=4, wb==len(wb_joints)) are hard errors. Goal: round-trip every ghost in the repo
today, and give new authoring code ONE typed contract to build against.

This is the *structural* contract only. Physical feasibility (COM-over-support, joint
limits, edit-envelope, symmetry) is the separate, calibrated
``projects/policies/training/ghost_validator.py`` gate -- run BOTH before training.

CLI:
    python ghost_lut.py <lut.json>                 # describe + validate one
    python ghost_lut.py --all                       # round-trip every ghost in the repo
    python ghost_lut.py --all --strict              # + flag unknown keys / soft drift
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# The canonical G1 leg slot order used by ``leg_lut`` (nb x 12): left leg then right,
# each [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]. Documented here so
# authoring code has a single source of truth for the slot layout.
LEG_SLOTS: tuple[str, ...] = (
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
)

# Scalar metadata keys and their expected python type (for a light type check).
_SCALARS: dict[str, tuple[type, ...]] = {
    "nb": (int,),
    "freq": (int, float),
    "cycle_s": (int, float),
    "vx": (int, float),
    "seq": (bool,),
    "hold_end": (bool,),
    "arm_A": (int, float),
    "shroll": (int, float),
    "base_pitch": (int, float),
    "ghost_z": (int, float),
    "ghost_y": (int, float),
    "source": (str,),
}


@dataclass(frozen=True)
class Channel:
    """A per-bin (nb x W) reference channel."""
    name: str
    # Fixed inner width, or None when derived from a companion field (see width_of()).
    width: int | None
    required: bool
    desc: str
    # Name of a companion list field whose length gives the inner width (e.g. wb_joints).
    width_from: str | None = None


# The known per-bin channels. Row count must equal ``nb``. Widths that are load-bearing
# for the controller / trainer are validated as errors; the rest as warnings.
CHANNELS: tuple[Channel, ...] = (
    # leg width is robot-dependent (G1 23dof = 12 slots, H1 = 10) -> validated for
    # internal row-consistency, not a fixed width; slot order LEG_SLOTS for the 12-DOF G1.
    Channel("leg_lut", None, True, "leg joint angles [rad], 12 for G1 / 10 for H1"),
    Channel("arm_lut", 2, False, "L/R shoulder pitch [rad] (absent => synthesised from hip sway)"),
    Channel("att_lut", 2, False, "base roll, pitch [rad]"),
    Channel("elbow_lut", 2, False, "L/R elbow [rad]"),
    Channel("waist_lut", None, False, "waist joint(s) [rad]"),
    Channel("root_lut", 4, False, "base trajectory x,y,z[m], yaw[rad] (drives base pose when seq)"),
    Channel("wb_lut", None, False, "whole-body joint track [rad], width == len(wb_joints)", width_from="wb_joints"),
    Channel("kp_lut", None, False, "keypoint positions [m], width == 3*len(kp_bodies)", width_from="kp_bodies"),
    Channel("lp_lut", None, False, "FK link positions [m], width == 3*len(lp_bodies)", width_from="lp_bodies"),
    Channel("lo_lut", None, False, "FK link orientations, width == k*len(lp_bodies)", width_from="lp_bodies"),
)
_CHAN_BY_NAME = {c.name: c for c in CHANNELS}

# Keys that are lists/metadata companions (not per-bin nb-row channels).
_COMPANION_KEYS = {"wb_joints", "kp_bodies", "lp_bodies", "lp_frame"}

_KNOWN_KEYS = set(_SCALARS) | {c.name for c in CHANNELS} | _COMPANION_KEYS


@dataclass
class Issue:
    level: str   # "error" | "warn"
    msg: str

    def __str__(self) -> str:  # pragma: no cover - formatting only
        tag = "ERROR" if self.level == "error" else "warn "
        return f"[{tag}] {self.msg}"


@dataclass
class GhostLut:
    """A parsed ghost look-up table. ``raw`` is the untouched JSON dict."""
    path: Path
    raw: dict[str, Any]

    # ---- constructors -----------------------------------------------------
    @classmethod
    def load(cls, path: str | Path) -> "GhostLut":
        p = Path(path)
        with open(p, "r", encoding="utf-8") as f:
            raw = json.load(f)
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: top-level JSON must be an object, got {type(raw).__name__}")
        return cls(path=p, raw=raw)

    # ---- typed accessors --------------------------------------------------
    @property
    def nb(self) -> int:
        return int(self.raw.get("nb", 0))

    @property
    def seq(self) -> bool:
        return bool(self.raw.get("seq", False))

    @property
    def hold_end(self) -> bool:
        return bool(self.raw.get("hold_end", False))

    @property
    def vx(self) -> float:
        return float(self.raw.get("vx", 0.0) or 0.0)

    @property
    def source(self) -> str:
        return str(self.raw.get("source", ""))

    @property
    def wb_joints(self) -> list[str]:
        return list(self.raw.get("wb_joints", []) or [])

    def channel(self, name: str) -> list | None:
        """Return a per-bin channel (list of rows) or None if absent."""
        return self.raw.get(name)

    def has(self, name: str) -> bool:
        return name in self.raw and self.raw.get(name) is not None

    @property
    def channels_present(self) -> list[str]:
        return [c.name for c in CHANNELS if self.has(c.name)]

    @property
    def motion_class(self) -> str:
        """The semantic axis the whole stack branches on.

        * ``sequence`` -- non-periodic, root_lut drives the base pose (turns, climbs,
          dances, walk->stop->walk). ``seq: true``.
        * ``static``   -- a held pose (nb small / vx 0 / constant leg track): stands.
        * ``cyclic``   -- a periodic gait glided at ``vx`` beside the robot: walk, carry.
        """
        if self.seq:
            return "sequence"
        # A constant leg track (all rows equal) with ~zero speed is a static hold.
        legs = self.channel("leg_lut")
        if legs and self.vx == 0.0 and _rows_constant(legs):
            return "static"
        return "cyclic"

    # ---- validation -------------------------------------------------------
    def validate(self, strict: bool = False) -> list[Issue]:
        issues: list[Issue] = []
        raw = self.raw

        nb = raw.get("nb")
        if not isinstance(nb, int) or nb <= 0:
            issues.append(Issue("error", f"nb must be a positive int, got {nb!r}"))
            nb = None

        # scalar type checks (soft: wrong-type metadata is a warning)
        for key, types in _SCALARS.items():
            if key in raw and raw[key] is not None and not isinstance(raw[key], types):
                # bool is an int subclass; only flag genuine mismatches
                if not (types == (int, float) and isinstance(raw[key], bool)):
                    issues.append(Issue("warn", f"{key} expected {types}, got {type(raw[key]).__name__}"))

        # at minimum a ghost needs a leg track
        if not self.has("leg_lut"):
            issues.append(Issue("error", "missing required channel 'leg_lut'"))

        # per-bin channel shape checks
        for chan in CHANNELS:
            rows = raw.get(chan.name)
            if rows is None:
                if chan.required:
                    pass  # already reported above for leg_lut
                continue
            if not isinstance(rows, list):
                issues.append(Issue("error", f"{chan.name} must be a list of rows, got {type(rows).__name__}"))
                continue
            if nb is not None and len(rows) != nb:
                issues.append(Issue("error", f"{chan.name} has {len(rows)} rows, expected nb={nb}"))
            # (1) internal consistency: every row must share one width (a mid-channel
            #     width change is real corruption, robot-independent) -> always an error.
            common, inconsistent = _row_widths(rows)
            if inconsistent is not None:
                i, got = inconsistent
                issues.append(Issue("error", f"{chan.name} row {i} width {got} != {common} (rows inconsistent)"))
            # (2) declared width: compare the common width to the expected one.
            w = _width_of(chan, raw)
            if w is not None and common is not None and common != w:
                lvl = "error" if chan.name in ("att_lut", "root_lut", "wb_lut") else "warn"
                issues.append(Issue(lvl, f"{chan.name} width {common}, expected {w}"))
            # (3) leg DOF sanity: G1=12, H1=10; anything else is suspicious (warn only).
            if chan.name == "leg_lut" and common is not None and common not in (10, 12):
                issues.append(Issue("warn", f"leg_lut width {common} is neither 12 (G1) nor 10 (H1)"))

        # wb_lut <-> wb_joints coupling
        if self.has("wb_lut") and not self.wb_joints:
            issues.append(Issue("error", "wb_lut present but wb_joints missing (cannot map columns)"))

        # sequence semantics: a seq ghost should carry a root trajectory
        if self.seq and not self.has("root_lut"):
            issues.append(Issue("warn", "seq=true but no root_lut (base pose will not advance)"))

        if strict:
            for key in raw:
                if key not in _KNOWN_KEYS:
                    issues.append(Issue("warn", f"unknown key '{key}' (not in schema)"))

        return issues

    # ---- summary ----------------------------------------------------------
    def describe(self) -> str:
        raw = self.raw
        bits = [
            f"{self.path.name}",
            f"nb={self.nb}",
            f"class={self.motion_class}",
        ]
        if "cycle_s" in raw:
            bits.append(f"cycle={raw['cycle_s']:.3g}s")
        if "freq" in raw:
            bits.append(f"freq={raw['freq']:.3g}Hz")
        bits.append(f"vx={self.vx:.3g}")
        if self.has("wb_lut"):
            bits.append(f"wb={len(self.wb_joints)}dof")
        chans = ",".join(self.channels_present)
        bits.append(f"channels=[{chans}]")
        if self.source:
            bits.append(f"src={self.source!r}")
        return "  ".join(bits)


# ---- helpers --------------------------------------------------------------
def _width_of(chan: Channel, raw: dict[str, Any]) -> int | None:
    if chan.width is not None:
        return chan.width
    if chan.width_from:
        companion = raw.get(chan.width_from)
        if isinstance(companion, list) and companion:
            n = len(companion)
            # kp/lp positions are 3 per body; wb is 1 per joint; lo is k per body (2..4).
            if chan.name in ("kp_lut", "lp_lut"):
                return 3 * n
            if chan.name == "wb_lut":
                return n
            return None  # lo_lut: k varies (quat/6d); validated loosely below
    return None


def _row_widths(rows: list) -> tuple[int | None, tuple[int, int] | None]:
    """Return (common_width, inconsistency).

    common_width  = the width of row 0 (bare scalar rows count as width 1).
    inconsistency = (index, width) of the first row whose width differs, else None.
    """
    if not rows:
        return None, None

    def _w(r: Any) -> int:
        return len(r) if isinstance(r, (list, tuple)) else 1

    common = _w(rows[0])
    for i, r in enumerate(rows):
        if _w(r) != common:
            return common, (i, _w(r))
    return common, None


def _rows_constant(rows: list, tol: float = 1e-6) -> bool:
    if not rows:
        return False
    first = rows[0]
    if not isinstance(first, (list, tuple)):
        return all(abs(float(r) - float(first)) < tol for r in rows)
    for r in rows:
        if not isinstance(r, (list, tuple)) or len(r) != len(first):
            return False
        for a, b in zip(r, first):
            if abs(float(a) - float(b)) > tol:
                return False
    return True


def _repo_root() -> Path:
    # projects/policies/skills/ghost_lut.py -> repo root is parents[3]
    return Path(__file__).resolve().parents[3]


def _ghost_dir() -> Path:
    return _repo_root() / "projects" / "policies" / "controllers" / "g1_ghost"


# ---- CLI ------------------------------------------------------------------
def _cmd_one(path: Path, strict: bool) -> int:
    try:
        lut = GhostLut.load(path)
    except Exception as e:  # noqa: BLE001 - CLI surface
        print(f"[ERROR] {path}: {e}")
        return 1
    print(lut.describe())
    issues = lut.validate(strict=strict)
    errs = [i for i in issues if i.level == "error"]
    for i in issues:
        print("   ", i)
    return 1 if errs else 0


def _cmd_all(strict: bool) -> int:
    d = _ghost_dir()
    luts = sorted(d.glob("ghost_*_lut.json"))
    if not luts:
        print(f"no ghost luts found under {d}")
        return 1
    n_err = 0
    n_warn = 0
    by_class: dict[str, int] = {}
    for p in luts:
        try:
            lut = GhostLut.load(p)
        except Exception as e:  # noqa: BLE001
            print(f"[ERROR] {p.name}: parse failed: {e}")
            n_err += 1
            continue
        issues = lut.validate(strict=strict)
        errs = [i for i in issues if i.level == "error"]
        warns = [i for i in issues if i.level == "warn"]
        by_class[lut.motion_class] = by_class.get(lut.motion_class, 0) + 1
        if errs or (strict and warns):
            status = "ERR " if errs else "warn"
            print(f"[{status}] {lut.describe()}")
            for i in issues:
                print("        ", i)
        n_err += len(errs)
        n_warn += len(warns)
    print("-" * 72)
    print(f"round-tripped {len(luts)} ghost luts: "
          f"{n_err} errors, {n_warn} warnings; by class {by_class}")
    return 1 if n_err else 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Validate/describe GHOST look-up tables.")
    ap.add_argument("path", nargs="?", help="a single ghost lut json")
    ap.add_argument("--all", action="store_true", help="round-trip every ghost lut in the repo")
    ap.add_argument("--strict", action="store_true", help="also flag unknown keys / soft drift")
    args = ap.parse_args(argv)
    if args.all:
        return _cmd_all(args.strict)
    if not args.path:
        ap.error("give a lut path or --all")
    return _cmd_one(Path(args.path), args.strict)


if __name__ == "__main__":
    sys.exit(main())
