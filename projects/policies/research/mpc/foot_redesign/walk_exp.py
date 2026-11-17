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

"""Foot-redesign experiment runner.

Thin wrapper over projects/policies/research/mpc/humanoid_walk_offline.py: it registers
the modified-foot model COPIES (./models/, made by make_models.py) as extra --robot
choices, then calls the harness's own main() unchanged. No harness logic is duplicated,
and the original robot models / harness are not modified.

Variants:
  g1_orig   g1_long   g1_long_strong   g1_big     (G1: short foot -> longer/wider/stronger)
  h1_orig   h1_wide   h1_wide_xl                   (H1: narrow foot -> wider)

Usage (same flags as the harness):
  python -u projects/policies/research/mpc/foot_redesign/walk_exp.py --robot g1_long --secs 8
"""
from __future__ import annotations
import importlib
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())
sys.path.insert(0, str(REPO))

M = "projects/policies/research/mpc/foot_redesign/models"
G1 = "projects.policies.control.gait.g1_human_gait"
H1 = "projects.policies.control.gait.h1_human_gait"

mod = importlib.import_module("projects.policies.research.mpc.humanoid_walk_offline")
mod.ROBOTS.update({
    "g1_orig":        dict(gait=G1, legs=f"{M}/g1_orig_legs.mjcf.xml",          full=f"{M}/g1_orig_full.mjcf.xml"),
    "g1_long":        dict(gait=G1, legs=f"{M}/g1_longfoot_legs.mjcf.xml",      full=f"{M}/g1_orig_full.mjcf.xml"),
    "g1_long_strong": dict(gait=G1, legs=f"{M}/g1_longfoot_strong_legs.mjcf.xml", full=f"{M}/g1_orig_full.mjcf.xml"),
    "g1_big":         dict(gait=G1, legs=f"{M}/g1_bigfoot_legs.mjcf.xml",       full=f"{M}/g1_bigfoot_full.mjcf.xml"),
    "h1_orig":        dict(gait=H1, legs=f"{M}/h1_orig.mjcf.xml",               full=f"{M}/h1_orig.mjcf.xml"),
    "h1_wide":        dict(gait=H1, legs=f"{M}/h1_widefoot.mjcf.xml",           full=f"{M}/h1_widefoot.mjcf.xml"),
    "h1_wide_xl":     dict(gait=H1, legs=f"{M}/h1_widefoot_xl.mjcf.xml",        full=f"{M}/h1_widefoot_xl.mjcf.xml"),
})

if __name__ == "__main__":
    mod.main()
