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

"""Run the UNMODIFIED deterministic quad MPC (quad_mpc_offline.py) on a
foot-redesign variant or the original, on flat OR rough terrain.

Touches NO original file: it imports the offline harness and monkeypatches its
module globals (ROBOTS gets one extra entry, YAW_W gets the per-run heading
weights), then calls its main(). Mirrors foot_redesign/walk_exp.py.

  python projects/policies/research/mpc/quad_morph/walk_exp.py \
      --base spot --foot boxwide --terrain rough --secs 8 [--stance 1.0] [--yaw 0] [--wz 0]

  --foot  orig | boxwide | box | bigsphere   (orig = the unmodified deploy model)
  --terrain flat | rough
  --stance scales the gait lateral foot target (1.0 = stock; >1 widens the stance)
  --yaw/--wz override the MPC heading weights for THIS robot (stock spot=0/0, go2=12/4)
"""
from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODELS = HERE / "models"
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())
sys.path.insert(0, str(REPO))
MJCF = REPO / "projects/policies/research/training/mjcf"

# base gait kwargs matching the deploy launcher (same as quad_mpc_offline.ROBOTS)
BASE = {
    "spot": ("projects.policies.control.gait.spot_trot_gait",
             dict(vx=0.4, freq=1.4, duty=0.6, step_height=0.06, body_height=0.55), 0.344),
    "go2":  ("projects.policies.control.gait.go2_trot_gait",
             dict(vx=0.4, freq=1.8, duty=0.6, step_height=0.05, body_height=0.30), 0.20),
    "b2":   ("projects.policies.control.gait.b2_trot_gait",
             dict(vx=0.5, freq=1.3, duty=0.6, step_height=0.08, body_height=0.50), 0.40),
}


def mjcf_path(base, foot, terrain):
    if foot == "orig":
        if terrain == "rough":
            return MJCF / {"spot": "spot_newton_fixed2_rough.xml"}[base]
        return MJCF / {"spot": "spot_newton_fixed2.xml",
                       "go2": "go2_newton.xml", "b2": "b2_newton.xml"}[base]
    return MODELS / f"{base}_{foot}_{terrain}.xml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", choices=list(BASE), default="spot")
    ap.add_argument("--foot", default="boxwide")
    ap.add_argument("--terrain", choices=["flat", "rough"], default="flat")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--stance", type=float, default=1.0)
    ap.add_argument("--yaw", type=float, default=None)
    ap.add_argument("--wz", type=float, default=None)
    ap.add_argument("--K", type=int, default=96)
    ap.add_argument("--H", type=int, default=24)
    args = ap.parse_args()

    mjcf = mjcf_path(args.base, args.foot, args.terrain)
    if not Path(mjcf).exists():
        sys.exit(f"missing model {mjcf} -- run make_models.py {args.base}")

    mpc = importlib.import_module("projects.policies.research.mpc.quad_mpc_offline")
    gait_mod, gkw, lat0 = BASE[args.base]
    gkw = dict(gkw)
    if args.stance != 1.0:
        gkw["lateral_y"] = round(lat0 * args.stance, 5)
    key = f"_exp_{args.base}_{args.foot}_{args.terrain}"
    mpc.ROBOTS[key] = (str(mjcf), gait_mod, gkw)
    yw = mpc.YAW_W.get(args.base, (0.0, 0.0))
    mpc.YAW_W[key] = (yw[0] if args.yaw is None else args.yaw,
                      yw[1] if args.wz is None else args.wz)

    print(f"[exp] base={args.base} foot={args.foot} terrain={args.terrain} "
          f"stance={args.stance} yaw={mpc.YAW_W[key]} mjcf={Path(mjcf).name}")
    sys.argv = ["quad_mpc_offline", "--robot", key, "--secs", str(args.secs),
                "--K", str(args.K), "--H", str(args.H)]
    mpc.main()


if __name__ == "__main__":
    main()
