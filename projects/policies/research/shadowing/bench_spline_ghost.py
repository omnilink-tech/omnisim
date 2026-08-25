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

"""A/B benchmark: spline-parameterized + smoothness-cost MPPI vs the vanilla per-timestep
sampler, on the G1 hold-squat ghost (same intent as ghost_generator._demo).

Demonstrates the OPT-IN upgrades (n_knots=..., weights={"smooth":...}) produce a SMOOTHER
reference (lower control rate + jerk) at EQUAL or FEWER samples, without losing balance.

The vanilla path (n_knots=None, smooth=0) is byte-identical to the old generator, so this is
a pure A/B of the new opt-in features. Run:
    python projects/policies/research/shadowing/bench_spline_ghost.py
"""
from __future__ import annotations

import os
import time
import numpy as np
import mujoco

from ghost_generator import GhostGenerator, Intent

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
MJCF = os.path.join(REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")

HOLD = {"left_hip_pitch_joint": -0.30, "right_hip_pitch_joint": -0.30,
        "left_knee_joint": 0.60, "right_knee_joint": 0.60,
        "left_ankle_pitch_joint": -0.30, "right_ankle_pitch_joint": -0.30}


def make_intent(weights=None):
    return Intent(total_time=2.0,
                  joint_keys=[(0.0, HOLD), (2.0, HOLD)],
                  base_keys=[(0.0, {"z": 0.74, "pitch": 0.0}), (2.0, {"z": 0.74, "pitch": 0.0})],
                  weights=weights)


def metrics(g):
    """Reference-quality metrics from a generated ghost dict."""
    c = g["ctrl"]                                   # (T, n_pos) applied position targets
    rate = float(np.mean(np.linalg.norm(np.diff(c, axis=0), axis=1)))        # control velocity
    jerk = float(np.mean(np.linalg.norm(np.diff(c, axis=0, n=2), axis=1)))   # control accel (jerk proxy)
    base = g["base"]
    z_final = float(base[-1, 2])
    qx, qy = base[-1, 4], base[-1, 5]
    tilt = float(np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (qx ** 2 + qy ** 2))))))
    tilts = np.degrees(np.arccos(np.clip(1 - 2 * (base[:, 4] ** 2 + base[:, 5] ** 2), -1, 1)))
    return dict(rate=rate, jerk=jerk, z_final=z_final, tilt_final=tilt, tilt_max=float(tilts.max()))


def run(label, n_samples, n_knots, smooth, seed=0):
    gen = GhostGenerator(MJCF)
    mujoco.mj_resetData(gen.m, gen.d)
    init = gen.d.qpos.copy()
    init[2] = 0.78
    intent = make_intent(weights={"smooth": smooth} if smooth else None)
    t0 = time.time()
    g = gen.generate(intent, init, n_samples=n_samples, horizon=12, n_knots=n_knots,
                     seed=seed, verbose=False)
    dt = time.time() - t0
    m = metrics(g)
    m["secs"] = dt
    m["label"] = label
    m["budget"] = n_samples
    return m


def main():
    configs = [
        ("A vanilla         (K=48)", 48, None, 0.0),
        ("B spline+smooth   (K=48, knots=5)", 48, 5, 4.0),
        ("C spline+smooth   (K=24, knots=5)", 24, 5, 4.0),
    ]
    rows = []
    for label, K, knots, smo in configs:
        print(f"running: {label} ...", flush=True)
        rows.append(run(label, K, knots, smo))

    print("\n" + "=" * 92)
    print(f"{'config':<36}{'K':>4}{'ctrl_rate':>11}{'jerk':>9}{'z_fin':>8}{'tilt_f':>8}{'tilt_max':>9}{'secs':>7}")
    print("-" * 92)
    for m in rows:
        print(f"{m['label']:<36}{m['budget']:>4}{m['rate']:>11.4f}{m['jerk']:>9.4f}"
              f"{m['z_final']:>8.3f}{m['tilt_final']:>8.2f}{m['tilt_max']:>9.2f}{m['secs']:>7.1f}")
    print("=" * 92)

    a, b, c = rows
    print(f"\nB vs A  (same K=48): ctrl_rate {b['rate']/a['rate']*100:.0f}% of vanilla, "
          f"jerk {b['jerk']/a['jerk']*100:.0f}% of vanilla")
    print(f"C vs A  (HALF K=24): ctrl_rate {c['rate']/a['rate']*100:.0f}% of vanilla, "
          f"jerk {c['jerk']/a['jerk']*100:.0f}% of vanilla")
    print("\n(lower ctrl_rate/jerk = smoother reference; tilt_max should stay bounded = still balanced)")


if __name__ == "__main__":
    main()
