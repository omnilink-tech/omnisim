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

"""60-second sustained-walk verification harness for the promoted policy.
Uses pick_best_omnisim.run_one_deploy (the same plumbing that produced the
selection result) to launch Webots ODE headlessly, run for 60 seconds, then
parse the body-trace CSV and report metrics."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pick_best_omnisim import run_one_deploy

POLICY = Path("projects/policies/research/inference/policies/spot_omnisim_main/policy.onnx").resolve()

if __name__ == "__main__":
    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    print(f"[verify] {duration_s:.0f}s OmniSim deploy  policy={POLICY.name}  vx_cmd={vx}")
    m = run_one_deploy(POLICY, duration_s, vx)
    if m.get("error"):
        print(f"  ERROR: {m['error']}")
        sys.exit(1)
    print()
    print("=== SUSTAINED OMNISIM RESULT ===")
    print(f"  samples           : {m['n']}")
    print(f"  sim duration      : {m['duration']:.1f} s")
    print(f"  forward distance  : {m['forward_distance']:+.3f} m")
    print(f"  lateral drift     : {m['lateral_drift']:+.3f} m")
    print(f"  path length       : {m['path_length']:.3f} m")
    print(f"  yaw drift         : {m['yaw_drift_rad']:+.3f} rad "
          f"({m['yaw_drift_rad'] * 57.2958:+.1f} deg)")
    print(f"  mean speed        : {m['mean_speed']:.3f} m/s")
    print(f"  mean vx           : {m['mean_vx']:+.3f} m/s")
    print(f"  upright pct       : {m['upright_pct']:.1f}%")
    print(f"  min body z        : {m['min_bz']:.3f} m")
    print(f"  time to fall      : {m['time_to_fall']}  "
          f"(None = never fell)")
