#!/usr/bin/env bash
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

# OmniQuad probe runner. Launches OmniSim headless with OMNIQUAD_PROBE=<mode> so
# the controller self-quits cleanly, then analyzes the CSV trace.
#
# Usage: scripts/dev/omniquad_walk_iter.sh <mode> [sim_seconds] [wall_timeout]
#   mode  : hold | lift_fr | sweep | walk
#   sim_s : default 10
set -uo pipefail

MODE="${1:-walk}"
SIM_DURATION="${2:-10}"
WALL_TIMEOUT="${3:-60}"

WORLD="projects/robots/omnisim/omniquad/worlds/omniquad.omniworld"
TRACE_DIR="C:/tmp/husky_trace"
CSV="${TRACE_DIR}/omniquad_probe_${MODE}.csv"
WB="./msys64/mingw64/bin/omnisim-bin.exe"

rm -f "$CSV"
mkdir -p "$TRACE_DIR"

OMNIQUAD_PROBE="$MODE" OMNIQUAD_SIM_DURATION_S="$SIM_DURATION" OMNISIM_HOME="$(pwd)" \
  "$WB" "$WORLD" --batch --mode=fast --no-rendering --minimize \
  --stdout --stderr > /tmp/omniquad_iter_stdout.txt 2>&1 &
WB_PID=$!
echo "[iter] mode=${MODE} sim=${SIM_DURATION}s bash_pid=$WB_PID"

ELAPSED=0
while [ $ELAPSED -lt $WALL_TIMEOUT ]; do
  if ! kill -0 $WB_PID 2>/dev/null; then
    break
  fi
  sleep 1
  ELAPSED=$((ELAPSED+1))
done

if kill -0 $WB_PID 2>/dev/null; then
  powershell.exe -Command "Get-Process omnisim-bin -ErrorAction SilentlyContinue | Stop-Process -Force" >/dev/null 2>&1
  sleep 1
fi

if [ ! -f "$CSV" ]; then
  echo "[iter] NO CSV PRODUCED -- check /tmp/omniquad_iter_stdout.txt"
  tail -20 /tmp/omniquad_iter_stdout.txt 2>/dev/null
  exit 1
fi

# Summary via Python.
python - "$CSV" "$MODE" <<'PY'
import csv, sys, math, statistics
path = sys.argv[1]
mode = sys.argv[2]

with open(path) as f:
    rdr = csv.DictReader(f)
    rows = list(rdr)

if not rows:
    print("(no rows)")
    sys.exit(0)

def F(s):
    try: return float(s)
    except: return 0.0

print(f"=== PROBE: {mode}, {len(rows)} samples, sim={F(rows[-1]['t_ms'])/1000:.2f}s ===")

# Phase transitions
phases = []
last_ph = None
for r in rows:
    ph = r['phase']
    if ph != last_ph:
        phases.append((F(r['t_ms']), ph))
        last_ph = ph
print("Phases:")
for t, p in phases:
    print(f"  t={t:6.0f}ms  {p}")

# Find rows of the "action" phase (= probe mode)
action_rows = [r for r in rows if r['phase'] == mode]
if not action_rows:
    print(f"(no rows in '{mode}' phase)")
    sys.exit(0)

# Equilibrium body position (last 1s of data)
last_s = action_rows[-25:]  # last ~1.25s at 50ms cadence
bz_eq = statistics.mean(F(r['bz']) for r in last_s)
bx_eq = statistics.mean(F(r['bx']) for r in last_s)
by_eq = statistics.mean(F(r['by']) for r in last_s)
roll_eq = statistics.mean(F(r['roll']) for r in last_s)
pitch_eq = statistics.mean(F(r['pitch']) for r in last_s)
yaw_eq = statistics.mean(F(r['yaw']) for r in last_s)
print(f"\nBody equilibrium (last 1.25s of {mode}):")
print(f"  pos=({bx_eq:+.3f}, {by_eq:+.3f}, {bz_eq:+.3f})")
print(f"  rpy=({roll_eq:+.3f}, {pitch_eq:+.3f}, {yaw_eq:+.3f}) rad")
print(f"      ({math.degrees(roll_eq):+.1f}, {math.degrees(pitch_eq):+.1f}, {math.degrees(yaw_eq):+.1f}) deg")

# Foot z (lower_leg world position, which is at the knee — foot tip is below)
print(f"\nLower-leg world positions (= knee joint world, foot is ~0.30m below):")
for leg_short, leg_name in (("fl", "front_left"), ("fr", "front_right"),
                            ("rl", "rear_left"), ("rr", "rear_right")):
    fz_vals = [F(r[f'{leg_short}_fz']) for r in last_s]
    fx_vals = [F(r[f'{leg_short}_fx']) for r in last_s]
    fy_vals = [F(r[f'{leg_short}_fy']) for r in last_s]
    print(f"  {leg_name:14s}  ({statistics.mean(fx_vals):+.3f}, "
          f"{statistics.mean(fy_vals):+.3f}, {statistics.mean(fz_vals):+.3f})")

# Joint sag (cmd vs actual) per leg per joint at equilibrium
print(f"\nJoint sag (cmd vs actual, last 1.25s):")
print(f"  {'leg':14s} {'hip_x_cmd':>10s} {'hip_x_act':>10s} {'hip_y_cmd':>10s} {'hip_y_act':>10s} {'knee_cmd':>10s} {'knee_act':>10s}")
for leg_short, leg_name in (("fl", "front_left"), ("fr", "front_right"),
                            ("rl", "rear_left"), ("rr", "rear_right")):
    def mean_col(c): return statistics.mean(F(r[c]) for r in last_s)
    print(f"  {leg_name:14s} {mean_col(leg_short+'_hxc'):+10.3f} {mean_col(leg_short+'_hxa'):+10.3f} "
          f"{mean_col(leg_short+'_hyc'):+10.3f} {mean_col(leg_short+'_hya'):+10.3f} "
          f"{mean_col(leg_short+'_knc'):+10.3f} {mean_col(leg_short+'_kna'):+10.3f}")

# Velocity
vx_eq = statistics.mean(F(r['vx']) for r in last_s)
vy_eq = statistics.mean(F(r['vy']) for r in last_s)
vz_eq = statistics.mean(F(r['vz']) for r in last_s)
print(f"\nBody velocity (mean over last 1.25s): vx={vx_eq:+.3f} vy={vy_eq:+.3f} vz={vz_eq:+.3f} m/s")

# For mode-specific analysis:
if mode == "lift_fr":
    # When FR knee is at its deepest cmd, what's the FR foot z relative to standing?
    deep_rows = [r for r in action_rows if abs(F(r['fr_knc']) - (-1.30)) < 0.05]
    standing_rows = [r for r in action_rows if abs(F(r['fr_knc']) - (-0.60)) < 0.02]
    if deep_rows and standing_rows:
        deep_fz = statistics.mean(F(r['fr_fz']) for r in deep_rows[-20:])
        stand_fz = statistics.mean(F(r['fr_fz']) for r in standing_rows[-20:])
        lift = deep_fz - stand_fz
        print(f"\nLIFT measurement:")
        print(f"  FR lower-leg world z, standing (knee=-0.60): {stand_fz:+.3f}")
        print(f"  FR lower-leg world z, bent    (knee=-1.30): {deep_fz:+.3f}")
        print(f"  Lift (positive = foot rose):  {lift:+.3f} m   (per 0.70 rad knee delta)")

if mode == "sweep":
    # Hip sweep period 4s, amplitude 0.15. Look at body x velocity over the sweep cycles.
    # First half period (going low-to-high), second half (high-to-low)
    # Just print full bx trajectory summary.
    bx0 = F(action_rows[0]['bx'])
    bx_max = max(F(r['bx']) for r in action_rows)
    bx_min = min(F(r['bx']) for r in action_rows)
    bx_end = F(action_rows[-1]['bx'])
    print(f"\nSWEEP measurement:")
    print(f"  body x: start={bx0:+.3f} max={bx_max:+.3f} min={bx_min:+.3f} end={bx_end:+.3f}")
    print(f"  net translation: {bx_end - bx0:+.3f} m  range: {bx_max - bx_min:+.3f} m")

if mode == "walk":
    bx_start = F(action_rows[0]['bx'])
    bx_end = F(action_rows[-1]['bx'])
    walk_dur = (F(action_rows[-1]['t_ms']) - F(action_rows[0]['t_ms'])) / 1000.0
    upright = sum(1 for r in action_rows if F(r['bz']) > 0.2 and abs(F(r['pitch'])) < 0.5 and abs(F(r['roll'])) < 0.5)
    print(f"\nWALK measurement:")
    print(f"  duration: {walk_dur:.2f}s   forward dx: {bx_end-bx_start:+.3f} m   mean v: {(bx_end-bx_start)/max(walk_dur,1e-3):+.3f} m/s")
    print(f"  upright (bz>0.2, |pitch|<0.5, |roll|<0.5): {100*upright/len(action_rows):.0f}% of samples")
PY
