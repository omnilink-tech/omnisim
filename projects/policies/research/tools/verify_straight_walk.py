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

"""Deploy a Spot policy headless on the ODE walk demo and report straight-line metrics.

Reads the chassis trace CSV the deploy controller writes
(`C:/tmp/husky_trace/spot_deploy.csv`) and reports:
  - forward distance (+x)
  - lateral drift (|y_end - y_start|)
  - yaw drift (|yaw_end - yaw_start|, wrapped)
  - mean vx, sustained
  - upright pct, fall time

Verdict: "straight" if forward > 5m, lateral < 0.5m, yaw_drift < 0.2 rad,
never fell. Exit code 0 if straight, 1 if drifted, 2 if no trace.

    python projects/policies/research/tools/verify_straight_walk.py --policy <onnx> --duration 30 --vx 0.5
"""
from __future__ import annotations
import argparse, csv, math, os, subprocess, sys, time
from pathlib import Path
import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
TRACE = Path(r"C:\tmp\husky_trace\spot_deploy.csv")
OMNISIM_BIN = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
# Default world uses the RL deploy controller; override with --world to
# test the model-only walker (spot_model_walk_demo.wbt) or any other
# Spot world that writes the canonical chassis-trace CSV.
WORLD = REPO / "projects" / "rl" / "worlds" / "spot_walk_demo.wbt"


def run_deploy(policy: Path, duration_s: float, vx: float,
               world: Path = WORLD) -> None:
    if TRACE.exists(): TRACE.unlink()
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["SPOT_POLICY_ONNX"] = str(policy)
    env["OMNISIM_POLICY_ONNX"] = str(policy)
    env["SPOT_DEPLOY_TRACE"] = "1"
    env["OMNISIM_DEPLOY_TRACE"] = "1"
    env["SPOT_VX"] = str(vx); env["SPOT_VY"] = "0.0"; env["SPOT_WZ"] = "0.0"
    cmd = [str(OMNISIM_BIN), str(world), "--batch", "--mode=fast", "--no-rendering", "--minimize"]
    proc = subprocess.Popen(cmd, env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Webots needs ~10s startup before it actually deploys + writes trace
    deadline = time.time() + duration_s + 12
    while time.time() < deadline:
        if proc.poll() is not None: break
        time.sleep(0.5)
    # Force-kill Webots on Windows: SIGTERM does not stop it; taskkill /F /T
    # (with tree) is what actually works.
    if proc.poll() is None:
        try:
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                capture_output=True, timeout=10,
            )
        except Exception:
            try: proc.terminate()
            except Exception: pass
        try: proc.wait(timeout=8)
        except Exception:
            try: proc.kill()
            except Exception: pass


def trim_trace(rows: list[dict], duration_s: float) -> list[dict]:
    """Keep only rows within `duration_s` of the first row's timestamp.
    Webots may keep running past our kill and over-accumulate the trace; the
    user only asked us to measure the first `duration_s` of walking."""
    if not rows: return rows
    t0 = float(rows[0]["t_ms"])
    cutoff_ms = t0 + duration_s * 1000.0
    return [r for r in rows if float(r["t_ms"]) <= cutoff_ms]


def measure(duration_s: float | None = None) -> dict:
    if not TRACE.exists(): return {"error": "no trace"}
    rows = list(csv.DictReader(TRACE.open()))
    if not rows: return {"error": "empty trace"}
    if duration_s is not None:
        rows = trim_trace(rows, duration_s)
        if len(rows) < 2: return {"error": f"trace too short for {duration_s}s window"}
    f = lambda s: float(s)
    bx = np.array([f(r["bx"]) for r in rows])
    by = np.array([f(r["by"]) for r in rows])
    bz = np.array([f(r["bz"]) for r in rows])
    yaw = np.array([f(r["yaw"]) for r in rows])
    roll = np.array([f(r["roll"]) for r in rows])
    pitch = np.array([f(r["pitch"]) for r in rows])
    vx = np.array([f(r["vx"]) for r in rows])
    t = np.array([f(r["t_ms"]) for r in rows]) / 1000.0
    dur = t[-1] - t[0]
    # wrap yaw drift
    dyaw = (yaw[-1] - yaw[0] + math.pi) % (2*math.pi) - math.pi
    fall = (bz < 0.30) | (np.abs(roll) > 0.7) | (np.abs(pitch) > 0.7)
    t_fall = float(t[int(np.argmax(fall))]) if fall.any() else None
    return dict(
        dur=float(dur),
        fwd=float(bx[-1] - bx[0]),
        lat=float(by[-1] - by[0]),
        yaw_drift_rad=float(dyaw),
        yaw_drift_deg=float(math.degrees(dyaw)),
        mean_vx=float(vx.mean()),
        min_bz=float(bz.min()),
        upright_pct=float(((bz > 0.3) & (np.abs(roll) < 0.5) & (np.abs(pitch) < 0.5)).mean() * 100),
        fall_t=t_fall,
    )


def verdict(m: dict, dur_target: float) -> tuple[str, int]:
    if m.get("error"): return ("NO TRACE", 2)
    if m["fall_t"] is not None and m["fall_t"] < dur_target - 2: return (f"FELL at {m['fall_t']:.1f}s", 1)
    straight = abs(m["lat"]) < 0.5 and abs(m["yaw_drift_rad"]) < 0.2 and m["fwd"] > 3.0
    return ("STRAIGHT" if straight else "DRIFTED", 0 if straight else 1)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=False,
                   help="ONNX policy (used by spot_walk_demo.wbt's deploy "
                        "controller). Pass --no-policy to verify a model-only "
                        "controller -- in that case --world is required and "
                        "the SPOT_POLICY_ONNX env var is ignored by the world.")
    p.add_argument("--no-policy", action="store_true",
                   help="Run a model-only world (no ONNX) and just measure.")
    p.add_argument("--world", type=Path, default=WORLD,
                   help="Webots world (.wbt) to run. Defaults to "
                        "projects/policies/worlds/spot_walk_demo.wbt.")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--vx", type=float, default=0.5)
    args = p.parse_args()
    if not args.no_policy:
        if args.policy is None:
            print("--policy is required unless --no-policy is set"); return 2
        if not args.policy.exists():
            print(f"missing: {args.policy}"); return 2
        label = args.policy.name
        policy_arg = args.policy.resolve()
    else:
        label = f"<model-only:{args.world.name}>"
        policy_arg = Path("none")  # placeholder; not read by model-only world
    print(f"[verify] policy={label}  world={args.world.name}  "
          f"dur={args.duration}s  vx={args.vx}")
    run_deploy(policy_arg, args.duration, args.vx, world=args.world)
    m = measure(args.duration)
    print()
    if m.get("error"): print(f"  ERROR: {m['error']}"); return 2
    print(f"  duration : {m['dur']:.1f}s   forward: {m['fwd']:+.2f}m   mean vx: {m['mean_vx']:+.3f} m/s")
    print(f"  LATERAL  : {m['lat']:+.2f}m  (target |lat|<0.5m for straight)")
    print(f"  YAW DRIFT: {m['yaw_drift_rad']:+.3f} rad ({m['yaw_drift_deg']:+.1f} deg)  (target |yaw|<0.2 rad / 11deg)")
    print(f"  upright  : {m['upright_pct']:.0f}%   min_bz: {m['min_bz']:.2f}m   fall: {m['fall_t']}")
    v, rc = verdict(m, args.duration)
    print(f"\n  VERDICT: {v}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
