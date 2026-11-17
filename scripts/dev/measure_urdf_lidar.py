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

"""measure_urdf_lidar.py - measure a URDF-declared planar lidar against known walls.

Reproduces the TurtleBot3 LDS-01 and Jackal SICK LMS1xx measurements
(2026-09-30). For one of the two known-geometry worlds it:

  1. launches the engine WITH the renderer (a Lidar is a render-based device:
     under ``--no-rendering`` the controller blocks on a frame that never
     arrives -- see tests/benchmarks/omnibench/common/engine_launch.py), with
     ``OMNISIM_URDF_USE_SENSORS=1`` unless ``--control`` is given;
  2. asks the robot's ``omnilink_mobile_bridge`` for ``/list_sensors`` and
     ``/read_sensor``;
  3. checks the scan against the world's walls: for every wall it finds the
     beam of minimum range (the perpendicular one, which is where that beam
     index says "this direction") and compares it with the distance computed
     from the lidar's own pose (bridge-reported mount x robot pose);
  4. drives the robot forward through the GATED ``POST /tool drive_forward``
     and checks that the forward range shrank by the distance the robot's
     pose moved.

The beam-angle convention is not assumed: the script reports the index at
which each wall's perpendicular lands, and the angle the engine's own formula
(``OmLidar::updatePointCloud``: theta_i = fov/2 - (i + 0.5) * fov / n, i.e.
index 0 at the LEFT end of the FOV, sweeping CLOCKWISE seen from above) gives
that index, so a reader can see whether the two agree.

  python scripts/dev/measure_urdf_lidar.py tb3            # TurtleBot3 Burger
  python scripts/dev/measure_urdf_lidar.py tb3 --urdf turtlebot3_waffle
  python scripts/dev/measure_urdf_lidar.py jackal
  python scripts/dev/measure_urdf_lidar.py tb3 --control  # flag unset: no Lidar

Exits 0 when every check passes, 1 when one fails, 2 when the run could not
be made. One engine at a time: this script owns the engine it spawns and
tree-kills it (and any controller left holding the bridge port) on exit.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
IS_WIN = os.name == "nt"
SHOWCASE = REPO / "projects" / "samples" / "demos" / "worlds" / "showcase"

# World-frame wall faces the scan must see. ``axis``/``value``: the plane the
# near face lies on; ``span``: the extent of that face along the other axis
# (a beam that crosses the plane outside it misses). ``beyond_max``: the face
# is farther than max range, so its beams must read "no return".
SCENARIOS = {
    "tb3": {
        "world": SHOWCASE / "tb3_lidar_walls.omniworld",
        "port": 8781,
        "urdf_default": "turtlebot3_burger",
        "walls": [
            {"name": "FRONT_WALL", "axis": "x", "value": 1.50, "span": (-1.0, 1.0)},
            {"name": "LEFT_WALL", "axis": "y", "value": 1.00, "span": (-1.3, 1.3)},
            {"name": "RIGHT_BOX", "axis": "y", "value": -0.60, "span": (-0.15, 0.15)},
            {"name": "REAR_WALL", "axis": "x", "value": -4.00, "span": (-1.5, 1.5),
             "beyond_max": True},
        ],
        # Every obstacle as (centre_x, centre_y, size_x, size_y), exactly as
        # authored in the world, for the per-beam check. The stage's 0.05 m rim
        # is below the scan plane and is left out on purpose.
        "boxes": [(1.55, 0.0, 0.1, 2.0), (0.0, 1.05, 2.6, 0.1), (0.0, -0.75, 0.3, 0.3),
                  (-4.05, 0.0, 0.1, 3.0)],
        "drive_m": 0.5,
        "tolerance_m": 0.02,
        # The burger comes to rest after a drive pitched ~4 deg nose-up (its
        # two layers at +-1.9 deg then disagree by 5.5 mm), which moves the
        # focal point ~13 mm back and slants every beam: measured 2026-09-30,
        # range shrink 0.4624 m vs pose 0.4814 m. That is the chassis, not the
        # lidar, so the post-drive checks get their own, stated tolerance.
        "drive_tolerance_m": 0.03,
    },
    "jackal": {
        "world": SHOWCASE / "jackal_lidar_walls.omniworld",
        "port": 8782,
        "urdf_default": None,
        "walls": [
            {"name": "FRONT_WALL", "axis": "x", "value": 3.00, "span": (-2.0, 2.0)},
            {"name": "LEFT_WALL", "axis": "y", "value": 2.00, "span": (-2.5, 2.5)},
            {"name": "RIGHT_BOX", "axis": "y", "value": -1.50, "span": (-0.08, 0.32)},
            # Straight behind, inside the 90-degree blind sector of a 270-degree
            # scanner: no beam may report it.
            {"name": "REAR_WALL", "axis": "x", "value": -1.50, "span": (-1.0, 1.0),
             "outside_fov": True},
        ],
        "boxes": [(3.05, 0.0, 0.1, 4.0), (0.0, 2.05, 5.0, 0.1), (0.12, -1.7, 0.4, 0.4),
                  (-1.55, 0.0, 0.1, 2.0)],
        "drive_m": 1.0,
        "tolerance_m": 0.02,
    },
    # Same layout as the Jackal's; the Husky's SICK LMS111 is also a 270-degree
    # scanner, so the rear wall sits in its blind sector too.
    "husky": {
        "world": SHOWCASE / "husky_lidar_walls.omniworld",
        "port": 8783,
        "urdf_default": None,
        "walls": [
            {"name": "FRONT_WALL", "axis": "x", "value": 3.00, "span": (-2.0, 2.0)},
            {"name": "LEFT_WALL", "axis": "y", "value": 2.00, "span": (-2.5, 2.5)},
            {"name": "RIGHT_BOX", "axis": "y", "value": -1.50, "span": (-0.08, 0.32)},
            {"name": "REAR_WALL", "axis": "x", "value": -1.50, "span": (-1.0, 1.0),
             "outside_fov": True},
        ],
        "boxes": [(3.05, 0.0, 0.1, 4.0), (0.0, 2.05, 5.0, 0.1), (0.12, -1.7, 0.4, 0.4),
                  (-1.55, 0.0, 0.1, 2.0)],
        "drive_m": 1.0,
        "tolerance_m": 0.02,
    },
}


# ---------------------------------------------------------------- HTTP ----

def http(port: int, path: str, body: dict | None = None, timeout: float = 30.0):
    url = f"http://127.0.0.1:{port}{path}"
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode() or "null")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode() or "null")
        except Exception:
            return e.code, None


def port_answers(port: int) -> bool:
    try:
        http(port, "/list_sensors", timeout=1.5)
        return True
    except Exception:
        return False


# ------------------------------------------------------------- process ----

def resolve_binary() -> Path:
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override)
    for c in (REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe", REPO / "bin" / "omnisim-bin"):
        if c.exists():
            return c
    raise SystemExit("omnisim-bin not found (run `python -m omnisim doctor`)")


def kill_tree(proc: subprocess.Popen) -> None:
    if IS_WIN:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)
    else:
        proc.kill()


def reap_bridge(port: int) -> list[int]:
    """Kill any python controller still holding OUR bridge port (an orphaned
    bridge keeps its port and answers for the next world)."""
    if not IS_WIN:
        return []
    ps = ("Get-CimInstance Win32_Process -Filter \"name='python.exe'\" | "
          "Where-Object { $_.CommandLine -like '*omnilink_mobile_bridge*' -and "
          f"$_.CommandLine -like '*--port*{port}*' }} | ForEach-Object {{ $_.ProcessId }}")
    out = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                         capture_output=True, text=True).stdout
    pids = [int(x) for x in out.split() if x.strip().isdigit()]
    for pid in pids:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)], capture_output=True)
    return pids


# ------------------------------------------------------------ geometry ----

def beam_angle(i: int, n: int, fov: float) -> float:
    """OmLidar::updatePointCloud's azimuth for a fixed (non-rotating) lidar."""
    return fov / 2.0 - (i + 0.5) * fov / n


def ray_to_face(px, py, heading, wall):
    """Distance along a horizontal ray to the wall face, or None if it misses."""
    dx, dy = math.cos(heading), math.sin(heading)
    if wall["axis"] == "x":
        if abs(dx) < 1e-9:
            return None
        t = (wall["value"] - px) / dx
        other = py + t * dy
    else:
        if abs(dy) < 1e-9:
            return None
        t = (wall["value"] - py) / dy
        other = px + t * dx
    lo, hi = wall["span"]
    return t if t > 0 and lo <= other <= hi else None


def ray_to_box(px, py, heading, box):
    """Slab-method distance to an axis-aligned box (cx, cy, sx, sy), or None."""
    cx, cy, sx, sy = box
    d = (math.cos(heading), math.sin(heading))
    lo, hi = (cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy + sy / 2)
    tmin, tmax = -math.inf, math.inf
    for k, p in enumerate((px, py)):
        if abs(d[k]) < 1e-12:
            if p < lo[k] or p > hi[k]:
                return None
            continue
        t1, t2 = (lo[k] - p) / d[k], (hi[k] - p) / d[k]
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmax >= tmin and tmin > 0 else None


def per_beam(row, n, fov, lidar_xy, lidar_yaw, boxes, slant, rmin, rmax, tol):
    """Compare EVERY beam with the first box it should hit (or with 'no
    return' when nothing lies inside (rmin, rmax))."""
    agree_hit = agree_none = 0
    mismatch = []
    errs = []
    for i in range(n):
        h = lidar_yaw + beam_angle(i, n, fov)
        ts = [t for t in (ray_to_box(lidar_xy[0], lidar_xy[1], h, b) for b in boxes) if t is not None]
        exp = min(ts) * slant if ts else None
        if exp is not None and not (rmin < exp < rmax):
            exp = None
        got = row[i]
        if exp is None and got is None:
            agree_none += 1
        elif exp is not None and got is not None and abs(got - exp) <= tol:
            agree_hit += 1
            errs.append(got - exp)
        else:
            mismatch.append({"beam": i, "angle_deg": round(math.degrees(beam_angle(i, n, fov)), 2),
                             "expected_m": None if exp is None else round(exp, 4),
                             "measured_m": None if got is None else round(got, 4)})
    return {"beams": n, "agree_hit": agree_hit, "agree_no_return": agree_none,
            "mismatch": len(mismatch), "mismatches": mismatch[:40],
            "hit_max_abs_err_m": round(max(abs(e) for e in errs), 4) if errs else None,
            "hit_mean_err_m": round(sum(errs) / len(errs), 5) if errs else None}


def yaw_of(state: dict) -> float | None:
    for key in ("yaw", "heading", "theta"):
        v = state.get(key)
        if isinstance(v, (int, float)):
            return float(v)
    pose = state.get("pose") or {}
    for key in ("yaw", "theta", "heading"):
        v = pose.get(key)
        if isinstance(v, (int, float)):
            return float(v)
    return None


def xy_of(state: dict):
    for src in (state, state.get("pose") or {}):
        if isinstance(src.get("x"), (int, float)) and isinstance(src.get("y"), (int, float)):
            return float(src["x"]), float(src["y"])
        pos = src.get("position")
        if isinstance(pos, list) and len(pos) >= 2:
            return float(pos[0]), float(pos[1])
    return None


# ---------------------------------------------------------------- scan ----

def read_scan(port: int, name: str, deadline_s: float = 30.0) -> dict:
    t_end = time.time() + deadline_s
    last = None
    while time.time() < t_end:
        code, r = http(port, "/read_sensor", {"sensor": name})
        last = (code, r)
        if code == 200 and r and r.get("value"):
            return r
        time.sleep(0.5)
    raise RuntimeError(f"no lidar sample within {deadline_s}s: {last}")


def robot_state(port: int) -> dict:
    code, r = http(port, "/get_robot_state", {})
    if code != 200 or not isinstance(r, dict):
        raise RuntimeError(f"/get_robot_state -> {code} {r}")
    return r


def analyse(scan: dict, lidar_xy, lidar_yaw, walls, tol, boxes=()) -> dict:
    lay = scan["layout"]
    n, layers, fov = lay["horizontal_resolution"], lay["number_of_layers"], lay["fov"]
    vals = scan["value"]
    out = {"layout": lay, "layers": []}
    vfov = lay.get("vertical_fov") or 0.0
    for L in range(layers):
        row = vals[L * n:(L + 1) * n]
        finite = [v for v in row if v is not None]
        # OmLidar::updatePointCloud: layer 0 is the TOP one, phi0 = vfov/2,
        # stepping down by vfov/(layers-1). A slanted beam's range to a vertical
        # face is the horizontal distance / cos(phi).
        phi = (vfov / 2.0 - L * vfov / (layers - 1)) if layers > 1 else 0.0
        slant = 1.0 / math.cos(phi)
        lay_out = {"layer": L, "elevation_deg": round(math.degrees(phi), 3),
                   "finite_returns": len(finite),
                   "min": min(finite) if finite else None,
                   "max": max(finite) if finite else None,
                   "below_min_range": sum(1 for v in finite if v < lay["min_range"]),
                   "above_max_range": sum(1 for v in finite if v > lay["max_range"]),
                   "walls": []}
        if boxes:
            lay_out["per_beam"] = per_beam(row, n, fov, lidar_xy, lidar_yaw, boxes, slant,
                                           lay["min_range"], lay["max_range"], tol)
        for w in walls:
            # Beams whose ray, by geometry, lands on this face.
            hits = []
            for i in range(n):
                a = beam_angle(i, n, fov)
                exp = ray_to_face(lidar_xy[0], lidar_xy[1], lidar_yaw + a, w)
                if exp is not None:
                    hits.append((i, a, exp * slant))
            rec = {"wall": w["name"], "beams_on_face": len(hits)}
            if w.get("outside_fov"):
                # The face must be invisible: check no beam reads its distance.
                perp = abs(w["value"] - (lidar_xy[0] if w["axis"] == "x" else lidar_xy[1]))
                rec["perpendicular_m"] = round(perp, 4)
                rec["ok"] = len(hits) == 0
                rec["note"] = ("GEOMETRIC precondition only (no beam of the FOV points at this "
                               "face); per_beam is the measurement that nothing is reported there")
                lay_out["walls"].append(rec)
                continue
            if not hits:
                rec["ok"] = False
                rec["note"] = "no beam lands on this face by geometry"
                lay_out["walls"].append(rec)
                continue
            if w.get("beyond_max"):
                got = [row[i] for i, _, _ in hits]
                rec["expected_m"] = f">{lay['max_range']} (min geometric {min(e for *_, e in hits):.3f})"
                rec["finite_readings_on_face"] = sum(1 for g in got if g is not None)
                rec["ok"] = rec["finite_readings_on_face"] == 0
                lay_out["walls"].append(rec)
                continue
            # The perpendicular beam (minimum expected range) and the beam of
            # minimum MEASURED range over the face's beams.
            i_exp, a_exp, r_exp = min(hits, key=lambda h: h[2])
            errs = [(row[i] - e) for i, _, e in hits if row[i] is not None]
            meas = [(row[i], i) for i, _, _ in hits if row[i] is not None]
            rec.update({
                "perpendicular_beam": i_exp,
                "perpendicular_beam_angle_deg": round(math.degrees(a_exp), 3),
                "expected_m": round(r_exp, 4),
                "measured_m": None if row[i_exp] is None else round(row[i_exp], 4),
                "argmin_measured_beam": min(meas)[1] if meas else None,
                "beams_compared": len(errs),
                "max_abs_err_m": round(max(abs(e) for e in errs), 4) if errs else None,
                "mean_err_m": round(sum(errs) / len(errs), 4) if errs else None,
            })
            rec["ok"] = (row[i_exp] is not None and abs(row[i_exp] - r_exp) <= tol
                         and rec["max_abs_err_m"] is not None and rec["max_abs_err_m"] <= 3 * tol)
            lay_out["walls"].append(rec)
        out["layers"].append(lay_out)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--urdf", default=None,
                    help="tb3 only: turtlebot3_burger (default) | turtlebot3_waffle | turtlebot3_waffle_pi")
    ap.add_argument("--control", action="store_true",
                    help="launch WITHOUT OMNISIM_URDF_USE_SENSORS (expect no Lidar)")
    ap.add_argument("--no-drive", action="store_true")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--json", default=None, help="write the full report here")
    args = ap.parse_args()

    sc = SCENARIOS[args.scenario]
    port = sc["port"]
    world = sc["world"]
    tmp_world = None
    if args.urdf and args.urdf != sc["urdf_default"]:
        if args.scenario != "tb3":
            raise SystemExit("--urdf is tb3-only")
        text = world.read_text(encoding="utf-8").replace(sc["urdf_default"] + ".urdf", args.urdf + ".urdf")
        # Hidden sibling (a dotfile is deliberately-not-catalogued) so the
        # relative URDF url still resolves; removed on exit.
        tmp_world = world.with_name(f".measure_{args.urdf}.omniworld")
        tmp_world.write_text(text, encoding="utf-8")
        world = tmp_world

    if port_answers(port):
        print(f"port {port} already answers -- another bridge (or an orphan) holds it; refusing")
        return 2

    logdir = Path(tempfile.mkdtemp(prefix="urdf_lidar_"))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(logdir / "omnisim_log.txt")
    if args.control:
        env.pop("OMNISIM_URDF_USE_SENSORS", None)
    else:
        env["OMNISIM_URDF_USE_SENSORS"] = "1"
    argv = [str(resolve_binary()), str(world), "--minimize", "--batch", "--mode=realtime",
            "--stdout", "--stderr"]
    console = open(logdir / "console.txt", "w", encoding="utf-8", errors="replace")
    proc = subprocess.Popen(argv, env=env, cwd=str(REPO), stdout=console, stderr=subprocess.STDOUT)
    report = {"scenario": args.scenario, "world": str(world.relative_to(REPO)),
              "urdf": args.urdf or sc["urdf_default"], "control": args.control,
              "engine_pid": proc.pid, "log_dir": str(logdir)}
    rc = 2
    try:
        t_end = time.time() + args.timeout
        sensors = None
        while time.time() < t_end:
            if proc.poll() is not None:
                raise RuntimeError(f"engine exited rc={proc.returncode}; see {logdir}")
            try:
                log_text = (logdir / "omnisim_log.txt").read_text(encoding="utf-8", errors="replace")
            except OSError:
                log_text = ""
            bad = [ln for ln in log_text.splitlines()
                   if "URDF parse error" in ln or "Skipped unknown 'URDFRobot'" in ln]
            if bad:
                raise RuntimeError(f"world did not load its robot: {bad[0]}")
            try:
                code, sensors = http(port, "/list_sensors", timeout=3)
                if code == 200:
                    break
            except Exception:
                pass
            time.sleep(1.0)
        else:
            raise RuntimeError(f"bridge on :{port} never answered in {args.timeout}s; see {logdir}")
        report["list_sensors"] = sensors
        lidars = [s for s in (sensors or {}).get("sensors", []) if s.get("type") == "Lidar"]
        report["lidar_count"] = len(lidars)
        if args.control:
            report["checks"] = {"control_no_lidar": len(lidars) == 0}
            rc = 0 if len(lidars) == 0 else 1
            return rc
        if not lidars:
            raise RuntimeError("flag set but /list_sensors has no Lidar")
        lidar = lidars[0]
        name = lidar["name"]
        mount = lidar.get("mount") or {}
        read_scan(port, name)            # first call enables; wait out warm-up
        time.sleep(1.0)

        def snapshot(label, tol=sc["tolerance_m"]):
            st = robot_state(port)
            xy, yaw = xy_of(st), yaw_of(st)
            if xy is None or yaw is None:
                raise RuntimeError(f"cannot read pose from get_robot_state: {json.dumps(st)[:600]}")
            tr = (mount.get("translation") or [0.0, 0.0, 0.0])
            lx = xy[0] + tr[0] * math.cos(yaw) - tr[1] * math.sin(yaw)
            ly = xy[1] + tr[0] * math.sin(yaw) + tr[1] * math.cos(yaw)
            scan = read_scan(port, name)
            res = analyse(scan, (lx, ly), yaw, sc["walls"], tol, sc.get("boxes", ()))
            res.update({"label": label, "robot_xy": xy, "robot_yaw_rad": yaw,
                        "lidar_xy": (round(lx, 4), round(ly, 4)), "sim_time": scan.get("sim_time")})
            return res, scan

        before, scan0 = snapshot("before_drive")
        report["mount"] = mount
        report["before"] = before
        report["raw_scan_before"] = scan0["value"]
        checks = {}
        # The layer nearest the horizontal is the planar scan. (Picking the
        # layer with the most returns, as the ROS sensor_node does, picks a
        # downward layer that sees the FLOOR on a multi-layer device.)
        best = min(before["layers"], key=lambda L: (abs(L["elevation_deg"]), L["layer"]))
        report["checked_layer"] = best["layer"]
        checks["single_layer_planar"] = before["layout"]["number_of_layers"] == 1
        for w in best["walls"]:
            checks[f"{w['wall']}"] = w["ok"]
        pb = best["per_beam"]
        # A handful of silhouette-edge beams may legitimately snap to the
        # neighbouring surface (OmLidar's discontinuity-aware resample); more
        # than 2% disagreeing is a failure.
        checks["per_beam_agreement"] = pb["mismatch"] <= max(2, int(0.02 * pb["beams"]))
        checks["no_reading_below_min_range"] = all(L["below_min_range"] == 0 for L in before["layers"])
        checks["no_reading_above_max_range"] = all(L["above_max_range"] == 0 for L in before["layers"])

        if not args.no_drive:
            code, r = http(port, "/tool", {"tool": "drive_forward", "distance": sc["drive_m"],
                                           "wait": True}, timeout=120)
            report["drive_call"] = {"route": "/tool drive_forward", "http": code, "reply": r}
            if code != 200:
                # The gated /tool path needs the OmniLink relay's tool registry,
                # which is not up without an OmniKey; fall back to the direct
                # (UNGATED) verb and say so in the report.
                code, r = http(port, "/drive_forward", {"distance": sc["drive_m"], "wait": True},
                               timeout=120)
                report["drive_call"] = {"route": "/drive_forward (ungated; /tool was "
                                                  f"{report['drive_call']['reply']})",
                                        "http": code, "reply": r}
            time.sleep(1.5)  # let it settle to a stop
            dtol = sc.get("drive_tolerance_m", sc["tolerance_m"])
            report["drive_tolerance_m"] = dtol
            after, _ = snapshot("after_drive", dtol)
            report["after"] = after
            moved = math.hypot(after["robot_xy"][0] - before["robot_xy"][0],
                               after["robot_xy"][1] - before["robot_xy"][1])
            fb = next(w for w in best["walls"] if w["wall"] == "FRONT_WALL")
            fa = next(w for w in after["layers"][best["layer"]]["walls"] if w["wall"] == "FRONT_WALL")
            shrink = (fb["measured_m"] - fa["measured_m"]) if fb.get("measured_m") and fa.get("measured_m") else None
            report["drive"] = {"pose_moved_m": round(moved, 4),
                               "front_range_before_m": fb.get("measured_m"),
                               "front_range_after_m": fa.get("measured_m"),
                               "front_range_shrink_m": None if shrink is None else round(shrink, 4),
                               "shrink_minus_moved_m": None if shrink is None else round(shrink - moved, 4)}
            checks["drive_shrink_matches_pose"] = (shrink is not None
                                                   and abs(shrink - moved) <= dtol)
            checks["after_drive_FRONT_WALL"] = fa["ok"]
        report["checks"] = checks
        rc = 0 if all(checks.values()) else 1
        return rc
    except Exception as exc:
        report["error"] = str(exc)
        rc = 2
        return rc
    finally:
        kill_tree(proc)
        try:
            proc.wait(timeout=20)
        except Exception:
            pass
        report["reaped_bridge_pids"] = reap_bridge(port)
        console.close()
        if tmp_world is not None and tmp_world.exists():
            tmp_world.unlink()
        report["exit"] = rc
        slim = {k: v for k, v in report.items() if k != "raw_scan_before"}
        print(json.dumps(slim, indent=1, default=str))
        if args.json:
            Path(args.json).write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
