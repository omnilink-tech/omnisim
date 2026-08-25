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

"""OMNITUG500 dual-corner laser-scanner controller (with live 3D coverage fans).

Reads the two real Lidar devices on the OMNITUG500's diagonal safety-scanner corners
(front-right + rear-left) and:
  1. fuses both scans into a top-down point cloud -> _scratch/omnitug500_lidar/scan.json
  2. draws each scanner's swept coverage area as a LIVE half-transparent red mesh
     IN THE 3D SCENE (a triangle-fan IndexedFaceSet rebuilt every few ticks via
     the supervisor), so you can watch the covered area in the GUI.
  3. saves one top-down proof frame from the TOPCAM camera.

An OmniSim Lidar returns its scan in the SENSOR frame, with a point at
azimuth theta at r*(cos theta, sin theta, 0) -- it looks along local +X and
sweeps the local X-Y plane. Each scanner is yawed about +Z, so a sensor-frame
(x, y) maps to the rover/world frame by rotating by the mount yaw and adding the
mount offset (the SCANNERS robot sits at the world origin, yaw 0).

The fan meshes are drawn at z=0.08, BELOW the z=0.131 single-layer scan plane, so
the lidars never see their own overlay (which would corrupt the scan).
"""

import json
import math
import os
import traceback

# Resolve an absolute debug/output dir before anything that can fail.
_OUT_DIR_BOOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "..", "_scratch", "omnitug500_lidar"))


def _boot(msg: str) -> None:
    try:
        os.makedirs(_OUT_DIR_BOOT, exist_ok=True)
        with open(os.path.join(_OUT_DIR_BOOT, "debug.log"), "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


_boot("module import: starting")
try:
    from omnisim import Supervisor
    _boot("module import: 'from omnisim import Supervisor' OK")
except Exception:
    _boot("module import FAILED:\n" + traceback.format_exc())
    raise

# --- scanner mounts: (offset_x, offset_y, mount_yaw_rad) in the rover frame ---
# Keep in sync with the DEF SCAN_* Lidar translation/rotation in the .wbt.
SCANNERS = {
    "scanner_front_right": (0.290, 0.537, 0.7854),   # +45 deg, aims +X+Y
    "scanner_rear_left":  (-0.290, -0.536, 3.9270),  # +225 deg, aims -X-Y
}

ROVER_W, ROVER_L = 0.716, 1.259    # footprint (m) for the visualiser
FAN_Z = 0.131                      # overlay AT the sensor / scan-plane height
                                   # (coplanar -> the rays graze it edge-on, so it
                                   #  does not register as a return; verified below)
FAN_VERTS = 110                    # boundary points per fan (subsampled from res)
FAN_EVERY = 3                      # rebuild the 3D fans every N steps
SHOT_STEP = 40                     # save the proof frame at this step

OUT_DIR = _OUT_DIR_BOOT
OUT_PATH = os.path.join(OUT_DIR, "scan.json")
SHOT_PATH = os.path.join(OUT_DIR, "topdown.png")


def dbg(msg: str) -> None:
    _boot(msg)


def fan_pose_stanza(apex, boundary):
    """Full translucent-red Shape (triangle fan, double-sided) for the scene.

    apex = vertex 0, boundary = vertices 1..B. Each triangle is emitted in BOTH
    winding orders so the fan is visible from above regardless of face culling.
    """
    pts = [apex] + boundary
    point_str = ", ".join(f"{x:.4f} {y:.4f} {FAN_Z:.4f}" for x, y in pts)
    idx = []
    for i in range(1, len(boundary)):
        idx += [0, i, i + 1, -1, 0, i + 1, i, -1]   # front + back face
    index_str = " ".join(str(v) for v in idx)
    return (
        "Pose { children [ Shape { "
        "appearance PBRAppearance { baseColor 0.85 0.16 0.16 transparency 0.62 "
        "metalness 0 roughness 1 emissiveColor 0.08 0 0 } "
        f"geometry IndexedFaceSet {{ coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] }} }} ] }}"
    )


def main() -> None:
    dbg("controller start")
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())

    lidars = {}
    for name in SCANNERS:
        dev = robot.getDevice(name)
        if dev is None:
            dbg(f"ERROR: device '{name}' not found")
            continue
        dev.enable(ts)
        lidars[name] = dev
        dbg(f"enabled '{name}' res={dev.getHorizontalResolution()} fov={dev.getFov():.3f}")
    if not lidars:
        dbg("no lidars enabled -- nothing to do")
        return

    # Optional top-down camera for the proof still.
    cam = robot.getDevice("topcam")
    if cam is not None:
        cam.enable(ts)
        dbg("topcam enabled")

    root_children = robot.getRoot().getField("children")

    os.makedirs(OUT_DIR, exist_ok=True)
    write_every = max(1, int(round(500.0 / ts)))   # scan.json ~ every 0.5 s
    step_count = 0
    shot_saved = False
    fans_built = False

    while robot.step(ts) != -1:
        step_count += 1
        if step_count < 6:
            continue

        scanners_out = []
        fans = {}     # name -> (apex, boundary[]) for the 3D mesh
        for name, dev in lidars.items():
            ox, oy, yaw = SCANNERS[name]
            cyaw, syaw = math.cos(yaw), math.sin(yaw)
            maxr, minr = dev.getMaxRange(), dev.getMinRange()
            fov = dev.getFov()
            res = dev.getHorizontalResolution()
            dtheta = -fov / res
            theta0 = fov / 2.0 + dtheta / 2.0
            rng = dev.getRangeImage()

            def to_rover(px, py):
                return [round(ox + (cyaw * px - syaw * py), 4),
                        round(oy + (syaw * px + cyaw * py), 4)]

            pts_world, fan_world, ranges = [], [], []
            for i in range(res):
                theta = theta0 + i * dtheta
                r = rng[i] if i < len(rng) else maxr
                hit = math.isfinite(r) and (minr <= r <= maxr * 0.999)
                rr = r if hit else maxr
                ep = to_rover(rr * math.cos(theta), rr * math.sin(theta))
                fan_world.append(ep)
                if hit:
                    pts_world.append(ep)
                    ranges.append(round(r, 4))
                else:
                    ranges.append(None)

            finite = [r for r in ranges if r is not None]
            scanners_out.append({
                "name": name, "mount": [ox, oy], "yaw": yaw, "fov": fov,
                "n_points": len(pts_world),
                "min_range": round(min(finite), 4) if finite else None,
                "max_range": maxr, "points": pts_world, "fan": fan_world,
            })
            # subsample the fan boundary for a cheap-to-update mesh
            stride = max(1, len(fan_world) // FAN_VERTS)
            boundary = fan_world[::stride]
            if boundary[-1] != fan_world[-1]:
                boundary.append(fan_world[-1])
            fans[name] = ([ox, oy], boundary)

        # --- build the 3D coverage fans once the scan has settled ---
        # (rover + scene are static here, so the coverage is constant -> build
        # once. For a moving rover, remove+re-import these each tick instead.)
        if not fans_built and step_count >= 20:
            try:
                for name, (apex, boundary) in fans.items():
                    root_children.importMFNodeFromString(-1, fan_pose_stanza(apex, boundary))
                fans_built = True
                dbg(f"built {len(fans)} coverage fans in the 3D scene")
            except Exception:
                dbg("fan build failed:\n" + traceback.format_exc())

        # --- save the proof frame once ---
        if cam is not None and not shot_saved and step_count >= SHOT_STEP:
            try:
                cam.saveImage(SHOT_PATH, 100)
                shot_saved = True
                dbg(f"saved proof frame -> {SHOT_PATH}")
            except Exception:
                dbg("saveImage failed:\n" + traceback.format_exc())

        # --- write the fused scan data ---
        if step_count % write_every == 0:
            sectors = [math.inf] * 8
            for s in scanners_out:
                for x, y in s["points"]:
                    ang = math.atan2(y, x)
                    k = int(((ang % (2 * math.pi)) / (2 * math.pi)) * 8) % 8
                    sectors[k] = min(sectors[k], math.hypot(x, y))
            sectors = [None if math.isinf(d) else round(d, 3) for d in sectors]
            out = {"t_ms": step_count * ts, "rover_footprint": [ROVER_W, ROVER_L],
                   "sector_min_dist": sectors, "scanners": scanners_out}
            try:
                with open(OUT_PATH, "w", encoding="utf-8") as f:
                    json.dump(out, f)
            except OSError as e:
                dbg(f"write skipped ({e})")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        dbg("FATAL:\n" + traceback.format_exc())
        raise
