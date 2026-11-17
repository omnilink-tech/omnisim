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

"""OMNITUG500 MOVING laser-scanner demo controller.

Drives the visual OMNITUG500 and the sidecar SCANNERS robot (which carries the two
corner Lidars) along one elliptical patrol loop, and every tick rebuilds the two
fused coverage fans as half-transparent red meshes IN THE SCENE, in WORLD
coordinates, so the covered area sweeps around with the moving rover.

Frames: an OmniSim Lidar returns its scan in the SENSOR frame (a point at azimuth
theta is at r*(cos theta, sin theta, 0); it looks along local +X). With the rover
at world pose (x, y, yaw), scanner k (corner offset ox,oy, mount yaw phi) has
world pose:
    lidar_xy  = (x, y) + Rz(yaw) * (ox, oy)
    lidar_yaw = yaw + phi
and a sensor point maps to the world by  lidar_xy + Rz(lidar_yaw) * (r cos, r sin).

The fans are drawn at z=0.131 (the scan-plane height); being coplanar with the
single horizontal scan layer they graze the rays edge-on and do not corrupt it.
The 3D fan node is removed + re-imported each tick (the reliable way to refresh
the renderer); doing both within one control step means the gap is never drawn,
so there is no flicker.
"""

import math
import os
import traceback

_OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "..", "_scratch", "omnitug500_lidar_patrol"))


def _boot(msg: str) -> None:
    try:
        os.makedirs(_OUT_DIR, exist_ok=True)
        with open(os.path.join(_OUT_DIR, "debug.log"), "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


_boot("module import: starting")
try:
    from omnisim import Supervisor
    _boot("module import OK")
except Exception:
    _boot("module import FAILED:\n" + traceback.format_exc())
    raise

# scanner mounts: (offset_x, offset_y, mount_yaw_rad) -- match the .wbt
SCANNERS = {
    "scanner_front_right": (0.290, 0.537, 0.7854),
    "scanner_rear_left":  (-0.290, -0.536, 3.9270),
}

# patrol path (matches the obstacle ring in omnitug500_lidar_patrol.omniworld)
A, B = 2.8, 1.8                       # ellipse semi-axes (m)
PERIOD = 12.0                         # seconds per loop
FLOOR_Z = 0.0
TURN_RATE_LIMIT = math.radians(160)
FORWARD_OFFSET = -math.pi / 2.0       # rover forward = local +Y

FAN_Z = 0.131                         # overlay at the scan-plane height
FAN_VERTS = 80                        # boundary points per fan (subsampled)


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def fan_pose_stanza(apex, boundary):
    """Translucent-red triangle-fan Shape (double-sided) in WORLD coordinates."""
    pts = [apex] + boundary
    point_str = ", ".join(f"{x:.4f} {y:.4f} {FAN_Z:.4f}" for x, y in pts)
    idx = []
    for i in range(1, len(boundary)):
        idx += [0, i, i + 1, -1, 0, i + 1, i, -1]
    index_str = " ".join(str(v) for v in idx)
    return (
        "Pose { children [ Shape { "
        "appearance PBRAppearance { baseColor 0.85 0.16 0.16 transparency 0.62 "
        "metalness 0 roughness 1 emissiveColor 0.08 0 0 } "
        f"geometry IndexedFaceSet {{ coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] }} }} ] }}"
    )


def main() -> None:
    _boot("controller start")
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    dt = ts / 1000.0

    lidars = {}
    for name in SCANNERS:
        dev = robot.getDevice(name)
        if dev is not None:
            dev.enable(ts)
            lidars[name] = dev
    if not lidars:
        _boot("no lidars -- abort")
        return
    _boot(f"enabled {len(lidars)} scanners")

    me = robot.getSelf()                      # the SCANNERS robot
    my_tf, my_rf = me.getField("translation"), me.getField("rotation")
    tug = robot.getFromDef("OMNITUG500")
    tug_tf = tug.getField("translation") if tug else None
    tug_rf = tug.getField("rotation") if tug else None
    root_children = robot.getRoot().getField("children")

    fan_nodes = {name: None for name in lidars}
    w = 2.0 * math.pi / PERIOD
    phi = -math.pi / 2.0                       # start at (0, -B)
    yaw = None
    step_count = 0

    while robot.step(ts) != -1:
        step_count += 1

        # --- advance the patrol pose ---
        phi += w * dt
        x, y = A * math.cos(phi), B * math.sin(phi)
        vx, vy = -A * math.sin(phi), B * math.cos(phi)
        target_yaw = math.atan2(vy, vx) + FORWARD_OFFSET
        if yaw is None:
            yaw = target_yaw
        else:
            d = max(-TURN_RATE_LIMIT * dt, min(TURN_RATE_LIMIT * dt, wrap(target_yaw - yaw)))
            yaw = wrap(yaw + d)

        pose, rot = [x, y, FLOOR_Z], [0.0, 0.0, 1.0, yaw]
        my_tf.setSFVec3f(pose)
        my_rf.setSFRotation(rot)
        if tug_tf:
            tug_tf.setSFVec3f(pose)
            tug_rf.setSFRotation(rot)

        # let the scan settle after the first repositioning
        if step_count < 4:
            continue

        cyaw, syaw = math.cos(yaw), math.sin(yaw)
        for name, dev in lidars.items():
            ox, oy, phi_m = SCANNERS[name]
            # scanner world pose
            lwx = x + (cyaw * ox - syaw * oy)
            lwy = y + (syaw * ox + cyaw * oy)
            lyaw = yaw + phi_m
            clw, slw = math.cos(lyaw), math.sin(lyaw)

            fov = dev.getFov()
            res = dev.getHorizontalResolution()
            maxr, minr = dev.getMaxRange(), dev.getMinRange()
            dtheta = -fov / res
            theta0 = fov / 2.0 + dtheta / 2.0
            rng = dev.getRangeImage()

            boundary = []
            for k in range(FAN_VERTS):
                i = int(round(k * (res - 1) / (FAN_VERTS - 1)))
                theta = theta0 + i * dtheta
                r = rng[i] if i < len(rng) else maxr
                hit = math.isfinite(r) and (minr <= r <= maxr * 0.999)
                rr = r if hit else maxr
                px, py = rr * math.cos(theta), rr * math.sin(theta)
                wx = lwx + (clw * px - slw * py)
                wy = lwy + (slw * px + clw * py)
                boundary.append((round(wx, 4), round(wy, 4)))

            # rebuild the fan node (remove + import within one step -> no flicker)
            try:
                if fan_nodes[name] is not None:
                    fan_nodes[name].remove()
                root_children.importMFNodeFromString(-1, fan_pose_stanza((round(lwx, 4), round(lwy, 4)), boundary))
                fan_nodes[name] = root_children.getMFNode(-1)
            except Exception:
                _boot(f"fan rebuild failed ({name}):\n" + traceback.format_exc())

        if step_count == 6:
            _boot("first coverage fans drawn; patrol running")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        _boot("FATAL:\n" + traceback.format_exc())
        raise
