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

"""Metazoa director: ONE supervisor runs the whole reef (DESIGN.md, PLAN.md).

Cells are `controller "<none>"` Robots with no process of their own. This
supervisor reads every living cell's pose, hands the positions to the pure
ecology (`mz.ecology.Reef.step`), executes the ACTIONS it returns as field
writes, drives every organism's gait through batched
`CELL_<i>_HINGE_PARAMS.position` writes (`mz.organism.chain_targets`), runs the
docking manoeuvre that turns a free cell into a member, and serves the agent
surface (`mz.surface`). Nothing here re-implements a rule that lives in a pure
module; this file owns the engine round trips, the clock, the files and the
HTTP thread.

Engine facts this file is written against (projects/metazoa/README.md, P1):
  * hinge sign: +position = nose DOWN (right-hand about the authored +y axis);
    on a cell rolled 90 deg (a yaw hinge) positive bends the nose to the
    cell's RIGHT. The polarity of the steering channel is NOT assumed: every
    organism calibrates it once with a wiggle (the alife method).
  * the ORGANISM (P2, probe_wave): the head (spine[-1]) is a yaw cell driven
    as a RUDDER only -- hinge = bias_yaw + steer_gain*steer, no wave; every
    other spine cell is pitch and carries the wave with -|dphi|, which moves
    the chain HEAD-FIRST (0.094 m/s straight, steer +-1 -> +-0.95 rad per
    15 s, turn radius ~1 m). +|dphi| runs it TAIL-FIRST (the reverse gear).
    Any mid-spine yaw cell is held at bias_yaw (wave-free). Axes are MEASURED.
  * GROWTH IS AT THE TAIL: a recruit's f_nose meets spine[0]'s f_tail; the
    lock is written on the free cell's f_nose (symmetric connectors, the free
    cell is inert). The engine welds only when a partner face is within
    tolerance, so a lock write is a request and the read-back is the answer.
    Trailer manoeuvre: run head-first onto the free cell's nose normal beyond
    it (runway R1 then R2), back up (reverse wave) tail-first, capture assist
    inside CAPTURE_M / CAPTURE_AXIS, lock, verify while still reversing,
    `Reef.recruit(at_tail=True)`, wave back in head-first.
  * lone cells do not move (Finding 3): free cells get NO actuation.
  * teleport = translation + rotation + resetPhysics(), NEVER setVelocity()
    (alife: a velocity reset freezes a Newton body for ~2 s).
  * every DEF is resolved once, LOUDLY; a missing cell is fatal (exit 1).

Files (relative to `_run/metazoa/`, overridable with METAZOA_RUN_DIR):
  reef.json          [in]  metazoa.py build_reef(): cells, organisms, free, parked
  config.json        [in]  {arena, n_patches, epoch_s, watch, epoch, seed, dim, ...}
  telemetry.json     [out] every TELEMETRY_EVERY ticks
  epoch_result.json  [out] at epoch_s, then simulationQuit(0) unless config.watch

Env: METAZOA_PORT (default 8790) for the HTTP bridge.

THREADING CONTRACT (mz.surface): the HTTP thread never touches the supervisor.
It validates (Router), queues (SimThreadQueue) and waits; the sim thread
drains the queue once per tick and completes body-moving verbs only after the
engine has stepped and the result was READ BACK. /census is a snapshot
(CensusBox) republished every CENSUS_EVERY ticks.
"""
import json
import math
import os
import random
import sys
import threading
import time
from http.server import ThreadingHTTPServer

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))          # projects/metazoa
sys.path.insert(0, ROOT)
from mz import cell as C          # noqa: E402
from mz import ecology as ECO     # noqa: E402
from mz import organism as ORG    # noqa: E402
from mz import surface as SF      # noqa: E402

RUN = os.environ.get("METAZOA_RUN_DIR") or os.path.join(ROOT, "_run", "metazoa")
REEF_PATH = os.path.join(RUN, "reef.json")
CFG_PATH = os.path.join(RUN, "config.json")
RESULT_PATH = os.path.join(RUN, "epoch_result.json")

PORT = int(os.environ.get("METAZOA_PORT", "8790"))
CENSUS_EVERY = 25           # ticks between /census republishes (0.2 s at dt 8)
TELEMETRY_EVERY = 250       # ticks between telemetry.json writes (2 s)
VERIFY_TICKS = 3            # steps before a command's result is read back
FADE_S = 1.0                # gait amplitude fade-in after dock / undock / teleport / birth
CAL_DELAY_S = 2.0           # first wiggle this long after an organism's birth
CAL_TRIAL_S = 1.0           # each half of the wiggle
CAL_STEER = 0.5
TRAVEL_WINDOW_S = 1.0       # head position history for the measured heading / speed
TRAVEL_MIN_M = 0.03         # below this displacement the nose yaw is the heading
DOCK_GAP = C.DEFAULT_GAP    # 0.01 m authored face gap of a docked pair
LOCK_SEP = C.DISTANCE_TOLERANCE      # 0.03: write the lock when the faces are this close
LOCK_AXIS = C.AXIS_TOLERANCE         # 0.45 rad: ... and the normals are opposed this well
VERIFY_S = 0.5              # after a lock write, the faces must stay inside VERIFY_SEP this long
VERIFY_SEP = 0.08           # a welded recruit stays this close while the body moves (measured 0.0407 under reverse pull)
                                     # a face that was merely near is left behind / pushed away
BACKOFF_S = 3.0             # after a failed verify: head-first away for this long, then the runway again
# Trailer manoeuvre geometry (n = the free cell's NOSE normal, N = free nose face position,
# L = organism length from its tail FACE to its head ROOT):
RUNWAY_R1_M = 0.35          # R1 = N + n*(L + 0.35): the head here puts the tail face 0.35 m off the nose
RUNWAY_R2_M = 1.0           # (legacy) R2 = N + n*(L + 1.0)
RUNWAY_FAR_M = 6.0          # aligned window ends here. MEASURED (epoch 12): a 7-cell body arriving at P_NEAR facing inward U-turns with a 1.9 m tail swing and is back on the axis only at 5.7 m; the steered reverse holds 1 cm, 6 m is ~100 s
R_FAR_REACHED_M = 0.45
RUNWAY_NEAR_M = 0.7         # P_NEAR = N + n*(L + 0.7): the turn-in point just outside the bay
P_NEAR_REACHED_M = 0.7      # >= the pair's turning radius (0.3 m orbited P_NEAR at 0.5-0.6 m, 14 K-turns, no arrival)
TURN_SLOWDOWN = 0.0         # MEASURED (probe_wave 2026-08-29, gain 0.5): a smaller wave does NOT tighten the turn -- A 0.6 gives 0.13 rad/15 s at 0.6 m (radius 4.6 m), A 0.45 none; full A gives 0.62 rad at 1.25 m (radius 2 m). Earlier "0.6 starved the rudder" was measured with the 1.0 rad over-steer.
LOOKAHEAD_LINE_M = 0.4      # line-following aim point ahead of the head's projection on the axis (0.6 converged over ~3 m -- too slow for the runway)
LINE_ERR_FULL_RAD = 0.3     # full lock beyond this heading error while following the axis (0.6: a 0.6 m offset took ~4 m of runway to converge -- the rudder only turns the body decisively near full lock)
LINE_ABORT_LATERAL_M = 2.5  # the U-turn at R_FAR swings ~2 m wide at a 1 m turn radius (measured
                            # 0.9 aborted every U-turn); only a real runaway backs out
LINE_ABORT_ALONG_M = 1.5    # ... or heading further out than R_FAR by this much
R1_REACHED_M = 0.30         # the head is "at" R1 inside this radius (then it aims at R2)
R2_OVERSHOOT_M = 0.25       # head past R2 by this without alignment -> go around and retry
GO_AROUND_SIDE_M = 1.6      # go-around loop point: R1 + side*perp*1.6 - n*0.8 (turn radius ~1 m)
GO_AROUND_BACK_M = 0.8
ALIGN_TAIL_LATERAL_M = 0.15 # reverse when the tail face is within this of the normal (the reverse leg is steered now; 0.10 with a 0.12 rad spine gate missed a body that was on the axis at 5.4 m) ...
ALIGN_SPINE_RAD = 0.20      # ... and the spine (tail->head) is within this of n (an undulating body wobbles +-0.1 rad)
                            # misalignment reverse 2.5 m and drift 0.35 m off-axis, measured x4) ...
ALIGN_TAIL_ALONG_M = 0.15   # ... and the tail face is at least this far beyond the nose
REVERSE_ABORT_LATERAL_M = 0.45   # reversing with the tail this far off the normal -> runway again
# REVERSE STEERING (measured, probe_wave REVERSE=1, 6 cells, pair, gain 0.5):
# the trailing pair steers the reverse gear with the SAME sign as forward
# (steer+ -> +0.52 rad, steer- -> -0.97 rad per 15 s) at a speed cost (0.37 m
# straight, 0.16 m at steer+). A single trailing rudder has none (0.00 rad).
# So the reverse leg is closed-loop: the TAIL face pursues a point on the
# axis REV_LOOKAHEAD_M ahead of it (toward the free nose), gently.
REV_LOOKAHEAD_M = 0.5
REV_ERR_FULL_RAD = 0.8
REV_STEER_MAX = 0.6
REV_GENTLE_ALONG_M = 0.6    # inside this (tail face to free nose) the reverse runs at the genome's A
REV_A = 1.2                 # reverse-gear wave amplitude (rad). MEASURED (probe_wave REVERSE=1, pair):
                            # A 0.9 reverses 0.37 m / 15 s, A 1.2 reverses 0.87 m (2.4x) and still steers
REVERSE_TIMEOUT_S = 200.0   # the pair reverses at ~0.025 m/s (measured): 3 m of runway is 120 s    # a reverse that has not captured by then -> runway again
FACE_CHECK_M = 0.35         # read the two face nodes only when the tail root is this close
CAPTURE_SETTLE_TICKS = 1    # ticks between the capture teleport and the lock write. MEASURED
                            # (epochs 9-10): a lock written the tick after the capture welded the
                            # cell at a STALE pose -- sep read a constant 0.122 m (root on the tail
                            # face) while the body moved; the retry without capture held at 0.04-0.05
CAPTURE_LOCK_SEP = 0.011    # after a capture, lock only at the captured gap (see the stale-pose note)
CAPTURE_M = 0.22            # capture assist range (tail face <-> free nose face)
CAPTURE_AXIS = 0.9          # ... and alignment (rad, normals opposed)
MAX_DOCK_ATTEMPTS = 5       # then the cell is blacklisted for this organism for BLACKLIST_S
BLACKLIST_S = 60.0
POLARITY_WINDOW_S = 10.0    # heading-error growth window that flips the steering sign
RECRUIT_TIMEOUT_S = 90.0
# ARENA + STUCK HANDLING (measured 2026-08-29, 14-cell reef, 420 s): both
# organisms recruited once and then ended the epoch pinned against the arena
# wall -- one stalled at 0.003 m/s in line_out for 250 s, the other drove AWAY
# from its target for 100 s with the rudder swinging +-1 every few seconds as
# the heading error wrapped at +-pi (a target dead astern has no turn
# direction; alternating full lock nets zero turn).
RUDDER_MAX_RAD = 0.6        # hard cap on the head rudder angle (see drive_organism)
# K-TURN (measured, probe_dock off-axis 2026-08-29): a target inside the
# turning circle is ORBITED -- the body circled P_NEAR at d 0.8-2.0 m for
# 100 s with |err| pinned at 1.7-2.2 rad and full lock. A vehicle whose
# target is closer than its turn radius backs up first.
# BASK (measured, epoch 14): a roaming body aims at the patch CENTRE, reaches
# it, and then chases a point under its own head -- 0.007 m/s for 350 s,
# K-turns, no light gained. A body within BASK_M of its roam target lies
# still on the patch (wave amplitude 0, no steer) until the ecology gives it
# something to do; leaving the target more than BASK_LEAVE_M away resumes.
BASK_M = 0.6
BASK_LEAVE_M = 1.2
KTURN_DIST_M = 0.9          # aim closer than this ...
KTURN_ERR_RAD = 1.4         # ... and more than this off the heading -> reverse for KTURN_S
KTURN_S = 25.0              # ceiling; the K-turn ends when the aim is KTURN_CLEAR_M away (or reached)
KTURN_CLEAR_M = 1.5
KTURN_COOLDOWN_S = 6.0      # forward driving between two K-turn reverses
RUDDER_CELLS = 2            # yaw cells at the head that carry the steer command. MEASURED
                            # (probe_wave, 6 cells, gain 0.5, 2026-08-29): one rudder turns
                            # 0.62 rad/15 s at 0.083 m/s (radius 2 m); a HEAD PAIR turns
                            # 1.8-2.0 rad/15 s over 0.61 m (radius ~0.3 m) at 0.081 m/s straight
TURN_COMMIT_RAD = 2.3       # |err| beyond this: commit to one turn direction ...
TURN_RELEASE_RAD = 1.8      # ... until the error is back inside this
WALL_MARGIN_M = 0.9         # head this close to a wall and heading at it -> aim at the centre
WALL_BACKOFF_M = 1.2        # ... and this close: REVERSE until the head is this far off the wall (rudder PAIR turn radius ~0.3 m; was 2.6 for the single rudder)
                            # (measured: a body pressed nose-first into the wall at 4.71 m kept a
                            # 0.06 m/s head wobble and never turned -- a rudder needs way on)
WALL_LOOK_M = 2.2           # a wall this far ahead is "the wall ahead" for the state machine
WALL_LOG_S = 20.0
STUCK_SPEED = 0.015         # m/s over a STUCK_S window (head wobble defeats a 1 s speed) ...
STUCK_S = 25.0
UNSTICK_S = 8.0             # ... -> reverse gear, no steering, for this long    # a forced /recruit that has not docked by then is dropped
UPRIGHT_MIN = 0.35          # tail-block up-vector z below this = on its side (reported, not righted)
LOCK_SETTLE_TICKS = 25      # 0.2 s: seeded welds wait for the first registered step; at 1 s a
                            # side-roller (yaw) cell had time to tip flat before its weld took (measured)


_LOG_FH = None


def log(msg):
    """stdout AND _run/metazoa/world.log: the engine discards a controller's
    stdout on Windows, so without the file the docking trace is unreadable."""
    global _LOG_FH
    line = "[reef] %s" % msg
    # stdout carries EVENTS only, never the per-2 s trace: four long runs ended
    # with omnisim-bin at exit code 1 and no diagnostic, each earlier than the
    # last (336, 72, 40, 14 s) as the trace volume grew -- the controller's
    # stdout pipe is the one per-tick load that changed. And print() must
    # never raise into a tick.
    if not msg.startswith("  "):
        try:
            print(line, flush=True)
        except Exception:                            # noqa: BLE001
            pass
    try:
        if _LOG_FH is None:
            _LOG_FH = open(os.path.join(os.environ.get("METAZOA_RUN_DIR", RUN), "world.log"), "a", encoding="utf-8")
        _LOG_FH.write(line + chr(10))
        _LOG_FH.flush()
    except Exception:                                # noqa: BLE001
        pass


def wrap(a):
    return ORG.wrap_angle(a)


def write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)


def yaw_of(o):
    """Yaw of a getOrientation() 9-vector: the world direction of local +x."""
    return math.atan2(o[3], o[0])


# ================================================================== engine side
class CellNodes:
    """Every node + field a cell exposes to the supervisor, resolved ONCE."""

    __slots__ = ("i", "robot", "nose", "pos_field", "motor_field", "ring_field",
                 "faces", "lock_fields", "tr_field", "rot_field", "pose_t", "pose_yaw",
                 "spawn_z", "max_torque", "missing")

    def __init__(self, sup, i):
        d = C.cell_defs(i)
        self.i = i
        self.missing = []
        self.robot = sup.getFromDef(d["robot"])
        self.nose = sup.getFromDef(d["nose"])
        hp = sup.getFromDef(d["hinge_params"])
        self.pos_field = hp.getField("position") if hp is not None else None
        mo = sup.getFromDef(d["motor"])
        self.motor_field = mo.getField("maxTorque") if mo is not None else None
        ra = sup.getFromDef(d["ring_app"])
        self.ring_field = ra.getField("emissiveColor") if ra is not None else None
        self.faces = {f: sup.getFromDef(n) for f, n in d["faces"].items()}
        self.lock_fields = {f: (n.getField("isLocked") if n is not None else None)
                            for f, n in self.faces.items()}
        self.tr_field = self.robot.getField("translation") if self.robot is not None else None
        self.rot_field = self.robot.getField("rotation") if self.robot is not None else None
        for name, v in (("robot", self.robot), ("nose", self.nose),
                        ("hinge_params.position", self.pos_field),
                        ("motor.maxTorque", self.motor_field),
                        ("ring_app.emissiveColor", self.ring_field),
                        ("robot.translation", self.tr_field), ("robot.rotation", self.rot_field)):
            if v is None:
                self.missing.append("%s.%s" % (d["robot"], name))
        for f, n in self.faces.items():
            if n is None or self.lock_fields[f] is None:
                self.missing.append("%s.isLocked" % d["faces"][f])
        # The Pose wrapper (worldgen: `Pose { translation X Y Z rotation 0 0 1
        # YAW children [ DEF CELL_i Robot { rotation 1 0 0 ROLL } ] }`). A
        # teleport writes the ROBOT's fields in the Pose's frame -- the write
        # path alife proved -- so the wrapper's transform is read once here.
        self.pose_t, self.pose_yaw = (0.0, 0.0, 0.0), 0.0
        self.spawn_z = C.spawn_z(False)
        self.max_torque = None
        if self.robot is not None:
            try:
                parent = self.robot.getParentNode()
                pt = parent.getField("translation") if parent is not None else None
                pr = parent.getField("rotation") if parent is not None else None
                if pt is not None and pr is not None and parent.getTypeName() == "Pose":
                    self.pose_t = tuple(float(v) for v in pt.getSFVec3f())
                    rot = pr.getSFRotation()
                    self.pose_yaw = float(rot[3]) if float(rot[2]) > 0 else -float(rot[3])
                    self.spawn_z = self.pose_t[2]      # A's SPAWN_Z, whatever cell version
            except Exception as exc:                   # noqa: BLE001  a missing parent is not fatal
                self.missing.append("%s.parent_pose (%r)" % (d["robot"], exc))
        if self.motor_field is not None:
            try:
                self.max_torque = float(self.motor_field.getSFFloat())
            except Exception:                          # noqa: BLE001
                self.max_torque = C.MAX_TORQUE

    # -- reads --------------------------------------------------------------
    def position(self):
        return self.robot.getPosition()

    def orientation(self):
        return self.robot.getOrientation()

    def face_pose2d(self, face):
        """(x, y, normal-yaw) of a face's connector node, MEASURED: the
        connector's +x is its docking normal."""
        n = self.faces[face]
        p = n.getPosition()
        o = n.getOrientation()
        return (p[0], p[1], math.atan2(o[3], o[0])), p

    # -- writes -------------------------------------------------------------
    def set_hinge(self, target):
        self.pos_field.setSFFloat(float(target))

    def lock(self, face, state):
        fld = self.lock_fields.get(face)
        if fld is None:
            return False
        fld.setSFBool(bool(state))
        return True

    def set_torque(self, value):
        self.motor_field.setSFFloat(float(value))

    def set_ring(self, rgb):
        self.ring_field.setSFColor([float(v) for v in rgb])

    def teleport(self, x, y, yaw, z=None, roll=0.0):
        """Robot fields in the Pose's frame; roll about the cell's own x (0 =
        flat, pi/2 = a yaw cell), hinge 0, resetPhysics. No velocity write
        (see module docstring)."""
        z = self.spawn_z if z is None else float(z)
        c, s = math.cos(-self.pose_yaw), math.sin(-self.pose_yaw)
        dx, dy, dz = x - self.pose_t[0], y - self.pose_t[1], z - self.pose_t[2]
        self.tr_field.setSFVec3f([c * dx - s * dy, s * dx + c * dy, dz])
        self.rot_field.setSFRotation(axis_angle_zx(wrap(yaw - self.pose_yaw), float(roll)))
        self.pos_field.setSFFloat(0.0)
        self.robot.resetPhysics()


def axis_angle_zx(yaw, roll):
    """Axis-angle of Rz(yaw) * Rx(roll) (a yaw in the Pose frame, then a roll
    about the cell's own x). Quaternion product, then back to axis-angle."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    # q = qz * qx  (w, x, y, z)
    w = cy * cr
    x = cy * sr
    y = sy * sr
    z = sy * cr
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    ang = 2.0 * math.atan2(n, w)
    return [x / n, y / n, z / n, ang]


# ================================================================== the director
class Director:
    def __init__(self, sup, reef_dict, cfg, run_dir=RUN):
        self.sup = sup
        self.run_dir = run_dir
        self.cfg = cfg
        self.DT = int(sup.getBasicTimeStep())
        self.DT_S = self.DT / 1000.0
        self.ARENA = float(cfg.get("arena", reef_dict.get("arena", 18.0)))
        self.N_PATCHES = int(cfg.get("n_patches", ECO.N_PATCHES))
        self.EPOCH_S = float(cfg.get("epoch_s", 240.0))
        self.WATCH = bool(cfg.get("watch", False))
        self.rng = random.Random(int(cfg.get("seed", 0)) * 1000003 + int(cfg.get("epoch", 0)))
        self.tick = 0
        self.t = 0.0
        self.engine_ms = []
        self.missing = []
        self.quit_code = None
        self.epoch_written = False

        # ---- resolve every DEF once ------------------------------------------
        n_cells = int(reef_dict["n_cells"])
        self.cells = {}
        for i in range(n_cells):
            cn = CellNodes(sup, i)
            self.missing.extend(cn.missing)
            self.cells[i] = cn
        self.patch_nodes, self.patch_tr, self.patch_z, patch_homes = [], [], [], []
        for k in range(self.N_PATCHES):
            n = sup.getFromDef("PATCH_%d" % k)
            fld = n.getField("translation") if n is not None else None
            if fld is None:
                self.missing.append("PATCH_%d.translation" % k)
                self.patch_nodes.append(None)
                self.patch_tr.append(None)
                self.patch_z.append(0.005)
                continue
            v = fld.getSFVec3f()
            self.patch_nodes.append(n)
            self.patch_tr.append(fld)
            self.patch_z.append(float(v[2]))
            patch_homes.append((float(v[0]), float(v[1])))
        cell_missing = [m for m in self.missing if m.startswith("CELL_")]
        if self.missing:
            shown = self.missing[:60]
            log("MISSING %d DEF/field(s): %s%s" % (len(self.missing), " ".join(shown),
                                                    " ..." if len(self.missing) > 60 else ""))
            log("MISSING -> expected CELL_<i>, CELL_<i>_NOSE, CELL_<i>_HINGE_PARAMS.position, "
                "CELL_<i>_MOTOR.maxTorque, CELL_<i>_RING_APP.emissiveColor, "
                "CELL_<i>_F_{TAIL,NOSE,LEFT,RIGHT}.isLocked, PATCH_<k>.translation "
                "(mz/cell.py cell_defs, mz/scene.py patch_lines)")
        self.fatal = bool(cell_missing)

        # ---- the ecology ------------------------------------------------------
        self.reef = ECO.Reef.from_reef_dict(
            reef_dict, self.rng, dim=float(cfg.get("dim", 1.0)),
            n_patches=self.N_PATCHES,
            patch_homes=patch_homes if len(patch_homes) == self.N_PATCHES else None)

        # ---- per-organism runtime state ---------------------------------------
        self.org = {}              # oid -> dict (ramp_t0, cal, hist, dock, ...)
        self.targets = {}          # oid -> (x, y) from this tick's "target" actions
        self.forced = {}           # oid -> {"cell": j, "since": t}  (/recruit)
        self._relock = []          # [(junction, due_tick)] from reroll actions
        self._excluded_logged = set()
        self.free_relaxed = set()  # free cells whose hinge was written 0 once
        self.welds = set()         # (i, face, j) locks this supervisor wrote and did not release
        self.positions = {}        # i -> (x, y, z) read THIS tick (alive cells)
        self.yaws = {}
        self.ups = {}
        self.axis_z = {}           # i -> |world z of the hinge axis| (1 = yaw hinge)
        self.dock_stats = {"attempts": 0, "locks_written": 0, "recruits": 0,
                           "failed_verify": 0, "forced": 0, "captures": 0}
        self._tbuf = {}

        # ---- HTTP bridge -----------------------------------------------------
        self.router = SF.Router(n_patches=self.N_PATCHES, arena=self.ARENA)
        self.queue = SF.SimThreadQueue()
        self.census = SF.CensusBox()
        self.server = None

    # ----------------------------------------------------------------- setup
    def setup(self):
        if self.fatal:
            log("FATAL %d cell DEF(s) unresolved -- nothing to run" % len(self.missing))
            write_json(RESULT_PATH if self.run_dir == RUN else os.path.join(self.run_dir, "epoch_result.json"),
                       {"error": "missing_defs", "missing": self.missing})
            self.quit(1)
            return False
        # Seeded welds are deferred to LOCK_SETTLE_TICKS: a lock written at tick 0
        # hits "Connectors could not be attached because neither ... has a
        # Physics node" (the bodies are not registered yet) and never engages.
        # Rings and patch positions are safe to write now.
        self.pending_initial = []
        for a in self.reef.initial_actions():
            if "lock" in a:
                self.pending_initial.append(a)
            else:
                self.execute(a)
        for o in self.reef.organisms.values():
            self.org_state(o.id)
        self.start_server()
        log("driving %d cells: %d organisms / %d free / %d debris(parked); %d patches; "
            "dt=%d ms epoch_s=%.0f watch=%s arena=%g mutate=%s"
            % (self.reef.n_cells, len(self.reef.organisms), len(self.reef.free_cells()),
               len(self.reef.debris_cells()), self.N_PATCHES, self.DT, self.EPOCH_S,
               self.WATCH, self.ARENA, ECO.mutation_source()))
        return True

    def start_server(self):
        cfg = dict(self.cfg, basic_time_step_ms=self.DT, verify_ticks=VERIFY_TICKS)

        def capabilities():
            doc = self.router.capabilities(config=cfg, verify_ticks=VERIFY_TICKS)
            doc["notes"].append(
                "/recruit is accepted, not completed: the organism must approach and dock "
                "(seconds to minutes); its result reports the MEASURED distance at "
                "acceptance and /census organisms[<id>].dock reports progress.")
            doc["notes"].append(
                "Free cells are inert (P1 finding 3: a symmetric two-block cell cannot "
                "somersault); organisms do the approaching.")
            return doc
        handler = SF.make_handler(self.router, self.queue, self.census.get,
                                  timeout=10.0, capabilities=capabilities)
        try:
            self.server = ThreadingHTTPServer(("127.0.0.1", PORT), handler)
        except OSError as exc:
            log("WARNING bridge not started on port %d (%s) -- the reef runs without it" % (PORT, exc))
            self.server = None
            return
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        log("listening on http://127.0.0.1:%d" % PORT)

    def quit(self, code):
        self.quit_code = code
        if self.server is not None:
            try:
                self.server.shutdown()
            except Exception:              # noqa: BLE001
                pass
        self.sup.simulationQuit(code)

    # ----------------------------------------------------------- org state
    def org_state(self, oid):
        st = self.org.get(oid)
        if st is None:
            st = {"born": self.t, "ramp_t0": self.t, "hist": [], "last_yaw": None,
                  "cal": {"phase": "wait", "t0": None, "acc": 0.0, "dp": None, "dm": None,
                          "b": None, "sign": 1.0, "next": self.t + CAL_DELAY_S, "trial_s": None},
                  "dock": None, "speed": None, "heading": None, "travel_vs_nose": None,
                  "axis_mismatch_logged": False}
            self.org[oid] = st
        return st

    def ramp(self, oid):
        if oid is not None and oid in self.reef.organisms:
            self.org_state(oid)["ramp_t0"] = self.t

    # -------------------------------------------------------------- actions
    def execute(self, action):
        """One ecology action -> field writes. Never raises (a bad action is
        logged, not fatal)."""
        kind, v = next(iter(action.items()))
        try:
            if kind == "lock" or kind == "unlock":
                i, face, j = v
                if self.cells[i].lock(face, kind == "lock"):
                    key = (int(i), face, int(j))
                    if kind == "lock":
                        self.welds.add(key)
                    else:
                        self.welds.discard(key)
                if kind == "unlock":
                    # A tail recruit was locked on the RECRUIT's f_nose, while the
                    # ecology's junction list names the front cell's f_tail as the
                    # active side: release every face this supervisor locked
                    # between the two cells, whichever side carried the write.
                    for key in [k for k in self.welds if {k[0], k[2]} == {int(i), int(j)}]:
                        self.cells[key[0]].lock(key[1], False)
                        self.welds.discard(key)
                self.ramp(self.reef.cells[i].organism)
                self.ramp(self.reef.cells[j].organism)
                if kind == "unlock":
                    self.free_relaxed.discard(int(i))
                    self.free_relaxed.discard(int(j))
            elif kind == "limp":
                self.cells[v].set_torque(0.0)
            elif kind == "unlimp":
                cn = self.cells[v]
                cn.set_torque(cn.max_torque if cn.max_torque is not None else C.MAX_TORQUE)
            elif kind == "reroll":
                # DOCK-FACE ROTATION: a cell re-orients 90 deg about the spine
                # in place so it becomes a rudder (a divided rear half has
                # none). Real reconfigurable modules rotate their connectors
                # relative to neighbours (Roombots, M-TRAN). Bounded: zero
                # displacement, the junction weld is released for one tick
                # and re-engaged, and the engine's tolerance decides.
                i, roll, junction = v          # junction = (active_cell, face, partner), a list of them, or None
                cn = self.cells[i]
                p = self.cell_pose(i)
                if p is not None:
                    junctions = [] if junction is None else (
                        list(junction) if isinstance(junction, list) else [junction])
                    for a, face, b in junctions:
                        self.cells[a].lock(face, False)
                        self.welds.discard((int(a), face, int(b)))
                    cn.teleport(p[0], p[1], p[2], roll=float(roll))
                    for j in junctions:
                        self._relock.append((j, self.tick + 2))
                    self.dock_stats["rerolls"] = self.dock_stats.get("rerolls", 0) + 1
                    log("reroll CELL_%d to roll %.2f (junction %s)" % (i, float(roll), junction))
            elif kind == "teleport":
                i, x, y, yaw = v
                self.cells[i].teleport(float(x), float(y), float(yaw))
                self.free_relaxed.add(int(i))
                self.ramp(self.reef.cells[i].organism)
            elif kind == "ring":
                i, rgb = v
                self.cells[i].set_ring(rgb)
            elif kind == "target":
                oid, xy = v
                self.targets[oid] = (float(xy[0]), float(xy[1]))
            elif kind == "light":
                k, x, y = v
                fld = self.patch_tr[int(k)] if 0 <= int(k) < len(self.patch_tr) else None
                if fld is not None:
                    fld.setSFVec3f([float(x), float(y), self.patch_z[int(k)]])
            else:
                log("WARNING unknown action %r" % (action,))
        except Exception as exc:                   # noqa: BLE001
            log("WARNING action %r failed: %r" % (action, exc))

    # ---------------------------------------------------------------- reads
    def read_cells(self):
        self.positions.clear()
        self.yaws.clear()
        self.ups.clear()
        self.axis_z.clear()
        killed = []
        for c in self.reef.cells:
            if not c.alive:
                continue
            cn = self.cells[c.i]
            p = cn.position()
            if self.reef.watchdog(c.i, p):
                killed.append((c.i, p))
                continue
            o = cn.orientation()
            self.positions[c.i] = (p[0], p[1], p[2])
            self.yaws[c.i] = yaw_of(o)
            self.ups[c.i] = o[8]
            self.axis_z[c.i] = abs(o[7])          # world z of local +y (the hinge axis)
        for i, p in killed:
            log("WATCHDOG t=%.2f CELL_%d diverged: %s -> killed" % (self.t, i, p))
            for a in self.reef.kill_cell(i, cause="watchdog"):
                self.execute(a)
            self.forced = {k: v for k, v in self.forced.items() if v["cell"] != i}

    def head_pose(self, o):
        h = o.head
        if h not in self.positions:
            return None
        p = self.positions[h]
        return (p[0], p[1], self.yaws[h])

    def cell_pose(self, i):
        if i not in self.positions:
            return None
        p = self.positions[i]
        return (p[0], p[1], self.yaws[i])

    # ------------------------------------------------------------ measured
    def update_travel(self, o, st):
        """Head position history -> measured speed + travel heading."""
        hp = self.head_pose(o)
        if hp is None:
            return
        hist = st["hist"]
        hist.append((self.t, hp[0], hp[1]))
        while hist and hist[0][0] < self.t - TRAVEL_WINDOW_S:
            hist.pop(0)
        st["heading"] = hp[2]
        st["travel_vs_nose"] = None
        st["speed"] = 0.0
        if len(hist) > 1 and hist[-1][0] > hist[0][0]:
            dx, dy = hp[0] - hist[0][1], hp[1] - hist[0][2]
            d = math.hypot(dx, dy)
            st["speed"] = d / (hist[-1][0] - hist[0][0])
            if d > TRAVEL_MIN_M:
                st["heading"] = math.atan2(dy, dx)
                st["travel_vs_nose"] = math.cos(wrap(st["heading"] - hp[2]))
        # SPINE HEADING. The head's 1 s travel history is dominated by the
        # lateral wobble of an undulating body (measured: heading error swinging
        # +-2 rad every ~16 s at 0.1 m/s, the recruiter never converging). The
        # tail->head spine axis is stable; a chain travels along it, with a
        # sign this learns from the travel direction whenever the body is
        # clearly moving.
        if len(o.spine) >= 2:
            tp = self.cell_pose(o.spine[0])
            if tp is not None:
                sx, sy = hp[0] - tp[0], hp[1] - tp[1]
                if math.hypot(sx, sy) > 0.05:
                    spine = math.atan2(sy, sx)
                    if st["travel_vs_nose"] is not None and st["speed"] > 0.03 and not st.get("reverse"):
                        agree = math.cos(wrap(st["heading"] - spine))
                        st["spine_sign"] = 1.0 if agree >= 0.0 else -1.0
                    sgn = st.get("spine_sign", 1.0)
                    st["heading"] = wrap(spine + (0.0 if sgn > 0 else math.pi))

    def effective_bodyplan(self, o, st):
        """B's chain_targets reads the hinge axis class from the body plan's
        cycled dock_rotation_pattern; the PHYSICAL axis is what the hinge is.
        Recruits dock flat (roll 0 = pitch) whatever the plan says, and the
        driver does not cycle the pattern, so the axis class per spine cell is
        MEASURED here (|world z of the hinge axis| > 0.7 = yaw) and handed to
        chain_targets as a full-length pattern. Disagreement with the plan is
        logged once per organism."""
        pat = []
        for i in o.spine:
            pat.append(1 if self.axis_z.get(i, 0.0) > 0.7 else 0)
        bp = dict(o.bodyplan)
        planned = [ORG.axis_of(o.bodyplan, k) for k in range(len(o.spine))]
        measured = ["yaw" if r else "pitch" for r in pat]
        if planned != measured and not st["axis_mismatch_logged"]:
            st["axis_mismatch_logged"] = True
            log("%s hinge axes measured %s vs planned %s -- driving the measured axes"
                % (o.id, measured, planned))
        bp["dock_rotation_pattern"] = pat or [0]
        return bp

    # ------------------------------------------------------------ steering
    def calibrate(self, o, st, hp):
        """The alife wiggle: CAL_TRIAL_S of steer +CAL_STEER then -CAL_STEER,
        after CAL_DELAY_S; sign = sign of the yaw-response DIFFERENCE (the
        drift cancels). Returns the forced steer while a trial runs, else None."""
        cal = st["cal"]
        yaw = hp[2]
        ly = st["last_yaw"]
        dyaw = wrap(yaw - ly) if ly is not None else 0.0
        st["last_yaw"] = yaw
        if cal["phase"] == "done":
            return None
        if cal["phase"] == "wait":
            if self.t < cal["next"]:
                return None
            # A trial spans WHOLE gait cycles (>= CAL_TRIAL_S): the body wave
            # itself wobbles the yaw at the gait period, and a trial that cuts
            # a cycle reads that wobble as a steering response.
            period = ORG.TAU / max(o.genome["omega"], 1e-6)
            cal["trial_s"] = period * math.ceil(CAL_TRIAL_S / period)
            cal["phase"], cal["t0"], cal["acc"] = "plus", self.t, 0.0
            return CAL_STEER
        cal["acc"] += dyaw
        if self.t - cal["t0"] >= cal["trial_s"]:
            if cal["phase"] == "plus":
                cal["dp"] = cal["acc"]
                cal["phase"], cal["t0"], cal["acc"] = "minus", self.t, 0.0
                return -CAL_STEER
            cal["dm"] = cal["acc"]
            b = 0.5 * (cal["dp"] - cal["dm"])
            cal["b"] = b
            cal["sign"] = 1.0 if b >= 0.0 else -1.0
            cal["phase"] = "done"
            log("%s steer calibration (%.2f s trials): d(+%.1f)=%+.4f d(-%.1f)=%+.4f rad -> b=%+.4f sign=%+.0f"
                % (o.id, cal["trial_s"], CAL_STEER, cal["dp"], CAL_STEER, cal["dm"], b, cal["sign"]))
            return None
        return CAL_STEER if cal["phase"] == "plus" else -CAL_STEER

    def spine_geometry(self, o):
        """(tail_root, head_root, spine_dir (unit tail->head), L) from this
        tick's poses, or None. L = tail FACE to head ROOT along the spine."""
        tp, hp = self.cell_pose(o.spine[0]), self.cell_pose(o.head)
        if tp is None or hp is None:
            return None
        if len(o.spine) == 1:
            ux, uy = math.cos(tp[2]), math.sin(tp[2])
        else:
            sx, sy = hp[0] - tp[0], hp[1] - tp[1]
            m = math.hypot(sx, sy)
            if m < 1e-6:
                ux, uy = math.cos(tp[2]), math.sin(tp[2])
            else:
                ux, uy = sx / m, sy / m
        L = (len(o.spine) - 1) * (C.CELL_LENGTH + DOCK_GAP) + C.HALF
        return tp, hp, (ux, uy), L

    def runway(self, fp, L):
        """The trailer geometry for a free cell at pose fp: N (its nose face),
        n (nose normal), R1, R2 (head aim points on the normal beyond it)."""
        nx_, ny_, nyaw = ORG.face_pose(fp, "f_nose")
        n = (math.cos(nyaw), math.sin(nyaw))
        R1 = (nx_ + n[0] * (L + RUNWAY_R1_M), ny_ + n[1] * (L + RUNWAY_R1_M))
        R2 = (nx_ + n[0] * (L + RUNWAY_R2_M), ny_ + n[1] * (L + RUNWAY_R2_M))
        return (nx_, ny_), n, R1, R2

    def _fail_attempt(self, o, st, d, j, why):
        d["attempts"] += 1
        d["captures"] = 0
        d.pop("cap_root", None)
        d.pop("cap_face", None)
        self.dock_stats["attempts"] += 1
        st["reverse"] = False
        if d["attempts"] >= MAX_DOCK_ATTEMPTS:
            st.setdefault("blacklist", {})[j] = self.t + BLACKLIST_S
            self.forced.pop(o.id, None)
            log("%s gave up on CELL_%d after %d attempts (%s); blacklisted %.0f s"
                % (o.id, j, d["attempts"], why, BLACKLIST_S))
            st["dock"] = None
            return
        d["state"], d["phase"], d["until"] = "backoff", "backoff", self.t + BACKOFF_S
        log("%s dock attempt %d failed (%s): head-first away for %.0f s, then the runway again"
            % (o.id, d["attempts"], why, BACKOFF_S))

    def recruit_target_of(self, o):
        """The free cell this organism is trying to dock, forced (/recruit)
        before the ecology's own choice; None when there is none."""
        f = self.forced.get(o.id)
        if f is not None:
            c = self.reef.cells[f["cell"]]
            if c.free and self.t - f["since"] < RECRUIT_TIMEOUT_S and f["cell"] in self.positions:
                return f["cell"]
            log("%s forced recruit of CELL_%d dropped (%s)" % (
                o.id, f["cell"], "docked/not free" if not c.free else "timeout"))
            del self.forced[o.id]
        j = o.recruit_target
        if j is not None and self.reef.cells[j].free and j in self.positions:
            return j
        return None

    def docking(self, o, st, hp, j):
        """One tick of the trailer manoeuvre onto free cell j. Returns
        (aim_xy, reverse): the point the head steers at (None = no steering)
        and whether the wave runs tail-first this tick.

        States: runway (head-first: aim R1, then R2; the moment the tail face
        is within ALIGN_TAIL_LATERAL_M of the free nose normal, the spine is
        within ALIGN_SPINE_RAD of it and the tail is beyond the nose ->
        reverse), go_around (the head overshot R2 unaligned: loop back to R1
        via a side point), reverse (+|dphi|, no steering; capture assist
        inside CAPTURE_M / CAPTURE_AXIS draws the inert free cell onto the
        tail face; lock when inside the engine tolerance), verify (still
        reversing VERIFY_S; faces within VERIFY_SEP -> Reef.recruit(at_tail),
        else release + backoff), backoff (head-first away for BACKOFF_S)."""
        geo = self.spine_geometry(o)
        fp = self.cell_pose(j)
        if geo is None or fp is None:
            return None, False
        tp, _hp, (ux, uy), L = geo
        tail = o.spine[0]
        d = st["dock"]
        if d is None or d["cell"] != j:
            d = {"cell": j, "tail": tail, "state": "runway", "phase": "to_R1", "since": self.t,
                 "attempts": 0, "until": 0.0, "lock_t": None, "sep": None, "axis_err": None,
                 "captures": 0}
            st["dock"] = d
            self.dock_stats["attempts"] += 1
            log("%s docking CELL_%d: runway (L=%.2f m, tail=CELL_%d)" % (o.id, j, L, tail))
        d["tail"] = tail
        N, n, R1, R2 = self.runway(fp, L)
        px, py = -n[1], n[0]                               # perp (left of n)
        # tail FACE position from the tail root (the f_tail origin is -HALF along the cell's +x)
        tfx, tfy = tp[0] - C.HALF * math.cos(tp[2]), tp[1] - C.HALF * math.sin(tp[2])
        rx, ry = tfx - N[0], tfy - N[1]
        tail_along, tail_lat = rx * n[0] + ry * n[1], rx * px + ry * py
        hx_, hy_ = hp[0] - N[0], hp[1] - N[1]
        head_along, head_lat = hx_ * n[0] + hy_ * n[1], hx_ * px + hy_ * py
        spine_err = abs(wrap(math.atan2(uy, ux) - math.atan2(n[1], n[0])))
        d.update({"along": tail_along, "lateral": tail_lat, "head_along": head_along,
                  "head_lateral": head_lat, "spine_err": spine_err, "dist_goal": ORG.distance_xy(tp, fp),
                  "R1": [round(R1[0], 3), round(R1[1], 3)], "R2": [round(R2[0], 3), round(R2[1], 3)]})
        # ...and only from the near end of the runway: a body 2.5 m out that
        # happens to be aligned has 2.5 m of unsteered reverse ahead of it
        aligned = (abs(tail_lat) <= ALIGN_TAIL_LATERAL_M and spine_err <= ALIGN_SPINE_RAD
                   and ALIGN_TAIL_ALONG_M <= tail_along <= RUNWAY_FAR_M)

        if d["state"] == "backoff":
            if self.t < d["until"]:
                return R2, False
            d["state"], d["phase"] = "runway", "to_R1"
            log("%s runway again (attempt %d)" % (o.id, d["attempts"] + 1))

        if d["state"] == "runway":
            if aligned:
                d["state"], d["phase"], d["until"] = "reverse", "reverse", self.t + REVERSE_TIMEOUT_S
                d["reverse_t0"] = self.t
                st["reverse"] = True
                self.ramp(o.id)
                log("%s ALIGNED: tail face lat %+.3f along %.3f spine_err %.2f rad -> REVERSE (+dphi, no steer)"
                    % (o.id, tail_lat, tail_along, spine_err))
                return None, True
            # RUNWAY, redesigned after four measured loops of "at R1 unaligned ->
            # overshoot -> go-around": with a ~1 m turn radius the old R1/R2 pair
            # (0.65 m apart, near point FIRST) could never straighten the body.
            # Now: (far) drive to R_FAR well out on the free cell's nose normal,
            # then (line) follow the normal back toward the cell -- the aim is a
            # point on the axis LOOKAHEAD ahead of the head's projection, so the
            # body converges onto the line -- and the aligned test above fires
            # near R1. Passing R1 still unaligned sends it back out to R_FAR.
            # RUNWAY v3 (measured twice): line-following INTO the cell converges
            # beautifully -- and arrives head-first, i.e. facing the wrong way to
            # back in (spine_err ~2.5 rad at R1 every time). A trailer is parked
            # by driving PAST the bay and reversing, so: (near) drive to P_NEAR,
            # a point on the free cell's nose normal just outside the bay; then
            # (line_out) follow the axis OUTWARD -- the aim is a point on the axis
            # LOOKAHEAD beyond the head's projection -- which straightens the body
            # with its tail toward the cell; the aligned test above then fires
            # and the reverse leg backs straight in.
            P_NEAR = (N[0] + n[0] * (L + RUNWAY_NEAR_M), N[1] + n[1] * (L + RUNWAY_NEAR_M))
            if d["phase"] not in ("near", "line_out"):
                d["phase"] = "near"
            if d["phase"] == "near":
                if ORG.distance_xy(hp, P_NEAR) <= P_NEAR_REACHED_M:
                    d["phase"] = "line_out"
                    log("%s at P_NEAR (head lat %+.2f along %.2f, spine_err %.2f): following the axis OUT"
                        % (o.id, head_lat, head_along, spine_err))
                else:
                    return P_NEAR, False
            if d["phase"] == "line_out":
                if head_along > L + RUNWAY_FAR_M + LINE_ABORT_ALONG_M or abs(head_lat) > LINE_ABORT_LATERAL_M:
                    d["phase"] = "near"
                    log("%s ran out of runway unaligned (head lat %+.2f along %.2f, spine_err %.2f): back to P_NEAR"
                        % (o.id, head_lat, head_along, spine_err))
                    return P_NEAR, False
                s_aim = head_along + LOOKAHEAD_LINE_M
                st["line_follow"] = True
                return (N[0] + n[0] * s_aim, N[1] + n[1] * s_aim), False
            return P_NEAR, False

        if d["state"] in ("reverse", "verify"):
            # abort conditions
            if d["state"] == "reverse":
                if abs(tail_lat) > REVERSE_ABORT_LATERAL_M or tail_along < -0.05 or self.t > d["until"]:
                    self._fail_attempt(o, st, d, j, "reverse drifted (tail lat %+.2f along %.2f) or timed out"
                                       % (tail_lat, tail_along))
                    return R2, False
            if d["state"] == "reverse":
                # closed-loop reverse: the tail face pursues the axis toward N
                s_aim = max(0.0, tail_along - REV_LOOKAHEAD_M)
                aim_x, aim_y = N[0] + n[0] * s_aim, N[1] + n[1] * s_aim
                rev_heading = wrap(math.atan2(uy, ux) + math.pi)        # the tail travels along -spine
                rev_err = ORG.heading_error((tfx, tfy, rev_heading), (aim_x, aim_y))
                # SIGN MEASURED on the engine (probe_dock --behind, 2026-08-29): the
                # tail moves the OTHER way from the head for a given rudder in
                # reverse -- +0.6 took the tail from +0.08 to +0.39 m off the axis.
                st["rev_steer"] = -ORG.clamp(ORG.steer_from_error(rev_err, err_full=REV_ERR_FULL_RAD),
                                             -REV_STEER_MAX, REV_STEER_MAX)
                d["rev_err"] = rev_err
            if ORG.distance_xy(tp, fp) > FACE_CHECK_M and d["state"] == "reverse":
                return None, True
            (ax, ay, ayaw), _pa = self.cells[tail].face_pose2d("f_tail")
            (bx, by, byaw), _pb = self.cells[j].face_pose2d("f_nose")
            sep = math.hypot(ax - bx, ay - by)
            axis_err = abs(wrap(ayaw - byaw - math.pi))
            d["sep"], d["axis_err"] = sep, axis_err
            if d["state"] == "reverse":
                # CAPTURE ASSIST. A reversing chain cannot hold a 3 cm / 0.45 rad
                # tolerance by itself (measured). Real docking mechanisms capture
                # with magnets or guide cones over a short range; here, inside
                # CAPTURE_M and roughly aligned, the INERT free cell is drawn onto
                # the tail socket: its nose face DOCK_GAP in front of the tail face
                # with the normals opposed, so its ROOT (the tail block, 0.09 m
                # behind its nose face at hinge 0) sits at
                #   tail_face + m*(DOCK_GAP + NOSE_FACE_X), yaw = tail normal + pi
                # (m = the tail face's outward normal). Bounded: once per attempt,
                # counted in dock_stats["captures"]; the engine's own weld
                # tolerance still decides whether the lock takes.
                if (sep <= CAPTURE_M and axis_err <= CAPTURE_AXIS and (sep > LOCK_SEP or axis_err > LOCK_AXIS)
                        and d["captures"] == 0):
                    mx, my = math.cos(ayaw), math.sin(ayaw)
                    off = DOCK_GAP + C.NOSE_FACE_X
                    root = (ax + mx * off, ay + my * off, wrap(ayaw + math.pi))
                    self.cells[j].teleport(*root)
                    d["cap_root"] = root
                    d["cap_face"] = (ax + mx * DOCK_GAP, ay + my * DOCK_GAP)   # where the nose face should land
                    d["captures"] += 1
                    self.dock_stats["captures"] = self.dock_stats.get("captures", 0) + 1
                    log("%s capture assist: CELL_%d drawn onto CELL_%d.f_tail (sep %.3f, axis %.2f rad)"
                        % (o.id, j, tail, sep, axis_err))
                    d["lock_after"] = self.tick + CAPTURE_SETTLE_TICKS
                    return None, True
                # STALE-POSE WELD (measured 5x, epochs 9-12): every lock written after
                # a capture with sep > 0.011 welded the cell 0.12 m off; every one at
                # <= 0.0097 held (and un-captured locks at 0.03 hold). So after a
                # capture, re-capture rather than lock a drifted gap.
                if (d["captures"] and "cap_face" in d and CAPTURE_LOCK_SEP < sep <= CAPTURE_M
                        and d["captures"] < 4 and self.tick >= d.get("lock_after", 0)):
                    # the landed face is off by a residual (measured 2-10 mm); the
                    # next teleport is the intended root minus that residual
                    tx, ty = d["cap_face"]
                    rx, ry = bx - tx, by - ty
                    rx0, ry0, ryaw = d["cap_root"]
                    root = (rx0 - rx, ry0 - ry, ryaw)
                    self.cells[j].teleport(*root)
                    d["cap_root"] = root
                    log("%s re-capture %d: nose face landed %.1f mm off (residual %+.4f, %+.4f), sep %.4f"
                        % (o.id, d["captures"], 1000.0 * math.hypot(rx, ry), rx, ry, sep))
                    d["captures"] += 1
                    d["lock_after"] = self.tick + CAPTURE_SETTLE_TICKS
                    self.dock_stats["recaptures"] = self.dock_stats.get("recaptures", 0) + 1
                    return None, True
                if sep <= LOCK_SEP and axis_err <= LOCK_AXIS and self.tick >= d.get("lock_after", 0):
                    self.cells[j].lock("f_nose", True)
                    self.welds.add((j, "f_nose", tail))
                    self.dock_stats["locks_written"] += 1
                    d["state"], d["phase"], d["lock_t"] = "verify", "verify", self.t
                    log("%s lock written: CELL_%d.f_nose -> CELL_%d.f_tail (sep %.4f, axis %.3f rad)"
                        % (o.id, j, tail, sep, axis_err))
                return None, True
            # verify: keep reversing, the faces must stay together
            held = self.t - d["lock_t"]
            if held < VERIFY_S and sep <= 1.5 * VERIFY_SEP:
                return None, True          # judged at VERIFY_S; only a gross gap fails early
            if sep <= VERIFY_SEP:
                try:
                    acts = self.reef.recruit(o.id, j, at_tail=True)
                except (ValueError, TypeError) as exc:
                    log("%s recruit of CELL_%d refused by the ecology: %s" % (o.id, j, exc))
                    acts = None
                if acts is not None:
                    for a in acts:
                        self.execute(a)
                    self.dock_stats["recruits"] += 1
                    self.forced.pop(o.id, None)
                    self.free_relaxed.discard(j)
                    st["reverse"] = False
                    self.ramp(o.id)
                    log("%s RECRUITED CELL_%d at the tail, t=%.2f (junction sep %.4f m held %.2f s, %d attempt(s), "
                        "spine %s)" % (o.id, j, self.t, sep, held, d["attempts"] + 1, o.spine))
                    st["dock"] = None
                    return None, False
            self.cells[j].lock("f_nose", False)
            self.welds.discard((j, "f_nose", tail))
            self.dock_stats["failed_verify"] += 1
            self._fail_attempt(o, st, d, j, "verify: sep %.4f > %.3f after %.2f s" % (sep, VERIFY_SEP, held))
            return R2, False
        return None, False

    def clamp_aim(self, aim):
        if aim is None:
            return None
        h = self.ARENA / 2.0 - WALL_MARGIN_M * 0.6
        return (ORG.clamp(float(aim[0]), -h, h), ORG.clamp(float(aim[1]), -h, h))

    def wall_ahead(self, hp, st):
        """(nx, ny, dist): the nearest arena wall within WALL_LOOK_M that the
        measured heading points at (outward normal, distance), else None."""
        h = self.ARENA / 2.0
        heading = st["heading"] if st["heading"] is not None else hp[2]
        cx, cy = math.cos(heading), math.sin(heading)
        best = None
        for nx_, ny_, d_, into in ((1.0, 0.0, h - hp[0], cx), (-1.0, 0.0, hp[0] + h, -cx),
                                   (0.0, 1.0, h - hp[1], cy), (0.0, -1.0, hp[1] + h, -cy)):
            if d_ < WALL_LOOK_M and into > 0.2 and (best is None or d_ < best[2]):
                best = (nx_, ny_, d_)
        return best

    def dockable(self, j):
        """A free cell whose docking runway (the turn-in point beyond its nose,
        for a typical 0.7 m body) lies outside the arena cannot be docked: the
        organism would have to stand inside the wall to line up on it."""
        fp = self.cell_pose(j)
        if fp is None:
            return False
        nx_, ny_, nyaw = ORG.face_pose(fp, "f_nose")
        # the WHOLE runway: the organism drives to the far end of the axis
        # (L + RUNWAY_FAR_M beyond the nose) and reverses down it; measured
        # (epoch 7, 12 m arena): checking only the turn-in point let bodies
        # chase cells whose runway ended in a wall -- 15 wall events, 0 locks
        r = 0.7 + 3.0                  # the runway a body actually needs to line up (not the full window)
        px, py = nx_ + math.cos(nyaw) * r, ny_ + math.sin(nyaw) * r
        h = self.ARENA / 2.0 - 0.3
        return abs(px) <= h and abs(py) <= h

    def drive_organism(self, o):
        st = self.org_state(o.id)
        hp = self.head_pose(o)
        if hp is None:
            return
        st["line_follow"] = False
        self.update_travel(o, st)
        forced_steer = self.calibrate(o, st, hp)
        steer = 0.0
        reverse = False
        j = self.recruit_target_of(o)
        if j is not None:
            aim, reverse = self.docking(o, st, hp, j)
        else:
            if st.get("dock") is not None:
                st["dock"] = None
            st["reverse"] = False
            aim = self.targets.get(o.id)
        # WALL AVOIDANCE + STUCK RECOVERY (see the constants). Aim points are
        # clamped into the arena; a head inside WALL_MARGIN_M of a wall and
        # heading at it aims at the centre instead; a body driving forward at
        # under STUCK_SPEED for STUCK_S backs off in reverse gear for UNSTICK_S.
        aim = self.clamp_aim(aim)
        st["mode"] = ""
        bask = False
        if o.state == "roam" and aim is not None and j is None:
            da = ORG.distance_xy(hp, aim)
            if st.get("basking"):
                bask = da <= BASK_LEAVE_M
            else:
                bask = da <= BASK_M
            if bask and not st.get("basking"):
                self.dock_stats["basks"] = self.dock_stats.get("basks", 0) + 1
                log("%s basking on the patch at (%.2f, %.2f)" % (o.id, aim[0], aim[1]))
            st["basking"] = bask
        else:
            st["basking"] = False
        if bask:
            aim = None
            st["mode"] = "BASK"
        if not reverse:
            # WALL STATE MACHINE (measured: a margin-only rule limit-cycled at
            # 0.87 m off the wall for 300 s -- reverse until clear of the
            # margin, forward straight back into it). None -> "rev" (inside
            # WALL_MARGIN_M and heading at the wall; back off until
            # WALL_BACKOFF_M, a rudder needs way on) -> "turn" (aim at the
            # centre, turn commitment does the rest) -> None once the heading
            # no longer points at a wall within WALL_LOOK_M.
            wall = self.wall_ahead(hp, st)
            ws = st.get("wall_state")
            if ws is None and wall is not None and wall[2] < WALL_MARGIN_M:
                ws = "rev" if wall[2] < WALL_BACKOFF_M else "turn"
                st["turn_dir"] = None
                self.dock_stats["wall_events"] = self.dock_stats.get("wall_events", 0) + 1
                log("%s at the wall (%.2f, %.2f) heading %.2f, %.2f m off it: %s"
                    % (o.id, hp[0], hp[1], st["heading"] if st["heading"] is not None else hp[2],
                       wall[2], "backing off" if ws == "rev" else "turning to the centre"))
            if ws == "rev":
                if wall is None or wall[2] >= WALL_BACKOFF_M:
                    ws = "turn"
                    st["turn_dir"] = None
                else:
                    reverse, aim = True, None
                    st["mode"] = "WALL_REV"
            if ws == "turn":
                if wall is None:
                    ws = None
                elif wall[2] < WALL_BACKOFF_M * 0.4:
                    ws = "rev"
                    reverse, aim = True, None
                    st["mode"] = "WALL_REV"
                else:
                    aim = (0.0, 0.0)
                    st["mode"] = "WALL"
            st["wall_state"] = ws
        if self.t < st.get("kturn_until", 0.0):
            da = ORG.distance_xy(hp, aim) if aim is not None else None
            if da is not None and (da >= KTURN_CLEAR_M or da <= P_NEAR_REACHED_M * 0.7):
                st["kturn_until"] = 0.0          # geometry changed: resume
            else:
                reverse, aim = True, None
                st["mode"] = "KTURN"
        elif (not reverse and aim is not None and st["heading"] is not None
              and not st.get("line_follow")     # line_out from a wrong-way arrival IS a U-turn; let it turn
              and self.t >= st.get("kturn_next", 0.0)
              and ORG.distance_xy(hp, aim) < KTURN_DIST_M
              and abs(ORG.heading_error((hp[0], hp[1], st["heading"]), aim)) > KTURN_ERR_RAD):
            st["kturn_until"] = self.t + KTURN_S
            st["kturn_next"] = self.t + KTURN_S + KTURN_COOLDOWN_S
            st["turn_dir"] = None
            self.dock_stats["kturns"] = self.dock_stats.get("kturns", 0) + 1
            log("%s K-TURN: aim %.2f m away, %.2f rad off the heading -> reverse %.0f s"
                % (o.id, ORG.distance_xy(hp, aim),
                   ORG.heading_error((hp[0], hp[1], st["heading"]), aim), KTURN_S))
            reverse, aim = True, None
            st["mode"] = "KTURN"
        if self.t < st.get("unstick_until", 0.0):
            reverse, aim = True, None
            st["mode"] = "UNSTICK"
        elif not reverse:
            # STUCK: head displacement over a STUCK_S window (the 1 s speed is
            # dominated by the wobble of an undulating body pressed on something)
            ref = st.get("stuck_ref")
            if ref is None or self.t - st["ramp_t0"] < FADE_S + 2.0:
                st["stuck_ref"] = (self.t, hp[0], hp[1])
            elif self.t - ref[0] >= STUCK_S:
                moved = math.hypot(hp[0] - ref[1], hp[1] - ref[2])
                st["stuck_ref"] = (self.t, hp[0], hp[1])
                if moved < STUCK_SPEED * STUCK_S:
                    st["unstick_until"] = self.t + UNSTICK_S
                    st["turn_dir"] = None
                    self.dock_stats["unsticks"] = self.dock_stats.get("unsticks", 0) + 1
                    log("%s STUCK (%.2f m in %.0f s at (%.2f, %.2f)): reverse gear for %.0f s"
                        % (o.id, moved, STUCK_S, hp[0], hp[1], UNSTICK_S))
                    reverse, aim = True, None
                    st["mode"] = "UNSTICK"
        else:
            st["stuck_ref"] = None
        if reverse:
            d_ = st.get("dock") or {}
            steer = st.get("rev_steer", 0.0) if (d_.get("state") == "reverse" and st["mode"] == "") else 0.0
        elif forced_steer is not None:
            steer = forced_steer
        elif aim is not None:
            # steer on the MEASURED heading (spine axis, signed by travel)
            head_for_error = (hp[0], hp[1], st["heading"] if st["heading"] is not None else hp[2])
            err = ORG.heading_error(head_for_error, aim)
            # TURN COMMITMENT: a target dead astern wraps err at +-pi and the
            # proportional law flips the rudder every sample (measured, above).
            td = st.get("turn_dir")
            if td is None and abs(err) >= TURN_COMMIT_RAD:
                td = 1.0 if err >= 0.0 else -1.0
            elif td is not None and abs(err) < TURN_RELEASE_RAD:
                td = None
            st["turn_dir"] = td
            if td is not None:
                steer = st["cal"]["sign"] * td
            else:
                err_full = LINE_ERR_FULL_RAD if st.pop("line_follow", False) else ORG.STEER_ERR_FULL
                steer = ORG.steer_from_error(err, err_full=err_full, sign=st["cal"]["sign"])
            # POLARITY WATCH (the alife lesson, applied online): the wiggle
            # calibration is weak on some bodies (b ~ +0.07 rad). If the heading
            # error keeps GROWING while the command is saturated, the sign is
            # wrong -- flip it. Measured: a recruiter sat 156 s in standoff at
            # 0.007 m/s pivoting away from its target.
            pw = st.setdefault("polarity", {"t0": None, "e0": None})
            if st.get("rudder") is not None:
                st["cal"]["sign"] = 1.0          # measured convention for a head rudder (probe_wave)
                pw["t0"] = None
            elif abs(steer) >= 0.99:
                if pw["t0"] is None:
                    pw["t0"], pw["e0"] = self.t, abs(err)
                elif self.t - pw["t0"] >= POLARITY_WINDOW_S:
                    if abs(err) > pw["e0"] + 0.15:
                        st["cal"]["sign"] = -st["cal"]["sign"]
                        log("%s polarity flipped -> %+d (|err| %.2f -> %.2f rad over %.0f s at full steer)"
                            % (o.id, int(st["cal"]["sign"]), pw["e0"], abs(err), POLARITY_WINDOW_S))
                    pw["t0"], pw["e0"] = self.t, abs(err)
            else:
                pw["t0"] = None
            st["err"] = err
        st["steer"] = steer
        st["aim"] = aim
        st["reverse"] = bool(reverse)
        # HEAD RUDDER + PITCH WAVE (probe_wave, measured). The wave is computed
        # for an all-pitch spine and travels toward the LOW-index end of its
        # phase ramp: -|dphi| moves the chain head-first, +|dphi| tail-first
        # (the reverse gear). Every MEASURED yaw cell is then overridden: the
        # head is the rudder (bias_yaw + steer_gain*steer), any other yaw cell
        # is held at bias_yaw -- neither carries the wave.
        bp = self.effective_bodyplan(o, st)
        all_pitch = dict(bp, dock_rotation_pattern=[0])
        branches = [side for (_k, side) in sorted(o.branches)]
        buf = self._tbuf.setdefault(o.id, [])
        wave = dict(o.genome)
        wave["dphi"] = abs(o.genome["dphi"]) if reverse else -abs(o.genome["dphi"])
        if st.get("basking") and not reverse:
            wave["A"] = 0.0
        if reverse:
            # fast reverse on the runway, the genome's own amplitude inside the
            # bay: a lock written at 12 mm under the A 1.2 reverse read 122 mm
            # 0.22 s later (probe_dock --behind), while every lock under A 0.9 held
            d_ = st.get("dock") or {}
            near_bay = d_.get("state") in ("reverse", "verify") and (d_.get("along") or 9.0) < REV_GENTLE_ALONG_M
            wave["A"] = o.genome["A"] if near_bay else max(o.genome["A"], REV_A)
        # TURN RADIUS = speed / yaw rate. The rudder's yaw rate is fixed by the
        # physics, so a hard turn must SLOW the wave: measured, at full speed the
        # body orbits a target inside its ~1 m turning circle forever (26 trace
        # rows circling P_NEAR at 0.75-2.5 m). Scaling the amplitude down with
        # |steer| roughly halves the radius at full lock.
        if not reverse:
            wave["A"] = o.genome["A"] * (1.0 - TURN_SLOWDOWN * min(1.0, abs(steer)))
        targets = ORG.chain_targets(wave, all_pitch, len(o.spine), self.t, 0.0,
                                    branches=branches, out=buf)
        g = o.genome
        # RUDDER_MAX_RAD: measured (probe_wave, 6 cells) a 1.0 rad head is an
        # anchor -- half the speed, half the yaw rate of 0.5 rad.
        rudder = ORG.clamp(g["bias_yaw"] + g["steer_gain"] * steer, -RUDDER_MAX_RAD, RUDDER_MAX_RAD)
        n_spine = len(o.spine)
        for k in range(n_spine):
            if self.axis_z.get(o.spine[k], 0.0) > 0.7:
                targets[k] = rudder if k >= n_spine - RUDDER_CELLS else g["bias_yaw"]
        head_up = self.axis_z.get(o.head, 0.0)
        # hysteresis + persistence: the A 1.2 reverse rocks a 4-cell body's head
        # to |z| 0.63-0.72 for a sample or two (measured, division probe: 47
        # spurious "lost" flips in 240 s); a rudder is lost below 0.6 for 1 s
        # and back above 0.75
        low_since = st.get("rudder_low_since")
        if head_up < 0.6:
            if low_since is None:
                st["rudder_low_since"] = self.t
            elif self.t - low_since >= 1.0:
                st["rudder_down"] = True
        else:
            st["rudder_low_since"] = None
            if head_up > 0.75:
                st["rudder_down"] = False
        st["rudder"] = None if st.get("rudder_down") else rudder
        # RUDDER WATCH: a head cell knocked off its side (hinge axis no longer
        # vertical) leaves the body with no steering at all -- log the
        # transitions, they explain every "polarity flipped" on a rudder body.
        had = st.get("rudder_ok")
        has = st["rudder"] is not None
        if had is not None and had != has:
            self.dock_stats["rudder_lost" if not has else "rudder_back"] =                 self.dock_stats.get("rudder_lost" if not has else "rudder_back", 0) + 1
            log("%s RUDDER %s: head CELL_%d hinge-axis |z| = %.2f at t=%.1f"
                % (o.id, "LOST" if not has else "back", o.head, head_up, self.t))
        st["rudder_ok"] = has
        st["head_up"] = head_up
        fade = min(1.0, max(0.0, (self.t - st["ramp_t0"]) / FADE_S)) if FADE_S > 0 else 1.0
        members = o.members()
        for idx, i in enumerate(members):
            v = targets[idx] * fade if idx < len(targets) else 0.0
            cn = self.cells.get(i)
            if cn is not None and cn.pos_field is not None:
                cn.set_hinge(v)

    def relax_free_cells(self):
        """Free cells are inert. A cell that just became free (shed, division
        rear tail, death) still holds its last gait target; write 0 ONCE so its
        faces sit where approach_pose assumes."""
        for c in self.reef.cells:
            if c.free and c.i not in self.free_relaxed:
                self.cells[c.i].set_hinge(0.0)
                self.free_relaxed.add(c.i)
            elif not c.free:
                self.free_relaxed.discard(c.i)

    # ------------------------------------------------------------- commands
    def executor(self, cmd, tick):
        a = cmd.args
        R = self.reef
        if cmd.verb == "dim":
            before = R.dim
            R.set_dim(a["factor"])
            return 200, {"ok": True, "dim": R.dim, "previous": before, "tick": tick}
        if cmd.verb == "light":
            k = int(a["k"])
            px, py = R.place_light(k, a["x"], a["y"])
            fld = self.patch_tr[k]
            if fld is None:
                return 500, {"ok": False, "error": "patch_unresolved", "k": k}
            fld.setSFVec3f([px, py, self.patch_z[k]])
            node = self.patch_nodes[k]

            def fin(k=k, px=px, py=py):
                p = node.getPosition()
                return 200, {"ok": True, "k": k, "position_measured": [round(v, 4) for v in p],
                             "requested": [a["x"], a["y"]],
                             "clamped": abs(px - a["x"]) > 1e-9 or abs(py - a["y"]) > 1e-9,
                             "lit_cells": [c.i for c in R.cells if c.alive and c.i in self.positions
                                           and math.hypot(self.positions[c.i][0] - p[0],
                                                          self.positions[c.i][1] - p[1]) <= ECO.PATCH_RADIUS]}
            return self.queue.defer(cmd, tick + VERIFY_TICKS, fin)
        oid = str(a["organism"])
        o = R.organisms.get(oid)
        if o is None:
            return 404, {"ok": False, "error": "no_such_organism", "organism": oid,
                         "valid": sorted(R.organisms)}
        if cmd.verb == "split":
            if len(o.spine) < 2:
                return 409, {"ok": False, "error": "too_short", "organism": oid, "spine": list(o.spine)}
            k = len(o.spine) // 2
            rear_c, front_c = o.spine[k - 1], o.spine[k]
            rear, front, acts = R.divide(oid)
            for act in acts:
                self.execute(act)
            self.forced.pop(oid, None)
            self.org.pop(oid, None)
            for kid in (rear, front):
                self.org_state(kid.id)            # fresh fade-in + calibration

            def fin(rear=rear, front=front, rear_c=rear_c, front_c=front_c):
                (ax, ay, _), _pa = self.cells[rear_c].face_pose2d("f_nose")
                (bx, by, _), _pb = self.cells[front_c].face_pose2d("f_tail")
                return 200, {"ok": True, "parent": oid, "split_at": k,
                             "children": [rear.id, front.id],
                             "spines": {rear.id: list(rear.spine), front.id: list(front.spine)},
                             "junction_sep_measured": round(math.hypot(ax - bx, ay - by), 4),
                             "heads_measured": {c.id: [round(v, 4) for v in self.positions[c.head]]
                                                if c.head in self.positions else None
                                                for c in (rear, front)},
                             "genomes_mutated": True, "tick": tick}
            return self.queue.defer(cmd, tick + VERIFY_TICKS, fin)
        if cmd.verb == "recruit":
            j = int(a["cell"])
            if not (0 <= j < R.n_cells):
                return 404, {"ok": False, "error": "no_such_cell", "cell": j,
                             "valid": "0..%d" % (R.n_cells - 1)}
            c = R.cells[j]
            if not c.free:
                return 409, {"ok": False, "error": "cell_not_free", "cell": j,
                             "alive": c.alive, "organism": c.organism}
            if len(o) >= o.target_length:
                return 409, {"ok": False, "error": "at_target_length", "organism": oid,
                             "length": len(o), "target_length": o.target_length}
            self.forced[oid] = {"cell": j, "since": self.t}
            self.dock_stats["forced"] += 1
            st = self.org_state(oid)
            st["dock"] = None
            st.get("blacklist", {}).pop(j, None)

            def fin(j=j):
                geo, fp = self.spine_geometry(o), self.cell_pose(j)
                if geo is None or fp is None:
                    return 409, {"ok": False, "error": "unmeasured", "organism": oid, "cell": j}
                tp, hp, _u, L = geo
                N, n, R1, R2 = self.runway(fp, L)
                return 200, {"ok": True, "accepted": True, "organism": oid, "cell": j,
                             "tail": o.spine[0], "tail_face": "f_tail", "recruit_face": "f_nose",
                             "distance_measured": round(ORG.distance_xy(tp, fp), 4),
                             "runway": {"nose_face": [round(v, 4) for v in N],
                                        "normal": [round(v, 4) for v in n],
                                        "R1": [round(v, 4) for v in R1], "R2": [round(v, 4) for v in R2],
                                        "L_org": round(L, 4)},
                             "state": "runway",
                             "note": "trailer manoeuvre: head-first to R1 then R2, reverse tail-first onto "
                                     "the nose, capture assist, lock, verify, recruit at the tail; watch "
                                     "/census organisms[%s].dock" % oid}
            return self.queue.defer(cmd, tick + VERIFY_TICKS, fin)
        return 400, {"ok": False, "error": "unknown_verb", "verb": cmd.verb}

    # ------------------------------------------------------------ reporting
    def step_median(self):
        w = sorted(self.engine_ms[40:] or self.engine_ms)
        return round(w[len(w) // 2], 3) if w else None

    def organisms_measured(self):
        out = {}
        for oid, o in self.reef.organisms.items():
            st = self.org.get(oid)
            if st is None:
                continue
            d = st.get("dock")
            out[oid] = {
                "speed": round(st["speed"], 4) if st.get("speed") is not None else None,
                "heading": round(st["heading"], 4) if st.get("heading") is not None else None,
                "travel_vs_nose": round(st["travel_vs_nose"], 3) if st.get("travel_vs_nose") is not None else None,
                "steer_sign": st["cal"]["sign"], "steer_b": st["cal"]["b"],
                "cal_phase": st["cal"]["phase"], "steer": round(st.get("steer", 0.0), 3),
                "aim": [round(v, 3) for v in st["aim"]] if st.get("aim") else None,
                "fade": round(min(1.0, (self.t - st["ramp_t0"]) / FADE_S), 3) if FADE_S > 0 else 1.0,
                "upright": all(self.ups.get(i, 1.0) >= UPRIGHT_MIN for i in o.members()),
                "wave_speed_estimate": round(ORG.wave_speed_estimate(o.genome), 4),
                "forced_recruit": self.forced.get(oid, {}).get("cell"),
                "reverse": bool(st.get("reverse")),
                "rudder": round(st["rudder"], 3) if st.get("rudder") is not None else None,
                "dock": None if d is None else {k: (round(v, 4) if isinstance(v, float) else v)
                                                for k, v in d.items() if k != "G"},
            }
        return out

    def snapshot(self):
        snap = self.reef.snapshot(self.positions)
        snap["organisms_measured"] = self.organisms_measured()
        snap["engine_ms_per_step_median"] = self.step_median()
        snap["dock_stats"] = dict(self.dock_stats)
        snap["bridge"] = dict(self.queue.stats)
        snap["welds_held"] = len(self.welds)
        snap["missing_defs"] = self.missing
        snap["dt_ms"] = self.DT
        return snap

    def finish_epoch(self):
        res = self.reef.epoch_result()
        res.update({"ticks": self.tick, "dt_ms": self.DT, "missing_defs": self.missing,
                    "engine_ms_per_step_median": self.step_median(),
                    "dock_stats": dict(self.dock_stats), "bridge": dict(self.queue.stats),
                    "organisms_measured": self.organisms_measured(),
                    "steer_signs": {oid: st["cal"]["sign"] for oid, st in self.org.items()}})
        write_json(os.path.join(self.run_dir, "epoch_result.json"), res)
        log("epoch done at %.1fs: divisions=%d recruits=%d sheds=%d deaths=%d recycles=%d wd=%d -> %s"
            % (self.t, self.reef.divisions, self.reef.recruits, self.reef.sheds, self.reef.deaths,
               self.reef.recycles, self.reef.watchdog_kills, os.path.join(self.run_dir, "epoch_result.json")))
        self.epoch_written = True

    # ------------------------------------------------------------ the tick
    def tick_once(self):
        """One engine step + everything that follows it. Returns False when
        the run is over."""
        ts = time.perf_counter()
        if self.sup.step(self.DT) == -1:
            if not self.epoch_written:
                self.finish_epoch()
            return False
        self.engine_ms.append((time.perf_counter() - ts) * 1000.0)
        if len(self.engine_ms) > 4000:
            del self.engine_ms[:2000]
        self.tick += 1
        self.t = self.tick * self.DT_S
        if self.tick == LOCK_SETTLE_TICKS and self.pending_initial:
            for a in self.pending_initial:
                self.execute(a)
            log("seeded %d welds at tick %d" % (len(self.pending_initial), self.tick))
            self.pending_initial = []

        self.read_cells()
        for junction, due in list(self._relock):
            if self.tick >= due:
                if junction is not None:
                    a, face, b = junction
                    if self.cells[a].lock(face, True):
                        self.welds.add((int(a), face, int(b)))
                self._relock.remove((junction, due))
        self.targets.clear()
        excluded = {c.i for c in self.reef.cells if c.free and c.i in self.positions and not self.dockable(c.i)}
        if excluded != self._excluded_logged:
            log("undockable free cells (runway outside the arena): %s" % sorted(excluded))
            self._excluded_logged = set(excluded)
        # the ecology acts only once the seeded welds are in (a division at
        # t=0 re-rolled cells whose junction welds did not exist yet, measured)
        if self.tick > LOCK_SETTLE_TICKS + 2:
            for a in self.reef.step(self.DT_S, self.positions, moving_free=set(), excluded=excluded):
                self.execute(a)
        # organisms born / retired this tick
        for oid in list(self.org):
            if oid not in self.reef.organisms:
                del self.org[oid]
                self.forced.pop(oid, None)
                self._tbuf.pop(oid, None)
        self.queue.drain(self.executor, self.tick)
        for o in list(self.reef.organisms.values()):
            self.drive_organism(o)
        self.relax_free_cells()

        if self.tick % CENSUS_EVERY == 0 or self.tick % TELEMETRY_EVERY == 0:
            snap = self.snapshot()
            self.census.publish(snap)
            if self.tick % TELEMETRY_EVERY == 0:
                write_json(os.path.join(self.run_dir, "telemetry.json"), snap)
                log("t=%7.1fs tick=%6d orgs=%d free=%d debris=%d div=%d rec=%d shed=%d dead=%d "
                    "recyc=%d wd=%d docks=%d/%d step=%s ms"
                    % (self.t, self.tick, len(self.reef.organisms), snap["free"], snap["debris"],
                       self.reef.divisions, self.reef.recruits, self.reef.sheds, self.reef.deaths,
                       self.reef.recycles, self.reef.watchdog_kills, self.dock_stats["recruits"],
                       self.dock_stats["attempts"], self.step_median()))
                for o in self.reef.organisms.values():
                    st = self.org.get(o.id)
                    if not st:
                        continue
                    d = st.get("dock") or {}
                    log("  %s %s spd=%.3f up=%.2f err=%+.2f steer=%+.2f sign=%+d %s%s%s d=%.2f tail_along=%.2f "
                        "tail_lat=%+.2f spine_err=%.2f sep=%s"
                        % (o.id, o.state, st.get("speed") or 0.0, st.get("head_up") or 0.0, st.get("err") or 0.0, st.get("steer") or 0.0,
                           int(st["cal"]["sign"]), d.get("phase", "-"), " REV" if st.get("reverse") else "",
                           (" " + st["mode"]) if st.get("mode") else "",
                           d.get("dist_goal", 0.0) or 0.0, d.get("along", 0.0) or 0.0,
                           d.get("lateral", 0.0) or 0.0, d.get("spine_err", 0.0) or 0.0,
                           None if d.get("sep") is None else round(d["sep"], 3)))
        if not self.epoch_written and self.t >= self.EPOCH_S:
            self.finish_epoch()
            if not self.WATCH:
                # End the run NOW: run-headless --duration is a wall-clock sleep.
                self.quit(0)
                return False
        return True

    def run(self):
        if not self.setup():
            return 1
        # RESILIENCE + EVIDENCE: three long runs ended with the engine at exit
        # code 1 and no CRASH line (epochs 5, 15, 16). The only exit-1 path is
        # this controller's own crash handler, and its log() can itself fail
        # silently -- so a tick that raises is recorded to crash.txt with a
        # plain file write and skipped; only a run that keeps raising quits.
        self.tick_errors = 0
        while True:
            try:
                if not self.tick_once():
                    break
            except Exception:                           # noqa: BLE001
                import traceback
                self.tick_errors += 1
                text = "tick %d t=%.2f" % (self.tick, self.t) + chr(10) + traceback.format_exc()
                try:
                    with open(os.path.join(self.run_dir, "crash.txt"), "a", encoding="utf-8") as fh:
                        fh.write(text + chr(10))
                except OSError:
                    pass
                log("TICK ERROR %d (skipped): %s" % (self.tick_errors, text.splitlines()[-1]))
                if self.tick_errors > 50:
                    raise
                self.tick += 1
                if self.sup.step(self.DT) == -1:
                    break
        return self.quit_code or 0


# ================================================================== entry point
def load_inputs():
    if not os.path.exists(REEF_PATH) or not os.path.exists(CFG_PATH):
        return None, None
    with open(REEF_PATH, encoding="utf-8") as f:
        reef = json.load(f)
    with open(CFG_PATH, encoding="utf-8") as f:
        cfg = json.load(f)
    return reef, cfg


def main():
    reef, cfg = load_inputs()
    sup = Supervisor()
    if reef is None:
        log("FATAL no %s / %s -- run metazoa.py first" % (REEF_PATH, CFG_PATH))
        write_json(RESULT_PATH, {"error": "no_inputs", "reef": REEF_PATH, "config": CFG_PATH})
        sup.simulationQuit(1)
        sup.step(int(sup.getBasicTimeStep()))
        return 1
    d = Director(sup, reef, cfg)
    try:
        code = d.run()
    except Exception:                                   # noqa: BLE001
        # The engine discards a controller's stderr on Windows: an uncaught
        # exception here read as "exited with status: 1" and nothing else.
        import traceback
        log("CRASH " + traceback.format_exc())
        try:
            sup.simulationQuit(1)
            sup.step(d.DT)
        except Exception:                               # noqa: BLE001
            pass
        return 1
    if code:
        sup.step(d.DT)      # let simulationQuit reach the engine
    return code


if __name__ == "__main__":
    sys.exit(main())
