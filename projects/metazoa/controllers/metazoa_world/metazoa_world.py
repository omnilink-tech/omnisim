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
  * docking is the docking cell's `f_tail.isLocked` <- TRUE (the ACTIVE side,
    `Reef.junctions()`); the engine welds only when a partner face is within
    tolerance, so a lock write is a request and the read-back is the answer.
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
VERIFY_SEP = LOCK_SEP + 0.01         # ... while the body keeps crawling; a weld that took tracks,
                                     # a face that was merely near is left behind / pushed away
BACKOFF_M = 0.2
BACKOFF_S = 3.0
STANDOFF_M = 0.30           # line up on the free face's normal this far out before the run-in
LATERAL_MAX_M = 0.12        # off the approach axis by more than this -> back to the standoff
GO_AROUND_M = 0.6           # swing this wide when the head is behind the free face
LOOKAHEAD_M = 0.15          # pure-pursuit point ahead on the approach axis during the run-in
FACE_CHECK_M = 0.30         # read the two face nodes only when the member is this close
CAPTURE_M = 0.22            # capture assist range (see dock_step)
CAPTURE_AXIS = 0.9          # ... and alignment
RECRUIT_TIMEOUT_S = 90.0    # a forced /recruit that has not docked by then is dropped
UPRIGHT_MIN = 0.35          # tail-block up-vector z below this = on its side (reported, not righted)
LOCK_SETTLE_TICKS = 25      # 0.2 s: seeded welds wait for the first registered step; at 1 s a
                            # side-roller (yaw) cell had time to tip flat before its weld took (measured)


_LOG_FH = None


def log(msg):
    """stdout AND _run/metazoa/world.log: the engine discards a controller's
    stdout on Windows, so without the file the docking trace is unreadable."""
    global _LOG_FH
    line = "[reef] %s" % msg
    print(line, flush=True)
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

    def teleport(self, x, y, yaw, z=None):
        """Robot fields in the Pose's frame; roll back to 0 (flat), hinge 0,
        resetPhysics. No velocity write (see module docstring)."""
        z = self.spawn_z if z is None else float(z)
        c, s = math.cos(-self.pose_yaw), math.sin(-self.pose_yaw)
        dx, dy, dz = x - self.pose_t[0], y - self.pose_t[1], z - self.pose_t[2]
        self.tr_field.setSFVec3f([c * dx - s * dy, s * dx + c * dy, dz])
        self.rot_field.setSFRotation([0.0, 0.0, 1.0, wrap(yaw - self.pose_yaw)])
        self.pos_field.setSFFloat(0.0)
        self.robot.resetPhysics()


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
        self.free_relaxed = set()  # free cells whose hinge was written 0 once
        self.welds = set()         # (i, face, j) locks this supervisor wrote and did not release
        self.positions = {}        # i -> (x, y, z) read THIS tick (alive cells)
        self.yaws = {}
        self.ups = {}
        self.axis_z = {}           # i -> |world z of the hinge axis| (1 = yaw hinge)
        self.dock_stats = {"attempts": 0, "locks_written": 0, "recruits": 0,
                           "failed_verify": 0, "forced": 0}
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

    def dock_member(self, o):
        """(member cell, member face) the next recruit's f_tail meets."""
        slot = o.open_branch_slot()
        if slot is not None:
            k, side = slot
            return o.spine[k], ORG.BRANCH_SIDE_FACE[side]
        return o.head, "f_nose"

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
        """One tick of the recruit-and-dock manoeuvre. Returns (aim_xy,
        member_mode, member): the point the head steers at, how the docking
        member's own hinge is driven this tick ("wave" = its normal gait
        target, "dock" = no wave: steer bias only on a yaw member, flat on a
        pitch member -- the face must be where approach_pose assumes) and
        that member.

        States: approach (pure pursuit onto the free face's normal: the
        standoff point while far or off-axis, then a look-ahead point on the
        axis; the lock is written when the two faces are within LOCK_SEP with
        normals opposed within LOCK_AXIS), verify (the body keeps crawling
        for VERIFY_S; the faces must stay within VERIFY_SEP, then the ecology
        recruits; a weld that did not take is released), backoff (aim
        BACKOFF_M further out for BACKOFF_S, then retry)."""
        member, mface = self.dock_member(o)
        mp = self.cell_pose(member)
        fp = self.cell_pose(j)
        if mp is None or fp is None:
            return None, "wave", member
        d = st["dock"]
        if d is None or d["cell"] != j or d["member"] != member:
            d = {"cell": j, "member": member, "face": mface, "state": "approach",
                 "since": self.t, "attempts": 0, "backoff_until": 0.0, "lock_t": None,
                 "sep": None, "axis_err": None, "sep_lock": None}
            st["dock"] = d
            self.dock_stats["attempts"] += 1
        goal = ORG.approach_pose(fp, "f_tail", DOCK_GAP, head_face=mface)   # member root pose
        _fx, _fy, fyaw = ORG.face_pose(fp, "f_tail")                        # free face outward normal
        nx, ny = math.cos(fyaw), math.sin(fyaw)
        rx, ry = mp[0] - goal[0], mp[1] - goal[1]
        along = rx * nx + ry * ny                 # + = outside the free face, along its normal
        lateral = abs(-rx * ny + ry * nx)
        d["dist_goal"], d["along"], d["lateral"] = ORG.distance_xy(mp, goal), along, lateral
        far_aim = (goal[0] + nx * (STANDOFF_M + BACKOFF_M), goal[1] + ny * (STANDOFF_M + BACKOFF_M))
        standoff = (goal[0] + nx * STANDOFF_M, goal[1] + ny * STANDOFF_M)
        if d["state"] == "backoff":
            if self.t < d["backoff_until"]:
                return far_aim, "wave", member
            d["state"] = "approach"
        if d["state"] == "approach":
            if along < -0.05 and ORG.distance_xy(mp, fp) < GO_AROUND_M:
                # The head is BEHIND the free face (on the wrong side of the
                # cell): pure pursuit toward the standoff would push straight
                # through the cell. Measured: three organisms parked at
                # along -1.3 / -2.9 / -1.8 m for a whole epoch. Swing wide to
                # the side the head is already on, then come back in front.
                side = 1.0 if (-rx * ny + ry * nx) >= 0.0 else -1.0
                d["phase"] = "go_around"
                aim = (fp[0] + nx * (STANDOFF_M * 0.5) - side * ny * GO_AROUND_M,
                       fp[1] + ny * (STANDOFF_M * 0.5) + side * nx * GO_AROUND_M)
                return aim, "wave", member
            if along > STANDOFF_M + 0.05 or along < -0.05 or lateral > LATERAL_MAX_M:
                d["phase"] = "standoff"
                return standoff, "wave", member
            d["phase"] = "run_in"
            s_ahead = max(0.0, along - LOOKAHEAD_M)
            aim = (goal[0] + nx * s_ahead, goal[1] + ny * s_ahead)
            if ORG.distance_xy(mp, fp) <= FACE_CHECK_M:
                (ax, ay, ayaw), _pa = self.cells[member].face_pose2d(mface)
                (bx, by, byaw), _pb = self.cells[j].face_pose2d("f_tail")
                sep = math.hypot(ax - bx, ay - by)
                axis_err = abs(wrap(ayaw - byaw - math.pi))
                d["sep"], d["axis_err"] = sep, axis_err
                # CAPTURE ASSIST. An undulating head cannot hold a 3 cm / 0.45
                # rad tolerance by itself (measured: it reaches the face plane
                # 19 cm off-axis). Real docking mechanisms capture with magnets
                # or guide cones over a short range; here, inside CAPTURE_M and
                # roughly aligned, the INERT free cell is drawn onto the socket
                # (a bounded teleport of the passive part; the engine's own
                # weld tolerance still decides whether the lock takes).
                if sep <= CAPTURE_M and axis_err <= CAPTURE_AXIS and (sep > LOCK_SEP or axis_err > LOCK_AXIS):
                    nx2, ny2 = math.cos(ayaw), math.sin(ayaw)
                    off = DOCK_GAP + C.BLOCK / 2.0
                    self.cells[j].teleport(ax + nx2 * off, ay + ny2 * off, ayaw)
                    self.dock_stats["captures"] = self.dock_stats.get("captures", 0) + 1
                    log("%s capture assist: CELL_%d drawn onto CELL_%d.%s (sep %.3f, axis %.2f rad)"
                        % (o.id, j, member, mface, sep, axis_err))
                    return aim, "dock", member          # lock on the next tick, after the step
                if sep <= LOCK_SEP and axis_err <= LOCK_AXIS:
                    self.cells[j].lock("f_tail", True)
                    self.welds.add((j, "f_tail", member))
                    self.dock_stats["locks_written"] += 1
                    d["state"], d["lock_t"], d["sep_lock"] = "verify", self.t, sep
                    log("%s lock written: CELL_%d.f_tail -> CELL_%d.%s (sep %.4f, axis %.3f rad)"
                        % (o.id, j, member, mface, sep, axis_err))
            return aim, "dock", member
        if d["state"] == "verify":
            (ax, ay, _), _pa = self.cells[member].face_pose2d(mface)
            (bx, by, _), _pb = self.cells[j].face_pose2d("f_tail")
            sep = math.hypot(ax - bx, ay - by)
            d["sep"] = sep
            held = self.t - d["lock_t"]
            if sep <= VERIFY_SEP and held < VERIFY_S:
                return (goal[0], goal[1]), "dock", member      # keep crawling, keep watching
            if sep <= VERIFY_SEP:
                try:
                    acts = self.reef.recruit(o.id, j)
                except ValueError as exc:
                    log("%s recruit of CELL_%d refused by the ecology: %s" % (o.id, j, exc))
                    acts = None
                if acts is not None:
                    for a in acts:
                        self.execute(a)
                    self.welds.discard((j, "f_tail", member))
                    self.dock_stats["recruits"] += 1
                    self.forced.pop(o.id, None)
                    self.free_relaxed.discard(j)
                    log("%s RECRUITED CELL_%d at t=%.2f (junction sep %.4f m held %.2f s, %d attempt(s), "
                        "now %d cells)" % (o.id, j, self.t, sep, held, d["attempts"] + 1, len(o)))
                    st["dock"] = None
                    return None, "wave", member
            # the weld did not take (the faces drifted apart while the body moved): release, back off
            self.cells[j].lock("f_tail", False)
            self.welds.discard((j, "f_tail", member))
            self.dock_stats["failed_verify"] += 1
            d["attempts"] += 1
            d["state"], d["backoff_until"] = "backoff", self.t + BACKOFF_S
            log("%s dock verify FAILED (sep %.4f > %.3f after %.2f s): unlocked, backing off %.1f m for %.0f s"
                % (o.id, sep, VERIFY_SEP, held, BACKOFF_M, BACKOFF_S))
            self.ramp(o.id)
            return far_aim, "wave", member
        return None, "wave", member

    def drive_organism(self, o):
        st = self.org_state(o.id)
        hp = self.head_pose(o)
        if hp is None:
            return
        self.update_travel(o, st)
        forced_steer = self.calibrate(o, st, hp)
        steer = 0.0
        member_mode, member = "wave", None
        j = self.recruit_target_of(o)
        if j is not None:
            aim, member_mode, member = self.docking(o, st, hp, j)
        else:
            st["dock"] = None
            aim = self.targets.get(o.id)
        if forced_steer is not None:
            steer = forced_steer
        elif aim is not None:
            # steer on the MEASURED travel heading (alife: a gait may carry a
            # body sideways or backwards; the nose is the fallback)
            head_for_error = (hp[0], hp[1], st["heading"] if st["heading"] is not None else hp[2])
            err = ORG.heading_error(head_for_error, aim)
            steer = ORG.steer_from_error(err, sign=st["cal"]["sign"])
        st["steer"] = steer
        st["aim"] = aim
        bp = self.effective_bodyplan(o, st)
        branches = [side for (_k, side) in sorted(o.branches)]
        buf = self._tbuf.setdefault(o.id, [])
        targets = ORG.chain_targets(o.genome, bp, len(o.spine), self.t, steer,
                                    branches=branches, out=buf)
        fade = min(1.0, max(0.0, (self.t - st["ramp_t0"]) / FADE_S)) if FADE_S > 0 else 1.0
        members = o.members()
        for idx, i in enumerate(members):
            v = targets[idx] * fade if idx < len(targets) else 0.0
            if member_mode == "dock" and i == member:
                # no wave on the docking member: its face must sit where
                # approach_pose assumes. A yaw member keeps the steer bias
                # (it may be the body's only steering hinge); a pitch member
                # is held flat.
                v = (o.genome["steer_gain"] * steer) if self.axis_z.get(i, 0.0) > 0.7 else 0.0
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
            member, mface = self.dock_member(o)

            def fin(j=j, member=member, mface=mface):
                mp, fp = self.cell_pose(member), self.cell_pose(j)
                if mp is None or fp is None:
                    return 409, {"ok": False, "error": "unmeasured", "organism": oid, "cell": j}
                goal = ORG.approach_pose(fp, "f_tail", DOCK_GAP, head_face=mface)
                return 200, {"ok": True, "accepted": True, "organism": oid, "cell": j,
                             "member": member, "member_face": mface,
                             "distance_measured": round(ORG.distance_xy(mp, fp), 4),
                             "approach_pose": [round(v, 4) for v in goal],
                             "state": "approaching",
                             "note": "docking completes when the faces mate and the weld is "
                                     "read back; watch /census organisms[%s].dock" % oid}
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
                "dock": None if d is None else {k: (round(v, 4) if isinstance(v, float) else v)
                                                for k, v in d.items()},
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
        self.targets.clear()
        for a in self.reef.step(self.DT_S, self.positions, moving_free=set()):
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
        while self.tick_once():
            pass
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
    code = d.run()
    if code:
        sup.step(d.DT)      # let simulationQuit reach the engine
    return code


if __name__ == "__main__":
    sys.exit(main())
