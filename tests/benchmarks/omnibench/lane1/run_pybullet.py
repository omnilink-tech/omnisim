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

"""OmniBench lane-1 runner — raw PyBullet.

Implements T1..T7 per SPEC.md / common/scenes.py, records the trajectory per
common/recording.py (quats reordered xyzw -> wxyz), scores via score.py,
writes a results row per common/results.py.

Usage:
    python run_pybullet.py --test T1..T7|all --dt-ms 4 --out <dir>

Engine notes (recorded per-row in `deviations`):
* Bullet combines friction and restitution multiplicatively across the pair,
  so static geometry gets mu=1.0 / e=1.0 and the dynamic body carries the
  SPEC value; where dynamic-dynamic pairs matter (T6) every body gets
  sqrt(mu) so all pairs combine to the SPEC mu.
* linearDamping/angularDamping default to 0.04 in PyBullet — zeroed on every
  dynamic body (they are unphysical drag, fatal for T4/T5/T7).
* Inertias are set explicitly from common/scenes.py via changeDynamics
  localInertiaDiagonal, so the scorer's energy/momentum bookkeeping matches.
* Solver iteration count is the engine default, recorded per row.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import results, scenes  # noqa: E402
from common.recording import write_recording  # noqa: E402
import score as scorer  # noqa: E402

import pybullet as p  # noqa: E402

ENGINE = "pybullet"


def _zero_damping(body, link=-1):
    p.changeDynamics(body, link, linearDamping=0.0, angularDamping=0.0,
                     jointDamping=0.0)


def _quat_wxyz(q):
    return (q[3], q[0], q[1], q[2])


class Handle:
    """One recorded body: (body_id, link_index). link -1 = base."""

    def __init__(self, body, link=-1):
        self.body, self.link = body, link

    def sample(self):
        if self.link < 0:
            pos, q = p.getBasePositionAndOrientation(self.body)
            vel, omega = p.getBaseVelocity(self.body)
        else:
            st = p.getLinkState(self.body, self.link, computeLinkVelocity=1)
            pos, q = st[0], st[1]          # COM pos, inertial-frame quat
            vel, omega = st[6], st[7]      # COM lin vel, world ang vel
        return pos, vel, _quat_wxyz(q), omega


# ---------------------------------------------------------------------------
# scene builders -> (handles {name: Handle}, ctrl_fn or None, deviations)
# ---------------------------------------------------------------------------

def build_t1():
    T1 = scenes.T1
    # Two measured artifacts require overrides (both recorded):
    # * default contactProcessingThreshold makes the contact persist and
    #   kills the bounce after the first peak (2nd peak 0.027 m vs 0.41);
    # * contact ERP push-out adds a constant ~+0.35 m/s (at dt=1 ms) to the
    #   separation velocity, inflating e 0.8 -> ~0.88. erp=contactERP=0
    #   removes it; with these, e=0.8 set directly lands peaks within ~3%
    #   of analytic at dt=1 ms.
    p.setPhysicsEngineParameter(erp=0.0, contactERP=0.0)
    plane = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE),
                              basePosition=[0, 0, scenes.FLOOR_TOP])
    sph = p.createCollisionShape(p.GEOM_SPHERE, radius=T1.sphere_r)
    ball = p.createMultiBody(T1.mass, sph, basePosition=[0, 0, T1.start_z])
    i = T1.inertia
    p.changeDynamics(ball, -1, restitution=T1.restitution, lateralFriction=0.0,
                     localInertiaDiagonal=[i, i, i],
                     contactProcessingThreshold=0.0)
    p.changeDynamics(plane, -1, restitution=1.0, lateralFriction=0.0,
                     contactProcessingThreshold=0.0)
    _zero_damping(ball)
    dev = ["bullet restitution combines multiplicatively: plane e=1.0, "
           "ball e=0.8 (set directly); restitutionVelocityThreshold default",
           "override: contactProcessingThreshold=0 (default persistence "
           "kills bounces after the 1st) and erp=contactERP=0 (ERP push-out "
           "otherwise adds ~+0.35 m/s per impact, e_eff~0.88)"]
    return {T1.body: Handle(ball)}, None, dev


def _make_incline(theta_deg, lane_y, mu):
    shp = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(scenes.INCLINE_HALF))
    q = p.getQuaternionFromEuler([0, math.radians(theta_deg), 0])
    inc = p.createMultiBody(0, shp, basePosition=[0, lane_y,
                                                  scenes.INCLINE_CENTER_Z],
                            baseOrientation=q)
    p.changeDynamics(inc, -1, lateralFriction=mu, restitution=0.0)
    return inc


def build_t2():
    T2 = scenes.T2
    handles = {}
    shp = p.createCollisionShape(p.GEOM_BOX,
                                 halfExtents=[T2.box_half] * 3)
    for k, th in enumerate(T2.angles_deg):
        y = k * T2.lane_spacing
        _make_incline(th, y, 1.0)
        pos = scenes.incline_body_start(th, y, T2.box_half)
        q = p.getQuaternionFromEuler([0, math.radians(th), 0])
        box = p.createMultiBody(T2.mass, shp, basePosition=list(pos),
                                baseOrientation=q)
        p.changeDynamics(box, -1, lateralFriction=T2.mu, restitution=0.0,
                         localInertiaDiagonal=list(T2.inertia_diag))
        _zero_damping(box)
        handles["box%d" % int(th)] = Handle(box)
    dev = ["bullet friction combines multiplicatively: incline mu=1.0, "
           "box mu=0.5"]
    return handles, None, dev


def build_t3():
    T3 = scenes.T3
    _make_incline(T3.theta_deg, 0.0, 1.0)
    shp = p.createCollisionShape(p.GEOM_SPHERE, radius=T3.sphere_r)
    pos = scenes.incline_body_start(T3.theta_deg, 0.0, T3.sphere_r)
    ball = p.createMultiBody(T3.mass, shp, basePosition=list(pos))
    i = T3.inertia
    p.changeDynamics(ball, -1, lateralFriction=T3.mu, restitution=0.0,
                     localInertiaDiagonal=[i, i, i],
                     rollingFriction=0.0, spinningFriction=0.0)
    _zero_damping(ball)
    dev = ["bullet friction combines multiplicatively: incline mu=1.0, "
           "ball mu=0.8"]
    return {T3.body: Handle(ball)}, None, dev


def _chain_dynamics(body, links):
    T4 = scenes.T4
    for li in links:
        p.changeDynamics(body, li, mass=T4.link_m,
                         localInertiaDiagonal=list(T4.link_inertia_diag))
        _zero_damping(body, li)
    for j in range(p.getNumJoints(body)):
        p.setJointMotorControl2(body, j, p.VELOCITY_CONTROL, force=0.0)


def build_t4():
    """Pivoted 3-link chain, horizontal along +x. Static massless base at the
    origin; links have no collision shapes (SPEC: no contacts).
    Empirically verified convention: linkPositions are relative to the
    PARENT LINK (joint) frame; linkInertialFramePositions to the link frame."""
    T4 = scenes.T4
    l = T4.link_l
    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=-1, basePosition=[0, 0, 0],
        linkMasses=[T4.link_m] * 3,
        linkCollisionShapeIndices=[-1] * 3,
        linkVisualShapeIndices=[-1] * 3,
        linkPositions=[[0, 0, 0], [l, 0, 0], [l, 0, 0]],
        linkOrientations=[[0, 0, 0, 1]] * 3,
        linkInertialFramePositions=[[l / 2, 0, 0]] * 3,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 3,
        linkParentIndices=[0, 1, 2],
        linkJointTypes=[p.JOINT_REVOLUTE] * 3,
        linkJointAxis=[[0, 1, 0]] * 3)
    _chain_dynamics(body, links=range(3))
    handles = {"link%d" % (i + 1): Handle(body, i) for i in range(3)}
    return handles, None, []


def build_t5():
    """Free-floating chain: base IS link1 (COM at x=0.25); joint j12 driven
    with tau = 5 sin(2 pi t) for 2 s (interpreted 'middle joint')."""
    T4 = scenes.T4
    l = T4.link_l
    body = p.createMultiBody(
        baseMass=T4.link_m, baseCollisionShapeIndex=-1,
        basePosition=[T4.com_x0[0], 0, 0],
        linkMasses=[T4.link_m] * 2,
        linkCollisionShapeIndices=[-1] * 2,
        linkVisualShapeIndices=[-1] * 2,
        linkPositions=[[l / 2, 0, 0], [l, 0, 0]],
        linkOrientations=[[0, 0, 0, 1]] * 2,
        linkInertialFramePositions=[[l / 2, 0, 0]] * 2,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 2,
        linkParentIndices=[0, 1],
        linkJointTypes=[p.JOINT_REVOLUTE] * 2,
        linkJointAxis=[[0, 1, 0]] * 2)
    p.changeDynamics(body, -1, mass=T4.link_m,
                     localInertiaDiagonal=list(T4.link_inertia_diag))
    _zero_damping(body)
    _chain_dynamics(body, links=range(2))

    def ctrl(t):
        p.setJointMotorControl2(body, 0, p.TORQUE_CONTROL,
                                force=scenes.T5.tau(t))
    handles = {"link1": Handle(body), "link2": Handle(body, 0),
               "link3": Handle(body, 1)}
    dev = ["'middle joint' = link1-link2 hinge (3 links have only 2 internal "
           "joints)"]
    return handles, ctrl, dev


def build_t6():
    T6 = scenes.T6
    mu_pair = math.sqrt(T6.mu)   # so box-box AND box-plane combine to mu
    plane = p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE),
                              basePosition=[0, 0, scenes.FLOOR_TOP])
    p.changeDynamics(plane, -1, lateralFriction=mu_pair, restitution=0.0)
    shp = p.createCollisionShape(p.GEOM_BOX, halfExtents=[T6.box_half] * 3)
    handles = {}
    for k in range(T6.n_boxes):
        box = p.createMultiBody(T6.mass, shp,
                                basePosition=[0, 0, scenes.T6.start_z(k)])
        p.changeDynamics(box, -1, lateralFriction=mu_pair,
                         restitution=T6.restitution,
                         localInertiaDiagonal=list(T6.inertia_diag))
        _zero_damping(box)
        handles["box%d" % k] = Handle(box)
    dev = ["bullet friction combines multiplicatively: all bodies mu="
           "sqrt(0.8)=%.4f so box-box and box-plane pairs both = 0.8"
           % mu_pair]
    return handles, None, dev


def build_t7():
    T7 = scenes.T7
    box = p.createMultiBody(T7.mass, -1, basePosition=[0, 0, 1])
    p.changeDynamics(box, -1,
                     localInertiaDiagonal=list(T7.inertia_diag))
    _zero_damping(box)
    p.resetBaseVelocity(box, angularVelocity=list(T7.omega0))
    dev = ["pybullet damps the off-axis seed ~3x in the first second and "
           "the intermediate-axis (Dzhanibekov) eruption does NOT occur "
           "within the 10 s window (timing is chaotically sensitive: a "
           "1e-10 inertia perturbation moves it by seconds; measured "
           "eruption ~8-10 s in some configs, adding ~2% rot KE). "
           "Conservation metrics here therefore describe a near-steady "
           "spin, not a tumble; MuJoCo's instability grows within the "
           "window."]
    return {T7.body: Handle(box)}, None, dev


_BUILDERS = {"T1": build_t1, "T2": build_t2, "T3": build_t3, "T4": build_t4,
             "T5": build_t5, "T6": build_t6, "T7": build_t7}
_NO_GRAVITY = {"T5", "T7"}


# ---------------------------------------------------------------------------
# sim loop
# ---------------------------------------------------------------------------

def run_test(test, dt):
    p.resetSimulation()
    p.setTimeStep(dt)
    g = 0.0 if test in _NO_GRAVITY else -scenes.GRAVITY
    p.setGravity(0, 0, g)
    handles, ctrl_fn, dev = _BUILDERS[test]()
    params = p.getPhysicsEngineParameters()
    dev.append("bullet default solver: numSolverIterations=%s numSubSteps=%s"
               % (params.get("numSolverIterations"),
                  params.get("numSubSteps")))
    duration = scenes.DURATIONS[test]
    steps = int(round(duration / dt))
    S = steps + 1
    rec = {n: {"pos": np.empty((S, 3)), "vel": np.empty((S, 3)),
               "quat": np.empty((S, 4)), "omega": np.empty((S, 3))}
           for n in handles}

    def sample(k):
        for n, h in handles.items():
            pos, vel, quat, omega = h.sample()
            r = rec[n]
            r["pos"][k] = pos
            r["vel"][k] = vel
            r["quat"][k] = quat
            r["omega"][k] = omega

    sample(0)
    wall = 0.0
    for k in range(steps):
        if ctrl_fn is not None:
            ctrl_fn(k * dt)     # torque applied over [k*dt, (k+1)*dt)
        t0 = time.perf_counter()
        p.stepSimulation()
        wall += time.perf_counter() - t0
        sample(k + 1)
    t = np.arange(S) * dt
    return t, rec, wall * 1000.0 / max(steps, 1), steps, dev


def run_and_score(test, dt_ms, out_dir):
    dt = dt_ms / 1000.0
    t, bodies, wall, steps, dev = run_test(test, dt)
    dev.append("pybullet %s" % getattr(p, "__version__", "unknown"))
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    npz = out_dir / ("%s_%s_dt%g.npz" % (scenes.TEST_NAMES[test], ENGINE,
                                         dt_ms))
    write_recording(npz, t, bodies)
    metrics, notes, extras = scorer.score(test, npz)
    row = results.write_row(out_dir / "results.jsonl",
                            test=scenes.TEST_NAMES[test], engine=ENGINE,
                            dt_ms=dt_ms, metrics=metrics,
                            wall_ms_per_step=wall, steps=steps,
                            sim_seconds=steps * dt,
                            deviations=dev + notes)
    print("[%s %s dt=%gms] steps=%d wall=%.4f ms/step  metrics=%s"
          % (ENGINE, test, dt_ms, steps, wall,
             {k: round(v, 6) for k, v in metrics.items()}))
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--test", required=True,
                    help="T1..T7, SPEC name, or 'all'")
    ap.add_argument("--dt-ms", type=float, default=4.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    tests = list(scenes.TESTS) if args.test.lower() == "all" \
        else [scenes.normalize_test_id(args.test)]
    p.connect(p.DIRECT)
    try:
        for tid in tests:
            run_and_score(tid, args.dt_ms, args.out)
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
