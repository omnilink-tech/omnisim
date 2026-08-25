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

"""OmniBench lane-1 runner — raw MuJoCo.

Implements T1..T7 exactly per SPEC.md / common/scenes.py, records the
trajectory per common/recording.py, scores via score.py, writes a results
row per common/results.py.

Usage:
    python run_mujoco.py --test T1..T7|all --dt-ms 4 --out <dir>

Restitution note (T1): MuJoCo has no restitution coefficient. We use direct
solref = (-k, -b) contact dynamics with k fixed and b calibrated by bisection
on a dt=1 ms drop until the first bounce peak hits h1 = h0*e^2 = 0.64 m; the
sweep then reuses that fixed mapping (recorded in `deviations`).
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

import mujoco  # noqa: E402

ENGINE = "mujoco"
BOUNCE_K = 1.0e5   # fixed direct-solref stiffness for T1 (calibrate b only)


# ---------------------------------------------------------------------------
# generic sim loop
# ---------------------------------------------------------------------------

def run_model(xml, duration, body_names, ctrl_fn=None, init_fn=None):
    """Build, run, record. Returns (t, bodies, wall_ms_per_step, steps)."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if init_fn is not None:
        init_fn(m, d)
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    steps = int(round(duration / dt))
    ids = {n: m.body(n).id for n in body_names}
    S = steps + 1
    rec = {n: {"pos": np.empty((S, 3)), "vel": np.empty((S, 3)),
               "quat": np.empty((S, 4)), "omega": np.empty((S, 3))}
           for n in body_names}
    vel6 = np.empty(6)

    def sample(k):
        for n, bid in ids.items():
            r = rec[n]
            r["pos"][k] = d.xpos[bid]
            r["quat"][k] = d.xquat[bid]          # wxyz, world<-body
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, bid,
                                     vel6, 0)    # world frame
            r["omega"][k] = vel6[:3]
            r["vel"][k] = vel6[3:]

    sample(0)
    wall = 0.0
    for k in range(steps):
        if ctrl_fn is not None:
            ctrl_fn(d)
        t0 = time.perf_counter()
        mujoco.mj_step(m, d)
        wall += time.perf_counter() - t0
        sample(k + 1)
    t = np.arange(S) * dt
    return t, rec, wall * 1000.0 / max(steps, 1), steps


# ---------------------------------------------------------------------------
# scene builders (MJCF via XML strings; explicit inertials from scenes.py)
# ---------------------------------------------------------------------------

def _inertial(mass, diag, pos="0 0 0"):
    return ('<inertial pos="%s" mass="%g" diaginertia="%.10g %.10g %.10g"/>'
            % (pos, mass, diag[0], diag[1], diag[2]))


def xml_t1(dt, b_damp):
    T1 = scenes.T1
    # constant-impedance solimp => linear spring-damper contact (verified to
    # converge to the analytic peak sequence at dt=0.25 ms, rmse ~0.7%)
    solref = ('solref="%g %g" solimp="0.95 0.95 0.001"'
              % (-BOUNCE_K, -b_damp))
    i = T1.inertia
    return f"""
<mujoco model="bounce">
  <option timestep="{dt}" gravity="0 0 -{scenes.GRAVITY}"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" pos="0 0 {scenes.FLOOR_TOP}"
          condim="1" {solref}/>
    <body name="ball" pos="0 0 {T1.start_z}">
      <freejoint/>
      <geom type="sphere" size="{T1.sphere_r}" condim="1" {solref}/>
      {_inertial(T1.mass, (i, i, i))}
    </body>
  </worldbody>
</mujoco>"""


def _incline_lane(theta_deg, lane_y, mu):
    return ('<geom type="box" size="%g %g %g" pos="0 %g %g" euler="0 %g 0" '
            'friction="%g 0 0"/>'
            % (*scenes.INCLINE_HALF, lane_y, scenes.INCLINE_CENTER_Z,
               theta_deg, mu))


def xml_t2(dt):
    T2 = scenes.T2
    lanes = []
    for k, th in enumerate(T2.angles_deg):
        y = k * T2.lane_spacing
        lanes.append(_incline_lane(th, y, T2.mu))
        pos = scenes.incline_body_start(th, y, T2.box_half)
        lanes.append(f"""
    <body name="box{int(th)}" pos="{pos[0]:.10g} {pos[1]:.10g} {pos[2]:.10g}" euler="0 {th} 0">
      <freejoint/>
      <geom type="box" size="{T2.box_half} {T2.box_half} {T2.box_half}" friction="{T2.mu} 0 0"/>
      {_inertial(T2.mass, T2.inertia_diag)}
    </body>""")
    body = "\n".join(lanes)
    return f"""
<mujoco model="incline">
  <option timestep="{dt}" gravity="0 0 -{scenes.GRAVITY}" cone="elliptic" impratio="10"/>
  <worldbody>
{body}
  </worldbody>
</mujoco>"""


def xml_t3(dt):
    T3 = scenes.T3
    pos = scenes.incline_body_start(T3.theta_deg, 0.0, T3.sphere_r)
    i = T3.inertia
    return f"""
<mujoco model="roll">
  <option timestep="{dt}" gravity="0 0 -{scenes.GRAVITY}" cone="elliptic" impratio="10"/>
  <worldbody>
    {_incline_lane(T3.theta_deg, 0.0, T3.mu)}
    <body name="ball" pos="{pos[0]:.10g} {pos[1]:.10g} {pos[2]:.10g}">
      <freejoint/>
      <geom type="sphere" size="{T3.sphere_r}" friction="{T3.mu} 0 0"/>
      {_inertial(T3.mass, (i, i, i))}
    </body>
  </worldbody>
</mujoco>"""


def _chain_links(free_base):
    """3 capsule links along +x, body frames AT the link COMs, hinges about
    +y anchored at the link's uphill (-x) end. contype=0 (no contacts)."""
    T4 = scenes.T4
    hl = T4.link_l / 2.0
    cap = ('<geom type="capsule" fromto="-%g 0 0 %g 0 0" size="%g" '
           'contype="0" conaffinity="0"/>' % (hl, hl, T4.link_r))
    inert = _inertial(T4.link_m, T4.link_inertia_diag)
    j1 = ('<freejoint/>' if free_base else
          f'<joint name="j1" type="hinge" pos="-{hl} 0 0" axis="0 1 0"/>')
    return f"""
    <body name="link1" pos="{T4.com_x0[0]} 0 0">
      {j1}
      {cap}
      {inert}
      <body name="link2" pos="{T4.link_l} 0 0">
        <joint name="j12" type="hinge" pos="-{hl} 0 0" axis="0 1 0"/>
        {cap}
        {inert}
        <body name="link3" pos="{T4.link_l} 0 0">
          <joint name="j23" type="hinge" pos="-{hl} 0 0" axis="0 1 0"/>
          {cap}
          {inert}
        </body>
      </body>
    </body>"""


def xml_t4(dt):
    return f"""
<mujoco model="pendulum_energy">
  <option timestep="{dt}" gravity="0 0 -{scenes.GRAVITY}"/>
  <worldbody>
{_chain_links(free_base=False)}
  </worldbody>
</mujoco>"""


def xml_t5(dt):
    return f"""
<mujoco model="momentum">
  <option timestep="{dt}" gravity="0 0 0"/>
  <worldbody>
{_chain_links(free_base=True)}
  </worldbody>
  <actuator>
    <motor name="drive" joint="j12"/>
  </actuator>
</mujoco>"""


def xml_t6(dt):
    T6 = scenes.T6
    boxes = []
    for k in range(T6.n_boxes):
        boxes.append(f"""
    <body name="box{k}" pos="0 0 {T6.start_z(k):.10g}">
      <freejoint/>
      <geom type="box" size="{T6.box_half} {T6.box_half} {T6.box_half}" friction="{T6.mu} 0 0"/>
      {_inertial(T6.mass, T6.inertia_diag)}
    </body>""")
    body = "\n".join(boxes)
    return f"""
<mujoco model="stack">
  <option timestep="{dt}" gravity="0 0 -{scenes.GRAVITY}"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" pos="0 0 {scenes.FLOOR_TOP}"
          friction="{T6.mu} 0 0"/>
{body}
  </worldbody>
</mujoco>"""


def xml_t7(dt):
    T7 = scenes.T7
    h = [s / 2.0 for s in T7.box_full]
    return f"""
<mujoco model="spin">
  <option timestep="{dt}" gravity="0 0 0"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="{h[0]} {h[1]} {h[2]}" contype="0" conaffinity="0"/>
      {_inertial(T7.mass, T7.inertia_diag)}
    </body>
  </worldbody>
</mujoco>"""


# ---------------------------------------------------------------------------
# T1 restitution calibration (dt = 1 ms, bisection on damping b)
# ---------------------------------------------------------------------------

def _first_peak(b_damp, dt=0.001):
    T1 = scenes.T1
    m = mujoco.MjModel.from_xml_string(xml_t1(dt, b_damp))
    d = mujoco.MjData(m)
    steps = int(round(1.3 / dt))
    zmax = 0.0
    bounced = False
    for _ in range(steps):
        mujoco.mj_step(m, d)
        if not bounced and d.qvel[2] > 0.5:
            bounced = True
        if bounced:
            zmax = max(zmax, d.qpos[2])
    # Apex measured in the SAME frame as score.py: above the floor's top face,
    # which is scenes.FLOOR_TOP, not z=0. Without this subtraction the
    # bisection calibrates against a target that is FLOOR_TOP too high.
    return zmax - T1.sphere_r - scenes.FLOOR_TOP


_CAL_CACHE = None


def calibrate_bounce():
    """Bisection on solref damping b at dt=1ms -> h1 = 0.64 m. Cached."""
    global _CAL_CACHE
    if _CAL_CACHE is not None:
        return _CAL_CACHE
    target = scenes.T1.drop_h * scenes.T1.restitution ** 2
    lo, hi = 0.0, 400.0     # h1 decreases monotonically with b
    h_lo, h_hi = _first_peak(lo), _first_peak(hi)
    if not (h_lo > target > h_hi):
        raise RuntimeError("calibration bracket failed: h(%g)=%.3f h(%g)=%.3f"
                           % (lo, h_lo, hi, h_hi))
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        h = _first_peak(mid)
        if abs(h - target) < 1e-4:
            lo = hi = mid
            break
        if h > target:
            lo = mid
        else:
            hi = mid
    b = 0.5 * (lo + hi)
    h = _first_peak(b)
    _CAL_CACHE = (b, h)
    return _CAL_CACHE


# ---------------------------------------------------------------------------
# per-test drivers
# ---------------------------------------------------------------------------

def run_test(test, dt):
    """-> (t, bodies, wall_ms_per_step, steps, deviations)"""
    dev = []
    if test == "T1":
        b, h1 = calibrate_bounce()
        dev.append("mujoco has no restitution coeff; direct solref=(-%g,-%.4f)"
                   " calibrated at dt=1ms -> first peak %.4f m (target %.4f)"
                   % (BOUNCE_K, b, h1,
                      scenes.T1.drop_h * scenes.T1.restitution ** 2))
        dev.append("solimp=(0.95,0.95,0.001) constant impedance => linear "
                   "contact (speed-independent e; converges to analytic "
                   "peaks at dt=0.25ms)")
        dev.append("fixed contact stiffness k=%g has sqrt(k/m)*dt>2 for "
                   "dt>=8ms: contact integration unstable there (energy "
                   "GAIN), which is the honest sweep result of freezing "
                   "the dt=1ms mapping" % BOUNCE_K)
        out = run_model(xml_t1(dt, b), scenes.T1.duration, scenes.BODIES["T1"])
    elif test == "T2":
        dev.append("override: cone=elliptic impratio=10 (default pyramidal "
                   "cone creeps ~5mm/3s at 15-26 deg; with override "
                   "sub-0.5mm stick below theta_c)")
        out = run_model(xml_t2(dt), scenes.T2.duration, scenes.BODIES["T2"])
    elif test == "T3":
        dev.append("override: cone=elliptic impratio=10 (same as T2)")
        out = run_model(xml_t3(dt), scenes.T3.duration, scenes.BODIES["T3"])
    elif test == "T4":
        out = run_model(xml_t4(dt), scenes.T4.duration, scenes.BODIES["T4"])
    elif test == "T5":
        def ctrl(d):
            d.ctrl[0] = scenes.T5.tau(d.time)
        dev.append("'middle joint' = link1-link2 hinge (3 links have only "
                   "2 internal joints)")
        out = run_model(xml_t5(dt), scenes.T5.duration, scenes.BODIES["T5"],
                        ctrl_fn=ctrl)
    elif test == "T6":
        out = run_model(xml_t6(dt), scenes.T6.duration, scenes.BODIES["T6"])
    elif test == "T7":
        def init(m, d):
            d.qvel[3:6] = scenes.T7.omega0   # body frame == world at t=0
        out = run_model(xml_t7(dt), scenes.T7.duration, scenes.BODIES["T7"],
                        init_fn=init)
    else:
        raise ValueError(test)
    t, bodies, wall, steps = out
    dev.append("mujoco defaults: integrator=Euler, solver=Newton, "
               "iterations=100, mujoco %s" % mujoco.__version__)
    return t, bodies, wall, steps, dev


def run_and_score(test, dt_ms, out_dir):
    dt = dt_ms / 1000.0
    t, bodies, wall, steps, dev = run_test(test, dt)
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
    for tid in tests:
        run_and_score(tid, args.dt_ms, args.out)


if __name__ == "__main__":
    main()
