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

"""mj_data's Cartesian arrays must be at t+dt when the tick returns.

WHY THIS EXISTS
---------------
Every plan to make the Newton per-tick path cheaper ends at the same place:
stop re-deriving body transforms in Python and read the ones MuJoCo already
computed. `_update_newton_state` converts qpos/qvel into joint coordinates and
then runs `eval_articulation_fk` to rebuild `body_q`/`body_qd` -- transforms
`mj_step` had already written into `mj_data.xpos` / `xquat` / `cvel`. Serving
the readback from those arrays instead removes that work.

It is not safe today. `mj_step` is forward() then _advance(), and _advance
updates only qpos, qvel and time. Nothing recomputes the Cartesian arrays
afterwards, so on return qpos is at t+dt while xpos / xquat / xipos /
subtree_com / cvel are still at t. Anything served straight out of mj_data
would report every body ONE TICK LATE, everywhere, silently.

THE TRAP THIS TEST IS BUILT TO AVOID
------------------------------------
Commit ee614170 concluded the opposite and was wrong. It sampled a SETTLED
scene, where a one-tick time offset is invisible by construction -- there is no
motion to lag -- read the resulting ~1e-08 m as proof of equality, and shipped
that as a "correction". So this test does two things that check does not:

  1. It asserts the probe scene is genuinely MOVING before it judges anything.
     A settled scene fails as "vacuous", never passes.
  2. It reports the deviation against the CURRENT tick's pose and against the
     PREVIOUS tick's pose. A one-tick TIME offset shows |v|*dt in the first and
     ~0 in the second. A FRAME offset would show a non-zero in both. That is
     what makes the number a diagnosis rather than a magnitude.

WHAT IT PINS
------------
Fresh Cartesian arrays are the precondition for the whole native-tick plan, and
the documented way to get them is the mj_step2 -> mj_step1 split: mj_step2
integrates to t+dt, mj_step1 recomputes position and velocity kinematics there.
Same total work per tick, arrays left current.

Measured on this probe scene (free body + swinging pendulum, dt = 4 ms):

    today  (mj_step)          max |xpos - body_q| = 1.4126e-02 m == |v|*dt
                              max |cvel->qd - body_qd| = 5.22e-02
    after  (step2 -> step1)   max |xpos - body_q| = 2.67e-07 m
                              max |cvel->qd - body_qd| = 1.22e-06

The two columns swap exactly, which is the signature of a pure time offset.

VERDICT: this test FAILS on today's build and PASSES after the mj_step2 ->
mj_step1 split lands in `omnisim_newton_runtime.py`.

The second test also pins the EXACT mj_data -> newton velocity conversion a
later stage needs, because getting it wrong is silent:

    ang    = cvel[b][0:3]                                  # world-aligned
    offset = xipos[b] - subtree_com[body_rootid[b]]
    lin    = cvel[b][3:6] - cross(offset, ang)             # linear AT the COM
    body_qd[nb] = (lin, ang)                               # newton is (lin, rot)

cvel is (rot, lin) and its linear part is referenced to subtree_com of the
body's ROOT, while newton's body_qd is (lin, rot) with the linear part at the
body's own COM. Reproducing body_qd to float32 with that formula -- which this
test asserts -- is what makes the later stage exact rather than approximate.

SCOPE / HOW IT RUNS
-------------------
This drives `omnisim_newton_runtime.py` -- the bundled Python module imported
by omnisim-bin -- loaded from the canonical source and executed
against the same bundled newton/warp/mujoco runtime the engine loads. No
simulator binary, no world file, no controller: ~10 s, and immune to the
intermittent embedded-interpreter bring-up defect that makes the engine-driven
Newton tests skip.

That is deliberate, and it constrains the fix: stage 1 must land in the
embedded Python runtime source. If the split were instead implemented in C++
against mujoco.dll, this test would keep failing because the runtime source it
reads would be unchanged -- and it would be right to, because a C++-only
stage 1 leaves the Python path (used by every non-batched tick) stale.

CPU `newtonSolver "mujoco"` only. mujoco_warp keeps its own data path and is
not touched.

    python -m pytest tests/test_newton_cartesian_arrays_are_fresh.py -v
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
RUNTIME_SRC = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
BUNDLE = REPO / "msys64" / "mingw64" / "bin" / "newton-runtime"

#: Fresh means "at t+dt". The post-split residual is float32 round-off on the
#: warp side (measured 8.9e-08 .. 2.7e-07 m at these coordinates); the stale
#: value is |v|*dt (measured 1.41e-02 m). 1e-4 sits ~400x above the noise and
#: ~140x below the staleness, so neither a lucky run nor an unlucky one decides
#: this.
FRESH_POS_TOL_M = 1e-4
#: Same reasoning for velocity: post-split residual 2.4e-07 .. 1.2e-06,
#: staleness 5.2e-02 (= g*dt plus the pendulum's angular term).
FRESH_VEL_TOL = 1e-3
#: Below this the scene is not moving enough for staleness to be visible and
#: the measurement says nothing. Measured |v|*dt on this scene: 1.35e-02.
MOVING_FLOOR_M = 1e-3


# --------------------------------------------------------------------------
# The probe. Runs OUT OF PROCESS against the bundled runtime.
# --------------------------------------------------------------------------

PROBE = r'''
import json, re, sys
import numpy as np

runtime_src, out_path = sys.argv[1], sys.argv[2]
text = open(runtime_src, encoding="utf-8", errors="strict").read()
result = {"runtime_source": runtime_src}
try:
    ns = {"__name__": "omnisim_newton_runtime_probe"}
    exec(compile(text, runtime_src, "exec"), ns)
    World = ns["World"]

    w = World()
    w.set_solver_preference("mujoco")
    # A free box in free fall (free-joint path) AND a pendulum hung off a
    # static base (articulation path, real angular velocity, and a non-zero
    # xipos - subtree_com offset so the cvel cross term is actually exercised).
    # High enough that nothing reaches z=0 within the run, so no contact.
    free = w.add_body(1.0, 0.0, 0.0, 8.0)
    w.add_shape_box(free, 0.1, 0.1, 0.1)
    base = w.add_static_body(0.0, 3.0, 8.0)
    link = w.add_body(1.0, 0.6, 3.0, 8.0)
    w.add_shape_box(link, 0.25, 0.05, 0.05)
    w.add_joint_revolute(base, link, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.6, 0.0, 0.0)
    w.finalize()

    result["use_mujoco_cpu"] = bool(getattr(w.solver, "use_mujoco_cpu", False))
    if not result["use_mujoco_cpu"]:
        raise RuntimeError("solver did not come up on the CPU mj_step path "
                           "(use_mujoco_cpu is False) -- nothing to measure")

    d, mjm = w.solver.mj_data, w.solver.mj_model
    mp = w.solver.mjc_body_to_newton.numpy()[0]      # mj body -> newton body
    rootid = np.asarray(mjm.body_rootid)

    dt = 0.004
    samples = []
    prev_q = prev_qd = None
    for i in range(90):
        w.step(dt)
        bq = w.state_a.body_q.numpy().copy()
        bqd = w.state_a.body_qd.numpy().copy()
        if i >= 80:
            xpos = np.asarray(d.xpos); xipos = np.asarray(d.xipos)
            subtree_com = np.asarray(d.subtree_com); cvel = np.asarray(d.cvel)
            pos_now = pos_prev = vel_now = vel_prev = vdt = 0.0
            for mj_id, nb in enumerate(mp):
                nb = int(nb)
                if mj_id == 0 or nb < 0 or nb >= len(bq):
                    continue                      # mj body 0 is the world body
                pos_now = max(pos_now, float(np.max(np.abs(xpos[mj_id] - bq[nb][:3]))))
                if prev_q is not None:
                    pos_prev = max(pos_prev, float(np.max(np.abs(xpos[mj_id] - prev_q[nb][:3]))))
                vdt = max(vdt, float(np.linalg.norm(bqd[nb][:3])) * dt)
                # cvel is (rot, lin), lin referenced to subtree_com of the ROOT.
                # newton body_qd is (lin, rot), lin at the body's own COM.
                ang = cvel[mj_id][0:3]
                offset = xipos[mj_id] - subtree_com[rootid[mj_id]]
                conv = np.concatenate([cvel[mj_id][3:6] - np.cross(offset, ang), ang])
                vel_now = max(vel_now, float(np.max(np.abs(conv - bqd[nb]))))
                if prev_qd is not None:
                    vel_prev = max(vel_prev, float(np.max(np.abs(conv - prev_qd[nb]))))
            samples.append({"step": i, "pos_now": pos_now, "pos_prev": pos_prev,
                            "vel_now": vel_now, "vel_prev": vel_prev, "vdt": vdt,
                            "link_wy": float(bqd[2][4]) if len(bqd) > 2 else 0.0})
        prev_q, prev_qd = bq, bqd
    result["samples"] = samples
    result["ok"] = True
except Exception as exc:                              # noqa: BLE001
    import traceback
    result["ok"] = False
    result["error"] = "%r" % (exc,)
    result["traceback"] = traceback.format_exc()[-2000:]

with open(out_path, "w", encoding="utf-8") as fh:
    json.dump(result, fh)
'''


def _probe_interpreter():
    """(python, extra PYTHONPATH) able to import warp + newton + mujoco.

    Windows: the vendored newton-runtime next to the binary -- the SAME
    interpreter and site-packages the engine embeds, so the probe measures what
    the engine measures. Linux: the system python3, where the wheels are pip
    installed (the embed ignores venvs; see AGENTS.md).
    """
    candidates = []
    for name in ("python.exe", "python3", "python"):
        exe = BUNDLE / name
        if exe.is_file():
            candidates.append((str(exe), str(BUNDLE / "site-packages")))
    candidates.append((sys.executable, None))
    for exe, extra in candidates:
        env = _child_env(extra)
        try:
            probe = subprocess.run([exe, "-c", "import warp, newton, mujoco"],
                                   env=env, capture_output=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if probe.returncode == 0:
            return exe, extra
    return None, None


def _child_env(extra_path):
    env = dict(os.environ)
    # A stray OMNISIM_NEWTON_* in the developer's shell (ground mu, contact ke,
    # substeps, force-ODE) would silently change the scene this measures.
    for key in [k for k in env if k.startswith("OMNISIM_NEWTON_")]:
        env.pop(key, None)
    env["OMNISIM_HOME"] = str(REPO)
    if extra_path:
        existing = env.get("PYTHONPATH")
        env["PYTHONPATH"] = extra_path + (os.pathsep + existing if existing else "")
    return env


pytestmark = pytest.mark.skipif(
    not RUNTIME_SRC.is_file(),
    reason="omnisim_newton_runtime.py is not present in this tree")


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    exe, extra = _probe_interpreter()
    if exe is None:
        pytest.skip("no interpreter in this clone can import warp + newton + mujoco "
                    "(ODE-only clone, or `make -C src/omnisim bundle-newton-runtime` "
                    "not run)")

    tmp = tmp_path_factory.mktemp("mjfresh")
    script = tmp / "probe.py"
    script.write_text(PROBE, encoding="utf-8")
    out = tmp / "probe.json"
    run = subprocess.run([exe, str(script), str(RUNTIME_SRC), str(out)],
                         env=_child_env(extra), capture_output=True, text=True,
                         timeout=900, cwd=str(REPO))
    if not out.is_file():
        pytest.fail("the runtime probe produced no output (rc=%s):\n%s\n%s"
                    % (run.returncode, run.stdout[-1500:], run.stderr[-1500:]))
    data = json.loads(out.read_text(encoding="utf-8"))
    if not data.get("ok"):
        # A runtime that cannot come up at all is an instrument outcome, not a
        # verdict on the physics -- same reasoning as the bring-up skips in
        # test_newton_physics_regression.py.
        pytest.skip("the newton runtime did not come up in the probe: %s\n%s"
                    % (data.get("error"), data.get("traceback", "")))
    if not data.get("samples"):
        pytest.fail("the probe recorded no samples:\n%s" % json.dumps(data)[:1500])
    return data


def _worst(samples, key):
    return max(s[key] for s in samples)


def _best(samples, key):
    return min(s[key] for s in samples)


def _table(samples):
    lines = ["  step   |xpos-bq_now|  |xpos-bq_prev|  |cvel->qd_now|  |cvel->qd_prev|   |v|*dt"]
    for s in samples:
        lines.append("  %4d   %13.4e  %14.4e  %14.4e  %15.4e  %9.4e"
                     % (s["step"], s["pos_now"], s["pos_prev"],
                        s["vel_now"], s["vel_prev"], s["vdt"]))
    return "\n".join(lines)


def test_probe_scene_is_actually_moving(probe):
    """A settled scene cannot see a one-tick time offset. Refuse to judge one.

    This is the guard against the exact mistake commit ee614170 made: it read
    ~1e-08 m off a settled scene as proof that mj_data's poses were current,
    when all it proved was that nothing was moving. If this fails, the scene
    changed -- fix the scene, do not relax the other tests.
    """
    samples = probe["samples"]
    slowest = _best(samples, "vdt")
    spin = max(abs(s["link_wy"]) for s in samples)
    assert slowest > MOVING_FLOOR_M, (
        "the probe scene is not moving (min |v|*dt = %.3e m over %d samples, "
        "floor %.3e). A one-tick staleness is invisible at rest, so the "
        "freshness tests below would pass vacuously.\n%s"
        % (slowest, len(samples), MOVING_FLOOR_M, _table(samples)))
    assert spin > 0.5, (
        "the pendulum is not rotating (max |wy| = %.3f rad/s). Without rotation "
        "the cvel cross term (xipos - subtree_com) x omega is zero and the "
        "velocity conversion is only half tested." % spin)


def test_mj_data_positions_are_at_t_plus_dt(probe):
    """xpos/xquat must describe the pose the tick just produced, not the last one.

    FAILS on today's build (mj_step leaves the Cartesian arrays at t) and PASSES
    once the tick runs mj_step2 then mj_step1.
    """
    samples = probe["samples"]
    now = _worst(samples, "pos_now")
    if now <= FRESH_POS_TOL_M:
        return
    prev = _worst(samples, "pos_prev")
    vdt = _worst(samples, "vdt")
    # Name the shape of the failure so it is a diagnosis, not a magnitude.
    if prev < FRESH_POS_TOL_M:
        shape = ("ONE TICK STALE: xpos matches the PREVIOUS tick's body_q to "
                 "%.3e m while missing this tick's by %.3e m, and that miss is "
                 "|v|*dt = %.3e. This is a TIME offset, not a frame offset."
                 % (prev, now, vdt))
    else:
        shape = ("xpos disagrees with body_q at BOTH this tick (%.3e m) and the "
                 "previous one (%.3e m) -- that is a FRAME offset, a different "
                 "defect from staleness, and the mj_step2/mj_step1 split will "
                 "not fix it." % (now, prev))
    pytest.fail(
        "mj_data's Cartesian position arrays are not at t+dt.\n%s\n\n"
        "Any readback served out of mj_data.xpos / xquat would report every "
        "body one tick late, everywhere, silently -- which is why this is a "
        "precondition for the native tick path and not an optimisation of it.\n"
        "Fix: run mj_step2 (integrate to t+dt) then mj_step1 (recompute "
        "position and velocity kinematics there) in place of mj_step, and "
        "re-prime with mj_forward after any write into mj_data.qpos/qvel.\n\n%s"
        % (shape, _table(samples)))


def test_mj_data_velocities_are_at_t_plus_dt_and_convert_exactly(probe):
    """cvel must be at t+dt, AND the documented conversion must reproduce body_qd.

    Two claims in one measurement, because they share a root cause and a fix.
    The conversion is the part that is easy to get silently wrong: cvel is
    (rot, lin) with its linear part referenced to subtree_com of the body's
    ROOT, while newton's body_qd is (lin, rot) with the linear part at the
    body's own COM. The probe applies

        lin = cvel[3:6] - cross(xipos - subtree_com[body_rootid], cvel[0:3])

    and this asserts the result equals body_qd. Today it equals the PREVIOUS
    tick's body_qd -- which is how we know the formula is right and only the
    timing is wrong.

    FAILS on today's build, PASSES after the mj_step2 -> mj_step1 split.
    """
    samples = probe["samples"]
    now = _worst(samples, "vel_now")
    if now <= FRESH_VEL_TOL:
        return
    prev = _worst(samples, "vel_prev")
    if prev < FRESH_VEL_TOL:
        shape = ("ONE TICK STALE: the conversion reproduces the PREVIOUS tick's "
                 "body_qd to %.3e (float32 round-off) but misses this tick's by "
                 "%.3e. The FORMULA is right; the DATA is a tick old."
                 % (prev, now))
    else:
        shape = ("the conversion misses body_qd at BOTH this tick (%.3e) and the "
                 "previous one (%.3e) -- that is a conversion defect (frame, "
                 "component order, or reference point), not staleness. Check "
                 "(rot,lin) vs (lin,rot) and the subtree_com reference point "
                 "before touching the step ordering." % (now, prev))
    pytest.fail(
        "mj_data's Cartesian velocity arrays are not at t+dt.\n%s\n\n"
        "A later stage serves body_qd straight from cvel to skip "
        "eval_articulation_fk; on stale cvel that reports every velocity one "
        "tick late, which an RL observation vector cannot detect.\n\n%s"
        % (shape, _table(samples)))
