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

"""test_ladder_hygiene.py -- tests of the LADDER ITSELF, not of any simulator.

``selftest.py`` proves the ladder's *assertions* can go red.  This file proves
the ladder's *plumbing* is honest: that an arm which cannot be imported is
reported rather than crashing the runner, that no arm can measure another
simulator's scenes, and that no arm re-declares a scene constant the contract
already owns.

Every one of these was a real defect found after all three arms were built in
parallel, and each was invisible to the ladder's own green rows:

* ``run_ladder.load_arm`` referenced the ``except ... as exc`` name from inside
  a closure that outlives the ``except`` block.  Python unbinds it at the end
  of the block, so the *reporting* path for a broken arm raised ``NameError``
  and took the whole runner down -- which is how it masked the next one.
* All three arms shipped a module called ``worldgen`` and one of them imported
  it bare.  ``sys.modules`` is process-global, so the second arm loaded got the
  first arm's generator: it would have generated AND measured another
  simulator's scenes while reporting them as its own.
* The MuJoCo arm's committed models carried a floor half-extent from an earlier
  revision of ``rungs.FLOOR_SIZE``.  A scene that has drifted from the contract
  is measured against the wrong expectation, and the row is silently
  meaningless rather than red.

These run without a simulator except where explicitly marked; the MuJoCo cases
skip when ``mujoco`` is not importable.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types

import pytest

HERE = os.path.abspath(os.path.dirname(__file__))
MUJOCO_ARM = os.path.join(HERE, "mujoco")

if HERE not in sys.path:
    sys.path.insert(0, HERE)

import run_ladder                                   # noqa: E402
import rungs                                        # noqa: E402


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _load_by_path(name, path, extra_sys_path=()):
    """Import a module by file path under ``name``.

    ``extra_sys_path`` is prepended for the duration of the load only, so a
    module that still does a bare sibling import can be exercised without
    leaving the entry behind for the rest of the session.
    """
    added = [p for p in extra_sys_path if p not in sys.path]
    for p in added:
        sys.path.insert(0, p)
    try:
        sp = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(sp)
        sys.modules[name] = mod
        sp.loader.exec_module(mod)
        return mod
    finally:
        for p in added:
            try:
                sys.path.remove(p)
            except ValueError:
                pass


@pytest.fixture
def clean_modules():
    """Restore ``sys.modules`` and ``sys.path`` after a test that loads arms."""
    before_mods = dict(sys.modules)
    before_path = list(sys.path)
    yield
    for k in list(sys.modules):
        if k not in before_mods:
            del sys.modules[k]
    for k, v in before_mods.items():
        sys.modules[k] = v
    sys.path[:] = before_path


# ==========================================================================
# DEFECT 1 -- an arm must not re-declare a scene constant the contract owns
# ==========================================================================

@pytest.fixture(scope="module")
def mj_scenes():
    """The MuJoCo arm's scene emitter, loaded without a simulator."""
    if not os.path.isfile(os.path.join(MUJOCO_ARM, "scenes.py")):
        pytest.skip("the MuJoCo arm is not in this tree")
    return _load_by_path("ladder0_test_mj_scenes",
                         os.path.join(MUJOCO_ARM, "scenes.py"),
                         extra_sys_path=(MUJOCO_ARM,))


def _floor_size_from_mjcf(xml):
    """(hx, hy, hz) of the geom named ``floor`` in an MJCF string."""
    import re
    m = re.search(r'name="floor"[^>]*?size="([^"]+)"', xml, re.S)
    assert m, "the emitted scene has no geom named 'floor':\n%s" % xml[:400]
    return tuple(float(v) for v in m.group(1).split())


def test_mujoco_floor_is_the_contract_floor(mj_scenes):
    """Every floored rung must emit ``rungs.FLOOR_SIZE``, halved for MJCF."""
    want = tuple(v / 2.0 for v in rungs.FLOOR_SIZE)
    for rung in (1, 2, 4):
        got = _floor_size_from_mjcf(mj_scenes.mjcf(rung))
        assert got == pytest.approx(want, abs=1e-12), (
            "rung %d floor half-extent %s != rungs.FLOOR_SIZE/2 %s"
            % (rung, got, want))


def test_mujoco_committed_models_match_the_contract(mj_scenes):
    """``models/rung*.xml`` are committed artefacts; a stale one is a scene
    that has silently drifted from the expectation it is judged against."""
    for rung in rungs.RUNGS:
        path = mj_scenes.model_path(rung)
        assert os.path.isfile(path), "missing committed model %s" % path
        with open(path, "r", encoding="utf-8") as f:
            committed = f.read()
        fresh = mj_scenes.mjcf(rung)
        assert committed == fresh, (
            "%s is stale -- regenerate with `python run.py --write-models`.\n"
            "committed floor: %s\ncontract floor:  %s"
            % (os.path.relpath(path, HERE),
               _floor_size_from_mjcf(committed) if 'name="floor"' in committed
               else "(no floor)",
               _floor_size_from_mjcf(fresh) if 'name="floor"' in fresh
               else "(no floor)"))


def test_mujoco_floor_follows_the_contract_not_a_copy(mj_scenes, monkeypatch):
    """Change the contract, and the scene must change with it.

    Equality with today's value is not evidence of derivation -- a hard-coded
    copy passes that.  This moves the contract and requires the scene to move.
    """
    spec = mj_scenes.spec
    monkeypatch.setattr(spec, "FLOOR_SIZE", (31.0, 33.0, 0.5), raising=True)
    monkeypatch.setattr(spec, "FLOOR_CENTER_Z", 1.25, raising=True)
    got = _floor_size_from_mjcf(mj_scenes.mjcf(1))
    assert got == pytest.approx((15.5, 16.5, 0.25), abs=1e-12)


def test_mujoco_refuses_a_rung4_floor_the_rover_would_run_off(mj_scenes,
                                                              monkeypatch):
    """A floor shorter than the run needs must be REFUSED, not warned about.

    A rover that drives off the lip is beached with its wheels spinning, and
    that signature -- short distance, wheels turning, roll ratio < 1 -- reads
    exactly like a traction defect.  It masked a real engine defect once
    already, so a scene that cannot hold the whole run is not a scene this arm
    is allowed to emit.
    """
    spec = mj_scenes.spec
    need = mj_scenes.RUNG4_REQUIRED_HALF_X
    short = need * 2.0 * 0.5          # a floor exactly at the requirement...
    monkeypatch.setattr(spec, "FLOOR_SIZE", (2.0 * short, 20.0, 0.2))
    mj_scenes.mjcf(4)                 # ... is allowed
    monkeypatch.setattr(spec, "FLOOR_SIZE", (2.0 * (need - 0.5), 20.0, 0.2))
    with pytest.raises(Exception) as exc:
        mj_scenes.mjcf(4)             # ... and half a metre short is not
    assert "floor" in str(exc.value).lower()


def test_no_arm_hard_codes_a_floor_half_extent():
    """No module under ``mujoco/`` may carry its own floor size constant."""
    import re
    bad = []
    for root, dirs, files in os.walk(MUJOCO_ARM):
        dirs[:] = [d for d in dirs
                   if d not in ("__pycache__", "models", "results")]
        for fn in files:
            if not fn.endswith(".py"):
                continue
            p = os.path.join(root, fn)
            with open(p, "r", encoding="utf-8") as f:
                for ln, line in enumerate(f, 1):
                    if re.search(r"FLOOR_HALF_X\s*=\s*[0-9]", line) or \
                            re.search(r"floor_half_x\s*=\s*[0-9]", line):
                        bad.append("%s:%d %s" % (os.path.relpath(p, HERE), ln,
                                                 line.strip()))
    assert not bad, ("a floor extent is declared outside rungs.py:\n  %s"
                     % "\n  ".join(bad))


@pytest.mark.parametrize("arm_name", ["webots", "omnisim"])
def test_wbt_arms_derive_their_scenes_from_the_contract(arm_name, monkeypatch,
                                                        clean_modules):
    """The two ``.wbt`` arms must emit the contract's numbers, not copies.

    ``omnisim/`` is owned by another lane and is only READ here.
    """
    gen_path = os.path.join(HERE, arm_name, "worldgen.py")
    if not os.path.isfile(gen_path):
        pytest.skip("no %s arm in this tree" % arm_name)
    try:
        wg = _load_by_path("ladder0_test_wg_%s" % arm_name, gen_path)
    except Exception as exc:                        # noqa: BLE001
        pytest.skip("%s worldgen is not importable right now: %r"
                    % (arm_name, exc))
    # The generator holds its own reference to the contract module.
    contract = wg.rungs
    monkeypatch.setattr(contract, "FLOOR_SIZE", (13.0, 17.0, 0.3))
    monkeypatch.setattr(contract, "BOX_EDGE", 0.37)
    monkeypatch.setattr(contract, "WHEEL_R", 0.11)
    assert "13 17 0.3" in wg.wbt(1), "the floor does not follow FLOOR_SIZE"
    assert "0.37 0.37 0.37" in wg.wbt(1), "the box does not follow BOX_EDGE"
    assert "radius 0.11" in wg.wbt(4), "the wheel does not follow WHEEL_R"


# ==========================================================================
# DEFECT 2 -- a broken arm must be REPORTED, not crash the runner
# ==========================================================================

BROKEN_ARM = '''\
raise RuntimeError("deliberate import failure: bad_arm_2026")
'''


def test_load_arm_reports_an_unimportable_arm(tmp_path, monkeypatch,
                                              clean_modules):
    """An arm whose module raises at import is an UNAVAILABLE arm.

    Before the fix, ``available()`` closed over the ``except ... as exc`` name,
    which Python unbinds at the end of the block: calling it raised
    ``NameError`` and took the runner down.  "The arm is broken" then read as
    "the ladder is broken", and the crash masked a second defect.
    """
    root = str(tmp_path)
    _write(os.path.join(root, "brokenarm", "arm.py"), BROKEN_ARM)
    monkeypatch.setattr(run_ladder, "HERE", root)

    arm = run_ladder.load_arm("brokenarm")
    assert arm is not None, "an arm.py that exists must not read as absent"
    assert arm.NAME == "brokenarm"

    ok, why = arm.available()           # this is where the NameError fired
    assert ok is False
    assert "bad_arm_2026" in why, why

    samples, meta = arm.run(0, root)    # and this is the other closure
    assert meta["error"] and "bad_arm_2026" in meta["error"]
    assert meta["exit_code"] is None
    assert samples["steps"] == 0


def test_a_broken_arm_does_not_abort_the_run(tmp_path, monkeypatch,
                                             clean_modules, capsys):
    """End to end: a broken arm is listed under ARMS NOT RUN and the runner
    still exits with a verdict rather than a traceback."""
    root = str(tmp_path)
    _write(os.path.join(root, "brokenarm", "arm.py"), BROKEN_ARM)
    monkeypatch.setattr(run_ladder, "HERE", root)
    monkeypatch.setattr(run_ladder, "ARM_NAMES", ("brokenarm",))
    monkeypatch.setattr(run_ladder, "RESULTS", os.path.join(root, "results"))

    rc = run_ladder.main(["--out", os.path.join(root, "out")])
    out = capsys.readouterr().out
    assert rc != 0, "no cell ran, so the run cannot be a pass"
    assert "ARMS NOT RUN" in out
    assert "bad_arm_2026" in out


def test_load_arm_returns_none_for_an_absent_arm(tmp_path, monkeypatch):
    monkeypatch.setattr(run_ladder, "HERE", str(tmp_path))
    assert run_ladder.load_arm("nosucharm") is None


# ==========================================================================
# DEFECT 3 -- one arm must never load another arm's modules
# ==========================================================================

COLLIDING_ARM = '''\
import os, sys
HERE = os.path.abspath(os.path.dirname(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import worldgen                       # the bare import that collides

NAME = "%(name)s"


def available():
    return True, None


def run(rung, out_dir, **kw):
    return ({"rung": rung, "sim": NAME, "t": [], "steps": 0},
            {"exit_code": 0, "error": None})
'''

ARM_WORLDGEN = 'TAG = "%(name)s"\n'


def _two_colliding_arms(root):
    for name in ("alpha", "beta"):
        _write(os.path.join(root, name, "arm.py"),
               COLLIDING_ARM % {"name": name})
        _write(os.path.join(root, name, "worldgen.py"),
               ARM_WORLDGEN % {"name": name})


def test_an_arm_cannot_load_another_arms_worldgen(tmp_path, monkeypatch,
                                                  clean_modules):
    """The exact historical shape: two arms, both with a module called
    ``worldgen``, both importing it bare.  ``sys.modules`` is process-global,
    so the SECOND arm loaded silently gets the FIRST arm's generator -- and
    would then generate and measure another simulator's scenes while
    reporting them as its own.

    Either the runner refuses the second arm, or the second arm really is
    holding the first arm's module.  There is no acceptable third outcome.
    """
    root = str(tmp_path)
    _two_colliding_arms(root)
    monkeypatch.setattr(run_ladder, "HERE", root)

    alpha = run_ladder.load_arm("alpha")
    assert alpha.worldgen.TAG == "alpha"

    refused = None
    try:
        beta = run_ladder.load_arm("beta")
    except Exception as exc:                        # noqa: BLE001
        refused = exc
    else:
        assert beta.worldgen.TAG == "beta", (
            "arm 'beta' is holding arm 'alpha's worldgen (%s) and the runner "
            "did not object -- this is the cross-arm collision"
            % beta.worldgen.__file__)

    assert refused is not None, (
        "the runner accepted an arm that had loaded another arm's module")
    msg = str(refused)
    assert "beta" in msg and "alpha" in msg and "worldgen" in msg, msg


def test_the_collision_guard_can_be_satisfied(tmp_path, monkeypatch,
                                              clean_modules):
    """The guard must pass an arm that loads its sibling BY PATH under an
    arm-qualified name -- the fix the Webots arm already carries.  A guard
    that fired on the correct pattern too would just be noise."""
    root = str(tmp_path)
    good = '''\
import importlib.util, os, sys
HERE = os.path.abspath(os.path.dirname(__file__))
_sp = importlib.util.spec_from_file_location(
    "ladder0_gamma_worldgen", os.path.join(HERE, "worldgen.py"))
worldgen = importlib.util.module_from_spec(_sp)
sys.modules["ladder0_gamma_worldgen"] = worldgen
_sp.loader.exec_module(worldgen)

NAME = "gamma"


def available():
    return True, None


def run(rung, out_dir, **kw):
    return {}, {}
'''
    _write(os.path.join(root, "alpha", "arm.py"),
           COLLIDING_ARM % {"name": "alpha"})
    _write(os.path.join(root, "alpha", "worldgen.py"),
           ARM_WORLDGEN % {"name": "alpha"})
    _write(os.path.join(root, "gamma", "arm.py"), good)
    _write(os.path.join(root, "gamma", "worldgen.py"),
           ARM_WORLDGEN % {"name": "gamma"})
    monkeypatch.setattr(run_ladder, "HERE", root)

    run_ladder.load_arm("alpha")
    gamma = run_ladder.load_arm("gamma")            # must not raise
    assert gamma.worldgen.TAG == "gamma"


def test_shared_contract_modules_are_not_flagged(tmp_path, monkeypatch,
                                                 clean_modules):
    """An arm holding ``rungs``/``analysis`` is doing exactly what CONTRACT.md
    section 2 tells it to; the guard must not mistake the shared layer for a
    foreign arm."""
    root = str(tmp_path)
    _write(os.path.join(root, "rungs.py"), "REST_Z = 0.6\n")
    _write(os.path.join(root, "analysis.py"), "def reduce_samples(*a): pass\n")
    _write(os.path.join(root, "delta", "arm.py"), '''\
import os, sys
LADDER0 = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
LADDER0 = os.path.dirname(os.path.abspath(__file__))
LADDER0 = os.path.dirname(LADDER0)
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)
import rungs
import analysis

NAME = "delta"


def available():
    return True, None


def run(rung, out_dir, **kw):
    return {}, {}
''')
    monkeypatch.setattr(run_ladder, "HERE", root)
    arm = run_ladder.load_arm("delta")              # must not raise
    assert arm.rungs.REST_Z == 0.6


def test_real_arms_pass_the_guard(clean_modules):
    """Loading the arms that actually ship, in the runner's own order, must
    not trip the guard.  This is the regression that keeps the fix honest:
    the guard is only worth having if the corrected tree is clean under it."""
    loaded = []
    for name in run_ladder.ARM_NAMES:
        arm = run_ladder.load_arm(name)             # raises on a collision
        if arm is not None:
            loaded.append(name)
    assert loaded, "no arm at all could be loaded"


# ==========================================================================
# The MuJoCo arm must produce the sample document CONTRACT.md section 4 asks
# for -- an omitted series is judged RED, which is correct, but it means the
# arm is measuring less than it claims to.
# ==========================================================================

# rung -> the series the reducer needs.  CONTRACT.md section 4.
REQUIRED_SERIES = {
    1: ("box_z",),
    2: ("box_z",),
    3: ("joint_q",),
    4: ("body_x", "body_y", "body_z"),
}


@pytest.mark.parametrize("rung", sorted(REQUIRED_SERIES))
def test_mujoco_sample_document_is_complete(rung):
    pytest.importorskip("mujoco")
    run_mod = _load_by_path("ladder0_test_mj_run",
                            os.path.join(MUJOCO_ARM, "run.py"),
                            extra_sys_path=(MUJOCO_ARM,))
    samples, _meta = run_mod.run_rung(rung)
    n = len(samples["t"])
    assert n > 0
    for key in REQUIRED_SERIES[rung]:
        assert key in samples, (
            "rung %d omits %r, so the reducer cannot measure it and the check "
            "that reads it is RED for want of a number rather than because "
            "the physics is wrong" % (rung, key))
        assert len(samples[key]) == n, (
            "rung %d: %s has %d samples but t has %d"
            % (rung, key, len(samples[key]), n))
    if rung == 4:
        assert set(samples["wheel_q"]) == set(rungs.WHEEL_TAGS)
        for tag, series in samples["wheel_q"].items():
            assert len(series) == n, "wheel %s series is short" % tag


def test_module_graph_helper_is_bounded():
    """The guard walks a module graph; a cycle must not hang it."""
    a = types.ModuleType("cyc_a")
    b = types.ModuleType("cyc_b")
    a.__file__ = os.path.join(HERE, "alpha", "a.py")
    b.__file__ = os.path.join(HERE, "alpha", "b.py")
    a.other = b
    b.other = a
    assert run_ladder.check_arm_modules("alpha", a, root=HERE) == []


# ==========================================================================
# TIER C/E -- the machinery rungs 9, 11 and 18 added
#
# Every test below is a defect that would be INVISIBLE to a green row: a
# perturbation the scene file cannot express, a subset an arm chose for
# itself, a refusal and a measurement of the same quantity, or a second
# implementation of a ruler that already exists.
# ==========================================================================

def test_rung9_replicas_a_and_b_are_the_SAME_file():
    """They are not two copies of one scene; they are one scene run twice.

    A byte difference between them -- even in a comment -- is a difference the
    rung exists to rule out, and it would present as an engine that is not
    reproducible.
    """
    wg = _load_by_path("ladder0_test_wg_r9",
                       os.path.join(HERE, "omnisim", "worldgen.py"))
    a = wg.world_path(9, "none", "a")
    b = wg.world_path(9, "none", "b")
    c = wg.world_path(9, "none", "c")
    assert a == b, "rung 9's replicas must resolve to ONE world file"
    assert c != a, "the sensitivity control must be a DIFFERENT scene"


def test_rung9_fault_nudge_survives_the_scene_file():
    """The 0.1 um fault must still be there after a round trip through the
    ``.wbt``.

    MEASURED, and it is why this test exists: at 1e-12 m the fault produced a
    BITWISE IDENTICAL run and the self-test reported a ladder defect that was
    not one.  Two things can eat it -- a coarse number formatter here, and the
    engine's own single-precision pose path -- and only the first is ours.
    """
    import re
    import struct
    wg = _load_by_path("ladder0_test_wg_r9n",
                       os.path.join(HERE, "omnisim", "worldgen.py"))
    honest = wg.wbt(9, "none", {"tag": "a", "eps_m": 0.0, "nudge_m": 0.0})
    nudged = wg.wbt(9, "seed_nudge",
                    {"tag": "b", "eps_m": 0.0,
                     "nudge_m": rungs.RUNG9_FAULT_NUDGE})

    def drop_x(text):
        m = re.search(r"DEF DROP Solid \{\s*\n\s*translation (\S+)", text)
        assert m, "no DEF DROP translation in the emitted scene"
        return float(m.group(1))

    d = drop_x(nudged) - drop_x(honest)
    assert d != 0.0, "the fault nudge was formatted away to zero"
    # Compared RELATIVELY: differencing two coordinates near 0.5 cancels ~7
    # significant digits, so an absolute bound here would be testing float64
    # subtraction rather than the formatter.
    assert abs(d - rungs.RUNG9_FAULT_NUDGE) < 1e-6 * rungs.RUNG9_FAULT_NUDGE, (
        "the emitted nudge is %r, not RUNG9_FAULT_NUDGE %r"
        % (d, rungs.RUNG9_FAULT_NUDGE))
    # And it must clear single precision at that coordinate, because the pose
    # is MEASURED to reach the solver as float32 (a cube authored at x = 0.502
    # reads back as float32(0.502)).
    x = drop_x(honest)
    bits = struct.unpack("<I", struct.pack("<f", x))[0]
    f = struct.unpack("<f", struct.pack("<I", bits))[0]
    nxt = struct.unpack("<f", struct.pack("<I", bits + 1))[0]
    ulp = abs(nxt - f)
    # Strictly greater than ONE ulp is a guarantee rather than a margin: a
    # nudge that large changes the float32 representation wherever the
    # coordinate happens to sit between two representable values.
    assert rungs.RUNG9_FAULT_NUDGE > ulp, (
        "RUNG9_FAULT_NUDGE %g does not clear one float32 ulp (%g) at x = %g, "
        "so whether the fault fires depends on where the coordinate happens "
        "to sit between two representable values"
        % (rungs.RUNG9_FAULT_NUDGE, ulp, x))


def test_rung11_fleet_follows_the_contract_not_a_copy(monkeypatch,
                                                      clean_modules):
    """Move the contract's lane spacing and the emitted scene must move.

    Equality with today's number is not evidence of derivation -- a hard-coded
    copy passes an equality check, which is how the MuJoCo arm's committed
    models came to sit at a floor size from an earlier revision.
    """
    wg = _load_by_path("ladder0_test_wg_r11",
                       os.path.join(HERE, "omnisim", "worldgen.py"))
    contract = wg.rungs
    monkeypatch.setattr(contract, "RUNG11_LANE_DY", 2.75)
    text = wg.wbt(11, "none", {"tag": "n4", "n": 4})
    assert "translation 0 -4.125" in text, (
        "rung 11's lanes do not follow RUNG11_LANE_DY")
    monkeypatch.setattr(contract, "RUNG11_NJMAX", 8191)
    text = wg.wbt(11, "none", {"tag": "n4", "n": 4})
    assert "newtonNjmax 8191" in text, (
        "the declared constraint budget is not read from the contract")


def test_rung11_budget_is_generous_never_the_peak():
    """CONTRACT.md amendment E, as an assertion rather than a caution.

    Sizing a constraint budget at a scene's MEASURED PEAK moved results 8.81 m
    versus every other size, with a 1.71 m run-to-run spread, while
    512 / 2048 / 4096 agreed to 1e-4.
    """
    peak = rungs.RUNG11_ROWS_PER_ROBOT * max(
        rungs.RUNG11_N + (rungs.RUNG11_N_VARIANT,))
    assert rungs.RUNG11_NJMAX >= 2 * peak, (
        "declared njmax %d is not generous against a %d-row peak"
        % (rungs.RUNG11_NJMAX, peak))


def test_rung18_subset_must_be_contract_owned():
    """An arm free to pick its own tosses could pick the easy ones.

    The reducer leaves ``tosses_missing`` unmeasured for any other subset, and
    an unmeasured quantity is RED -- never skipped.
    """
    import analysis
    good = {"rung": 18, "requested": list(rungs.RUNG18_INDICES), "runs": []}
    bad = {"rung": 18, "requested": [0, 1, 2], "runs": []}
    assert analysis.reduce_samples(good).get("tosses_missing") is not None
    m = analysis.reduce_samples(bad)
    assert m.get("tosses_missing") is None
    assert not m["subset_is_contract_owned"]
    chk = {c.name: c for c in rungs.check_rung(18, m)}["tosses_unscored"]
    assert not chk.ok, "an off-contract subset must be RED for provenance"


def test_rung18_reuses_lane1r_and_declares_no_constant_of_its_own():
    """No physical constant of the dataset may be re-declared in this ladder.

    lane1r owns the cube's mass, geometry, inertia, sampling rate and
    calibration, and re-derives the last of those on every run.  A second copy
    here is a place for the two to disagree, and the disagreement would present
    as "OmniSim does not match reality".
    """
    D = rungs.rung18_dataset()
    for name in ("CUBE_EDGE_M", "CUBE_MASS_KG", "CUBE_INERTIA", "SAMPLE_HZ"):
        assert hasattr(D, name), "lane1r lost %s" % name
        assert not hasattr(rungs, "RUNG18_" + name), (
            "rungs.py re-declares the dataset's %s" % name)


def test_rung18_reducer_agrees_with_lane1r_scorer():
    """One ruler, two implementations -- and they must agree to a hair.

    ``analysis`` is stdlib-only because the ladder must run on a bare python;
    lane1r's scorer is numpy.  That is a real second implementation of the same
    metric, so it is pinned rather than trusted.
    """
    np = pytest.importorskip("numpy")
    import analysis
    D = rungs.rung18_dataset()
    lane1r_dir = os.path.join(os.path.dirname(HERE), "omnibench", "lane1r")
    score = _load_by_path("ladder0_test_lane1r_score",
                          os.path.join(lane1r_dir, "score.py"),
                          extra_sys_path=(lane1r_dir,))
    try:
        ref = D.load(0, scale="none")
    except Exception as exc:                        # noqa: BLE001
        pytest.skip("lane1r dataset not loadable here: %r" % (exc,))

    top = 0.5
    # A synthetic "replay": the recording itself, lifted onto the world's table
    # and pushed 7 mm off in x.  The expected error is then known in CLOSED
    # FORM, so this pins both implementations to the metric rather than one of
    # them to the other.
    off = 0.007
    t_sim = [float(v) for v in ref["t"]]
    p_sim = [[float(p[0]) + off, float(p[1]), float(p[2]) + top]
             for p in ref["pos"]]
    q_sim = [[float(c) for c in q] for q in ref["quat"]]
    run = {"tag": "toss0000", "params": {"index": 0},
           "t": t_sim, "pos": p_sim, "quat": q_sim,
           "reference": {"index": 0, "t": t_sim,
                         "pos": [[float(c) for c in p] for p in ref["pos"]],
                         "quat": q_sim, "cube_edge_m": D.CUBE_EDGE_M,
                         "table_top": top}}
    m = analysis.reduce_samples({"rung": 18,
                                 "requested": list(rungs.RUNG18_INDICES),
                                 "runs": [run]})
    expect_pct = 100.0 * off / D.CUBE_EDGE_M
    assert abs(m["real_pos_err"] - expect_pct) < 1e-9, (
        "the ladder's position metric is not |dp| / cube width")
    # 1e-6 deg, not 0: the geodesic angle goes through acos, whose derivative
    # is unbounded at 1, so an identical pair scores ~1e-8 deg of round-off.
    assert m["real_rot_err"] < 1e-6, "an identical orientation must score 0 deg"
    P, _Q = score.resample(np.asarray(t_sim),
                           np.asarray([[p[0], p[1], p[2] - top]
                                       for p in p_sim]),
                           np.asarray(q_sim), ref["t"])
    dp = np.linalg.norm(P - ref["pos"], axis=1)
    assert abs(float((100.0 * dp / D.CUBE_EDGE_M).mean()) - expect_pct) < 1e-9


def test_not_expressible_rules():
    """CONTRACT.md amendment B, in four assertions.

    The one that matters most is the third: a refusal and a measurement of the
    same quantity cannot both be true, and whichever is wrong the row is not a
    measurement of anything -- so it stops the run rather than being reported.
    """
    # ``finite_clock`` is deliberately NOT measured here: rule 3 says an arm
    # may not declare a refusal for something it also produced a number for,
    # so a well-formed N/E case has to leave that key out.
    m = {"steps": float(rungs.RUNG0_STEPS), "exit_code": 0.0}
    measured = dict(m, sim_time_end=rungs.RUNG0_STEPS * rungs.DT)
    good = {"missing": "no such device on this engine", "citation": "x.md:1",
            "status": "CITED"}

    ne = rungs.apply_not_expressible(rungs.check_rung(0, m),
                                     {"finite_clock": good})
    got = {c.name: c for c in ne}
    assert got["finite_clock"].not_expressible == good, (
        "a valid declaration must mark the check N/E")
    assert got["steps_completed"].not_expressible is None

    # ...but only when the quantity was NOT measured.
    with pytest.raises(rungs.NotExpressible):
        rungs.apply_not_expressible(rungs.check_rung(0, measured),
                                    {"finite_clock": good})

    # A declaration without evidence is RED, never skipped.
    bare = rungs.apply_not_expressible(rungs.check_rung(0, m),
                                       {"finite_clock": {"missing": "dunno"}})
    fc = {c.name: c for c in bare}["finite_clock"]
    assert fc.not_expressible is None and fc.ne_invalid
    assert fc.as_dict().get("not_expressible") is None

    # An N/E cell is PARTIAL -- never a pass, never a failure.
    cell = {"checks": [c.as_dict() for c in ne], "meta": {}}
    assert run_ladder.cell_verdict(cell) == "PARTIAL"
    allne = rungs.apply_not_expressible(
        rungs.check_rung(0, {}),
        {c.name: good for c in rungs.check_rung(0, {})})
    assert run_ladder.cell_verdict({"checks": [c.as_dict() for c in allne],
                                    "meta": {}}) == "NOT_EXPRESSIBLE"


def test_embed_gap_is_never_computed_against_ourselves():
    """Rung 18's headline is a CROSS-ARM check and it must stay one.

    ``|its error - its own error|`` is zero by construction, so a reference arm
    that scored itself would report a free green on the one row in this ladder
    that is supposed to be unwinnable.
    """
    m = {"per_toss": {"0": {"pos_pct": 20.0}, "1": {"pos_pct": 30.0}}}
    ref = {"per_toss": {"0": {"pos_pct": 22.0}, "1": {"pos_pct": 25.0}}}
    chk = rungs.check_rung18_embed_gap(m, ref)
    assert abs(chk.measured - 1.5) < 1e-12
    # No shared tosses -> unmeasured -> RED, never a pass.
    assert rungs.check_rung18_embed_gap(m, {"per_toss": {}}).measured is None
    cells = [{"rung": 18, "sim": run_ladder.EMBED_REFERENCE, "_m": ref,
              "checks": [], "meta": {}},
             {"rung": 18, "sim": "omnisim", "_m": m, "checks": [], "meta": {}}]
    run_ladder.cross_cell_checks(cells)
    assert not [c for c in cells[0]["checks"] if c["name"] == "embed_gap"], (
        "the reference arm must not be handed its own gap")
    assert [c for c in cells[1]["checks"] if c["name"] == "embed_gap"]


def test_sample_document_round_trips_float64_exactly():
    """CONTRACT.md amendment D.

    A document written through ``"%.6f"`` -- or recorded as float32 -- would
    make any engine look deterministic to six decimals, which is the one thing
    rung 9 must never be able to report.
    """
    import json
    vals = [0.10000000000000001, 0.50199997425079346, 1.6000000000000001,
            rungs.RUNG9_EPS, rungs.RUNG9_FAULT_NUDGE]
    back = json.loads(json.dumps({"t": vals}))["t"]
    assert back == vals, "json did not round-trip float64"
    assert [float("%.6f" % v) for v in vals] != vals, (
        "the counterexample no longer distinguishes the two formats")
