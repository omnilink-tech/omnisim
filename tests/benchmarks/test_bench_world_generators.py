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

"""Gate: every benchmark's OWN world generator must still produce a world.

WHY THIS EXISTS
---------------
Nothing in this tree ever ran the benchmark tooling's world generators, and a
broken one shipped: `omnibench/step_cost/run_step_cost.py`'s WorldInfo template
carried a leftover `%s` with no argument behind it, so `gen_world(...)` raised
`TypeError: not enough arguments for format string` — the bench could not
produce a single scene. It was found by hand, not by a test, because no test
imported it. Every bench here that builds its own scenes is one edit away from
the same failure, and it surfaces late (a benchmark session that dies on its
first world) or silently (a malformed world the engine loads differently than
intended).

WHAT IT COVERS
--------------
Each generator is imported and asked for at least one world, then the output is
checked for the failure shapes an unexercised string template actually produces:
unconsumed `%`-placeholders, unsubstituted `{field}` slots, unbalanced braces, a
missing `WorldInfo`, and an empty file. Plus one physics-honesty ratchet:
a generated bench world must not be `coordinateSystem "NUE"` — until commit
c77cbe98 the engine projected NUE gravity to exactly zero and stood the implicit
floor up as a vertical wall, so a NUE bench world measured a frozen scene.

CHEAP BY CONSTRUCTION: no engine launch, no GPU, no network, no controller
build. This is string building. Output goes to tmp_path; the generators whose
output dir is a module global are monkeypatched so nothing lands in the repo.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent          # tests/benchmarks
_OMNIBENCH = _HERE / "omnibench"


def _load(path: Path, name: str):
    """Import a benchmark module by file path (they are scripts, not packages)."""
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# the shared plausibility check
# ---------------------------------------------------------------------------

# A leftover printf slot. `.wbt` has no legitimate use for `%`, so any of these
# means a template argument was dropped (the failure mode that shipped).
_PRINTF_LEFTOVER = re.compile(r"%[-+ #0-9.]*[sdiouxXeEfgGrc%]")
# A leftover str.format slot: `{identifier}` with no whitespace. Real VRML braces
# always open a node body, i.e. they are followed by whitespace/newline.
_FORMAT_LEFTOVER = re.compile(r"\{[A-Za-z_][A-Za-z0-9_]*\}")
_QUOTED = re.compile(r'"(?:[^"\\]|\\.)*"')


def assert_plausible_world(text: str, label: str):
    assert text.strip(), "%s: generator produced an EMPTY world" % label
    first = text.lstrip().splitlines()[0]
    assert first.startswith("#"), (
        "%s: first line is not a `#...` file header, got %r" % (label, first))
    assert "WorldInfo" in text, "%s: no WorldInfo node" % label

    m = _PRINTF_LEFTOVER.search(text)
    assert m is None, (
        "%s: unconsumed printf placeholder %r at offset %d — a template argument "
        "was dropped (this is exactly how the step_cost generator shipped broken)"
        % (label, m.group(0), m.start()))
    m = _FORMAT_LEFTOVER.search(text)
    assert m is None, (
        "%s: unsubstituted format slot %r — a .format()/f-string field never got "
        "a value" % (label, m.group(0)))

    # Brace balance, ignoring anything inside a quoted string.
    stripped = _QUOTED.sub('""', text)
    opens, closes = stripped.count("{"), stripped.count("}")
    assert opens == closes, (
        "%s: unbalanced braces (%d '{' vs %d '}') — the engine will refuse this "
        "world with a parse error" % (label, opens, closes))

    cs = re.search(r'coordinateSystem\s+"([A-Z]+)"', text)
    if cs:
        assert cs.group(1) != "NUE", (
            "%s: generated world is coordinateSystem \"NUE\". Before commit "
            "c77cbe98 the Newton backend never read that field: gravity projected "
            "to exactly zero and the implicit floor stood up as a vertical wall, "
            "so a NUE bench world measured a FROZEN SCENE. Generate ENU."
            % label)


# ---------------------------------------------------------------------------
# omnibench lane 1 — the T1..T7 correctness scenes
# ---------------------------------------------------------------------------

def test_lane1_gen_worlds_produces_every_scene(tmp_path, monkeypatch):
    """lane1/gen_worlds.py must emit one plausible world per scene at one dt."""
    gw = _load(_OMNIBENCH / "lane1" / "gen_worlds.py", "ob_gen_worlds")
    monkeypatch.setattr(gw, "WORLDS_DIR", str(tmp_path))
    written = gw.generate(dts=[4], force=True)
    # T2 fans out into one world per incline angle, so this is > len(TESTS).
    assert len(written) >= len(gw.TESTS), (
        "generated %d worlds for %d tests" % (len(written), len(gw.TESTS)))
    for path in written:
        assert Path(path).parent == tmp_path, (
            "generator wrote OUTSIDE the tmp dir: %s" % path)
        assert_plausible_world(Path(path).read_text(encoding="utf-8"),
                               "lane1 " + Path(path).name)


# ---------------------------------------------------------------------------
# omnibench step_cost — the generator that actually shipped broken
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("solver", ["mujoco", "mujoco_warp", ""])
@pytest.mark.parametrize("airborne", [False, True])
def test_step_cost_gen_world(tmp_path, solver, airborne):
    """`solver=""` (omit the field, take the shipped default) is the exact arm
    whose template was broken, so every solver/contact combination is built."""
    rsc = _load(_OMNIBENCH / "step_cost" / "run_step_cost.py", "ob_step_cost")
    out = tmp_path / ("sc_%s_%s.wbt" % (solver or "default", int(airborne)))
    rsc.gen_world(out, bodies=2, dt_ms=4.0, steps=10, airborne=airborne,
                  solver=solver)
    text = out.read_text(encoding="utf-8")
    assert_plausible_world(text, "step_cost solver=%r airborne=%s"
                           % (solver, airborne))
    if solver:
        assert 'newtonSolver "%s"' % solver in text
    else:
        assert "newtonSolver" not in text, (
            'solver="" must OMIT the field so the world takes the shipped default')


def test_step_cost_help_is_printable(capsys, monkeypatch):
    """`--help` must not die on the console encoding.

    It did: a decorative U+26A0 in the module docstring (argparse's
    `description`) made `run_step_cost.py --help` raise UnicodeEncodeError under
    Windows cp1252 — the first command a stranger runs. The docstring is the
    thing written to stdout, so it has to survive the narrowest encoding we ship
    on.
    """
    rsc = _load(_OMNIBENCH / "step_cost" / "run_step_cost.py", "ob_step_cost_help")
    rsc.__doc__.encode("cp1252")          # raises if a glyph slipped back in
    monkeypatch.setattr(sys, "argv", ["run_step_cost.py", "--help"])
    with pytest.raises(SystemExit) as exc:
        rsc.main()
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "--solver" in out
    # The help text used to say the shipped default "is SolverXPBD". XPBD was
    # REMOVED (94f04222) and the default flipped to CPU MuJoCo (7b431e81), so a
    # reader following that sentence would pick the wrong contrast arm. Assert the
    # correction by its commit citations rather than by wrapped substrings.
    assert "SolverXPBD" not in out, "--help still names the removed XPBD default"
    assert "94f04222" in out and "7b431e81" in out, (
        "--help must cite the XPBD removal + the default flip, got:\n" + out)


def test_run_all_help_is_printable():
    """Same rule for the suite's top-level entry point, which had the same bug."""
    ra = _load(_OMNIBENCH / "run_all.py", "ob_run_all")
    ra.__doc__.encode("cp1252")


# ---------------------------------------------------------------------------
# optim_bench — the multi-instance / throughput scene generators
# ---------------------------------------------------------------------------

def _optim(monkeypatch, tmp_path):
    ob = _load(_HERE / "optim_bench.py", "optim_bench_gate")
    monkeypatch.setattr(ob, "WORLDS_DIR", tmp_path)
    return ob


@pytest.mark.parametrize("call", [
    ("gen_many_robots", (2, 10), {}),
    ("gen_many_cameras", (1, 10), {}),
    ("gen_noisy_robots", (2, 1024, 10), {}),
    ("gen_chunky_robots", (2, 1024, 10), {}),
    ("gen_multi_instance_world", (10,), {}),
])
def test_optim_bench_generators(tmp_path, monkeypatch, call):
    name, args, kwargs = call
    ob = _optim(monkeypatch, tmp_path)
    out = Path(getattr(ob, name)(*args, **kwargs))
    assert out.parent == tmp_path, "generator wrote OUTSIDE the tmp dir: %s" % out
    assert_plausible_world(out.read_text(encoding="utf-8"), "optim " + name)


def test_optim_bench_supervisor_block_carries_its_args(tmp_path, monkeypatch):
    """The supervisor node must actually receive --steps (and the optional
    state-out args): the template field was renamed and three of four call sites
    kept passing the OLD name, which raised KeyError instead of emitting a world.
    """
    ob = _optim(monkeypatch, tmp_path)
    out = Path(ob.gen_many_robots(2, 25, state_out=tmp_path / "state.json"))
    text = out.read_text(encoding="utf-8")
    assert 'controllerArgs ["--steps" "25"' in text, text[:400]
    assert "--state-out" in text and "--robot-count" in text
    assert_plausible_world(text, "optim gen_many_robots(state_out=...)")


def test_optim_bench_refuses_the_deleted_backend(tmp_path, monkeypatch):
    """NEGATIVE CONTROL: pinning the deleted backend must RAISE, never generate.

    A world pinned to `physicsBackend "ode"` does not simulate (src/ode was
    DELETED, commit bdc02139) — bodies never move — so a throughput number from
    one would be void rather than merely wrong.
    """
    ob = _optim(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match="bdc02139"):
        ob.gen_many_robots(2, 10, backend="ode")
    assert not [*tmp_path.glob("*.omniworld"), *tmp_path.glob("*.wbt")], "a world was written anyway"


# ---------------------------------------------------------------------------
# omnibench lane 3 — the static scene + the DELIBERATELY broken probe world
# ---------------------------------------------------------------------------

def test_lane3_static_world_is_plausible():
    drv_world = _OMNIBENCH / "lane3" / "worlds" / "lane3_drive.wbt"
    assert drv_world.is_file(), "lane 3c's world is missing: %s" % drv_world
    assert_plausible_world(drv_world.read_text(encoding="utf-8"), "lane3_drive")


def test_lane3_broken_probe_world_is_still_broken():
    """lane 3c probes the harness's structured diagnostics with a world that is
    MEANT to fail parsing. Assert it stays broken — a 'fixed' one would silently
    turn that probe into a no-op that always passes."""
    drv = _load(_OMNIBENCH / "lane3" / "driveability.py", "ob_driveability")
    text = drv.BROKEN_WBT
    assert text.strip(), "the broken-world template is empty"
    stripped = _QUOTED.sub('""', text)
    assert stripped.count("{") != stripped.count("}"), (
        "BROKEN_WBT now has balanced braces — it may no longer be a parse "
        "error, which would make the structured-diagnostic probe vacuous")
