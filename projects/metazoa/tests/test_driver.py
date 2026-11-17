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

"""metazoa.py / watch_metazoa.py / mz.scene: the driver plans without A/B/C,
writes nothing on --dry-run, conserves cells, scores + selects per DESIGN,
honours the thermal protocol, and every scene it emits has balanced braces."""
import importlib
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

from mz import scene as S  # noqa: E402
import metazoa as MZ  # noqa: E402


# ------------------------------------------------------------------ scene

def test_scene_starts_at_the_viewpoint_and_is_balanced():
    L = S.scene_lines()
    assert L[0] == "Viewpoint {"
    assert not any(ln.startswith(("WorldInfo", "EXTERNPROTO", "#OMNISIM")) for ln in L)
    assert S.brace_balance(L)["balanced"]


def test_scene_contents():
    text = "\n".join(S.scene_lines(arena=18.0, n_patches=5, controller="metazoa_world"))
    for needle in ("OmniSimSky { }", "DEF SUN OmniSimSun { }", "DEF SUN_MARKER OmniSimSunMarker { }",
                   "DEF FLOOR Solid", "DEF WALL_N", "DEF WALL_S", "DEF WALL_E", "DEF WALL_W",
                   "DEF CRYPT Solid", "DEF EDGE Solid", "DEF DIRECTOR Robot",
                   'controller "metazoa_world"', "supervisor TRUE", "synchronization TRUE"):
        assert needle in text, needle
    for k in range(5):
        assert "DEF PATCH_%d Solid" % k in text
    assert "DEF PATCH_5" not in text
    # floor top at z 0: centre -0.05, thickness 0.1
    assert "translation 0.000 0.000 -0.050" in text and "Box { size 18.000 18.000 0.100 }" in text
    # walls are 0.25 m
    assert "Box { size 18.000 0.200 0.250 }" in text
    # crypt slab x 57..93, y 57..63
    assert "translation 75.000 60.000 -0.050" in text and "Box { size 36.000 6.000 0.100 }" in text


def test_patches_and_edge_are_visual_only():
    patches = "\n".join(S.patch_lines(18.0, 5))
    edge = "\n".join(S.edge_lines(18.0))
    for text in (patches, edge):
        assert "boundingObject" not in text and "physics" not in text
    assert "Cylinder { radius 1.20 height 0.010 }" in patches
    assert "translation 5.400 0.000 0.005" in patches
    # emissive warm white, every channel <= 0.6
    assert all(c <= 0.6 for c in S.PATCH_EMISSIVE) and S.PATCH_EMISSIVE[0] >= S.PATCH_EMISSIVE[2]
    # patches start inside the arena, clear of the walls
    for x, y in S.patch_positions(18.0, 5):
        assert max(abs(x), abs(y)) + S.PATCH_RADIUS < 9.0 - S.WALL_T
    # EDGE frame 0.6 m inside the walls' inner face, lighter than the floor
    assert S.edge_half_side(18.0) == pytest.approx(9.0 - 0.2 - 0.6)
    assert sum(S.EDGE_COLOUR) > sum(S.GROUND_COLOUR)


def test_scene_preamble_and_header_form_a_whole_world():
    L = S.scene_lines(preamble=True, header_line=True)
    assert L[0] == "#OMNISIM R2025a utf8"
    text = "\n".join(L)
    assert text.count("WorldInfo {") == 1 and text.count("EXTERNPROTO") == 3
    assert 'newtonSolver "mujoco"' in text and "newtonRobotColliders TRUE" in text
    assert "newtonNjmax 2048" in text and "newtonGroundMu 1.0" in text
    assert S.brace_balance(L)["balanced"]
    assert S.externproto_lines() == S.EXTERNPROTOS and len(S.externproto_lines()) == 3


def test_brace_balance_detects_faults():
    assert S.brace_balance(["A {", "}"])["balanced"]
    r = S.brace_balance(["A {", "  children [", "}"])
    assert not r["balanced"] and r["brackets"]["net"] == 1
    r = S.brace_balance(["}", "{"])                       # net zero but goes negative
    assert not r["balanced"] and r["first_negative_line"] == 1
    assert S.brace_balance(['A { name "}" }', "# {{{"])["balanced"]   # strings + comments ignored


# -------------------------------------------------------------- the reef

def _lineages(n=6):
    import random
    return MZ.initial_lineages(n, random.Random(1), None)


def test_build_reef_conserves_cells_and_shapes_the_start():
    import random
    reef = MZ.build_reef(_lineages(6), 24, 18.0, 0, random.Random(3), mods={"scene": S})
    assert MZ.check_conserved(reef)
    assert len(reef["organisms"]) == 6 and all(len(o["members"]) == 4 for o in reef["organisms"])
    assert len(reef["free"]) == 0 and reef["parked"] == []
    assert len(reef["cells"]) == 24
    inner = S.edge_half_side(18.0)
    for c in reef["cells"]:
        x, y, z = c["pos"]
        assert not c["parked"] and max(abs(x), abs(y)) < inner
        assert z == pytest.approx(MZ.CELL_Z, abs=0.01)
        assert set(c) >= {"id", "pos", "yaw", "roll", "organism", "parked",
                          "dock_rotation", "charge_wh"}
    # the two cells of an organism sit one cell-length (+gap) apart, nose to tail
    o = reef["organisms"][0]
    a, b = (reef["cells"][i] for i in o["members"][:2])
    d = ((a["pos"][0] - b["pos"][0]) ** 2 + (a["pos"][1] - b["pos"][1]) ** 2) ** 0.5
    assert d == pytest.approx(MZ.CELL_LEN + MZ.DOCK_GAP, abs=1e-3)
    assert a["organism"] == b["organism"] == o["id"] and o["lineage"] == "L0"


def test_build_reef_parks_the_surplus_on_the_crypt():
    import random
    reef = MZ.build_reef(_lineages(6), 42, 18.0, 2, random.Random(3), mods={})
    assert MZ.check_conserved(reef)
    assert len(reef["parked"]) == 6 and len(reef["free"]) == 12
    for i in reef["parked"]:
        c = reef["cells"][i]
        assert c["parked"] and 57.0 < c["pos"][0] < 93.0 and 57.0 < c["pos"][1] < 63.0
    with pytest.raises(ValueError):
        MZ.build_reef(_lineages(6), 11, 18.0, 0, random.Random(3))


def test_chain_placement_fallback_follows_design_geometry():
    p = MZ.chain_placement_fallback((1.0, 2.0), 0.0, 4, [0, 1, 0, 1], gap=0.01)
    assert [c["pos"][0] for c in p] == pytest.approx([1.0, 1.13, 1.26, 1.39])
    assert all(c["pos"][1] == 2.0 for c in p)
    # rotations are relative to the HEAD (A's convention): [0,1,0,1] = pitch,yaw,pitch,yaw
    assert [c["roll"] for c in p] == pytest.approx([0.0, 1.5708, 0.0, 1.5708], abs=1e-3)
    assert [c["dock_rotation"] for c in p] == [0, 1, 0, 1]


def test_chain_placement_fallback_matches_A_when_present():
    try:
        CELL = importlib.import_module("mz.cell")
    except ImportError:
        pytest.skip("mz.cell (implementer A) not present yet")
    ours = MZ.chain_placement_fallback((1.0, 2.0), 0.7, 4, [0, 1, 0, 1], z=CELL.SPAWN_Z)
    theirs = MZ.chain_placement(CELL, (1.0, 2.0), 0.7, 4, [0, 1, 0, 1], note=lambda m: None)
    for a, b in zip(ours, theirs):
        assert a["pos"] == pytest.approx(b["pos"], abs=1e-3)
        assert a["yaw"] == pytest.approx(b["yaw"], abs=1e-3)
        assert a["roll"] == pytest.approx(b["roll"], abs=1e-3)


def test_chain_placement_uses_A_when_present_and_falls_back_on_signature_mismatch():
    class FakeCell:
        SPAWN_Z = 0.1

        @staticmethod
        def chain_poses(head_pose, n, gap=0.01, dock_rotations=None):
            return [{"pos": [head_pose[0] + k, head_pose[1], head_pose[2]], "yaw": head_pose[3],
                     "roll": 0.0} for k in range(n)]
    p = MZ.chain_placement(FakeCell, (0.0, 0.0), 0.0, 2, [0, 1], note=lambda m: None)
    assert p[1]["pos"] == [1.0, 0.0, 0.1] and p[1]["dock_rotation"] == 1

    class WrongSig:
        @staticmethod
        def chain_poses(n):
            return []
    notes = []
    p = MZ.chain_placement(WrongSig, (0.0, 0.0), 0.0, 2, [0], note=notes.append)
    assert len(p) == 2 and notes and "fallback" in notes[0]


# ------------------------------------------------------ scoring / selection

def test_score_lineages_per_design():
    res = {"lineages": {"L0": {"divisions": 2, "recruited": 3, "light_wh": 40, "mean_length": 2.5},
                        "L1": {}, "junk": 5}}
    s = MZ.score_lineages(res)
    assert s["L0"]["score"] == pytest.approx(2 * 10 + 3 + 4 + 2.5)
    assert s["L1"]["score"] == 0.0 and "junk" not in s
    assert MZ.score_lineages({"L2": {"divisions": 1}})["L2"]["score"] == 10.0   # bare dict


def test_select_and_refill_keeps_top_half_and_mutates_keepers():
    import random
    lineages = _lineages(6)
    scores = {ln["id"]: {"score": float(i)} for i, ln in enumerate(lineages)}
    scores["L5"]["genome"] = dict(lineages[5]["genome"], A=1.11)
    nxt, kept, next_n = MZ.select_and_refill(lineages, scores, random.Random(0), 4, 6, None)
    assert [k["id"] for k in kept] == ["L5", "L4", "L3"]
    assert len(nxt) == 6 and next_n == 9
    assert nxt[0]["genome"]["A"] == 1.11                       # evolved-in-epoch genome carried
    fresh = [ln for ln in nxt if ln["born_epoch"] == 5]
    assert [ln["id"] for ln in fresh] == ["L6", "L7", "L8"]
    assert all(ln["parent"] in ("L5", "L4", "L3") for ln in fresh)


# ------------------------------------------------------------------ thermal

def test_gpu_temperature_parses_and_tolerates_absence():
    assert MZ.gpu_temperature(runner=lambda cmd: "65\n") == 65.0
    assert MZ.gpu_temperature(runner=lambda cmd: "58, 61\n") == 58.0
    assert MZ.gpu_temperature(runner=lambda cmd: "NVIDIA-SMI has failed") is None

    def missing(cmd):
        raise FileNotFoundError("nvidia-smi")
    assert MZ.gpu_temperature(runner=missing) is None


def test_wait_for_cool_waits_while_above_70():
    temps = iter([80.0, 72.0, 70.0])
    slept, logged = [], []
    t = MZ.wait_for_cool(limit=70, poll_s=60, temp=lambda: next(temps),
                         sleep=slept.append, log=logged.append)
    assert t == 70.0 and slept == [60, 60]
    assert sum("waiting" in m for m in logged) == 2 and "ok" in logged[-1]
    assert MZ.wait_for_cool(temp=lambda: None, sleep=slept.append, log=logged.append) is None
    with pytest.raises(RuntimeError):
        MZ.wait_for_cool(temp=lambda: 90.0, sleep=lambda s: None, log=lambda m: None,
                         poll_s=60, max_wait_s=120)


# ---------------------------------------------------------------- dry run

def test_dry_run_writes_nothing_and_exits_zero(tmp_path):
    env = dict(os.environ, METAZOA_RUN_DIR=str(tmp_path / "run"))
    p = subprocess.run([sys.executable, os.path.join(ROOT, "metazoa.py"),
                        "--dry-run", "--epochs", "2", "--cells", "24", "--organisms", "6"],
                       cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, timeout=120)
    assert p.returncode == 0, p.stdout
    assert "DRY RUN" in p.stdout and "=== epoch 0 ===" in p.stdout and "=== epoch 1 ===" in p.stdout
    assert "run-headless" in p.stdout and "OMNISIM_LOG_PATH=" in p.stdout
    assert "temperature.gpu" in p.stdout
    assert not (tmp_path / "run").exists()


def test_dry_run_survives_absent_A_B_C(tmp_path):
    """Force every mz.* seam absent (even after A/B/C land) and plan anyway."""
    env = dict(os.environ, METAZOA_RUN_DIR=str(tmp_path / "run"))
    code = ("import sys, builtins\n"
            "real = builtins.__import__\n"
            "def fake(name, *a, **k):\n"
            "    if name in ('mz.cell', 'mz.worldgen', 'mz.organism', 'mz.ecology'):\n"
            "        raise ImportError('absent for the test')\n"
            "    return real(name, *a, **k)\n"
            "builtins.__import__ = fake\n"
            "import importlib\n"
            "orig = importlib.import_module\n"
            "def imp(name, *a, **k):\n"
            "    if name in ('mz.cell', 'mz.worldgen', 'mz.organism', 'mz.ecology'):\n"
            "        raise ImportError('absent for the test')\n"
            "    return orig(name, *a, **k)\n"
            "importlib.import_module = imp\n"
            "sys.path.insert(0, %r)\n"
            "import metazoa\n"
            "sys.exit(metazoa.main(['--dry-run', '--epochs', '1']))\n" % ROOT)
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    assert p.returncode == 0, p.stdout
    for name in ("cell", "worldgen", "organism", "ecology"):
        assert "mz.%s=STUB" % name in p.stdout
    assert "mz.scene=present" in p.stdout
    assert not (tmp_path / "run").exists()


def test_real_run_refuses_without_worldgen(tmp_path):
    env = dict(os.environ, METAZOA_RUN_DIR=str(tmp_path / "run"))
    code = ("import sys, importlib\n"
            "orig = importlib.import_module\n"
            "def imp(name, *a, **k):\n"
            "    if name == 'mz.worldgen':\n"
            "        raise ImportError('absent for the test')\n"
            "    return orig(name, *a, **k)\n"
            "importlib.import_module = imp\n"
            "sys.path.insert(0, %r)\n"
            "import metazoa\n"
            "sys.exit(metazoa.main(['--epochs', '1']))\n" % ROOT)
    p = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    assert p.returncode != 0 and "required for a real run" in p.stdout
    assert not (tmp_path / "run").exists()


def test_watch_dry_run(tmp_path):
    env = dict(os.environ, METAZOA_RUN_DIR=str(tmp_path / "run"))
    p = subprocess.run([sys.executable, os.path.join(ROOT, "watch_metazoa.py"), "--dry-run"],
                       cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, timeout=120)
    assert p.returncode == 0, p.stdout
    for k, v in (("OMNISIM_WGPU_SSR", "0"), ("OMNISIM_WGPU_TAA", "0"),
                 ("OMNISIM_WGPU_VOLUMETRIC", "0"), ("OMNISIM_WGPU_PCSS", "0")):
        assert "%s=%s" % (k, v) in p.stdout
    assert "run-world" in p.stdout and "--mode=fast" not in p.stdout
    assert not (tmp_path / "run").exists()


def test_watch_lean_env_matches_alife_verbatim():
    import watch_metazoa as WM
    alife = os.path.normpath(os.path.join(ROOT, "..", "alife"))
    sys.path.insert(0, alife)
    try:
        import watch as alife_watch
    finally:
        sys.path.remove(alife)
    assert WM.LEAN == alife_watch.LEAN


# ------------------------------------------------- A's worldgen, if present

def test_worldgen_assembles_a_balanced_world_if_present(tmp_path):
    try:
        W = importlib.import_module("mz.worldgen")
    except ImportError:
        pytest.skip("mz.worldgen (implementer A) not present yet")
    import random
    reef = MZ.build_reef(_lineages(6), 24, 18.0, 0, random.Random(3),
                         mods={"scene": S, "cell": importlib.import_module("mz.cell")
                               if importlib.util.find_spec("mz.cell") else None})
    path = str(tmp_path / "metazoa_test.omniworld")
    W.write_world(reef["cells"], path, scene_lines=S.scene_lines(18.0, 5, "metazoa_world"),
                  controller="metazoa_world")
    text = open(path, encoding="utf-8").read()
    assert text.startswith("#OMNISIM R2025a utf8")
    assert S.brace_balance(text)["balanced"], S.brace_balance(text)
    assert text.count("WorldInfo {") == 1 and text.count("EXTERNPROTO") == 3
    assert "DEF DIRECTOR Robot" in text and "DEF PATCH_4 Solid" in text and "DEF EDGE Solid" in text
    assert "DEF CELL_0 Robot" in text and "DEF CELL_23 Robot" in text
