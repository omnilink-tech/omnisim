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

"""Engine-free tests for OmniSim Manufacturing's dependency inventory (projects/manufacturing/
deps.py, requirements.txt) and for what the router does when a dependency is missing:
`manufacturing setup`, `manufacturing doctor`, and the stage gate in product_cli.py.

Nothing here installs a package, launches an engine or needs the packages themselves:
"missing" is simulated by replacing deps.present, and pip by a recorder."""
import ast
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SECTOR = ROOT / "projects/manufacturing"
if str(SECTOR) not in sys.path:
    sys.path.insert(0, str(SECTOR))

import deps  # noqa: E402
import product_cli as pc  # noqa: E402

CONTROLLERS = [ROOT / f"projects/samples/demos/controllers/{c}/{c}.py"
               for c in ("fdm_twin", "grbl_twin", "spark_rover_drive")]
LOCAL_DIRS = [SECTOR, ROOT / "projects/policies/common", ROOT / "projects/samples/demos/controllers",
              ROOT / "packages", ROOT / "omnisim"]


def _local_names():
    names = set()
    for base in LOCAL_DIRS:
        for f in base.rglob("*.py"):
            names.add(f.stem)
            names.add(f.parent.name)
    return names


def _third_party_imports(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module.split(".")[0]


# ---- the inventory --------------------------------------------------------------------------------

def test_requirements_and_packages_name_the_same_set_and_every_one_is_pinned():
    reqs = deps.requirements()
    assert set(reqs) == {p.dist for p in deps.PACKAGES.values()}
    assert all(v and v[0].isdigit() for v in reqs.values()), reqs
    for line in deps.REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        if line.strip() and not line.startswith("#"):
            assert "#" in line, f"no comment saying which stage needs it: {line}"


def test_every_third_party_import_in_the_sector_is_inventoried():
    """Any import under projects/manufacturing (and the twins' controllers) that is not the
    standard library, not the repository's own, is a PACKAGES entry, so setup installs it."""
    local = _local_names()
    unknown = {}
    for path in [*SECTOR.rglob("*.py"), *CONTROLLERS]:
        if "__pycache__" in path.parts:
            continue
        for name in _third_party_imports(path):
            if (name in sys.stdlib_module_names or name == "__future__" or name in local
                    or name in deps.PACKAGES or name in deps.NOT_FROM_PIP):
                continue
            unknown.setdefault(name, []).append(path.relative_to(ROOT).as_posix())
    assert not unknown, f"imports not in deps.PACKAGES / requirements.txt: {unknown}"


def test_every_stage_of_both_products_has_its_needs_and_they_are_packages():
    for name, table in deps.PRODUCT_NEEDS.items():
        product = SECTOR / "products" / name
        for stage in pc.STAGES:
            if pc.commands(product, stage.name) is None:
                continue
            assert stage.name in table, (name, stage.name)
            assert set(table[stage.name]) <= set(deps.PACKAGES), (name, stage.name)
            assert "numpy" in table[stage.name]


def test_the_tables_cover_what_each_stage_script_itself_imports():
    """A lower bound: the declared needs hold at least what the stage's own scripts and their
    sibling modules import (dynamic loads, like integrate.load(), come on top)."""
    for name, table in deps.PRODUCT_NEEDS.items():
        product = SECTOR / "products" / name
        for stage in pc.STAGES:
            cmds = pc.commands(product, stage.name)
            if cmds is None:
                continue
            scanned = set(deps.scan_needs(product, [c[0] for c in cmds]))
            assert scanned <= set(table[stage.name]), (name, stage.name, scanned - set(table[stage.name]))


def test_the_twins_controllers_need_only_what_the_engine_bundle_has():
    """fdm_twin, grbl_twin and spark_rover_drive run on the engine's bundled interpreter
    (numpy, Pillow). Their own imports, and their twin modules' module-level ones, need nothing else."""
    for path in CONTROLLERS:
        mods = set(_third_party_imports(path)) & set(deps.PACKAGES)
        assert mods <= {"numpy"}, (path.name, mods)


def _cli():
    import importlib.util
    spec = importlib.util.spec_from_file_location("manufacturing_cli_for_setup", SECTOR / "cli.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_tool_table_matches_the_cli_processes():
    cli = _cli()
    tools = {(proc, tool) for proc, spec in cli.PROCESSES.items() for tool in spec["tools"]}
    assert tools | {("cad-pack", "")} == set(deps.TOOL_NEEDS)
    assert all(set(m) <= set(deps.PACKAGES) for m in deps.TOOL_NEEDS.values())


def test_an_unknown_product_is_read_from_its_scripts(tmp_path):
    prod = tmp_path / "products/my_bot"
    prod.mkdir(parents=True)
    (prod / "skeleton.py").write_text("import json\n", encoding="utf-8")
    (prod / "integrate.py").write_text("import numpy\nimport parts\n", encoding="utf-8")
    (prod / "parts.py").write_text("import trimesh\nm = trimesh.collision.CollisionManager()\n", encoding="utf-8")
    assert set(deps.needs(prod, "check")) == {"numpy", "trimesh", "fcl", "scipy", "rtree", "networkx"}
    assert deps.needs(prod, "wire") == ()          # no harness.py: no stage


# ---- the gate: a missing package is named, with the fix, and nothing runs --------------------------

def _fake(tmp_path: Path) -> Path:
    prod = tmp_path / "products/toy_bot"
    prod.mkdir(parents=True)
    rec = ('import json, sys\nfrom pathlib import Path\n'
           'Path(__file__).with_name("ran.txt").write_text(Path(__file__).name)\n')
    (prod / "skeleton.py").write_text('"""Toy."""\n' + rec, encoding="utf-8")
    (prod / "integrate.py").write_text('"""Toy."""\nimport shapely\nimport OCP\n' + rec, encoding="utf-8")
    (prod / "make.py").write_text("import argparse\nimport numpy\nargparse.ArgumentParser()\n" + rec, encoding="utf-8")
    return prod


def _without(monkeypatch, *modules):
    real = deps.present
    monkeypatch.setattr(deps, "present", lambda m, do_import=False: m not in modules and real(m, do_import))


def test_a_missing_package_stops_the_stage_with_its_name_and_the_setup_command(tmp_path, monkeypatch, capsys):
    prod = _fake(tmp_path)
    _without(monkeypatch, "OCP", "shapely")
    assert pc.main(["product", "toy_bot", "check"], products_dir=prod.parent) == 3
    err = capsys.readouterr().err
    assert "toy_bot check needs Python packages" in err
    assert f"pip: {deps.pin('OCP')}" in err and f"pip: {deps.pin('shapely')}" in err
    assert "cadquery-ocp==" in err and deps.SETUP_COMMAND in err
    assert "Traceback" not in err
    assert not (prod / "ran.txt").exists()        # neither script started


def test_dry_run_still_plans_and_says_what_is_missing(tmp_path, monkeypatch, capsys):
    prod = _fake(tmp_path)
    _without(monkeypatch, "numpy")
    assert pc.main(["product", "toy_bot", "make", "--dry-run"], products_dir=prod.parent) == 0
    out = capsys.readouterr()
    assert "make.py" in out.out and "cannot run here yet" in out.err and "numpy" in out.err
    assert pc.main(["product", "toy_bot", "make", "--help"], products_dir=prod.parent) == 3   # its --help imports numpy
    assert not (prod / "ran.txt").exists()


def test_an_optional_package_does_not_block(tmp_path, monkeypatch):
    _without(monkeypatch, "numba")
    r = deps.readiness("spark_rover", "wire", engine={"ok": True, "path": None})
    assert r["can_run"] and r["slower"] == ["numba"] and r["missing"] == []


def test_with_everything_present_the_stage_runs(tmp_path, monkeypatch):
    prod = _fake(tmp_path)
    monkeypatch.setattr(deps, "present", lambda m, do_import=False: True)
    assert pc.main(["product", "toy_bot", "check"], products_dir=prod.parent) == 0
    assert (prod / "ran.txt").read_text() == "integrate.py"


def test_cura_is_required_only_until_the_plan_exists(tmp_path, monkeypatch):
    monkeypatch.setattr(deps, "ROOT", tmp_path)
    no_cura = {"ok": False, "path": None, "version": None}
    every = {m: True for m in deps.PACKAGES}
    r = deps.readiness("spark_rover", "print-ideal", have=every, cura=no_cura, engine={"ok": True})
    assert r["cura"] == "missing" and not r["can_run"]
    plan = tmp_path / deps.CURA_NEEDS["spark_rover"]["print-ideal"]
    plan.parent.mkdir(parents=True)
    plan.write_text("{}")
    r = deps.readiness("spark_rover", "print-ideal", have=every, cura=no_cura, engine={"ok": True})
    assert r["cura"].startswith("not needed") and r["can_run"]
    # the desk robot's make slices only for --only print: reported, not required
    r = deps.readiness("desk_robot", "make", have=every, cura=no_cura, engine={"ok": True})
    assert r["cura"].startswith("missing (") and r["can_run"]


def test_the_cura_message_gives_the_official_source(monkeypatch, tmp_path):
    monkeypatch.setattr(deps, "ROOT", tmp_path)                       # no plan
    monkeypatch.setattr(deps, "present", lambda m, do_import=False: True)
    monkeypatch.setattr(deps, "cura_status", lambda: {"ok": False, "path": None, "version": None})
    msg = deps.gate(SECTOR / "products/spark_rover", "print")
    assert "CuraEngine" in msg and "ultimaker.com" in msg and "CURA_HOME" in msg


def test_a_process_tool_names_its_missing_package(monkeypatch, capsys):
    cli = _cli()
    _without(monkeypatch, "scipy")
    assert cli.main(["cnc", "inspect", "--help"]) == 3
    err = capsys.readouterr().err
    assert "manufacturing cnc inspect needs" in err and "scipy==" in err and deps.SETUP_COMMAND in err


# ---- doctor and setup -----------------------------------------------------------------------------

def test_doctor_reports_every_stage_and_exits_nonzero_when_one_cannot_run(monkeypatch, capsys):
    monkeypatch.setattr(deps, "cura_status", lambda: {"ok": True, "path": "C:/Cura", "version": deps.CURA_TESTED})
    monkeypatch.setattr(deps, "engine_status", lambda: {"ok": True, "path": "omnisim-bin"})
    _without(monkeypatch, "OCP")
    assert deps.doctor(["--json"]) == 1
    rep = json.loads(capsys.readouterr().out)
    stages = {(p["product"], r["stage"]): r for p in rep["products"] for r in p["stages"]}
    assert stages[("spark_rover", "check")]["missing"] == ["OCP"]
    assert stages[("desk_robot", "check")]["can_run"]           # the desk robot has no B-rep solids
    assert set(stages) >= {("spark_rover", s.name) for s in pc.STAGES}
    monkeypatch.setattr(deps, "present", lambda m, do_import=False: True)
    assert deps.doctor([]) == 0
    assert "spark_rover" in capsys.readouterr().out


def test_setup_shows_the_plan_and_installs_nothing_without_yes(monkeypatch, capsys):
    plan = [{"name": "shapely", "version": "2.1.2", "requested": True, "installed": None},
            {"name": "vtk", "version": "9.6.2", "requested": False, "installed": None}]
    monkeypatch.setattr(deps, "pip_plan", lambda: plan)
    calls = []
    monkeypatch.setattr(deps.subprocess, "run", lambda *a, **k: calls.append(a) or None)
    monkeypatch.setattr(sys, "stdin", None)
    assert deps.setup([]) == 2
    out = capsys.readouterr().out
    assert "* shapely==2.1.2" in out and "  vtk==9.6.2" in out and "--yes" in out
    assert calls == []


def test_setup_yes_runs_pip_on_the_requirements_then_reports(monkeypatch, capsys):
    monkeypatch.setattr(deps, "pip_plan", lambda: [{"name": "psutil", "version": "7.0.0", "requested": True,
                                                    "installed": "6.0.0"}])
    monkeypatch.setattr(deps, "cura_status", lambda: {"ok": False, "path": None, "version": None})
    monkeypatch.setattr(deps, "engine_status", lambda: {"ok": False, "path": None})

    class Done:
        returncode = 0
    calls = []
    monkeypatch.setattr(deps.subprocess, "run", lambda cmd, *a, **k: calls.append(cmd) or Done())
    assert deps.setup(["--yes"]) == 0
    assert calls == [[sys.executable, "-m", "pip", "install", "--disable-pip-version-check", "-r",
                      str(deps.REQUIREMENTS)]]
    out = capsys.readouterr().out
    assert "(replaces 6.0.0)" in out
    assert "CuraEngine: not found" in out and "ultimaker.com" in out      # reported, never downloaded
    assert "spark_rover" in out and "Engine  NOT FOUND" in out           # ends with the table


def test_setup_dry_run_installs_nothing(monkeypatch, capsys):
    monkeypatch.setattr(deps, "pip_plan", lambda: [{"name": "numba", "version": "0.67.0", "requested": True,
                                                    "installed": None}])
    calls = []
    monkeypatch.setattr(deps.subprocess, "run", lambda *a, **k: calls.append(a))
    assert deps.setup(["--dry-run"]) == 0
    assert calls == [] and "nothing installed" in capsys.readouterr().out


@pytest.mark.parametrize("stage", [s.name for s in pc.STAGES])
def test_the_real_products_gate_is_quiet_on_a_full_install(stage, monkeypatch):
    monkeypatch.setattr(deps, "present", lambda m, do_import=False: True)
    monkeypatch.setattr(deps, "cura_status", lambda: {"ok": True, "path": "C:/Cura", "version": deps.CURA_TESTED})
    assert deps.gate(SECTOR / "products/spark_rover", stage) is None
