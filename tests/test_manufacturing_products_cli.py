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

"""Engine-free tests for `python -m omnisim manufacturing products` / `product <name> <stage>`
(projects/manufacturing/product_cli.py): discovery, the stage map, --help, and that a stage
runs the product's own script with the arguments given. Nothing here launches an engine:
the real products are only planned (--dry-run, --help), and the runs use a fake product."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SECTOR = ROOT / "projects/manufacturing"


def _mod():
    spec = importlib.util.spec_from_file_location("manufacturing_product_cli", SECTOR / "product_cli.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # dataclasses look their module up while it loads
    spec.loader.exec_module(mod)
    return mod


pc = _mod()
SPARK = SECTOR / "products/spark_rover"
DESK = SECTOR / "products/desk_robot"


def _cli(*args, timeout=120):
    return subprocess.run([sys.executable, "-m", "omnisim", "manufacturing", *args], cwd=ROOT,
                          capture_output=True, text=True, timeout=timeout)


# ---- discovery and the stage map --------------------------------------------------------------

def test_products_are_found_by_their_scripts():
    names = [p.name for p in pc.products()]
    assert names == ["desk_robot", "spark_rover"]
    assert "clothes_rack" in pc.others()           # a product folder with none of the stage scripts


def test_spark_rover_has_every_stage_by_convention():
    for stage in pc.BY_NAME:
        cmds = pc.commands(SPARK, stage)
        assert cmds == pc.BY_NAME[stage].commands, stage
    assert [c[0] for c in pc.commands(SPARK, "check")] == ["skeleton.py", "integrate.py"]


def test_desk_robot_runs_its_readme_commands_and_lacks_the_print_stages():
    assert pc.commands(DESK, "print") is None and pc.commands(DESK, "print-ideal") is None
    assert pc.commands(DESK, "make")[0] == ("make.py", ("--out", ".tmp/desk_robot"), ())
    assert pc.commands(DESK, "release")[0][0] == "release.py"
    assert [c[0] for c in pc.commands(DESK, "check")] == ["design.py", "bought_parts.py"]
    for stage, script in (("wire", "wiring.py"), ("assemble", "assemble.py"), ("drive", "robot.py")):
        assert pc.commands(DESK, stage)[0][0] == script
    for script, _, _ in (c for s in pc.BY_NAME for c in (pc.commands(DESK, s) or ())):
        assert (DESK / script).is_file(), script


def test_plan_places_the_arguments_and_the_defaults():
    run = pc.plan(SPARK, "print", [])
    assert run[0][1:] == [str(SPARK / "make_print.py"), "--run"]
    assert pc.plan(SPARK, "print", ["--plan"])[0][2:] == ["--plan"]          # an argument replaces --run
    assert pc.plan(SPARK, "print-ideal", ["--only", "rev03"])[0][2:] == ["--ideal", "--only", "rev03"]
    assert pc.plan(DESK, "make", ["--only", "laser"])[0][2:] == ["--out", ".tmp/desk_robot", "--only", "laser"]
    drive = pc.plan(SPARK, "drive", ["--mu", "0.8"], duration=90)
    assert drive[0][2:] == ["--quit", "--mu", "0.8"]                         # robot.py takes --quit
    assert drive[1][2:4] == ["omnisim", "run-headless"] and drive[1][-2:] == ["--duration", "90"]
    assert "--quit" not in pc.plan(DESK, "drive", [])[0]                     # desk_robot/robot.py does not
    with pytest.raises(ValueError):
        pc.plan(SPARK, "check", ["--x"])                                      # two scripts: no arguments
    with pytest.raises(LookupError):
        pc.plan(DESK, "print", [])


# ---- the command line -------------------------------------------------------------------------

def test_products_lists_both_with_their_results():
    out = _cli("products")
    assert out.returncode == 0, out.stderr
    assert "desk_robot:" in out.stdout and "spark_rover:" in out.stdout
    assert "Simulation only" in out.stdout
    rows = {r["product"]: r for r in json.loads(_cli("products", "--json").stdout)}
    assert set(rows) == {"desk_robot", "spark_rover"}
    spark = rows["spark_rover"]
    assert spark["lacks"] == [] and spark["printed"]["file"] == "release/make/print/printed.json"
    p = spark["printed"]
    assert p["parts"] == p["twin"] + p["ideal_only"] + p["pending"]          # every part in one mode, or pending
    assert rows["desk_robot"]["lacks"] == ["print", "print-ideal"]


def test_help_and_unknowns():
    assert "print-ideal" in _cli("product", "--help").stdout
    out = _cli("product", "spark_rover")
    assert out.returncode == 0 and "robot.py --quit" in out.stdout and "make_print.py --run" in out.stdout
    assert _cli("product", "nope", "check").returncode == 2
    bad = _cli("product", "spark_rover", "nope")
    assert bad.returncode == 2 and "print-ideal" in bad.stderr
    lacks = _cli("product", "desk_robot", "print")
    assert lacks.returncode == 2 and "make --only print" in lacks.stderr


def test_a_script_without_arguments_answers_help_from_its_docstring():
    """harness.py reads no arguments, so --help must not run it (it would route every wire)."""
    out = _cli("product", "spark_rover", "wire", "--help")
    assert out.returncode == 0 and "route every wire" in out.stdout and "takes no arguments" in out.stdout
    bad = _cli("product", "spark_rover", "wire", "--fast")
    assert bad.returncode == 2 and "takes no arguments" in bad.stderr


def test_an_argparse_stage_answers_its_own_help():
    out = _cli("product", "spark_rover", "make", "--help")
    assert out.returncode == 0 and "--plan" in out.stdout and "--process" in out.stdout


def test_print_ideal_needs_its_plates():
    out = _cli("product", "spark_rover", "print-ideal")
    assert out.returncode == 2 and "--only" in out.stderr and "plate01" in out.stderr


def test_all_dry_run_plans_every_stage_in_order():
    out = _cli("product", "spark_rover", "all", "--dry-run")
    assert out.returncode == 0, out.stderr
    shown = [line for line in out.stdout.splitlines() if line.startswith("$ ")]
    scripts = [Path(line.split()[2]).name if line.split()[1] == "python" and line.split()[2] != "-m" else "run-headless"
               for line in shown]
    assert scripts == ["skeleton.py", "integrate.py", "harness.py", "make.py", "make_print.py", "assemble.py",
                       "robot.py", "run-headless", "release.py"]
    desk = _cli("product", "desk_robot", "all", "--dry-run")
    assert desk.returncode == 0 and "skipped" in desk.stdout and "make.py --out .tmp/desk_robot" in desk.stdout
    assert _cli("product", "spark_rover", "all", "--mu", "1").returncode == 2


# ---- a stage runs the right script (a fake product, no engine) ----------------------------------

RECORD = """import json, sys
from pathlib import Path
with open(Path(__file__).with_name("calls.jsonl"), "a", encoding="utf-8") as f:
    f.write(json.dumps([Path(__file__).name, sys.argv[1:]]) + "\\n")
{tail}
"""


def _fake(tmp_path: Path, fail: str = "") -> Path:
    prod = tmp_path / "products/toy_bot"
    prod.mkdir(parents=True)
    for script in ("skeleton.py", "integrate.py", "harness.py"):
        (prod / script).write_text('"""Toy: no arguments."""\n' + RECORD.format(tail=""), encoding="utf-8")
    for script in ("make.py", "make_print.py", "assemble.py", "release.py"):
        tail = "import argparse\nargparse.ArgumentParser()\nraise SystemExit(3 if %r == Path(__file__).name else 0)" % fail
        (prod / script).write_text(RECORD.format(tail=tail), encoding="utf-8")
    world = prod / "toy.omniworld"
    tail = f"import argparse\nargparse.ArgumentParser()\nprint('wrote {world.as_posix()}')"
    (prod / "robot.py").write_text(RECORD.format(tail=tail), encoding="utf-8")
    return prod


def _calls(prod: Path):
    path = prod / "calls.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()] if path.exists() else []


def test_a_stage_runs_the_products_own_script(tmp_path):
    prod = _fake(tmp_path)
    pdir = prod.parent
    assert [p.name for p in pc.products(pdir)] == ["toy_bot"]
    assert pc.main(["product", "toy_bot", "make", "--only", "deck"], products_dir=pdir) == 0
    assert pc.main(["product", "toy_bot", "print"], products_dir=pdir) == 0
    assert pc.main(["product", "toy_bot", "print-ideal", "--only", "p1"], products_dir=pdir) == 0
    assert pc.main(["product", "toy_bot", "check"], products_dir=pdir) == 0
    assert _calls(prod) == [["make.py", ["--only", "deck"]], ["make_print.py", ["--run"]],
                            ["make_print.py", ["--ideal", "--only", "p1"]], ["skeleton.py", []], ["integrate.py", []]]


def test_dry_run_runs_nothing(tmp_path):
    prod = _fake(tmp_path)
    assert pc.main(["product", "toy_bot", "all", "--dry-run"], products_dir=prod.parent) == 0
    assert _calls(prod) == []


def test_all_stops_at_the_first_failure_unless_told_to_keep_going(tmp_path, monkeypatch):
    prod = _fake(tmp_path, fail="make.py")
    ran = []
    monkeypatch.setattr(pc, "_run_drive", lambda cmds: ran.append(cmds) or 0)   # no engine
    assert pc.main(["product", "toy_bot", "all"], products_dir=prod.parent) == 1
    assert [c[0] for c in _calls(prod)] == ["skeleton.py", "integrate.py", "harness.py", "make.py"]
    (prod / "calls.jsonl").unlink()
    assert pc.main(["product", "toy_bot", "all", "--keep-going"], products_dir=prod.parent) == 1
    assert [c[0] for c in _calls(prod)] == ["skeleton.py", "integrate.py", "harness.py", "make.py",
                                            "make_print.py", "assemble.py", "release.py"]
    assert len(ran) == 1 and ran[0][0][2:] == [] and ran[0][1][-2:] == ["--duration", "60"]


def test_drive_runs_the_world_robot_py_wrote(tmp_path, monkeypatch):
    prod = _fake(tmp_path)
    seen = []
    monkeypatch.setattr(pc, "_run", lambda cmd: seen.append(cmd) or 0)          # stands in for run-headless
    assert pc.main(["product", "toy_bot", "drive", "--duration", "5", "--mu", "1"], products_dir=prod.parent) == 0
    assert _calls(prod) == [["robot.py", ["--mu", "1"]]]                       # --duration is the router's
    assert seen == [[sys.executable, "-m", "omnisim", "run-headless", (prod / "toy.omniworld").as_posix(),
                     "--duration", "5"]]
