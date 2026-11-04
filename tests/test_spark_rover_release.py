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

"""SparkRover Mini's release.py without an engine: the release folder writes, its BOM adds up
to integrate.mass(), every made part has a drawing and a model, and a printed part whose plate
has not printed reads PENDING (never a predicted number), from synthetic records and from the
real printed.json as it stands.

The full write takes a minute or two (the design and the printed parts' solids are cached in
.tmp/ by robot._key(); the FCL interference run is skipped here)."""
import copy
import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))


def _release():
    key = "_spark_rover_test_release"
    if key not in sys.modules:
        spec = importlib.util.spec_from_file_location(key, PRODUCT / "release.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
    return sys.modules[key]


R = _release()


# ---- the printed parts' state, on records whose answer is known ----------------------------------

def _record():
    """A printed.json in make_print.collect()'s shape: one plate printed and measured, one planned,
    one plate holding a superseded drawing that DID print, and a part on no plate."""
    meas = {"outer_surface_error_mm": {"p05": -0.03, "p95": 0.04}, "size_error_mm": {"x_mean": 0.01, "y_mean": 0.02},
            "height_mm": {"printed": 2.05, "drawing": 2.0}, "part_g": 1.7, "hanging_strands": {"beads": 1, "length_mm": 0.5}}
    return {
        "parts": [{"label": "a", "design_tris_sha": "A1", "superseded": None},
                  {"label": "b", "design_tris_sha": "B1", "superseded": None},
                  {"label": "c", "design_tris_sha": "C0", "superseded": "the old drawing"},
                  {"label": "c", "design_tris_sha": "C1", "superseded": None}],
        "plates": [{"name": "p1", "parts": ["a"], "part_sha": {"a": "A1"}, "printed": True, "run": {"error": None},
                    "measured": {"parts": {"a": meas}}},
                   {"name": "p2", "parts": ["b"], "part_sha": {"b": "B1"}, "printed": False,
                    "run": {"error": "the engine exited before the print finished"}, "measured": None},
                   {"name": "p3", "parts": ["b"], "part_sha": {"b": "B1"}, "printed": False, "run": None, "measured": None},
                   {"name": "old", "parts": ["c"], "part_sha": {"c": "C0"}, "printed": True, "run": {},
                    "measured": {"parts": {"c": meas}}}],
    }


def test_a_part_on_a_printed_plate_is_printed_with_its_measurement():
    st = R.print_status(_record(), {"a": "A1"})
    assert st["a"]["state"] == "printed" and st["a"]["plate"] == "p1"
    assert st["a"]["measured"]["part_g"] == 1.7


def test_a_part_not_yet_printed_is_pending_with_its_plates_and_why():
    st = R.print_status(_record(), {"b": "B1"})["b"]
    assert st["state"] == "pending" and st["plates"] == ["p2", "p3"]
    assert "engine exited" in st["why"] and "measured" not in st


def test_a_superseded_drawing_that_printed_does_not_count():
    st = R.print_status(_record(), {"c": "C1"})["c"]
    assert st["state"] == "pending" and st["plates"] == []


def test_a_drawing_changed_since_the_plan_is_pending():
    st = R.print_status(_record(), {"a": "A2"})["a"]
    assert st["state"] == "pending" and "since" in st["why"]


def test_no_printed_json_means_everything_pending():
    st = R.print_status(None, {"a": "A1", "b": "B1"})
    assert {s["state"] for s in st.values()} == {"pending"}


def test_a_measurement_without_its_plate_printed_is_not_used():
    rec = _record()
    rec["plates"][0]["printed"] = False
    assert R.print_status(rec, {"a": "A1"})["a"]["state"] == "pending"


# ---- the release, written ------------------------------------------------------------------------

@pytest.fixture(scope="module")
def written(tmp_path_factory):
    out = tmp_path_factory.mktemp("spark_rover_release")
    ctx = R.Ctx(integration="mass", height_maps=False, log=lambda *_: None)
    res = R.write(out, readme=False, cam=False, ctx=ctx)
    return out, res, ctx


def test_made_as_names_every_laser_and_cnc_part_once(written):
    _, _, ctx = written
    made = set(ctx.made_labels(("laser", "cnc")))
    assert made == set(R.MADE_AS), "release.MADE_AS must name exactly the design's laser and CNC parts"
    jobs = {(j, p) for j, p in R.MADE_AS.values()}
    assert len(jobs) == len(R.MADE_AS)
    for label in made:                                   # each maps to a job make.py lays out
        job, part = R.MADE_AS[label]
        assert part in ctx.jobs[job].drawn, (label, job, part)
    if ctx.made["jobs"]:                                 # and to a measured part once make.py has run
        for label in made:
            assert ctx.made_rec(label)[1] is not None, label


def test_every_made_part_has_a_drawing_and_a_model(written):
    out, res, ctx = written
    names = {R.drawing_of(lab) for lab in ctx.made_labels(("laser", "cnc"))} | set(ctx.print_parts)
    assert set(ctx.made_labels(("print",))) == set(ctx.print_parts)
    for n in names:
        svg = out / "drawings" / f"{n}.svg"
        assert svg.exists() and "<svg" in svg.read_text(encoding="utf-8")[:2000], n
        stl = out / "models" / f"{n}.stl"
        assert stl.exists() and stl.stat().st_size > 84, n


def test_the_bom_adds_up_to_integrate_mass(written):
    out, res, ctx = written
    im = R.integrate.mass(ctx.mods)
    assert res["bom_total_g"] == pytest.approx(im["total_g"], abs=0.06)
    made, bought = R.bom_rows(ctx)
    assert sum(r["qty"] for r in made) + sum(r["qty"] for r in bought) == len(ctx.parts)
    text = (out / "BOM.md").read_text(encoding="utf-8")
    assert f"**{im['total_g']:.1f} g**" in text
    for r in made:
        assert f"| {r['part']} |" in text


def test_pending_print_parts_read_pending_everywhere(written):
    out, res, ctx = written
    report = (out / "REPORT.md").read_text(encoding="utf-8")
    pending = [k for k, s in ctx.status.items() if s["state"] != "printed"]
    assert res["pending"] == pending
    rows = {m.group(1): m.group(0) for m in re.finditer(r"^\| (\w+) \| (?:PETG|TPU 95A) \|.*$", report, re.M)}
    assert set(rows) == set(ctx.status)
    for k in pending:
        assert "**pending**" in rows[k] and not re.search(r"\| [+-]\d\.\d{3} / ", rows[k]), k
    for k, s in ctx.status.items():
        if s["state"] == "printed":
            q = s["measured"]
            assert "| printed |" in rows[k] and f"{q['part_g']} (" in rows[k], k
    if pending:
        assert "Printing is not finished" in report


def test_the_printed_state_agrees_with_printed_json():
    """Independently of release.py's reading: a part is printed exactly when a plate holding its
    current drawing printed and measured it."""
    rec = R._read_json(R.PRINT_RELEASE / "printed.json")
    if rec is None:
        pytest.skip("make_print.py has not written printed.json")
    current = {r["label"]: r["design_tris_sha"] for r in rec["parts"] if not r.get("superseded")}
    st = R.print_status(rec, current)
    for label, sha in current.items():
        done = any(pl["printed"] and label in ((pl.get("measured") or {}).get("parts") or {})
                   and pl["part_sha"].get(label) == sha for pl in rec["plates"])
        assert (st[label]["state"] == "printed") == done, label


def test_assembly_follows_the_logged_build_and_shows_its_pictures(written):
    out, _, ctx = written
    text = (out / "ASSEMBLY.md").read_text(encoding="utf-8")
    if ctx.assembly is None:
        pytest.skip("assemble.py has not been run")
    titles = re.findall(r"^## \d+\. (.*?)(?: \(bench\))?$", text, re.M)
    assert titles == [s["title"] for s in ctx.assembly["steps"]]
    for s in ctx.assembly["steps"]:
        assert f"](assembly/{s['screenshot']})" in text
    assert "Fasteners in this step" in text


def test_report_states_its_conditions(written):
    out, _, ctx = written
    report = (out / "REPORT.md").read_text(encoding="utf-8")
    assert "No part has been made on a real machine" in report
    for stem in ctx.drives:
        mu, arm = R._drive_conditions(stem)
        assert f"mu {mu}, armature {arm}" in report
    assert "The XC330 horn's dimensions are unverified" in report and "1984 cells" in report
    assert re.search(r"margin \*\*\d+\.\d g\*\*", report)


def test_the_readme_block_is_generated_between_its_markers(written):
    _, _, ctx = written
    block = R.readme_block(ctx)
    assert block.startswith(R.README_BEGIN) and block.endswith(R.README_END)
    readme = (PRODUCT / "README.md").read_text(encoding="utf-8")
    assert R.README_BEGIN in readme and R.README_END in readme
