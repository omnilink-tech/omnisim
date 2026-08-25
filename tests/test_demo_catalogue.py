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

"""The demo launcher catalogue must not reference files that don't exist.

`demos.json` drives the launcher gallery -- the no-args entry point of the
product. A card whose `world` has been moved or renamed is a dead button: the
user clicks Launch and nothing loads.

This is not hypothetical. Commit 96934580 renamed the g1 grasp lane book -> box
and missed this file, leaving a `g1_book_grasp` card pointing at a world that no
longer existed (and a blurb telling the user to run a script that had also been
renamed). Nothing caught it, because nothing was looking.

Run with:
    pytest tests/test_demo_catalogue.py
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMOS_JSON = (REPO_ROOT / "projects" / "samples" / "demos" / "controllers"
              / "omnilink_launcher" / "demos.json")

# Repo-relative paths named in a blurb's "RUN:" instruction. Deliberately narrow:
# blurbs are prose, so only match tokens that look like a real script path.
SCRIPT_RE = re.compile(r"(?:^|[\s\"'=])((?:projects|scripts|agents|tests)/[\w./-]+\.(?:sh|ps1|py))")


def _load() -> dict:
    return json.loads(DEMOS_JSON.read_text(encoding="utf-8"))


def _iter_demos(node, out: list):
    """Walk the catalogue and yield every dict that carries a `world` key."""
    if isinstance(node, dict):
        if isinstance(node.get("world"), str):
            out.append(node)
        for value in node.values():
            _iter_demos(value, out)
    elif isinstance(node, list):
        for value in node:
            _iter_demos(value, out)
    return out


def _demos() -> list[dict]:
    demos = _iter_demos(_load(), [])
    assert demos, f"no demo cards found in {DEMOS_JSON} -- did the schema change?"
    return demos


def test_catalogue_is_valid_json():
    _load()


@pytest.mark.parametrize("demo", _demos(), ids=lambda d: d.get("id", "?"))
def test_demo_world_exists(demo):
    world = demo["world"]
    assert world.endswith((".wbt", ".omniworld")), f"{demo.get('id')}: world is not a .wbt: {world}"
    path = REPO_ROOT / world
    assert path.is_file(), (
        f"{demo.get('id')}: launcher card points at a world that does not exist:\n"
        f"  {world}\n"
        f"Clicking Launch on this card does nothing. Either fix the path (the world "
        f"was probably moved or renamed) or remove the card."
    )


@pytest.mark.parametrize("demo", _demos(), ids=lambda d: d.get("id", "?"))
def test_demo_blurb_scripts_exist(demo):
    """A blurb that says 'RUN: bash foo/bar.sh' must name a script that exists."""
    for script in SCRIPT_RE.findall(demo.get("blurb", "") or ""):
        assert (REPO_ROOT / script).is_file(), (
            f"{demo.get('id')}: blurb tells the user to run a script that does not "
            f"exist:\n  {script}\nUpdate the blurb -- the file was renamed or removed."
        )


def test_demo_ids_are_unique():
    ids = [d.get("id") for d in _demos() if d.get("id")]
    duplicates = sorted({i for i in ids if ids.count(i) > 1})
    assert not duplicates, f"duplicate demo ids in the catalogue: {duplicates}"
