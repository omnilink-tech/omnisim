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

CHAT_DIR = REPO_ROOT / "projects" / "samples" / "demos" / "worlds" / "chat"
DEMOS_MD = REPO_ROOT / "DEMOS.md"
CHAT_INDEX_MD = CHAT_DIR / "OMNILINK_CHAT_DEMOS.md"

# The one true count, asserted here so it cannot drift on any surface:
# 21 chat demos = 20 `omnilink_<robot>.omniworld` + `omniarm6_talk.omniworld`.
# Quoted in DEMOS.md section 1, WORLDS.md 1a, the chat index, AGENTS.md 3c and
# the smoke_chat_demos.py docstring. If you add or remove a chat demo, change
# this number AND every one of those pages in the same commit.
CHAT_DEMO_COUNT = 21

# Not a demo: the fixture `tests/benchmarks/langsoak/` drives, pinned to port
# 8775 so it cannot collide with a chat demo on 8765. Deliberately absent from
# demos.json, DEMOS.md and the smoke sweep -- an exclusion recorded is better
# than an exclusion hidden, so it is named here and asserted absent below.
CHAT_FIXTURES = {"omnilink_husky_langsoak.omniworld"}


def _chat_demo_worlds() -> list[Path]:
    """Every VISIBLE, non-fixture chat world on disk.

    ⚠️ `pathlib.Path.glob` matches dotfiles, unlike the glob module's "*".
    The chat folder carries ~40 untracked `.eval_*` / `.ddbot_*` / `.harness_*`
    scratch worlds; a bare glob finds 62 files and a sweep built on it once
    reported "39 worlds" nobody had ever catalogued. Hidden means
    deliberately-not-catalogued, so name-starts-with-dot is the filter.
    """
    return sorted(p for p in CHAT_DIR.glob("*.omniworld")
                  if not p.name.startswith(".") and p.name not in CHAT_FIXTURES)


def _catalogued_worlds() -> set[str]:
    """Repo-relative world paths named in demos.json, in posix form."""
    return {d["world"].replace("\\", "/") for d in _demos()}

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


# --------------------------------------------------------------------------
# Chat-demo parity.
#
# The forward check above (a card must point at a world that exists) has always
# been there. The REVERSE check was not, which is why the launcher drifted to
# 15 chat cards against 21 chat worlds without anything going red: the five Deep
# Robotics quadrupeds and `omniarm6_talk` were shipped, documented in DEMOS.md,
# and simply never added to the gallery. A user opening the launcher saw ONE
# quadruped and concluded the quadruped surface was one robot.
#
# A world that exists but is in no catalogue does not exist, per the project
# rule. These tests make that rule executable in both directions.
# --------------------------------------------------------------------------


@pytest.mark.parametrize("world", _chat_demo_worlds(), ids=lambda p: p.stem)
def test_chat_world_is_in_the_launcher(world: Path):
    rel = world.relative_to(REPO_ROOT).as_posix()
    assert rel in _catalogued_worlds(), (
        f"chat demo {world.name} ships but has NO card in the launcher:\n"
        f"  missing from: {DEMOS_JSON.relative_to(REPO_ROOT).as_posix()}\n"
        f"  world:        {rel}\n"
        f"A user browsing the in-sim gallery cannot find this robot at all. Add "
        f"an entry to the right chat category (chat_arms / chat_mobile / "
        f"chat_quadruped / chat_aerial), or -- if it is a benchmark fixture "
        f"rather than a demo -- name it in CHAT_FIXTURES here and say why."
    )


@pytest.mark.parametrize("world", _chat_demo_worlds(), ids=lambda p: p.stem)
def test_chat_world_is_in_demos_md(world: Path):
    text = DEMOS_MD.read_text(encoding="utf-8")
    assert world.name in text, (
        f"chat demo {world.name} ships but is NOT catalogued:\n"
        f"  missing from: {DEMOS_MD.relative_to(REPO_ROOT).as_posix()}\n"
        f"Add a row under 'Chat demos' in the table for its robot class. A world "
        f"not in the catalogue does not exist."
    )


@pytest.mark.parametrize("world", _chat_demo_worlds(), ids=lambda p: p.stem)
def test_chat_world_is_in_the_folder_index(world: Path):
    text = CHAT_INDEX_MD.read_text(encoding="utf-8")
    assert world.name in text, (
        f"chat demo {world.name} ships but is NOT in the folder index:\n"
        f"  missing from: {CHAT_INDEX_MD.relative_to(REPO_ROOT).as_posix()}\n"
        f"That file calls itself the complete index of the chat demos; it has to "
        f"be one. Add its row under the heading for its robot class."
    )


def test_chat_demo_count_is_the_one_true_count():
    worlds = _chat_demo_worlds()
    named = sorted(p.name for p in worlds)
    pattern = [n for n in named if n.startswith("omnilink_")]
    assert len(worlds) == CHAT_DEMO_COUNT, (
        f"the chat demo count moved: {len(worlds)} worlds on disk, "
        f"CHAT_DEMO_COUNT says {CHAT_DEMO_COUNT}.\n"
        f"  on disk: {named}\n"
        f"That number is quoted on five surfaces and they must move together: "
        f"DEMOS.md section 1, WORLDS.md 1a, "
        f"projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md, "
        f"AGENTS.md section 3c, and the docstring of "
        f"scripts/dev/smoke_chat_demos.py (which reports 'N of N'). Update all "
        f"five and CHAT_DEMO_COUNT in the same commit."
    )
    assert len(pattern) == CHAT_DEMO_COUNT - 1 and "omniarm6_talk.omniworld" in named, (
        f"the fleet is documented as {CHAT_DEMO_COUNT - 1} `omnilink_<robot>` "
        f"worlds plus `omniarm6_talk.omniworld`; on disk it is "
        f"{len(pattern)} + {sorted(set(named) - set(pattern))}. Fix the docs or "
        f"the filenames -- the distinction is why AGENTS.md may not say all "
        f"{CHAT_DEMO_COUNT} are `omnilink_<robot>.omniworld`."
    )


@pytest.mark.parametrize("fixture", sorted(CHAT_FIXTURES))
def test_chat_fixture_is_not_catalogued_as_a_demo(fixture: str):
    """A benchmark fixture must stay OUT of the demo surfaces.

    The langsoak Husky is a measurement rig on port 8775. Promoting it to a demo
    would make every '21' on every page wrong in the other direction.
    """
    assert (CHAT_DIR / fixture).is_file(), (
        f"{fixture} is declared a chat fixture here but does not exist. If it "
        f"was deleted, drop it from CHAT_FIXTURES."
    )
    rel = (CHAT_DIR / fixture).relative_to(REPO_ROOT).as_posix()
    assert rel not in _catalogued_worlds(), (
        f"{fixture} is a benchmark fixture, not a demo, but it has a launcher "
        f"card in {DEMOS_JSON.relative_to(REPO_ROOT).as_posix()}. Remove the "
        f"card, or promote it properly and fix every count that says "
        f"{CHAT_DEMO_COUNT}."
    )
