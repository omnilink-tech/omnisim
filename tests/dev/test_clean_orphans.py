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

"""Pins on `scripts/dev/clean_orphans.py`'s orphan-object matcher.

The post-checkout hook runs this script, and its third pass deletes any
`src/<x>/build/{debug,release}/<stem>.o` whose source stem it cannot find.
"Source" used to mean C/C++ (+ .qrc) only, so `omnisim.o` -- built by
`src/omnisim/Makefile` from `gui/omnisim.rc` via windres -- was read as an
orphan and deleted after every `git checkout -- <file>`, forcing a relink
(measured three times on 2026-09-02).

Pure functions on a synthetic tree under `tmp_path`; `REPO` is monkeypatched
so nothing here touches the real build dir. No engine, no make.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

import clean_orphans  # noqa: E402


def _fake_repo(tmp_path: Path) -> Path:
    """A src/ tree with every compiled-source kind the Makefiles know, each
    with a matching object in build/release, plus one genuinely dead object."""
    src = tmp_path / "src" / "omnisim"
    (src / "gui").mkdir(parents=True)
    (src / "nodes").mkdir()
    (src / "compute" / "cuda").mkdir(parents=True)
    build = src / "build" / "release"
    build.mkdir(parents=True)

    # Live sources, one per kind.
    (src / "gui" / "omnisim.rc").write_text("1 ICON omnisim.ico\n")
    (src / "nodes" / "OmSolid.cpp").write_text("// cpp\n")
    (src / "nodes" / "OmSolid.hpp").write_text("// hpp\n")
    (src / "gui" / "resources.qrc").write_text("<RCC/>\n")
    (src / "gui" / "OmDialog.ui").write_text("<ui/>\n")
    (src / "compute" / "cuda" / "OmGranular.cu").write_text("// cu\n")
    (src / "compute" / "cuda" / "OmCudaDispatch.cuh").write_text("// cuh\n")

    # Objects the build would leave behind for each of them.
    for name in (
        "omnisim.o",
        "OmSolid.o", "OmSolid.d", "OmSolid.moc.o",
        "qrc_resources.o", "moc_OmSolid.o",
        "OmDialog.o",
        "OmGranular.o", "OmCudaDispatch.o",
        # The control: no source anywhere carries this stem.
        "OmDeletedUpstream.o", "OmDeletedUpstream.d",
    ):
        (build / name).write_bytes(b"\x00")
    return tmp_path


def test_source_exts_cover_every_kind_a_makefile_compiles_into_an_object():
    # .rc is the regression (omnisim.o <- gui/omnisim.rc, windres); the rest are
    # the other non-C++ inputs the tree's Makefiles turn into objects.
    for ext in (".cpp", ".c", ".rc", ".qrc", ".ui", ".cu"):
        assert ext in clean_orphans.SOURCE_EXTS, ext


def test_windres_object_from_an_rc_source_is_not_an_orphan(tmp_path, monkeypatch):
    monkeypatch.setattr(clean_orphans, "REPO", _fake_repo(tmp_path))
    orphans = {p.name for p in clean_orphans.orphan_build_files()}
    assert "omnisim.o" not in orphans


def test_only_the_object_with_no_source_of_any_kind_is_reported(tmp_path, monkeypatch):
    monkeypatch.setattr(clean_orphans, "REPO", _fake_repo(tmp_path))
    orphans = sorted(p.name for p in clean_orphans.orphan_build_files())
    assert orphans == ["OmDeletedUpstream.d", "OmDeletedUpstream.o"]


@pytest.mark.parametrize("missing", ["omnisim.rc", "OmGranular.cu", "resources.qrc"])
def test_deleting_the_non_cpp_source_makes_its_object_an_orphan_again(
    tmp_path, monkeypatch, missing
):
    # Red-capable: the matcher must still see a dead .rc/.cu/.qrc object as
    # orphan, so the fix is a wider match and not a blanket exemption.
    repo = _fake_repo(tmp_path)
    victim = next(repo.rglob(missing))
    victim.unlink()
    monkeypatch.setattr(clean_orphans, "REPO", repo)
    orphans = {p.name for p in clean_orphans.orphan_build_files()}
    expected = {
        "omnisim.rc": "omnisim.o",
        "OmGranular.cu": "OmGranular.o",
        "resources.qrc": "qrc_resources.o",
    }[missing]
    assert expected in orphans
    assert "OmSolid.o" not in orphans
