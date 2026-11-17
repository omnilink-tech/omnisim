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

"""Engine-free pins for scripts/release/public_strip.py (the public-snapshot strip stage)."""
import importlib.util
import json
import pathlib

import pytest

_P = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "release" / "public_strip.py"
_spec = importlib.util.spec_from_file_location("public_strip", _P)
ps = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ps)

B, E = "public:strip-" + "begin", "public:strip-" + "end"
L = "public:strip-" + "line"
O = "public:" + "only - "


def test_markdown_block_removed_with_its_markers():
    text = f"keep 1\n<!-- {B} -->\nprivate row\n<!-- {E} -->\nkeep 2\n"
    assert ps.strip_text(text) == ("keep 1\nkeep 2\n", 1)


def test_python_comment_markers_and_two_blocks():
    text = f"a = 1\n# {B}\nb = 2\n# {E}\nc = 3\n    # {B}\n    d = 4\n    # {E}\n"
    assert ps.strip_text(text) == ("a = 1\nc = 3\n", 2)


def test_crlf_kept():
    assert ps.strip_text(f"x\r\n# {B}\r\ny\r\n# {E}\r\nz\r\n") == ("x\r\nz\r\n", 1)


@pytest.mark.parametrize("text", [f"# {B}\nx\n", f"x\n# {E}\n", f"# {B}\n# {B}\n# {E}\n", f"<!-- {B} --> <!-- {E} -->\n"])
def test_malformed_markers_fail_closed(text):
    with pytest.raises(ps.StripError):
        ps.strip_text(text)


def test_public_only_line_becomes_its_content():
    text = (f"<!-- {B} -->\nprivate wording\n<!-- {E} -->\n<!-- {O}Public wording. -->\n"
            f"    # {O}call_public()\r\nend\n")
    assert ps.strip_text(text) == ("Public wording.\n    call_public()\r\nend\n", 3)


def test_public_only_inside_text_fails_closed():
    with pytest.raises(ps.StripError):
        ps.strip_text(f"x = 1  # {O}y\n")


def test_strip_line_and_quoted_public_only():
    text = (f"| a | b |\n| x | y | <!-- {L} -->\n> keep\n> <!-- {B} -->\n> old\n> <!-- {E} -->\n"
            f"> <!-- {O}new -->\n")
    assert ps.strip_text(text) == ("| a | b |\n> keep\n> new\n", 3)


def test_prune_json_drops_paths_that_do_not_ship_and_keeps_layout():
    src = json.dumps({"tolerance": 0.1, "worlds": {"a/kept.omniworld": {"v": 1}, "b/held.omniworld": {"v": 2}}},
                     indent=2) + "\n"
    out, gone = ps.prune_json(src, "worlds", lambda p: p.startswith("a/"))
    assert gone == 1
    assert out == json.dumps({"tolerance": 0.1, "worlds": {"a/kept.omniworld": {"v": 1}}}, indent=2) + "\n"


def test_cpp_comment_public_only():
    text = f"  // private note  // {L}\n  // {O}// public note\nint x;\n"
    assert ps.strip_text(text) == ("  // public note\nint x;\n", 2)


def test_text_without_markers_is_unchanged():
    t = "no markers here\n"
    assert ps.strip_text(t) == (t, 0)
