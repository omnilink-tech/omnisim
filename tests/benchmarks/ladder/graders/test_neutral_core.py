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

"""The structural guard on the ladder's neutral cores. **No simulator.**

Replicated from ``agentbench/graders/test_neutral_core.py`` -- same AST walk,
same rule -- because a guard that only covers the frozen suite stops being a
guard the moment a new tier is written. It is the thing that keeps
"sim-agnostic graders, per-sim adapters" true as the ladder grows, rather than
true on the day it was written.

    pytest tests/benchmarks/ladder/graders/test_neutral_core.py -q

The token list is agentbench's, plus the names of the roster's own products.
A core that mentions any of them **in code** -- a string literal, an
identifier, an attribute -- has leaked back across the line the split exists to
draw. Docstrings are exempt on purpose: explaining *why* a boundary exists
requires naming what is on the other side of it.
"""

from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ladder.graders import (ladder_evidence, t1_core, t2_core,  # noqa: E402
                            t3_core, t3_evidence, t4_core, t4_evidence)

# agentbench's list, imported rather than retyped so the two guards cannot
# drift apart, plus the roster's product names -- a neutral core may not know
# which simulator it is grading, and that includes not knowing ours.
_AB_GUARD = importlib.import_module("agentbench.graders.test_neutral_core")

_LADDER_EXTRA_TOKENS = ("urdf", "omnisim", "webots", "gazebo", "isaac",
                        "mujoco", "playground", "genesis", "vrml",
                        "externproto")

SIM_TOKENS = tuple(_AB_GUARD._SIM_TOKENS) + _LADDER_EXTRA_TOKENS

# Every module a ladder core is allowed to consist of. **A new tier's core
# goes in this tuple in the same commit that creates it** -- that is the whole
# reason this guard was replicated here instead of being left in agentbench:
# a guard that only covers the tiers that existed when it was written stops
# being a guard the moment the ladder grows.
NEUTRAL_MODULES = (t1_core, t2_core, t3_core, t3_evidence, t4_core,
                   t4_evidence, ladder_evidence)


def _code_strings_and_names(path):
    """Every string literal and identifier in a module, MINUS docstrings."""
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docstrings:
                out.append(node.value)
        elif isinstance(node, ast.Name):
            out.append(node.id)
        elif isinstance(node, ast.Attribute):
            out.append(node.attr)
    return out


@pytest.mark.parametrize("mod", NEUTRAL_MODULES)
def test_ladder_cores_contain_no_simulator_specific_vocabulary(mod):
    path = mod.__file__
    for text in _code_strings_and_names(path):
        low = text.lower()
        for token in SIM_TOKENS:
            assert token not in low, (
                "%s mentions %r in code (not a docstring): %r"
                % (Path(path).name, token, text))


def test_the_guard_inherits_agentbenchs_token_list_rather_than_copying_it():
    """A second list is a list that drifts. This one must be the same one."""
    for token in _AB_GUARD._SIM_TOKENS:
        assert token in SIM_TOKENS


def test_the_guard_would_actually_catch_a_leak(tmp_path):
    """The guard's own falsifier: plant a leak, prove it is seen.

    A structural test that has never been observed failing is a structural
    test nobody has checked -- the same rule the graders live under.
    """
    leaky = tmp_path / "leaky_core.py"
    leaky.write_text('"""A docstring may say urdf freely."""\n'
                     'SCENE_SUFFIX = ".wbt"\n', encoding="utf-8")
    found = [t for text in _code_strings_and_names(leaky)
             for t in SIM_TOKENS if t in text.lower()]
    assert ".wbt" in found
    # ...and the docstring mention did NOT trip it
    assert "urdf" not in found


def test_the_cores_import_no_simulator_module():
    """A neutral core may import the neutral contract and numpy. Not a column.

    Checked on the import graph rather than the vocabulary, because a core
    could import a per-simulator module without ever spelling its name in a
    string.
    """
    banned = ("adapters", "controllers", "headless", "launcher")
    for mod in NEUTRAL_MODULES:
        tree = ast.parse(Path(mod.__file__).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for name in names:
                parts = (name or "").split(".")
                for b in banned:
                    assert b not in parts, (
                        "%s imports %r, which is column-specific"
                        % (Path(mod.__file__).name, name))
