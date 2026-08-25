# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Pure-stdlib guards for the extracted OmniSim Newton Python runtime."""

from __future__ import annotations

import ast
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
PHYSICS = REPO / "src" / "omnisim" / "physics"
RUNTIME = PHYSICS / "omnisim_newton_runtime.py"
CPP = PHYSICS / "OmNewtonBackend.cpp"
HPP = PHYSICS / "OmNewtonBackend.hpp"
BUNDLER = REPO / "scripts" / "packaging" / "bundle_newton_runtime.py"
SMOKE = PHYSICS / "newton_embed_smoke.cpp"


def test_runtime_is_a_valid_importable_python_source():
    tree = ast.parse(RUNTIME.read_text(encoding="utf-8"), filename=str(RUNTIME))
    assert any(isinstance(node, ast.ClassDef) and node.name == "World"
               for node in tree.body)


def test_cpp_imports_module_instead_of_embedding_a_second_copy():
    cpp = CPP.read_text(encoding="utf-8")
    assert b"\0" not in CPP.read_bytes()
    assert 'PyImport_ImportModule("omnisim_newton_runtime")' in cpp
    assert "kNewtonRuntimeSource" not in cpp
    assert 'R"PY(' not in cpp


def test_bundler_stages_and_verifies_engine_runtime_module():
    bundler = BUNDLER.read_text(encoding="utf-8")
    assert '"omnisim_newton_runtime"' in bundler
    assert "stage_omnisim_runtime_module" in bundler
    assert "OMNISIM_RUNTIME_SOURCE" in bundler


def test_cpp_smoke_uses_the_same_import_path_as_production():
    smoke = SMOKE.read_text(encoding="utf-8")
    assert 'PyImport_ImportModule("omnisim_newton_runtime")' in smoke
    assert "kNewtonRuntimeSource" not in smoke


def test_preload_api_is_world_independent_and_reused_by_constructor():
    hpp = HPP.read_text(encoding="utf-8")
    cpp = CPP.read_text(encoding="utf-8")
    assert "static bool preloadRuntime();" in hpp
    body = cpp[cpp.index("bool OmNewtonBackend::preloadRuntime()") :]
    body = body[:body.index("\n}") + 2]
    assert "tryInitNewtonRuntime(&probe)" in body
    assert "beginWorld" not in body
    assert "finalizeWorld" not in body
    assert "attachCachedRuntime(runtime)" in cpp
