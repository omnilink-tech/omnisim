"""Regression checks for one-pass compiler dependency generation.

These tests intentionally cover both makefile families: most OmniSim targets use
``resources/Makefile.include``, while the simulator has its own hand-written
rules in ``src/omnisim/Makefile``.
"""

from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_shared_make_rules_generate_dependencies_during_compilation():
    makefile = _text("resources/Makefile.include")

    assert "DEPENDENCY_FLAGS ?= -MMD -MP" in makefile
    assert "$(BUILD_GOAL_DIR)/%.d:%.cpp" not in makefile
    assert " -MM $<" not in makefile

    compile_commands = [
        line.strip()
        for line in makefile.splitlines()
        if " -c " in line and ("$(CC)" in line or "$(CXX)" in line)
    ]
    assert compile_commands
    assert all("$(DEPENDENCY_FLAGS)" in line for line in compile_commands)

    # Fat binaries must read the files emitted beside each architecture's object.
    assert "$(BUILD_GOAL_DIR)/x86_64/,$(DEPENDENCY_NAMES)" in makefile
    assert "$(BUILD_GOAL_DIR)/arm64/,$(DEPENDENCY_NAMES)" in makefile


def test_simulator_rules_keep_generated_moc_dependencies_incremental():
    makefile = _text("src/omnisim/Makefile")

    assert "DEPENDENCY_OUTPUT = $(if $(filter $(OBJDIR)/%,$@)" in makefile
    assert "DEPENDENCY_FLAGS = -MMD -MP -MF $(DEPENDENCY_OUTPUT) -MT $@" in makefile
    assert "CXXFLAGS += -std=c++17 -DQT_NO_DEBUG $(DEPENDENCY_FLAGS)" in makefile
    assert "$(OBJDIR)/%.d: %.cpp" not in makefile
    assert " -MM " not in makefile

    # Generated MOC sources remain precious, are compiled by the generated-source
    # rule, and have their emitted dependency files included on every platform.
    assert ".PRECIOUS: $(MOC_FILES)" in makefile
    assert "%.o: $(OBJDIR)/%.cpp" in makefile
    assert "$(MOC_FILES:.cpp=.d)" in makefile
    assert "$(OBJDIR)/x86_64/.deps/,$(DEPENDENCY_NAMES)" in makefile
    assert "$(OBJDIR)/arm64/.deps/,$(DEPENDENCY_NAMES)" in makefile
    assert "dependency-stats:" in makefile
    assert "OLD_DEPENDENCY_PREPASS_COUNT" in makefile
    # Keep the generation rule restricted to canonical MOC_FILES.  A broad
    # implicit rule combines with -MP phony headers and invents nested outputs
    # such as build/release/nodes/OmMotor.moc.cpp.
    assert "$(MOC_FILES): $(OBJDIR)/%.moc.cpp: %.hpp" in makefile
    assert "\n$(OBJDIR)/%.moc.cpp: %.hpp" not in makefile
    assert "$(DEPENDENCY_FILES): | $(DEPENDENCY_MIGRATION_STAMP)" in makefile
    assert "$(DEPENDENCY_MIGRATION_STAMP):" in makefile
    assert "$(OBJDIR)/.deps/%.d: $(OBJDIR)/%.d" not in makefile

    # The sole C source also gains dependency tracking; it was missed by the old
    # SOURCES:.cpp=.d substitution.
    assert "$(CC) -c $(DEPENDENCY_FLAGS)" in makefile
    assert "$(C_SOURCES:.c=.d)" in makefile


def test_top_level_ccache_configuration_is_relocatable_and_diagnosable():
    makefile = _text("Makefile")

    assert "CCACHE_BASEDIR ?= $(OMNISIM_HOME)" in makefile
    assert "export CCACHE_BASEDIR" in makefile
    assert "ccache-stats:" in makefile
    assert "$(CCACHE) -p" in makefile
    assert "$(CCACHE) -s" in makefile


def test_compiler_default_depfile_tracks_generated_source_and_header(tmp_path: Path):
    """Prove the default -MMD output follows ``-o`` into the object directory.

    This is the behavior the MOC rule relies on.  Skip only on source-only hosts
    that do not have a C++ compiler installed.
    """

    compiler = next(
        (path for name in ("c++", "g++", "clang++") if (path := shutil.which(name))),
        None,
    )
    if compiler is None:
        raise unittest.SkipTest("no C++ compiler installed")

    source = tmp_path / "generated.moc.cpp"
    header = tmp_path / "generated.hpp"
    object_dir = tmp_path / "objects"
    object_dir.mkdir()
    header.write_text("#define GENERATED_VALUE 7\n", encoding="utf-8")
    source.write_text('#include "generated.hpp"\nint value = GENERATED_VALUE;\n', encoding="utf-8")
    object_path = object_dir / "generated.moc.o"

    subprocess.run(
        [compiler, "-c", "-MMD", "-MP", str(source), "-o", str(object_path)],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )

    dependency_path = object_path.with_suffix(".d")
    dependency = dependency_path.read_text(encoding="utf-8")
    assert dependency_path.exists()
    assert str(object_path) in dependency
    assert str(header) in dependency
    # -MP emits a phony header target, which keeps make usable after a header is
    # removed or renamed between incremental builds.
    assert f"{header}:" in dependency
