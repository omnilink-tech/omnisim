"""Regression contracts for the fast agent build loop."""

from pathlib import Path

from omnisim.dev.changed_build import affected_objects, dependency_prerequisites


ROOT = Path(__file__).resolve().parents[1]


def test_public_gui_build_uses_ccache_enabled_root_makefile():
    commands = (ROOT / "omnisim/dev/commands.py").read_text(encoding="utf-8")
    assert '"gui":              ["make", f"-j{j}", "sim-gui-staged"' in commands
    assert '["make", "-C", "src/omnisim"' not in commands


def test_dependency_parser_normalizes_simulator_and_repo_relative_paths(tmp_path):
    dep = tmp_path / "OmThing.d"
    dep.write_text(
        "OmThing.o: nodes/OmThing.cpp ../../include/controller/c/omnisim/types.h \\\n"
        " core/OmPrecision.hpp\n",
        encoding="utf-8",
    )
    prerequisites = dependency_prerequisites(dep)
    assert "src/omnisim/nodes/OmThing.cpp" in prerequisites
    assert "include/controller/c/omnisim/types.h" in prerequisites
    assert "src/omnisim/core/OmPrecision.hpp" in prerequisites


def test_affected_objects_follow_depfiles_and_new_sources(tmp_path):
    (tmp_path / "OmOne.d").write_text(
        "OmOne.o: nodes/OmOne.cpp core/OmShared.hpp\n", encoding="utf-8"
    )
    (tmp_path / "OmTwo.d").write_text(
        "OmTwo.o: nodes/OmTwo.cpp core/OmShared.hpp\n", encoding="utf-8"
    )
    assert affected_objects({"src/omnisim/core/OmShared.hpp"}, tmp_path) == ["OmOne.o", "OmTwo.o"]
    assert affected_objects({"src/omnisim/nodes/OmNew.cpp"}, tmp_path) == ["OmNew.o"]


def test_compile_only_and_staged_targets_are_declared():
    root_makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
    simulator_makefile = (ROOT / "src/omnisim/Makefile").read_text(encoding="utf-8")
    assert "sim-check-objects:" in root_makefile
    assert "sim-gui-staged:" in root_makefile
    assert "check-objects: $(CHECK_OBJECTS)" in simulator_makefile
    assert "sha256sum $(WGPU_NATIVE_HOME)/lib/wgpu_native.dll" in simulator_makefile
