from pathlib import Path

from scripts.dev.include_fanout import collect_fanout, normalized_dependencies


def test_normalized_dependencies_handles_make_continuations(tmp_path: Path) -> None:
    dependency = tmp_path / "one.d"
    dependency.write_text("one.o: source.cpp include/OmNode.hpp \\\n include/space\\ name.hpp\n", encoding="utf-8")

    assert normalized_dependencies(dependency) == {
        "source.cpp",
        "include/OmNode.hpp",
        "include/space name.hpp",
    }


def test_collect_fanout_counts_each_translation_unit_once(tmp_path: Path) -> None:
    (tmp_path / "a.d").write_text("a.o: OmNode.hpp OmNode.hpp OmWorld.hpp\n", encoding="utf-8")
    (tmp_path / "b.d").write_text("b.o: OmNode.hpp source.cpp\n", encoding="utf-8")

    translation_units, fanout = collect_fanout(tmp_path)

    assert translation_units == 2
    assert fanout == {"OmNode.hpp": 2, "OmWorld.hpp": 1}
