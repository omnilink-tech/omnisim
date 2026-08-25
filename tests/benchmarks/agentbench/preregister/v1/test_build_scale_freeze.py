"""Freeze guard for BuildScale v1 before any comparative score exists."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
SPEC = HERE / "build_scale_spec.json"
CORE = HERE / "build_scale_core.py"
CORE_TEST = AGENTBENCH / "graders" / "test_build_scale_core.py"
MANIFEST = HERE / "build_scale_freeze_manifest.json"
FROZEN = (SPEC, CORE, CORE_TEST, Path(__file__).resolve())


def _normal_bytes(path: Path) -> bytes:
    return path.read_bytes().replace(b"\r\n", b"\n")


def _sha(path: Path) -> str:
    return hashlib.sha256(_normal_bytes(path)).hexdigest()


def _scored_rows() -> list[str]:
    found = []
    for path in AGENTBENCH.rglob("*.jsonl"):
        try:
            for number, line in enumerate(path.read_text(
                    encoding="utf-8", errors="replace").splitlines(), 1):
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if (isinstance(row, dict)
                        and row.get("suite") == "agenticsimbench/v1"):
                    found.append(f"{path}:{number}")
        except OSError:
            continue
    return found


def build_manifest() -> dict:
    return {
        "schema": "agenticsimbench/build-scale-freeze/v1",
        "suite": "agenticsimbench/v1",
        "frozen_utc": "2026-08-13",
        "hash_rule": "sha256 over LF-normalised bytes",
        "files": {
            str(path.relative_to(AGENTBENCH)).replace("\\", "/"): _sha(path)
            for path in FROZEN
        },
    }


class TestBuildScaleFreeze(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = json.loads(SPEC.read_text(encoding="utf-8"))

    def test_levels_match_the_parent_v1_contract(self):
        parent = json.loads((HERE / "contract.json").read_text(encoding="utf-8"))
        self.assertEqual(self.spec["levels"], parent["build_scale"]["robots"])

    def test_all_frozen_physical_requirements_are_operational(self):
        assertions = self.spec["assertions"]
        self.assertEqual(list(assertions), [f"BS.{i}" for i in range(1, 10)])
        for term in ("interpenetration", "move", "runaway", "collision",
                     "teleport", "replay", "control", "portable"):
            self.assertIn(term, " ".join(assertions.values()).lower())

    def test_scale_is_not_an_fps_benchmark(self):
        prompt = self.spec["prompt_template"].lower()
        self.assertIn("independently controlled", prompt)
        self.assertIn("portable", prompt)
        self.assertIn("20 simulated seconds", prompt)

    def test_missing_numeric_values_cannot_become_zero(self):
        self.assertIn("INVALID", self.spec["outcome"])

    def test_no_scored_v1_row_predates_this_freeze(self):
        self.assertEqual(_scored_rows(), [])

    def test_frozen_files_match_manifest(self):
        recorded = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(recorded, build_manifest())


def _write() -> None:
    rows = _scored_rows()
    if rows:
        raise SystemExit(
            "REFUSING: v1 scored rows exist; BuildScale v1 cannot be rewritten:\n"
            + "\n".join(rows))
    MANIFEST.write_text(json.dumps(build_manifest(), indent=2) + "\n",
                        encoding="utf-8")
    print(f"wrote {MANIFEST}")


if __name__ == "__main__":
    if "--write" in sys.argv[1:]:
        _write()
    else:
        unittest.main()
