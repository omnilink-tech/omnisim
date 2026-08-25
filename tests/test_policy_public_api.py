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

"""Contracts for the installable OmniSim policy brain."""

from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from omnisim.policy import baton as PUBLIC_BATON  # noqa: E402
from omnisim.policy.artifacts import build_record, evaluate_promotion  # noqa: E402
from omnisim.policy.compatibility import verify_g1_envs  # noqa: E402
from omnisim.policy.matrix import load as load_matrix  # noqa: E402
from omnisim.policy.matrix import validate as validate_matrix  # noqa: E402
from omnisim.policy.motion_ir import MotionBinding, MotionIR  # noqa: E402
from omnisim.policy.repository import PolicyRepository  # noqa: E402
from omnisim.policy.skill_graph import SkillGraph  # noqa: E402
from projects.policies.training import baton as LEGACY_BATON  # noqa: E402


def _legacy_module():
    path = REPO / "projects" / "policies" / "skills" / "skill_lib.py"
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("_policy_public_api_legacy", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PolicyPublicApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = PolicyRepository(REPO)
        cls.skills = cls.repo.skill_catalog()
        cls.sequences = cls.repo.sequence_catalog()

    def test_old_baton_import_is_the_public_implementation(self):
        self.assertIs(LEGACY_BATON.BatonState, PUBLIC_BATON.BatonState)
        self.assertIs(LEGACY_BATON.step, PUBLIC_BATON.step)
        self.assertIs(LEGACY_BATON._blend, PUBLIC_BATON._blend)

    def test_every_sequence_is_a_typed_lossless_graph(self):
        for name, sequence in self.sequences.items():
            with self.subTest(sequence=name):
                graph = SkillGraph.from_sequence(sequence, self.skills)
                self.assertEqual([], graph.validate())
                self.assertEqual(sequence["arbiter"], graph.compile_legacy())
                self.assertEqual(max(0, len(graph.nodes) - 1), len(graph.edges))

    def test_every_skill_has_explicit_valid_motion_ir_and_binding(self):
        motions = self.repo.motion_catalog()
        self.assertEqual(set(self.skills), set(motions))
        for name, skill in self.skills.items():
            with self.subTest(skill=name):
                motion = MotionIR.from_skill({**skill, "motion_ir": motions[name]})
                binding = MotionBinding.from_skill(skill, motion)
                self.assertEqual([], motion.validate())
                self.assertEqual([], binding.validate())
                self.assertNotIn(binding.robot, motion.control_axes)

    def test_flagship_g1_deploy_envs_are_byte_stable(self):
        legacy = _legacy_module()
        registry = legacy.M.Registry.discover()

        def assemble(name):
            return legacy._assemble_env(registry.sequences[name], registry)

        self.assertEqual([], verify_g1_envs(assemble))

    def test_front_door_advertises_every_skill_library_verb(self):
        """`omnisim policy --help` must list every verb it will actually dispatch.

        The two surfaces are built from one table (skill_verbs.py). They used to be
        two lists: skill_lib.py declared the subparsers, omnisim/policy/cli.py kept a
        hand-copied name set that it forwarded but never registered -- so the
        documented front door ran `list`/`preview`/`train`/`sequence` while its own
        --help denied they existed. This pins the surfaces together.
        """
        from omnisim.policy import cli as policy_cli

        legacy = _legacy_module()
        advertised = set(policy_cli._parser()._subparsers._group_actions[0].choices)
        self.assertTrue(legacy._SUBCOMMANDS <= advertised,
                        f"not advertised by `omnisim policy --help`: "
                        f"{sorted(legacy._SUBCOMMANDS - advertised)}")
        for verb in policy_cli.NATIVE:
            self.assertIn(verb, advertised)
        # Everything advertised is either implemented here or delegated -- nothing
        # can be listed in the help and then fall through to `return 2`.
        self.assertEqual(set(), advertised - set(policy_cli.NATIVE) - policy_cli.PASSTHROUGH)

    def test_artifact_record_is_deterministic_and_box_is_release_eligible(self):
        legacy = _legacy_module()
        registry = legacy.M.Registry.discover()
        env = legacy._assemble_env(registry.sequences["box_delivery"], registry)
        evidence = ["projects/policies/benchmarks/results/g1_box_delivery_e2e_2026-07-19.json"]
        first = build_record(self.repo, "sequence", "box_delivery", assembled_env=env,
                             evidence_paths=evidence)
        second = build_record(self.repo, "sequence", "box_delivery", assembled_env=env,
                              evidence_paths=evidence)
        self.assertEqual(first, second)
        matrix = load_matrix(REPO / "projects/policies/benchmarks/matrix.json")
        self.assertEqual([], validate_matrix(self.repo, matrix))
        acceptance = json.loads(
            (REPO / "projects/policies/benchmarks/suite.json").read_text(encoding="utf-8"))
        cases = [case for case in acceptance["cases"] if case["name"] == "g1_box_delivery_e2e"]
        verdict = evaluate_promotion(first, self.sequences["box_delivery"], cases)
        self.assertEqual("release", verdict["promotion_tier"])
        self.assertEqual("9722d23d12a3", verdict["benchmark_evidence"][0]["machine_id"])


if __name__ == "__main__":
    unittest.main()
