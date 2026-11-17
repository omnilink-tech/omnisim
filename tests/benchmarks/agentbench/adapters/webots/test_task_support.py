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

"""WSL-free unit tests for the task-world support module (AABB merge, B2
camera channel, prober injection). The live half is exercised by the oracle
runs whose artifacts are committed under ``preregister/evidence/``."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.adapters.webots import task_support  # noqa: E402


class TestProberStanza(unittest.TestCase):
    def test_stanza_names_the_prober_and_out_dir(self):
        s = task_support.prober_stanza("/mnt/o/run/aabb")
        self.assertIn('controller "agentbench_aabb_prober"', s)
        self.assertIn("supervisor TRUE", s)
        self.assertIn('"--out-dir=/mnt/o/run/aabb"', s)

    def test_prober_controller_exists_and_is_upstream_only(self):
        src = (task_support.PROBER_DIR
               / "agentbench_aabb_prober.py").read_text(encoding="utf-8")
        self.assertIn("from controller import Supervisor", src)
        # Must run under upstream's python3: no OmniSim/agentbench imports.
        self.assertNotIn("agentbench", src.replace(
            "agentbench_aabb_prober", "").replace("agentbench_recorder", ""))
        self.assertNotIn("import omnisim", src)


class TestAugmentRun(unittest.TestCase):
    def _run_dir(self, tmp, bodies):
        d = Path(tmp)
        (d / "roster.json").write_text(json.dumps(
            {"t_s": 0.0, "frozen": True, "synchronization": True,
             "bodies": bodies}), encoding="utf-8")
        return d

    def test_merge_by_name_and_absent_stays_absent(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            d = self._run_dir(tmp, [
                {"name": "a", "position": [0, 0, 0]},
                {"name": "b", "position": [1, 0, 0]},
            ])
            doc = {"bodies": {"a": {"bbox_min": [0, 0, 0],
                                    "bbox_max": [1, 1, 1]},
                              "c": {"bbox_min": [9, 9, 9],
                                    "bbox_max": [9, 9, 9]}},
                   "source": "test"}
            run, merged = task_support.augment_run(d, doc)
            self.assertEqual(merged, ["a"])
            recs = {r["name"]: r for r in run.roster["bodies"]}
            self.assertIn("bounds", recs["a"])
            self.assertNotIn("bounds", recs["b"])  # never invented
            self.assertIn("aabb_merge", run.roster)

    def test_no_aabbs_doc_is_a_clean_no_op(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            d = self._run_dir(tmp, [{"name": "a"}])
            run, merged = task_support.augment_run(d, None)
            self.assertEqual(merged, [])


class TestCameraPose(unittest.TestCase):
    def test_flu_identity(self):
        doc = {"viewpoint": {"position": [1, 2, 3],
                             "orientation": [0, 0, 1, 0],
                             "field_of_view": 0.785398}}
        cam = task_support.camera_pose(doc, source="t")
        for got, want in zip(cam.forward, (1, 0, 0)):
            self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(cam.up, (0, 0, 1)):
            self.assertAlmostEqual(got, want, places=9)

    def test_flu_yaw_90(self):
        # +90 deg about z turns +x forward into +y; up stays +z.
        doc = {"viewpoint": {"position": [0, 0, 0],
                             "orientation": [0, 0, 1, math.pi / 2],
                             "field_of_view": 0.785398}}
        cam = task_support.camera_pose(doc, source="t")
        self.assertAlmostEqual(cam.forward[0], 0.0, places=9)
        self.assertAlmostEqual(cam.forward[1], 1.0, places=9)
        self.assertAlmostEqual(cam.up[2], 1.0, places=9)

    def test_missing_viewpoint_is_none(self):
        self.assertIsNone(task_support.camera_pose({}, source="t"))
        self.assertIsNone(task_support.camera_pose(None, source="t"))


class TestViewEvidence(unittest.TestCase):
    def test_artifact_parsed_reflects_final_measurement(self):
        vp = {"viewpoint": {"position": [0, 0, 5],
                            "orientation": [0, 0, 1, 0.5],
                            "field_of_view": 0.785398}}
        ok = task_support.view_evidence(vp, vp, artifact="x.wbt")
        self.assertTrue(ok.artifact_parsed)
        self.assertIsNotNone(ok.initial)
        broken = task_support.view_evidence(vp, None, artifact="x.wbt")
        self.assertFalse(broken.artifact_parsed)
        self.assertIsNone(broken.final)


if __name__ == "__main__":
    unittest.main()
