# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from cinema import agent_build  # noqa: E402
from cinema import cli as cinema_cli  # noqa: E402


class AgentBuildContractTests(unittest.TestCase):
    def test_starter_manifest_obeys_locked_story_and_delivery_contracts(self) -> None:
        spec = agent_build.parse(agent_build.template("A Build Story"))

        self.assertEqual(spec.fps, 30)
        self.assertEqual((spec.width, spec.height), (1920, 1080))
        self.assertEqual(spec.repository, "github.com/omnilink-tech/omnisim")
        self.assertEqual(spec.voice.blocks[0].start_s, 10.0)
        self.assertEqual(list(agent_build.REQUIRED_BEATS), [
            "question", "attempt", "control", "evidence", "method", "boundary", "conclusion"
        ])

    def test_edl_hard_codes_silent_two_screen_intro_and_github_outro(self) -> None:
        spec = agent_build.parse(agent_build.template())
        edl = agent_build.build_edl(spec)
        entries = edl["entries"]

        self.assertEqual(
            (entries[0]["id"], entries[0]["source"], entries[0]["master_in_s"],
             entries[0]["master_out_s"]),
            ("disclaimer", "MOTION_GRAPHIC:disclaimer", 0.0, 5.0),
        )
        self.assertEqual(
            (entries[1]["id"], entries[1]["source"], entries[1]["master_in_s"],
             entries[1]["master_out_s"]),
            ("story_intro", "MOTION_GRAPHIC:story_intro", 5.0, 10.0),
        )
        self.assertEqual(entries[2]["master_in_s"], 10.0)
        self.assertEqual(entries[-1]["source"], "MOTION_GRAPHIC:locked_outro")
        self.assertEqual(entries[-1]["master_out_s"], spec.duration_s)
        self.assertEqual(edl["transition_vocabulary"], ["direct_cut"])

    def test_missing_or_reordered_evidence_beats_fail_closed(self) -> None:
        payload = agent_build.template()
        payload["segments"] = [item for item in payload["segments"] if item["beat"] != "control"]
        with self.assertRaisesRegex(ValueError, "missing required beats: control"):
            agent_build.parse(payload)

        payload = agent_build.template()
        payload["segments"][1], payload["segments"][2] = payload["segments"][2], payload["segments"][1]
        with self.assertRaisesRegex(ValueError, "required beats must first appear in order"):
            agent_build.parse(payload)

    def test_first_footage_and_voice_cannot_move_into_silent_intro(self) -> None:
        payload = agent_build.template()
        payload["segments"][0]["kind"] = "plate"
        payload["segments"][0]["headline"] = "A PLATE"
        with self.assertRaisesRegex(ValueError, "first post-intro segment must be real"):
            agent_build.parse(payload)

        payload = agent_build.template()
        payload["voice"]["blocks"][0]["start_s"] = 9.9
        with self.assertRaisesRegex(ValueError, "exactly at 10.0 seconds"):
            agent_build.parse(payload)

    def test_source_rewind_requires_visible_analytical_replay(self) -> None:
        payload = agent_build.template()
        payload["segments"][1]["source"] = payload["segments"][0]["source"]
        payload["segments"][1]["source_in_s"] = 2.0
        with self.assertRaisesRegex(ValueError, "rewinds source time"):
            agent_build.parse(payload)

        payload["segments"][1]["replay"] = True
        payload["segments"][1]["replay_label"] = "MATCHED RUNS · ANALYTICAL REPLAY"
        self.assertTrue(agent_build.parse(payload).segments[1].replay)

    def test_evidence_boundary_layout_and_voice_pacing_fail_closed(self) -> None:
        payload = agent_build.template()
        payload["evidence"] = []
        with self.assertRaisesRegex(ValueError, "needs evidence files"):
            agent_build.parse(payload)

        payload = agent_build.template()
        next(item for item in payload["segments"] if item["beat"] == "boundary").pop("claim_boundary")
        with self.assertRaisesRegex(ValueError, "must carry an exact on-screen claim_boundary"):
            agent_build.parse(payload)

        payload = agent_build.template()
        payload["segments"][0]["overlay"]["headline"] = "W" * 100
        with self.assertRaisesRegex(ValueError, "overlay text exceeds its safe card"):
            agent_build.parse(payload)

        payload = agent_build.template()
        payload["voice"]["blocks"][0]["speed"] = 1.2
        with self.assertRaisesRegex(ValueError, "natural narration speed"):
            agent_build.parse(payload)

    def test_cli_validate_reports_locked_signature(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            manifest = Path(temp) / "film.json"
            manifest.write_text(json.dumps(agent_build.template()), encoding="utf-8")
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result_code = cinema_cli.main(["agent-build-validate", str(manifest)])

        self.assertEqual(result_code, 0)
        result = json.loads(output.getvalue())
        self.assertTrue(result["valid"])
        self.assertEqual(result["locked_intro"], {
            "disclosure_s": [0, 5], "story_signature_s": [5, 10], "voiceover": False,
        })
        self.assertEqual(result["locked_outro"], agent_build.GITHUB_DESTINATION)


if __name__ == "__main__":
    unittest.main()
