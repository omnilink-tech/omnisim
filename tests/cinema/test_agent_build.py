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
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))

from cinema import agent_build  # noqa: E402
from cinema import agent_build_capture  # noqa: E402
from cinema import agent_build_pipeline  # noqa: E402
from cinema import agent_build_review  # noqa: E402
from cinema import cli as cinema_cli  # noqa: E402
from cinema import sim_quality  # noqa: E402
import omnisim_capture as capture_service  # noqa: E402


class AgentBuildContractTests(unittest.TestCase):
    def test_starter_manifest_obeys_locked_story_and_delivery_contracts(self) -> None:
        spec = agent_build.parse(agent_build.template("A Build Story"))

        self.assertEqual(spec.fps, 30)
        self.assertEqual((spec.width, spec.height), (1920, 1080))
        self.assertEqual(spec.repository, "github.com/omnilink-tech/omnisim")
        self.assertEqual(agent_build.STYLE_VERSION, "agent_build_v8")
        self.assertEqual(spec.voice.blocks[0].start_s, 10.0)
        self.assertEqual(spec.structure, "three_act")
        self.assertGreaterEqual(spec.simulator_footage_ratio, 0.75)
        self.assertEqual(spec.climax_segment, "climax")
        self.assertTrue(spec.spatial_reorientation_required)
        self.assertEqual(spec.max_consecutive_detail, 2)
        self.assertEqual(spec.wide_reference_segments, ("build_question", "climax"))
        self.assertLess(agent_build.INTRO_SCORE_GAIN, agent_build.STORY_SCORE_GAIN)
        self.assertLess(agent_build.INTRO_MASTER_GAIN, 1.0)
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
        self.assertEqual(result["structure"], "three_act")
        self.assertGreaterEqual(result["simulator_footage_ratio"], 0.75)

    def test_three_act_and_simulator_first_contracts_fail_closed(self) -> None:
        payload = agent_build.template()
        payload["segments"][1]["act"] = 3
        with self.assertRaisesRegex(ValueError, "ordered 1 -> 2 -> 3"):
            agent_build.parse(payload)

        payload = agent_build.template()
        evidence_plate = next(item for item in payload["segments"] if item["kind"] == "plate")
        evidence_plate["duration_s"] = 400
        with self.assertRaisesRegex(ValueError, "simulator footage ratio"):
            agent_build.parse(payload)

        payload = agent_build.template()
        climax = next(item for item in payload["segments"] if item["id"] == "climax")
        climax["duration_s"] = 8
        with self.assertRaisesRegex(ValueError, "climax clip must be at least 12 seconds"):
            agent_build.parse(payload)

    def test_spatial_reorientation_contract_fails_closed(self) -> None:
        payload = agent_build.template()
        payload["segments"][0].pop("coverage")
        with self.assertRaisesRegex(ValueError, "coverage on every clip"):
            agent_build.parse(payload)

        payload = agent_build.template()
        payload["segments"][0]["coverage"] = "detail"
        with self.assertRaisesRegex(ValueError, "must name a wide simulator clip"):
            agent_build.parse(payload)

        payload = agent_build.template()
        clips = [item for item in payload["segments"] if item["kind"] == "clip"]
        clips[0]["coverage"] = "wide"
        clips[1]["coverage"] = "detail"
        clips[2]["coverage"] = "detail"
        clips[3]["coverage"] = "detail"
        with self.assertRaisesRegex(ValueError, "too many consecutive detail clips"):
            agent_build.parse(payload)

    def test_artifact_cache_reuses_only_matching_nonempty_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            output = root / "part.mp4"
            cache = agent_build.ArtifactCache(root, agent_build.PROXY_PROFILE)
            key = cache.key("clip", {"source": "abc", "range": [1, 2]})
            self.assertFalse(cache.hit("part", key, output))
            output.write_bytes(b"media")
            cache.store("part", key, output)

            resumed = agent_build.ArtifactCache(root, agent_build.PROXY_PROFILE)
            self.assertTrue(resumed.hit("part", key, output))
            output.write_bytes(b"corrupt")
            self.assertFalse(resumed.hit("part", key, output))
            self.assertFalse(resumed.hit("part", resumed.key("clip", {"source": "changed"}), output))

    def test_preflight_rejects_impossible_source_range_before_render(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            payload = agent_build.template()
            for source in {item.get("source") for item in payload["segments"] if item.get("source")}:
                path = root / str(source)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"capture")
            for evidence in payload["evidence"]:
                path = root / evidence
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("evidence", encoding="utf-8")
            (root / "narration.txt").write_text("\n\n".join(["Short line."] * 7), encoding="utf-8")
            manifest = root / "film.json"
            manifest.write_text(json.dumps(payload), encoding="utf-8")
            spec = agent_build.parse(manifest)
            facts = {"sha256": "a" * 64, "duration_s": 5.0, "width": 1920,
                     "height": 1080, "fps": 30.0, "frames": 150}
            with mock.patch.object(agent_build, "probe_media", side_effect=lambda path: {
                    "path": str(path), **facts}):
                with self.assertRaisesRegex(ValueError, "requests 0.000-12.000s"):
                    agent_build.preflight(spec)

    def test_declared_tail_hold_makes_short_source_explicit(self) -> None:
        payload = agent_build.template()
        payload["segments"][0]["source_tail_hold_s"] = 1.0
        segment = agent_build.parse(payload).segments[0]
        self.assertEqual(segment.source_tail_hold_s, 1.0)
        payload["segments"][0]["source_tail_hold_s"] = 3.1
        with self.assertRaisesRegex(ValueError, "source_tail_hold_s"):
            agent_build.parse(payload)

    def test_proxy_motion_metric_separates_held_frame_from_small_moving_subject(self) -> None:
        still = np.full((180, 320, 3), 80, dtype=np.uint8)
        held_score, _ = agent_build_review._motion([still, still.copy()])
        moved = still.copy()
        moved[80:90, 100:110] = 255
        moved_again = still.copy()
        moved_again[80:90, 120:130] = 255
        moving_score, _ = agent_build_review._motion([moved, moved_again])
        self.assertEqual(held_score, 0.0)
        self.assertGreater(moving_score, held_score)

    def test_make_never_reaches_full_render_before_proxy_approval(self) -> None:
        spec = agent_build.parse(agent_build.template())
        order: list[str] = []

        def fake_render(_spec, _out, *, profile, **_kwargs):
            order.append(f"render:{profile.name}")
            return {"master": Path(f"{profile.name}.mp4")}

        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(agent_build_pipeline, "preflight", return_value={"valid": True}), \
                mock.patch.object(agent_build_pipeline.agent_build_voice, "generate",
                                  return_value=(Path("voice.wav"), Path("voice.json"))), \
                mock.patch.object(agent_build_pipeline, "render", side_effect=fake_render), \
                mock.patch.object(agent_build_pipeline.agent_build_review, "review_proxy",
                                  side_effect=lambda *_: order.append("review") or {"approved": True}), \
                mock.patch.object(agent_build_pipeline, "verify", return_value={"approved": True}):
            report = agent_build_pipeline.make(spec, out_dir=Path(temp))

        self.assertTrue(report["complete"])
        self.assertEqual(order, ["render:proxy", "review", "render:final"])

    def test_capture_plan_requires_a_complete_sequence_contract(self) -> None:
        plan = agent_build_capture.template("world.omniworld")
        agent_build_capture._validate(plan)
        del plan["shots"][0]["sequence"]["fps"]
        with self.assertRaisesRegex(ValueError, "path_keyframes, duration_s, and fps"):
            agent_build_capture._validate(plan)

    def test_verified_frame_cleanup_is_scoped_and_receipted(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frames = root / "hero_frames"
            frames.mkdir()
            for index in range(3):
                (frames / f"frame_{index:06d}.png").write_bytes(b"png" + bytes([index]))
            output = root / "hero.mp4"
            output.write_bytes(b"encoded-video")
            receipt = root / "hero.mp4.encode.json"
            payload = {
                "input": {"frame_count": 3},
                "output_sha256": sim_quality.sha256(output),
                "output_probe": {"frames": 3, "width": 1920, "height": 1080},
            }

            cleanup = sim_quality._cleanup_verified_frame_spool(
                frames, output, receipt, payload,
            )

            self.assertFalse(frames.exists())
            self.assertEqual(cleanup["removed_files"], 3)
            saved = json.loads(receipt.read_text(encoding="utf-8"))
            self.assertEqual(saved["frame_cleanup"]["status"],
                             "removed_after_verified_encode")

    def test_frame_cleanup_rejects_mixed_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frames = root / "hero_frames"
            frames.mkdir()
            (frames / "frame_000000.png").write_bytes(b"png")
            (frames / "notes.txt").write_text("keep me", encoding="utf-8")
            output = root / "hero.mp4"
            output.write_bytes(b"encoded-video")
            payload = {
                "input": {"frame_count": 1},
                "output_sha256": sim_quality.sha256(output),
                "output_probe": {"frames": 1, "width": 1920, "height": 1080},
            }

            with self.assertRaisesRegex(ValueError, "unexpected entries"):
                sim_quality._cleanup_verified_frame_spool(
                    frames, output, root / "hero.mp4.encode.json", payload,
                )
            self.assertTrue(frames.exists())

    def test_frame_cleanup_rejects_short_encode(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frames = root / "hero_frames"
            frames.mkdir()
            (frames / "frame_000000.png").write_bytes(b"png")
            output = root / "hero.mp4"
            output.write_bytes(b"encoded-video")
            payload = {
                "input": {"frame_count": 1},
                "output_sha256": sim_quality.sha256(output),
                "output_probe": {"frames": 0, "width": 1920, "height": 1080},
            }

            with self.assertRaisesRegex(ValueError, "frame count"):
                sim_quality._cleanup_verified_frame_spool(
                    frames, output, root / "hero.mp4.encode.json", payload,
                )
            self.assertTrue(frames.exists())

    def test_capture_checkpoint_http_routes_use_one_supervisor_session(self) -> None:
        class State:
            started_at = 0.0

            def __init__(self):
                self.calls: list[tuple[str, dict]] = []

            def supervisor_call(self, command, args=None):
                self.calls.append((command, args or {}))
                return {"ok": True}

            def sim_state(self):
                return {"running": True}

        state = State()
        server = ThreadingHTTPServer(("127.0.0.1", 0), capture_service.make_handler(state))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            for endpoint in ("snapshot", "restore"):
                request = urllib.request.Request(
                    f"{base}/sim/{endpoint}", data=b'{"name":"hero"}',
                    headers={"Content-Type": "application/json"}, method="POST",
                )
                with urllib.request.urlopen(request, timeout=5) as response:
                    self.assertEqual(response.status, 200)
            with urllib.request.urlopen(f"{base}/sim/snapshots", timeout=5) as response:
                self.assertEqual(response.status, 200)
        finally:
            server.shutdown()
            server.server_close()
        self.assertEqual(state.calls, [
            ("sim_snapshot", {"name": "hero"}),
            ("sim_restore", {"name": "hero"}),
            ("sim_snapshots", {}),
        ])

    def test_completed_capture_session_resumes_without_contacting_service(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            world = root / "world.omniworld"
            world.write_text("world", encoding="utf-8")
            plan = agent_build_capture.template(str(world))
            plan["settle_steps"] = 0
            plan_path = root / "capture_plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            receipt = root / "receipt.json"

            def fake_request(_service, path, payload=None, timeout_s=0):
                if path == "/healthz":
                    return {"ok": True}
                if path == "/world/load":
                    return {"ok": True}
                if path == "/capture/sequence":
                    Path(payload["output"]).parent.mkdir(parents=True, exist_ok=True)
                    Path(payload["output"]).write_bytes(b"captured")
                    return {"ok": True}
                return {"ok": True}

            with mock.patch.object(agent_build_capture, "_request", side_effect=fake_request):
                first = agent_build_capture.run(plan_path, receipt_path=receipt)
            self.assertTrue(first["complete"])
            self.assertEqual(first["cache"], {"hits": 0, "misses": 1})

            with mock.patch.object(
                    agent_build_capture, "_request",
                    side_effect=AssertionError("cached resume touched service")):
                resumed = agent_build_capture.run(plan_path, receipt_path=receipt)
            self.assertFalse(resumed["service_contacted"])
            self.assertEqual(resumed["cache"], {"hits": 1, "misses": 0})


if __name__ == "__main__":
    unittest.main()
