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

"""One-command, measured Agent Build production pipeline."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable

from . import agent_build_capture, agent_build_review, agent_build_voice
from .agent_build import FINAL_PROFILE, PROXY_PROFILE, AgentBuildSpec, preflight, render, verify


BENCHMARK_TARGET_MINUTES = 35.0
REVISION_TARGET_MINUTES = 5.0


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _stage_summary(name: str, value: Any) -> Any:
    """Keep the performance receipt compact; detailed receipts stay per stage."""
    if name.endswith("preflight") and isinstance(value, dict):
        return {
            "valid": value.get("valid"),
            "sources": len(value.get("sources", [])),
            "ranges": len(value.get("ranges", [])),
            "voice_blocks": len(value.get("voice_blocks", [])),
            "warnings": len(value.get("warnings", [])),
        }
    if name == "proxy_review" and isinstance(value, dict):
        return {
            "approved": value.get("approved"),
            "blockers": len(value.get("blockers", [])),
            "warnings": len(value.get("warnings", [])),
            "overview_sheet": value.get("overview_sheet"),
            "cut_pair_sheet": value.get("cut_pair_sheet"),
        }
    if name == "capture_session" and isinstance(value, dict):
        return {key: value.get(key) for key in
                ("complete", "elapsed_s", "cache", "service_contacted")}
    if name == "release_verify" and isinstance(value, dict):
        return {key: value.get(key) for key in
                ("approved", "master", "master_sha256", "duration_s", "frames")}
    return _jsonable(value)


def make(spec: AgentBuildSpec, *, out_dir: Path | None = None,
         capture_plan: Path | None = None,
         service: str = "http://127.0.0.1:6791",
         target_minutes: float = BENCHMARK_TARGET_MINUTES) -> dict[str, Any]:
    """Run capture (optional) -> preflight -> voice -> proxy -> critique -> final.

    The full-resolution render is unreachable until the proxy critique writes
    an approval.  Every stage is resumable through the lower-level caches.
    """
    base_out = (out_dir or spec.source_path.parent / "build" / spec.slug).resolve()
    base_out.mkdir(parents=True, exist_ok=True)
    report_path = base_out / "workflow_performance.json"
    started = time.perf_counter()
    stages: list[dict[str, Any]] = []
    outputs: dict[str, Any] = {}

    def stage(name: str, operation: Callable[[], Any]) -> Any:
        stage_started = time.perf_counter()
        try:
            value = operation()
        except Exception as exc:
            stages.append({
                "name": name, "status": "failed",
                "elapsed_s": round(time.perf_counter() - stage_started, 3),
                "error": str(exc),
            })
            _write(False, str(exc))
            raise
        stages.append({
            "name": name, "status": "complete",
            "elapsed_s": round(time.perf_counter() - stage_started, 3),
        })
        outputs[name] = _stage_summary(name, value)
        return value

    def _write(complete: bool, error: str | None = None) -> dict[str, Any]:
        elapsed = time.perf_counter() - started
        cache_summary: dict[str, Any] = {}
        for label, path in (
            ("proxy", base_out / "proxy" / "render_report.json"),
            ("final", base_out / "render_report.json"),
            ("voice", (spec.source_path.parent / spec.voice.wav).resolve()
             .with_suffix(".segments.json")),
        ):
            try:
                cached = json.loads(path.read_text(encoding="utf-8")).get("cache")
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                cached = None
            if cached is not None:
                cache_summary[label] = {
                    key: cached.get(key) for key in ("hits", "misses", "master_hit")
                    if key in cached
                }
        payload = {
            "version": 1, "pipeline": "agent_build_make_v1",
            "complete": complete, "error": error,
            "manifest": str(spec.source_path), "output_dir": str(base_out),
            "targets": {
                "ready_simulation_minutes": target_minutes,
                "normal_revision_minutes": REVISION_TARGET_MINUTES,
            },
            "elapsed_s": round(elapsed, 3),
            "elapsed_minutes": round(elapsed / 60.0, 3),
            "within_benchmark_target": complete and elapsed <= target_minutes * 60.0,
            "stages": stages, "outputs": outputs, "cache": cache_summary,
            "scope": (
                "The 35-minute benchmark starts with a working simulation. "
                "Novel controller/physics engineering is measured separately."
            ),
        }
        report_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return payload

    if capture_plan is not None:
        # Validate narrative/evidence/voice shape before opening the capture session.
        stage("manifest_preflight", lambda: preflight(spec, require_captures=False))
        stage("capture_session", lambda: agent_build_capture.run(
            capture_plan, service=service,
            receipt_path=base_out / "capture_session_receipt.json",
        ))
    stage("media_preflight", lambda: preflight(spec, require_captures=True))
    stage("voice", lambda: agent_build_voice.generate(spec, preflight_checked=True))
    stage("proxy_render", lambda: render(
        spec, base_out, profile=PROXY_PROFILE, preflight_checked=True,
    ))
    review = stage("proxy_review", lambda: agent_build_review.review_proxy(spec, base_out))
    if not review.get("approved"):
        raise RuntimeError("proxy review did not approve the edit")
    stage("final_render", lambda: render(
        spec, base_out, profile=FINAL_PROFILE, preflight_checked=True,
    ))
    stage("release_verify", lambda: verify(spec, base_out))
    return _write(True)
