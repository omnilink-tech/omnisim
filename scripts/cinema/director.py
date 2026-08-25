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

"""Director — orchestrates a full storyboard end-to-end.

Owns the workflow:
  1. Spin up (or reuse) a capture service
  2. Load the storyboard's world
  3. Resolve the subject's live pose
  4. For each shot: pick lens, compute keyframes, render mp4
  5. Hand the raw shots to the editor for grade + assembly + multi-aspect
  6. Optionally run the critique loop and re-render flagged shots

This is the entry point the CLI calls.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.request
import urllib.error
from dataclasses import dataclass, field
from pathlib import Path

from . import beats, critique, edit, looks, shot, storyboard, subjects


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_SVC = "http://127.0.0.1:6791"


@dataclass
class DirectorOptions:
    svc_base_url: str = DEFAULT_SVC
    output_root: Path = REPO_ROOT / "social" / "youtube_videos" / "captures"
    skip_critique: bool = False
    keep_raw_shots: bool = True
    """Keep the per-shot mp4s next to the assembled deliverables. Helpful
    for re-editing later without re-rendering."""
    skip_edit: bool = False
    """Render shots but don't assemble — useful when debugging an
    individual beat. Implies keep_raw_shots."""


@dataclass
class DirectorResult:
    storyboard_title: str
    output_dir: Path
    raw_shots: list[shot.ShotResult] = field(default_factory=list)
    deliverables: dict[str, Path] = field(default_factory=dict)
    """aspect_ratio_label -> path of the assembled mp4."""
    notes: list[str] = field(default_factory=list)


def _wait_healthy(svc: str, timeout_s: float = 30.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"{svc}/healthz", timeout=2) as resp:
                if resp.status == 200:
                    return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def _ensure_service(opts: DirectorOptions) -> bool:
    if _wait_healthy(opts.svc_base_url, timeout_s=2):
        return True
    print(f"[director] capture service not running on {opts.svc_base_url}; "
          "start it with `python -m omnisim capture --port 6791` and retry",
          file=sys.stderr)
    return False


def _post(svc: str, endpoint: str, payload: dict, timeout_s: float = 300.0) -> dict:
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{svc}{endpoint}", data=body,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _load_world(svc: str, world_path: str, lens_fov_rad: float, wait_s: int = 75) -> dict:
    return _post(svc, "/world/load", {
        "path": world_path,
        "width": 1920, "height": 1080,
        "fov": lens_fov_rad,
        "wait_s": wait_s,
    }, timeout_s=wait_s + 30)


def direct(sb: storyboard.Storyboard, opts: DirectorOptions | None = None) -> DirectorResult:
    """Render and assemble a storyboard. Returns paths to all deliverables."""
    opts = opts or DirectorOptions()
    if not _ensure_service(opts):
        raise RuntimeError("capture service unreachable")

    out_dir = opts.output_root / f"cinema_{sb.slug}_{int(time.time())}"
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = out_dir / "shots"
    raw_dir.mkdir(parents=True, exist_ok=True)
    result = DirectorResult(storyboard_title=sb.title, output_dir=out_dir)

    look = sb.resolved_look()
    print(f"[director] '{sb.title}' — subject={sb.subject_name} "
          f"look={look.name} shots={len(sb.shots)} "
          f"total~{sb.total_duration_s():.1f}s", flush=True)

    # World load with the FIRST shot's lens FoV — OmniSim bakes the field
    # of view into the viewport at world load and changing it mid-render
    # requires a reload. Most storyboards share a lens family within a
    # piece; a future improvement is to group shots by lens and reload
    # between groups, but for now we accept the small framing variance.
    first_lens = sb.shots[0].resolve_lens(look)
    load = _load_world(opts.svc_base_url, sb.world, first_lens.horizontal_fov_rad)
    if not load.get("ok"):
        raise RuntimeError(f"world load failed: {load.get('error')}")
    print(f"[director] loaded {sb.world} in {load.get('load_ms')}ms", flush=True)

    # Settle the sim before resolving subject pose — many worlds spawn
    # robots at translation 0 0 X and then drop to a stable resting pose.
    if sb.world_settle_steps > 0:
        _post(opts.svc_base_url, "/sim/step", {"steps": int(sb.world_settle_steps)},
              timeout_s=60)

    # Resolve subject AFTER settling — its pose is now stable.
    try:
        subject = subjects.resolve(opts.svc_base_url, sb.subject_name)
    except Exception as exc:
        # Fall back to a synthetic "origin subject" so we still produce
        # output — the storyboard is graded for the subject this world
        # actually contains, and the failure mode here is usually
        # "world doesn't have that named robot".
        prof = subjects.find_profile(sb.subject_name)
        subject = subjects.Subject(
            profile=prof or list(subjects.PROFILES.values())[0],
            position=(0.0, 0.0, 0.0),
        )
        result.notes.append(
            f"subject {sb.subject_name!r} not found in world; using origin pose ({exc})"
        )
        print(f"[director] WARN: {result.notes[-1]}", flush=True)
    else:
        print(f"[director] subject {sb.subject_name} at {subject.position}", flush=True)

    # Render shots one by one.
    for i, spec in enumerate(sb.shots):
        lens = spec.resolve_lens(look)
        print(f"[director] shot {i+1}/{len(sb.shots)} "
              f"beat={spec.beat} lens={lens.name} dur={spec.resolve_duration_s():.1f}s",
              flush=True)
        try:
            sr = shot.render_shot(
                opts.svc_base_url, subject, spec, look, lens, raw_dir, i, fps=sb.fps,
            )
            result.raw_shots.append(sr)
            print(f"[director]   -> {sr.output_path.name}", flush=True)
        except Exception as exc:
            print(f"[director]   FAILED: {exc}", flush=True)
            result.notes.append(f"shot {i} ({spec.beat}) failed: {exc}")

    if not result.raw_shots:
        raise RuntimeError("no shots rendered — aborting before edit")

    # Critique loop — optional, requires ANTHROPIC_API_KEY. The agent reviews
    # each rendered shot and flags any that need a reshoot. We retry once per
    # flagged shot with the suggested adjustment; if it still scores low, we
    # ship the better of the two takes.
    if not opts.skip_critique:
        print(f"[director] running critique on {len(result.raw_shots)} shots", flush=True)
        for i, sr in enumerate(result.raw_shots):
            spec = sb.shots[i]
            crit = critique.critique_shot(
                sr.output_path, sr.name,
                beat_intent=beats.get(spec.beat).feels_like,
                subject_name=sb.subject_name,
                shot_kind=spec.resolve_shot_primitive().replace("_", " "),
            )
            if crit.skipped:
                print(f"[director]   crit skipped: {crit.error}", flush=True)
                break  # No key/SDK → don't keep retrying for every shot.
            if crit.error:
                print(f"[director]   crit error on {sr.name}: {crit.error}", flush=True)
                continue
            scores_str = "/".join(str(v) for v in crit.scores.values())
            print(f"[director]   {sr.name}: {scores_str} avg={crit.overall:.1f} "
                  f"{'RESHOOT' if crit.needs_reshoot else 'OK'}", flush=True)
            if not crit.needs_reshoot:
                continue
            delta = critique.adjustment_to_param_delta(crit.adjust, crit.magnitude)
            if not delta:
                continue
            tweaked_params = dict(spec.params)
            for k, v in delta.items():
                if k == "distance_chardim_mult":
                    base = tweaked_params.get("distance_chardim", 3.0)
                    tweaked_params["distance_chardim"] = max(1.0, base * v)
                elif k == "azimuth_deg_offset":
                    tweaked_params["azimuth_deg"] = tweaked_params.get("azimuth_deg", 0.0) + v
                elif k == "elevation_deg_offset":
                    tweaked_params["elevation_deg"] = (
                        tweaked_params.get("elevation_deg", 10.0) + v
                    )
            tweaked_spec = storyboard.ShotSpec(
                beat=spec.beat, duration_s=spec.duration_s, lens=spec.lens,
                shot=spec.shot, params=tweaked_params, look_override=spec.look_override,
            )
            try:
                lens2 = tweaked_spec.resolve_lens(look)
                sr2 = shot.render_shot(
                    opts.svc_base_url, subject, tweaked_spec, look, lens2,
                    raw_dir, i, fps=sb.fps,
                )
                # Re-critique the retake; ship the better one.
                crit2 = critique.critique_shot(
                    sr2.output_path, sr2.name,
                    beat_intent=beats.get(spec.beat).feels_like,
                    subject_name=sb.subject_name,
                    shot_kind=tweaked_spec.resolve_shot_primitive().replace("_", " "),
                )
                if not crit2.skipped and not crit2.error and crit2.overall > crit.overall:
                    print(f"[director]   retake improved {crit.overall:.1f}->{crit2.overall:.1f}, "
                          "using retake", flush=True)
                    sr2.output_path.replace(sr.output_path)
                    result.raw_shots[i] = sr2
                else:
                    print(f"[director]   retake didn't improve, keeping original",
                          flush=True)
                    try:
                        sr2.output_path.unlink()
                    except OSError:
                        pass
            except Exception as exc:
                print(f"[director]   retake failed: {exc}", flush=True)

    if opts.skip_edit:
        print("[director] skip_edit=True; raw shots kept, no assembly", flush=True)
        return result

    print(f"[director] assembling {len(result.raw_shots)} shots", flush=True)
    deliverables = edit.assemble(
        raw_shots=[s.output_path for s in result.raw_shots],
        storyboard=sb,
        look=look,
        out_dir=out_dir,
    )
    result.deliverables = deliverables
    for label, path in deliverables.items():
        print(f"[director] deliverable {label}: {path.name}", flush=True)

    # Write a manifest so the director's outputs are self-describing.
    manifest = {
        "title": sb.title,
        "subject": sb.subject_name,
        "world": sb.world,
        "look": look.name,
        "shots": [
            {
                "name": s.name, "duration_s": s.duration_s,
                "fov_rad": s.fov_rad, "output": s.output_path.name,
                "keyframes": s.keyframes,
            }
            for s in result.raw_shots
        ],
        "deliverables": {k: str(v) for k, v in deliverables.items()},
        "notes": result.notes,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return result
