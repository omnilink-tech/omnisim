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

"""Fail-closed simulation-image gate for Agent Build Films.

Video resolution is not simulation fidelity.  This gate separately proves that
source frames came from OmniSim's wgpu main view, that the authored world uses
the modern lighting/material stack, and that sampled frames are large, exposed,
and visually non-degenerate.  It also encodes lossless PNG sequences only after
that proof passes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image


REQUIRED_LOG_MARKERS = (
    "[OmWgpuBackend] wgpu-native init OK",
    "main view now rendering through the wgpu backend",
    "wgpu main view tonemap: HDR linear-light + AgX",
)
REQUIRED_WORLD_MARKERS = (
    "OmniSimSky",
    "castShadows TRUE",
    "PBRAppearance",
)
FRAME_NAME_RE = re.compile(r"frame_\d+\.png")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sampled_frames(frame_dir: Path, count: int = 9) -> list[Path]:
    frames = sorted(frame_dir.glob("frame_*.png"))
    if not frames:
        raise ValueError(f"no frame_*.png files in {frame_dir}")
    if len(frames) <= count:
        return frames
    indices = np.linspace(0, len(frames) - 1, count, dtype=int)
    return [frames[int(index)] for index in indices]


def analyse_image(path: Path) -> dict:
    image = Image.open(path).convert("RGB")
    rgb = np.asarray(image, dtype=np.float32)
    luma = rgb[..., 0] * 0.2126 + rgb[..., 1] * 0.7152 + rgb[..., 2] * 0.0722
    # Gradient energy catches blank/flat/game-card frames without requiring a
    # scene-specific detector. It is intentionally conservative.
    gx = np.abs(np.diff(luma, axis=1)).mean()
    gy = np.abs(np.diff(luma, axis=0)).mean()
    return {
        "path": str(path.resolve()),
        "width": image.width,
        "height": image.height,
        "mean_luma": round(float(luma.mean()), 3),
        "luma_std": round(float(luma.std()), 3),
        "black_fraction": round(float((luma <= 4).mean()), 5),
        "clipped_fraction": round(float((luma >= 251).mean()), 5),
        "gradient_energy": round(float(gx + gy), 4),
        "sha256": sha256(path),
    }


def verify(frame_dir: Path, log: Path, world: Path, out: Path) -> dict:
    log_text = log.read_text(encoding="utf-8", errors="replace")
    world_text = world.read_text(encoding="utf-8", errors="replace")
    frames = sorted(frame_dir.glob("frame_*.png"))
    samples = [analyse_image(path) for path in sampled_frames(frame_dir)]
    failures: list[str] = []

    for marker in REQUIRED_LOG_MARKERS:
        if marker not in log_text:
            failures.append(f"renderer log missing: {marker}")
    for marker in REQUIRED_WORLD_MARKERS:
        if marker not in world_text:
            failures.append(f"world missing high-fidelity authoring marker: {marker}")
    # Both reviewed appearance PROTOs carry a real normal map.  Asphalt also
    # carries base-colour, roughness, and occlusion maps; requiring its caller
    # to restate the internal ``normalMap`` token in the world would reject a
    # genuinely textured surface merely because it is packaged as a PROTO.
    if not ("Roughcast" in world_text or "Asphalt" in world_text or "normalMap" in world_text):
        failures.append("world has no reviewed normal-mapped surface")
    if not ("Asphalt" in world_text or "baseColorMap" in world_text):
        failures.append("world has no reviewed textured PBR floor/surface")
    for sample in samples:
        if sample["width"] < 1280 or sample["height"] < 720:
            failures.append(f"undersized frame: {sample['path']}")
        if sample["luma_std"] < 14:
            failures.append(f"flat/low-contrast frame: {sample['path']}")
        if sample["gradient_energy"] < 1.2:
            failures.append(f"visually degenerate frame: {sample['path']}")
        if sample["black_fraction"] > 0.60:
            failures.append(f"mostly black frame: {sample['path']}")
        if sample["clipped_fraction"] > 0.32:
            failures.append(f"severely clipped frame: {sample['path']}")

    dimensions = sorted({(item["width"], item["height"]) for item in samples})
    if len(dimensions) != 1:
        failures.append(f"frame dimensions drift: {dimensions}")
    report = {
        "version": 1,
        "gate": "agent_build_simulation_fidelity",
        "passed": not failures,
        "renderer": "wgpu_main_view",
        "features_proven": [
            "HDR linear-light + AgX",
            "PBR materials",
            "normal-mapped surfaces",
            "atmospheric sky",
            "shadow-casting authored sun",
        ],
        "frame_count": len(frames),
        "sample_count": len(samples),
        "dimensions": [list(value) for value in dimensions],
        "frames": samples,
        "world": {"path": str(world.resolve()), "sha256": sha256(world)},
        "log": {"path": str(log.resolve()), "sha256": sha256(log)},
        "failures": failures,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if failures:
        raise SystemExit("simulation fidelity gate failed:\n- " + "\n- ".join(failures))
    return report


def _encode_fingerprint(frame_dir: Path, fps: int, crf: int) -> dict:
    """Build a cheap, change-sensitive receipt without hashing every giant PNG."""
    frames = sorted(frame_dir.glob("frame_*.png"))
    if not frames:
        raise ValueError(f"no frame_*.png files in {frame_dir}")
    indices = sorted({0, len(frames) // 4, len(frames) // 2,
                      (3 * len(frames)) // 4, len(frames) - 1})
    return {
        "version": 1,
        "frame_count": len(frames),
        "total_bytes": sum(path.stat().st_size for path in frames),
        "sampled_frames": [
            {"name": frames[index].name, "bytes": frames[index].stat().st_size,
             "sha256": sha256(frames[index])}
            for index in indices
        ],
        "fps": fps,
        "crf": crf,
        "filter": "scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080",
        "codec": "libx264/slow/yuv420p",
    }


def _cleanup_verified_frame_spool(
    frame_dir: Path,
    output: Path,
    receipt: Path,
    receipt_payload: dict,
) -> dict:
    """Remove only the exact PNG spool bound to a verified encoded output."""
    frame_dir = frame_dir.resolve()
    output = output.resolve()
    if not frame_dir.is_dir() or frame_dir.is_symlink():
        raise ValueError(f"frame spool is not a regular directory: {frame_dir}")
    if frame_dir.parent != output.parent:
        raise ValueError("refusing frame cleanup outside the encoded output directory")
    if "frame" not in frame_dir.name.lower():
        raise ValueError(f"refusing non-frame directory cleanup: {frame_dir}")
    entries = list(frame_dir.iterdir())
    if not entries or any(
            not path.is_file() or not FRAME_NAME_RE.fullmatch(path.name)
            for path in entries):
        raise ValueError(f"frame spool contains unexpected entries: {frame_dir}")
    if not output.is_file() or output.stat().st_size == 0:
        raise ValueError(f"encoded output is missing or empty: {output}")
    expected_sha = receipt_payload.get("output_sha256")
    if not expected_sha or sha256(output) != expected_sha:
        raise ValueError(f"encoded output hash does not match receipt: {output}")
    fingerprint = receipt_payload.get("input", {})
    output_probe = receipt_payload.get("output_probe", {})
    if output_probe.get("frames") != fingerprint.get("frame_count"):
        raise ValueError(f"encoded output frame count does not match spool: {output}")
    if (output_probe.get("width"), output_probe.get("height")) != (1920, 1080):
        raise ValueError(f"encoded output dimensions are not 1920x1080: {output}")

    removed_bytes = sum(path.stat().st_size for path in entries)
    removed_files = len(entries)
    shutil.rmtree(frame_dir)
    cleanup = {
        "status": "removed_after_verified_encode",
        "frame_dir": str(frame_dir),
        "removed_files": removed_files,
        "removed_bytes": removed_bytes,
    }
    receipt_payload["frame_cleanup"] = cleanup
    receipt.write_text(json.dumps(receipt_payload, indent=2) + "\n", encoding="utf-8")
    return cleanup


def _probe_encoded_output(output: Path) -> dict:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise SystemExit("ffprobe is required on PATH before frame cleanup")
    completed = subprocess.run([
        ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
        "-show_entries", "stream=nb_read_frames,width,height",
        "-of", "json", str(output),
    ], check=True, capture_output=True, text=True)
    payload = json.loads(completed.stdout)
    streams = payload.get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"encoded output has no single video stream: {output}")
    stream = streams[0]
    try:
        return {
            "frames": int(stream["nb_read_frames"]),
            "width": int(stream["width"]),
            "height": int(stream["height"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"encoded output probe is incomplete: {output}") from exc


def encode(
    frame_dir: Path,
    output: Path,
    fps: int,
    crf: int,
    *,
    keep_frames: bool = False,
) -> bool:
    """Encode a verified frame spool, reusing a hash-bound local result when fresh."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("ffmpeg is required on PATH")
    output.parent.mkdir(parents=True, exist_ok=True)
    fingerprint = _encode_fingerprint(frame_dir, fps, crf)
    receipt = output.with_suffix(output.suffix + ".encode.json")
    reused = False
    receipt_payload: dict = {}
    if output.exists() and receipt.exists():
        try:
            cached = json.loads(receipt.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            cached = {}
        if (cached.get("input") == fingerprint and
                cached.get("output_sha256") == sha256(output)):
            reused = True
            receipt_payload = cached
    if not reused:
        subprocess.run([
            ffmpeg, "-y", "-xerror", "-hide_banner", "-loglevel", "error",
            "-framerate", str(fps),
            "-i", str(frame_dir / "frame_%06d.png"),
            "-vf", "scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080",
            "-c:v", "libx264", "-preset", "slow", "-crf", str(crf),
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
        ], check=True)
        receipt_payload = {
            "version": 1,
            "input": fingerprint,
            "output": str(output.resolve()),
            "output_sha256": sha256(output),
        }
        receipt.write_text(json.dumps(receipt_payload, indent=2) + "\n", encoding="utf-8")

    output_probe = _probe_encoded_output(output)
    receipt_payload["output_probe"] = output_probe
    receipt.write_text(json.dumps(receipt_payload, indent=2) + "\n", encoding="utf-8")
    if output_probe["frames"] != fingerprint["frame_count"]:
        raise ValueError(
            f"encoded {output_probe['frames']} of {fingerprint['frame_count']} frames; "
            f"retaining spool for repair: {frame_dir.resolve()}"
        )

    if keep_frames:
        receipt_payload["frame_cleanup"] = {
            "status": "retained_by_request",
            "frame_dir": str(frame_dir.resolve()),
            "removed_files": 0,
            "removed_bytes": 0,
        }
        receipt.write_text(json.dumps(receipt_payload, indent=2) + "\n", encoding="utf-8")
    else:
        _cleanup_verified_frame_spool(frame_dir, output, receipt, receipt_payload)
    return reused


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    verify_parser = sub.add_parser("verify")
    verify_parser.add_argument("--frames", type=Path, required=True)
    verify_parser.add_argument("--log", type=Path, required=True)
    verify_parser.add_argument("--world", type=Path, required=True)
    verify_parser.add_argument("--out", type=Path, required=True)
    encode_parser = sub.add_parser("encode")
    encode_parser.add_argument("--frames", type=Path, required=True)
    encode_parser.add_argument("--output", type=Path, required=True)
    encode_parser.add_argument("--fps", type=int, default=30)
    encode_parser.add_argument("--crf", type=int, default=12)
    encode_parser.add_argument(
        "--keep-frames", action="store_true",
        help="retain the verified PNG spool after encoding (off by default)",
    )
    args = parser.parse_args()
    if args.command == "verify":
        report = verify(args.frames, args.log, args.world, args.out)
        print(json.dumps({"passed": True, "frames": report["frame_count"], "dimensions": report["dimensions"]}))
    else:
        reused = encode(
            args.frames, args.output, args.fps, args.crf,
            keep_frames=args.keep_frames,
        )
        print(f"{'reused' if reused else 'encoded'}: {args.output.resolve()}")


if __name__ == "__main__":
    main()
