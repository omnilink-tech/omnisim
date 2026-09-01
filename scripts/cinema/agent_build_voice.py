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

"""Natural local narration for :mod:`scripts.cinema.agent_build`.

The runtime is pinned, checksum-verified, and stored under the repository's
ignored ``.tmp/`` directory.  Voice blocks carry explicit editorial windows;
copy that does not fit is rejected instead of being unnaturally accelerated.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import urllib.request
from pathlib import Path

import numpy as np

from .agent_build import AgentBuildSpec, REPO_ROOT, parse, preflight


DEFAULT_RUNTIME = REPO_ROOT / ".tmp" / "agent_build_voice"
REQUIREMENTS = Path(__file__).with_name("requirements-agent-build-voice.txt")
RELEASE = "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.1"
MODEL_FILES = {
    "kokoro-v1.0.fp16.onnx": "f3a290d384fbb27966d462905c71a46cef9e5fd00516b40df32a0b4afe77ac96",
    "voices-v1.0.bin": "bca610b8308e8d99f32e6fe4197e7ec01679264efed0cac9140fe9c29f1fbf7d",
}
VOICE_CACHE_VERSION = 1


def _runtime_root(runtime: Path | None = None) -> Path:
    override = os.environ.get("OMNISIM_AGENT_BUILD_VOICE_RUNTIME")
    return (Path(override) if override else (runtime or DEFAULT_RUNTIME)).resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _runtime_paths(root: Path) -> tuple[Path, Path, Path]:
    """Resolve the canonical layout and the original milestone's local layout."""
    canonical = (root / "deps", root / "models" / "kokoro-v1.0.fp16.onnx",
                 root / "models" / "voices-v1.0.bin")
    if all(path.exists() for path in canonical):
        return canonical
    return (root / "kokoro_deps", root / "kokoro_model" / "kokoro-v1.0.fp16.onnx",
            root / "kokoro_model" / "voices-v1.0.bin")


def setup_runtime(runtime: Path | None = None) -> Path:
    """Install the pinned Kokoro runtime and verify both model files."""
    root = _runtime_root(runtime)
    deps = root / "deps"
    models = root / "models"
    deps.mkdir(parents=True, exist_ok=True)
    models.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        sys.executable, "-m", "pip", "install", "--disable-pip-version-check",
        "--target", str(deps), "-r", str(REQUIREMENTS),
    ], check=True)
    for name, expected in MODEL_FILES.items():
        destination = models / name
        if destination.exists() and _sha256(destination) == expected:
            print(f"[agent-build voice] verified {name}")
            continue
        partial = destination.with_suffix(destination.suffix + ".partial")
        print(f"[agent-build voice] downloading {name}")
        urllib.request.urlretrieve(f"{RELEASE}/{name}", partial)
        actual = _sha256(partial)
        if actual != expected:
            partial.unlink(missing_ok=True)
            raise RuntimeError(f"checksum mismatch for {name}: {actual}")
        partial.replace(destination)
    return root


def _output_path(spec: AgentBuildSpec, value: str) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (spec.source_path.parent / path).resolve()


def _stable_hash(payload: object) -> str:
    body = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(body).hexdigest()


def generate(spec: AgentBuildSpec, runtime: Path | None = None, *,
             preflight_checked: bool = False) -> tuple[Path, Path]:
    """Generate the manifest's narration WAV and timing receipt locally."""
    if not preflight_checked:
        preflight(spec)
    root = _runtime_root(runtime)
    deps, model, voices = _runtime_paths(root)
    missing = [path for path in (deps, model, voices) if not path.exists()]
    if missing:
        raise RuntimeError("voice runtime is not ready; run `python -m omnisim cinema agent-build-voice-setup`")
    script_path = _output_path(spec, spec.voice.script)
    if not script_path.exists():
        raise FileNotFoundError(script_path)
    blocks = [part.strip() for part in script_path.read_text(encoding="utf-8").split("\n\n") if part.strip()]
    if len(blocks) != len(spec.voice.blocks):
        raise ValueError(f"narration has {len(blocks)} blocks; manifest declares {len(spec.voice.blocks)} windows")

    output = _output_path(spec, spec.voice.wav)
    output.parent.mkdir(parents=True, exist_ok=True)
    timings = output.with_suffix(".segments.json")
    cache_dir = output.parent / ".voice_cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    block_inputs = [{
        "text": text, "voice": spec.voice.voice, "speed": window.speed,
        "sentence_pause_s": window.sentence_pause_s,
        "clause_pause_s": window.clause_pause_s,
    } for text, window in zip(blocks, spec.voice.blocks)]
    build_key = _stable_hash({
        "version": VOICE_CACHE_VERSION, "duration_s": spec.duration_s,
        "model": MODEL_FILES, "blocks": block_inputs,
        "windows": [window.__dict__ for window in spec.voice.blocks],
    })
    try:
        previous = json.loads(timings.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        previous = {}
    if (previous.get("build_key") == build_key and output.exists()
            and output.stat().st_size > 0):
        previous["cache"] = {"hits": len(blocks), "misses": 0, "master_hit": True}
        timings.write_text(json.dumps(previous, indent=2) + "\n", encoding="utf-8")
        print(f"[agent-build voice] master cache hit: {output}")
        return output, timings

    sys.path.insert(0, str(deps))
    import soundfile as sf  # type: ignore  # noqa: PLC0415

    kokoro = None
    sample_rate = 24000
    master = np.zeros(round(spec.duration_s * sample_rate), dtype=np.float32)
    records: list[dict[str, object]] = []
    overlong: list[str] = []
    cache_hits = 0
    cache_misses = 0
    for index, (text, window) in enumerate(zip(blocks, spec.voice.blocks), 1):
        cache_key = _stable_hash({
            "version": VOICE_CACHE_VERSION, "model": MODEL_FILES,
            **block_inputs[index - 1],
        })
        cached_wav = cache_dir / f"{cache_key}.wav"
        cached_meta = cache_dir / f"{cache_key}.json"
        if cached_wav.exists() and cached_meta.exists():
            samples, rate = sf.read(str(cached_wav), dtype="float32")
            samples = np.asarray(samples, dtype=np.float32)
            cache_hit = True
            cache_hits += 1
        else:
            if kokoro is None:
                os.environ.setdefault("ORT_LOG_SEVERITY_LEVEL", "3")
                import onnxruntime as ort  # type: ignore  # noqa: PLC0415
                from kokoro_onnx import Kokoro  # type: ignore  # noqa: PLC0415
                ort.set_default_logger_severity(3)
                kokoro = Kokoro(str(model), str(voices))
            audio, rate = kokoro.create(
                text, voice=spec.voice.voice, speed=window.speed, lang="en-us",
                sentence_pause=window.sentence_pause_s, clause_pause=window.clause_pause_s,
                continuous=False,
            )
            samples = np.asarray(audio, dtype=np.float32)
            samples -= float(np.mean(samples))
            peak = float(np.max(np.abs(samples))) or 1.0
            samples *= min(1.0, 0.86 / peak)
            sf.write(str(cached_wav), samples, rate, subtype="PCM_24")
            cached_meta.write_text(json.dumps({
                "key": cache_key, "text": text, "sample_rate": rate,
                "samples": len(samples),
            }, indent=2) + "\n", encoding="utf-8")
            cache_hit = False
            cache_misses += 1
        if rate != sample_rate:
            raise RuntimeError(f"unexpected Kokoro sample rate: {rate}")
        duration = len(samples) / sample_rate
        if window.start_s + duration > window.window_end_s:
            overlong.append(f"block {index}: {duration:.2f}s in {window.window_end_s - window.start_s:.2f}s")
        offset = round(window.start_s * sample_rate)
        master[offset:offset + len(samples)] += samples
        records.append({
            "index": index, "start_s": window.start_s,
            "end_s": round(window.start_s + duration, 3), "duration_s": round(duration, 3),
            "window_end_s": window.window_end_s, "speed": window.speed,
            "sentence_pause_s": window.sentence_pause_s,
            "clause_pause_s": window.clause_pause_s, "text": text,
            "cache_key": cache_key, "cache_hit": cache_hit,
        })
        print(f"[agent-build voice] {index:02d} {window.start_s:6.2f}-"
              f"{window.start_s + duration:6.2f}s "
              f"({'cache' if cache_hit else 'generated'})")
    if overlong:
        raise RuntimeError("shorten copy rather than forcing an unnatural pace — " + ", ".join(overlong))

    sf.write(str(output), master, sample_rate, subtype="PCM_24")
    timings.write_text(json.dumps({
        "engine": "kokoro-onnx", "model": model.name, "voice": spec.voice.voice,
        "local_generation": True, "master_duration_s": spec.duration_s,
        "build_key": build_key,
        "cache": {"hits": cache_hits, "misses": cache_misses, "master_hit": False},
        "performance_direction": "calm first-person expert; conversational emphasis; natural chapter pauses",
        "segments": records,
    }, indent=2) + "\n", encoding="utf-8")
    return output, timings


def generate_from_manifest(path: Path, runtime: Path | None = None) -> tuple[Path, Path]:
    return generate(parse(path), runtime)
