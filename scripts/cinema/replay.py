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
"""Portable, evidence-bound cinematic replay jobs. Blender is a subprocess.

The manifest selects recorded motion and an authored, packed .blend scene.
It never launches a simulation or changes its controllers. Relative input and
output paths resolve beside the manifest, independent of the caller's cwd.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import subprocess
from pathlib import Path

SCHEMA = "omnisim_cinematic_replay_v1"
RENDERER = "blender_cycles"
DISCLOSURE = "OMNISIM MOTION / CINEMATIC REPLAY"
ROOT = Path(__file__).resolve().parents[2]
PROFILES = {"proxy": {"scale": 50, "samples": 8},
            "final": {"scale": 100, "samples": 32}}


def template(title="New Cinema Piece", world="world.omniworld", subject="robot"):
    return {
        "schema": SCHEMA, "renderer": RENDERER, "title": title,
        "coordinate_system": "right_handed_z_up_meters",
        "scene": "scene.blend", "output": "renders",
        "fps": 24, "resolution": [1920, 1080],
        "scenario": {
            "objective": "TODO: state the robot's concrete task",
            "challenge": "TODO: state the obstacle or interaction worth watching",
            "outcome": "TODO: describe the measured result, including failure",
            "limitations": ["TODO: record support conditions and untested claims"],
        },
        "runs": [{"id": "take_01", "world": world,
                  "controllers": ["controller.py"],
                  "newton_sidecar": "run.log.newton.json",
                  "evidence": ["result.json"]}],
        "recordings": [{"path": "motion.jsonl", "run": "take_01",
                        "bindings": {"body": f"{subject}/body"}}],
        "shots": [{"id": "action", "camera": "Camera",
                   "source_start_s": 0.0, "duration_s": 6.0, "speed": 1.0,
                   "purpose": "Show the task, obstacle, and resulting action together"}],
    }


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def number(value, label, minimum=0, maximum=math.inf):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{label} must be numeric")
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be finite and between {minimum} and {maximum}")
    return value


def _text(value, label):
    if not isinstance(value, str) or not value.strip() or "TODO" in value:
        raise ValueError(f"Fill in {label} before rendering")
    return value


def read_trace(path: Path, names) -> list[dict]:
    """Reject holes, resets, non-rigid poses, and invalid JSON before GPU work."""
    rows, last = [], -math.inf
    with path.open(encoding="utf-8-sig") as stream:
        for line, text in enumerate(stream, 1):
            if not text.strip():
                continue
            row = json.loads(text)
            t = number(row.get("time"), f"{path.name}:{line} time")
            if t <= last:
                raise ValueError(f"{path.name}:{line}: timestamps must strictly increase")
            poses = row.get("poses", {})
            for name in names:
                p = poses.get(name)
                if not isinstance(p, list) or len(p) != 16:
                    raise ValueError(f"{path.name}:{line}: missing/invalid pose {name!r}")
                for v in p:
                    number(v, "matrix element", -math.inf)
                if max(abs(p[12 + j] - v) for j, v in enumerate((0, 0, 0, 1))) > 1e-5:
                    raise ValueError("Poses must be row-major affine 4x4 matrices")
                r = [p[i:i + 3] for i in (0, 4, 8)]
                error = max(abs(sum(r[i][k] * r[j][k] for k in range(3)) - (i == j))
                            for i in range(3) for j in range(3))
                det = (r[0][0] * (r[1][1]*r[2][2] - r[1][2]*r[2][1])
                       - r[0][1] * (r[1][0]*r[2][2] - r[1][2]*r[2][0])
                       + r[0][2] * (r[1][0]*r[2][1] - r[1][1]*r[2][0]))
                if error > 1e-3 or abs(det - 1) > 1e-3:
                    raise ValueError(f"Pose {name!r} must be rigid (no scale/reflection)")
            rows.append({"time": t, "poses": {n: poses[n] for n in names}})
            last = t
    if len(rows) < 2:
        raise ValueError(f"{path}: need at least two samples")
    return rows


def validate(manifest: Path) -> dict:
    manifest = manifest.resolve()
    data = json.loads(manifest.read_text(encoding="utf-8-sig"))
    if data.get("schema") != SCHEMA or data.get("renderer") != RENDERER:
        raise ValueError(f"Expected schema {SCHEMA!r} and renderer {RENDERER!r}")
    if data.get("coordinate_system") != "right_handed_z_up_meters":
        raise ValueError("Convert source coordinates to right-handed Z-up metres first")
    _text(data.get("title"), "title")
    scenario = data.get("scenario", {})
    for field in ("objective", "challenge", "outcome"):
        _text(scenario.get(field), f"scenario.{field}")
    if not isinstance(scenario.get("limitations"), list):
        raise ValueError("scenario.limitations must be a list (empty is allowed)")
    for item in scenario["limitations"]:
        _text(item, "scenario.limitations")
    fps = number(data.get("fps"), "fps", 1, 120)
    if int(fps) != fps:
        raise ValueError("fps must be an integer")
    data["fps"] = int(fps)
    resolution = data.get("resolution")
    if not isinstance(resolution, list) or len(resolution) != 2:
        raise ValueError("resolution must be [width, height]")
    for n in resolution:
        number(n, "resolution", 16, 8192)
        if int(n) != n or n % 4:
            raise ValueError("Resolution dimensions must be multiples of four")
    data["resolution"] = [int(n) for n in resolution]
    inputs = {}

    def source(value):
        _text(value, "input path")
        path = (manifest.parent / value).resolve()
        if not path.is_file():
            raise ValueError(f"Missing input: {path}")
        inputs[str(path)] = digest(path)
        return str(path)

    data["scene"] = source(data.get("scene"))
    if Path(data["scene"]).suffix.lower() != ".blend":
        raise ValueError("scene must be a packed .blend file")
    runs = set()
    for run in data.get("runs", []):
        name = _text(run.get("id"), "run.id")
        if name in runs:
            raise ValueError(f"Duplicate run {name}")
        runs.add(name)
        run["world"] = source(run.get("world"))
        if not run.get("controllers"):
            raise ValueError("Each run needs its controller source files")
        run["controllers"] = [source(p) for p in run["controllers"]]
        run["newton_sidecar"] = source(run.get("newton_sidecar"))
        proof = json.loads(Path(run["newton_sidecar"]).read_text(encoding="utf-8-sig"))
        if proof.get("backend") != "newton" or proof.get("finalised") is not True or proof.get("degraded") is not False:
            raise ValueError(f"Run {name}: Newton did not finalise in a non-degraded state")
        if not run.get("evidence"):
            raise ValueError("Each run needs result/log evidence; failures are valid evidence")
        run["evidence"] = [source(p) for p in run["evidence"]]
    recordings, objects, ranges = data.get("recordings", []), set(), []
    if not recordings:
        raise ValueError("At least one recording is required")
    for recording in recordings:
        if recording.get("run") not in runs:
            raise ValueError("Every recording must reference a declared run")
        recording["path"] = source(recording.get("path"))
        bindings = recording.get("bindings")
        if not isinstance(bindings, dict) or not bindings:
            raise ValueError("Each recording needs pose-name to Blender-object bindings")
        for pose, obj in bindings.items():
            _text(pose, "pose name")
            _text(obj, "Blender object name")
            if obj in objects:
                raise ValueError(f"Object {obj!r} is driven by multiple recordings/poses")
            objects.add(obj)
        rows = read_trace(Path(recording["path"]), bindings)
        ranges.append((rows[0]["time"], rows[-1]["time"]))
    shots, ids = data.get("shots", []), set()
    if not shots:
        raise ValueError("At least one shot is required")
    for shot in shots:
        name = _text(shot.get("id"), "shot.id")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name) or name in ids:
            raise ValueError("Shot ids must be unique letters/digits/underscore/hyphen")
        ids.add(name)
        _text(shot.get("purpose"), "shot.purpose")
        _text(shot.get("camera"), "shot.camera")
        start = number(shot.get("source_start_s"), "source_start_s")
        duration = number(shot.get("duration_s"), "duration_s", 1 / fps)
        speed = number(shot.get("speed", 1), "speed", 0.001, 100)
        frames = round(duration * fps)
        if abs(duration * fps - frames) > 1e-6:
            raise ValueError("Shot duration must be an exact number of output frames")
        shot["speed"], shot["frames"] = speed, frames
        end = start + (frames - 1) * speed / fps
        if any(start < lo - 1e-8 or end > hi + 1e-8 for lo, hi in ranges):
            raise ValueError(f"Shot {name}: source range exceeds a recording; no pose clamping")
    data["output"] = str((manifest.parent / data.get("output", "renders")).resolve())
    for name in ("replay.py", "blender_replay.py"):
        source(str(Path(__file__).with_name(name)))
    source(str(ROOT / "resources/branding/omnisim/fonts/SpaceGrotesk.ttf"))
    data["input_hashes"] = inputs
    data["input_key"] = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
    return data


def find_blender(explicit=None):
    if explicit:
        path = shutil.which(str(explicit))
        if path:
            return path
        raise ValueError(f"Blender executable not found: {explicit}")
    path = shutil.which("blender")
    if path:
        return path
    candidates = sorted(Path("C:/Program Files/Blender Foundation").glob("Blender */blender.exe"))
    if candidates:
        return str(candidates[-1])
    raise ValueError("Blender is required. Install it locally or pass --blender PATH; no native fallback")


def job_directory(data, profile):
    return Path(data["output"]) / data["input_key"][:16] / profile


def _receipt(data, profile):
    path = job_directory(data, profile) / "receipt.json"
    if not path.is_file():
        raise ValueError(f"Render {profile} first: {path}")
    receipt = json.loads(path.read_text(encoding="utf-8"))
    if receipt.get("input_key") != data["input_key"] or receipt.get("profile") != profile:
        raise ValueError("Render receipt does not match current inputs/profile")
    expected = {s["id"] for s in data["shots"]}
    if {s["id"] for s in receipt.get("shots", [])} != expected:
        raise ValueError("Render receipt does not contain every shot")
    for shot in receipt["shots"]:
        for key in ("video", "bake_report"):
            file = Path(shot[key])
            if not file.is_file() or digest(file) != shot[key + "_sha256"]:
                raise ValueError(f"Rendered artifact changed: {file}")
    return receipt


def record_review(manifest: Path, notes: str):
    data = validate(manifest)
    _text(notes, "review notes")
    proxy = _receipt(data, "proxy")
    review = {"input_key": data["input_key"], "notes": notes,
              "proxy_receipt_sha256": digest(job_directory(data, "proxy") / "receipt.json"),
              "shots": [s["id"] for s in proxy["shots"]]}
    path = job_directory(data, "proxy").parent / "review.json"
    write_json(path, review)
    return {"review": str(path), **review}


def render(manifest: Path, *, profile="proxy", blender=None, device="CPU"):
    data = validate(manifest)
    if profile not in PROFILES or device not in ("CPU", "OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
        raise ValueError("Unknown rendering profile or device")
    if profile == "final":
        _receipt(data, "proxy")
        review_path = job_directory(data, "proxy").parent / "review.json"
        if not review_path.is_file():
            raise ValueError("Inspect the proxy, then run cinema replay-review with your notes before final rendering")
        review = json.loads(review_path.read_text(encoding="utf-8"))
        if (review.get("input_key") != data["input_key"] or
                review.get("proxy_receipt_sha256") != digest(job_directory(data, "proxy") / "receipt.json")):
            raise ValueError("Proxy review is stale; inspect the current proxy again")
        _text(review.get("notes"), "review notes")
    executable = find_blender(blender)
    from scripts.capture.encode import find_ffmpeg
    ffmpeg = find_ffmpeg()
    if not ffmpeg:
        raise ValueError("ffmpeg is required for replay video encoding")
    # Only complete, hash-verified jobs are reused. Interrupted shots rerender.
    try:
        return _receipt(data, profile)
    except (ValueError, KeyError, OSError):
        pass
    out = job_directory(data, profile)
    out.mkdir(parents=True, exist_ok=True)
    version = subprocess.run([executable, "--version"], capture_output=True,
                             text=True, check=True).stdout.splitlines()[0]
    write_json(out / "resolved.json", data)
    shutil.copyfile(ROOT / "resources/branding/omnisim/fonts/SpaceGrotesk.ttf", out / "brand.ttf")
    shots = []
    for shot in data["shots"]:
        folder = out / shot["id"]
        folder.mkdir(exist_ok=True)
        job = {"data": data, "shot": shot, "profile": PROFILES[profile],
               "device": device, "folder": str(folder)}
        write_json(folder / "job.json", job)
        print(f"[replay] {profile}: {shot['id']} ({shot['frames']} frames)", flush=True)
        try:
            with (folder / "blender.log").open("w", encoding="utf-8") as log:
                subprocess.run([executable, "--background", "--disable-autoexec", data["scene"],
                                "--python-exit-code", "1", "--python",
                                str(Path(__file__).with_name("blender_replay.py")),
                                "--", str(folder / "job.json")], stdout=log,
                               stderr=subprocess.STDOUT, check=True)
        except subprocess.CalledProcessError as exc:
            raise ValueError(f"Blender failed for {shot['id']}; see {folder / 'blender.log'}") from exc
        frames = [folder / f"frame_{i:06d}.png" for i in range(1, shot["frames"] + 1)]
        if not all(p.is_file() and p.stat().st_size > 0 for p in frames):
            raise ValueError(f"Incomplete frame sequence: {folder}")
        video = folder / f"{shot['id']}.mp4"
        subprocess.run([ffmpeg, "-v", "error", "-y", "-framerate", str(data["fps"]),
                        "-start_number", "1", "-i", str(folder / "frame_%06d.png"),
                        "-frames:v", str(shot["frames"]), "-vf",
                        f"drawtext=fontfile=brand.ttf:text='{DISCLOSURE}':fontsize=h/42:"
                        "fontcolor=white:x=w*0.035:y=h*0.94:box=1:boxcolor=black@0.6:boxborderw=5",
                        "-c:v", "libx264", "-crf", "16", "-preset", "slow",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)],
                       cwd=out, check=True)
        subprocess.run([ffmpeg, "-v", "error", "-xerror", "-i", str(video),
                        "-f", "null", "-"], check=True)
        report = folder / "bake_report.json"
        shots.append({"id": shot["id"], "video": str(video), "video_sha256": digest(video),
                      "bake_report": str(report), "bake_report_sha256": digest(report),
                      "frames": shot["frames"], "source_start_s": shot["source_start_s"],
                      "speed": shot["speed"]})
    # Easy local review without paying for another full render.
    from PIL import Image, ImageDraw
    sheet = Image.new("RGB", (960, 204 * len(shots)), "#0A0A06")
    draw = ImageDraw.Draw(sheet)
    for row, shot in enumerate(data["shots"]):
        for column, frame in enumerate((1, (shot["frames"] + 1) // 2, shot["frames"])):
            with Image.open(out / shot["id"] / f"frame_{frame:06d}.png") as img:
                img.thumbnail((320, 180))
                sheet.paste(img, (column * 320, row * 204))
        draw.text((8, row * 204 + 184), shot["id"], fill="white")
    sheet.save(out / "contact_sheet.jpg")
    receipt = {"schema": SCHEMA, "renderer": RENDERER, "profile": profile,
               "input_key": data["input_key"], "input_hashes": data["input_hashes"],
               "blender": version, "requested_device": device, "shots": shots,
               "disclosure": DISCLOSURE, "contact_sheet": str(out / "contact_sheet.jpg"),
               "scenario": data["scenario"]}
    write_json(out / "receipt.json", receipt)
    return receipt
