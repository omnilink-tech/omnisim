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

"""Persistent, resumable capture sessions for Agent Build films.

One capture service and one world load serve an entire shot list.  Completed
shots are hash-bound in a receipt and skipped on resume.  Named engine scene
checkpoints make alternate takes cheap while explicitly disclosing that an
arbitrary controller's private Python memory is outside the snapshot contract.
"""

from __future__ import annotations

import hashlib
import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stable_hash(payload: Any) -> str:
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")).hexdigest()


def _request(service: str, path: str, payload: dict[str, Any] | None = None,
             timeout_s: float = 300.0) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        service.rstrip("/") + path, data=data,
        headers={"Content-Type": "application/json"} if data is not None else {},
        method="POST" if data is not None else "GET",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"capture service {path} returned {exc.code}: {body}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(
            f"capture service is unreachable at {service}; start it once with "
            "`python -m omnisim capture`"
        ) from exc


def template(world: str = "projects/samples/demos/worlds/flagship/build.omniworld") -> dict[str, Any]:
    return {
        "version": 1,
        "world": world,
        "load": {"wait_s": 75, "width": 1920, "height": 1080, "fov": 0.785398},
        "settle_steps": 30,
        "checkpoints": [
            {"name": "capture_start", "from": "capture_start", "after_steps": 0}
        ],
        "shots": [{
            "id": "build_question", "output": "captures/question.mp4",
            "restore": "capture_start", "pre_steps": 0,
            "sequence": {
                "duration_s": 8.0, "fps": 30, "codec": "h264", "crf": 12,
                "preset": "slow", "keep_frames": False,
                "path_keyframes": [
                    {"t": 0.0, "position": [4.0, -4.0, 2.0], "target": [0.0, 0.0, 0.5]},
                    {"t": 1.0, "position": [4.0, -4.0, 2.0], "target": [0.0, 0.0, 0.5]},
                ],
            },
        }],
    }


def _validate(data: dict[str, Any]) -> None:
    if int(data.get("version", 0)) != 1:
        raise ValueError("capture plan version must be 1")
    if not str(data.get("world", "")).strip():
        raise ValueError("capture plan needs a world")
    shots = data.get("shots") or []
    if not shots:
        raise ValueError("capture plan has no shots")
    ids: list[str] = []
    for index, shot in enumerate(shots, 1):
        shot_id = str(shot.get("id", "")).strip()
        output = str(shot.get("output", "")).strip()
        sequence = shot.get("sequence") or {}
        if not shot_id or not output:
            raise ValueError(f"capture shot #{index} needs id and output")
        if not all(key in sequence for key in ("path_keyframes", "duration_s", "fps")):
            raise ValueError(
                f"capture shot {shot_id!r} needs path_keyframes, duration_s, and fps"
            )
        if float(sequence["duration_s"]) <= 0 or int(sequence["fps"]) <= 0:
            raise ValueError(f"capture shot {shot_id!r} duration/fps must be positive")
        ids.append(shot_id)
    if len(ids) != len(set(ids)):
        raise ValueError("capture shot ids must be unique")


def run(plan_path: Path, *, service: str = "http://127.0.0.1:6791",
        receipt_path: Path | None = None) -> dict[str, Any]:
    """Run or resume one content-addressed capture plan."""
    started = time.perf_counter()
    plan_path = plan_path.resolve()
    data = json.loads(plan_path.read_text(encoding="utf-8"))
    _validate(data)
    world = Path(str(data["world"]))
    if not world.is_absolute():
        local = (plan_path.parent / world).resolve()
        world = local if local.exists() else (Path.cwd() / world).resolve()
    if not world.exists():
        raise FileNotFoundError(world)

    receipt_path = (receipt_path or plan_path.parent / "build"
                    / f"{plan_path.stem}_capture_session.json").resolve()
    try:
        previous = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        previous = {}
    previous_shots = {item.get("id"): item for item in previous.get("shots", [])}
    plan_world_sha = _sha256(world)

    # A fully cached resume does not need to touch the service or reload the
    # world.  This is the common editing-loop path after captures are approved.
    cached_records: list[dict[str, Any]] = []
    for shot in data["shots"]:
        output = Path(str(shot["output"]))
        if not output.is_absolute():
            output = (plan_path.parent / output).resolve()
        request_payload = dict(shot["sequence"])
        request_payload["output"] = str(output)
        key = _stable_hash({
            "world_sha256": plan_world_sha,
            "restore": str(shot.get("restore", "capture_start")),
            "pre_steps": int(shot.get("pre_steps", 0)),
            "sequence": request_payload,
        })
        old = previous_shots.get(str(shot["id"])) or {}
        if not (old.get("key") == key and output.exists() and output.stat().st_size > 0
                and old.get("sha256") == _sha256(output)):
            cached_records = []
            break
        record = dict(old)
        record["cache_hit"] = True
        cached_records.append(record)
    if len(cached_records) == len(data["shots"]):
        report = dict(previous)
        report.update({
            "complete": True, "service_contacted": False,
            "shots": cached_records,
            "cache": {"hits": len(cached_records), "misses": 0},
            "elapsed_s": round(time.perf_counter() - started, 3),
        })
        receipt_path.parent.mkdir(parents=True, exist_ok=True)
        receipt_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return report

    health = _request(service, "/healthz", timeout_s=5.0)
    load = dict(data.get("load") or {})
    load["path"] = str(world)
    result = _request(service, "/world/load", load,
                      timeout_s=float(load.get("wait_s", 75)) + 45.0)
    if not result.get("ok"):
        raise RuntimeError(f"capture world load failed: {result}")
    settle_steps = int(data.get("settle_steps", 0))
    if settle_steps:
        _request(service, "/sim/step", {"steps": settle_steps}, timeout_s=60.0)
    _request(service, "/sim/snapshot", {"name": "capture_start"}, timeout_s=30.0)

    checkpoints: list[dict[str, Any]] = [{"name": "capture_start", "after_steps": 0}]
    for checkpoint in data.get("checkpoints") or []:
        name = str(checkpoint.get("name", "")).strip()
        if not name or name == "capture_start":
            continue
        source = str(checkpoint.get("from", "capture_start"))
        _request(service, "/sim/restore", {"name": source}, timeout_s=30.0)
        steps = int(checkpoint.get("after_steps", 0))
        if steps:
            _request(service, "/sim/step", {"steps": steps}, timeout_s=120.0)
        checkpoint_result = _request(
            service, "/sim/snapshot", {"name": name}, timeout_s=30.0,
        )
        checkpoints.append({"name": name, "from": source,
                            "after_steps": steps, "result": checkpoint_result})

    shot_records: list[dict[str, Any]] = []
    hits = 0
    misses = 0
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    for shot in data["shots"]:
        shot_id = str(shot["id"])
        output = Path(str(shot["output"]))
        if not output.is_absolute():
            output = (plan_path.parent / output).resolve()
        request_payload = dict(shot["sequence"])
        request_payload["output"] = str(output)
        restore = str(shot.get("restore", "capture_start"))
        pre_steps = int(shot.get("pre_steps", 0))
        key = _stable_hash({
            "world_sha256": plan_world_sha, "restore": restore,
            "pre_steps": pre_steps, "sequence": request_payload,
        })
        old = previous_shots.get(shot_id) or {}
        reusable = (old.get("key") == key and output.exists()
                    and output.stat().st_size > 0
                    and old.get("sha256") == _sha256(output))
        if reusable:
            hits += 1
            record = dict(old)
            record["cache_hit"] = True
        else:
            misses += 1
            _request(service, "/sim/restore", {"name": restore}, timeout_s=30.0)
            if pre_steps:
                _request(service, "/sim/step", {"steps": pre_steps}, timeout_s=120.0)
            output.parent.mkdir(parents=True, exist_ok=True)
            response = _request(
                service, "/capture/sequence", request_payload,
                timeout_s=float(shot.get("timeout_s", 1800)),
            )
            if not output.exists() or output.stat().st_size <= 0:
                raise RuntimeError(f"capture shot {shot_id!r} produced no file at {output}")
            record = {
                "id": shot_id, "key": key, "output": str(output),
                "sha256": _sha256(output), "bytes": output.stat().st_size,
                "cache_hit": False, "restore": restore, "pre_steps": pre_steps,
                "response": response,
            }
        shot_records.append(record)
        # Persist after every completed shot: an interrupted session resumes at
        # the next shot instead of throwing away expensive simulator footage.
        receipt_path.write_text(json.dumps({
            "version": 1, "plan": str(plan_path), "plan_sha256": _sha256(plan_path),
            "world": str(world), "world_sha256": plan_world_sha,
            "service": service, "service_contacted": True,
            "health": health, "load": result,
            "checkpoint_boundary": (
                "engine scene state only; controller process memory and the simulation clock "
                "are not rewound"
            ),
            "checkpoints": checkpoints, "shots": shot_records,
            "cache": {"hits": hits, "misses": misses},
        }, indent=2) + "\n", encoding="utf-8")

    report = json.loads(receipt_path.read_text(encoding="utf-8"))
    report["elapsed_s"] = round(time.perf_counter() - started, 3)
    report["complete"] = len(shot_records) == len(data["shots"])
    receipt_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
