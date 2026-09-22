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
"""Blender-side replay baker. Run through `omnisim cinema render`.

Geometry, materials, lighting and cameras belong to the authored .blend file.
Only bound rigid-body transforms are replaced by recorded OmniSim motion.
"""
from __future__ import annotations

import bisect
import json
import math
import platform
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay import read_trace, write_json  # noqa: E402


def matrix(values):
    return Matrix([values[i:i + 4] for i in range(0, 16, 4)])


def pose_at(rows, times, t, name):
    if t < times[0] - 1e-8 or t > times[-1] + 1e-8:
        raise ValueError(f"Pose time {t} is outside {name}'s recording")
    i = min(len(times) - 2, max(0, bisect.bisect_right(times, t) - 1))
    a, b = matrix(rows[i]["poses"][name]), matrix(rows[i + 1]["poses"][name])
    u = (t - times[i]) / (times[i + 1] - times[i])
    return Matrix.LocRotScale(a.translation.lerp(b.translation, u),
                             a.to_quaternion().slerp(b.to_quaternion(), u),
                             Vector((1, 1, 1)))


def curves(action):
    # Blender 4.4 introduced slotted actions. Also accept legacy actions.
    if hasattr(action, "layers") and len(action.layers):
        for layer in action.layers:
            for strip in layer.strips:
                if hasattr(strip, "channelbags"):
                    for bag in strip.channelbags:
                        yield from bag.fcurves
    elif hasattr(action, "fcurves"):
        yield from action.fcurves


def main(job_path):
    job = json.loads(Path(job_path).read_text(encoding="utf-8"))
    data, shot, profile = job["data"], job["shot"], job["profile"]
    folder = Path(job["folder"])
    scene = bpy.context.scene
    if abs(scene.unit_settings.scale_length - 1) > 1e-8:
        raise ValueError("Scene scale_length must be 1: all coordinates are metres")
    if bpy.data.libraries:
        raise ValueError("Make linked libraries local and pack the scene before rendering")
    for img in bpy.data.images:
        if img.source in ("MOVIE", "SEQUENCE", "TILED"):
            raise ValueError(f"Bake unsupported external image source into the scene: {img.name}")
        if img.source == "FILE" and not img.packed_file:
            raise ValueError(f"Pack image into .blend so its content is hash-bound: {img.name}")
    for font in bpy.data.fonts:
        if font.filepath and font.filepath != "<builtin>" and not font.packed_file:
            raise ValueError(f"Pack font into .blend: {font.name}")
    camera = scene.objects.get(shot["camera"])
    if camera is None or camera.type != "CAMERA":
        raise ValueError(f"Camera {shot['camera']!r} is missing from the active scene")
    scene.camera = camera
    if scene.rigidbody_world:
        scene.rigidbody_world.enabled = False
    tracks = []
    for recording in data["recordings"]:
        rows = read_trace(Path(recording["path"]), recording["bindings"])
        times = [r["time"] for r in rows]
        for name, target in recording["bindings"].items():
            obj = scene.objects.get(target)
            if obj is None:
                raise ValueError(f"Missing Blender object {target!r} for pose {name!r}")
            if obj.parent or obj.constraints or obj.rigid_body or any(abs(v - 1) > 1e-4 for v in obj.scale):
                raise ValueError(f"{target}: bind an unparented, unit-scale Empty with no constraints/rigid body")
            if obj.animation_data and obj.animation_data.drivers:
                raise ValueError(f"{target}: drivers would override recorded motion")
            obj.animation_data_clear()
            obj.scale = (1, 1, 1)  # Remove only floating-point decomposition noise.
            obj.rotation_mode = "QUATERNION"
            tracks.append((obj, name, rows, times))
    scene.render.engine = "CYCLES"
    scene.cycles.samples = profile["samples"]
    scene.cycles.use_denoising = True
    requested = job["device"]
    devices = []
    if requested != "CPU":
        preferences = bpy.context.preferences.addons["cycles"].preferences
        preferences.compute_device_type = requested
        preferences.get_devices()
        for dev in preferences.devices:
            dev.use = dev.type == requested
            if dev.use:
                devices.append(dev.name)
        if not devices:
            raise ValueError(f"No {requested} device; choose an available device explicitly")
        scene.cycles.device = "GPU"
    else:
        scene.cycles.device = "CPU"
        devices = [platform.processor() or "CPU"]
    scene.cycles.denoiser = "OPTIX" if requested == "OPTIX" else "OPENIMAGEDENOISE"
    scene.cycles.max_bounces = 5
    scene.cycles.diffuse_bounces = 3
    scene.cycles.glossy_bounces = 3
    scene.cycles.transmission_bounces = 4
    scene.render.resolution_x, scene.render.resolution_y = data["resolution"]
    scene.render.resolution_percentage = profile["scale"]
    scene.render.pixel_aspect_x = scene.render.pixel_aspect_y = 1
    scene.render.use_border = False
    scene.render.use_compositing = False  # Beauty pass; grading/overlays happen downstream.
    scene.render.use_sequencer = False
    scene.render.fps, scene.render.fps_base = data["fps"], 1
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.image_settings.color_depth = "8"
    scene.render.use_file_extension = True
    scene.render.use_motion_blur = True
    scene.render.motion_blur_shutter = 0.35
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.55
    scene.frame_start, scene.frame_end = 1, shot["frames"]
    scene.timeline_markers.clear()  # Markers must not silently switch cameras.
    previous = {}
    for frame in range(1, shot["frames"] + 1):
        t = shot["source_start_s"] + (frame - 1) * shot["speed"] / data["fps"]
        for obj, name, rows, times in tracks:
            pose = pose_at(rows, times, t, name)
            rotation = pose.to_quaternion()
            if obj.name in previous and previous[obj.name].dot(rotation) < 0:
                rotation.negate()
            previous[obj.name] = rotation.copy()
            obj.location, obj.rotation_quaternion = pose.translation, rotation
            obj.keyframe_insert(data_path="location", frame=frame)
            obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)
    for obj, *_ in tracks:
        for curve in curves(obj.animation_data.action):
            for key in curve.keyframe_points:
                key.interpolation = "LINEAR"
    max_position, max_angle, checks = 0.0, 0.0, 0
    for frame in range(1, shot["frames"] + 1):
        scene.frame_set(frame)
        bpy.context.view_layer.update()
        t = shot["source_start_s"] + (frame - 1) * shot["speed"] / data["fps"]
        for obj, name, rows, times in tracks:
            expected = pose_at(rows, times, t, name)
            got = obj.matrix_world
            pos = (expected.translation - got.translation).length
            angle = expected.to_quaternion().rotation_difference(got.to_quaternion()).angle
            angle = min(abs(angle), abs(2 * math.pi - angle))
            max_position, max_angle = max(max_position, pos), max(max_angle, angle)
            checks += 1
    if max_position > 1e-5 or max_angle > 1e-3:
        raise ValueError(f"Baked motion drift: {max_position} m / {max_angle} rad")
    write_json(folder / "bake_report.json", {
        "checks": checks, "max_position_error_m": max_position,
        "max_angle_error_rad": max_angle, "blender": bpy.app.version_string,
        "machine": platform.node(), "devices": devices,
        "samples": scene.cycles.samples, "denoiser": scene.cycles.denoiser,
        "source_start_s": shot["source_start_s"], "speed": shot["speed"],
        "note": "Output-frame poses checked; shutter subframes interpolate baked animation",
    })
    scene.frame_set(1)
    scene.render.filepath = str(folder / "frame_")
    bpy.ops.file.pack_all()
    bpy.ops.wm.save_as_mainfile(filepath=str(folder / "replay.blend"))
    # Explicit filenames avoid Blender's default four-digit frame numbering.
    for frame in range(1, shot["frames"] + 1):
        scene.frame_set(frame)
        scene.render.filepath = str(folder / f"frame_{frame:06d}.png")
        bpy.ops.render.render(write_still=True)
    print(f"REPLAY_DONE {shot['id']}", flush=True)


if __name__ == "__main__":
    main(sys.argv[sys.argv.index("--") + 1])
