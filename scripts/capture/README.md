# scripts/capture/ — agent-facing capture service

Long-running HTTP service on `127.0.0.1:6791` (sister to the validation harness on `:6789`) that wraps an OmniSim subprocess for cinematic output: high-resolution stills, real-time movie recording, and **deterministic offline frame-sequence renders that get encoded to mp4 / prores via ffmpeg**.

The service injects a supervisor controller and, by default, renders through `Supervisor.exportImage`, so output is **viewport-bound** (typically ~1896x1184). An embedded-`Camera`-device path that would size renders to an arbitrary output resolution exists but is **experimental and disabled by default** (`cam.enable()` segfaults OmniSim in `--batch` render mode; gated behind `OMNISIM_CAPTURE_ENABLE_CAMERA=1`). Shot lists drive scripted multi-shot runs end-to-end.

| File | Purpose |
|---|---|
| [`omnisim_capture.py`](omnisim_capture.py) | The HTTP service. Run directly or via `python -m omnisim capture` (wired in). |
| [`camera_path.py`](camera_path.py) | Pure-math camera-path interpolation: Catmull-Rom for positions, slerp for orientations, six easing curves. No OmniSim dependency. |
| [`encode.py`](encode.py) | ffmpeg wrappers. Presets: `h264` (web-friendly, CRF 16 default), `h265`, `prores` (422 HQ master), `vp9`. |
| [`render.py`](render.py) | Shot-list CLI. Reads JSON/YAML, drives the service, optionally starts an ad-hoc service for one-shot runs. |
| [`shotlists/orbit_warehouse.json`](shotlists/orbit_warehouse.json) | Example shot list. |

The supervisor controller injected at world load is [`projects/default/controllers/capture_supervisor/`](../../projects/default/controllers/capture_supervisor/). Default outputs land in `social/youtube_videos/captures/` (gitignored).

## Reality check on render quality

OmniSim renders with OpenGL (WREN), not a path-traced engine like Blender Cycles, so this service does not produce raytraced GI / caustics / spectral lighting. What it does give you, though, is meaningfully better than ad-hoc screenshots:

- **Deterministic, viewport-bound output** (typically ~1896x1184). The `width`/`height` on `/world/load` are advisory in the shipped default — renders go through `Supervisor.exportImage` and are bound to the host viewport, so true 4K/8K via an embedded Camera is *not* currently delivered (that path is experimental, behind `OMNISIM_CAPTURE_ENABLE_CAMERA`; see the camera-segfault note below).
- **Deterministic frame-by-frame stepping**, so your final video is jitter-free even if the sim runs slowly.
- **Lossless PNG intermediate** plus **CRF-controlled ffmpeg encode** for visually-lossless h264 or a ProRes 422 HQ master.
- **Smooth camera moves** via Catmull-Rom spline + slerp, not the linear/jerky interpolation you'd get from a hand-rolled lerp.

If you ever need true raytraced output, the recommended path is: render this service's PNG sequence, then drop it into a Blender compositor scene as the source — same camera path, full GI on top.

### Quality knobs at a glance

Three knobs determine final video quality. Tune them together — there's no point at CRF 14 if your resolution is 640x360.

| Setting | Cheap / quick | Default (good) | High quality | Master / archival |
|---|---|---|---|---|
| `resolution` | `[1280, 720]` | `[1920, 1080]` | `[3840, 2160]` (4K) | `[3840, 2160]` |
| `codec` | `h264` | `h264` | `h264` | `prores` (ignores CRF) |
| `crf` | `22` | `16` (default) | `12-14` | `lossless: true` |
| `preset` | `medium` | `slow` (default) | `slow` | `veryslow` |
| `fps` | `30` | `60` | `60` | `60` (or sim native) |

A 1080p60 / 3-second / CRF 14 / `slow`-preset h264 clip lands around **3.5 MB** at ~10 Mbps; raise to CRF 10 for ~7 MB at ~20 Mbps. ProRes 422 HQ at the same resolution is ~30 MB/sec — use it as an edit master, not the deliverable.

A subtle gotcha: h264 is *very* good at compressing static scenes. If your scene is mostly motionless, a 2-second clip can compress smaller than a single PNG of one of its frames — this is normal, not a quality bug. To make CRF differences visible, render scenes with motion (camera moves, dynamics) and compare at full resolution.

## Starting it

```bash
PATH="/path/to/msys64/mingw64/bin:$PATH" \
OMNISIM_HOME=$(pwd) \
python scripts/capture/omnisim_capture.py --port 6791
```

Same Windows gotchas as the harness: `PATH` must contain a full msys2 mingw64 `bin`, and `ffmpeg` should be on `PATH` (or installed at `C:\ffmpeg\bin\ffmpeg.exe`, which `encode.py` falls back to). Without ffmpeg, screenshots and movies still work, but `/capture/sequence` fails at the encode step.

## Quickstart: a single high-res still

```bash
# 1. Load a world at 4K.
curl -s -X POST http://127.0.0.1:6791/world/load \
  -H "Content-Type: application/json" \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.wbt",
       "width":3840,"height":2160}'

# 2. Aim the camera.
curl -s -X POST http://127.0.0.1:6791/capture/camera \
  -H "Content-Type: application/json" \
  -d '{"position":[-12,-12,6],"target":[0,0,1]}'

# 3. Render a still. Returned as image/png (or pass {"path":"..."} to write server-side).
curl -s -X POST http://127.0.0.1:6791/capture/screenshot \
  -H "Content-Type: application/json" \
  -d '{}' -o still.png
```

## Cinematic offline render: shot-list-driven

```bash
python scripts/capture/render.py scripts/capture/shotlists/orbit_warehouse.json
```

Or for one-shot runs (the script starts a dedicated service, renders, exits):

```bash
python scripts/capture/render.py shotlist.yaml --ad-hoc
```

## Endpoint cheatsheet

| Endpoint | Purpose |
|---|---|
| `POST /world/load {path, wait_s?, width?, height?, fov?}` | Load a `.wbt` and inject the capture supervisor with a Camera device at the requested resolution. Resolution is baked into the sibling `.wbt`, so changing it re-launches OmniSim (no hot reload). |
| `POST /capture/camera {position, target?, orientation?, sync_viewpoint?}` | Move the supervisor robot (and therefore the camera). Pass `target` to look at a point or `orientation` to set axis-angle directly. `sync_viewpoint=true` (default) mirrors the move to the GUI Viewpoint. |
| `POST /capture/screenshot {path?, quality?, source?}` | Render a still. `source` defaults to `"viewport"` in the shipped default (the embedded Camera is disabled — see above), which uses `Supervisor.exportImage` → viewport-bound. `source: "camera"` uses the embedded Camera device → arbitrary resolution, but only when the experimental Camera path is enabled (`OMNISIM_CAPTURE_ENABLE_CAMERA=1`); otherwise it errors. |
| `POST /capture/movie/start {path, width?, height?, codec?, quality?, fps?}` | Start OmniSim's native (real-time) movie recording. Output is viewport-bound. |
| `POST /capture/movie/stop` | Stop recording; reports ready/failed state. |
| `GET  /capture/movie/status` | `{active, path, ready, failed}`. |
| `POST /capture/sequence {path_keyframes, duration_s, fps, output, codec?, crf?, ease?, warmup_steps?, settle_steps_per_frame?, keep_frames?}` | The cinematic offline path: walk the camera path frame-by-frame, dump PNGs from the Camera device, ffmpeg-encode. |
| `POST /sim/step {steps?}` | Advance N basic timesteps. |
| `POST /sim/reset` | Reset the simulation to t=0. |
| `GET  /sim/state` | Current world, camera resolution, supervisor connection. |
| `GET  /healthz` | Liveness + ffmpeg-resolution status. |

## Shot list format

JSON or YAML. See [shotlists/orbit_warehouse.json](shotlists/orbit_warehouse.json) for a worked example.

```json
{
  "world": "projects/samples/demos/worlds/flagship/warehouse_industrial.wbt",
  "resolution": [3840, 2160],
  "fov": 0.785398,
  "load_wait_s": 12.0,
  "shots": [
    {
      "name": "orbit",
      "output": "warehouse_orbit.mp4",
      "duration_s": 8.0,
      "fps": 60,
      "codec": "h264",
      "crf": 16,
      "ease": "smoothstep",
      "warmup_steps": 60,
      "settle_steps_per_frame": 1,
      "path_keyframes": [
        {"t": 0.0, "position": [-12, -12, 6], "target": [0, 0, 1]},
        {"t": 4.0, "position": [12, -12, 6],  "target": [0, 0, 1]},
        {"t": 8.0, "position": [12,  12, 6],  "target": [0, 0, 1]}
      ]
    }
  ]
}
```

### Field reference

- `world` (str, required): path to a `.wbt`, repo-relative or absolute.
- `resolution` ([w, h]): camera resolution. Default `[1920, 1080]`.
- `fov` (float, radians): camera horizontal field of view. Default `0.785398` (45°).
- `load_wait_s` (float): seconds to wait for the supervisor to bind. Default `12.0`.
- `shots[*].output` (str): output filename or absolute path. Relative paths resolve under `social/youtube_videos/captures/`.
- `shots[*].duration_s` (float): clip length.
- `shots[*].fps` (int): output framerate. Pair with the world's `WorldInfo.basicTimeStep` and `settle_steps_per_frame` to control sim speed vs. wall-clock motion.
- `shots[*].codec` (str): one of `h264`, `h265`, `prores`, `vp9`. Default `h264`.
- `shots[*].crf` (int): quality knob for h264/h265/vp9; lower = better. Default `16` (visually lossless on most content). Ignored for ProRes (uses fixed-quality 422 HQ profile) and when `lossless` is true.
- `shots[*].preset` (str): encoder speed/quality tradeoff for h264/h265: `ultrafast`, `superfast`, `veryfast`, `faster`, `fast`, `medium`, `slow` (default), `slower`, `veryslow`. `veryslow` buys ~5-10% smaller files than `slow` at significantly higher encode cost.
- `shots[*].lossless` (bool): mathematically lossless mode. h264: `-qp 0`. h265: `-x265-params lossless=1`. vp9: `-lossless 1`. ProRes is already a master format and ignores this. Files become large fast — keep clips short.
- `shots[*].ease` (str): `linear`, `smoothstep` (default), `smootherstep`, `ease_in`, `ease_out`, `ease_in_out`.
- `shots[*].warmup_steps` (int): basic timesteps to step before frame 0. Lets the world settle (physics, controllers) before the camera starts moving.
- `shots[*].playback_speed` (float): the high-level knob for how fast the world appears to play in the rendered video. `1.0` = real-time, `0.5` = half-speed slow-mo, `2.0` = 2x time-lapse. The service computes the correct `settle_steps_per_frame` from this and the world's `WorldInfo.basicTimeStep`. **Use this instead of `settle_steps_per_frame` unless you have a specific reason.** Robots commanded near their max wheel speed (e.g. ~1 m/s for the Husky) often look frantic at 1.0x combined with a moving camera; 0.5x reads more deliberate without losing visible coverage.
- `shots[*].settle_steps_per_frame` (int): low-level override. Number of sim timesteps advanced between frame dumps. Wins over `playback_speed` when both are given. Default `1` (one sim tick per frame).
- `shots[*].keep_frames` (bool): retain the intermediate PNG sequence next to the output. Off by default.
- `shots[*].path_keyframes` (list): camera keyframes. Each entry has `t` (seconds within the shot), `position` ([x, y, z]), and either `target` ([x, y, z]) or `orientation` ([ax, ay, az, angle]).

## When to reach for it

- **Cinematic demos and YouTube content** — drive a shot list, get a finished mp4. Source assets land next to the project's `social/youtube_videos/` scripts.
- **Marketing / landing-page hero stills** — still mode renders at the host viewport resolution (~1896x1184 in the shipped default; the arbitrary-resolution Camera path that would bypass viewport limits is experimental and disabled by default).
- **Regression video** — render the same shot list before and after a change, diff the videos.
- **Building a Blender compositor scene** — use `keep_frames: true` to retain the PNG sequence, then load it as the source for offline grading or compositing.

For the harness reference (sibling-file injection, hot-reload mechanics, structured-diagnostic mapper), see [scripts/harness/README.md](../harness/README.md).
