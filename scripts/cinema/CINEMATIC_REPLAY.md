# Cinematic replay: simulate once, render the recorded motion

This is the default production method for new OmniSim advertisements, cinematic
demos, hero footage and other videos intended to look photorealistic. OmniSim
computes the physics and controller behavior. Blender Cycles renders the
recorded motion in an authored cinematic scene. The first advertisement used
this separation successfully; the reusable implementation lives beside this
guide, independently of that film's temporary scripts and output directory.

**Do not substitute native screenshots when cinematic quality is requested.**
If Blender or the required assets are missing, identify the missing prerequisite
and prepare it. Native capture remains an explicit choice for debugging,
regression comparisons, sensor images, and demonstrations of the live renderer.
An explicit user request for native footage takes precedence.

## The production contract

1. **Design an action worth watching.** Write a concrete objective, obstacle or
   interaction, visible result, and claim boundaries. Show purposeful action,
   consequence and, when relevant, recovery. A collection of flattering angles
   on random driving is not a substitute for a compelling scenario.
2. **Run and measure it in OmniSim.** Follow the engine doctor, world-authoring
   and physics guides. Keep the actual world, controller source, seeds/settings,
   result logs and the run's Newton sidecar. A finalized physics runtime does
   not prove that a task succeeded. Failed outcomes remain failed in the film.
3. **Record world-space poses.** Capture every visible moving robot link,
   articulated prop and payload after simulation steps. Include the interval
   needed by all shots. Preserve the recording as source evidence.
4. **Prepare the cinematic scene.** Import the same robot/prop geometry, retain
   physical dimensions and contact surfaces, and parent visual meshes to named
   pose anchors. Author lighting, materials, set dressing and cameras in Blender.
5. **Validate, bake and render a proxy.** Verify source intervals and bindings;
   interpolate translation and quaternion rotation; compare baked output-frame
   poses against the recording. Review moving proxy clips and the contact sheet.
6. **Improve the action or presentation as needed.** Inspect visible outcomes,
   contact, occlusion, shot continuity, pacing, framing, materials and disclosure.
   If the action is dull, revise and rerun the simulation. Never repair its result
   with invented robot/payload animation. Re-render cameras without rerunning
   unchanged physics.
7. **Render final clips, edit and deliver.** Retain the packed scenes, recordings,
   manifests, proxy review, bake reports, input hashes, MP4s, license notices and
   final edit. Inspect the finished film as well as its component shots.

No recurring automation or external service is needed. The agent directing the
film can perform and record the review; this is not an extra user-approval step.

## Start a production

```bash
python -m omnisim cinema new --title "Warehouse recovery" --world world.omniworld > film.json
```

`new` now creates an `omnisim_cinematic_replay_v1` manifest with
`renderer: "blender_cycles"`. Fill its TODOs, create the scene and recordings,
and point its run evidence to real files. Every relative path, including
`output`, resolves beside the manifest. Keep takes in separate directories.

```bash
python -m omnisim cinema replay-validate film.json
python -m omnisim cinema render film.json --blender "/path/to/blender" --device OPTIX
# Watch every proxy MP4 and inspect contact_sheet.jpg before this command:
python -m omnisim cinema replay-review film.json --notes "Describe the actual review findings here."
python -m omnisim cinema render film.json --profile final --blender "/path/to/blender" --device OPTIX
```

Blender on PATH is discovered automatically, as are standard Windows installs.
`--device CPU` is the portable default; select OPTIX/CUDA/HIP/METAL/ONEAPI when
available. A requested unavailable device fails explicitly. Install Blender and
ffmpeg locally; the CLI does not install software, spend cloud credits or launch
a simulation. The implementation was exercised with Blender 4.4.3; other
versions need a short local smoke render before a large production.

`render` defaults to the **proxy**, at 50% resolution and eight Cycles samples.
Final uses the manifest resolution and 32 samples. Both use the same frame rate,
shot timing, scene and pose interpolation. Defaults are 1080p24, AgX with medium
high contrast, exposure -0.55, denoising, five light bounces and 0.35-shutter
motion blur. Depth of field, focus, focal length and lighting are authored in
the source scene. These are starting settings, not a guarantee of photorealism.
The visual quality depends heavily on geometry, materials, light and composition.

## Record motion without changing the controller

The helper has no engine import and issues no commands or simulation steps.
Integrate it into an existing supervisor/controller loop; preserve the original
control law and step cadence. Add the repository's `scripts` directory to that
controller's import path when necessary:

```python
from cinema.replay_record import PoseRecorder, discover_rigid_nodes

# `robot` is the existing supervisor, not a second instance.
nodes = discover_rigid_nodes(robot.getSelf())
nodes["PAYLOAD"] = robot.getFromDef("BLOCK")
with PoseRecorder("/absolute/path/to/take_01/motion.jsonl", nodes) as recording:
    while robot.step(timestep) != -1:
        recording.sample(robot.getTime())
        # The existing controller's logic stays here, unchanged.
```

Adapt this example to the existing loop rather than adding another loop or
stepping twice. Use supervisor access to read poses. A dedicated recording
supervisor can instead receive an explicit mapping of nodes. Discovery traverses
`children` and joint `endPoint` fields and rejects duplicate names. Unusual
PROTOs may require explicit mappings; compare discovered links with the robot
model. Missing payloads must fail, not disappear silently.

Record one file per robot when names overlap. Give each object one authoritative
recording; record a shared payload only once. Sample every physics step for fast
motion or contact; lower sampling rates can lose wheel turns and brief impacts.
The JSONL format is:

```json
{"time":0.008,"poses":{"body":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}}
```

Time is simulation seconds. Each pose is a **row-major world-space 4×4 rigid
transform**, in metres, with a right-handed Z-up coordinate system. The recorder
writes `getPose()` as received; it does not convert a Y-up world. Convert geometry
and poses consistently before replay if the source uses different coordinates,
and retain the conversion script. Resets require a new take. Files are opened
exclusively so a recording cannot overwrite an earlier take.

## Build a reusable Blender scene

- Use the original URDF/CAD visual meshes and their visual-origin transforms.
  Name an unparented, unit-scale Empty for each recorded rigid link, for example
  `robot/link1`. Parent its visual meshes to that Empty in link-local coordinates.
  Do not reapply the URDF joint hierarchy: recorded link poses are already in
  world space. A payload needs its own anchor.
- Map recording keys to object names in `recordings[].bindings`. Include every
  moving element that the camera can see. Unbound source keys may be omitted
  from the scene, but visible moving bodies must not keep unrelated animation.
- Anchors must have no parent, constraint, rigid body or driver. Their existing
  animation is replaced by recorded poses. Children carry the mesh scale and
  visual offsets. The scene's unit scale is one metre.
- Cameras may use authored animation and focus targets. Each shot names a
  camera in the active scene. Camera animation starts at Blender frame 1 for
  each shot; `source_start_s` applies to simulation motion, not the camera clock.
  Timeline camera-switch markers are cleared by the renderer.
- Add bevels, plausible roughness, fine surface variation, credible industrial
  details, motivated area lights and controlled reflections. Cosmetics follow
  the same pose anchors. Preserve wheel radius, support planes, obstacle
  boundaries, payload dimensions and all geometry relevant to a claim.
- Make linked libraries local and **pack external images and fonts** into the
  `.blend` file. This binds their contents to the scene hash and makes source
  delivery portable. The first version rejects movie/sequence/UDIM image
  sources. Keep asset licenses beside the project.

Scene preparation is still an authored step. This is a reusable rigid-pose
recorder, binder and renderer, **not an automatic `.omniworld`-to-Blender scene
converter**. It supports arbitrary named rigid objects and multiple robots;
cloth, fluids and deforming meshes need a separate measured geometry-cache
exporter before they can use this method faithfully. Do not approximate their
physics with cosmetic animation and label it simulated evidence.

## Timing, evidence and reuse

For output frame `i`, starting at one:

`simulation_time = source_start_s + (i - 1) × speed / fps`

`speed: 1` preserves speed; `0.5` gives slow motion; `2` compresses time. Duration
must be an exact number of frames. Translation uses linear interpolation and
orientation uses shortest-path quaternion interpolation. Motion-blur subframes
interpolate the baked output animation; they are not new physics samples.

All recordings used in a scene must cover every selected frame; out-of-range
poses are rejected instead of held or clamped. For different worlds or takes,
use separate manifests/scenes, then edit their clips together. Keep meaningful
failures, support conditions and retiming visible in the edit and its notes.

The input key includes the resolved manifest, source scene, recordings, world,
controller files, Newton sidecars, result evidence, font and rendering scripts.
Keep imported controller modules/settings in `runs[].evidence` as well. The
sidecar gate checks `backend: newton`, `finalised: true`, `degraded: false`.
This proves runtime initialization, not successful physics behavior or that
unrelated files came from the same run; the producer must preserve that chain.

Artifacts live under `<output>/<input-key-prefix>/<proxy|final>/`:

- Each shot has its PNG sequence, labeled MP4, packed `replay.blend`, Blender
  log and `bake_report.json` with output-frame pose errors and actual devices.
- `resolved.json`, `receipt.json` and `contact_sheet.jpg` describe the production.
- `review.json` above those profile directories binds review notes to the proxy.

Completed videos and bake reports are hash-checked before reuse. Interrupted
jobs rerender their shots; they never treat a partial sequence as complete.
Changed inputs create a new job and invalidate the old review. Final rendering
requires a review of the current proxy. Concurrent writers to the same job
directory are unsupported; use separate output roots for simultaneous work.

The MP4s carry **“OMNISIM MOTION / CINEMATIC REPLAY”** throughout. Raw PNGs and
Blender files are editing intermediates; carry this disclosure into any edit
made from them. These clips demonstrate recorded OmniSim behavior, not native
viewport quality, native sensor imagery or real-time renderer performance.

## Existing capture and film editing

Native tools still work explicitly:

```bash
python -m omnisim cinema new --renderer native --title "Renderer regression" > native.json
python -m omnisim cinema render native.json
# Old native storyboards without a renderer field:
python -m omnisim cinema render old_storyboard.json --renderer native
python -m omnisim capture
```

There is no silent fallback to native capture. Add `"renderer": "native"` to
an old storyboard or choose the explicit override when native footage is the
intended evidence. The low-level capture HTTP service retains its native
stills/sequences contract; it cannot turn a screenshot into relightable geometry.

Feed final replay MP4s into the existing edit/voice/branding tools. For Agent
Build Films, use replay for cinematic presentation and keep native captures
where the picture itself is the evidence (UI, sensors, renderer comparisons).
Attach replay receipts, run evidence and claim boundaries to the film project.
The Agent Build editor does not automatically create replay scenes or validate
their provenance: run this workflow first. A faithful replay can be a `clip`
showing recorded simulation behavior; put its source type and relevant limits
in `claim_boundary` and retain the on-screen label. Never describe it as native
capture merely to satisfy a real-footage quota. Its source type must remain
explicit in production notes and on screen.
