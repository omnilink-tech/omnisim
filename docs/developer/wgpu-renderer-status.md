# wgpu renderer — current state (snapshot 2026-06-11)

**Decision (2026-06-11, user call): WREN stays the default main-view renderer.**
The Phase ζ default flip is deferred — not for stability reasons, but because wgpu
is not yet a feature superset of WREN (gaps listed below). wgpu remains fully
available as an opt-in and is the faster renderer on the showcase content.

This doc is the canonical "where is wgpu right now" snapshot. The living task
tracker is [r4-completion-checklist.md](r4-completion-checklist.md); the
architecture/coupling map is [r4-step3c-plan.md](r4-step3c-plan.md); build setup
is [wgpu-native-setup.md](wgpu-native-setup.md).

---

## How to enable wgpu

- **Per-world (the supported path):** set `renderBackend "wgpu"` on the world's
  `Viewpoint`. Worlds without it are untouched (WREN default, byte-identical).
- **Force-everything (test lever):** env `OMNISIM_WGPU_MAINVIEW_FORCE=1`.
- Requires a wgpu-ON build (`OMNISIM_WITH_VULKAN=ON` + `WGPU_NATIVE_HOME`, see
  [wgpu-native-setup.md](wgpu-native-setup.md)); `wgpu_native.dll` ships
  automatically next to the binary.

## Measured comparison vs WREN (2026-06-11, Windows/Vulkan, city baseline)

Same world (`city_traffic.wbt` copy), same machine, same viewpoint, ~1,400 steps
each at `--mode=realtime`, instrumented with `--log-performance=` (WREN render
cost) and `OMNISIM_WGPU_REPORT=` (wgpu frame stats).

| metric | WREN | wgpu |
|---|---|---|
| Render cost / frame @1896×1113 (3,523 draws) | 21.5 ms | **8–9 ms** (~2.4× faster) |
| Whole-sim FPS, full traffic sim | 14.0 | 14.3 (tie — world is sim-bound) |
| Render-bound FPS (controllers idle) | **30.8** | ~25 (app-loop pacing gap, see below) |
| Frame pacing | n/a | maxGap 46–79 ms typical after Mailbox present |
| Quality gates | golden reference | coverage 85.2% PASS, golden PASS, deterministic readback PASS |
| 5-world sweep (city/panda/cobot-arm/spot/omni_quest) — `panda` was a scratch/private bench world not in the repo; substitute any in-repo URDF arm | — | clean, zero crashes |

**Lighting interpretation now diverges deliberately.** The city is authored as
full day (sun intensity 4.3, day sky); wgpu renders it that way. WREN renders the
same world markedly dark ("WREN is legacy-dark" — flagged in the parity-gate
commit). Because of this, the pixel-level `within-tol` advisory vs WREN (~37%)
measures distance from WREN's legacy-dark output and is NOT a quality score; the
hard gates are coverage + golden.

## What the wgpu main view has (and WREN here doesn't)

- MSAA 4×, SSAO (curvature kernel, half-res), bloom, optional HDR + AgX tonemap
  (`OMNISIM_WGPU_AGX`), ±½-LSB dither.
- Reversed-Z float depth (far-field z-fighting structurally gone — the
  road/grass flicker class), authored near plane honored.
- PCF 5×5 sun shadows with receiver-plane depth bias + normal-offset receivers,
  camera-following fitted ortho frustum.
- WREN-exact fog curve, mixed in linear space; procedural sky dome.
- Native Vulkan presentation (input-transparent child window) with Mailbox
  present mode — no readback on the present path, no vblank hitching.
- Full editor integration: picking, selection highlight, manipulators
  (translate/rotate drag incl. nested/rotated parents), all 8 optional
  renderings, screenshots, live-pane overlays.
- Full PBR material ladder: albedo/roughness/metalness/normal maps,
  Cook-Torrance GGX, sRGB color management, TextureTransform (2D affine).
- R5 sensor pipeline (Camera/RangeFinder/Lidar) parity-confirmed against the
  WREN oracle (separate offscreen path; see checklist).

## Known gaps — why WREN stays default (feature superset not reached)

| gap | impact |
|---|---|
| **Single-sun lighting** — only the first `DirectionalLight` is lit + shadowed; extra fill lights ignored; no PointLight/SpotLight illumination in the main view | worlds relying on multi-light setups render flatter than WREN (night scenes work via emissive) |
| **No transparency** — alpha-blended materials unimplemented (alpha forced 1.0) | windows/glass/ghost materials render opaque |
| **No image backgrounds/cubemaps** — procedural sky only | `TexturedBackground`-style worlds lose their backdrop |
| **Render-bound FPS gap** — GPU is 2.4× faster but the app loop converts it to ~25 vs WREN's 30.8 FPS | pacing/loop investigation pending (backlog) |
| **Windows/Vulkan only** — no Metal (macOS) / Linux surfaces yet | ζ blocker per plan |
| **Sensor/main-view path unification** pending | two code paths to keep in sync (WbCamera carries inline equivalents) |

## Recently fixed bug classes (world-author relevant)

- **`castShadows FALSE` honored** (`b5fed840`): previously ALL draws entered the
  shadow map — the floating SUN_MARKER sphere shadow-bombed small worlds
  (spot.wbt's "concentric rings"). Diagnostics that found it (reusable):
  `OMNISIM_WGPU_SHADOWMAP_DUMP=<png>` (dump the light-depth map; also set
  `OMNISIM_WGPU_MAINVIEW_DUMP` to force the readback path) and
  `OMNISIM_WGPU_SHADOW_DEBUG=1` (render the depth-compare error field).
- **`TextureTransform` applied** (`f8c98c66`): previously raw mesh UVs were
  sampled — `Grass { scale 40 40 }` on a bare Plane smeared one tile across
  200 m (omni_quest). Derived as an exact 2D affine from WREN's
  `transformUVCoordinate`, so scale/rotation/center/translation all match.

## Diagnostic env levers (all off by default, zero-cost when unset)

| env | effect |
|---|---|
| `OMNISIM_WGPU_MAINVIEW_FORCE=1` | force wgpu main view on any world |
| `OMNISIM_WGPU_MAINVIEW_DUMP=<png>` (+`_FRAME=<n>`) | dump a main-view frame |
| `OMNISIM_WGPU_REPORT=<file>` | per-100-frame stats (draws, renderMs, maxGapMs, resource counts) |
| `OMNISIM_WGPU_NO_SWAP` / `NO_REVZ` / `NO_SSAO` / `NO_SHADOW` | feature kill-switches for bisecting |
| `OMNISIM_WGPU_SHADOWMAP_DUMP=<png>` | dump the pass-1 light-depth map |
| `OMNISIM_WGPU_SHADOW_DEBUG=1` | render shadow depth-compare error field |
| `OMNISIM_WGPU_AGX=1` | HDR + AgX tonemap |
| `OMNISIM_WGPU_ERRLOG=<file>` | wgpu uncaptured-error callback log |
| `OMNISIM_PROBE_PICK/READBACK/LINE/TEX/INSET/CSM=<file>` | headless feature probes (run via `--help`) |

Parity/quality harness: `scripts/dev/render_oracle.py --world <w> --report`
(writes `.wren.png` / `.wgpu.png` / `.wgpu_shadow.png` + coverage/within-tol
report; within-tol is advisory, hard gate = coverage + golden). Known harness
limitation: the uniform-golden guard misfires on legitimately plain scenes
(e.g. spot's top-down gray floor) → perpetual UNAVAILABLE; use a window capture
for the WREN reference there.

## Resume path for the eventual flip

Phase ζ checklist (cross-platform surfaces, CI golden gate, perf bar, build
matrix) in [r4-completion-checklist.md](r4-completion-checklist.md) §ζ. The gaps
table above is the de-facto pre-flip worklist; the flip itself is a one-line
`Viewpoint.wrl` default change and is **human-gated — do not flip without an
explicit user decision.**
