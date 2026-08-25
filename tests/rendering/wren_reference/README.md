# WREN reference renders — the last images the retired renderer produced

Captured 2026-08-23 (machine `9722d23d12a3`, RTX 3060 laptop), via the capture service
(`scripts/capture/omnisim_capture.py`, which pins `renderBackend "wren"`), at 1280×720,
immediately before Lane F1 made WREN unselectable and D1.4 deleted it.

**Why these exist (coupling C9):** the WREN-vs-wgpu comparison tooling (`render_ab.py`'s
`OMNISIM_FORCE_WREN` arm, `render_oracle.py`, the parity self-check) loses its reference arm
the moment WREN is unselectable. These stills are the frozen replacement: a post-deletion
wgpu change can still be compared against what WREN last rendered, pixel by pixel, without a
WREN binary existing anywhere.

| file | world | why this world |
|---|---|---|
| `beauty_bench_wren_1280x720.png` | `projects/samples/demos/worlds/rendering/beauty_bench.omniworld` | the LOOK benchmark; static, noise floor 0 |
| `warehouse_industrial_wren_1280x720.png` | `projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld` | flagship demo; conveyor (Track texture scroll), PBR + legacy materials mix |

**How to read a comparison against these honestly:**
- These are WREN's output, not ground truth. The wgpu renderer is *deliberately* different
  (HDR+AgX, GTAO, PCSS, SSR — the whole realism push). A diff against these answers "what
  changed relative to WREN", never "what is wrong".
- Environment: captured with a wedged zombie engine holding TCP 1234 (see the runbook's P13);
  the capture engines ran on fallback ports. No known pixel effect, recorded for completeness.
- The capture service renders through a Camera DEVICE at the requested resolution — not the
  main-view pane. Compare against sensor-path renders, or accept the documented main-view
  deltas.
