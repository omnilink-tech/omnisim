# Capture must preserve the simulator window state

On 30 October 2026, repeated `Supervisor.exportImage()` calls reproduced a
focus-stealing bug on Windows. The screenshot path maximized a minimized
OmniSim window before every frame, then minimized it again after saving.
Agent videos assembled from these images consequently flashed the simulator
over the operator's application. Native movie start/stop used the same
unnecessary window-state changes.

The fix removes those changes from `OmSimulationView`. wgpu already renders
and reads back frames while the view is unexposed. Capture temporarily enables
rendering when needed and restores the rendering setting afterward, without
restoring, maximizing or activating the window. This also avoids changing a
window state the operator selected during recording.

## Reproduction and verification

Machine `9722d23d12a3`, Windows, RTX 3060 Laptop GPU, wgpu, 16 ms world step.
The probe reads Win32 window state only; it never changes focus or minimizes
the window itself. It watches only the simulator process it owns, after a
barrier confirms the main window is minimized. A marker changes position
between frames so a blank or stale capture cannot pass.

| Run | Output | Restores observed | Foreground samples |
|---|---|---:|---:|
| Original build | 6 changing PNGs | 6 | 199 |
| Fixed candidate, rendering on | 12 changing PNGs | 0 | 0 |
| Fixed candidate, rendering initially off | 12 changing PNGs | 0 | 0 |
| Installed fixed build | 12 changing PNGs | 0 | 0 |
| Installed fixed build, native movie | 62 decoded frames, 640 × 360 | 0 | 0 |

The movie decoded completely with no errors, has no audio stream, and contains
changing scene pixels. The three live regression tests passed. These are
observations on Windows, not a claim of testing every OS/window manager.
This rendering fixture uses kinematic markers; it measures capture and window
behavior, not robot physics.

Original executable SHA-256:
`1a5283b9e47e3037c08c96c084ca9c381d946d2b9857d55370a982cb1174f9ae`.
Installed fixed executable SHA-256:
`b37b94793163416521bc93820919ee60cffce3b1dd3b26f7eff72416170b795a`.
The previous local executable is retained alongside the installed binary as
`omnisim-bin.pre-capture-window-fix.exe`. No unrelated simulator was stopped.

Retained measurements, per-sample window observations, logs, example images,
movie and test XML are in
[`tests/rendering/evidence/capture-window-state-20261030`](../../tests/rendering/evidence/capture-window-state-20261030).

## Rerun

From the OmniSim root on a Windows desktop with the engine built, Pillow,
psutil and ffmpeg/ffprobe available:

```powershell
python -m pytest tests/test_capture_window_state.py -q
python tests/rendering/capture_window_probe.py --out .tmp/capture-window-new --count 12
python tests/rendering/capture_window_probe.py --out .tmp/capture-movie-new --kind movie
```

Use a fresh output directory each time. `--binary <path>` selects a candidate
or the retained original for the negative control; `--no-rendering` exercises
capture with ordinary rendering initially disabled. The negative control is
expected to fail the window-state checks and briefly flash while recording.
