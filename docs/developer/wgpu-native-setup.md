# wgpu-native build-host setup

**Status (2026-09-02): mandatory.** wgpu-native is the ONLY renderer — WREN was deleted on
2026-08-23 (`976b9449d`) and there is no fallback. A build that cannot find wgpu-native is
**REFUSED** (`WGPU_NATIVE_HOME` is auto-discovered by the Makefile; `OMNISIM_RENDERERLESS=ON`
is the only way to build without it, by name). A host whose wgpu-native cannot initialise at
runtime has no renderer at all: one loud log line, physics and controllers unaffected.
The R3-era text below (2026-05-28) is the install recipe; where it says "optional" or
"falls back to WREN", read the paragraph above instead.

This doc lives so that when a developer wants to actually exercise
the wgpu path (R3.2 mesh cache testing, R3.3 single-Camera RTT
work, ...) they have the install recipe in one place.

## 1 — What you need

A copy of `wgpu-native` built for your platform + the matching C
headers. wgpu-native is the Rust implementation of WebGPU that ships
a C ABI via `wgpu.h`/`webgpu.h`. We pin to release tags rather than
tracking `main` — wgpu's API is stable but tag-pinning lets us bump
deliberately.

Layout we expect (matches the official release ZIP layout):

```
$WGPU_NATIVE_HOME/
  include/
    webgpu/
      webgpu.h       # WebGPU C ABI (standardised)
    wgpu.h           # wgpu-native extension symbols (e.g. wgpuInstanceRelease)
  lib/
    libwgpu_native.dll.a    # on MinGW64 / MSYS2 Windows
    wgpu_native.dll         # runtime
    # OR libwgpu_native.so / libwgpu_native.dylib on Linux/macOS
```

## 2 — Install path (MSYS2 / MinGW64, the OmniSim default on Windows)

1. Pick a release from https://github.com/gfx-rs/wgpu-native/releases.
   Verified working: v24.x and later. Avoid pre-release tags.
2. Download `wgpu-windows-x86_64-msvc-release.zip` (despite the name,
   the headers + dynamic library are toolchain-agnostic; MinGW links
   against the `.dll.a` fine).
3. Extract into a path of your choice, e.g.
   `D:\dev\wgpu-native-v24.0.0\`.
4. Verify the layout — `include/webgpu/webgpu.h` and
   `lib/libwgpu_native.dll.a` must exist.
5. Export `WGPU_NATIVE_HOME` before building:

   ```bash
   export WGPU_NATIVE_HOME=/d/dev/wgpu-native-v24.0.0
   ```

   (Or set it inline on the make invocation:
   `make ... WGPU_NATIVE_HOME=/d/dev/wgpu-native-v24.0.0`.)

6. Build with both flags on:

   ```bash
   OMNISIM_WITH_VULKAN=ON \
   WGPU_NATIVE_HOME=/d/dev/wgpu-native-v24.0.0 \
   bash scripts/dev/build_with_cd.sh
   ```

   …or invoke `make` directly with the flags as command-line args (the
   canonical wgpu-ON recipe; flags passed this way are unambiguous and
   override the Makefile `?=` defaults):

   ```bash
   make -C src/omnisim release \
     OMNISIM_WITH_CUDA=OFF OMNISIM_WITH_NEWTON=ON OMNISIM_WITH_VULKAN=ON \
     WGPU_NATIVE_HOME=$PWD/_scratch/wgpu-native
   ```

   The link step now **auto-ships `wgpu_native.dll`** next to
   `omnisim-bin.exe` (via the Makefile's post-link `EXTRA_CMD`, guarded
   on `OSTYPE=windows` + the dll's presence) — no manual copy or PATH
   tweak is needed any more. Non-wgpu / non-Windows builds are
   byte-for-byte unaffected (the copy lives inside the
   `OMNISIM_WITH_VULKAN=ON` block).

7. Launch OmniSim on any world. The log should include
   `[OmWgpuBackend] wgpu-native init OK ...` once the
   `OmRenderBackendRegistry` first resolves a wgpu backend.

## 3 — Install path (Linux / macOS)

Same idea, different archive:

- Linux:  `wgpu-linux-x86_64-release.zip`
- macOS:  `wgpu-macos-aarch64-release.zip` or `wgpu-macos-x86_64-release.zip`

The Makefile detection in `src/omnisim/Makefile` is identical —
checks for `$(WGPU_NATIVE_HOME)/include/webgpu/webgpu.h`. The link
line picks up `-lwgpu_native` from the `lib/` directory.

## 4 — Verifying

With `WGPU_NATIVE_HOME` set and the build green, run the smoke world
and look for the init log:

```bash
$ py -3 tests/smoke/run_smoke.py --nomake 2>&1 | grep OmWgpuBackend
INFO: [OmWgpuBackend] wgpu-native init OK (instance + adapter + device + queue)
```

If the log is silent (no init message), check:

- `WGPU_NATIVE_HOME` actually pointed at a directory the build host
  could read at compile time. The Makefile uses `$(wildcard ...)`
  for detection, so a typo or missing header silently disables the
  whole path — the binary still builds, just without the wgpu code
  compiled in.
- `OMNISIM_WITH_VULKAN` was `ON` at build time (the default since the
  2026-06-07 baseline flip; since 2026-08-23 wgpu is the only renderer and
  the `renderBackend` field is a warned no-op).
- The `wgpu_native.dll` is present next to `omnisim-bin.exe`. The build
  now copies it there automatically on the link step (see §2.6); if it's
  missing, force a re-link (`touch` any source + rebuild) or confirm
  `$(WGPU_NATIVE_HOME)/lib/wgpu_native.dll` existed at build time.

## 5 — When the flag is OFF (opt-out)

The Makefile detection is a no-op; no wgpu headers get included; no
linker dependency on `libwgpu_native`; the constructor stays in the
"wgpu-native unavailable" branch and `mAvailable` is false.

This is the explicit opt-out build state (`OMNISIM_WITH_VULKAN=OFF`);
the flag defaults to `ON` since the 2026-06-07 baseline flip. The wgpu
work is introduced behind these flags so that the `Newton-ON,
Vulkan-OFF` cell of the build matrix stays byte-equivalent to its
pre-R3.1 self.

## 6 — Next steps

- R3.2 (`engine-migration-plan.md` §14.3): wgpu mesh cache. Uses the
  device the constructor opens here.
- R3.3: first single-Camera render-to-texture. Opt in by adding
  `renderBackend "vulkan"` to a Viewpoint or Camera.
- R3.4: Path-3 shader port (GLSL→WGSL hand-port + naga long tail).
- R3.5: texture bridge.
- R3.6: golden-image parity vs WREN (done; WREN deleted 2026-08-23).
- R3.7: Newton interop — wgpu storage buffer = Newton body buffer.
  Gated on Physics Phase D (which fired 2026-05-28; see the §15
  decision log).
