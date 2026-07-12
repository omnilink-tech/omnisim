#!/bin/bash
# setup_wgpu_native.sh — download wgpu-native + drop it where the
# Makefile expects it, so OMNISIM_WITH_VULKAN=ON builds actually
# link against the wgpu runtime (and the R3.4 wgpu Camera path
# stops short-circuiting to the WB_WGPU_NATIVE_AVAILABLE=0 stub).
#
# Why this is a script and not a recipe in build_with_cd.sh: the
# wgpu-native binary is an external prebuilt release pulled from
# GitHub. The auto-mode classifier blocks Claude from running
# downloads on its own (see r3.4-step-5 commit message); a human
# can opt in by running this script once.
#
# After this script runs successfully:
#   1. $REPO_ROOT/_scratch/wgpu-native/ holds include/ + lib/.
#   2. $REPO_ROOT/msys64/mingw64/bin/wgpu_native.dll exists.
#   3. Subsequent `bash scripts/dev/build_with_cd.sh` invocations
#      build the WB_WGPU_NATIVE_AVAILABLE=1 path — the R3.4 wgpu
#      Camera scene walk runs for real.
#
# After the install + a rebuild, runtime-verify with:
#   OMNISIM_HOME=$(pwd) \
#     python scripts/dev/headless_runner.py \
#     projects/samples/demos/worlds/rendering/camera_wgpu_scene_smoke.wbt \
#     --duration 6
#
# Expected log line in omnisim_log.txt:
#   INFO: [WbCamera scene] first draw: idx=36 baseColor=(0.00,1.00,1.00) ...
#   INFO: [WbCamera] 'cam_wgpu' rendered through wgpu (64x48, 1 draw)
# And `_scratch/r33b_smoke.txt`: `non-clear pixels: > 0 of 3072`
# with the center pixel near `BGRA=(255,255,0,255)` (cyan-shaded
# under the hardcoded lambertian light).

set -euo pipefail

# Pinned to v29.0.0.0: WbVulkanBackend.cpp is "API tracked against wgpu-native
# v29.0.0+" (the adapterCb/deviceCb two-userdata callback signature; releases
# <= v24 used a single-userdata signature and will NOT compile). The previous
# default v24.0.5.1 was also a NON-EXISTENT tag (404) — gfx-rs/wgpu-native never
# published it. v29.0.0.0 is the latest release.
WGPU_VERSION="${WGPU_NATIVE_VERSION:-v29.0.0.0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSTALL_DIR="$REPO_ROOT/_scratch/wgpu-native"

case "$(uname -s)" in
  # GNU (MinGW) variant, NOT msvc: OmniSim builds with MinGW g++, the Makefile
  # links `-lwgpu_native` and this script verifies `libwgpu_native.dll.a` — both
  # require the GNU import lib. The msvc zip ships only `.lib` (MSVC) and would
  # fail the .dll.a check + the MinGW link.
  MINGW*|MSYS*|CYGWIN*) ARCHIVE="wgpu-windows-x86_64-gnu-release.zip"; PLATFORM="windows" ;;
  Linux*)   ARCHIVE="wgpu-linux-x86_64-release.zip";   PLATFORM="linux"   ;;
  Darwin*)  ARCHIVE="wgpu-macos-aarch64-release.zip";  PLATFORM="macos"   ;;
  *) echo "[setup_wgpu_native] unknown platform $(uname -s); manual install required" >&2; exit 2 ;;
esac

URL="https://github.com/gfx-rs/wgpu-native/releases/download/${WGPU_VERSION}/${ARCHIVE}"

echo "[setup_wgpu_native] target version : $WGPU_VERSION"
echo "[setup_wgpu_native] platform        : $PLATFORM"
echo "[setup_wgpu_native] install dir     : $INSTALL_DIR"
echo "[setup_wgpu_native] source URL      : $URL"

if [ -f "$INSTALL_DIR/include/webgpu/webgpu.h" ] && [ -f "$INSTALL_DIR/lib/libwgpu_native.dll.a" ]; then
  echo "[setup_wgpu_native] already installed at $INSTALL_DIR — skipping download"
else
  rm -rf "$INSTALL_DIR"
  mkdir -p "$INSTALL_DIR"
  TMP="$(mktemp -d)"
  echo "[setup_wgpu_native] downloading $ARCHIVE..."
  curl -sSfL -o "$TMP/$ARCHIVE" "$URL"
  echo "[setup_wgpu_native] extracting..."
  ( cd "$INSTALL_DIR" && unzip -q "$TMP/$ARCHIVE" )
  rm -rf "$TMP"
fi

if [ ! -f "$INSTALL_DIR/include/webgpu/webgpu.h" ]; then
  echo "[setup_wgpu_native] FAIL: $INSTALL_DIR/include/webgpu/webgpu.h missing after install" >&2
  exit 3
fi
if [ ! -f "$INSTALL_DIR/lib/libwgpu_native.dll.a" ] && [ ! -f "$INSTALL_DIR/lib/libwgpu_native.so" ] && [ ! -f "$INSTALL_DIR/lib/libwgpu_native.dylib" ]; then
  echo "[setup_wgpu_native] FAIL: no wgpu_native library found under $INSTALL_DIR/lib/" >&2
  exit 4
fi

# Drop the runtime DLL next to omnisim-bin.exe so the simulator can
# discover it at process start. The build hook in Makefile adds
# $(WGPU_NATIVE_HOME)/lib to the link line; the runtime needs the
# .dll alongside the .exe on Windows.
case "$PLATFORM" in
  windows)
    if [ -f "$INSTALL_DIR/lib/wgpu_native.dll" ]; then
      mkdir -p "$REPO_ROOT/msys64/mingw64/bin"
      cp -f "$INSTALL_DIR/lib/wgpu_native.dll" "$REPO_ROOT/msys64/mingw64/bin/wgpu_native.dll"
      echo "[setup_wgpu_native] copied wgpu_native.dll -> $REPO_ROOT/msys64/mingw64/bin/"
    fi
    ;;
esac

echo
echo "[setup_wgpu_native] DONE."
echo "Next: rebuild with the wgpu-native path turned ON."
echo
echo "    OMNISIM_WITH_VULKAN=ON \\"
echo "    WGPU_NATIVE_HOME=$INSTALL_DIR \\"
echo "    bash scripts/dev/build_with_cd.sh"
echo
echo "Then verify the R3.4 scene walk (engine-migration-plan §14.3, row R3.4):"
echo
echo "    OMNISIM_HOME=\$(pwd) \\"
echo "    python scripts/dev/headless_runner.py \\"
echo "      projects/samples/demos/worlds/rendering/camera_wgpu_scene_smoke.wbt \\"
echo "      --duration 6"
echo
echo "    cat _scratch/r33b_smoke.txt"
