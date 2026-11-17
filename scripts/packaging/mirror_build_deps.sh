#!/usr/bin/env bash
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

# Mirror OmniSim's prebuilt build dependencies to OmniSim's own GitHub Releases.
#
# WHY THIS EXISTS
# ---------------
# Building OmniSim used to download its prebuilt dependencies (assimp, OpenVR, OIS,
# ffmpeg, freetype, Qt) from Cyberbotics' server. That made a clean-clone build of
# OmniSim depend on infrastructure owned by the upstream project OmniSim forked from:
# if that host goes away, OmniSim cannot be built on any platform.
#
# This script copies those exact archives to OmniSim's own releases, so the build
# depends only on us. The archives are byte-identical, and dependencies/Makefile.*
# MD5-verifies every one of them after download -- so a mirror is safe by construction:
# a corrupted or substituted file fails the build loudly.
#
# RUN THIS ONCE, then delete the DEPENDENCIES_FALLBACK_URL lines from the three
# dependencies/Makefile.* files.
#
# NOTE ON Qt -- deliberately NOT mirrored, and that is correct on every platform:
#   * Windows takes Qt from MSYS2; dependencies/Makefile.windows never fetches it.
#   * Linux runs scripts/install/qt_linux_installer.sh, which pulls Qt from Qt's OWN
#     servers via aqtinstall. The `webots-qt-*.tar.bz2` name in Makefile.linux is only
#     a make marker file -- it is never downloaded. (Do not be misled by the fact that
#     the equivalent URL on the upstream host now 404s: nothing fetches it.)
#   * macOS follows the same installer pattern.
# So mirroring Qt would add ~150 MB and buy nothing. Verified 2026-07-12: Qt 6.5.3 for
# linux/gcc_64 resolves 200 from download.qt.io.
#
# Requires: gh (authenticated with write access to the repo), curl.
#
#   bash scripts/packaging/mirror_build_deps.sh            # dry run: fetch + checksum
#   bash scripts/packaging/mirror_build_deps.sh --publish   # also upload to GitHub

set -euo pipefail

REPO="${OMNISIM_REPO:-omnilink-tech/omnisim}"
UPSTREAM="https://cyberbotics.com/files/repository/dependencies"
WORK="$(mktemp -d)"
PUBLISH=0
[ "${1:-}" = "--publish" ] && PUBLISH=1

trap 'rm -rf "$WORK"' EXIT

# platform : upstream-dir : release-tag : archives...
MANIFEST=(
  "windows:windows:deps-windows-v1:assimp-5.2.3.zip openvr-1.0.7.zip libOIS.zip"
  "linux:linux64:deps-linux-v1:libOIS.1.4.tar.bz2 libassimp-5.2.3.tar.bz2"
  "mac:mac:deps-mac-v1:assimp-5.2.3.tar.bz2 ffmpeg.tar.bz2 freetype2.tar.bz2 libOIS-1.3.tar.bz2"
)

for row in "${MANIFEST[@]}"; do
  IFS=':' read -r plat updir tag files <<< "$row"
  echo ""
  echo "=== $plat  ->  release tag: $tag"
  mkdir -p "$WORK/$plat"
  got=()
  for f in $files; do
    url="$UPSTREAM/$updir/release/$f"
    if curl -sfL --max-time 300 -o "$WORK/$plat/$f" "$url"; then
      sz=$(du -h "$WORK/$plat/$f" | cut -f1)
      md5=$(md5sum "$WORK/$plat/$f" | awk '{print $1}')
      echo "    fetched  $f  ($sz)  md5=$md5"
      got+=("$WORK/$plat/$f")
    else
      echo "    !! could NOT fetch $f from upstream ($url)"
      echo "       If upstream is already gone, source this archive elsewhere and add it by hand."
    fi
  done

  if [ "$PUBLISH" = "1" ] && [ ${#got[@]} -gt 0 ]; then
    gh release view "$tag" --repo "$REPO" >/dev/null 2>&1 \
      || gh release create "$tag" --repo "$REPO" --title "Build dependencies ($plat)" \
           --notes "Prebuilt build dependencies for OmniSim on $plat. Mirrored so that building OmniSim does not depend on third-party infrastructure. Consumed by dependencies/Makefile.$plat; every archive is MD5-verified at build time." \
           --latest=false
    gh release upload "$tag" --repo "$REPO" --clobber "${got[@]}"
    echo "    uploaded ${#got[@]} archive(s) to $REPO @ $tag"
  fi
done

echo ""
if [ "$PUBLISH" = "1" ]; then
  echo "Done. Now remove the DEPENDENCIES_FALLBACK_URL lines from dependencies/Makefile.*"
  echo "and verify a clean build fetches from $REPO."
else
  echo "Dry run complete. Re-run with --publish to upload."
fi
