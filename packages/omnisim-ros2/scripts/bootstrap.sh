#!/usr/bin/env bash
# Fetch dependencies and build the OmniSim ROS 2 workspace.
#
#   bash packages/omnisim-ros2/scripts/bootstrap.sh
#
# Idempotent: re-running it re-clones nothing and just rebuilds.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "error: no ROS 2 environment at $ROS_SETUP" >&2
  echo "       install ROS 2 (Humble or newer), or set ROS_SETUP=/path/to/setup.bash" >&2
  exit 1
fi
# `set -u` and ROS's setup scripts disagree; ROS's own docs source them unset.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
echo "[bootstrap] ROS_DISTRO=$ROS_DISTRO"

# --- dependencies -----------------------------------------------------------
if [[ ! -d src/simulation_interfaces ]]; then
  if command -v vcs >/dev/null 2>&1; then
    echo "[bootstrap] vcs import src < deps.repos"
    vcs import src < deps.repos
  else
    echo "[bootstrap] vcstool not found; cloning simulation_interfaces directly"
    git clone --branch 2.1.0 --depth 1 \
      https://github.com/ros-simulation/simulation_interfaces.git \
      src/simulation_interfaces
  fi
else
  echo "[bootstrap] src/simulation_interfaces already present"
fi

# --- build ------------------------------------------------------------------
echo "[bootstrap] colcon build"
colcon build --symlink-install "$@"

cat <<EOF

[bootstrap] done. Activate with:

    source $HERE/install/setup.bash

Then launch against a running OmniSim harness:

    ros2 launch omnisim_ros2 omnisim_bringup.launch.py

EOF
