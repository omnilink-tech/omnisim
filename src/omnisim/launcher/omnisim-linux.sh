#!/bin/bash

# prepare for handling program termination
kill_omnisim() {
  kill -TERM "${omnisim_pid}" &> /dev/null
}
handle_termination() {
  if [ "${omnisim_pid}" ]; then
    kill_omnisim
  else
    term_kill_needed="yes"
  fi
}
unset omnisim_pid
unset term_kill_needed
trap 'handle_termination' TERM INT

# Locate the install root, following relative symlinks.
#
# This script is installed at TWO paths, so it cannot simply assume the install
# root is its own directory:
#   $OMNISIM_HOME/bin/omnisim   the primary launcher (next to bin/omnisim-bin,
#                               matching where Windows puts omnisim.exe)
#   $OMNISIM_HOME/webots        the legacy alias, kept for compatibility
# Resolve by looking for the simulator binary rather than by guessing depth, so
# both install paths -- and a symlink from anywhere on PATH -- work unchanged.
omnisim_self="$(readlink -f "$0")"
omnisim_self_dir="$(dirname "$omnisim_self")"
if [ -f "$omnisim_self_dir/bin/omnisim-bin" ]; then
  omnisim_home="$omnisim_self_dir"
elif [ -f "$(dirname "$omnisim_self_dir")/bin/omnisim-bin" ]; then
  omnisim_home="$(dirname "$omnisim_self_dir")"
else
  echo "omnisim: cannot find bin/omnisim-bin relative to $omnisim_self" >&2
  echo "omnisim: this launcher must stay inside the OmniSim install tree (or be symlinked to it)." >&2
  exit 1
fi

# remove wrong desktop files if needed. `webots.desktop` is the pre-rebrand name:
# drop it so an upgrading user does not end up with two entries in their app menu.
for stale in omnisim-bin.desktop webots.desktop; do
  if [ -e ~/.local/share/applications/$stale ]; then
    rm ~/.local/share/applications/$stale
  fi
done

# create a desktop file if it doesn't exist
if [ ! -e /usr/share/applications/omnisim.desktop ] && [ ! -e ~/.local/share/applications/omnisim.desktop ]; then
  mkdir -p ~/.local/share/applications
  FILE=~/.local/share/applications/omnisim.desktop
  echo "[Desktop Entry]" > $FILE
  echo "Name=OmniSim" >> $FILE
  echo "Comment=OmniSim — robot simulator for agentic AI by OmniLink" >> $FILE
  echo "Exec="$omnisim_home"/bin/omnisim" >> $FILE
  echo "Icon="$omnisim_home"/resources/icons/core/omnisim.png" >> $FILE
  echo "Terminal=false" >> $FILE
  echo "Type=Application" >> $FILE
fi

#prevent CI Warnings
if [[ -z "$XDG_RUNTIME_DIR" ]]
then
export XDG_RUNTIME_DIR="/tmp/runtime-runner"
fi

# we need this to start OmniSim from snap
if [[ ! -z "$SNAP" ]]
then
mkdir -p "$XDG_RUNTIME_DIR"
export QTCOMPOSE=$SNAP/usr/share/X11/locale
fi

if [[ -z "$TMPDIR" ]]
then
TMPDIR=/tmp
fi

# OMNISIM_TMPDIR is preferred; WEBOTS_TMPDIR is the legacy alias (both are exported below).
if [[ ! -z "$OMNISIM_TMPDIR" ]]
then
WEBOTS_TMPDIR=$OMNISIM_TMPDIR
fi
if [[ -z "$WEBOTS_TMPDIR" ]]
then
if [[ ! -z "$SNAP" ]]
then
WEBOTS_TMPDIR="$SNAP_USER_COMMON/tmp"
mkdir -p $WEBOTS_TMPDIR
else
WEBOTS_TMPDIR=$TMPDIR
fi
fi

export TMPDIR=$WEBOTS_TMPDIR
export OMNISIM_TMPDIR=$WEBOTS_TMPDIR
export WEBOTS_TMPDIR=$WEBOTS_TMPDIR

# add the "lib" directory into LD_LIBRARY_PATH as the first entry
export OMNISIM_ORIGINAL_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
export WEBOTS_ORIGINAL_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$omnisim_home/lib/webots":$LD_LIBRARY_PATH

# set the QT platform to use the X11 server for compatibility with Wayland
export QT_QPA_PLATFORM="xcb"

# Fix for i3 window manager not working with Qt6
if [ "$XDG_CURRENT_DESKTOP" == "i3" ]; then
  DPI=`xrdb -query -all | grep Xft.dpi | awk '{print $2}'`
  if [[ "$DPI" -gt 96 ]]; then
    export QT_ENABLE_HIGHDPI_SCALING=0
    export QT_SCALE_FACTOR=2
    export QT_FONT_DPI=80
  fi
# Fix for MATE desktop
elif [ "$XDG_CURRENT_DESKTOP" == "MATE" ]; then
  export QT_ENABLE_HIGHDPI_SCALING=0
else
  export QT_ENABLE_HIGHDPI_SCALING=1
fi

# execute the real OmniSim binary in a child process
if command -v primusrun >/dev/null 2>&1; then
  primusrun "$omnisim_home/bin/omnisim-bin" "$@" &
else
  "$omnisim_home/bin/omnisim-bin" "$@" &
fi

# wait for termination
omnisim_pid=$!
if [ "${term_kill_needed}" ]; then
  kill_omnisim
fi
wait ${omnisim_pid}
trap - TERM INT
wait ${omnisim_pid}

omnisim_return_code=$?

exit $omnisim_return_code
