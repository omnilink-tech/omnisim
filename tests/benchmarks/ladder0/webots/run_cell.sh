#!/bin/bash
# run_cell.sh -- run ONE ladder0 cell on upstream Webots, inside WSL.
#
# This is a FILE on purpose.  A multi-line command handed to `wsl.exe` loses
# its quoting on the Windows round trip and arrives with empty variables; that
# has cost this repo real cycles more than once.  Everything the Windows side
# needs to vary arrives as a positional argument, and `arm.py` passes them as
# separate argv entries, so nothing here has a quoting surface.
#
#   $1 rung        $2 fault       $3 out dir (WSL path)
#   $4 world file  $5 timeout s   $6 TCP port
#
# Optional, and only ever set for a MULTI-RUN rung (CONTRACT.md amendment A):
#
#   $7 run tag     $8 sample stride   $9 short fraction
#
# They are passed to the CONTROLLER as environment rather than as
# controllerArgs because rung 9's replicas `a` and `b` are deliberately the
# SAME world file -- a per-run value that reached them through the scene would
# make the two files differ, which is the one thing that rung must rule out.
# Absent, they default to "one run, every step, the whole run", which is what
# every other rung wants.
#
# T0/T1 are taken INSIDE WSL, in the same clock domain as the controller's own
# time.time(), so "process spawn -> first step" is a subtraction of two numbers
# from one clock rather than across the Windows/WSL boundary.
set -u

RUNG="$1"; FAULT="$2"; OUT_DIR="$3"; WORLD="$4"; TIMEOUT="$5"; PORT="$6"
TAG="${7:-}"; STRIDE="${8:-1}"; SHORT="${9:-1.0}"

export WEBOTS_HOME="${LADDER0_WEBOTS_HOME:-/opt/upstream-webots/R2025a}"
export PYTHONPATH="$WEBOTS_HOME/lib/controller/python"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller"
export LADDER0_OUT="$OUT_DIR/samples.json"
export LADDER0_FAULT="$FAULT"
export LADDER0_TAG="$TAG"
export LADDER0_STRIDE="$STRIDE"
export LADDER0_SHORT="$SHORT"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT_DIR"
rm -f "$LADDER0_OUT"

# Make the row self-describing: which build produced these numbers.
echo "VERSION=$(head -1 "$WEBOTS_HOME/resources/version.txt" 2>/dev/null)"
BIN="$WEBOTS_HOME/bin/webots-bin"
[ -x "$BIN" ] || BIN="$WEBOTS_HOME/webots"
echo "BINSHA=$(sha256sum "$BIN" 2>/dev/null | cut -c1-16)"
echo "RUNG=$RUNG"
echo "TAG=$TAG"

T0=$(date +%s.%N)
timeout "$TIMEOUT" xvfb-run -a "$WEBOTS_HOME/webots" \
  --batch --mode=fast --no-rendering --minimize --stdout --stderr \
  --port="$PORT" "$WORLD" > "$OUT_DIR/console.log" 2>&1
RC=$?
T1=$(date +%s.%N)

echo "RC=$RC"
echo "T0=$T0"
echo "T1=$T1"
