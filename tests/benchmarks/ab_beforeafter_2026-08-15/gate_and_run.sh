#!/usr/bin/env bash
# Wait until the machine is quiet (no OTHER agent's harness/capture listener,
# no live build), then run one arm. The cc_lane port sweep reaps any listener
# no live cell claims -- including another agent's -- so starting while the
# engine agent has a harness up would destroy their run.
set -u
ARM="$1"; SHA="$2"; N="${3:-3}"
QUIET_NEEDED=3          # consecutive quiet checks (60 s apart)
MAX_WAIT_MIN="${4:-90}"

quiet_checks=0
waited=0
while :; do
  listeners=$(netstat -ano 2>/dev/null | grep -E ':(6789|6790|6791|6792)[^0-9]' | grep LISTENING | wc -l)
  building=$(tasklist 2>/dev/null | grep -icE '^(make|cc1plus|g\+\+|ld)\.exe' || true)
  if [ "$listeners" -eq 0 ] && [ "$building" -eq 0 ]; then
    quiet_checks=$((quiet_checks+1))
  else
    [ "$quiet_checks" -gt 0 ] && echo "[gate] machine busy again (listeners=$listeners building=$building); resetting"
    quiet_checks=0
  fi
  echo "[gate] $(date -u +%FT%TZ) listeners=$listeners building=$building quiet=$quiet_checks/$QUIET_NEEDED waited=${waited}m"
  [ "$quiet_checks" -ge "$QUIET_NEEDED" ] && break
  if [ "$waited" -ge "$MAX_WAIT_MIN" ]; then
    echo "[gate] GAVE UP after ${waited} min -- machine never went quiet. NOT starting $ARM."
    exit 3
  fi
  sleep 60
  waited=$((waited+1))
done

echo "[gate] machine quiet; starting $ARM"
exec bash /o/omnisim/_scratch/ab_beforeafter/run_arm.sh "$ARM" "$SHA" "$N"
