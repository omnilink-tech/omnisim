#!/usr/bin/env bash
# Sample the machine while a smoke runs. One block every 30 s: memory, the
# engine's RSS/state, its main thread's syscall and wait channel, a histogram
# of every thread's wait channel, its open TCP sockets, its child processes
# (controllers) with their states, and the log/cache counters.
# Usage: bash scripts/dev/ci_timeline.sh <runtime-log-path> &
RL="${1:-}"; T0=$(date +%s)
while :; do
  sleep 30; t=$(( $(date +%s) - T0 ))
  eng=$(pgrep -f 'bin/omnisim-bin' | head -1)
  fa=$(free -m | awk '/Mem:/{print $7}'); sw=$(free -m | awk '/Swap:/{print $3}')
  el=$(wc -l < omnisim_log.txt 2>/dev/null || echo 0); rl=$(wc -l < "$RL" 2>/dev/null || echo 0)
  wc_=$(find ~/.cache/warp -maxdepth 2 -type d 2>/dev/null | wc -l)
  if [ -z "$eng" ]; then echo "timeline t+${t}s engine=none free=${fa}MB engine-log=$el runtime-log=$rl warp-cache=$wc_"; continue; fi
  rss=$(awk '/VmRSS/{print $2}' /proc/$eng/status 2>/dev/null || echo 0); st=$(awk '/^State/{print $2}' /proc/$eng/status 2>/dev/null)
  thr=$(awk '/^Threads/{print $2}' /proc/$eng/status 2>/dev/null)
  echo "timeline t+${t}s engine=$eng state=$st threads=$thr rss=$((rss/1024))MB free=${fa}MB swapused=${sw}MB engine-log=$el runtime-log=$rl warp-cache=$wc_"
  echo "  main: wchan=$(cat /proc/$eng/wchan 2>/dev/null) syscall=$(cut -d' ' -f1 /proc/$eng/syscall 2>/dev/null) utime=$(cut -d' ' -f14 /proc/$eng/stat)"
  echo "  thread wchans: $(for x in /proc/$eng/task/*/wchan; do cat $x 2>/dev/null; echo; done | sort | uniq -c | sort -rn | head -5 | awk '{printf "%s x%s; ", $2, $1}')"
  echo "  hottest threads (utime ticks, name): $(for d in /proc/$eng/task/*; do echo "$(cut -d' ' -f14 $d/stat 2>/dev/null) $(cat $d/comm 2>/dev/null)"; done | sort -rn | head -4 | awk '{printf "%s=%s; ", $2, $1}')"
  echo "  sockets: $(ss -tnp 2>/dev/null | grep -c "pid=$eng,") tcp; $(ss -tnp 2>/dev/null | grep "pid=$eng," | awk '{print $1, $5}' | head -3 | tr '\n' ' ')"
  echo "  children: $(ps -o pid=,stat=,etimes=,comm= --ppid $eng 2>/dev/null | awk '{printf "%s[%s,%ss,%s] ", $4, $2, $3, $1}')"
  echo "  grandchildren: $(for c in $(pgrep -P $eng); do ps -o stat=,etimes=,args= --ppid $c 2>/dev/null | cut -c1-70 | awk '{printf "[%s] ", $0}'; done)"
done
