#!/usr/bin/env bash
# Live OmniLink-mode smoke test:
#   1. Launch omnisim-bin headless with OMNI_KEY set
#   2. POST a representative prompt to /prompt — bridge routes through OmniLink
#   3. Verify response is non-empty AND has tool actions matching the intent
#
# Requires OMNI_KEY in env. Run from repo root:
#     OMNI_KEY=olink_... bash tests/smoke_omnilink_demos_live.sh

set -u

if [[ -z "${OMNI_KEY:-}" ]]; then
  echo "ERROR: OMNI_KEY is not set."
  exit 1
fi

WEBOTS=./msys64/mingw64/bin/omnisim-bin.exe
WORLDS_DIR=projects/samples/demos/worlds/chat
RESULT=tests/omnilink_smoke_live_result.txt

# Wait until port 8765 is no longer responding. Prevents stale bridges
# from a prior run (or killed-mid-test) from poisoning the next world.
wait_for_port_free() {
  local timeout=$1
  for _ in $(seq 1 $timeout); do
    if ! curl -s -m 1 -o /dev/null -X POST http://127.0.0.1:8765/list_robots 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# id | world | prompt | expected-tool-regex
WORLDS=(
  "tb3_burger|omnilink_tb3_burger.wbt|drive forward one meter|drive_forward"
  "tb3_waffle|omnilink_tb3_waffle.wbt|drive forward one meter|drive_forward"
  "tb3_waffle_pi|omnilink_tb3_waffle_pi.wbt|drive forward one meter|drive_forward"
  "husky|omnilink_husky.wbt|drive forward one meter|drive_forward"
  "jackal|omnilink_jackal.wbt|drive forward one meter|drive_forward"
  "rosbot|omnilink_rosbot.wbt|drive forward one meter|drive_forward"
  "rosbot_xl|omnilink_rosbot_xl.wbt|drive forward one meter|drive_forward"
  "spot|omnilink_spot.wbt|sit down|sit"
)

PASS=0
FAIL=0
FAILED=()
: > "$RESULT"

for entry in "${WORLDS[@]}"; do
  IFS='|' read -r id world prompt expected <<<"$entry"
  echo
  echo "================================================================"
  echo "[$id] world=$world prompt=\"$prompt\" expected_tool~=$expected"
  echo "================================================================"

  # Make sure no prior bridge is still on the port.
  if ! wait_for_port_free 5; then
    echo "  WARN: port 8765 still occupied; bailing on this world"
    echo "$id FAIL port_blocked" >> "$RESULT"
    FAILED+=("$id"); FAIL=$((FAIL+1))
    continue
  fi

  OMNI_KEY="$OMNI_KEY" "$WEBOTS" --port=1235 --no-rendering --batch --mode=fast --minimize \
    "$WORLDS_DIR/$world" >/tmp/omnilink_live_$id.log 2>&1 &
  WPID=$!

  ready=0
  for _ in $(seq 1 40); do
    sleep 1
    body=$(curl -s -m 1 -X POST http://127.0.0.1:8765/list_robots 2>/dev/null)
    if [[ -n "$body" ]] && echo "$body" | grep -q '"id"'; then
      ready=1; break
    fi
  done
  if [[ $ready -eq 0 ]]; then
    echo "  FAIL: bridge never came up"
    echo "$id FAIL bridge_never_up" >> "$RESULT"
    FAILED+=("$id"); FAIL=$((FAIL+1))
    kill -9 $WPID 2>/dev/null; wait $WPID 2>/dev/null
    continue
  fi
  echo "  /list_robots OK"

  # Verify we hit the RIGHT robot (sanity check against stale bridges).
  actual_id=$(curl -s -m 2 -X POST http://127.0.0.1:8765/list_robots 2>/dev/null | grep -oE '"id": "[^"]+"' | head -1 | sed 's/"id": "\(.*\)"/\1/')
  if [[ "$actual_id" != "$id" ]]; then
    echo "  FAIL: wrong bridge on port (saw '$actual_id', expected '$id')"
    echo "$id FAIL wrong_bridge_$actual_id" >> "$RESULT"
    FAILED+=("$id"); FAIL=$((FAIL+1))
    kill -9 $WPID 2>/dev/null; wait $WPID 2>/dev/null
    continue
  fi

  prompt_resp=$(curl -s -m 120 -X POST -H 'Content-Type: application/json' \
    -d "{\"text\": \"$prompt\"}" http://127.0.0.1:8765/prompt 2>/dev/null)

  ok=0
  if echo "$prompt_resp" | grep -q '"response"'; then
    if echo "$prompt_resp" | grep -qE "\"tool\": \"($expected)\""; then
      ok=1
    fi
  fi

  if [[ $ok -eq 1 ]]; then
    echo "  PASS: $(echo "$prompt_resp" | head -c 180)"
    echo "$id PASS $prompt_resp" >> "$RESULT"
    PASS=$((PASS+1))
  else
    echo "  FAIL: $prompt_resp"
    echo "$id FAIL $prompt_resp" >> "$RESULT"
    FAILED+=("$id"); FAIL=$((FAIL+1))
  fi

  kill -9 $WPID 2>/dev/null
  wait $WPID 2>/dev/null
  # Wait for the bridge to actually release the port before we move on.
  wait_for_port_free 10 || echo "  WARN: port still busy after kill"
done

echo
echo "================================================================"
echo "RESULTS (OmniLink mode): $PASS pass, $FAIL fail"
if [[ ${#FAILED[@]} -gt 0 ]]; then
  echo "Failed: ${FAILED[*]}"
fi
echo "Detail: $RESULT"
