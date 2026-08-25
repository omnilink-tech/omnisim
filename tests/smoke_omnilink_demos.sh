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

# Smoke-test every omnilink_*.wbt demo:
#   1. Launch omnisim-bin headless on the world.
#   2. Poll the bridge until /list_robots responds (or 30 s timeout).
#   3. POST a representative prompt; verify the bridge dispatches it.
#   4. Kill omnisim-bin.
#
# Run from the repo root:
#     bash tests/smoke_omnilink_demos.sh

set -u

WEBOTS=./msys64/mingw64/bin/omnisim-bin.exe
WORLDS_DIR=projects/samples/demos/worlds/chat
RESULT=tests/omnilink_smoke_result.txt

WORLDS=(
  # robot world                            prompt
  "tb3_burger    omnilink_tb3_burger.omniworld    forward 1 m"
  "tb3_waffle    omnilink_tb3_waffle.omniworld    forward 1 m"
  "tb3_waffle_pi omnilink_tb3_waffle_pi.omniworld forward 1 m"
  "husky         omnilink_husky.omniworld         forward 1 m"
  "jackal        omnilink_jackal.omniworld        forward 1 m"
  "rosbot        omnilink_rosbot.omniworld        forward 1 m"
  "rosbot_xl     omnilink_rosbot_xl.omniworld     forward 1 m"
  "omniquad          omnilink_omniquad.omniworld          wave hello"
)

PASS=0
FAIL=0
FAILED_LIST=()
: > "$RESULT"

for entry in "${WORLDS[@]}"; do
  read -r id world prompt <<<"$entry"
  echo
  echo "================================================================"
  echo "[$id] world=$world prompt=$prompt"
  echo "================================================================"

  # Launch OmniSim headless.
  "$WEBOTS" --port=1235 --no-rendering --batch --mode=fast --minimize \
    "$WORLDS_DIR/$world" >/tmp/omnilink_smoke_$id.log 2>&1 &
  WPID=$!

  # Poll /list_robots up to 30 s.
  ready=0
  for _ in $(seq 1 30); do
    sleep 1
    body=$(curl -s -m 1 -X POST http://127.0.0.1:8765/list_robots 2>/dev/null)
    if [[ -n "$body" ]] && echo "$body" | grep -q '"id"'; then
      ready=1
      break
    fi
  done

  if [[ $ready -eq 0 ]]; then
    echo "FAIL: bridge never came up"
    echo "$id FAIL bridge_never_up" >> "$RESULT"
    FAILED_LIST+=("$id")
    FAIL=$((FAIL+1))
    kill -9 $WPID 2>/dev/null
    wait $WPID 2>/dev/null
    # let socket close
    sleep 1
    continue
  fi

  echo "  /list_robots OK"

  # POST a prompt and check actions array is non-empty (or response is informative).
  prompt_resp=$(curl -s -m 3 -X POST -H 'Content-Type: application/json' \
    -d "{\"text\": \"$prompt\"}" http://127.0.0.1:8765/prompt 2>/dev/null)
  if echo "$prompt_resp" | grep -q '"response"'; then
    if echo "$prompt_resp" | grep -q '"actions"\s*:\s*\[\s*{'; then
      echo "  /prompt OK -> $(echo "$prompt_resp" | head -c 120)"
      echo "$id PASS $prompt_resp" >> "$RESULT"
      PASS=$((PASS+1))
    else
      echo "  /prompt response with no actions: $prompt_resp"
      echo "$id FAIL no_actions $prompt_resp" >> "$RESULT"
      FAILED_LIST+=("$id")
      FAIL=$((FAIL+1))
    fi
  else
    echo "  /prompt FAIL: $prompt_resp"
    echo "$id FAIL prompt_failed $prompt_resp" >> "$RESULT"
    FAILED_LIST+=("$id")
    FAIL=$((FAIL+1))
  fi

  # Kill OmniSim and wait for port to free.
  kill -9 $WPID 2>/dev/null
  wait $WPID 2>/dev/null
  sleep 2
done

echo
echo "================================================================"
echo "RESULTS: $PASS pass, $FAIL fail"
if [[ ${#FAILED_LIST[@]} -gt 0 ]]; then
  echo "Failed: ${FAILED_LIST[*]}"
fi
echo "Detail: $RESULT"
echo "Per-run logs: /tmp/omnilink_smoke_*.log"
