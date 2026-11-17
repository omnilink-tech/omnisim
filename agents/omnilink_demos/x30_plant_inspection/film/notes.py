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

"""    python agents/omnilink_demos/x30_plant_inspection/film/notes.py <take_dir>

Production notes for a take: what was said, who answered, what was measured,
and every edit. Kept beside the footage, never on screen."""
import json
import os
import sys

TAKE = sys.argv[1]
ev = [json.loads(l) for l in open(os.path.join(TAKE, "events.jsonl"), encoding="utf-8")]
ed = json.load(open(os.path.join(TAKE, "edit_notes.json"), encoding="utf-8"))
out = []
w = out.append
w("# X30 plant inspection -- production notes (%s)\n" % os.path.basename(TAKE.rstrip("/\\")))
w("World: `projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld`. "
  "Robot: Deep Robotics X30 (BSD-3 model), `omnilink_quadruped_bridge --locomotion trot`.\n")
w("Every order below was sent to the bridge's real `POST /prompt`. `parser` = OmniLink's "
  "deterministic parser answered; `relay` = the OmniLink model relay answered (engine and model: "
  "the bridge defaults). The gate vetted both.\n")
w("| # | Operator | Answered by | Wall latency | Reply (verbatim) | Measured |")
w("|---|---|---|---|---|---|")
for e in ev:
    if e["event"] != "reply":
        continue
    r = e["reply"]
    lm = e.get("last_motion") or {}
    meas = ""
    if lm.get("verb") == "walk_to":
        meas = "%s: %.2f m from target, heading error %.1f deg, path %.1f m" % (
            lm.get("label"), lm.get("error_m") or 0, abs(lm.get("yaw_error_rad") or 0) * 57.2958,
            lm.get("path_m") or 0)
    elif lm.get("verb") in ("walk", "turn"):
        meas = "%s: achieved %s of %s" % (lm["verb"], lm.get("achieved"), lm.get("commanded"))
    reply = (r.get("response") or "").replace("|", "/").replace("\n", " ")
    w("| %d | %s | %s | %.1f s | %s | %s |" % (e["i"], e["text"], r.get("via"), e["wall_s"], reply, meas))
w("\n## Edits\n")
w("- Native OmniSim frames only (wgpu renderer, 1600x900, recorded every 40 ms of SIM time: "
  "plays back in real time at 25 fps). Camera: follows the robot at a fixed world-frame offset, "
  "re-aimed between orders (eased on sim time).")
w("- Time compression: %d idle frames (the robot standing while the model answered) cut; "
  "at most %.1f s of each idle stretch kept." % (ed["idle_frames_cut"], ed["idle_keep_s"]))
w("- Speed-up: walking stretches longer than %.0f s with no card on screen play at %dx after their first %.0f s (%d frames dropped)."
  % (ed["fast_after_s"], ed["fast_x"], ed["fast_after_s"] / 2, ed["fast_frames_dropped"]))
w("- Reading holds: %d frozen frames (a card still on screen after the take's footage for it ended)."
  % ed["frozen_frames"])
w("- Prompt cards %.1f s; answer cards %.1f s, shown only for questions and clarifications."
  % (ed["prompt_card_s"], ed["answer_card_s"]))
w("- Output: %d frames = %.1f s. Silent (no audio stream)." % (ed["output_frames"], ed["output_frames"] / ed["fps"]))
w("\n## What this footage does and does not show\n")
w("- The legs are real contact physics: a scripted trot through closed-form leg IK on position "
  "servos. No supervisor pin, no learned policy. Flat ground only.")
w("- Navigation reads the robot's pose from the Supervisor (standing in for leg odometry + IMU) and "
  "follows the walkway graph declared in the world. Nothing senses obstacles; there is no camera "
  "on the robot and no gauge is read.")
w("- A simulation. No hardware and no claim about the real X30's gait or speed.")
open(os.path.join(TAKE, "PRODUCTION_NOTES.md"), "w", encoding="utf-8").write("\n".join(out) + "\n")
print("\n".join(out))
