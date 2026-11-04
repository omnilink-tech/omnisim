# Crew order: one order, three robots

Sam, the shift lead, gives one sentence to one robot: *"Order 42 is one red,
one blue and one green part, packed in the shipping crate. Work it out with
Tug and Packer, and tell me when it's packed."* After that, nothing else is
sent to any robot. The three robots each run their own OmniLink agent and work
the job out by talking to each other.

| Robot | Body | Job |
|---|---|---|
| Picker | OmniArm 6, vacuum | parts shelf → cart |
| Tug | OmniTug 500 | tows the cart between two pads |
| Packer | OmniArm 6, vacuum | cart → shipping crate |

**Take 5 (2026-10-20, Claude Opus 5.5, g4-engine):** 8 robot-to-robot messages,
order packed 141 simulated seconds after Sam spoke. The final scan has all three
parts in the crate, each within 3 mm of its slot. The arms cannot see the cart,
so the tug tells them where it left it, and Picker passes the measured part
positions on.

**Take 6 (2026-10-20), on the new tug and cart:** 10 messages, packed 128
simulated seconds after the order. Draft 4 is cut from it.

**Take 9 (2026-10-20), all props meshed, a frame every 64 ms:** 11 messages,
packed 118 simulated seconds after the order. Packer read the parts' old
position, and Tug corrected it as stale before delivering. Draft 5 is cut from it.

## Models

- **OmniTug 500**: rebuilt as Blender-scripted meshes
  (`projects/robots/omnisim/omnitug500/meshes/build_meshes.py`). Until this
  change its primitive body was laid out along +X, while the bridge and every
  OmniTug controller drive it along +Y, so it drove sideways in every world. The
  new body faces +Y. Its footprint and its tow pin, 0.63 m behind centre, are
  unchanged.
- **Tugger cart**: `projects/objects/factory/tugger_cart/meshes/`, same method.
  It keeps the trolley's contract: deck top 0.325 m, tow eye at local
  (0.70, 0, 0.25). It still collides as one box. Its casters roll along the tow
  axis; the first build had them a quarter turn off.
- **Kitting props**: `projects/objects/factory/kitting/meshes/`, same method: the
  workbenches, the tote and the 50 mm parts. They are visual only; the world
  keeps its box colliders at the old sizes.
- **Set**: epoxy floor, lane lines, a back wall, and static pallets, crates and
  boxes from `projects/objects/factory/`.

## How it works

- `omnisim_bridges/crew.py`: the `message_robot` tool. A world declares its crew
  with `OMNILINK_CREW="Picker=8765,Tug=8766,Packer=8767"`. A message is delivered
  to the crewmate's own `/prompt` as `"Picker: …"`. It always goes to the
  crewmate's model, never the parser (the parser would answer the sender, who
  never hears that answer), and the crewmate's safety gate vets whatever it then
  does. Delivery is fire-and-forget, so two robots messaging each other cannot
  deadlock. There is a per-robot rate cap.
- `world.py`: the set. The layout is measured, not guessed: `probe_reach.py`
  runs the whole job through typed `/tool` calls with no model, and only passed
  once the shelf and crate sat on each arm's +x side and the cart drops were
  kept to the reachable band.
- `take.py`: hitches the cart, gives each robot one briefing from Sam (its job
  and the floor facts), gives the order, then only watches. Everything lands in
  `<take>/crew.jsonl` with simulator time. By default the briefing is STANDING
  (`--briefing standing`): it joins each robot's own task text through
  `OMNILINK_CREW_BRIEFINGS`, which is cached, instead of costing a model turn
  per robot per run. `--briefing prompt` has Sam say it as a chat turn, as
  the first takes did.
- `edit.py`: picture on the left, crew channel on the right. Every line is
  verbatim; "…" abridgements go through `abridge.json` and are checked. The cut
  follows the work: wide while the tug moves, the close shot of the pad it is
  parked at otherwise. Anything moving plays at one speed (2×). Motion is
  measured from the frames, so no robot looks fast in one shot and slow in the
  next. Still stretches, while the agents think, are skipped, and slow down only
  to give a new line its reading time. A corner chip says which is playing.

## Physics of the tow (since 2026-10-20)

The cart is a free dynamic body. The tug's bridge velocity-servos it onto the
trailer pose behind the tow pin, under an acceleration limit; nothing writes
its pose. The parts are never pinned: they ride the deck on contact and
friction. The cart states low friction (it rolls on casters), and the parts
state 0.6. Measured over the 3.6 m tow (`probe_reach.py`): each part slips
about 2 cm on the deck. The tug ramps its own speed at 1.5 m/s^2.

## Many runs: `campaign.py` and `montage.py`

`campaign.py` runs the whole order N times, K in parallel. Each run is
independent: its own engine, its own ports, its own agent identity and memory
(`OMNILINK_AGENT_TAG`). Each run is judged by `take.judge()`. It passes only if
the final scan measures all three parts in the crate, Sam got a report, and no
message failed to deliver. Gate refusals are listed per run. A crashed run is
recorded, never dropped. `--model` picks the model. The reported campaign
runs on Sonnet 5.5, measured at about $0.60 per run, against about $1.30 on
Opus 5.5. `montage.py` plays 16 runs side by side at a constant
speed, marks each with its verdict, and closes on `summary.json`.

## What it does not show

- Simulation only.
- The tow is a velocity servo, not a modelled drawbar joint.
- The vacuum grasp is a weld.
- Each robot was briefed once before the order. Those briefings are in the log
  and not in the film.
- Takes 1–4 failed, each for a reason that was fixed before the next:
  - the setup hitch failed (the tug's forward axis is its local +Y);
  - the parser answered a crewmate;
  - the gate refused "…tell me when it's ready" as a deferred order — a real
    false refusal, fixed in `gate.py` with tests;
  - the arm had no IK solution for the first shelf layout.
- Take 10 failed: a repeated request made Tug restart its drive, which then
  stopped short. A second order to the same destination now JOINS the drive
  under way. The gate also refused "…message Packer directly once it's parked"
  (a named recipient).
- In the first 10 campaign runs on the fixed code, run 6 failed. The gate read
  "…measured after release. Please hitch it…" as a deferred hitch. A wait now
  defers only the action named in its own sentence.
- The next campaign's run 15 failed. "I'll load them once you confirm it's
  unhitched" was read as a deferred unhitch: the verb sat inside the waiting
  clause, as its condition. A verb now counts only outside the waiting clause.
- Showcase take 12 failed while campaign 4 was running clean (11 of 11).
  Tug's model chose an unhitch that the message never named, and the old
  fallback held "After release I measured…" against it. Now an unnamed tool is
  judged strictly only when no sentence free of any wait states an order (an
  imperative, or "please").
- Campaign 5's run 7 failed. "Red, blue and green are still reported aboard"
  (passive voice) was read as reported speech. The question and
  reported-speech rules are now sentence-scoped the same way as deferral.
  Not scoping the prohibition rule was a deliberate choice: "You may no longer
  drive into the bay. Drive forward 1 metre." must stay refused. Scoping it was
  tried and reverted. The crew tool now tells robots not to describe state
  with "no longer" or "don't" in a message that asks for action.
- Campaign 5's run 8 was an engine start failure (the arm controllers never
  paired with the engine). The campaign now marks such runs and redoes them
  on a resumed pass, and the summary counts them.
- Campaign 6 got 31 of 33 runs through. Two problems ended it:
  - It hit the account's 100-agent limit (one agent tag per run; slots now
    reuse tags).
  - Run 32 failed on a place name: "on the pick pad … when I let it go" held a
    wait against a pick. Verbs inside place names ("pick pad", "loading bay")
    no longer count as the action. The same fix exposed a Python regex whose
    `` had been mangled into backspace characters.
- Those findings changed the crew channel itself. A message is now a REQUEST
  (one imperative sentence, the only part the crewmate's gate judges) plus
  DETAILS (facts, delivered to the crewmate's model as liveContext). The gate
  now sees what was asked, not a paragraph of description.
- The reported campaign (`campaign_sonnet`, Sonnet 5.5, 8 in parallel)
  stopped at run 80, when the test account's API credit ran out. All 79 runs
  that finished before that passed: parts measured in the crate, Sam told,
  no delivery errors, $37.31 in all. Run 80 had already packed and reported
  when two closing replies were refused for credit. Run 87 was an engine
  start failure. Runs 81-100 never finished. In run 14 the gate refused a
  `drive_forward` with a distance nobody had stated (`invented_magnitude`);
  Tug went by the named place instead. `summary_partial.json` holds those
  counts, and the montage card says all of it. `campaign.py` now treats
  provider credit exhaustion like the agent quota (an account fault, redone
  on resume). The film's showcase is take 14: Opus 5.5, with spoken
  briefings, recorded after the physical tow landed. Its footnote names
  Opus, and the card names Sonnet. The first montage cut ended run 7's tile
  at 0:08 on an early progress note to Sam. A tile now ends at the completion
  report, by the same rule as the verdict (`take.report_time`).
- Every gate fix is mirrored in OmniLink's `api/_robot-gate.ts` through the
  parity fixture. No existing parity verdict changed. Campaigns 1-6
  (`campaign_v1..v6`) are kept as evidence. The reported campaign is
  `campaign_sonnet`.
- Campaign 7 was interrupted when the session that ran it ended
  (`campaign_v7_interrupted`). Cost then moved the campaigns to Sonnet 5.5,
  with standing briefings. Sonnet pilot 3 passed 2 of 4 runs, and both
  failures were the briefing's fault. Picker's briefing said "Packer at
  shipping unloads the cart" but never said who takes the cart there. Picker
  asked Packer to unload a cart still at the pick pad, and nobody asked Tug
  (run 1, timed out). In run 2 the "packed" report went Packer → Picker →
  Sam, and the take ended while Picker was still writing it. Now Picker asks
  Tug to take the loaded cart to shipping, Packer reports to Sam itself, and
  a packed run that has not yet been reported waits 300 s of silence before
  the take ends (it was 120). Pilot 4 then passed 4 of 4, at about 3.6
  minutes and $0.60 per run. Opus had covered the same gap by inference.
  The crew tool also gained CLOSE THE LOOP: whoever finishes a request
  tells whoever asked.
- Take 7 died on a bug in the take script (a cp1252 console). Take 8 was
  refused by the gate again: "…I'll load it after that" (the speaker's own plan)
  read as a deferred order. That was a second real false refusal, also fixed in
  `gate.py` with tests.

Video files are not committed: `social/youtube_videos/captures/crew_order/`.
