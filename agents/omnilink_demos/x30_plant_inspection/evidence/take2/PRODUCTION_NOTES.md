# X30 plant inspection -- production notes (take2)

World: `projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld`. Robot: Deep Robotics X30 (BSD-3 model), `omnilink_quadruped_bridge --locomotion trot`.

Every order below was sent to the bridge's real `POST /prompt`. `parser` = OmniLink's deterministic parser answered; `relay` = the OmniLink model relay answered (engine and model: the bridge defaults). The gate vetted both.

| # | Operator | Answered by | Wall latency | Reply (verbatim) | Measured |
|---|---|---|---|---|---|
| 1 | Walk forward 2 metres. | parser | 52.1 s | Walk (distance=2.0). | walk: achieved 2.022819996477233 of 2.0 |
| 2 | Head over to the feed pump and check on it. | relay | 206.5 s | I've arrived at feed pump P-101. My measured position is [2.51, -5.83] meters, and I am standing facing the pump and its discharge pressure gauge. | pump P-101: 0.07 m from target, heading error 0.3 deg, path 11.6 m |
| 3 | Go to the tank. | relay | 6.2 s | Did you mean storage tank T-1 at the north-west side or storage tank T-2 at the north-east side? | pump P-101: 0.07 m from target, heading error 0.3 deg, path 11.6 m |
| 4 | Tank T-2, please. | relay | 416.9 s | I have arrived at storage tank T-2. My measured position is [3.51, 4.83] meters, and I am facing the tank and its level gauge. | tank T-2: 0.07 m from target, heading error 0.3 deg, path 17.5 m |
| 5 | Now go to valve manifold V-7. | relay | 107.3 s | Stopped. I cancelled the rest of that request because you told me to stop; tell me what to do next. | None: 0.00 m from target, heading error 0.0 deg, path 0.0 m |
| 6 | Stop. | parser | 13.0 s | Stopped - measured 0.000 m/s over 0.50 s, so it is standing still. |  |
| 7 | Where are you? | relay | 5.3 s | I am currently at coordinates [2.97, 3.94] meters, facing west on the walkway. |  |
| 8 | Change of plan: inspect tank T-1, then return to the dock. | relay | 581.6 s | I have completed the inspection. I visited storage tank T-1, measuring my position at [-3.51, 4.83] meters, and I am now back at the charging dock at [-9.97, -5.01] meters. | charging dock: 0.07 m from target, heading error 0.4 deg, path 17.4 m |

## Edits

- Native OmniSim frames only (wgpu renderer, 1600x900, recorded every 40 ms of SIM time: plays back in real time at 25 fps). Camera: follows the robot at a fixed world-frame offset, re-aimed between orders (eased on sim time).
- Time compression: 375 idle frames (the robot standing while the model answered) cut; at most 1.0 s of each idle stretch kept.
- Speed-up: walking stretches longer than 12 s with no card on screen play at 2x after their first 6 s (1248 frames dropped).
- Reading holds: 0 frozen frames (a card still on screen after the take's footage for it ended).
- Prompt cards 3.0 s; answer cards 4.5 s, shown only for questions and clarifications.
- Output: 4395 frames = 175.8 s. Silent (no audio stream).

## What this footage does and does not show

- The legs are real contact physics: a scripted trot through closed-form leg IK on position servos. No supervisor pin, no learned policy. Flat ground only.
- Navigation reads the robot's pose from the Supervisor (standing in for leg odometry + IMU) and follows the walkway graph declared in the world. Nothing senses obstacles; there is no camera on the robot and no gauge is read.
- A simulation. No hardware and no claim about the real X30's gait or speed.

## Music version

`x30_plant_inspection_draft_music.mp4`: the same cut, video stream copied untouched, with an original synthesized ambient bed (`film/music.py`: pads, sub-bass, sparse pluck; 4 s fade-in, 6 s fade-out; -23.9 LUFS integrated, peak -13 dB). No sound effects. An owner-approved exception to the silent OmniLink demo style; the silent cut remains the version for muted-autoplay feeds.
