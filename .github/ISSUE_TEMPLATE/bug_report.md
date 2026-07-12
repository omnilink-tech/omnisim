---
name: Bug report
about: Report something in OmniSim that behaves incorrectly
labels: bug

---

**Describe the bug**
A clear and concise description of what goes wrong.

**Steps to reproduce**
1. World file / demo used (path in the repo, or attach the `.wbt`):
2. How you launched it (`launch.bat …`, `python -m omnisim run-headless …`, the harness, …):
3. What you did:
4. What you saw:

**Expected behavior**
What you expected to happen instead.

**Environment — please paste the output of `python -m omnisim doctor`**

```
(paste here — this reports the binary path, port status, worlds present, and recent commits)
```

**Physics backend**
Which backend actually drove the world? OmniSim defaults to Newton where its
runtime is present and falls back to ODE otherwise — and the fallback is easy
to miss, so a lot of "the physics is wrong" reports are really "it silently ran
on ODE". `[WbNewtonBackend] imports OK` is *not* proof; the proof line is:

```
[WbNewtonBackend] world finalised (solver=...)
```

- [ ] Newton (I saw the `world finalised` line in `omnisim_log.txt`)
- [ ] ODE
- [ ] Not sure

**System**
- Operating System: [e.g. Windows 11, Ubuntu 22.04, macOS Sonoma]
- GPU + driver: [e.g. NVIDIA RTX 5070 Ti, 561.09] — required for any Newton / RL / locomotion issue
- Built from source, or a release binary?

**Logs**
Attach or paste the relevant part of `omnisim_log.txt` (repo root). For crashes,
include the last ~50 lines.

**Screenshots**
If the issue is visual, add a screenshot.

**Additional context**
Anything else worth knowing. If the behaviour is inherited from upstream Webots
and is not specific to OmniSim, say so — it helps us route the fix.
