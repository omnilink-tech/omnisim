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

**Physics — did Newton actually drive the world?**
Newton/MuJoCo is the *only* physics backend (ODE was deleted on 2026-08-08).
There is no fallback and no degraded mode: if the Newton Python runtime
(`newton` / `warp` / `mujoco`) will not come up, the world has **no physics at
all** — bodies stand still and nothing falls. That is a broken install, not a
world bug, so please check it before filing: `python -m omnisim doctor`
(add `--fingerprint` for Newton-runtime presence + GPU).

The authoritative verdict is the sidecar the engine writes next to its log at
world-finalise, `omnisim_log.txt.newton.json`:

```json
{"backend":"newton","degraded":false,"finalised":true,"runtime":{...},"solver":"..."}
```

Its mere presence means Newton finalised **this** run, and `runtime` names the
newton / warp / mujoco versions and the model device that produced your result.
Please paste it. (`[OmNewtonBackend] imports OK` is *not* proof — it only says
the runtime loaded. The log-scrape fallback, if you have no file log, is the
line `[OmNewtonBackend] world finalised (solver=...)`.)

- [ ] Sidecar present, `"finalised":true` (pasted above)
- [ ] No sidecar — the run may never have reached finalise (was it long enough?)
      or the Newton runtime did not come up
- [ ] Not sure

**Solver** — which one did the world run on? This is what actually varies:

- [ ] `newtonSolver ""` / `"auto"` / `"mujoco"` — reference CPU `mj_step` (the default)
- [ ] `newtonSolver "mujoco_warp"` — the same solver batched on the GPU (not
      run-to-run reproducible; some device paths decline on it)
- [ ] `newtonSolver "mujoco+vbd"` or `"vbd"` — a world with cloth / soft bodies
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
