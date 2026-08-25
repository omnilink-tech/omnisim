# Upstream Webots as the control simulator (Phase W)

**2026-07-26.** How to stand up **upstream Webots R2025a** as the control arm of
the agent-edge validation programme
([agent-edge-validation-plan.md](agent-edge-validation-plan.md), Phase W), what it
does and does not ship, and the traps that will otherwise invalidate a
comparison.

Why upstream is the control: same file format, same base engine, no harness. If
an agent does as well on plain Webots as on OmniSim, everything we built on top
buys nothing and the programme stops. It is the cheapest decisive experiment we
have, and the harshest.

---

## 1. ⛔ Do NOT run the Windows installer

`webots-R2025a_setup.exe` **will damage this checkout's tooling.** Verified in
upstream's packaging source (`scripts/packaging/windows_distro.py` in the
**cyberbotics/webots** repository — note that a file of the same name exists here,
and is ours, not theirs), which emits this Inno Setup directive:

```
Root: HKA; SubKey: "SYSTEM\CurrentControlSet\Control\Session Manager\Environment";
ValueType: string; ValueName: "WEBOTS_HOME"; ValueData: "{app}"; Flags: preservestringtype
```

That is the **machine-scope** environment key, with `ChangesEnvironment=yes`, and
**no `uninsdeletevalue` flag** — so uninstalling leaves `WEBOTS_HOME` behind. That
is how a stale `WEBOTS_HOME` (Machine) pointing at a `C:\Program Files\Webots`
that no longer exists survives an uninstall, which `python -m omnisim doctor`
reports as a warning.

> ⚠️ **We do the same thing.** `scripts/packaging/windows_distro.py` in THIS repo
> writes `OMNISIM_HOME` to the same machine-scope key with `preservestringtype`
> and no `uninsdeletevalue`, so our own installer leaves the same kind of residue
> behind. This section is a hazard warning for anyone running the upstream
> installer next to this checkout — not a claim that we got it right.

Second hazard, previously undocumented: **`HKCU\Software\Cyberbotics\Webots-R2025a`
is OmniSim's own QSettings key** (it holds our theme, `omnisim_dusk.qss`, and our
RecentFiles). Upstream R2025a uses the *same* key and would overwrite the GUI
preferences of this install.

And there is **no Windows archive** to fall back on. R2025a ships
`webots-R2025a_setup.exe` (Windows), `webots-R2025a-x86-64.tar.bz2` (**Linux**),
`.deb`, `.snap`, `.dmg`, plus `assets-R2025a.zip` (a texture/PROTO pack, not the
program). Unpacking Inno 6.4 needs a third-party tool (`innoextract` 1.9 is from
2020 and too old).

## 2. The install that is safe: the Linux archive, in WSL2

WSL2 is a separate OS instance with its own network namespace, so the Windows
environment, registry and TCP ports are structurally untouchable.

```bash
# in WSL2 (Ubuntu 22.04)
sudo mkdir -p /opt/upstream-webots && cd /opt/upstream-webots
# download the archive from cyberbotics.com and place it here
sudo tar xjf webots-R2025a-x86-64.tar.bz2      # -> /opt/upstream-webots/R2025a  (460 MB)
# runtime deps: use upstream's own list, scripts/install/linux_runtime_dependencies.sh, + xvfb
```

**`WEBOTS_HOME` is set per-process only** — never in `.bashrc`, `.profile`,
`/etc/environment` or `/etc/profile.d/`. Verify with `grep` before believing it.

Upstream publishes **no checksums** for its release assets. Computed here:

| asset | sha256 |
|---|---|
| `webots-R2025a-x86-64.tar.bz2` | `c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38` |
| `webots-R2025a_setup.exe` | `9e326a54c104fc5fc88121e26014a409d1e35f0bbf30e23f3a712e7f842b08e7` |

Source: `https://github.com/cyberbotics/webots/releases/download/R2025a/<asset>`.
R2025a, Apache-2.0, published **2025-02-04** (tag commit `234898f4`).

## 3. Headless invocation, and the two things it lacks

Every flag our own contract uses exists upstream. Two do not:

- **no `--duration`** and **no auto-exit** — a run terminates only when an
  in-world `Supervisor` calls `simulationQuit()`, or when something kills it.
- **`--log-performance` only flushes in `worldClosed()`**
  (`OmPerformanceLog.cpp:123-150`), so a *killed* run leaves an **empty** perf
  log. A benchmark that kills the process gets no data and no error.

Working, verified invocation (exit code 0 = a clean quit, not a kill):

```bash
WEBOTS_HOME=/opt/upstream-webots/R2025a \
timeout 420 xvfb-run -a /opt/upstream-webots/R2025a/webots \
  --batch --mode=fast --no-rendering --minimize --stdout --stderr \
  --log-performance=/path/perf.txt,1500 --port=1504 <world.omniworld>
```

Proof it stepped physics on a ten-robot world built from upstream assets:
`117 solids, 80 joints`, `1500 steps`, `sim_time=6.00 s`, `finite=10/10`,
`moved=10`, total displacement 155.01 m (mean 15.50 m/robot), ~90 steps/s wall
including load, average speed factor 0.839. Separately, upstream's own shipped
`moose_demo.wbt` drove its full waypoint circuit for multiple laps unmodified.

## 4. ⚠️ Upstream ships NO Husky — the flagship task must be re-expressed

Checked all **15,404 paths** in the R2025a tag: **zero** matches for `husky`
(also none for jackal / warthog / ridgeback). `projects/robots/clearpath/`
contains **heron** (USV), **moose** and **pr2** only. Our `husky_description/`
arrived with *our* URDF importer (`6a342fc2`) — it was never upstream's.

So "ten Huskies" is not a portable task. Either the asset is pre-converted with
upstream's own `urdf2webots` and published alongside the benchmark, or the task
is re-expressed by **outcome** ("ten four-wheeled UGVs moving randomly") with each
simulator using its native robot. Fair analogues upstream:

| robot | why |
|---|---|
| **Pioneer 3-AT** (adept) | best kinematic match: 4-wheel skid-steer outdoor, `front/back left/right wheel` motors, ships a working obstacle-avoidance controller |
| **Clearpath Moose** | best brand/class match (same vendor, outdoor UGV) but **8**-wheel; ships `moose_path_following` (GPS+Compass, CPU-only) |

Either way, **publish the asymmetry rather than hide it**: we import URDF
natively, upstream needs a conversion step. That is a real difference and it cuts
in our favour, which is exactly why it has to be stated rather than buried.

What upstream *does* give the agent for free: the robot PROTO and a working
reactive/path-following controller. What it does **not**: a random-walk
controller (~35 lines) or any measurement/termination harness (a Supervisor).

## 5. Three traps that will invalidate a run

1. **The distribution ships ZERO `.proto` files.** Every world `EXTERNPROTO`s
   from `raw.githubusercontent.com` into `~/.cache/Cyberbotics`, so worlds are
   **network-dependent on first load** — directly against our own rule that
   benchmark worlds be local-asset-only. Pre-seed with `assets-R2025a.zip`
   (692 MB) and run offline, or a network hiccup becomes a benchmark result.
2. **Lidar is GPU-bound.** Ten `SickLms291` could not complete 1500 steps in
   600 s; the GPU-free GPS+Compass pair does it in ~16 s. Sensor choice, not
   simulator speed, will dominate any careless comparison.
3. **A `rotation` axis-angle of exactly ±π** made one Moose go non-finite within
   10 steps, reproducibly, under *both* controllers. Nudge yaw off ±π. Do not
   blame a controller for that one.

## 6. Agent surface: there is no `simulation_interfaces`

**Zero** matches in the R2025a tree — confirming the earlier survey. Upstream
has: extern controllers via the `webots-controller` launcher
(`--protocol=ipc|tcp`, `--ip-address`, `--port`, `--robot-name`), `--stream`
(w3d/mjpeg), robot windows on the same TCP port, `--extern-urls`, `--heartbeat`,
`--log-performance`, and a `convert` subcommand. No HTTP/JSON scene authoring, no
hot reload, no scene-tree query, no structured load diagnostics, and no ROS 2
bridge of its own in this tree (OmniSim's is `packages/omnisim-ros2/`; upstream's is the
separately maintained `webots_ros2`). **OmniSim's harness loop has no upstream counterpart** — which is
precisely the thing Phase W exists to price.

### The port-1234 collision is live, not theoretical

Upstream's default TCP port is **1234** — the same default OmniSim uses, and an
`omnisim-bin.exe` was observed LISTENING on it during this work. Separation, in
order of reliability: (a) WSL2's network namespace — a WSL process bound
`0.0.0.0:1234` successfully *while* a Windows process held 1234; (b) pass
`--port` outside OmniSim's `1234-1244` auto-scan range (1500/1502/1504 were
used); (c) never pin OmniSim's `--port` into the Webots range.

## 7. Maintenance state (verified, not inherited)

Latest **stable** is R2025a, 2025-02-04 — ~17.7 months old at time of writing.
R2025b exists **only** as nightly prereleases, never tagged stable.
`stats/participation`: **48 commits in the last 52 weeks, with 32 of those 52
weeks at zero** (the last 13 weeks hold 21, and 14 of those landed in a single
week). 4,473 stars, 2,032 forks, 229 open issues, not archived. Alive, but
low-cadence — relevant because OmniSim inherits this codebase's ecosystem.

## 8. Caveat that must appear in any published Phase W number

The comparison as installed is **cross-platform**: WSL2 Linux Webots versus
native Windows OmniSim, same CPU and GPU. WSL2 CPU performance is near-native,
but this is not same-platform parity and the writeup must say so rather than
imply it. A same-platform comparison requires either a Windows-native Webots
install (see §1 — currently refused on the grounds that it breaks this checkout)
or running OmniSim under Linux too.
