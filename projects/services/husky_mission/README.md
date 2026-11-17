# Husky Mission Agent · 1.0.0

An OmniLink agent package built and tested in OmniSim. Ask it to visit station A,
then B, then return to base. The robot must physically arrive and stop before a
station is counted. Cancellation and independent typed Stop interrupt the route.

## Prepared Windows delivery

Extract the whole ZIP to a short writable directory (for example `C:\robot-agent`).
No separate Python installation is needed. First check the supplied evidence:

```powershell
.\mission.cmd verify
```

Open `report/report.html` for the measured report, charts and every case. The
delivery retains 27 qualification attempts: 26 valid passes and one controller
startup-invalid attempt with no mission executed. Its cause was not identified;
the same case passed on an unchanged rerun. It is retained, not counted as a pass.

Connect your own OmniKey and active model provider through the normal OmniLink
setup: https://github.com/omnilink-tech/omnisim/blob/main/docs/guide/omnilink-chat-demos.md
Make `OMNI_KEY` available to this process using your normal secret manager or
private environment setup. No account or provider credits are included.

```powershell
.\mission.cmd accept --out runs\acceptance-01
.\mission.cmd run --case case-003 --capture --out runs\live-route-01
.\mission.cmd suite --out runs\suite-01
```

To operate the agent interactively, with your OmniKey available in the caller's
environment, run `.\launch-agent.cmd`. It opens the native OmniSim world and its
OmniLink text interface. Ask: “Visit station A, then station B, and return to
base.” Use “Cancel the mission now.” or the independent typed Stop control.
The source add-on uses the public OmniSim launch command below.

`accept` runs a fresh typed route, live AI route, cancellation, Stop and a rejected
informational tool proposal. `suite` runs all 26 prespecified cases. Every output
directory must be new. Exit 0 means PASS, 1 means a valid simulated FAIL, and 2
means invalid/error. Keep failed evidence and investigate it. Repeatability means
passing the same task criteria, not identical AI wording, latency or trajectories.

## Public OmniSim source add-on

Use the public OmniSim 9.2.7 source checkout with its working engine/runtime.
Windows has the only prebuilt package; other platforms build from source. Extract
the source add-on at the checkout root. It includes the mission and bridge source,
Husky assets, world, and CLI; it does not include the engine or frozen evidence.

```powershell
python -m omnisim doctor
python mission.py run --case case-003 --out .tmp\mission-live-01
python mission.py accept --out .tmp\mission-acceptance-01
```

For the engine-free checks, with pytest installed in your development Python:
`python -m pytest tests/test_husky_mission_service.py -q`. These include invented
route, forged arrival, false completion, stale telemetry and cancellation races.

Use the customer guide to launch an interactive world:

```powershell
python -m omnisim run-world projects/samples/demos/worlds/services/husky_mission_agent_visible.omniworld
```

The visible demo world uses three raised, non-colliding station pads. Qualification
uses the original world; physics and controller settings are identical. Base is
green, station A blue, and station B gold.

The native OmniLink window and `/prompt` use verified mission responses. Ordinary
typed Stop remains available independently of the AI connection. See SCOPE.md
for acceptance thresholds and excluded capabilities. This is independent work,
not a Clearpath product or endorsement.
