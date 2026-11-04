# Run your {{LABEL}} assessment

Unzip the complete Windows package into a **short local writable folder**, such
as a RobotKit folder directly under your Windows user profile. You may rename
the outer package folder. Keep its internal directories together. The full
resolved package-folder path must be at most 100 characters; nested AppData
folders can redirect to longer paths and exceed the engine's Windows path limit.
The launcher refuses an unsupported deep install with a clear message.
No Git, separately installed Python, network connection,
cloud account or API key is required. Do not run from inside the ZIP viewer.

Open REPORT.html for the findings and SCOPE.md for the exact US$100 scope.
The evidence folder preserves the delivered measurements. New runs go into new
folders; an existing output folder is refused rather than overwritten.

Open PowerShell in the extracted folder:

```powershell
.\assessment.cmd verify
.\assessment.cmd accept --out runs\acceptance-01
```

`verify` checks every inventoried delivery file, including runtime and sources,
and regrades all 64 saved episodes. It does not launch the simulator. It can
take a little time because it hashes the included runtime.

`accept` first verifies the delivery, then launches four fresh simulator runs:
two reproductions each of one recorded failing/passing braking pair. It writes
acceptance.json and full per-run evidence under the specified output directory.
Exit 0 means acceptance succeeded, including reproduction of the expected FAIL.
Exit 2 means acceptance or evidence validation failed; retain the new evidence.

## Reproduce a specific case

```powershell
.\assessment.cmd rerun case-013 --out runs\case-013-01
```

Use a case ID from the report. The original case's spec, source, engine and
runtime fingerprints must match the delivered ones. This starts a fresh run from
an authored initial condition, not a full-state checkpoint replay. One-case exit
codes: **0 PASS, 1 FAIL, 2 invalid/inconclusive/error**. A known failing case
returning 1 is expected; check its report and verdict rather than interpreting
every nonzero code as a launch failure.

## Repeat the full suite

```powershell
.\assessment.cmd suite --out runs\full-suite-01
```

This repeats 48 grid conditions and budgets 12 adaptive refinement episodes plus
four confirmations. It runs sequentially. Boundary search can choose different
conditions on another machine; if no eligible transition exists, fewer extra
episodes are possible. Inspect campaign.json for the actual coverage. Suite exit
0 means the recorded evidence is valid, regardless of how many robot cases FAIL;
2 means invalid evidence or a launch/validation error. Use `accept` for explicit
comparison with the shipped outcomes and 5 mm tolerance.

The first confirmation on each side requests native screenshots; it needs a
compatible graphics driver. Other measurements and acceptance reruns do not
request images. Expect several minutes for a full suite, without a timing
guarantee. Retain incomplete attempts and use a new directory for another run.

## Interpreting findings

PASS means the measured episode met the frozen simulation requirement. FAIL
means a valid modeled requirement violation. Hold/acquisition failures precede
a qualified STOP and are labelled separately. ERROR or INCONCLUSIVE means the
evidence is insufficient, modified, or the runtime failed; it is not a robot
capability result. A running robot at the observation end has no measured final
stopping distance. Reported forward excursion covers the observation only.

An inventoried file change makes verification fail intentionally. Keep this
delivery frozen; use a separate working copy for engineering changes and produce
a newly qualified delivery after changing source, model or runtime. Do not edit
the checksum inventory to make changed files appear to be the original shipment.

## Troubleshooting and handover

- Missing/changed files: re-extract the original archive and verify its adjacent
  SHA256 file against the separately provided delivery checksum.
- Simulator launch failure: retain stdout.log, engine.log, sidecar, manifest,
  fingerprint and any controller-error.txt in the new run folder. Do not delete
  a failed attempt. Extract into a local writable folder and confirm disk space.
- Acceptance mismatch: retain acceptance.json and every new run. Check the
  machine/runtime fingerprints. The original measurements were taken on the
  delivery machine; independent hardware agreement is not promised.
- The runtime changes no global environment variables, installs no services,
  needs no administrator access, and starts only its own test processes. Close
  your own runs normally; do not terminate unrelated simulator sessions.

Use the support channel agreed with your OmniSim delivery contact. Include SKU,
version, archive checksum and the new run evidence. Scope includes one
consolidated clarification round within 14 days; model/controller changes and
hardware correlation require separate engineering scope.
