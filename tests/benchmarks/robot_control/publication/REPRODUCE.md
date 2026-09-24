# Reproduce the robotics control benchmark

There are two distinct checks: independently recompute the published scores
from recorded observations, and collect new observations in OmniSim. The first
works offline with this archive. The second requires the matching OmniSim build,
model access, and a new run. No external replication has yet been claimed.

## 1. Verify the published evidence offline

Extract the archive into a new directory and run:

```powershell
python verify_archive.py
```

Python 3.12 is the reference interpreter. No key, simulator, Node, pip packages,
network connection, or private checkout is needed for this check. The verifier
checks the archive manifest, lock and row digests, complete coverage, all saved
source snapshots, measured task grades, available model request hashes, returned model
identities, token-cost calculations and per-episode physics sidecars. It prints
the six recomputed result rows. Review the supplied code before executing it.

A matching checksum proves file consistency, not independent authorship or
attestation of a physical experiment. The evidence was collected by the product
developer in simulation. The SHA256 of the downloaded ZIP should match the one
published beside it on the benchmark page.

## 2. Prepare the measured simulator version

The campaign used OmniSim **9.0.0**, Windows 11, Newton/MuJoCo CPU physics and
headless FAST mode. Windows has the prebuilt distribution; other systems require
a build and a separate calibration. The executable and selected runtime/asset
hashes are in the evidence directory's `lock.json`; `MANIFEST.json` identifies
that directory.

**Release dependency:** the measured 9.0.0 source and Windows runtime must be
published together with this benchmark. At preparation time the public latest
release was 8.2.0; that release is not interchangeable with the measured build.
Use the exact release reference in the final benchmark launch notes. The archive
does not include a multi-gigabyte simulator installation or every transitive
runtime binary. Do not describe live reproduction as available until that
matching release is downloadable. Offline verification is available immediately.

Use a clean, separate checkout of that release. Overlay the archived files in
the evidence directory's `source/` and `reproduction/extra-inputs/` at their matching
repository-relative paths. This restores the measured benchmark, bridge fixes,
robot assets, and compiled Lobster SDK. Inspect the overlay before copying it;
do not overwrite an unrelated working checkout. Heavy executable/runtime files
remain supplied by the matching release. The archived benchmark can also be
invoked directly with `python -m omnisim.control_bench.cli` if the main CLI in
that release does not expose `control-bench` yet.

Create a dedicated Python 3.12 environment, install `reproduction/requirements.lock.txt`,
and use Node 22.19.0. The controller runtime must include OmniLink SDK 0.6.3,
as in the measured simulator bundle. Lobster is pinned to commit
`71bd8145055498116de35623dbea0d7dda9b4cd5`; its compiled JavaScript and package
metadata are included. If rebuilding, use its locked dependencies and build
instructions. Its license is included beside the SDK.

Install the SDK's locked runtime dependencies in the restored vendor directory:

```powershell
Push-Location tests/benchmarks/harness_comparison/vendor/lobster
npm ci --omit=dev --ignore-scripts
Pop-Location
```

Run the environment check from the clean simulator checkout:

```powershell
python -m omnisim doctor
```

Resolve binary, physics and controller compatibility failures before proceeding.
The lock's machine fingerprint documents the original machine; your machine and
timings may differ. Record those differences rather than editing the old lock.

## 3. Calibrate, then collect a new campaign

Use the dedicated environment's interpreter for every command below. Copy the
archived suite to `reproduction-suite.json` in the checkout. Set `$profile` to an
existing OmniLink profile your account is entitled to use, and `$keyFile` to a
file containing your own existing OmniKey. Never put a key in an argument value,
task file, report, lock or archive. Every OmniLink AI experience requires a key.
The measured profile was `HuskySwarm`; that is an access configuration, not a
public entitlement. Using another profile is a disclosed configuration change.
Where `profile-observation-*.json` is present, the live profile was checked during or after the run:
its stored prompt fields (persona, user name and tool descriptions) were absent.
Use an entitled profile named `HuskySwarm` with those fields unset to match the
observed prompt configuration. If that name is already in use, create a separate
benchmark account/profile rather than changing an unrelated agent. The archived
`profile-observation-*.json` files record the check time and settings update time;
this is an observational check, not a frozen server deployment. Other campaigns
do not gain this attestation retroactively. The request body pins the main task
and disables service tool execution; the local benchmark executes the robot tools.

```powershell
python -m omnisim.control_bench.cli freeze --suite reproduction-suite.json --lock calibration.json --arms oracle --repeats 1
python -m omnisim.control_bench.cli run --lock calibration.json --out calibration --calibrate --key-file $keyFile
python -m omnisim.control_bench.cli verify calibration

python -m omnisim.control_bench.cli freeze --suite reproduction-suite.json --lock replication.json --repeats {{REPEATS}} --cap-usd {{CAP}} --profile $profile --model {{MODEL}} --input-rate {{RATE_INPUT}} --cached-rate {{RATE_CACHED}} --output-rate {{RATE_OUTPUT}}
python -m omnisim.control_bench.cli run --lock replication.json --out replication --key-file $keyFile
python -m omnisim.control_bench.cli verify replication
python -m omnisim.control_bench.cli report replication
```

Calibration must pass all {{TASKS}} archived tasks. It uses expected actions and
is never an agent score. This campaign comprises {{TOTAL}} live episodes:
{{TASKS}} tasks, six implementations and {{REPEATS}} repeat(s). These counts and
the requested model/rates above are generated from the campaign's frozen lock.
Do not adjust prompts,
remove failures, stop when a result looks good or overwrite a previous output.
Fixes require a new named protocol and all earlier attempts remain disclosed.
The cost limit stops new requests with a reserve; it is not a guaranteed billing
ceiling for an in-flight model request. Provider changes or unknown usage stop
further model calls, with campaign 04's explicitly classified overload exception
below. Credentials and an entitled profile are required for all
six implementations; none silently falls back to another model.

Campaign 04 retains recognized Google high-demand 503 requests with null usage
and a conditional $0 token-charge estimate based on the published failed-request
billing policy. It retries once after at least 60 seconds; the episode retains
that waiting time. The lock, error records, protocol and page disclose this
assumption and a $0.01/rejection sensitivity calculation. Unrecognized failures
and missing usage on successful responses still stop spending. Other campaigns
retain the transport and accounting implementation archived in their source.

The runner creates and cleans up its own simulator process trees. Use a host
where that is permitted. Run sequentially without other simulation workloads
if comparing timing. The first launch may compile physics kernels.

## What should reproduce

Offline scores and accounting should match exactly. New model outputs and wall
times need not: the shared service exposes a model ID, not an immutable backend
deployment, and requested temperature/output limits are not certified upstream.
Publish your new lock, hardware, source hashes, complete rows and scorecard.
Compare the six implementations within your own run before comparing machines.

Success covers these authored tasks, including simple mobile motion, conditions,
clarification, grounding, recovery and assisted arm operations. It does not
establish arbitrary-language generalization, visual navigation, fleet behavior,
real-hardware performance, friction-only grasping or a market-wide winner.
