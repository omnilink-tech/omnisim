# Husky braking assessment — US$100

SKU: **HUSKY-BRAKING-100**, delivery version **1.0.0**. One-time fixed price of
US$100 for the standard assessment and reusable delivery described below;
applicable tax is separate. This is the delivery specification, not an invoice
or a claim that a customer has purchased it. Open-source licenses continue to
apply to the software; the charge is for prepared assessment work and handover.

## Included

- One included Husky URDF configuration with preserved Clearpath BSD notices,
  driven by OmniSim's reference wheel-velocity assessment controller.
- A completed 48-condition grid: downhill slopes 0/5/10/20 degrees × imposed
  tyre/surface contact coefficients 0.10/0.20/0.40/0.80 × rigid payload 0/10/20 kg.
- Twelve additional local friction-refinement episodes and four fresh
  confirmations of one passing/failing pair: **64 recorded episodes**. All use
  immediate braking with no injected command delay. Confirmations do not repeat
  every condition. These test points do not establish exhaustive coverage.
- Target speed 0.6 m/s. Payload is a 0.30 × 0.25 × 0.20 m rigid box fixed to the
  chassis, centered 0.35 m above the chassis-frame origin, with zero forward
  offset. Geometry, command envelopes and experimental requirements are frozen
  in the included source and per-case specifications.
- A readable HTML report, machine-readable outcomes and sampled transition
  brackets, per-step traces, compiled-physics adoption checks, all attempts,
  source/runtime provenance and selected native images where captured.
- A Windows x64 runtime capsule, embedded Python, dependencies and a launcher.
  No Git checkout, Python installation, cloud account or model-provider key is
  required for this independent assessment. The report and existing evidence can
  be checked offline; local reruns use the included simulator.
- Reusable tests for local reruns under the applicable open-source/model licenses.
  Local reruns have no assessment-service per-run fee. Customer hardware and
  electricity are customer costs; this says nothing about OmniLink AI pricing.
- One handover and one consolidated setup/interpretation clarification round
  submitted within 14 calendar days of delivery. Changes to robot models,
  controller interfaces, task definitions or requirements need a separate scope.

## Frozen task requirement

Acquire a qualified approach: at least 0.30 m measured travel and 0.30 s in the
specified speed band. After STOP, forward excursion must be at most **0.25 m**;
settle to at most **0.025 m/s** within **1.5 s** and remain settled for at least
**0.4 s** and the remaining observation; rollback must not exceed **0.05 m**.
Observe for at least **2.5 s** after STOP. Initial hold, acquisition, tilt and
departure checks are also assessed and labelled separately from qualified
braking failures. Missing/incomplete evidence is ERROR/INCONCLUSIVE, not PASS.
Sampling is 16 ms with eight physics substeps, using Newton / CPU MuJoCo.
These are experimental requirements, not Clearpath specifications.

## Delivery acceptance

The deliverable is accepted technically when its file hashes verify, all 64
saved episodes regrade consistently, and the four fresh customer acceptance
episodes reproduce their recorded PASS/FAIL outcomes within 5 mm of recorded
forward excursion. A robot FAIL is a finding, not a defective delivery. The
acceptance command exits successfully only when the expected outcomes reproduce.
See CUSTOMER-ACCEPTANCE.md. A mismatch on another machine is a compatibility or
repeatability issue to investigate; do not loosen the requirement to hide it.

## Supported environment

Windows x64, tested on Windows 11 with an AMD64 CPU. CPU physics is used; native
image capture additionally depends on a compatible graphics driver. Use a local
unpacked writable folder whose full resolved root path is at most 100 characters
(for example a HuskyKit folder under your user profile), with sufficient free
storage (allow at least 3 GB for
the extracted runtime, evidence and new runs). The original measurements and
fingerprint are retained. The release has been tested by extraction on the
delivery machine; that alone is not verification on an independent fresh PC.
Linux source builds exist in OmniSim, but this delivery's tested runtime is
Windows only; macOS is outside this package.

## Outside this price and evidence

The included driver is not Clearpath firmware or a customer's production
autonomy stack. Customer controller integration, a different URDF/CAD variant,
new sensors/tasks, rough terrain, loose cargo, thermal behavior, structural
fracture/fatigue, hardware calibration/correlation and safety certification are
outside this scope. A physically calibrated assessment needs separate measured
data and qualification. This package establishes findings about the included
simulation configuration; it does not certify the real robot's safe limits.

No affiliation or endorsement by Clearpath Robotics is implied. Preserve the
licenses and notices when redistributing permitted material. The source and
checksum inventory are included; the ZIP checksum is not a digital signature.
