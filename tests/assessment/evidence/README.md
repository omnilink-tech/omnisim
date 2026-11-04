# Husky braking evidence

For natural failures with **no braking delay**, start with the
[64-case Husky environment campaign](husky-envelope-20261028/README.md), its
[report](husky-envelope-20261028/report.html), and the
[seven-model physical-wheeled coverage](mobile-coverage-20261028/README.md).
The older injected-delay proof below is a separate control experiment.
The [final validation record](VALIDATION_20261028.json) identifies the three
campaigns (99 episodes), their hashes, and the integrity/physics/cleanup checks.

The frozen [2026-10-28 proof](husky-braking-20261028/proof.json) was executed with
the implementation documented in the [assessment guide](../../../docs/guide/robot-assessments.md).
Open [report.html](husky-braking-20261028/report.html) or
[braking.png](husky-braking-20261028/braking.png) for the comparison.

Conditions: unloaded Husky URDF; dedicated deterministic wheel-velocity
assessment controller; flat floor; `newtonGroundMu 0.8`; Newton 1.5.0 / MuJoCo
3.11.0 CPU; 16 ms samples and 8 physics substeps; machine `9722d23d12a3`, Windows
11, AMD64 Family 25 Model 80, 16 logical cores. The full fingerprint and
engine/controller/model hashes are saved per case. The nominal requested speed
was 0.6 m/s, with measured speed 0.599701 m/s when STOP was requested.

| Cases | Forward excursion after STOP | Settled after STOP | Outcome |
|---|---|---|---|
| Five fresh baselines | 0.019692 m in each | 0.080 s | 5/5 PASS |
| Injected 0.8 s command delay | 0.499454 m | 0.880 s | FAIL: distance exceeded |
| Fresh reproduction | 0.499454 m | 0.880 s | FAIL: same violation |
| Injected delay removed | 0.019692 m | 0.080 s | PASS |
| Separate 0.5 m/s confirmation | 0.013177 m | 0.064 s | PASS |

The 0.25 m distance requirement was frozen before the live cases. The repair
removes the deliberately injected delay; this is not a discovered Clearpath
firmware or hardware defect. The failure/reproduction distances match exactly
in these runs; that is an observation for this configuration, not a universal
determinism claim. All nine offline regrades matched, and captured owned engine
and controller processes were confirmed reaped.

Reproduce the complete workflow with a new directory:

```powershell
python -m omnisim assess prove --capture --out tests/assessment/evidence/my-new-proof
```

Every artifact is kept with its episode; the original bad and repaired native
stills are separate recordings of their respective runs. Local `development-*`
attempts are retained but ignored by Git. No case here establishes agreement
with physical hardware or the performance of the manufacturer's controller.
