# Mounted-payload adapter coverage — 2026-10-28

All [seven measured cases](campaign.json) passed with valid compiled-model checks
and matching offline regrades; see [report.html](report.html). Each registered
physical-wheeled model ran flat ground at contact μ=0.40 with a rigidly mounted
2 kg box and immediate braking. This checks that the payload-generation and
measurement path executes across both two- and four-wheel configurations.

Robot-specific speed targets and mount heights are frozen in each `spec.json`.
They are experimental fixture choices, not manufacturer payload ratings or
claims about hardware. A mount represents the stated fixed mass/geometry; it is
not a model of a particular commercial bracket, fastening or cargo package.
Husky uses a 0.35 m chassis-frame payload center; other models use the recorded
profile-derived height. The compiled model verifies actual total mass, contact
friction and surface orientation for every case.

These extend the [28 unloaded slope/friction checks](../mobile-coverage-20261028/README.md)
to 35 coverage cases across Husky, Jackal, ROSbot, ROSbot XL and three TurtleBot3
variants. They do not establish those other robots' failure envelopes.
Conditions: Newton 1.5.0 / MuJoCo 3.11.0 CPU, 16 ms samples, 8 substeps, machine
`9722d23d12a3`. The individual fingerprints and `source/` snapshots preserve the
implementation/runtime context.

```powershell
python -m omnisim assess sweep --robots all --slopes 0 --frictions 0.4 --payloads 2 --workers 2 --out .tmp/my-new-payload-coverage
```
