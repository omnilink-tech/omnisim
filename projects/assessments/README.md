# Robot assessments

The [mobility assessment](../../docs/guide/robot-assessments.md) measures physical
wheeled models over slopes, contact friction and mounted payloads, with immediate
braking. The generic wheel controller reuses existing robot configurations and
handles two- and four-wheel layouts; it does not run vendor firmware.

Use `python -m omnisim assess sweep --robots husky --out <new-directory>` from the
repository root. `assess robots` inventories URDF models and explicitly separates
available task adapters from assets without an assessment adapter. `--robots all`
selects the seven physical-wheel configurations, not every robot class.
The CLI generates each incline/model copy and supplies per-run configuration;
opening the original world alone does not run a configured assessment.

Model/controller inventory: [robots/husky.json](robots/husky.json).
Original flat-world proof: [worlds/husky_braking.omniworld](worlds/husky_braking.omniworld).
Controller: [controllers/husky_braking/husky_braking.py](controllers/husky_braking/husky_braking.py).

Results are simulation findings under frozen conditions, not hardware calibration,
material fracture predictions, or a certified operating envelope.
