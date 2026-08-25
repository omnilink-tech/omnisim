# Making a friction grasp hold

A two-finger grip that carries a part by **contact friction** — no weld, no
teleport — is the most settings-sensitive thing this engine does. This page is
what is *measured* to decide whether one holds, and what is measured not to.

> ### ⚠ This page was re-derived against measurement on 2026-08-12, and two of
> ### its five recipe fields turned out to be wrong
>
> The old version prescribed a five-field `WorldInfo` recipe. On a scene where
> a grasp demonstrably holds, **`newtonContactKe 8000` ejects the part at
> 2.3 m/s**, `newtonIterations 150` / `newtonLsIterations 50` change the result
> by **nothing at all** (bit-identical), and the whole recipe applied verbatim
> **fails a grasp that its own two remaining fields pass**. The old controller
> section was worse: it told you to hold the grip with `setForce`, fifty lines
> below its own paragraph explaining that `setForce` does not put a Newton
> joint in force mode. Both are corrected below.
>
> The numbers on this page come from **ladder0 rung 8**
> ([`tests/benchmarks/ladder0/`](../../tests/benchmarks/ladder0/)), a
> first-principles grasp scene whose expectations are analytic and whose
> `no_grip` control proves the *fingers* did the lifting. Every row is
> reproducible: `python tests/benchmarks/ladder0/omnisim/variants.py <family>`.

---

## The scene these numbers come from

Read this before copying anything: a grasp setting is not a constant of the
engine, it is a property of a contact, and the contact depends on the part.

| | |
|---|---|
| part | 60 mm cube, **0.2 kg**, `Box` boundingObject |
| pads | 30 × 12 × 50 mm, 0.02 kg each, two, centred on opposite faces |
| friction | `newtonGroundMu 3` |
| grip | **3.0 N per pad** — 9× the Coulomb bound (below) |
| motion | lift 0.15 m at 0.10 m/s, then carry 0.45 m at 0.15 m/s |
| engine | `newtonSolver "mujoco"` (CPU `mj_step`), `basicTimeStep 4`, newton 1.2.0 / mujoco 3.8.1 |
| machine | `9722d23d12a3` (RTX 3060 laptop) |

The measurement is `carry_rel`: the **largest** distance between the payload's
centre and the gripper's origin over the whole run, in metres. The two are
authored coincident, so **0 is the analytic answer** and the number is a pure
slip. A payload that is dropped ends up around 0.45 (the gripper travels on
without it).

**Start from the Coulomb bound, not from a recipe.** Two pads holding mass *m*
by friction alone need

```
N  >=  m g / (2 mu)
```

per pad — 0.33 N here. If your grip is not several times that, no amount of
solver configuration will save it, and the first thing to check is the *force*.
Everything below is about a grip that is already far above the bound and still
slides.

---

## The two fields that decide it

```
WorldInfo {
  newtonSolver  "mujoco"      # the default since 2026-08-07; declare it anyway
  newtonGroundMu 3            # or whatever m g / (2 mu) says you need
  newtonCone    "elliptic"    # <- the exact Coulomb cone
  newtonImpratio 10           # <- friction as stiff as the normal constraint
}
```

| configuration | `carry_rel` | what happened |
|---|---|---|
| engine defaults | 0.4747 | creeps 56 mm through the pads during the lift, **drops** the part, `part_speed_max` 1.11 m/s |
| `newtonCone "elliptic"` alone | 0.0213 | **carried** the full 0.45 m, slipped 21 mm |
| `newtonImpratio 10` alone (pyramidal cone) | 0.0178 | carried, slipped 18 mm |
| `newtonImpratio 100` alone (pyramidal cone) | 0.3597 | **worse than doing nothing** — part dropped |
| elliptic + impratio 1 / 2 | 0.0213 / 0.0424 | 2 is worse than 1 |
| elliptic + impratio 4 | 0.0044 | holds — the knee |
| **elliptic + impratio 10 / 30 / 100 / 300** | **0.0026 / 0.0026 / 0.0026 / 0.0026** | **holds; 0.09 mm of spread over a 30× range** |

Three things to take from that table:

- **`newtonCone "elliptic"` is not an optimisation, it is the model.** MuJoCo's
  default cone is a pyramid *inscribed* in the Coulomb cone — a polygonal
  approximation. If your world says µ = 3, the elliptic cone is the one that
  means it.
- **`newtonImpratio` is the ratio of frictional to normal constraint
  impedance.** At the stock 1 the friction constraint is exactly as soft as the
  normal one, so a contact under sustained tangential load **drifts** while its
  normal force sits precisely where you commanded it. That is why a grip can be
  9× the Coulomb bound and still creep: nothing is being violated, the
  constraint is just soft. Raising it does not change µ; it changes how strictly
  µ binds.
- **Use them together or not at all.** Each alone leaves ~20 mm of slip, and a
  high `impratio` on the *pyramidal* cone measured **worse than the default** —
  a pyramid is a fixed set of friction directions, and stiffening the response
  along them is not the same operation as stiffening it against a circle.
  Anyone quoting "raise impratio" without the cone is quoting a setting that
  measured worse than doing nothing.

Pick `impratio` from the plateau, not the knee: 10 is the first value inside
it, with 30× of headroom above.

---

## The fields that did not help — including one that breaks it

Every row below is a **single change against the working configuration above**,
on the same scene.

| field | `carry_rel` | verdict |
|---|---|---|
| `newtonContactKe 4000` (kd at its default) | 0.3227 | ✗ **breaks it** — part ejected at 1.06 m/s |
| `newtonContactKe 6000` (kd at its default) | 0.3928 | ✗ ejected at 1.12 m/s |
| `newtonContactKe 8000` (kd at its default) | 0.3316 | ✗ ejected at **2.33 m/s** |
| `newtonContactKe 8000` + `newtonContactKd 200`, as the recipe pairs them | 0.1497 | ✗ ejected at 1.07 m/s |
| `newtonIterations 150` + `newtonLsIterations 50` | 0.002559 | no effect — **bit-identical** |
| `newtonCondim 4` | 0.002559 | no effect here — bit-identical |
| `newtonNoslipIterations 5` | 0.0027 | no material effect (+0.09 mm) |
| **the whole old five-field recipe, verbatim** | **0.1520** | ✗ **fails a grasp its own subset passes** |

**`newtonContactKe` is the trap, and it is the opposite of what this page used
to say.** The old text said the 2500 default "lets fingers sink into the part
and squirt it out sideways". Measured on a working grasp, *raising* it is what
squirts the part out — at every value tried, from 4000 up, whether or not the
finger command is adjusted to hold the grip force constant. Both variants were
run: with the interference recomputed so the pads still develop exactly 3.0 N
(0.3316, 2.33 m/s) and with the finger command left alone so the same bite now
develops 4.3 N (0.3317, 2.22 m/s). The stiffness is the variable, not the force.

**What is NOT established:** *why*. A stiffer contact against a 0.02 kg pad at
`basicTimeStep 4` plausibly goes marginal and releases stored energy at
contact establishment, but that is a hypothesis, not a measurement, and a
timestep sweep would be needed to settle it.

**And the honest caveat in the other direction.** `newtonContactKe 8000` came
from [`omniarm6_real_pick_place`](../../projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld),
a different scale (90 mm block, µ = 6, `basicTimeStep 8`, a real 6-axis arm).
**No A/B has ever been published for that world** — nobody has shown it fails
at the 2500 default. So the state of the evidence is: unmeasured on the world
that ships it, and refuted on the one scene where a controlled comparison
exists. Treat contact stiffness as a per-scene value you have to *test*, not a
constant to copy — and if you do raise it, prove the part is carried rather
than launched (`part_speed_max` is what catches that).

`newtonCondim 4` is a genuine fix for a *different* failure and its evidence
still stands: at the default `3`, contacts carry **sliding friction only**, so a
pinched part is free to rotate about the contact normal at zero cost. Measured
on `omniarm6_real_pick_place` with the pads centred on a 90 mm block: at `3` the
block rotates inside the gripper during a 0.47 m carry and levers the pads from
24 mm apart to 41 mm, arriving tipped on 3 of 3 runs; at `4` the same run keeps
two contacts per pad and places it standing. It does nothing on rung 8 because
that pinch is symmetric and centred, so there is no torque about the normal for
it to resist. Reach for it when your part **twists** in the grip, not when it
**slides**. ⚠ It is global, not per-shape.

`newtonNoslipIterations` (new, 2026-08-12) is MuJoCo's own post-solve pass over
the friction constraints — the textbook remedy for exactly this drift, and what
the ladder's bare-MuJoCo arm uses to fix the *same scene* in *its* engine. On
OmniSim's contact parameterisation it does not reproduce that: on the engine
default cone it left the payload dropped at every count from 1 to 20
(`carry_rel` 0.4855 → 0.4779), and on the working configuration it moves the
answer by 0.09 mm. The two engines derive `solref` differently (bare MuJoCo's
stock acceleration-referenced `(0.02, 1)` vs newton's `ke`/`kd`-derived direct
stiffness), which is the obvious suspect and has not been tested. Try it when a
grip creeps; do not assume it.

---

## The controller: producing a known grip force

> ⚠ **`setForce` does NOT put a Newton joint in force mode.** This is the single
> most expensive misunderstanding on this page and the old version of it
> repeated the mistake in its own example. Every joint is built
> `POSITION_VELOCITY` with `targetKe = effortLimit * 10`
> ([OmBasicJoint.cpp:675](../../src/omnisim/nodes/OmBasicJoint.cpp)), so the PD
> servo stays live, **anchored at the last `setPosition`**. A call that reads
> like "squeeze with 25 N" is really a spring pulling toward wherever the joint
> was last told to go — measured on `friction_grasp_minimal`, a target ~20 mm
> *inside* the part, which buries the pads 4.1 mm deep and releases the stored
> interference as a launch.

There are two supported ways to get a known force. Pick one deliberately and
**record which one you used**, because a grasp measured through an interference
and a grasp measured through a true force mode are not the same measurement.

**1. A position target with a known interference** (the default path; no env
vars). The pad's servo and the contact act in series, so the interference that
develops force *F* is

```
bite = F * (1/kp + 1/ke)          kp = 10 * maxForce      ke = newtonContactKe
```

A **larger** `maxForce` means a **stiffer** servo and therefore a **smaller**
bite for the same force. On rung 8, `maxForce 200` gives `kp = 2000 N/m`, so
3.0 N is 2.7 mm of bite — under a quarter of the pad's own thickness, and the
pad stops at the part's surface instead of inside it. The 200 N is never
delivered: at the commanded target the servo develops 1.5 % of it.

```python
KP   = 10.0 * FINGER_MAXFORCE            # the engine's own joint construction
KE   = 2500.0                            # newtonContactKe, default
bite = GRIP_N * (1.0 / KP + 1.0 / KE)
finger.setVelocity(CLOSE_SPEED)
finger.setPosition(surface_q + bite)     # a few mm past the face, not 20
```

The runnable version with the algebra written out once is
[`ladder0/omnisim/engine_facts.py`](../../tests/benchmarks/ladder0/omnisim/engine_facts.py)
and `rungs.rung8_bite_m()`.

**2. Real force mode:** set `OMNISIM_NEWTON_TORQUE_MODE=1` and `setForce` means
what it says. Then, and only then, does the old advice apply — a motor in force
control keeps that force until something withdraws it, `setPosition` does not
undo it, and you release by reversing the sign (`setForce(-RELEASE_N)`).

**Mind the sign either way.** Force and displacement follow the joint axis, not
the word "grip": if a finger's slider axis points away from the part, closing is
*negative*.

---

## Proving it held

`getContactPoints` reads native Newton contacts by default since 2026-08-07, so
an empty set is now more likely to mean "nothing is touching" than "cannot see"
— but a contact read is still a **weaker** claim than a lifted part. Prove it
geometrically:

- the part is **airborne** (well above its rest height), **and**
- it **tracks the gripper** (the distance between them stays constant — this is
  exactly `carry_rel`), **and**
- it never moves faster than the gripper commanded it to (`part_speed_max`).

That third one is not optional. A pinch that *ejects* its part can leave it in
an entirely plausible final pose: `friction_grasp_minimal` ejects at 3.5 m/s and
the part lands on the gripper's own wrist plate, where "airborne + tracking"
both hold.

> ⚠ **A grasp demo without a DROP CONTROL proves nothing.** The identical motion
> with no squeeze must leave the part behind. That is the only thing separating
> "the fingers carried it" from "something else did"; it is why ladder0 rung 8
> ships a `no_grip` fault as a required control, and it is what
> [`omniarm6_real_pick_place`](../../projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld)
> uses `PICK_CONTROL_DROP=1` for.
>
> ⚠ **Airborne + tracking proves the RIG holds the part, not that the PADS do.**
> The failure it cannot see is a **palm wedge**: a part taller than the pads,
> jammed against the gripper body and carried perfectly while the fingers do
> nothing. To rule it out you need contact **attribution**, which takes two
> queries: the `node_id` on a `ContactPoint` names the **queried** side, so
> `block.getContactPoints()` stamps the block's own id on every point. Query the
> *robot* with `includeDescendants=True` — there `node_id` is the link — and
> match the two lists by point.
> [`omniarm6_real_pick_place`](../../projects/samples/demos/controllers/omniarm6_real_pick_place/omniarm6_real_pick_place.py)
> (`census()`) does this and requires contacts from **both** pads on the part's
> own grasp faces while it is off the table.
>
> ⚠ **Contact positions and depths were wrong until 2026-08-10.** On the CPU
> `mj_step` path `get_contacts` answered from newton's own narrow-phase — which
> under `SolverMuJoCo` never runs the step — and published
> `rigid_contact_point0`, a **support point in shape 0's body-local frame**, as a
> world coordinate, with `depth` hard-coded `0.0`. Measured: a block resting on a
> table reported the contact at `(-0.0250, -0.0251, 0.1000)` — that point in the
> *table's* frame — and a block "touched" an arm link half a metre away. It now
> reads `mjData.contact` (world `pos`, signed `dist`, real `mj_contactForce`).
> If you are on an older binary, do not trust a contact position or a depth.
>
> ⚠ **Controller `print` is not reliable evidence.** On Windows the simulator is
> a GUI-subsystem binary with no console, so controller stdout goes nowhere
> unless the launcher captures it. Write results to a **file**.

**The runnable example that does hold, with its drop control:**

```bash
python -m omnisim run-headless \
  projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld --duration 45
```

⚠ **`friction_grasp_minimal.omniworld` is NOT a working grasp. Do not copy it.**
Instrumented 2026-08-10: the box is not carried by the pinch, it is ejected by
interpenetration blow-out at 3.5 m/s, arcs to z = 0.97 m, and lands on the
gripper's own wrist plate — a 100 × 40 mm horizontal shelf sitting directly
above the fingers. During the "hold", all 12 of its contact points lie in one
plane on its own bottom face and the pads are 20 mm clear of it; driving the
fingers fully open afterwards does not move it. The verdict it prints
(`held_z - start_z > 0.05`) cannot tell a carry from a tray. It also still
declares the whole old recipe, including the `newtonContactKe 8000` this page
now measures as harmful.

---

## Other settings, and what is actually known about them

| setting | status |
|---|---|
| `newtonSolver "mujoco"` | **The default since 2026-08-07** — declare it anyway so the file states its own requirement. Recorded because it explains older worlds: XPBD, the previous default, **structurally cannot** hold a static pinch (its own paper: *"we assume zero compliance in contact"*). It was removed on 2026-08-07 and a world still declaring `"xpbd"` gets a warning and the default |
| `newtonGroundMu` | The field that sets contact friction. Default **1.0**. Size it from `N >= m g / (2 mu)` against the grip force you can actually deliver, rather than from a number on a page. Above ~6 the floor itself destabilises even at `basicTimeStep 8` |
| `basicTimeStep 8` | Inherited advice — "high-friction contact goes unstable at 16 and the part jitters out". **Not measured here**, and note that rung 8's grasp holds at **4**. If you are debugging a jittery grip, halving the timestep is cheap to try; it is not established as a requirement |
| ~~`physicsDisableTime 0`~~ | **Drop it.** It disabled ODE's auto-sleep. Newton has no body sleep and the field is retired and unread. Harmless if left in an old world |

> ⚠ **`WorldInfo.contactProperties` does not set friction here.** It was the
> ODE-path friction declaration and it is **not read** — ODE was deleted on
> 2026-08-08. A world declaring `coulombFriction [ 5 ]` gets **1.0**.
> `newtonGroundMu` is the field that works, and the engine warns when a world
> declares one without the other. Do **not** reach for the old escape hatch of
> pinning `physicsBackend "ode"` on the gripper: a Solid pinned to `"ode"` is
> not simulated at all now.
>
> (One narrow exception: a world that pins `defaultPhysicsBackend "newton"` has
> its first positive `coulombFriction` **bridged** to Newton's ground friction,
> and the engine logs that it did so. Prefer declaring `newtonGroundMu`
> outright.)

## Shape traps

- A **`Cylinder` boundingObject is substituted** on the Newton path — it becomes
  a capsule, so the collider is not the shape the file draws and the part rests
  at a height the geometry does not explain. `Box`, `Sphere` and `Capsule` are
  exact. Prefer them for anything a gripper touches.
- A **single-link body imported from a URDF** used to rest on its own frame
  rather than its geometry (fixed 2026-08-03); if you see a part sunk half its
  height, check you are on a current build.

## If it still slips

In order of what the measurements support:

1. **The grip is genuinely too weak.** Check `N >= m g / (2 mu)` first. Nothing
   below matters if the force is marginal.
2. **Friction is still 1.0** because `contactProperties` was set instead of
   `newtonGroundMu`.
3. **`newtonCone` is not `"elliptic"`** — at the pyramidal default a 9×-margin
   grip still creeps a millimetre at a time.
4. **`newtonImpratio` is at the stock 1**, or is raised *without* the elliptic
   cone, which measured worse than leaving it alone.
5. **The fingers are `setForce`-controlled without `OMNISIM_NEWTON_TORQUE_MODE=1`**,
   so the "force" is a spring pulling at the last `setPosition`.
6. **`newtonContactKe` has been raised.** If the part is not slipping out but
   being *thrown*, this is the first thing to put back.
7. **The part twists rather than slides** — that is `newtonCondim`, not
   friction magnitude.
8. Only then reach for `basicTimeStep`, `newtonNoslipIterations`, or more
   solver iterations, none of which moved this scene.
