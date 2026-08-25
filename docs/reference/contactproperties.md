## ContactProperties

```
ContactProperties {
  SFString material1          "default"                                                                                             # any string
  SFString material2          "default"                                                                                             # any string
  MFFloat  coulombFriction    1                                                                                                     # [0, inf)
  SFVec2f  frictionRotation   0 0                                                                                                   # any positive vector
  SFVec3f  rollingFriction    0 0 0                                                                                                 # rolling/spinning friction
  SFFloat  bounce             0.5                                                                                                   # [0, 1]
  SFFloat  bounceVelocity     0.01                                                                                                  # [0, inf)
  MFFloat  forceDependentSlip 0                                                                                                     # [0, inf)
  SFFloat  softERP            0.2                                                                                                   # [0, 1]
  SFFloat  softCFM            0.001                                                                                                 # (0, inf)
  SFString bumpSound          "omnisim://projects/default/worlds/sounds/bump.wav"                                                      # any string
  SFString rollSound          "omnisim://projects/default/worlds/sounds/roll.wav"                                                      # any string
  SFString slideSound         "omnisim://projects/default/worlds/sounds/slide.wav"                                                     # any string
  SFInt32  maxContactJoints   10                                                                                                    # (0, inf)
}
```

> ⚠️ **THE `ContactProperties` NODE HAS NO EFFECT ON CONTACT. Read this before tuning anything here.**
> Every field on this node was implemented on the ODE contact path, and ODE was deleted on
> 2026-08-08 (commit `bdc02139`). Newton/MuJoCo is the only physics backend and it does **not** read
> this node. The node and `WorldInfo.contactProperties` are still declared so legacy worlds parse —
> removing them would turn ~240 live worlds into load errors — but changing a value here changes
> nothing.
>
> **The Newton fields that actually control contact live on [WorldInfo](worldinfo.md):**
>
> | you wanted | use instead |
> |---|---|
> | `coulombFriction` | `WorldInfo.newtonGroundMu` (default 1.0; a two-finger pinch needed 3; above ~6 the floor destabilises) |
> | `softCFM` / `softERP` (contact softness) | `WorldInfo.newtonContactKe` / `newtonContactKd` (defaults 2500 / 100) |
> | frictional accuracy near the cone boundary | `WorldInfo.newtonCone "elliptic"` + `newtonImpratio 10` |
> | contact that must HOLD rather than creep | `WorldInfo.newtonIterations` / `newtonLsIterations` |
> | `bounce` / `bounceVelocity` (restitution) | **no equivalent field, and there will not be one.** ⚠️ This row used to call it "an open gap, not a solved one"; it is a **hard limit**. MuJoCo has no coefficient of restitution — the concept does not exist in mujoco 3.11.0 — and models contact as a `solref`/`solimp` spring-damper. Our defaults compile to `solref (0.02, 1.0)`, MuJoCo stock and critically damped: `bounce 0.8` predicts a 0.64 m rebound and measures **−0.4 mm**, reproducible in *bare* MuJoCo. Only e≈0 (today) and e≈1 (zero damping) are stable; one intermediate configuration rebounds **661 m** from a 1 m drop. As of 2026-08-17 the engine **warns** when a world *authors* a bounce, rather than accepting it silently. If a scene needs a bounce, model it in the controller. |
> | `rollingFriction` | **no equivalent field.** Also an open gap. |
> | `bumpSound` / `rollSound` / `slideSound` | **nothing.** Contact sound produces no audio on Newton. |
> | `maxContactJoints` | **nothing.** Newton's contact budget is `WorldInfo.newtonNconmax` / `newtonNjmax`, which are per-world caps, not a per-pair limit. |
>
> One narrow exception, so that a world which *deliberately* chose Newton is not ignored: if the
> world pins `WorldInfo.defaultPhysicsBackend "newton"` (or `OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES`
> is set), the first positive `coulombFriction` value is **bridged** to Newton's ground friction and
> the engine logs that it did so. An ordinary world is never re-frictioned this way, because the 240
> worlds declaring `coulombFriction` were tuned under an effective mu of 1.0.
>
> ⚠️ **The old escape hatch — "pin `physicsBackend "ode"` on the Solids you are tuning" — no longer
> works and must not be used.** A Solid pinned to `"ode"` is not simulated at all.
>
> Everything below records what this node did while ODE shipped. It is kept because the field
> semantics explain a great many existing world files, and because `material1` / `material2` /
> `Solid.contactMaterial` matching is still how you would *identify* a contact pair if these fields
> are ever re-implemented. Read it as history, not as a tuning guide.

### Description

[ContactProperties](#contactproperties) nodes defined the contact properties to use in case of contact between [Solid](solid.md) nodes (or any node derived from [Solid](solid.md)).
[ContactProperties](#contactproperties) nodes are placed in the `contactProperties` field of the [WorldInfo](worldinfo.md) node.
Each [ContactProperties](#contactproperties) node specifies the name of two *materials* for which these [ContactProperties](#contactproperties) are valid.

When two [Solid](solid.md) nodes collide, a matching [ContactProperties](#contactproperties) node is searched in the [WorldInfo](worldinfo.md).`contactProperties` field.
A [ContactProperties](#contactproperties) node will match if its `material1` and `material2` fields correspond (in any order) to the `contactMaterial` fields of the two colliding [Solid](solid.md)s.
The values of the first matching [ContactProperties](#contactproperties) are applied to the contact.
If no matching node is found, default values are used.
The default values are the same as those indicated above.

> **Note**: In older OmniSim versions, contact properties used to be specified in [Physics](physics.md) nodes.
For compatibility reasons, contact properties specified like this are still functional in OmniSim, but they trigger deprecation warnings.
To remove these warning you need to switch to the new scheme described in this page.
This can be done in three steps: 1.
Add [ContactProperties](#contactproperties) nodes in [WorldInfo](worldinfo.md), 2.
Define the `contactMaterial` fields of [Solid](solid.md) nodes, 3.
Reset the values of `coulombFriction, bounce, bounceVelocity` and `forceDependentSlip` in the [Physics](physics.md) nodes. (⚠️ **This migration is history, not advice** — the destination it points at is itself retired and not read; see the banner at the top of this page.)

### Field Summary

- The `material1` and `material2` fields specify the two *contact materials* to which this [ContactProperties](#contactproperties) node must be applied.
The values in this fields should match the `contactMaterial` fields of [Solid](solid.md) nodes in the simulation.
The values in `material1` and `material2` are exchangeable.

- The `coulombFriction` are the Coulomb friction coefficients.
They must be in the range 0 to infinity (use -1 for infinity).
0 results in a frictionless contact, and infinity results in a contact that never slips.
This field can hold one to four values.
If it has only one value, the friction is fully symmetric.
With two values, the friction is fully asymmetric using the same coefficients for both solids.
With three values, the first solid (corresponding to `material1`) uses asymmetric coefficients (first two values) and the other solid (corresponding to `material2`) uses a symmetric coefficient (last value).
Finally, with four values, both solids use asymmetric coefficients, first two for the first solid and last two for the second solid.
The two friction directions are defined for each faces of the geometric primitives and match with the U and V components used in the texture mapping.
Only the `Box`, `Plane` and `Cylinder` primitives support asymmetric friction.
If another primitive is used, only the first value will be used for symmetric friction.

- The `frictionRotation` allows the user to rotate the friction directions used in case of asymmetric `coulombFriction` and/or asymmetric `forceDependentSlip`.
By default, the directions are the same than the ones used for texture mapping (this can ease defining an asymmetric friction for a textured surface matching the rotation field of the corresponding TextureTransform node).

- The `rollingFriction` field specifies the coefficients of rolling/spinning friction.
The field holds three coefficients, using ODE's nomenclature they are [rho, rho2, rhoN].
Each coefficient accepts only positive values or -1.0, where -1.0 corresponds to infinity.
For a value of zero no rolling friction is applied.
`rho` is the rolling friction coefficient in the first friction direction.
`rho2` is the rolling friction coefficient in the second friction direction, perpendicular to that of `rho`.
`rhoN` is the rolling friction coefficient around the normal direction.

- The `bounce` field is the coefficient of restitution (COR) between 0 and 1.
The coefficient of restitution (COR), or *bounciness* of an object is a fractional value representing the ratio of speeds after and before an impact.
An object with a COR of 1 collides elastically, while an object with a COR < 1 collides inelastically.
For a COR = 0, the object effectively "stops" at the surface with which it collides, not bouncing at all.
COR = (relative speed after collision) / (relative speed before collision).

- The `bounceVelocity` field represents the minimum incoming velocity necessary for bouncing.
Solid objects with velocities below this threshold will have a `bounce` value set to 0.

- The `forceDependentSlip` field defines the *force dependent slip* (FDS) for friction, as explained in the ODE documentation: "FDS is an effect that causes the contacting surfaces to side past each other with a velocity that is proportional to the force that is being applied tangentially to that surface.
Consider a contact point where the coefficient of friction mu is infinite.
Normally, if a force f is applied to the two contacting surfaces, to try and get them to slide past each other, they will not move.
However, if the FDS coefficient is set to a positive value k then the surfaces will slide past each other, building up to a steady velocity of k*f relative to each other.
Note that this is quite different from normal frictional effects: the force does not cause a constant acceleration of the surfaces relative to each other&mdash;it causes a brief acceleration to achieve the steady velocity."

    This field can hold one to four values. If it has only one value, this
    coefficient is applied to both directions (force dependent slip is disabled if
    the value is 0). With two values, force dependent slip is fully asymmetric using
    the same coefficients for both solids (if one value is 0, force dependent slip
    is disabled in the corresponding direction). With three values, the first solid
    (corresponding to `material1`) uses asymmetric coefficients (first two values)
    and the other solid (corresponding to `material2`) uses a symmetric coefficient
    (last value). Finally, with four values, both solids use asymmetric
    coefficients, first two for the first solid and last two for the second solid.
    The friction directions and the supported geometric primitives are the same as
    the ones documented with the `coulombFriction` field.

- The `softERP` field defined the *Error Reduction Parameter* ODE used for local contact joints, and `softCFM` the soft *Constraint Force Mixing*. Both are ODE concepts and both are gone; the `WorldInfo` `ERP` / `CFM` fields they mirrored are retired too. The Newton equivalents of contact stiffness and damping are `WorldInfo.newtonContactKe` and `newtonContactKd`.

- ⚠️ The `bumpSound`, `rollSound` and `slideSound` fields **produce no audio.** Contact sound was driven off the ODE contact stream and is dead on Newton — no bump, no roll, no slide. The fields still parse. What they described:
They name URLs to WAVE files used to render the sounds of contacts.
If the value of these fields starts with `http://` or `https://`, OmniSim will get the file from the web.
Otherwise, these URLs are expressed relatively to the world or PROTO file containing the `ContactProperties` node.
`bumpSound` is the sound produced by the impact of a collision.
Its gain is modulated by the energy involved in the collision.
`rollSound` is the sound produced by a rolling object.
Its gain and pitch are modulated by the angular velocities of the bodies in contact.
`slideSound` is the sound produced by the friction of a body sliding on another body.
Its gain and pitch are modulated by the linear velocity of the contact surface.
The formulas affecting the gain and pitch of these sounds were determined empirically to produce fairly realistic sounds.
They are subject to improvements.

- ⚠️ The `maxContactJoints` field has **no effect.** It capped ODE's per-collision contact-joint generation to the deepest N contact points, which mattered because ODE's cost scaled with the cube of the contact-joint count. Newton's contact budget is a **per-world** allocation, not a per-pair cap: see `WorldInfo.newtonNconmax` and `newtonNjmax`, and note that overflowing those is silent and damaging on the `mujoco_warp` path.

