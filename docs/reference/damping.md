## Damping

```
Damping {
  SFFloat linear  0.2    # [0, 1]
  SFFloat angular 0.2    # [0, 1]
}
```

> ⚠️ **THE `Damping` NODE HAS NO EFFECT.** It was implemented entirely on the ODE path
> (`dBodySetDamping`, applied through `OmSolidMerger::setOdeDamping`), and ODE was deleted on
> 2026-08-08 (commit `bdc02139`). `setOdeDamping()` is now an empty function and damping is **not
> plumbed to the Newton/MuJoCo backend**, so neither `WorldInfo.defaultDamping` nor a
> [Physics](physics.md) node's `damping` field changes anything. The node is still declared so that
> legacy worlds parse — removing it would turn every world that sets it into a load ERROR — but it
> is inert.
>
> **There is no `Damping` replacement.** If you need a body to lose speed, apply the loss yourself:
> a controller can subtract velocity each tick, and joint-level losses can be modelled with motor
> damping gains. Whether Newton-side body damping should be wired up is an **open question** — it
> has not been decided.
>
> The description below records what the node did while ODE shipped. Read it as history.

### Description

A [Damping](#damping) node could be used to slow down a body (a [Solid](solid.md) node with [Physics](physics.md)).
The speed of each body was reduced by the specified amount (between 0.0 and 1.0) every second.
A value of 0.0 meant "no slowing down" and a value of 1.0 meant a "complete stop"; a value of 0.1 meant the speed was decreased by 10 percent every second.
The behavior of this value on solid speeds was nonlinear: a linear damping of 0.99 was far from affecting solid speeds as a linear damping of 1.0 did.
A damped body could come to rest and become disabled depending on the values specified in [WorldInfo](worldinfo.md) — itself another ODE-only mechanism, since Newton has no body sleep.
Damping added no force to the simulation; it directly affected the velocity of the body, after all forces had been applied.
It was commonly used to reduce simulation instability.

> **Note**: When several rigidly linked [Solid](solid.md)s are merged (see [Physics](physics.md)'s [solid merging](physics.md#implicit-solid-merging-and-joints) section) damping values of the aggregate body were averaged over the volumes of all [Solid](solid.md) components.
The volume of a [Solid](solid.md) is the sum of the volumes of the geometries found in its `boundingObject`; overlaps are not handled.

The `linear` field indicated the amount of damping applied to the body's linear motion.
The `angular` field indicated the amount of damping applied to the body's angular motion.
Linear damping was used, e.g., to slow down a vehicle by simulating air or water friction; angular damping to slow the rotation of a rolling ball or the spin of a coin.
Damping was applied regardless of the shape of the object, so it could never model complex fluid dynamics.
(The `ImmersionProperties` and `Fluid` nodes that once handled buoyancy and drag were **removed with ODE** — fluid interaction is not simulated at all today.)

A [Damping](#damping) node can still be written in the `defaultDamping` field of the [WorldInfo](worldinfo.md) node, or in the `damping` field of a [Physics](physics.md) node, and the `Physics`-level one still overrides the default — but neither has any effect on the simulation.
