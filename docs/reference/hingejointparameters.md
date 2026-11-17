## HingeJointParameters

Derived from [JointParameters](jointparameters.md).

```
HingeJointParameters {
  SFVec3f anchor                    0 0 0   # any vector
  SFVec3f axis                      1 0 0   # unit axis
  SFFloat suspensionSpringConstant  0       # [0, inf)
  SFFloat suspensionDampingConstant 0       # [0, inf)
  SFVec3f suspensionAxis            1 0 0   # unit axis
  SFFloat stopERP                   -1      # -1 or [0, inf)
  SFFloat stopCFM                   -1      # -1 or (0, inf)
}
```

> ⚠️ **FIVE OF THE SEVEN FIELDS ON THIS NODE DO NOTHING ON THE CURRENT ENGINE.**
> ODE was deleted on 2026-08-08 (commit `bdc02139`) and Newton/MuJoCo is the only physics backend.
> Live: **`anchor`** and **`axis`** (plus everything inherited from
> [JointParameters](jointparameters.md) that is itself marked live there — notably `position`,
> `minStop`/`maxStop` and `minPosition`/`maxPosition`).
> Retired, still parsed, **not read**:
>
> | field | why it is dead |
> |---|---|
> | `suspensionSpringConstant` | `OmHingeJoint::applyToOdeSuspension()` is an empty function: "UNIMPLEMENTED … Newton has no suspension concept". |
> | `suspensionDampingConstant` | same. |
> | `suspensionAxis` | same — `applyToOdeSuspensionAxis()` is kept only because axis updates call it, and it too does nothing. |
> | `stopERP` | named ODE's constraint error-reduction parameter for joint limits. `applyToOdeStopErp()` is empty; it "has no Newton analogue". Hard limits are now a post-step clamp. |
> | `stopCFM` | same as `stopERP`. |
>
> There is no Newton equivalent to substitute. If you were modelling vehicle suspension with these
> fields, model it as a real [SliderJoint](sliderjoint.md) with a motor instead.

### Description

The [HingeJointParameters](#hingejointparameters) node can be used to specify the hinge rotation axis and various joint parameters (e.g., angular position, stop angles, spring and damping constants etc.) related to this rotation axis.

### Field Summary

- `anchor`: This field specifies the anchor position, i.e. a point through which the hinge axis passes.
Together with the `axis` field inherited from the [JointParameters](jointparameters.md) node, the `anchor` field determines the hinge rotation axis in a unique way.
It is expressed in relative coordinates with respect to the closest upper [Pose](pose.md) node's frame.

- `suspensionSpringConstant`: ⚠️ **not read** (see banner). This field specifies the suspension spring constant along the suspension axis.

- `suspensionDampingConstant`: ⚠️ **not read** (see banner). This field specifies the suspension damping constant along the suspension axis.

- `suspensionAxis`: ⚠️ **not read** (see banner). This field specifies the direction of the suspension axis.

- `stopERP`: ⚠️ **not read** (see banner); `WorldInfo.ERP`, which it defers to, is retired too. This field specifies the local `ERP` used by joint limits. By default it imposes the global `ERP` (value -1) but can be different from it.

- `stopCFM`: ⚠️ **not read** (see banner); `WorldInfo.CFM`, which it defers to, is retired too. This field specifies the local `CFM` used by joint limits. By default it imposes the global `CFM` (value -1) but can be different from it.

⚠️ The paragraph below describes the ODE-era behaviour and is retained as intent only — no spring or damping is applied today.

The `suspensionSpringConstant` and `suspensionDampingConstant` fields can be used to add a linear spring and/or damping behavior *along* the axis defined in `suspensionAxis`.
These fields are described in more detail in [JointParameters](jointparameters.md)'s ["Springs and Dampers"](jointparameters.md#springs-and-dampers) section.
