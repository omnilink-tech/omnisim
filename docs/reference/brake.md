## Brake

Derived from [Device](device.md).

```
Brake {
}
```

> ⚠️ **THE `Brake` NODE DOES NOTHING ON THE CURRENT ENGINE. Do not reach for it to slow a joint.**
> Braking was implemented as ODE joint damping, and ODE was deleted on 2026-08-08
> (commit `bdc02139`). Newton/MuJoCo is the only physics backend and no equivalent is wired:
> `OmHingeJoint::applyToOdeSpringAndDampingConstants` records that "JointParameters
> `springConstant` / `dampingConstant` **and Brake's `brakingDampingConstant`** are now
> UNIMPLEMENTED … no Newton equivalent is wired". The node still parses and
> `wb_brake_set_damping_constant` still returns normally — it is accepted and silently ignored,
> which is the expensive failure mode, so it is called out here rather than left to be discovered.
>
> **What to do instead:** brake a joint from the controller — set the motor's velocity target to 0
> (`wb_motor_set_velocity`), or hold a position target. The `PositionSensor` and the motor PD servo
> are live on Newton; the passive-damping path is not.

### Description

A [Brake](#brake) node can be used in a mechanical simulation in order to change the friction of a joint.
The [Brake](#brake) node can be inserted in the `device` field of a [HingeJoint](hingejoint.md), a [Hinge2Joint](hinge2joint.md), a [SliderJoint](sliderjoint.md), or a Track (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)).

### Brake Functions

#### `wb_brake_set_damping_constant`
#### `wb_brake_get_type`

%tab-component "language"

%tab "C"

```c
#include <omnisim/brake.h>

void wb_brake_set_damping_constant(WbDeviceTag tag, double damping_constant);
int wb_brake_get_type(WbDeviceTag tag);
```
%tab-end

%tab "C++"

```cpp
#include <omnisim/Brake.hpp>

namespace omnisim {
  class Brake : public Device {
    enum {ROTATIONAL, LINEAR};

    void setDampingConstant(double dampingConstant) const;
    int getType() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from omnisim import Brake

class Brake (Device):
    ROTATIONAL, LINEAR

    def setDampingConstant(self, dampingConstant):
    def getType(self):
    # ...
```

%tab-end

%end

##### Description

*set the damping constant coefficient of the joint and get the type of brake*

⚠️ **`wb_brake_set_damping_constant` has no effect on the current engine** — see the banner at the top of this page. The description below is the intended semantics, retained because the API is unchanged and may be re-wired to Newton; it is not what happens today.

The `wb_brake_set_damping_constant` function sets the value of the dampingConstant coefficient (Ns/m or Nms) of the joint.
If any dampingConstant is already set using [JointParameters](jointparameters.md) the resulting dampingConstant coefficient is the sum of the one in the [JointParameters](jointparameters.md) and the one set using the `wb_brake_set_damping_constant` function.

The `wb_brake_get_type` function returns the type of the brake.
It will return `WB_ROTATIONAL` if the sensor is associated with a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md) node, and `WB_LINEAR` if it is associated with a [SliderJoint](sliderjoint.md) or a Track (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) node.

---

#### `wb_brake_get_motor`
#### `wb_brake_get_position_sensor`

%tab-component "language"

%tab "C"

```c
#include <omnisim/brake.h>
#include <omnisim/motor.h>
#include <omnisim/position_sensor.h>

WbDeviceTag wb_brake_get_motor(WbDeviceTag tag);
WbDeviceTag wb_brake_get_position_sensor(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <omnisim/Brake.hpp>
#include <omnisim/Motor.hpp>
#include <omnisim/PositionSensor.hpp>

namespace omnisim {
  class Brake : public Device {
    Motor *getMotor() const;
    PositionSensor *getPositionSensor() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from omnisim import Brake, Motor, PositionSensor

class Brake (Device):
    def getMotor(self):
    def getPositionSensor(self):
    # ...
```

%tab-end

%end

##### Description

*get associated devices*

The `wb_brake_get_motor` and `wb_brake_get_position_sensor` functions return the [Motor](motor.md) and [PositionSensor](positionsensor.md) instances defined in the same [Joint](joint.md) or Track (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) `device` field.
If none is defined they return 0.
