# Assets

OmniSim ships its asset library **locally, inside the checkout** — there is no cloud catalog to browse, and a stock install fetches nothing over the network at simulation time.
The library is a set of [PROTO](../reference/proto.md) nodes under `projects/`:

| Directory | Contents |
| --- | --- |
| [`projects/objects/`](https://github.com/omnilink-tech/omnisim/tree/main/projects/objects) | ~313 object PROTOs — furniture, buildings, traffic, industrial, household, kitchen, garden, street furniture, and more. |
| [`projects/appearances/`](https://github.com/omnilink-tech/omnisim/tree/main/projects/appearances) | ~68 [PBR appearance](../reference/pbrappearance.md) PROTOs — metals, woods, fabrics, concrete, plastics, ground materials. |
| [`projects/robots/`](https://github.com/omnilink-tech/omnisim/tree/main/projects/robots) | Robot models, grouped by manufacturer. |

## Browsing the library

The simplest way to explore the assets is the **Add Node dialog** in the OmniSim GUI: select a node in the scene tree, press the `+` button, then browse or keyword-search the bundled PROTOs.
Every PROTO carries a `keywords:` header tag (`furniture/table`, `appearance/mineral`, `exterior/building`, …), and the dialog searches those tags alongside the node name and description.
The list it reads is `resources/proto-list.xml`, generated from the PROTO headers themselves — it is a local file, so the dialog works fully offline.

## Adding your own assets

Drop a new `.proto` file into your project's `protos/` directory and it is picked up automatically — the Add Node dialog lists project PROTOs alongside the bundled ones.
To contribute an asset back to OmniSim, open a pull request against the [OmniSim repository](https://github.com/omnilink-tech/omnisim); see [CONTRIBUTING.md](https://github.com/omnilink-tech/omnisim/blob/main/CONTRIBUTING.md).

A world may also reference a PROTO by `https://` URL, in which case OmniSim downloads and caches it.
That is opt-in per world — nothing in a stock OmniSim install resolves assets remotely.

## Robots

Robot models are listed in the Add Node dialog under the *robot* keyword.
They derive from the [Robot](../reference/robot.md) node and include wheeled robots, tracked robots, legged robots, robotic arms and drones.
OmniSim also imports URDF directly — see the `URDFRobot` node — which is usually the fastest way to bring in a robot that is not already modelled.

## Actuators

Actuator assets are listed under the *actuator* keyword.
They typically correspond to commercially available devices.
For the generic actuator nodes they build on, see the [Actuators](actuators.md) section.

## Sensors

Sensor assets are listed under the *sensor* keyword.
They typically correspond to commercially available devices.
For the generic sensor nodes they build on, see the [Sensors](sensors.md) section.

## Joints

OmniSim already disposes of several nodes to model the transmission of power across bodies, such as [HingeJoint](../reference/hingejoint.md), [Hinge2Joint](../reference/hinge2joint.md), [BallJoint](../reference/balljoint.md) and [SliderJoint](../reference/sliderjoint.md).
However, in some instances it is necessary to more accurately describe the real behavior by taking into consideration factors like, for example, the clearance among gears in a drive train, which induces a gear latch.
The PROTO nodes under the *joint* keyword allow that more accurate modeling.

## Appearances

All the [PBR appearance](../reference/pbrappearance.md) assets live in [`projects/appearances/`](https://github.com/omnilink-tech/omnisim/tree/main/projects/appearances).
They represent various materials and can be customized and re-used to create the appearance of other PROTO nodes.
