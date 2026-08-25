## Nodes and Keywords

### VRML97 Nodes

OmniSim implements only a subset of the nodes and fields specified by the VRML97 standard.
In the other hand, OmniSim also adds many nodes, which are not part of the VRML97 standard, but are specialized to model robotic experiments.

The following VRML97 nodes are supported by OmniSim:

`Appearance, Background, Box, Color, Cone, Coordinate, Cylinder, DirectionalLight, ElevationGrid, Fog, Group, ImageTexture, IndexedFaceSet, IndexedLineSet, Material, Normal, PointLight, PointSet, Shape, Sphere, SpotLight, TextureCoordinate, TextureTransform, Transform, Viewpoint` and `WorldInfo`.

Please refer to [this chapter](nodes-and-api-functions.md) for a detailed description of OmniSim nodes and fields.
It specifies which fields are actually used.
For a comprehensive description of the VRML97 nodes, you can also refer to the VRML97 documentation.

The exact features of VRML97 are subject to a standard managed by the International Standards Organization (ISO/IEC 14772-1:1997).
You can find the complete specification of VRML97 on the [Web3D Web site](http://www.web3d.org).

### OmniSim Specific Nodes

In order to describe more precisely robotic simulations, OmniSim supports additional nodes that are not specified by the VRML97 standard.
These nodes are principally used to model commonly used robot devices.
Here are OmniSim additional nodes:

`Accelerometer, Altimeter, BallJoint, BallJointParameters, Billboard, Brake, CadShape, Camera, Capsule, Charger, Compass, Connector, ContactProperties, Damping, Display, DistanceSensor, Emitter, Focus, GPS, Gyro, HingeJoint, HingeJointParameters, Hinge2Joint, InertialUnit, JointParameters, LED, Lens, LensFlare, Lidar, LightSensor, LinearMotor, Mesh, Muscle, PBRAppearance, Pen, Physics, Plane, PositionSensor, Propeller, Radar, RangeFinder, Receiver, Recognition, Robot, RotationalMotor, Skin, SliderJoint, Slot, Solid, SolidReference, Speaker, Supervisor, TouchSensor, Track, TrackWheel, VacuumGripper, ` and `Zoom`.

Please refer to [this chapter](nodes-and-api-functions.md) for a detailed description of OmniSim nodes and fields.

### Reserved Keywords

These reserved keywords cannot be used in DEF or PROTO names:

`DEF, USE, PROTO, IS, TRUE, FALSE, NULL, field, w3dField, SFNode, SFColor, SFFloat, SFInt32, SFString, SFVec2f, SFVec3f, SFRotation, SFBool, MFNode, MFColor, MFFloat, MFInt32, MFString, MFVec2f, MFVec3f, MFRotation` and `MFBool`.
