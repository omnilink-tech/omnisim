## Structure of ODE Objects

This table shows how common ".wbt" constructs are mapped to ODE objects.
This information shall be useful for implementing physics plugins.

%figure "Mapping between common OmniSim constructs and ODE objects."

| OmniSim construct                              | ODE construct                                  |
| --------------------------------------------- | ---------------------------------------------- |
| Solid { physics Physics {...} }               | dBodyID                                        |
| Solid { boundingObject ... }                  | dGeomID                                        |
| Solid { boundingObject Box {...} }            | dGeomID (dBoxClass)                            |
| Solid { boundingObject Sphere {...} }         | dGeomID (dSphereClass)                         |
| Solid { boundingObject Capsule {...} }        | dGeomID (dCapsuleClass)                        |
| Solid { boundingObject Cylinder {...} }       | dGeomID (dCylinderClass)                       |
| Solid { boundingObject Plane {...} }          | dGeomID (dPlaneClass)                          |
| Solid { boundingObject IndexedFaceSet {...} } | dGeomID (dTriMeshClass)                        |
| Solid { boundingObject Mesh {...} }           | dGeomID (dTriMeshClass)                        |
| Solid { boundingObject ElevationGrid {...} }  | dGeomID (dHeightfieldClass)                    |
| Solid { boundingObject Pose {...} }           | dGeomID (child geometry's geom, with the Pose offset baked in) |
| Solid { boundingObject Group {...} }          | dSpaceID (dSimpleSpaceClass)                   |
| BallJoint { }                                 | dJointID (dJointTypeBall)                      |
| HingeJoint { }                                | dJointID (dJointTypeHinge)                     |
| Hinge2Joint { }                               | dJointID (dJointTypeHinge2)                    |
| SliderJoint { }                               | dJointID (dJointTypeSlider)                    |

%end

> **Note**: Although a physics plugin grants you access to the `dGeomIDs` created and managed by OmniSim, you should never attempt to set a user-defined data pointer by means of the `dGeomSetData` function for these `dGeomIDs` as OmniSim stores its own data pointer in them.
Using the `dGeomSetData` function on a `dGeomID` defined by OmniSim will almost surely result into an OmniSim crash.
