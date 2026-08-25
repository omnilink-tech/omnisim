// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#ifndef OM_ODE_TYPES_HPP
#define OM_ODE_TYPES_HPP

typedef struct dxSpace *dSpaceID;
typedef struct dxBody *dBodyID;
typedef struct dxJoint *dJointID;
typedef struct dxWorld *dWorldID;
typedef struct dxGeom *dGeomID;
typedef struct dxFluid *dFluidID;
// These WERE identical to the typedefs in <ode/common.h> / <ode/fluid_dynamics/
// common_fluid_dynamics.h>. Those headers are gone (src/ode and include/ode were
// deleted), so THIS FILE IS NOW THE ONLY DEFINITION of the handle types, and it
// is the compatibility keystone that keeps the ~44 handle-carrying engine
// headers compiling. Keep it unconditional; the handles are opaque pointers to
// structs that are never defined, so nothing can dereference one by accident.
typedef struct dxJointGroup *dJointGroupID;
typedef struct dxImmersionLinkGroup *dImmersionLinkGroupID;

// Value-type mirrors, so the few headers that carry ODE VALUE types
// (OmOdeContact.hpp's dContactGeom member, OmWorld's QList<dImmersionGeom>,
// OmConnector.hpp's dQuaternion parameters) keep their class shape. Layout was
// copied from the deleted <ode/common.h> (dDOUBLE build) and <ode/contact.h>;
// with ODE gone these are simply the definitions. They are type declarations
// only -- no physics code runs behind them and nothing here fabricates a
// runtime value.
typedef double dReal;
typedef dReal dVector3[4];
typedef dReal dVector4[4];
typedef dReal dMatrix3[4 * 3];
typedef dReal dMatrix4[4 * 4];
typedef dReal dQuaternion[4];
typedef struct dContactGeom {
  dVector3 pos;
  dVector3 normal;
  dReal depth;
  dGeomID g1, g2;
  int side1, side2;
} dContactGeom;
// Placeholder: only ever default-constructed through OmWorld's (permanently
// empty) QList<dImmersionGeom> member. Its producer -- the immersion branch of
// odeNearCallback -- and every reader were deleted with ODE.
struct dImmersionGeom {};

#endif
