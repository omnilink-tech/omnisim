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

#ifndef OM_PHYSICS_VECTOR_REPRESENTATION_HPP
#define OM_PHYSICS_VECTOR_REPRESENTATION_HPP

//
// Description: class handling the rendering of a force or a torque dragged by the mouse.
// D1.4: the WREN scene objects (arrow/tail/coil renderables) are gone -- the drag arrow is
// now drawn by the wgpu overlay (gui/OmDragArrowLines reads the drag event state directly).
// The class survives state-only so the drag events (OmDragForceEvent/OmDragTorqueEvent)
// keep their construct/update/delete lifecycle unchanged.
//

#include <QtCore/QObject>

// Abstract class //
////////////////////

class OmRotation;
class OmVector3;

class OmPhysicsVectorRepresentation : public QObject {
  Q_OBJECT
public:
  virtual ~OmPhysicsVectorRepresentation();

  void initializeTailAndArrow(const float *materialColor);

  void updatePosition(const OmVector3 &begin, const OmVector3 &end, const OmRotation &orientation);

  void setScale(float scale);

protected:
  OmPhysicsVectorRepresentation() {}
};

// Implemented classes

class OmForceRepresentation : public OmPhysicsVectorRepresentation {
  Q_OBJECT
public:
  OmForceRepresentation();
};

class OmTorqueRepresentation : public OmPhysicsVectorRepresentation {
  Q_OBJECT
public:
  OmTorqueRepresentation();
  virtual ~OmTorqueRepresentation() override;
};

#endif
