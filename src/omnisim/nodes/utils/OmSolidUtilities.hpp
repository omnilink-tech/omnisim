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

#ifndef OM_SOLID_UTILITIES_HPP
#define OM_SOLID_UTILITIES_HPP

#include <QtCore/QVector>

class OmGeometry;
class OmInertia;
class OmNode;


namespace OmSolidUtilities {
  // ODE-free mirror of addMass over OmInertia -- the Newton-side mass-property
  // composer (parity-tested against the dMass pipeline while ODE still ships).
  // Warning-free by design: the dMass pass already emitted the user warnings.
  void addInertia(OmInertia *const inertia, OmNode *const node, double density);
  // Appends the Newton body index of every Solid at or under `root` --
  // including Solids behind OmBasicJoint/OmSlot endPoints, which the ray
  // consumers' original per-file OmGroup-only walks missed (an articulated
  // robot could see its own jointed arm under Newton where ODE's same-robot
  // rule excluded it). The shared exclusion walk for all five ray consumers.
  void collectNewtonBodies(OmNode *const root, QVector<int> &out);
  void subtractInertiaMatrix(double *I, const double *J);
  bool checkBoundingObject(OmNode *const node);  // debug method for testing the validity of a boundingObject
  // extracts the OmGeometry placed into a simple bounding object, i.e. a OmGeometry of a OmShape
  OmGeometry *geometry(OmNode *const node);
  bool isPermanentlyKinematic(const OmNode *node);  // depends on node parent, hence can't be called in node's destructor
};  // namespace OmSolidUtilities

#endif  // OM_SOLID_UTILITIES_HPP
