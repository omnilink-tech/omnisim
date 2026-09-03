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

#ifndef OM_SOLID_MERGER_HPP
#define OM_SOLID_MERGER_HPP

#include "OmVector3.hpp"

#include <QtCore/QMap>
#include <QtCore/QObject>

class OmMatrix4;
class OmSolid;

class OmSolidMerger : public QObject {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSolidMerger(OmSolid *solid);
  virtual ~OmSolidMerger();

  OmSolid *solid() const { return mSolid; }
  void removeExtraSpace();
  const OmVector3 &centerOfMass() const { return mCenterOfMass; }
  const OmVector3 &absoluteCenterOfMass() const { return mAbsoluteCenterOfMass; }
  void appendSolid(OmSolid *solid);
  void removeSolid(OmSolid *solid);
  void mergeMass(OmSolid *const solid, bool subtract = true);
  void setGeomAndBodyPositions(bool zeroVelocities = false, bool resetJoints = false);
  void setupOdeBody();
  void updateMasses();
  bool isSet() const;

  void setBodyArtificiallyDisabled(bool disabled);
  bool isBodyArtificiallyDisabled() const { return mBodyArtificiallyDisabled; }

public slots:
  void setOdeDamping();

private:
  OmSolidMerger(const OmSolidMerger &other);
  OmSolidMerger &operator=(const OmSolidMerger &other);
  OmSolid *mSolid;
  OmVector3 mCenterOfMass;
  OmVector3 mAbsoluteCenterOfMass;
  void updateCenterOfMass();
  bool mBodyArtificiallyDisabled;

  void addMassToBody();
  void mergeMasses();
  void setGeomOffsetPositions();
  void subtractSolidMass(OmSolid *solid);
  void transformMass(OmSolid *const solid, const OmMatrix4 &m4) const;
  void transformMass(OmSolid *const solid) const;
  void transformMasses() const;
  void reserveSpace();
  OmMatrix4 inverseMatrix() const;

private slots:
  void setOdeAutoDisable();
};
#endif
