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

#ifndef OM_HIDDEN_KINEMATIC_PARAMETERS_HPP
#define OM_HIDDEN_KINEMATIC_PARAMETERS_HPP

#include <QtCore/QMap>

class OmField;
class OmRotation;
class OmVector3;

namespace OmHiddenKinematicParameters {

  typedef QMap<int, OmVector3 *> PositionMap;

  class HiddenKinematicParameters {
  public:
    HiddenKinematicParameters();
    HiddenKinematicParameters(const OmVector3 *t, const OmRotation *r, PositionMap *p, const OmVector3 *l,
                              const OmVector3 *a);
    ~HiddenKinematicParameters();
    const OmVector3 *translation() const;
    const OmRotation *rotation() const;
    const PositionMap *positions() const;
    const OmVector3 *linearVelocity() const;
    const OmVector3 *angularVelocity() const;
    void createTranslation(double x, double y, double z);
    void createRotation(double x, double y, double z, double angle);
    void createLinearVelocity(double x, double y, double z);
    void createAngularVelocity(double x, double y, double z);
    // cppcheck-suppress constParameterPointer
    void insertPositions(int index, OmVector3 *positions);
    OmVector3 *positions(int index);

  private:
    const OmVector3 *mTranslation;
    const OmRotation *mRotation;
    PositionMap *mPositions;
    const OmVector3 *mLinearVelocity;
    const OmVector3 *mAngularVelocity;
    bool mTranslationIsCreated;
    bool mRotationIsCreated;
  };

  typedef QMap<int, HiddenKinematicParameters *> HiddenKinematicParametersMap;
  void createHiddenKinematicParameter(const OmField *field, HiddenKinematicParametersMap &map);
};  // namespace OmHiddenKinematicParameters

#endif  // OM_HIDDEN_KINEMATIC_PARAMETERS_HPP
