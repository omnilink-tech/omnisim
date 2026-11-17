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

//
//  OmTrack.hpp
//

// Implemented node class representing a wheel of the OmTrack node
//   special OmPose node where 'translation', 'rotation, 'translationStep'
//   and 'rotationStep' fields are not open to the user but defined internally

#ifndef OM_TRACK_WHEEL_HPP
#define OM_TRACK_WHEEL_HPP

#include "OmPose.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector2.hpp"

class OmTrackWheel : public OmPose {
  Q_OBJECT
public:
  explicit OmTrackWheel(OmTokenizer *tokenizer = NULL);
  OmTrackWheel(const OmTrackWheel &other);
  explicit OmTrackWheel(const OmNode &other);
  virtual ~OmTrackWheel() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TRACK_WHEEL; }
  void preFinalize() override;
  void postFinalize() override;
  void write(OmWriter &writer) const override;
  QStringList fieldsToSynchronizeWithW3d() const override;
  void exportNodeFields(OmWriter &writer) const override;
  bool shallExport() const override;

  const OmVector2 position() const { return mPosition->value(); }
  double radius() const { return mRadius->value(); }
  bool inner() const { return mInner->value(); }

  void rotate(double traveledDistance);

signals:
  void changed();

private:
  OmTrackWheel &operator=(const OmTrackWheel &);  // non copyable
  OmNode *clone() const override { return new OmTrackWheel(*this); }
  void init();

  OmSFVector2 *mPosition;
  OmSFDouble *mRadius;
  OmSFBool *mInner;

private slots:
  void updatePosition();
  void updateRadius();
};

#endif
