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

#ifndef OM_TRANSFORM_HPP
#define OM_TRANSFORM_HPP

//
// Description: extends the Pose node by implementing a scaling factor
//

#include "OmMatrix3.hpp"
#include "OmPose.hpp"

class OmBaseNode;

class OmTransform : public OmPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmTransform(OmTokenizer *tokenizer = NULL);
  OmTransform(const OmTransform &other);
  explicit OmTransform(const OmNode &other);

  // reimplemented functions
  int nodeType() const override { return WB_NODE_TRANSFORM; }
  void preFinalize() override;
  void postFinalize() override;

  void setScaleNeedUpdate() override;

  QStringList fieldsToSynchronizeWithW3d() const override;

  const OmVector3 &scale() const { return mScale->value(); }
  OmSFVector3 *scaleFieldValue() const { return mScale; }

  void setScale(double x, double y, double z) { mScale->setValue(x, y, z); };
  void setScale(const OmVector3 &s) { mScale->setValue(s); }
  void setScale(int coordinate, double s) { mScale->setComponent(coordinate, s); }

  // scaling
  const OmVector3 &absoluteScale() const;

  mutable OmVector3 mAbsoluteScale;
  mutable bool mAbsoluteScaleNeedUpdate;

  // 3x3 absolute rotation matrix
  const OmMatrix4 &vrmlMatrix() const override;

protected:
  void applyToScale();

  // A specific scale check is done in the OmSolid class
  OmSFVector3 *mScale;
  double mPreviousXscaleValue;
  void sanitizeScale();

  void setScaleNeedUpdateFlag() const;

protected slots:
  void updateScale(bool warning = false);

private:
  OmTransform &operator=(const OmTransform &);  // non copyable
  OmNode *clone() const override { return new OmTransform(*this); }
  void init();

  void applyToOdeScale();

  void updateAbsoluteScale() const;
  void updateMatrix() const override;
};

#endif
