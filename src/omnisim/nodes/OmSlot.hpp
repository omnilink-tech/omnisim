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

#ifndef OM_SLOT_HPP
#define OM_SLOT_HPP

#include "OmBaseNode.hpp"
#include "OmSFNode.hpp"
#include "OmSFString.hpp"

class OmGroup;
class OmSolidReference;

class OmSlot : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSlot(OmTokenizer *tokenizer = NULL);
  OmSlot(const OmSlot &other);
  explicit OmSlot(const OmNode &other);
  virtual ~OmSlot() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SLOT; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void validateProtoNode() override;
  void write(OmWriter &writer) const override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  void attachResizeManipulator() override;
  void detachResizeManipulator() const override;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;
  void updateSegmentationColor(const OmRgb &color) override;

  // field accessors
  bool hasEndPoint() const { return mEndPoint->value() != NULL; }
  OmSFNode *endPointField() const { return mEndPoint; }
  OmNode *endPoint() const { return mEndPoint->value(); }
  OmSolid *solidEndPoint() const;
  OmSolidReference *solidReferenceEndPoint() const;
  OmSlot *slotEndPoint() const;
  OmGroup *groupEndPoint() const;
  const QString &slotType() const { return mSlotType->value(); }

  void setEndPoint(OmNode *node);

  // selection
  void propagateSelection(bool selected) override;

  // bounding sphere
  OmBoundingSphere *boundingSphere() const override;

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

  QString endPointName() const override;

signals:
  void endPointInserted(OmBaseNode *);  // called when a node is inserted in the endPoint

private:
  OmSlot &operator=(const OmSlot &);  // non copyable
  OmNode *clone() const override { return new OmSlot(*this); }
  void init();

  // user accessible fields
  OmSFNode *mEndPoint;
  OmSFString *mSlotType;

private slots:
  void endPointChanged();
  void updateType();
};

#endif
