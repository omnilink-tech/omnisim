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

#ifndef OM_SHAPE_HPP
#define OM_SHAPE_HPP

#include "OmBaseNode.hpp"
#include "OmGeometry.hpp"
#include "OmSFNode.hpp"

class OmAbstractAppearance;
class OmAppearance;
class OmPbrAppearance;
class OmRay;

class OmShape : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmShape(OmTokenizer *tokenizer = NULL);
  OmShape(const OmShape &other);
  explicit OmShape(const OmNode &other);
  virtual ~OmShape() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SHAPE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  bool isAValidBoundingObject(bool checkOde = true, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void propagateSelection(bool selected) override;
  void reset(const QString &id) override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  // field accessors
  OmAppearance *appearance() const;
  OmPbrAppearance *pbrAppearance() const;
  OmAbstractAppearance *abstractAppearance() const;
  OmGeometry *geometry() const { return dynamic_cast<OmGeometry *>(mGeometry->value()); }
  OmSFNode *geometryField() const { return mGeometry; }
  bool isCastShadowsEnabled() const;

  // infrared related functions
  void pickColor(const OmRay &ray, OmRgb &pickedColor, double *roughness = NULL, double *occlusion = NULL) const;

  // for a shape lying into a boundingObject
  void connectGeometryField() const;
  void disconnectGeometryField() const;

  // ray tracing
  OmBoundingSphere *boundingSphere() const override;

  void setAppearance(OmAppearance *appearance);
  void setPbrAppearance(OmPbrAppearance *appearance);
  void setGeometry(OmGeometry *geometry);
  void updateSegmentationColor(const OmRgb &color) override;

  // export
  bool exportNodeHeader(OmWriter &writer) const override;
  void exportBoundingObjectToW3d(OmWriter &writer) const override;
  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void wrenMaterialChanged();
  void castShadowsChanged();
  void geometryInShapeInserted();

private:
  // user accessible fields
  OmSFNode *mAppearance;
  OmSFNode *mGeometry;
  OmSFBool *mCastShadows;
  OmSFBool *mIsPickable;

  OmShape &operator=(const OmShape &);  // non copyable
  OmNode *clone() const override { return new OmShape(*this); }
  void init();
  void applyMaterialToGeometry();

private slots:
  void updateAppearance();
  void updateGeometry();
  void updateGeometryMaterial();
  void updateBoundingSphere(OmBaseNode *subNode);
  void updateCastShadows();
  void updateIsPickable();
  void createOdeGeom();
};

#endif
