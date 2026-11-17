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

#ifndef OM_INDEXED_FACE_SET_HPP
#define OM_INDEXED_FACE_SET_HPP

#include "OmTriangleMeshGeometry.hpp"

class OmCoordinate;
class OmNormal;
class OmTextureCoordinate;
class OmVector3;

class OmIndexedFaceSet : public OmTriangleMeshGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmIndexedFaceSet(OmTokenizer *tokenizer = NULL);
  OmIndexedFaceSet(const OmIndexedFaceSet &other);
  explicit OmIndexedFaceSet(const OmNode &other);
  virtual ~OmIndexedFaceSet() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INDEXED_FACE_SET; }
  void preFinalize() override;
  void postFinalize() override;
  void createResizeManipulator() override;
  void attachResizeManipulator() override;
  void reset(const QString &id) override;

  // field accessors
  OmCoordinate *coord() const;
  OmNormal *normal() const;
  OmTextureCoordinate *texCoord() const;
  const OmMFInt *coordIndex() const { return static_cast<const OmMFInt *>(mCoordIndex); }
  const OmMFInt *normalIndex() const { return static_cast<const OmMFInt *>(mNormalIndex); }
  const OmMFInt *texCoordIndex() const { return static_cast<const OmMFInt *>(mTexCoordIndex); }
  const OmSFDouble *creaseAngle() const { return static_cast<const OmSFDouble *>(mCreaseAngle); }
  const OmSFBool *ccw() const { return static_cast<const OmSFBool *>(mCcw); }
  const OmSFBool *normalPerVertex() const { return static_cast<const OmSFBool *>(mNormalPerVertex); }

  // Rescaling and translating
  void rescale(const OmVector3 &v) override;
  void rescaleAndTranslate(int coordinate, double scale, double translation);
  void rescaleAndTranslate(double factor, const OmVector3 &t);
  void rescaleAndTranslate(const OmVector3 &scale, const OmVector3 &translation);
  void translate(const OmVector3 &v);

  void updateTriangleMesh(bool issueWarnings = true) override;

  uint64_t computeHash() const override;

  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void validIndexedFaceSetInserted();

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;
  bool exportNodeHeader(OmWriter &writer) const override;

private:
  OmIndexedFaceSet &operator=(const OmIndexedFaceSet &);  // non copyable
  OmNode *clone() const override { return new OmIndexedFaceSet(*this); }
  void init();

  // user accessible fields
  OmSFNode *mCoord;
  OmSFNode *mNormal;
  OmSFNode *mTexCoord;
  OmSFBool *mCcw;
  OmSFBool *mNormalPerVertex;
  OmMFInt *mCoordIndex;
  OmMFInt *mNormalIndex;
  OmMFInt *mTexCoordIndex;
  OmSFDouble *mCreaseAngle;

private slots:
  void updateCoord();
  void updateNormal();
  void updateTexCoord();
  void updateCcw();
  void updateNormalPerVertex();
  void updateCoordIndex();
  void updateNormalIndex();
  void updateTexCoordIndex();
  void updateCreaseAngle();
};

#endif
