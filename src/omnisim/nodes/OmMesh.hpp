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

#ifndef OM_MESH_HPP
#define OM_MESH_HPP

#include "OmTriangleMeshGeometry.hpp"

class OmDownloader;
class OmMFString;
struct aiScene;

class OmMesh : public OmTriangleMeshGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmMesh(OmTokenizer *tokenizer = NULL);
  OmMesh(const OmMesh &other);
  explicit OmMesh(const OmNode &other);
  virtual ~OmMesh() override;

  void updateTriangleMesh(bool issueWarnings = true) override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MESH; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createResizeManipulator() override;
  void rescale(const OmVector3 &scale) override{};

  // OmTriangleMesh management (see OmTriangleMeshCache.hpp)
  uint64_t computeHash() const override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

private:
  // user accessible fields
  OmMFString *mUrl;
  OmSFBool *mCcw;
  OmSFString *mName;
  OmSFInt *mMaterialIndex;
  bool mIsCollada;
  OmDownloader *mDownloader;
  bool mBoundingObjectNeedUpdate;

  OmMesh &operator=(const OmMesh &);  // non copyable
  OmNode *clone() const override { return new OmMesh(*this); }
  void init();
  bool checkIfNameExists(const aiScene *scene, const QString &name) const;

private slots:
  void updateUrl();
  void updateCcw();
  void updateName();
  void updateMaterialIndex();
  void downloadUpdate();
};

#endif
