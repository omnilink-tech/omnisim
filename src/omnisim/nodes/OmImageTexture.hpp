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

#ifndef OM_IMAGE_TEXTURE_HPP
#define OM_IMAGE_TEXTURE_HPP

#include "OmBaseNode.hpp"
#include "OmVector2.hpp"

#include <QtCore/QSet>

#include <assimp/material.h>

class OmRgb;
class OmDownloader;

class QImage;
class QIODevice;

class OmImageTexture : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmImageTexture(OmTokenizer *tokenizer = NULL);
  OmImageTexture(const OmImageTexture &other);
  explicit OmImageTexture(const OmNode &other);
  OmImageTexture(const aiMaterial *material, aiTextureType textureType, const QString &parentPath);
  virtual ~OmImageTexture() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_IMAGE_TEXTURE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  void pickColor(const OmVector2 &uv, OmRgb &pickedColor);

  // Texture features
  int width() const;
  int height() const;

  // D1.4: presence test replacing the old `wrenTexture() != NULL` check.
  bool hasImage() const;

  // R4 material fidelity: the decoded RGBA image (ARGB32), for uploading the albedo
  // to the wgpu renderer via OmWgpuImageAdapter::acquireFromQImage. NULL until the
  // texture is loaded (postFinalize). Non-owning.
  const QImage *image() const { return mImage; }

  // external texture (CPU pixels fed by another node, e.g. the Display device)
  void setExternalTexture(const unsigned char *image, int width, int height, double ratioX, double ratioY);
  void removeExternalTexture();

  const QString path() const;

  void setRole(const QString &role) { mRole = role; }

  void exportShallowNode(const OmWriter &writer) const;

  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void changed();

protected:
  bool exportNodeHeader(OmWriter &writer) const override;
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

private:
  // user accessible fields
  OmMFString *mUrl;
  OmSFBool *mRepeatS;
  OmSFBool *mRepeatT;
  OmSFInt *mFiltering;

  // The following attributes are used when a texture is coming from an external source (e.g Display device)
  bool mExternalTexture;
  OmVector2 mExternalTextureRatio;
  const unsigned char *mExternalTextureData;
  int mExternalTextureWidth;
  int mExternalTextureHeight;

  QImage *mImage;
  int mUsedFiltering;
  bool mIsMainTextureTransparent;
  QString mRole;  // Role in a PBR appearance.
  OmDownloader *mDownloader;

  OmImageTexture &operator=(const OmImageTexture &);  // non copyable
  OmNode *clone() const override { return new OmImageTexture(*this); }
  void init();
  void initFields();
  // D1.4: despite the names these now manage only the CPU image lifecycle (the wgpu
  // texture source); they keep the exact load/replace/clear timing of the WREN era.
  void updateWrenTexture();
  void destroyWrenTexture();
  bool loadTexture();
  bool loadTextureData(QIODevice *device);

private slots:
  void updateUrl();
  void updateRepeatS();
  void updateRepeatT();
  void updateFiltering();
  void downloadUpdate();
};

#endif
