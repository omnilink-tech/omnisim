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

#ifndef OM_ABSTRACT_APPEARANCE_HPP
#define OM_ABSTRACT_APPEARANCE_HPP

#include "OmBaseNode.hpp"
#include "OmSFString.hpp"

class OmSFNode;
class OmTextureTransform;

struct aiMaterial;

class OmAbstractAppearance : public OmBaseNode {
  Q_OBJECT

public:
  virtual ~OmAbstractAppearance() override;

  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void reset(const QString &id) override;

  const QString &name() const { return mName->value(); }

  virtual OmTextureTransform *textureTransform() const;

  OmVector2 transformUVCoordinate(const OmVector2 &uv) const;

signals:
  void changed();
  void nameChanged(const QString &newName, const QString &prevName);

protected:
  OmAbstractAppearance(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmAbstractAppearance(const OmAbstractAppearance &other);
  OmAbstractAppearance(const OmNode &other);
  OmAbstractAppearance(const QString &modelName, const aiMaterial *material);

  OmSFNode *mTextureTransform;

private:
  OmAbstractAppearance &operator=(const OmAbstractAppearance &);  // non copyable
  void init();

  OmSFString *mName;
  QString mNameValue;

private slots:
  void updateName();
  void updateTextureTransform();
};

#endif
