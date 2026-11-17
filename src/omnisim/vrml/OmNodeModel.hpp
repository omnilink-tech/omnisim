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

#ifndef OM_NODE_MODEL_HPP
#define OM_NODE_MODEL_HPP

//
// Description: a class for managing built-in node definitions stored in .wrl files
//

#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QStringList>

class OmFieldModel;
class OmTokenizer;

class OmNodeModel {
public:
  // static functions
  static OmNodeModel *findModel(const QString &modelName);
  static bool fuzzyParseNode(const QString &fileName, QString &nodeInfo);

  // alphabetical list of all nodes, i.e. ("Accelerometer", "Appearance", "Background", "Box" ...)
  static QStringList baseModelNames();
  static bool isBaseModelName(const QString &modelName);

  // backward compatibility
  static QString compatibleNodeName(const QString &modelName);

  // node name, e.g. "Transform", "Solid" ...
  const QString &name() const { return mName; }

  // the info comments (#) found at the beginning of the .wrl or .proto file
  const QString &info() const { return mInfo; }

  // field models
  OmFieldModel *findFieldModel(const QString &fieldName) const;
  const QList<OmFieldModel *> &fieldModels() const { return mFieldModels; }
  QStringList fieldNames() const;

  // parent nodes (only defined for base nodes. For proto nodes, see
  // OmProtoModel::ancestorProtoName/ancestorProtoNameModel/baseType)
  const QString &parentName() const { return mParentName; };
  const OmNodeModel *parentModel() const { return mParentModel; };

  QStringList documentationBookAndPage() const { return QStringList() << "reference" << mName.toLower(); }

private:
  explicit OmNodeModel(OmTokenizer *tokenizer);
  ~OmNodeModel();

  QString mInfo;
  QString mName;
  QList<OmFieldModel *> mFieldModels;
  QString mParentName;
  OmNodeModel *mParentModel;

  static OmNodeModel *readModel(const QString &fileName);
  static void readAllModels();
  static void cleanup();
  static QMap<QString, OmNodeModel *> cModels;
};

#endif
