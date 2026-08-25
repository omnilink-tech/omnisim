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

#ifndef OM_CLIPBOARD_HPP
#define OM_CLIPBOARD_HPP

//
// Description: Singleton class representing the application clipboard
//

#include "OmVariant.hpp"

#include <QtCore/QString>
#include <QtGui/QClipboard>

class OmField;
class QClipboard;

class OmClipboard : public OmVariant {
public:
  static OmClipboard *instance();
  static void deleteInstance();
  OmClipboard &operator=(const OmVariant &other);

  // redefine OmVariant setters
  void setBool(bool b) override;
  void setInt(int i) override;
  void setDouble(double d) override;
  void setString(const QString &s) override;
  void setVector2(const OmVector2 &v) override;
  void setVector3(const OmVector3 &v) override;
  void setColor(const OmRgb &c) override;
  void setRotation(const OmRotation &r) override;

  virtual bool isEmpty() const { return OmVariant::isEmpty() && !mNodeInfo; }
  void clear() override;

  // store copied node value as a string to be used when pasting
  // the isBoundingObjectNode property has to be set manually
  void setNode(OmNode *n, bool persistent = false) override;
  const QString &nodeExportString() const { return mNodeExportString; }
  QString computeNodeExportStringForInsertion(OmNode *parentNode, OmField *field, int fieldIndex) const;
  void replaceAllExternalDefNodesInString();

  struct OmClipboardNodeInfo {
    QString modelName;
    QString nodeModelName;
    QStringList protoParentList;
    QString slotType;
    bool hasADeviceDescendant;
    bool hasAConnectorDescendant;
  };
  const struct OmClipboardNodeInfo *nodeInfo() { return mNodeInfo; }

  QString stringValue();

  // synchronize OmniSim clipboard with system clipboard
  void update();

private:
  explicit OmClipboard();

  OmNode *toNode() const override { return NULL; }
  OmClipboardNodeInfo *mNodeInfo;
  QString mNodeExportString;
  struct LinkedDefNodeDefinitions {
    int position;  // USE position in string
    int type;
    QString defName;
    QString definition;
  };
  QList<struct LinkedDefNodeDefinitions *> mLinkedDefNodeDefinitions;
  int replaceExternalNodeDefinitionInString(QString &nodeString, int index) const;

  QClipboard *mSystemClipboard;
  QString mStringValue;
};

#endif
