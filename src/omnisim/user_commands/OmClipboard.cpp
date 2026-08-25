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

#include "OmClipboard.hpp"

#include "OmBaseNode.hpp"
#include "OmDictionary.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProtoModel.hpp"
#include "OmRgb.hpp"
#include "OmRotation.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include "../../../include/controller/c/omnisim/supervisor.h"

#include <QtGui/QClipboard>
#include <QtWidgets/QApplication>
#include <cassert>

static OmClipboard *gInstance = NULL;

OmClipboard *OmClipboard::instance() {
  if (!gInstance)
    gInstance = new OmClipboard();

  return gInstance;
}

void OmClipboard::deleteInstance() {
  if (!gInstance)
    return;
  gInstance->clear();
  delete gInstance;
  gInstance = NULL;
}

OmClipboard::OmClipboard() : OmVariant(), mNodeInfo(NULL), mSystemClipboard(QApplication::clipboard()) {
  update();
}

OmClipboard &OmClipboard::operator=(const OmVariant &other) {
  mSystemClipboard->blockSignals(true);
  OmVariant::setValue(other);
  mSystemClipboard->blockSignals(false);
  return *this;
}

void OmClipboard::update() {
  const QString systemClipboardValue = mSystemClipboard->text();
  if (systemClipboardValue != mStringValue) {
    OmVariant::setString(systemClipboardValue);
    mStringValue = systemClipboardValue;
  }
}

QString OmClipboard::stringValue() {
  update();
  return mStringValue;
}

void OmClipboard::setBool(bool b) {
  OmVariant::setBool(b);
  mStringValue = b ? "true" : "false";

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setInt(int i) {
  OmVariant::setInt(i);
  mStringValue.setNum(i);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setDouble(double d) {
  OmVariant::setDouble(d);
  mStringValue.setNum(d);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setString(const QString &s) {
  OmVariant::setString(s);
  mStringValue = s;

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setVector2(const OmVector2 &v) {
  OmVariant::setVector2(v);
  mStringValue = v.toString(OmPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setVector3(const OmVector3 &v) {
  OmVariant::setVector3(v);
  mStringValue = v.toString(OmPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setColor(const OmRgb &c) {
  OmVariant::setColor(c);
  mStringValue = c.toString(OmPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setRotation(const OmRotation &r) {
  OmVariant::setRotation(r);
  mStringValue = r.toString(OmPrecision::DOUBLE_MAX);

  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::setNode(OmNode *n, bool persistent) {
  if (!n)
    return;

  OmVariant::setNode(NULL, persistent);
  assert(!mNodeInfo);
  mNodeInfo = new OmClipboardNodeInfo();
  mNodeInfo->modelName = n->modelName();
  mNodeInfo->nodeModelName = n->nodeModelName();
  mNodeInfo->protoParentList = n->proto() ? n->proto()->parentProtoNames() : QStringList();
  mNodeInfo->slotType = OmNodeUtilities::slotType(n);
  mNodeInfo->hasADeviceDescendant = OmNodeUtilities::hasADeviceDescendant(n, true);
  mNodeInfo->hasAConnectorDescendant = mNodeInfo->hasADeviceDescendant || OmNodeUtilities::hasADeviceDescendant(n, false);
  OmNode::enableDefNodeTrackInWrite(false);
  mNodeExportString = OmVrmlNodeUtilities::exportNodeToString(n);
  QList<std::pair<OmNode *, int>> externalDefNodes(*OmNode::externalUseNodesPositionsInWrite());
  OmNode::disableDefNodeTrackInWrite();
  // store all the required external DEF nodes data in order to work correctly
  // independently if other nodes are deleted
  for (int i = 0; i < externalDefNodes.size(); ++i) {
    const OmBaseNode *node = dynamic_cast<OmBaseNode *>(externalDefNodes[i].first);
    LinkedDefNodeDefinitions *data = new LinkedDefNodeDefinitions();
    data->position = externalDefNodes[i].second;
    data->type = node->nodeType();
    data->defName = node->defName();
    OmNode::enableDefNodeTrackInWrite(false);
    data->definition = OmVrmlNodeUtilities::exportNodeToString(node);
    OmNode::disableDefNodeTrackInWrite();
    mLinkedDefNodeDefinitions.append(data);
  }

  mStringValue = n->fullName();
  mSystemClipboard->blockSignals(true);
  mSystemClipboard->setText(mStringValue, QClipboard::Clipboard);
  mSystemClipboard->blockSignals(false);
}

void OmClipboard::clear() {
  OmVariant::clear();
  delete mNodeInfo;
  mNodeInfo = NULL;
  mNodeExportString.clear();
  for (int i = 0; i < mLinkedDefNodeDefinitions.size(); ++i)
    delete mLinkedDefNodeDefinitions[i];
  mLinkedDefNodeDefinitions.clear();
}

int OmClipboard::replaceExternalNodeDefinitionInString(QString &nodeString, int index) const {
  int position = mLinkedDefNodeDefinitions[index]->position;
  int oldStringSize = mLinkedDefNodeDefinitions[index]->defName.size() + 4;
  nodeString.remove(position, oldStringSize);
  const QString &newString = mLinkedDefNodeDefinitions[index]->definition;
  nodeString.insert(position, newString);
  return newString.size() - oldStringSize;
}

void OmClipboard::replaceAllExternalDefNodesInString() {
  while (!mLinkedDefNodeDefinitions.isEmpty()) {
    replaceExternalNodeDefinitionInString(mNodeExportString, mLinkedDefNodeDefinitions.size() - 1);
    mLinkedDefNodeDefinitions.removeLast();
  }
}

QString OmClipboard::computeNodeExportStringForInsertion(OmNode *parentNode, OmField *field, int fieldIndex) const {
  QString nodeString(mNodeExportString);
  QList<OmNode *> existingDefNodes = OmDictionary::instance()->computeDefForInsertion(parentNode, field, fieldIndex, false);
  const int existingDefNodesSize = existingDefNodes.size();
  for (int i = mLinkedDefNodeDefinitions.size() - 1; i >= 0; --i) {
    bool found = false;
    for (int j = 0; j < existingDefNodesSize && !found; ++j) {
      const OmBaseNode *node = dynamic_cast<OmBaseNode *>(existingDefNodes[j]);
      found =
        node->defName() == mLinkedDefNodeDefinitions[i]->defName && node->nodeType() == mLinkedDefNodeDefinitions[i]->type;
    }
    if (!found)
      replaceExternalNodeDefinitionInString(nodeString, i);
  }
  return nodeString;
}
