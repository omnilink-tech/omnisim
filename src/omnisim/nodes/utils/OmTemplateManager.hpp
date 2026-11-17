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

#ifndef OM_TEMPLATE_MANAGER_HPP
#define OM_TEMPLATE_MANAGER_HPP

//
// Description:    template manager
// Responsability: manage the upates of the templates
//

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QSet>

class OmNode;
class OmField;

class OmTemplateManager : public QObject {
  Q_OBJECT

public:
  static OmTemplateManager *instance();

  void clear();
  void subscribe(OmNode *node, bool subscribedDescendant);

  // when unblocked, all the templates which have required
  // a regeneration inbetween are regenerated
  void blockRegeneration(bool block);

  static bool isRegenerating() { return cRegeneratingNodeCount > 0; }
  static bool isNodeChangeTriggeringRegeneration(const OmNode *node) {
    return OmTemplateManager::instance()->mNodesSubscribedForRegeneration.contains(node);
  }

signals:
  void preNodeRegeneration(OmNode *node, bool nested);
  void abortNodeRegeneration();
  void postNodeRegeneration(OmNode *node);

private slots:
  void unsubscribe(QObject *node);
  void regenerateNodeFromField(OmField *field);
  void regenerateNode(OmNode *node, bool restarted = false);
  void nodeNeedRegeneration();

private:
  static void cleanup();
  static OmTemplateManager *cInstance;
  static int cRegeneratingNodeCount;

  OmTemplateManager();
  virtual ~OmTemplateManager();

  bool nodeNeedsToSubscribe(OmNode *node);
  void recursiveFieldSubscribeToRegenerateNode(OmNode *node, bool subscribedNode, bool subscribedDescendant);

  QList<OmNode *> mTemplates;
  QSet<const OmNode *> mNodesSubscribedForRegeneration;

  bool mBlockRegeneration;
  bool mTemplatesNeedRegeneration;
  OmNode *mRegeneratingUpperTemplateNode;
};

#endif
