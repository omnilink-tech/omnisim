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

#ifndef OM_DICTIONARY_HPP
#define OM_DICTIONARY_HPP

//
// Description: Handles the updates of the DEF names Dictionary
//

#include <QtCore/QMultiMap>
#include <QtCore/QStringList>

class OmBaseNode;
class OmField;
class OmNode;
class OmMFNode;
class OmSFNode;

class OmDictionary {
public:
  static OmDictionary *instance();
  static void cleanup();

  // Recompute all DEF-USE dependencies (but protos' dependencies) and update
  // them if needed according to the VRML rule:
  // a USE node refers to its closest previous DEF node if it exists (otherwise it is turned into a DEF node)
  bool update(bool load = false);
  void updateProtosPrivateDef(OmBaseNode *&node);

  // If dictionary update is called after PROTO regeneration, then store the upper template PROTO node to prevent infinite
  // DEF/USE updates
  void setRegeneratedNode(const OmNode *node);

  // Loop through current world and return all the available DEF nodes
  // that could be used in the specified field.
  // If 'suitableOnly' is true, then only the DEF nodes that can be inserted in
  // the field are returned.
  QList<OmNode *> computeDefForInsertion(OmNode *const targetNode, OmField *const targetField, int targetIndex,
                                         bool suitableOnly = true);

  // Get node from DEF name
  OmNode *getNodeFromDEF(const QString &defName) const;
  void updateNodeDefName(OmNode *node, bool fromUseToDef);
  void removeNodeFromDictionary(const OmNode *node);

private:
  static OmDictionary *cInstance;
  OmDictionary();
  ~OmDictionary();

  bool updateDef(OmBaseNode *&node, OmSFNode *sfNode, OmMFNode *mfNode, int index, bool isTemplateRegenerator,
                 bool &regenerationRequired);
  void updateProtosDef(OmBaseNode *&node, OmSFNode *sfNode = NULL, OmMFNode *mfNode = NULL, int index = -1);
  void updateForInsertion(const OmNode *const node, bool suitableOnly, QList<OmNode *> &defNodes);
  void makeDefNodeAndUpdateDictionary(OmBaseNode *node, bool updateSceneDictionary);
  QList<QMultiMap<QString, OmNode *>> mNestedDictionaries;
  void clearNestedDictionaries() {
    mNestedUseNodes.clear();
    mNestedDictionaries.clear();
    mNestedDictionaries << QMultiMap<QString, OmNode *>();
  }
  OmNode *mTargetNode;
  OmField *mTargetField;
  int mTargetIndex;
  QList<OmBaseNode *> mNestedProtos;
  QList<OmBaseNode *> mNestedUseNodes;
  bool mStopUpdate;
  bool mLoad;                      // true if the update occurs right after world is loaded
  bool mCurrentProtoRegeneration;  // true if the update occurs due to a PROTO regeneration
  const OmNode *mCurrentProtoRegenerationNode;
  bool isSuitable(const OmNode *defNode, const QString &type) const;
  static bool checkChargerAndLedConstraints(OmNode *useNodeParent, const OmBaseNode *defNode, QString &deviceModelName,
                                            bool isFirstChild);
  static bool checkBoundingObjectConstraints(const OmBaseNode *defNode, QString &errorMessage);

  // List of DEF nodes visible in the scene tree
  QList<std::pair<OmNode *, QString>> mSceneDictionary;
};

#endif
