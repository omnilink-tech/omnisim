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

#ifndef OM_SCENE_TREE_HPP
#define OM_SCENE_TREE_HPP

//
// Description: GUI part of the Scene Tree, contains the toolbar to edit the Scene Tree
//

#include "OmActionManager.hpp"

#include <QtCore/QModelIndex>
#include <QtWidgets/QWidget>

class OmAbstractPose;
class OmBaseNode;
class OmClipboard;
class OmField;
class OmFieldEditor;
class OmNode;
class OmSceneTreeModel;
class OmSourceFileEditor;
class OmTreeItem;
class OmTreeView;
class OmWorld;

class QModelIndex;
class QSplitter;
class QPushButton;

struct TreeItemState;

// cppcheck-suppress noConstructor
class OmSceneTree : public QWidget {
  Q_OBJECT
  Q_PROPERTY(int handleWidth MEMBER mHandleWidth READ handleWidth WRITE setHandleWidth)

public:
  explicit OmSceneTree(QWidget *parent = NULL);
  virtual ~OmSceneTree();

  void setWorld(OmWorld *world);
  OmSourceFileEditor *sourceFileEditor() const;

  void cleanup();

  void prepareWorldLoading();
  void applyChanges();

  // save/restore splitter perspective
  QByteArray saveState() const;
  void restoreState(QByteArray state);
  void restoreFactoryLayout();

  int &handleWidth() { return mHandleWidth; }
  void setHandleWidth(const int &handleWidth) { mHandleWidth = handleWidth; }

public slots:
  void selectPose(OmAbstractPose *p);
  void updateValue();
  void updateApplicationActions();
  void updateSelection();

signals:
  void valueChangedFromGui();
  void nodeSelected(OmBaseNode *n);
  void editRequested(const QString &filePath, bool modify = false, bool isRobot = false);
  void documentationRequest(const QString &book, const QString &page, bool visible);

private slots:
  void handleUserCommand(OmAction::OmActionKind actionKind);
  void reset();
  void transform(const QString &modelName);
  void convertToBaseNode();
  void convertRootToBaseNode();
  void moveViewpointToObject();
  void addNew();
  void startWatching(const QModelIndex &index);
  void stopWatching(const QModelIndex &index);
  void handleRowRemoval(const QModelIndex &parentIndex, int start, int end);
  void refreshItems();
  void handleDoubleClickOrEnterPress();
  void editFileFromFieldEditor(const QString &fileName);

  void prepareNodeRegeneration(OmNode *node, bool nested);
  void abortNodeRegeneration();
  void applyNodeRegeneration(OmNode *node);
  void refreshTreeView();

  void help();
  void exportUrdf();
  void openProtoInTextEditor();
  void editProtoInTextEditor();
  void openTemplateInstanceInTextEditor();
  void showFieldEditor(bool force = false);

  void del(OmNode *nodeToDel = NULL);

private:
  QSplitter *mSplitter;
  QString mWorldFileName;
  OmSceneTreeModel *mModel;
  OmTreeItem *mSelectedItem;
  QPushButton *mExternProtoButton;
  OmTreeView *mTreeView;
  OmFieldEditor *mFieldEditor;
  bool mRowsAreAboutToBeRemoved;
  QWidget *mFocusWidgetBeforeNodeRegeneration;

  OmActionManager *mActionManager;
  OmClipboard *mClipboard;
  int mHandleWidth;

  // Stuff about the recovery of the scene tree state after node regeneration.
  bool mSelectionInsideTreeStateRecovery;
  OmTreeItem *mSelectionBeforeTreeStateRegeneration;
  TreeItemState *mTreeItemState;
  void storeTreeItemState(OmTreeItem *treeItem, TreeItemState *treeItemState);
  void restoreTreeItemState(OmTreeItem *treeItem, TreeItemState *treeItemState, OmBaseNode *lastNode);
  void cleanTreeItemState(TreeItemState *item);
  // void printTreeItemState(TreeItemState *item, int indentation = 0); // Debug function to print the stored tree state.

  void showExternProtoPanel();

  void restoreState(OmTreeView *t1, OmTreeView *t2, const QModelIndex &i1, const QModelIndex &i2);
  void updateToolbar();
  bool isPasteAllowed();
  void pasteInSFValue();
  void pasteInMFValue();
  void clearSelection();
  bool isIndexAncestorOfCurrentIndex(const QModelIndex &index, int start, int end);
  void convertProtoToBaseNode(bool rootOnly);
  bool insertInertiaMatrix(const OmField *selectedField);
  void cut();
  void copy();
  void paste();
  void enableObjectViewActions(bool enabled);
};

#endif
