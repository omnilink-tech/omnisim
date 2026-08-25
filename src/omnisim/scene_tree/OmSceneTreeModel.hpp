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

#ifndef OM_SCENE_TREE_MODEL_HPP
#define OM_SCENE_TREE_MODEL_HPP

//
// Description: QAbstractItemModel for the OmSceneTree: defines how the object must be organized in the Scene tree
//

#include <QtCore/QAbstractItemModel>
#include <QtCore/QMap>
#include <QtCore/QModelIndex>
#include <QtCore/QVariant>

class OmTreeItem;
class OmGroup;
class OmNode;
class OmField;

class OmSceneTreeModel : public QAbstractItemModel {
  Q_OBJECT

public:
  explicit OmSceneTreeModel(OmGroup *worldRoot);
  virtual ~OmSceneTreeModel();

  // inherited from QAbstractItemModel
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
  Qt::ItemFlags flags(const QModelIndex &index) const override;
  QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex &index) const override;
  int rowCount(const QModelIndex &parent) const override;
  int columnCount(const QModelIndex &parent) const override { return 1; }
  bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
  bool insertRows(int row, int count, const QModelIndex &parent) override;

  void startWatching(const QModelIndex &index);
  void stopWatching(const QModelIndex &index);

  // Index mappings
  OmTreeItem *indexToItem(const QModelIndex &index) const;
  int itemToTreeIndex(OmTreeItem *item) const;
  int modelIndexToTreeIndex(const QModelIndex &index) const { return itemToTreeIndex(indexToItem(index)); }
  QModelIndex treeIndexToModelIndex(int index) const { return itemToIndex(treeIndexToItem(index)); }
  QModelIndex itemToIndex(const OmTreeItem *item) const;

  OmTreeItem *rootItem() const { return mRootItem; }

  OmTreeItem *findUpperNodeItem(const OmTreeItem *item) const;
  QModelIndex findModelIndexFromNode(OmNode *node, OmTreeItem *current) const;
  QModelIndex findModelIndexFromNode(OmNode *node) const { return findModelIndexFromNode(node, mRootItem); }
  static OmTreeItem *findTreeItemFromNode(OmNode *node, OmTreeItem *current);

  QModelIndex findModelIndexFromField(OmField *field, OmTreeItem *current) const;
  QModelIndex findModelIndexFromField(OmField *field) const { return findModelIndexFromField(field, mRootItem); }

  void createChildrenItemForNode(OmNode *node);

  void emitLayoutChanged();
  void updateAllSceneTreeValues();
  void updateItem(OmTreeItem *item);

signals:
  void itemInserted(const QModelIndex &index);
  void rowsAboutToBeRemovedSoon(const QModelIndex &parent, int start, int end);

private slots:
  void updateData();
  void removeItems(int row, int count);
  void insertItems(int position, int count);
  void updateItemAndChildren(OmNode *node, bool createChildren);

private:
  OmTreeItem *mRootItem;
  OmTreeItem *treeIndexToItem(int targetIndex) const;
  static OmTreeItem *treeIndexToItem(OmTreeItem *currentItem, int &index);
  static void treeIndex(const OmTreeItem *const currentItem, const OmTreeItem *const targetItem, bool &itemFound, int &index);

  OmTreeItem *createItemForNode(OmNode *node);
  OmTreeItem *createItemForField(OmField *field);
};

#endif
