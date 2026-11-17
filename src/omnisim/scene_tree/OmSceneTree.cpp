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

#include "OmSceneTree.hpp"

#include "OmAbstractPose.hpp"
#include "OmAddInertiaMatrixDialog.hpp"
#include "OmAddItemCommand.hpp"
#include "OmAddNodeDialog.hpp"
#include "OmBoundingSphere.hpp"
#include "OmClipboard.hpp"
#include "OmConcreteNodeFactory.hpp"
#include "OmContextMenuGenerator.hpp"
#include "OmEditCommand.hpp"
#include "OmField.hpp"
#include "OmFieldEditor.hpp"
#include "OmGroup.hpp"
#include "OmGuiRefreshOracle.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMFVector3.hpp"
#include "OmMessageBox.hpp"
#include "OmNetwork.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPhysics.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRemoveItemCommand.hpp"
#include "OmResetCommand.hpp"
#include "OmSFNode.hpp"
#include "OmSceneTreeModel.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmTemplateManager.hpp"
#include "OmTreeItem.hpp"
#include "OmTreeView.hpp"
#include "OmUndoStack.hpp"
#include "OmUrl.hpp"
#include "OmValueEditor.hpp"
#include "OmVariant.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

#include <cassert>

#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>

static int gFactoryFieldEditorHeightHint = 0;

struct TreeItemState {
  bool expanded;
  bool selected;
  QList<TreeItemState *> children;
};

OmSceneTree::OmSceneTree(QWidget *parent) :
  QWidget(parent),
  mSplitter(new QSplitter(Qt::Vertical, this)),
  mActionManager(OmActionManager::instance()),
  mClipboard(OmClipboard::instance()) {
  mModel = NULL;
  mTreeView = NULL;
  mSelectedItem = NULL;
  mExternProtoButton = NULL;
  mRowsAreAboutToBeRemoved = false;
  mFocusWidgetBeforeNodeRegeneration = NULL;

  mSelectionInsideTreeStateRecovery = false;
  mSelectionBeforeTreeStateRegeneration = NULL;
  mTreeItemState = NULL;

  setObjectName("SceneTree");

  mFieldEditor = new OmFieldEditor(this);
  connect(mFieldEditor, &OmFieldEditor::dictionaryUpdateRequested, OmNodeOperations::instance(),
          &OmNodeOperations::requestUpdateDictionary);
  connect(mFieldEditor, &OmFieldEditor::valueChanged, this, &OmSceneTree::valueChangedFromGui);
  connect(mFieldEditor, &OmFieldEditor::editRequested, this, &OmSceneTree::editFileFromFieldEditor);
  connect(OmGuiRefreshOracle::instance(), &OmGuiRefreshOracle::canRefreshActivated, this, &OmSceneTree::refreshItems);

  QScrollArea *fieldEditorScrollArea = new QScrollArea(mSplitter);
  fieldEditorScrollArea->setObjectName("editorPane");
  fieldEditorScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  fieldEditorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  fieldEditorScrollArea->setWidgetResizable(true);
  fieldEditorScrollArea->setFocusPolicy(Qt::ClickFocus);
  fieldEditorScrollArea->setWidget(mFieldEditor);
  gFactoryFieldEditorHeightHint = fieldEditorScrollArea->sizeHint().height();

  mSplitter->addWidget(fieldEditorScrollArea);
  mSplitter->setObjectName("verticalSplitter");

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(mSplitter, 0);

  connect(mActionManager, &OmActionManager::userWorldEditCommandReceived, this, &OmSceneTree::handleUserCommand);
  connect(mActionManager, &OmActionManager::transformRequested, this, &OmSceneTree::transform);
  connect(mActionManager->action(OmAction::ADD_NEW), &QAction::triggered, this, &OmSceneTree::addNew);
  connect(mActionManager->action(OmAction::MOVE_VIEWPOINT_TO_OBJECT), &QAction::triggered, this,
          &OmSceneTree::moveViewpointToObject);
  connect(mActionManager->action(OmAction::RESET_VALUE), &QAction::triggered, this, &OmSceneTree::reset);
  connect(mActionManager->action(OmAction::EDIT_FIELD), &QAction::triggered, this, &OmSceneTree::showFieldEditor);
  connect(mActionManager->action(OmAction::CONVERT_TO_BASE_NODES), &QAction::triggered, this, &OmSceneTree::convertToBaseNode);
  connect(mActionManager->action(OmAction::CONVERT_ROOT_TO_BASE_NODES), &QAction::triggered, this,
          &OmSceneTree::convertRootToBaseNode);
  connect(mActionManager->action(OmAction::OPEN_HELP), &QAction::triggered, this, &OmSceneTree::help);
  connect(mActionManager->action(OmAction::EDIT_PROTO_SOURCE), &QAction::triggered, this, &OmSceneTree::editProtoInTextEditor);
  connect(mActionManager->action(OmAction::SHOW_PROTO_SOURCE), &QAction::triggered, this, &OmSceneTree::openProtoInTextEditor);
  connect(mActionManager->action(OmAction::SHOW_PROTO_RESULT), &QAction::triggered, this,
          &OmSceneTree::openTemplateInstanceInTextEditor);
  connect(mActionManager->action(OmAction::EXPORT_URDF), &QAction::triggered, this, &OmSceneTree::exportUrdf);
  connect(OmUndoStack::instance(), &OmUndoStack::changed, this, &OmSceneTree::updateValue);

  connect(OmTemplateManager::instance(), &OmTemplateManager::preNodeRegeneration, this, &OmSceneTree::prepareNodeRegeneration);
  connect(OmTemplateManager::instance(), &OmTemplateManager::abortNodeRegeneration, this, &OmSceneTree::abortNodeRegeneration);
  connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this, &OmSceneTree::applyNodeRegeneration);
}

OmSceneTree::~OmSceneTree() {
  cleanup();
}

void OmSceneTree::cleanup() {
  OmTreeItem::enableUpdates(false);
  mFieldEditor->resetFocus();
  mSelectedItem = NULL;

  delete mTreeView;
  mTreeView = NULL;
  delete mModel;
  mModel = NULL;

  // disconnect all signals
  disconnect(this, 0);
}

void OmSceneTree::prepareWorldLoading() {
  OmUndoStack::instance()->clear();
  mSelectedItem = NULL;
  mFieldEditor->resetFocus();
  OmTreeItem::enableUpdates(false);
  updateToolbar();
  disconnect(OmSelection::instance(), &OmSelection::selectionChangedFromSceneTree, this, &OmSceneTree::updateSelection);
}

void OmSceneTree::applyChanges() {
  mFieldEditor->applyChanges();
}

// compare old tree and new tree side by side and recursively.
// if an index is expanded in the old tree, expand it also in the new tree
void OmSceneTree::restoreState(OmTreeView *t1, OmTreeView *t2, const QModelIndex &i1, const QModelIndex &i2) {
  QModelIndex selectedIndex = t1->currentIndex();

  for (int i = 0; true; i++) {
    // explore indices side by side
    QModelIndex j1 = t1->model()->index(i, 0, i1);
    QModelIndex j2 = t2->model()->index(i, 0, i2);

    // break when no more children
    if (!j1.isValid() || !j2.isValid())
      break;

    // recurse into tree
    restoreState(t1, t2, j1, j2);

    // restore 'expanded' state
    if (t1->isExpanded(j1))
      t2->setExpanded(j2, true);

    // restore current selection
    if (j1 == selectedIndex)
      t2->setCurrentIndex(j2);
  }
}

void OmSceneTree::setWorld(OmWorld *world) {
  // keep to restore state
  OmTreeView *const oldTreeView = mTreeView;
  OmSceneTreeModel *const oldModel = mModel;

  // create new tree widget and model
  mTreeView = new OmTreeView(this);
  mTreeView->setHeaderHidden(true);
  mModel = new OmSceneTreeModel(world->root());

  // connect widget to model
  mTreeView->setModel(mModel);
  mModel->startWatching(mTreeView->rootIndex());

  // enable updates of scene tree items
  OmTreeItem::enableUpdates(true);

  // this must be done before restoreState()
  connect(mTreeView, &OmTreeView::refreshRequested, this, &OmSceneTree::refreshTreeView);
  connect(mTreeView, &OmTreeView::doubleClickOrEnterPressed, this, &OmSceneTree::handleDoubleClickOrEnterPress);
  connect(mTreeView, &OmTreeView::focusIn, this, &OmSceneTree::updateApplicationActions);
  connect(mTreeView, &OmTreeView::expanded, this, &OmSceneTree::startWatching);
  connect(mTreeView, &OmTreeView::collapsed, this, &OmSceneTree::stopWatching);
  connect(mModel, &OmSceneTreeModel::itemInserted, mTreeView, &OmTreeView::itemInserted);
  connect(mModel, &OmSceneTreeModel::rowsAboutToBeRemovedSoon, this, &OmSceneTree::handleRowRemoval);
  connect(mTreeView, &OmTreeView::beforeContextMenuShowed, this, &OmSceneTree::updateSelection);

  connect(mTreeView, &OmTreeView::selectionHasChanged, this, &OmSceneTree::updateSelection);
  connect(OmSelection::instance(), &OmSelection::selectionChangedFromSceneTree, this, &OmSceneTree::updateSelection);

  // attempt to restore expanded state (only if reloading)
  if (world->fileName() == mWorldFileName)
    restoreState(oldTreeView, mTreeView, oldTreeView->rootIndex(), mTreeView->rootIndex());
  else {
    mSelectedItem = NULL;
    mFieldEditor->setTitle("");
    updateToolbar();
  }

  bool hasFocus = oldTreeView && oldTreeView->hasFocus();

  // delete old widget and model
  delete oldTreeView;
  delete oldModel;
  delete mExternProtoButton;

  // create extern proto button
  mExternProtoButton = new QPushButton("IMPORTABLE EXTERNPROTO");
  mExternProtoButton->setObjectName("importableExternProto");
  connect(mExternProtoButton, &QPushButton::pressed, this, &OmSceneTree::showExternProtoPanel);

  // insert new widget before value editor
  mSplitter->insertWidget(0, mExternProtoButton);
  mSplitter->insertWidget(1, mTreeView);
  mSplitter->setStretchFactor(0, 1);
  mSplitter->setStretchFactor(1, 0);

  // set focus if needed
  if (hasFocus)
    mTreeView->setFocus(Qt::OtherFocusReason);

  // just to know if we are reloading
  mWorldFileName = world->fileName();
  mTreeView->scrollToSelection();
}

void OmSceneTree::showExternProtoPanel() {
  clearSelection();
  // uncollapse the field editor
  showFieldEditor(true);
  emit nodeSelected(NULL);
  mFieldEditor->editExternProto();
}

void OmSceneTree::handleUserCommand(OmAction::OmActionKind actionKind) {
  switch (actionKind) {
    case OmAction::CUT:
      cut();
      return;
    case OmAction::COPY:
      copy();
      return;
    case OmAction::PASTE:
      paste();
      return;
    case OmAction::UNDO:
      OmUndoStack::instance()->undo();
      return;
    case OmAction::REDO:
      OmUndoStack::instance()->redo();
      return;
    case OmAction::DEL:
      del();
    default:
      return;
  }
}

void OmSceneTree::cut() {
  copy();
  del();
  updateToolbar();
}

void OmSceneTree::copy() {
  OmValue *value;
  int row = -1;

  // make a shallow copy of item value
  if (mSelectedItem->isField()) {
    // copy action should not be enabled for multiple fields
    assert(mSelectedItem->field()->isSingle());
    value = mSelectedItem->field()->value();
  } else {
    // node or item
    value = mSelectedItem->parent()->field()->value();
    row = mSelectedItem->row();
  }

  const OmSingleValue *singleValue = dynamic_cast<OmSingleValue *>(value);
  const OmMultipleValue *multipleValue = dynamic_cast<OmMultipleValue *>(value);
  if (mSelectedItem->isNode() || mSelectedItem->isSFNode()) {
    const QList<const OmNode *> clipboardNodes = OmVrmlNodeUtilities::protoNodesInWorldFile(mSelectedItem->node());
    if (!OmProtoManager::instance()->externProtoClipboardBuffer().isEmpty())
      OmProtoManager::instance()->clearExternProtoClipboardBuffer();
    OmProtoManager::instance()->saveToExternProtoClipboardBuffer(clipboardNodes);
    mClipboard->setNode(mSelectedItem->node());
  } else if (singleValue)
    *mClipboard = singleValue->variantValue();
  else if (multipleValue)
    *mClipboard = multipleValue->variantValue(row);
  else  // reset clipboard
    *mClipboard = OmVariant();

  updateToolbar();
}

void OmSceneTree::paste() {
  if (!mSelectedItem)
    return;

  const QList<OmExternProto *> &clipboardBuffer = OmProtoManager::instance()->externProtoClipboardBuffer();
  foreach (const OmExternProto *item, clipboardBuffer)
    OmProtoManager::instance()->declareExternProto(item->name(), item->url(), item->isImportable());

  if (mSelectedItem->isField() && mSelectedItem->field()->isSingle())
    pasteInSFValue();
  else
    pasteInMFValue();
  OmWorld::instance()->setModifiedFromSceneTree();
}

void OmSceneTree::pasteInSFValue() {
  OmTreeItem *selectedItem = mSelectedItem;
  OmField *field = selectedItem->field();
  OmValue *item = field->value();

  if (mClipboard->type() == WB_SF_NODE) {
    const QString &nodeString = mClipboard->computeNodeExportStringForInsertion(selectedItem->parent()->node(), field, -1);
    OmNodeOperations::OperationResult result = OmNodeOperations::instance()->importNode(
      selectedItem->parent()->node(), field, -1, OmNodeOperations::FROM_PASTE, nodeString);
    if (result == OmNodeOperations::FAILURE)
      return;

    if (result == OmNodeOperations::SUCCESS) {
      // update selection scroll position
      QModelIndex currentIndex = mModel->itemToIndex(selectedItem->child(0));
      mTreeView->setCurrentIndex(currentIndex);
    } else
      updateSelection();

    mTreeView->scrollToSelection();

  } else {
    // item
    OmSingleValue *singleValue = dynamic_cast<OmSingleValue *>(item);
    OmUndoStack::instance()->push(new OmEditCommand(singleValue, singleValue->variantValue(), *mClipboard));
  }

  updateValue();
  updateToolbar();
}

// paste item or node
void OmSceneTree::pasteInMFValue() {
  assert(!mClipboard->isEmpty());

  OmMultipleValue *parentItem;
  OmNode *parentNode = NULL;
  OmField *field = NULL;
  OmTreeItem *fieldItem;
  int index = 0;

  if (mSelectedItem->isField()) {
    // multiple field
    const OmTreeItem *nodeItem = mSelectedItem->parent();
    fieldItem = mSelectedItem;
    field = mSelectedItem->field();
    assert(field && field->isMultiple());

    parentItem = static_cast<OmMultipleValue *>(field->value());
    if (nodeItem->isNode() || nodeItem->hasNode())
      parentNode = nodeItem->node();
  } else {
    // sibling is selected (node or item)
    fieldItem = mSelectedItem->parent();
    field = fieldItem->field();
    assert(fieldItem->isField() && field && field->isMultiple());

    index = mSelectedItem->row() + 1;
    parentItem = static_cast<OmMultipleValue *>(field->value());
    if (mSelectedItem->isNode())
      parentNode = mSelectedItem->node()->parentNode();
  }

  if (mClipboard->type() == WB_SF_NODE) {
    assert(parentNode);

    // if newNode is in a template regenerated field, its pointer will be invalid after this call
    const QString &nodeString = mClipboard->computeNodeExportStringForInsertion(parentNode, field, index);
    OmNodeOperations::OperationResult result =
      OmNodeOperations::instance()->importNode(parentNode, field, index, OmNodeOperations::FROM_PASTE, nodeString, true);
    if (result == OmNodeOperations::FAILURE)
      return;

    if (result == OmNodeOperations::SUCCESS) {
      // update selection and scroll position
      QModelIndex currentIndex = mModel->itemToIndex(fieldItem->child(index));
      mTreeView->setCurrentIndex(currentIndex);
    }

    mTreeView->scrollToSelection();

    OmUndoStack::instance()->clear();  // TODO remove after implementing UNDO action

  } else
    OmUndoStack::instance()->push(new OmAddItemCommand(parentItem, *mClipboard, index));

  updateSelection();
  if (mSelectedItem && mSelectedItem->isField()) {  // if node insertion failed mSelectedItem is NULL
    QModelIndex newNodeIndex = mModel->itemToIndex(mSelectedItem->child(index));
    mTreeView->setCurrentIndex(newNodeIndex);
    mTreeView->scrollToModelIndex(newNodeIndex);
  }

  updateValue();
  updateToolbar();
}

void OmSceneTree::del(OmNode *nodeToDel) {
  OmNode *node = nodeToDel;

  const OmTreeItem *deletedItem;
  if (node == NULL) {
    node = mSelectedItem->node();
    deletedItem = mSelectedItem;
  } else
    deletedItem = mModel->indexToItem(mModel->findModelIndexFromNode(node));

  bool dictionaryUpdated = false;
  if (node) {
    dictionaryUpdated = OmVrmlNodeUtilities::hasAreferredDefNodeDescendant(node);
    if (dictionaryUpdated &&
        OmMessageBox::question(
          tr("This node is a DEF node, or has a descendant DEF node, on which at least one external USE node depends. "
             "Deleting it will make its USE nodes to refer to a previous node having the same DEF keyword if it exists, "
             "or will turn its first USE node into a DEF node.\n"
             "Do you want to continue?"),
          this, tr("DEF node deletion")) == QMessageBox::Cancel)
      return;

    bool previousRowsAboutToBeRemoved = mRowsAreAboutToBeRemoved;
    mFieldEditor->editField(NULL, NULL);  // reset field editor
    if (!(node->isUseNode() && deletedItem->isSFNode()))
      mRowsAreAboutToBeRemoved = true;
    // else no rows will be deleted

    if (!OmNodeOperations::instance()->deleteNode(node)) {
      mRowsAreAboutToBeRemoved = previousRowsAboutToBeRemoved;
      return;
    }

    OmUndoStack::instance()->clear();  // clear undo stack if no available UNDO/REDO implementation of del action
  } else {
    // item
    mRowsAreAboutToBeRemoved = true;
    OmMultipleValue *mvalue = static_cast<OmMultipleValue *>(mSelectedItem->parent()->field()->value());
    OmUndoStack::instance()->push(new OmRemoveItemCommand(mvalue, mSelectedItem->row()));
  }

  mRowsAreAboutToBeRemoved = false;

  if (dictionaryUpdated) {
    mModel->emitLayoutChanged();  // makes the 'expandable' triangle visible for USE nodes turned into DEF nodes
    if (!OmNodeOperations::instance()->isFromSupervisor())
      // selection already removed in handleRowRemoval function but it is changed when updating the dictionay
      clearSelection();
  }

  OmWorld::instance()->setModifiedFromSceneTree();

  refreshTreeView();
  updateValue();
  updateToolbar();
}

void OmSceneTree::reset() {
  OmField *field = mSelectedItem->field();
  assert(field);

  if (field->isTemplateRegenerator())
    // stop editing otherwise unapplied changes could cause issue during template PROTO regeneration
    mFieldEditor->currentEditor()->stopEditing();

  if (field->singleType() == WB_SF_NODE) {
    bool dictionaryNeedsUpdate = false;
    OmNode *parentNode = mSelectedItem->parent()->node();

    // check if referred DEF node is going to be deleted
    bool containsReferredNode = false;
    const OmSFNode *sfnode = dynamic_cast<OmSFNode *>(field->value());
    const OmMFNode *mfnode = dynamic_cast<OmMFNode *>(field->value());
    if (sfnode) {
      mRowsAreAboutToBeRemoved = sfnode->value();
      containsReferredNode = sfnode->value() && OmVrmlNodeUtilities::hasAreferredDefNodeDescendant(sfnode->value());
    } else if (mfnode) {
      mRowsAreAboutToBeRemoved = mfnode->size() > 0;
      OmMFIterator<OmMFNode, OmNode *> it(mfnode);
      while (it.hasNext()) {
        if (OmVrmlNodeUtilities::hasAreferredDefNodeDescendant(it.next())) {
          containsReferredNode = true;
          break;
        }
      }
    }
    if (containsReferredNode) {
      if (OmMessageBox::question(tr("This field contains a DEF node on which at least one external USE node depends. "
                                    "Deleting it will turn its first USE node into a DEF node.\n"
                                    "Do you want to continue?"),
                                 this, tr("DEF node deletion")) == QMessageBox::Cancel) {
        mRowsAreAboutToBeRemoved = false;
        return;
      }

      dictionaryNeedsUpdate = true;
    }

    mFieldEditor->editField(NULL, NULL);

    const OmValue *defaultValue = field->defaultValue();
    // in case changes to this field triggers the PROTO template regeneration
    // and the default value is not the empty one, then we want to skip the first
    // template regeneration so that the field pointer is valid when applying the
    // default value
    bool blockTemplateRegeneration =
      field->isTemplateRegenerator() && ((sfnode && dynamic_cast<const OmSFNode *>(defaultValue)->value() != NULL) ||
                                         (mfnode && dynamic_cast<const OmMFNode *>(defaultValue)->size() > 0));

    // notify node deletion (needed for example to propagate it during the streaming)
    if (sfnode && sfnode->value() != NULL)
      OmNodeOperations::instance()->notifyNodeDeleted(sfnode->value());
    else if (mfnode && mfnode->size() > 0) {
      OmMFIterator<OmMFNode, OmNode *> it(mfnode);
      while (it.hasNext())
        OmNodeOperations::instance()->notifyNodeDeleted(it.next());
    }

    // reset field to default value
    // in case of SFNode/MFNode field the value is set to NULL or []
    field->reset(blockTemplateRegeneration);

    // create and finalize new node instances
    if (sfnode) {
      OmNode *defaultNode = dynamic_cast<const OmSFNode *>(defaultValue)->value();
      if (defaultNode) {
        OmNode::setGlobalParentNode(parentNode);
        OmNode *newNode = OmConcreteNodeFactory::instance()->createCopy(*defaultNode);
        OmNode::setGlobalParentNode(NULL);
        newNode->setParentNode(parentNode);

#ifndef NDEBUG
        const OmNodeOperations::OperationResult result =
#endif
          OmNodeOperations::instance()->initNewNode(newNode, parentNode, field, -1, true);
        assert(result != OmNodeOperations::FAILURE);
      }

    } else if (mfnode) {
      const OmMFNode *defaultMFNode = dynamic_cast<const OmMFNode *>(defaultValue);
      OmMFIterator<const OmMFNode, OmNode *> it(defaultMFNode);
      int i = 0;
      while (it.hasNext()) {
        const OmNode *defaultNode = it.next();
        OmNode::setGlobalParentNode(parentNode);
        OmNode *newNode = OmConcreteNodeFactory::instance()->createCopy(*defaultNode);
        OmNode::setGlobalParentNode(NULL);
        newNode->setParentNode(parentNode);
#ifndef NDEBUG
        const OmNodeOperations::OperationResult result =
#endif
          OmNodeOperations::instance()->initNewNode(newNode, parentNode, field, i, true);
        assert(result != OmNodeOperations::FAILURE);
        ++i;
      }
    }

    mRowsAreAboutToBeRemoved = false;
    // no undo function available for SFNode and MFNode
    OmUndoStack::instance()->clear();

    if (dictionaryNeedsUpdate)
      OmNodeOperations::instance()->updateDictionary(false, NULL);
    updateSelection();

  } else {
    OmUndoStack::instance()->push(new OmResetCommand(mSelectedItem->field()));
    mModel->updateItem(mSelectedItem);
  }

  OmWorld::instance()->setModifiedFromSceneTree();
  OmNodeOperations::instance()->purgeUnusedExternProtoDeclarations();

  updateValue();
  updateToolbar();
}

void OmSceneTree::transform(const QString &modelName) {
  OmNode *const currentNode = mSelectedItem->node();
  assert(dynamic_cast<OmGroup *>(currentNode));

  // check if loosing information
  const OmNodeUtilities::Answer answer = OmNodeUtilities::isSuitableForTransform(currentNode, modelName, NULL);
  if (answer == OmNodeUtilities::LOOSING_INFO) {
    const QString message = tr("Warning: Turning a %1 into a %2 will loose some information%3")
                              .arg(currentNode->nodeModelName())
                              .arg(modelName)
                              .arg(modelName == "Transform" ? tr(", including possibly children.") : ".") +
                            "\n" + tr("Do you still want to proceed?");
    if (OmMessageBox::question(message, this) == QMessageBox::Cancel) {
      mFieldEditor->updateValue();
      return;
    }
  }

  mRowsAreAboutToBeRemoved = true;  // As rows may be removed during the transform operation, we deactivate the item selection
                                    // update and restore it afterwards

  const QModelIndex currentModelIndex = mModel->itemToIndex(mSelectedItem);
  const bool isExpanded = mTreeView->isExpanded(currentModelIndex);

  OmTreeItem *selectedItem = mSelectedItem;

  OmGroup *group = dynamic_cast<OmGroup *>(currentNode);
  if (group && modelName == "Transform")
    group->deleteAllSolids();

  // create new node
  OmNode::setGlobalParentNode(currentNode->parentNode());
  OmNode *const newNode = OmConcreteNodeFactory::instance()->createNode(modelName, 0, currentNode->parentNode());
  if (!newNode) {
    OmLog::error(tr("Transformation aborted: impossible to create a node of type %1.").arg(modelName));
    mRowsAreAboutToBeRemoved = false;
    return;
  }

  // copy fields and adopt children
  OmNode::setGlobalParentNode(newNode);
  QVector<OmField *> fields = currentNode->fieldsOrParameters();
  foreach (const OmField *originalField, fields) {
    // copy field if it exists
    OmField *const newField = newNode->findField(originalField->name());
    if (newField)
      newField->copyValueFrom(originalField);
  }
  newNode->setDefName(currentNode->defName());
  OmNode::setGlobalParentNode(NULL);

  // reassign pointer in parent
  OmField *parentField = selectedItem->parent()->field();
  const OmNode *upperTemplate =
    OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(parentField, currentNode->parentNode());
  bool isInsideATemplateRegenerator = upperTemplate && upperTemplate != currentNode;
  if (selectedItem->isSFNode()) {
    OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(selectedItem->field()->value());
    assert(sfnode);
    OmNodeOperations::instance()->notifyNodeDeleted(currentNode);
    OmTemplateManager::instance()->blockRegeneration(true);
    selectedItem->del();  // remove previous item
    sfnode->setValue(newNode);
  } else {
    assert(selectedItem->parent()->isField());
    OmMFNode *mfnode = dynamic_cast<OmMFNode *>(parentField->value());
    assert(mfnode);
    int nodeIndex = mfnode->nodeIndex(currentNode);
    OmNodeOperations::instance()->notifyNodeDeleted(currentNode);
    OmTemplateManager::instance()->blockRegeneration(true);
    mfnode->setItem(nodeIndex, newNode);
  }

  newNode->validate();
  if (newNode->parentNode() && newNode->parentNode()->isProtoInstance())
    newNode->parentNode()->redirectInternalFields(parentField);
  OmTemplateManager::instance()->blockRegeneration(false);

  if (!isInsideATemplateRegenerator)
    static_cast<OmBaseNode *>(newNode)->finalize();

  mRowsAreAboutToBeRemoved = false;

  if (!isInsideATemplateRegenerator) {
    const QModelIndex newModelIndex = mModel->findModelIndexFromNode(newNode);
    mTreeView->setCurrentIndex(newModelIndex);
    mTreeView->setExpanded(newModelIndex, isExpanded);
    mTreeView->scrollToModelIndex(newModelIndex);
    OmNodeOperations::instance()->requestUpdateDictionary();
  }

  updateSelection();
  updateValue();
  updateToolbar();

  OmUndoStack::instance()->clear();  // clear undo stack if no available UNDO/REDO implementation of transform action
}

void OmSceneTree::convertToBaseNode() {
  convertProtoToBaseNode(false);
}

void OmSceneTree::convertRootToBaseNode() {
  convertProtoToBaseNode(true);
}

void OmSceneTree::convertProtoToBaseNode(bool rootOnly) {
  OmNode *const currentNode = mSelectedItem->node();
  if (currentNode->isProtoInstance()) {
    const OmSolid *const solid = dynamic_cast<OmSolid *>(currentNode);
    OmViewpoint *viewpoint = OmWorld::instance()->viewpoint();
    const bool isFollowedNode = (solid && viewpoint->followedSolid() == solid);
    int index;
    OmField *parentField = currentNode->parentFieldAndIndex(index);
    OmNode *parentNode = currentNode->parentNode();
    QString nodeString;
    OmWriter writer(&nodeString, currentNode->modelName() + ".proto");
    if (rootOnly)
      writer.setRootNode(currentNode);
    else
      writer.setRootNode(NULL);
    currentNode->write(writer);

    const bool skipTemplateRegeneration =
      OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(parentField, parentNode);
    if (skipTemplateRegeneration)
      // PROTO will be regenerated after importing the converted node
      parentField->blockSignals(true);
    // remove previous node
    mRowsAreAboutToBeRemoved = true;
    OmNodeOperations::instance()->deleteNode(currentNode);
    mRowsAreAboutToBeRemoved = false;
    if (skipTemplateRegeneration)
      parentField->blockSignals(false);

    // backup clipboard data
    const QList<QString> previousClipboardBuffer(OmProtoManager::instance()->externProtoClipboardBufferUrls());

    // declare PROTO nodes that have become visible at the world level
    OmProtoManager::instance()->clearExternProtoClipboardBuffer();
    std::pair<QString, QString> item;
    foreach (item, writer.declarations()) {
      const QString previousUrl(OmProtoManager::instance()->declareExternProto(item.first, item.second, false, false));
      if (!previousUrl.isEmpty()) {
        OmLog::warning(tr("Conflicting declarations for '%1' are provided: %2 and %3, the first one will be used. "
                          "To use the other instead you will need to change it manually in the world file.")
                         .arg(item.first)
                         .arg(previousUrl)
                         .arg(item.second));
        OmProtoManager::instance()->saveToExternProtoClipboardBuffer(previousUrl);
      } else
        OmProtoManager::instance()->saveToExternProtoClipboardBuffer(item.second);
    }

    // import new node
    if (OmNodeOperations::instance()->importNode(parentNode, parentField, index, OmNodeOperations::DEFAULT, nodeString) ==
        OmNodeOperations::SUCCESS) {
      OmNode *node = NULL;
      if (parentField->type() == WB_SF_NODE) {
        node = static_cast<OmSFNode *>(parentField->value())->value();
        mTreeView->setCurrentIndex(mModel->findModelIndexFromNode(node));
      } else if (parentField->type() == WB_MF_NODE) {
        node = static_cast<OmMFNode *>(parentField->value())->item(index);
        mTreeView->setCurrentIndex(mModel->findModelIndexFromNode(node));
      }
      if (isFollowedNode)
        viewpoint->startFollowUp(dynamic_cast<OmSolid *>(node), true);
    }
    OmWorld::instance()->setModifiedFromSceneTree();
    OmProtoManager::instance()->resetExternProtoClipboardBuffer(previousClipboardBuffer);
  }
  updateSelection();
  updateValue();
  updateToolbar();

  OmUndoStack::instance()->clear();
}

void OmSceneTree::moveViewpointToObject() {
  if (!mSelectedItem)
    return;

  const OmTreeItem *itemToMoveTo = mSelectedItem;
  while (true) {
    if (itemToMoveTo->isNode() || itemToMoveTo->isSFNode()) {
      OmNode *node = itemToMoveTo->node();
      OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(node);
      if (baseNode && OmWorld::instance()->viewpoint()->moveViewpointToObject(baseNode))
        break;
      if (node->isTopLevel())
        break;
    }
    itemToMoveTo = itemToMoveTo->parent();
  }
}

bool OmSceneTree::insertInertiaMatrix(const OmField *selectedField) {
  const QList<OmField *> &internalFields = selectedField->internalFields();
  const int n = internalFields.size();
  const QString &selectedFieldName = (n > 0) ? internalFields.at(0)->name() : selectedField->name();

  if (selectedFieldName != "inertiaMatrix")
    return false;

  OmPhysics *physics = NULL, *internalPhysics = NULL;
  OmSolid *solid = NULL;
  bool validBoundingObject = false;
  bool parameter = selectedField->alias().isEmpty() == false;

  if (n <= 1) {  // selectedField is either a non-parameter 'inertiaMatrix' field or a parameter with only one internal field
    const OmField *p = NULL;
    const OmField *ip = NULL;
    if (parameter == false) {  // non-parameter case
      const OmNode *const nodeParent = selectedField->parentNode();
      assert(nodeParent);
      p = nodeParent->parentField();
    } else
      p = OmVrmlNodeUtilities::findFieldParent(internalFields.at(0), true);

    assert(p);
    const int m = p->internalFields().size();
    if (m <= 1) {
      if (m == 1) {
        ip = p->internalFields().at(0);
        internalPhysics = dynamic_cast<OmPhysics *>(dynamic_cast<OmSFNode *>(ip->value())->value());
      }
      physics = dynamic_cast<OmPhysics *>(dynamic_cast<OmSFNode *>(p->value())->value());
      assert(physics);
      solid = internalPhysics ? internalPhysics->upperSolid() : physics->upperSolid();
      assert(solid);
      validBoundingObject |= solid->hasAvalidBoundingObject();
    }
  }

  OmAddInertiaMatrixDialog dialog(validBoundingObject && !parameter, this);

  if (dialog.exec() == QDialog::Rejected)
    return true;

  OmMFVector3 *const mfvector3 = dynamic_cast<OmMFVector3 *>(selectedField->value());
  assert(mfvector3->size() == 0);

  if (dialog.inertiaMatrixType() == OmAddInertiaMatrixDialog::IDENTITY_MATRIX) {
    if (physics && physics->mass() <= 0.0) {
      physics->setMass(1.0, true);
      physics->parsingInfo(tr("A positive mass is mandatory when using inertiaMatrix. 'mass' set to 1."));
    }

    if (physics && physics->centerOfMass().size() == 0) {
      physics->setCenterOfMass(0.0, 0.0, 0.0, true);
      physics->parsingInfo(tr("A center of mass is mandatory when using inertiaMatrix. Default center of mass inserted."));
    }

    mfvector3->insertItem(0, OmVector3(1.0, 1.0, 1.0));
    mfvector3->insertDefaultItem(1);

  } else if (dialog.inertiaMatrixType() == OmAddInertiaMatrixDialog::BOUNDING_OBJECT_BASED && solid)
    solid->setInertiaMatrixFromBoundingObject();

  OmWorld::instance()->setModified();

  updateToolbar();

  return true;
}

void OmSceneTree::addNew() {
  if (mSelectedItem == NULL) {
    mSelectedItem = mModel->rootItem()->lastChild();
    assert(mSelectedItem);
  }

  // set selected OmField and OmNode
  const OmTreeItem *selectedFieldItem = NULL;
  OmField *selectedField = NULL;
  OmNode *selectedNodeParent = NULL;
  int newNodeIndex = 0;

  if (!mSelectedItem->isNode()) {
    // field or item
    if (mSelectedItem->isField()) {  // field
      selectedFieldItem = mSelectedItem;
      selectedField = selectedFieldItem->field();
    } else {  // item
      newNodeIndex = mSelectedItem->row() + 1;
      selectedFieldItem = mSelectedItem->parent();
      selectedField = selectedFieldItem->field();
    }

    // if multiple item field
    // directly add item without opening the dialog
    OmMultipleValue *const mvalue = dynamic_cast<OmMultipleValue *>(selectedField->value());
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(selectedField->value());
    if (mvalue && !mfnode) {
      if (insertInertiaMatrix(selectedField))
        return;

      // add default item
      OmUndoStack::instance()->push(new OmAddItemCommand(selectedField, mvalue, newNodeIndex));
      return;
    }

    selectedNodeParent = mSelectedItem->parent()->node();
    if (!selectedNodeParent)
      return;
  } else {  // node
    newNodeIndex = mSelectedItem->row() + 1;
    selectedFieldItem = mSelectedItem->parent();
    selectedField = selectedFieldItem->field();
    selectedNodeParent = mSelectedItem->node()->parentNode();
  }

  assert(selectedNodeParent && selectedField);
  if (selectedField->name().startsWith("device") && OmNodeUtilities::hasARobotAncestor(selectedNodeParent) == false) {
    OmMessageBox::info("You cannot insert a device inside a Joint node which is not part of a Robot. Consider transforming the "
                       "Solid at the top into a Robot",
                       this, "Device insertion disabled");
    return;
  }

  OmAddNodeDialog dialog(selectedNodeParent, selectedField, newNodeIndex, this);

  if (dialog.exec() == QDialog::Rejected)
    return;

  // create node
  OmNode::setGlobalParentNode(selectedNodeParent);
  OmNode *newNode;
  if (dialog.isUseNode()) {
    // find last DEF node to be copied
    OmNode *const definitionNode = dialog.defNode();
    if (!definitionNode) {
      OmLog::error(tr("New node creation failed: node with DEF name %1 does not exist.").arg(dialog.modelName()));
      return;
    }
    newNode = definitionNode->cloneAndReferenceProtoInstance();
    newNode->makeUseNode(definitionNode);

  } else {
    const QString &strUrl = dialog.protoUrl();
    const QString *const protoUrl = strUrl.isEmpty() ? NULL : &strUrl;
    newNode = OmConcreteNodeFactory::instance()->createNode(dialog.modelName(), NULL, selectedNodeParent, protoUrl);
  }

  if (!newNode) {
    OmLog::error(tr("New node creation failed: model name %1.").arg(dialog.modelName()));
    return;
  }

  const OmNodeOperations::OperationResult result =
    OmNodeOperations::instance()->initNewNode(newNode, selectedNodeParent, selectedField, newNodeIndex);
  if (result == OmNodeOperations::FAILURE)
    return;
  const bool isNodeRegenerated = result == OmNodeOperations::REGENERATION_REQUIRED;

  // if selectedField is a template regenerator, the parent will anyway be regenerated
  if (!isNodeRegenerated && !selectedField->isTemplateRegenerator())
    OmNodeOperations::instance()->notifyNodeAdded(newNode);

  updateSelection();

  if (isNodeRegenerated && mSelectedItem && mSelectedItem->isField() && !mSelectedItem->isSFNode()) {
    QModelIndex newItemModelIndex = mModel->itemToIndex(mSelectedItem->child(newNodeIndex));
    mTreeView->setCurrentIndex(newItemModelIndex);
  }

  mTreeView->scrollToModelIndex(mModel->itemToIndex(mSelectedItem));

  OmWorld::instance()->setModifiedFromSceneTree();

  updateValue();
  updateToolbar();
}

void OmSceneTree::updateToolbar() {
  if (mRowsAreAboutToBeRemoved || OmNodeOperations::instance()->isSkipUpdates())
    // don't use mSelectedItem if updateSelection() was skipped
    return;

  mActionManager->setEnabled(OmAction::DEL, mSelectedItem && mSelectedItem->canDelete());
  mActionManager->setEnabled(OmAction::ADD_NEW, !mSelectedItem || mSelectedItem->canInsert());
  updateApplicationActions();
}

void OmSceneTree::updateApplicationActions() {
  mClipboard->update();
  mActionManager->setEnabled(OmAction::UNDO, OmUndoStack::instance()->canUndo());
  mActionManager->setEnabled(OmAction::REDO, OmUndoStack::instance()->canRedo());
  mActionManager->setEnabled(OmAction::COPY, mSelectedItem && mSelectedItem->canCopy());
  mActionManager->setEnabled(OmAction::CUT, mSelectedItem && mSelectedItem->canCut());
  mActionManager->setEnabled(OmAction::PASTE, isPasteAllowed());
  mActionManager->setEnabled(OmAction::SELECT_ALL, false);
}

bool OmSceneTree::isPasteAllowed() {
  if (!mSelectedItem || mSelectedItem->isInvalid() || mClipboard->isEmpty())
    return false;

  OmField *field = NULL;
  if (mClipboard->type() == WB_SF_NODE) {
    if (mSelectedItem->isItem())
      return false;

    if (mSelectedItem->isField() && !(mSelectedItem->field()->type() & WB_SF_NODE))
      // selected field is neither SFNode nor MFNode
      return false;

    // paste SFNode
    OmNode *parentNode = NULL;
    if (mSelectedItem->isField()) {
      field = mSelectedItem->field();
      parentNode = mSelectedItem->parent()->node();
    } else {  // else sibling node
      field = mSelectedItem->parent()->field();
      parentNode = mSelectedItem->node()->parentNode();
    }

    // prevent pasting a node between WorldInfo and Viewpoint nodes
    if (parentNode->isWorldRoot() && mSelectedItem->row() < 1)
      return false;

    if (!(field->type() & WB_SF_NODE))
      return false;

    // semantic checks
    const OmClipboard::OmClipboardNodeInfo *clipboardNodeInfo = mClipboard->nodeInfo();
    const QString &nodeModelName = clipboardNodeInfo->nodeModelName;
    QString errorMessage;
    if (!OmNodeUtilities::isAllowedToInsert(field, parentNode, errorMessage,
                                            static_cast<const OmBaseNode *>(parentNode)->nodeUse(), clipboardNodeInfo->slotType,
                                            nodeModelName, clipboardNodeInfo->modelName, clipboardNodeInfo->protoParentList))
      return false;

    if (clipboardNodeInfo->hasADeviceDescendant)
      // allow to paste devices node only in robot nodes
      return OmNodeUtilities::isRobotTypeName(nodeModelName) || OmNodeUtilities::hasARobotAncestor(parentNode);
    if (clipboardNodeInfo->hasAConnectorDescendant)
      // allow to paste connecter node only if it has a solid ancestor node
      return OmNodeUtilities::isSolidTypeName(nodeModelName) || dynamic_cast<OmSolid *>(parentNode) ||
             OmNodeUtilities::findUpperSolid(parentNode);

    return true;
  } else {
    if (mSelectedItem->isField())
      field = mSelectedItem->field();
    else if (mSelectedItem->isItem())
      field = mSelectedItem->parent()->field();
  }

  if (mSelectedItem->isNode() || mSelectedItem->isSFNode() || !field || (field->isMultiple() && !mSelectedItem->canInsert()))
    return false;

  int selectedType = mSelectedItem->field()->singleType();
  return mClipboard->type() == selectedType;
}

void OmSceneTree::clearSelection() {
  if (mModel == NULL || mTreeView == NULL)
    // quitting OmniSim
    return;

  mTreeView->clearSelection();
  mTreeView->setCurrentIndex(QModelIndex());
  mSelectedItem = NULL;

  mFieldEditor->setTitle("");
  mFieldEditor->editField(NULL, NULL);
}

void OmSceneTree::enableObjectViewActions(bool enabled) {
  mActionManager->action(OmAction::MOVE_VIEWPOINT_TO_OBJECT)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_FRONT_VIEW)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_BACK_VIEW)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_RIGHT_VIEW)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_LEFT_VIEW)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_TOP_VIEW)->setEnabled(enabled);
  mActionManager->action(OmAction::OBJECT_BOTTOM_VIEW)->setEnabled(enabled);
}

void OmSceneTree::updateSelection() {
  if (mTreeView == NULL)
    // quitting OmniSim
    return;

  OmNodeOperations *nodeOperations = OmNodeOperations::instance();
  if (nodeOperations->isFromSupervisor())
    // do not update selection if change come from supervisor
    return;

  if (nodeOperations->areNodesAboutToBeInserted() || mRowsAreAboutToBeRemoved || nodeOperations->isSkipUpdates())
    // avoid updating the selection if some nodes are about to be inserted or deleted
    return;

  QModelIndex currentIndex = mTreeView->currentIndex();
  if (!currentIndex.isValid()) {
    mSelectedItem = NULL;
    enableObjectViewActions(false);
    mActionManager->action(OmAction::OPEN_HELP)->setEnabled(false);
    mActionManager->action(OmAction::EDIT_FIELD)->setEnabled(false);
    updateToolbar();
    // no item selected
    return;
  }
  mSelectedItem = mModel->indexToItem(currentIndex);
  if (mSelectedItem->isInvalid()) {
    mSelectedItem = NULL;
    enableObjectViewActions(false);
    mActionManager->action(OmAction::OPEN_HELP)->setEnabled(false);
    mActionManager->action(OmAction::EDIT_FIELD)->setEnabled(false);
    updateToolbar();
    return;
  }

  bool isNonNullNode = false;

  OmField *const field = mSelectedItem->field();
  if (mSelectedItem->isField()) {
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(field->value());
    isNonNullNode = sfnode && sfnode->value();
    OmNode *const node = mSelectedItem->parent()->node();
    mFieldEditor->editField(node, field, -1);
  } else if (mSelectedItem->isItem()) {
    OmNode *const node = mSelectedItem->parent()->parent()->node();
    mFieldEditor->editField(node, field, mSelectedItem->row());
  } else {  // node
    OmNode *const node = mSelectedItem->parent()->node();
    isNonNullNode = true;
    mFieldEditor->editField(node, mSelectedItem->parent()->field(), mSelectedItem->row());
  }

  mActionManager->action(OmAction::EDIT_FIELD)->setEnabled(mSplitter->sizes()[2] == 0);
  OmContextMenuGenerator::enableNodeActions(mSelectedItem->isNode());
  OmContextMenuGenerator::enableRobotActions(mSelectedItem->node() &&
                                             OmNodeUtilities::isRobotTypeName(mSelectedItem->node()->nodeModelName()));
  if (mSelectedItem->node() && mSelectedItem->node()->isProtoInstance()) {
    OmContextMenuGenerator::enableProtoActions(true);
    const QString &url = mSelectedItem->node()->proto()->url();
    OmContextMenuGenerator::enableExternProtoActions(OmUrl::isWeb(url) && OmNetwork::instance()->isCachedWithMapUpdate(url));
  } else {
    OmContextMenuGenerator::enableProtoActions(false);
    OmContextMenuGenerator::enableExternProtoActions(false);
  }

  QWidget *lastEditorWidget = mFieldEditor->lastEditorWidget();
  if (lastEditorWidget)
    setTabOrder(lastEditorWidget, mTreeView);

  updateToolbar();

  // emit a message in order to inform OmSelection about the selected node
  const OmTreeItem *const item = isNonNullNode ? mSelectedItem : mModel->findUpperNodeItem(mSelectedItem);
  if (item) {
    OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(item->node());
    if (baseNode && baseNode->isProtoParameterNode())
      // select first proto parameter node instance
      baseNode = baseNode->getFirstFinalizedProtoInstance();

    if (baseNode && !baseNode->isPostFinalizedCalled())
      // ignore not initialized nodes
      baseNode = NULL;

    // enable move viewpoint to object if the item has a corresponding bounding sphere
    enableObjectViewActions(baseNode && OmNodeUtilities::boundingSphereAncestor(baseNode) != NULL &&
                            baseNode->nodeType() != WB_NODE_BILLBOARD &&
                            !OmNodeUtilities::findUpperNodeByType(baseNode, WB_NODE_BILLBOARD));
    mActionManager->action(OmAction::OPEN_HELP)->setEnabled(baseNode);
    emit nodeSelected(baseNode);
  }

  // uncollapse the field editor
  showFieldEditor();
}

void OmSceneTree::startWatching(const QModelIndex &index) {
  mModel->startWatching(index);
}

void OmSceneTree::stopWatching(const QModelIndex &index) {
  mModel->stopWatching(index);
}

bool OmSceneTree::isIndexAncestorOfCurrentIndex(const QModelIndex &index, int start, int end) {
  QModelIndex currentIndex = mTreeView->currentIndex();
  while (currentIndex.isValid()) {
    if (currentIndex.parent() == index && currentIndex.row() >= start && currentIndex.row() <= end)
      return true;
    currentIndex = currentIndex.parent();
  }
  return false;
}

void OmSceneTree::handleRowRemoval(const QModelIndex &parentIndex, int start, int end) {
  mRowsAreAboutToBeRemoved = false;
  if (!OmNodeOperations::instance()->isFromSupervisor() || isIndexAncestorOfCurrentIndex(parentIndex, start, end))
    clearSelection();
  updateToolbar();
}

void OmSceneTree::selectPose(OmAbstractPose *p) {
  if (p == NULL) {
    clearSelection();
    return;
  }

  QModelIndex newIndex = mModel->findModelIndexFromNode(p->baseNode());
  if (newIndex.isValid()) {
    mTreeView->clearSelection();
    mTreeView->setCurrentIndex(newIndex);
    mTreeView->scrollToModelIndex(newIndex);
  } else if (p->baseNode()->protoParameterNode())
    // if m is proto parameter node instance, select the corresponding parameter node in the scene tree
    selectPose(dynamic_cast<OmAbstractPose *>(p->baseNode()->protoParameterNode()));
}

// for the translation and rotation fields of Solid node we need to set
// the initial translation and rotation values
void OmSceneTree::updateValue() {
  OmWorld *world = OmWorld::instance();
  world->setModified();

  if (mSelectedItem && mSelectedItem->isField()) {
    OmField *const field = mSelectedItem->field();
    QString fieldName = field->name();
    if (fieldName == "scale" || mSelectedItem->isSFNode()) {
      // update values displayed in field editor
      mFieldEditor->editField(mSelectedItem->parent()->node(), field, -1);
    }

    mFieldEditor->updateValue();
  }

  emit valueChangedFromGui();
}

void OmSceneTree::refreshItems() {
  if (mModel && mModel->rootItem() && OmSimulationState::instance()->isPaused()) {
    mModel->updateAllSceneTreeValues();
  } else if (mSelectedItem && mSelectedItem->isDataRefreshNeeded()) {
    mSelectedItem->refreshData();
  } else if (mModel)
    // QTreeView refresh is problematic and sometimes won't happen unless forced
    // so we force it here in the default case by telling the OmSceneTreeModel
    // to refresh its data
    mModel->emitLayoutChanged();
}

QByteArray OmSceneTree::saveState() const {
  return mSplitter->saveState();
}

void OmSceneTree::restoreState(QByteArray state) {
  mSplitter->restoreState(state);
  mSplitter->setHandleWidth(mHandleWidth);
}

void OmSceneTree::restoreFactoryLayout() {
  const int halfSplitterHeight = mSplitter->height() * 0.5;
  int preferredFieldEditorHeight = gFactoryFieldEditorHeightHint;
  if (preferredFieldEditorHeight > halfSplitterHeight)
    // default field editor height should never be bigger than scene tree height
    preferredFieldEditorHeight = halfSplitterHeight;

  QList<int> sizes;
  sizes << (mSplitter->height() - preferredFieldEditorHeight) << preferredFieldEditorHeight;
  mSplitter->setSizes(sizes);
  mSplitter->setHandleWidth(mHandleWidth);
}

void OmSceneTree::prepareNodeRegeneration(OmNode *node, bool nested) {
  // The node given as argument will be regenerated soon:
  // - the tree state at this position is stored for a later restoration.
  // - the view updates are blocked.
  // - the focus widget is stored.

  if (nested) {
    if (mSelectedItem)
      // In the case of nested PROTOs, prepareNodeRegeneration() will be
      // called again later on the uppermost PROTO which requires a regeneration.
      // The mSelectedItem pointer should be kept until this next call.
      // Setting it as invalid helps to avoid bad pointer references during the procedural PROTO regeneration.
      mSelectedItem->makeInvalid();
    return;
  }

  assert(node);

  setUpdatesEnabled(false);

  mFocusWidgetBeforeNodeRegeneration = QApplication::focusWidget();

  mSelectionBeforeTreeStateRegeneration = NULL;

  // Store the selected item only if not inside the node which will be regenerated.
  // Indeed this node (and its OmTreeItem(s)) will be destroyed and recreated.
  const OmNode *n = NULL;
  if (mSelectedItem && !mSelectedItem->isInvalid()) {
    if (mSelectedItem->isField()) {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(mSelectedItem->field()->value());
      if (sfnode && sfnode->value())
        n = sfnode->value();
      else
        n = mSelectedItem->parent()->node();
    } else if (mSelectedItem->isItem())
      n = mSelectedItem->parent()->parent()->node();
    else  // node
      n = mSelectedItem->node();
  }
  mSelectionInsideTreeStateRecovery = n == NULL;
  while (n) {
    if (n == node || n->protoParameterNode() == node) {
      mSelectionInsideTreeStateRecovery = true;
      break;
    }
    n = n->parentNode();
  }
  if (!mSelectionInsideTreeStateRecovery)
    mSelectionBeforeTreeStateRegeneration = mModel->indexToItem(mTreeView->currentIndex());

  // Store the tree state of the node about to be regenerated.
  cleanTreeItemState(mTreeItemState);
  mTreeItemState = new TreeItemState;
  storeTreeItemState(mModel->indexToItem(mModel->findModelIndexFromNode(node)), mTreeItemState);

  // Clear the selection the time to regnerate the node, in order to avoid invalid pointers.
  clearSelection();
}

void OmSceneTree::abortNodeRegeneration() {
  // The node regeneration failed: restore the best possible state.

  cleanTreeItemState(mTreeItemState);
  mTreeItemState = NULL;

  // Restore the tree state as best as possible
  if (!mSelectionInsideTreeStateRecovery) {
    QModelIndex index = mModel->itemToIndex(mSelectionBeforeTreeStateRegeneration);
    if (index.isValid()) {
      mSelectedItem = mSelectionBeforeTreeStateRegeneration;
      mTreeView->setCurrentIndex(index);
    } else {
      mSelectedItem = NULL;
      mTreeView->setCurrentIndex(QModelIndex());  // mTreeView->clearSelection() doesn't change the current index.
    }
    mSelectionBeforeTreeStateRegeneration = NULL;
  }
  updateSelection();

  setUpdatesEnabled(true);

  if (mFocusWidgetBeforeNodeRegeneration) {
    mFocusWidgetBeforeNodeRegeneration->setFocus();
    mFocusWidgetBeforeNodeRegeneration = NULL;
  }
}

void OmSceneTree::applyNodeRegeneration(OmNode *node) {
  assert(node);

  // Restore the tree state as best as possible
  restoreTreeItemState(mModel->indexToItem(mModel->findModelIndexFromNode(node)), mTreeItemState, NULL);
  cleanTreeItemState(mTreeItemState);
  mTreeItemState = NULL;

  // Restore the selection if the previous selection was outside the regenerated node.
  if (!mSelectionInsideTreeStateRecovery) {
    QModelIndex index = mModel->itemToIndex(mSelectionBeforeTreeStateRegeneration);
    if (index.isValid()) {
      mSelectedItem = mSelectionBeforeTreeStateRegeneration;
      mTreeView->setCurrentIndex(index);
    } else {
      mSelectedItem = NULL;
      mTreeView->setCurrentIndex(QModelIndex());  // mTreeView->clearSelection() doesn't change the current index.
    }
    mSelectionBeforeTreeStateRegeneration = NULL;
  }
  updateSelection();

  setUpdatesEnabled(true);

  if (mFocusWidgetBeforeNodeRegeneration) {
    mFocusWidgetBeforeNodeRegeneration->setFocus();
    mFocusWidgetBeforeNodeRegeneration = NULL;
  }
}

void OmSceneTree::storeTreeItemState(OmTreeItem *treeItem, TreeItemState *treeItemState) {
  // Store a tree state at a given item.

  assert(treeItem);
  assert(treeItemState);

  QModelIndex index = mModel->itemToIndex(treeItem);

  treeItemState->expanded = mTreeView->isExpanded(index);
  treeItemState->selected = mSelectedItem == treeItem;

  if (treeItemState->expanded) {  // no need to store the children of an unexpanded node.
    for (int i = 0; i < treeItem->childCount(); ++i) {
      TreeItemState *newChild = new TreeItemState;
      storeTreeItemState(treeItem->child(i), newChild);
      treeItemState->children.append(newChild);
    }
  }
}

void OmSceneTree::restoreTreeItemState(OmTreeItem *treeItem, TreeItemState *treeItemState, OmBaseNode *lastNode) {
  // Restore a tree state at a given item.

  assert(treeItem);
  assert(treeItemState);

  QModelIndex index = mModel->itemToIndex(treeItem);

  // Keep the bottommost node in the selection tree, in order to be able to restore its selection later.
  OmBaseNode *newLastNode = (treeItem->isNode()) ? dynamic_cast<OmBaseNode *>(treeItem->node()) : lastNode;

  // Restore the expansion status.
  if (treeItemState->expanded)
    mTreeView->setExpanded(index, true);
  // Restore the selection status.
  if (treeItemState->selected) {
    // 1. Restore the node selection.
    if (newLastNode && newLastNode->isPostFinalizedCalled())
      OmSelection::instance()->selectNodeFromSceneTree(newLastNode);
    // 2. Restore the tree index which could be a field or a node.
    mTreeView->setCurrentIndex(index);
    mSelectedItem = treeItem;
  }

  int counter = 0;
  foreach (TreeItemState *child, treeItemState->children) {
    if (counter < treeItem->childCount())
      restoreTreeItemState(treeItem->child(counter), child, newLastNode);
    counter++;
  }
}

void OmSceneTree::cleanTreeItemState(TreeItemState *item) {
  // clean a tree state.
  if (item == NULL)
    return;

  foreach (TreeItemState *fChild, item->children)
    cleanTreeItemState(fChild);
  delete item;
}

// Debug function to print the stored tree state.
// #include <QtCore/QDebug>
// void OmSceneTree::printTreeItemState(TreeItemState *treeItemState, int indentation) {
//
//   QString indent(2 * indentation, ' ');
//
//   qDebug() << indent << treeItemState->expanded;
//   qDebug() << indent << treeItemState->selected;
//
//   foreach (TreeItemState *it, treeItemState->children)
//     printTreeItemState(it, indentation + 1);
// }

void OmSceneTree::handleDoubleClickOrEnterPress() {
  if (!mSelectedItem)
    return;

  // we can't use isDefault() on the SFNode field because PROTOs can have
  // non-NULL default SFNode values, so cast to SFNode and get the real value
  // stored in the field
  if ((mSelectedItem->isSFNode() && mSelectedItem->node() == NULL) ||
      (mSelectedItem->isField() && mSelectedItem->field()->isMultiple() &&
       reinterpret_cast<OmMultipleValue *>(mSelectedItem->field()->value())->isEmpty()))
    addNew();
  // set focus on first edit box of the current value editor for immediate keyboard editing
  else if ((mSelectedItem->isItem() && !mSelectedItem->isNode() && mSelectedItem->field()->isMultiple()) ||
           (mSelectedItem->isField() && !mSelectedItem->isSFNode() && !mSelectedItem->field()->isMultiple()))
    mFieldEditor->currentEditor()->takeKeyboardFocus();
  // default behavior, collapse/expand tree item
  else if (!mTreeView->isExpanded(mTreeView->currentIndex()))
    mTreeView->expand(mTreeView->currentIndex());
  else {
    mTreeView->collapse(mTreeView->currentIndex());
    return;  // do not show field editor when collasping tree item
  }

  showFieldEditor(true);
}

void OmSceneTree::refreshTreeView() {
  mModel->emitLayoutChanged();
}

void OmSceneTree::help() {
  if (!mSelectedItem)
    return;

  const OmNode *node = mSelectedItem->node();
  if (!node && mSelectedItem->field())
    node = mSelectedItem->field()->parentNode();
  if (node) {
    const QStringList &bookAndPage = node->documentationBookAndPage(OmNodeUtilities::isRobotTypeName(node->nodeModelName()));
    emit documentationRequest(bookAndPage[0], bookAndPage[1], true);
  }
}

void OmSceneTree::exportUrdf() {
  assert(mSelectedItem && mSelectedItem->node() && mSelectedItem->node()->isRobot());

  // Fix for Qt 5.3.0 that does not work correctly on Ubuntu
  // if dialog parent widget is not a top level widget
  QWidget *topLevelWidget = this;
  while (topLevelWidget->parentWidget())
    topLevelWidget = topLevelWidget->parentWidget();

  const QString fileName = QFileDialog::getSaveFileName(
    topLevelWidget, tr("Export to URDF"),
    OmProject::computeBestPathForSaveAs(OmPreferences::instance()->value("Directories/objects").toString() + "/" +
                                        mSelectedItem->node()->modelName() + ".urdf"),
    tr("URDF (*.urdf *.URDF)"));

  if (fileName.isEmpty())
    return;

  if (!fileName.endsWith(".urdf", Qt::CaseInsensitive)) {
    OmLog::error(tr("Unsupported '%1' extension.").arg(QFileInfo(fileName).suffix()));
    return;
  }

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly)) {
    OmLog::error(tr("Impossible to write file: '%1'.").arg(fileName) + "\n" + tr("URDF export failed."));
    return;
  }

  OmNode::enableDefNodeTrackInWrite(true);
  OmWriter writer(&file, fileName);
  writer.writeHeader(fileName);
  mSelectedItem->node()->write(writer);
  writer.writeFooter();
  OmNode::disableDefNodeTrackInWrite();
  file.close();
}

void OmSceneTree::editFileFromFieldEditor(const QString &fileName) {
  emit editRequested(fileName);
}

void OmSceneTree::openProtoInTextEditor() {
  if (mSelectedItem && mSelectedItem->node())
    emit editRequested(mSelectedItem->node()->proto()->url(), false, mSelectedItem->node()->isRobot());
}

void OmSceneTree::editProtoInTextEditor() {
  if (mSelectedItem && mSelectedItem->node())
    emit editRequested(mSelectedItem->node()->proto()->url(), true, mSelectedItem->node()->isRobot());
}

void OmSceneTree::openTemplateInstanceInTextEditor() {
  if (!mSelectedItem)
    return;
  const OmNode *node = mSelectedItem->node();
  if (!node || !node->isTemplate())
    return;
  QDir tmpDir(OmStandardPaths::webotsTmpPath());
  const QString generatedProtos("generated_protos");
  tmpDir.mkdir(generatedProtos);
  QFile file(
    QString("%1%2/%3.generated_proto").arg(OmStandardPaths::webotsTmpPath()).arg(generatedProtos).arg(node->proto()->name()));
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    OmLog::error(tr("Could not create temporary file: '%1'.").arg(file.fileName()));
    return;
  }
  file.write(node->protoInstanceTemplateContent());
  file.close();
  if (!file.fileName().isEmpty())
    emit editRequested(file.fileName());
}

void OmSceneTree::showFieldEditor(bool force) {
  if (dynamic_cast<QAction *>(sender()) != NULL)
    force = true;
  static bool hiddenByUser = false;
  const QList<int> currentSize = mSplitter->sizes();
  if (currentSize[2] != 0) {
    hiddenByUser = true;
    return;
  }
  if (!force && hiddenByUser)
    return;
  QList<int> sizes;
  sizes << currentSize[0] << (mSplitter->height() - 1) << 1;
  mSplitter->setSizes(sizes);
  mSplitter->setHandleWidth(mHandleWidth);
  return;
}
