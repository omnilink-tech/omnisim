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

#include "OmAddNodeDialog.hpp"

#include "OmBaseNode.hpp"
#include "OmClipboard.hpp"
#include "OmDesktopServices.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMessageBox.hpp"
#include "OmNetwork.hpp"
#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmProjectRelocationDialog.hpp"
#include "OmProtoIcon.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmSFNode.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

#include <QtCore/QRegularExpression>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>

#include <cassert>

enum Category { NEW = 20001, USE = 20002 };

OmAddNodeDialog::OmAddNodeDialog(OmNode *currentNode, OmField *field, int index, QWidget *parent) :
  QDialog(parent),
  mCurrentNode(currentNode),
  mField(field),
  mIndex(index),
  mUsesItem(NULL),
  mNewNodeType(UNKNOWN),
  mDefNodeIndex(-1),
  mRetrievalTriggered(false) {
  assert(mCurrentNode && mField);

  // check if top node is a robot node
  const OmNode *const topNode =
    field ? OmVrmlNodeUtilities::findTopNode(mCurrentNode) : OmVrmlNodeUtilities::findTopNode(mCurrentNode->parentNode());
  mHasRobotTopNode = topNode ? OmNodeUtilities::isRobotTypeName(topNode->nodeModelName()) : false;

  setWindowTitle(tr("Add a node"));

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(mTree, &QTreeWidget::doubleClicked, this, &OmAddNodeDialog::checkAndAddSelectedItem);

  QFont font;
  font.fromString(OmPreferences::instance()->value("Editor/font").toString());

  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setFont(font);
  mFindLineEdit->setClearButtonEnabled(true);
  connect(mFindLineEdit, &QLineEdit::textChanged, this, &OmAddNodeDialog::buildTree);

  QHBoxLayout *filterLayout = new QHBoxLayout;
  filterLayout->addStretch();
  QLabel *findLabel = new QLabel(tr("Find:"), this);
  filterLayout->addWidget(findLabel);
  filterLayout->addWidget(mFindLineEdit);

  QString toolTip(tr("Filter node names. "
                     "Only the node names containing the given string are displayed in the tree below. "
                     "Regular expressions can be used."));
  findLabel->setToolTip(toolTip);
  mFindLineEdit->setToolTip(toolTip);

  mInfoText = new QPlainTextEdit(this);
  mInfoText->setFont(font);
  mInfoText->setReadOnly(true);
  mInfoText->setFocusPolicy(Qt::ClickFocus);

  mNodeInfoGroupBox = new QGroupBox(this);
  mNodeInfoGroupBox->setObjectName("dialogInfoGroupBox");
  mNodeInfoGroupBox->setFont(font);
  mNodeInfoGroupBox->setFlat(false);

  mPixmapLabel = new QLabel(this);
  mPixmapLabel->setObjectName("nodePixmapLabel");
  mPixmapLabel->setMinimumSize(128, 128);
  mPixmapLabel->setMaximumSize(mPixmapLabel->minimumSize());

  mDocumentationLabel = new QLabel(this);
  mDocumentationLabel->setObjectName("documentationLabel");
  connect(mDocumentationLabel, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  mDocumentationLabel->setWordWrap(true);
  font.setItalic(true);
  mDocumentationLabel->setFont(font);

  mLicenseLabel = new QLabel(this);
  mLicenseLabel->setObjectName("licenseLabel");
  connect(mLicenseLabel, &QLabel::linkActivated, &OmDesktopServices::openUrl);
  mLicenseLabel->setWordWrap(true);
  font.setItalic(true);
  mLicenseLabel->setFont(font);

  QPushButton *const cancelButton = new QPushButton(tr("Cancel"), this);
  cancelButton->setFocusPolicy(Qt::ClickFocus);
  mAddButton = new QPushButton(tr("Add"), this);
  mAddButton->setFocusPolicy(Qt::ClickFocus);
  connect(cancelButton, &QPushButton::pressed, this, &OmAddNodeDialog::reject);
  connect(mAddButton, &QPushButton::pressed, this, &OmAddNodeDialog::accept);

  QHBoxLayout *const mainLayout = new QHBoxLayout(this);
  QVBoxLayout *const rightPaneLayout = new QVBoxLayout();
  QVBoxLayout *const nodeInfoLayout = new QVBoxLayout();

  nodeInfoLayout->addWidget(mPixmapLabel, 0, Qt::AlignHCenter);
  nodeInfoLayout->addWidget(mInfoText);
  nodeInfoLayout->addWidget(mDocumentationLabel);
  nodeInfoLayout->addWidget(mLicenseLabel);
  mNodeInfoGroupBox->setLayout(nodeInfoLayout);

  QDialogButtonBox *const buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mAddButton, QDialogButtonBox::AcceptRole);

  buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
  buttonBox->setFocusPolicy(Qt::ClickFocus);

  QHBoxLayout *buttonLayout = new QHBoxLayout();
  buttonLayout->addWidget(buttonBox);

  if (OmSimulationState::instance()->hasStarted()) {
    QPixmap pixmap("coreIcons:warning.png");
    QLabel *pixmapLabel = new QLabel(this);
    pixmapLabel->setPixmap(pixmap.scaledToHeight(20, Qt::SmoothTransformation));
    pixmapLabel->setToolTip(tr("The simulation has run!"));
    buttonLayout->addWidget(pixmapLabel);
  }

  rightPaneLayout->addLayout(filterLayout);
  rightPaneLayout->addWidget(mNodeInfoGroupBox);
  rightPaneLayout->addLayout(buttonLayout);

  mainLayout->addWidget(mTree);
  mainLayout->addLayout(rightPaneLayout);

  setMinimumSize(800, 500);

  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &OmAddNodeDialog::updateItemInfo);

  // retrieve PROTO dependencies of all locally available PROTO prior to generating the dialog
  connect(OmProtoManager::instance(), &OmProtoManager::dependenciesAvailable, this, &OmAddNodeDialog::buildTree);
  OmProtoManager::instance()->retrieveLocalProtoDependencies();
}

void OmAddNodeDialog::setPixmap(const QString &pixmapPath) {
  QPixmap pixmap(pixmapPath);
  if (!pixmap.isNull()) {
    if (pixmap.size() != QSize(128, 128)) {
      OmLog::warning(tr("The \"%1\" icon should have a dimension of 128x128 pixels.").arg(pixmapPath));
      pixmap = pixmap.scaled(128, 128);
    }
    mPixmapLabel->show();
    mPixmapLabel->setPixmap(pixmap);
  }
}

void OmAddNodeDialog::updateIcon(const QString &path) {
  setPixmap(path.isEmpty() ? OmUrl::missingProtoIcon() : path);

  OmProtoIcon *protoIcon = dynamic_cast<OmProtoIcon *>(sender());
  assert(protoIcon);
  protoIcon->deleteLater();
}

QString OmAddNodeDialog::modelName() const {
  QString modelName(mTree->selectedItems().at(0)->text(MODEL_NAME));
  if (mNewNodeType == PROTO || mNewNodeType == USE)
    return modelName.split(QRegularExpression("\\W+"))[0];  // return only proto/use name without model name

  return modelName;
}

QString OmAddNodeDialog::protoUrl() const {
  if (mNewNodeType != PROTO)
    return QString();

  return OmUrl::resolveUrl(mTree->selectedItems().at(0)->text(FILE_NAME));
}

OmNode *OmAddNodeDialog::defNode() const {
  assert(mDefNodeIndex >= 0);
  return mDefNodes[mDefNodeIndex];
}

void OmAddNodeDialog::updateItemInfo() {
  if (mTree->selectedItems().size() != 1)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  const QString selectedNode(selectedItem->text(MODEL_NAME));
  mNodeInfoGroupBox->setTitle(selectedNode);
  bool validForInsertionInBoundingObject = true;
  mLicenseLabel->hide();
  mDocumentationLabel->hide();
  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    // a folder is selected
    mPixmapLabel->hide();

    switch (topLevel->type()) {
      case Category::NEW:
        mInfoText->setPlainText(tr("This folder lists all OmniSim base nodes that are suitable to insert at (or below) the "
                                   "currently selected Scene Tree line."));
        break;
      case Category::USE:
        mInfoText->setPlainText(
          tr("This folder lists all suitable node that were defined (using DEF) above the current line of the Scene Tree."));
        break;
      case OmProtoManager::PROTO_WORLD: {
        const QString &worldFile = OmWorld::instance()->fileName();
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the world file: '%1'.").arg(worldFile));
        break;
      }
      case OmProtoManager::PROTO_PROJECT:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the local 'protos' directory: '%1'.")
                                  .arg(OmProject::current()->protosPath()));
        break;
      case OmProtoManager::PROTO_EXTRA: {
        QString title("This folder lists all suitable PROTO nodes from the preferences Extra project path and "
                      "the 'WEBOTS_EXTRA_PROJECT_PATH' environment variable:\n");
        foreach (const OmProject *project, *OmProject::extraProjects())
          title.append(QString("- " + project->path() + "\n"));
        mInfoText->setPlainText(title);
      } break;
      case OmProtoManager::PROTO_OMNISIM:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes provided by OmniSim."));
        break;
      default:
        // no information
        mInfoText->setPlainText(tr("No info available."));
        break;
    }
  } else {
    // a node is selected
    // check if USE node
    switch (topLevel->type()) {
      case Category::USE: {
        mNewNodeType = USE;
        QString boi(selectedItem->text(BOUNDING_OBJECT_INFO));
        validForInsertionInBoundingObject = boi.isEmpty();
        showNodeInfo(selectedItem->text(FILE_NAME), USE, -1, boi);
        mDefNodeIndex = mUsesItem->indexOfChild(const_cast<QTreeWidgetItem *>(selectedItem));
        assert(mDefNodeIndex < mDefNodes.size() && mDefNodeIndex >= 0);
        break;
      }
      case OmProtoManager::PROTO_WORLD:
      case OmProtoManager::PROTO_PROJECT:
      case OmProtoManager::PROTO_EXTRA:
      case OmProtoManager::PROTO_OMNISIM:
        mDefNodeIndex = -1;
        mNewNodeType = PROTO;
        showNodeInfo(selectedItem->text(FILE_NAME), PROTO, topLevel->type());
        break;
      default:
        mDefNodeIndex = -1;
        mNewNodeType = BASIC;
        showNodeInfo(selectedNode, BASIC, -1);
        break;
    }
  }

  mAddButton->setEnabled(!selectedItem->icon(0).isNull() && validForInsertionInBoundingObject);
}

void OmAddNodeDialog::showNodeInfo(const QString &nodeFileName, NodeType nodeType, int variant,
                                   const QString &boundingObjectInfo) {
  QString description;
  QString pixmapPath;

  QString path = nodeFileName;
  if (OmFileUtil::isLocatedInDirectory(path, OmStandardPaths::cachedAssetsPath()))
    path = OmNetwork::instance()->getUrlFromEphemeralCache(nodeFileName);

  const QFileInfo fileInfo(path);
  const QString fileName = fileInfo.baseName();
  if (nodeType != PROTO && OmNodeModel::isBaseModelName(fileName)) {
    OmNodeModel *nodeModel = OmNodeModel::findModel(fileName);
    assert(nodeModel);
    description = nodeModel->info();

    // set icon path
    pixmapPath = "icons:" + fileInfo.baseName() + ".png";
  } else {
    assert(nodeType == USE || variant > 0);
    // the node is a PROTO
    QMap<QString, OmProtoInfo *> list;
    if (variant == OmProtoManager::PROTO_OMNISIM)
      list = OmProtoManager::instance()->omniSimProtoList();
    else
      list = OmProtoManager::instance()->protoInfoMap(nodeType == USE ? OmProtoManager::PROTO_WORLD : variant);

    if (!list.contains(fileName)) {
      OmLog::error(tr("'%1' is not a known proto in category '%2'.\n").arg(fileName).arg(variant));
      return;
    }

    const OmProtoInfo *info = list.value(fileName);
    assert(info);

    // set documentation url
    if (!info->documentationUrl().isEmpty()) {
      mDocumentationLabel->show();
      mDocumentationLabel->setText(
        tr("Documentation: <a style='color: #5DADE2;' href='%1'>%1</a>").arg(info->documentationUrl()));
    }

    // set license
    if (!info->license().isEmpty()) {
      QString license =
        info->license() + tr(" <a style='color: #5DADE2;' href='%1'>More information.</a>").arg(info->licenseUrl());
      mLicenseLabel->show();
      mLicenseLabel->setText(tr("License: ") + license);
    }

    description = info->description();
  }

  mInfoText->clear();

  if (!boundingObjectInfo.isEmpty())
    mInfoText->appendHtml(tr("<font color=\"red\">WARNING: this node contains a Geometry with non-positive dimensions and "
                             "hence cannot be inserted in a bounding object.</font><br/>"));

  if (description.isEmpty())
    mInfoText->setPlainText(tr("No info available."));
  else {
    // replace carriage returns with spaces where appropriate:
    // "\n\n" => "\n\n": two consecutive carriage returns are preserved (new paragraph)
    // "\n-"  => "\n-": a carriage return followed by a "-" are preserved (bullet list)
    // "\n"   => " ": a single carriage return is transformed into a space (comment line wrap)
    for (int i = 0; i < description.length(); i++) {
      if (description[i] == '\n') {
        if (i < (description.length() - 1)) {
          if (description[i + 1] == '\n' || description[i + 1] == '-') {
            i++;
            continue;
          }
        }
        description[i] = ' ';
      }
    }
    mInfoText->appendPlainText(description.trimmed());
  }
  mInfoText->moveCursor(QTextCursor::Start);

  mPixmapLabel->hide();
  if (pixmapPath.isEmpty()) {
    OmProtoIcon *icon = new OmProtoIcon(fileName, path, this);
    if (icon->isReady()) {
      setPixmap(icon->path());
      delete icon;
    } else
      connect(icon, &OmProtoIcon::iconReady, this, &OmAddNodeDialog::updateIcon);
  } else
    setPixmap(pixmapPath);
}

bool OmAddNodeDialog::doFieldRestrictionsAllowNode(const OmNode *node) const {
  foreach (const OmFieldValueRestriction restriction, mField->acceptedValues()) {
    if (restriction.isNodeAccepted(node))
      return true;
  }
  return false;
}

void OmAddNodeDialog::buildTree() {
  if (qobject_cast<OmProtoManager *>(sender()))
    disconnect(OmProtoManager::instance(), &OmProtoManager::retrievalCompleted, this, &OmAddNodeDialog::buildTree);

  mTree->clear();
  mUsesItem = NULL;
  mDefNodes.clear();

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), Category::NEW);
  QTreeWidgetItem *const worldFileProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current World File)"), OmProtoManager::PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), OmProtoManager::PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), OmProtoManager::PROTO_EXTRA);
  QTreeWidgetItem *const omniSimProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (OmniSim Projects)"), OmProtoManager::PROTO_OMNISIM);
  mUsesItem = new QTreeWidgetItem(QStringList("USE"), Category::USE);

  const QStringList basicNodes = OmNodeModel::baseModelNames();
  QTreeWidgetItem *item = NULL;

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mFindLineEdit->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  // add valid basic nodes
  const OmNode::NodeUse nodeUse = static_cast<OmBaseNode *>(mCurrentNode)->nodeUse();
  foreach (const QString &basicNodeName, basicNodes) {
    QFileInfo fileInfo(basicNodeName);
    QString errorMessage;
    if (fileInfo.baseName().contains(regexp) &&
        OmNodeUtilities::isAllowedToInsert(mField, mCurrentNode, errorMessage, nodeUse, QString(), fileInfo.baseName(),
                                           fileInfo.baseName(), QStringList())) {
      item = new QTreeWidgetItem(nodesItem, QStringList(fileInfo.baseName()));
      item->setIcon(0, QIcon("enabledIcons:node.png"));
      nodesItem->addChild(item);
    }
  }

  // add USE nodes that are suitable for insertion
  if (mUsesItem) {
    static const QString INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT("N");

    const OmField *const actualField =
      (!mField->internalFields().isEmpty() && !mField->alias().isEmpty()) ? mField->internalFields().at(0) : mField;
    bool boInfo = actualField->name() == "boundingObject";
    if (!boInfo)
      boInfo = nodeUse & OmNode::BOUNDING_OBJECT_USE;

    // populates the DEF-USE dictionary with suitable definitions located above mCurrentNode
    mDefNodes = OmDictionary::instance()->computeDefForInsertion(mCurrentNode, mField, mIndex, true);
    foreach (const OmNode *const node, mDefNodes) {
      const QString &currentDefName = node->defName();
      const QString &currentModelName = node->modelName();
      const QString &currentFullDefName = currentDefName + " (" + currentModelName + ")";
      if (!currentFullDefName.contains(regexp))
        continue;
      if (mField->hasRestrictedValues() && !doFieldRestrictionsAllowNode(node))
        continue;
      QString nodeFilePath(currentModelName);
      if (!OmNodeModel::isBaseModelName(currentModelName)) {
        nodeFilePath = OmProtoManager::instance()->externProtoUrl(node);
        if (OmUrl::isWeb(nodeFilePath))
          nodeFilePath = OmNetwork::instance()->get(nodeFilePath);
      }
      QStringList strl(QStringList() << currentFullDefName << nodeFilePath);

      if (boInfo && !(dynamic_cast<const OmBaseNode *const>(node))->isSuitableForInsertionInBoundingObject())
        strl << INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT;

      QTreeWidgetItem *const child = new QTreeWidgetItem(mUsesItem, strl);

      child->setIcon(0, QIcon("enabledIcons:node.png"));
    }
  }

  // when filtering, don't regenerate OmProtoInfo
  const bool regenerate = qobject_cast<QLineEdit *>(sender()) ? false : true;

  // note: the dialog must be populated in this order so as to ensure the correct priority is enforced (ex: PROTO in current
  // project shadows a similarly named PROTO in the extra projects path)
  mUniqueLocalProto.clear();
  // add World PROTO (i.e. referenced as EXTERNPROTO by the world file)
  int nWorldFileProtosNodes = addProtosFromProtoList(worldFileProtosItem, OmProtoManager::PROTO_WORLD, regexp, regenerate);
  // add Current Project PROTO (all PROTO locally available in the project location)
  int nProjectProtosNodes = addProtosFromProtoList(projectProtosItem, OmProtoManager::PROTO_PROJECT, regexp, regenerate);
  // add Extra PROTO (all PROTO available in the extra location)
  int nExtraProtosNodes = addProtosFromProtoList(extraProtosItem, OmProtoManager::PROTO_EXTRA, regexp, regenerate);
  // add Webots PROTO
  int nOmniSimProtosNodes = addProtosFromProtoList(omniSimProtosItem, OmProtoManager::PROTO_OMNISIM, regexp, false);

  if (nodesItem->childCount() > 0)
    mTree->addTopLevelItem(nodesItem);
  if (mUsesItem->childCount() > 0)
    mTree->addTopLevelItem(mUsesItem);
  if (worldFileProtosItem->childCount() > 0)
    mTree->addTopLevelItem(worldFileProtosItem);
  if (projectProtosItem->childCount() > 0)
    mTree->addTopLevelItem(projectProtosItem);
  if (extraProtosItem->childCount() > 0)
    mTree->addTopLevelItem(extraProtosItem);
  if (omniSimProtosItem->childCount() > 0)
    mTree->addTopLevelItem(omniSimProtosItem);

  // initial selection
  const int nBasicNodes = nodesItem->childCount();
  const int nUseNodes = mUsesItem ? mUsesItem->childCount() : 0;

  // if everything can fit in the tree height then show all
  if (nBasicNodes + nUseNodes + nWorldFileProtosNodes + nProjectProtosNodes + nExtraProtosNodes + nOmniSimProtosNodes < 20)
    mTree->expandAll();

  // if no USE nor PROTO items
  if (nBasicNodes && !nUseNodes && !nWorldFileProtosNodes && !nProjectProtosNodes && !nExtraProtosNodes && !nOmniSimProtosNodes)
    // then select first basic node
    mTree->setCurrentItem(nodesItem->child(0));
  else
    mTree->setCurrentItem(nodesItem);

  updateItemInfo();
}

int OmAddNodeDialog::addProtosFromProtoList(QTreeWidgetItem *parentItem, int type, const QRegularExpression &regexp,
                                            bool regenerate) {
  int nAddedNodes = 0;
  const QRegularExpression re(OmUrl::remoteAssetRegex(true));
  const OmNode::NodeUse nodeUse = static_cast<OmBaseNode *>(mCurrentNode)->nodeUse();

  OmProtoManager::instance()->generateProtoInfoMap(type, regenerate);

  // filter incompatible nodes
  QStringList protoList;
  QMapIterator<QString, OmProtoInfo *> it(OmProtoManager::instance()->protoInfoMap(type));
  while (it.hasNext()) {
    OmProtoInfo *info = it.next().value();

    // don't display PROTOs which contain a "hidden" or a "deprecated" tag
    const QStringList tags = info->tags();
    if (tags.contains("deprecated", Qt::CaseInsensitive) || tags.contains("hidden", Qt::CaseInsensitive))
      continue;

    // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
    const QString &baseType = info->baseType();
    QString path = info->url();
    const QString cleanPath = path.replace("omnisim://", "").replace(re, "").replace(OmStandardPaths::omniSimHomePath(), "");
    if (!cleanPath.contains(regexp) && !baseType.contains(regexp))
      continue;

    // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
    if (!mHasRobotTopNode && !OmNodeUtilities::isRobotTypeName(baseType) && info->needsRobotAncestor())
      continue;

    QString errorMessage;
    const QString nodeName = it.key();
    if (!OmNodeUtilities::isAllowedToInsert(mField, mCurrentNode, errorMessage, nodeUse, info->slotType(), baseType, nodeName,
                                            info->parents()))
      continue;

    // keep track of unique local proto that may clash
    if (!mUniqueLocalProto.contains(nodeName) && !OmUrl::isWeb(info->url()))
      mUniqueLocalProto.insert(nodeName, info->url());

    protoList << cleanPath;
  }

  // sort the list so the items are organized alphabetically
  protoList.sort(Qt::CaseInsensitive);

  // populate tree
  foreach (QString path, protoList) {
    const QString protoName = QUrl(path).fileName().replace(".proto", "", Qt::CaseInsensitive);
    QTreeWidgetItem *parent = parentItem;
    // generate sub-items based on path (they are sorted already) only for WEBOTS_PROTO
    if (type == OmProtoManager::PROTO_OMNISIM) {
      QStringList categories = path.split('/', Qt::SkipEmptyParts);
      categories.removeLast();
      foreach (const QString &category, categories) {
        if (category == "projects" || category == "protos")
          continue;

        bool exists = false;
        for (int i = 0; i < parent->childCount(); ++i) {
          if (parent->child(i)->text(0) == category) {
            parent = parent->child(i);
            exists = true;
            break;
          }
        }
        if (!exists) {
          // create sub-folder
          QTreeWidgetItem *subFolder = new QTreeWidgetItem(QStringList() << category);
          parent->addChild(subFolder);
          parent = subFolder;
        }
      }
    }

    // insert proto itself
    const OmProtoInfo *info = OmProtoManager::instance()->protoInfo(protoName, type);
    QTreeWidgetItem *protoItem =
      new QTreeWidgetItem(QStringList() << QString("%1 (%2)").arg(protoName).arg(info->baseType()) << info->url());
    protoItem->setIcon(0, QIcon("enabledIcons:proto.png"));
    if (isDeclarationConflicting(protoName, info->url())) {
      protoItem->setDisabled(true);
      protoItem->setToolTip(
        0, tr("PROTO node not available because another with the same name and different URL already exists.") +
             QString("\nEXTERNPROTO \"%1\"").arg(OmProtoManager::instance()->formatExternProtoPath(info->url())));
    } else
      protoItem->setToolTip(0,
                            QString("EXTERNPROTO \"%1\"").arg(OmProtoManager::instance()->formatExternProtoPath(info->url())));

    parent->addChild(protoItem);
    ++nAddedNodes;
  }

  return nAddedNodes;
}

bool OmAddNodeDialog::isDeclarationConflicting(const QString &protoName, const QString &url) {
  // checks if the provided proto name / URL conflicts with the declared EXTERNPROTOs
  foreach (const OmExternProto *declaration, OmProtoManager::instance()->externProto()) {
    // the URL might differ, but they might point to the same object (ex: one is omnisim://, the other absolute)
    if (declaration->name() != protoName || OmUrl::resolveUrl(declaration->url()) == OmUrl::resolveUrl(url))
      continue;

    return true;
  }

  return false;
}

void OmAddNodeDialog::checkAndAddSelectedItem() {
  if (!mAddButton->isEnabled())
    return;

  accept();
}

void OmAddNodeDialog::accept() {
  if (mNewNodeType != PROTO) {
    QDialog::accept();
    return;
  }

  const QList<OmExternProto *> &clipboardBuffer = OmProtoManager::instance()->externProtoClipboardBuffer();
  const QString protoName =
    QUrl(mTree->selectedItems().at(0)->text(FILE_NAME)).fileName().replace(".proto", "", Qt::CaseInsensitive);

  bool conflict = false;
  foreach (const OmExternProto *proto, clipboardBuffer) {
    if (proto && proto->name() == protoName && !mRetrievalTriggered) {
      conflict = true;
      break;
    }
  }

  if (conflict) {
    const QMessageBox::StandardButton clipboardBufferWarningDialog = OmMessageBox::warning(
      "One or more PROTO nodes with the same name as the one you are about to insert is contained in the clipboard. Do "
      "you want to continue? This operation will clear the clipboard.",
      this, "Warning", QMessageBox::Cancel, QMessageBox::Ok | QMessageBox::Cancel);

    if (clipboardBufferWarningDialog == QMessageBox::Ok) {
      OmProtoManager::instance()->clearExternProtoClipboardBuffer();
      OmClipboard::instance()->clear();
    } else
      return;
  }

  // Before inserting a PROTO, it is necessary to ensure it is available locally (both itself and all the sub-proto it depends
  // on). This is not typically the case, so it must be assumed that nothing is available (the root proto might be available,
  // but not necessarily all its subs, or vice-versa); then trigger the cascaded download and only when the retriever gives
  // the go ahead the dialog's accept method can actually be executed entirely. In short, two passes are unavoidable for any
  // inserted proto.
  if (!mRetrievalTriggered) {
    mSelectionPath = mTree->selectedItems().at(0)->text(FILE_NAME);  // selection may change during download, store it
    connect(OmProtoManager::instance(), &OmProtoManager::retrievalCompleted, this, &OmAddNodeDialog::accept);
    mRetrievalTriggered = true;  // the second time the accept function is called, no retrieval should occur
    OmProtoManager::instance()->retrieveExternProto(mSelectionPath);
    return;
  }

  // this point should only be reached after the retrieval and therefore from this point the PROTO must be available locally
  if (OmUrl::isWeb(mSelectionPath) && !OmNetwork::instance()->isCachedWithMapUpdate(mSelectionPath)) {
    OmLog::error(tr("Retrieval of PROTO '%1' was unsuccessful, the asset should be cached but it is not.")
                   .arg(QUrl(mSelectionPath).fileName()));
    QDialog::reject();
    return;
  }

  // the insertion must be declared as EXTERNPROTO so that it is added to the world file when saving
  OmProtoManager::instance()->declareExternProto(QUrl(mSelectionPath).fileName().replace(".proto", "", Qt::CaseInsensitive),
                                                 mSelectionPath, false);

  QDialog::accept();
}
