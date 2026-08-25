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

#ifndef OM_ADD_NODE_DIALOG_HPP
#define OM_ADD_NODE_DIALOG_HPP

//
// Description: Dialog that lets the user choose a new node or proto to add in the Scene Tree
//

#include <QtCore/QDir>
#include <QtWidgets/QDialog>

class OmField;
class OmNode;
class OmProtoIcon;

class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QRegularExpression;
class QTreeWidget;
class QTreeWidgetItem;

class OmAddNodeDialog : public QDialog {
  Q_OBJECT

public:
  explicit OmAddNodeDialog(OmNode *currentNode, OmField *field, int index, QWidget *parent = NULL);

  QString modelName() const;
  QString protoUrl() const;
  bool isUseNode() const { return mNewNodeType == USE; };
  OmNode *defNode() const;  // returns the closest DEF node above the insertion location which matches the chosen USE name and
                            // avoids infinite recursion

public slots:
  // cppcheck-suppress virtualCallInConstructor
  void accept() override;

private slots:
  void updateItemInfo();
  void checkAndAddSelectedItem();
  void buildTree();

private:
  enum NodeType { UNKNOWN, BASIC, USE, PROTO };
  enum ItemText { MODEL_NAME, FILE_NAME, BOUNDING_OBJECT_INFO };

  OmNode *mCurrentNode;
  OmField *mField;
  int mIndex;
  QTreeWidget *mTree;
  QTreeWidgetItem *mUsesItem;
  QLabel *mPixmapLabel;
  QLabel *mDocumentationLabel;
  QLabel *mLicenseLabel;
  QPlainTextEdit *mInfoText;
  QPushButton *mAddButton;
  QGroupBox *mNodeInfoGroupBox;
  QLineEdit *mFindLineEdit;
  NodeType mNewNodeType;
  QList<OmNode *> mDefNodes;
  int mDefNodeIndex;
  bool mHasRobotTopNode;

  QString mSelectionPath;
  bool mRetrievalTriggered;

  QMap<QString, QString> mUniqueLocalProto;

  int addProtosFromProtoList(QTreeWidgetItem *parentItem, int type, const QRegularExpression &regexp, bool regenerate);
  int addProtosFromDirectory(QTreeWidgetItem *parentItem, const QString &dirPath, const QRegularExpression &regexp,
                             const QDir &rootDirectory, bool recurse = true, bool inProtos = false);
  int addProtos(QTreeWidgetItem *parentItem, const QStringList &protoList, const QString &dirPath,
                const QRegularExpression &regexp, const QDir &rootDirectory);
  void showNodeInfo(const QString &nodeFileName, NodeType nodeType, int variant = -1, const QString &boundingObjectInfo = "");
  bool doFieldRestrictionsAllowNode(const OmNode *node) const;

  bool isDeclarationConflicting(const QString &protoName, const QString &url);

  void setPixmap(const QString &pixmapPath);

private slots:
  void updateIcon(const QString &path);
};

#endif
