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

#ifndef OM_PROTO_TREE_ITEM_HPP
#define OM_PROTO_TREE_ITEM_HPP

//
// Description: a class representing an EXTERNPROTO declaration tree
//

class OmDownloader;
class OmProtoTreeItem;

#include <QtCore/QMap>
#include <QtCore/QObject>

class OmProtoTreeItem : public QObject {
  Q_OBJECT
public:
  OmProtoTreeItem(const QString &url, OmProtoTreeItem *parent, bool importable);
  ~OmProtoTreeItem();

  const QString &name() const { return mName; }
  const QString &url() const { return mUrl; }
  const QStringList &error() const { return mError; }
  const OmProtoTreeItem *parent() const { return mParent; }
  const QList<OmProtoTreeItem *> &children() const { return mChildren; }
  bool isImportable() const { return mImportable; }
  void setImportable(bool value) { mImportable = value; }

  void download();
  void insert(const QString &url);  // inserts in the sub-proto list of the node its being called on

  void generateSessionProtoList(QStringList &sessionList);

signals:
  void finished();

protected slots:
  void downloadUpdate();

private:
  QString mUrl;
  OmProtoTreeItem *mParent;
  bool mImportable;
  bool mIsReady;
  OmDownloader *mDownloader;
  QString mName;
  QStringList mError;
  OmProtoTreeItem *mRoot;

  bool isReady() const { return mIsReady; }
  void parseItem();
  void readyCheck();
  bool isRecursiveProto(const QString &protoUrl);
  void recursiveErrorAccumulator(QStringList &list);

  void deleteChild(const OmProtoTreeItem *child);

  QList<OmProtoTreeItem *> mChildren;  // list of referenced sub-proto
};

#endif
