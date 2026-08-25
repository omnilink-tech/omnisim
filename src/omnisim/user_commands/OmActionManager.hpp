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

#ifndef OM_ACTION_MANAGER_HPP
#define OM_ACTION_MANAGER_HPP

//
// Description: Singleton class creating and storing the Qt actions
//

#include "OmAction.hpp"

#include <QtCore/QHash>
#include <QtCore/QObject>

class QAction;

class OmActionManager : public QObject {
  Q_OBJECT

public:
  static OmActionManager *instance();

  QAction *action(OmAction::OmActionKind kind);

  void setEnabled(OmAction::OmActionKind kind, bool enabled);

  void resetApplicationActionsState();
  void enableTextEditActions(bool enabled, bool isReadOnly);
  QObject *focusObject() const { return mFocusObject; }
  void setFocusObject(QObject *object) { mFocusObject = object; }

  static void setActionEnabledSilently(QAction *action, bool enabled);
  static const QString mapControlKey();

  void updateRenderingButton();

signals:
  void userConsoleEditCommandReceived(OmAction::OmActionKind action);
  void userDocumentationEditCommandReceived(OmAction::OmActionKind action);
  void userWorldEditCommandReceived(OmAction::OmActionKind action);
  void transformRequested(QString newModelName);

public slots:
  void forwardTransformToActionToSceneTree();

private slots:
  void updateEnabled();
  void dispatchUserCommand();

private:
  static void cleanup();

  OmActionManager();
  virtual ~OmActionManager();

  void populateActions();
  void connectActions();

  static OmActionManager *cInstance;

  QHash<OmAction::OmActionKind, QAction *> mActions;
  QObject *mFocusObject;
};

#endif
