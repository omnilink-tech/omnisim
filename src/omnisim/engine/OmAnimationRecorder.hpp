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

#ifndef OM_ANIMATION_RECORDER_HPP
#define OM_ANIMATION_RECORDER_HPP

// LEGACY: this in-process animation/movie recorder is the GUI-driven path
// inherited from upstream Webots. For new automation prefer the OmniSim
// HTTP capture service in scripts/capture/ — it produces deterministic
// renders from a shot list and supports 4K/8K independent of GUI viewport.
// Kept for the GUI's "Animation > Record" workflow and existing call sites.

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QSet>

#include "OmField.hpp"
#include "OmRotation.hpp"
#include "OmVector3.hpp"

class QFile;

class OmField;
class OmNode;

class OmAnimationCommand : public QObject {
  Q_OBJECT

public:
  OmAnimationCommand(const OmNode *n, const QStringList &fields, bool saveInitialValue);

  const OmNode *node() const { return mNode; }
  QList<QString> allFields() const { return mFields.keys(); }
  QList<QString> dirtyFields() const { return mChangedFields.keys(); }
  OmField *field(const QString &field) const { return mFields[field]; }
  const QString sanitizeField(const OmField *field);

  // Keep track of initial state that will be written to the animation file if the command changes during the animation
  const QString &initialState() const { return mInitialState; }
  bool isChangedFromStart() const { return mChangedFromStart; }

  void resetChanges();

signals:
  void changed(OmAnimationCommand *command);

private:
  void markFieldDirty(const OmField *field) { mChangedFields[field->name()] = true; }

  const OmNode *mNode;
  QHash<QString, OmField *> mFields;
  QHash<QString, bool> mChangedFields;
  OmVector3 mLastTranslation;
  OmRotation mLastRotation;
  QString mInitialState;
  bool mChangedFromStart;

private slots:
  void updateValue();
};

class OmAnimationRecorder : public QObject {
  Q_OBJECT

public:
  static OmAnimationRecorder *instance();
  static bool isInstantiated() { return (cInstance != NULL); }

  void setStartFromGuiFlag(bool flag) { mStartedFromGui = flag; }
  void initFromStreamingServer();
  QString computeUpdateData(bool force = false);
  void cleanupFromStreamingServer();

signals:
  void animationStartStatusChanged(int status);
  void animationStopStatusChanged(int status);
  void initalizedFromStreamingServer();
  void cleanedUpFromStreamingServer();
  void requestOpenUrl(const QString &fileName, const QString &message, const QString &title);

public slots:
  void start(const QString &fileName);
  void stop();
  void propagateNodeAddition(OmNode *node);

private slots:
  void update();
  void updateCommandsAfterNodeDeletion(QObject *);
  void addChangedCommandToList(OmAnimationCommand *command);
  void addChangedLabelToList(const QString &label);

private:
  static OmAnimationRecorder *cInstance;
  static void cleanup();

  OmAnimationRecorder();
  virtual ~OmAnimationRecorder();

  void startRecording(const QString &targetFile);
  void stopRecording();

  void populateCommands();
  void cleanCommands();

  QString mResults;
  bool mIsRecording;
  bool mStartedFromGui;

  double mLastUpdateTime;
  double mStartTime;

  QString mAnimationFilename;
  QFile *mFile;
  bool mFirstFrame;
  bool mStreamingServer;

  QList<OmAnimationCommand *> mCommands;
  QList<OmAnimationCommand *> mChangedCommands;
  QList<QString> mChangedLabels;
  QSet<QString> mLabelsIds;
};

#endif
