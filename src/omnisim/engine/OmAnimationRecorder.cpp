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

#include "OmAnimationRecorder.hpp"

#include "OmGroup.hpp"
#include "OmLog.hpp"
#include "OmMFInt.hpp"
#include "OmRobot.hpp"
#include "OmSFRotation.hpp"
#include "OmSimulationState.hpp"
#include "OmSupervisorUtilities.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QMutableListIterator>
#include <QtCore/QRegularExpression>

// this function is used to round the pose position coordinates
#define ROUND(x, precision) (roundf((x) / precision) * precision)

OmAnimationCommand::OmAnimationCommand(const OmNode *n, const QStringList &fields, bool saveInitialValue) :
  mNode(n),
  mChangedFromStart(false) {
  QString state;
  for (int i = 0; i < fields.size(); ++i) {
    OmField *f = mNode->findField(fields[i], true);
    if (f) {
      connect(f, &OmField::valueChanged, this, &OmAnimationCommand::updateValue);
      connect(f, &OmField::valueChangedByOde, this, &OmAnimationCommand::updateValue);
      connect(f, &OmField::valueChangedByOmniSim, this, &OmAnimationCommand::updateValue);
      mFields[f->name()] = f;

      if (saveInitialValue) {
        const OmSFVector3 *sfVector3 = dynamic_cast<OmSFVector3 *>(f->value());
        const OmSFRotation *sfRotation = dynamic_cast<OmSFRotation *>(f->value());
        const OmSFString *sfString = dynamic_cast<OmSFString *>(f->value());
        const OmMFString *mfString = dynamic_cast<OmMFString *>(f->value());
        const OmMFInt *mfInt = dynamic_cast<OmMFInt *>(f->value());
        const QString &fieldName = f->name();
        if (!state.isEmpty())
          state += ",";
        state += "\"" + fieldName + "\":\"";
        if (sfVector3 && fieldName.compare("translation") == 0) {
          // special translation case
          state += QString("%1 %2 %3")
                     .arg(ROUND(sfVector3->x(), 0.0001))
                     .arg(ROUND(sfVector3->y(), 0.0001))
                     .arg(ROUND(sfVector3->z(), 0.0001));
          mLastTranslation =
            OmVector3(ROUND(sfVector3->x(), 0.001), ROUND(sfVector3->y(), 0.001), ROUND(sfVector3->z(), 0.001));
        } else if (sfRotation && fieldName.compare("rotation") == 0) {
          // special rotation case
          state += QString("%1 %2 %3 %4")
                     .arg(ROUND(sfRotation->x(), 0.0001))
                     .arg(ROUND(sfRotation->y(), 0.0001))
                     .arg(ROUND(sfRotation->z(), 0.0001))
                     .arg(ROUND(sfRotation->angle(), 0.0001));
          mLastRotation = OmRotation(ROUND(sfRotation->x(), 0.001), ROUND(sfRotation->y(), 0.001),
                                     ROUND(sfRotation->z(), 0.001), ROUND(sfRotation->angle(), 0.001));
        } else if (sfString && (fieldName.compare("name") == 0 || fieldName.compare("fogType") == 0))
          state += f->value()->toString();
        else if (mfInt && (fieldName.compare("coordIndex") == 0 || fieldName.compare("normalIndex") == 0 ||
                           fieldName.compare("texCoordIndex") == 0)) {
          const int size = mfInt->size();
          QString intArray = QString("[");

          for (int j = 0; j < size - 1; j++)
            intArray.append(QString("%1,").arg(mfInt->item(j)));

          if (size > 0)
            intArray.append(QString("%1").arg(mfInt->item(size - 1)));

          intArray.append("]");
          state += intArray;
        } else if (mfString and fieldName.compare("url") == 0) {
          QStringList urls = mfString->value();
          QString urlArray = urls.join("\",\"");
          urlArray.prepend("[\"");
          urlArray.append("\"]");
          state += urlArray;
        } else  // generic case
          state += f->value()->toString(OmPrecision::FLOAT_MAX);
        state += "\"";
      }
    }
  }

  if (!state.isEmpty())
    mInitialState = QString("{\"id\":%1,%2}").arg(n->uniqueId()).arg(state);
}

void OmAnimationCommand::resetChanges() {
  mChangedFields.clear();
}

void OmAnimationCommand::updateValue() {
  const OmField *f = dynamic_cast<OmField *>(sender());
  if (f) {
    markFieldDirty(f);
    emit changed(this);
  }
}

const QString OmAnimationCommand::sanitizeField(const OmField *field) {
  const OmSFVector3 *sfVector3 = dynamic_cast<OmSFVector3 *>(field->value());
  const OmSFRotation *sfRotation = dynamic_cast<OmSFRotation *>(field->value());
  const OmSFString *sfString = dynamic_cast<OmSFString *>(field->value());
  const OmMFString *mfString = dynamic_cast<OmMFString *>(field->value());
  const OmMFInt *mfInt = dynamic_cast<OmMFInt *>(field->value());

  if (sfVector3 && field->name().compare("translation") == 0) {
    // special translation case
    const OmVector3 translationRounded =
      OmVector3(ROUND(sfVector3->x(), 0.001), ROUND(sfVector3->y(), 0.001), ROUND(sfVector3->z(), 0.001));
    if (translationRounded != mLastTranslation) {
      mLastTranslation = translationRounded;
      mChangedFromStart = true;
      return QString("\"%1 %2 %3\"")
        .arg(ROUND(sfVector3->x(), 0.0001))
        .arg(ROUND(sfVector3->y(), 0.0001))
        .arg(ROUND(sfVector3->z(), 0.0001));
    }
  } else if (sfRotation && field->name().compare("rotation") == 0) {
    // special rotation case
    const OmRotation rotationRounded = OmRotation(ROUND(sfRotation->x(), 0.001), ROUND(sfRotation->y(), 0.001),
                                                  ROUND(sfRotation->z(), 0.001), ROUND(sfRotation->angle(), 0.001));
    if (rotationRounded != mLastRotation) {
      mLastRotation = rotationRounded;
      mChangedFromStart = true;
      return QString("\"%1 %2 %3 %4\"")
        .arg(ROUND(sfRotation->x(), 0.0001))
        .arg(ROUND(sfRotation->y(), 0.0001))
        .arg(ROUND(sfRotation->z(), 0.0001))
        .arg(ROUND(sfRotation->angle(), 0.0001));
    }
  } else if (sfString && (field->name().compare("name") == 0 || field->name().compare("fogType") == 0))
    return field->value()->toString();
  else if (mfInt && (field->name().compare("coordIndex") == 0 || field->name().compare("normalIndex") == 0 ||
                     field->name().compare("texCoordIndex") == 0)) {
    const int size = mfInt->size();
    QString intArray = QString("[");

    for (int i = 0; i < size - 1; i++)
      intArray.append(QString("%1,").arg(mfInt->item(i)));

    if (size > 0)
      intArray.append(QString("%1").arg(mfInt->item(size - 1)));

    intArray.append("]");
    return intArray;
  } else if (mfString and field->name().compare("url") == 0) {
    QStringList urls = mfString->value();
    QString urlArray = urls.join("\",\"");
    urlArray.prepend("[\"");
    urlArray.append("\"]");
    return urlArray;
  } else {
    // generic case
    mChangedFromStart = true;
    return QString("\"%1\"").arg(field->value()->toString(OmPrecision::FLOAT_MAX));
  }

  return "";
}

OmAnimationRecorder *OmAnimationRecorder::cInstance = NULL;

OmAnimationRecorder *OmAnimationRecorder::instance() {
  if (cInstance == NULL) {
    cInstance = new OmAnimationRecorder;
    qAddPostRoutine(OmAnimationRecorder::cleanup);
  }
  return cInstance;
}

void OmAnimationRecorder::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

OmAnimationRecorder::OmAnimationRecorder() :
  mIsRecording(false),
  mStartedFromGui(false),
  mLastUpdateTime(0.0),
  mStartTime(0.0),
  mFile(NULL),
  mFirstFrame(true),
  mStreamingServer(false) {
}

OmAnimationRecorder::~OmAnimationRecorder() {
  try {
    stopRecording();
  } catch (const QString &e) {
    OmLog::warning(tr("Error when stopping the HTML5 animation recording: '%1'").arg(e), true);
  }
}

void OmAnimationRecorder::initFromStreamingServer() {
  if (mFile)
    throw tr("HTML5 animation recorder is enabled.");

  if (mStreamingServer)
    throw tr("Streaming server already initialized.");

  emit initalizedFromStreamingServer();

  populateCommands();

  mStreamingServer = true;
}

void OmAnimationRecorder::cleanupFromStreamingServer() {
  emit cleanedUpFromStreamingServer();

  cleanCommands();

  mStreamingServer = false;
}

void OmAnimationRecorder::propagateNodeAddition(OmNode *node) {
  if (!mStreamingServer)
    return;

  populateCommands();
}

void OmAnimationRecorder::populateCommands() {
  cleanCommands();

  const OmWorld *world = OmWorld::instance();
  if (world) {
    QList<OmNode *> nodes = OmWorld::instance()->root()->subNodes(true);
    for (int i = 0; i < nodes.size(); ++i) {
      const OmNode *node = nodes.at(i);
      if (node->isUseNode())
        // skip updates for USE nodes
        // DEF/USE mechanism is handled in webots.min.js
        continue;
      const QStringList fields = node->fieldsToSynchronizeWithW3d();
      if (fields.size() > 0) {
        // cppcheck-suppress constVariablePointer
        OmAnimationCommand *command = new OmAnimationCommand(node, fields, !mStreamingServer);
        mCommands << command;
      }
    }

    const QList<OmRobot *> &robots = OmWorld::instance()->robots();
    foreach (const OmRobot *const robot, robots) {
      if (robot->supervisor()) {
        foreach (const QString &label, robot->supervisorUtilities()->labelsState())
          addChangedLabelToList(label);

        connect(robot->supervisorUtilities(), &OmSupervisorUtilities::labelChanged, this,
                &OmAnimationRecorder::addChangedLabelToList);
      }
    }
  }

  foreach (OmAnimationCommand *command, mCommands) {
    connect(command, &OmAnimationCommand::changed, this, &OmAnimationRecorder::addChangedCommandToList);
    // support node deletions
    connect(command->node(), &OmNode::destroyed, this, &OmAnimationRecorder::updateCommandsAfterNodeDeletion);
  }
}

void OmAnimationRecorder::cleanCommands() {
  foreach (OmAnimationCommand *command, mCommands) {
    disconnect(command, &OmAnimationCommand::changed, this, &OmAnimationRecorder::addChangedCommandToList);
    disconnect(command->node(), &OmNode::destroyed, this, &OmAnimationRecorder::updateCommandsAfterNodeDeletion);
    delete command;
  }
  mCommands.clear();
  mChangedCommands.clear();
  mChangedLabels.clear();
}

void OmAnimationRecorder::addChangedCommandToList(OmAnimationCommand *command) {
  if (!mChangedCommands.contains(command))
    mChangedCommands.append(command);
}

void OmAnimationRecorder::addChangedLabelToList(const QString &label) {
  if (!mChangedLabels.contains(label))
    mChangedLabels.append(label);
}

void OmAnimationRecorder::updateCommandsAfterNodeDeletion(QObject *node) {
  QMutableListIterator<OmAnimationCommand *> it(mCommands);
  while (it.hasNext()) {
    OmAnimationCommand *command = it.next();
    if (command->node() == node) {
      it.remove();
      mChangedCommands.removeAll(command);
      disconnect(command, &OmAnimationCommand::changed, this, &OmAnimationRecorder::addChangedCommandToList);
      disconnect(command->node(), &OmNode::destroyed, this, &OmAnimationRecorder::updateCommandsAfterNodeDeletion);
      delete command;
    }
  }
}

void OmAnimationRecorder::update() {
  double currentTime = OmSimulationState::instance()->time() - mStartTime;
  if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / OmWorld::instance()->worldInfo()->fps()) {
    const QString data = computeUpdateData();
    if (data.isEmpty())
      return;

    QTextStream out(mFile);

    if (!mFirstFrame)
      out << ",\n";

    out << data;

    mFirstFrame = false;
    mLastUpdateTime = currentTime;
  }
}

QString OmAnimationRecorder::computeUpdateData(bool force) {
  QString result;
  QTextStream out(&result);
  const double time = OmSimulationState::instance()->time() - mStartTime;
  out << "{\"time\":" << QString::number(time);
  if (mChangedCommands.size() == 0 && mChangedLabels.size() == 0) {
    out << "}";
    return result;
  }
  out << ",\"updates\":[";
  bool hasPreviousUpdate = false;
  foreach (OmAnimationCommand *command, mChangedCommands) {
    const QList<QString> keys = force ? command->allFields() : command->dirtyFields();
    if (keys.isEmpty())
      continue;
    QString nodeString = QString("{\"id\":%1").arg(command->node()->uniqueId());
    if (hasPreviousUpdate)
      nodeString.prepend(",");
    bool emptyUpdate = true;
    foreach (const QString &fieldName, keys) {
      const QString value = command->sanitizeField(command->field(fieldName));
      if (!value.isEmpty()) {
        nodeString.append(QString(",\"%1\":%2").arg(fieldName).arg(value));
        emptyUpdate = false;
      }
    }
    if (!emptyUpdate) {
      out << nodeString;
      out << "}";

      hasPreviousUpdate = true;
    }
    command->resetChanges();
  }
  out << "]";

  if (mChangedLabels.size() != 0) {
    out << ",\"labels\":[";
    foreach (const QString &label, mChangedLabels) {
      out << "{";
      out << label;
      mLabelsIds.insert(label.mid(5, label.indexOf("font") - 7));
      if (label == mChangedLabels.last())
        out << "}";
      else
        out << "},";
    }
    out << "]";
  }

  out << "}";

  mChangedCommands.clear();
  mChangedLabels.clear();

  return result;
}

void OmAnimationRecorder::startRecording(const QString &targetFile) {
  mStartTime = OmSimulationState::instance()->time();
  mFile = new QFile(targetFile);
  if (!mFile->open(QIODevice::WriteOnly))
    throw tr("Cannot open HTML5 animation file '%1'").arg(mFile->fileName());

  populateCommands();

  connect(OmSimulationState::instance(), &OmSimulationState::physicsStepEnded, this, &OmAnimationRecorder::update);

  mLastUpdateTime = -1;
  mIsRecording = true;
  mFirstFrame = true;

  OmLog::info(tr("Start HTML5 animation export\n"));
}

void OmAnimationRecorder::stop() {
  try {
    stopRecording();
  } catch (const QString &e) {
    OmLog::warning(tr("Error when stopping the HTML5 animation recording: '%1'").arg(e), true);
    emit animationStopStatusChanged(false);
  }
}

void OmAnimationRecorder::start(const QString &fileName) {
  const OmWorld *world = OmWorld::instance();
  connect(world, &OmWorld::destroyed, this, &OmAnimationRecorder::stop);

  mAnimationFilename = fileName;
  mAnimationFilename.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".json");

  try {
    const bool success = world->exportAsHtml(fileName, true);
    if (!success)
      throw tr("HTML5 export failed");

    startRecording(mAnimationFilename);

    emit animationStartStatusChanged(true);
  } catch (const QString &e) {
    OmLog::error(tr("Error when starting the HTML5 animation recording: '%1'").arg(e), true);
    emit animationStartStatusChanged(false);
  }
}

void OmAnimationRecorder::stopRecording() {
  disconnect(OmSimulationState::instance(), &OmSimulationState::physicsStepEnded, this, &OmAnimationRecorder::update);
  mIsRecording = false;
  if (!mFile)
    return;
  mFile->close();
  const OmWorld *const world = OmWorld::instance();
  if (!world) {  // the world is being reverted, aborting the animation and deleting the incomplete animation file
    mFile->remove();
    delete mFile;
    mFile = NULL;
    return;
  }
  // prepend header and initial state to the file containing updates
  mFile->open(QFile::ReadOnly | QFile::Text);
  const QByteArray updates = mFile->readAll();
  mFile->close();
  mFile->open(QFile::WriteOnly | QFile::Text);

  QTextStream out(mFile);
  out << "{\n";
  // write header
  const OmWorldInfo *const worldInfo = world->worldInfo();
  const double step = worldInfo->basicTimeStep() * ceil((1000.0 / worldInfo->fps()) / worldInfo->basicTimeStep());
  out << QString(" \"basicTimeStep\":%1,\n").arg(step);
  QList<OmAnimationCommand *> commandsChangedFromStart;
  // cppcheck-suppress constVariablePointer
  foreach (OmAnimationCommand *command, mCommands) {
    // store only ids of nodes that changed during the animation
    if (command->isChangedFromStart())
      commandsChangedFromStart << command;
  }
  out << " \"labelsIds\":\"";
  bool firstLabel = true;
  foreach (const QString &id, mLabelsIds) {
    // cppcheck-suppress knownConditionTrueFalse
    if (!firstLabel)
      out << ";";
    else
      firstLabel = false;
    out << id;
  }

  out << "\",\n";

  out << " \"frames\":[\n";
  // write initial state
  out << "{\"time\":0,\"updates\":[";
  if (commandsChangedFromStart.isEmpty()) {
    OmLog::info(tr("Error: No animation content is available because no changes occurred in the simulation. "
                   "If you just want a 3D environment file, consider exporting a scene instead."));
    return;
  }
  foreach (const OmAnimationCommand *command, commandsChangedFromStart) {
    // store only initial state of nodes that changed during the animation
    if (command != commandsChangedFromStart.first())
      out << ",";
    out << command->initialState();
  }

  cleanCommands();
  out << "]}";
  if (!updates.isEmpty()) {
    out << ",\n";
    out << updates;
  }
  out << "\n ]\n}\n";
  mFile->close();

  const QFileInfo fi(mFile->fileName());
  const QString fileName = fi.absolutePath() + "/" + fi.baseName() + ".html";
  OmLog::info(tr("HTML5 animation successfully exported in '%1'\n").arg(fileName));

  if (mStartedFromGui && !mStreamingServer)
    emit requestOpenUrl(fileName,
                        tr("The animation has been created:<br>%1<br><br>Do you want to view it locally now?<br><br>"
                           "Note: if your browser prevents local-file CORS requests, open the file via a local "
                           "web server (e.g. <code>python -m http.server</code> in the export directory).")
                          .arg(fileName),
                        tr("Make HTML5 Animation"));

  delete mFile;
  mFile = NULL;
}
