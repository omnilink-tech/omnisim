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

#include "OmPerspective.hpp"

#include "OmApplicationInfo.hpp"
#include "OmLog.hpp"
#include "OmPerspectiveFileFormat.hpp"
#include "OmProject.hpp"
#include "OmSolid.hpp"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>

#include <cassert>

#ifdef _WIN32
#include <windows.h>
#endif

OmPerspective::OmPerspective(const QString &worldPath) :
  mMaximizedDockId(-1),
  mCentralWidgetVisible(true),
  mSelectedTab(-1),
  mOrthographicViewHeight(1.0) {
  const QFileInfo info(worldPath);
  mBaseName = info.absolutePath() + "/." + info.completeBaseName();
  mVersion = OmApplicationInfo::version();
}

OmPerspective::~OmPerspective() {
  clearRenderingDevicesPerspectiveList();
}

bool OmPerspective::readContent(QTextStream &in, bool reloading) {
  // only version later than v6 are currently supported
  if (mVersion.majorNumber() < 6)
    return false;

  const bool skipNodeIdsOptions = mVersion.majorNumber() < 2018;
  mConsolesSettings.clear();
  while (!in.atEnd()) {
    QString line(in.readLine());
    QTextStream ls(&line, QIODevice::ReadOnly);
    QString key;
    ls >> key;
    // ignore perspective for version 6 (wxWidgets): we are not compatible
    if (key == "perspectives:" && mVersion.majorNumber() > 6) {
      QByteArray hex;
      ls >> hex;
      mState = QByteArray::fromHex(hex);
    } else if (key == "simulationViewPerspectives:") {
      QByteArray hex;
      ls >> hex;
      mSimulationViewState = QByteArray::fromHex(hex);
    } else if (key == "sceneTreePerspectives:") {
      QByteArray hex;
      ls >> hex;
      mSceneTreeState = QByteArray::fromHex(hex);
    } else if (key == "minimizedPerspectives:") {
      QByteArray hex;
      ls >> hex;
      mMinimizedState = QByteArray::fromHex(hex);
    } else if (key == "maximizedDockId:")
      ls >> mMaximizedDockId;
    else if (key == "centralWidgetVisible:") {
      int i;
      ls >> i;
      mCentralWidgetVisible = i;
    } else if (key == "projectionMode:") {
      if (reloading)
        continue;
      ls >> mProjectionMode;
    } else if (key == "renderingMode:") {
      if (reloading)
        continue;
      ls >> mRenderingMode;
    } else if (key == "selectionDisabled:") {  // backward compatibility < R2020b
      if (reloading)
        continue;
      int i;
      ls >> i;
      mDisabledUserInteractionsMap[OmAction::DISABLE_SELECTION] = i;
    } else if (key == "viewpointLocked:") {  // backward compatibility < R2020b
      if (reloading)
        continue;
      int i;
      ls >> i;
      mDisabledUserInteractionsMap[OmAction::LOCK_VIEWPOINT] = i;
    } else if (key == "userInteractions:") {
      if (!mDisabledUserInteractionsMap.isEmpty() || reloading)
        continue;
      const QString s = line.right(line.length() - 17).trimmed();  // remove label
      QStringList actionNamesList;
      splitUniqueNameList(s, actionNamesList);
      foreach (const QString &name, actionNamesList)
        mDisabledUserInteractionsMap[getActionFromString(name)] = true;
    } else if (key == "orthographicViewHeight:") {
      double value;
      ls >> value;
      setOrthographicViewHeight(value);
    } else if (key == "textFiles:") {
      ls >> mSelectedTab;
      mFilesList.clear();
      const QDir dir(OmProject::current()->dir());
      const QRegularExpression rx("(\"[^\"]*\")");  // match string literals
      QRegularExpressionMatch match = rx.match(line);
      while (match.hasMatch()) {
        mFilesList.append(dir.absoluteFilePath(match.captured().remove("\"")));
        match = rx.match(line, match.capturedEnd());
      }
    } else if (key == "robotWindow:") {
      if (!mRobotWindowNodeNames.isEmpty() || skipNodeIdsOptions)
        continue;
      QString s = line.right(line.length() - 12).trimmed();  // remove label
      splitUniqueNameList(s, mRobotWindowNodeNames);
    } else if (key == "globalOptionalRendering:") {
      if (!mEnabledOptionalRenderingList.isEmpty() || reloading)
        continue;
      const QString s = line.right(line.length() - 24).trimmed();  // remove label
      splitUniqueNameList(s, mEnabledOptionalRenderingList);
    } else if (key == "centerOfMass:") {
      if (!mCenterOfMassNodeNames.isEmpty() || skipNodeIdsOptions)
        continue;
      QString s = line.right(line.length() - 13).trimmed();  // remove label
      splitUniqueNameList(s, mCenterOfMassNodeNames);
    } else if (key == "centerOfBuoyancy:") {
      if (!mCenterOfBuoyancyNodeNames.isEmpty() || skipNodeIdsOptions)
        continue;
      QString s = line.right(line.length() - 17).trimmed();  // remove label
      splitUniqueNameList(s, mCenterOfBuoyancyNodeNames);
    } else if (key == "supportPolygon:") {
      if (!mSupportPolygonNodeNames.isEmpty() || skipNodeIdsOptions)
        continue;
      QString s = line.right(line.length() - 15).trimmed();  // remove label
      splitUniqueNameList(s, mSupportPolygonNodeNames);
    } else if (key == "consoles:") {
      const QStringList s = line.right(line.length() - 10).trimmed().split(':');  // remove label
      assert(s.size() == 3);
      ConsoleSettings settings;
      settings.name = s[0];
      settings.enabledFilters = s[1].split(';');
      settings.enabledLevels = s[2].split(';');
      mConsolesSettings.append(settings);
    } else if (key == "renderingDevicePerspectives:") {
      if (skipNodeIdsOptions)
        continue;

      QString s = line.right(line.length() - 29).trimmed();  // remove label
      QStringList values = s.split(";");
      int count = values.size();
      if (count < 5)
        // invalid
        continue;

      QString deviceUniqueName = values.takeFirst();
      while (values.size() > 9)
        // handle case where a Solid name contains the character ';'
        deviceUniqueName += ";" + values.takeFirst();
      mRenderingDevicesPerspectiveList.insert(deviceUniqueName, values);
    } else
      OmLog::warning(QObject::tr("Unknown key in perspective file: %1 (ignored).").arg(key));
  }

  // Backward compatibility with < R2020b
  if (mConsolesSettings.isEmpty() && mVersion < OmVersion(2020, 1, 0))
    addDefaultConsole();

  return true;
}

void OmPerspective::addDefaultConsole() {
  ConsoleSettings settings;
  settings.name = "Console";
  settings.enabledFilters = QStringList() << "All";
  settings.enabledLevels = QStringList() << "All";
  mConsolesSettings.append(settings);
}

QString OmPerspective::fileName() const {
  return mBaseName + OmPerspectiveFileFormat::writeExtension();
}

QString OmPerspective::existingFileName() const {
  // DUAL-READ: prefer the current extension, fall back to the legacy sibling so
  // that a layout saved by an older release survives the upgrade.
  return OmPerspectiveFileFormat::resolveExisting(fileName());
}

bool OmPerspective::load(bool reloading) {
  // reset version
  mVersion = OmApplicationInfo::version();

  mRobotWindowNodeNames.clear();
  if (!reloading)
    mEnabledOptionalRenderingList.clear();
  mConsolesSettings.clear();
  addDefaultConsole();
  clearRenderingDevicesPerspectiveList();
  clearEnabledOptionalRenderings();

  QFile file(existingFileName());
  if (!file.open(QIODevice::ReadOnly))
    return false;

  QTextStream in(&file);
  if (in.atEnd())
    return false;

  const QString header(in.readLine());

  bool found = mVersion.fromString(header, "^OmniSim Project File version ", "$");
  if (!found)
    // accept legacy project files written before the OmniSim rebrand
    found = mVersion.fromString(header, "^Webots Project File version ", "$");
  if (!found || mVersion > OmApplicationInfo::version())
    // don't support forward compatibility
    return false;

  bool success = readContent(in, reloading);

  // make sure we explicitly close our input file
  file.close();

  return success;
}

bool OmPerspective::save() const {
  QFile outputFile(fileName());
  if (!outputFile.open(QIODevice::WriteOnly))
    return false;

  QTextStream out(&outputFile);
  out << "OmniSim Project File version " << OmApplicationInfo::version().toString(false) << "\n";
  assert(!mState.isEmpty());
  out << "perspectives: " << mState.toHex() << "\n";
  assert(!mSimulationViewState.isEmpty());
  out << "simulationViewPerspectives: " << mSimulationViewState.toHex() << "\n";
  assert(!mSceneTreeState.isEmpty());
  out << "sceneTreePerspectives: " << mSceneTreeState.toHex() << "\n";
  if (!mMinimizedState.isEmpty())
    out << "minimizedPerspectives: " << mMinimizedState.toHex() << "\n";
  out << "maximizedDockId: " << mMaximizedDockId << "\n";
  out << "centralWidgetVisible: " << (int)mCentralWidgetVisible << "\n";
  if (!mProjectionMode.isEmpty())
    out << "projectionMode: " << mProjectionMode << "\n";
  if (!mRenderingMode.isEmpty())
    out << "renderingMode: " << mRenderingMode << "\n";

  // save disabled user interaction options
  QStringList userInteractionList;
  QList<OmAction::OmActionKind> actions(mDisabledUserInteractionsMap.keys());
  foreach (OmAction::OmActionKind action, actions) {
    if (mDisabledUserInteractionsMap.value(action))
      userInteractionList << getActionName(action);
  }
  if (!userInteractionList.isEmpty())
    out << "userInteractions: " << joinUniqueNameList(userInteractionList) << "\n";

  out << "orthographicViewHeight: " << (double)mOrthographicViewHeight << "\n";
  out << "textFiles: " << mSelectedTab;
  // convert to relative paths and save
  const QDir dir(OmProject::current()->dir());
  foreach (const QString &file, mFilesList)
    out << " \"" << dir.relativeFilePath(file) << "\"";
  out << "\n";
  if (!mRobotWindowNodeNames.isEmpty())
    out << "robotWindow: " << joinUniqueNameList(mRobotWindowNodeNames) << "\n";
  if (!mEnabledOptionalRenderingList.isEmpty())
    out << "globalOptionalRendering: " << joinUniqueNameList(mEnabledOptionalRenderingList) << "\n";
  if (!mCenterOfMassNodeNames.isEmpty())
    out << "centerOfMass: " << joinUniqueNameList(mCenterOfMassNodeNames) << "\n";
  if (!mCenterOfBuoyancyNodeNames.isEmpty())
    out << "centerOfBuoyancy: " << joinUniqueNameList(mCenterOfBuoyancyNodeNames) << "\n";
  if (!mSupportPolygonNodeNames.isEmpty())
    out << "supportPolygon: " << joinUniqueNameList(mSupportPolygonNodeNames) << "\n";

  for (int i = 0; i < mConsolesSettings.size(); ++i)
    out << "consoles: " << mConsolesSettings.at(i).name << ":" << mConsolesSettings.at(i).enabledFilters.join(";") << ":"
        << mConsolesSettings.at(i).enabledLevels.join(";") << "\n";

  QMap<QString, QStringList>::const_iterator it;
  for (it = mRenderingDevicesPerspectiveList.constBegin(); it != mRenderingDevicesPerspectiveList.constEnd(); ++it)
    out << "renderingDevicePerspectives: " << it.key() << ";" << it.value().join(";") << "\n";

  outputFile.close();

#ifdef _WIN32
  // set the hidden attribute on the perspective file we just wrote
  const QByteArray nativePathByteArray = QDir::toNativeSeparators(fileName()).toUtf8();
  const LPCSTR nativePath = nativePathByteArray.constData();
  SetFileAttributes(nativePath, GetFileAttributes(nativePath) | FILE_ATTRIBUTE_HIDDEN);
#endif

  return true;
}

void OmPerspective::setSimulationViewState(const QList<QByteArray> &state) {
  assert(state.size() == 2);
  mSimulationViewState = state[0];
  mSceneTreeState = state[1];
}

QList<QByteArray> OmPerspective::simulationViewState() const {
  QList<QByteArray> state;
  state << mSimulationViewState << mSceneTreeState;
  return state;
}

void OmPerspective::enableGlobalOptionalRendering(const QString &optionalRenderingName, bool enable) {
  if (!enable)
    mEnabledOptionalRenderingList.removeAll(optionalRenderingName);
  else if (!mEnabledOptionalRenderingList.contains(optionalRenderingName))
    mEnabledOptionalRenderingList.append(optionalRenderingName);
}

void OmPerspective::clearEnabledOptionalRenderings() {
  mCenterOfMassNodeNames.clear();
  mCenterOfBuoyancyNodeNames.clear();
  mSupportPolygonNodeNames.clear();
}

void OmPerspective::setRenderingDevicePerspective(const QString &deviceUniqueName, const QStringList &perspective) {
  QStringList value(perspective);
  if (deviceUniqueName.contains(";") && value.size() < 9) {
    assert(value.size() == 4);
    // in order to correctly retrieve the device unique name at load we have to add the external window properties
    value << "0"
          << "0"
          << "0"
          << "0"
          << "0";
  }
  mRenderingDevicesPerspectiveList.insert(deviceUniqueName, value);
}

QStringList OmPerspective::renderingDevicePerspective(const QString &deviceUniqueName) const {
  return mRenderingDevicesPerspectiveList.value(deviceUniqueName);
}

void OmPerspective::clearRenderingDevicesPerspectiveList() {
  mRenderingDevicesPerspectiveList.clear();
}

QString OmPerspective::joinUniqueNameList(const QStringList &nameList) {
  return nameList.join("::");
}

void OmPerspective::splitUniqueNameList(const QString &text, QStringList &targetList) {
  targetList.clear();
  if (text.isEmpty())
    return;
  // extract solid unique names joined by '::'
  targetList = OmSolid::splitUniqueNamesByEscapedPattern(text, "::");
}

QString OmPerspective::getActionName(OmAction::OmActionKind action) {
  switch (action) {
    case OmAction::DISABLE_SELECTION:
      return "selectionDisabled";
    case OmAction::LOCK_VIEWPOINT:
      return "viewpointLocked";
    case OmAction::DISABLE_3D_VIEW_CONTEXT_MENU:
      return "3dContextMenuDisabled";
    case OmAction::DISABLE_OBJECT_MOVE:
      return "objectMoveDisabled";
    case OmAction::DISABLE_FORCE_AND_TORQUE:
      return "forceAndTorqueDisabled";
    case OmAction::DISABLE_RENDERING:
      return "renderingDisabled";
    default:
      return QString();
  }
}

OmAction::OmActionKind OmPerspective::getActionFromString(const QString &actionString) {
  if (actionString == "selectionDisabled")
    return OmAction::DISABLE_SELECTION;
  if (actionString == "viewpointLocked")
    return OmAction::LOCK_VIEWPOINT;
  if (actionString == "3dContextMenuDisabled")
    return OmAction::DISABLE_3D_VIEW_CONTEXT_MENU;
  if (actionString == "objectMoveDisabled")
    return OmAction::DISABLE_OBJECT_MOVE;
  if (actionString == "forceAndTorqueDisabled")
    return OmAction::DISABLE_FORCE_AND_TORQUE;
  if (actionString == "renderingDisabled")
    return OmAction::DISABLE_RENDERING;

  assert(false);
  return OmAction::NACTIONS;
}
