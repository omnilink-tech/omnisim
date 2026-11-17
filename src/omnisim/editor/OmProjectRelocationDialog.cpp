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

#include "OmProjectRelocationDialog.hpp"
#include "OmPerspectiveFileFormat.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmFileUtil.hpp"
#include "OmLanguage.hpp"
#include "OmLineEdit.hpp"
#include "OmLog.hpp"
#include "OmMessageBox.hpp"
#include "OmNetwork.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRobot.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmWorld.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <QtCore/QRegularExpression>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>

QString OmProjectRelocationDialog::mExternalProtoProjectPath = QString();

OmProjectRelocationDialog::OmProjectRelocationDialog(OmProject *project, const QString &relativeFilename,
                                                     const QString &absoluteFilePath, QWidget *parent) :
  QDialog(parent),
  mProject(project),
  mRelativeFilename(relativeFilename),
  mAbsoluteFilePath(absoluteFilePath),
  mSourceEdit(NULL),
  mTargetEdit(NULL),
  mFilesLabel(NULL),
  mConclusionLabel(NULL),
  mStatusEdit(NULL),
  mSelectButton(NULL),
  mCancelButton(NULL),
  mCopyButton(NULL),
  mButtonBox(NULL) {
  setWindowTitle(tr("Project relocation"));

  mButtonBox = new QDialogButtonBox(this);
  mCancelButton = mButtonBox->addButton(QDialogButtonBox::Cancel);
  mCopyButton = mButtonBox->addButton(tr("Copy"), QDialogButtonBox::AcceptRole);
  connect(mCancelButton, &QPushButton::pressed, this, &OmProjectRelocationDialog::reject);
  connect(mCopyButton, &QPushButton::pressed, this, &OmProjectRelocationDialog::copy);

  mStatusEdit = new QPlainTextEdit(this);
  mStatusEdit->setReadOnly(true);

  mIsCompleteRelocation = mProject->isReadOnly();

  if (mIsCompleteRelocation)
    initCompleteRelocation();
  else
    initProtoSourceRelocation();
}

void OmProjectRelocationDialog::initCompleteRelocation() {
  // cppcheck-suppress constVariablePointer
  QLabel *title = new QLabel(tr("<b>Copy necessary files from source to target directory?</b>"), this);

  const QString &sourcePath = QDir::toNativeSeparators(mProject->path());
  mTargetPath = OmPreferences::instance()->value("Directories/projects").toString() + mProject->dirName();

  mSourceEdit = new OmLineEdit(sourcePath, this);
  mSourceEdit->setReadOnly(true);
  mSourceEdit->setMinimumWidth(sourcePath.length() * 8);

  mTargetEdit = new OmLineEdit(QDir::toNativeSeparators(mTargetPath), this);
#ifdef __APPLE__
  mTargetEdit->setMinimumWidth(mTargetPath.length() * 8);
#endif
  connect(mTargetEdit, &OmLineEdit::textEdited, this, &OmProjectRelocationDialog::targetEdited);

  mSelectButton = new QPushButton(tr("Select"), this);
  connect(mSelectButton, &QPushButton::pressed, this, &OmProjectRelocationDialog::selectDirectory);

  QHBoxLayout *layout = new QHBoxLayout();
  layout->addWidget(mTargetEdit);
  layout->addWidget(mSelectButton);

  QFormLayout *formLayout = new QFormLayout();
  formLayout->addRow(tr("Source directory: "), mSourceEdit);
  formLayout->addRow(tr("Target directory: "), layout);

  setStatus(tr("Please select a target directory then push the [Copy] button."));

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(title);
  mainLayout->addLayout(formLayout);
  mainLayout->addWidget(mStatusEdit);
  mainLayout->addWidget(mButtonBox);
}

void OmProjectRelocationDialog::initProtoSourceRelocation() {
  // cppcheck-suppress constVariablePointer
  QLabel *title = new QLabel(tr("<b>Copy necessary files from source to current project directory?</b>"), this);

  mTargetPath = mProject->path();

  mSourceEdit = new OmLineEdit(QDir::toNativeSeparators(mAbsoluteFilePath), this);
  mSourceEdit->setReadOnly(true);
  mSourceEdit->setMinimumWidth(mAbsoluteFilePath.length() * 8);

  mTargetEdit = new OmLineEdit(QDir::toNativeSeparators(mTargetPath), this);
  mTargetEdit->setReadOnly(true);
  mTargetEdit->setMinimumWidth(mTargetPath.length() * 8);

  QFormLayout *formLayout = new QFormLayout();
  formLayout->addRow(tr("Source directory: "), mSourceEdit);
  formLayout->addRow(tr("Target directory: "), mTargetEdit);

  setStatus(tr("Please select the copy options then push the [Copy] button."));

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(title);
  mainLayout->addLayout(formLayout);
  mainLayout->addWidget(mStatusEdit);
  mainLayout->addWidget(mButtonBox);
}

OmProjectRelocationDialog::~OmProjectRelocationDialog() {
}

void OmProjectRelocationDialog::setStatus(const QString &text, bool ok) {
  QTextCharFormat format;

  if (ok)
    format.setForeground(QBrush(Qt::black));
  else
    format.setForeground(QBrush(Qt::red));

  mStatusEdit->setCurrentCharFormat(format);
  mStatusEdit->setPlainText(text);

#ifdef __APPLE__
  // Qt bug on macOS: the QDialog::exec() doesn't deal correctly the repaint events as it should.
  repaint();
#endif
}

void OmProjectRelocationDialog::targetEdited(const QString &text) {
  mTargetPath = QDir::fromNativeSeparators(text);
  setStatus(tr("Push the [Copy] button to copy the project."));
}

void OmProjectRelocationDialog::copy() {
  const QString &home = OmStandardPaths::omniSimHomePath();
  if (OmFileUtil::isLocatedInDirectory(mTargetPath, home)) {
    setStatus(tr("Target directory is located in the OmniSim installation directory.") + '\n' +
                tr("Please select another target directory."),
              false);
    return;
  }

  QDir dir(mTargetPath);
  if (mIsCompleteRelocation) {
    if (dir.exists()) {
      if (dir.count() > 2) {  // 2 because special cases: "." and ".." are counted
        setStatus(tr("Target directory is not empty.") + '\n' + tr("Please select another directory."), false);
        return;
      }
    } else {
      if (!QDir::root().mkpath(mTargetPath)) {
        setStatus(
          tr("Could not create target directory.") + '\n' + tr("Make sure that you have write access at this location."),
          false);
        return;
      }
    }
  }

  int copiedFilesCount = 0;
  if (mIsCompleteRelocation) {
    copiedFilesCount += copyWorldFiles();
    const QString currentProjectPath(QDir(mProject->path()).absolutePath() + '/');
    copiedFilesCount += copyProject(currentProjectPath);
  }
  if (!mExternalProtoProjectPath.isEmpty())
    copiedFilesCount += copyProject(mExternalProtoProjectPath);

  if (copiedFilesCount == 0) {
    setStatus(tr("Project relocation failed.") + '\n' + tr("Some files or directories could not be copied."));
    return;
  }

  // change buttons
  if (mSelectButton)
    mSelectButton->setEnabled(false);
  mSourceEdit->setEnabled(false);
  mTargetEdit->setEnabled(false);
  mButtonBox->removeButton(mCancelButton);
  mButtonBox->removeButton(mCopyButton);
  QPushButton *closeButton = mButtonBox->addButton(QDialogButtonBox::Close);
  closeButton->setFocus();
  connect(closeButton, &QPushButton::pressed, this, &OmProjectRelocationDialog::accept);

  mProject->setPath(dir.path());

  // store the accepted project directory in the preferences
  // store the upper level, probably the path where the directories are stored
  if (!dir.cdUp())
    assert(false);
  OmPreferences::instance()->setValue("Directories/projects", dir.absolutePath() + '/');

  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (OmRobot *robot, robots)
    robot->updateControllerDir();

  // good news
  setStatus(tr("Project successfully relocated.") + '\n' + tr("%1 file(s) copied.").arg(copiedFilesCount));
}

int OmProjectRelocationDialog::copyProject(const QString &projectPath) {
  int result = 0;

  // copy all local resources
  result += OmFileUtil::copyDir(projectPath + "protos", mTargetPath + "/protos", true, false, true);
  result += OmFileUtil::copyDir(projectPath + "motions", mTargetPath + "/motions", true, false, true);
  result += OmFileUtil::copyDir(projectPath + "plugins", mTargetPath + "/plugins", true, false, true);
  result += OmFileUtil::copyDir(projectPath + "libraries", mTargetPath + "/libraries", true, false, true);

  // copy only the needed robot controllers
  QStringList copiedControllers;
  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (OmRobot *robot, robots) {
    const QString &controllerName = robot->controllerName();
    const QString destinationPath = mTargetPath + "/controllers/" + controllerName;
    if (controllerName.isEmpty())
      continue;
    if (controllerName.front() == '<' && controllerName.back() == '>')  // <none>, <generic> or <extern>
      continue;
    if (!copiedControllers.contains(controllerName) && !QDir(destinationPath).exists()) {
      const QString &controllerPath = robot->controllerDir();
      result += OmFileUtil::copyDir(controllerPath, destinationPath, true, false, true);
      copiedControllers << controllerName;
    }
  }

  // copy the current source folder
  QString relativeDirPath = mRelativeFilename;
  if (QFileInfo(projectPath + relativeDirPath).isFile())
    // if it's not a directory, but a file, get the containing controller directory from after the "controllers/", "libraries/",
    // "protos/", etc. part of the string
    relativeDirPath = mRelativeFilename.left(mRelativeFilename.indexOf('/', mRelativeFilename.indexOf('/') + 1));
  const QString destinationPath = mTargetPath + '/' + relativeDirPath;
  if (!QDir(destinationPath).exists())
    result += OmFileUtil::copyDir(projectPath + relativeDirPath, destinationPath, true, true, true);

  return result;
}

int OmProjectRelocationDialog::copyWorldFiles() {
  // Files to be copied:
  // 1) the current world file and its associated perspective file
  // 2) all the textures used explicitly by this world file and present in the local worlds
  //    folder (but not the ones used implicitly by PROTOs).

  const OmWorld *world = OmWorld::instance();
  const QString &worldFileBaseName = QFileInfo(world->fileName()).baseName();

  int result = 0;

  // copy the world file and its hidden sibling perspective file
  QDir targetPathDir(mTargetPath + "/worlds");
  targetPathDir.mkpath(".");
  // The source may carry either world extension; relocate it under its own name
  // rather than forcing a rename mid-copy.
  const QString sourceWorld = OmWorldFileFormat::resolveExisting(mProject->worldsPath() + worldFileBaseName +
                                                                 OmWorldFileFormat::writeExtension());
  mTargetWorld = mTargetPath + "/worlds/" + QFileInfo(sourceWorld).fileName();
  if (QFile::copy(sourceWorld, mTargetWorld)) {
    QFile::setPermissions(mTargetWorld, QFile::permissions(mTargetWorld) | QFile::WriteOwner | QFile::WriteUser);
    result++;
  }
  // The perspective is a HIDDEN sibling -- note the leading '.' before the world
  // stem -- and it may carry either extension; relocate whichever one exists
  // under its own name rather than forcing a rename mid-copy.
  const QString sourceProjectFile = OmPerspectiveFileFormat::resolveExisting(
    mProject->worldsPath() + '.' + worldFileBaseName + OmPerspectiveFileFormat::writeExtension());
  const QString targetProjectFile(mTargetPath + "/worlds/" + QFileInfo(sourceProjectFile).fileName());
  if (QFile::copy(sourceProjectFile, targetProjectFile)) {
    QFile::setPermissions(targetProjectFile, QFile::permissions(targetProjectFile) | QFile::WriteOwner | QFile::WriteUser);
    result++;
  }

  // copy only the needed texture files
  const QList<std::pair<QString, OmMFString *>> textureList = world->listTextureFiles();
  for (int i = 0; i < textureList.size(); ++i) {
    const QString &textureFile = textureList[i].first;
    if (!QDir::isRelativePath(textureFile) || OmUrl::isWeb(textureFile))
      continue;

    const QString sourceTexturePath = QDir::cleanPath(mProject->worldsPath() + textureFile);
    if (!sourceTexturePath.startsWith(mTargetPath)) {  // it will be outside the project, so we need to copy it
      // create folder if it doesn't exist already
      const QDir texturesDirectory(mTargetPath + "/worlds/textures/");
      if (!texturesDirectory.exists())
        texturesDirectory.mkpath(".");

      // ensure there are no ambiguities
      const QString baseName = QFileInfo(sourceTexturePath).baseName();
      const QString extension = QFileInfo(sourceTexturePath).suffix();

      QFileInfo fi(texturesDirectory.absolutePath() + '/' + baseName + '.' + extension);
      int attempt = 0;
      while (attempt < 100 && (fi.exists() && !OmFileUtil::areIdenticalFiles(sourceTexturePath, fi.absoluteFilePath())))
        fi = QFileInfo(texturesDirectory.absolutePath() + '/' + baseName + QString::number(attempt++) + '.' + extension);

      if (attempt >= 100) {
        OmLog::error(QObject::tr("Impossible to create a non-ambiguous path for asset '%1'in the local directory.")
                       .arg(sourceTexturePath));
        return result;
      }

      // copy textures to the new location
      const QString relativePath = QDir(QFileInfo(mTargetWorld).absolutePath()).relativeFilePath(fi.absoluteFilePath());
      if (!fi.exists() && QFile::copy(sourceTexturePath, fi.absoluteFilePath()))
        result++;

      mFieldsToUpdate.insert(textureList[i].second, relativePath);
    }
  }

  // copy forests if the world files references any
  QFile file(world->fileName());
  if (file.open(QIODevice::ReadOnly)) {
    QRegularExpression re("\"([^\\.\"]+\\.forest)\"");
    QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

    QStringList forests;
    while (it.hasNext()) {
      const QRegularExpressionMatch match = it.next();
      if (match.hasMatch())
        forests << match.captured(1);
    }
    file.close();

    foreach (const QString &forest, forests) {
      const QFileInfo absolutePath = QFileInfo(QDir(OmProject::current()->worldsPath()).filePath(forest));
      const QFileInfo targetFile = QFileInfo(QDir(mTargetPath + "/worlds/").filePath(forest));
      QDir().mkpath(targetFile.absolutePath());  // create any necessary directories prior to copying the file
      if (QFile::copy(absolutePath.absoluteFilePath(), targetFile.absoluteFilePath()))
        result++;
      else
        setStatus(
          tr("Impossible to copy file '%1' to '%2'.").arg(absolutePath.absoluteFilePath()).arg(targetFile.absoluteFilePath()));
    }
  } else
    setStatus(tr("Impossible to read file '%1'").arg(world->fileName()));

  // copy SUMO net directory if any
  QString fileName = world->fileName();
  const QString netDir = OmWorldFileFormat::replaceExtension(fileName, "_net");
  if (QDir().exists(netDir))
    result += OmFileUtil::copyDir(netDir, mTargetPath + "/worlds/" + QFileInfo(netDir).baseName(), true, false, true);

  return result;
}

void OmProjectRelocationDialog::selectDirectory() {
  const QString &path = QFileDialog::getExistingDirectory(this, "Select a directory", mTargetEdit->text());

  if (path.isEmpty())
    return;

  mTargetEdit->setText(path);
  mTargetPath = QDir::fromNativeSeparators(path);
  setStatus(tr("Push the [Copy] button to copy the necessary project files."));
}

bool OmProjectRelocationDialog::validateLocation(QWidget *parent, QString &fileName) {
  mExternalProtoProjectPath.clear();

  if (OmFileUtil::isLocatedInDirectory(fileName, OmStandardPaths::cachedAssetsPath())) {
    OmMessageBox::warning(tr("You are trying to modify a remote file.") + "\n\n'" + tr("This operation is not permitted."),
                          parent);
    return false;
  }

  // if file is not in installation directory: it's ok
  if (!OmFileUtil::isLocatedInInstallationDirectory(fileName))
    return true;

  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  // use native separators in user dialog
  const QString &nativeFilename = QDir::toNativeSeparators(fileName);

  OmProject *current = OmProject::current();
  if (!OmFileUtil::isLocatedInDirectory(fileName, current->path()) ||
      OmFileUtil::isLocatedInDirectory(fileName, OmStandardPaths::resourcesPath())) {
    const QList<OmRobot *> &robots = OmWorld::instance()->robots();
    foreach (const OmRobot *robot, robots) {
      const OmProtoModel *proto = robot->proto();
      if (!proto)
        continue;

      QDir protoProjectDir(proto->projectPath());
      if (OmFileUtil::isLocatedInDirectory(fileName, protoProjectDir.absolutePath())) {
        mExternalProtoProjectPath = protoProjectDir.absolutePath() + '/';
        break;
      }
    }

    if (mExternalProtoProjectPath.isEmpty() && fileName != OmProject::newWorldPath()) {
      // file is not in current project
      OmMessageBox::warning(tr("You are trying to modify a file located in OmniSim installation directory:") + "\n\n'" +
                              nativeFilename + "'\n\n" + tr("This operation is not permitted."),
                            parent);
      simulationState->resumeSimulation();
      return false;
    }
  }

  // file is in current project
  if (OmMessageBox::question(
        tr("You are trying to modify a file located in the OmniSim installation directory:") + "\n\n'" + nativeFilename +
          "'\n\n" + tr("This operation is not permitted: would you like to copy the necessary files to another location?"),
        parent) == QMessageBox::Cancel) {
    simulationState->resumeSimulation();
    return false;
  }

  // change fileName parameter: get relative fileName with respect to previous project path
  QString absolutePath;
  if (mExternalProtoProjectPath.isEmpty())
    absolutePath = current->path();
  else
    absolutePath = mExternalProtoProjectPath;
  fileName = QDir(absolutePath).relativeFilePath(fileName) + (fileName.endsWith('/') ? "/" : "");

  // relocate dialog
  OmProjectRelocationDialog dialog(current, fileName, absolutePath, parent);
  dialog.exec();
  if (dialog.result() == QDialog::Rejected) {
    simulationState->resumeSimulation();
    return false;
  }

  // change fileName parameter: set absolute fileName with respect to the new project path
  fileName = QDir(dialog.targetPath()).absoluteFilePath(fileName);

  simulationState->resumeSimulation();
  return true;
}

void OmProjectRelocationDialog::accept() {
  QMapIterator<OmMFString *, QString> it(mFieldsToUpdate);
  while (it.hasNext()) {
    it.next();
    OmMFString *field = it.key();
    for (int i = 0; i < field->size(); ++i) {
      if (!field->item(i).isEmpty())
        field->setItem(i, it.value());
    }
  }
  mFieldsToUpdate.clear();

  OmProtoManager::instance()->updateCurrentWorld(mTargetWorld);
  OmProject::current()->setPath(mTargetPath);

  QDialog::accept();
}
