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

#include "OmExtendedStringEditor.hpp"

#include "OmCadShape.hpp"
#include "OmControllerPlugin.hpp"
#include "OmField.hpp"
#include "OmFieldLineEdit.hpp"
#include "OmFileUtil.hpp"
#include "OmLanguage.hpp"
#include "OmMesh.hpp"
#include "OmMessageBox.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProject.hpp"
#include "OmProtoModel.hpp"
#include "OmSolid.hpp"
#include "OmSolidReference.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmWorld.hpp"

#include <QtCore/QEvent>
#include <QtCore/QStringList>
#include <QtWidgets/QAbstractItemView>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QPushButton>

#include <cassert>

const QDir::Filters FILTERS = QDir::Dirs | QDir::NoDotAndDotDot;

// This input dialog accepts the user selection if
// any QAbstractItemView child (e.g. string list)
// receives a double click.
class DoubleClickInputDialog : public QInputDialog {
public:
  explicit DoubleClickInputDialog(QWidget *parent) : QInputDialog(parent) {}

protected:
  void childEvent(QChildEvent *event) override {
    QAbstractItemView *itemView;
    if (event->type() != QEvent::ChildRemoved && (itemView = dynamic_cast<QAbstractItemView *>(event->child())))
      connect(itemView, &QAbstractItemView::doubleClicked, this, &QInputDialog::accept, Qt::UniqueConnection);
  }
};

const QStringList OmExtendedStringEditor::ITEM_LIST_INFO[N_STRING_TYPE_INFO] = {
  QStringList() << "controllers/" << tr("Controller choice")
                << tr("Please select a controller from the list\n(it will start at the next time step)"),
  QStringList() << "" << tr("Fluid choice") << tr("Please select a fluid from the list\n"),
  QStringList() << "" << tr("Reference area choice") << tr("Please select a reference area from the list\n"),
  QStringList()
    << "plugins/remote_controls/" << tr("Remote control plugins choice")
    << tr("Please select a remote control plugin from the list\n(takes effect only after you save and reload the world)"),
  QStringList() << "plugins/robot_windows/" << tr("Robot window plugins choice")
                << tr("Please select a robot window plugin from the list\n(takes effect only after you reset the "
                      "simulation\nand show the new robot window)"),
  QStringList() << "" << tr("Solid choice") << tr("Please select a solid from the list\n")};

const QStringList OmExtendedStringEditor::REFERENCE_AREA_ITEMS = QStringList() << "immersed area"
                                                                               << "xyz-projection";

OmExtendedStringEditor::OmExtendedStringEditor(QWidget *parent) : OmStringEditor(parent) {
  mEditButton = new QPushButton(tr("Edit"), this);
  mEditButton->setStatusTip(tr("Edit controller main file in Text Editor."));
  mEditButton->setToolTip(mEditButton->statusTip());
  connect(mEditButton, &QPushButton::pressed, this, &OmExtendedStringEditor::editInTextEditor);

  mSelectButton = new QPushButton(tr("Select..."), this);
  mSelectButton->setStatusTip(tr("Select controller program."));
  mSelectButton->setToolTip(mSelectButton->statusTip());
  connect(mSelectButton, &QPushButton::pressed, this, &OmExtendedStringEditor::select);
  QHBoxLayout *horizontalLayout = new QHBoxLayout();
  horizontalLayout->addStretch(1);
  horizontalLayout->addWidget(mSelectButton);
  horizontalLayout->addWidget(mEditButton);
  horizontalLayout->addStretch(1);

  mStringType = REGULAR;

  mLayout->addLayout(horizontalLayout, 2, 1);
}

OmExtendedStringEditor::~OmExtendedStringEditor() {
}

void OmExtendedStringEditor::recursiveBlockSignals(bool block) {
  OmStringEditor::recursiveBlockSignals(block);
  mEditButton->blockSignals(block);
  mSelectButton->blockSignals(block);
}

const QStringList &OmExtendedStringEditor::defaultControllersEntryList() const {
  static bool firstCall = true;
  if (firstCall) {
    QDir defaultDir(OmStandardPaths::projectsPath() + "default/controllers");
    QDir resourcesDir(OmStandardPaths::resourcesControllersPath());
    mDefaultControllersEntryList << defaultDir.entryList(FILTERS) << resourcesDir.entryList(FILTERS);
    foreach (const OmProject *extraProject, *OmProject::extraProjects())
      mDefaultControllersEntryList << QDir(extraProject->controllersPath()).entryList(FILTERS);
    mDefaultControllersEntryList.replaceInStrings("generic", "<generic>");
    firstCall = false;
  }
  return mDefaultControllersEntryList;
}

const QStringList &OmExtendedStringEditor::defaultRobotWindowsPluginsEntryList() const {
  static bool firstCall = true;
  if (firstCall) {
    const QStringList &list = OmControllerPlugin::defaultList(OmControllerPlugin::ROBOT_WINDOW);
    foreach (const QString &item, list) {
      QFileInfo fi(item);
      QString name = fi.dir().dirName();
      mDefaultRobotWindowPluginsEntryList << ((name != "generic") ? name : "<generic>");
    }
    firstCall = false;
  }
  return mDefaultRobotWindowPluginsEntryList;
}

const QStringList &OmExtendedStringEditor::defaultRemoteControlsPluginsEntryList() const {
  static bool firstCall = true;
  if (firstCall) {
    const QStringList &list = OmControllerPlugin::defaultList(OmControllerPlugin::REMOTE_CONTROL);
    foreach (const QString &item, list) {
      QFileInfo fi(item);
      QString name = fi.dir().dirName();
      mDefaultRemoteControlPluginsEntryList << name;
    }
    firstCall = false;
  }
  return mDefaultRemoteControlPluginsEntryList;
}

const QStringList &OmExtendedStringEditor::defaultEntryList() const {
  switch (mStringType) {
    case CONTROLLER:
      return defaultControllersEntryList();
    case ROBOT_WINDOW_PLUGIN:
      return defaultRobotWindowsPluginsEntryList();
    case REMOTE_CONTROL_PLUGIN:
      return defaultRemoteControlsPluginsEntryList();
    default:
      assert(false);
      static const QStringList EMPTY_STRING_LIST;
      return EMPTY_STRING_LIST;
  }
}

void OmExtendedStringEditor::editInTextEditor() {
  if (stringValue().isEmpty()) {
    OmMessageBox::warning(tr("Impossible to edit the file: no file defined."), this);
    return;
  }

  enum { projectFile, extraProjectFile, protoFile, externalProtoFile, resourcesFile, omniSimProjectsFile, noFile };
  int dirLocation = noFile;
  const QString &fileType = ITEM_LIST_INFO[mStringType].at(0);

  // Filters
  QStringList filterNames = OmLanguage::sourceFileExtensions();
  filterNames.replaceInStrings(QRegularExpression("^"), stringValue());  // prepend controller name to each item

  QStringList matchingSourceFiles;
  // Searches into the current projects plugins directory or controllers directory
  // We look first for a perfect match of the file name (deprived of extension) with the plugin or controller directory name
  if (OmProject::current()) {
    const QString &projectDirPath = OmProject::current()->path() + fileType + stringValue();
    QDir projectDir(projectDirPath);
    if (projectDir.exists()) {
      dirLocation = projectFile;
      matchingSourceFiles = projectDir.entryList(filterNames, QDir::Files);
      if (!matchingSourceFiles.isEmpty()) {
        emit editRequested(projectDirPath + "/" + matchingSourceFiles[0]);
        return;
      }
    }
  }

  // Searches into the controllers/plugins associated with selected proto instance
  if (dirLocation == noFile && node()->isProtoInstance()) {
    const OmProtoModel *proto = node()->proto();
    if (!proto->projectPath().isEmpty()) {
      QDir protoDir(proto->projectPath() + "/" + ITEM_LIST_INFO[mStringType].at(0) + stringValue());
      if (protoDir.exists()) {
        dirLocation = protoFile;
        matchingSourceFiles = protoDir.entryList(filterNames, QDir::Files);
        if (!matchingSourceFiles.isEmpty()) {
          emit editRequested(protoDir.absolutePath() + "/" + matchingSourceFiles[0]);
          return;
        }
      }
    }
  }

  // Define extraDirPath here in case we find it in an extra project directory
  QString extraDirPath;
  // Look into the extra default project directory
  if (dirLocation == noFile && !OmProject::extraProjects()->isEmpty()) {
    foreach (const OmProject *extraProject, *OmProject::extraProjects()) {
      const QString &projectDirPath = extraProject->path() + fileType + stringValue();
      QDir projectDir(projectDirPath);
      if (projectDir.exists()) {
        dirLocation = extraProjectFile;
        matchingSourceFiles = projectDir.entryList(filterNames, QDir::Files);
        if (!matchingSourceFiles.isEmpty()) {
          extraDirPath = projectDirPath + "/" + matchingSourceFiles[0];
          emit editRequested(extraDirPath);
          return;
        }
      }
    }
  }

  // Look into the default project directory
  if (dirLocation == noFile) {
    bool matchingLocalDefaultDirectory = false;
    QString localDefaultDirPath = OmStandardPaths::projectsPath() + "default/" + fileType;
    QDir localDefaultSourceFilesDir(localDefaultDirPath);
    if (localDefaultSourceFilesDir.exists(stringValue())) {
      localDefaultDirPath += stringValue();
      localDefaultSourceFilesDir.setPath(localDefaultDirPath);
      matchingLocalDefaultDirectory = true;
    }

    if (matchingLocalDefaultDirectory) {
      dirLocation = omniSimProjectsFile;
      matchingSourceFiles = localDefaultSourceFilesDir.entryList(filterNames, QDir::Files);
      if (!matchingSourceFiles.isEmpty()) {
        emit editRequested(localDefaultDirPath + "/" + matchingSourceFiles[0]);
        return;
      }
    }
  }

  // Looks into the resources project directory
  if (dirLocation == noFile) {
    bool matchingDefaultDirectory = false;
    QString defaultDirPath = OmStandardPaths::resourcesProjectsPath() + fileType;
    QDir defaultSourceFilesDir(defaultDirPath);
    if (defaultSourceFilesDir.exists(stringValue())) {
      defaultDirPath += stringValue();
      defaultSourceFilesDir.setPath(defaultDirPath);
      matchingDefaultDirectory = true;
    }

    if (matchingDefaultDirectory) {
      dirLocation = resourcesFile;
      matchingSourceFiles = defaultSourceFilesDir.entryList(filterNames, QDir::Files);
      if (!matchingSourceFiles.isEmpty()) {
        emit editRequested(defaultDirPath + "/" + matchingSourceFiles[0]);
        return;
      }
    }
  }

  // Open file dialog in the corresponding folder
  QString dirPath;
  switch (dirLocation) {
    case projectFile:
      dirPath = OmProject::current()->path();
      break;
    case extraProjectFile:
      dirPath = extraDirPath;
      break;
    case protoFile:
      dirPath = node()->proto()->path() + "../";
      break;
    case resourcesFile:
      dirPath = OmStandardPaths::resourcesProjectsPath();
      break;
    case omniSimProjectsFile:
      dirPath = OmStandardPaths::projectsPath() + "default/";
      break;
    default:
      // Folder not found
      OmMessageBox::warning(tr("The file does not exist, or has an extension which is not supported."), this);
      return;
  }

  dirPath += fileType + stringValue();
  QStringList files = QFileDialog::getOpenFileNames(this, ITEM_LIST_INFO[mStringType].at(1), dirPath);
  foreach (const QString &fileName, files)
    emit editRequested(fileName);
}

void OmExtendedStringEditor::select() {
  if (selectItem())
    return;

  QStringList items;

  // add controllers/plugins of current project
  if (OmProject::current()) {
    QDir dir(OmProject::current()->path() + ITEM_LIST_INFO[mStringType].at(0));
    items += dir.entryList(FILTERS);
  }

  // add controllers/plugins associated with selected proto instance
  if (node()->isProtoInstance()) {
    QString protoProjectPath = node()->proto()->projectPath();
    if (!protoProjectPath.isEmpty() && protoProjectPath + "/" != OmProject::current()->path()) {
      QDir dir(protoProjectPath + "/" + ITEM_LIST_INFO[mStringType].at(0));
      items += dir.entryList(FILTERS);
    }
  }

  // add webots resources and default controllers/plugins
  items += defaultEntryList();
  items.sort();

  if (mStringType == CONTROLLER)
    items.prepend("<extern>");
  if (mStringType == CONTROLLER || mStringType == ROBOT_WINDOW_PLUGIN)
    items.prepend("<none>");

  items.removeDuplicates();

  // let the user choose from an item list
  DoubleClickInputDialog dialog(this);
  dialog.setWindowTitle(ITEM_LIST_INFO[mStringType].at(1));
  dialog.setLabelText(ITEM_LIST_INFO[mStringType].at(2));
  dialog.setComboBoxItems(items);
  dialog.setOption(QInputDialog::UseListViewForComboBoxItems, true);
  int result = dialog.exec();
  if (result == QDialog::Rejected)
    return;
  lineEdit()->setText(dialog.textValue());
  apply();
}

OmExtendedStringEditor::StringType OmExtendedStringEditor::fieldNameToStringType(const QString &fieldName,
                                                                                 const OmNode *parentNode) {
  if (fieldName == "controller")
    return CONTROLLER;
  else if (fieldName == "window")
    return ROBOT_WINDOW_PLUGIN;
  else if (fieldName == "remoteControl")
    return REMOTE_CONTROL_PLUGIN;
  else if (fieldName == "sound" || fieldName.endsWith("Sound", Qt::CaseSensitive))
    return SOUND;
  else if (fieldName.endsWith("IrradianceUrl", Qt::CaseSensitive))
    return HDR_TEXTURE_URL;
  else if (fieldName.endsWith("url", Qt::CaseSensitive) || fieldName.endsWith("Url", Qt::CaseSensitive)) {
    const OmMesh *mesh = dynamic_cast<const OmMesh *>(parentNode);
    if (mesh)
      return MESH_URL;
    const OmCadShape *cadShape = dynamic_cast<const OmCadShape *>(parentNode);
    if (cadShape)
      return CAD_URL;
    return TEXTURE_URL;
  } else if (fieldName == "solidName")
    return SOLID_REFERENCE;
  else if (fieldName == "fluidName")
    return FLUID_NAME;
  else if (fieldName == "referenceArea")
    return REFERENCE_AREA;
  else
    return REGULAR;
}

void OmExtendedStringEditor::updateWidgets() {
  const bool protoParameter = field()->isParameter();
  const bool regular = mStringType == REGULAR;
  const bool sound = mStringType == SOUND;
  const bool texture = mStringType == TEXTURE_URL || mStringType == HDR_TEXTURE_URL;
  const bool solidReference = mStringType == SOLID_REFERENCE;
  const bool fluidName = mStringType == FLUID_NAME;
  const bool referenceArea = mStringType == REFERENCE_AREA;
  const bool mesh = mStringType == MESH_URL || mStringType == CAD_URL;
  const bool enableLineEdit = regular || mesh || sound || texture || (solidReference && protoParameter) ||
                              (fluidName && protoParameter) || (referenceArea && protoParameter);
  const bool showSelectButton = mesh || sound || texture || !regular || (solidReference && !protoParameter) ||
                                (fluidName && !protoParameter) || (referenceArea && !protoParameter);
  const bool showEditButton = !regular && !mesh && !sound && !texture && !solidReference && !fluidName && !referenceArea;

  // show/hide widgets
  lineEdit()->setReadOnly(!enableLineEdit);
  mSelectButton->setVisible(!field()->hasRestrictedValues() && showSelectButton);
  mEditButton->setVisible(showEditButton);
}

void OmExtendedStringEditor::edit(bool copyOriginalValue) {
  OmStringEditor::edit(copyOriginalValue);

  if (copyOriginalValue) {
    const OmField *effectiveField = field();
    if (!effectiveField->internalFields().isEmpty())
      effectiveField = effectiveField->internalFields().at(0);

    mStringType = fieldNameToStringType(effectiveField->name(), effectiveField->parentNode());
  }

  updateWidgets();
}

void OmExtendedStringEditor::resetFocus() {
  mSelectButton->clearFocus();
  mEditButton->clearFocus();

  OmStringEditor::resetFocus();
}

void OmExtendedStringEditor::selectFile(const QString &folder, const QString &title, const QString &types) {
  QString path = OmProject::current()->worldsPath();
  const QDir worldPath(path);

  if (!stringValue().isEmpty()) {
    QDir dir(QDir::cleanPath(worldPath.absoluteFilePath(stringValue())));
    if (dir.cdUp())
      path = dir.absolutePath();
  } else {
    if (worldPath.exists(folder))
      path += folder + '/';
  }

  const QString fileName =
    QFileDialog::getOpenFileName(this, tr("Open %1").arg(title), path, tr("%1 files (%2)").arg(title, types));
  if (fileName.isEmpty())
    return;

  lineEdit()->setText(worldPath.relativeFilePath(fileName));
  apply();
}

bool OmExtendedStringEditor::populateItems(QStringList &items) {
  switch (mStringType) {
    case SOLID_REFERENCE: {
      const OmNode *const np = field()->parentNode();
      assert(np);
      const OmSolid *const upperSolid = OmNodeUtilities::findUpperSolid(np);
      const OmSolid *const topSolid = upperSolid->topSolid();
      items << OmSolidReference::STATIC_ENVIRONMENT;
      topSolid->collectSolidDescendantNames(items, upperSolid);
      break;
    }
    case FLUID_NAME:
      // Fluid was removed with the ODE backend; no node declares a fluidName
      // field any more, so this case is unreachable. The enum slot stays so
      // the ITEM_LIST_INFO indices do not shift.
      break;
    case REFERENCE_AREA:
      items = REFERENCE_AREA_ITEMS;
      break;
    case SOUND:
      selectFile("sounds", "Sound", "*.wav *.WAV");
      break;
    case TEXTURE_URL:
      selectFile("textures", "Texture", "*.png *.PNG *.jpg *.JPG *.jpeg *.JPEG");
      break;
    case HDR_TEXTURE_URL:
      selectFile("textures", "Texture", "*.hdr *.HDR");
      break;
    case MESH_URL:
      selectFile("meshes", "Meshes", "*.dae *.DAE *.stl *.STL *.obj *.OBJ");
      break;
    case CAD_URL:
      selectFile("meshes", "Meshes", "*.dae *.DAE *.obj *.OBJ");
      break;
    default:
      return false;
  }

  return true;
}

bool OmExtendedStringEditor::selectItem() {
  QStringList items;
  if (!populateItems(items))
    return false;

  if (mStringType >= N_STRING_TYPE_INFO)
    return true;

  // let the user choose from an item list
  DoubleClickInputDialog dialog(this);
  dialog.setWindowTitle(ITEM_LIST_INFO[mStringType].at(1));
  dialog.setLabelText(ITEM_LIST_INFO[mStringType].at(2));
  dialog.setComboBoxItems(items);
  dialog.setOption(QInputDialog::UseListViewForComboBoxItems, true);
  if (dialog.exec() == QDialog::Rejected)
    return true;
  lineEdit()->setText(dialog.textValue());
  apply();

  return true;
}
