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

#include "OmSingleTaskApplication.hpp"

#include "OmApplicationInfo.hpp"
#include "OmBasicJoint.hpp"
#include "OmField.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmSolid.hpp"
#include "OmSolidReference.hpp"
#include "OmSoundEngine.hpp"
#include "OmSysInfo.hpp"
#include "OmTokenizer.hpp"
#include "OmVersion.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCommandLineParser>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFunctions>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtWidgets/QMainWindow>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#endif

#include <iostream>

using namespace std;

void OmSingleTaskApplication::run() {
  if (mTask == OmGuiApplication::SYSINFO)
    showSysInfo();
  else if (mTask == OmGuiApplication::HELP)
    showHelp();
  else if (mTask == OmGuiApplication::VERSION)
    cout << tr("OmniSim version: %1 (built on Webots %2)")
              .arg(OmApplicationInfo::omniSimVersion())
              .arg(OmApplicationInfo::version().toString(true, false, true))
              .toUtf8()
              .constData()
         << endl;
  else if (mTask == OmGuiApplication::UPDATE_WORLD)
    OmWorld::instance()->save();
  else if (mTask == OmGuiApplication::CONVERT)
    convertProto();

  emit finished(mTask == OmGuiApplication::FAILURE ? EXIT_FAILURE : EXIT_SUCCESS);
}

void OmSingleTaskApplication::convertProto() const {
  QCommandLineParser cliParser;
  cliParser.setApplicationDescription("Convert a PROTO file to a URDF file");
  cliParser.addHelpOption();
  cliParser.addPositionalArgument("input", "Path to the input PROTO file.");
  cliParser.addOption(QCommandLineOption("o", "Path to the output file.", "output"));
  cliParser.addOption(QCommandLineOption("p", "Override default PROTO parameters.", "parameter=value"));
  cliParser.process(mTaskArguments);
  const QStringList positionalArguments = cliParser.positionalArguments();
  if (positionalArguments.size() != 1)
    cliParser.showHelp(1);

  const bool toStdout = cliParser.values("o").size() == 0;

  QString outputFile;
  if (!toStdout)
    outputFile = cliParser.values("o")[0];

  // Compute absolute paths for input and output files
  QString inputFile = positionalArguments[0];
  if (QDir::isRelativePath(inputFile))
    inputFile = mStartupPath + '/' + inputFile;
  if (!toStdout && QDir::isRelativePath(outputFile))
    outputFile = mStartupPath + '/' + outputFile;

  if (!QFile(inputFile).exists()) {
    cerr << tr("File '%1' is not locally available, the conversion cannot take place.").arg(inputFile).toUtf8().constData()
         << endl;
    return;
  }

  // Get user parameters strings
  QMap<QString, QString> userParameters;
  for (const QString &param : cliParser.values("p")) {
    QStringList pair = param.split("=");
    if (pair.size() != 2) {
      cerr << tr("A parameter is not properly formatted!\n").toUtf8().constData();
      cliParser.showHelp(1);
    }
    userParameters[pair[0]] = pair[1].replace(QRegularExpression("^\"*"), "").replace(QRegularExpression("\"*$"), "");
  }

  // Parse PROTO
  OmNode::setInstantiateMode(false);
  OmProtoModel *model = OmProtoManager::instance()->readModel(inputFile, "");
  if (!toStdout)
    cout << tr("Parsing the %1 PROTO...").arg(model->name()).toUtf8().constData() << endl;

  // Combine the user parameters with the default ones
  QVector<OmField *> fields;
  for (const OmFieldModel *fieldModel : model->fieldModels()) {
    OmField *field = new OmField(fieldModel);
    if (userParameters.contains(field->name())) {
      OmTokenizer tokenizer;
      tokenizer.tokenizeString(userParameters[field->name()]);
      field->readValue(&tokenizer, "");
    }

    if (!toStdout)
      cout << tr("  field %1 [%2] = %3")
                .arg(field->name())
                .arg(field->value()->vrmlTypeName())
                .arg(field->value()->toString())
                .toUtf8()
                .constData()
           << endl;
    fields.append(field);
  }

  // Generate a node structure
  OmNode::setInstantiateMode(true);
  const OmNode *node = OmNode::createProtoInstanceFromParameters(model, fields, "");
  for (OmNode *subNode : node->subNodes(true)) {
    if (dynamic_cast<OmSolidReference *>(subNode))
      cout << tr("Warning: Exporting a Joint node with a SolidReference endpoint (%1) to URDF is not supported.")
                .arg(static_cast<OmSolidReference *>(subNode)->name())
                .toUtf8()
                .constData()
           << endl;
    if (dynamic_cast<OmSolid *>(subNode))
      static_cast<OmSolid *>(subNode)->updateChildren();
    if (dynamic_cast<OmBasicJoint *>(subNode)) {
      static_cast<OmBasicJoint *>(subNode)->updateEndPoint();
      static_cast<OmBasicJoint *>(subNode)->updateEndPointZeroTranslationAndRotation();
    }
  }

  // Export
  QString output;
  OmWriter writer(&output, "robot.urdf");
  writer.writeHeader(outputFile);
  node->write(writer);
  writer.writeFooter();

  // Output the content
  if (toStdout)
    cout << output.toUtf8().toStdString() << endl;
  else {
    QFile file(outputFile);
    if (!file.open(QIODevice::WriteOnly)) {
      cerr << tr("Cannot open the file!\n").toUtf8().constData();
      cliParser.showHelp(1);
    }
    file.write(output.toUtf8());
    file.close();
  }

  if (!toStdout)
    cout << tr("The %1 PROTO is written to the file.").arg(model->name()).toUtf8().constData() << endl;
}

void OmSingleTaskApplication::showHelp() const {
  cerr << tr("Usage: omnisim [options] [worldfile]").toUtf8().constData() << endl << endl;
  cerr << tr("Options:").toUtf8().constData() << endl << endl;
  cerr << "  --help" << endl;
  cerr << tr("    Display this help message and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --version" << endl;
  cerr << tr("    Display version information and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --sysinfo" << endl;
  cerr << tr("    Display information about the system and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --mode=<mode>" << endl;
  cerr << tr("    Choose the startup mode, overriding application preferences. The <mode>").toUtf8().constData() << endl;
  cerr << tr("    argument must be either pause, realtime or fast.").toUtf8().constData() << endl << endl;
  cerr << "  --no-rendering" << endl;
  cerr << tr("    Disable rendering in the main 3D view.").toUtf8().constData() << endl << endl;
  cerr << "  --fullscreen" << endl;
  cerr << tr("    Start OmniSim in fullscreen.").toUtf8().constData() << endl << endl;
  cerr << "  --minimize" << endl;
  cerr << tr("    Minimize the OmniSim window on startup.").toUtf8().constData() << endl << endl;
  cerr << "  --no-window" << endl;
  cerr << tr("    Run fully in the background: the main window is never shown,").toUtf8().constData() << endl;
  cerr << tr("    so no taskbar entry appears. Implies --no-rendering and --batch.").toUtf8().constData() << endl << endl;
  cerr << "  --batch" << endl;
  cerr << tr("    Prevent OmniSim from creating blocking pop-up windows.").toUtf8().constData() << endl << endl;
  cerr << "  --clear-cache" << endl;
  cerr << tr("    Clear the cache of OmniSim on startup.").toUtf8().constData() << endl << endl;
  cerr << "  --stdout" << endl;
  cerr << tr("    Redirect the stdout of the controllers to the terminal.").toUtf8().constData() << endl << endl;
  cerr << "  --stderr" << endl;
  cerr << tr("    Redirect the stderr of the controllers to the terminal.").toUtf8().constData() << endl << endl;
  cerr << "  --port" << endl;
  cerr << tr("    Change the TCP port used by OmniSim (default value is 1234).").toUtf8().constData() << endl << endl;
  cerr << "  --stream[=<mode>]" << endl;
  cerr << tr("    Start the OmniSim streaming server. The <mode> argument should be either").toUtf8().constData() << endl;
  cerr << tr("    w3d (default) or mjpeg.").toUtf8().constData() << endl << endl;
  cerr << "  --extern-urls" << endl;
  cerr << tr("    Print on stdout the URL of extern controllers that should be started.").toUtf8().constData() << endl << endl;
  cerr << "  --heartbeat[=<time>]" << endl;
  cerr << tr("    Print a dot (.) on stdout every second or <time> milliseconds if specified.").toUtf8().constData() << endl
       << endl;
  cerr << "  --log-performance=<file>[,<steps>]" << endl;
  cerr << tr("    Measure the performance of OmniSim and log it in the file specified in the").toUtf8().constData() << endl;
  cerr << tr("    <file> argument. The optional <steps> argument is an integer value that").toUtf8().constData() << endl;
  cerr << tr("    specifies how many steps are logged. If the --sysinfo option is used, the").toUtf8().constData() << endl;
  cerr << tr("    system information is prepended into the log file.").toUtf8().constData() << endl << endl;
  cerr << "  convert" << endl;
  cerr << tr("    Convert a PROTO file to a URDF file.").toUtf8().constData() << endl << endl;
  cerr << tr("Please report any bug at https://github.com/omnilink-tech/omnisim/issues").toUtf8().constData() << endl;
}

void OmSingleTaskApplication::showSysInfo() const {
  cout << tr("System: %1").arg(OmSysInfo::sysInfo()).toUtf8().constData() << endl;
  cout << tr("Processor: %1").arg(OmSysInfo::processor()).toUtf8().constData() << endl;
  cout << tr("Number of cores: %1").arg(OmSysInfo::coreCount()).toUtf8().constData() << endl;
  cout << tr("OpenAL device: %1").arg(OmSoundEngine::device()).toUtf8().constData() << endl;

  // create simply an OpenGL context
  QMainWindow mainWindow;
  QOpenGLWidget openGlWidget(&mainWindow);
  mainWindow.setCentralWidget(&openGlWidget);
  mainWindow.show();

  // An OpenGL context is required there for the OpenGL calls like `glGetString`.
  // The format is QSurfaceFormat::defaultFormat() => OpenGL 3.3 defined in main.cpp.
  QOpenGLContext *context = new QOpenGLContext();
  if (!context->create())
    assert(false);
  QOpenGLFunctions *gl = context->functions();  // QOpenGLFunctions_3_3_Core cannot be initialized here on some systems like
                                                // macOS High Sierra and some Ubuntu environments.
  assert(gl);
#ifdef _WIN32
  const quint32 vendorId = OmSysInfo::gpuVendorId(gl);
  const quint32 rendererId = OmSysInfo::gpuDeviceId(gl);
#else
  const quint32 vendorId = 0;
  const quint32 rendererId = 0;
#endif

  const char *vendor = reinterpret_cast<const char *>(gl->glGetString(GL_VENDOR));
  const char *renderer = reinterpret_cast<const char *>(gl->glGetString(GL_RENDERER));
  // cppcheck-suppress knownConditionTrueFalse
  if (vendorId == 0)
    cout << tr("OpenGL vendor: %1").arg(vendor).toUtf8().constData() << endl;
  else
    cout << tr("OpenGL vendor: %1 (0x%2)").arg(vendor).arg(vendorId, 0, 16).toUtf8().constData() << endl;
  // cppcheck-suppress knownConditionTrueFalse
  if (rendererId == 0)
    cout << tr("OpenGL renderer: %1").arg(renderer).toUtf8().constData() << endl;
  else
    cout << tr("OpenGL renderer: %1 (0x%2)").arg(renderer).arg(rendererId, 0, 16).toUtf8().constData() << endl;
  cout << tr("OpenGL version: %1").arg(reinterpret_cast<const char *>(gl->glGetString(GL_VERSION))).toUtf8().constData()
       << endl;

  delete context;
}
