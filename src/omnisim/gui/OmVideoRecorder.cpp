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

#include "OmVideoRecorder.hpp"

#include "OmApplication.hpp"
#include "OmDesktopServices.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmMainWindow.hpp"
#include "OmMessageBox.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationView.hpp"
#include "OmStandardPaths.hpp"
#include "OmVideoRecorderDialog.hpp"
#include "OmView3D.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"
#include "OmWrenLabelOverlay.hpp"

#include <QtCore/QBuffer>
#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QTextStream>
#include <QtCore/QThread>
#include <QtGui/QImage>
#include <QtGui/QImageWriter>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#endif

#define EXPECTED_FRAME_STEP 40  // ms (corresponding to 25 fps)

class FrameWriterThread : public QThread {
public:
  // D1.4: frames arrive as already-upright QImages from grabWindowBufferNow(). NO FLIP (the
  // old flip undid GL's bottom-left-origin readback, an artifact wgpu frames do not have). The
  // deep copy stays: it is taken in the CONSTRUCTOR, on the caller's thread.
  FrameWriterThread(const QImage &frame, const QString &fileName, const QSize &resolution, int quality) :
    mImage(frame.copy()),
    mFileName(fileName),
    mResolution(resolution),
    mPixelRatio(1),
    mQuality(quality),
    mSuccess(false) {}

  void run() override {
    // On devicePixelRatio != 1 the grab is DPR × the requested resolution (physical pixels —
    // the wgpu render target is sized width()*DPR); scale it back here, off the GUI thread, so
    // the encoded video is always exactly the requested mVideoResolution. On DPR == 1 (and
    // matching sizes) this is a straight pass-through.
    const QImage img = mImage.size() == mResolution ?
                         mImage :
                         mImage.scaled(mResolution, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    QImageWriter writer(mFileName);
    writer.setQuality(mQuality);
    mSuccess = writer.write(img);
    if (!mSuccess) {
      OmLog::warning(
        QObject::tr("Problem while saving file: '%1'").arg(QDir::toNativeSeparators(mFileName)) + "\n" + writer.errorString(),
        false);
      QString supportedFormatsLog = QObject::tr("Supported image formats:") + " ";
      QList<QByteArray> supportedFormats = QImageWriter::supportedImageFormats();
      for (int i = 0; i < supportedFormats.size(); ++i)
        supportedFormatsLog.append(QString::fromUtf8(supportedFormats[i]) + " ");
      OmLog::info(supportedFormatsLog, false);
    }
  }

  bool success() const { return mSuccess; }

private:
  const QImage mImage;  // deep copy, taken on the caller's thread
  const QString mFileName;
  const QSize mResolution;
  const int mPixelRatio;
  const int mQuality;
  bool mSuccess;
};

static const QString TEMP_FRAME_FILENAME_PREFIX = "omnisimFrame_";

OmVideoRecorder *OmVideoRecorder::cInstance = NULL;
OmMainWindow *OmVideoRecorder::cMainWindow = NULL;
int OmVideoRecorder::cDisplayRefresh = 1;

OmVideoRecorder *OmVideoRecorder::instance() {
  if (cInstance == NULL)
    cInstance = new OmVideoRecorder();

  return cInstance;
}

OmVideoRecorder::OmVideoRecorder() :
  mIsGraphicFeedbackEnabled(false),
  mIsInitialized(false),
  mIsFullScreen(false),
  mFrameFilePrefix(TEMP_FRAME_FILENAME_PREFIX + QString::number(QCoreApplication::applicationPid()) + "_"),
  mLastFileNumber(-1),
  mScreenPixelRatio(1),
  mVideoQuality(0),
  mVideoAcceleration(1),
  mShowCaption(false),
  mMovieFPS(25.0),
  mSimulationView(NULL),
  mScriptProcess(NULL) {
}

OmVideoRecorder::~OmVideoRecorder() {
}

bool OmVideoRecorder::initRecording(OmSimulationView *view, double basicTimeStep) {
  // show dialog to select parameters
  // compute maximum slow down with the current basicTimeStep
  OmVideoRecorderDialog dialog(NULL, view->view3D()->size(), 1.0 / ceil(EXPECTED_FRAME_STEP / basicTimeStep));
  bool accept = dialog.exec();
  if (!accept) {
    // cancel button - reject
    if (mIsInitialized) {
      // reset state
      disconnect(view->view3D(), &OmView3D::mainRenderingEnded, this, &OmVideoRecorder::requestSnapshotIfNeeded);
      // enable window resize
      view->disableView3DFixedSize();
      OmLog::info(tr("Video creation canceled."));
    }
    return false;
  }

  // initialize recording
  return initRecording(view, basicTimeStep, dialog.resolution(), dialog.quality(), 0, dialog.acceleration(),
                       dialog.showCaption());
}

bool OmVideoRecorder::setMainWindowFullScreen(bool fullScreen) {
  bool success = false;
  if (fullScreen) {
    success = cMainWindow->setFullScreen(true, true);
    if (success)
      cMainWindow->lockFullScreen(true);
  } else {
    // first unlock, otherwise not possible
    // to switch to normal mode
    cMainWindow->lockFullScreen(false);
    success = cMainWindow->setFullScreen(false, true);
  }

  return success;
}

void OmVideoRecorder::estimateMovieInfo(double basicTimeStep) {
  const int ceilBasicTimeStep = ceil(basicTimeStep);
  const double refresh = mVideoAcceleration * EXPECTED_FRAME_STEP / (double)ceilBasicTimeStep;
  const int floorRefresh = floor(refresh);
  const int ceilRefresh = ceil(refresh);
  const int frameStep0 = floorRefresh * ceilBasicTimeStep;
  const int frameStep1 = ceilRefresh * ceilBasicTimeStep;

  if (frameStep0 == 0 || abs(frameStep0 - EXPECTED_FRAME_STEP) > abs(frameStep1 - EXPECTED_FRAME_STEP)) {
    mMovieFPS = mVideoAcceleration * 1000.0 / frameStep1;
    cDisplayRefresh = frameStep1;
  } else {
    mMovieFPS = mVideoAcceleration * 1000.0 / frameStep0;
    cDisplayRefresh = frameStep0;
  }
}

bool OmVideoRecorder::initRecording(OmSimulationView *view, double basicTimeStep, const QSize &videoResolution, int quality,
                                    int codec, double acceleration, bool caption, const QString &filename) {
  cDisplayRefresh = 1;
  mSimulationView = view;
  mVideoName = filename;
  mIsGraphicFeedbackEnabled = filename.isEmpty();

  if (mIsInitialized) {
    // reset state
    disconnect(mSimulationView->view3D(), &OmView3D::mainRenderingEnded, this, &OmVideoRecorder::requestSnapshotIfNeeded);

    // enable window resize
    mSimulationView->disableView3DFixedSize();
  }

  // set video parameters
  mVideoQuality = quality;
  mVideoAcceleration = acceleration;
  mVideoResolution = videoResolution;

  mShowCaption = caption;
  if (mShowCaption) {
    // add caption overlay
    OmWrenLabelOverlay *label = OmWrenLabelOverlay::createOrRetrieve(OmWrenLabelOverlay::movieCaptionOverlayId(),
                                                                     OmStandardPaths::fontsPath() + "LiberationSans-Regular.ttf");
    label->setText(QString::number(mVideoAcceleration) + "x");
    // the resulting label text size depends on the 3D view height
    // 0.227 is an empirical value so that the label "0.01x" is fully displayed
    label->setPosition(1.0 - (0.227 * mVideoResolution.height() / mVideoResolution.width()), 0.025);
    label->setSize(0.2);
    label->setColor(0xffffff);
    label->applyChangesToWren();
  }

  // set folder where temp files are stored
  mTempDirPath = OmStandardPaths::webotsTmpPath();

  mLastFileNumber = -1;

  // remove old files
  removeOldTempFiles();

  const QScreen *screen = QGuiApplication::screenAt(QCursor::pos());
  const QSize fullScreen(screen->geometry().width(), screen->geometry().height());

  mIsFullScreen = (mVideoResolution == fullScreen);
  if (mIsFullScreen) {
    bool success = setMainWindowFullScreen(true);
    if (!success) {
      // do not start recording
      removeOldTempFiles();
      // enable window resize
      mSimulationView->disableView3DFixedSize();

      // remove caption if needed
      if (mShowCaption)
        OmWrenLabelOverlay::removeLabel(OmWrenLabelOverlay::movieCaptionOverlayId());

      OmLog::info(tr("Video creation canceled."));
      emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_READY);
      return false;
    }
  }

  // disable window resize while making movie
  mSimulationView->enableView3DFixedSize(mVideoResolution);

  // disable some menus while the movie is beeing created
  // ...

  // estimate movie parameters
  estimateMovieInfo(basicTimeStep);

  connect(mSimulationView->view3D(), &OmView3D::mainRenderingEnded, this, &OmVideoRecorder::requestSnapshotIfNeeded);

  if (mIsGraphicFeedbackEnabled)
    OmLog::info(tr("Video recording starts when you run a simulation..."));

  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_RECORDING);
  mIsInitialized = true;
  return true;
}

void OmVideoRecorder::stopRecording(bool canceled) {
  disconnect(mSimulationView->view3D(), &OmView3D::mainRenderingEnded, this, &OmVideoRecorder::requestSnapshotIfNeeded);
  (void)canceled;

  // enable window resize
  mSimulationView->disableView3DFixedSize();

  // Fullscreen
  if (mIsFullScreen)
    setMainWindowFullScreen(false);

  // remove caption if needed
  if (mShowCaption)
    OmWrenLabelOverlay::removeLabel(OmWrenLabelOverlay::movieCaptionOverlayId());

  if (mLastFileNumber == -1 || canceled) {
    cancelRecording();
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_SIMULATION_ERROR);

    if (!canceled)
      OmMessageBox::warning("Nothing was recorded because the simulation didn't run.", cMainWindow, "Warning");

    return;
  }

  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_SAVING);
  OmLog::info(tr("Creating video..."));

  if (mVideoName.isEmpty()) {
    // pause simulation before recording video
    OmSimulationState::Mode currentMode = OmSimulationState::instance()->mode();
    if (!OmSimulationState::instance()->isPaused())
      OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);

    // ask for video name
    static QString videoFilter = ".mp4";

    QFileInfo fi(OmWorld::instance()->fileName());
    QString worldBaseName = fi.baseName();

    QString proposedFilename;
    for (int i = 0; i < 100; ++i) {
      QString suffix = i == 0 ? "" : QString("_%1").arg(i);
      proposedFilename =
        OmPreferences::instance()->value("Directories/movies").toString() + worldBaseName + suffix + videoFilter;
      if (!QFileInfo::exists(proposedFilename))
        break;
    }

    mVideoName =
      QFileDialog::getSaveFileName(cMainWindow, tr("Save Video"), OmProject::computeBestPathForSaveAs(proposedFilename),
                                   tr("Videos (*%1)").arg(videoFilter));
    if (mVideoName.isEmpty()) {
      // canceled by user
      cancelRecording();
      // reset simulation mode
      OmSimulationState::instance()->setMode(currentMode);
      return;
    }

    OmPreferences::instance()->setValue("Directories/movies", QFileInfo(mVideoName).absolutePath() + "/");
    mVideoName = QDir::toNativeSeparators(mVideoName);

    // reset simulation mode
    OmSimulationState::instance()->setMode(currentMode);
  }

  createMpeg();

  mIsInitialized = false;
}

void OmVideoRecorder::terminateSnapshotWrite() {
  FrameWriterThread *thread = dynamic_cast<FrameWriterThread *>(sender());
  if (!thread->success())
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_WRITE_ERROR);
  delete thread;
}

void OmVideoRecorder::requestSnapshotIfNeeded(bool fromPhysics) {
  if (!fromPhysics)
    return;
  OmView3D *const view3D = mSimulationView->view3D();
  // D1.4: the frame is grabbed synchronously through the same wgpu-aware grab the --stream
  // mjpeg feed uses — already upright, RGB32 (the WREN GL PBO ring is gone).
  writeSnapshotImage(view3D->grabWindowBufferNow());
}

void OmVideoRecorder::writeSnapshotImage(const QImage &frame) {
  if (frame.isNull())
    return;
  FrameWriterThread *thread = new FrameWriterThread(frame, nextFileName(), mVideoResolution, mVideoQuality);
  connect(thread, &QThread::finished, this, &OmVideoRecorder::terminateSnapshotWrite);
  thread->start();
}

void OmVideoRecorder::terminateVideoCreation(int exitCode, QProcess::ExitStatus exitStatus) {
  // cleanup
  delete mScriptProcess;
  mScriptProcess = NULL;
  removeOldTempFiles();

  // remove script file
  QFile scriptFile(mScriptPath);
  if (scriptFile.exists()) {
    bool success = scriptFile.remove();
    if (!success)
      OmLog::warning(tr("Impossible to delete temporary file: '%1'").arg(QDir::toNativeSeparators(mScriptPath)), false);
  }

  // report exit status to user or supervisor controller
  if (exitCode > 0 || exitStatus == QProcess::CrashExit) {
    OmLog::error(tr("Video generation failed."));
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_ENCODING_ERROR);

    if (mIsGraphicFeedbackEnabled)
      OmMessageBox::warning(tr("Video generation failed due to an encoding problem\n"), mSimulationView, tr("Make Movie"));

    return;
  }

  QFile::remove(mVideoName);
  QDir().rename(mTempDirPath + "video.mp4", mVideoName);

  OmLog::info(tr("Video creation finished."));
  emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_READY);

  if (mIsGraphicFeedbackEnabled) {
    QCheckBox *checkBox = new QCheckBox(tr("Open containg folder and YouTube upload page."));
    QMessageBox box(cMainWindow);
    box.setWindowTitle(tr("Make Movie"));
    box.setText(tr("The movie has been created:\n%1\n\nDo you want to play it back?\n").arg(mVideoName));
    box.setIcon(QMessageBox::Icon::Question);
    box.addButton(QMessageBox::Yes);
    box.addButton(QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Cancel);
    box.setCheckBox(checkBox);
    if (box.exec() == QMessageBox::Yes) {
      if (checkBox->isChecked()) {
        OmDesktopServices::openUrl("https://www.youtube.com/upload");
        OmFileUtil::revealInFileManager(mVideoName);
      }
      OmDesktopServices::openUrl(QUrl::fromLocalFile(mVideoName).toString());
    }
  }
}

void OmVideoRecorder::cancelRecording() {
  removeOldTempFiles();
  mIsInitialized = false;
  OmLog::info(tr("Video creation canceled."));
}

void OmVideoRecorder::readStdout() {
  QByteArray bytes = mScriptProcess->readAllStandardOutput();
  QString out = QString::fromUtf8(bytes.data(), bytes.size());
  OmLog::appendStdout(out);
}

void OmVideoRecorder::readStderr() {
  QByteArray bytes = mScriptProcess->readAllStandardError();
  QString err = QString::fromUtf8(bytes.data(), bytes.size());
  OmLog::appendStdout(err);  // ffmpeg/avconv prints all the messages on stderr
}

void OmVideoRecorder::removeOldTempFiles() {
  QDir tempDir(mTempDirPath);
  if (!tempDir.exists()) {
    OmLog::error(tr("Temporary directory '%1' does not exist.").arg(QDir::toNativeSeparators(mTempDirPath)), false);
    return;
  }

  // remove all files starting with selected prefix
  QStringList nameFilters;
  nameFilters.append(mFrameFilePrefix + "*");
  QStringList tempFiles = tempDir.entryList(nameFilters);
  foreach (QString file, tempFiles) {
    bool success = tempDir.remove(file);
    if (!success) {
      OmLog::warning(tr("Impossible to delete temporary file: '%1'").arg(QDir::toNativeSeparators(file.toUtf8())), false);
    }
  }
}

QString OmVideoRecorder::nextFileName() {
  mLastFileNumber++;
  return mTempDirPath + mFrameFilePrefix + QString::asprintf("%06d", mLastFileNumber) + ".jpg";
}

void OmVideoRecorder::createMpeg() {
#ifdef __linux__
  // OMNISIM_ORIGINAL_LD_LIBRARY_PATH is preferred; WEBOTS_ORIGINAL_LD_LIBRARY_PATH is the legacy alias
  // (both exported by the Linux launcher shell).
  static const QString ffmpeg("LD_LIBRARY_PATH=\"${OMNISIM_ORIGINAL_LD_LIBRARY_PATH:-$WEBOTS_ORIGINAL_LD_LIBRARY_PATH}\" ffmpeg");
  static const QString percentageChar = "%";
  mScriptPath = "ffmpeg_script.sh";
#elif defined(__APPLE__)
  static const QString ffmpeg(QString("\"%1Contents/util/ffmpeg\"").arg(OmStandardPaths::omniSimHomePath()));
  static const QString percentageChar = "%";
  mScriptPath = "ffmpeg_script.sh";
#else  // _WIN32
  static const QString ffmpeg = "ffmpeg.exe";
  static const QString percentageChar = "%%";
  mScriptPath = "ffmpeg_script.bat";
#endif

  const QString initialDir = QDir::currentPath();
  QDir::setCurrent(mTempDirPath);
  // for MPEG-4: requires ffmpeg / avconv (installed on Linux, distributed on Win32 and Mac)
  QFile ffmpegScript(mScriptPath);
  if (ffmpegScript.open(QIODevice::WriteOnly)) {
    // bitrate range between 4 and 24000000
    // cast into 'long long int' is mandatory on 32-bit machine
    long long int bitrate =
      (long long int)mVideoQuality * mMovieFPS * mVideoResolution.width() * mVideoResolution.height() / 256;

    QTextStream stream(&ffmpegScript);
#ifndef _WIN32
    stream << "#!/bin/sh\n";
    static const QString openParenthesis = "\\(";
    static const QString closeParenthesis = "\\)";
#else
    stream << "@echo off\n";
    static const QString openParenthesis = "(";
    static const QString closeParenthesis = ")";
#endif
    stream << "echo " + tr("Recording at %1 FPS, %2 bit/s.").arg(mMovieFPS).arg(bitrate) + "\n";
    stream << "echo " + tr("Video encoding stage 1... ") + openParenthesis + tr("please wait") + closeParenthesis + "\n";
    stream << ffmpeg << " -loglevel warning -y -f image2 -r " << (float)mMovieFPS << " -i \"" << mFrameFilePrefix
           << percentageChar << "06d.jpg\" -b:v " << bitrate;
    stream << " -vcodec libx264 -pass 1 -g 132 -an -pix_fmt yuvj420p video.mp4\n";
#ifdef _WIN32
    stream << "IF ERRORLEVEL 1 Exit 1\n";
#else
    stream << "rc=$?\n";
    stream << "if [ $rc != 0 ] ; then\n";
    stream << "  exit 1\n";
    stream << "fi\n";
#endif

    stream << "echo " + tr("Video encoding stage 2... ") + openParenthesis + tr("please wait") + closeParenthesis + "\n";
    stream << ffmpeg << " -loglevel warning -y -f image2 -r " << (float)mMovieFPS << " -i \"" << mFrameFilePrefix
           << percentageChar << "06d.jpg\" -b:v " << bitrate;
    stream << " -vcodec libx264 -pass 2 -g 132 -an -pix_fmt yuvj420p video.mp4\n";
#ifdef _WIN32
    stream << "IF ERRORLEVEL 1 Exit 1\n";
#else
    stream << "rc=$?\n";
    stream << "if [ $rc != 0 ] ; then\n";
    stream << "  exit 1\n";
    stream << "fi\n";
#endif

    // at the end remove log files
#ifdef _WIN32
    stream << "del ffmpeg2pass-0.log\n";
    stream << "del ffmpeg2pass-0.log.mbtree\n";
    stream << "Exit 0\n";
#else  // __APPLE__ and __linux__
    stream << "rm -f *2pass-0.log*\n";
    stream << "exit 0\n";
#endif

    stream << "echo Video Encoding complete.\n";

    // close file
    ffmpegScript.close();

    // change file properties
    QFile::setPermissions(mScriptPath, QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);

    // run script
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("AV_LOG_FORCE_COLOR", "1");  // force output message to use ANSI Escape sequences
    mScriptProcess = new QProcess();
    mScriptProcess->setProcessEnvironment(env);
    mScriptProcess->start("./" + mScriptPath, QStringList());
    // clang-format off
    connect(mScriptProcess, (void (QProcess::*)(int, QProcess::ExitStatus)) & QProcess::finished, this,
            &OmVideoRecorder::terminateVideoCreation);
    // clang-format on
    connect(mScriptProcess, &QProcess::readyReadStandardOutput, this, &OmVideoRecorder::readStdout);
    connect(mScriptProcess, &QProcess::readyReadStandardError, this, &OmVideoRecorder::readStderr);
  } else {
    OmLog::error(tr("Impossible to write file: '%1'.").arg(mScriptPath) + "\n" + tr("Video generation failed."),
                 mIsGraphicFeedbackEnabled);
    emit videoCreationStatusChanged(WB_SUPERVISOR_MOVIE_WRITE_ERROR);
  }

  QDir::setCurrent(initialDir);
}
