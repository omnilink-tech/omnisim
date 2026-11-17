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

#ifndef OM_SIMULATION_STATE_HPP
#define OM_SIMULATION_STATE_HPP

//
// Description: current simulation state
//

#include <QtCore/QObject>

class OmSimulationState : public QObject {
  Q_OBJECT

public:
  // singleton
  static OmSimulationState *instance() { return cInstance ? cInstance : new OmSimulationState(); };

  // simulation mode
  enum Mode { NONE, PAUSE, STEP, REALTIME, FAST };
  void setMode(Mode mode);
  void setRendering(bool show);
  void undoMode() { setMode(mPreviousMode); }
  Mode mode() const { return mEnabled ? mMode : PAUSE; }
  Mode previousMode() const { return mEnabled ? mPreviousMode : PAUSE; }
  bool isPaused() const { return mMode == PAUSE; }
  bool isStep() const { return mMode == STEP; }
  bool isRealTime() const { return mMode == REALTIME; }
  bool isFast() const { return mMode == FAST; }
  bool isRendering() const { return mPerformRendering; }
  // CLI intent: the process was launched with --no-rendering / --no-window.
  // Distinct from mPerformRendering, which the preferences override in
  // OmGuiApplication::setup() can flip after argument parsing: this flag is
  // set exactly once, at parse time, and never changes for the life of the
  // process. Read by the headless texture-decode gate (OmImageTexture).
  void setStartedWithoutRendering(bool value) { mStartedWithoutRendering = value; }
  bool startedWithoutRendering() const { return mStartedWithoutRendering; }
  // pause/resume simulation for executing application dialogs
  void pauseSimulation();
  void resumeSimulation();

  // enabled
  void setEnabled(bool enabled);
  bool isEnabled() const { return mEnabled; }

  // simulation time
  double time() const { return mTime; }  // milliseconds
  void resetTime();
  void increaseTime(double dt);
  bool hasStarted() const { return mTime > 0.0; }

  // ray tracing
  void subscribeToRayTracing();
  void unsubscribeToRayTracing();
  bool isRayTracingEnabled() { return mRayTracingSubscribersCount != 0; }

signals:
  // the simulation mode has changed
  void modeChanged();
  void renderingStateChanged();
  void enabledChanged(bool);

  // steps execution
  void physicsStepStarted();
  void physicsStepEnded();
  void cameraRenderingStarted();

  void controllerReadRequestsCompleted();

  // ray tracing is enabled
  void rayTracingEnabled();

protected:
  OmSimulationState();
  virtual ~OmSimulationState();

private:
  static OmSimulationState *cInstance;
  Mode mMode, mPreviousMode;

  bool mPerformRendering;
  bool mStartedWithoutRendering = false;
  bool mEnabled;
  double mTime;

  // ray tracing
  int mRayTracingSubscribersCount;
};

#endif
