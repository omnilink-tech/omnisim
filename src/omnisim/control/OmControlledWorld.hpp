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

#ifndef OM_CONTROLLED_WORLD_HPP
#define OM_CONTROLLED_WORLD_HPP

#include <QtCore/QList>
#include "OmSimulationWorld.hpp"  // TODO: should we rename OmSimulationWorld to OmSimulatedWorld ?

class OmController;

class OmControlledWorld : public OmSimulationWorld {
  Q_OBJECT

public:
  // singleton instance
  static OmControlledWorld *instance();

  // constructors and destructor
  explicit OmControlledWorld(OmTokenizer *tokenizer = NULL);
  virtual ~OmControlledWorld() override;

  void startController(OmRobot *robot);
  void externConnection(OmController *controller, bool connect);
  QStringList activeControllersNames() const;
  bool needToWait(bool *waitForExternControllerStart = NULL);
  bool isExecutingStep() const { return mIsExecutingStep; }
  void checkIfReadRequestCompleted();
  void reset(bool restartControllers) override;

  void step() override;

  const QList<OmController *> &controllers() const { return mControllers; }
  const QList<OmController *> &disconnectedExternControllers() const { return mDisconnectedExternControllers; }

public slots:
  void deleteController(OmController *controller);
  void triggerStepFromTimer() override;

signals:
  void stepBlocked(bool blocked);

protected:
  void setUpControllerForNewRobot(OmRobot *robot) override;

private:
  void updateRobotController(OmRobot *robot);

#ifndef NDEBUG  // debug methods
  bool controllerInOnlyOneList(OmController *controller) const;
  bool controllerInNoList(OmController *controller) const;
  bool showControllersLists(const QString &message) const;
#endif

  QList<OmController *> mControllers;         // currently running controllers (both intern and extern)
  QList<OmController *> mWaitingControllers;  // controllers inserted in previous step and waiting to be started in current step
  QList<OmController *> mNewControllers;      // controllers inserted in current step and waiting next step to start
  QList<OmController *> mTerminatingControllers;         // controllers waiting to be deleted
  QList<OmController *> mDisconnectedExternControllers;  // extern controllers started but unconnected
  QList<double> mRequests;
  bool mNeedToYield;
  bool mFirstStep;

  // wait for controller synchronization in step mode
  bool mRetryEnabled;
  void retryStepLater();
  void processWaitingStep();

  // avoid executing a new step before the current one is completed
  bool mIsExecutingStep;  // flag indicating if a step is currently being processed
  bool mHasWaitingStep;   // flag indicating if a new step execution has been requested

private slots:
  void updateCurrentRobotController();
  void waitForRobotWindowIfNeededAndCompleteStep();
};

#endif  // OM_CONTROLLED_WORLD_HPP
