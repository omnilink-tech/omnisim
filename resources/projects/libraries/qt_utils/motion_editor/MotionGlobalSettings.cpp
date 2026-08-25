#include "MotionGlobalSettings.hpp"

using namespace omnisimQtUtils;

QList<Motor *> MotionGlobalSettings::cMotors;

void MotionGlobalSettings::setAvailableMotorList(const QList<Motor *> &motors) {
  cMotors.clear();
  cMotors << motors;
}
