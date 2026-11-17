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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <omnisim/brake.h>
#include <omnisim/device.h>
#include <omnisim/Brake.hpp>
#include <omnisim/Motor.hpp>
#include <omnisim/PositionSensor.hpp>
#include <omnisim/Robot.hpp>

using namespace omnisim;

Brake::Type Brake::getType() const {
  return Type(wb_brake_get_type(getTag()));
}

void Brake::setDampingConstant(double dampingConstant) const {
  wb_brake_set_damping_constant(getTag(), dampingConstant);
}

Motor *Brake::getMotor() {
  if (motor == NULL)
    motor = dynamic_cast<Motor *>(Robot::getDeviceFromTag(getMotorTag()));
  return motor;
}

int Brake::getMotorTag() const {
  return wb_brake_get_motor(getTag());
}

PositionSensor *Brake::getPositionSensor() {
  if (positionSensor == NULL)
    positionSensor = dynamic_cast<PositionSensor *>(Robot::getDeviceFromTag(getPositionSensorTag()));
  return positionSensor;
}

int Brake::getPositionSensorTag() const {
  return wb_brake_get_position_sensor(getTag());
}
