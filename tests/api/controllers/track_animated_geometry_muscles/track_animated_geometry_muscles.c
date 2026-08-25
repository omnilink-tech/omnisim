#include <stdio.h>
#include <omnisim/brake.h>
#include <omnisim/motor.h>
#include <omnisim/position_sensor.h>
#include <omnisim/robot.h>
#include <omnisim/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  wb_robot_step(TIME_STEP);
  // Test that a world with a Muscle node will load. See issue #6659.
  wb_supervisor_world_load("../../worlds/supervisor_set_hinge_position_dynamic.omniworld");

  ts_send_success();
  return EXIT_SUCCESS;
}
