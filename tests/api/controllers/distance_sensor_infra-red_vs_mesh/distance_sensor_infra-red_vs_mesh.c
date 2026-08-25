/*
 * Description:
 *
 * This controller tests infra-red DistanceSensor vs Meshes.
 */

#include <omnisim/camera.h>
#include <omnisim/distance_sensor.h>
#include <omnisim/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N_DISTANCE_SENSORS 20

#define NO_OBSTACLE 1000.0

// Sensors A-J rake along -z at x=0 firing +x; K-T mirror them at x=-0.11 firing -x.
// Both rakes look at the same mesh placed at two poses, so the pattern below is a
// property of that mesh's geometry: A-E strike it within a few millimetres while
// F-J pass through open space 0.52-0.70 m away, and K-R strike it while S and T
// miss by ~0.2 m. Regenerate the mesh with scripts/dev/gen_mesh_fixtures.py and
// this array has to be re-derived with it.
const bool expected_detection[N_DISTANCE_SENSORS] = {true, true, true, true, true, false, false, false, false, false,
                                                     true, true, true, true, true, true,  true,  true,  false, false};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds[N_DISTANCE_SENSORS];
  int i;
  char name[2] = "A";
  for (i = 0; i < N_DISTANCE_SENSORS; ++i) {
    ds[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
    name[0] += 1;
  }

  wb_robot_step(TIME_STEP);

  for (i = 0; i < N_DISTANCE_SENSORS; ++i) {
    const bool detection = wb_distance_sensor_get_value(ds[i]) < NO_OBSTACLE;
    ts_assert_boolean_equal(detection == expected_detection[i],
                            "Distance sensor '%c' doesn't return the right distance when hitting an object", 'A' + i);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
