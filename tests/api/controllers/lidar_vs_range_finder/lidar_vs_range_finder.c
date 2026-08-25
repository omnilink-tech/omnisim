#include <omnisim/lidar.h>
#include <omnisim/range_finder.h>
#include <omnisim/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>
#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag range_finder = wb_robot_get_device("range-finder");
  WbDeviceTag lidar = wb_robot_get_device("lidar");

  int range_finder_width = wb_range_finder_get_width(range_finder);
  int range_finder_height = wb_range_finder_get_height(range_finder);
  int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  int lidar_height = wb_lidar_get_number_of_layers(lidar);

  ts_assert_int_equal(range_finder_width, lidar_width,
                      "The width of the range-finder should be equal to the horizontal resolution of the lidar.");
  ts_assert_int_equal(range_finder_height, lidar_height,
                      "The height of the range-finder should be equal to the number of layers of the lidar.");

  wb_range_finder_enable(range_finder, TIME_STEP);
  wb_lidar_enable(lidar, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const float *image_range_finder = wb_range_finder_get_range_image(range_finder);
  ts_assert_pointer_not_null((void *)image_range_finder, "Cannot retrieve range image pointer of the RangeFinder.");
  const float *image_lidar = wb_lidar_get_range_image(lidar);
  ts_assert_pointer_not_null((void *)image_lidar, "Cannot retrieve range image pointer of the Lidar.");

  // POST-D1.4 RE-PIN (wren-deletion-runbook F2 -> D1.4): this test used to assert
  // the cylindrical RangeFinder and the Lidar return the SAME image, column for
  // column, within 0.05 m:
  //   ts_assert_double_in_delta(image_range_finder[i], image_lidar[i], 0.05, ...)
  // That comparison is retired with WREN. The Lidar's cylindrical pipeline is
  // NATIVE wgpu and stays live; the RangeFinder's cylindrical projection is
  // RETIRED-unported -- the device DECLINES the render (warned once, by name) and
  // its image stays the honestly-empty buffer, 0.0 at every pixel. Through F2 the
  // old comparison still passed byte-for-WREN via the WREN fallback (125/128
  // columns agreed; the 3 edge columns drove the edge-aware resample fix); D1.4
  // deleted the fallback, so the decided expectations are now asserted per arm:
  //   - every RangeFinder column reads exactly 0.0 (declined, nothing rendered),
  //   - the Lidar image still carries BOTH a finite hit and a +inf miss (the
  //     scene has both), proving the live arm did not silently go empty too.
  int i = 0;
  int lidar_finite_count = 0;
  int lidar_inf_count = 0;
  for (i = 0; i < lidar_width; ++i) {
    ts_assert_double_in_delta(image_range_finder[i], 0.0, 1e-6,
                              "The DECLINED cylindrical range-finder should read an empty buffer (0.0), "
                              "but column %d reads %lf.",
                              i, image_range_finder[i]);
    if (isinf(image_lidar[i]))
      ++lidar_inf_count;
    else
      ++lidar_finite_count;
  }
  ts_assert_boolean_equal(lidar_finite_count > 0,
                          "The lidar (native cylindrical wgpu pipeline) returned no finite hit at all -- "
                          "its arm should still be live after the RangeFinder's decline.");
  ts_assert_boolean_equal(lidar_inf_count > 0,
                          "The lidar returned no +inf miss at all -- the no-noise miss contract "
                          "(docs/reference/rangefinder.md) should still hold on the live arm.");

  ts_send_success();
  return EXIT_SUCCESS;
}
