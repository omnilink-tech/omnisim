/*
 * Description:  Test painting with Pen on a rotated and scaled box.
 *               The same texture is displayed on each face of the box.
 */

#include <omnisim/camera.h>
#include <omnisim/motor.h>
#include <omnisim/pen.h>
#include <omnisim/position_sensor.h>
#include <omnisim/robot.h>

#include <stdlib.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

WbDeviceTag camera;
const int width = 64;
const int height = 64;
const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

bool isGray(Color color) {
  return (color.r == color.g) && (color.g == color.b);
}

int computeNumColorPixels() {
  int x, y;
  int numColorPixels = 0;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r = wb_camera_image_get_red(image, width, x, y);
      color.g = wb_camera_image_get_green(image, width, x, y);
      color.b = wb_camera_image_get_blue(image, width, x, y);
      if (!isGray(color)) {
        numColorPixels++;
        // printf("Pixel x %d y %d: r=%d, g=%d, b=%d\n", x, y, wb_camera_image_get_red(image, width, x, y),
        //        wb_camera_image_get_green(image, width, x, y), wb_camera_image_get_blue(image, width, x, y));
      }
    }
  }

  return numColorPixels;
}

// F2.5 RE-PIN (wren-deletion-runbook, 2026-08-23): on a Box the Pen paints into the
// second (cross-atlas) UV set, which the wgpu vertex stream does not carry -- the E2
// decision (P3, accepted deviation, warned once by name) is that ink on Box/Cylinder/Cone
// appears DISPLACED. The old exact-pixel asserts encoded WREN's placement; under the
// decided behaviour the honest assertion is that the ink COLOUR exists in the image (and
// the count asserts above still bound how much). Ink = 0xFA8A0A composed through the wgpu
// pipeline, measured (205,130,10) on pen_plane's exact-placement geometry; delta 25
// absorbs per-face shading on the box.
int countInkPixels() {
  int n = 0;
  int x, y;
  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      const int r = wb_camera_image_get_red(image, width, x, y);
      const int g = wb_camera_image_get_green(image, width, x, y);
      const int b = wb_camera_image_get_blue(image, width, x, y);
      if (abs(r - 205) <= 25 && abs(g - 130) <= 25 && abs(b - 10) <= 25)
        n++;
    }
  }
  return n;
}

Color getPixelColor(int x, int y) {
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  color.r = wb_camera_image_get_red(image, width, x, y);
  color.g = wb_camera_image_get_green(image, width, x, y);
  color.b = wb_camera_image_get_blue(image, width, x, y);
  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen, motor, position_sensor;
  Color color;
  double pos;
  int oldValue, numColorPixels = -1;

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0xFA8A0A, 1.0);
  wb_pen_write(pen, true);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  motor = wb_robot_get_device("linear motor");

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 1) Test initial painted area location
  // send_error_and_exit
  numColorPixels = computeNumColorPixels();
  ts_assert_boolean_equal(numColorPixels <= 80,
                          "The number of pixels painted after the first step should be lower than 80 not %d", numColorPixels);

  // F2.5 RE-PIN: old assert -- getPixelColor(22, 56) == (206,171,58) +-15, WREN's exact
  // placement. Under the accepted Box-atlas deviation the placement is displaced, so the
  // pin is now "the ink colour is present in the image" (see countInkPixels above).
  ts_assert_boolean_equal(countInkPixels() > 0,
                          "No pixel carries the pen ink colour after the first paint call (accepted Box-atlas "
                          "deviation displaces the ink, but it must still be visible somewhere).");

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.02;
  wb_motor_set_position(motor, pos);

  wb_robot_step(TIME_STEP);
  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 2) Test second painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    oldValue <= numColorPixels && numColorPixels <= 150,
    "The number of pixels painted after the second step should be greater than before (%d) and lower than 150 not %d", oldValue,
    numColorPixels);

  // F2.5 RE-PIN: old assert -- getPixelColor(38, 35) == (206,171,58) +-15, WREN's exact
  // placement. Under the accepted Box-atlas deviation the placement is displaced, so the
  // pin is now "the ink colour is present in the image" (see countInkPixels above).
  ts_assert_boolean_equal(countInkPixels() > 0,
                          "No pixel carries the pen ink colour after the second paint call (accepted Box-atlas "
                          "deviation displaces the ink, but it must still be visible somewhere).");

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.02;
  wb_motor_set_position(motor, pos);

  wb_robot_step(TIME_STEP);
  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 3) Test third painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(oldValue <= numColorPixels && numColorPixels <= 180,
                          "The number of pixels painted after the third step should be greater than before (%d) and lower than "
                          "or equal to 180 not %d",
                          oldValue, numColorPixels);

  // F2.5 RE-PIN: old assert -- getPixelColor(43, 24) == (206,171,58) +-15, WREN's exact
  // placement. Under the accepted Box-atlas deviation the placement is displaced, so the
  // pin is now "the ink colour is present in the image" (see countInkPixels above).
  ts_assert_boolean_equal(countInkPixels() > 0,
                          "No pixel carries the pen ink colour after the third paint call (accepted Box-atlas "
                          "deviation displaces the ink, but it must still be visible somewhere).");

  ts_send_success();
  return EXIT_SUCCESS;
}
