/*
 * Description:  Test painting with Pen on an oriented plane.
 */

#include <omnisim/camera.h>
#include <omnisim/motor.h>
#include <omnisim/pen.h>
#include <omnisim/position_sensor.h>
#include <omnisim/robot.h>

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
  int numColorPixels = 0;
  int x, y;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r = wb_camera_image_get_red(image, width, x, y);
      color.g = wb_camera_image_get_green(image, width, x, y);
      color.b = wb_camera_image_get_blue(image, width, x, y);
      if (!isGray(color))
        numColorPixels++;
    }
  }

  return numColorPixels;
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

  wb_pen_write(pen, false);

  // 1) Test initial painted area location
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    numColorPixels >= 0 && numColorPixels <= 50,
    "The number of pixels painted after the first step should be greater than -1 and lower than 50 not %d", numColorPixels);

  color = getPixelColor(17, 53);
  // F2.5 RE-GOLDEN (wren-deletion-runbook, 2026-08-23): old expected (206,171,58) -- the
  // 0xFA8A0A ink as WREN's pipeline shaded it. The wgpu pipeline composes the same ink to
  // (205,130,10), which is NEARER the authored ink (250,138,10): green 130 vs authored 138
  // and blue 10 exactly, where WREN lifted both (171/58). Placement on the Plane is exact
  // (the paint-location/count asserts are untouched); only the colour golden moves.
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 205, 130, 10, 15,
    "Pixel (17, 53) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after first paint call", 205, 130,
    10, color.r, color.g, color.b);

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
    oldValue <= numColorPixels && numColorPixels < 106,
    "The number of pixels painted after the second step should be greater than before (%d) and lower than 106 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(11, 40);
  // F2.5 RE-GOLDEN: old (206,171,58), see the first re-golden comment above.
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 205, 130, 10, 15,
    "Pixel (11, 40) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after second paint call", 205, 130,
    10, color.r, color.g, color.b);

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

  ts_assert_boolean_equal(
    oldValue <= numColorPixels && numColorPixels < 177,
    "The number of pixels painted after the third step should be greater than before (%d) and lower than 177 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(6, 26);
  ts_assert_color_in_delta(
    // F2.5 RE-GOLDEN: old (206,171,58) delta 27, see the first re-golden comment above.
    color.r, color.g, color.b, 205, 130, 10, 27,
    "Pixel (6, 26) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after third paint call", 205, 130, 10,
    color.r, color.g, color.b);

  ts_send_success();
  return EXIT_SUCCESS;
}
